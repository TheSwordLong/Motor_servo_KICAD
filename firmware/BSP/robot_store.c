/*
 * robot_store.c — 末尾两页 Flash 的追加式记录存储；记录包含 schema、递增序号、dirty、镜像和 CRC32。
 * 先写正文并回读，再写提交标志；切页时保留上一页有效记录，启动选择最新完整记录。
 */
#include "robot_store.h"
#include <string.h>
/*
 * 槽布局（32位字）：0魔数，1schema，2镜像字节数，3记录序号，4dirty，5开始镜像。
 * 剩余填FF，倒数第2字CRC32，最后1字COMMIT；每256字节槽占64字。
 */
#define MAGIC 0x524F4231UL
#define COMMIT 0xC04D17EDUL
enum { WORDS=STORE_SLOT_SIZE/4, SLOTS=STORE_SIZE/STORE_SLOT_SIZE, CRC_WORD=WORDS-2, COMMIT_WORD=WORDS-1 };
_Static_assert(sizeof(robot_image_t)<=((CRC_WORD-5)*4),"image too large");
/**
 * @brief 存储记录CRC32：初值FFFFFFFF，多项式EDB88320，最终取反；n为字节数。
 *
 * 输入：data指向n字节镜像数据；n=0返回0。
 * 用途：覆盖槽内CRC字之前的所有字节，包含填充FF；不覆盖CRC自身和COMMIT字。
 */
uint32_t robot_crc32(const void *data,size_t n)
{
    const uint8_t *p=data; uint32_t crc=0xFFFFFFFFU;
    while(n--) { crc^=*p++; for(unsigned i=0;i<8;i++) crc=(crc>>1)^((crc&1) ? 0xEDB88320U:0); }
    return ~crc;
}
/**
 * @brief 按槽号读取完整记录到字数组；底层读偏移以存储区起点为基准。
 *
 * 输入：slot必须在0～SLOTS-1，w至少WORDS个32位元素，s->ops.read有效。
 * 过程：按4字节偏移顺序读取整槽；无校验/选择逻辑，返回内容由valid判断。
 */
static void read_slot(robot_store_t *s,unsigned slot,uint32_t *w)
{ for(unsigned i=0;i<WORDS;i++) w[i]=s->ops.read(slot*STORE_SLOT_SIZE+i*4); }
/**
 * @brief 验证魔数/schema/镜像长度/dirty/提交标志/CRC，半写记录不视为有效。
 *
 * 输入：一个完整槽的32位数组。
 * 检查：MAGIC/schema/镜像尺寸/dirty0或1/末尾COMMIT/CRC全部匹配才为真。
 * 限制：只验证存储完整性，不验证通道配置语义，配置由robot_config_valid另查。
 */
static int valid(const uint32_t *w)
{
    return w[0]==MAGIC && w[1]==ROBOT_SCHEMA && w[2]==sizeof(robot_image_t) &&
           w[4]<=1 && w[COMMIT_WORD]==COMMIT && w[CRC_WORD]==robot_crc32(w,CRC_WORD*4);
}
/**
 * @brief 扫描两页选最新有效记录；找到返回1并填镜像/dirty，否则返回0供调用者使用默认值。
 *
 * 输入：s和ops有效，p/dirty可写；安装回调并把latest初始化为-1。
 * 过程：遍历全部16槽，有效记录用有符号序号差选择较新者，兼顾正常序号回绕。
 * 返回：1表示找到完整记录并复制；0时调用者必须使用默认值，不依赖p/dirty被填充。
 */
int robot_store_load(robot_store_t *s,const store_ops_t *ops,robot_image_t *p,uint32_t *dirty)
{
    uint32_t w[WORDS]; s->ops=*ops; s->latest=-1; s->sequence=0;
    for(unsigned slot=0;slot<SLOTS;slot++) {
        read_slot(s,slot,w);
        if(valid(w) && (s->latest<0 || (int32_t)(w[3]-s->sequence)>0)) {
            s->latest=(int)slot; s->sequence=w[3]; memcpy(p,&w[5],sizeof(*p)); *dirty=w[4];
        }
    }
    return s->latest>=0;
}
/**
 * @brief 追加记录，当前页满则擦另一页；正文回读通过后最后写COMMIT，任何失败返回0。
 *
 * 输入：s已load初始化，p完整镜像，dirty由调用者传0/1。
 * 选槽：优先当前页全FF空槽；满页切另一页并擦除，保留最近有效记录所在页。
 * 提交：填头和镜像/CRC，写至提交字之前，回读memcmp，再写COMMIT，整槽校验。
 * 失败：任何步骤返回0且不前移latest/sequence；成功1才更新存储游标。
 */
int robot_store_save(robot_store_t *s,const robot_image_t *p,uint32_t dirty)
{
    uint32_t w[WORDS], verify[WORDS]; int slot=-1;
    unsigned page=s->latest<0 ? 0U:(unsigned)s->latest/(STORE_PAGE_SIZE/STORE_SLOT_SIZE);
    for(unsigned k=0;k<STORE_PAGE_SIZE/STORE_SLOT_SIZE;k++) {
        unsigned candidate=page*(STORE_PAGE_SIZE/STORE_SLOT_SIZE)+k;
        read_slot(s,candidate,verify);
        unsigned i; for(i=0;i<WORDS && verify[i]==0xFFFFFFFFU;i++) {}
        if(i==WORDS) { slot=(int)candidate; break; }
    }
    /* 本页无空白槽则切到另一页；旧页最后有效记录在新提交完成前仍可恢复。 */
    if(slot<0) {
        page^=1U;
        /* Never erase the page containing the most recent committed record. */
        if(!s->ops.erase(page*STORE_PAGE_SIZE)) return 0;
        slot=(int)(page*(STORE_PAGE_SIZE/STORE_SLOT_SIZE));
    }
    memset(w,0xFF,sizeof(w)); w[0]=MAGIC; w[1]=ROBOT_SCHEMA; w[2]=sizeof(*p);
    w[3]=s->sequence+1; w[4]=dirty; memcpy(&w[5],p,sizeof(*p));
    w[CRC_WORD]=robot_crc32(w,CRC_WORD*4); w[COMMIT_WORD]=COMMIT;
    for(unsigned i=0;i<COMMIT_WORD;i++) {
        if(!s->ops.program((unsigned)slot*STORE_SLOT_SIZE+i*4,w[i])) return 0;
    }
    read_slot(s,(unsigned)slot,verify);
    if(memcmp(w,verify,COMMIT_WORD*4)!=0) return 0;
    /* 提交字最后写；断电留下的半记录不会通过启动扫描的valid检查。 */
    if(!s->ops.program((unsigned)slot*STORE_SLOT_SIZE+COMMIT_WORD*4,COMMIT)) return 0;
    read_slot(s,(unsigned)slot,verify);
    if(!valid(verify)) return 0;
    s->latest=slot; s->sequence=w[3]; return 1;
}
