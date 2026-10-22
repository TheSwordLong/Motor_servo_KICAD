/*
 * robot_protocol.c — Modbus RTU 与 8 字节 CAN 载荷编解码，共用 robot_read/write/command。
 * 寄存器字节为高字节在前，Modbus CRC 为低字节在前；这里不负责串口帧间隔或 CAN ID 过滤。
 */
#include "robot_protocol.h"
#include <string.h>
/**
 * @brief 计算 Modbus CRC16（初值FFFF，多项式A001）；含CRC的完整正确帧计算结果为0。
 *
 * 输入：p可读n字节；n=0返回FFFF，不解引用p。
 * 算法：逐字节异或，逐位右移并按最低位异或A001。
 * 用途：请求连同低字节优先CRC计算为0才合法；此函数本身不检查地址。
 */
uint16_t robot_crc16(const uint8_t *p,size_t n)
{
    uint16_t c=0xFFFF;
    while(n--) { c^=*p++; for(unsigned i=0;i<8;i++) c=(uint16_t)((c>>1)^((c&1)?0xA001:0)); }
    return c;
}
/**
 * @brief 按网络字节序从两字节读取 uint16_t。
 *
 * 输入：p至少2字节可读。
 * 结果：高字节左移8位后按位或低字节，用于寄存器/地址/长度，不用于CRC低字节优先解码。
 */
static uint16_t get16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0]<<8)|p[1]); }
/**
 * @brief 将 uint16_t 以高字节在前写入两字节缓冲。
 *
 * 输入：p至少2字节可写，n为16位数。
 * 过程：先高8位后低8位；没有长度检查，由协议层预分配缓冲。
 */
static void put16(uint8_t *p,uint16_t n) { p[0]=(uint8_t)(n>>8); p[1]=(uint8_t)n; }
/**
 * @brief 将业务错误映射为Modbus异常：地址2、参数/序号3、忙6，其余4。
 *
 * 输入：robot返回码e，调用者只对非0错误调用。
 * 映射：地址→02；参数/序号→03；忙→06；其余未就绪/存储等→04。
 * 限制：Modbus异常04不是电机故障4，具体原因应继续读命令结果和通道故障。
 */
static unsigned exception(int e)
{ return e==R_ADDRESS ? 2U:(e==R_VALUE || e==R_SEQUENCE ? 3U:(e==R_BUSY ? 6U:4U)); }
/**
 * @brief 解析Modbus请求，分辨静默丢帧、正常应答与异常应答。
 *
 * 输入：r有效，q可读n字节，p至少256字节可写；当前实现不处理广播。
 * 静默：总长小于4/大于256、从站地址不匹配或CRC错误直接返回0。
 * 业务：FC03读1～125字；FC06写单字；FC10写1～123字，检查字节数和总长度。
 * 错误：通过CRC后功能内格式/地址/业务错误返回异常帧；不支持功能返回01，不是都静默。
 * 返回：应答字节数含CRC；正常写应答仅代表接受，不代表动作完成。
 */
size_t robot_modbus(robot_t *r,const uint8_t *q,size_t n,uint8_t *p)
{
    uint16_t v[125]; size_t out=0; int e=R_OK;
    if(n<4 || n>256 || q[0]!=r->address || robot_crc16(q,n)!=0) return 0;
    p[0]=q[0]; p[1]=q[1];
    if(q[1]==3) {
        if(n!=8) e=R_VALUE;
        else {
            uint16_t a=get16(q+2), count=get16(q+4);
            if(!count || count>125) e=R_VALUE;
            else e=robot_read(r,a,count,v);
            if(!e) { p[2]=(uint8_t)(count*2); for(unsigned i=0;i<count;i++) put16(p+3+i*2,v[i]); out=3+count*2; }
        }
    } else if(q[1]==6) {
        if(n!=8) e=R_VALUE;
        else { v[0]=get16(q+4); e=robot_write(r,get16(q+2),1,v); if(!e) { memcpy(p,q,6); out=6; } }
    } else if(q[1]==16) {
        if(n<9) e=R_VALUE;
        else {
            uint16_t count=get16(q+4);
            if(!count || count>123 || q[6]!=count*2 || n!=9U+count*2U) e=R_VALUE;
            else { for(unsigned i=0;i<count;i++) v[i]=get16(q+7+i*2); e=robot_write(r,get16(q+2),count,v); }
            if(!e) { memcpy(p,q,6); out=6; }
        }
    } else { p[1]|=0x80; p[2]=1; out=3; }
    if(e) { p[1]|=0x80; p[2]=(uint8_t)exception(e); out=3; }
    /* 响应CRC低字节先发送，与16位寄存器高字节优先的顺序不同。 */
    uint16_t crc=robot_crc16(p,out); p[out++]=(uint8_t)crc; p[out++]=(uint8_t)(crc>>8);
    return out;
}
/**
 * @brief 解析固定8字节载荷并写8字节应答，业务结果放p[2]；返回1不代表命令成功。
 *
 * 输入：q/p分别固定8字节；CAN ID和帧类型必须已由robot_port筛选。
 * 操作1：q[2:4]地址、q[4]读1～2字；操作2写单寄存器；操作3快捷执行命令。
 * 命令：序号q[1:3]、op=q[3]、ch=q[4]、arg有符号16位q[5:7]；保留字段需为0。
 * 返回：总是1表示已构造响应，p[2]才是R_*业务结果，不能据返回值判断运动通过。
 */
int robot_can(robot_t *r,const uint8_t q[8],uint8_t p[8])
{
    uint16_t v[2]={0,0}; int e=R_VALUE;
    memset(p,0,8); p[0]=q[0]|0x80; p[1]=q[1];
    if(q[0]==1 && q[4]>=1 && q[4]<=2 && !q[5] && !q[6] && !q[7]) {
        e=robot_read(r,get16(q+2),q[4],v); p[3]=q[4];
        if(!e) { put16(p+4,v[0]); put16(p+6,v[1]); }
    } else if(q[0]==2 && !q[4] && !q[7]) {
        v[0]=get16(q+5); e=robot_write(r,get16(q+2),1,v);
    /* CAN快捷命令角度参数为有符号16位；Modbus完整命令的参数为有符号32位。 */
    } else if(q[0]==3 && !q[7]) {
        e=robot_command(r,get16(q+1),q[3],q[4],(int16_t)get16(q+5)); p[3]=q[2];
    }
    p[2]=(uint8_t)e; return 1;
}
