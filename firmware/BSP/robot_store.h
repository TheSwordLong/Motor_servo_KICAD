/*
 * robot_store.h — 末尾两页 Flash 的追加式记录存储；记录包含 schema、递增序号、dirty、镜像和 CRC32。
 * 先写正文并回读，再写提交标志；切页时保留上一页有效记录，启动选择最新完整记录。
 */
#ifndef ROBOT_STORE_H
#define ROBOT_STORE_H
#include "robot.h"
/* 保留Flash最后4KiB，两页各2KiB；每槽256字节，链接脚本也必须保留该区域。 */
#define STORE_BASE 0x0807F000UL
#define STORE_PAGE_SIZE 2048U
#define STORE_SLOT_SIZE 256U
#define STORE_SIZE (2U*STORE_PAGE_SIZE)
/* 读/写/擦回调参数为相对STORE_BASE的字节偏移；program/erase返回1成功、0失败。 */
typedef struct {
    uint32_t (*read)(uint32_t offset);
    int (*program)(uint32_t offset,uint32_t word);
    int (*erase)(uint32_t page_offset);
} store_ops_t;
typedef struct { store_ops_t ops; uint32_t sequence; int latest; } robot_store_t;
/* Returns 1 for a committed record, 0 for blank/unrecoverable storage. */
/**
 * @brief 扫描两页选最新有效记录；找到返回1并填镜像/dirty，否则返回0供调用者使用默认值。
 *
 * 输入：s和ops有效，p/dirty可写；安装回调并把latest初始化为-1。
 * 过程：遍历全部16槽，有效记录用有符号序号差选择较新者，兼顾正常序号回绕。
 * 返回：1表示找到完整记录并复制；0时调用者必须使用默认值，不依赖p/dirty被填充。
 */
int robot_store_load(robot_store_t *,const store_ops_t *,robot_image_t *,uint32_t *dirty);
/**
 * @brief 追加记录，当前页满则擦另一页；正文回读通过后最后写COMMIT，任何失败返回0。
 *
 * 输入：s已load初始化，p完整镜像，dirty由调用者传0/1。
 * 选槽：优先当前页全FF空槽；满页切另一页并擦除，保留最近有效记录所在页。
 * 提交：填头和镜像/CRC，写至提交字之前，回读memcmp，再写COMMIT，整槽校验。
 * 失败：任何步骤返回0且不前移latest/sequence；成功1才更新存储游标。
 */
int robot_store_save(robot_store_t *,const robot_image_t *,uint32_t dirty);
/**
 * @brief 存储记录CRC32：初值FFFFFFFF，多项式EDB88320，最终取反；n为字节数。
 *
 * 输入：data指向n字节镜像数据；n=0返回0。
 * 用途：覆盖槽内CRC字之前的所有字节，包含填充FF；不覆盖CRC自身和COMMIT字。
 */
uint32_t robot_crc32(const void *,size_t);
#endif
