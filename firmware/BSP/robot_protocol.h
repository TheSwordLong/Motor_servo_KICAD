/*
 * robot_protocol.h — Modbus RTU 与 8 字节 CAN 载荷编解码，共用 robot_read/write/command。
 * 寄存器字节为高字节在前，Modbus CRC 为低字节在前；这里不负责串口帧间隔或 CAN ID 过滤。
 */
#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H
#include "robot.h"
/**
 * @brief 计算 Modbus CRC16（初值FFFF，多项式A001）；含CRC的完整正确帧计算结果为0。
 *
 * 输入：p可读n字节；n=0返回FFFF，不解引用p。
 * 算法：逐字节异或，逐位右移并按最低位异或A001。
 * 用途：请求连同低字节优先CRC计算为0才合法；此函数本身不检查地址。
 */
uint16_t robot_crc16(const uint8_t *,size_t);
/* Complete RTU ADU in/out (capacity >=256); CRC/address failures return 0. */
/**
 * @brief 解析Modbus请求，分辨静默丢帧、正常应答与异常应答。
 *
 * 输入：r有效，q可读n字节，p至少256字节可写；当前实现不处理广播。
 * 静默：总长小于4/大于256、从站地址不匹配或CRC错误直接返回0。
 * 业务：FC03读1～125字；FC06写单字；FC10写1～123字，检查字节数和总长度。
 * 错误：通过CRC后功能内格式/地址/业务错误返回异常帧；不支持功能返回01，不是都静默。
 * 返回：应答字节数含CRC；正常写应答仅代表接受，不代表动作完成。
 */
size_t robot_modbus(robot_t *,const uint8_t *,size_t,uint8_t *);
/* Standard CAN ID 0x600+node request, 0x580+node reply, always DLC8.
 * 01 read <=2 registers; 02 stage one register; 03 execute command.
 * Reads/writes: [op,tag,addrHi,addrLo,count/reserved,valueHi,valueLo,reserved].
 * Command: [03,seqHi,seqLo,op,ch,argHi,argLo,0]; signed 16-bit arg.
 * Reply: [op|80,tag/seqHi,status,count/seqLo,data0Hi,data0Lo,data1Hi,data1Lo]. */
/**
 * @brief 解析固定8字节载荷并写8字节应答，业务结果放p[2]；返回1不代表命令成功。
 *
 * 输入：q/p分别固定8字节；CAN ID和帧类型必须已由robot_port筛选。
 * 操作1：q[2:4]地址、q[4]读1～2字；操作2写单寄存器；操作3快捷执行命令。
 * 命令：序号q[1:3]、op=q[3]、ch=q[4]、arg有符号16位q[5:7]；保留字段需为0。
 * 返回：总是1表示已构造响应，p[2]才是R_*业务结果，不能据返回值判断运动通过。
 */
int robot_can(robot_t *,const uint8_t request[8],uint8_t response[8]);
#endif
