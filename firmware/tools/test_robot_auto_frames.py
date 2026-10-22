"""Validate manual zero-sequence examples with the real protocol/core on host.

No serial/CAN device is opened. Requires the same local Zig as test_robot.ps1.
"""
from pathlib import Path
import os
import re
import struct
import subprocess

root = Path(__file__).resolve().parents[1]
text = (root / "docs/Robot_免序号操作.md").read_text(encoding="utf-8")


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc


def carray(data):
    return ",".join(str(byte) for byte in data)


source = r'''
#include "robot.h"
#include "robot_protocol.h"
#include <assert.h>
#include <stdio.h>
static robot_t r;
static void drive(uint8_t a,uint16_t b,uint8_t c,uint8_t d){(void)a;(void)b;(void)c;(void)d;}
static int save(const robot_image_t *p,uint32_t d){(void)p;(void)d;return 1;}
static void frequency(uint32_t hz){(void)hz;}
static void fresh(void){
 robot_image_t p;robot_defaults(&p);
 for(unsigned i=0;i<4;i++){p.channel[i].enabled=1;p.channel[i].role=i+1;p.channel[i].gear=516;p.valid[i]=1;}
 const robot_ops_t ops={drive,save,frequency};const uint32_t uid[3]={1,2,3};
 robot_init(&r,&ops,&p,0,uid,0);robot_feedback_t f[4]={{0}};robot_update(&r,600,f);
}
int main(void){
'''
commands = 0
reads = 0
for line in text.splitlines():
    if not line.startswith("|"):
        continue
    frames = [bytes.fromhex(hx) for hx in re.findall(r"`((?:[0-9A-F]{2} ){4,}[0-9A-F]{2})`", line)]
    if not frames:
        continue
    modbus = frames[0]
    assert modbus[0] == 1 and crc16(modbus) == 0, line
    if modbus[1] == 16:
        assert len(modbus) == 19 and modbus[2:7] == bytes.fromhex("003000050A"), line
        seq, op, ch, hi, lo = struct.unpack(">5H", modbus[7:17])
        assert seq == 0, line
        arg = struct.unpack(">i", struct.pack(">HH", hi, lo))[0]
        can = struct.pack(">BHBBhB", 3, 0, op, ch, arg, 0)
        assert len(frames) == 2 and frames[1] == can, line
        source += ('{fresh();const uint8_t q[]={' + carray(modbus) + '};'
                   'uint8_t a[256];'
                   'assert(robot_modbus(&r,q,sizeof(q),a)==8 && !robot_crc16(a,8));'
                   'assert(r.last_seq==1 && r.last_result==R_OK && r.command[0]==0);'
                   'fresh();const uint8_t c[]={' + carray(can) + '};robot_can(&r,c,a);'
                   'assert(a[0]==131 && !a[1] && !a[2] && !a[3] && r.last_seq==1);}\n')
        commands += 1
    else:
        assert modbus[1] == 3 and len(modbus) == 8, line
        count = int.from_bytes(modbus[4:6], "big")
        source += ('{fresh();const uint8_t q[]={' + carray(modbus) + '};uint8_t a[256];'
                   f'assert(robot_modbus(&r,q,sizeof(q),a)=={5+2*count});'
                   f'assert(a[1]==3 && !robot_crc16(a,{5+2*count}));}}\n')
        reads += 1
assert commands == 22 and reads == 16, (commands, reads)
source += 'puts("Manual zero-sequence RTU/CAN examples: PASS");return 0;}\n'
out = root / "build/host-tests"
out.mkdir(parents=True, exist_ok=True)
cfile = out / "robot_auto_frames.c"
exe = out / "robot_auto_frames.exe"
cfile.write_text(source, encoding="utf-8")
env = dict(os.environ, ZIG_GLOBAL_CACHE_DIR=str(root / "build/zig-cache"))
subprocess.run([str(root / "build/test-tools/ziglang/zig.exe"), "cc", "-target", "x86_64-windows-gnu",
                "-std=c11", "-Wall", "-Wextra", "-Werror", "-IBSP", str(cfile),
                "BSP/robot.c", "BSP/robot_protocol.c", "-o", str(exe)], cwd=root, env=env, check=True)
subprocess.run([str(exe)], cwd=root, check=True)
print(f"PASS: {commands} zero-sequence command variants per transport; {reads} RTU reads; CRC/fields/real parser.")
print("No hardware accessed.")
