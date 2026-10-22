"""Exercise the actual port guard/output/poll functions with mocked MCU IO.

This extracts their current source bodies, not rewritten models. It does not
validate electrical timing, interrupt latency, or any physical motor.
"""
from pathlib import Path
import os
import subprocess

root = Path(__file__).resolve().parents[1]
source = (root / 'BSP/robot_port.c').read_text(encoding='utf-8')
def function(start, end):
    return source[source.index(start):source.index(end, source.index(start))]

code = r'''
#include "robot.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#define BUTTON_ESTOP_ENABLE 0
static robot_t g_robot;
static uint32_t sys_tick_ms;
static struct { uint32_t pulse_count, errors; } g_fg[4];
static uint8_t initialized=1, output_dir[4], output_brake[4];
static volatile uint8_t probe_armed, probe_latched, probe_channel;
static volatile uint32_t probe_started, probe_stopped, probe_target;
static volatile robot_feedback_t probe_origin[4], probe_cutoff[4];
static volatile uint32_t probe_unexpected_allowance[4];
static unsigned writes;
static uint16_t pwm[4];
static uint8_t brake[4], dir[4];
static uint32_t __get_PRIMASK(void) { return 0; }
static void __disable_irq(void) {}
static void __set_PRIMASK(uint32_t p) { (void)p; }
static void pwm_set_permille(uint8_t i,uint16_t v) { pwm[i]=v; }
static void motor_set_brake(uint8_t i,uint8_t v) { brake[i]=v; }
static void motor_set_dir(uint8_t i,uint8_t v) { dir[i]=v; }
static uint32_t capture_get_speed_rpm(uint8_t i) { (void)i;return 0; }
static void poll_serial(void) {}
static void poll_can(void) {}
'''
code += function('void robot_selftest_safety_tick(void)', 'uint32_t robot_micros(void)')
code += function('static void drive(', 'static uint32_t flash_read(')
code += source[source.index('void robot_port_update(void)'):]
code += r'''
static int save(const robot_image_t *p,uint32_t dirty) {
 (void)p;(void)dirty; writes++;
 for(unsigned i=0;i<4;i++) assert(!pwm[i] && brake[i]);
 return 1;
}
static void frequency(uint32_t hz) {(void)hz;}
static uint32_t test_gear=290;
static void setup_with_unexpected(unsigned initial) {
 memset(g_fg,0,sizeof(g_fg)); probe_latched=probe_armed=0; writes=0;sys_tick_ms=0;
 const robot_ops_t ops={drive,save,frequency}; const uint32_t uid[3]={1,2,3};
 robot_image_t p;robot_defaults(&p);p.channel[0].enabled=1;p.channel[0].role=1;p.channel[0].gear=516;p.valid[0]=1;p.channel[2].gear=test_gear;
 robot_init(&g_robot,&ops,&p,0,uid,0);
 sys_tick_ms=600;g_fg[2].pulse_count=initial;robot_port_update();
 if(initial) {sys_tick_ms=1100;robot_port_update();}
 assert(robot_command(&g_robot,1,CMD_SELFTEST,0,0)==R_OK);
 assert(probe_armed && probe_target==3 && !brake[0]);
}
static void setup(void) {setup_with_unexpected(0);}
static void outbound(void) {
 sys_tick_ms=610;g_fg[0].pulse_count=3;robot_selftest_safety_tick();
 assert(probe_latched && brake[0] && !pwm[0]);
 /* Delayed main must first consume cutoff=3, then live coast=5. */
 sys_tick_ms=620;g_fg[0].pulse_count=5;robot_port_update();
 assert(g_robot.selftest[0].outbound_pulses==3 && !probe_latched);
 robot_port_update();assert(g_robot.selftest[0].outbound_pulses==5);
 sys_tick_ms=1119;robot_port_update();assert(!probe_armed && brake[0]);
 sys_tick_ms=1120;robot_port_update();
 assert(probe_armed && probe_target==5 && probe_origin[0].count==5 && probe_started==1120);
 assert(g_robot.selftest[0].phase==SP_RETURN && dir[0]==1 && !brake[0] && writes==1);
}
int main(void) {
 const uint32_t gears[]={516,290,82}, limits[]={86,48,13};
 for(unsigned k=0;k<3;k++) {
  test_gear=gears[k];setup();
  sys_tick_ms=610;g_fg[2].pulse_count=limits[k];robot_selftest_safety_tick();
  assert(probe_armed && !probe_latched);robot_port_update();
  sys_tick_ms=620;g_fg[2].pulse_count++;robot_selftest_safety_tick();
  assert(probe_latched && brake[0]);robot_port_update();
  assert(g_robot.motor[2].fault==F_UNEXPECTED);
 }
 test_gear=0;setup();
 sys_tick_ms=610;g_fg[2].pulse_count=UINT32_MAX;robot_selftest_safety_tick();
 assert(probe_armed && !probe_latched);robot_port_update();
 assert(!g_robot.motor[2].fault && !g_robot.probe.aborting);
 sys_tick_ms=620;g_fg[2].errors++;robot_selftest_safety_tick();
 assert(probe_latched);robot_port_update();assert(g_robot.motor[2].fault==F_CAPTURE);
 test_gear=290;
 /* Outbound must remain armed past the old 300ms limit, stop at 2000ms. */
 setup();
 sys_tick_ms=900;robot_selftest_safety_tick();assert(probe_armed && !brake[0]);
 sys_tick_ms=2599;robot_selftest_safety_tick();assert(probe_armed && !brake[0]);
 sys_tick_ms=2600;robot_selftest_safety_tick();assert(probe_latched && brake[0] && !pwm[0]);
 robot_port_update();assert(g_robot.selftest[0].powered_ms==2000 && g_robot.selftest[0].reason==ST_NO_PULSES);
 setup();outbound();
 sys_tick_ms=1130;g_fg[0].pulse_count=8;robot_selftest_safety_tick();
 assert(probe_armed && !probe_latched && !brake[0]);robot_port_update();
 sys_tick_ms=1140;g_fg[0].pulse_count=10;robot_selftest_safety_tick();
 assert(probe_latched && brake[0] && !pwm[0]);
 sys_tick_ms=1150;g_fg[0].pulse_count=14;robot_port_update();
 assert(g_robot.selftest[0].return_pulses==5);robot_port_update();
 sys_tick_ms=1650;robot_port_update();
 assert(g_robot.selftest[0].state==ST_PASSED && g_robot.selftest[0].residual==-4);
 assert(g_robot.selftest[0].powered_ms==10 && g_robot.selftest[0].return_ms==20 && writes==2);
 setup();outbound();
 sys_tick_ms=3119;robot_selftest_safety_tick();assert(probe_armed);
 sys_tick_ms=3120;robot_selftest_safety_tick();assert(probe_latched && brake[0]);
 robot_port_update();assert(g_robot.selftest[0].reason==ST_RETURN_TIMEOUT && g_robot.selftest[0].return_ms==2000);
 setup();outbound();sys_tick_ms=1121;g_fg[2].pulse_count=49;robot_selftest_safety_tick();
 assert(probe_latched && brake[0]);robot_port_update();assert(g_robot.probe.aborting);
 /* ISR counts from the fixed origin, even as main consumes smaller deltas. */
 setup();
 for(unsigned n=1;n<=48;n++) {
  sys_tick_ms=600+n;g_fg[2].pulse_count=n;robot_selftest_safety_tick();
  assert(probe_armed && !probe_latched && !brake[0]);robot_port_update();
  assert(g_robot.motor[2].unexpected_fg==n && !g_robot.probe.aborting);
 }
 sys_tick_ms=649;g_fg[2].pulse_count=49;robot_selftest_safety_tick();
 assert(probe_latched && brake[0]);robot_port_update();
 assert(g_robot.motor[2].fault==F_UNEXPECTED && g_robot.probe.aborting);
 /* Return arming preserves budget used before the test and during coast. */
 setup_with_unexpected(46);
 assert(probe_unexpected_allowance[2]==2 && probe_origin[2].count==46);
 sys_tick_ms=1110;g_fg[0].pulse_count=3;robot_selftest_safety_tick();robot_port_update();
 sys_tick_ms=1120;g_fg[2].pulse_count=47;robot_port_update();
 sys_tick_ms=1620;robot_port_update();
 assert(probe_armed && probe_unexpected_allowance[2]==1 && probe_origin[2].count==47);
 sys_tick_ms=1621;g_fg[2].pulse_count=48;robot_selftest_safety_tick();
 assert(probe_armed && !probe_latched);robot_port_update();
 sys_tick_ms=1622;g_fg[2].pulse_count=49;robot_selftest_safety_tick();
 assert(probe_latched && brake[0]);robot_port_update();
 assert(g_robot.motor[2].unexpected_fg==49 && g_robot.motor[2].fault==F_UNEXPECTED);
 setup_with_unexpected(48);
 assert(probe_unexpected_allowance[2]==0);
 sys_tick_ms=1101;robot_selftest_safety_tick();assert(probe_armed && !probe_latched);
 g_fg[2].pulse_count++;robot_selftest_safety_tick();assert(probe_latched && brake[0]);
 robot_port_update();assert(g_robot.motor[2].fault==F_UNEXPECTED);
 setup();outbound();sys_tick_ms=1121;g_fg[0].errors=1;robot_selftest_safety_tick();
 assert(probe_latched && brake[0]);robot_port_update();assert(g_robot.selftest[0].reason==ST_CAPTURE_ERROR);
 puts("port actual ISR / dynamic return target / cutoff-coast order / per-leg deadline / faults: PASS");
 return 0;
}
'''
out = root/'build/host-tests'
out.mkdir(parents=True, exist_ok=True)
cfile=out/'robot_port_selftest.c'
cfile.write_text(code, encoding='utf-8')
env=os.environ.copy();env['ZIG_GLOBAL_CACHE_DIR']=str(root/'build/zig-cache')
exe=out/'robot_port_selftest.exe'
subprocess.run([str(root/'build/test-tools/ziglang/zig.exe'), 'cc', '-target', 'x86_64-windows-gnu',
                '-std=c11','-Wall','-Wextra','-Werror','-IBSP',str(cfile),'BSP/robot.c','-o',str(exe)],
               cwd=root,env=env,check=True)
subprocess.run([str(exe)],cwd=root,check=True)
