#include "robot.h"
#include "robot_store.h"
#include "robot_protocol.h"
#include "robot_rtu.h"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

static uint32_t flash[STORE_SIZE/4];
static int fail_after=-1, operations,erases;
static uint32_t rd(uint32_t off) { assert(off<STORE_SIZE && !(off&3)); return flash[off/4]; }
static int pg(uint32_t off,uint32_t v)
{
    assert(off<STORE_SIZE && !(off&3)); assert((flash[off/4]&v)==v);
    if(operations++==fail_after) { flash[off/4]&=v|0xFFFF0000U; return 0; }
    flash[off/4]&=v; return 1;
}
static int erase(uint32_t off)
{
    assert(off==0 || off==STORE_PAGE_SIZE); erases++;
    if(operations++==fail_after) { memset((uint8_t *)flash+off,255,STORE_PAGE_SIZE/2); return 0; }
    memset((uint8_t *)flash+off,255,STORE_PAGE_SIZE); return 1;
}
static const store_ops_t storage={rd,pg,erase};
static robot_image_t saved;
static uint32_t saved_dirty, saves;
static int save_fail;
static uint16_t duties[4];
static uint8_t brakes[4],directions[4];
static void drive(uint8_t c,uint16_t d,uint8_t dir,uint8_t b)
{ assert(c<4 && d<=950); assert(!b || !d); duties[c]=d; brakes[c]=b; directions[c]=dir; }
static int save_mock(const robot_image_t *p,uint32_t d)
{
    saves++; for(unsigned i=0;i<4;i++) assert(duties[i]==0);
    if(save_fail) return 0;
    saved=*p; saved_dirty=d; return 1;
}
static void frequency(uint32_t hz) { assert(hz>=20000 && hz<=30000); }
static const robot_ops_t ops={drive,save_mock,frequency};
static const uint32_t uid[3]={0x12345678,0xABCDEF01,0x98765432};
static robot_t r;
static robot_feedback_t f[4];
static uint16_t seq;
static void fresh(void)
{
    memset(duties,0,sizeof(duties)); memset(f,0,sizeof(f)); save_fail=0; saves=0; seq=0;
    robot_init(&r,&ops,0,0,uid,0); robot_update(&r,600,f);
}
static unsigned command(unsigned op,unsigned ch,int32_t arg)
{ if(!++seq) seq=1; return robot_command(&r,seq,(uint16_t)op,(uint16_t)ch,arg); }
static void ready(void)
{
    fresh();
    const unsigned gears[4]={516,82,290,82};
    for(unsigned i=0;i<4;i++) {
        r.staged.channel[i].enabled=1; r.staged.channel[i].role=i+1;
        r.staged.channel[i].gear=gears[i];
    }
    assert(command(CMD_CONFIG,0,0)==R_OK);
    for(unsigned i=0;i<4;i++) assert(command(CMD_ZERO,i,0)==R_OK);
}
static void test_motion(void)
{
    fresh(); assert(command(CMD_MOVE,0,900)==R_UNREADY);
    ready(); unsigned n=saves;
    assert(command(CMD_MOVE,0,3600)==R_OK); assert(saves==n+1 && saved_dirty==1);
    int32_t target=r.motor[0].target; assert(target==3096);
    assert(robot_command(&r,seq,CMD_MOVE,0,3600)==R_OK && r.motor[0].target==target && saves==n+1);
    assert(robot_command(&r,seq,CMD_MOVE,0,-3600)==R_SEQUENCE);
    assert(command(CMD_MOVE,0,100)==R_BUSY);
    for(unsigned i=1;i<4;i++) assert(command(CMD_MOVE,i,-900)==R_OK);
    assert(saves==n+1); /* no Flash access while any motor is in motion */
    assert(command(CMD_CONFIG,0,0)==R_BUSY);
    assert(command(CMD_ZERO,0,0)==R_BUSY);
    assert(command(CMD_DIRECTION,0,1)==R_BUSY);
    robot_update(&r,610,f); assert(duties[0]>0 && duties[1]>0);
    for(unsigned i=0;i<4;i++) f[i].count=(uint32_t)(r.motor[i].target<0 ? -r.motor[i].target:r.motor[i].target);
    robot_update(&r,700,f);
    for(unsigned i=0;i<4;i++) assert(r.motor[i].state==S_SETTLING && brakes[i] && !duties[i]);
    robot_update(&r,1201,f); assert(saved_dirty==0 && saves==n+2);
    for(unsigned i=0;i<4;i++) assert(r.motor[i].state==S_DONE && r.motor[i].valid);
    assert(robot_angle(&r,0,r.motor[0].position)==3600);
    robot_t reboot; robot_init(&reboot,&ops,&saved,0,uid,1300);
    assert(reboot.motor[0].position==3096 && reboot.motor[0].valid);
    assert(robot_command(&reboot,seq,CMD_MOVE,0,3600)==R_SEQUENCE);
    robot_init(&reboot,&ops,&saved,1,uid,1300);
    for(unsigned i=0;i<4;i++) assert(!reboot.motor[i].valid);
    assert(command(CMD_MOVE,0,-900)==R_OK); assert(directions[0]==1);
    assert(command(CMD_BRAKE,0,1)==R_OK); assert(!duties[0] && brakes[0]);
    assert(command(CMD_BRAKE,0,0)==R_BUSY);
    robot_update(&r,1800,f);
    assert(command(CMD_BRAKE,0,0)==R_OK); assert(!duties[0] && !brakes[0]);
    assert(command(CMD_MOVE,0,3601)==R_VALUE);
}
static void test_faults(void)
{
    ready(); save_fail=1; assert(command(CMD_MOVE,0,100)==R_STORAGE); assert(!duties[0]);
    save_fail=0; assert(command(CMD_SAVE,0,0)==R_OK);
    assert(command(CMD_MOVE,0,100)==R_OK);
    robot_update(&r,1700,f); assert(r.motor[0].fault==F_FEEDBACK && !r.motor[0].valid && !duties[0]);
    ready(); assert(command(CMD_MOVE,0,900)==R_OK); f[0].errors=1;
    robot_update(&r,650,f); assert(r.motor[0].fault==F_CAPTURE && !r.motor[0].valid);
    ready(); f[1].count=14; robot_update(&r,700,f); assert(r.motor[1].fault==F_UNEXPECTED);
    ready(); r.staged.channel[0].gear=82; assert(command(CMD_CONFIG,0,0)==R_OK);
    assert(!r.motor[0].valid && r.motor[1].valid);
    r.staged.channel[0].role=r.staged.channel[1].role; assert(command(CMD_CONFIG,0,0)==R_VALUE);
    ready(); r.motor[0].position=INT32_MAX-10; assert(command(CMD_MOVE,0,3600)==R_VALUE);
    ready(); assert(command(CMD_MOVE,0,900)==R_OK); f[0].count=1000;
    robot_update(&r,700,f); robot_update(&r,1201,f); assert(r.motor[0].fault==F_POSITION);
    ready(); assert(command(CMD_MOVE,0,3600)==R_OK);
    for(uint32_t ms=1100;ms<27000;ms+=500) {
        f[0].count++; robot_update(&r,ms,f);
        if(r.motor[0].fault) break;
    }
    assert(r.motor[0].fault==F_TIMEOUT && !duties[0]);
    ready(); save_fail=1; r.staged.channel[0].gear=82;
    assert(command(CMD_CONFIG,0,0)==R_STORAGE);
    assert(r.config.channel[0].gear==516 && r.motor[0].valid);
    ready(); uint16_t old_seq=seq;
    for(unsigned i=0;i<20;i++) assert(command(CMD_DIRECTION,0,(int32_t)(i&1))==R_OK);
    assert(robot_command(&r,old_seq,CMD_ZERO,3,0)==R_SEQUENCE);
    fresh(); seq=65534;
    assert(command(CMD_DIRECTION,0,1)==R_OK); assert(command(CMD_DIRECTION,0,0)==R_OK);
    assert(seq==1 && robot_command(&r,65500,CMD_MOVE,0,50)==R_SEQUENCE);
}
static void test_store(void)
{
    robot_store_t s,t; robot_image_t p,q; uint32_t d;
    robot_defaults(&p); memset(flash,255,sizeof(flash));
    assert(!robot_store_load(&s,&storage,&q,&d));
    assert(robot_store_save(&s,&p,0));
    uint32_t clean[STORE_SIZE/4]; memcpy(clean,flash,sizeof(flash));
    robot_store_t clean_state=s;
    for(int cut=0;cut<64;cut++) {
        memcpy(flash,clean,sizeof(flash)); s=clean_state; operations=0; fail_after=cut;
        assert(!robot_store_save(&s,&p,1)); fail_after=-1;
        assert(robot_store_load(&t,&storage,&q,&d) && !d);
        /* Admission rejects this failed dirty save: motion has not begun. */
    }
    memcpy(flash,clean,sizeof(flash)); s=clean_state;
    /* Test every interrupted word including the commit word and page erase.
     * Full page forces rollover; old committed dirty record must survive. */
    for(unsigned i=0;i<7;i++) { p.position[0]=(int32_t)i; assert(robot_store_save(&s,&p,1)); }
    uint32_t baseline[STORE_SIZE/4]; memcpy(baseline,flash,sizeof(flash));
    robot_store_t before=s;
    for(int cut=0;cut<65;cut++) {
        memcpy(flash,baseline,sizeof(flash)); s=before; operations=0; fail_after=cut;
        p.position[0]=123456; assert(!robot_store_save(&s,&p,0));
        fail_after=-1; assert(robot_store_load(&t,&storage,&q,&d));
        assert(d==1 && q.position[0]==6);
    }
    memcpy(flash,baseline,sizeof(flash)); s=before; fail_after=-1;
    assert(robot_store_save(&s,&p,0)); assert(robot_store_load(&t,&storage,&q,&d));
    assert(!d && q.position[0]==123456);
    for(unsigned i=0;i<100;i++) { p.position[0]=(int32_t)i; assert(robot_store_save(&s,&p,i&1)); }
    assert(robot_store_load(&t,&storage,&q,&d) && q.position[0]==99 && d==1);
    /* CRC corruption rejects latest record and falls back to previous. */
    flash[(unsigned)s.latest*64+5]^=1; assert(robot_store_load(&t,&storage,&q,&d));
    assert(q.position[0]==98);
    assert(erases>10);
}
static size_t adu(uint8_t *q,size_t n)
{ uint16_t c=robot_crc16(q,n); q[n++]=(uint8_t)c; q[n++]=(uint8_t)(c>>8); return n; }
static void test_protocol(void)
{
    ready(); uint8_t p[256],q[256]={1,3,0,8,0,6};
    size_t n=adu(q,6); assert(robot_modbus(&r,q,n,p)==17);
    assert(robot_crc16(p,17)==0 && p[3]==0x12 && p[4]==0x34);
    q[7]^=1; assert(!robot_modbus(&r,q,n,p));
    uint8_t w[]={1,16,1,8,0,5,10,0,1,0,1,2,4,0,0,0x1F,0x40,0,0};
    assert(robot_modbus(&r,w,adu(w,17),p)==8); assert(r.staged.channel[0].gear==516);
    uint8_t invalid[]={1,16,1,12,0,2,4,1,0,2,0,0,0};
    uint32_t old=r.staged.channel[0].rpm;
    assert(robot_modbus(&r,invalid,adu(invalid,11),p)==5 && p[2]==2);
    assert(r.staged.channel[0].rpm==old); /* transactional address validation */
    uint8_t can[8]={1,42,0,8,2,0,0,0},reply[8];
    robot_can(&r,can,reply); assert(reply[2]==0 && reply[4]==0x12 && reply[7]==0x78);
    can[0]=3; can[1]=0; can[2]=++seq; can[3]=CMD_MOVE; can[4]=1; can[5]=3; can[6]=132;
    robot_can(&r,can,reply); assert(!reply[2]); int32_t target=r.motor[1].target;
    robot_can(&r,can,reply); assert(!reply[2] && target==r.motor[1].target);
    /* Malformed frames with valid CRC, lengths and random contents. */
    uint32_t random=1;
    for(unsigned iteration=0;iteration<20000;iteration++) {
        n=2+iteration%253;
        for(unsigned j=0;j<n;j++) { random=random*1664525U+1013904223U; q[j]=(uint8_t)(random>>24); }
        q[0]=1; if(iteration%3==0) q[1]=16;
        size_t size=robot_modbus(&r,q,adu(q,n),p); assert(size<=256);
        if(size) assert(robot_crc16(p,size)==0);
    }
}

static void test_direct_commands(void)
{
    uint8_t q[32], reply[256]; uint16_t v[6]={0,CMD_STOP_ALL,0,0,0,0};
    fresh(); uint16_t revision=0;
    assert(robot_read(&r,15,1,&revision)==R_OK && revision==2);
    for(unsigned offset=0;offset<5;offset++) {
        for(unsigned count=1;count<=6;count++) {
            if(!offset && count==5) continue;
            assert(robot_write(&r,(uint16_t)(0x30+offset),(uint16_t)count,v)==R_ADDRESS);
            assert(!r.have_seq && !saves && !r.command[1]);
        }
    }
    /* Register writes through CAN cannot stage or trigger commands. */
    uint8_t can[8]={2,1,0,0x30,0,0,0,0};
    robot_can(&r,can,reply); assert(reply[2]==R_ADDRESS && !r.have_seq);
    uint8_t single[8]={1,6,0,0x30,0,1};
    assert(robot_modbus(&r,single,adu(single,6),reply)==5 && reply[2]==2 && !r.have_seq);
    uint8_t legacy[8]={1,6,0,0x35,0xA5,0x5A};
    assert(robot_modbus(&r,legacy,adu(legacy,6),reply)==5 && reply[2]==2 && !r.have_seq);
    /* A complete MOVE fails atomically on a blank board. */
    uint8_t move[19]={1,16,0,0x30,0,5,10,0,0,0,CMD_MOVE,0,2,0,0,0,200};
    size_t n=adu(move,17);
    move[18]^=1; assert(!robot_modbus(&r,move,n,reply) && !r.have_seq);
    move[18]^=1;
    assert(robot_modbus(&r,move,n,reply)==5 && reply[1]==0x90 && reply[2]==4);
    assert(r.last_result==R_UNREADY && !saves && !duties[2]);
    ready(); unsigned before=saves; save_fail=1;
    assert(robot_modbus(&r,move,n,reply)==5 && reply[2]==4);
    assert(r.last_result==R_STORAGE && saves==before+1 && !duties[2] && brakes[2]);
    ready(); move[7]=(uint8_t)((seq+1)>>8); move[8]=(uint8_t)(seq+1); adu(move,17);
    assert(robot_modbus(&r,move,n,reply)==8 && r.motor[2].state==S_RUNNING);
    int32_t target=r.motor[2].target; before=saves;
    assert(robot_modbus(&r,move,n,reply)==8 && saves==before && r.motor[2].target==target);
    assert(robot_modbus(&r,legacy,8,reply)==5 && saves==before && r.motor[2].target==target);
    /* Same sequence/different command rejects, does not stop an active move. */
    memcpy(q,move,n); q[10]=CMD_STOP_ALL; adu(q,17);
    assert(robot_modbus(&r,q,n,reply)==5 && reply[2]==3 && r.motor[2].state==S_RUNNING);
    q[7]=q[8]=0; adu(q,17);
    assert(robot_modbus(&r,q,n,reply)==8 && !duties[2] && brakes[2]);
    /* Complete but unknown opcode has a value exception and no output. */
    fresh(); memcpy(q,move,n); q[7]=q[8]=0; q[10]=99; adu(q,17);
    assert(robot_modbus(&r,q,n,reply)==5 && reply[2]==3 && !saves);
}

static unsigned frames,bytes;
static void received(const uint8_t *p,size_t n,void *ctx)
{ (void)p; (void)ctx; frames++; bytes+=(unsigned)n; }
static void test_framing(void)
{
    robot_rtu_t t={0}; frames=bytes=0;
    robot_rtu_byte(&t,1,0,0,received,0); robot_rtu_byte(&t,2,100,0,received,0);
    robot_rtu_idle(&t,1849,received,0); assert(frames==0);
    robot_rtu_idle(&t,1850,received,0); assert(frames==1 && bytes==2);
    robot_rtu_byte(&t,1,2000,0,received,0); robot_rtu_byte(&t,2,2800,0,received,0);
    robot_rtu_idle(&t,5000,received,0); assert(frames==1);
    for(unsigned i=0;i<257;i++) robot_rtu_byte(&t,1,6000+i*87,0,received,0);
    robot_rtu_idle(&t,40000,received,0); assert(frames==1);
    robot_rtu_byte(&t,1,0xFFFFFFF0U,0,received,0);
    robot_rtu_byte(&t,2,0x40,0,received,0); robot_rtu_idle(&t,0x1000,received,0);
    assert(frames==2);
    robot_rtu_byte(&t,1,5000,1,received,0); robot_rtu_idle(&t,7000,received,0); assert(frames==2);
}
static void test_selftest(void)
{
    _Static_assert(sizeof(robot_image_t)==136, "Flash image must remain compatible");
    fresh(); assert(command(CMD_SELFTEST,0,0)==R_UNREADY);
    assert(command(CMD_SELFTEST_ALL,999,0)==R_OK);
    assert(r.probe.current==0 && r.selftest[3].state==ST_QUEUED);
    ready(); assert(command(CMD_SELFTEST,4,0)==R_VALUE);
    assert(command(CMD_SELFTEST,0,2)==R_VALUE);
    r.motor[3].fault=F_CAPTURE; assert(command(CMD_SELFTEST,0,0)==R_UNREADY);
    ready(); robot_update(&r,499,f); assert(command(CMD_SELFTEST,0,0)==R_BUSY);
    ready(); assert(command(CMD_MOVE,1,900)==R_OK);
    assert(command(CMD_SELFTEST,0,0)==R_BUSY);
    ready(); unsigned n=saves;
    r.motor[0].valid=0; r.dirty=1; r.config.channel[0].inverted=1;
    assert(command(CMD_SELFTEST,0,1)==R_OK); uint16_t request=seq;
    assert(saves==n+1 && saved_dirty && saved.request_sequence==request);
    assert(robot_is_busy(&r) && !duties[0] && !brakes[0] && directions[0]==0);
    assert(robot_command(&r,request,CMD_SELFTEST,0,1)==R_OK && saves==n+1);
    const unsigned blocked[]={CMD_MOVE,CMD_DIRECTION,CMD_ZERO,CMD_CONFIG,CMD_SAVE,CMD_CLEAR_FAULT,CMD_SELFTEST,CMD_SELFTEST_ALL};
    for(unsigned i=0;i<sizeof(blocked)/sizeof(blocked[0]);i++) assert(command(blocked[i],1,0)==R_BUSY);
    assert(command(CMD_BRAKE,1,0)==R_BUSY);
    for(uint32_t ms=610;ms<=710;ms+=10) {
        robot_update(&r,ms,f); assert(duties[0]<=100 && saves==n+1);
        assert(duties[0]==(ms<700 ? ms-600:100));
    }
    f[0].count=3; robot_update(&r,720,f);
    assert(!duties[0] && brakes[0] && r.selftest[0].state==ST_SETTLING);
    assert(r.selftest[0].powered_ms==120 && r.motor[0].position==-3);
    r.probe.stopped_ms=721; /* port output crosses a tick after the snapshot */
    robot_update(&r,720,f); assert(r.selftest[0].reason==ST_REASON_NONE);
    f[0].count=5; robot_update(&r,800,f); assert(r.motor[0].position==-5);
    robot_update(&r,1299,f); assert(robot_is_busy(&r) && saves==n+1);
    robot_update(&r,1300,f);
    assert(r.selftest[0].phase==SP_RETURN && r.probe.leg_target==5 && directions[0]==1);
    assert(saves==n+1 && r.motor[0].position==-5);
    f[0].count=8; robot_update(&r,1310,f); assert(!brakes[0]); /* return target is five, not three */
    f[0].count=10; robot_update(&r,1320,f); assert(brakes[0]);
    robot_update(&r,1820,f);
    assert(!robot_is_busy(&r) && saves==n+2 && !saved_dirty);
    assert(r.selftest[0].state==ST_PASSED && r.selftest[0].pulses==10 && !r.motor[0].valid);
    assert(r.motor[0].position==0 && r.selftest[0].residual==0 && r.selftest[0].return_ms==20);
    assert(r.selftest[0].completed_ms==1820 && r.selftest[0].sequence==request);
    robot_t reboot; robot_init(&reboot,&ops,&saved,1,uid,1400);
    assert(!reboot.probe.active && reboot.selftest[0].state==ST_UNTESTED);
    for(unsigned i=0;i<4;i++) assert(!reboot.motor[i].valid);
    assert(robot_command(&reboot,request,CMD_SELFTEST,0,1)==R_SEQUENCE);

    for(unsigned pulses=0;pulses<3;pulses++) {
        ready(); n=saves; assert(command(CMD_SELFTEST_ALL,255,0)==R_OK);
        f[0].count=pulses; robot_update(&r,2599,f); assert(!brakes[0]);
        robot_update(&r,2600,f); assert(brakes[0] && !duties[0]);
        assert(r.selftest[0].powered_ms==2000 && !r.motor[0].valid && r.motor[1].valid);
        assert(r.selftest[1].state==ST_QUEUED && r.selftest[1].sequence==seq);
        f[0].count+=3; robot_update(&r,2610,f); /* coast cannot rescue a failed probe */
        robot_update(&r,3110,f);
        assert(r.selftest[0].state==ST_FAILED && r.motor[0].fault==F_SELFTEST);
        assert(r.selftest[0].pulses==pulses+3);
        assert(r.selftest[0].reason==(pulses ? ST_TOO_FEW_PULSES:ST_NO_PULSES));
        assert(saves==n+1 && robot_is_busy(&r) && r.probe.current==1);
        robot_stop_all(&r); robot_update(&r,3610,f);
        assert(saves==n+2 && !robot_is_busy(&r));
    }
    ready(); n=saves; assert(command(CMD_SELFTEST,0,0)==R_OK);
    f[0].count=3; robot_update(&r,610,f);
    for(uint32_t ms=1000;ms<=2610;ms+=10) { f[0].count++; robot_update(&r,ms,f); }
    assert(r.selftest[0].reason==ST_STOP_TIMEOUT && robot_is_busy(&r));
    assert(!duties[0] && brakes[0] && saves==n+1);
    assert(command(CMD_MOVE,1,100)==R_BUSY && command(CMD_CLEAR_FAULT,0,0)==R_BUSY);
    robot_update(&r,3110,f); assert(r.selftest[0].state==ST_FAILED && !robot_is_busy(&r));
    assert(command(CMD_SELFTEST,1,0)==R_UNREADY); /* unresolved stop fault */
    assert(command(CMD_MOVE,1,100)==R_UNREADY && command(CMD_BRAKE,1,0)==R_UNREADY);
    r.staged.channel[0].inverted=1; assert(command(CMD_CONFIG,0,0)==R_OK);
    assert(command(CMD_MOVE,1,100)==R_UNREADY); /* result reset cannot clear interlock */
    assert(command(CMD_CLEAR_FAULT,0,0)==R_OK);
    assert(command(CMD_MOVE,1,100)==R_OK);

    const unsigned cancellations[]={CMD_STOP,CMD_STOP_ALL,CMD_BRAKE};
    for(unsigned i=0;i<3;i++) {
        ready(); assert(command(CMD_SELFTEST_ALL,0,0)==R_OK);
        robot_update(&r,610,f); assert(command(cancellations[i],3,1)==R_OK);
        f[0].count=2; robot_update(&r,620,f);
        robot_update(&r,1120,f);
        assert(r.selftest[0].state==ST_CANCELLED && r.selftest[0].reason==ST_CANCEL);
        assert(r.motor[0].position==2 && !r.motor[0].valid && r.motor[3].valid);
        assert(r.selftest[1].state==ST_UNTESTED && !robot_is_busy(&r));
    }
    ready(); assert(command(CMD_SELFTEST,0,0)==R_OK);
    f[0].count=3; robot_update(&r,610,f); robot_stop_all(&r);
    robot_update(&r,1110,f); assert(r.selftest[0].state==ST_CANCELLED);
    r.stop_input=1;
    assert(command(CMD_SELFTEST,0,0)==R_BUSY && command(CMD_MOVE,1,100)==R_BUSY);
    assert(command(CMD_BRAKE,1,0)==R_BUSY && command(CMD_STOP_ALL,0,0)==R_OK);
    uint16_t status; assert(robot_read(&r,0x120,1,&status)==R_OK && status==ST_CANCELLED);

    ready(); n=saves; save_fail=1;
    assert(command(CMD_SELFTEST,0,0)==R_STORAGE);
    assert(r.storage_error && !r.probe.active && brakes[0] && r.motor[0].valid);
    assert(r.selftest[0].reason==ST_STORAGE_ERROR && saves==n+1);
    save_fail=0; assert(command(CMD_SELFTEST,0,0)==R_STORAGE);
    assert(command(CMD_SAVE,0,0)==R_OK);
    assert(command(CMD_SELFTEST,0,0)==R_OK);
    f[0].count=3; robot_update(&r,610,f); save_fail=1;
    robot_update(&r,1110,f);
    assert(r.selftest[0].phase==SP_RETURN && !r.storage_error);
    f[0].count=6; robot_update(&r,1120,f); robot_update(&r,1620,f);
    assert(r.selftest[0].state==ST_PASSED && r.selftest[0].reason==ST_STORAGE_ERROR);
    assert(r.storage_error && r.dirty && r.pending_save && r.motor[0].valid);
    n=saves; robot_update(&r,2000,f); assert(saves==n);
    assert(command(CMD_MOVE,1,100)==R_STORAGE);
    save_fail=0; assert(command(CMD_SAVE,0,0)==R_OK && !r.dirty && !r.storage_error);

    for(unsigned failure=0;failure<3;failure++) {
        ready(); assert(command(CMD_SELFTEST_ALL,0,0)==R_OK);
        if(failure==0) { f[0].count=3; f[0].errors=1; }
        else if(failure==1) f[2].count=49;
        else { r.motor[0].position=INT32_MAX; f[0].count=1; }
        robot_update(&r,610,f); robot_update(&r,1110,f);
        assert(r.selftest[0].state==ST_FAILED && !r.motor[0].valid);
        assert(r.selftest[0].reason==(failure==0 ? ST_CAPTURE_ERROR:failure==1 ? ST_OTHER_ACTIVITY:ST_POSITION_OVERFLOW));
        assert(r.selftest[1].state==ST_UNTESTED);
        if(failure==1) assert(!r.motor[2].valid && r.selftest[2].reason==ST_OTHER_ACTIVITY);
    }
    ready(); r.staged.channel[1].enabled=0; assert(command(CMD_CONFIG,0,0)==R_OK);
    n=saves; assert(command(CMD_SELFTEST_ALL,999,0)==R_OK);
    assert(r.selftest[1].state==ST_QUEUED);
    uint32_t ms=600;
    for(unsigned i=0;i<4;i++) {
        assert(r.probe.current==i && saves==n+1);
        f[i].count=3; robot_update(&r,ms+10,f);
        robot_update(&r,ms+510,f);
        f[i].count=6; robot_update(&r,ms+520,f);
        robot_update(&r,ms+1020,f); ms+=1020;
        assert(r.selftest[i].state==ST_PASSED && r.motor[i].valid && r.motor[i].position==0);
    }
    assert(!robot_is_busy(&r) && saves==n+2 && !saved_dirty);
    uint16_t values[8]; assert(robot_read(&r,0x1E0,8,values)==R_OK);
    assert(values[0]==ST_PASSED && values[3]==6 && values[4]==10 && values[7]==seq);
    assert(robot_write(&r,0x120,1,values)==R_ADDRESS);
    r.staged.channel[0].rpm=7000; assert(command(CMD_CONFIG,0,0)==R_OK);
    assert(r.selftest[0].state==ST_PASSED);
    r.staged.channel[0].inverted=1; assert(command(CMD_CONFIG,0,0)==R_OK);
    assert(r.selftest[0].state==ST_UNTESTED && r.selftest[2].state==ST_PASSED);
    ready(); r.now_ms=UINT32_MAX-100;
    assert(command(CMD_SELFTEST,0,0)==R_OK);
    robot_update(&r,1898,f); assert(!brakes[0]);
    robot_update(&r,1899,f); assert(brakes[0] && r.selftest[0].powered_ms==2000);
    robot_update(&r,2399,f); assert(r.selftest[0].state==ST_FAILED);
}

static void test_discovery(void)
{
    for(unsigned mixed=0;mixed<2;mixed++) {
        fresh(); robot_image_t config=r.config; unsigned n=saves;
        assert(command(CMD_SELFTEST_ALL,999,0)==R_OK);
        unsigned request=seq;
        assert(robot_command(&r,request,CMD_SELFTEST_ALL,999,0)==R_OK && saves==n+1);
        uint32_t ms=600;
        for(unsigned i=0;i<4;i++) {
            assert(r.probe.current==i && !brakes[i] && saves==n+1);
            for(unsigned j=0;j<4;j++) if(j!=i) assert(brakes[j] && !duties[j]);
            unsigned pulses=mixed ? i:0;
            for(unsigned tick=10;tick<2000;tick+=10) {
                robot_update(&r,ms+tick,f); assert(duties[i]<=100 && saves==n+1);
            }
            f[i].count=pulses; robot_update(&r,ms+2000,f);
            assert(brakes[i] && !duties[i] && r.selftest[i].powered_ms==2000);
            robot_update(&r,ms+2499,f); assert(r.probe.current==i && saves==n+1);
            robot_update(&r,ms+2500,f); ms+=2500;
            if(pulses>=3) {
                assert(r.selftest[i].phase==SP_RETURN && saves==n+1);
                f[i].count+=pulses; robot_update(&r,ms+10,f);
                robot_update(&r,ms+510,f); ms+=510;
            }
            assert(r.selftest[i].state==(pulses>=3 ? ST_PASSED:ST_FAILED));
            assert(r.selftest[i].reason==(pulses>=3 ? ST_REASON_NONE:pulses ? ST_TOO_FEW_PULSES:ST_NO_PULSES));
            assert(!r.motor[i].valid && r.motor[i].position==(int32_t)(pulses>=3 ? 0:pulses));
            assert(r.selftest[i].sequence==request && r.selftest[i].pulses==(pulses>=3 ? 2*pulses:pulses));
            assert(!memcmp(&config.channel[i],&r.config.channel[i],sizeof(robot_config_t)));
        }
        assert(!robot_is_busy(&r) && saves==n+2 && !r.dirty && !r.pending_save);
        for(unsigned i=0;i<4;i++) assert(!saved.valid[i] && !saved.channel[i].enabled && !saved.channel[i].gear);
    }
    /* A weak-response coast is still cancellable and safety-monitored. */
    for(unsigned failure=0;failure<3;failure++) {
        fresh();
        r.staged.channel[2].gear=290;
        assert(command(CMD_CONFIG,0,0)==R_OK);
        assert(command(CMD_SELFTEST_ALL,0,0)==R_OK);
        robot_update(&r,2600,f); assert(!r.probe.aborting);
        if(failure==0) robot_stop_all(&r);
        else if(failure==1) { f[0].errors=1; robot_update(&r,2610,f); }
        else { f[2].count=49; robot_update(&r,2610,f); }
        robot_update(&r,3110,f);
        assert(!robot_is_busy(&r) && r.selftest[1].state==ST_UNTESTED);
        assert(r.selftest[0].reason==(failure==0 ? ST_CANCEL:failure==1 ? ST_CAPTURE_ERROR:ST_OTHER_ACTIVITY));
    }
    fresh(); assert(command(CMD_SELFTEST_ALL,0,0)==R_OK);
    robot_update(&r,2600,f);
    for(uint32_t ms=2700;ms<=4600;ms+=100) { f[0].count++; robot_update(&r,ms,f); }
    assert(r.probe.aborting && r.probe.stop_fault_mask && r.selftest[1].state==ST_UNTESTED);
    assert(robot_is_busy(&r) && saves==1 && brakes[0]);
    robot_update(&r,5100,f); assert(!robot_is_busy(&r) && saves==2);
    /* Blank boards accept op 11 via both transports; replay never restarts. */
    for(unsigned transport=0;transport<2;transport++) {
        fresh(); uint8_t reply[256];
        if(!transport) {
            uint8_t q[19]={1,16,0,0x30,0,5,10,0,1,0,CMD_SELFTEST_ALL,0,0,0,0,0,0};
            assert(robot_modbus(&r,q,adu(q,17),reply)==8);
            assert(robot_modbus(&r,q,19,reply)==8 && saves==1);
        } else {
            uint8_t q[8]={3,0,1,CMD_SELFTEST_ALL,0,0,0,0};
            robot_can(&r,q,reply); assert(reply[2]==R_OK);
            robot_can(&r,q,reply); assert(reply[2]==R_OK && saves==1);
        }
        assert(r.probe.active && r.probe.current==0 && r.selftest[3].state==ST_QUEUED);
    }
}

static void test_roundtrip(void)
{
    /* Both logical signs/inversions, nonzero origins and diagnostic return coast residuals. */
    for(unsigned neg=0;neg<2;neg++) for(unsigned inv=0;inv<2;inv++) for(unsigned extra=0;extra<6;extra++) {
        ready(); r.config.channel[0].inverted=inv; r.motor[0].position=100;
        unsigned n=saves; assert(command(CMD_SELFTEST_ALL,0,neg)==R_OK);
        assert(directions[0]==(neg^inv));
        f[0].count=3; robot_update(&r,610,f);
        f[0].count=5; robot_update(&r,620,f);
        robot_update(&r,1119,f); assert(brakes[0] && r.selftest[0].phase==SP_OUTBOUND_SETTLING);
        robot_update(&r,1120,f); assert(!brakes[0] && directions[0]==(neg^inv^1U) && r.probe.leg_target==5);
        f[0].count=8; robot_update(&r,1130,f); assert(!brakes[0]);
        f[0].count=10; robot_update(&r,1140,f); assert(brakes[0]);
        f[0].count+=extra; robot_update(&r,1150,f);
        robot_update(&r,1650,f);
        int32_t residual=neg ? (int32_t)extra:-(int32_t)extra;
        assert(r.selftest[0].residual==residual && r.motor[0].position==100+residual);
        assert(r.selftest[0].outbound_pulses==5 && r.selftest[0].return_pulses==5+extra);
        assert(r.selftest[0].phase==SP_FINISHED && r.selftest[0].powered_ms==10 && r.selftest[0].return_ms==20);
        assert(r.selftest[0].state==ST_PASSED && r.selftest[0].reason==ST_REASON_NONE);
        assert(r.motor[0].valid && r.motor[0].fault==F_NONE && r.probe.current==1 && saves==n+1);
        uint16_t words[2];
        assert(robot_read(&r,0x12E,2,words)==R_OK);
        assert((((uint32_t)words[0]<<16)|words[1])==(uint32_t)residual);
        robot_stop_all(&r); robot_update(&r,2150,f);
        assert(saves==n+2 && r.motor[0].valid && r.motor[0].position==100+residual);
    }
    _Static_assert(ROBOT_SELFTEST_POWER_MS==2000U, "Selftest leg limit must be 2 seconds");
    /* Near target does not rescue a return deadline miss; no subsequent channel. */
    ready(); assert(command(CMD_SELFTEST_ALL,0,0)==R_OK);
    f[0].count=3; robot_update(&r,610,f); robot_update(&r,1110,f);
    for(uint32_t ms=1120;ms<3110;ms+=10) { robot_update(&r,ms,f); assert(duties[0]<=100); }
    f[0].count=5; robot_update(&r,3110,f);
    assert(r.selftest[0].reason==ST_RETURN_TIMEOUT && r.selftest[0].return_ms==2000 && brakes[0]);
    robot_update(&r,3610,f); assert(r.selftest[0].state==ST_FAILED && r.selftest[1].state==ST_UNTESTED);
    assert(!r.motor[0].valid && r.selftest[0].residual==1);
    /* Cancellation at every powered/coasting phase never starts another leg. */
    for(unsigned phase=1;phase<=4;phase++) {
        ready(); assert(command(CMD_SELFTEST_ALL,0,0)==R_OK);
        uint32_t ms=600;
        if(phase>=2) { f[0].count=3; robot_update(&r,610,f); ms=610; }
        if(phase>=3) { robot_update(&r,1110,f); ms=1110; }
        if(phase>=4) { f[0].count=6; robot_update(&r,1120,f); ms=1120; }
        assert(r.selftest[0].phase==phase);
        assert(command(CMD_STOP_ALL,0,0)==R_OK);
        robot_update(&r,ms+500,f);
        assert(r.selftest[0].state==ST_CANCELLED && r.selftest[0].phase==SP_FINISHED);
        assert(!r.motor[0].valid && !robot_is_busy(&r) && r.selftest[1].phase==SP_NONE);
    }
    for(unsigned failure=0;failure<3;failure++) {
        ready(); assert(command(CMD_SELFTEST_ALL,0,0)==R_OK);
        f[0].count=3; robot_update(&r,610,f); robot_update(&r,1110,f);
        if(failure==0) f[0].errors++;
        else if(failure==1) f[2].count+=49;
        else { r.motor[0].position=INT32_MIN; f[0].count++; }
        robot_update(&r,1120,f); robot_update(&r,1620,f);
        assert(r.selftest[0].state==ST_FAILED && !robot_is_busy(&r) && r.selftest[1].state==ST_UNTESTED);
    }
    /* Return settling timeout retains board-wide lock. */
    ready(); assert(command(CMD_SELFTEST_ALL,0,0)==R_OK);
    f[0].count=3; robot_update(&r,610,f); robot_update(&r,1110,f);
    f[0].count=6; robot_update(&r,1120,f);
    for(uint32_t ms=1200;ms<=3120;ms+=10) { f[0].count++; robot_update(&r,ms,f); }
    assert(r.selftest[0].reason==ST_STOP_TIMEOUT && r.probe.stop_fault_mask && robot_is_busy(&r));
    robot_update(&r,3620,f); assert(!robot_is_busy(&r) && command(CMD_MOVE,1,100)==R_UNREADY);
    /* The return deadline survives millisecond wrap, independent of outbound. */
    ready(); r.now_ms=UINT32_MAX-800; assert(command(CMD_SELFTEST,0,0)==R_OK);
    f[0].count=3; robot_update(&r,UINT32_MAX-790,f); robot_update(&r,UINT32_MAX-290,f);
    robot_update(&r,1708,f); assert(!brakes[0]);
    robot_update(&r,1709,f); assert(r.selftest[0].return_ms==2000 && r.selftest[0].reason==ST_RETURN_TIMEOUT);
}

static void test_selftest_protocol(void)
{
    ready(); uint16_t request=seq+1;
    uint8_t q[32]={1,16,0,0x30,0,5,10,0,(uint8_t)request,0,CMD_SELFTEST,0,2,0,0,0,1},reply[256];
    assert(robot_modbus(&r,q,adu(q,17),reply)==8);
    assert(r.probe.active && r.probe.current==2 && r.probe.negative);
    unsigned n=saves;
    assert(robot_modbus(&r,q,19,reply)==8 && saves==n);
    uint8_t can[8]={3,0,(uint8_t)request,CMD_SELFTEST,2,0,1,0};
    robot_can(&r,can,reply); assert(reply[2]==R_OK && saves==n);
    f[2].count=3; robot_update(&r,610,f); robot_update(&r,1110,f);
    f[2].count=6; robot_update(&r,1120,f); robot_update(&r,1620,f);
    uint8_t read[8]={1,3,1,0xA0,0,8};
    assert(robot_modbus(&r,read,adu(read,6),reply)==21 && reply[4]==ST_PASSED);
    uint8_t c[8]; memcpy(c,reply+3,4);
    can[0]=1; can[1]=42; can[2]=1; can[3]=0xA0; can[4]=2;
    can[5]=can[6]=can[7]=0;
    robot_can(&r,can,reply); assert(!reply[2] && !memcmp(reply+4,c,4));
    /* Existing register offsets retain their meaning. */
    uint16_t old[7]; assert(robot_read(&r,0x190,7,old)==R_OK);
    assert(old[0]==S_IDLE && old[2]==1 && old[3]==1);
    uint8_t ext[8]={1,3,1,0xA8,0,8};
    assert(robot_modbus(&r,ext,adu(ext,6),reply)==21);
    uint8_t detail[16]; memcpy(detail,reply+3,16);
    for(unsigned j=0;j<4;j++) {
        uint8_t query[8]={1,42,1,(uint8_t)(0xA8+2*j),2,0,0,0};
        robot_can(&r,query,reply); assert(!reply[2] && !memcmp(reply+4,detail+4*j,4));
    }
    assert(detail[1]==SP_FINISHED && detail[3]==10 && detail[7]==3 && detail[11]==3);
    assert(robot_write(&r,0x1A8,1,old)==R_ADDRESS);
    uint8_t batch[8]={3,0,(uint8_t)(request+1),CMD_SELFTEST_ALL,255,0,1,0};
    robot_can(&r,batch,reply); assert(!reply[2] && r.probe.current==0 && r.probe.negative);
    batch[2]++; batch[3]=CMD_STOP_ALL;
    robot_can(&r,batch,reply); assert(!reply[2] && r.probe.aborting && brakes[0]);
}
static void test_auto_sequence(void)
{
    fresh();
    assert(robot_command(&r,0,CMD_SAVE,0,0)==R_OK && r.last_seq==1 && saves==1);
    assert(robot_command(&r,0,CMD_SAVE,0,0)==R_OK && r.last_seq==2 && saves==2);
    assert(saved.request_sequence==2 && saved.request_seen==1);
    assert(robot_command(&r,2,CMD_SAVE,0,0)==R_OK && saves==2);
    assert(robot_command(&r,2,CMD_ZERO,0,0)==R_SEQUENCE && r.last_seq==2);
    assert(robot_command(&r,3,CMD_SAVE,0,0)==R_OK && saves==3);
    robot_t reboot;
    robot_init(&reboot,&ops,&saved,0,uid,0); robot_update(&reboot,600,f);
    assert(robot_command(&reboot,3,CMD_SAVE,0,0)==R_SEQUENCE);
    assert(robot_command(&reboot,0,CMD_SAVE,0,0)==R_OK && reboot.last_seq==4);

    /* Cached low IDs can survive several legal explicit jumps through wrap. */
    fresh();
    assert(robot_command(&r,1,CMD_SAVE,0,0)==R_OK);
    assert(robot_command(&r,30000,CMD_SAVE,0,0)==R_OK);
    assert(robot_command(&r,60000,CMD_SAVE,0,0)==R_OK);
    assert(robot_command(&r,65535,CMD_SAVE,0,0)==R_OK);
    assert(robot_command(&r,0,CMD_SAVE,0,0)==R_OK && r.last_seq==2 && saves==5);
    assert(robot_command(&r,0,CMD_SAVE,0,0)==R_OK && r.last_seq==3 && saves==6);
    robot_image_t legacy=saved; legacy.request_sequence=0; legacy.request_seen=1;
    robot_init(&reboot,&ops,&legacy,0,uid,0); robot_update(&reboot,600,f);
    assert(robot_command(&reboot,0,CMD_SAVE,0,0)==R_OK && reboot.last_seq==1);

    ready();
    assert(robot_command(&r,0,CMD_MOVE,0,200)==R_OK);
    uint16_t first=r.last_seq; int32_t target=r.motor[0].target;
    assert(saved.request_sequence==first && saved_dirty==1);
    assert(robot_command(&r,0,CMD_MOVE,0,200)==R_BUSY && r.last_seq==first+1);
    assert(r.motor[0].target==target); /* no interlock bypass */
    f[0].count=(uint32_t)target; robot_update(&r,700,f); robot_update(&r,1201,f);
    assert(r.motor[0].state==S_DONE && r.motor[0].position==target);
    assert(robot_command(&r,first,CMD_MOVE,0,200)==R_OK && r.motor[0].target==target);
    assert(robot_command(&r,0,CMD_MOVE,0,200)==R_OK && r.motor[0].target==2*target);
    assert(robot_command(&r,0,CMD_STOP_ALL,0,0)==R_OK && brakes[0] && !duties[0]);

    fresh(); r.motor[1].fault=F_UNEXPECTED;
    assert(robot_command(&r,0,CMD_SELFTEST_ALL,0,0)==R_UNREADY && !saves && !r.probe.active);
    assert(robot_command(&r,0,CMD_CLEAR_FAULT,1,0)==R_OK);
    save_fail=1;
    assert(robot_command(&r,0,CMD_SELFTEST_ALL,0,0)==R_STORAGE && !r.probe.active && brakes[0]);
    assert(r.last_seq==3 && r.last_result==R_STORAGE && !duties[0]);

    /* Complete Modbus commands execute directly; legacy commit never replays. */
    fresh(); uint8_t reply[256]; uint16_t values[2];
    uint8_t stage[]={1,16,0,0x30,0,5,10,0,0,0,CMD_SELFTEST_ALL,0,0,0,0,0,0,0,0};
    uint8_t commit[]={1,6,0,0x35,0xA5,0x5A,0,0};
    size_t sn=adu(stage,17), cn=adu(commit,6);
    assert(robot_modbus(&r,stage,sn,reply)==8 && r.last_seq==1 && r.probe.active);
    assert(!robot_crc16(reply,8) && r.command[0]==0 && saves==1);
    for(unsigned i=0;i<4;i++) assert(r.selftest[i].sequence==1);
    assert(saved.request_sequence==1 && saved_dirty==1);
    assert(robot_modbus(&r,commit,cn,reply)==5 && reply[1]==0x86 && reply[2]==2);
    assert(r.last_seq==1 && saves==1);
    assert(robot_modbus(&r,stage,sn,reply)==5 && reply[1]==0x90 && reply[2]==6);
    assert(r.last_seq==2 && saves==1 && r.selftest[0].sequence==1);
    assert(robot_read(&r,0x36,2,values)==R_OK && values[0]==2 && values[1]==R_BUSY);
    /* CAN echoes wire seq=0; actual allocated identity is queried at 0036. */
    uint8_t stop[8]={3,0,0,CMD_STOP_ALL,0,0,0,0};
    robot_can(&r,stop,reply);
    assert(reply[0]==0x83 && !reply[1] && !reply[2] && !reply[3]);
    assert(r.last_seq==3 && r.probe.aborting && brakes[0]);
    robot_init(&reboot,&ops,&saved,1,uid,0); robot_update(&reboot,600,f);
    assert(!reboot.probe.active && !reboot.motor[0].valid && brakes[0]);

    fresh(); uint8_t batch[8]={3,0,0,CMD_SELFTEST_ALL,0,0,0,0};
    robot_can(&r,batch,reply); assert(!reply[2] && r.last_seq==1 && saved.request_sequence==1);
    robot_can(&r,batch,reply); assert(reply[2]==R_BUSY && r.last_seq==2 && saves==1);
    robot_can(&r,stop,reply); assert(!reply[2] && r.probe.aborting && r.last_seq==3);
}
static void test_unexpected_angle_threshold(void)
{
    const uint32_t gears[]={516,290,82}, limits[]={86,48,13};
    for(unsigned k=0;k<3;k++) {
        fresh(); r.staged.channel[0].gear=gears[k];
        assert(command(CMD_CONFIG,0,0)==R_OK);
        assert(robot_unexpected_fg_limit(gears[k])==limits[k]);
        f[0].count=limits[k]; robot_update(&r,700,f);
        assert(!r.motor[0].fault);
        f[0].count++; robot_update(&r,710,f);
        assert(r.motor[0].fault==F_UNEXPECTED);
    }
    assert(robot_unexpected_fg_limit(0)==UINT32_MAX);
    assert(robot_unexpected_fg_limit(123)==UINT32_MAX);
    fresh(); r.staged.channel[0].gear=82; /* staging alone does not enable */
    f[0].count=100; robot_update(&r,700,f); assert(!r.motor[0].fault);
    robot_update(&r,1200,f); save_fail=1;
    assert(command(CMD_CONFIG,0,0)==R_STORAGE && r.config.channel[0].gear==0);
    f[0].count++; robot_update(&r,1210,f); assert(!r.motor[0].fault);
    robot_update(&r,1710,f); save_fail=0;
    assert(command(CMD_CONFIG,0,0)==R_OK);
    /* Existing accumulation is preserved, including activity before CONFIG. */
    f[0].count++; robot_update(&r,1720,f); assert(r.motor[0].fault==F_UNEXPECTED);
    fresh(); f[0].count=UINT32_MAX; robot_update(&r,700,f);
    assert(!r.motor[0].fault && r.motor[0].unexpected_fg==UINT32_MAX);
    f[0].count++; robot_update(&r,710,f); assert(!r.motor[0].fault);
}

static void test_unexpected_fg(void)
{
    ready(); unsigned n=saves;
    /* Independent totals, accumulated one at a time across long quiet gaps. */
    for(unsigned i=1;i<=13;i++) {
        f[1].count=i; robot_update(&r,600+400*i,f);
        assert(!r.motor[1].fault && r.motor[1].valid && r.motor[1].position==0);
        assert(r.motor[1].unexpected_fg==i && !r.motor[0].unexpected_fg);
        assert(saves==n);
    }
    assert(command(CMD_CLEAR_FAULT,1,0)==R_BUSY && r.motor[1].unexpected_fg==13);
    f[2].count=10; robot_update(&r,6700,f);
    assert(!r.motor[2].fault && r.motor[2].unexpected_fg==10);
    f[1].count++; robot_update(&r,6800,f);
    assert(r.motor[1].fault==F_UNEXPECTED && !r.motor[1].valid && brakes[1] && !duties[1]);
    assert(r.motor[1].unexpected_fg==0 && r.motor[1].recovery_events==1 && r.pending_save && saves==n);
    robot_update(&r,7300,f); assert(saves==n+1 && !r.pending_save);
    assert(command(CMD_CLEAR_FAULT,1,0)==R_OK && !r.motor[1].unexpected_fg);
    assert(!r.motor[1].valid && r.motor[2].unexpected_fg==10);
    f[1].count+=13; robot_update(&r,7400,f); assert(!r.motor[1].fault);
    robot_update(&r,7900,f);
    save_fail=1; assert(command(CMD_ZERO,1,0)==R_STORAGE && r.motor[1].unexpected_fg==13);
    save_fail=0; assert(command(CMD_ZERO,1,0)==R_OK && !r.motor[1].unexpected_fg);
    assert(r.motor[1].valid);

    /* A burst, saturating accumulation and hardware count wrap cannot bypass. */
    ready(); f[0].count=100; robot_update(&r,700,f);
    assert(r.motor[0].fault==F_UNEXPECTED && r.motor[0].unexpected_fg==0);
    r.motor[0].unexpected_fg=UINT32_MAX-2;
    f[0].count+=3; robot_update(&r,710,f); assert(r.motor[0].unexpected_fg==(uint32_t)(((uint64_t)UINT32_MAX+1U)%87U));
    ready(); r.motor[0].count=f[0].count=UINT32_MAX-5;
    f[0].count+=86; robot_update(&r,700,f); assert(!r.motor[0].fault && r.motor[0].unexpected_fg==86);
    f[0].count++; robot_update(&r,710,f); assert(r.motor[0].fault==F_UNEXPECTED);

    /* Commanded motion, including coast, neither consumes nor resets budget. */
    ready(); f[0].count=86; robot_update(&r,700,f); robot_update(&r,1200,f);
    assert(command(CMD_MOVE,0,200)==R_OK);
    uint32_t target=(uint32_t)r.motor[0].target;
    f[0].count+=target; robot_update(&r,1210,f);
    f[0].count++; robot_update(&r,1220,f); robot_update(&r,1720,f);
    assert(r.motor[0].state==S_DONE && r.motor[0].position==(int32_t)target+1);
    assert(r.motor[0].unexpected_fg==86 && !r.motor[0].fault);
    f[0].count++; robot_update(&r,1730,f); assert(r.motor[0].fault==F_UNEXPECTED);

    /* Other-channel tolerance persists through outbound, coast and return. */
    ready(); f[2].count=46; robot_update(&r,700,f); robot_update(&r,1200,f);
    assert(command(CMD_SELFTEST,0,0)==R_OK);
    f[2].count=47; f[0].count=3; robot_update(&r,1210,f);
    assert(!r.probe.aborting && r.selftest[0].phase==SP_OUTBOUND_SETTLING);
    f[2].count=48; robot_update(&r,1220,f); robot_update(&r,1720,f);
    assert(r.selftest[0].phase==SP_RETURN && !r.probe.aborting);
    assert(!r.motor[0].unexpected_fg && r.motor[2].valid && r.motor[2].position==0);
    f[2].count=49; robot_update(&r,1730,f);
    assert(r.probe.aborting && r.selftest[0].reason==ST_OTHER_ACTIVITY);
    assert(r.motor[2].fault==F_UNEXPECTED && !r.motor[2].valid && !r.motor[0].valid);

    /* Tolerated edges do not hide capture errors, or reset the quiet timer. */
    ready(); assert(command(CMD_SELFTEST,0,0)==R_OK);
    f[2].count=1; f[2].errors=1; robot_update(&r,610,f);
    assert(r.motor[2].fault==F_CAPTURE && r.selftest[0].reason==ST_CAPTURE_ERROR);
    ready(); f[0].count=86; robot_update(&r,700,f);
    assert(command(CMD_SELFTEST,0,0)==R_BUSY);
    robot_t reboot; robot_init(&reboot,&ops,&saved,0,uid,0);
    for(unsigned i=0;i<4;i++) assert(!reboot.motor[i].unexpected_fg);
}

static void test_relative_without_zero(void)
{
    /* Configure without ZERO. Preserve a nonzero diagnostic counter. */
    fresh(); r.staged.channel[0]=(robot_config_t){1,1,516,0,400};
    assert(command(CMD_CONFIG,0,0)==R_OK);
    r.motor[0].position=1234;
    unsigned n=saves;
    assert(!r.motor[0].valid && command(CMD_MOVE,0,300)==R_OK);
    int32_t target=r.motor[0].target;
    assert(target==1492 && r.motor[0].position==1234 && !r.motor[0].valid);
    assert(saved_dirty && saved.position[0]==1234 && !saved.valid[0] && saves==n+1);
    f[0].count=258; robot_update(&r,700,f); robot_update(&r,1201,f);
    assert(r.motor[0].state==S_DONE && !r.motor[0].fault && !r.motor[0].valid);
    assert(r.motor[0].position==target && saved.position[0]==target && !saved.valid[0]);
    assert(!saved_dirty && brakes[0] && !duties[0]);
    /* Clean reboot keeps an unknown zero, but permits another relative move. */
    robot_t boot; robot_init(&boot,&ops,&saved,0,uid,1300);
    robot_feedback_t bf[4]={{0}}; robot_update(&boot,1900,bf);
    assert(!boot.motor[0].valid && boot.motor[0].position==target);
    assert(robot_command(&boot,0,CMD_MOVE,0,-300)==R_OK);
    assert(boot.motor[0].target==1234 && boot.motor[0].direction==1 && !boot.motor[0].valid);
    bf[0].count=258; robot_update(&boot,2000,bf); robot_update(&boot,2501,bf);
    assert(boot.motor[0].state==S_DONE && !boot.motor[0].valid && !saved.valid[0]);
    /* Dirty recovery still does not restart or invent a zero. */
    robot_init(&boot,&ops,&saved,1,uid,2600); memset(bf,0,sizeof(bf));
    assert(!boot.motor[0].valid && brakes[0] && !duties[0]);
    robot_update(&boot,3200,bf);
    assert(robot_command(&boot,0,CMD_MOVE,0,100)==R_OK && !boot.motor[0].valid);

    /* An existing fault remains a gate; clearing it never restores the zero. */
    ready(); f[0].count=87; robot_update(&r,700,f); robot_update(&r,1201,f);
    assert(command(CMD_MOVE,0,100)==R_UNREADY);
    assert(command(CMD_CLEAR_FAULT,0,0)==R_OK && !r.motor[0].valid);
    assert(command(CMD_MOVE,0,100)==R_OK && !r.motor[0].valid);
    robot_update(&r,2300,f); assert(r.motor[0].fault==F_FEEDBACK && !r.motor[0].valid);
    ready(); r.motor[0].valid=0; save_fail=1;
    assert(command(CMD_MOVE,0,100)==R_STORAGE && brakes[0] && !duties[0]);
    ready(); r.motor[0].valid=0; r.stop_input=1;
    assert(command(CMD_MOVE,0,100)==R_BUSY);
    r.stop_input=0; r.probe.stop_fault_mask=1;
    assert(command(CMD_MOVE,0,100)==R_UNREADY);
    r.probe.stop_fault_mask=0; r.motor[0].last_edge_ms=r.now_ms;
    assert(command(CMD_MOVE,0,100)==R_BUSY);
    ready(); r.motor[0].valid=0; r.motor[0].position=INT32_MAX-10;
    assert(command(CMD_MOVE,0,3600)==R_VALUE);
    ready(); r.motor[0].valid=0; assert(command(CMD_MOVE,0,100)==R_OK);
    f[0].errors=1; robot_update(&r,650,f); assert(r.motor[0].fault==F_CAPTURE);
    ready(); r.motor[0].valid=0; assert(command(CMD_MOVE,0,100)==R_OK);
    f[0].count=200; robot_update(&r,700,f); robot_update(&r,1201,f);
    assert(r.motor[0].fault==F_POSITION && !r.motor[0].valid);

    /* Both wire protocols use the same policy; replay never repeats a move. */
    ready(); r.motor[0].valid=0;
    uint8_t q[19]={1,16,0,0x30,0,5,10,0,0,0,CMD_MOVE,0,0,0,0,0,100};
    uint8_t reply[256]; assert(robot_modbus(&r,q,adu(q,17),reply)==8);
    assert(!r.motor[0].valid && r.motor[0].target==86);
    ready(); r.motor[0].valid=0; r.config.channel[0].inverted=1;
    uint16_t request=(uint16_t)(seq+1);
    uint8_t can[8]={3,(uint8_t)(request>>8),(uint8_t)request,CMD_MOVE,0,0xFF,0x9C,0};
    robot_can(&r,can,reply); assert(reply[2]==R_OK && r.motor[0].target==-86);
    assert(!r.motor[0].valid && r.motor[0].direction==0);
    n=saves; robot_can(&r,can,reply);
    assert(reply[2]==R_OK && r.motor[0].target==-86 && saves==n);
}

static void test_manual_adjust(void)
{
    ready(); unsigned n=saves; int32_t pos=r.motor[0].position;
    assert(command(CMD_MANUAL_ADJUST,0,1)==R_OK);
    assert(saves==n+1 && saved_dirty && !r.motor[0].valid);
    assert(r.motor[0].manual==1 && r.motor[0].state==S_MANUAL && !brakes[0] && !duties[0]);
    assert(command(CMD_MANUAL_ADJUST,0,1)==R_OK && saves==n+1);
    assert(command(CMD_MANUAL_ADJUST,2,1)==R_OK && saves==n+1);
    assert(command(CMD_MOVE,1,100)==R_BUSY);
    assert(command(CMD_SELFTEST_ALL,0,0)==R_BUSY);
    assert(command(CMD_CONFIG,0,0)==R_BUSY);
    assert(command(CMD_ZERO,0,0)==R_BUSY);
    assert(command(CMD_SAVE,0,0)==R_BUSY);
    f[0].count=10000; robot_update(&r,700,f);
    robot_update(&r,6000,f);
    assert(!r.motor[0].fault && r.motor[0].position==pos && !r.motor[0].valid && saves==n+1);
    assert(command(CMD_MANUAL_ADJUST,0,0)==R_OK && brakes[0] && r.motor[0].manual==2);
    f[0].count+=500; robot_update(&r,6100,f); robot_update(&r,6599,f);
    assert(r.motor[0].manual==2 && !r.motor[0].fault);
    robot_update(&r,6600,f); assert(!r.motor[0].manual && saves==n+1);
    assert(command(CMD_STOP_ALL,0,0)==R_OK && brakes[2]);
    robot_update(&r,7100,f); assert(!r.motor[2].manual && saves==n+2 && !saved_dirty);
    assert(!saved.valid[0] && !saved.valid[2] && saved.valid[1]);
    ready(); save_fail=1;
    assert(command(CMD_MANUAL_ADJUST,0,1)==R_STORAGE && brakes[0] && r.motor[0].valid && !r.motor[0].manual);
    ready(); f[0].count=87; robot_update(&r,700,f); robot_update(&r,1200,f);
    assert(command(CMD_MANUAL_ADJUST,0,1)==R_OK && !r.motor[0].fault);
    f[0].errors=1; robot_update(&r,1210,f);
    assert(r.motor[0].fault==F_CAPTURE && brakes[0] && r.motor[0].manual==2);
    robot_update(&r,1710,f); assert(!r.motor[0].manual);
    ready(); r.motor[0].fault=F_CAPTURE;
    assert(command(CMD_MANUAL_ADJUST,0,1)==R_UNREADY);
    ready(); r.stop_input=1; assert(command(CMD_MANUAL_ADJUST,0,1)==R_BUSY);
    ready(); assert(command(CMD_MANUAL_ADJUST,0,1)==R_OK);
    robot_init(&r,&ops,&saved,1,uid,0); assert(!r.motor[0].manual && brakes[0] && !r.motor[0].valid);
    ready(); assert(command(CMD_MANUAL_ADJUST,0,1)==R_OK);
    assert(command(CMD_BRAKE,0,1)==R_OK); save_fail=1; robot_update(&r,1100,f);
    assert(r.storage_error && r.pending_save && brakes[0]);
    ready();
    uint8_t request[]={1,16,0,0x30,0,5,10,0,0,0,CMD_MANUAL_ADJUST,0,1,0,0,0,1,0,0};
    uint8_t reply[64]; uint16_t crc=robot_crc16(request,17); request[17]=(uint8_t)crc; request[18]=(uint8_t)(crc>>8);
    assert(robot_modbus(&r,request,sizeof(request),reply)==8 && r.motor[1].manual==1);
    unsigned written=saves; uint16_t actual=r.last_seq;
    uint8_t can[]={3,(uint8_t)(actual>>8),(uint8_t)actual,CMD_MANUAL_ADJUST,1,0,1,0};
    assert(robot_can(&r,can,reply) && reply[2]==R_OK && saves==written); /* same sequence/content replay */
    can[1]=0; can[2]=0; can[6]=0; assert(robot_can(&r,can,reply) && reply[2]==R_OK && r.motor[1].manual==2);
    robot_update(&r,1100,f); assert(r.motor[1].manual==0 && brakes[1]);
    uint16_t value; assert(robot_read(&r,0x10,1,&value)==R_OK && value==3);
    assert(robot_read(&r,0x130,1,&value)==R_OK && value==0);
}
static void test_fault4_recovery(void)
{
    const unsigned gears[3]={516,290,82};
    for(unsigned k=0;k<3;k++) {
        ready(); r.config.channel[0].gear=gears[k]; unsigned threshold=robot_unexpected_fg_limit(gears[k])+1;
        f[0].count=threshold; robot_update(&r,700,f);
        assert(r.motor[0].fault==F_UNEXPECTED && r.motor[0].recovery_events==1);
        f[0].count++; robot_update(&r,800,f); assert(r.motor[0].recovery_events==1);
        f[0].count+=threshold-1; robot_update(&r,1000,f);
        assert(r.motor[0].recovery_events==2 && r.motor[0].recovery_ms==700);
        robot_update(&r,5699,f); assert(r.motor[0].fault==F_UNEXPECTED);
        unsigned n=saves; robot_update(&r,5700,f);
        assert(!r.motor[0].fault && !r.motor[0].valid && brakes[0] && !duties[0] && saves==n);
        assert(!r.motor[0].unexpected_fg && !r.motor[0].recovery_events);
        ready(); r.config.channel[0].gear=gears[k];
        f[0].count=threshold; robot_update(&r,700,f);
        f[0].count+=threshold; robot_update(&r,1000,f);
        f[0].count+=threshold; robot_update(&r,5700,f);
        assert(r.motor[0].fault==F_UNEXPECTED && r.motor[0].recovery_events==1 && r.motor[0].recovery_ms==5700);
        robot_update(&r,10699,f); assert(r.motor[0].fault==F_UNEXPECTED);
        robot_update(&r,10700,f); assert(!r.motor[0].fault);
    }
    ready(); f[0].count=87; f[0].errors=1; robot_update(&r,700,f); robot_update(&r,6000,f);
    assert(r.motor[0].fault==F_CAPTURE);
    ready(); assert(command(CMD_SELFTEST,0,0)==R_OK);
    f[1].count=14; robot_update(&r,700,f);
    assert(r.motor[1].recovery_events==1 && r.motor[1].fault==F_UNEXPECTED);
    f[1].count+=14; robot_update(&r,1000,f); assert(r.motor[1].recovery_events==2);
    f[1].count+=14; robot_update(&r,5700,f);
    assert(r.motor[1].recovery_events==1 && r.motor[1].recovery_ms==5700);
    robot_update(&r,10699,f); assert(r.motor[1].fault==F_UNEXPECTED);
    robot_update(&r,10700,f); assert(!r.motor[1].fault && brakes[1]);
    ready(); uint32_t start=UINT32_MAX-2000U; f[0].count=87; robot_update(&r,start,f);
    robot_update(&r,start+4999U,f); assert(r.motor[0].fault==F_UNEXPECTED);
    robot_update(&r,start+5000U,f); assert(!r.motor[0].fault);
    ready(); f[0].count=87; robot_update(&r,700,f); f[0].count=UINT32_MAX; robot_update(&r,800,f);
    assert(r.motor[0].fault==F_UNEXPECTED); /* bounded handling of large batches */
}
int main(void)
{
    test_manual_adjust(); test_fault4_recovery(); puts("manual adjustment / interlocks / dirty / fault4 recovery windows: PASS");
    test_relative_without_zero(); puts("relative MOVE / unknown zero / persistence / reboot / faults / Modbus-CAN: PASS");
    test_motion(); puts("motion / four-channel independence / replay / reboot: PASS");
    test_faults(); puts("faults / limits / config invalidation / sequence wrap: PASS");
    test_unexpected_angle_threshold();
    test_unexpected_fg(); puts("unexpected FG / cumulative output angle >10 deg boundary / independent channels / reset / wrap / motion / selftest: PASS");
    test_store(); puts("Flash journal / 129 interruption cases / rollover / CRC: PASS");
    test_direct_commands(); puts("direct commands / revision / atomic writes / old commit / replay / STOP / failures: PASS");
    test_protocol(); puts("Modbus / CAN / 20000 malformed ADUs: PASS");
    test_framing(); puts("RTU gaps / overflow / errors / timestamp wrap: PASS");
    test_selftest(); puts("selftest admission / ramp / timing / coast / batch / cancel / storage / position: PASS");
    test_selftest_protocol(); puts("selftest Modbus / CAN / replay / legacy register compatibility: PASS");
    test_discovery(); puts("unconfigured four-channel discovery / mixed response / safety abort / transports: PASS");
    test_roundtrip(); puts("roundtrip / coast target / signed residual / return timeout / phase cancel / safety: PASS");
    test_auto_sequence(); puts("auto sequence / direct command / explicit replay / wrap cache / reboot / interlocks / Modbus-CAN: PASS");
    return 0;
}
