"""Offline firmware-model and worker tests; never connect to real serial ports."""
import time
import pytest
from robot_motor_driver import protocol as p
from robot_motor_driver.fake_board import make_fake_stack
from robot_motor_driver.scheduler import TaskRequest
from test_scheduler import build, shutdown, commissioning

@pytest.mark.parametrize("gear, event", [(516,87),(290,49),(82,14)])
def test_fault4_new_events_and_restart(gear,event):
    transport, b, stub, session, clock = make_fake_stack()
    clock.auto_advance=0; clock.advance(1)
    c=b.channels[0]; c.gear=gear
    b.inject_unexpected_pulses(0,event-1); assert c.fault==0
    b.inject_unexpected_pulses(0,1); assert c.fault==4 and c.recovery_events==1
    start=c.recovery_started
    b.inject_unexpected_pulses(0,1); assert c.recovery_events==1
    clock.advance(4.999)
    b.inject_unexpected_pulses(0,event-1); assert c.recovery_events==2 and c.recovery_started==start
    clock.advance(.001)
    b.inject_unexpected_pulses(0,event); b.advance(0)
    assert c.fault==4 and c.recovery_events==1
    b.advance(4.999); assert c.fault==4
    b.advance(.002); assert c.fault==0 and not c.valid and c.brake and c.duty==0
    assert c.unexpected_fg==0 and c.recovery_events==0

def test_manual_multi_dirty_freeze_and_final_save():
    _,b,_,s,clock=make_fake_stack();clock.auto_advance=0;clock.advance(1)
    assert s.send_command(p.Opcode.MANUAL_ADJUST,0,1).accepted
    c=b.channels[0]; pos=c.fg_count; direction=c.direction; writes=b.flash_writes
    assert c.manual==1 and not c.valid and c.duty==0 and not c.brake
    b.inject_unexpected_pulses(0,10000);b.advance(.501)
    assert c.fg_count==pos and c.direction==direction and c.fault==0
    assert s.send_command(p.Opcode.MANUAL_ADJUST,1,1).accepted
    assert b.flash_writes==writes
    assert not s.send_command(p.Opcode.MOVE,2,50).accepted
    assert not s.send_command(p.Opcode.SELFTEST,2,0).accepted
    assert not s.send_command(p.Opcode.CONFIG,0,0).accepted
    assert s.send_command(p.Opcode.MANUAL_ADJUST,0,0).accepted
    assert c.brake and c.manual==2
    b.inject_unexpected_pulses(0,999);b.advance(.499);assert c.manual==2
    b.advance(.002);assert c.manual==0 and b.flash_writes==writes
    assert s.send_command(p.Opcode.STOP_ALL,0,0).accepted
    b.advance(.501);b.advance(.1)
    assert not any(x.manual for x in b.channels)
    assert not b.dirty and not b.pending_save and not b.storage_error
    assert not c.valid and not b.channels[1].valid

@pytest.mark.parametrize("fault",[1,2,3,5,6,7])
def test_other_fault_refuses_manual(fault):
    _,b,_,s,clock=make_fake_stack();clock.auto_advance=0;clock.advance(1)
    b.inject_fault(0,fault)
    assert not s.send_command(p.Opcode.MANUAL_ADJUST,0,1).accepted
    b.advance(6); assert b.channels[0].fault==fault

def test_manual_dirty_failure_reboot_and_capture_stop():
    _,b,_,s,clock=make_fake_stack();clock.auto_advance=0;clock.advance(1)
    b.storage_failure=True
    assert not s.send_command(p.Opcode.MANUAL_ADJUST,0,1).accepted
    assert b.channels[0].brake and b.channels[0].manual==0
    b.storage_failure=False;b.storage_error=False
    assert s.send_command(p.Opcode.MANUAL_ADJUST,0,1).accepted
    b.inject_fault(0,3);assert b.channels[0].manual==2 and b.channels[0].brake
    b.reboot();assert all(x.manual==0 and x.brake for x in b.channels)
    assert not b.channels[0].valid

def test_worker_manual_service_maintenance_and_stop():
    w,s,b,_,_=build(allow_motion=False)
    try:
        h=w.submit(TaskRequest(kind="manual_adjust",channel=0,argument=1))
        assert h.wait(3) and h.result.accepted and h.result.manual_mode==1
        assert not h.result.position_valid and not h.result.persisted
        assert w.refresh_now().manual_modes[0]==1
        h=w.request_stop(0);assert h.wait(3) and h.result.stop_confirmed
        assert b.channels[0].manual==0 and b.channels[0].brake
    finally: shutdown(w)

def test_worker_old_firmware_and_readonly_no_new_command():
    for readonly in (False,True):
        w,s,b,stub,_=build(read_only=readonly)
        try:
            s.capabilities=0; before=b.commands_executed
            h=w.submit(TaskRequest(kind="manual_adjust",channel=0,argument=1))
            assert h.wait(3) and not h.result.accepted
            assert b.commands_executed==before
        finally: shutdown(w)

def test_worker_pending_motion_rejects_manual():
    w,s,b,_,_=build(motion_scale=.5)
    try:
        m=w.submit(TaskRequest(kind="move",channel=0,opcode=p.Opcode.MOVE,argument=100,**commissioning()))
        h=w.submit(TaskRequest(kind="manual_adjust",channel=1,argument=1))
        assert h.rejected and not h.result.accepted
        w.request_stop(0).wait(3);m.wait(3)
    finally: shutdown(w)


def test_capability_missing_only_illegal_address_means_unsupported():
    from robot_motor_driver.board_session import BoardSession
    t,b,_,s,_=make_fake_stack()
    original=t.read_registers
    def legacy(address,count):
        if address==p.REG_CAPABILITIES: raise p.ModbusExceptionResponse(2,"legacy hole")
        return original(address,count)
    t.read_registers=legacy
    s.identify(); assert s.capabilities==0
    def broken(address,count):
        if address==p.REG_CAPABILITIES: raise p.ModbusExceptionResponse(4,"broken board")
        return original(address,count)
    t.read_registers=broken
    with pytest.raises(p.ModbusExceptionResponse):s.identify()

def test_worker_manual_blocks_other_channel_motion_without_bus_write():
    w,s,b,_,_=build()
    try:
        h=w.submit(TaskRequest(kind="manual_adjust",channel=0,argument=1));assert h.wait(3) and h.result.accepted
        before=b.commands_executed
        h=w.submit(TaskRequest(kind="move",channel=2,opcode=p.Opcode.MOVE,argument=50,**commissioning()))
        assert h.rejected and not h.result.accepted and b.commands_executed==before
        w.request_stop().wait(3)
    finally:shutdown(w)


def test_manual_readback_failure_preserves_accepted_sequence():
    w,s,b,_,_=build(allow_motion=False)
    try:
        def broken_flags():raise p.ProtocolError("save read unavailable")
        s.read_save_flags=broken_flags
        h=w.submit(TaskRequest(kind="manual_adjust",channel=0,argument=1))
        assert h.wait(3) and h.result.accepted and h.result.sequence!=0 and h.result.unknown
        assert not h.result.persisted and b.channels[0].manual==1
        w.request_stop(0).wait(3)
    finally:shutdown(w)
