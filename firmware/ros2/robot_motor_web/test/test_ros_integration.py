import asyncio,copy,os,threading,time
from pathlib import Path
import pytest,yaml
os.environ.setdefault('ROS_DOMAIN_ID','174')
rclpy=pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from robot_motor_driver.config import load_config_from_text
from robot_motor_driver.motor_node import RobotMotorNode
from robot_motor_web.runtime import RuntimeManager
from robot_motor_web.bridge import RosBridge
from robot_motor_web.gateway import Gateway
from robot_motor_web.api import create_app
from fastapi.testclient import TestClient
from test_web import config,completed

@pytest.mark.parametrize('board', ['main_board','right_board'])
def test_ros_fake_services_actions_and_braking(tmp_path,board):
 raw=config();f=tmp_path/'robot.yaml';f.write_text(yaml.safe_dump(raw));m=RuntimeManager(f,tmp_path/'state',domain=174)
 for b,r in m.records.items():
  r['options']['namespace']='webtest_'+('a' if b=='main_board' else 'b')
  m.active[b]=copy.deepcopy(r);m.active[b]['options'].update(mode='motion',allow_unreferenced_relative=True)
 if not rclpy.ok():rclpy.init()
 executor=MultiThreadedExecutor(num_threads=4);nodes=[]
 for b in raw['boards']:
  overrides=[Parameter('board_name',value=b),Parameter('transport',value='fake'),Parameter('allow_motion',value=True),
             Parameter('maintenance',value=False),Parameter('allow_unreferenced_relative',value=True),Parameter('commissioning',value=False),Parameter('polling_hz',value=20.0)]
  node=RobotMotorNode(config=load_config_from_text(yaml.safe_dump(raw)),namespace=m.active[b]['options']['namespace'],parameter_overrides=overrides)
  # Configure only the in-memory fake board, before executor starts. No wire command.
  for ch in node._fake_board.channels:
   ch.enabled=False;ch.role=0;ch.gear=0;ch.rpm=400;ch.last_edge=node._fake_board._now()-1
  for j,v in m.active[b]['joints'].items():
   ch=node._fake_board.channels[v['channel']-1];ch.enabled=True;ch.role=v['role'];ch.gear=v['gear_ratio'];ch.inverted=0;ch.rpm=400;ch.valid=True
  nodes.append(node);executor.add_node(node)
 thread=threading.Thread(target=executor.spin,daemon=True);thread.start();bridge=RosBridge(m);g=Gateway(m,bridge,m.directory)
 try:
  end=time.monotonic()+8
  while time.monotonic()<end:
   if bridge.snapshot(board).get('identity_ok'):break
   time.sleep(.1)
  with TestClient(create_app(g)) as client:
   h={'X-CSRF-Token':client.get('/api/v1/session').json()['csrf'],'X-Request-ID':'claim'}
   assert client.post('/api/v1/control/acquire',json={},headers=h).status_code==200
   joint='head_joint' if board=='main_board' else 'right_forearm_joint'
   base='/api/v1/boards/'+board+'/joints/'+joint
   def send(path,p):
    assert client.post('/api/v1/control/heartbeat',json={},headers=h).status_code==200
    h['X-Request-ID']=str(time.time_ns());r=client.post(path,json=p,headers=h);assert r.status_code==202,r.text
    ident=r.json()['id'];last=time.monotonic()
    for _ in range(2000):
     result=client.get('/api/v1/jobs/'+ident).json()
     if result['status'] in ('succeeded','failed','unknown'):break
     if time.monotonic()-last>.5:
      assert client.post('/api/v1/control/heartbeat',json={},headers=h).status_code==200;last=time.monotonic()
     time.sleep(.01)
    assert result['status']=='succeeded',result;return result
   send(base+'/move',dict(mode=0,angle_deg=5,motor_rpm=400,timeout_sec=10))
   send(base+'/move',dict(mode=1,angle_deg=0,motor_rpm=0,timeout_sec=10))
   send(base+'/zero',dict(confirm=True))
   time.sleep(.6)
   before=copy.deepcopy(next(n for n in nodes if n.board_name==board)._fake_board.channels[m.active[board]['joints'][joint]['channel']-1])
   high=send(base+'/direction',dict(high=True));assert high['result']['direction'] and high['result']['direction_confirmed']
   low=send(base+'/direction',dict(high=False));assert not low['result']['direction'] and low['result']['direction_confirmed']
   after=next(n for n in nodes if n.board_name==board)._fake_board.channels[m.active[board]['joints'][joint]['channel']-1]
   assert after.fg_count==before.fg_count and after.valid==before.valid and after.brake
   send(base+'/manual-adjust',dict(enable=True))
   r=client.post('/api/v1/control/release',json={},headers=dict(h,**{'X-Request-ID':'release'}))
   assert r.status_code==200 and r.json()['cleanup'][0]['stop_confirmed'],r.text
   h['X-Request-ID']=str(time.time_ns());assert client.post('/api/v1/control/acquire',json={},headers=h).status_code==200
   send(base+'/zero',dict(confirm=True))
   time.sleep(.6)  # Respect firmware quiet time after ZERO/manual recovery.
   h['X-Request-ID']=str(time.time_ns());r=client.post(base+'/move',json=dict(mode=0,angle_deg=30,motor_rpm=400,timeout_sec=30),headers=h)
   assert r.status_code==202,r.text
   move_id=r.json()['id'];end=time.monotonic()+3
   while move_id not in bridge.goals and time.monotonic()<end:time.sleep(.01)
   assert move_id in bridge.goals
   channel=m.active[board]['joints'][joint]['channel']-1
   end=time.monotonic()+3
   while time.monotonic()<end:
    snapshot=bridge.snapshot(board)
    if snapshot.get('channels',[])[channel]['state'] in (2,3):break
    client.post('/api/v1/control/heartbeat',json={},headers=h);time.sleep(.05)
   assert bridge.snapshot(board)['channels'][channel]['state'] in (2,3),client.get('/api/v1/jobs/'+move_id).json()
   h['X-Request-ID']=str(time.time_ns());cancel=client.post('/api/v1/jobs/'+move_id+'/cancel',json={},headers=h)
   assert cancel.status_code==200 and cancel.json()['result']['cancel_requested'],cancel.text
   original=completed(client,move_id);assert original['status']=='cancelled',original
   h['X-Request-ID']=str(time.time_ns());assert client.post('/api/v1/control/release',json={},headers=h).status_code==200
   assert all(node._effective_transport=='fake' for node in nodes)
   h['X-Request-ID']=str(time.time_ns());assert client.post('/api/v1/control/acquire',json={},headers=h).status_code==200
   target=next(n for n in nodes if n.board_name==board)
   target.worker.save_recovery_only=True
   g.manager.active[board]['options']['mode']='save_recovery'
   fake=target._fake_board
   fake.storage_error=True;fake.dirty=True;fake.pending_save=True
   ch=fake.channels[0];ch.fault=1;ch.valid=False;ch.fg_count=123;ch.last_edge=fake._now()-2
   time.sleep(.3)
   save=send('/api/v1/boards/'+board+'/save',{'confirm':True})
   assert save['result']['flags_valid'] and save['result']['persisted']
   assert not any(save['result'][k] for k in ('dirty','storage_error','pending_save'))
   assert ch.fault==1 and not ch.valid and ch.fg_count==123
   h['X-Request-ID']=str(time.time_ns());assert client.post('/api/v1/control/release',json={},headers=h).status_code==200

 finally:
  # TestClient shuts down the bridge without shutting this shared ROS context.
  executor.shutdown(timeout_sec=3);thread.join(timeout=3)
  for n in nodes:n.destroy_node()
  rclpy.shutdown()


def test_save_recovery_node_permissions_and_confirmation():
 from robot_motor_interfaces.srv import RetrySave,SetZero,ApplyConfig,SetDirection
 raw=config()
 if not rclpy.ok():rclpy.init()
 node=None
 try:
  node=RobotMotorNode(config=load_config_from_text(yaml.safe_dump(raw)),namespace='save_recovery_test',parameter_overrides=[
   Parameter('board_name',value='main_board'),Parameter('transport',value='fake'),
   Parameter('allow_motion',value=True),Parameter('save_recovery_only',value=True)])
  assert not node.allow_motion and node.maintenance and not node.session.read_only
  fake=node._fake_board
  for ch in fake.channels:ch.last_edge=fake._now()-2
  fake.storage_error=True;fake.dirty=True;fake.pending_save=True
  r=node._on_retry_save(RetrySave.Request(confirm=False),RetrySave.Response())
  assert not r.accepted and r.result_code==114
  r=node._on_set_direction(SetDirection.Request(joint_name='head_joint',high=True),SetDirection.Response())
  assert not r.accepted and r.result_code==100 and not r.direction_confirmed
  r=node._on_set_zero(SetZero.Request(joint_name='head_joint'),SetZero.Response())
  assert not r.accepted and r.result_code==100
  r=node._on_apply_config(ApplyConfig.Request(channels=[],commit=True),ApplyConfig.Response())
  assert not r.accepted
  r=node._on_retry_save(RetrySave.Request(confirm=True),RetrySave.Response())
  assert r.accepted and r.persisted and r.flags_valid and not r.storage_error
  node.session.read_only=True
  r=node._on_retry_save(RetrySave.Request(confirm=True),RetrySave.Response())
  assert not r.accepted and r.result_code==100
 finally:
  if node is not None:node.destroy_node()
  rclpy.shutdown()
