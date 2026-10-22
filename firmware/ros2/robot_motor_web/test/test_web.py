import asyncio,copy,json,time,uuid
from pathlib import Path
import pytest,yaml
from fastapi.testclient import TestClient
from robot_motor_web.api import create_app
from robot_motor_web.runtime import RuntimeManager
from robot_motor_web.gateway import Gateway,Refused
from robot_motor_web.models import RuntimeEdit,RuntimeOptions,Move,Channel
from robot_motor_web.store import JobStore

LABELS={'main_board': [('left_upper_arm_joint',1,2,516),('head_joint',2,1,516),('left_forearm_joint',3,4,290),('left_wrist_joint',4,6,82)],
        'right_board':[('right_upper_arm_joint',1,3,516),('right_wrist_joint',2,7,82),('right_forearm_joint',3,5,290)]}

def config():
 return dict(driver={},boards={b:dict(uid=('C23B378115B092083830364A' if b=='main_board' else 'C23B37C715B092083830364A'),transport='fake',port='') for b in LABELS},
  joints={n:dict(board=b,channel=ch,role=role,gear_ratio=gear,inverted=0,expected_motor_rpm=400,limits_deg=[-180,180]) for b,entries in LABELS.items() for n,ch,role,gear in entries})

class FakeBridge:
 def __init__(self,manager):
  self.manager=manager;self.calls=[];self.snapshots={};self.fail=None;self.delay=.01
  for b,r in manager.records.items():
   rows=[dict(channel=i,role=0,enabled=False,gear_ratio=0,inverted=0,configured_rpm=400,state=0,fault=0,position_valid=True,
     brake=True,direction=False,duty_permille=0,measured_rpm=0,angle_deci_deg=0,fg_count=0,target_deci_deg=0,angle_deg=0.,target_deg=0.) for i in range(4)]
   for j,v in r['joints'].items():rows[v['channel']-1].update(role=v['role'],enabled=True,gear_ratio=v['gear_ratio'])
   self.snapshots[b]=dict(uid=r['board']['uid'],identity_ok=True,communication_ok=True,stale=False,read_only=False,unknown_command_state=False,
    storage_error=False,dirty=False,pending_save=False,last_sequence=0,channels=rows,manual={j:dict(mode=0,supported=True,stale=False) for j in r['joints']})
 def snapshot(self,b):return copy.deepcopy(self.snapshots[b])
 def discovered(self):return {r['options']['namespace'] for r in self.manager.active.values()}
 def invalidate(self,b):pass
 def refresh_subscriptions(self):pass
 async def service(self,b,op,p):
  self.calls.append((b,op,p));s=self.snapshots[b]
  if self.fail:raise RuntimeError(self.fail)
  s['last_sequence']+=1;seq=s['last_sequence'];rows=s['channels']
  names=list(self.manager.active[b]['joints']) if not p.get('joint_name') else [p['joint_name']]
  if op=='stop':
   for j in names:
    x=rows[self.manager.active[b]['joints'][j]['channel']-1];x.update(brake=True,duty_permille=0,state=0,measured_rpm=0);s['manual'][j]['mode']=0
   return dict(accepted=True,result_code=0,sequence=seq,stop_confirmed=True)
  if op=='set_direction':
   x=rows[self.manager.active[b]['joints'][p['joint_name']]['channel']-1];x['direction']=p['high']
   return dict(accepted=True,result_code=0,sequence=seq,direction=x['direction'],direction_confirmed=True,unknown=False)
  if op in ('set_brake','set_manual_adjust','set_zero','clear_fault'):
   j=p['joint_name'];x=rows[self.manager.active[b]['joints'][j]['channel']-1]
   if op=='set_brake':x['brake']=p['engage']
   if op=='set_manual_adjust':
    x['brake']=not p['enable'];x['position_valid']=False;x['state']=8 if p['enable'] else 0;s['manual'][j]['mode']=1 if p['enable'] else 0
   if op=='set_zero':x.update(position_valid=True,angle_deg=0,angle_deci_deg=0)
   if op=='clear_fault':x['fault']=0
   return dict(accepted=True,result_code=0,sequence=seq,brake=x['brake'],active=not x['brake'] if op=='set_manual_adjust' else False,
    exiting=False,position_valid=x['position_valid'],persisted=True,fault_after=x['fault'])
  if op=='retry_save':
   s.update(dirty=False,storage_error=False,pending_save=False)
   return dict(accepted=True,result_code=0,sequence=seq,persisted=True,flags_valid=True,dirty=False,storage_error=False,pending_save=False,unknown=False)
  if op=='apply_config':
   if p['commit']:
    for c in p['channels']:rows[c['channel']].update(enabled=c['enabled'],role=c['role'],gear_ratio=c['gear_ratio'],inverted=c['inverted'],configured_rpm=c['rpm'])
   return dict(accepted=True,result_code=0,sequence=seq,persisted=True,staged=p['channels'],applied=p['channels'])
  if op=='acknowledge_unknown':s['unknown_command_state']=False;return dict(success=True)
  return dict(success=True)
 async def action(self,b,op,p,job,feedback):
  self.calls.append((b,op,p));s=self.snapshots[b]
  feedback(dict(goal_accepted=True));await asyncio.sleep(self.delay)
  if self.fail:raise RuntimeError(self.fail)
  if op=='selftest':return dict(accepted=True,completed=True,passed=True,persisted=True,error_code=0,batch_completed=True,all_passed=True)
  x=s['channels'][self.manager.active[b]['joints'][p['joint_name']]['channel']-1]
  feedback(dict(state=3,brake=False));x['direction']=bool((p['angle_deg']<0)^x.get('inverted',0));x['angle_deg']=x['angle_deg']+p['angle_deg'] if p['mode']==0 else p['angle_deg'];x['angle_deci_deg']=int(x['angle_deg']*10)
  return dict(accepted=True,motion_completed=True,persisted=True,position_valid=x['position_valid'],error_code=0,final_angle_deg=x['angle_deg'])
 async def cancel(self,j):return dict(goals_canceling=[{'id':j}],return_code=0)

@pytest.fixture
def env(tmp_path):
 f=tmp_path/'robot.yaml';f.write_text(yaml.safe_dump(config()));m=RuntimeManager(f,tmp_path/'state')
 # Runtime mode used by gateway position check, while fake state itself reports write permission.
 for r in m.active.values():
  r['options']['allow_unreferenced_relative']=True;r['options']['mode']='motion'
 bridge=FakeBridge(m);g=Gateway(m,bridge,m.directory)
 with TestClient(create_app(g)) as c:
  s=c.get('/api/v1/session').json();headers={'X-CSRF-Token':s['csrf'],'X-Request-ID':str(uuid.uuid4())}
  assert c.post('/api/v1/control/acquire',json={},headers=headers).status_code==200
  yield c,g,bridge,headers

def post(env,path,payload):
 c,g,b,h=env;h=dict(h,**{'X-Request-ID':str(uuid.uuid4())});return c.post(path,json=payload,headers=h)

PATH='/api/v1/boards/main_board/joints/head_joint'
def completed(c,ident):
 for _ in range(2000):
  j=c.get('/api/v1/jobs/'+ident).json()
  if j['status'] in ('succeeded','failed','unknown','cancelled'):return j
  time.sleep(.01)
 raise AssertionError('job did not finish')

def test_no_login_and_seven_joints(env):
 c,g,b,h=env;v=c.get('/api/v1/boards').json();assert sum(len(x['state']['joints']) for x in v)==7
 assert c.get('/').status_code==200 and c.get('/api/docs').status_code==200 and c.get('/api/openapi.json').status_code==200
 assert 'get' in c.get('/api/openapi.json').json()['paths']['/api/v1/parameters']

def test_csrf_origin_and_request_id(env):
 c,g,b,h=env
 assert c.post(PATH+'/stop',json={}).status_code==403
 assert c.post(PATH+'/stop',json={},headers=dict(h,Origin='http://evil.invalid')).status_code==403
 assert c.post(PATH+'/stop',json={},headers={'X-CSRF-Token':h['X-CSRF-Token']}).status_code==400
 assert not b.calls

def test_control_exclusive(env):
 c,g,b,h=env;g.owner='other'
 assert post(env,PATH+'/stop',{}).status_code==403

def test_dedup_and_conflict(env):
 c,g,b,h=env;p={'mode':0,'angle_deg':5,'motor_rpm':400,'timeout_sec':30}
 r=c.post(PATH+'/move',json=p,headers=h);assert r.status_code==202
 assert completed(c,r.json()['id'])['status']=='succeeded'
 r2=c.post(PATH+'/move',json=p,headers=h);assert r2.json()['id']==r.json()['id']
 assert len(b.calls)==1
 p['angle_deg']=10;assert c.post(PATH+'/move',json=p,headers=h).status_code==409

@pytest.mark.parametrize('field,value',[('angle_deg','NaN'),('angle_deg','Infinity'),('motor_rpm',1),('motor_rpm',9000),('mode',2),('timeout_sec',0)])
def test_invalid_move(env,field,value):
 p=dict(angle_deg=5);p[field]=value;assert post(env,PATH+'/move',p).status_code==422
 assert not env[2].calls

@pytest.mark.parametrize('field,value',[('read_only',True),('stale',True),('identity_ok',False),('unknown_command_state',True),('storage_error',True)])
def test_board_safety_gate(env,field,value):
 env[2].snapshots['main_board'][field]=value
 assert post(env,PATH+'/move',dict(angle_deg=5)).status_code==409
 assert not env[2].calls

def test_binding_conflict_and_invalid_position(env):
 c,g,b,h=env;x=b.snapshots['main_board']['channels'][1];x['gear_ratio']=82
 assert post(env,PATH+'/brake',{'engage':False}).status_code==409
 x['gear_ratio']=516;x['position_valid']=False
 v=c.get(PATH+'/state').json();assert v['angle_deg'] is None and v['angle_deci_deg']==0
 assert post(env,PATH+'/move',dict(mode=1,angle_deg=0)).status_code==409

def test_motion_absolute_zero_config_and_probe(env):
 c,g,b,h=env
 for p in (dict(mode=0,angle_deg=10),dict(mode=1,angle_deg=0)):
  r=post(env,PATH+'/move',p);assert completed(c,r.json()['id'])['result']['motion_completed']
 r=post(env,PATH+'/zero',{'confirm':True});assert completed(c,r.json()['id'])['result']['persisted']
 for board in LABELS:
  r=post(env,'/api/v1/boards/'+board+'/selftest',dict(scope=1));assert completed(c,r.json()['id'])['status']=='succeeded'
 channels=[dict(channel=1,enabled=True,role=1,gear_ratio=516,inverted=0,rpm=400)]
 r=post(env,'/api/v1/boards/main_board/config/apply',dict(channels=channels,confirm=True));assert completed(c,r.json()['id'])['result']['persisted']

def test_manual_interlock_and_exit(env):
 c,g,b,h=env;r=post(env,PATH+'/manual-adjust',dict(enable=True));assert completed(c,r.json()['id'])['status']=='succeeded'
 assert ('main_board','head_joint') in g.hazards
 assert post(env,PATH+'/move',dict(angle_deg=5)).status_code==409
 r=post(env,PATH+'/manual-adjust',dict(enable=False));assert completed(c,r.json()['id'])['status']=='succeeded'
 assert not g.hazards

def test_disconnect_locks_released_joint(env):
 c,g,b,h=env;r=post(env,PATH+'/brake',dict(engage=False));completed(c,r.json()['id'])
 assert g.hazards
 r=post(env,'/api/v1/control/release',{});assert r.json()['cleanup'][0]['stop_confirmed']
 assert b.snapshots['main_board']['channels'][1]['brake'] and not g.hazards

def test_disconnect_unconfirmed_is_not_success(env):
 c,g,b,h=env;r=post(env,PATH+'/brake',dict(engage=False));completed(c,r.json()['id']);b.fail='wire unplugged'
 r=post(env,'/api/v1/control/release',{});assert not r.json()['cleanup'][0]['stop_confirmed'] and g.hazards

def test_timeout_unknown_no_retry(env):
 c,g,b,h=env;b.fail='response timeout';r=post(env,PATH+'/move',dict(angle_deg=5))
 assert completed(c,r.json()['id'])['status']=='unknown';assert len(b.calls)==1

def test_runtime_revision_limits_and_no_flash(env):
 c,g,b,h=env;r=c.get('/api/v1/boards/main_board/runtime-config').json();p={k:r[k] for k in ('revision','options','board','joints')}
 p['options']['max_commissioning_step_deg']=30
 result=c.put('/api/v1/boards/main_board/runtime-config',json=p,headers=h)
 assert result.status_code==200 and result.json()['status']=='succeeded' and not b.calls
 assert c.get('/api/v1/boards/main_board/runtime-config').json()['pending_restart']
 p['options']['max_commissioning_step_deg']=-1;assert c.put('/api/v1/boards/main_board/runtime-config',json=p,headers=h).status_code==422

def test_duplicate_channels_and_limits_conflict(env):
 c,g,b,h=env;r=g.manager.get('main_board');p={k:r[k] for k in ('revision','options','board','joints')}
 p['joints']['head_joint']['channel']=1
 with pytest.raises(Exception):g.manager.edit('main_board',RuntimeEdit(**p))
 p['joints']['head_joint']['channel']=2;p['joints']['head_joint']['limits_rad']=None
 with pytest.raises(Exception):g.manager.edit('main_board',RuntimeEdit(**p))

def test_external_driver_never_killed(env):
 m=env[1].manager
 with pytest.raises(RuntimeError,match='external'):m.external_conflicts('main_board',{'board_a'})
 assert not m.processes

def test_store_restart_does_not_replay(tmp_path):
 path=tmp_path/'jobs.db';s=JobStore(path);j,_=s.create('req','move',{'angle':5});s2=JobStore(path)
 assert s2.get(j['id'])['status']=='unknown'
 assert s2.create('req','move',{'angle':5})[1] is False

def test_lease_five_seconds_boundary(env):
 c,g,b,h=env;now=[0.];g.clock=lambda:now[0];g.heartbeat_at=0
 now[0]=4.999;g.require_owner(g.owner)
 now[0]=5
 with pytest.raises(Refused):g.require_owner(g.owner)


def test_save_failure_is_distinct_from_acceptance(env):
 c,g,b,h=env
 old=b.service
 async def save_failure(board,op,payload):
  result=await old(board,op,payload)
  if op=='set_zero':result['persisted']=False
  return result
 b.service=save_failure
 r=post(env,PATH+'/zero',dict(confirm=True));j=completed(c,r.json()['id'])
 assert j['status']=='failed' and j['result']['accepted'] and not j['result']['persisted']

def test_partial_double_board_stop(env):
 c,g,b,h=env
 old=b.service
 async def partial(board,op,payload):
  if board=='right_board':raise RuntimeError('B reply lost')
  return await old(board,op,payload)
 b.service=partial
 r=post(env,'/api/v1/stop-all',{});assert r.json()['atomic'] is False
 results=[completed(c,j['id']) for j in r.json()['boards']]
 assert [j['status'] for j in results]==['succeeded','unknown']

def test_runtime_defaults_readonly_even_with_old_saved_policy(env):
 m=env[1].manager
 r=m.get('main_board');edit={k:r[k] for k in ('revision','options','board','joints')}
 edit['options']['mode']='motion';m.edit('main_board',RuntimeEdit(**edit))
 again=RuntimeManager(m.source,m.directory)
 assert again.active['main_board']['options']['mode']=='read_only' and not again.processes

def test_watchdog_locks_after_five_seconds(env):
 c,g,b,h=env;r=post(env,PATH+'/manual-adjust',dict(enable=True));completed(c,r.json()['id'])
 now=[10.];g.clock=lambda:now[0];g.heartbeat_at=4.99
 deadline=time.monotonic()+3
 while g.owner and time.monotonic()<deadline:time.sleep(.02)
 assert g.owner is None
 deadline=time.monotonic()+3
 while g.cleaning and time.monotonic()<deadline:time.sleep(.02)
 assert g.cleanup_result[0]['stop_confirmed'] and not g.hazards

def test_stopped_during_pending_move_does_not_replay(env):
 c,g,b,h=env;b.delay=.5
 r=post(env,PATH+'/move',dict(angle_deg=5));assert r.status_code==202
 stop=post(env,PATH+'/stop',{});assert stop.status_code==202
 assert completed(c,stop.json()['id'])['result']['stop_confirmed']
 completed(c,r.json()['id']);assert len([x for x in b.calls if x[1]=='move_joint_deg'])==1

def test_config_preview_does_not_commit(env):
 c,g,b,h=env
 r=post(env,'/api/v1/boards/main_board/config/preview',dict(channels=[]));completed(c,r.json()['id'])
 assert b.calls[-1][2]['commit'] is False

def test_restart_failure_reports_error_without_killing_external(env):
 c,g,b,h=env
 b.discovered=lambda:{'board_a'}
 r=post(env,'/api/v1/boards/main_board/restart',dict(confirm=True))
 assert r.status_code==200 and r.json()['status']=='failed' and not g.manager.processes


def test_empty_motion_whitelist_uses_nonempty_launch_value(tmp_path,monkeypatch):
 f=tmp_path/'robot.yaml';f.write_text(yaml.safe_dump(config()));m=RuntimeManager(f,tmp_path/'state');seen=[]
 class Process:
  pid=999999;returncode=None
 async def spawn(*args,**kwargs):seen.extend(args);return Process()
 async def check(board):raise AssertionError('readonly startup must not command hardware')
 monkeypatch.setattr(asyncio,'create_subprocess_exec',spawn)
 result=asyncio.run(m.restart('main_board',set(),check))
 assert result['mode']=='read_only'
 assert 'allowed_motion_joints:= ' in seen and 'allowed_motion_joints:=' not in seen
 assert 'allow_motion:=false' in seen


def test_ros_uuid_array_is_json_serializable():
 import array,json
 from robot_motor_web.bridge import plain
 assert json.loads(json.dumps(plain(array.array('B',[0,255]))))==[0,255]


def test_maintenance_cannot_move_or_probe(env):
 c,g,b,h=env;g.manager.active['main_board']['options']['mode']='maintenance'
 assert post(env,PATH+'/move',dict(angle_deg=5)).status_code==409
 assert post(env,'/api/v1/boards/main_board/selftest',dict(scope=1)).status_code==409
 assert not b.calls

SAVE='/api/v1/boards/main_board/save'

def test_retry_save_recovers_flags_preserves_fault_and_zero(env):
 c,g,b,h=env;s=b.snapshots['main_board'];s.update(dirty=True,storage_error=True,pending_save=True)
 s['channels'][0].update(fault=1,position_valid=False,angle_deci_deg=123,fg_count=106)
 g.manager.active['main_board']['options']['mode']='save_recovery'
 r=post(env,SAVE,{'confirm':True});assert r.status_code==202,r.text
 j=completed(c,r.json()['id']);assert j['status']=='succeeded' and j['result']['flags_valid']
 assert b.calls==[('main_board','retry_save',{'confirm':True})]
 assert s['channels'][0]['fault']==1 and not s['channels'][0]['position_valid'] and s['channels'][0]['fg_count']==106
 assert post(env,PATH+'/zero',{'confirm':True}).status_code==409

@pytest.mark.parametrize('condition',['read_only','stale','uid','unknown','manual','unbraked','busy'])
def test_retry_save_interlocks(env,condition):
 c,g,b,h=env;s=b.snapshots['main_board'];s['storage_error']=True
 if condition=='read_only':s['read_only']=True
 if condition=='stale':s['stale']=True
 if condition=='uid':s['uid']='0'*24
 if condition=='unknown':s['unknown_command_state']=True
 if condition=='manual':s['manual']['head_joint']['mode']=1
 if condition=='unbraked':s['channels'][0]['brake']=False
 if condition=='busy':s['channels'][0].update(state=2,duty_permille=100)
 assert post(env,SAVE,{'confirm':True}).status_code==409
 assert not b.calls

def test_retry_save_needs_confirmation_and_dedupes(env):
 c,g,b,h=env
 assert post(env,SAVE,{'confirm':False}).status_code==422
 a=c.post(SAVE,json={'confirm':True},headers=h);assert a.status_code==202
 completed(c,a.json()['id'])
 assert c.post(SAVE,json={'confirm':True},headers=h).json()['id']==a.json()['id']
 assert len(b.calls)==1

def test_retry_save_readback_failure_not_success(env):
 c,g,b,h=env
 async def fail(*args):return dict(accepted=True,result_code=0,persisted=False,flags_valid=False,unknown=True,message='read-back unavailable')
 b.service=fail
 r=post(env,SAVE,{'confirm':True});assert completed(c,r.json()['id'])['status']=='unknown'

def test_storage_flags_only_allow_recovery_restart(env):
 c,g,b,h=env;b.snapshots['main_board'].update(dirty=True,storage_error=True,pending_save=True)
 async def run():
  with pytest.raises(Refused):await g.stop_check('main_board')
  g.manager.records['main_board']['options']['mode']='save_recovery'
  await g.stop_check('main_board')
  b.snapshots['main_board']['channels'][0]['brake']=False
  with pytest.raises(Refused):await g.stop_check('main_board')
 asyncio.run(run())


def test_direction_high_low_preserves_configuration_position_and_dedupes(env):
 c,g,b,h=env;row=b.snapshots['main_board']['channels'][1];before=copy.deepcopy(row)
 path=PATH+'/direction';r=c.post(path,json={'high':True},headers=h);assert r.status_code==202
 assert completed(c,r.json()['id'])['result']['direction_confirmed'];assert row['direction'] is True
 assert c.post(path,json={'high':True},headers=h).json()['id']==r.json()['id'];assert len(b.calls)==1
 r=post(env,path,{'high':False});assert completed(c,r.json()['id'])['status']=='succeeded'
 assert row==before
 assert post(env,path,{'high':1}).status_code==422

@pytest.mark.parametrize('condition',['busy','unbraked','fault','storage','manual','readonly','recovery','unknown','stale'])
def test_direction_interlocks(env,condition):
 c,g,b,h=env;s=b.snapshots['main_board'];row=s['channels'][1]
 if condition=='busy':s['channels'][0]['state']=2
 if condition=='unbraked':row['brake']=False
 if condition=='fault':row['fault']=1
 if condition=='storage':s['storage_error']=True
 if condition=='manual':s['manual']['head_joint']['mode']=1
 if condition=='readonly':s['read_only']=True
 if condition=='recovery':g.manager.active['main_board']['options']['mode']='save_recovery'
 if condition=='unknown':s['unknown_command_state']=True
 if condition=='stale':s['stale']=True
 assert post(env,PATH+'/direction',{'high':True}).status_code==409 and not b.calls

def test_direction_unconfirmed_result_not_success(env):
 c,g,b,h=env
 async def mismatch(*args):return dict(accepted=True,result_code=0,direction=False,direction_confirmed=False)
 b.service=mismatch;r=post(env,PATH+'/direction',{'high':True})
 assert completed(c,r.json()['id'])['status']=='failed'


def serial_env(env, tmp_path):
 c,g,b,h=env
 for board in LABELS:
  port=tmp_path/(board+'-ttyUSB');port.touch()
  g.manager.active[board]['board'].update(transport='modbus_rtu',port=str(port))
 return c,g,b,h


def test_connection_get_is_readonly_without_control(env,tmp_path):
 c,g,b,h=serial_env(env,tmp_path);g.owner=None
 for board in LABELS:
  s=c.get('/api/v1/boards/'+board+'/state').json();v=s['connection']
  assert v['serial_status']=='present' and v['communication_status']=='connected'
  assert v['current_uid']==v['expected_uid'] and v['device']==g.manager.active[board]['board']['port']
 assert len(c.get('/api/v1/boards').json())==2 and not b.calls and not g.manager.processes and g.owner is None


@pytest.mark.parametrize('unplugged',[['main_board'],['right_board'],['main_board','right_board']])
def test_connection_unplugged_independent(env,tmp_path,unplugged):
 c,g,b,h=serial_env(env,tmp_path)
 for board in unplugged:Path(g.manager.active[board]['board']['port']).unlink()
 for board in LABELS:
  v=g.state(board)['connection']
  assert v['serial_status']==('missing' if board in unplugged else 'present')
  assert v['communication_status']==('serial_missing' if board in unplugged else 'connected')
  if board in unplugged:assert v['current_uid'] is None and v['last_uid']==v['expected_uid']
 assert not b.calls


@pytest.mark.parametrize('condition,expected',[('node','node_not_started'),('timeout','communication_error'),('stale','state_stale'),('uid','uid_mismatch'),('unconfirmed','uid_unconfirmed'),('empty','waiting_state')])
def test_connection_requires_node_fresh_communication_identity(env,tmp_path,condition,expected):
 c,g,b,h=serial_env(env,tmp_path);state=b.snapshots['main_board']
 if condition=='node':b.discovered=lambda:{'board_b'}
 if condition=='timeout':state['communication_ok']=False
 if condition=='stale':state['stale']=True
 if condition=='uid':state.update(uid='0'*24,identity_ok=False)
 if condition=='unconfirmed':state.update(uid=None,identity_ok=False,header={'stamp':{'sec':1,'nanosec':0}})
 if condition=='empty':b.snapshots['main_board']={}
 v=g.state('main_board')['connection'];assert v['serial_status']=='present' and v['communication_status']==expected
 if condition in ('timeout','stale','node'):assert v['current_uid'] is None
 assert not b.calls


def test_connection_simulated_never_claims_serial_presence(env):
 c,g,b,h=env;v=g.state('main_board')['connection']
 assert v['serial_status']=='simulated' and v['device'] is None and v['communication_status']=='connected'
 assert not b.calls


def test_connection_cache_one_second_and_active_not_draft(env,tmp_path):
 c,g,b,h=serial_env(env,tmp_path);now=[0.];g.connections.clock=lambda:now[0]
 port=Path(g.manager.active['main_board']['board']['port']);g.manager.records['main_board']['board']['port']='unused-draft'
 first=g.state('main_board')['connection'];assert first['path']==str(port)
 port.unlink();now[0]=.999;assert g.state('main_board')['connection']['serial_status']=='present'
 now[0]=1;assert g.state('main_board')['connection']['serial_status']=='missing'
 assert not b.calls


def test_connection_history_survives_snapshot_loss_and_resets_new_binding(env,tmp_path):
 c,g,b,h=serial_env(env,tmp_path);b.snapshots['main_board']['header']={'stamp':{'sec':123,'nanosec':0}}
 first=g.state('main_board')['connection'];b.snapshots['main_board']={}
 v=g.state('main_board')['connection'];assert v['current_uid'] is None and v['last_uid']==first['current_uid']
 assert v['last_state_stamp']=={'sec':123,'nanosec':0}
 g.manager.active['main_board']['board']['port']='new-binding'
 v=g.state('main_board')['connection'];assert v['last_uid'] is None and v['last_state_stamp'] is None


def test_connection_error_is_not_missing(env,tmp_path,monkeypatch):
 c,g,b,h=serial_env(env,tmp_path)
 def denied(path,*a,**k):raise PermissionError('denied')
 monkeypatch.setattr(Path,'stat',denied)
 v=g.state('main_board')['connection'];assert v['serial_status']=='error' and v['communication_status']=='serial_error'
 assert 'denied' in v['serial_error'] and not b.calls


def test_connection_broken_and_resolved_symlink(env,tmp_path):
 c,g,b,h=serial_env(env,tmp_path);target=Path(g.manager.active['main_board']['board']['port']);link=tmp_path/'by-path'
 try:link.symlink_to(target)
 except OSError as e:pytest.skip('host cannot create symlinks: '+str(e))
 g.manager.active['main_board']['board']['port']=str(link)
 v=g.state('main_board')['connection'];assert v['device']==str(target) and v['serial_status']=='present'
 target.unlink();g.connections.cache.clear()
 assert g.state('main_board')['connection']['serial_status']=='missing'


def test_connection_graph_failure_reported_not_connected(env,tmp_path):
 c,g,b,h=serial_env(env,tmp_path)
 def failed():raise RuntimeError('graph unavailable')
 b.discovered=failed;v=g.state('main_board')['connection']
 assert v['communication_status']=='node_check_error' and v['node_error']=='graph unavailable'
 assert not b.calls


def test_connection_websocket_readonly_without_control(env):
 c,g,b,h=env;g.owner=None
 with c.websocket_connect('/events?csrf='+h['X-CSRF-Token'],headers={'origin':'http://testserver','host':'testserver'}) as ws:
  data=ws.receive_json()
  assert len(data['boards'])==2 and not data['control']
  assert all(x['state']['connection']['serial_status']=='simulated' for x in data['boards'])
 assert not b.calls and not g.manager.processes and g.owner is None
