"""Connection-only browser fixture. Temporary files stand in for USB paths."""
import copy, sys, tempfile, time
from pathlib import Path
import yaml, uvicorn
here=Path(__file__).resolve()
sys.path[:0]=[str(here.parents[1]),str(here.parents[2]/'robot_motor_driver')]
from test_web import config, FakeBridge
from robot_motor_web.runtime import RuntimeManager
from robot_motor_web.gateway import Gateway
from robot_motor_web.api import create_app
root=Path(tempfile.mkdtemp(prefix='web-connection-fake-'))
f=root/'robot.yaml';f.write_text(yaml.safe_dump(config()))
m=RuntimeManager(f,root/'state',domain=174)
b=FakeBridge(m);g=Gateway(m,b,m.directory);app=create_app(g)
original=copy.deepcopy(b.snapshots)

@app.post('/mock/scenario/{name}')
def scenario(name: str):
 b.snapshots=copy.deepcopy(original)
 b.discovered=lambda:{r['options']['namespace'] for r in m.active.values()}
 for board in m.active:
  path=root/(board+'-ttyUSB');path.touch()
  m.active[board]['board'].update(transport='modbus_rtu',port=str(path))
  b.snapshots[board].update(header={'stamp':{'sec':int(time.time()),'nanosec':0}},gateway_age_sec=.1)
 if name in ('a_missing','both_missing'):(root/'main_board-ttyUSB').unlink()
 if name in ('b_missing','both_missing'):(root/'right_board-ttyUSB').unlink()
 if name=='a_stale':b.snapshots['main_board']['stale']=True
 if name=='a_timeout':b.snapshots['main_board']['communication_ok']=False
 if name=='a_uid':b.snapshots['main_board'].update(uid='0'*24,identity_ok=False)
 if name=='a_no_node':b.discovered=lambda:{'board_b'}
 if name=='fake':
  for r in m.active.values():r['board'].update(transport='fake',port='')
 # Simulate unplug while leaving the old cached ROS message intact.
 return {'scenario':name}

@app.get('/mock/calls')
def calls():return {'writes':b.calls,'control_owner':g.owner,'processes':list(m.processes)}

scenario('normal')
uvicorn.run(app,host='127.0.0.1',port=8444,workers=1)
