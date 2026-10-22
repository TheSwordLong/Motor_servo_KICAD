"""Offline UI validation only; binds localhost:8444 and never opens a motor port."""
from pathlib import Path
import sys,tempfile,yaml
here=Path(__file__).resolve();sys.path[:0]=[str(here.parents[1]),str(here.parents[2]/'robot_motor_driver')]
from test_web import config,FakeBridge
from robot_motor_web.runtime import RuntimeManager
from robot_motor_web.gateway import Gateway
from robot_motor_web.api import create_app
import uvicorn
root=Path(tempfile.mkdtemp(prefix='web-fake-'));f=root/'robot.yaml';f.write_text(yaml.safe_dump(config()))
m=RuntimeManager(f,root/'state',domain=174)
for r in m.active.values():r['options'].update(mode='motion',allow_unreferenced_relative=True)
b=FakeBridge(m);g=Gateway(m,b,m.directory)
job,_=g.store.create('browser-failed-result','/mock/failed-move',{'angle_deg':5})
g.store.update(job['id'],status='failed',result={'error_code':105,'message':'simulated failure; no command sent'})
uvicorn.run(create_app(g),host='127.0.0.1',port=8444,workers=1)
