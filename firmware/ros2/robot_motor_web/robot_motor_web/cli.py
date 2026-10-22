"""No-login LAN server. It never auto-starts nodes or moves hardware."""
import argparse,ipaddress,os,socket,subprocess
from pathlib import Path

def main():
    p=argparse.ArgumentParser()
    p.add_argument('command',choices=['certificate','serve'])
    p.add_argument('--state-dir',default=str(Path.home()/'.local/share/robot_motor_web'))
    p.add_argument('--config',default='/data/ros2/robot_motor_ros2_ws/review_logs/head_back10_config.yaml')
    p.add_argument('--domain',type=int,default=173);p.add_argument('--host',default='0.0.0.0');p.add_argument('--port',type=int,default=8443)
    p.add_argument('--ip',default='192.168.1.113');p.add_argument('--https',action='store_true')
    a=p.parse_args();directory=Path(a.state_dir).resolve();directory.mkdir(parents=True,exist_ok=True);os.chmod(str(directory),0o700)
    if a.command=='certificate':
        address=str(ipaddress.ip_address(a.ip))
        if (directory/'server.key').exists():raise SystemExit('certificate exists; back it up before rotation')
        subprocess.run(['openssl','req','-x509','-newkey','rsa:3072','-nodes','-days','365',
            '-keyout',str(directory/'server.key'),'-out',str(directory/'server.crt'),'-subj','/CN=robot-motor-web',
            '-addext','subjectAltName=IP:'+address+',DNS:localhost'],check=True)
        os.chmod(str(directory/'server.key'),0o600);print('Trust server.crt in browser; keep server.key private.');return
    with socket.socket() as sock:
        try:sock.bind((a.host,a.port))
        except OSError as e:raise SystemExit('Port unavailable: '+str(e))
    options={}
    if a.https:
        for n in ['server.key','server.crt']:
            if not (directory/n).exists():raise SystemExit('generate certificate first')
        options=dict(ssl_keyfile=str(directory/'server.key'),ssl_certfile=str(directory/'server.crt'))
    os.environ['ROS_DOMAIN_ID']=str(a.domain)
    from .runtime import RuntimeManager
    from .bridge import RosBridge
    from .gateway import Gateway
    from .api import create_app
    import uvicorn
    manager=RuntimeManager(a.config,directory,a.domain);bridge=RosBridge(manager)
    app=create_app(Gateway(manager,bridge,directory),secure=a.https,allowed_hosts=[a.ip,'127.0.0.1','localhost'])
    print('No-login LAN console: do not expose this port to the Internet.')
    uvicorn.run(app,host=a.host,port=a.port,workers=1,**options)
