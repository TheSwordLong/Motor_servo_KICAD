"""No-login LAN API. ROS remains the sole owner of motor communication."""
import asyncio
from pathlib import Path
from fastapi import FastAPI,Request,WebSocket
from fastapi.responses import FileResponse,JSONResponse
from fastapi.staticfiles import StaticFiles
from .models import Move,Brake,Manual,Confirm,Probe,ConfigApply,RuntimeEdit,Direction
from .gateway import Refused
from .security import Sessions

def static_directory():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('robot_motor_web'))/'static'
    except Exception:return Path(__file__).parent/'static'

def create_app(gateway,secure=False,allowed_hosts=None):
    app=FastAPI(title='机器人电机管理API',version='1.0',docs_url=None,redoc_url=None,openapi_url=None)
    sessions=Sessions();app.state.gateway=gateway;app.state.sessions=sessions
    from starlette.middleware.trustedhost import TrustedHostMiddleware
    app.add_middleware(TrustedHostMiddleware,allowed_hosts=allowed_hosts or ['127.0.0.1','localhost','192.168.1.113','testserver'])
    @app.exception_handler(Refused)
    async def refusal(request,e):return JSONResponse(status_code=e.code,content={'detail':str(e)})
    @app.exception_handler(KeyError)
    async def missing(request,e):return JSONResponse(status_code=404,content={'detail':'不存在的资源'})
    @app.exception_handler(ValueError)
    async def invalid(request,e):return JSONResponse(status_code=422,content={'detail':str(e)})
    @app.middleware('http')
    async def protect(request,call_next):
        if request.url.path.startswith('/api/'):
            origin=request.headers.get('origin')
            if origin and origin.rstrip('/')!=str(request.base_url).rstrip('/'):
                return JSONResponse(status_code=403,content={'detail':'禁止跨域操作'})
            if request.method in ('POST','PUT','DELETE'):
                t=request.cookies.get('motor_session','')
                if not sessions.check_csrf(t,request.headers.get('x-csrf-token')):
                    return JSONResponse(status_code=403,content={'detail':'先GET /api/v1/session取得匿名会话和CSRF令牌'})
                if request.url.path!='/api/v1/control/heartbeat' and (not request.headers.get('x-request-id') or len(request.headers.get('x-request-id',''))>128):
                    return JSONResponse(status_code=400,content={'detail':'需要X-Request-ID'})
        response=await call_next(request)
        response.headers['Cache-Control']='no-store';response.headers['X-Content-Type-Options']='nosniff'
        response.headers['Content-Security-Policy']="default-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'self'; form-action 'self'"
        return response
    def token(r):return r.cookies.get('motor_session','')
    def ident(r):return r.headers.get('x-request-id','')
    @app.on_event('startup')
    async def startup():gateway.watchdog=asyncio.create_task(gateway.watch())
    @app.on_event('shutdown')
    async def shutdown():
        if gateway.watchdog:gateway.watchdog.cancel()
        if gateway.owner:await gateway.release(gateway.owner)
        if gateway.tasks:await asyncio.wait(list(gateway.tasks.values()),timeout=10)
        await gateway.manager.shutdown()
        if hasattr(gateway.bridge,'close'):gateway.bridge.close()
    @app.get('/')
    async def index():return FileResponse(static_directory()/'index.html')
    app.mount('/static',StaticFiles(directory=str(static_directory())),name='static')
    @app.get('/api/v1/session')
    async def session(request:Request):
        t=token(request);s=sessions.session(t)
        if not s:t,csrf=sessions.create()
        else:csrf=s['csrf']
        result=JSONResponse(dict(csrf=csrf,control=gateway.owner==t,cleaning=gateway.cleaning,cleanup=gateway.cleanup_result))
        result.set_cookie('motor_session',t,httponly=True,secure=secure,samesite='strict',max_age=8*3600)
        return result
    @app.post('/api/v1/control/acquire')
    async def acquire(request:Request):return gateway.acquire(token(request))
    @app.post('/api/v1/control/heartbeat')
    async def heartbeat(request:Request):return gateway.heartbeat(token(request))
    @app.post('/api/v1/control/release')
    async def release(request:Request):return await gateway.release(token(request))
    @app.get('/api/v1/boards')
    async def boards():
        """Read-only connection/UID diagnostics from ROS and cached path metadata."""
        return gateway.boards()
    @app.get('/api/v1/boards/{board}/state')
    async def state(board:str):
        """Board status including connection diagnostics; never opens the serial port."""
        return gateway.state(board)
    @app.get('/api/v1/boards/{board}/joints/{joint}/state')
    async def joint_state(board:str,joint:str):return gateway.state(board)['joints'][joint]
    @app.get('/api/v1/parameters')
    async def parameters():return PARAMETER_REFERENCE
    @app.get('/api/v1/jobs')
    async def jobs():return gateway.store.recent()
    @app.get('/api/v1/jobs/{job_id}')
    async def job(job_id:str):return gateway.store.get(job_id)
    @app.post('/api/v1/jobs/{job_id}/cancel')
    async def cancel(job_id:str,request:Request):
        gateway.require_owner(token(request));record,new=gateway.store.create(ident(request),'cancel/'+job_id,{})
        if new:
            try:
                result=await gateway.bridge.cancel(job_id)
                gateway.store.update(record['id'],status='succeeded' if result.get('goals_canceling') else 'failed',result=dict(ros_response=result,cancel_requested=bool(result.get('goals_canceling')),stop_confirmed=False),message='取消请求不等于已停；核对原动作最终结果和刹车状态')
            except Exception as e:gateway.store.update(record['id'],status='unknown',message=str(e))
        return gateway.store.get(record['id'])
    def submit(request,board,joint,operation,payload):
        return gateway.submit(token(request),ident(request),request.url.path,board,joint,operation,payload)
    @app.post('/api/v1/boards/{board}/joints/{joint}/move',status_code=202)
    async def move(board:str,joint:str,q:Move,request:Request):return submit(request,board,joint,'move',q.dict())
    @app.post('/api/v1/boards/{board}/joints/{joint}/direction',status_code=202)
    async def direction(board:str,joint:str,q:Direction,request:Request):return submit(request,board,joint,'direction',q.dict())
    @app.post('/api/v1/boards/{board}/joints/{joint}/brake',status_code=202)
    async def brake(board:str,joint:str,q:Brake,request:Request):return submit(request,board,joint,'brake',q.dict())
    @app.post('/api/v1/boards/{board}/joints/{joint}/manual-adjust',status_code=202)
    async def manual(board:str,joint:str,q:Manual,request:Request):return submit(request,board,joint,'manual-adjust',q.dict())
    @app.post('/api/v1/boards/{board}/joints/{joint}/zero',status_code=202)
    async def zero(board:str,joint:str,q:Confirm,request:Request):return submit(request,board,joint,'zero',{})
    @app.post('/api/v1/boards/{board}/joints/{joint}/clear-fault',status_code=202)
    async def clear(board:str,joint:str,q:Confirm,request:Request):return submit(request,board,joint,'clear-fault',{})
    @app.post('/api/v1/boards/{board}/joints/{joint}/stop',status_code=202)
    async def stop_joint(board:str,joint:str,request:Request):return submit(request,board,joint,'stop',{})
    @app.post('/api/v1/boards/{board}/stop',status_code=202)
    async def stop_board(board:str,request:Request):return submit(request,board,None,'stop',{})
    @app.post('/api/v1/stop-all',status_code=202)
    async def stop_all(request:Request):
        gateway.require_owner(token(request));results=[]
        for board in gateway.manager.records:
            try:results.append(gateway.submit(token(request),ident(request)+':'+board,request.url.path,board,None,'stop',{}))
            except Exception as e:results.append(dict(board=board,stop_confirmed=False,message=str(e)))
        return {'boards':results,'atomic':False}
    @app.post('/api/v1/boards/{board}/save',status_code=202)
    async def retry_save(board:str,q:Confirm,request:Request):return submit(request,board,None,'save',q.dict())
    @app.post('/api/v1/boards/{board}/selftest',status_code=202)
    async def probe(board:str,q:Probe,request:Request):
        joint=q.joint_name if q.scope==0 else None
        if q.scope==0 and not joint:raise Refused('单路自检必须指定关节',422)
        return submit(request,board,joint,'selftest',q.dict())
    @app.post('/api/v1/boards/{board}/config/preview',status_code=202)
    async def preview(board:str,q:ConfigApply,request:Request):return submit(request,board,None,'config/preview',q.dict())
    @app.post('/api/v1/boards/{board}/config/apply',status_code=202)
    async def apply(board:str,q:ConfigApply,request:Request):
        if not q.confirm:raise Refused('写Flash前需要明确确认',422)
        return submit(request,board,None,'config/apply',q.dict())
    @app.post('/api/v1/boards/{board}/acknowledge-unknown',status_code=202)
    async def ack(board:str,q:Confirm,request:Request):return submit(request,board,None,'acknowledge-unknown',{})
    @app.get('/api/v1/boards/{board}/runtime-config')
    async def runtime(board:str):return gateway.manager.get(board)
    @app.put('/api/v1/boards/{board}/runtime-config')
    async def edit(board:str,q:RuntimeEdit,request:Request):
        gateway.require_owner(token(request))
        if gateway.pending.get(board) or any(b==board for b,j in gateway.hazards):raise Refused('先停止并锁定网页控制的关节')
        record,new=gateway.store.create(ident(request),'runtime/'+board,q.dict())
        if new:
            try:gateway.store.update(record['id'],status='succeeded',result=gateway.manager.edit(board,q))
            except Exception as e:gateway.store.update(record['id'],status='failed',message=str(e));raise
        return gateway.store.get(record['id'])
    @app.post('/api/v1/boards/{board}/restart')
    async def restart(board:str,q:Confirm,request:Request):return await gateway.restart(token(request),ident(request),board)
    @app.get('/api/openapi.json')
    async def schema():
        schema=app.openapi()
        schema['info']['description']='无需登录。GET /api/v1/session取得匿名Cookie和CSRF；写操作取得唯一控制权。HTTP202只代表任务建立，核对/jobs/{id}最终ROS结果。浏览器关闭或5秒心跳超时将尝试停止并制动，不能保证断电或通信故障下已停。'
        for path,methods in schema['paths'].items():
            for method,details in methods.items():
                if method not in ('post','put','delete'):continue
                details['parameters']=[p for p in details.get('parameters',[]) if p.get('in')!='header']+[dict(name='X-CSRF-Token',**{'in':'header'},required=True,schema={'type':'string'})]
                if path!='/api/v1/control/heartbeat':details['parameters'].append(dict(name='X-Request-ID',**{'in':'header'},required=True,schema={'type':'string','maxLength':128}))
        return schema
    @app.get('/api/docs')
    async def docs():return FileResponse(static_directory()/'api.html')
    @app.websocket('/events')
    async def events(ws:WebSocket):
        t=ws.cookies.get('motor_session','');origin=ws.headers.get('origin','')
        expected=('https' if secure else 'http')+'://'+ws.headers.get('host','')
        if origin!=expected or not sessions.check_csrf(t,ws.query_params.get('csrf')):
            await ws.close(code=4403);return
        await ws.accept()
        try:
            while sessions.session(t):
                await ws.send_json(dict(boards=gateway.boards(),jobs=gateway.store.recent(),control=gateway.owner==t,
                                        cleaning=gateway.cleaning,cleanup=gateway.cleanup_result))
                await asyncio.sleep(.2)
        except Exception:pass
        finally:
            if gateway.owner==t:await gateway.release(t)
    return app

PARAMETER_REFERENCE = [
 dict(key='save_recovery_only',default=False,range='布尔',source='motor_node.py:__init__ + scheduler.py:BoardWorker.submit',effect='受限恢复；只允许STOP/RetrySave，不允许运动/设零/配置/手调'),
 dict(key='mode',default='read_only',range='read_only/maintenance/motion/save_recovery',source='motor_node.py:RobotMotorNode.__init__',effect='重启本板；不写Flash'),
 dict(key='allowed_motion_joints',default=[],range='本板标签列表',source='motor_node.py:_validate_move_goal',effect='仅自动运动白名单'),
 dict(key='commissioning',default=False,range='布尔',source='motor_node.py:_on_move',effect='未知限位调试；不是机械限位'),
 dict(key='max_commissioning_step_deg',default=10.0,range='0.1..360',unit='输出轴度',source='bringup.launch.py + scheduler.py:_task_move',effect='调试单次位移，非累计行程'),
 dict(key='allow_unreferenced_relative',default=False,range='布尔',source='scheduler.py:_task_move',effect='不自动设零；绝对运动仍需有效位置'),
 dict(key='polling_hz',default=5.0,range='0<值<=100',unit='Hz',source='motor_node.py:__init__',effect='查询频率，非固件控制周期'),
 dict(key='uid',range='24位十六进制',source='config.py:_parse_boards + board_session.py:identify',effect='板身份，不允许绕过'),
 dict(key='port',range='独立稳定串口路径',source='config.py:_parse_boards',effect='重启；一板一串口所有者'),
 dict(key='namespace',range='ROS合法名称',source='bringup.launch.py',effect='服务与话题前缀，重启生效'),
 dict(key='board_name',range='YAML板名',source='motor_node.py:_select_board',effect='选择板配置，重启生效'),
 dict(key='channel',range='YAML1..4、服务0..3',source='config.py:_parse_joints',effect='文件绑定不自动迁移板端'),
 dict(key='gear_ratio',range='0/82/290/516',source='config.py + BSP/robot.c:robot_config_valid',effect='板端修改走ApplyConfig，0未配置'),
 dict(key='inverted',default=0,range='0/1',source='BSP/robot.c:MOVE',effect='逻辑方向反相，板端修改可能使零点无效'),
 dict(key='expected_motor_rpm',default=400,range='400..8000',unit='减速前RPM',source='config.py:_parse_joints',effect='文件预期，不写板转速'),
 dict(key='motor_rpm',default=0,range='0或400..8000',unit='减速前RPM',source='scheduler.py:_task_move',effect='0沿用，非零只校验；修改转速走ApplyConfig'),
 dict(key='limits_deg',default=None,range='null或有限[最小,最大]，±360',unit='输出轴度',source='config.py:_parse_degree_limits + scheduler.py:_check_soft_limits',effect='需有效零点；与limits_rad互斥'),
 dict(key='timeout_sec',default=30,range='0.5..300',unit='秒',source='motor_node.py:_validate_move_goal',effect='监控期限，不是运动时长'),
 dict(key='duty_permille',range='底层0..1000，正常运动上限950',unit='低电平千分比',source='BSP/pwm.c:pwm_set_permille',effect='只读；0为持续高电平停止')]
