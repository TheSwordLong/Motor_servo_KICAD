"""Control lease, safety gates, jobs and disconnect braking over ROS only."""
import asyncio, copy, math, time, uuid
from .store import JobStore
from .connection import ConnectionMonitor

class Refused(Exception):
    def __init__(self, message, code=409):super().__init__(message);self.code=code

ERRORS={1:'参数非法',2:'电机或整板忙，存在互锁',3:'通道未就绪',4:'Flash存储错误',5:'请求序号冲突',6:'寄存器地址非法',101:'板身份不匹配',102:'执行结果未知',103:'等待结果超时',106:'位移量化后为零',107:'任务忙',108:'软件与板上配置不匹配',110:'板连接未建立',113:'读取的状态不可用于运动',114:'请求参数或关节不合法',104:'零点无效；绝对运动必须先确认零点',105:'角度步长或机械范围超限',109:'UNKNOWN锁定；先恢复通信并人工核对',
        111:'转速与板上保存值不匹配',112:'机械限位未确认',100:'节点只读，写操作被拒绝'}

class Gateway:
    def __init__(self, manager, bridge, state_dir, clock=time.monotonic):
        self.manager=manager;self.bridge=bridge;self.clock=clock
        self.store=JobStore(state_dir/'jobs.sqlite3');self.owner=None;self.heartbeat_at=0
        self.hazards=set();self.tasks={};self.pending={};self.restarting=set();self.cleanup_result=[]
        self.cleaning=False;self.watchdog=None
        self.connections=ConnectionMonitor(manager,bridge)
    def boards(self):
        return [dict(id=b,**self.manager.get(b),state=self.state(b)) for b in self.manager.records]
    def state(self, board):
        if board not in self.manager.records:raise Refused('未知板子',404)
        s=self.bridge.snapshot(board);r=self.manager.active[board]
        s['connection']=self.connections.snapshot(board,s)
        s['expected_uid']=r['board']['uid'];s['namespace']=r['options']['namespace'];s['web_read_only']=r['options']['mode']=='read_only';s['web_mode']=r['options']['mode'];s['joints']={}
        for label,j in r['joints'].items():
            ch=int(j['channel'])-1;rows=s.get('channels',[]);row=copy.deepcopy(rows[ch]) if ch<len(rows) else {}
            expected=dict(channel=ch,role=int(j['role']),gear_ratio=int(j['gear_ratio']),inverted=int(j.get('inverted',0)),enabled=True)
            mismatch={k:dict(expected=v,actual=row.get(k)) for k,v in expected.items() if row and row.get(k)!=v}
            row.update(joint_name=label,physical_channel='M'+str(ch+1),binding_conflict=mismatch,
                       manual_adjust=s.get('manual',{}).get(label,{}))
            if not row.get('position_valid'):
                row['angle_deg']=None;row['target_deg']=None
            s['joints'][label]=row
        return s
    def acquire(self, token):
        if self.cleaning:raise Refused('正在确认失联制动，暂不允许取得控制权')
        if self.owner and self.owner!=token:raise Refused('其他浏览器正在控制')
        self.owner=token;self.heartbeat_at=self.clock()
        return dict(control=True,heartbeat_timeout_sec=5)
    def require_owner(self, token):
        if token!=self.owner or self.clock()-self.heartbeat_at>=5:raise Refused('未取得控制权或心跳已超时',403)
    def heartbeat(self, token):
        self.require_owner(token);self.heartbeat_at=self.clock();return dict(control=True)
    def guard(self, board, joint=None, operation='', payload=None):
        s=self.state(board)
        if joint is not None and joint not in s['joints']:raise Refused('该关节不属于本板',404)
        if board in self.restarting:raise Refused('本板正在重启')
        stoplike=operation=='stop' or operation=='brake' and (payload or {}).get('engage') or operation=='manual-adjust' and not (payload or {}).get('enable')
        if s.get('web_mode')=='save_recovery' and operation not in ('save','stop'):
            raise Refused('保存恢复模式仅允许停止和重试保存')
        if stoplike:
            if s.get('uid') and s['uid']!=s['expected_uid']:raise Refused('UID不匹配，不能控制这块板')
            return s
        if not s.get('identity_ok') or s.get('uid')!=s['expected_uid']:raise Refused('UID尚未核对或不匹配')
        if not s.get('communication_ok') or s.get('stale',True):raise Refused('通信失效或状态过期')
        if s.get('web_read_only') or s.get('read_only',True):raise Refused('节点只读；先通过维护模式或运动模式启动')
        if operation in ('move','selftest') and s.get('web_mode')!='motion':raise Refused('维护模式不允许试转；需明确启用运动模式')
        if s.get('unknown_command_state') and operation!='acknowledge-unknown':raise Refused('UNKNOWN锁定；需人工核对后确认')
        if s.get('storage_error') and operation!='save':raise Refused('板上存储错误；停稳后使用重试保存')
        if self.pending.get(board):raise Refused('本板有未结束的网页任务；停止入口仍可用')
        row=s['joints'].get(joint,{})
        if joint and row.get('binding_conflict'):raise Refused('配置与物理通道绑定不匹配，禁止写入')
        if joint and row.get('fault') and operation not in ('clear-fault','manual-adjust'):raise Refused('该关节存在故障')
        modes=[v for v in s.get('manual',{}).values() if v.get('mode') in (1,2)]
        if modes and operation not in ('manual-adjust','acknowledge-unknown'):raise Refused('手动或退出等待阶段：整板运动/配置互锁')
        if operation=='direction':
            if len(s.get('channels',[]))!=4 or any(x.get('state') in (2,3,4,7,8) for x in s.get('channels',[])) or not row.get('brake') or row.get('duty_permille') or row.get('measured_rpm'):
                raise Refused('DIR换向要求停止运动、退出手调并锁定该路刹车')
        if operation=='save':
            if len(s.get('channels',[]))!=4 or any(
                not x.get('brake') or x.get('duty_permille') or x.get('measured_rpm') or x.get('state') in (2,3,4,7,8)
                for x in s.get('channels',[])):
                raise Refused('重试保存要求整板停止、锁刹车并退出手调')
        if operation=='move':
            p=payload;pos=row.get('position_valid');m=self.manager.active[board]['options']
            if p['mode']==1 and not pos:raise Refused('104：没有有效零点，不能绝对运动')
            if p['mode']==0 and not pos and not m['allow_unreferenced_relative']:raise Refused('104：禁止无基准相对运动')
            if p['motor_rpm'] and p['motor_rpm']!=row.get('configured_rpm'):raise Refused('111：motor_rpm只校验板上转速')
            # ROS worker still does authoritative permission/limit checks from fresh reads.
        return s
    def submit(self, token, request_id, path, board, joint, operation, payload):
        self.require_owner(token)
        if not request_id or len(request_id)>128:raise Refused('写请求必须携带不超过128字符的X-Request-ID',400)
        try:job,new=self.store.create(request_id,path,payload)
        except ValueError as e:raise Refused(str(e))
        if not new:return job
        try:self.guard(board,joint,operation,payload)
        except Exception as e:
            self.store.update(job['id'],status='failed',message=str(e));raise
        if operation not in ('stop','brake','manual-adjust') or operation=='brake' and not payload['engage'] or operation=='manual-adjust' and payload['enable']:
            self.pending.setdefault(board,set()).add(job['id'])
        if operation in ('move','selftest'):
            names=[joint] if joint else list(self.manager.active[board]['joints'])
            self.hazards.update((board,j) for j in names)
        if operation=='brake' and not payload['engage'] or operation=='manual-adjust' and payload['enable']:
            self.hazards.add((board,joint))
        task=asyncio.create_task(self.execute(job,token,board,joint,operation,payload));self.tasks[job['id']]=task
        return job
    def feedback(self, ident, data):
        job=self.store.get(ident)
        if job['status'] not in ('succeeded','failed','unknown','cancelled'):
            status='accepted' if data.get('goal_accepted') else 'settling' if data.get('state')==4 else 'running'
            self.store.update(ident,status=status,feedback=data)
    async def execute(self, job, token, board, joint, op, p):
        ident=job['id']
        try:
            stoplike=op=='stop' or op=='brake' and p.get('engage') or op=='manual-adjust' and not p.get('enable')
            if not stoplike and token!=self.owner:raise Refused('控制会话已撤销；排队任务未发送')
            self.store.update(ident,status='running')
            if op=='move':
                result=await self.bridge.action(board,'move_joint_deg',dict(p,joint_name=joint),ident,lambda f:self.feedback(ident,f))
            elif op=='selftest':
                result=await self.bridge.action(board,'selftest',p,ident,lambda f:self.feedback(ident,f))
            else:
                services={'stop':'stop','brake':'set_brake','manual-adjust':'set_manual_adjust','zero':'set_zero',
                          'clear-fault':'clear_fault','config/preview':'apply_config','config/apply':'apply_config',
                          'acknowledge-unknown':'acknowledge_unknown','save':'retry_save','direction':'set_direction'}
                data={} if op=='acknowledge-unknown' else dict(joint_name=joint or '')
                if op=='save':data=dict(confirm=True)
                if op=='direction':data['high']=p['high']
                if op=='brake':data['engage']=p['engage']
                if op=='manual-adjust':data['enable']=p['enable']
                if op.startswith('config/'):
                    data=dict(channels=p['channels'],commit=op=='config/apply')
                result=await self.bridge.service(board,services[op],data)
            code=result.get('error_code',result.get('result_code',0))
            ok=result.get('accepted',result.get('success',True)) and not code
            if op=='move':ok=ok and result.get('motion_completed',False) and result.get('persisted',False)
            if op=='selftest':ok=ok and result.get('completed',False) and result.get('passed',False) and result.get('persisted',False)
            if op=='stop':ok=ok and result.get('stop_confirmed',False)
            if op=='brake':ok=ok and result.get('brake')==p['engage']
            if op=='direction':ok=ok and result.get('direction_confirmed',False) and result.get('direction')==p['high']
            if op=='clear-fault':ok=ok and result.get('fault_after')==0
            if op=='zero' or op=='config/apply':ok=ok and result.get('persisted',False)
            if op=='save':ok=ok and result.get('persisted',False) and result.get('flags_valid',False) and not any(
                result.get(k,True) for k in ('dirty','storage_error','pending_save'))
            if op=='manual-adjust':ok=ok and (result.get('active') if p['enable'] else not result.get('active') and not result.get('exiting'))
            unknown=result.get('unknown',False) or 'unknown' in result.get('message','').lower()
            self.store.update(ident,status='unknown' if unknown else 'cancelled' if result.get('goal_status')==5 else 'succeeded' if ok else 'failed',result=result,
                              explanation=ERRORS.get(code,''))
            # Hazards remain until fresh braking is verified, including uncertain outcomes.
            if not (op=='brake' and not p['engage'] or op=='manual-adjust' and p['enable']):
                await self.forget_braked(board, result.get('sequence'))
        except (asyncio.TimeoutError,RuntimeError) as e:
            self.store.update(ident,status='unknown',message=str(e) or 'ROS response timeout; outcome unknown; no retry')
        except Exception as e:self.store.update(ident,status='failed',message=str(e))
        finally:
            self.pending.get(board,set()).discard(ident);self.tasks.pop(ident,None)
    async def forget_braked(self, board, sequence=None):
        s=self.state(board)
        if not s.get('communication_ok') or s.get('stale'):return
        if sequence and s.get('last_sequence') != sequence:return
        for b,j in list(self.hazards):
            if b!=board:continue
            row=s['joints'].get(j,{})
            if row.get('brake') and row.get('duty_permille')==0 and row.get('state') not in (2,3,4,7,8) and not row.get('manual_adjust',{}).get('mode',0):
                self.hazards.discard((b,j))
    async def release(self, token):
        if token!=self.owner:return dict(control=False,cleanup=self.cleanup_result)
        self.owner=None;self.cleaning=True
        outcomes=[]
        try:
            for board in sorted({b for b,j in self.hazards}):
                try:
                    result=await self.bridge.service(board,'stop',dict(joint_name=''))
                    # STOP_ALL is not proof until state and per-joint brake/manual exit are fresh.
                    deadline=time.monotonic()+7
                    while time.monotonic()<deadline:
                        await self.forget_braked(board, result.get('sequence'))
                        if not any(b==board for b,j in self.hazards):break
                        await asyncio.sleep(.2)
                    confirmed=result.get('stop_confirmed',False) and not any(b==board for b,j in self.hazards)
                    outcomes.append(dict(board=board,stop_confirmed=confirmed,result=result,
                                         message='已确认停止并制动' if confirmed else '停止或刹车未确认；需要现场处理'))
                except Exception as e:outcomes.append(dict(board=board,stop_confirmed=False,message='状态未知：'+str(e)))
            self.cleanup_result=outcomes
            if outcomes:
                job,_=self.store.create(str(uuid.uuid4()),'disconnect-braking',{})
                self.store.update(job['id'],status='succeeded' if all(x['stop_confirmed'] for x in outcomes) else 'unknown',result=outcomes)
        finally:self.cleaning=False
        return dict(control=False,cleanup=outcomes)
    async def watch(self):
        while True:
            await asyncio.sleep(.25)
            if self.owner and self.clock()-self.heartbeat_at>=5:await self.release(self.owner)
    async def stop_check(self, board):
        if self.pending.get(board):raise Refused('有执行中的网页任务；先停止并等待结果')
        s=self.state(board)
        if not s.get('communication_ok') or not s.get('identity_ok') or s.get('stale') or s.get('unknown_command_state'):
            raise Refused('重启写入模式前必须有新鲜身份和停止状态；先以只读模式接管')
        recovery = self.manager.records[board]['options']['mode']=='save_recovery'
        if not recovery and (s.get('dirty') or s.get('storage_error') or s.get('pending_save')):
            raise Refused('dirty/存储/待保存未清，禁止普通模式重启；可在停稳后选择保存恢复模式')
        if any(x.get('mode') for x in s.get('manual',{}).values()):raise Refused('先退出人工调整并确认保存')
        if any(not x.get('brake') or x.get('duty_permille') or x.get('measured_rpm') or x.get('state') in (2,3,4,7,8) for x in s.get('channels',[])):
            raise Refused('先停止、锁刹车并确认整板静止')
        await asyncio.sleep(.6)
        after=self.state(board)
        if after.get('stale') or after.get('channels')!=s.get('channels'):raise Refused('静止确认期间状态变化，禁止重启')
    async def restart(self, token, request_id, board):
        self.require_owner(token)
        if board in self.restarting:raise Refused('已经在重启')
        job,new=self.store.create(request_id,'restart/'+board,{'revision':self.manager.records[board]['revision']})
        if not new:return job
        self.restarting.add(board)
        try:
            result=await self.manager.restart(board,self.bridge.discovered(),self.stop_check)
            self.bridge.invalidate(board);self.bridge.refresh_subscriptions()
            self.store.update(job['id'],status='succeeded',result=result)
        except Exception as e:self.store.update(job['id'],status='failed',message=str(e))
        finally:self.restarting.discard(board)
        return self.store.get(job['id'])
