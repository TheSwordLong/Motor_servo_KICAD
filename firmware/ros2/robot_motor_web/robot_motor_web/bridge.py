"""ROS-only bridge; executor callbacks run in one separate thread."""
import asyncio, copy, threading, time, array

def plain(message):
    if hasattr(message,'get_fields_and_field_types'):
        return {key:plain(getattr(message,key)) for key in message.get_fields_and_field_types()}
    if isinstance(message,(list,tuple,array.array,bytes,bytearray)):return [plain(v) for v in message]
    if hasattr(message,'tolist'):return plain(message.tolist())
    if hasattr(message,'item'):return plain(message.item())
    return message

class RosBridge:
    def __init__(self, manager):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        self.rclpy = rclpy;self.owns_context=not rclpy.ok()
        if self.owns_context:rclpy.init()
        self.node = Node('robot_motor_web_bridge');self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node);self.manager = manager
        self.lock = threading.RLock();self.states = {};self.manual = {};self.received = {};self.subscriptions = []
        self.goals = {};self.clients = {};self.watched = set();self.refresh_subscriptions()
        self.thread = threading.Thread(target=self.executor.spin,daemon=True);self.thread.start()
    def refresh_subscriptions(self):
        from robot_motor_interfaces.msg import MotorStatesDeg,ManualAdjustState
        for board,r in self.manager.active.items():
            ns = r['options']['namespace']
            key = (board,ns,tuple(sorted(r['joints'])))
            if key in self.watched:continue
            # Old subscriptions cannot populate a new binding after an explicit restart.
            self.watched.add(key)
            def state_cb(msg,b=board,k=key):
                if self.manager.active[b]['options']['namespace']!=k[1]:return
                with self.lock:self.states[b]=plain(msg);self.received[b]=time.monotonic()
            self.subscriptions.append(self.node.create_subscription(MotorStatesDeg,'/'+ns+'/robot/motor_states_deg',state_cb,10))
            for j in r['joints']:
                def manual_cb(msg,b=board,j=j,k=key):
                    if self.manager.active[b]['options']['namespace']!=k[1] or j not in self.manager.active[b]['joints']:return
                    with self.lock:self.manual[(b,j)]=(plain(msg),time.monotonic())
                self.subscriptions.append(self.node.create_subscription(ManualAdjustState,'/'+ns+'/robot/joints/'+j+'/manual_adjust',manual_cb,10))
    def snapshot(self, board):
        with self.lock:
            s = copy.deepcopy(self.states.get(board,{}));stamp = self.received.get(board,0)
            for j in self.manager.active[board]['joints']:
                item,t = self.manual.get((board,j),({},0));s.setdefault('manual',{})[j] = dict(item,stale=item.get('stale',True) or time.monotonic()-t>2)
        s['gateway_age_sec'] = time.monotonic()-stamp if stamp else None
        s['stale'] = s.get('stale',True) or not stamp or time.monotonic()-stamp>2
        return s
    def discovered(self):
        return {ns.strip('/') for name,ns in self.node.get_node_names_and_namespaces() if name=='robot_motor_driver'}
    def invalidate(self, board):
        with self.lock:
            self.states.pop(board,None);self.received.pop(board,None)
            for key in list(self.manual):
                if key[0]==board:self.manual.pop(key)
    async def wait(self, future, timeout):
        loop = asyncio.get_running_loop();done = loop.create_future()
        def resolve(f):
            def finish():
                if done.done():return
                try:done.set_result(f.result())
                except Exception as e:done.set_exception(e)
            loop.call_soon_threadsafe(finish)
        future.add_done_callback(resolve)
        # Shield ROS future: timeout means unknown, never cancel/retry a possibly sent service.
        return await asyncio.wait_for(asyncio.shield(done),timeout)
    async def service(self, board, service, payload):
        from std_srvs.srv import Trigger
        from robot_motor_interfaces.srv import Stop,ClearFault,SetZero,SetBrake,SetManualAdjust,ApplyConfig,RetrySave,SetDirection
        from robot_motor_interfaces.msg import ChannelConfig
        types={'stop':Stop,'clear_fault':ClearFault,'set_zero':SetZero,'set_brake':SetBrake,
               'set_manual_adjust':SetManualAdjust,'apply_config':ApplyConfig,
               'acknowledge_unknown':Trigger,'board_identity':Trigger,'retry_save':RetrySave,'set_direction':SetDirection}
        typ=types[service];ns=self.manager.active[board]['options']['namespace']
        key=(ns,service);client=self.clients.get(key)
        if client is None:client=self.node.create_client(typ,'/'+ns+'/robot/'+service);self.clients[key]=client
        if not client.service_is_ready():raise RuntimeError('ROS service unavailable: '+service)
        request=typ.Request()
        for k,v in payload.items():
            if k=='channels':
                values=[]
                for row in v:
                    x=ChannelConfig()
                    for field,value in row.items():setattr(x,field,value)
                    values.append(x)
                setattr(request,k,values)
            else:setattr(request,k,v)
        return plain(await self.wait(client.call_async(request),30 if service=='apply_config' else 12))
    async def action(self, board, action, payload, job_id, feedback):
        from rclpy.action import ActionClient
        from robot_motor_interfaces.action import MoveJointDeg,MoveJoint,RunSelfTest
        types={'move_joint_deg':MoveJointDeg,'move_joint':MoveJoint,'selftest':RunSelfTest};typ=types[action]
        ns=self.manager.active[board]['options']['namespace'];key=(ns,action);client=self.clients.get(key)
        if client is None:client=ActionClient(self.node,typ,'/'+ns+'/robot/'+action);self.clients[key]=client
        if not client.server_is_ready():raise RuntimeError('ROS action server unavailable')
        goal=typ.Goal()
        for k,v in payload.items():setattr(goal,k,v)
        loop=asyncio.get_running_loop()
        def cb(msg):loop.call_soon_threadsafe(feedback,plain(msg.feedback))
        handle=await self.wait(client.send_goal_async(goal,feedback_callback=cb),8)
        if not handle.accepted:return dict(accepted=False,error_code=114,message='ROS goal rejected; no movement accepted')
        self.goals[job_id]=handle;feedback({'goal_accepted':True})
        try:
            result=await self.wait(handle.get_result_async(),float(payload['timeout_sec'])+15)
            return dict(plain(result.result),goal_status=int(result.status),accepted=True)
        finally:self.goals.pop(job_id,None)
    async def cancel(self, job_id):
        handle=self.goals.get(job_id)
        if not handle:raise RuntimeError('goal handle unavailable; use STOP and verify')
        return plain(await self.wait(handle.cancel_goal_async(),8))
    def close(self):
        self.executor.shutdown(timeout_sec=3);self.thread.join(timeout=3)
        self.node.destroy_node()
        if self.owns_context:self.rclpy.shutdown()
