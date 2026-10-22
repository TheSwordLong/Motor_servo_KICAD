"""Validated drafts and owned subprocesses. No serial I/O, shell=True, or pkill."""
import asyncio, copy, json, math, os, re, signal, subprocess, time
from pathlib import Path
import yaml
from robot_motor_driver.config import parse_config
from .models import RuntimeEdit, RuntimeOptions

class RuntimeManager:
    def __init__(self, config_file, state_dir, domain=173):
        self.source = Path(config_file).resolve()
        self.directory = Path(state_dir).resolve();self.directory.mkdir(parents=True,exist_ok=True)
        os.chmod(str(self.directory),0o700)
        self.domain = int(domain);self.processes = {};self.active = {}
        self.file = self.directory/'runtime.json'
        raw = yaml.safe_load(self.source.read_text());parse_config(raw)
        if self.file.exists():
            self.records = json.loads(self.file.read_text())
        else:
            self.records = {}
            for index,(name,b) in enumerate(raw['boards'].items()):
                self.records[name] = dict(revision=0, options=dict(board_name=name, namespace='board_'+chr(97+index),
                    mode='read_only',allowed_motion_joints=[],allow_unreferenced_relative=False,
                    commissioning=False,max_commissioning_step_deg=10.0,polling_hz=5.0),
                    board=b,joints={k:v for k,v in raw['joints'].items() if v['board']==name})
        for name,r in self.records.items():
            RuntimeOptions(**r['options'])
        self.validate(self.records)
        # On gateway restart runtime policy is read-only; never resume previous launch mode.
        for name,r in self.records.items():
            self.active[name] = copy.deepcopy(r)
            self.active[name]['options']['mode'] = 'read_only'
            self.active[name]['options']['allow_unreferenced_relative'] = False
    def validate(self, records):
        image = dict(driver=dict(allow_motion=False),boards={},joints={})
        namespaces = set()
        for ident,r in records.items():
            o = RuntimeOptions(**r['options']);name = o.board_name or ident
            if name in image['boards']:raise ValueError('duplicate board name')
            if o.namespace in namespaces:raise ValueError('duplicate ROS namespace')
            namespaces.add(o.namespace);image['boards'][name] = r['board']
            for label,j in r['joints'].items():
                if not re.fullmatch('[a-zA-Z][a-zA-Z0-9_]*',label):raise ValueError('invalid joint label')
                if label in image['joints']:raise ValueError('duplicate joint label')
                item = copy.deepcopy(j);item['board'] = name;image['joints'][label] = item
            if not set(o.allowed_motion_joints) <= set(r['joints']):raise ValueError('unknown motion whitelist joint')
        parse_config(image)
        return image
    def get(self, board):
        if board not in self.records:raise KeyError(board)
        r = copy.deepcopy(self.records[board]);r['owned'] = self.owned(board)
        r['active_options'] = copy.deepcopy(self.active[board]['options'])
        r['source_file'] = str(self.source)
        r['pending_restart'] = any(r[k] != self.active[board][k] for k in ('options','board','joints'))
        return r
    def edit(self, board, edit):
        current = self.records[board]
        if edit.revision != current['revision']:raise ValueError('configuration revision changed; reload')
        r = edit.dict();r['revision'] += 1
        if not r['options'].get('board_name'):r['options']['board_name'] = board
        candidate = copy.deepcopy(self.records);candidate[board] = r;self.validate(candidate)
        text = json.dumps(candidate,ensure_ascii=False,indent=2,allow_nan=False)
        if self.file.exists():self.file.with_name('runtime.before.json').write_bytes(self.file.read_bytes())
        temp = self.file.with_suffix('.tmp');temp.write_text(text);os.chmod(str(temp),0o600);temp.replace(self.file)
        self.records = candidate;return self.get(board)
    def owned(self, board):
        p = self.processes.get(board)
        return bool(p and p.returncode is None)
    def port_owners(self, port):
        if not port or not Path(port).exists():return []
        result = subprocess.run(['fuser', str(Path(port).resolve())],capture_output=True,text=True,timeout=3)
        if result.returncode not in (0,1):raise RuntimeError('cannot verify serial ownership: '+result.stderr)
        return [int(v) for v in result.stdout.split() if v.isdigit()]
    def external_conflicts(self, board, discovered):
        r = self.records[board];old = self.active[board]
        if not self.owned(board) and (old['options']['namespace'] in discovered or r['options']['namespace'] in discovered):
            raise RuntimeError('external ROS driver exists; stop it manually before gateway takeover')
        port = r['board'].get('port','')
        if r['board']['transport']=='modbus_rtu':
            owners = self.port_owners(port)
            own = self.processes.get(board)
            for pid in owners:
                try:belongs = own and os.getpgid(pid)==own.pid
                except ProcessLookupError:continue
                if not belongs:raise RuntimeError('serial port owned by external PID %d; not killed'%pid)
    async def restart(self, board, discovered, stop_check):
        self.external_conflicts(board,discovered)
        was_owned = self.owned(board)
        if was_owned:
            # Verify before touching an owned process; callback is supplied by the gateway.
            await stop_check(board)
            p = self.processes[board];os.killpg(p.pid,signal.SIGTERM)
            try:await asyncio.wait_for(p.wait(),10)
            except asyncio.TimeoutError:raise RuntimeError('owned node did not terminate; no duplicate started')
        elif self.records[board]['options']['mode'] != 'read_only':
            await stop_check(board)
        self.external_conflicts(board,set() if was_owned else discovered)
        image = self.validate(self.records);r = self.records[board];o = r['options']
        name = o.get('board_name') or board
        # Whole-image copy preserves other boards; no edits of the user's original YAML.
        path = self.directory/('launch_'+board+'.yaml')
        path.write_text(yaml.safe_dump(image,allow_unicode=True,sort_keys=False));os.chmod(str(path),0o600)
        args = ['ros2','launch','robot_motor_bringup','bringup.launch.py',
            'config_file:='+str(path),'board_name:='+name,'board_namespace:='+o['namespace'],
            'transport:='+r['board']['transport'],'ros_domain_id:='+str(self.domain),
            'allow_motion:='+str(o['mode']=='motion').lower(),'maintenance:='+str(o['mode'] in ('maintenance','save_recovery')).lower(),
            'save_recovery_only:='+str(o['mode']=='save_recovery').lower(),
            'allowed_motion_joints:='+(','.join(o['allowed_motion_joints']) or ' '),
            'allow_unreferenced_relative:='+str(o['allow_unreferenced_relative']).lower(),
            'commissioning:='+str(o['commissioning']).lower(),
            'max_commissioning_step_deg:='+str(o['max_commissioning_step_deg']),
            'polling_hz:='+str(o['polling_hz']),'require_uid:=true']
        log = open(str(self.directory/('node_'+board+'.log')),'ab',buffering=0)
        try:p = await asyncio.create_subprocess_exec(*args,stdout=log,stderr=log,start_new_session=True)
        finally:log.close()
        self.processes[board] = p;self.active[board] = copy.deepcopy(r)
        await asyncio.sleep(1)
        if p.returncode is not None:raise RuntimeError('node startup failed; inspect node log; no motion resumed')
        return dict(started=True,pid=p.pid,mode=o['mode'],message='waiting for fresh UID verification; no movement resumed')
    async def shutdown(self):
        for board,p in self.processes.items():
            if p.returncode is None:
                os.killpg(p.pid,signal.SIGTERM)
                try:await asyncio.wait_for(p.wait(),10)
                except asyncio.TimeoutError:pass
