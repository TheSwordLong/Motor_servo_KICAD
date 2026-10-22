let csrf='',owns=false,boards=[],socket=null;
const $=s=>document.querySelector(s), names={head_joint:'头部',left_upper_arm_joint:'左大臂',left_forearm_joint:'左小臂',left_wrist_joint:'左手腕',right_upper_arm_joint:'右大臂',right_forearm_joint:'右小臂',right_wrist_joint:'右手腕'};
const id=()=>crypto.randomUUID?crypto.randomUUID():Date.now().toString(36)+'-'+Array.from(crypto.getRandomValues(new Uint32Array(4))).join('-');
const statusNames={queued:'排队',accepted:'已接受',running:'执行中',settling:'等待停止',succeeded:'成功',failed:'失败',unknown:'结果未知',cancelled:'已取消'};
const text=(el,v)=>{el.textContent=v??'—';};
function error(e){$('#error').hidden=false;text($('#error'),e.message||e);}
async function api(path,body,method=body===undefined?'GET':'POST'){
 const headers={'Content-Type':'application/json'};
 if(method!=='GET'){headers['X-CSRF-Token']=csrf;headers['X-Request-ID']=id();}
 const r=await fetch(path,{method,headers,body:body===undefined?undefined:JSON.stringify(body)});
 const data=await r.json();if(!r.ok)throw Error(typeof data.detail==='string'?data.detail:JSON.stringify(data.detail));return data;
}
async function control(){if(!owns){await api('/api/v1/control/acquire',{});owns=true;}}
async function write(path,data){await control();return api(path,data);}
const jointPath=(b,j)=>'/api/v1/boards/'+encodeURIComponent(b)+'/joints/'+encodeURIComponent(j);
function button(label,fn,cls=''){const b=document.createElement('button');b.textContent=label;b.className=cls;b.onclick=()=>Promise.resolve().then(fn).catch(error);return b;}
function makeBoard(b){
 const el=document.createElement('div');el.className='board';el.id='board-'+b.id;
 el.innerHTML='<div class="board-title"><span></span><span class="badge"></span></div><div class="connection-status"><span class="serial-badge badge"></span><span class="communication-badge badge"></span></div><dl class="connection-details"></dl><p></p><div class="board-actions"></div>';
 const area=el.querySelector('.board-actions');
 area.append(button('整板停止',()=>write('/api/v1/boards/'+b.id+'/stop',{}),'danger'),button('重试保存',()=>{if(confirm('确认整板停止并锁刹车？仅重试保存当前已应用配置和位置状态，不设零、不清故障、不运动。'))return write('/api/v1/boards/'+b.id+'/save',{confirm:true});}),button('启动 / 参数',()=>runtime(b.id)),button('板上配置 / 保存',()=>config(b.id)),button('整板自检',()=>{if(confirm('自检会逐路往返试转。确认机械活动空间？'))return write('/api/v1/boards/'+b.id+'/selftest',{joint_name:'',scope:1,direction:0,timeout_sec:60});}),button('确认 UNKNOWN',()=>{if(confirm('已核对实际请求结果、机构停稳和通信？此操作不会恢复运动。'))return write('/api/v1/boards/'+b.id+'/acknowledge-unknown',{confirm:true});}));
 $('#boards').append(el);
}
function makeJoint(b,j){
 const row=b.state.joints[j];const el=document.createElement('article');el.className='joint '+(j==='head_joint'?'head':'');el.id='joint-'+b.id+'-'+j;
 el.innerHTML='<div class="joint-title"><h3></h3><span class="badge"></span></div><div class="label"></div><div class="angle"></div><div class="metrics"><div><span>目标角度</span><strong data-m="target"></strong></div><div><span>减速前 RPM</span><strong data-m="rpm"></strong></div><div><span>低电平 PWM</span><strong data-m="pwm"></strong></div></div><div class="warning" hidden></div><form class="move-form"><label>运动模式<select name="mode"><option value="0">相对运动</option><option value="1">绝对坐标</option></select></label><label>输出轴角度 °<input name="angle" type="number" value="5" step="0.1" required></label><label>RPM校验（0沿用）<input name="rpm" type="number" value="0" min="0" max="8000" step="1"></label><label>监控超时 秒<input name="timeout" type="number" value="30" min="0.5" max="300" step="0.1"></label><button class="primary" type="submit">发送一次运动 →</button></form><div class="buttons"></div><details class="raw"><summary>原始状态 / FG / 保存信息</summary><pre></pre></details>';
 text(el.querySelector('h3'),names[j]||j);text(el.querySelector('.label'),j+' · '+row.physical_channel+' · '+b.id);
 el.querySelector('form').onsubmit=async ev=>{ev.preventDefault();try{const f=ev.target;if(!confirm('确认对'+(names[j]||j)+'执行一次运动？'))return;await write(jointPath(b.id,j)+'/move',{mode:Number(f.mode.value),angle_deg:Number(f.angle.value),motor_rpm:Number(f.rpm.value),timeout_sec:Number(f.timeout.value)});}catch(e){error(e);}};
 const area=el.querySelector('.buttons');
 area.append(button('停止',()=>write(jointPath(b.id,j)+'/stop',{}),'danger'),button('锁刹车',()=>write(jointPath(b.id,j)+'/brake',{engage:true})),button('DIR换向',async()=>{
 const row=boards.find(x=>x.id===b.id)?.state.joints[j];
 if(typeof row?.direction!=='boolean')throw Error('DIR状态未知，请先恢复通信');
 const high=!row.direction;
 if(confirm('确认'+(names[j]||j)+' DIR指令由'+(row.direction?'高':'低')+'→'+(high?'高':'低')+'？应已停稳并锁刹车。只切换DIR，不运动；下一次MOVE仍会按角度正负重设DIR。'))return write(jointPath(b.id,j)+'/direction',{high});
 },'direction-toggle'),button('释放刹车',()=>{if(confirm('请支撑负载。普通释放仍检测故障4。'))return write(jointPath(b.id,j)+'/brake',{engage:false});}),button('进入手调',()=>{if(confirm('请支撑负载，进入后位置无效；失联将尝试自动制动。'))return write(jointPath(b.id,j)+'/manual-adjust',{enable:true});}),button('退出手调',()=>write(jointPath(b.id,j)+'/manual-adjust',{enable:false})),button('当前姿态设零',()=>{if(confirm('确认当前机械姿态作为0°？这会保存到Flash。'))return write(jointPath(b.id,j)+'/zero',{confirm:true});}),button('清故障',()=>{if(confirm('已排查故障原因并停稳？清故障不会恢复零点。'))return write(jointPath(b.id,j)+'/clear-fault',{confirm:true});}),button('往返自检',()=>{if(confirm('确认允许这个关节往返试转？'))return write('/api/v1/boards/'+b.id+'/selftest',{joint_name:j,scope:0,direction:0,timeout_sec:60});}));
 const group=j==='head_joint'?'head':j.startsWith('right_')?'right':'left';$('#'+group).append(el);
}
const serialLabels={present:'串口存在',missing:'串口未连接',error:'串口检查失败',simulated:'模拟连接 / 无实机串口',not_applicable:'此通信方式不使用串口'};
const communicationLabels={connected:'驱动板通信正常',uid_mismatch:'UID不匹配',serial_missing:'未建立通信：串口未连接',serial_error:'未建立通信：串口检查失败',node_not_started:'节点未启动',node_check_error:'节点检查失败',waiting_state:'等待ROS状态',state_stale:'状态过期',communication_error:'通信异常',uid_unconfirmed:'UID尚未确认'};
function renderConnection(el,c){
 const serial=el.querySelector('.serial-badge'),communication=el.querySelector('.communication-badge');
 text(serial,serialLabels[c.serial_status]||'串口状态未知');serial.className='serial-badge badge '+(['present','simulated'].includes(c.serial_status)?'good':c.serial_status==='not_applicable'?'':'bad');
 text(communication,c.transport==='fake'&&c.communication_status==='connected'?'模拟板通信正常':communicationLabels[c.communication_status]||'通信状态未知');communication.className='communication-badge badge '+(c.communication_status==='connected'?'good':'bad');
 const stamp=c.last_state_stamp;const updated=stamp?new Date((stamp.sec+stamp.nanosec/1e9)*1000).toLocaleString():'尚未收到';
 const items=[['生效串口',c.transport==='fake'?'模拟板（不检查实机串口）':c.path||'不适用'],['实际设备',c.device||'未解析'],['预期UID',c.expected_uid||'未知'],['当前实际UID',c.current_uid||'当前未确认'],['最近状态更新时间',updated],['状态接收间隔',Number.isFinite(c.state_age_sec)?c.state_age_sec.toFixed(2)+' 秒':'未知']];
 if(!c.current_uid&&c.last_uid)items.push(['历史UID（非当前身份）',c.last_uid]);
 if(c.serial_error||c.node_error)items.push(['检查错误',c.serial_error||c.node_error]);
 const area=el.querySelector('.connection-details');area.replaceChildren();for(const [label,value]of items){const dt=document.createElement('dt'),dd=document.createElement('dd');text(dt,label);text(dd,value);area.append(dt,dd);}
}
async function checkConnections(){
 const btn=$('#check-connections');btn.disabled=true;
 try{
  const result=await api('/api/v1/boards');for(const b of result){const el=$('#board-'+b.id);if(el)renderConnection(el,b.state.connection||{});}
  text(btn,'已检查两板连接');setTimeout(()=>text(btn,'检查两板连接'),1800);
 }catch(e){error(e);}finally{btn.disabled=false;}
}
function render(data){
 boards=data.boards;owns=data.control;$('#control').disabled=owns;$('#release').disabled=!owns;
 text($('#connection'),owns?'本会话控制中':'仅查看 / 未取得控制权');$('#connection').className='badge '+(owns?'good':'');
 for(const b of boards){if(!$('#board-'+b.id))makeBoard(b);const el=$('#board-'+b.id),s=b.state;
 text(el.querySelector('.board-title span'),b.id+' / '+b.active_options.namespace);
 const permission=s.read_only||s.web_read_only?'只读':({read_only:'只读',maintenance:'维护',motion:'运动',save_recovery:'保存恢复'}[s.web_mode]||'权限未知');
 text(el.querySelector('.board-title .badge'),'运行权限：'+permission);el.querySelector('.board-title .badge').className='badge';
 renderConnection(el,s.connection||{});
 text(el.querySelector('p'),'dirty '+(s.dirty==null?'未知':s.dirty)+' / 存储错误 '+(s.storage_error==null?'未知':s.storage_error)+' / 待保存 '+(s.pending_save==null?'未知':s.pending_save)+(b.pending_restart?' · 配置待重启（连接检查使用当前生效配置）':''));
 for(const btn of el.querySelectorAll('.board-actions button')){
  if(btn.textContent==='重试保存')btn.disabled=Boolean(s.stale||!s.identity_ok||!s.communication_ok||s.read_only||s.web_read_only||s.unknown_command_state||Object.values(s.manual||{}).some(x=>x.mode)||(s.channels||[]).length!==4||(s.channels||[]).some(x=>!x.brake||x.duty_permille||x.measured_rpm||[2,3,4,7,8].includes(x.state)));
  if(['板上配置 / 保存','确认 UNKNOWN'].includes(btn.textContent))btn.disabled=s.web_mode==='save_recovery';
 }
 for(const btn of el.querySelectorAll('.board-actions button'))if(btn.textContent==='整板自检')btn.disabled=Boolean(s.stale||s.read_only||s.web_read_only||s.unknown_command_state||s.web_mode!=='motion');
 for(const [j,row]of Object.entries(s.joints||{})){if(!$('#joint-'+b.id+'-'+j))makeJoint(b,j);const card=$('#joint-'+b.id+'-'+j);
 text(card.querySelector('.angle'),s.stale||!s.communication_ok?'状态未知':row.angle_deg==null?'无有效零点':row.angle_deg.toFixed(1)+'°');text(card.querySelector('.badge'),s.stale||!s.communication_ok||row.brake==null?'状态未知':row.fault?'故障 '+row.fault:row.brake?'刹车锁定':'刹车释放');card.querySelector('.badge').className='badge '+(row.fault?'bad':'');
 text(card.querySelector('[data-m=target]'),row.target_deg==null?'—':row.target_deg+'°');text(card.querySelector('[data-m=rpm]'),(row.measured_rpm??'—')+' / '+(row.configured_rpm??'—'));text(card.querySelector('[data-m=pwm]'),row.duty_permille==null?'—':(row.duty_permille/10)+'%');
 const warnings=[];if(s.stale||!s.communication_ok)warnings.push('通信失效或状态过期');if(s.unknown_command_state)warnings.push('UNKNOWN：先排查并人工确认');if(Object.keys(row.binding_conflict).length)warnings.push('软件绑定与板上配置冲突：'+JSON.stringify(row.binding_conflict));if(row.manual_adjust?.mode)warnings.push('人工调整 / 退出等待：整板运动被互锁');
 const warn=card.querySelector('.warning');warn.hidden=!warnings.length;text(warn,warnings.join('\n'));
 const blocked=s.stale||!s.identity_ok||!s.communication_ok||s.read_only||s.web_read_only||s.unknown_command_state||s.storage_error||Object.keys(row.binding_conflict).length;
 const manualAny=Object.values(s.manual||{}).some(x=>x.mode===1||x.mode===2);
 for(const btn of card.querySelectorAll('.buttons button')){
  const emergency=['停止','锁刹车','退出手调'].includes(btn.textContent);
  btn.disabled=(s.web_mode==='save_recovery'&&btn.textContent!=='停止')||!emergency&&(blocked||(btn.textContent==='进入手调'?!row.manual_adjust.supported||row.fault&&row.fault!==4:btn.textContent==='清故障'?false:row.fault||manualAny));
 }
 card.querySelector('form button[type=submit]').disabled=Boolean(blocked||manualAny||row.fault||s.web_mode!=='motion');
 const directionButton=card.querySelector('.direction-toggle'),directionKnown=typeof row.direction==='boolean';
 text(directionButton,directionKnown?'DIR换向：'+(row.direction?'高→低':'低→高'):'DIR换向：未知');
 directionButton.title='只切换DIR电平指令，不运动；回读为软件指令位，电压需实测；下一次MOVE会重设DIR';
 directionButton.disabled=Boolean(blocked||row.fault||manualAny||s.web_mode==='save_recovery'||!directionKnown||!row.brake||row.duty_permille||row.measured_rpm||(s.channels||[]).length!==4||(s.channels||[]).some(x=>[2,3,4,7,8].includes(x.state)));
 for(const btn of card.querySelectorAll('.buttons button'))if(btn.textContent==='往返自检')btn.disabled=btn.disabled||s.web_mode!=='motion';text(card.querySelector('.raw pre'),JSON.stringify(row,null,2));
 }}
 renderJobs(data.jobs);
 if(data.cleanup.some(x=>!x.stop_confirmed))error('失联后存在停止未确认，请现场检查：'+JSON.stringify(data.cleanup));
}
// Keep task elements alive across 5 Hz snapshots so native details stay open.
function renderJobs(jobs){
 const list=$('#jobs'),existing=new Map(Array.from(list.children).map(el=>[el.dataset.jobId,el]));
 const visible=jobs.slice(0,12),keep=new Set(visible.map(j=>j.id));
 for(const el of Array.from(list.children))if(!keep.has(el.dataset.jobId))el.remove();
 visible.forEach((j,index)=>{
  let el=existing.get(j.id);
  if(!el){
   el=document.createElement('details');el.className='job';el.dataset.jobId=j.id;
   el.append(document.createElement('summary'),document.createElement('pre'));
   const cancel=button('取消动作',()=>write('/api/v1/jobs/'+j.id+'/cancel',{}));cancel.className='cancel-job';el.append(cancel);
  }
  const summary=new Date(j.created_at*1000).toLocaleTimeString()+' · '+(statusNames[j.status]||j.status)+' · '+j.path+(j.explanation?' · '+j.explanation:'');
  const raw=JSON.stringify(j,null,2),pre=el.querySelector('pre');
  if(el.querySelector('summary').textContent!==summary)text(el.querySelector('summary'),summary);
  if(pre.textContent!==raw){const offset=pre.scrollTop;text(pre,raw);pre.scrollTop=offset;}
  el.querySelector('.cancel-job').hidden=!['running','accepted','settling'].includes(j.status);
  if(list.children[index]!==el)list.insertBefore(el,list.children[index]||null);
 });
}
function field(label,value,type='text'){const l=document.createElement('label');text(l,label);const i=document.createElement('input');i.type=type;i.value=value;l.append(i);return [l,i];}
async function runtime(b){
 const r=await api('/api/v1/boards/'+b+'/runtime-config');const dialog=$('#settings');text($('#settings-title'),b+' · 文件设置 / 节点管理');const area=$('#settings-body');area.replaceChildren();
 const note=document.createElement('p');text(note,'保存文件不写Flash。仅管理网页启动的节点，终端节点须先手动停止。修改后默认不继续运动。');area.append(note);const fields=document.createElement('div');fields.className='fields';const inputs={};
 for(const [k,label]of Object.entries({board_name:'YAML板名',namespace:'ROS命名空间',max_commissioning_step_deg:'单次步长上限 °',polling_hz:'轮询频率 Hz'})){const [l,i]=field(label,r.options[k]);fields.append(l);inputs[k]=i;}
 const mode=document.createElement('label');text(mode,'权限模式');const select=document.createElement('select');for(const v of ['read_only','maintenance','motion','save_recovery']){const o=document.createElement('option');o.value=v;text(o,{read_only:'只读',maintenance:'维护',motion:'运动',save_recovery:'保存恢复（仅STOP/SAVE）'}[v]);select.append(o);}select.value=r.options.mode;mode.append(select);fields.append(mode);inputs.mode=select;
 for(const k of ['commissioning','allow_unreferenced_relative']){const [l,i]=field(k,r.options[k],'checkbox');i.checked=r.options[k];fields.append(l);inputs[k]=i;}
 const [wl,wi]=field('运动白名单（逗号分隔关节标签）',r.options.allowed_motion_joints.join(','));fields.append(wl);inputs.allowed=wi;area.append(fields);
 const boardTitle=document.createElement('p');text(boardTitle,'板通信绑定：UID、port、transport、address、baudrate（不会写入板端）');const bt=document.createElement('textarea');bt.value=JSON.stringify(r.board,null,2);area.append(boardTitle,bt);
 const jt=document.createElement('textarea');jt.value=JSON.stringify(r.joints,null,2);const title=document.createElement('p');text(title,'关节文件配置：channel为1～4；限位请用limits_deg且删除limits_rad。');area.append(title,jt);
 const output=document.createElement('pre');area.append(button('保存文件草稿',async()=>{const options={...r.options};for(const k of Object.keys(inputs)){if(k==='allowed')options.allowed_motion_joints=inputs[k].value.split(',').map(x=>x.trim()).filter(Boolean);else if(inputs[k].type==='checkbox')options[k]=inputs[k].checked;else options[k]=['polling_hz','max_commissioning_step_deg'].includes(k)?Number(inputs[k].value):inputs[k].value;}
 await control();const result=await api('/api/v1/boards/'+b+'/runtime-config',{revision:r.revision,options,board:JSON.parse(bt.value),joints:JSON.parse(jt.value)},'PUT');text(output,JSON.stringify(result,null,2));if(result.status==='succeeded')r.revision=result.result.revision;},'primary'),button('应用并重启本板节点',async()=>{if(confirm('确认整板静止、刹车锁定、无手调及存储错误？仅重启网页自有节点；外部节点不自动关闭。'))text(output,JSON.stringify(await write('/api/v1/boards/'+b+'/restart',{confirm:true}),null,2));}));
 area.append(output);dialog.showModal();
}
async function config(b){
 const s=await api('/api/v1/boards/'+b+'/state');const area=$('#settings-body');area.replaceChildren();text($('#settings-title'),b+' · 板上配置 / Flash');
 const p=document.createElement('p');text(p,'下面为实读四路配置。预览写入暂存区；提交会写Flash。请核对完整差异，不能自动搬用零点。channel为0～3。');area.append(p);
 const before=s.channels.map(c=>({channel:c.channel,enabled:c.enabled,role:c.role,gear_ratio:c.gear_ratio,inverted:c.inverted,rpm:c.configured_rpm}));const input=document.createElement('textarea');input.value=JSON.stringify(before,null,2);area.append(input);const out=document.createElement('pre');
 area.append(button('预览差异 / 暂存回读',async()=>{const channels=JSON.parse(input.value);text(out,'修改前：'+JSON.stringify(before,null,2)+'\n请求：'+JSON.stringify(channels,null,2)+'\n任务：'+JSON.stringify(await write('/api/v1/boards/'+b+'/config/preview',{channels}),null,2));}),button('确认提交并保存 Flash',async()=>{const channels=JSON.parse(input.value);if(confirm('确认这些修改写入板上Flash？\n'+JSON.stringify(channels)))text(out,JSON.stringify(await write('/api/v1/boards/'+b+'/config/apply',{channels,confirm:true}),null,2));},'primary'));area.append(out);$('#settings').showModal();
}
async function connect(){try{const s=await api('/api/v1/session');csrf=s.csrf;owns=s.control;socket=new WebSocket((location.protocol==='https:'?'wss:':'ws:')+'//'+location.host+'/events?csrf='+encodeURIComponent(csrf));socket.onmessage=e=>render(JSON.parse(e.data));socket.onclose=()=>{owns=false;text($('#connection'),'连接断开 / 操作结果需核对');setTimeout(connect,2000);};}catch(e){error(e);setTimeout(connect,3000);}}
$('#control').onclick=()=>control().catch(error);$('#release').onclick=()=>api('/api/v1/control/release',{}).then(()=>owns=false).catch(error);$('#allstop').onclick=()=>write('/api/v1/stop-all',{}).catch(error);
setInterval(()=>{if(owns)api('/api/v1/control/heartbeat',{}).catch(e=>{owns=false;error(e);});},1000);
addEventListener('beforeunload',()=>{if(owns)fetch('/api/v1/control/release',{method:'POST',headers:{'Content-Type':'application/json','X-CSRF-Token':csrf,'X-Request-ID':id()},body:'{}',keepalive:true});});
connect();

$('#check-connections').onclick=checkConnections;
