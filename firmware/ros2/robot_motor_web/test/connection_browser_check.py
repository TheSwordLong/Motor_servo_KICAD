"""All scenario changes use a localhost mock; production page gets GET only."""
import json, os
from pathlib import Path
from playwright.sync_api import sync_playwright
root=Path(os.environ.get('WEB_CONNECTION_EVIDENCE','/data/ros2/robot_motor_ros2_ws/review_logs/web_connection_20261013'))
root.mkdir(exist_ok=True)
with sync_playwright() as p:
 browser=p.chromium.launch(headless=True,args=['--no-sandbox','--disable-gpu'])
 context=browser.new_context(viewport={'width':1440,'height':1000})
 page=context.new_page();errors=[];requests=[]
 page.on('pageerror',lambda e:errors.append(str(e)))
 page.on('request',lambda r:requests.append((r.method,r.url)))
 page.goto('http://127.0.0.1:8444/');page.wait_for_selector('.communication-badge')
 a=page.locator('#board-main_board');b=page.locator('#board-right_board')
 def contains(board,selector,label):board.locator(selector).filter(has_text=label).wait_for()
 contains(a,'.communication-badge','驱动板通信正常');contains(b,'.communication-badge','驱动板通信正常')
 assert page.locator('#control').is_enabled()
 page.get_by_role('button',name='检查两板连接',exact=True).click()
 page.get_by_role('button',name='已检查两板连接',exact=True).wait_for()
 assert page.request.get('http://127.0.0.1:8444/mock/calls').json()=={'writes':[],'control_owner':None,'processes':[]}
 for scenario,expected_a,expected_b in [('a_missing','串口未连接','驱动板通信正常'),('b_missing','驱动板通信正常','串口未连接'),('both_missing','串口未连接','串口未连接'),('a_stale','状态过期','驱动板通信正常'),('a_timeout','通信异常','驱动板通信正常'),('a_uid','UID不匹配','驱动板通信正常'),('a_no_node','节点未启动','驱动板通信正常')]:
  page.request.post('http://127.0.0.1:8444/mock/scenario/'+scenario)
  contains(a,'.communication-badge',expected_a);contains(b,'.communication-badge',expected_b)
  if scenario=='a_stale':assert '历史UID（非当前身份）' in a.locator('.connection-details').inner_text()
 page.request.post('http://127.0.0.1:8444/mock/scenario/fake')
 contains(a,'.serial-badge','模拟连接');contains(a,'.communication-badge','模拟板通信正常')
 assert not any(method!='GET' and '/api/' in url for method,url in requests),requests
 assert page.request.get('http://127.0.0.1:8444/mock/calls').json()['writes']==[]
 page.set_viewport_size({'width':390,'height':844})
 assert page.evaluate('() => document.documentElement.scrollWidth<=innerWidth')
 page.screenshot(path=str(root/'connection_mobile.png'),full_page=True)
 # Real board page: read-only check button issues GET only, never acquires control.
 requests.clear();page.set_viewport_size({'width':1440,'height':1000})
 page.goto('http://127.0.0.1:8443/');page.wait_for_selector('.connection-details')
 page.get_by_role('button',name='检查两板连接',exact=True).click()
 page.get_by_role('button',name='已检查两板连接',exact=True).wait_for()
 page.screenshot(path=str(root/'connection_real_readonly.png'),full_page=True)
 assert not any(method!='GET' and '/api/' in url for method,url in requests),requests
 assert not errors,errors
 result=dict(passed=True,independent_boards=True,missing_serial=True,node_not_started=True,stale=True,timeout=True,uid_mismatch=True,historical_uid=True,simulated_label=True,no_control_required=True,mobile_layout=True,console_errors=errors,hardware_writes=0)
 (root/'connection_browser_result.json').write_text(json.dumps(result,indent=2))
 print('PASS: dual-board connection states, GET check, no control/write, historical UID, mock unplug and mobile layout')
 browser.close()
