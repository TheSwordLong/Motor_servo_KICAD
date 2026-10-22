"""Headless browser acceptance: all writes go to localhost:8444 mock, never hardware."""
import json,time
from pathlib import Path
from playwright.sync_api import sync_playwright
ROOT=Path('/data/ros2/robot_motor_ros2_ws/review_logs/web_dir_20261013');ROOT.mkdir(exist_ok=True)
with sync_playwright() as p:
 browser=p.chromium.launch(headless=True,args=['--no-sandbox','--disable-gpu'])
 # Real deployment is read-only inspection. No click or control claim on port8443.
 context=browser.new_context(viewport={'width':1440,'height':1100})
 page=context.new_page();errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
 page.goto('http://127.0.0.1:8443/');page.wait_for_selector('.joint',timeout=15000)
 assert page.locator('.joint').count()==7
 assert page.locator('#head .joint').count()==1
 page.screenshot(path=str(ROOT/'web_dashboard_20261012.png'),full_page=True)
 page.goto('http://127.0.0.1:8443/api/docs');page.wait_for_selector('#reference details')
 assert page.locator('#reference details').count()>20
 context.close()
 # All further operations are in-memory mock API, separate port, no rclpy/serial.
 a=browser.new_context(viewport={'width':1280,'height':1000});page=a.new_page();page.on('pageerror',lambda e:errors.append(str(e)))
 page.on('dialog',lambda d:d.accept())
 page.goto('http://127.0.0.1:8444/');page.wait_for_selector('.joint')
 assert page.locator('.joint').count()==7
 page.get_by_role('button',name='取得控制权',exact=True).click()
 page.wait_for_selector('#control:disabled')
 other=browser.new_context();otherpage=other.new_page();otherpage.goto('http://127.0.0.1:8444/');otherpage.wait_for_selector('.joint')
 otherpage.get_by_role('button',name='取得控制权',exact=True).click();otherpage.wait_for_selector('#error:not([hidden])');assert '其他浏览器' in otherpage.locator('#error').inner_text();other.close()
 failed=page.locator('#jobs .job').filter(has_text='/mock/failed-move')
 failed.locator('summary').click();failed.locator('pre').wait_for(state='visible')
 page.wait_for_timeout(1200)
 assert failed.get_attribute('open') is not None
 assert 'simulated failure' in failed.locator('pre').inner_text()
 move_requests=[]
 page.on('request',lambda r:move_requests.append(r.post_data_json) if r.method=='POST' and r.url.endswith('/move') else None)
 card=page.locator('#joint-main_board-head_joint')
 angle=card.locator('input[name=angle]');toggle=card.locator('.direction-toggle')
 low_high='低→高';high_low='高→低'
 angle.fill('15');toggle.filter(has_text=low_high).wait_for();toggle.click()
 toggle.filter(has_text=high_low).wait_for();assert angle.input_value()=='15' and not move_requests
 toggle.click();toggle.filter(has_text=low_high).wait_for()
 toggle.click();toggle.filter(has_text=high_low).wait_for()
 assert not move_requests and angle.input_value()=='15'
 card.locator('input[name=angle]').fill('5');card.locator('button[type=submit]').click()
 card.locator('.angle').filter(has_text='5.0°').wait_for()
 toggle.filter(has_text=low_high).wait_for()  # MOVE chooses DIR from angle sign again.
 assert failed.get_attribute('open') is not None  # New task did not collapse previous error.
 # Results must remain open across multiple 5Hz updates, including arrival of new jobs.
 job=page.locator('#jobs .job').first
 job_id=job.get_attribute('data-job-id');assert job_id
 job.locator('summary').click();job.locator('pre').wait_for(state='visible')
 page.wait_for_timeout(1400)
 assert page.locator('#jobs .job[data-job-id="'+job_id+'"]').get_attribute('open') is not None
 assert page.locator('#jobs .job[data-job-id="'+job_id+'"] pre').inner_text().find('result')>=0
 page.locator('#jobs .job[data-job-id="'+job_id+'"] summary').click()
 page.wait_for_timeout(600)
 assert page.locator('#jobs .job[data-job-id="'+job_id+'"]').get_attribute('open') is None
 # SAVE button is tested only on the in-memory mock, never the real page.
 page.locator('#board-main_board').get_by_role('button',name='重试保存',exact=True).click()
 saved=page.locator('#jobs .job').filter(has_text='/api/v1/boards/main_board/save')
 saved.wait_for();saved.locator('summary').filter(has_text='成功').wait_for()
 saved.locator('summary').click();saved.locator('pre').wait_for(state='visible')
 assert 'flags_valid' in saved.locator('pre').inner_text()
 # Settings dialogue is read-only display; no restart or physical configuration change.
 page.locator('#board-main_board').get_by_role('button',name='启动 / 参数',exact=True).click()
 page.locator('#settings').wait_for(state='visible');assert page.locator('#settings input').count()>=7
 page.locator('#settings .close').click()
 card.get_by_role('button',name='进入手调',exact=True).click()
 card.locator('.angle').filter(has_text='无有效零点').wait_for()
 assert card.locator('form button[type=submit]').is_disabled();assert toggle.is_disabled()
 page.screenshot(path=str(ROOT/'web_mock_manual_20261012.png'),full_page=True)
 # Closing owner browser must make mock STOP_ALL and brake confirmed; inspect via separate HTTP browser request.
 a.close();time.sleep(2)
 q=browser.new_context();r=q.request.get('http://127.0.0.1:8444/api/v1/boards/main_board/state').json()
 assert r['joints']['head_joint']['brake'] is True and r['manual']['head_joint']['mode']==0
 # Mobile layout with same source and no write actions.
 mobile=browser.new_context(viewport={'width':390,'height':844});m=mobile.new_page();m.goto('http://127.0.0.1:8444/');m.wait_for_selector('.joint')
 assert m.evaluate('() => document.documentElement.scrollWidth<=innerWidth')
 m.screenshot(path=str(ROOT/'web_mobile_20261012.png'),full_page=True)
 assert not errors,errors
 (ROOT/'web_browser_result_20261012.json').write_text(json.dumps(dict(passed=True,joints=7,head_first=True,
  no_login=True,api_reference=True,exclusive_control=True,degree_motion_mock=True,manual_invalid_position=True,
  disconnect_brake_mock=True,job_expansion_persistent=True,dir_toggle_mock=True,retry_save_mock=True,mobile_layout=True,console_errors=errors,real_hardware_writes=0),indent=2))
 print('PASS: browser dashboard, API reference, seven joints, mock motion/manual, control conflict, disconnect brake and mobile layout')
 browser.close()
