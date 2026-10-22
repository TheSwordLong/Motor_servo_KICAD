"""Read-only connection diagnostics. Never open a motor serial port."""
import copy
import time
from pathlib import Path


class ConnectionMonitor:
    def __init__(self, manager, bridge, clock=time.monotonic, wall_clock=time.time):
        self.manager = manager
        self.bridge = bridge
        self.clock = clock
        self.wall_clock = wall_clock
        self.cache = {}
        self.history = {}
        self.nodes = set()
        self.nodes_checked = None
        self.nodes_error = None

    def serial(self, board, config):
        now = self.clock()
        key = (config['transport'], config.get('port', ''))
        cached = self.cache.get(board)
        if cached and cached[0] == key and now - cached[1] < 1:
            return copy.deepcopy(cached[2])
        result = dict(transport=key[0], path=key[1], device=None,
                      serial_status='not_applicable', serial_error=None,
                      checked_at=self.wall_clock())
        if key[0] == 'fake':
            result['serial_status'] = 'simulated'
        elif key[0] == 'modbus_rtu':
            try:
                if not key[1]:
                    raise FileNotFoundError('serial path is empty')
                path = Path(key[1])
                path.stat()  # Follows symlinks; does not open the device.
                result['device'] = str(path.resolve(strict=True))
                result['serial_status'] = 'present'
            except FileNotFoundError:
                result['serial_status'] = 'missing'
            except (OSError, RuntimeError) as error:
                result.update(serial_status='error', serial_error=str(error))
        self.cache[board] = (key, now, copy.deepcopy(result))
        return result

    def snapshot(self, board, state):
        active = self.manager.active[board]
        config = active['board']
        result = self.serial(board, config)
        now = self.clock()
        if self.nodes_checked is None or now - self.nodes_checked >= 1:
            try:
                self.nodes = self.bridge.discovered()
                self.nodes_error = None
            except Exception as error:
                self.nodes = set()
                self.nodes_error = str(error)
            self.nodes_checked = now
        node_present = self.manager.owned(board) or active['options']['namespace'] in self.nodes
        binding = (config['transport'], config.get('port', ''), config['uid'], active['options']['namespace'])
        previous = self.history.get(board, {})
        if previous.get('binding') != binding:
            previous = dict(binding=binding, last_uid=None, last_state_stamp=None)
        uid = state.get('uid')
        stamp = state.get('header', {}).get('stamp')
        if uid:
            previous['last_uid'] = uid
        if stamp is not None:
            previous['last_state_stamp'] = copy.deepcopy(stamp)
        self.history[board] = previous
        fresh = not state.get('stale', True)
        communicating = bool(state.get('communication_ok'))
        serial_ok = result['serial_status'] not in ('missing', 'error')
        current = uid if node_present and fresh and communicating and serial_ok else None
        if current and (current != config['uid'] or not state.get('identity_ok')):
            status = 'uid_mismatch'
        elif result['serial_status'] == 'missing':
            status = 'serial_missing'
        elif result['serial_status'] == 'error':
            status = 'serial_error'
        elif not node_present:
            status = 'node_check_error' if self.nodes_error else 'node_not_started'
        elif not uid and stamp is None:
            status = 'waiting_state'
        elif not fresh:
            status = 'state_stale'
        elif not communicating:
            status = 'communication_error'
        elif not current or not state.get('identity_ok'):
            status = 'uid_unconfirmed'
        else:
            status = 'connected'
        result.update(node_present=node_present, node_error=self.nodes_error,
                      communication_status=status, expected_uid=config['uid'],
                      current_uid=current, last_uid=previous['last_uid'],
                      last_state_stamp=copy.deepcopy(previous['last_state_stamp']),
                      state_age_sec=state.get('gateway_age_sec'),
                      identity_ok=status == 'connected')
        return result
