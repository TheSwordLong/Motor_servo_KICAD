import copy
import importlib.util
import pathlib
import unittest
from unittest import mock
import contextlib
import io

ROOT = pathlib.Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("robot_host", ROOT / "tools/robot_host.py")
host = importlib.util.module_from_spec(spec)
spec.loader.exec_module(host)


class HostTests(unittest.TestCase):
    def inventory(self):
        return {"boards": {"12345678ABCDEF0198765432": {
            "protocol": 0, "channels": [
                {"enabled": True, "role": "head", "gear": 516},
                {"enabled": False, "role": "unassigned", "gear": 0},
                {"enabled": False, "role": "unassigned", "gear": 0},
                {"enabled": False, "role": "unassigned", "gear": 0}]}}}

    def test_crc_and_signed(self):
        self.assertEqual(host.crc16(bytes.fromhex("01030000000A")), 0xCDC5)
        self.assertEqual(host.signed32([0xFFFF, 0xFC7C]), -900)

    def test_inventory_duplicates(self):
        inv = self.inventory()
        host.validate_inventory(inv)
        inv["boards"]["AAAAAAAAAAAAAAAAAAAAAAAA"] = copy.deepcopy(next(iter(inv["boards"].values())))
        with self.assertRaises(ValueError):
            host.validate_inventory(inv)
        for index, board in enumerate(inv["boards"].values()):
            board["channels"][0]["role"] = "head" if index == 0 else "left_upper"
            board["protocol"] = 1
        with self.assertRaises(ValueError):
            host.validate_inventory(inv)  # both default node 1

    def test_uid_rebind(self):
        with self.assertRaises(ValueError):
            host.configure(None, self.inventory(), "AAAAAAAAAAAAAAAAAAAAAAAA", 1)

    def test_modbus_direct_command(self):
        client = object.__new__(host.Modbus)
        calls = []
        client.exchange = lambda pdu: calls.append(pdu)
        client.read = lambda address, count: [2]
        client.command(123, 3, 2, -900)
        self.assertEqual(calls[0].hex(), "10003000050a007b00030002fffffc7c")
        self.assertEqual(len(calls), 1)
        client.read = lambda address, count: [1]
        with self.assertRaises(RuntimeError):
            client.command(124, 3, 2, 200)
        self.assertEqual(len(calls), 1)

    def test_can_encoding(self):
        client = object.__new__(host.Can)
        calls = []
        client.exchange = lambda data: calls.append(data)
        client.command(123, 3, 2, -900)
        self.assertEqual(calls[0].hex(), "03007b0302fc7c00")
        client.tag = 0
        client.write(0x10A, [516])
        self.assertEqual(calls[1].hex(), "0201010a00020400")

    def test_selftest_decode(self):
        result = host.selftest_result([4, 7, 0x8000, 5, 300, 0xFFFF, 12, 65535])
        self.assertEqual(result["state_name"], "passed")
        self.assertEqual(result["reason_name"], "storage-error")
        self.assertEqual(result["pulses"], 0x80000005)
        self.assertEqual(result["completed_uptime_ms"], 0xFFFF000C)
        self.assertEqual(result["sequence"], 65535)
        detail = host.selftest_result([5,10,0,11,120,0,1800,7], [5,100,0,5,0,6,65535,65535])
        self.assertEqual(detail["residual_pulses"], -1)
        self.assertEqual(detail["outbound_pulses"], 5)
        self.assertEqual(detail["return_pulses"], 6)
        self.assertEqual(detail["phase_name"], "finished")
        self.assertEqual(detail["reason_name"], "return-error")

    def test_selftest_cli(self):
        for transport in ("modbus", "can"):
            for motor in ("1", "2", "3", "4", "all"):
                for direction in (None, "positive", "negative"):
                    with self.subTest(transport=transport, motor=motor, direction=direction):
                        client = mock.Mock()
                        client.read.side_effect = [[0x1234,0x5678,0xABCD,0xEF01,0x9876,0x5432], [65535]]
                        argv = ["robot_host.py", "--transport", transport, "--port", "unused",
                                "--uid", "12345678ABCDEF0198765432", "selftest", motor]
                        if direction:
                            argv += ["--direction", direction]
                        with mock.patch("sys.argv", argv), mock.patch.object(host, "Modbus", return_value=client), \
                             mock.patch.object(host, "Can", return_value=client), contextlib.redirect_stdout(io.StringIO()):
                            host.main()
                        client.command.assert_called_once_with(1, 11 if motor == "all" else 10,
                                                               0 if motor == "all" else int(motor)-1,
                                                               int(direction == "negative"))
                        client.close.assert_called_once()

    def test_retrying_clients_reject_auto_sequence(self):
        for cls in (host.Modbus, host.Can):
            client = object.__new__(cls)
            client.exchange = mock.Mock()
            with self.assertRaises(ValueError):
                client.command(0, 3, 0, 200)
            client.exchange.assert_not_called()
        client = mock.Mock()
        client.read.return_value = [0x1234,0x5678,0xABCD,0xEF01,0x9876,0x5432]
        argv = ["robot_host.py", "--port", "unused", "--uid", "12345678ABCDEF0198765432",
                "--seq", "0", "move", "1", "20"]
        with mock.patch("sys.argv", argv), mock.patch.object(host, "Modbus", return_value=client):
            with self.assertRaises(ValueError):
                host.main()
        client.command.assert_not_called()
        client.close.assert_called_once()

    def test_selftest_uid_required(self):
        with mock.patch("sys.argv", ["robot_host.py", "--port", "unused", "selftest", "all"]), \
             mock.patch.object(host, "Modbus") as connect, contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                host.main()
            connect.assert_not_called()

    def test_status_selftest(self):
        client = mock.Mock()
        def read(address, count):
            if address >= 0x100 and address % 0x40 == 0x20:
                return [4, 0, 0, 3, 20, 0, 1100, 5]
            return [0] * count
        client.read.side_effect = read
        output = io.StringIO()
        with mock.patch("sys.argv", ["robot_host.py", "--port", "unused", "status"]), \
             mock.patch.object(host, "Modbus", return_value=client), contextlib.redirect_stdout(output):
            host.main()
        self.assertEqual(output.getvalue().count('"state_name": "passed"'), 4)
        client.command.assert_not_called()


if __name__ == "__main__":
    unittest.main()
