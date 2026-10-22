"""ROS 2 driver for the GD32F303 robot motor board.

Importing this package never opens a serial port and never sends a frame: the
protocol, session, scheduler and simulated board layers are usable on their own,
which is what makes them unit testable without hardware.
"""

from . import protocol  # noqa: F401  (re-exported for convenience)

__all__ = ["protocol", "board_session", "scheduler", "transport_modbus",
           "transport_can", "fake_board", "config", "motor_node"]
__version__ = "0.1.0"
