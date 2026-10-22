"""Modbus RTU master for one serial port, with bounded transactions.

Design rules taken from the execution plan and the firmware:

* ONE owner per port. Every public method takes the transport lock, and with
  `strict_ownership` a second thread that ever touches the port raises instead
  of silently interleaving frames. The ROS driver keeps all serial I/O on its
  worker thread; nothing else may call in.
* Bounded transactions. Each exchange has a monotonic deadline; there is no
  unbounded wait anywhere, so a STOP request can always be serviced.
* Retries only ever resend the IDENTICAL bytes. That is safe because the board
  caches the last 16 (sequence, op, ch, arg) tuples and replays a match with
  zero side effects (BSP/robot.c:480-483). A retry never allocates a new
  sequence; that decision belongs to board_session, not to the transport.
* Silence is normal. A CRC or address error makes the board not answer at all
  (BSP/robot_protocol.c:16), and an intra-frame gap above 750 us invalidates the
  whole ADU (BSP/robot_rtu.c:9-15), so writes are issued as one buffer.
"""

import threading
import time
from typing import List, Optional, Sequence

from . import protocol as proto


class TransportError(proto.ProtocolError):
    """Base class for transport level failures."""


class TransportNotAvailable(TransportError):
    """The transport cannot be used on this host (missing dependency, no device)."""


class TransportOwnershipError(TransportError):
    """A second thread tried to use a port that already has an owner."""


class TransportClosed(TransportError):
    """The port is not open."""


def _default_serial_factory(port, baudrate, timeout, write_timeout, inter_byte_timeout=0.005):
    import serial  # imported lazily so unit tests need no pyserial

    # inter_byte_timeout is what keeps polling cheap: without it, read(n) blocks for
    # the whole `timeout` whenever fewer than n bytes are available, so every
    # transaction costs 50 ms and a 14-transaction poll cannot reach 5 Hz (observed
    # as "data stale (age 1.46 s)" against a real board). The firmware treats a gap
    # above 1750 us as end of frame (BSP/robot_rtu.c:9-15), so a 5 ms inter-byte gap
    # is a safe "the frame is complete" signal.
    return serial.Serial(port=port, baudrate=baudrate, bytesize=serial.EIGHTBITS,
                         parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                         timeout=timeout, write_timeout=write_timeout,
                         inter_byte_timeout=inter_byte_timeout)


class ModbusRtuTransport(object):
    """Blocking Modbus RTU master. Thread safe, single owner, bounded timeouts."""

    #: How many bytes of leading garbage to discard while hunting for our address.
    MAX_RESYNC_BYTES = 512

    def __init__(self, port, address=1, baudrate=115200,
                 response_timeout=proto.DEFAULT_RESPONSE_TIMEOUT,
                 inter_frame_delay=proto.DEFAULT_INTER_FRAME_DELAY,
                 read_retries=2, write_retries=1,
                 serial_factory=None, clock=None, strict_ownership=True,
                 read_timeout=0.05, write_timeout=0.5, inter_byte_timeout=0.005):
        # type: (...) -> None
        self.port = port
        self.address = proto.validate_address(address)
        self.baudrate = int(baudrate)
        self.response_timeout = float(response_timeout)
        self.inter_frame_delay = float(inter_frame_delay)
        self.read_retries = int(read_retries)
        self.write_retries = int(write_retries)
        self.strict_ownership = bool(strict_ownership)
        self._serial_factory = serial_factory or _default_serial_factory
        self._clock = clock or time.monotonic
        self._read_timeout = float(read_timeout)
        self._write_timeout = float(write_timeout)
        self._inter_byte_timeout = float(inter_byte_timeout)
        self._serial = None
        self._lock = threading.Lock()
        self._owner = None          # type: Optional[int]
        self.transactions = 0
        self.timeouts = 0
        self.frame_errors = 0
        self.exceptions = 0
        self.resyncs = 0

    # -- lifecycle ---------------------------------------------------------

    def open(self):
        # type: () -> bool
        with self._lock:
            if self._serial is not None:
                return True
            self._serial = self._serial_factory(self.port, self.baudrate,
                                                self._read_timeout, self._write_timeout,
                                                self._inter_byte_timeout)
            return True

    def close(self):
        # type: () -> None
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                finally:
                    self._serial = None

    @property
    def is_open(self):
        # type: () -> bool
        return self._serial is not None

    @property
    def owner(self):
        # type: () -> Optional[int]
        return self._owner

    def release_ownership(self):
        # type: () -> None
        """Hand the port to another thread.

        Needed when one thread opens and identifies the board (for example a node's
        constructor) and a different thread then owns all further traffic (the
        worker). Without this, `strict_ownership` would make the worker's very first
        transaction fail, because the port would still be recorded as owned by the
        thread that identified it.
        """
        with self._lock:
            self._owner = None

    def _claim(self):
        # type: () -> None
        current = threading.current_thread().ident
        if self._owner is None:
            self._owner = current
        elif self.strict_ownership and self._owner != current:
            raise TransportOwnershipError(
                "port %s is owned by thread %s; thread %s must not use it"
                % (self.port, self._owner, current))

    def _require_open(self):
        if self._serial is None:
            raise TransportClosed("port %s is not open" % (self.port,))

    # -- transactions ------------------------------------------------------

    def exchange(self, pdu, retries=None, expect_response=True):
        # type: (bytes, Optional[int], bool) -> bytes
        """Send one PDU and return the response PDU (function byte onwards).

        Raises ModbusExceptionResponse for an exception frame, ModbusTimeout when
        no valid frame arrived inside the deadline, and FrameError for a reply
        that fails structural validation. `retries` counts ADDITIONAL identical
        attempts; None picks the read or write default.
        """
        with self._lock:
            self._claim()
            self._require_open()
            is_read = pdu[0] == 3
            if retries is None:
                retries = self.read_retries if is_read else self.write_retries
            attempts = 1 + max(0, int(retries))
            frame = proto.build_adu(self.address, pdu)
            last_error = None      # type: Optional[Exception]
            for attempt in range(attempts):
                if attempt:
                    time.sleep(self.inter_frame_delay)
                try:
                    return self._transact(frame, pdu, expect_response)
                except proto.ModbusTimeout as error:
                    last_error = error
                    self.timeouts += 1
                except proto.FrameError as error:
                    # A structurally broken reply means the stream desynced; the
                    # next attempt starts from a cleared input buffer.
                    last_error = error
                    self.frame_errors += 1
                except proto.ModbusExceptionResponse:
                    # An exception frame is a valid answer: never retried.
                    self.exceptions += 1
                    raise
            raise last_error if last_error else proto.ModbusTimeout("no response")

    def _transact(self, frame, request_pdu, expect_response):
        # type: (bytes, bytes, bool) -> bytes
        self.transactions += 1
        try:
            self._serial.reset_input_buffer()
            self._serial.write(frame)
            self._serial.flush()
        except Exception as error:  # pyserial raises SerialException and friends
            raise proto.ModbusTimeout("write failed: %s" % (error,))
        if not expect_response:
            return b""
        deadline = self._clock() + self.response_timeout
        buffer = bytearray()
        while True:
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise proto.ModbusTimeout(
                    "no complete reply within %.3f s (got %d bytes: %s)"
                    % (self.response_timeout, len(buffer), bytes(buffer).hex()))
            chunk = self._read(min(self._missing_bytes(buffer), proto.MAX_ADU_BYTES),
                               min(remaining, self._read_timeout))
            if chunk:
                buffer.extend(chunk)
                response = self._extract_frame(buffer, request_pdu)
                if response is not None:
                    return response
                if len(buffer) > proto.MAX_ADU_BYTES:
                    raise proto.FrameError("reply exceeded %d bytes: %s"
                                           % (proto.MAX_ADU_BYTES, bytes(buffer)[:64].hex()))
            else:
                time.sleep(0.001)

    def _missing_bytes(self, buffer):
        # type: (bytearray) -> int
        """How many more bytes to ask the port for.

        Asking for a fixed large count is what makes polling slow: pyserial only
        returns early once the requested number of bytes has arrived, so `read(64)`
        against a 9 byte reply blocks for the whole read timeout. Measured on the real
        board that is 50.5 ms per transaction and ~911 ms for one 18-transaction
        poll, which cannot sustain 5 Hz. Requesting exactly what is still missing
        lets read() return the moment the frame is complete.
        """
        if len(buffer) < 2:
            return 2 - len(buffer)
        expected = proto.expected_response_length(buffer)
        if expected is None:
            # Undecodable function code: pull a few more bytes so _extract_frame can
            # either resynchronise or fail loudly, instead of stalling.
            return max(1, 5 - len(buffer))
        return max(1, expected - len(buffer))

    def _read(self, count, timeout):
        # type: (int, float) -> bytes
        try:
            if timeout is not None and timeout <= 0:
                return b""
            return self._serial.read(count)
        except Exception as error:
            raise proto.ModbusTimeout("read failed: %s" % (error,))

    def _extract_frame(self, buffer, request_pdu):
        # type: (bytearray, bytes) -> Optional[bytes]
        """Return the validated response PDU once a complete frame is present.

        Bytes that cannot start a valid frame are dropped IN PLACE and the scan
        continues, so a stale half frame left by a previous timeout, or another
        master's traffic on a shared RS-485 bus, resynchronises instead of killing
        the transaction. What bounds the scan is the transaction deadline, not a
        retry counter. Returns None when more bytes are still needed.
        """
        while True:
            index = 0
            while index < len(buffer) and buffer[index] != self.address:
                index += 1
                if index > self.MAX_RESYNC_BYTES:
                    raise proto.FrameError("no frame start within %d bytes" % self.MAX_RESYNC_BYTES)
            if index:
                del buffer[:index]
            if len(buffer) < 2:
                return None
            expected = proto.expected_response_length(buffer)
            if expected is None:
                if len(buffer) < 5:
                    return None
                # An unknown function code cannot be length-decoded: drop one byte
                # and keep looking rather than trusting a bogus length.
                del buffer[0]
                self.resyncs += 1
                continue
            if len(buffer) < expected:
                return None
            frame = bytes(buffer[:expected])
            if not proto.frame_crc_ok(frame):
                del buffer[0]
                self.resyncs += 1
                continue
            del buffer[:expected]
            pdu = frame[1:-2]
            if pdu[0] == (request_pdu[0] | 0x80):
                if len(pdu) < 2:
                    raise proto.FrameError("truncated exception frame %s" % frame.hex())
                raise proto.ModbusExceptionResponse(pdu[1], "request %s" % request_pdu.hex())
            if pdu[0] != request_pdu[0]:
                # A well-formed frame for a different function is a desync, not
                # noise: fail loudly instead of silently waiting for the deadline.
                raise proto.FrameError("reply function 0x%02X does not match request 0x%02X"
                                       % (pdu[0], request_pdu[0]))
            if request_pdu[0] in (6, 16):
                proto.validate_write_echo(pdu, request_pdu)
            return pdu

    # -- register level helpers -------------------------------------------

    def read_registers(self, address, count, retries=None):
        # type: (int, int, Optional[int]) -> List[int]
        """FC3 read. Reads are idempotent, so limited retries are allowed."""
        pdu = proto.read_pdu(address, count)
        response = self.exchange(pdu, retries=retries)
        return proto.parse_read_response(response, count)

    def write_registers(self, address, values, retries=None):
        # type: (int, Sequence[int], Optional[int]) -> None
        """FC6 for one register, FC16 for several. Never used for the command block."""
        values = list(values)
        pdu = (proto.write_single_pdu(address, values[0]) if len(values) == 1
               else proto.write_multi_pdu(address, values))
        self.exchange(pdu, retries=retries)

    def write_command(self, sequence, opcode, channel, argument, retries=None):
        # type: (int, int, int, int, Optional[int]) -> None
        """The single frame command: FC16 to 0x0030 with exactly 5 registers."""
        self.exchange(proto.command_pdu(sequence, opcode, channel, argument), retries=retries)

    # -- introspection -----------------------------------------------------

    def statistics(self):
        # type: () -> dict
        return {"port": self.port, "address": self.address, "baudrate": self.baudrate,
                "open": self.is_open, "transactions": self.transactions,
                "timeouts": self.timeouts, "frame_errors": self.frame_errors,
                "exceptions": self.exceptions, "resyncs": self.resyncs,
                "owner_thread": self._owner}

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False
