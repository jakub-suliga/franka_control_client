from __future__ import annotations

from collections import deque
from dataclasses import dataclass
# Remove IntEnum import if not used by other enums, otherwise keep.
# For now, I'll assume ControlMode still needs it.
from enum import IntEnum
from typing import Deque, Final, List, Optional, Tuple
import socket

from ...core.message import MsgID # Add this import
import struct
import threading
import time

import zmq

from ...core.remote_device import RemoteDevice
from ...core.exception import CommandError, DeviceNotReadyError


_HEADER_STRUCT: Final = struct.Struct("!BHx")  # uint8 id | uint16 len | pad
_HEADER_SIZE: Final = _HEADER_STRUCT.size

_STATE_STRUCT: Final = struct.Struct(
    "!I"  # timestamp [ms]                             –   4 B
    "16d16d"  # O_T_EE, O_T_EE_d                           – 256 B
    "7d7d7d7d7d"  # q, q_d, dq, dq_d, tau_ext_hat_filt         – 280 B
    "6d6d"  # O_F_ext_hat_K, K_F_ext_hat_K               –  96 B
)
_STATE_SIZE: Final = _STATE_STRUCT.size


class ControlMode(IntEnum):
    """Control modes supported by the robot."""

    CARTESIAN_POSITION = 0
    CARTESIAN_VELOCITY = 1
    JOINT_POSITION = 2
    JOINT_VELOCITY = 3
    HUMAN_MODE = 4


@dataclass(frozen=True)
class FrankaArmState:
    """The robot state."""

    timestamp_ms: int

    O_T_EE: Tuple[float, ...]  # actual end-effector pose (4 × 4 matrix, row major)
    O_T_EE_d: Tuple[float, ...]  # set-point pose

    q: Tuple[float, ...]  # measured joint positions [rad]
    q_d: Tuple[float, ...]  # set-point joint positions [rad]
    dq: Tuple[float, ...]  # measured joint velocities [rad s⁻¹]
    dq_d: Tuple[float, ...]  # set-point joint velocities [rad s⁻¹]

    tau_ext_hat_filtered: Tuple[float, ...]  # external torque estimate [N m]
    O_F_ext_hat_K: Tuple[float, ...]  # Cartesian force estimate [N, N m]
    K_F_ext_hat_K: Tuple[float, ...]  # desired wrench [N, N m]


class RemoteFranka(RemoteDevice):
    """
    RemoteFranka class for controlling a Franka Emika robot arm.

    This class implements the `RemoteDevice` interface to provide specific
    functionality for interacting with a Franka robot via a ZMQ-based
    server. It handles sending commands and receiving state updates.

    The typical lifecycle is:
    1. Initialize `RemoteFranka(device_addr, device_port)`.
    2. Call `connect()` to establish the connection to the server.
    3. Interact with the robot using methods like `get_state()`, `start_control()`, etc.
    4. Call `disconnect()` to close the connection and clean up resources.

    Attributes:
        device_addr (str): IP address of the Franka robot control server.
        device_port (int): Port number for the Franka robot control server.
        _ctx (zmq.Context): ZMQ context for managing sockets.
        _sock_req (Optional[zmq.Socket]): ZMQ REQ socket for request-reply communication.
        _state_buffer (Deque[FrankaArmState]): Buffer for storing received robot states.
        _sub_sock (Optional[zmq.Socket]): ZMQ SUB socket for subscribing to state updates.
        _sub_thread (Optional[threading.Thread]): Thread for listening to state updates.
        _listen_flag (threading.Event): Event to control the state listener thread.
    """

    def __init__(self, device_addr: str, device_port: int) -> None:
        """
        Initializes the RemoteFranka instance but does not connect.

        Args:
            device_addr: The IP address of the Franka robot control server.
            device_port: The port number for the Franka robot control server.
        """
        super().__init__(device_addr, device_port)
        # self.device_addr and self.device_port are available from superclass
        self._ctx: zmq.Context = zmq.Context.instance()
        self._sock_req: Optional[zmq.Socket] = None # Initialized as None

        self._state_buffer: Deque[FrankaArmState] = deque()
        self._sub_sock: Optional[zmq.Socket] = None
        self._sub_thread: Optional[threading.Thread] = None
        self._listen_flag: threading.Event = threading.Event()

    def connect(self) -> None:
        """
        Connects to the Franka robot's REQ/REP server.
        If already connected, it will try to close the existing connection first.
        """
        if self._sock_req is not None and not self._sock_req.closed:
            try:
                self._sock_req.close()
            except zmq.ZMQError:  # ZMQErrors can occur if socket is already closed
                pass
        self._sock_req = self._ctx.socket(zmq.REQ)
        # TODO: Consider adding timeouts for connect
        self._sock_req.connect(f"tcp://{self.device_addr}:{self.device_port}")

    def disconnect(self) -> None:
        """
        Disconnects from the Franka robot, stops the state listener,
        and cleans up socket resources.
        """
        self.stop_state_listener()
        if self._sock_req and not self._sock_req.closed:
            try:
                self._sock_req.close()
            except zmq.ZMQError:
                pass  # Socket might already be closed
        self._sock_req = None # Ensure it's marked as None after closing
        # Note: self._ctx.term() is usually called globally upon application exit,
        # not per device instance, to avoid interfering with other ZMQ users.

    def get_status(self) -> ControlMode:
        """
        Retrieves the current control mode of the robot.
        This method effectively wraps `query_state` for semantic clarity
        as per `RemoteDevice` interface.

        Returns:
            The current `ControlMode` of the robot.

        Raises:
            DeviceNotReadyError: If the robot is not connected.
            CommandError: If the server returns an error or communication fails.
        """
        # The check for connection is handled by query_state -> _send/_recv_expect
        return self.query_state()

    def get_state(self) -> FrankaArmState:
        """
        Requests and returns a single, most recent state sample from the robot.

        Returns:
            A `FrankaArmState` object representing the robot's current state.

        Raises:
            DeviceNotReadyError: If the robot is not connected.
            CommandError: If the server returns an error or communication fails.
        """
        # Connection check is handled by _send/_recv_expect
        self._send(MsgID.GET_STATE_REQ, b"")
        payload = self._recv_expect(MsgID.GET_STATE_RESP)
        return self._decode_state(payload)

    def query_state(self) -> ControlMode:
        """
        Queries the robot server for its currently active control mode.

        Returns:
            The current `ControlMode` reported by the server.

        Raises:
            DeviceNotReadyError: If the robot is not connected.
            CommandError: If the server returns an error or communication fails.
        """
        # Connection check is handled by _send/_recv_expect
        self._send(MsgID.QUERY_STATE_REQ, b"")
        payload = self._recv_expect(MsgID.QUERY_STATE_RESP)
        return ControlMode(payload[0])

    def get_sub_port(self) -> int:
        """
        Asks the robot server on which TCP port it publishes state updates
        for the ZMQ SUB socket.

        Returns:
            The port number for the state update publisher.

        Raises:
            DeviceNotReadyError: If the robot is not connected.
            CommandError: If the server returns an error, communication fails,
                          or the payload format is incorrect.
        """
        # Connection check is handled by _send/_recv_expect
        self._send(MsgID.GET_SUB_PORT_REQ, b"")
        payload = self._recv_expect(MsgID.GET_SUB_PORT_RESP)

        if len(payload) != 2:
            raise CommandError("SUB-port payload must be exactly 2 bytes")

        (port,) = struct.unpack("!H", payload)
        return port

    def start_control(
        self,
        mode: ControlMode,
        *,
        controller_ip: Optional[str] = None,
        controller_port: Optional[int] = None,
        subscribe_server: bool = False,
    ) -> None:
        """
        Switches the robot into the specified control `mode`.

        For modes requiring external control (e.g., JOINT_POSITION, CARTESIAN_VELOCITY),
        `controller_ip` and `controller_port` must be provided to inform the robot
        server where the external controller's SUB socket is listening.

        If `subscribe_server` is True, this method will also attempt to start
        the background state listener thread after successfully switching the mode.

        Args:
            mode: The `ControlMode` to switch the robot into.
            controller_ip: IP address of the machine running this client's SUB socket
                           (needed for non-human modes).
            controller_port: Port number of this client's SUB socket
                             (needed for non-human modes).
            subscribe_server: If True, starts the state listener after changing mode.

        Raises:
            DeviceNotReadyError: If the robot is not connected.
            ValueError: If required parameters for the control mode are missing or invalid.
            CommandError: If the server fails to start the control mode or returns an error.
        """
        # Connection check is handled by _send/_recv_expect
        payload = bytearray([mode.value])

        if mode != ControlMode.HUMAN_MODE:
            if controller_ip is None or controller_port is None:
                raise ValueError("Controller_ip and Controller_port required")

            try:
                payload += socket.inet_aton(controller_ip)
            except OSError as exc:
                raise ValueError(f"Invalid IPv4 address: {controller_ip}") from exc

            payload += struct.pack("!H", controller_port)

        self._send(MsgID.START_CONTROL_REQ, payload)
        status = self._recv_expect(MsgID.START_CONTROL_RESP)[0]
        if status != 0:
            raise CommandError(f"START_CONTROL failed (status={status})")
        if subscribe_server:
            self.start_state_listener()

    def _send(self, msg_id: MsgID, payload: bytes) -> None:
        """
        Serializes `msg_id` + `payload` and sends it to the server via REQ socket.
        Internal method.

        Raises:
            DeviceNotReadyError: If the REQ socket is not connected.
        """
        if not self._sock_req or self._sock_req.closed:
            raise DeviceNotReadyError("Not connected. Call connect() before sending commands.")
        self._sock_req.send(_HEADER_STRUCT.pack(msg_id, len(payload)) + payload)

    def _recv_expect(self, expected_id: MsgID) -> bytes:
        """
        Receives one frame from the REQ socket and validates it against `expected_id`.
        Internal method.

        Returns:
            The payload of the received message.

        Raises:
            DeviceNotReadyError: If the REQ socket is not connected.
            CommandError: On communication errors (frame too short, length mismatch,
                          server error response, or unexpected message ID).
        """
        if not self._sock_req or self._sock_req.closed:
            raise DeviceNotReadyError("Not connected. Call connect() before receiving commands.")
        # TODO: Consider adding timeout for recv
        raw = self._sock_req.recv()
        if len(raw) < _HEADER_SIZE:
            raise CommandError("Frame too short")

        msg_id, length = _HEADER_STRUCT.unpack_from(raw, 0)
        payload = raw[_HEADER_SIZE:]

        if len(payload) != length:
            raise CommandError(f"Payload length mismatch ({len(payload)} ≠ {length})")

        if msg_id == MsgID.ERROR:
            code = payload[0] if payload else 0xFF
            raise CommandError(f"Server returned error code 0x{code:02X}")

        if msg_id != expected_id:
            raise CommandError(
                f"Unexpected Msg-ID 0x{msg_id:02X} (expected 0x{expected_id:02X})"
            )

        return payload

    def start_state_listener(
        self,
        *,
        port: Optional[int] = None,
        buffer_size: int = 2048,
    ) -> None:
        """
        Starts a background thread that subscribes to state updates from the robot server.

        The state updates are stored in an internal buffer (`self._state_buffer`).
        If `port` is not provided, it will query the server for the SUB port first.

        Args:
            port: The TCP port on the server publishing state updates. If None,
                  `get_sub_port()` will be called.
            buffer_size: Maximum number of state samples to store in the buffer.

        Raises:
            DeviceNotReadyError: If attempting to query SUB port while not connected.
            CommandError: If `get_sub_port()` fails.
            zmq.ZMQError: If socket connection for SUB socket fails.
        """
        if self._sub_thread and self._sub_thread.is_alive():
            # TODO: Consider logging that listener is already running
            return

        # If port is not given, we must be connected to query it.
        # get_sub_port() will raise DeviceNotReadyError if not connected.
        if port is None:
            port = self.get_sub_port() # This can raise DeviceNotReadyError

        self._state_buffer = deque(maxlen=buffer_size)
        sock: zmq.Socket = self._ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        # Use self.device_addr from the parent class
        sock.connect(f"tcp://{self.device_addr}:{port}")
        self._sub_sock = sock

        self._listen_flag.set()

        def _worker() -> None:
            # TODO: Add error handling for sock.recv() itself, e.g., if socket closes unexpectedly
            while self._listen_flag.is_set() and self._sub_sock and not self._sub_sock.closed:
                try:
                    raw = self._sub_sock.recv(flags=zmq.NOBLOCK) # Use NOBLOCK to allow thread to exit
                    state = self._decode_state(raw)
                    self._state_buffer.append(state)
                except zmq.Again: # No message received, try again
                    time.sleep(0.001) # Small sleep to prevent busy-waiting if NOBLOCK is used
                    continue
                except zmq.ZMQError: # Socket error (e.g. closed by disconnect)
                    break # Exit worker thread
                except CommandError: # Error decoding state
                    # TODO: Log this error
                    continue
            # TODO: Log worker thread exit

        self._sub_thread = threading.Thread(target=_worker, daemon=True)
        self._sub_thread.start()
        # TODO: Log state listener thread started

    def stop_state_listener(self) -> None:
        """
        Stops the background state listener thread and closes the SUB socket.
        """
        if not self._sub_thread or not self._sub_thread.is_alive():
            return # Nothing to do if thread is not running

        self._listen_flag.clear()
        self._sub_thread.join(timeout=1.0) # Wait for the thread to finish
        if self._sub_thread.is_alive():
            # TODO: Log a warning if thread doesn't join
            pass
        self._sub_thread = None

        if self._sub_sock and not self._sub_sock.closed:
            try:
                self._sub_sock.close()
            except zmq.ZMQError:
                pass # Socket might already be closed
        self._sub_sock = None
        # TODO: Log state listener stopped

    def get_state_buffer(self) -> List[FrankaArmState]:
        """
        Returns a *copy* of the internal state buffer, containing recent robot states
        collected by the state listener.
        """
        return list(self._state_buffer)

    def latest_state(self) -> Optional[FrankaArmState]:
        """
        Returns the most recent `FrankaArmState` from the state buffer,
        or `None` if the buffer is empty.
        """
        return self._state_buffer[-1] if self._state_buffer else None

    @staticmethod
    def _decode_state(buf: bytes) -> FrankaArmState:
        """
        Converts a raw byte buffer into a `FrankaArmState` instance.
        Internal static method.

        Raises:
            CommandError: If the buffer size does not match the expected state size.
        """
        if len(buf) != _STATE_SIZE:
            raise CommandError(f"RobotState payload size mismatch. Expected {_STATE_SIZE}, got {len(buf)}")
        values = _STATE_STRUCT.unpack(buf)
        i = 0

        ts = values[i]
        i += 1
        O_T_EE = values[i : i + 16]
        i += 16
        O_T_EE_d = values[i : i + 16]
        i += 16
        q = values[i : i + 7]
        i += 7
        q_d = values[i : i + 7]
        i += 7
        dq = values[i : i + 7]
        i += 7
        dq_d = values[i : i + 7]
        i += 7
        tau_ext_hat = values[i : i + 7]
        i += 7
        O_F_ext_hat_K = values[i : i + 6]
        i += 6
        K_F_ext_hat_K = values[i : i + 6]

        return FrankaArmState(
            timestamp_ms=ts,
            O_T_EE=O_T_EE,
            O_T_EE_d=O_T_EE_d,
            q=q,
            q_d=q_d,
            dq=dq,
            dq_d=dq_d,
            tau_ext_hat_filtered=tau_ext_hat,
            O_F_ext_hat_K=O_F_ext_hat_K,
            K_F_ext_hat_K=K_F_ext_hat_K,
        )
