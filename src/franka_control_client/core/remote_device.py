"""
Core module for remote device control.

This module contains the abstract base class RemoteDevice that defines
the common interface for all remote devices (robots, cameras, etc.).
"""

import struct
import threading
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from typing import Deque, Final, List, Optional

import zmq

from franka_control_client.core.exception import CommandError

from .message import MsgID


@dataclass(frozen=True)
class State:
    """State of the remote device."""

    timestamp_ms: int


class RemoteDevice(ABC):
    """
    Abstract base class for all remote devices.

    This class defines the common interface that all remote devices
    (robots, cameras, sensors, etc.) must implement.

    Attributes:
        device_addr (str): The address of the remote device.
        device_port (int): The port for communicating with the remote device.
    """

    _HEADER_STRUCT: Final = struct.Struct("!BHx")  # uint8 id | uint16 len | pad
    _HEADER_SIZE: Final = _HEADER_STRUCT.size

    def __init__(self, device_addr: str, device_port: int):
        """
        Initialize the remote device.

        Args:
            device_addr (str): The address of the remote device.
            device_port (int): The port for communicating with the remote device.
        """
        self._device_addr = device_addr
        self._device_port = device_port
        self._ctx: zmq.Context = zmq.Context.instance()

        self._sock_req: Optional[zmq.Socket] = None
        self._state_buffer: Deque[State] = deque()
        self._sub_sock: Optional[zmq.Socket] = None
        self._sub_thread: Optional[threading.Thread] = None
        self._listen_flag: threading.Event = threading.Event()

    def connect(self) -> None:
        """
        Connects to the server.
        """
        if self._sock_req is not None and not self._sock_req.closed:
            try:
                self._sock_req.close()
            except zmq.ZMQError:
                pass
        self._sock_req = self._ctx.socket(zmq.REQ)
        if self._sock_req is None:
            raise ConnectionError
        self._sock_req.connect(f"tcp://{self._device_addr}:{self._device_port}")

    def disconnect(self) -> None:
        if self._sock_req and not self._sock_req.closed:
            try:
                self._sock_req.close()
            except zmq.ZMQError:
                pass
        self._sock_req = None

    def get_state_buffer(self) -> List[State]:
        return list(self._state_buffer)

    def latest_state(self) -> Optional[State]:
        return self._state_buffer[-1] if self._state_buffer else None

    def _send(self, msg_id: MsgID, payload: bytes) -> None:
        """Serialize *msg_id* + *payload* and send it to the Server."""
        if self._sock_req is None:
            raise ConnectionError
        self._sock_req.send(self._HEADER_STRUCT.pack(msg_id, len(payload)) + payload)

    def get_sub_port(self) -> int:
        """Ask the Server on which TCP port it publishes state updates."""
        self._send(MsgID.GET_SUB_PORT_REQ, b"")
        payload = self._recv_expect(MsgID.GET_SUB_PORT_RESP)

        if len(payload) != 2:
            raise CommandError("SUB-port payload must be exactly 2 bytes")

        (port,) = struct.unpack("!H", payload)
        return port

    def _recv_expect(self, expected_id: MsgID) -> bytes:
        """
        Receive one frame and validate it against *expected* MsgID.

        Raises:
            CommandError: On checksum/length mismatch or if the controller reports an error.
        """
        if self._sock_req is None:
            raise ConnectionError
        raw = self._sock_req.recv()
        if len(raw) < self._HEADER_SIZE:
            raise CommandError("Frame too short")

        msg_id, length = self._HEADER_STRUCT.unpack_from(raw, 0)
        payload = raw[self._HEADER_SIZE :]

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
        topic: bytes = b"",
    ) -> None:
        if self._sock_req is None:
            raise ConnectionError
        if self._sub_thread and self._sub_thread.is_alive():
            return

        if port is None:
            port = self.get_sub_port()

        self._state_buffer = deque(maxlen=buffer_size)
        sock: zmq.Socket = self._ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, topic)
        sock.connect(f"tcp://{self._device_addr}:{port}")
        self._sub_sock = sock

        self._listen_flag.set()

        def _worker() -> None:
            while self._listen_flag.is_set():
                _topic, payload = sock.recv_multipart()
                try:
                    state = self._decode_state(payload)
                    self._state_buffer.append(state)
                except zmq.ZMQError:
                    break
                except CommandError:
                    continue

        self._sub_thread = threading.Thread(target=_worker, daemon=True)
        self._sub_thread.start()

    def stop_state_listener(self) -> None:
        if not self._sub_thread or not self._sub_thread.is_alive():
            return
        self._listen_flag.clear()
        self._sub_thread.join(timeout=1.0)
        self._sub_thread = None
        if self._sub_sock and not self._sub_sock.closed:
            try:
                self._sub_sock.close()
            except zmq.ZMQError:
                pass
            self._sub_sock = None

    @staticmethod
    @abstractmethod
    def _decode_state(buf: bytes) -> State:
        pass
