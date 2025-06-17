from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from typing import Deque, Final, List, Optional
import threading
import struct

import zmq

from ...core.remote_device import RemoteDevice
from ...core.exception import CommandError

_HEADER_STRUCT: Final = struct.Struct("!BHx")
_HEADER_SIZE: Final = _HEADER_STRUCT.size

_STATE_STRUCT: Final = struct.Struct(
    "!dd?d"  # width, max_width, is_grasped, temperature
)
_STATE_SIZE: Final = _STATE_STRUCT.size


class MsgID(IntEnum):
    """Message identifiers for the gripper."""

    GET_STATE_REQ = 0x01
    GET_SUB_PORT_REQ = 0x02

    GET_STATE_RESP = 0x51
    GET_SUB_PORT_RESP = 0x52

    ERROR = 0xFF


@dataclass(frozen=True)
class FrankaGripperState:
    """State information for the Franka gripper."""

    width: float
    max_width: float
    is_grasped: bool
    temperature: float


class RemoteFrankaGripper(RemoteDevice):
    """Remote client for controlling a Franka gripper."""

    def __init__(self, device_addr: str, device_port: int) -> None:
        super().__init__(device_addr, device_port)
        self._device_addr: str = device_addr
        self._ctx: zmq.Context = zmq.Context.instance()

        self._sock_req: zmq.Socket = self._ctx.socket(zmq.REQ)
        self._sock_req.connect(f"tcp://{device_addr}:{device_port}")

        self._state_buffer: Deque[FrankaGripperState] = deque()
        self._sub_sock: Optional[zmq.Socket] = None
        self._sub_thread: Optional[threading.Thread] = None
        self._listen_flag: threading.Event = threading.Event()

    def close(self) -> None:
        self.stop_state_listener()
        if not self._sock_req.closed:
            self._sock_req.close()

    def get_state(self) -> FrankaGripperState:
        """Return a single state sample."""
        self._send(MsgID.GET_STATE_REQ, b"")
        payload = self._recv_expect(MsgID.GET_STATE_RESP)
        return self._decode_state(payload)

    def get_sub_port(self) -> int:
        """Ask the server on which TCP port it publishes state updates."""
        self._send(MsgID.GET_SUB_PORT_REQ, b"")
        payload = self._recv_expect(MsgID.GET_SUB_PORT_RESP)

        if len(payload) != 2:
            raise CommandError("SUB-port payload must be exactly 2 bytes")

        (port,) = struct.unpack("!H", payload)
        return port

    def _send(self, msg_id: MsgID, payload: bytes) -> None:
        self._sock_req.send(_HEADER_STRUCT.pack(msg_id, len(payload)) + payload)

    def _recv_expect(self, expected_id: MsgID) -> bytes:
        raw = self._sock_req.recv()
        if len(raw) < _HEADER_SIZE:
            raise CommandError("Frame too short")

        msg_id, length = _HEADER_STRUCT.unpack_from(raw, 0)
        payload = raw[_HEADER_SIZE:]

        if len(payload) != length:
            raise CommandError(f"Payload length mismatch ({len(payload)} \u2260 {length})")

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
        if self._sub_thread and self._sub_thread.is_alive():
            return

        if port is None:
            port = self.get_sub_port()

        self._state_buffer = deque(maxlen=buffer_size)
        sock: zmq.Socket = self._ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.connect(f"tcp://{self._device_addr}:{port}")
        self._sub_sock = sock

        self._listen_flag.set()

        def _worker() -> None:
            while self._listen_flag.is_set():
                raw = sock.recv()
                try:
                    state = self._decode_state(raw)
                    self._state_buffer.append(state)
                except CommandError:
                    continue

        self._sub_thread = threading.Thread(target=_worker, daemon=True)
        self._sub_thread.start()

    def stop_state_listener(self) -> None:
        if not self._sub_thread:
            return
        self._listen_flag.clear()
        self._sub_thread.join(timeout=1.0)
        self._sub_thread = None
        if self._sub_sock and not self._sub_sock.closed:
            self._sub_sock.close()
            self._sub_sock = None

    def get_state_buffer(self) -> List[FrankaGripperState]:
        """Return a *copy* of the internal state buffer."""
        return list(self._state_buffer)

    def latest_state(self) -> Optional[FrankaGripperState]:
        """Return the most recent state or *None* if buffer is empty."""
        return self._state_buffer[-1] if self._state_buffer else None

    @staticmethod
    def _decode_state(buf: bytes) -> FrankaGripperState:
        if len(buf) != _STATE_SIZE:
            raise CommandError("GripperState payload size mismatch")
        width, max_width, is_grasped, temperature = _STATE_STRUCT.unpack(buf)
        return FrankaGripperState(
            width=width,
            max_width=max_width,
            is_grasped=is_grasped,
            temperature=temperature,
        )
