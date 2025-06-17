from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Final, List, Optional
import struct
import threading

import zmq

from ...core.remote_device import RemoteDevice
from ...core.exception import CommandError
from ...core.message import MsgID

_HEADER_STRUCT: Final = struct.Struct("!BHx")
_HEADER_SIZE: Final = _HEADER_STRUCT.size

_STATE_STRUCT: Final = struct.Struct("!dd?d")  # width, max_width, grasped, temperature
_STATE_SIZE: Final = _STATE_STRUCT.size


@dataclass(frozen=True)
class GripperState:
    """State of the gripper."""

    width: float
    max_width: float
    grasped: bool
    temperature: float


class RemoteGripper(RemoteDevice):
    """Remote client for a gripper device."""

    def __init__(self, device_addr: str, device_port: int) -> None:
        super().__init__(device_addr, device_port)
        self._ctx: zmq.Context = zmq.Context.instance()

        self._sock_req: Optional[zmq.Socket] = None
        self._state_buffer: Deque[GripperState] = deque()
        self._sub_sock: Optional[zmq.Socket] = None
        self._sub_thread: Optional[threading.Thread] = None
        self._listen_flag: threading.Event = threading.Event()

    def connect(self) -> None:
        """
        Connects to the Gripper server.
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

    def get_state(self) -> GripperState:
        """Return a single state sample"""
        self._send(MsgID.GET_STATE_REQ, b"")
        payload = self._recv_expect(MsgID.GET_STATE_RESP)
        return self._decode_state(payload)

    def _send(self, msg_id: MsgID, payload: bytes) -> None:
        if self._sock_req is None:
            raise ConnectionError
        self._sock_req.send(_HEADER_STRUCT.pack(msg_id, len(payload)) + payload)

    def _recv_expect(self, expected_id: MsgID) -> bytes:
        if self._sock_req is None:
            raise ConnectionError
        raw = self._sock_req.recv()
        if len(raw) < _HEADER_SIZE:
            raise CommandError("Frame too short")

        msg_id, length = _HEADER_STRUCT.unpack_from(raw, 0)
        payload = raw[_HEADER_SIZE:]

        if len(payload) != length:
            raise CommandError(
                f"Payload length mismatch ({len(payload)} \u2260 {length})"
            )

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
        if self._sock_req is None:
            raise ConnectionError
        if self._sub_thread and self._sub_thread.is_alive():
            return

        if port is None:
            self._send(MsgID.GET_SUB_PORT_REQ, b"")
            payload = self._recv_expect(MsgID.GET_SUB_PORT_RESP)
            if len(payload) != 2:
                raise CommandError("SUB-port payload must be exactly 2 bytes")
            (port,) = struct.unpack("!H", payload)

        self._state_buffer = deque(maxlen=buffer_size)
        sock: zmq.Socket = self._ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.SUBSCRIBE, b"gripper")
        sock.connect(f"tcp://{self._device_addr}:{port}")
        self._sub_sock = sock

        self._listen_flag.set()

        def _worker() -> None:
            while self._listen_flag.is_set():
                topic, payload = sock.recv_multipart()
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

    def get_state_buffer(self) -> List[GripperState]:
        return list(self._state_buffer)

    def latest_state(self) -> Optional[GripperState]:
        return self._state_buffer[-1] if self._state_buffer else None

    @staticmethod
    def _decode_state(buf: bytes) -> GripperState:
        if len(buf) != _STATE_SIZE:
            raise CommandError("State payload size mismatch")
        width, max_width, grasped, temperature = _STATE_STRUCT.unpack(buf)
        return GripperState(
            width=width,
            max_width=max_width,
            grasped=grasped,
            temperature=temperature,
        )
