import struct
import unittest
from collections import deque
from typing import Deque, List, Optional

from franka_control_client.device.franka_robot import franka_gripper as rg_mod

_HEADER_STRUCT = struct.Struct("!BHx")
_STATE_STRUCT = rg_mod._STATE_STRUCT
MsgID = rg_mod.MsgID
RemoteGripper = rg_mod.RemoteGripper


class _FakeSocket:
    def __init__(self, frames: Optional[List[bytes]] = None) -> None:
        self._frames: Deque[bytes] = deque(frames or [])
        self.sent: list[bytes] = []
        self.closed = False

    def send(self, data: bytes) -> None:
        self.sent.append(data)

    def recv(self, flags: int = 0) -> bytes:
        try:
            return self._frames.popleft()
        except IndexError as exc:
            raise RuntimeError("No more frames queued for FakeSocket.recv()") from exc

    def connect(self, *_a, **_kw) -> None:
        pass

    def setsockopt(self, *_a, **_kw) -> None:
        pass

    def close(self) -> None:
        self.closed = True


def _instantiate_rg(fake_socket: _FakeSocket) -> RemoteGripper:
    rg: RemoteGripper = RemoteGripper.__new__(RemoteGripper)
    rg._sock_req = fake_socket  # type: ignore[attr-defined]
    rg._device_addr = "127.0.0.1"  # type: ignore[attr-defined]
    rg._ctx = None  # type: ignore[attr-defined]
    rg._state_buffer = deque()  # type: ignore[attr-defined]
    rg._sub_sock = None  # type: ignore[attr-defined]
    rg._sub_thread = None  # type: ignore[attr-defined]
    rg._listen_flag = None  # type: ignore[attr-defined]
    return rg


def _make_frame(msg_id: int, payload: bytes = b"") -> bytes:
    return _HEADER_STRUCT.pack(msg_id, len(payload)) + payload


class TestRemoteGripper(unittest.TestCase):
    def _dummy_state_bytes(self) -> bytes:
        return _STATE_STRUCT.pack(123456, 0.05, 0.1, True, 42.0)

    def test_get_state_decodes_payload(self) -> None:
        state_bytes = self._dummy_state_bytes()
        resp_frame = _make_frame(MsgID.GET_STATE_RESP, state_bytes)
        fake_sock = _FakeSocket([resp_frame])
        rg = _instantiate_rg(fake_sock)

        state = rg.get_state()

        self.assertEqual(state.timestamp_ms, 123456)
        self.assertAlmostEqual(state.width, 0.05)
        self.assertTrue(state.grasped)
        self.assertEqual(len(fake_sock.sent), 1)
        expected_header = _HEADER_STRUCT.pack(MsgID.GET_STATE_REQ, 0)
        self.assertEqual(fake_sock.sent[0], expected_header)


if __name__ == "__main__":
    unittest.main()
