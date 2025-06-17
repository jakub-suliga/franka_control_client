import struct
import unittest
from collections import deque
from typing import Deque, List, Optional

from franka_control_client.device.gripper import gripper as g_mod
from franka_control_client.core.exception import CommandError

_HEADER_STRUCT = g_mod._HEADER_STRUCT
_STATE_STRUCT = g_mod._STATE_STRUCT
_STATE_SIZE = g_mod._STATE_SIZE
MsgID = g_mod.MsgID
RemoteFrankaGripper = g_mod.RemoteFrankaGripper


class _FakeSocket:
    """Fake socket used for unit-testing."""

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


def _make_frame(msg_id: int, payload: bytes = b"") -> bytes:
    """Return header+payload frame complying with the custom protocol."""
    return _HEADER_STRUCT.pack(msg_id, len(payload)) + payload


def _instantiate_g(fake_socket: _FakeSocket) -> RemoteFrankaGripper:
    g: RemoteFrankaGripper = RemoteFrankaGripper.__new__(RemoteFrankaGripper)
    g._sock_req = fake_socket  # type: ignore[attr-defined]
    g._device_addr = "127.0.0.1"  # type: ignore[attr-defined]
    g._ctx = None  # type: ignore[attr-defined]
    g._state_buffer = deque()  # type: ignore[attr-defined]
    g._sub_sock = None  # type: ignore[attr-defined]
    g._sub_thread = None  # type: ignore[attr-defined]
    g._listen_flag = None  # type: ignore[attr-defined]
    return g


class TestRemoteFrankaGripper(unittest.TestCase):
    def test_recv_expect_success(self) -> None:
        frame = _make_frame(MsgID.GET_STATE_RESP, b"")
        fake_sock = _FakeSocket([frame])
        g = _instantiate_g(fake_sock)

        payload = g._recv_expect(MsgID.GET_STATE_RESP)
        self.assertEqual(payload, b"")

    def test_recv_expect_length_mismatch_raises(self) -> None:
        payload = b"x"
        frame = _HEADER_STRUCT.pack(MsgID.GET_STATE_RESP, 2) + payload
        fake_sock = _FakeSocket([frame])
        g = _instantiate_g(fake_sock)

        with self.assertRaises(CommandError):
            g._recv_expect(MsgID.GET_STATE_RESP)


class TestRemoteFrankaGripperAPI(unittest.TestCase):
    @staticmethod
    def _dummy_state_bytes() -> bytes:
        values = [
            0.1,  # width
            0.2,  # max_width
            True,  # is_grasped
            36.5,  # temperature
        ]
        return _STATE_STRUCT.pack(*values)

    def test_get_state_decodes_payload_and_sends_request(self) -> None:
        state_bytes = self._dummy_state_bytes()
        resp_frame = _make_frame(MsgID.GET_STATE_RESP, state_bytes)
        fake_sock = _FakeSocket([resp_frame])
        g = _instantiate_g(fake_sock)

        state = g.get_state()

        self.assertAlmostEqual(state.width, 0.1)
        self.assertTrue(state.is_grasped)
        self.assertEqual(len(fake_sock.sent), 1)

        expected_header = _HEADER_STRUCT.pack(MsgID.GET_STATE_REQ, 0)
        self.assertEqual(fake_sock.sent[0], expected_header)


if __name__ == "__main__":
    unittest.main()
