from __future__ import annotations

import socket
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Final, Optional, Tuple

from ...core.exception import CommandError
from ...core.message import MsgID
from ...core.remote_device import RemoteDevice, State

_STATE_STRUCT: Final = struct.Struct(
    "!I"  # timestamp [ms] – 4 B
    "16d16d"  # O_T_EE, O_T_EE_d – 256 B
    "7d7d7d7d7d"  # q, q_d, dq, dq_d, tau_ext_hat_filt – 280 B
    "6d6d"  # O_F_ext_hat_K, K_F_ext_hat_K –  96 B
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
class FrankaArmState(State):
    """The robot state."""

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
    RemoteFranka class for controlling a Franka robot.

    This class extends the RemoteDevice class and provides
    specific functionality for interacting with a Franka robot.
    Attributes:
        device_addr (str): Address of the Franka robot.
        device_port (int): Port for communication with the Franka robot.
    """

    def __init__(self, device_addr: str, device_port: int) -> None:
        """
        Initialize the RemoteFranka instance.

        Args:
            device_addr (str): The IP address of the Franka robot.
            device_port (int): The port number for communication with the Franka robot.
        """
        super().__init__(device_addr, device_port)

    def get_state(self) -> FrankaArmState:
        """Return a single state sample"""
        self._send(MsgID.GET_STATE_REQ, b"")
        payload = self._recv_expect(MsgID.GET_STATE_RESP)
        return self._decode_state(payload)

    def get_control_mode(self) -> ControlMode:
        """Return the currently active control mode."""
        self._send(MsgID.GET_CONTROL_MODE_REQ, b"")
        payload = self._recv_expect(MsgID.GET_CONTROL_MODE_RESP)
        return ControlMode(payload[0])

    def start_control(
        self,
        mode: ControlMode,
        *,
        controller_ip: Optional[str] = None,
        controller_port: Optional[int] = None,
        subscribe_server: bool = False,
    ) -> None:
        """
        Switch the robot into *mode*.

        For any mode **other than** :pyattr:`ControlMode.HUMAN_MODE` you *must*
        provide ``controller_ip`` and ``controller_port`` – these parameters
        tell the robot where your *SUB* socket lives.  If ``subscribe_server``
        is *True* we start a background thread that subscribes to
        state updates.
        """
        payload = bytearray([mode.value])

        if mode != ControlMode.HUMAN_MODE:
            if controller_ip is None or controller_port is None:
                raise ValueError("Controller_ip and Controller_port required")

            try:
                payload += socket.inet_aton(controller_ip)
            except OSError as exc:
                raise ValueError(f"Invalid IPv4 address: {controller_ip}") from exc

            payload += struct.pack("!H", controller_port)

        self._send(MsgID.SET_CONTROL_MODE_REQ, payload)
        status = self._recv_expect(MsgID.SET_CONTROL_MODE_RESP)[0]
        if status != 0:
            raise CommandError(f"START_CONTROL failed (status={status})")
        if subscribe_server:
            self.start_state_listener()

    def start_state_listener(
        self,
        *,
        port: int | None = None,
        buffer_size: int = 2048,
        topic: bytes = b"franka_arm",
    ) -> None:
        return super().start_state_listener(
            port=port,
            buffer_size=buffer_size,
            topic=topic,
        )

    @staticmethod
    def _decode_state(buf: bytes) -> FrankaArmState:
        """Convert the bytes into a :class:`RobotState` instance."""
        if len(buf) != _STATE_SIZE:
            raise CommandError("RobotState payload size mismatch")
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
