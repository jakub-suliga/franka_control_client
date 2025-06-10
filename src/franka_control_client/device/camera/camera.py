from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple, Any

from ...core.remote_device import RemoteDevice
from ...core.exception import DeviceNotReadyError


@dataclass
class CameraState:
    pass


class CameraDevice(RemoteDevice):
    def __init__(self, device_addr: str, device_port: int):
        super().__init__(device_addr, device_port)

    def connect(self) -> None:
        """Connects to the camera device."""
        pass

    def disconnect(self) -> None:
        """Disconnects from the camera device."""
        pass
