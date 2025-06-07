from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple, Any

from ...core.remote_device import RemoteDevice
from ...core.exception import DeviceNotReadyError # Optional: for more specific errors

@dataclass
class CameraState:
    """
    Represents the state of a camera device.

    Attributes:
        is_recording: True if the camera is currently recording, False otherwise.
        resolution: Current resolution of the camera as a (width, height) tuple.
        frame_rate: Current frame rate of the camera in frames per second.
        current_zoom_level: Current zoom level of the camera.
    """
    is_recording: bool = False
    resolution: Tuple[int, int] = (1920, 1080)
    frame_rate: float = 30.0
    current_zoom_level: float = 1.0

class CameraDevice(RemoteDevice):
    """
    Example implementation of a RemoteDevice for a generic camera.
    This is a mock implementation for demonstration purposes.
    """

    def __init__(self, device_addr: str, device_port: int):
        super().__init__(device_addr, device_port)
        self._is_connected: bool = False
        self._current_state: CameraState = CameraState()
        print(f"CameraDevice initialized for {self.device_addr}:{self.device_port}")

    def connect(self) -> None:
        """Connects to the camera device."""
        if self._is_connected:
            print(f"Camera {self.device_addr} already connected.")
            return
        print(f"Connecting to camera at {self.device_addr}:{self.device_port}...")
        # Simulate connection
        self._is_connected = True
        print(f"Camera {self.device_addr} connected successfully.")

    def disconnect(self) -> None:
        """Disconnects from the camera device."""
        if not self._is_connected:
            print(f"Camera {self.device_addr} already disconnected.")
            return
        print(f"Disconnecting from camera {self.device_addr}...")
        # Simulate disconnection
        self._is_connected = False
        print(f"Camera {self.device_addr} disconnected.")

    def get_status(self) -> str:
        """
        Returns the current status of the camera.

        Returns the current status of the camera.

        Raises:
            DeviceNotReadyError: If the camera is not connected.
        """
        if not self._is_connected:
            raise DeviceNotReadyError(f"Camera {self.device_addr}:{self.device_port} not connected.")
        return "Connected and Idle" if not self._current_state.is_recording else "Connected and Recording"

    def get_state(self) -> CameraState:
        """
        Returns the current state of the camera.

        Raises:
            DeviceNotReadyError: If the camera is not connected.
        """
        if not self._is_connected:
            raise DeviceNotReadyError(f"Camera {self.device_addr}:{self.device_port} not connected.")
        return self._current_state

    # Example camera-specific methods
    def start_recording(self) -> None:
        if not self._is_connected:
            raise DeviceNotReadyError(f"Camera {self.device_addr} not connected. Cannot start recording.")
        if self._current_state.is_recording:
            print("Camera is already recording.")
            return
        self._current_state.is_recording = True
        print(f"Camera {self.device_addr} started recording.")

    def stop_recording(self) -> None:
        """Stops recording if the camera is connected and currently recording."""
        if not self._is_connected:
            # Depending on desired behavior, could raise DeviceNotReadyError or just log
            print(f"Warning: Camera {self.device_addr}:{self.device_port} not connected. Cannot stop recording.")
            return
        if not self._current_state.is_recording:
            print("Camera is not currently recording.")
            return
        self._current_state.is_recording = False
        print(f"Camera {self.device_addr} stopped recording.")

    def set_resolution(self, resolution: Tuple[int, int]) -> None:
        if not self._is_connected:
            raise DeviceNotReadyError(f"Camera {self.device_addr} not connected. Cannot set resolution.")
        print(f"Camera {self.device_addr} setting resolution to {resolution}.")
        self._current_state.resolution = resolution
