"""
Core module for remote device control.

This module contains the abstract base class RemoteDevice that defines
the common interface for all remote devices (robots, cameras, etc.).
"""

from abc import ABC, abstractmethod


class RemoteDevice(ABC):
    """
    Abstract base class for all remote devices.

    This class defines the common interface that all remote devices
    (robots, cameras, sensors, etc.) must implement.

    Attributes:
        device_addr (str): The address of the remote device.
        device_port (int): The port for communicating with the remote device.
    """

    def __init__(self, device_addr: str, device_port: int):
        """
        Initialize the remote device.

        Args:
            device_addr (str): The address of the remote device.
            device_port (int): The port for communicating with the remote device.
        """
        self._device_addr = device_addr
        self._device_port = device_port

    @abstractmethod
    def connect(self) -> None:
        """Connect to the remote device."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the remote device."""
        pass
