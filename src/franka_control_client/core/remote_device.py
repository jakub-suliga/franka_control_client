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
        device_info: Information about the device
        connection_config: Configuration for device connection
        status: Current device status
        logger: Logger instance for this device
    """

    def __init__(self, device_addr: str, device_port: int):
        """
        Initialize the remote device.

        Args:
            connection_config: Configuration for connecting to the device
            device_id: Optional device identifier
        """
        pass

    @abstractmethod
    async def connect(self) -> bool:
        """
        Connect to the remote device.

        Returns:
            True if connection successful, False otherwise

        Raises:
            ConnectionError: If connection fails after all retry attempts
        """
        pass

    @abstractmethod
    async def disconnect(self) -> bool:
        """
        Disconnect from the remote device.

        Returns:
            True if disconnection successful, False otherwise
        """
        pass
