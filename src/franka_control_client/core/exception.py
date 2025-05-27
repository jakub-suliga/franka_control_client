class RemoteDeviceError(Exception):
    """Base exception for remote device operations."""

    pass


class ConnectionError(RemoteDeviceError):
    """Exception raised when connection fails."""

    pass


class DeviceNotReadyError(RemoteDeviceError):
    """Exception raised when device is not ready for operation."""

    pass


class CommandError(RemoteDeviceError):
    """Exception raised when command execution fails."""

    pass
