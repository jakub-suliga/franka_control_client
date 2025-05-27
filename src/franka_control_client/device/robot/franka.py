from ...core.remote_device import RemoteDevice


class RemoteFranka(RemoteDevice):
    """
    RemoteFranka class for controlling a Franka robot.

    This class extends the RemoteDevice class and provides
    specific functionality for interacting with a Franka robot.
    Attributes:
        device_addr (str): Address of the Franka robot.
        device_port (int): Port for communication with the Franka robot.
    """

    def __init__(self, device_addr: str, device_port: int):
        """
        Initialize the RemoteFranka instance.

        Args:
            device_addr (str): Address of the Franka robot.
            device_port (int): Port for communication with the Franka robot.
        """
        super().__init__(device_addr, device_port)
        self.device_addr = device_addr
        self.device_port = device_port
