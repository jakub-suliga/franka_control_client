# tests/core/test_remote_device_base.py
import unittest
from franka_control_client.core.exception import DeviceNotReadyError

class RemoteDeviceBaseTestMixin:
    """
    A mixin class for testing implementations of the RemoteDevice interface.

    This mixin provides a suite of common tests to ensure that a RemoteDevice
    subclass correctly implements the basic interface requirements, such as
    initialization, connect/disconnect semantics, and behavior of core methods
    like get_state() and get_status() both before and after connection.

    ---------------------------------------------------------------------------
    How to use this mixin for a new RemoteDevice subclass (e.g., MyNewDevice):
    ---------------------------------------------------------------------------

    1. Create your test file:
       Typically, this would be `tests/device/mynewdevice/test_my_new_device.py`.

    2. Import necessary modules:
       ```python
       import unittest
       from unittest.mock import patch # If your device needs mocks
       # Adjust path based on your test runner's discovery root
       from tests.core.test_remote_device_base import RemoteDeviceBaseTestMixin
       from franka_control_client.device.mynewdevice.my_new_device import MyNewDevice
       from franka_control_client.core.exception import DeviceNotReadyError # If needed for specific tests
       ```

    3. Create your test class:
       It must inherit from `RemoteDeviceBaseTestMixin` (usually first) AND
       `unittest.TestCase`.
       ```python
       class TestMyNewDevice(RemoteDeviceBaseTestMixin, unittest.TestCase):
           # ...
       ```

    4. Implement the `setUp` method:
       This is where you configure the mixin and prepare your device instance.
       ```python
       def setUp(self):
           # a. Handle Mocks (if any):
           #    If your MyNewDevice interacts with external systems (e.g., network,
           #    hardware libraries like ZMQ for RemoteFranka), you MUST mock these
           #    interactions to ensure your unit tests are isolated and repeatable.
           #    Example using unittest.mock.patch:
           #    self.zmq_context_patcher = patch('zmq.Context')
           #    self.mock_zmq_context = self.zmq_context_patcher.start()
           #    self.mock_zmq_socket = MagicMock()
           #    self.mock_zmq_context.instance.return_value.socket.return_value = self.mock_zmq_socket

           # b. Define `device_class`:
           #    This tells the mixin which class to test.
           self.device_class = MyNewDevice

           # c. Define `device_kwargs`:
           #    These are the keyword arguments needed to initialize your device.
           self.device_kwargs = {
               'device_addr': "test_addr_new_device",
               'device_port': 5678,
               # Add any other arguments MyNewDevice.__init__ requires
           }

           # d. Instantiate `self.device`:
           #    The mixin expects `self.device` to be an instance of `self.device_class`.
           self.device = self.device_class(**self.device_kwargs)

           # e. (Optional) Add a `tearDown` method for cleanup:
           #    Especially important if you started patchers or if the device
           #    needs explicit disconnection to free resources after each test.
           # def tearDown(self):
           #     self.zmq_context_patcher.stop() # Example for patcher
           #     if self.device and hasattr(self.device, '_sock_req') and self.device._sock_req:
           #         # Or use relevant attribute/method like self.device._is_connected
           #         if not self.device._sock_req.closed: # Check if socket object exists and is not closed
           #             with patch.object(self.device, 'stop_state_listener', MagicMock()): # Mock parts of disconnect if needed
           #                 self.device.disconnect()
       ```
       (Note: The `RemoteFranka` tests use `@patch` decorators on `setUp` or test methods
       for ZMQ, which is also a valid way to handle mocks.)


    5. Understand Mixin Tests:
       The mixin will automatically run several tests against `self.device`, including:
       - `test_interface_initialization`: Checks basic setup and `device_addr`/`device_port`.
       - `test_interface_connect_disconnect`: Checks if `connect()` and `disconnect()` run.
       - `test_interface_get_status_after_connect`: Checks `get_status()` post-connection.
       - `test_interface_get_state_after_connect`: Checks `get_state()` post-connection.
       - `test_interface_core_methods_raise_not_connected`: Checks if `get_state()` and
         `get_status()` raise `DeviceNotReadyError` before connection. If your device
         intentionally behaves differently (e.g., a mock returning default state),
         you might need to skip or override this specific test in your subclass
         using `@unittest.skip` or by redefining the test method.

    6. Add Device-Specific Tests:
       The mixin only tests the common `RemoteDevice` interface. You MUST add
       custom test methods in your `TestMyNewDevice` class to cover:
       - Any unique public methods of `MyNewDevice`.
       - Specific return values or side effects of interface methods (e.g., what
         `get_status()` returns in different scenarios *after* connection).
       - Detailed behavior of `connect` and `disconnect` if they involve more than
         just setting a flag (e.g., ZMQ socket interactions for `RemoteFranka`).
       - Behavior of any other methods specific to `MyNewDevice`.
       Example:
       ```python
       def test_my_new_device_specific_feature_active(self):
           self.device.connect() # Assuming it needs connection
           # ... set up conditions for the feature ...
           result = self.device.do_something_special(param="test")
           self.assertEqual(result, "expected_outcome_for_active")
           # ... other assertions ...
           self.device.disconnect()

       def test_my_new_device_specific_method_error_conditions(self):
           # ... test how MyNewDevice's methods handle errors ...
           pass
       ```

    By following these steps, your new device will be automatically tested for
    compliance with the basic `RemoteDevice` interface, allowing you to concentrate
    on testing its unique features and specific implementation details.
    """

    # Attributes that must be set by the inheriting test class's setUp method:
    # device_class = None  # e.g., MyNewDevice (the class itself)
    # device_kwargs = {}   # e.g., {"device_addr": "localhost", "device_port": 1234}
    # device = None        # Instantiated device: e.g., self.device_class(**self.device_kwargs)

    def test_interface_initialization(self):
        """Tests if the device can be initialized using device_class and kwargs."""
        self.assertIsNotNone(self.device_class, "self.device_class must be set by the test subclass's setUp method.")
        self.assertIsNotNone(self.device_kwargs, "self.device_kwargs must be set by the test subclass's setUp method.")
        self.assertIsNotNone(self.device, "self.device must be initialized in the test subclass's setUp method.")

        self.assertIsInstance(self.device, self.device_class,
                              f"self.device (type: {type(self.device)}) should be an instance of self.device_class ({self.device_class}).")

        self.assertTrue(hasattr(self.device, 'device_addr'), "Device instance must have 'device_addr' attribute.")
        self.assertTrue(hasattr(self.device, 'device_port'), "Device instance must have 'device_port' attribute.")
        self.assertEqual(self.device.device_addr, self.device_kwargs.get('device_addr'))
        self.assertEqual(self.device.device_port, self.device_kwargs.get('device_port'))


    def test_interface_connect_disconnect(self):
        """
        Tests the basic connect and disconnect methods of the device interface.
        Relies on connect/disconnect not raising errors for a basic success case.
        Device-specific tests should cover more detailed connection state.
        """
        self.assertIsNotNone(self.device, "self.device must be initialized by the test subclass.")
        try:
            self.device.connect()
        except Exception as e:
            self.fail(f"{self.device_class.__name__}.connect() raised an unexpected exception: {e}")

        # Add a check here if your device has an obvious way to tell it's connected,
        # e.g. if hasattr(self.device, '_is_connected'): self.assertTrue(self.device._is_connected)
        # However, this might be too implementation-specific for a generic mixin.

        try:
            self.device.disconnect()
        except Exception as e:
            self.fail(f"{self.device_class.__name__}.disconnect() raised an unexpected exception: {e}")

    def test_interface_get_status_after_connect(self):
        """Tests get_status after connecting. Expects a non-None return."""
        self.assertIsNotNone(self.device, "self.device must be initialized.")
        self.device.connect()
        try:
            status = self.device.get_status()
            self.assertIsNotNone(status, f"{self.device_class.__name__}.get_status() should return a value, not None, after connection.")
        except Exception as e:
            self.fail(f"{self.device_class.__name__}.get_status() raised an unexpected exception after connect: {e}")
        finally:
            # Ensure disconnect is attempted even if the test fails, to clean up.
            # Individual tests in the subclass should manage their own connect/disconnect
            # if they need finer-grained control or specific pre/post conditions.
            if hasattr(self.device, 'disconnect'): # Check if disconnect method exists
                 self.device.disconnect()


    def test_interface_get_state_after_connect(self):
        """Tests get_state after connecting. Expects a non-None return."""
        self.assertIsNotNone(self.device, "self.device must be initialized.")
        self.device.connect()
        try:
            state = self.device.get_state()
            self.assertIsNotNone(state, f"{self.device_class.__name__}.get_state() should return a value, not None, after connection.")
        except Exception as e:
            self.fail(f"{self.device_class.__name__}.get_state() raised an unexpected exception after connect: {e}")
        finally:
            if hasattr(self.device, 'disconnect'):
                self.device.disconnect()

    def test_interface_core_methods_raise_not_connected(self):
        """
        Tests that core device methods (get_state, get_status) raise
        DeviceNotReadyError if called before connect().

        If a specific device implementation deliberately allows these calls before
        connection (e.g., a mock returning a default state), that device's test
        class should skip or override this test method using `@unittest.skip` or
        by re-implementing it to assert the expected alternative behavior.
        """
        self.assertIsNotNone(self.device, "self.device must be initialized.")
        # Ensure device is in a disconnected state. For most devices, this means
        # ensuring connect() has not been called, or disconnect() has been called if
        # setUp auto-connects (which is not the pattern encouraged by this mixin's setUp).

        # The following assertion messages are made more specific.
        with self.assertRaises(DeviceNotReadyError,
                               msg=f"{self.device_class.__name__}.get_state() should raise DeviceNotReadyError before connect"):
            self.device.get_state()

        with self.assertRaises(DeviceNotReadyError,
                               msg=f"{self.device_class.__name__}.get_status() should raise DeviceNotReadyError before connect"):
            self.device.get_status()

        # Note to developers using the mixin:
        # Subclasses might want to extend this kind of test for more device-specific methods
        # that also require an active connection. For example:
        #
        # def test_my_device_action_raises_not_connected(self):
        #     # Assuming self.device is in a disconnected state
        #     with self.assertRaises(DeviceNotReadyError):
        #         self.device.perform_specific_action()
        #
        # This would be part of the concrete test class for "MyDevice", not this mixin.
