# tests/device/camera/test_camera.py
import unittest
from unittest.mock import patch

from franka_control_client.device.camera.camera import CameraDevice, CameraState
from franka_control_client.core.exception import DeviceNotReadyError
# Assuming 'tests' is discoverable or on PYTHONPATH.
# If running `python -m unittest discover tests` from project root, this should work.
# Or, if franka_control_client is installed editable, this structure is also fine.
from tests.core.test_remote_device_base import RemoteDeviceBaseTestMixin

class TestCameraDevice(RemoteDeviceBaseTestMixin, unittest.TestCase):
    def setUp(self):
        # Suppress print statements from the mock camera during tests
        self.patcher = patch('builtins.print')
        self.mock_print = self.patcher.start()

        self.device_class = CameraDevice
        # Using a slightly different addr to ensure these kwargs are used
        self.device_kwargs = {'device_addr': "localhost_cam_mixin", 'device_port': 5678}
        # self.device is created by this call, and test_interface_initialization will verify it.
        self.device: CameraDevice = self.device_class(**self.device_kwargs)


    def tearDown(self):
        self.patcher.stop()
        # Ensure device is disconnected if a test connected it and failed.
        # The mixin tests already handle their own connect/disconnect cycles.
        if self.device._is_connected:
            self.device.disconnect()

    # test_initialization is now covered by RemoteDeviceBaseTestMixin.test_interface_initialization
    # test_connect_disconnect is now covered by RemoteDeviceBaseTestMixin.test_interface_connect_disconnect

    def test_get_status_specific_values_when_connected(self):
        """ Tests specific status values returned by CameraDevice when connected. """
        # test_interface_core_methods_raise_not_connected (from mixin) covers disconnected case
        self.device.connect()
        self.assertEqual(self.device.get_status(), "Connected and Idle")
        self.device.start_recording()
        self.assertEqual(self.device.get_status(), "Connected and Recording")
        self.device.stop_recording()
        self.assertEqual(self.device.get_status(), "Connected and Idle")

    def test_get_state_specific_values_when_connected(self):
        """ Tests specific state values returned by CameraDevice when connected. """
        # test_interface_core_methods_raise_not_connected (from mixin) covers disconnected case
        self.device.connect()
        state = self.device.get_state()
        self.assertIsInstance(state, CameraState)
        self.assertFalse(state.is_recording) # Initial state after connect
        self.assertEqual(state.resolution, (1920,1080)) # Check default resolution from CameraState

    def test_recording_workflow(self):
        # DeviceNotReadyError before connect is tested by mixin for get_state/get_status.
        # start_recording itself raises DeviceNotReadyError if not connected.
        with self.assertRaisesRegex(DeviceNotReadyError, "not connected. Cannot start recording."):
            self.device.start_recording()

        self.device.connect()
        self.device.start_recording()
        self.assertTrue(self.device._current_state.is_recording)
        self.assertTrue(self.device.get_state().is_recording)

        # Test starting again when already recording (should be handled, print expected)
        self.mock_print.reset_mock()
        self.device.start_recording()
        self.mock_print.assert_any_call("Camera is already recording.")

        self.device.stop_recording()
        self.assertFalse(self.device._current_state.is_recording)
        self.assertFalse(self.device.get_state().is_recording)

        # Test stopping again when already stopped (should be handled, print expected)
        self.mock_print.reset_mock()
        self.device.stop_recording()
        self.mock_print.assert_any_call("Camera is not currently recording.")

        # Test stopping when not connected (mock CameraDevice prints warning, doesn't raise for stop_recording)
        self.device.disconnect() # Ensure it's disconnected for this part of the test
        self.mock_print.reset_mock()
        self.device.stop_recording() # This specific method in mock CameraDevice prints a warning
        self.mock_print.assert_any_call(f"Warning: Camera {self.device.device_addr}:{self.device.device_port} not connected. Cannot stop recording.")


    def test_set_resolution(self):
        new_resolution = (1280, 720)
        # DeviceNotReadyError before connect is tested by mixin for get_state/get_status.
        # set_resolution itself raises DeviceNotReadyError if not connected.
        with self.assertRaisesRegex(DeviceNotReadyError, "not connected. Cannot set resolution."):
            self.device.set_resolution(new_resolution)

        self.device.connect()
        self.device.set_resolution(new_resolution)
        self.assertEqual(self.device._current_state.resolution, new_resolution)
        self.assertEqual(self.device.get_state().resolution, new_resolution)

    def test_camera_state_defaults(self):
        state = CameraState()
        self.assertEqual(state.is_recording, False)
        self.assertEqual(state.resolution, (1920, 1080))
        self.assertEqual(state.frame_rate, 30.0)
        self.assertEqual(state.current_zoom_level, 1.0)

if __name__ == '__main__':
    unittest.main()
