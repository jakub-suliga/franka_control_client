# tests/device/robot/test_franka.py
import unittest
from unittest.mock import patch, MagicMock, ANY
import struct # For packing simulated server responses
import socket # For socket.inet_aton in payload construction

from franka_control_client.device.robot.franka import RemoteFranka, ControlMode, FrankaArmState
from franka_control_client.core.exception import DeviceNotReadyError, CommandError
from franka_control_client.core.message import MsgID
from tests.core.test_remote_device_base import RemoteDeviceBaseTestMixin

# Dummy header struct for packing message ID and length, simplified
_HEADER_STRUCT_TEST: struct.Struct = struct.Struct("!BHx")


class TestRemoteFranka(RemoteDeviceBaseTestMixin, unittest.TestCase):

    @patch('zmq.Context') # Mock the ZMQ Context
    def setUp(self, MockContext):
        self.mock_context_instance = MockContext.instance()
        self.mock_socket = MagicMock() # This will be returned by context.socket()
        self.mock_context_instance.socket.return_value = self.mock_socket

        self.device_class = RemoteFranka
        self.device_kwargs = {'device_addr': "dummy_addr_franka_test", 'device_port': 12345}
        # self.device is created by this call, and test_interface_initialization will verify it.
        self.device: RemoteFranka = self.device_class(**self.device_kwargs)

    def tearDown(self):
        # Ensure disconnect is called if a test fails mid-connection
        # This helps prevent ZMQ resource warnings in test output if not handled by test
        # Check if _sock_req exists and was initialized (i.e., connect() was called)
        if hasattr(self.device, '_sock_req') and self.device._sock_req is not None:
            if not self.device._sock_req.closed:
                # Mock stop_state_listener as it might involve threads/SUB sockets not mocked here
                with patch.object(self.device, 'stop_state_listener', MagicMock()):
                    self.device.disconnect()

    # test_initialization is covered by RemoteDeviceBaseTestMixin.test_interface_initialization

    def test_franka_specific_connect_disconnect_zmq_behavior(self):
        """ Tests ZMQ-specific connect/disconnect behavior of RemoteFranka. """
        self.device.connect()
        self.mock_context_instance.socket.assert_called_once_with(ANY) # zmq.REQ
        self.mock_socket.connect.assert_called_once_with(f"tcp://{self.device_kwargs['device_addr']}:{self.device_kwargs['device_port']}")
        self.assertIsNotNone(self.device._sock_req)
        self.assertEqual(self.device._sock_req, self.mock_socket)

        self.device._sock_req.closed = False

        with patch.object(self.device, 'stop_state_listener') as mock_stop_listener:
            self.device.disconnect()
            mock_stop_listener.assert_called_once()

        self.mock_socket.close.assert_called_once()
        self.assertIsNone(self.device._sock_req)

    def test_franka_specific_connect_already_connected_recreates_socket(self):
        """ Tests that reconnecting closes the old ZMQ socket and creates a new one. """
        self.device.connect()
        first_mock_socket = self.device._sock_req
        first_mock_socket.closed = False

        new_mock_socket = MagicMock()
        self.mock_context_instance.socket.return_value = new_mock_socket

        self.device.connect() # Connect again

        first_mock_socket.close.assert_called_once()
        # Check that socket() was called again for the new connection
        self.assertEqual(self.mock_context_instance.socket.call_count, 2)
        new_mock_socket.connect.assert_called_once_with(f"tcp://{self.device_kwargs['device_addr']}:{self.device_kwargs['device_port']}")
        self.assertEqual(self.device._sock_req, new_mock_socket)

    # test_interface_core_methods_raise_not_connected (from mixin) covers:
    # - get_state() before connect
    # - get_status() before connect
    #
    # test_operations_raise_when_not_connected from the original TestRemoteFranka also covered:
    # - query_state() (covered by get_status() in mixin)
    # - start_control()
    # - get_sub_port()
    # - _send() / _recv_expect()
    # We'll create a specific test for the remaining ones.

    def test_franka_specific_methods_raise_not_connected(self):
        """ Tests Franka-specific methods that require connection raise DeviceNotReadyError. """
        # self.device is already initialized and not connected from setUp

        with self.assertRaisesRegex(DeviceNotReadyError, "Not connected. Call connect()"):
            self.device.start_control(ControlMode.CARTESIAN_POSITION)

        with self.assertRaisesRegex(DeviceNotReadyError, "Not connected. Call connect()"):
            self.device.get_sub_port()

        # _send and _recv_expect are "protected" but are critical for connection checks.
        # The mixin doesn't test these internal methods directly.
        with self.assertRaisesRegex(DeviceNotReadyError, "Not connected. Call connect() before sending commands."):
            self.device._send(MsgID.GET_STATE_REQ, b"")

        with self.assertRaisesRegex(DeviceNotReadyError, "Not connected. Call connect() before receiving commands."):
            self.device._recv_expect(MsgID.GET_STATE_RESP)


    def test_get_status_connected_specific_franka_behavior(self):
        """ Tests get_status when connected, focusing on Franka's ControlMode. """
        self.device.connect()
        self.device._sock_req.closed = False

        with patch.object(self.device, '_send') as mock_send, \
             patch.object(self.device, '_recv_expect', return_value=bytes([ControlMode.HUMAN_MODE.value])) as mock_recv_expect:

            status = self.device.get_status() # get_status calls query_state

            mock_send.assert_called_once_with(MsgID.QUERY_STATE_REQ, b"")
            mock_recv_expect.assert_called_once_with(MsgID.QUERY_STATE_RESP)
            self.assertEqual(status, ControlMode.HUMAN_MODE)

    def test_get_state_connected_specific_franka_behavior(self):
        """ Tests get_state when connected, focusing on FrankaArmState parsing. """
        self.device.connect()
        self.device._sock_req.closed = False

        dummy_payload_values = [12345] + [0.0]*16 + [1.0]*16 + [2.0]*7 + [3.0]*7 + \
                               [4.0]*7 + [5.0]*7 + [6.0]*7 + [7.0]*6 + [8.0]*6
        state_struct_format = "!I16d16d7d7d7d7d7d6d6d"
        state_struct = struct.Struct(state_struct_format)
        packed_payload = state_struct.pack(*dummy_payload_values)

        with patch.object(self.device, '_send') as mock_send, \
             patch.object(self.device, '_recv_expect', return_value=packed_payload) as mock_recv_expect:

            state = self.device.get_state()

            mock_send.assert_called_once_with(MsgID.GET_STATE_REQ, b"")
            mock_recv_expect.assert_called_once_with(MsgID.GET_STATE_RESP)
            self.assertIsInstance(state, FrankaArmState)
            self.assertEqual(state.timestamp_ms, 12345)
            self.assertEqual(state.O_T_EE_d[0], 1.0)

    def test_start_control_human_mode(self):
        self.device.connect()
        self.device._sock_req.closed = False

        with patch.object(self.device, '_send') as mock_send, \
             patch.object(self.device, '_recv_expect', return_value=b'\x00') as mock_recv_expect:

            self.device.start_control(ControlMode.HUMAN_MODE, subscribe_server=False)

            expected_payload = bytearray([ControlMode.HUMAN_MODE.value])
            mock_send.assert_called_once_with(MsgID.START_CONTROL_REQ, expected_payload)
            mock_recv_expect.assert_called_once_with(MsgID.START_CONTROL_RESP)

    def test_start_control_joint_pos_with_params(self):
        self.device.connect()
        self.device._sock_req.closed = False

        controller_ip = "192.168.1.100"
        controller_port = 9000

        with patch.object(self.device, '_send') as mock_send, \
             patch.object(self.device, '_recv_expect', return_value=b'\x00') as mock_recv_expect:

            self.device.start_control(ControlMode.JOINT_POSITION,
                                     controller_ip=controller_ip,
                                     controller_port=controller_port,
                                     subscribe_server=False)

            expected_payload = bytearray([ControlMode.JOINT_POSITION.value])
            expected_payload += socket.inet_aton(controller_ip)
            expected_payload += struct.pack("!H", controller_port)

            mock_send.assert_called_once_with(MsgID.START_CONTROL_REQ, expected_payload)
            mock_recv_expect.assert_called_once_with(MsgID.START_CONTROL_RESP)

    def test_start_control_missing_params_raises_value_error(self):
        self.device.connect()
        with self.assertRaisesRegex(ValueError, "Controller_ip and Controller_port required"):
            self.device.start_control(ControlMode.JOINT_POSITION)

    def test_recv_expect_server_error(self):
        self.device.connect()
        self.device._sock_req.closed = False

        error_code = 0x01
        error_payload = _HEADER_STRUCT_TEST.pack(MsgID.ERROR.value, 1) + bytes([error_code])
        self.mock_socket.recv.return_value = error_payload

        with self.assertRaisesRegex(CommandError, f"Server returned error code 0x{error_code:02X}"):
            self.device._recv_expect(MsgID.GET_STATE_RESP)

    def test_recv_expect_unexpected_msg_id(self):
        self.device.connect()
        self.device._sock_req.closed = False

        unexpected_id = MsgID.GET_SUB_PORT_RESP.value
        dummy_data = b"somedata"
        response_payload = _HEADER_STRUCT_TEST.pack(unexpected_id, len(dummy_data)) + dummy_data
        self.mock_socket.recv.return_value = response_payload

        expected_id_for_call = MsgID.GET_STATE_RESP
        with self.assertRaisesRegex(CommandError,
                                   f"Unexpected Msg-ID 0x{unexpected_id:02X} \(expected 0x{expected_id_for_call.value:02X}\)"):
            self.device._recv_expect(expected_id_for_call)

if __name__ == '__main__':
    unittest.main()
