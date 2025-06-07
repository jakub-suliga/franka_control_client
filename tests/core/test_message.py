# tests/core/test_message.py
import unittest

from franka_control_client.core.message import MsgID

class TestMessage(unittest.TestCase):
    def test_msgid_import(self):
        self.assertTrue(hasattr(MsgID, 'GET_STATE_REQ'))
        self.assertEqual(MsgID.GET_STATE_REQ.value, 0x01)
        # Add a few more checks for other MsgID members
        self.assertTrue(hasattr(MsgID, 'ERROR'))
        self.assertEqual(MsgID.ERROR.value, 0xFF)
        self.assertTrue(hasattr(MsgID, 'START_CONTROL_RESP'))
        self.assertEqual(MsgID.START_CONTROL_RESP.value, 0x53)

if __name__ == '__main__':
    unittest.main()
