"""
Defines message identifiers used in the communication protocol.
"""
from enum import IntEnum

# TODO: This enum might need to be made more extensible if a significantly
# larger number of message IDs are expected in the future (e.g., by using a
# base class with a registration mechanism for different device types).
class MsgID(IntEnum):
    """Message identifiers"""

    GET_STATE_REQ = 0x01
    QUERY_STATE_REQ = 0x02
    START_CONTROL_REQ = 0x03
    GET_SUB_PORT_REQ = 0x04

    GET_STATE_RESP = 0x51
    QUERY_STATE_RESP = 0x52
    START_CONTROL_RESP = 0x53
    GET_SUB_PORT_RESP = 0x54

    ERROR = 0xFF
