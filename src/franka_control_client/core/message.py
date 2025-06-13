"""
Defines message identifiers used in the communication protocol.
"""

from enum import IntEnum


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
