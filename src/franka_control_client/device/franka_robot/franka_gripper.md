# RemoteGripper – Protocol Documentation

This README describes the binary protocol for the client franka gripper.

---

## 1 GripperState Payload

Sent both as reply to `GET_STATE_REQ` and periodically via the PUB socket.
Messages are published under the topic `franka_gripper`.

| Offset | Field          | Type       | Description          |
| -----: | -------------- | ---------- | -------------------- |
|      0 | `timestamp_ms` | `uint32`   | timestamp \[ms]      |
|      4 | `width`        | `float64`  | gripper opening \[m] |
|     12 | `max_width`    | `float64`  | maximum width \[m]   |
|     20 | `grasped`      | `bool`     | whether object held  |
|     21 | `temperature`  | `float64`  | finger temperature   |

Total size: **29 B**

Python struct: `!I d d ? d`

---

## 2 Message Identifiers (MsgID)

| Hex    | Symbol               | Direction | Description               |
| ------ | -------------------- | --------- | ------------------------- |
| `0x01` | `GET_STATE_REQ`      | C → S     | request a single state    |
| `0x04` | `GET_SUB_PORT_REQ`   | C → S     | query PUB port number     |
| `0x51` | `GET_STATE_RESP`     | S → C     | contains one state        |
| `0x54` | `GET_SUB_PORT_RESP`  | S → C     | 2 × `uint8` TCP port      |
| `0xFF` | `ERROR`              | S → C     | 1 × error code            |

---
