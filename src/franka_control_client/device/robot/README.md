# RemoteFranka – Protocol Documentation

This README explains **what** is sent over to the *Franka* robot and **what** the robot answers.

---

## 1 Overview

| Role       | ZMQ socket | Transport                         | Direction         |
| ---------- | ---------- | --------------------------------- | ----------------- |
| **Client** | `zmq.REQ`  | `tcp://<robot_ip>:<command_port>` | Request → Reply   |
| **Server** | `zmq.REP`  | same endpoint                     | Reply             |
| **Server** | `zmq.PUB`  | `tcp://<robot_ip>:<sub_port>`     | State broadcast → |
| **Client** | `zmq.SUB`  | same endpoint                     | receives states   |

* All multi‑byte fields are **big‑endian** (`!` in `struct`).
* Communication is binary and packet‑based.

---

## 2 Frame Layout (Header)

```text
0      1      3
+------+------+------+
| id  |  len | pad  |
+------+------+------+
 1 B   2 B    1 B  (always 0)
```

Python struct: `!BHx` (constant size = 4 B)

* **id** `uint8` → the Message ID
* **len** `uint16` → payload length in bytes (0 – 65 535)
* **pad** `uint8` → always 0 (reserved)

---

## 3 RobotState Payload

Sent both as reply to `GET_STATE_REQ` and periodically via the PUB socket.

| Offset | Field                  | Type / Count   | Description                                             |
| -----: | ---------------------- | -------------- | ------------------------------------------------------- |
|      0 | `timestamp_ms`         | `uint32`       | timestamp \[ms]                                         |
|      4 | `O_T_EE`               | 16 × `float64` | 4×4 homogeneous EE pose                                 |
|    132 | `O_T_EE_d`             | 16 × `float64` | pose set‑point                                          |
|    260 | `q`                    | 7 × `float64`  | joint angles actual \[rad]                              |
|    316 | `q_d`                  | 7 × `float64`  | joint angles target \[rad]                              |
|    372 | `dq`                   | 7 × `float64`  | joint velocities actual \[rad s⁻¹]                      |
|    428 | `dq_d`                 | 7 × `float64`  | joint velocities target \[rad s⁻¹]                      |
|    484 | `tau_ext_hat_filtered` | 7 × `float64`  | filtered external torque estimate \[N m]                |
|    540 | `O_F_ext_hat_K`        | 6 × `float64`  | Cartesian force estimate \[N, N m]                      |
|    588 | `K_F_ext_hat_K`        | 6 × `float64`  | desired wrench \[N, N m]                                |

Total size: **648 B**

Python struct: `!I 16d16d 7d7d7d7d7d 6d6d`

---

## 4 Message Identifiers (MsgID)

| Hex    | Symbol               | Direction | Description                                      |
| ------ | -------------------- | --------- | ------------------------------------------------ |
| `0x01` | `GET_STATE_REQ`      | C → S     | request a single state                           |
| `0x02` | `QUERY_STATE_REQ`    | C → S     | ask for active control mode                      |
| `0x03` | `START_CONTROL_REQ`  | C → S     | switch to desired mode                           |
| `0x04` | `GET_SUB_PORT_REQ`   | C → S     | query PUB port number                            |
| `0x51` | `GET_STATE_RESP`     | S → C     | contains one                                     |
| `0x52` | `QUERY_STATE_RESP`   | S → C     | 1 × `uint8` (see table 5)                        |
| `0x53` | `START_CONTROL_RESP` | S → C     | 1 × `uint8` status (0 = OK)                      |
| `0x54` | `GET_SUB_PORT_RESP`  | S → C     | 2 × `uint8` TCP port (big‑endian)                |
| `0xFF` | `ERROR`              | S → C     | 1 × error code                                   |

---

## 5 ControlMode Constants

| Value | Symbol               | Meaning                                |
| ----: | -------------------- | -------------------------------------- |
|     0 | `CARTESIAN_POSITION` | control end‑effector position          |
|     1 | `CARTESIAN_VELOCITY` | control end‑effector velocity          |
|     2 | `JOINT_POSITION`     | control joint angles                   |
|     3 | `JOINT_VELOCITY`     | control joint velocities               |
|     4 | `HUMAN_MODE`         | free‑floating                          |

---

## 6 Errors & Exceptions

| Error code              | Meaning              | Python exception                                  |
| ----------------------- | -------------------- | ------------------------------------------------- |
| any ≠ 0                 | server‑defined error | `CommandError("Server returned error code 0x..")` |
| header too short        | corrupted packet     | `CommandError("Frame too short")`                 |
| payload length mismatch | likewise             | `CommandError("Payload length mismatch …")`       |
| unexpected `msg_id`     | mismatched reply     | `CommandError("Unexpected Msg‑ID …")`             |

---
