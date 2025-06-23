# RemoteGripper

This README describes the binary protocol for the remote device.

---

## 1 Overview

| Role       | ZMQ socket | Transport                         | Direction |
| ---------- | ---------- | --------------------------------- | --------- |
| **Client** | `zmq.REQ`  | `tcp://<gripper_ip>:<command_port>` | Request → Reply |
| **Server** | `zmq.REP`  | same endpoint                     | Reply |
| **Server** | `zmq.PUB`  | `tcp://<gripper_ip>:<sub_port>`    | State broadcast → |
| **Client** | `zmq.SUB`  | same endpoint                     | receives states |

* All multi‑byte fields are **big‑endian** (`!` in `struct`).
* Communication is binary and packet‑based.

---

## 2 Frame Layout (Header)

```text
0      1      3
+------+------+------+
| id  |  len | pad  |
+------+------+------+
 1 B   2 B    1 B  (always 0)
```

Python struct: `!BHx` (constant size = 4 B)

* **id** `uint8` → the Message ID
* **len** `uint16` → payload length in bytes (0 – 65 535)
* **pad** `uint8` → always 0 (reserved)

---

## 3 Errors & Exceptions

| Error code              | Meaning              | Python exception                                  |
| ----------------------- | -------------------- | ------------------------------------------------- |
| any ≠ 0                 | server‑defined error | `CommandError("Server returned error code 0x..")` |
| header too short        | corrupted packet     | `CommandError("Frame too short")`                 |
| payload length mismatch | likewise             | `CommandError("Payload length mismatch …")`       |
| unexpected `msg_id`     | mismatched reply     | `CommandError("Unexpected Msg‑ID …")`             |

---
