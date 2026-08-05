# Agilex Piper — CAN Protocol Datasheet

## Overview

| Parameter | Value |
|-----------|-------|
| Bus | CAN 2.0A, 11-bit arbitration ID |
| Baud rate | 1 Mbps (fixed) |
| Frame data length | 8 bytes |
| Byte order | Big-endian (MSB first) |
| Signed encoding | Two's complement |
| Resolution (joints) | 0.001° (int32) |
| Resolution (position) | 0.001 mm (int32) |
| Resolution (gripper) | 0.001 mm (int32), 0.001 N·m (int16) |

---

## CAN ID Map

| ID | Direction | Name | Frame layout |
|----|-----------|------|--------------|
| `0x150` | PC → Arm | Motion Control 1 | Status/teach/trajectory |
| `0x151` | PC → Arm | Motion Control 2 | Control mode, speed, MIT, install |
| `0x152` | PC → Arm | Cartesian XY | X (int32), Y (int32) |
| `0x153` | PC → Arm | Cartesian Z_RX | Z (int32), RX (int32) |
| `0x154` | PC → Arm | Cartesian RY_RZ | RY (int32), RZ (int32) |
| `0x155` | PC → Arm | Joint Ctrl 12 | Joint 1 (int32), Joint 2 (int32) |
| `0x156` | PC → Arm | Joint Ctrl 34 | Joint 3 (int32), Joint 4 (int32) |
| `0x157` | PC → Arm | Joint Ctrl 56 | Joint 5 (int32), Joint 6 (int32) |
| `0x159` | PC → Arm | Gripper Ctrl | Angle (int32), Effort (uint16), Code, Zero |
| `0x15A` | PC → Arm | MIT Ctrl J1 | Bit-packed pos/vel/kp/kd/t + CRC |
| `0x15B` | PC → Arm | MIT Ctrl J2 | (same) |
| `0x15C` | PC → Arm | MIT Ctrl J3 | (same) |
| `0x15D` | PC → Arm | MIT Ctrl J4 | (same) |
| `0x15E` | PC → Arm | MIT Ctrl J5 | (same) |
| `0x15F` | PC → Arm | MIT Ctrl J6 | (same) |
| `0x2A1` | Arm → PC | Arm Status | ctrl_mode, arm_status, err_code, etc. |
| `0x2A2` | Arm → PC | End Pose XY | X (int32), Y (int32) |
| `0x2A3` | Arm → PC | End Pose Z_RX | Z (int32), RX (int32) |
| `0x2A4` | Arm → PC | End Pose RY_RZ | RY (int32), RZ (int32) |
| `0x2A5` | Arm → PC | Joint Feedback 12 | Joint 1 (int32), Joint 2 (int32) |
| `0x2A6` | Arm → PC | Joint Feedback 34 | Joint 3 (int32), Joint 4 (int32) |
| `0x2A7` | Arm → PC | Joint Feedback 56 | Joint 5 (int32), Joint 6 (int32) |
| `0x2A8` | Arm → PC | Gripper Feedback | Angle (int32), Effort (int16), Status |
| `0x251` | Arm → PC | Motor High-Speed FB J1 | Speed, Current, Position |
| `0x252` | Arm → PC | Motor High-Speed FB J2 | (same) |
| `0x253` | Arm → PC | Motor High-Speed FB J3 | (same) |
| `0x254` | Arm → PC | Motor High-Speed FB J4 | (same) |
| `0x255` | Arm → PC | Motor High-Speed FB J5 | (same) |
| `0x256` | Arm → PC | Motor High-Speed FB J6 | (same) |
| `0x261` | Arm → PC | Motor Low-Speed FB J1 | Voltage, Temp, Status, Bus Current |
| `0x262` | Arm → PC | Motor Low-Speed FB J2 | (same) |
| `0x263` | Arm → PC | Motor Low-Speed FB J3 | (same) |
| `0x264` | Arm → PC | Motor Low-Speed FB J4 | (same) |
| `0x265` | Arm → PC | Motor Low-Speed FB J5 | (same) |
| `0x266` | Arm → PC | Motor Low-Speed FB J6 | (same) |
| `0x4AF` | Arm → PC | Firmware Version | 7+ raw bytes |
| `0x470` | Bidirectional | Master/Slave Config | Mode, reserved |
| `0x471` | Bidirectional | Motor Enable/Disable | Reserved, enable flags |
| `0x472` | Bidirectional | Query/Response Motor Limits | Per-joint max vel/acc |
| `0x473` | Bidirectional | Motor Angle Limit Feedback | Per-joint max angle |
| `0x476` | Bidirectional | Set Instruction Response | Index, success |
| `0x478` | Bidirectional | End Vel/Acc Params | Vel, acc limits |
| `0x47B` | Bidirectional | Crash Protection Config | Per-joint protection level |
| `0x47C` | Bidirectional | Motor Max Acc Limit | Query/response |
| `0x47E` | Bidirectional | Gripper Teach Pendant Params | Percent, angle |

---

## Frame Definitions — Action (PC → Arm)

### Motion Control 1 — `0x150`

| Byte | Field | Type | Values | Description |
|------|-------|------|--------|-------------|
| 0 | emergency_stop | uint8 | `0x00` released, `0x01` stopped, `0x02` resumed | Emergency stop control |
| 1 | track_control | uint8 | `0x00` off, `0x01` pause, `0x02` resume, `0x03` clear, `0x04` all, `0x05` get, `0x06` terminate, `0x07` transmit, `0x08` end | Trajectory playback control |
| 2 | teach_control | uint8 | `0x00` off, `0x01` record, `0x02` end, `0x03` execute, `0x04` pause, `0x05` resume, `0x06` terminate, `0x07` start | Teach mode control |
| 3 | trajectory_index | uint8 | 0–255 | Trajectory number to operate |
| 4–5 | name_index | uint16 | 0–65535 | Name identifier |
| 6–7 | crc | uint16 | — | Checksum |

### Motion Control 2 — `0x151`

| Byte | Field | Type | Values | Description |
|------|-------|------|--------|-------------|
| 0 | ctrl_mode | uint8 | `0x00` standby, `0x01` CAN, `0x03` Ethernet, `0x04` WiFi, `0x07` offline | Control channel |
| 1 | move_mode | uint8 | `0x00` MOVE_P, `0x01` MOVE_J, `0x02` MOVE_L, `0x03` MOVE_C, `0x04` MIT, `0x05` CPV | Motion mode |
| 2 | speed_percent | uint8 | 0–100 | Speed scaling (%) |
| 3 | mit_mode | uint8 | `0x00` position, `0xAD` MIT enable, `0xFF` MIT off | MIT activation |
| 4 | residence_time | uint8 | 0–254 seconds, `0xFF` terminate | Dwell time at target |
| 5 | installation_position | uint8 | `0x00` inverted, `0x01` horizontal, `0x02` left, `0x03` right | Mounting orientation |
| 6–7 | reserved | — | `0x00 0x00` | Must be zero |

### Joint Angle Control 12 — `0x155`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | joint_1 | int32 | 0.001° |
| 4–7 | joint_2 | int32 | 0.001° |

### Joint Angle Control 34 — `0x156`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | joint_3 | int32 | 0.001° |
| 4–7 | joint_4 | int32 | 0.001° |

### Joint Angle Control 56 — `0x157`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | joint_5 | int32 | 0.001° |
| 4–7 | joint_6 | int32 | 0.001° |

### Cartesian Control XY — `0x152`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | x | int32 | 0.001 mm |
| 4–7 | y | int32 | 0.001 mm |

### Cartesian Control Z_RX — `0x153`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | z | int32 | 0.001 mm |
| 4–7 | rx | int32 | 0.001° |

### Cartesian Control RY_RZ — `0x154`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | ry | int32 | 0.001° |
| 4–7 | rz | int32 | 0.001° |

### Gripper Control — `0x159`

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0–3 | gripper_angle | int32 | 0.001 mm | Target gripper travel |
| 4–5 | grippers_effort | uint16 | 0.001 N·m | Max grip effort (unsigned) |
| 6 | gripper_code | uint8 | — | `0x00` disable, `0x01` enable, `0x02` disable+clear, `0x03` enable+clear |
| 7 | set_zero | uint8 | — | `0x00` invalid, `0xAE` set current position as zero |

### MIT Control — `0x15A`–`0x15F`

One frame per joint. Values are pre-converted from float to unsigned integers via `FloatToUint()`.

| Parameter | Float range | Mapped to | Bits | Bit range |
|-----------|-------------|-----------|------|-----------|
| pos_ref | [−12.5, 12.5] rad | uint16 | 16 | bytes 0–1 |
| vel_ref | [−45.0, 45.0] rad/s | uint12 | 12 | byte 2 + nibble of byte 3 |
| kp | [0.0, 500.0] | uint12 | 12 | nibble of byte 3 + byte 4 |
| kd | [−5.0, 5.0] | uint12 | 12 | byte 5 + nibble of byte 6 |
| t_ref | [−8.0, 8.0] N·m | uint8 | 8 | nibble of byte 6 + nibble of byte 7 |
| crc | — | uint4 | 4 | low nibble of byte 7 |

```
Byte 0:  pos_ref [15:8]
Byte 1:  pos_ref [7:0]
Byte 2:  vel_ref [11:4]
Byte 3:  vel_ref [3:0] | kp [11:8]
Byte 4:  kp [7:0]
Byte 5:  kd [11:4]
Byte 6:  kd [3:0] | t_ref [7:4]
Byte 7:  t_ref [3:0] | CRC [3:0]
```

**CRC computation:**
```
crc = (Byte0 ^ Byte1 ^ Byte2 ^ Byte3 ^ Byte4 ^ Byte5 ^ Byte6) & 0x0F
Byte7 = ((t_ref & 0x0F) << 4) | crc
```

---

## Frame Definitions — Feedback (Arm → PC)

### Arm Status — `0x2A1`

| Byte | Field | Type | Values | Description |
|------|-------|------|--------|-------------|
| 0 | ctrl_mode | uint8 | `0x00` standby, `0x01` CAN, `0x03` Ethernet, `0x04` WiFi, `0x07` offline | Active control channel |
| 1 | arm_status | uint8 | `0x00` disabled, `0x01` enabled | Arm enabled state |
| 2 | mode_feed | uint8 | `0x00` idle, `0x01` joint ctrl, `0x02` cartesian ctrl, `0x03` circular, `0x04` MIT, `0x05` CPV | Current motion mode |
| 3 | teach_status | uint8 | `0x00` off, `0x01` on | Teach pendant active |
| 4 | motion_status | uint8 | `0x00` idle, `0x01` running, `0x02` paused, `0x03` completed | Motion execution state |
| 5 | trajectory_num | uint8 | 0–255 | Active trajectory index |
| 6–7 | err_code | uint16 | — | Packed error code (see below) |

**Error code bit fields (uint16, big-endian):**

| Bits | Byte | Field | Description |
|------|------|-------|-------------|
| 7:0 | byte 7 | comm_status | Bit 0–5 = joints 1–6 communication error |
| 15:8 | byte 6 | angle_limit | Bit 8–13 = joints 1–6 angle limit exceeded |

### End-Effector Pose XY — `0x2A2`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | x | int32 | 0.001 mm |
| 4–7 | y | int32 | 0.001 mm |

### End-Effector Pose Z_RX — `0x2A3`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | z | int32 | 0.001 mm |
| 4–7 | rx | int32 | 0.001° |

### End-Effector Pose RY_RZ — `0x2A4`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | ry | int32 | 0.001° |
| 4–7 | rz | int32 | 0.001° |

### Joint Feedback 12 — `0x2A5`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | joint_1 | int32 | 0.001° |
| 4–7 | joint_2 | int32 | 0.001° |

### Joint Feedback 34 — `0x2A6`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | joint_3 | int32 | 0.001° |
| 4–7 | joint_4 | int32 | 0.001° |

### Joint Feedback 56 — `0x2A7`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | joint_5 | int32 | 0.001° |
| 4–7 | joint_6 | int32 | 0.001° |

### Gripper Feedback — `0x2A8`

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0–3 | gripper_angle | int32 | 0.001 mm | Current gripper travel |
| 4–5 | grippers_effort | int16 | 0.001 N·m | Current effort (**signed**) |
| 6 | gripper_status | uint8 | — | Bit-packed status (see below) |
| 7 | reserved | — | — | — |

**Gripper status bit fields:**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | voltage_low | Supply voltage below threshold |
| 1 | motor_over_temp | Motor temperature exceeds limit |
| 2 | driver_over_current | Driver over-current detected |
| 3 | driver_over_temp | Driver temperature exceeds limit |
| 4 | sensor_error | Gripper sensor fault |
| 5 | driver_error | Driver fault |
| 6 | driver_enabled | Gripper driver is enabled |
| 7 | homing_status | Homing procedure completed |

### Motor High-Speed Feedback — `0x251`–`0x256`

One frame per joint (0x251 = J1, 0x252 = J2, ..., 0x256 = J6).

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0–1 | motor_speed | int16 | 0.001 rad/s | Current motor velocity |
| 2–3 | current | int16 | 0.001 A | Motor current (signed; negative = reverse) |
| 4–7 | position | int32 | motor-specific | Encoder position count |

### Motor Low-Speed Feedback — `0x261`–`0x266`

One frame per joint (0x261 = J1, 0x262 = J2, ..., 0x266 = J6).

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0–1 | voltage | uint16 | 0.1 V | Bus voltage |
| 2–3 | foc_temp | int16 | 1°C | FOC controller temperature |
| 4 | motor_temp | int8 | 1°C | Motor temperature (signed) |
| 5 | foc_status | uint8 | — | Bit-packed FOC status (see below) |
| 6–7 | bus_current | uint16 | 0.001 A | Total bus current |

**FOC status bit fields:**

| Bit | Name | Description |
|-----|------|-------------|
| 0 | voltage_low | Supply voltage below threshold |
| 1 | motor_over_temp | Motor temperature exceeds limit |
| 2 | driver_over_current | Driver over-current detected |
| 3 | driver_over_temp | Driver temperature exceeds limit |
| 4 | collision_protection | Collision detected |
| 5 | driver_error | Driver fault |
| 6 | driver_enabled | Motor driver is enabled |
| 7 | stall_protection | Stall detected |

### Firmware Version — `0x4AF`

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0–6 | version_raw | raw bytes | Variable-length firmware version string |

---

## Frame Definitions — Config (Bidirectional)

### Master/Slave Config — `0x470`

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | mode | uint8 | `0x00` master, `0xFC` slave |
| 1–7 | reserved | — | Must be zero |

### Motor Enable/Disable — `0x471`

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0–3 | reserved | — | Must be zero |
| 4 | enable_all | uint8 | `0x01` enable all, `0x02` disable all |
| 5 | reserved | — | — |
| 6 | enable_bits | uint8 | Per-joint enable (bits 0–5) |
| 7 | disable_bits | uint8 | Per-joint disable (bits 0–5) |

### Query Motor Max Angle/Speed/Acc — `0x472`

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | index | uint8 | Joint index (1–6), 0 = query all |
| 1–7 | reserved | — | Must be zero |

Response (`0x473`): Per-joint max angle, max speed, max acceleration limits.

### Set Instruction Response — `0x476`

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | index | uint8 | Instruction index |
| 1 | success | uint8 | `0x01` success, `0x00` failure |
| 2–7 | reserved | — | — |

### End Velocity/Acceleration Params — `0x478`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–3 | max_vel | int32 | 0.001 m/s |
| 4–7 | max_acc | int32 | 0.001 m/s² |

### Crash Protection Config — `0x47B`

| Byte | Field | Type | Description |
|------|-------|------|-------------|
| 0 | level | uint8 | Protection level per joint |
| 1–7 | reserved | — | — |

### Gripper Teach Pendant Params — `0x47E`

| Byte | Field | Type | Unit |
|------|-------|------|------|
| 0–1 | teach_percent | uint16 | 0.1% |
| 2–3 | teach_angle | uint16 | 0.001 mm |
| 4–7 | reserved | — | — |

---

## Asymmetries Between Control and Feedback

| Feature | Control (TX) | Feedback (RX) |
|---------|--------------|---------------|
| Gripper effort | `uint16` (0–5000, unsigned) | `int16` (signed, supports reverse) |
| Gripper status | Single `enable` bit | 8-bit packed status fields |
| MIT mode | Bit-packed (pos/vel/kp/kd/t + CRC) | Unpacked: speed (int16), current (int16), position (int32) |
| Status | No direct counterpart | `0x2A1` provides ctrl_mode, arm_status, err_code |
| Joint/Cartesian | Identical layout, mirrored IDs | Identical layout, mirrored IDs |

---

## Encoding Example: Joint Control

To send joint 1 = 45.5°:

```
value = int(45.5 * 1000) = 45500
45500 in hex = 0x0000B1BC

Frame 0x155:
  Byte 0: 0x00
  Byte 1: 0x00
  Byte 2: 0xB1
  Byte 3: 0xBC
  Bytes 4–7: joint_2 (int32, 0.001°)
```

To send joint 1 = −30.0°:

```
value = int(-30.0 * 1000) = -30000
-30000 in two's complement (32-bit) = 0xFFFF8AD0

Frame 0x155:
  Byte 0: 0xFF
  Byte 1: 0xFF
  Byte 2: 0x8A
  Byte 3: 0xD0
  Bytes 4–7: joint_2 (int32, 0.001°)
```

---

## Encoding Example: Gripper Control

To set gripper to 50mm travel with 3.5 N·m effort, enabled:

```
angle = int(50.0 * 1000) = 50000 = 0x0000C350
effort = int(3.5 * 1000) = 3500 = 0x0DAC

Frame 0x159:
  Byte 0: 0x00
  Byte 1: 0x00
  Byte 2: 0xC3
  Byte 3: 0x50
  Byte 4: 0x0D
  Byte 5: 0xAC
  Byte 6: 0x01 (enable)
  Byte 7: 0x00 (no zero-set)
```

---

## Note on Unit Conversions

| Quantity | Internal unit | Conversion |
|----------|---------------|------------|
| Joint angle | 0.001° | `rad = deg × π / 180` |
| Position | 0.001 mm | `m = mm / 1000` |
| Gripper travel | 0.001 mm | `m = mm / 1000` |
| Gripper effort (TX) | 0.001 N·m | `N·m = raw / 1000` |
| Motor speed | 0.001 rad/s | `rad/s = raw / 1000` |
| Motor current | 0.001 A | `A = raw / 1000` |
| Motor voltage | 0.1 V | `V = raw / 10` |
| Temperature | 1°C | Direct |
