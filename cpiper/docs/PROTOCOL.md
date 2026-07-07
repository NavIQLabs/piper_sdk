# CAN Protocol Reference

## Physical Layer

- **Bus**: CAN 2.0A, 11-bit standard identifiers
- **Baud rate**: 1,000,000 bps (1 Mbps) -- fixed, never change
- **Frame size**: 8 bytes per frame
- **Byte order**: Big-endian (network byte order)

## Message Categories

| Category | Direction | CAN ID Range |
|----------|-----------|-------------|
| Feedback | arm → PC | 0x251-0x2A8, 0x473-0x4AF |
| Control | PC → arm | 0x150-0x15F, 0x176, 0x470-0x47F |

## Feedback Messages (arm → PC)

| CAN ID | Name | Content | Units |
|--------|------|---------|-------|
| 0x2A1 | Status | ctrl_mode, arm_status, mode_feed, teach_status, motion_status, trajectory_num, err_code | — |
| 0x2A2 | End Pose 1 | X_axis, Y_axis | 0.001mm |
| 0x2A3 | End Pose 2 | Z_axis, RX_axis | 0.001mm, 0.001° |
| 0x2A4 | End Pose 3 | RY_axis, RZ_axis | 0.001° |
| 0x2A5 | Joint 12 | joint_1, joint_2 | 0.001° |
| 0x2A6 | Joint 34 | joint_3, joint_4 | 0.001° |
| 0x2A7 | Joint 56 | joint_5, joint_6 | 0.001° |
| 0x2A8 | Gripper | angle, effort, status_code | 0.001mm, 0.001N/m |
| 0x251-0x256 | High-Speed Motor 1-6 | speed, current, position | 0.001 rad/s, 0.001A, 0.001 rad |
| 0x261-0x266 | Low-Speed Motor 1-6 | voltage, foc_temp, motor_temp, foc_status, bus_current | mV, 0.1°C, °C, —, mA |
| 0x4AF | Firmware | firmware_data[8] | raw bytes |
| 0x476 | Instruction Response | instruction_index, is_set_zero_successfully | — |
| 0x473 | Motor Angle Limit | motor_num, max_angle, min_angle, max_spd | 0.1°, 0.001 rad/s |
| 0x478 | End Vel/Acc Param | lin_vel, ang_vel, lin_acc, ang_acc | 0.001 m/s, etc. |
| 0x47B | Crash Protection | protection_level[6] | — |
| 0x47C | Motor Max Acc | motor_num, max_acc | 0.001 rad/s² |
| 0x47E | Gripper Teaching | range_per, max_range, friction | %, mm, N/m |

## Control Messages (PC → arm)

| CAN ID | Name | Content | Notes |
|--------|------|---------|-------|
| 0x150 | Motion Ctrl 1 | e_stop, track, teach | E-stop, track/teach mode |
| 0x151 | Motion Ctrl 2 | ctrl_mode, move_mode, spd_rate, mit_mode, residence_time, install_pos | Master control |
| 0x152 | Cartesian 1 | X_axis, Y_axis | 0.001mm |
| 0x153 | Cartesian 2 | Z_axis, RX_axis | 0.001mm, 0.001° |
| 0x154 | Cartesian 3 | RY_axis, RZ_axis | 0.001° |
| 0x155 | Joint Ctrl 12 | joint_1, joint_2 | 0.001° |
| 0x156 | Joint Ctrl 34 | joint_3, joint_4 | 0.001° |
| 0x157 | Joint Ctrl 56 | joint_5, joint_6 | 0.001° |
| 0x158 | Circular Ctrl | instruction_num | — |
| 0x159 | Gripper Ctrl | angle, effort, status_code, set_zero | 0.001mm, 0.001N/m |
| 0x15A-0x15F | MIT Ctrl 1-6 | pos_ref, vel_ref, kp, kd, t_ref, CRC | Per motor |
| 0x176 | Req Master Home | mode | — |
| 0x470 | Master/Slave | linkage, fb_offset, ctrl_offset, link_offset | — |
| 0x471 | Motor Enable | motor_num, enable_flag | — |
| 0x472 | Search Motor | motor_num, search_content | Query limits |
| 0x474 | Motor Angle Limit Set | motor_num, max_angle, min_angle, max_spd | — |
| 0x475 | Joint Config | joint_num, set_zero, acc_effective, max_acc, clear_err | — |
| 0x477 | Instruction Response | instruction_index, zero_config_success_flag | — |
| 0x479 | Param Enquiry | param_enquiry, param_setting, data_feedback, load_eff, set_load | — |
| 0x47A | End Vel/Acc Config | lin_vel, ang_vel, lin_acc, ang_acc | — |
| 0x47D | Crash Protection Config | protection_level[6] | — |
| 0x47F | Gripper Teaching Config | range_per, max_range, friction | — |

## Byte Layout Examples

### Joint Control (0x155) — 2 joints per frame

```
Byte:  [0]    [1]    [2]    [3]    [4]    [5]    [6]    [7]
       joint_1 (int32, big-endian, 0.001°)    joint_2 (int32, big-endian, 0.001°)
```

Example: joint_1 = 50000 (50.0°), joint_2 = -25000 (-25.0°)
```
Hex:   00 00 C3 50  FF FF 9E 58
```

### Gripper Control (0x159)

```
Byte:  [0]    [1]    [2]    [3]    [4]    [5]    [6]    [7]
       angle (int32, 0.001mm)           effort (int16)  status  zero
```

### MIT Control (0x15A-0x15F)

```
Byte:  [0]    [1]    [2]    [3]    [4]    [5]    [6]    [7]
       pos_ref (uint16)   vel(12b)|kp(4b)  kp(8b)  kd(12b)|t(4b)  t(4b)|CRC
```

- pos_ref: 16-bit, mapped from [-12.5, 12.5] deg
- vel_ref: 12-bit, mapped from [-45, 45] deg/s
- kp: 12-bit, mapped from [0, 500]
- kd: 12-bit, mapped from [-5, 5]
- t_ref: 8-bit, mapped from [-10, 10] N/m
- CRC: XOR of bytes 0-6, masked to 4 bits

### Status Feedback (0x2A1)

```
Byte:  [0]          [1]          [2]        [3]          [4]           [5]            [6-7]
       ctrl_mode    arm_status   mode_feed  teach_status motion_status trajectory_num err_code (uint16)
```

## Data Conversion

### Joint Angles

```c
// Degrees to raw (int32, 0.001° resolution)
int32_t raw = (int32_t)(degrees * 1000.0f);

// Raw to degrees
float degrees = raw / 1000.0f;
```

### End-Effector Position

```c
// mm to raw (int32, 0.001mm resolution)
int32_t raw = (int32_t)(mm * 1000.0f);

// Raw to mm
float mm = raw / 1000.0f;
```

### MIT Mode Float Mapping

```c
// Map float to unsigned integer for CAN
uint32_t raw = cpiper_float_to_uint(value, min, max, bits);

// Example: map torque -10.0..10.0 to 8 bits
uint32_t t_raw = cpiper_float_to_uint(torque, -10.0f, 10.0f, 8);
```

## CAN ID Reference Table

### Feedback (arm → PC)

| ID | Message |
|----|---------|
| 0x251-0x256 | Motor high-speed feedback 1-6 |
| 0x261-0x266 | Motor low-speed feedback 1-6 |
| 0x2A1 | Arm status |
| 0x2A2-0x2A4 | End-effector pose (XYZ, RX, RY, RZ) |
| 0x2A5-0x2A7 | Joint angles (12, 34, 56) |
| 0x2A8 | Gripper |
| 0x473 | Motor angle limit feedback |
| 0x476 | Instruction response |
| 0x478 | End vel/acc param feedback |
| 0x47B | Crash protection feedback |
| 0x47C | Motor max acc limit feedback |
| 0x47E | Gripper teaching param feedback |
| 0x4AF | Firmware version |

### Control (PC → arm)

| ID | Message |
|----|---------|
| 0x150 | Motion ctrl 1 (e-stop, track, teach) |
| 0x151 | Motion ctrl 2 (mode, speed, MIT) |
| 0x152-0x154 | Cartesian position (XY, Z_RX, RY_RZ) |
| 0x155-0x157 | Joint angle ctrl (12, 34, 56) |
| 0x158 | Circular ctrl |
| 0x159 | Gripper ctrl |
| 0x15A-0x15F | MIT ctrl per motor 1-6 |
| 0x176 | Request master arm home |
| 0x470 | Master/slave mode config |
| 0x471 | Motor enable/disable |
| 0x472 | Search motor max angle/speed |
| 0x474 | Set motor angle limit |
| 0x475 | Joint config |
| 0x477 | Instruction response config |
| 0x479 | Param enquiry and config |
| 0x47A | End vel/acc param config |
| 0x47D | Crash protection config |
| 0x47F | Gripper teaching config |
