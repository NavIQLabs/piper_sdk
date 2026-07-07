# API Reference

All functions are declared in `cpiper/interface.h` unless noted otherwise.
Include `<cpiper/cpiper.h>` for the complete API.

## Lifecycle

### cpiper_init

```c
int cpiper_init(cpiper_interface_t *p, const char *can_name);
```

Initialize an interface struct. Does not open the CAN socket.

- `p` -- pointer to uninitialized `cpiper_interface_t`
- `can_name` -- CAN device name, e.g. `"can0"`, `"vcan0"`
- Returns `0` on success, negative on error

### cpiper_connect

```c
int cpiper_connect(cpiper_interface_t *p, bool do_piper_init, bool start_thread);
```

Open the CAN socket and optionally run initialization.

- `do_piper_init` -- if `true`, queries motor limits and firmware version
- `start_thread` -- if `true`, starts internal receive thread
- Returns `0` on success, negative on error

### cpiper_disconnect

```c
void cpiper_disconnect(cpiper_interface_t *p);
```

Stop the receive thread (if running) and close the CAN socket.

### cpiper_destroy

```c
void cpiper_destroy(cpiper_interface_t *p);
```

Release all resources (mutexes, sockets). Call after `cpiper_disconnect`.

## Receive

### cpiper_poll

```c
void cpiper_poll(cpiper_interface_t *p);
```

Non-blocking: receive and decode all available CAN frames. Call in your
main loop when not using the internal receive thread. This function
updates the feedback cache under mutex protection.

## Motion Control

### cpiper_motion_ctrl_1

```c
void cpiper_motion_ctrl_1(cpiper_interface_t *p,
                          uint8_t e_stop,
                          uint8_t track,
                          uint8_t teach);
```

Motion control register 1.

| Param | Description |
|-------|-------------|
| `e_stop` | Emergency stop: 0=release, 1=engage |
| `track` | Track mode: 0=off, 1=on |
| `teach` | Teach mode: 0=off, 1=on |

CAN ID: 0x150

### cpiper_motion_ctrl_2

```c
void cpiper_motion_ctrl_2(cpiper_interface_t *p,
                          uint8_t ctrl_mode,
                          uint8_t move_mode,
                          uint8_t spd,
                          uint8_t mit,
                          uint8_t res_time);
```

Motion control register 2.

| Param | Description |
|-------|-------------|
| `ctrl_mode` | Control mode (0x01=joint, 0x02=cartesian) |
| `move_mode` | Move mode |
| `spd` | Speed percent (0-100) |
| `mit` | MIT mode enable |
| `res_time` | Residence time |

CAN ID: 0x151

### cpiper_mode_ctrl

```c
void cpiper_mode_ctrl(cpiper_interface_t *p,
                      uint8_t ctrl_mode,
                      uint8_t move_mode,
                      uint8_t spd,
                      uint8_t mit);
```

Simplified mode control (residence time = 0).

### cpiper_emergency_stop

```c
void cpiper_emergency_stop(cpiper_interface_t *p, uint8_t e_stop);
```

Convenience: send emergency stop via `motion_ctrl_1`.

### cpiper_reset

```c
void cpiper_reset(cpiper_interface_t *p);
```

Convenience: release emergency stop via `motion_ctrl_1`.

## Joint Control

### cpiper_joint_ctrl

```c
void cpiper_joint_ctrl(cpiper_interface_t *p,
                       float j1, float j2, float j3,
                       float j4, float j5, float j6);
```

Set joint angles in degrees. Sends 3 CAN frames (0x155, 0x156, 0x157).
Values are clamped to SDK limits if enabled.

CAN IDs: 0x155 (j1,j2), 0x156 (j3,j4), 0x157 (j5,j6)

## Cartesian Control

### cpiper_end_pose_ctrl

```c
void cpiper_end_pose_ctrl(cpiper_interface_t *p,
                          int32_t x, int32_t y, int32_t z,
                          int32_t rx, int32_t ry, int32_t rz);
```

Set end-effector pose. XYZ in 0.001mm, RPY in 0.001 degrees.
Sends 3 CAN frames.

CAN IDs: 0x152 (x,y), 0x153 (z,rx), 0x154 (ry,rz)

### cpiper_move_c_axis_update

```c
void cpiper_move_c_axis_update(cpiper_interface_t *p, uint8_t instruction_num);
```

Update circular motion axis.

CAN ID: 0x158

## Gripper

### cpiper_gripper_ctrl

```c
void cpiper_gripper_ctrl(cpiper_interface_t *p,
                         int32_t angle,
                         int16_t effort,
                         uint8_t status_code,
                         uint8_t set_zero);
```

Control the gripper.

| Param | Description |
|-------|-------------|
| `angle` | Target angle in 0.001mm |
| `effort` | Effort limit in 0.001N/m |
| `status_code` | Status code |
| `set_zero` | 1 = set current position as zero |

CAN ID: 0x159

## MIT Impedance Control

### cpiper_mit_ctrl

```c
void cpiper_mit_ctrl(cpiper_interface_t *p,
                     uint8_t motor_num,
                     float pos, float vel,
                     float kp, float kd, float t);
```

MIT mode impedance control for a single motor (1-6).

| Param | Range | Description |
|-------|-------|-------------|
| `pos` | [-12.5, 12.5] | Position reference (deg) |
| `vel` | [-45, 45] | Velocity reference (deg/s) |
| `kp` | [0, 500] | Position gain |
| `kd` | [-5, 5] | Velocity gain |
| `t` | [-10, 10] | Torque reference (N/m) |

CAN IDs: 0x15A (motor 1) through 0x15F (motor 6)

**Warning**: MIT mode can damage the arm. Use with caution.

## Configuration

### cpiper_master_slave_config

```c
void cpiper_master_slave_config(cpiper_interface_t *p,
                                uint8_t linkage,
                                uint8_t fb_off,
                                uint8_t ctrl_off,
                                uint8_t link_off);
```

Configure master/slave mode. Use `linkage=0xFC` for slave mode.

CAN ID: 0x470

### cpiper_enable_arm / cpiper_disable_arm

```c
void cpiper_enable_arm(cpiper_interface_t *p, uint8_t motor_num, uint8_t flag);
void cpiper_disable_arm(cpiper_interface_t *p, uint8_t motor_num, uint8_t flag);
```

Enable/disable a specific motor (1-6).

CAN ID: 0x471

### cpiper_enable_piper / cpiper_disable_piper

```c
bool cpiper_enable_piper(cpiper_interface_t *p);
bool cpiper_disable_piper(cpiper_interface_t *p);
```

Enable/disable all 6 motors. Returns `true` on success.

### cpiper_joint_config

```c
void cpiper_joint_config(cpiper_interface_t *p,
                         uint8_t joint_num,
                         uint8_t set_zero,
                         uint8_t acc_effective,
                         uint16_t max_acc,
                         uint8_t clear_err);
```

Joint configuration: set zero position, configure acceleration, clear errors.

CAN ID: 0x475

### cpiper_motor_angle_limit_max_spd_set

```c
void cpiper_motor_angle_limit_max_spd_set(cpiper_interface_t *p,
                                           uint8_t motor_num,
                                           int16_t max_angle,
                                           int16_t min_angle,
                                           uint16_t max_spd);
```

Set motor angle and speed limits.

CAN ID: 0x474

### cpiper_end_spd_acc_param_set

```c
void cpiper_end_spd_acc_param_set(cpiper_interface_t *p,
                                   uint16_t lin_vel,
                                   uint16_t ang_vel,
                                   uint16_t lin_acc,
                                   uint16_t ang_acc);
```

Set end-effector velocity and acceleration limits.

CAN ID: 0x47A

### cpiper_crash_protection_config

```c
void cpiper_crash_protection_config(cpiper_interface_t *p,
                                     uint8_t j1, uint8_t j2, uint8_t j3,
                                     uint8_t j4, uint8_t j5, uint8_t j6);
```

Configure crash protection level per joint.

CAN ID: 0x47D

### cpiper_gripper_teaching_param_config

```c
void cpiper_gripper_teaching_param_config(cpiper_interface_t *p,
                                           uint8_t range_per,
                                           uint8_t max_range,
                                           uint8_t friction);
```

Configure gripper teaching pendant parameters.

CAN ID: 0x47F

## Query

### cpiper_search_motor_max_angle_spd

```c
void cpiper_search_motor_max_angle_spd(cpiper_interface_t *p,
                                        uint8_t motor_num,
                                        uint8_t search_content);
```

Query motor angle/speed limits for a specific motor.

CAN ID: 0x472

### cpiper_search_all_motor_max_angle_spd

```c
void cpiper_search_all_motor_max_angle_spd(cpiper_interface_t *p);
```

Query all 6 motors' angle/speed limits.

### cpiper_search_all_motor_max_acc_limit

```c
void cpiper_search_all_motor_max_acc_limit(cpiper_interface_t *p);
```

Query all motors' acceleration limits.

### cpiper_search_firmware_version

```c
void cpiper_search_firmware_version(cpiper_interface_t *p);
```

Query firmware version. Sends raw frame on CAN ID 0x4AF.

### cpiper_arm_param_enquiry_and_config

```c
void cpiper_arm_param_enquiry_and_config(cpiper_interface_t *p,
                                          uint8_t enquiry,
                                          uint8_t setting,
                                          uint8_t fb,
                                          uint8_t load_effective,
                                          uint8_t set_load);
```

Parameter enquiry and configuration.

CAN ID: 0x479

### cpiper_req_master_arm_home

```c
void cpiper_req_master_arm_home(cpiper_interface_t *p, uint8_t mode);
```

Request master arm home position.

CAN ID: 0x176

## Getters (Thread-Safe)

All getters acquire `fb_mutex`, copy the data, and release the lock.
Safe to call from any thread.

### cpiper_get_status

```c
cpiper_status_t cpiper_get_status(cpiper_interface_t *p);
```

Returns arm status: ctrl_mode, arm_status, mode_feed, teach_status,
motion_status, trajectory_num, err_code.

### cpiper_get_end_pose

```c
cpiper_end_pose_t cpiper_get_end_pose(cpiper_interface_t *p);
```

Returns end-effector pose: X, Y, Z (0.001mm), RX, RY, RZ (0.001 deg).

### cpiper_get_joints

```c
cpiper_joint_state_t cpiper_get_joints(cpiper_interface_t *p);
```

Returns joint angles: joint_1..joint_6 (0.001 deg).

### cpiper_get_gripper

```c
cpiper_gripper_t cpiper_get_gripper(cpiper_interface_t *p);
```

Returns gripper state: angle (0.001mm), effort (0.001N/m), status_code.

### cpiper_get_motor_high_spd

```c
cpiper_motor_high_spd_t cpiper_get_motor_high_spd(cpiper_interface_t *p, int motor);
```

Returns high-speed motor feedback for motor 0-5.

### cpiper_get_motor_low_spd

```c
cpiper_motor_low_spd_t cpiper_get_motor_low_spd(cpiper_interface_t *p, int motor);
```

Returns low-speed motor feedback for motor 0-5.

### cpiper_get_can_fps

```c
double cpiper_get_can_fps(cpiper_interface_t *p);
```

Returns overall CAN frames per second.

### cpiper_is_ok

```c
bool cpiper_is_ok(cpiper_interface_t *p);
```

Returns `true` if the arm is in OK state (recv timeout < 1s).

### cpiper_get_firmware_version

```c
char* cpiper_get_firmware_version(cpiper_interface_t *p, char *buf, int buflen);
```

Copy firmware version string to caller buffer. Returns `buf`.

## Configuration

### cpiper_enable_fk

```c
void cpiper_enable_fk(cpiper_interface_t *p, bool enable);
```

Enable/disable forward kinematics. When enabled, `cpiper_poll()`
automatically computes end-effector pose from joint angles.

### cpiper_set_filter_abnormal

```c
void cpiper_set_filter_abnormal(cpiper_interface_t *p, bool enable);
```

Enable/disable abnormal data filtering. When enabled, feedback values
are clamped to expected ranges.

### cpiper_set_joint_limit / cpiper_set_gripper_range

```c
void cpiper_set_joint_limit(cpiper_interface_t *p, const char *joint,
                             float min_rad, float max_rad);
void cpiper_set_gripper_range(cpiper_interface_t *p, float min_mm, float max_mm);
```

Set joint/gripper limits for SDK-level clamping.

## Debug

### cpiper_send_raw

```c
int cpiper_send_raw(cpiper_interface_t *p, uint32_t can_id, const uint8_t data[8]);
```

Send a raw CAN frame. For debugging and custom protocols.

## Protocol Functions (`protocol.h`)

### cpiper_decode

```c
bool cpiper_decode(uint32_t can_id, const uint8_t data[8],
                   cpiper_message_t *msg, double timestamp);
```

Decode a CAN frame into a message struct. Returns `true` if CAN ID
was recognized.

### cpiper_encode

```c
bool cpiper_encode(const cpiper_message_t *msg, uint32_t *can_id,
                   uint8_t data[8]);
```

Encode a message struct into a CAN frame. Returns `true` if message
type was recognized.

### cpiper_float_to_uint

```c
uint32_t cpiper_float_to_uint(float x_float, float x_min, float x_max, int bits);
```

Map a float to an unsigned integer for CAN transmission (MIT mode).

## CAN Functions (`can.h`)

### cpiper_can_init

```c
int cpiper_can_init(cpiper_can_t *can, const char *name, bool judge);
```

Open a CAN socket. `judge=true` validates the interface.

### cpiper_can_close

```c
void cpiper_can_close(cpiper_can_t *can);
```

Close the CAN socket.

### cpiper_can_send

```c
int cpiper_can_send(cpiper_can_t *can, uint32_t can_id, const uint8_t data[8]);
```

Non-blocking CAN frame send. Returns 0 on success.

### cpiper_can_recv

```c
int cpiper_can_recv(cpiper_can_t *can, uint32_t *can_id,
                    uint8_t data[8], double *timestamp);
```

Non-blocking CAN frame receive. Returns 1 if frame received, 0 if
no data available.

## Forward Kinematics (`kinematics.h`)

### cpiper_fk_init

```c
void cpiper_fk_init(cpiper_fk_dh_t *dh, int dh_is_offset);
```

Initialize DH parameters. `dh_is_offset=1` applies 2 deg J2/J3 offset
for firmware >= S-V1.6-3.

### cpiper_fk_calculate

```c
void cpiper_fk_calculate(cpiper_fk_dh_t *dh, const float joints[6], float pos[6]);
```

Compute forward kinematics. `joints` in radians. `pos` output:
[x, y, z, roll, pitch, yaw] -- xyz in mm, rpy in degrees.

## Param Manager (`param_manager.h`)

### cpiper_param_manager_get

```c
cpiper_param_manager_t* cpiper_param_manager_get(void);
```

Get the singleton instance.

### cpiper_param_set_joint_limit / cpiper_param_get_joint_limit

```c
void cpiper_param_set_joint_limit(int joint, float min_val, float max_val);
void cpiper_param_get_joint_limit(int joint, float *min_val, float *max_val);
```

Set/get joint limits (joint 1-6, radians).

### cpiper_param_set_gripper_range / cpiper_param_get_gripper_range

```c
void cpiper_param_set_gripper_range(float min_val, float max_val);
void cpiper_param_get_gripper_range(float *min_val, float *max_val);
```

Set/get gripper range (mm).

### cpiper_param_clamp_joint / cpiper_param_clamp_gripper

```c
float cpiper_param_clamp_joint(int joint, float value);
float cpiper_param_clamp_gripper(float value);
```

Clamp values to configured limits.

## FPS Counter (`fps.h`)

### cpiper_fps_init

```c
void cpiper_fps_init(cpiper_fps_t *fps);
```

### cpiper_fps_update

```c
void cpiper_fps_update(cpiper_fps_t *fps, double current_time);
```

### cpiper_fps_get

```c
double cpiper_fps_get(cpiper_fps_t *fps);
```

Returns computed Hz.
