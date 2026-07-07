# Tutorial

Step-by-step guide to using cpiper for Piper robot arm control.

## Prerequisites

- Linux (Ubuntu 18.04/20.04/22.04)
- CAN bus adapter (USB-to-CAN or PCIe CAN module)
- Piper robot arm connected via CAN

## Setup

### 1. Build cpiper

```bash
cd cpiper
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
```

### 2. Activate CAN bus

```bash
# Detect CAN ports
../piper_sdk/find_all_can_port.sh

# Activate a single CAN port (1 Mbps)
sudo ../piper_sdk/can_activate.sh can0 1000000

# Or use virtual CAN for testing
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set up vcan0
```

### 3. Verify

```bash
# Send test zeros to all joints
sudo ./send_zeros vcan0   # or can0
```

## Example 1: Move Joints to Specific Angles

```c
#include <cpiper/cpiper.h>
#include <unistd.h>
#include <signal.h>

static volatile int running = 1;
static void sig_handler(int sig) { running = 0; }

int main(void) {
    signal(SIGINT, sig_handler);

    cpiper_interface_t arm;
    cpiper_init(&arm, "can0");
    cpiper_connect(&arm, true, true);   // init + recv thread

    // Enter slave mode (required for joint feedback)
    cpiper_master_slave_config(&arm, 0xFC, 0, 0, 0);

    // Enable all motors
    cpiper_enable_piper(&arm);

    // Set mode: joint control, 50% speed
    cpiper_motion_ctrl_2(&arm, 0x01, 0x00, 50, 0x00, 0);

    // Move joints to target angles
    cpiper_joint_ctrl(&arm, 10.0, -20.0, 30.0, 0.0, 0.0, 0.0);
    usleep(500000);  // wait 500ms

    // Read feedback
    cpiper_joint_state_t joints = cpiper_get_joints(&arm);
    printf("J1=%.3f  J2=%.3f  J3=%.3f\n",
           joints.joint_1 / 1000.0,
           joints.joint_2 / 1000.0,
           joints.joint_3 / 1000.0);

    // Cleanup
    cpiper_disable_piper(&arm);
    cpiper_disconnect(&arm);
    cpiper_destroy(&arm);
    return 0;
}
```

Compile: `gcc -o move_joints move_joints.c -I../include -L. -lcpiper -lpthread`

## Example 2: 500 Hz Control Loop (Single-Threaded)

```c
#include <cpiper/cpiper.h>
#include <unistd.h>
#include <signal.h>

static volatile int running = 1;
static void sig_handler(int sig) { running = 0; }

int main(void) {
    signal(SIGINT, sig_handler);

    cpiper_interface_t arm;
    cpiper_init(&arm, "can0");
    cpiper_connect(&arm, true, false);   // no recv thread

    cpiper_enable_piper(&arm);
    cpiper_motion_ctrl_2(&arm, 0x01, 0x00, 50, 0x00, 0);

    while (running) {
        // Send joint command
        cpiper_joint_ctrl(&arm, 0, 0, 0, 0, 0, 0);

        // Receive and decode all pending CAN frames
        cpiper_poll(&arm);

        // Read feedback
        cpiper_joint_state_t j = cpiper_get_joints(&arm);
        printf("J1=%.3f deg\n", j.joint_1 / 1000.0);

        usleep(2000);  // 500 Hz
    }

    cpiper_disable_piper(&arm);
    cpiper_disconnect(&arm);
    cpiper_destroy(&arm);
    return 0;
}
```

**Key pattern**: Send command → `cpiper_poll()` → read feedback → sleep.

## Example 3: Enable Forward Kinematics

```c
cpiper_interface_t arm;
cpiper_init(&arm, "can0");
cpiper_connect(&arm, true, true);

// Enable FK
cpiper_enable_fk(&arm, true);

// After receiving joint feedback...
cpiper_poll(&arm);
cpiper_end_pose_t pose = cpiper_get_end_pose(&arm);
printf("X=%.3f Y=%.3f Z=%.3f mm\n",
       pose.X_axis / 1000.0,
       pose.Y_axis / 1000.0,
       pose.Z_axis / 1000.0);
```

FK computes end-effector pose from joint angles using DH parameters.
The result is available via `cpiper_get_end_pose()`.

## Example 4: MIT Impedance Control

```c
// Switch to MIT mode
cpiper_motion_ctrl_2(&arm, 0x01, 0x00, 50, 0x01, 0);  // mit=1

// Control motor 1: position=0, velocity=0, kp=10, kd=0.5, torque=0
cpiper_mit_ctrl(&arm, 1, 0.0f, 0.0f, 10.0f, 0.5f, 0.0f);
```

**Warning**: MIT mode applies direct torque control. Use appropriate gains.
Incorrect parameters can damage the arm.

## Example 5: Gripper Control

```c
// Open gripper (100mm travel)
cpiper_gripper_ctrl(&arm, 100000, 500, 1, 0);  // angle=100mm, effort=0.5N/m

// Close gripper
cpiper_gripper_ctrl(&arm, 0, 500, 1, 0);

// Read gripper state
cpiper_gripper_t g = cpiper_get_gripper(&arm);
printf("Gripper: %.3f mm\n", g.angle / 1000.0);
```

## Example 6: Query Motor Limits

```c
// Query all motor limits
cpiper_search_all_motor_max_angle_spd(&arm);

// Wait for response (~25ms)
cpiper_poll(&arm);

// Read motor limits
for (int i = 0; i < 6; i++) {
    cpiper_motor_high_spd_t m = cpiper_get_motor_high_spd(&arm, i);
    printf("Motor %d: speed=%.3f rad/s\n", i + 1, m.motor_speed / 1000.0);
}
```

## Example 7: Cartesian Control

```c
// Set mode to cartesian
cpiper_motion_ctrl_2(&arm, 0x02, 0x00, 50, 0x00, 0);

// Move to x=200mm, y=0, z=300mm, no rotation
cpiper_end_pose_ctrl(&arm,
    200000,   // x: 200mm in 0.001mm
    0,        // y
    300000,   // z: 300mm
    0, 0, 0); // rx, ry, rz (no rotation)
```

## Example 8: Set Joint Limits

```c
// Configure SDK-level joint limits
cpiper_set_joint_limit(&arm, "joint_1", -2.79f, 2.79f);  // radians
cpiper_set_joint_limit(&arm, "joint_2", -2.79f, 2.79f);
// ... repeat for joints 3-6

// Set gripper range
cpiper_set_gripper_range(&arm, 0.0f, 150.0f);  // mm
```

When limits are enabled, `cpiper_joint_ctrl()` automatically clamps
values to the configured range.

## Example 9: Raw CAN Debugging

```c
// Send a raw CAN frame
uint8_t data[8] = {0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
cpiper_send_raw(&arm, 0x150, data);  // MOTION_CTRL_1
```

## Example 10: Python ctypes Binding

cpiper can be used from Python via ctypes with the shared library:

```python
import ctypes

lib = ctypes.CDLL('./libcpiper.so')

class CpiperInterface(ctypes.Structure):
    _fields_ = [("_opaque", ctypes.c_char * 4096)]  # simplified

arm = CpiperInterface()
lib.cpiper_init(ctypes.byref(arm), b"can0")
lib.cpiper_connect(ctypes.byref(arm), True, True)

# ... use API ...

lib.cpiper_disconnect(ctypes.byref(arm))
lib.cpiper_destroy(ctypes.byref(arm))
```

See `tests/test_compare.py` for a working example.

## Common Patterns

### Startup Sequence

```
1. cpiper_init()
2. cpiper_connect(do_piper_init=true, start_thread=...)
3. cpiper_master_slave_config(0xFC, 0, 0, 0)  // slave mode
4. cpiper_enable_piper()
5. cpiper_motion_ctrl_2(0x01, 0x00, speed, 0, 0)  // joint mode
```

### Shutdown Sequence

```
1. cpiper_disable_piper()
2. cpiper_disconnect()
3. cpiper_destroy()
```

### 500 Hz Loop

```
while (running) {
    cpiper_joint_ctrl(&arm, ...);  // send
    cpiper_poll(&arm);             // recv
    joints = cpiper_get_joints(&arm);  // read
    usleep(2000);                  // 500 Hz
}
```

### Error Handling

All lifecycle functions return `int` (0 = success, negative = error).
Check return values:

```c
if (cpiper_init(&arm, "can0") < 0) {
    fprintf(stderr, "Failed to init\n");
    return 1;
}
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "No such device" | Check CAN adapter is connected, run `ip link show` |
| "Permission denied" | Run with `sudo` |
| No feedback data | Ensure `cpiper_master_slave_config(0xFC, 0, 0, 0)` was called |
| Joint angles stuck at 0 | Wait ~25ms after `ConnectPort()` before reading |
| "Network is down" | Run `sudo ip link set up can0` |
| Wrong bitrate | Re-activate: `sudo can_activate.sh can0 1000000` |
