# Examples

## send_zeros

The only included example. Holds all 6 joints at 0 degrees in a tight loop.

### Build

```bash
cd cpiper/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make send_zeros
```

### Run

```bash
# On real hardware
sudo ./send_zeros can0

# On virtual CAN (testing)
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set up vcan0
sudo ./send_zeros vcan0
```

### What It Does

1. Opens CAN bus (default: `vcan0`)
2. Connects without init queries (`do_piper_init=false`)
3. Enables all motors
4. Sets joint control mode at 50% speed
5. Loops at ~500 Hz:
   - Sends joint command (all zeros)
   - Polls for feedback
   - Prints joint angles
6. On SIGINT: disables motors, disconnects, cleans up

### Code Walkthrough

```c
// Initialize
cpiper_init(&arm, can_name);
cpiper_connect(&arm, false, false);  // no init, no thread

// Enable
cpiper_enable_piper(&arm);
cpiper_motion_ctrl_2(&arm, 0x01, 0x00, 50, 0x00, 0);

// 500 Hz loop
while (running) {
    cpiper_joint_ctrl(&arm, 0, 0, 0, 0, 0, 0);  // send
    cpiper_poll(&arm);                             // recv
    cpiper_joint_state_t j = cpiper_get_joints(&arm);  // read
    usleep(2000);                                  // 500 Hz
}

// Cleanup
cpiper_disable_piper(&arm);
cpiper_disconnect(&arm);
cpiper_destroy(&arm);
```

## Writing Your Own Examples

### Minimal Skeleton

```c
#include <cpiper/cpiper.h>
#include <unistd.h>
#include <signal.h>

static volatile int running = 1;
static void sig_handler(int sig) { running = 0; }

int main(int argc, char **argv) {
    signal(SIGINT, sig_handler);
    const char *can = (argc > 1) ? argv[1] : "vcan0";

    cpiper_interface_t arm;
    cpiper_init(&arm, can);
    cpiper_connect(&arm, true, true);

    // Your code here

    cpiper_disconnect(&arm);
    cpiper_destroy(&arm);
    return 0;
}
```

### Compile

```bash
gcc -o my_example my_example.c \
    -I/path/to/cpiper/include \
    -L/path/to/cpiper/build -lcpiper \
    -lpthread -lm
```

Or with CMake:

```cmake
add_executable(my_example my_example.c)
target_link_libraries(my_example PRIVATE cpiper pthread m)
target_include_directories(my_example PRIVATE ../include)
```

## Test Scripts

### test_compare.py

Compares cpiper C library output with the Python SDK. Validates
byte-level correctness of encode/decode operations.

```bash
cd cpiper/build
cmake .. && make
cd ..
../.venv/bin/python3 tests/test_compare.py
```

Requires: `piper_sdk` installed, `libcpiper.so` built.

## More Examples

See the Python SDK demos at `piper_sdk/demo/V2/` for additional
use cases. The C API mirrors the Python API closely, so porting
is straightforward:

| Python | C |
|--------|---|
| `piper.JointCtrl(j1, j2, j3, j4, j5, j6)` | `cpiper_joint_ctrl(&arm, j1, j2, j3, j4, j5, j6)` |
| `piper.GetArmJointStates()` | `cpiper_get_joints(&arm)` |
| `piper.GripperCtrl(angle, effort, status, zero)` | `cpiper_gripper_ctrl(&arm, angle, effort, status, zero)` |
| `piper.MitCtrl(motor, pos, vel, kp, kd, t)` | `cpiper_mit_ctrl(&arm, motor, pos, vel, kp, kd, t)` |
| `piper.MasterSlaveConfig(0xFC, 0, 0, 0)` | `cpiper_master_slave_config(&arm, 0xFC, 0, 0, 0)` |
