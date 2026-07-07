# cpiper

C implementation of the Piper robot arm SDK. Communicates with Agilex Piper
arms over CAN bus at 500 Hz with zero dynamic allocation in the hot path.

**Zero external dependencies** beyond Linux socketcan (`<linux/can.h>`).

## Quick Start

```bash
# Build
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)

# Run on virtual CAN (no hardware)
sudo modprobe vcan
sudo ip link add vcan0 type vcan
sudo ip link set up vcan0
sudo ./send_zeros vcan0

# Run on real hardware
sudo ./send_zeros can0
```

## Build

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..   # or Debug for -O0 -g
make -j$(nproc)
```

Produces:
- `libcpiper.a` -- static library (162 KB)
- `libcpiper.so` -- shared library (for Python ctypes binding)
- `send_zeros` -- example: holds all joints at 0 degrees
- `test_protocol` -- 27 unit tests
- `test_can` -- 6 CAN integration tests (requires vcan0)
- `test_interface` -- 18 interface tests (requires vcan0)

## Usage

```c
#include <cpiper/cpiper.h>

cpiper_interface_t arm;
cpiper_init(&arm, "can0");
cpiper_connect(&arm, true, true);   // init queries + recv thread

// Enable motors
cpiper_enable_piper(&arm);
cpiper_motion_ctrl_2(&arm, 0x01, 0x00, 50, 0x00, 0);

// Move joints (degrees)
cpiper_joint_ctrl(&arm, 10.0, -20.0, 30.0, 0.0, 0.0, 0.0);

// Read feedback
cpiper_joint_state_t joints = cpiper_get_joints(&arm);
printf("J1=%.3f deg\n", joints.joint_1 / 1000.0);

// Cleanup
cpiper_disable_piper(&arm);
cpiper_disconnect(&arm);
cpiper_destroy(&arm);
```

## Architecture

```
User Application
       |
  [ interface.h ]        High-level API, threading, feedback cache
       |
  [ protocol.h ]         Encode/decode CAN frames
       |
  [ can.h ]              Linux socketcan (non-blocking)
       |
  [ CAN bus ]            1 Mbps, 8-byte frames, 11-bit IDs
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for details.

## API Reference

See [docs/API.md](docs/API.md).

## Tutorials

See [docs/TUTORIAL.md](docs/TUTORIAL.md) for step-by-step guides.

## Protocol

See [docs/PROTOCOL.md](docs/PROTOCOL.md) for CAN ID tables and message formats.

## Testing

```bash
cd build

# Unit tests (no hardware)
./test_protocol

# CAN tests (requires vcan0)
sudo modprobe vcan && sudo ip link add vcan0 type vcan && sudo ip link set up vcan0
sudo ./test_can
sudo ./test_interface

# Python SDK comparison (requires piper_sdk + libcpiper.so)
pip3 install -e ..
python3 ../tests/test_compare.py

# Run all C tests
ctest --output-on-failure
```

## Platform

- Linux only (socketcan)
- GCC or Clang, C11
- No external libraries required

## License

Part of the piper_sdk repository.
