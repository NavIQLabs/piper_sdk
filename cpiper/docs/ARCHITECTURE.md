# Architecture

## Overview

cpiper is a single-header-include C library for controlling Agilex Piper robot
arms over CAN bus. It is designed for real-time control loops running at 500 Hz
with minimal CPU overhead.

## Layer Diagram

```
+------------------------------------------------------------------+
|                     User Application                              |
+------------------------------------------------------------------+
        |                          |
        v                          v
+----------------+    +---------------------+
|  interface.h   |    |   protocol.h        |
|  High-level    |    |   Encode/decode     |
|  control API   |    |   CAN frames        |
+----------------+    +---------------------+
        |                          |
        v                          v
+----------------+    +---------------------+
|  kinematics.h  |    |   can_ids.h         |
|  Forward       |    |   CAN ID lookup     |
|  kinematics    |    |   tables            |
+----------------+    +---------------------+
        |                          |
        v                          v
+----------------+    +---------------------+
| param_manager  |    |   can.h             |
| Joint/gripper  |    |   Linux socketcan   |
| limits         |    |   Non-blocking I/O  |
+----------------+    +---------------------+
                                |
                                v
                    +---------------------+
                    |   Linux kernel      |
                    |   CAN subsystem     |
                    +---------------------+
```

## Layer 1: CAN Bus (`can.h`)

Thin wrapper around Linux socketcan. Key properties:

- **Non-blocking**: `recv()` uses `MSG_DONTWAIT`, returns immediately if no data
- **No threads**: the CAN layer itself never creates threads
- **Socket validation**: `judge=true` checks socket exists, is UP, and has correct bitrate
- **1 Mbps**: CAN baud rate is fixed at 1000000

```c
cpiper_can_t can;
cpiper_can_init(&can, "can0", true);   // open socket
cpiper_can_send(&can, 0x155, data);     // non-blocking send
int n = cpiper_can_recv(&can, &id, data, &ts);  // non-blocking recv
cpiper_can_close(&can);
```

## Layer 2: Protocol (`protocol.h`)

Pure encode/decode logic. No I/O, no threads. Operates on raw byte arrays.

- **Decode**: CAN ID → message type lookup → extract fields from 8-byte payload
- **Encode**: message type + fields → CAN ID lookup → pack into 8-byte payload
- **Byte order**: All data is big-endian
- **Inline functions**: Byte helpers are `static inline` for zero-overhead

### Data Units

| Field Type | C Type | Resolution | Range |
|-----------|--------|-----------|-------|
| Joint angle | int32 | 0.001 deg | ±300° |
| End pose XYZ | int32 | 0.001 mm | ±1000 mm |
| End pose RPY | int32 | 0.001 deg | ±360° |
| Gripper angle | int32 | 0.001 mm | 0-150 mm |
| Motor speed | int16 | 0.001 rad/s | ±45 rad/s |
| Motor current | int16 | 0.001 A | ±5 A |
| Temperature | int16 | 0.1 °C | -40 to 200°C |

### MIT Mode CRC

For MIT control messages (CAN IDs 0x15A-0x15F), CRC is computed as:

```
crc = (data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6]) & 0x0F
```

Packed into byte 7 lower 4 bits.

## Layer 3: Interface (`interface.h`)

High-level API. Manages:
- CAN bus lifecycle
- Thread-safe feedback storage (mutex-protected)
- Optional internal receive thread
- Forward kinematics
- FPS monitoring
- Joint/gripper limit clamping

### Threading Model

cpiper offers two modes:

**Single-threaded (recommended for 500 Hz loops):**
```c
cpiper_connect(&arm, true, false);  // no recv thread
while (running) {
    cpiper_joint_ctrl(&arm, ...);   // send command
    cpiper_poll(&arm);              // receive + decode all pending frames
    joints = cpiper_get_joints(&arm);  // read cached feedback
    usleep(2000);                   // 500 Hz
}
```

**Dual-threaded (for event-driven apps):**
```c
cpiper_connect(&arm, true, true);   // start recv thread
// Recv thread calls cpiper_poll() internally at max rate
while (running) {
    joints = cpiper_get_joints(&arm);  // thread-safe read
    cpiper_joint_ctrl(&arm, ...);      // send from main thread
    usleep(2000);
}
```

### Feedback Caching

All feedback data is stored in `cpiper_interface_t` behind a single
`pthread_mutex_t`. Getter functions acquire the lock, copy the data,
and release it. This allows safe reads from any thread.

### Data Flow (Send)

```
cpiper_joint_ctrl(&arm, j1, j2, j3, j4, j5, j6)
  |
  v
  ArmMsgJointCtrl(j1, j2)  -->  protocol encode  -->  CAN send (0x155)
  ArmMsgJointCtrl(j3, j4)  -->  protocol encode  -->  CAN send (0x156)
  ArmMsgJointCtrl(j5, j6)  -->  protocol encode  -->  CAN send (0x157)
```

### Data Flow (Receive)

```
cpiper_poll(&arm)
  |
  v
  cpiper_can_recv()        <-- non-blocking read from socketcan
  |
  v
  cpiper_decode(id, data)  <-- CAN ID → msg type, extract fields
  |
  v
  Update arm->fb_* cache   <-- mutex-protected copy
```

## Layer 4: Kinematics (`kinematics.h`)

DH-parameter forward kinematics. Converts 6 joint angles (radians) to
end-effector pose (x, y, z, roll, pitch, yaw).

```c
cpiper_fk_dh_t dh;
cpiper_fk_init(&dh, 1);  // dh_is_offset=1 for firmware >= S-V1.6-3

float joints[6] = {0, 0, 0, 0, 0, 0};  // radians
float pos[6];  // [x, y, z, roll, pitch, yaw]
cpiper_fk_calculate(&dh, joints, pos);
```

## Param Manager

Singleton that stores joint and gripper limits. Shared across all
`cpiper_interface_t` instances.

```c
cpiper_param_manager_init();
cpiper_param_set_joint_limit(1, -2.79, 2.79);  // radians
cpiper_param_set_gripper_range(0.0, 150.0);     // mm
```

## File Map

```
cpiper/
├── include/cpiper/
│   ├── cpiper.h               All-in-one include
│   ├── can_ids.h              CAN ID constants
│   ├── messages.h             Message structs + enum
│   ├── protocol.h             Encode/decode + byte helpers
│   ├── can.h                  CAN bus abstraction
│   ├── interface.h            High-level API
│   ├── kinematics.h           Forward kinematics
│   ├── param_manager.h        Joint/gripper limits
│   └── fps.h                  FPS counter
├── src/
│   ├── protocol.c             Protocol implementation
│   ├── can.c                  CAN implementation
│   ├── interface.c            Interface implementation
│   ├── kinematics.c           FK implementation
│   ├── param_manager.c        Param manager
│   └── fps.c                  FPS counter
├── examples/
│   └── send_zeros.c           Hold joints at 0 degrees
├── tests/
│   ├── test_protocol.c        27 unit tests
│   ├── test_can.c             6 CAN tests
│   ├── test_interface.c       18 interface tests
│   └── test_compare.py        Python SDK comparison
└── CMakeLists.txt             Build system
```

## Design Principles

1. **Zero allocation in hot path**: All buffers are stack or struct-local.
   No malloc after init.
2. **Non-blocking I/O**: Every CAN operation returns immediately.
   No blocking waits, no busy loops.
3. **Single compilation unit**: The entire library compiles to ~162 KB
   static archive. No link-time dependencies beyond libc and pthreads.
4. **Thread safety**: Feedback getters are mutex-protected. Multiple
   threads can safely read while the recv thread updates.
5. **No external dependencies**: Only Linux kernel CAN support.
   No third-party libraries.
