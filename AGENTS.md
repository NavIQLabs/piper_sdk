# AGENTS.md — piper_sdk

## What this repo is

Python SDK for controlling Agilex Piper robot arms over CAN bus.
Published to PyPI as `piper_sdk`. Current version: 0.6.2.
Only runtime dependency: `python-can>=3.3.4`. Python >=3.6, Linux-only.
Tested on Ubuntu 18.04/20.04/22.04 with Python 3.6/3.8/3.10.

## Build & install

```bash
pip3 install .          # from repo root (uses setup.py)
bash rm_tmp.sh          # clean build/, dist/, *.egg-info
pip3 show piper_sdk     # verify installation
```

## Tests

Unit tests live in `tests/` and run without CAN hardware:

```bash
conda activate piper_sdk   # python-can 4.6.1, pytest
python -m pytest tests/ -q
```

Uses a `FakeBus` (BusABC subclass) instead of real hardware. The
`C_PiperInterface_V2` singleton is keyed by `can_name` only — tests must
use unique `can_name` values to avoid cross-test singleton pollution.

## Package structure

```
piper_sdk/
├── hardware_port/          # Layer 1: CAN bus abstraction
│   ├── can_encapsulation_v0_4_0.py   # C_STD_CAN (active, v0.4.0+)
│   └── can_encapsulation.py          # old version, unused
├── protocol/               # Layer 2: CAN frame encode/decode
│   ├── piper_protocol_base.py        # C_PiperParserBase (ABC)
│   └── protocol_v2/
│       └── piper_protocol_v2.py      # C_PiperParserV2
├── piper_msgs/             # Layer 3: Message data classes
│   └── msg_v2/
│       ├── arm_messages.py           # PiperMessage (aggregate)
│       ├── arm_msg_type.py           # ArmMsgType enum
│       ├── can_id.py                 # CanIDPiper enum (all CAN IDs)
│       ├── arm_id_type_map.py        # ArmMessageMapping (ID↔Type)
│       ├── feedback/                 # 14 incoming message classes
│       └── transmit/                 # 19 outgoing message classes
├── interface/              # Layer 4: High-level API
│   ├── piper_interface.py            # C_PiperInterface (legacy name)
│   └── piper_interface_v2.py         # C_PiperInterface_V2 (main)
├── kinematics/             # Forward kinematics (DH-based)
│   └── piper_fk.py                   # C_PiperForwardKinematics
├── piper_param/            # Joint/gripper limit manager (singleton)
│   └── piper_param_manager.py        # C_PiperParamManager
├── utils/                  # FPS counter, logging, quaternion/Euler
│   ├── fps.py
│   ├── logger_mag.py
│   └── tf.py
├── demo/V2/                # 50+ working example scripts
├── can_activate.sh         # Activate single CAN port
├── can_muti_activate.sh    # Activate multiple CAN ports
├── find_all_can_port.sh    # List available CAN ports
├── can_config.sh           # CAN configuration helper
├── can_find_and_config.sh  # Auto-detect and configure CAN
└── log/                    # Log file output directory
```

## CAN communication architecture (detailed)

### Physical layer

The robot arm communicates via CAN bus at 1 Mbps (1000000 baud). The SDK
uses `python-can` with the `socketcan` backend on Linux. Serial CAN
(`slcan`) is also supported for MAC compatibility via `CreateCanBus()`.

CAN activation requires root. Scripts are inside `piper_sdk/` (not root):
- `find_all_can_port.sh` — detect USB-to-CAN adapters
- `can_activate.sh <name> <baudrate> [usb_addr]` — activate single CAN
- `can_muti_activate.sh` — activate multiple CANs (edit USB_PORTS inside)

### Layer 1: C_STD_CAN (hardware_port/can_encapsulation_v0_4_0.py)

Low-level CAN bus wrapper around `python-can`. Key details:

- **Singleton per channel**: `C_PiperInterface_V2.__init__` passes
  `self.ParseCANFrame` as a callback. `C_STD_CAN.ReadCanMessage()` calls
  this callback on every received frame.
- **No internal threads**: The class itself doesn't create threads. The
  interface's `ReadCan` thread calls `ReadCanMessage()` in a loop.
- **Bus validation on init**: If `judge_flag=True`, checks socket exists,
  port is UP, and bitrate matches before opening. Set to `False` for
  PCIe CAN modules.
- **Error codes**: `CAN_STATUS` IntEnum with ~25 status codes (100xxx
  range). `SendCanMessage` returns `SEND_MESSAGE_SUCCESS` or
  `SEND_MESSAGE_FAILED`. `ReadCanMessage` returns `READ_CAN_MSG_OK`,
  `READ_CAN_MSG_TIMEOUT` (1s recv timeout), or bus error status.
- **Bus state check**: Checks `bus.state == can.BusState.ACTIVE` before
  read/write. The read path throttles this check to every 50th call to
  reduce syscall overhead.

### Layer 2: C_PiperParserV2 (protocol/protocol_v2/piper_protocol_v2.py)

Pure encode/decode logic. No I/O, no threads. Operates on
`can.Message` objects and `PiperMessage` intermediaries.

**DecodeMessage(rx_can_frame → PiperMessage):**
- Dispatches on `rx_can_frame.arbitration_id`
- Extracts bytes from `rx_can_frame.data` (8-byte CAN frame)
- Uses `ConvertBytesToInt(data, start, end, byteorder='big')` for
  multi-byte extraction
- Uses `ConvertToNegative_Nbit()` for signed/unsigned conversion
- Sets `msg.type_` via `ArmMessageMapping.get_mapping(can_id=...)`
- Sets `msg.time_stamp` from `rx_can_frame.timestamp` (not system time)
- Returns True if CAN ID was recognized, False otherwise

**EncodeMessage(PiperMessage → tx_can_frame):**
- Looks up CAN ID via `ArmMessageMapping.get_mapping(msg_type=...)`
- Sets `tx_can_frame.arbitration_id` from mapping
- Serializes fields to 8-byte list using `ConvertToList_*bit()`
- For MIT mode (0x15A-0x15F): computes CRC (XOR of first 7 bytes
  masked to 4 bits) and packs into byte 7
- Does NOT set `channel`, `dlc`, or `is_extended_id` on the frame

**Byte ordering**: All data is big-endian. Joint angles are packed as
int32 (4 bytes, 0.001° resolution). Gripper travel as int32 (0.001mm).
Motor speed as int16 (0.001 rad/s). Current as int16 (0.001A).

### Layer 3: PiperMessage and message classes (piper_msgs/)

`PiperMessage` (arm_messages.py) is an aggregate data class holding
all possible message fields. Only the field matching the active `type_`
is populated. It contains:

- 14 feedback sub-objects (status, end pose, joints, gripper, 6× high-spd
  motor, 6× low-spd motor, gripper teach pendant param, instruction
  response, motor angle limit, end vel/acc, crash protection, motor max acc)
- 19 transmit sub-objects (motion ctrl 1/2, cartesian, joint ctrl 12/34/56,
  circular, gripper, 6× MIT ctrl, master/slave config, motor enable,
  search, joint config, instruction response, param enquiry, end vel/acc,
  crash protection, gripper teach pendant config)

`CanIDPiper` enum: All 50+ CAN IDs used by the protocol.
`ArmMessageMapping`: Bidirectional dict mapping CAN IDs ↔ ArmMsgType.
`ArmMsgType` enum: Internal message type identifiers.

**Feedback messages (CAN ID → interface storage):**

| CAN ID  | Data | Unit | Update method |
|---------|------|------|---------------|
| 0x2A1 | Status: ctrl_mode, arm_status, mode_feed, teach_status, motion_status, trajectory_num, err_code | — | `__UpdateArmStatus` |
| 0x2A2-2A4 | End pose X/Y/Z/RX/RY/RZ | 0.001mm / 0.001° | `__UpdateArmEndPoseState` |
| 0x2A5-2A7 | Joint angles 1-6 | 0.001° | `__UpdateArmJointState` |
| 0x2A8 | Gripper angle, effort, status | 0.001mm, 0.001N/m | `__UpdateArmGripperState` |
| 0x251-256 | Motor speed, current, position (per motor) | 0.001 rad/s, 0.001A | `__UpdateDriverInfoHighSpdFeedback` |
| 0x261-266 | Voltage, FOC temp, motor temp, FOC status, bus current | per field | `__UpdateDriverInfoLowSpdFeedback` |
| 0x4AF | Firmware version bytes | raw | `__UpdatePiperFirmware` |
| 0x473 | Motor angle limit feedback | 0.1° | `__UpdateCurrentMotorAngleLimitMaxVel` |
| 0x476 | Instruction response (index, success) | — | `__UpdateRespSetInstruction` |
| 0x478 | End vel/acc param feedback | 0.001 m/s, rad/s, m/s², rad/s² | `__UpdateCurrentEndVelAndAccParam` |
| 0x47B | Crash protection level per joint | — | `__UpdateCrashProtectionLevelFeedback` |
| 0x47E | Gripper/teach pendant param | %, mm | `__UpdateGripperTeachingPendantParamFeedback` |
| 0x47C | Motor max acc limit | 0.001 rad/s² | `__UpdateCurrentMotorMaxAccLimit` |

### Layer 4: C_PiperInterface_V2 (interface/piper_interface_v2.py)

High-level API. ~3700 lines. Key internals:

**Singleton per CAN name**: `__new__` uses a class-level `_instances`
dict keyed by `can_name`. Same CAN name returns same instance. Different
CAN names create separate instances with independent CAN buses. The key
is `can_name` only — `enable_performance_metrics`/`minimal_feedback_mode`
do not affect the key, so a later caller passing different flags on an
already-created name gets the first caller's instance (a warning is
logged on mismatch). Use distinct `can_name` values for distinct flag
sets.

**Threading model** (started by `ConnectPort()`, stopped by
`DisconnectPort()`):
1. **ReadCan thread** — loop calling `self.__arm_can.ReadCanMessage()`.
   Each call blocks up to 1s on `bus.recv(1)`. On receive, invokes the
   callback `self.ParseCANFrame(rx_message)`.
2. **CanMonitor thread** — runs at 20Hz (50ms sleep). Calls
   `self.__CanMonitor()` which computes FPS via `C_FPSCounter` and
   checks `isOk` status.
`DisconnectPort()` signals the CanMonitor thread to stop, joins both
threads, and stops the FPS counter.

**ParseCANFrame callback flow** (called from ReadCan thread):
```
rx_message (can.Message)
  → PiperMessage()                          # fresh aggregate
  → self.__parser.DecodeMessage(rx, msg)    # decode bytes → fields
  → __type_handlers[msg.type_]              # type-based dispatch dict
  → __UpdateMinimalArmState(msg)            # always, if enabled
```

The `__type_handlers` dict maps each `ArmMsgType` to its update handler.
Only the handler(s) matching the received frame type run, instead of
all 14 unconditionally. When `minimal_feedback_mode=True`, each frame
also updates the lightweight `__arm_state_snapshot` (joint/gripper/status
copies) without allocating SDK wrapper objects; read it via
`GetArmStateSnapshot()` and `GetArmEnableStatus()`.

**Send flow** (example: `JointCtrl(j1, j2, j3, j4, j5, j6)`):
```
JointCtrl()
  → __CalJointSDKLimit() per joint          # clamp to limits if enabled
  → __JointCtrl_12(j1, j2)
      → tx_can = Message()                   # empty can.Message
      → joint_ctrl = ArmMsgJointCtrl(j1, j2)
      → msg = PiperMessage(type_=..., arm_joint_ctrl=joint_ctrl)
      → self.__parser.EncodeMessage(msg, tx_can)  # encode → tx_can.data
      → self.__arm_can.SendCanMessage(tx_can.arbitration_id, tx_can.data)
  → __JointCtrl_34(j3, j4)
  → __JointCtrl_56(j5, j6)
```

**PiperInit** (called by `ConnectPort` with `piper_init=True`):
1. `SearchAllMotorMaxAngleSpd()` — queries motor limits
2. `SearchAllMotorMaxAccLimit()` — queries acceleration limits
3. `SearchPiperFirmwareVersion()` — queries firmware version

**Abnormal data filtering** (enabled by default):
- Gripper: clamps to [0, 150mm*1000]
- Joints: clamps to 300°*1000
- End pose XYZ: clamps to 1m*1000*1000, RPY: 361°*1000
- Disable with `DisableFilterAbnormalData()`

## Key CAN IDs reference

**Feedback (arm → PC):**
| Range | Purpose |
|-------|---------|
| 0x2A1 | Arm status |
| 0x2A2-2A4 | End-effector pose (split across 3 frames) |
| 0x2A5-2A7 | Joint angles (split: 12, 34, 56) |
| 0x2A8 | Gripper |
| 0x251-256 | Motor high-speed feedback (per motor) |
| 0x261-266 | Motor low-speed feedback (per motor) |
| 0x4AF | Firmware version |

**Control (PC → arm):**
| ID | Purpose |
|----|---------|
| 0x150 | Motion ctrl 1 (e-stop, track, teach) |
| 0x151 | Motion ctrl 2 (mode, move mode, speed %, MIT, install pos) |
| 0x152-154 | Cartesian position (XY, Z_RX, RY_RZ) |
| 0x155-157 | Joint angle ctrl (12, 34, 56) |
| 0x159 | Gripper ctrl |
| 0x15A-15F | MIT ctrl per joint (6 motors) |
| 0x470 | Master/slave mode config |
| 0x471 | Motor enable/disable |
| 0x472 | Query motor max angle/speed/acc limits |

## Usage gotchas

- Robot must be in **slave mode** to read joint feedback (call
  `MasterSlaveConfig(0xFC, 0, 0, 0)`)
- CAN baud rate is always 1000000. Never change it.
- MIT protocol can damage the arm; only use with explicit understanding
- Non-official CAN modules: set `judge_flag=False`
- `dh_is_offset=1` (default) applies 2° J2/J3 offset for firmware >= S-V1.6-3
- CAN activation scripts live inside `piper_sdk/`, not repo root
- `C_PiperParamManager` is a singleton — limits are shared across instances
- V1 protocol code was deleted in v0.4.0; only V2 remains
- Two `pyproject.toml` exist: root-level is correct;
  `piper_sdk/pyproject.toml` is a stale artifact
- First frame of all feedback messages defaults to 0; wait ~25ms after
  `ConnectPort()` before reading firmware version
- `ConnectPort()` blocks briefly during `PiperInit` queries
- `DisconnectPort()` joins the ReadCan and CanMonitor threads (0.1s timeout)
- `SetPerformanceMetricsEnabled(False)` disables FPS tracking for a hot
  loop; when `minimal_feedback_mode=True` prefer `GetArmStateSnapshot()`
  over the wrapper-object getters in the hot loop. FPS variables registered
  while disabled still track once the counter is re-enabled (regardless of
  `add_variable` call order), and re-enabling restarts the FPS thread.
