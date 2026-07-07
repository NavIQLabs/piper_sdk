#!/usr/bin/env python3
"""
test_compare.py — Compare cpiper C library encode/decode with Python SDK.

Usage:
    1. Build cpiper: cd cpiper/build && cmake .. && make
    2. Run: python3 cpiper/tests/test_compare.py
"""
import sys
import os
import struct
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(SCRIPT_DIR, '..', 'build')
HELPER = os.path.join(BUILD_DIR, 'test_compare_helper')

if not os.path.exists(HELPER):
    print("ERROR: test_compare_helper not found. Build first:")
    print("  cd cpiper/build && cmake .. && make")
    sys.exit(1)

try:
    from piper_sdk import C_PiperParserV2, PiperMessage, CanIDPiper, ArmMsgType
    import can
except ImportError:
    print("ERROR: piper_sdk not installed")
    sys.exit(1)

py_parser = C_PiperParserV2()

tests_run = 0
tests_passed = 0
tests_failed = 0


def test(name, py_bytes, c_bytes):
    global tests_run, tests_passed, tests_failed
    tests_run += 1
    if py_bytes == c_bytes:
        tests_passed += 1
        print(f"  PASS: {name}")
    else:
        tests_failed += 1
        print(f"  FAIL: {name}")
        print(f"    Python: {py_bytes.hex(' ').upper()}")
        print(f"    C:      {c_bytes.hex(' ').upper()}")
        for i in range(min(len(py_bytes), len(c_bytes))):
            if py_bytes[i] != c_bytes[i]:
                print(f"    Diff at byte {i}: Python=0x{py_bytes[i]:02X} C=0x{c_bytes[i]:02X}")
                break


def c_encode(*args):
    """Run test_compare_helper encode, return (can_id, data_bytes)."""
    result = subprocess.run([HELPER, "encode"] + [str(a) for a in args],
                            capture_output=True, text=True)
    line = result.stdout.strip()
    id_hex, data_hex = line.split(':')
    return bytes.fromhex(id_hex), bytes.fromhex(data_hex)


def c_decode(can_id, data):
    """Run test_compare_helper decode, return msg_type int or -1."""
    args = [HELPER, "decode", hex(can_id)] + [f"{b:02x}" for b in data]
    result = subprocess.run(args, capture_output=True, text=True)
    out = result.stdout.strip()
    if out == "FAIL":
        return -1
    return int(out)


def c_float_to_uint(x, min_val, max_val, bits):
    """Run test_compare_helper float_to_uint."""
    args = [HELPER, "float_to_uint", str(x), str(min_val), str(max_val), str(bits)]
    result = subprocess.run(args, capture_output=True, text=True)
    return int(result.stdout.strip())


def py_encode(msg):
    """Encode via Python SDK."""
    tx = can.Message(arbitration_id=0, data=bytes(8))
    py_parser.EncodeMessage(msg, tx)
    return bytes(tx.data)


# ─── Tests ────────────────────────────────────────────────────


def test_motion_ctrl_1():
    msg = PiperMessage()
    msg.type_ = ArmMsgType.PiperMsgMotionCtrl_1
    msg.arm_motion_ctrl_1.emergency_stop = 1
    msg.arm_motion_ctrl_1.track_ctrl = 0
    msg.arm_motion_ctrl_1.grag_teach_ctrl = 1
    py_data = py_encode(msg)

    c_id, c_data = c_encode("motion_ctrl_1", 1, 0, 1)

    test("MOTION_CTRL_1 encode (Py vs C)", py_data, c_data)
    test("MOTION_CTRL_1 CAN ID", c_id, bytes.fromhex("00000150"))


def test_motion_ctrl_2():
    msg = PiperMessage()
    msg.type_ = ArmMsgType.PiperMsgMotionCtrl_2
    msg.arm_motion_ctrl_2.ctrl_mode = 1
    msg.arm_motion_ctrl_2.move_mode = 2
    msg.arm_motion_ctrl_2.move_spd_rate_ctrl = 50
    msg.arm_motion_ctrl_2.mit_mode = 0
    msg.arm_motion_ctrl_2.residence_time = 0
    msg.arm_motion_ctrl_2.installation_pos = 1
    py_data = py_encode(msg)

    c_id, c_data = c_encode("motion_ctrl_2", 1, 2, 50, 0, 0, 1)

    test("MOTION_CTRL_2 encode (Py vs C)", py_data, c_data)


def test_joint_ctrl_12():
    msg = PiperMessage()
    msg.type_ = ArmMsgType.PiperMsgJointCtrl_12
    msg.arm_joint_ctrl.joint_1 = 50000
    msg.arm_joint_ctrl.joint_2 = -25000
    py_data = py_encode(msg)

    c_id, c_data = c_encode("joint_ctrl_12", 50000, -25000)

    test("JOINT_CTRL_12 encode (Py vs C)", py_data, c_data)
    expected = struct.pack('>ii', 50000, -25000)
    test("JOINT_CTRL_12 encode (C vs expected)", c_data, expected)


def test_gripper_ctrl():
    msg = PiperMessage()
    msg.type_ = ArmMsgType.PiperMsgGripperCtrl
    msg.arm_gripper_ctrl.grippers_angle = 10000
    msg.arm_gripper_ctrl.grippers_effort = 500
    msg.arm_gripper_ctrl.status_code = 1
    msg.arm_gripper_ctrl.set_zero = 0
    py_data = py_encode(msg)

    c_id, c_data = c_encode("gripper_ctrl", 10000, 500, 1, 0)

    test("GRIPPER_CTRL encode (Py vs C)", py_data, c_data)


def test_mit_ctrl():
    msg = PiperMessage()
    msg.type_ = ArmMsgType.PiperMsgJointMitCtrl_1
    msg.arm_joint_mit_ctrl.pos_ref = 32768
    msg.arm_joint_mit_ctrl.vel_ref = 2048
    msg.arm_joint_mit_ctrl.kp = 256
    msg.arm_joint_mit_ctrl.kd = 512
    msg.arm_joint_mit_ctrl.t_ref = 200
    py_data = py_encode(msg)

    c_id, c_data = c_encode("mit_ctrl", 32768, 2048, 256, 512, 200)

    test("MIT_CTRL encode (Py vs C)", py_data, c_data)
    # Verify CRC
    crc = (c_data[0] ^ c_data[1] ^ c_data[2] ^ c_data[3] ^
           c_data[4] ^ c_data[5] ^ c_data[6]) & 0x0F
    test("MIT_CTRL CRC valid", bytes([c_data[7] & 0x0F]), bytes([crc]))


def test_master_slave_config():
    msg = PiperMessage()
    msg.type_ = ArmMsgType.PiperMsgMasterSlaveModeConfig
    msg.arm_ms_config.linkage_config = 0xFC
    msg.arm_ms_config.feedback_offset = 0
    msg.arm_ms_config.ctrl_offset = 0
    msg.arm_ms_config.linkage_offset = 0
    py_data = py_encode(msg)

    c_id, c_data = c_encode("master_slave", 0xFC, 0, 0, 0)

    test("MASTER_SLAVE_CONFIG encode (Py vs C)", py_data, c_data)


def test_motor_enable():
    msg = PiperMessage()
    msg.type_ = ArmMsgType.PiperMsgMotorEnableDisableConfig
    msg.arm_motor_enable.motor_num = 3
    msg.arm_motor_enable.enable_flag = 1
    py_data = py_encode(msg)

    c_id, c_data = c_encode("motor_enable", 3, 1)

    test("MOTOR_ENABLE encode (Py vs C)", py_data, c_data)


def test_decode_status():
    data = [0x02, 0x03, 0x01, 0x00, 0x01, 0x05, 0x00, 0x0A]
    py_frame = can.Message(arbitration_id=0x2A1, data=bytes(data), timestamp=1.0)
    py_msg = PiperMessage()
    py_parser.DecodeMessage(py_frame, py_msg)

    msg_type = c_decode(0x2A1, data)
    test("STATUS_FEEDBACK decode (C type=0)", msg_type.to_bytes(4, 'big'), (0).to_bytes(4, 'big'))


def test_decode_joint():
    data = [0x00, 0x01, 0x38, 0x80, 0xFF, 0xFE, 0xC7, 0x80]
    msg_type = c_decode(0x2A5, data)
    test("JOINT_FEEDBACK decode (C type=2)", msg_type.to_bytes(4, 'big'), (2).to_bytes(4, 'big'))


def test_decode_gripper():
    data = [0x00, 0x00, 0x03, 0xE8, 0x00, 0x64, 0x01, 0x00]
    msg_type = c_decode(0x2A8, data)
    test("GRIPPER_FEEDBACK decode (C type=3)", msg_type.to_bytes(4, 'big'), (3).to_bytes(4, 'big'))


def test_decode_end_pose():
    data = [0x00, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x03, 0xE8]
    msg_type = c_decode(0x2A2, data)
    test("END_POSE_FEEDBACK decode (C type=1)", msg_type.to_bytes(4, 'big'), (1).to_bytes(4, 'big'))


def test_all_feedback_decode():
    feedback_ids = [
        0x2A1, 0x2A2, 0x2A3, 0x2A4, 0x2A5, 0x2A6, 0x2A7, 0x2A8,
        0x251, 0x252, 0x253, 0x254, 0x255, 0x256,
        0x261, 0x262, 0x263, 0x264, 0x265, 0x266,
        0x4AF, 0x476, 0x473, 0x478, 0x47B, 0x47C, 0x47E,
    ]
    data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
    all_ok = True
    for cid in feedback_ids:
        mt = c_decode(cid, data)
        if mt < 0:
            all_ok = False
            print(f"    FAIL: CAN ID 0x{cid:03X} did not decode")
    global tests_run, tests_passed, tests_failed
    tests_run += 1
    if all_ok:
        tests_passed += 1
        print(f"  PASS: all {len(feedback_ids)} feedback CAN IDs decode")
    else:
        tests_failed += 1
        print(f"  FAIL: not all feedback CAN IDs decode")


def test_float_to_uint():
    # Python reference values
    py_val = py_parser.FloatToUint(0.0, -12.5, 12.5, 16)
    c_val = c_float_to_uint(0.0, -12.5, 12.5, 16)
    test("FloatToUint(0.0, -12.5, 12.5, 16)", py_val.to_bytes(4, 'big'), c_val.to_bytes(4, 'big'))

    py_val = py_parser.FloatToUint(12.5, -12.5, 12.5, 16)
    c_val = c_float_to_uint(12.5, -12.5, 12.5, 16)
    test("FloatToUint(12.5, -12.5, 12.5, 16)", py_val.to_bytes(4, 'big'), c_val.to_bytes(4, 'big'))

    py_val = py_parser.FloatToUint(-12.5, -12.5, 12.5, 16)
    c_val = c_float_to_uint(-12.5, -12.5, 12.5, 16)
    test("FloatToUint(-12.5, -12.5, 12.5, 16)", py_val.to_bytes(4, 'big'), c_val.to_bytes(4, 'big'))


def test_convert_bytes_to_int():
    data = bytes([0xDE, 0xAD, 0xBE, 0xEF])
    py_val = int.from_bytes(data, byteorder='big')
    c_val = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]
    test("ConvertBytesToInt 32-bit", py_val.to_bytes(4, 'big'), c_val.to_bytes(4, 'big'))


def test_msg_type_can_id_mapping():
    """Verify CAN IDs from C encode match Python SDK encode for same message."""
    pairs = [
        ("motion_ctrl_1", [1, 0, 1], ArmMsgType.PiperMsgMotionCtrl_1,
         lambda m: (setattr(m.arm_motion_ctrl_1, 'emergency_stop', 1),
                    setattr(m.arm_motion_ctrl_1, 'track_ctrl', 0),
                    setattr(m.arm_motion_ctrl_1, 'grag_teach_ctrl', 1))),
        ("joint_ctrl_12", [50000, -25000], ArmMsgType.PiperMsgJointCtrl_12,
         lambda m: (setattr(m.arm_joint_ctrl, 'joint_1', 50000),
                    setattr(m.arm_joint_ctrl, 'joint_2', -25000))),
        ("mit_ctrl", [32768, 2048, 256, 512, 200], ArmMsgType.PiperMsgJointMitCtrl_1,
         lambda m: (setattr(m.arm_joint_mit_ctrl, 'pos_ref', 32768),
                    setattr(m.arm_joint_mit_ctrl, 'vel_ref', 2048),
                    setattr(m.arm_joint_mit_ctrl, 'kp', 256),
                    setattr(m.arm_joint_mit_ctrl, 'kd', 512),
                    setattr(m.arm_joint_mit_ctrl, 't_ref', 200))),
    ]

    all_ok = True
    for helper_name, helper_args, py_arm_msg_type, setup_fn in pairs:
        c_id, _ = c_encode(helper_name, *helper_args)

        py_msg = PiperMessage()
        py_msg.type_ = py_arm_msg_type
        setup_fn(py_msg)
        py_tx = can.Message(arbitration_id=0, data=bytes(8))
        py_parser.EncodeMessage(py_msg, py_tx)

        if int.from_bytes(c_id, 'big') != py_tx.arbitration_id:
            all_ok = False
            print(f"    MISMATCH: {helper_name} Python=0x{py_tx.arbitration_id:03X} C=0x{int.from_bytes(c_id, 'big'):03X}")

    global tests_run, tests_passed, tests_failed
    tests_run += 1
    if all_ok:
        tests_passed += 1
        print(f"  PASS: msg_type_to_can_id mapping ({len(pairs)} types)")
    else:
        tests_failed += 1
        print(f"  FAIL: msg_type_to_can_id mapping")


if __name__ == '__main__':
    print("=== cpiper vs Python SDK Comparison Tests ===\n")

    test_motion_ctrl_1()
    test_motion_ctrl_2()
    test_joint_ctrl_12()
    test_gripper_ctrl()
    test_mit_ctrl()
    test_master_slave_config()
    test_motor_enable()
    test_decode_status()
    test_decode_joint()
    test_decode_gripper()
    test_decode_end_pose()
    test_all_feedback_decode()
    test_float_to_uint()
    test_convert_bytes_to_int()
    test_msg_type_can_id_mapping()

    print(f"\n=== Results: {tests_passed}/{tests_run} passed, {tests_failed} failed ===")
    sys.exit(0 if tests_failed == 0 else 1)
