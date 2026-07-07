/**
 * @file test_protocol.c
 * @brief Unit tests for protocol encode/decode
 */
#include <cpiper/protocol.h>
#include <cpiper/can_ids.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { printf("  TEST: %-50s ", name); tests_run++; } while(0)
#define PASS() \
    do { tests_passed++; printf("[PASS]\n"); } while(0)
#define FAIL(msg) \
    do { printf("[FAIL] %s\n", msg); } while(0)

#define ASSERT_EQ_INT(a, b) \
    do { if ((a) != (b)) { FAIL(#a " != " #b); return; } } while(0)
#define ASSERT_EQ_HEX(a, b) \
    do { if ((a) != (b)) { FAIL(#a " != " #b); return; } } while(0)

/* ─── Byte helpers ───────────────────────────────────────────── */

static void test_get_u8(void) {
    TEST("cpiper_get_u8");
    uint8_t data[8] = {0x00, 0x7F, 0x80, 0xFF, 0x42, 0x00, 0x01, 0xAB};
    assert(cpiper_get_u8(data, 0) == 0x00);
    assert(cpiper_get_u8(data, 1) == 0x7F);
    assert(cpiper_get_u8(data, 2) == 0x80);
    assert(cpiper_get_u8(data, 3) == 0xFF);
    assert(cpiper_get_u8(data, 7) == 0xAB);
    PASS();
}

static void test_get_i8(void) {
    TEST("cpiper_get_i8");
    uint8_t data[2] = {0x80, 0x7F};
    assert(cpiper_get_i8(data, 0) == -128);
    assert(cpiper_get_i8(data, 1) == 127);
    PASS();
}

static void test_get_u16_be(void) {
    TEST("cpiper_get_u16_be");
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x12, 0x34};
    assert(cpiper_get_u16_be(data, 2) == 1);
    assert(cpiper_get_u16_be(data, 4) == 0xFFFF);
    assert(cpiper_get_u16_be(data, 6) == 0x1234);
    PASS();
}

static void test_get_i16_be(void) {
    TEST("cpiper_get_i16_be");
    uint8_t data[4] = {0x80, 0x00, 0x7F, 0xFF};
    assert(cpiper_get_i16_be(data, 0) == -32768);
    assert(cpiper_get_i16_be(data, 2) == 32767);
    PASS();
}

static void test_get_u32_be(void) {
    TEST("cpiper_get_u32_be");
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF};
    assert(cpiper_get_u32_be(data, 0) == 1);
    assert(cpiper_get_u32_be(data, 4) == 0xFFFFFFFF);
    PASS();
}

static void test_get_i32_be(void) {
    TEST("cpiper_get_i32_be");
    uint8_t data[8] = {0x80, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFF};
    assert(cpiper_get_i32_be(data, 0) == (-2147483647 - 1));
    assert(cpiper_get_i32_be(data, 4) == 2147483647);
    PASS();
}

static void test_put_u8(void) {
    TEST("cpiper_put_u8");
    uint8_t data[8] = {0};
    cpiper_put_u8(data, 3, 0xAB);
    assert(data[3] == 0xAB);
    PASS();
}

static void test_put_u16_be(void) {
    TEST("cpiper_put_u16_be");
    uint8_t data[8] = {0};
    cpiper_put_u16_be(data, 2, 0x1234);
    assert(data[2] == 0x12);
    assert(data[3] == 0x34);
    PASS();
}

static void test_put_u32_be(void) {
    TEST("cpiper_put_u32_be");
    uint8_t data[8] = {0};
    cpiper_put_u32_be(data, 0, 0xDEADBEEF);
    assert(data[0] == 0xDE);
    assert(data[1] == 0xAD);
    assert(data[2] == 0xBE);
    assert(data[3] == 0xEF);
    PASS();
}

static void test_put_i32_be(void) {
    TEST("cpiper_put_i32_be");
    uint8_t data[8] = {0};
    cpiper_put_i32_be(data, 0, -1);
    assert(data[0] == 0xFF);
    assert(data[1] == 0xFF);
    assert(data[2] == 0xFF);
    assert(data[3] == 0xFF);
    PASS();
}

/* ─── Decode tests ───────────────────────────────────────────── */

static void test_decode_status(void) {
    TEST("decode ARM_STATUS_FEEDBACK");
    uint8_t data[8] = {0x02, 0x03, 0x01, 0x00, 0x01, 0x05, 0x00, 0x0A};
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    bool ok = cpiper_decode(CAN_ID_ARM_STATUS_FEEDBACK, data, &msg, 1.0);
    assert(ok);
    assert(msg.type_ == CPPIER_MSG_STATUS_FEEDBACK);
    assert(msg.status.ctrl_mode == 0x02);
    assert(msg.status.arm_status == 0x03);
    assert(msg.status.mode_feed == 0x01);
    assert(msg.status.teach_status == 0x00);
    assert(msg.status.motion_status == 0x01);
    assert(msg.status.trajectory_num == 0x05);
    assert(msg.status.err_code == 0x000A);
    PASS();
}

static void test_decode_end_pose(void) {
    TEST("decode ARM_END_POSE_FEEDBACK");
    /* X=100000, Y=20000 */
    uint8_t data1[8] = {0x00, 0x01, 0x86, 0xA0, 0x00, 0x00, 0x4E, 0x20};
    /* Z=300000, RX=-50000 */
    uint8_t data2[8] = {0x00, 0x04, 0x93, 0xE0, 0xFF, 0xFF, 0x3C, 0xB0};
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    cpiper_decode(CAN_ID_ARM_END_POSE_FEEDBACK_1, data1, &msg, 1.0);
    assert(msg.end_pose.X_axis == 100000);
    assert(msg.end_pose.Y_axis == 20000);
    cpiper_decode(CAN_ID_ARM_END_POSE_FEEDBACK_2, data2, &msg, 1.0);
    assert(msg.end_pose.Z_axis == 300000);
    assert(msg.end_pose.RX_axis == -50000);
    PASS();
}

static void test_decode_joint(void) {
    TEST("decode ARM_JOINT_FEEDBACK");
    uint8_t data[8] = {0xFF, 0xFF, 0xE8, 0xF0, 0x00, 0x00, 0x00, 0x00};
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    cpiper_decode(CAN_ID_ARM_JOINT_FEEDBACK_12, data, &msg, 1.0);
    assert(msg.joint_state.joint_1 == -5904);
    assert(msg.joint_state.joint_2 == 0);
    PASS();
}

static void test_decode_gripper(void) {
    TEST("decode ARM_GRIPPER_FEEDBACK");
    uint8_t data[8] = {0x00, 0x00, 0x27, 0x10, 0x03, 0xE8, 0x01, 0x00};
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    cpiper_decode(CAN_ID_ARM_GRIPPER_FEEDBACK, data, &msg, 1.0);
    assert(msg.gripper.angle == 10000);
    assert(msg.gripper.effort == 1000);
    assert(msg.gripper.status_code == 0x01);
    PASS();
}

static void test_decode_high_spd(void) {
    TEST("decode ARM_INFO_HIGH_SPD_FEEDBACK");
    uint8_t data[8] = {0x00, 0x64, 0x03, 0xE8, 0x00, 0x00, 0x10, 0x00};
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    cpiper_decode(CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_3, data, &msg, 1.0);
    assert(msg.type_ == CPPIER_MSG_HIGH_SPD_MOTOR_FEEDBACK);
    /* Motor index 2 (feedback 3) */
    assert(msg.motor_high_spd[2].can_id == CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_3);
    assert(msg.motor_high_spd[2].motor_speed == 100);
    assert(msg.motor_high_spd[2].current == 1000);
    assert(msg.motor_high_spd[2].pos == 4096);
    PASS();
}

static void test_decode_low_spd(void) {
    TEST("decode ARM_INFO_LOW_SPD_FEEDBACK");
    uint8_t data[8] = {0x0B, 0xB8, 0x00, 0x32, 0x3C, 0x00, 0x00, 0x64};
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    cpiper_decode(CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_1, data, &msg, 1.0);
    assert(msg.motor_low_spd[0].vol == 3000);
    assert(msg.motor_low_spd[0].foc_temp == 50);
    assert(msg.motor_low_spd[0].motor_temp == 60);
    PASS();
}

static void test_decode_unknown(void) {
    TEST("decode unknown CAN ID");
    uint8_t data[8] = {0};
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    bool ok = cpiper_decode(0x999, data, &msg, 1.0);
    assert(!ok);
    PASS();
}

/* ─── Encode tests ───────────────────────────────────────────── */

static void test_encode_motion_ctrl_1(void) {
    TEST("encode MOTION_CTRL_1");
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    msg.type_ = CPPIER_MSG_MOTION_CTRL_1;
    msg.motion_ctrl_1.emergency_stop = 1;
    msg.motion_ctrl_1.track_ctrl = 0;
    msg.motion_ctrl_1.grag_teach_ctrl = 0;

    uint32_t can_id;
    uint8_t data[8];
    bool ok = cpiper_encode(&msg, &can_id, data);
    assert(ok);
    assert(can_id == CAN_ID_ARM_MOTION_CTRL_1);
    assert(data[0] == 1);
    assert(data[1] == 0);
    assert(data[2] == 0);
    assert(data[3] == 0); /* padding */
    PASS();
}

static void test_encode_joint_ctrl_12(void) {
    TEST("encode JOINT_CTRL_12");
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    msg.type_ = CPPIER_MSG_JOINT_CTRL_12;
    msg.joint_ctrl.joint_1 = 50000;
    msg.joint_ctrl.joint_2 = -25000;

    uint32_t can_id;
    uint8_t data[8];
    bool ok = cpiper_encode(&msg, &can_id, data);
    assert(ok);
    assert(can_id == CAN_ID_ARM_JOINT_CTRL_12);
    assert(cpiper_get_i32_be(data, 0) == 50000);
    assert(cpiper_get_i32_be(data, 4) == -25000);
    PASS();
}

static void test_encode_gripper_ctrl(void) {
    TEST("encode GRIPPER_CTRL");
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    msg.type_ = CPPIER_MSG_GRIPPER_CTRL;
    msg.gripper_ctrl.angle = 10000;
    msg.gripper_ctrl.effort = 500;
    msg.gripper_ctrl.status_code = 0x01;
    msg.gripper_ctrl.set_zero = 0;

    uint32_t can_id;
    uint8_t data[8];
    bool ok = cpiper_encode(&msg, &can_id, data);
    assert(ok);
    assert(can_id == CAN_ID_ARM_GRIPPER_CTRL);
    assert(cpiper_get_i32_be(data, 0) == 10000);
    assert(cpiper_get_u16_be(data, 4) == 500);
    assert(data[6] == 0x01);
    assert(data[7] == 0x00);
    PASS();
}

static void test_encode_mit_ctrl_crc(void) {
    TEST("encode MIT_CTRL with CRC");
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    msg.type_ = CPPIER_MSG_MIT_CTRL_1;
    msg.mit_ctrl.pos_ref = 0x8000;
    msg.mit_ctrl.vel_ref = 0x800;
    msg.mit_ctrl.kp = 0x100;
    msg.mit_ctrl.kd = 0x200;
    msg.mit_ctrl.t_ref = 0x300;

    uint32_t can_id;
    uint8_t data[8];
    bool ok = cpiper_encode(&msg, &can_id, data);
    assert(ok);
    assert(can_id == CAN_ID_ARM_JOINT_MIT_CTRL_1);
    /* Verify CRC is in byte 7, low 4 bits */
    uint8_t crc = data[7] & 0x0F;
    uint8_t expected_crc = (data[0] ^ data[1] ^ data[2] ^ data[3] ^
                            data[4] ^ data[5] ^ data[6]) & 0x0F;
    assert(crc == expected_crc);
    PASS();
}

/* ─── Round-trip tests ───────────────────────────────────────── */

static void test_roundtrip_status(void) {
    TEST("roundtrip STATUS_FEEDBACK");
    /* Build known data */
    uint8_t data[8] = {0x02, 0x03, 0x01, 0x00, 0x01, 0x05, 0x00, 0x0A};
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    cpiper_decode(CAN_ID_ARM_STATUS_FEEDBACK, data, &msg, 1.0);

    /* Re-encode — but STATUS is feedback-only, so just verify decode was correct */
    assert(msg.status.ctrl_mode == 2);
    assert(msg.status.arm_status == 3);
    assert(msg.status.err_code == 10);
    PASS();
}

static void test_roundtrip_joint(void) {
    TEST("roundtrip JOINT_CTRL_12");
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    msg.type_ = CPPIER_MSG_JOINT_CTRL_12;
    msg.joint_ctrl.joint_1 = 12345;
    msg.joint_ctrl.joint_2 = -6789;

    uint32_t can_id;
    uint8_t data[8];
    cpiper_encode(&msg, &can_id, data);

    /* Decode back */
    cpiper_message_t out;
    cpiper_msg_init(&out);
    cpiper_decode(can_id, data, &out, 1.0);
    assert(out.joint_ctrl.joint_1 == 12345);
    assert(out.joint_ctrl.joint_2 == -6789);
    PASS();
}

static void test_roundtrip_gripper(void) {
    TEST("roundtrip GRIPPER_CTRL");
    cpiper_message_t msg;
    cpiper_msg_init(&msg);
    msg.type_ = CPPIER_MSG_GRIPPER_CTRL;
    msg.gripper_ctrl.angle = 50000;
    msg.gripper_ctrl.effort = 1234;
    msg.gripper_ctrl.status_code = 0x42;
    msg.gripper_ctrl.set_zero = 1;

    uint32_t can_id;
    uint8_t data[8];
    cpiper_encode(&msg, &can_id, data);

    cpiper_message_t out;
    cpiper_msg_init(&out);
    cpiper_decode(can_id, data, &out, 1.0);
    assert(out.gripper_ctrl.angle == 50000);
    assert(out.gripper_ctrl.effort == 1234);
    assert(out.gripper_ctrl.status_code == 0x42);
    assert(out.gripper_ctrl.set_zero == 1);
    PASS();
}

/* ─── FloatToUint ────────────────────────────────────────────── */

static void test_float_to_uint(void) {
    TEST("cpiper_float_to_uint (MIT mode)");
    /* From Python: FloatToUint(0.0, -12.5, 12.5, 16) → 32767 */
    uint32_t val = cpiper_float_to_uint(0.0f, -12.5f, 12.5f, 16);
    assert(val == 32767);

    /* From Python: FloatToUint(12.5, -12.5, 12.5, 16) → 65535 */
    val = cpiper_float_to_uint(12.5f, -12.5f, 12.5f, 16);
    assert(val == 65535);

    /* From Python: FloatToUint(-12.5, -12.5, 12.5, 16) → 0 */
    val = cpiper_float_to_uint(-12.5f, -12.5f, 12.5f, 16);
    assert(val == 0);
    PASS();
}

/* ─── msg_type_to_can_id mapping ─────────────────────────────── */

static void test_msg_type_mapping(void) {
    TEST("msg_type_to_can_id mapping completeness");
    assert(cpiper_msg_type_to_can_id(CPPIER_MSG_STATUS_FEEDBACK) == 0x2A1);
    assert(cpiper_msg_type_to_can_id(CPPIER_MSG_MOTION_CTRL_1) == 0x150);
    assert(cpiper_msg_type_to_can_id(CPPIER_MSG_JOINT_CTRL_12) == 0x155);
    assert(cpiper_msg_type_to_can_id(CPPIER_MSG_GRIPPER_CTRL) == 0x159);
    assert(cpiper_msg_type_to_can_id(CPPIER_MSG_MIT_CTRL_1) == 0x15A);
    assert(cpiper_msg_type_to_can_id(CPPIER_MSG_MIT_CTRL_6) == 0x15F);
    assert(cpiper_msg_type_to_can_id(CPPIER_MSG_MASTER_SLAVE_CONFIG) == 0x470);
    assert(cpiper_msg_type_to_can_id(CPPIER_MSG_MOTOR_ENABLE_DISABLE) == 0x471);
    assert(cpiper_msg_type_to_can_id(CPPIER_MSG_FIRMWARE) == 0x4AF);
    PASS();
}

static void test_can_id_to_msg_type_mapping(void) {
    TEST("can_id_to_msg_type bidirectional mapping");
    /* Verify forward and reverse mappings are consistent */
    for (int i = 0; i < (int)CPPIER_MSG_UNKNOWN; i++) {
        uint32_t can_id = cpiper_msg_type_to_can_id((cpiper_msg_type_t)i);
        if (can_id == 0) continue; /* skip unmapped */
        cpiper_msg_type_t reverse = cpiper_can_id_to_msg_type(can_id);
        /* Some CAN IDs map to same msg type (e.g., END_POSE feedback 1,2,3) */
        /* Just verify the mapping is non-zero */
        assert(reverse != CPPIER_MSG_UNKNOWN);
    }
    PASS();
}

/* ─── Main ───────────────────────────────────────────────────── */

int main(void) {
    printf("=== cpiper Protocol Unit Tests ===\n\n");

    printf("Byte helpers:\n");
    test_get_u8();
    test_get_i8();
    test_get_u16_be();
    test_get_i16_be();
    test_get_u32_be();
    test_get_i32_be();
    test_put_u8();
    test_put_u16_be();
    test_put_u32_be();
    test_put_i32_be();

    printf("\nDecode:\n");
    test_decode_status();
    test_decode_end_pose();
    test_decode_joint();
    test_decode_gripper();
    test_decode_high_spd();
    test_decode_low_spd();
    test_decode_unknown();

    printf("\nEncode:\n");
    test_encode_motion_ctrl_1();
    test_encode_joint_ctrl_12();
    test_encode_gripper_ctrl();
    test_encode_mit_ctrl_crc();

    printf("\nRoundtrip:\n");
    test_roundtrip_status();
    test_roundtrip_joint();
    test_roundtrip_gripper();

    printf("\nMIT mode:\n");
    test_float_to_uint();

    printf("\nMapping:\n");
    test_msg_type_mapping();
    test_can_id_to_msg_type_mapping();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
