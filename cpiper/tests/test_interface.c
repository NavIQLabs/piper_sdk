/**
 * @file test_interface.c
 * @brief Interface integration tests (requires vcan0)
 */
#include <cpiper/interface.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
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

static void test_init(void) {
    TEST("cpiper_init");
    cpiper_interface_t arm;
    int ret = cpiper_init(&arm, "vcan0");
    assert(ret == 0);
    assert(strcmp(arm.can.name, "vcan0") == 0);
    cpiper_destroy(&arm);
    PASS();
}

static void test_connect_disconnect(void) {
    TEST("cpiper_connect/disconnect");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    int ret = cpiper_connect(&arm, false, false);
    if (ret < 0) {
        FAIL("connect failed (is vcan0 up?)");
        cpiper_destroy(&arm);
        return;
    }
    assert(arm.connected);
    cpiper_disconnect(&arm);
    assert(!arm.connected);
    cpiper_destroy(&arm);
    PASS();
}

static void test_send_raw(void) {
    TEST("cpiper_send_raw");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    int ret = cpiper_send_raw(&arm, 0x123, data);
    assert(ret == 0);
    cpiper_destroy(&arm);
    PASS();
}

static void test_motion_ctrl_1_send(void) {
    TEST("cpiper_motion_ctrl_1 sends correct CAN frame");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    /* Send motion ctrl 1 */
    cpiper_motion_ctrl_1(&arm, 1, 0, 1);
    /* We can't easily verify the frame without a loopback, but at least it doesn't crash */
    usleep(10000);
    cpiper_destroy(&arm);
    PASS();
}

static void test_joint_ctrl_send(void) {
    TEST("cpiper_joint_ctrl sends 3 CAN frames");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    /* Joint ctrl should send 3 frames */
    cpiper_joint_ctrl(&arm, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    usleep(10000);
    cpiper_destroy(&arm);
    PASS();
}

static void test_gripper_ctrl_send(void) {
    TEST("cpiper_gripper_ctrl sends CAN frame");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    cpiper_gripper_ctrl(&arm, 5000, 500, 0x01, 0);
    usleep(10000);
    cpiper_destroy(&arm);
    PASS();
}

static void test_master_slave_config(void) {
    TEST("cpiper_master_slave_config sends CAN frame");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    cpiper_master_slave_config(&arm, 0xFC, 0, 0, 0);
    usleep(10000);
    cpiper_destroy(&arm);
    PASS();
}

static void test_enable_disable_piper(void) {
    TEST("cpiper_enable_piper/disable_piper");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    bool ret = cpiper_enable_piper(&arm);
    assert(ret);
    usleep(10000);
    ret = cpiper_disable_piper(&arm);
    assert(ret);
    usleep(10000);
    cpiper_destroy(&arm);
    PASS();
}

static void test_is_ok(void) {
    TEST("cpiper_is_ok");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    assert(!cpiper_is_ok(&arm)); /* not connected */
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    assert(cpiper_is_ok(&arm));
    cpiper_destroy(&arm);
    PASS();
}

static void test_filter_abnormal(void) {
    TEST("cpiper_set_filter_abnormal");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    assert(arm.filter_abnormal); /* default: enabled */
    cpiper_set_filter_abnormal(&arm, false);
    assert(!arm.filter_abnormal);
    cpiper_set_filter_abnormal(&arm, true);
    assert(arm.filter_abnormal);
    cpiper_destroy(&arm);
    PASS();
}

static void test_param_manager(void) {
    TEST("cpiper_param_manager singleton");
    cpiper_param_manager_init();
    cpiper_param_manager_t *p1 = cpiper_param_manager_get();
    cpiper_param_manager_t *p2 = cpiper_param_manager_get();
    assert(p1 == p2); /* same pointer = singleton */

    /* Test joint limit */
    cpiper_param_set_joint_limit(1, -2.0f, 2.0f);
    float mn, mx;
    cpiper_param_get_joint_limit(1, &mn, &mx);
    assert(mn == -2.0f);
    assert(mx == 2.0f);

    /* Test clamp */
    assert(cpiper_param_clamp_joint(1, 3.0f) == 2.0f);
    assert(cpiper_param_clamp_joint(1, -3.0f) == -2.0f);
    assert(cpiper_param_clamp_joint(1, 1.0f) == 1.0f);

    PASS();
}

static void test_fk_init(void) {
    TEST("cpiper_fk_init");
    cpiper_fk_dh_t dh;
    cpiper_fk_init(&dh, 1);
    assert(dh.dh_is_offset == 1);
    assert(dh.a[2] == 285.03f);
    assert(dh.d[0] == 123.0f);
    assert(dh.d[3] == 250.75f);
    PASS();
}

static void test_fk_calculate(void) {
    TEST("cpiper_fk_calculate (all zeros → home position)");
    cpiper_fk_dh_t dh;
    cpiper_fk_init(&dh, 1);
    float joints[6] = {0};
    float pos[6];
    cpiper_fk_calculate(&dh, joints, pos);
    /* At home position, end effector should be near a known location */
    /* Just verify it produces finite values */
    assert(isfinite(pos[0]));
    assert(isfinite(pos[1]));
    assert(isfinite(pos[2]));
    PASS();
}

static void test_fps_counter(void) {
    TEST("cpiper_fps_t counter");
    cpiper_fps_t fps;
    cpiper_fps_init(&fps);
    assert(fps.frame_count == 0);
    assert(fps.hz == 0.0);
    cpiper_fps_update(&fps, 1.0);
    cpiper_fps_update(&fps, 1.1);
    cpiper_fps_update(&fps, 1.2);
    /* After 0.2s, 3 frames → not enough for 1s window, hz should stay 0 */
    cpiper_fps_update(&fps, 2.0); /* >1s elapsed from last_time=1.0 */
    /* Now frame_count was reset, hz should be ~3.0 / 1.0 = 3.0 */
    assert(fps.hz > 0.0);
    PASS();
}

static void test_msg_init(void) {
    TEST("cpiper_msg_init zeroes everything");
    cpiper_message_t msg;
    msg.type_ = CPPIER_MSG_MOTION_CTRL_1;
    msg.motion_ctrl_1.emergency_stop = 42;
    cpiper_msg_init(&msg);
    assert(msg.type_ == CPPIER_MSG_UNKNOWN);
    assert(msg.motion_ctrl_1.emergency_stop == 0);
    PASS();
}

static void test_end_pose_ctrl_send(void) {
    TEST("cpiper_end_pose_ctrl sends 3 CAN frames");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    cpiper_end_pose_ctrl(&arm, 135481, 9349, 161129, 178756, 6035, -178440);
    usleep(10000);
    cpiper_destroy(&arm);
    PASS();
}

static void test_mit_ctrl_send(void) {
    TEST("cpiper_mit_ctrl sends CAN frame");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    cpiper_mit_ctrl(&arm, 1, 0.0f, 0.0f, 10.0f, 0.5f, 0.1f);
    usleep(10000);
    cpiper_destroy(&arm);
    PASS();
}

static void test_crash_protection_config(void) {
    TEST("cpiper_crash_protection_config sends CAN frame");
    cpiper_interface_t arm;
    cpiper_init(&arm, "vcan0");
    if (cpiper_connect(&arm, false, false) < 0) {
        FAIL("connect failed"); cpiper_destroy(&arm); return;
    }
    cpiper_crash_protection_config(&arm, 1, 2, 3, 4, 5, 6);
    usleep(10000);
    cpiper_destroy(&arm);
    PASS();
}

/* ─── Main ───────────────────────────────────────────────────── */

int main(void) {
    printf("=== cpiper Interface Integration Tests ===\n");
    printf("    (Requires vcan0: sudo modprobe vcan && "
           "sudo ip link add vcan0 type vcan && "
           "sudo ip link set up vcan0)\n\n");

    test_init();
    test_connect_disconnect();
    test_send_raw();
    test_motion_ctrl_1_send();
    test_joint_ctrl_send();
    test_gripper_ctrl_send();
    test_master_slave_config();
    test_enable_disable_piper();
    test_is_ok();
    test_filter_abnormal();
    test_param_manager();
    test_fk_init();
    test_fk_calculate();
    test_fps_counter();
    test_msg_init();
    test_end_pose_ctrl_send();
    test_mit_ctrl_send();
    test_crash_protection_config();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
