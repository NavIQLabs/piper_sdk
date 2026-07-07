/**
 * @file test_can.c
 * @brief CAN integration tests (requires vcan0)
 *
 * Run: sudo ./test_can
 * Setup: sudo modprobe vcan && sudo ip link add vcan0 type vcan && sudo ip link set up vcan0
 */
#include <cpiper/can.h>
#include <cpiper/protocol.h>
#include <cpiper/can_ids.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) \
    do { printf("  TEST: %-50s ", name); tests_run++; } while(0)
#define PASS() \
    do { tests_passed++; printf("[PASS]\n"); } while(0)
#define FAIL(msg) \
    do { printf("[FAIL] %s\n", msg); } while(0)

static void test_can_init(void) {
    TEST("CAN init on vcan0");
    cpiper_can_t can;
    int ret = cpiper_can_init(&can, "vcan0", false);
    if (ret < 0) {
        FAIL("cpiper_can_init failed (is vcan0 up?)");
        return;
    }
    assert(can.fd >= 0);
    assert(strcmp(can.name, "vcan0") == 0);
    cpiper_can_close(&can);
    PASS();
}

static void test_can_send_recv(void) {
    TEST("CAN send/recv loopback");
    cpiper_can_t can_tx, can_rx;
    if (cpiper_can_init(&can_tx, "vcan0", false) < 0) { FAIL("init tx"); return; }
    if (cpiper_can_init(&can_rx, "vcan0", false) < 0) { cpiper_can_close(&can_tx); FAIL("init rx"); return; }

    uint8_t tx_data[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};
    int ret = cpiper_can_send(&can_tx, 0x123, tx_data);
    assert(ret == 0);

    usleep(10000); /* 10ms */

    uint32_t rx_id;
    uint8_t rx_data[8];
    double timestamp;
    ret = cpiper_can_recv(&can_rx, &rx_id, rx_data, &timestamp);
    assert(ret == 1);
    assert(rx_id == 0x123);
    assert(memcmp(tx_data, rx_data, 8) == 0);

    cpiper_can_close(&can_tx);
    cpiper_can_close(&can_rx);
    PASS();
}

static void test_can_nonblocking(void) {
    TEST("CAN non-blocking recv returns 0 when no data");
    cpiper_can_t can;
    if (cpiper_can_init(&can, "vcan0", false) < 0) { FAIL("init"); return; }

    uint32_t rx_id;
    uint8_t rx_data[8];
    double timestamp;
    int ret = cpiper_can_recv(&can, &rx_id, rx_data, &timestamp);
    assert(ret == 0); /* No data available */

    cpiper_can_close(&can);
    PASS();
}

static void test_can_batch_recv(void) {
    TEST("CAN batch recv multiple frames");
    cpiper_can_t can_tx, can_rx;
    if (cpiper_can_init(&can_tx, "vcan0", false) < 0) { FAIL("init tx"); return; }
    if (cpiper_can_init(&can_rx, "vcan0", false) < 0) { cpiper_can_close(&can_tx); FAIL("init rx"); return; }

    /* Send 5 frames */
    for (int i = 0; i < 5; i++) {
        uint8_t data[8] = {0};
        data[0] = (uint8_t)i;
        cpiper_can_send(&can_tx, 0x200 + i, data);
    }

    usleep(20000); /* 20ms */

    /* Receive all 5 */
    int count = 0;
    for (int i = 0; i < 10; i++) {
        uint32_t rx_id;
        uint8_t rx_data[8];
        double ts;
        int ret = cpiper_can_recv(&can_rx, &rx_id, rx_data, &ts);
        if (ret > 0) {
            count++;
            assert(rx_id >= 0x200 && rx_id <= 0x204);
        } else {
            break;
        }
    }
    assert(count == 5);

    cpiper_can_close(&can_tx);
    cpiper_can_close(&can_rx);
    PASS();
}

static void test_can_piper_encode_decode(void) {
    TEST("CAN: encode ARM_STATUS → send → recv → decode");
    cpiper_can_t can_tx, can_rx;
    if (cpiper_can_init(&can_tx, "vcan0", false) < 0) { FAIL("init tx"); return; }
    if (cpiper_can_init(&can_rx, "vcan0", false) < 0) { cpiper_can_close(&can_tx); FAIL("init rx"); return; }

    /* Encode ARM_STATUS_FEEDBACK (feedback msg — can be both encoded and decoded) */
    cpiper_message_t tx_msg;
    cpiper_msg_init(&tx_msg);
    tx_msg.type_ = CPPIER_MSG_STATUS_FEEDBACK;
    tx_msg.status.ctrl_mode = 1;
    tx_msg.status.arm_status = 2;
    tx_msg.status.motion_status = 3;
    tx_msg.status.trajectory_num = 5;

    uint32_t can_id;
    uint8_t data[8];
    assert(cpiper_encode(&tx_msg, &can_id, data));

    cpiper_can_send(&can_tx, can_id, data);
    usleep(10000);

    /* Recv and decode */
    uint32_t rx_id;
    uint8_t rx_data[8];
    double ts;
    int ret = cpiper_can_recv(&can_rx, &rx_id, rx_data, &ts);
    assert(ret == 1);
    assert(rx_id == CAN_ID_ARM_STATUS_FEEDBACK);

    cpiper_message_t rx_msg;
    cpiper_msg_init(&rx_msg);
    assert(cpiper_decode(rx_id, rx_data, &rx_msg, ts));
    assert(rx_msg.type_ == CPPIER_MSG_STATUS_FEEDBACK);
    assert(rx_msg.status.ctrl_mode == 1);
    assert(rx_msg.status.arm_status == 2);
    assert(rx_msg.status.motion_status == 3);
    assert(rx_msg.status.trajectory_num == 5);

    cpiper_can_close(&can_tx);
    cpiper_can_close(&can_rx);
    PASS();
}

static void test_can_500hz_stress(void) {
    TEST("CAN: 500Hz stress test (1000 frames)");
    cpiper_can_t can_tx, can_rx;
    if (cpiper_can_init(&can_tx, "vcan0", false) < 0) { FAIL("init tx"); return; }
    if (cpiper_can_init(&can_rx, "vcan0", false) < 0) { cpiper_can_close(&can_tx); FAIL("init rx"); return; }

    int sent = 0, received = 0;
    for (int i = 0; i < 1000; i++) {
        uint8_t data[8] = {0};
        data[0] = (uint8_t)(i & 0xFF);
        data[1] = (uint8_t)((i >> 8) & 0xFF);
        cpiper_can_send(&can_tx, 0x300, data);
        sent++;

        usleep(2000); /* ~500Hz */

        /* Try to receive */
        uint32_t rx_id;
        uint8_t rx_data[8];
        double ts;
        while (cpiper_can_recv(&can_rx, &rx_id, rx_data, &ts) > 0) {
            received++;
        }
    }

    /* Drain remaining */
    uint32_t rx_id;
    uint8_t rx_data[8];
    double ts;
    while (cpiper_can_recv(&can_rx, &rx_id, rx_data, &ts) > 0) {
        received++;
    }

    printf("\n    Sent: %d, Received: %d  ", sent, received);
    assert(received > 0);
    PASS();

    cpiper_can_close(&can_tx);
    cpiper_can_close(&can_rx);
}

/* ─── Main ───────────────────────────────────────────────────── */

int main(void) {
    printf("=== cpiper CAN Integration Tests ===\n");
    printf("    (Requires vcan0: sudo modprobe vcan && "
           "sudo ip link add vcan0 type vcan && "
           "sudo ip link set up vcan0)\n\n");

    test_can_init();
    test_can_send_recv();
    test_can_nonblocking();
    test_can_batch_recv();
    test_can_piper_encode_decode();
    test_can_500hz_stress();

    printf("\n=== Results: %d/%d passed ===\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
