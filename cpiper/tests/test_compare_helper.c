/**
 * @file test_compare_helper.c
 * @brief Helper for Python comparison tests — encodes/decodes messages and prints hex
 *
 * Usage:
 *   test_compare_helper encode <msg_type> <field1> <field2> ...
 *   test_compare_helper decode <can_id_hex> <b0> <b1> <b2> <b3> <b4> <b5> <b6> <b7>
 */
#include "cpiper/protocol.h"
#include "cpiper/messages.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(void) {
    fprintf(stderr, "Usage:\n");
    fprintf(stderr, "  encode motion_ctrl_1 <e_stop> <track> <teach>\n");
    fprintf(stderr, "  encode joint_ctrl_12 <j1_i32> <j2_i32>\n");
    fprintf(stderr, "  encode gripper_ctrl <angle_i32> <effort_i16> <status_u8> <zero_u8>\n");
    fprintf(stderr, "  encode mit_ctrl <pos_u16> <vel_u16> <kp_u16> <kd_u16> <t_u16>\n");
    fprintf(stderr, "  encode master_slave <linkage_u8> <fb_off_u8> <ctrl_off_u8> <link_off_u8>\n");
    fprintf(stderr, "  encode motor_enable <motor_u8> <enable_u8>\n");
    fprintf(stderr, "  encode motion_ctrl_2 <ctrl_mode> <move_mode> <spd_rate> <mit> <residence> <install>\n");
    fprintf(stderr, "  encode end_vel_acc <lin_vel_u16> <ang_vel_u16> <lin_acc_u16> <ang_acc_u16>\n");
    fprintf(stderr, "  encode crash_prot <j1> <j2> <j3> <j4> <j5> <j6>\n");
    fprintf(stderr, "  decode <can_id_hex> <b0> <b1> <b2> <b3> <b4> <b5> <b6> <b7>\n");
    fprintf(stderr, "  msg_type_to_can_id <type_id>\n");
    exit(1);
}

static void print_hex(const uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        printf("%02x", data[i]);
    }
    printf("\n");
}

int main(int argc, char **argv) {
    if (argc < 2) usage();

    if (strcmp(argv[1], "encode") == 0) {
        if (argc < 3) usage();
        const char *type = argv[2];

        cpiper_message_t msg;
        cpiper_msg_init(&msg);
        uint32_t can_id;
        uint8_t data[8] = {0};

        if (strcmp(type, "motion_ctrl_1") == 0) {
            if (argc != 6) usage();
            msg.type_ = CPPIER_MSG_MOTION_CTRL_1;
            msg.motion_ctrl_1.emergency_stop = (uint8_t)atoi(argv[3]);
            msg.motion_ctrl_1.track_ctrl = (uint8_t)atoi(argv[4]);
            msg.motion_ctrl_1.grag_teach_ctrl = (uint8_t)atoi(argv[5]);
        } else if (strcmp(type, "joint_ctrl_12") == 0) {
            if (argc != 5) usage();
            msg.type_ = CPPIER_MSG_JOINT_CTRL_12;
            msg.joint_ctrl.joint_1 = (int32_t)atoi(argv[3]);
            msg.joint_ctrl.joint_2 = (int32_t)atoi(argv[4]);
        } else if (strcmp(type, "gripper_ctrl") == 0) {
            if (argc != 7) usage();
            msg.type_ = CPPIER_MSG_GRIPPER_CTRL;
            msg.gripper_ctrl.angle = (int32_t)atoi(argv[3]);
            msg.gripper_ctrl.effort = (int16_t)atoi(argv[4]);
            msg.gripper_ctrl.status_code = (uint8_t)atoi(argv[5]);
            msg.gripper_ctrl.set_zero = (uint8_t)atoi(argv[6]);
        } else if (strcmp(type, "mit_ctrl") == 0) {
            if (argc != 8) usage();
            msg.type_ = CPPIER_MSG_MIT_CTRL_1;
            msg.mit_ctrl.pos_ref = (uint16_t)atoi(argv[3]);
            msg.mit_ctrl.vel_ref = (uint16_t)atoi(argv[4]);
            msg.mit_ctrl.kp = (uint16_t)atoi(argv[5]);
            msg.mit_ctrl.kd = (uint16_t)atoi(argv[6]);
            msg.mit_ctrl.t_ref = (uint16_t)atoi(argv[7]);
        } else if (strcmp(type, "master_slave") == 0) {
            if (argc != 7) usage();
            msg.type_ = CPPIER_MSG_MASTER_SLAVE_CONFIG;
            msg.ms_config.linkage_config = (uint8_t)atoi(argv[3]);
            msg.ms_config.feedback_offset = (uint8_t)atoi(argv[4]);
            msg.ms_config.ctrl_offset = (uint8_t)atoi(argv[5]);
            msg.ms_config.linkage_offset = (uint8_t)atoi(argv[6]);
        } else if (strcmp(type, "motor_enable") == 0) {
            if (argc != 5) usage();
            msg.type_ = CPPIER_MSG_MOTOR_ENABLE_DISABLE;
            msg.motor_enable.motor_num = (uint8_t)atoi(argv[3]);
            msg.motor_enable.enable_flag = (uint8_t)atoi(argv[4]);
        } else if (strcmp(type, "motion_ctrl_2") == 0) {
            if (argc != 9) usage();
            msg.type_ = CPPIER_MSG_MOTION_CTRL_2;
            msg.motion_ctrl_2.ctrl_mode = (uint8_t)atoi(argv[3]);
            msg.motion_ctrl_2.move_mode = (uint8_t)atoi(argv[4]);
            msg.motion_ctrl_2.move_spd_rate_ctrl = (uint8_t)atoi(argv[5]);
            msg.motion_ctrl_2.mit_mode = (uint8_t)atoi(argv[6]);
            msg.motion_ctrl_2.residence_time = (uint8_t)atoi(argv[7]);
            msg.motion_ctrl_2.installation_pos = (uint8_t)atoi(argv[8]);
        } else if (strcmp(type, "end_vel_acc") == 0) {
            if (argc != 7) usage();
            msg.type_ = CPPIER_MSG_END_VEL_ACC_PARAM_CONFIG;
            msg.end_vel_acc_config.end_max_linear_vel = (uint16_t)atoi(argv[3]);
            msg.end_vel_acc_config.end_max_angular_vel = (uint16_t)atoi(argv[4]);
            msg.end_vel_acc_config.end_max_linear_acc = (uint16_t)atoi(argv[5]);
            msg.end_vel_acc_config.end_max_angular_acc = (uint16_t)atoi(argv[6]);
        } else if (strcmp(type, "crash_prot") == 0) {
            if (argc != 9) usage();
            msg.type_ = CPPIER_MSG_CRASH_PROTECTION_CONFIG;
            msg.crash_config.joint_1_protection_level = (uint8_t)atoi(argv[3]);
            msg.crash_config.joint_2_protection_level = (uint8_t)atoi(argv[4]);
            msg.crash_config.joint_3_protection_level = (uint8_t)atoi(argv[5]);
            msg.crash_config.joint_4_protection_level = (uint8_t)atoi(argv[6]);
            msg.crash_config.joint_5_protection_level = (uint8_t)atoi(argv[7]);
            msg.crash_config.joint_6_protection_level = (uint8_t)atoi(argv[8]);
        } else {
            fprintf(stderr, "Unknown message type: %s\n", type);
            return 1;
        }

        if (!cpiper_encode(&msg, &can_id, data)) {
            fprintf(stderr, "encode failed\n");
            return 1;
        }
        printf("%08x:", can_id);
        print_hex(data, 8);

    } else if (strcmp(argv[1], "decode") == 0) {
        if (argc != 11) usage();
        uint32_t can_id = (uint32_t)strtol(argv[2], NULL, 16);
        uint8_t data[8];
        for (int i = 0; i < 8; i++) {
            data[i] = (uint8_t)strtol(argv[3 + i], NULL, 16);
        }

        cpiper_message_t msg;
        cpiper_msg_init(&msg);
        if (!cpiper_decode(can_id, data, &msg, 1.0)) {
            printf("FAIL\n");
            return 1;
        }
        printf("%d\n", msg.type_);

    } else if (strcmp(argv[1], "msg_type_to_can_id") == 0) {
        if (argc != 3) usage();
        int type_id = atoi(argv[2]);
        uint32_t can_id = cpiper_msg_type_to_can_id((cpiper_msg_type_t)type_id);
        printf("%08x\n", can_id);

    } else if (strcmp(argv[1], "can_id_to_msg_type") == 0) {
        if (argc != 3) usage();
        uint32_t can_id = (uint32_t)strtol(argv[2], NULL, 16);
        cpiper_msg_type_t type = cpiper_can_id_to_msg_type(can_id);
        printf("%d\n", type);

    } else if (strcmp(argv[1], "float_to_uint") == 0) {
        if (argc != 6) usage();
        float x = strtof(argv[2], NULL);
        float min = strtof(argv[3], NULL);
        float max = strtof(argv[4], NULL);
        int bits = atoi(argv[5]);
        uint32_t val = cpiper_float_to_uint(x, min, max, bits);
        printf("%u\n", val);

    } else {
        usage();
    }

    return 0;
}
