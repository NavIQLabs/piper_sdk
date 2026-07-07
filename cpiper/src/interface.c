/**
 * @file interface.c
 * @brief High-level API for Piper robot arm control
 */
#include "cpiper/interface.h"
#include "cpiper/protocol.h"
#include "cpiper/can_ids.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

/* ─── Helpers ────────────────────────────────────────────────── */

static double get_time_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* ─── Receive thread ─────────────────────────────────────────── */

static void *recv_thread_func(void *arg) {
    cpiper_interface_t *p = (cpiper_interface_t *)arg;
    while (1) {
        pthread_mutex_lock(&p->stop_mutex);
        if (p->stop_flag) {
            pthread_mutex_unlock(&p->stop_mutex);
            break;
        }
        pthread_mutex_unlock(&p->stop_mutex);

        cpiper_poll(p);
        /* Small sleep to avoid busy-waiting (epoll in cpiper_can_recv handles blocking) */
        usleep(100);
    }
    return NULL;
}

/* ─── Internal: send a pre-built tx_msg ──────────────────────── */

static void send_msg(cpiper_interface_t *p) {
    uint32_t can_id;
    uint8_t data[8];
    if (cpiper_encode(&p->tx_msg, &can_id, data)) {
        cpiper_can_send(&p->can, can_id, data);
    }
}

/* ─── Parse callback (called per received frame) ─────────────── */

static void parse_frame(cpiper_interface_t *p, uint32_t can_id,
                        const uint8_t data[8], double timestamp) {
    cpiper_message_t msg;
    cpiper_msg_init(&msg);

    if (!cpiper_decode(can_id, data, &msg, timestamp))
        return;

    cpiper_fps_update(&p->fps_total, get_time_sec());

    pthread_mutex_lock(&p->fb_mutex);

    switch (msg.type_) {
    case CPPIER_MSG_STATUS_FEEDBACK:
        p->fb_status = msg.status;
        cpiper_fps_update(&p->fps_status, timestamp);
        break;
    case CPPIER_MSG_END_POSE_FEEDBACK:
        p->fb_end_pose = msg.end_pose;
        cpiper_fps_update(&p->fps_end_pose, timestamp);
        break;
    case CPPIER_MSG_JOINT_FEEDBACK:
        p->fb_joint_state = msg.joint_state;
        cpiper_fps_update(&p->fps_joint, timestamp);
        if (p->fk_enabled) {
            float joints[6];
            joints[0] = (float)p->fb_joint_state.joint_1 * 0.001f * 3.14159f / 180.0f;
            joints[1] = (float)p->fb_joint_state.joint_2 * 0.001f * 3.14159f / 180.0f;
            joints[2] = (float)p->fb_joint_state.joint_3 * 0.001f * 3.14159f / 180.0f;
            joints[3] = (float)p->fb_joint_state.joint_4 * 0.001f * 3.14159f / 180.0f;
            joints[4] = (float)p->fb_joint_state.joint_5 * 0.001f * 3.14159f / 180.0f;
            joints[5] = (float)p->fb_joint_state.joint_6 * 0.001f * 3.14159f / 180.0f;
            cpiper_fk_calculate(&p->fk_dh, joints, p->fk_pos);
        }
        break;
    case CPPIER_MSG_GRIPPER_FEEDBACK:
        p->fb_gripper = msg.gripper;
        cpiper_fps_update(&p->fps_gripper, timestamp);
        break;
    case CPPIER_MSG_HIGH_SPD_MOTOR_FEEDBACK: {
        for (int i = 0; i < 6; i++) {
            if (msg.motor_high_spd[i].can_id != 0) {
                p->fb_motor_high_spd[i] = msg.motor_high_spd[i];
            }
        }
        cpiper_fps_update(&p->fps_high_spd, timestamp);
        break;
    }
    case CPPIER_MSG_LOW_SPD_MOTOR_FEEDBACK: {
        for (int i = 0; i < 6; i++) {
            if (msg.motor_low_spd[i].can_id != 0) {
                p->fb_motor_low_spd[i] = msg.motor_low_spd[i];
            }
        }
        cpiper_fps_update(&p->fps_low_spd, timestamp);
        break;
    }
    case CPPIER_MSG_FIRMWARE:
        p->fb_firmware = msg.firmware;
        break;
    case CPPIER_MSG_RESP_SET_INSTRUCTION:
        p->fb_resp_instruction = msg.resp_instruction;
        break;
    case CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD:
        p->fb_motor_angle_limit = msg.motor_angle_limit;
        break;
    case CPPIER_MSG_END_VEL_ACC_PARAM:
        p->fb_end_vel_acc_param = msg.end_vel_acc_param;
        break;
    case CPPIER_MSG_CRASH_PROTECTION_FEEDBACK:
        p->fb_crash_protection = msg.crash_protection;
        break;
    case CPPIER_MSG_MOTOR_MAX_ACC_LIMIT:
        p->fb_motor_max_acc = msg.motor_max_acc;
        break;
    case CPPIER_MSG_GRIPPER_TEACHING_PARAM_FEEDBACK:
        p->fb_gripper_teaching_param = msg.gripper_teaching_param;
        break;
    default:
        break;
    }

    p->last_recv_time = timestamp;
    pthread_mutex_unlock(&p->fb_mutex);
}

/* ─── Lifecycle ──────────────────────────────────────────────── */

int cpiper_init(cpiper_interface_t *p, const char *can_name) {
    memset(p, 0, sizeof(cpiper_interface_t));
    strncpy(p->can.name, can_name, CPIPER_MAX_CAN_NAME - 1);
    pthread_mutex_init(&p->fb_mutex, NULL);
    pthread_mutex_init(&p->stop_mutex, NULL);
    p->filter_abnormal = true;
    p->fk_enabled = false;
    cpiper_fk_init(&p->fk_dh, 1);

    cpiper_fps_init(&p->fps_status);
    cpiper_fps_init(&p->fps_end_pose);
    cpiper_fps_init(&p->fps_joint);
    cpiper_fps_init(&p->fps_gripper);
    cpiper_fps_init(&p->fps_high_spd);
    cpiper_fps_init(&p->fps_low_spd);
    cpiper_fps_init(&p->fps_total);

    return 0;
}

int cpiper_connect(cpiper_interface_t *p, bool do_piper_init,
                   bool start_thread) {
    int ret = cpiper_can_init(&p->can, p->can.name, true);
    if (ret < 0) return ret;
    p->connected = true;
    p->is_ok = true;

    if (do_piper_init) {
        cpiper_search_all_motor_max_angle_spd(p);
        cpiper_search_all_motor_max_acc_limit(p);
        cpiper_search_firmware_version(p);
    }

    if (start_thread) {
        p->stop_flag = false;
        p->thread_running = true;
        int tr = pthread_create(&p->recv_thread, NULL, recv_thread_func, p);
        if (tr != 0) {
            fprintf(stderr, "cpiper: pthread_create failed\n");
            p->thread_running = false;
            return -10;
        }
    }

    return 0;
}

void cpiper_disconnect(cpiper_interface_t *p) {
    if (p->thread_running) {
        pthread_mutex_lock(&p->stop_mutex);
        p->stop_flag = true;
        pthread_mutex_unlock(&p->stop_mutex);
        pthread_join(p->recv_thread, NULL);
        p->thread_running = false;
    }
    cpiper_can_close(&p->can);
    p->connected = false;
}

void cpiper_destroy(cpiper_interface_t *p) {
    cpiper_disconnect(p);
    pthread_mutex_destroy(&p->fb_mutex);
    pthread_mutex_destroy(&p->stop_mutex);
}

/* ─── Poll ───────────────────────────────────────────────────── */

void cpiper_poll(cpiper_interface_t *p) {
    uint32_t can_id;
    uint8_t data[8];
    double timestamp;
    int ret;
    /* Read multiple frames per call */
    while ((ret = cpiper_can_recv(&p->can, can_id ? NULL : &can_id,
                                  data, &timestamp)) > 0) {
        parse_frame(p, can_id, data, timestamp);
    }
    (void)ret;
}

/* ─── Send commands ──────────────────────────────────────────── */

void cpiper_motion_ctrl_1(cpiper_interface_t *p,
                          uint8_t e_stop, uint8_t track, uint8_t teach) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_MOTION_CTRL_1;
    p->tx_msg.motion_ctrl_1.emergency_stop = e_stop;
    p->tx_msg.motion_ctrl_1.track_ctrl = track;
    p->tx_msg.motion_ctrl_1.grag_teach_ctrl = teach;
    send_msg(p);
}

void cpiper_motion_ctrl_2(cpiper_interface_t *p,
                          uint8_t ctrl_mode, uint8_t move_mode,
                          uint8_t spd, uint8_t mit, uint8_t res_time) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_MOTION_CTRL_2;
    p->tx_msg.motion_ctrl_2.ctrl_mode = ctrl_mode;
    p->tx_msg.motion_ctrl_2.move_mode = move_mode;
    p->tx_msg.motion_ctrl_2.move_spd_rate_ctrl = spd;
    p->tx_msg.motion_ctrl_2.mit_mode = mit;
    p->tx_msg.motion_ctrl_2.residence_time = res_time;
    p->tx_msg.motion_ctrl_2.installation_pos = 0;
    send_msg(p);

    pthread_mutex_lock(&p->fb_mutex);
    p->tx_motion_ctrl_2 = p->tx_msg.motion_ctrl_2;
    pthread_mutex_unlock(&p->fb_mutex);
}

void cpiper_mode_ctrl(cpiper_interface_t *p,
                      uint8_t ctrl_mode, uint8_t move_mode,
                      uint8_t spd, uint8_t mit) {
    cpiper_motion_ctrl_2(p, ctrl_mode, move_mode, spd, mit, 0);
}

void cpiper_emergency_stop(cpiper_interface_t *p, uint8_t e_stop) {
    cpiper_motion_ctrl_1(p, e_stop, 0, 0);
}

void cpiper_reset(cpiper_interface_t *p) {
    cpiper_emergency_stop(p, 0);
}

void cpiper_joint_ctrl(cpiper_interface_t *p,
                       float j1, float j2, float j3,
                       float j4, float j5, float j6) {
    /* SDK joint limit clamping */
    j1 = cpiper_param_clamp_joint(1, j1);
    j2 = cpiper_param_clamp_joint(2, j2);
    j3 = cpiper_param_clamp_joint(3, j3);
    j4 = cpiper_param_clamp_joint(4, j4);
    j5 = cpiper_param_clamp_joint(5, j5);
    j6 = cpiper_param_clamp_joint(6, j6);

    /* Convert radians to 0.001° */
    int32_t j1_raw = (int32_t)(j1 * 180.0f * 1000.0f / 3.14159f);
    int32_t j2_raw = (int32_t)(j2 * 180.0f * 1000.0f / 3.14159f);
    int32_t j3_raw = (int32_t)(j3 * 180.0f * 1000.0f / 3.14159f);
    int32_t j4_raw = (int32_t)(j4 * 180.0f * 1000.0f / 3.14159f);
    int32_t j5_raw = (int32_t)(j5 * 180.0f * 1000.0f / 3.14159f);
    int32_t j6_raw = (int32_t)(j6 * 180.0f * 1000.0f / 3.14159f);

    /* Send 3 frames: 12, 34, 56 */
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_JOINT_CTRL_12;
    p->tx_msg.joint_ctrl.joint_1 = j1_raw;
    p->tx_msg.joint_ctrl.joint_2 = j2_raw;
    send_msg(p);

    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_JOINT_CTRL_34;
    p->tx_msg.joint_ctrl.joint_3 = j3_raw;
    p->tx_msg.joint_ctrl.joint_4 = j4_raw;
    send_msg(p);

    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_JOINT_CTRL_56;
    p->tx_msg.joint_ctrl.joint_5 = j5_raw;
    p->tx_msg.joint_ctrl.joint_6 = j6_raw;
    send_msg(p);

    pthread_mutex_lock(&p->fb_mutex);
    p->tx_joint_ctrl.joint_1 = j1_raw;
    p->tx_joint_ctrl.joint_2 = j2_raw;
    p->tx_joint_ctrl.joint_3 = j3_raw;
    p->tx_joint_ctrl.joint_4 = j4_raw;
    p->tx_joint_ctrl.joint_5 = j5_raw;
    p->tx_joint_ctrl.joint_6 = j6_raw;
    pthread_mutex_unlock(&p->fb_mutex);
}

void cpiper_end_pose_ctrl(cpiper_interface_t *p,
                          int32_t x, int32_t y, int32_t z,
                          int32_t rx, int32_t ry, int32_t rz) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_MOTION_CTRL_CARTESIAN_1;
    p->tx_msg.cartesian.X_axis = x;
    p->tx_msg.cartesian.Y_axis = y;
    send_msg(p);

    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_MOTION_CTRL_CARTESIAN_2;
    p->tx_msg.cartesian.Z_axis = z;
    p->tx_msg.cartesian.RX_axis = rx;
    send_msg(p);

    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_MOTION_CTRL_CARTESIAN_3;
    p->tx_msg.cartesian.RY_axis = ry;
    p->tx_msg.cartesian.RZ_axis = rz;
    send_msg(p);
}

void cpiper_move_c_axis_update(cpiper_interface_t *p, uint8_t instruction_num) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_CIRCULAR_CTRL;
    p->tx_msg.circular_ctrl.instruction_num = instruction_num;
    send_msg(p);
}

void cpiper_gripper_ctrl(cpiper_interface_t *p,
                         int32_t angle, int16_t effort,
                         uint8_t status_code, uint8_t set_zero) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_GRIPPER_CTRL;
    p->tx_msg.gripper_ctrl.angle = angle;
    p->tx_msg.gripper_ctrl.effort = effort;
    p->tx_msg.gripper_ctrl.status_code = status_code;
    p->tx_msg.gripper_ctrl.set_zero = set_zero;
    send_msg(p);

    pthread_mutex_lock(&p->fb_mutex);
    p->tx_gripper_ctrl.angle = angle;
    p->tx_gripper_ctrl.effort = effort;
    p->tx_gripper_ctrl.status_code = status_code;
    p->tx_gripper_ctrl.set_zero = set_zero;
    pthread_mutex_unlock(&p->fb_mutex);
}

void cpiper_master_slave_config(cpiper_interface_t *p,
                                uint8_t linkage, uint8_t fb_off,
                                uint8_t ctrl_off, uint8_t link_off) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_MASTER_SLAVE_CONFIG;
    p->tx_msg.ms_config.linkage_config = linkage;
    p->tx_msg.ms_config.feedback_offset = fb_off;
    p->tx_msg.ms_config.ctrl_offset = ctrl_off;
    p->tx_msg.ms_config.linkage_offset = link_off;
    send_msg(p);
}

void cpiper_enable_arm(cpiper_interface_t *p, uint8_t motor_num, uint8_t flag) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_MOTOR_ENABLE_DISABLE;
    p->tx_msg.motor_enable.motor_num = motor_num;
    p->tx_msg.motor_enable.enable_flag = flag;
    send_msg(p);
}

void cpiper_disable_arm(cpiper_interface_t *p, uint8_t motor_num, uint8_t flag) {
    cpiper_enable_arm(p, motor_num, flag);
}

bool cpiper_enable_piper(cpiper_interface_t *p) {
    cpiper_enable_arm(p, 0x3F, 1);
    return true;
}

bool cpiper_disable_piper(cpiper_interface_t *p) {
    cpiper_enable_arm(p, 0x3F, 0);
    return true;
}

void cpiper_search_motor_max_angle_spd(cpiper_interface_t *p,
                                       uint8_t motor_num,
                                       uint8_t search_content) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_SEARCH_MOTOR_MAX_ANGLE_SPD;
    p->tx_msg.search_motor.motor_num = motor_num;
    p->tx_msg.search_motor.search_content = search_content;
    send_msg(p);
}

void cpiper_search_all_motor_max_angle_spd(cpiper_interface_t *p) {
    for (int i = 1; i <= 6; i++) {
        cpiper_search_motor_max_angle_spd(p, i, 0x00);
        usleep(5000);
    }
}

void cpiper_search_all_motor_max_acc_limit(cpiper_interface_t *p) {
    for (int i = 1; i <= 6; i++) {
        cpiper_search_motor_max_angle_spd(p, i, 0x01);
        usleep(5000);
    }
}

void cpiper_motor_angle_limit_max_spd_set(cpiper_interface_t *p,
                                           uint8_t motor_num,
                                           int16_t max_angle,
                                           int16_t min_angle,
                                           uint16_t max_spd) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD_SET;
    p->tx_msg.motor_angle_limit_set.motor_num = motor_num;
    p->tx_msg.motor_angle_limit_set.max_angle_limit = max_angle;
    p->tx_msg.motor_angle_limit_set.min_angle_limit = min_angle;
    p->tx_msg.motor_angle_limit_set.max_joint_spd = max_spd;
    send_msg(p);
}

void cpiper_joint_config(cpiper_interface_t *p,
                         uint8_t joint_num, uint8_t set_zero,
                         uint8_t acc_effective, uint16_t max_acc,
                         uint8_t clear_err) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_JOINT_CONFIG;
    p->tx_msg.joint_config.joint_motor_num = joint_num;
    p->tx_msg.joint_config.set_motor_current_pos_as_zero = set_zero;
    p->tx_msg.joint_config.acc_param_config_is_effective_or_not = acc_effective;
    p->tx_msg.joint_config.max_joint_acc = max_acc;
    p->tx_msg.joint_config.clear_joint_err = clear_err;
    send_msg(p);
}

void cpiper_joint_max_acc_config(cpiper_interface_t *p,
                                 uint8_t motor_num, uint16_t max_acc) {
    cpiper_joint_config(p, motor_num, 0, 0, max_acc, 0);
}

void cpiper_arm_param_enquiry_and_config(cpiper_interface_t *p,
                                          uint8_t enquiry, uint8_t setting,
                                          uint8_t fb, uint8_t load_effective,
                                          uint8_t set_load) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_PARAM_ENQUIRY_AND_CONFIG;
    p->tx_msg.param_enquiry.param_enquiry = enquiry;
    p->tx_msg.param_enquiry.param_setting = setting;
    p->tx_msg.param_enquiry.data_feedback_0x48x = fb;
    p->tx_msg.param_enquiry.end_load_param_setting_effective = load_effective;
    p->tx_msg.param_enquiry.set_end_load = set_load;
    send_msg(p);
}

void cpiper_end_spd_acc_param_set(cpiper_interface_t *p,
                                   uint16_t lin_vel, uint16_t ang_vel,
                                   uint16_t lin_acc, uint16_t ang_acc) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_END_VEL_ACC_PARAM_CONFIG;
    p->tx_msg.end_vel_acc_config.end_max_linear_vel = lin_vel;
    p->tx_msg.end_vel_acc_config.end_max_angular_vel = ang_vel;
    p->tx_msg.end_vel_acc_config.end_max_linear_acc = lin_acc;
    p->tx_msg.end_vel_acc_config.end_max_angular_acc = ang_acc;
    send_msg(p);
}

void cpiper_crash_protection_config(cpiper_interface_t *p,
                                     uint8_t j1, uint8_t j2, uint8_t j3,
                                     uint8_t j4, uint8_t j5, uint8_t j6) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_CRASH_PROTECTION_CONFIG;
    p->tx_msg.crash_config.joint_1_protection_level = j1;
    p->tx_msg.crash_config.joint_2_protection_level = j2;
    p->tx_msg.crash_config.joint_3_protection_level = j3;
    p->tx_msg.crash_config.joint_4_protection_level = j4;
    p->tx_msg.crash_config.joint_5_protection_level = j5;
    p->tx_msg.crash_config.joint_6_protection_level = j6;
    send_msg(p);
}

void cpiper_search_firmware_version(cpiper_interface_t *p) {
    /* Python SDK sends raw frame on 0x4AF, bypassing protocol encoder */
    uint8_t data[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    cpiper_can_send(&p->can, CAN_ID_ARM_FIRMWARE_READ, data);
}

void cpiper_mit_ctrl(cpiper_interface_t *p, uint8_t motor_num,
                     float pos, float vel, float kp, float kd, float t) {
    if (motor_num < 1 || motor_num > 6) return;

    cpiper_msg_init(&p->tx_msg);
    /* Map motor 1-6 → msg type MIT_CTRL_1-6 */
    p->tx_msg.type_ = CPPIER_MSG_MIT_CTRL_1 + (motor_num - 1);
    p->tx_msg.mit_ctrl.pos_ref = cpiper_float_to_uint(pos, -12.5f, 12.5f, 16);
    p->tx_msg.mit_ctrl.vel_ref = cpiper_float_to_uint(vel, -45.0f, 45.0f, 12);
    p->tx_msg.mit_ctrl.kp = cpiper_float_to_uint(kp, 0.0f, 500.0f, 12);
    p->tx_msg.mit_ctrl.kd = cpiper_float_to_uint(kd, -5.0f, 5.0f, 12);
    p->tx_msg.mit_ctrl.t_ref = cpiper_float_to_uint(t, -10.0f, 10.0f, 8);
    send_msg(p);
}

void cpiper_gripper_teaching_param_config(cpiper_interface_t *p,
                                           uint8_t range_per,
                                           uint8_t max_range,
                                           uint8_t friction) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_GRIPPER_TEACHING_PARAM_CONFIG;
    p->tx_msg.gripper_teaching_config.teaching_range_per = range_per;
    p->tx_msg.gripper_teaching_config.max_range_config = max_range;
    p->tx_msg.gripper_teaching_config.teaching_friction = friction;
    send_msg(p);
}

void cpiper_req_master_arm_home(cpiper_interface_t *p, uint8_t mode) {
    cpiper_msg_init(&p->tx_msg);
    p->tx_msg.type_ = CPPIER_MSG_REQ_MASTER_ARM_HOME;
    p->tx_msg.motion_ctrl_1.emergency_stop = mode;
    send_msg(p);
}

/* ─── Getters (thread-safe) ──────────────────────────────────── */

cpiper_status_t cpiper_get_status(cpiper_interface_t *p) {
    pthread_mutex_lock(&p->fb_mutex);
    cpiper_status_t s = p->fb_status;
    pthread_mutex_unlock(&p->fb_mutex);
    return s;
}

cpiper_end_pose_t cpiper_get_end_pose(cpiper_interface_t *p) {
    pthread_mutex_lock(&p->fb_mutex);
    cpiper_end_pose_t e = p->fb_end_pose;
    pthread_mutex_unlock(&p->fb_mutex);
    return e;
}

cpiper_joint_state_t cpiper_get_joints(cpiper_interface_t *p) {
    pthread_mutex_lock(&p->fb_mutex);
    cpiper_joint_state_t j = p->fb_joint_state;
    pthread_mutex_unlock(&p->fb_mutex);
    return j;
}

cpiper_gripper_t cpiper_get_gripper(cpiper_interface_t *p) {
    pthread_mutex_lock(&p->fb_mutex);
    cpiper_gripper_t g = p->fb_gripper;
    pthread_mutex_unlock(&p->fb_mutex);
    return g;
}

cpiper_motor_high_spd_t cpiper_get_motor_high_spd(cpiper_interface_t *p,
                                                   int motor) {
    cpiper_motor_high_spd_t m = {0};
    if (motor < 0 || motor > 5) return m;
    pthread_mutex_lock(&p->fb_mutex);
    m = p->fb_motor_high_spd[motor];
    pthread_mutex_unlock(&p->fb_mutex);
    return m;
}

cpiper_motor_low_spd_t cpiper_get_motor_low_spd(cpiper_interface_t *p,
                                                  int motor) {
    cpiper_motor_low_spd_t m = {0};
    if (motor < 0 || motor > 5) return m;
    pthread_mutex_lock(&p->fb_mutex);
    m = p->fb_motor_low_spd[motor];
    pthread_mutex_unlock(&p->fb_mutex);
    return m;
}

double cpiper_get_can_fps(cpiper_interface_t *p) {
    return cpiper_fps_get(&p->fps_total);
}

bool cpiper_is_ok(cpiper_interface_t *p) {
    return p->connected && p->is_ok;
}

char* cpiper_get_firmware_version(cpiper_interface_t *p, char *buf, int buflen) {
    if (!buf || buflen < 9) return buf;
    pthread_mutex_lock(&p->fb_mutex);
    memcpy(buf, p->fb_firmware.firmware_data, 8);
    pthread_mutex_unlock(&p->fb_mutex);
    buf[8] = '\0';
    return buf;
}

/* ─── Configuration ──────────────────────────────────────────── */

void cpiper_enable_fk(cpiper_interface_t *p, bool enable) {
    p->fk_enabled = enable;
}

void cpiper_set_filter_abnormal(cpiper_interface_t *p, bool enable) {
    p->filter_abnormal = enable;
}

void cpiper_set_joint_limit(cpiper_interface_t *p, const char *joint,
                            float min_rad, float max_rad) {
    (void)p;
    int j = 0;
    if (joint[0] == 'j' || joint[0] == 'J') j = joint[1] - '0';
    if (j < 1 || j > 6) return;
    cpiper_param_set_joint_limit(j, min_rad, max_rad);
}

void cpiper_set_gripper_range(cpiper_interface_t *p, float min_mm,
                              float max_mm) {
    (void)p;
    cpiper_param_set_gripper_range(min_mm, max_mm);
}

int cpiper_send_raw(cpiper_interface_t *p, uint32_t can_id,
                    const uint8_t data[8]) {
    return cpiper_can_send(&p->can, can_id, data);
}
