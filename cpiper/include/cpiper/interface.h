/**
 * @file interface.h
 * @brief High-level API for Piper robot arm control
 */
#ifndef CPIPER_INTERFACE_H
#define CPIPER_INTERFACE_H

#include "messages.h"
#include "can.h"
#include "kinematics.h"
#include "param_manager.h"
#include "fps.h"
#include <pthread.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CPIPER_MAX_CAN_NAME 16

typedef struct {
    /* CAN bus */
    cpiper_can_t can;

    /* Parser state */
    cpiper_message_t tx_msg;

    /* Feedback storage (mutex-protected) */
    pthread_mutex_t fb_mutex;
    cpiper_status_t          fb_status;
    cpiper_end_pose_t        fb_end_pose;
    cpiper_joint_state_t     fb_joint_state;
    cpiper_gripper_t         fb_gripper;
    cpiper_motor_high_spd_t  fb_motor_high_spd[6];
    cpiper_motor_low_spd_t   fb_motor_low_spd[6];
    cpiper_firmware_t        fb_firmware;
    cpiper_resp_set_instruction_t fb_resp_instruction;
    cpiper_motor_angle_limit_t   fb_motor_angle_limit;
    cpiper_end_vel_acc_param_t   fb_end_vel_acc_param;
    cpiper_crash_protection_t    fb_crash_protection;
    cpiper_motor_max_acc_t       fb_motor_max_acc;
    cpiper_gripper_teaching_param_t fb_gripper_teaching_param;

    /* Transmit cache (for get-arm-ctrl queries) */
    cpiper_motion_ctrl_2_t   tx_motion_ctrl_2;
    cpiper_joint_ctrl_t      tx_joint_ctrl;
    cpiper_gripper_ctrl_t    tx_gripper_ctrl;

    /* FPS counters */
    cpiper_fps_t fps_status;
    cpiper_fps_t fps_end_pose;
    cpiper_fps_t fps_joint;
    cpiper_fps_t fps_gripper;
    cpiper_fps_t fps_high_spd;
    cpiper_fps_t fps_low_spd;
    cpiper_fps_t fps_total;

    /* FK */
    cpiper_fk_dh_t fk_dh;
    bool fk_enabled;
    float fk_pos[6];

    /* State */
    bool connected;
    bool is_ok;
    bool filter_abnormal;
    double last_recv_time;

    /* Threading */
    pthread_t recv_thread;
    bool thread_running;
    bool stop_flag;
    pthread_mutex_t stop_mutex;
} cpiper_interface_t;

/* ─── Lifecycle ──────────────────────────────────────────────── */

/**
 * Initialize an interface (does not open CAN).
 */
int cpiper_init(cpiper_interface_t *p, const char *can_name);

/**
 * Connect to the CAN bus and optionally start recv thread + init queries.
 */
int cpiper_connect(cpiper_interface_t *p, bool do_piper_init,
                   bool start_thread);

/**
 * Disconnect and stop threads.
 */
void cpiper_disconnect(cpiper_interface_t *p);

/**
 * Destroy resources.
 */
void cpiper_destroy(cpiper_interface_t *p);

/* ─── Polling (single-threaded mode) ─────────────────────────── */

/**
 * Non-blocking poll: receive and decode all available CAN frames.
 * Call this in your main loop when not using the internal thread.
 */
void cpiper_poll(cpiper_interface_t *p);

/* ─── Send commands (all non-blocking) ───────────────────────── */

void cpiper_motion_ctrl_1(cpiper_interface_t *p,
                          uint8_t e_stop, uint8_t track, uint8_t teach);

void cpiper_motion_ctrl_2(cpiper_interface_t *p,
                          uint8_t ctrl_mode, uint8_t move_mode,
                          uint8_t spd, uint8_t mit, uint8_t res_time);

void cpiper_mode_ctrl(cpiper_interface_t *p,
                      uint8_t ctrl_mode, uint8_t move_mode,
                      uint8_t spd, uint8_t mit);

void cpiper_emergency_stop(cpiper_interface_t *p, uint8_t e_stop);

void cpiper_reset(cpiper_interface_t *p);

void cpiper_joint_ctrl(cpiper_interface_t *p,
                       float j1, float j2, float j3,
                       float j4, float j5, float j6);

void cpiper_end_pose_ctrl(cpiper_interface_t *p,
                          int32_t x, int32_t y, int32_t z,
                          int32_t rx, int32_t ry, int32_t rz);

void cpiper_move_c_axis_update(cpiper_interface_t *p, uint8_t instruction_num);

void cpiper_gripper_ctrl(cpiper_interface_t *p,
                         int32_t angle, int16_t effort,
                         uint8_t status_code, uint8_t set_zero);

void cpiper_master_slave_config(cpiper_interface_t *p,
                                uint8_t linkage, uint8_t fb_off,
                                uint8_t ctrl_off, uint8_t link_off);

void cpiper_enable_arm(cpiper_interface_t *p, uint8_t motor_num, uint8_t flag);
void cpiper_disable_arm(cpiper_interface_t *p, uint8_t motor_num, uint8_t flag);
bool cpiper_enable_piper(cpiper_interface_t *p);
bool cpiper_disable_piper(cpiper_interface_t *p);

void cpiper_search_motor_max_angle_spd(cpiper_interface_t *p,
                                       uint8_t motor_num,
                                       uint8_t search_content);
void cpiper_search_all_motor_max_angle_spd(cpiper_interface_t *p);
void cpiper_search_all_motor_max_acc_limit(cpiper_interface_t *p);

void cpiper_motor_angle_limit_max_spd_set(cpiper_interface_t *p,
                                           uint8_t motor_num,
                                           int16_t max_angle,
                                           int16_t min_angle,
                                           uint16_t max_spd);

void cpiper_joint_config(cpiper_interface_t *p,
                         uint8_t joint_num, uint8_t set_zero,
                         uint8_t acc_effective, uint16_t max_acc,
                         uint8_t clear_err);

void cpiper_joint_max_acc_config(cpiper_interface_t *p,
                                 uint8_t motor_num, uint16_t max_acc);

void cpiper_arm_param_enquiry_and_config(cpiper_interface_t *p,
                                          uint8_t enquiry, uint8_t setting,
                                          uint8_t fb, uint8_t load_effective,
                                          uint8_t set_load);

void cpiper_end_spd_acc_param_set(cpiper_interface_t *p,
                                   uint16_t lin_vel, uint16_t ang_vel,
                                   uint16_t lin_acc, uint16_t ang_acc);

void cpiper_crash_protection_config(cpiper_interface_t *p,
                                     uint8_t j1, uint8_t j2, uint8_t j3,
                                     uint8_t j4, uint8_t j5, uint8_t j6);

void cpiper_search_firmware_version(cpiper_interface_t *p);

void cpiper_mit_ctrl(cpiper_interface_t *p, uint8_t motor_num,
                     float pos, float vel, float kp, float kd, float t);

void cpiper_gripper_teaching_param_config(cpiper_interface_t *p,
                                           uint8_t range_per,
                                           uint8_t max_range,
                                           uint8_t friction);

void cpiper_req_master_arm_home(cpiper_interface_t *p, uint8_t mode);

/* ─── Getters (thread-safe) ──────────────────────────────────── */

cpiper_status_t        cpiper_get_status(cpiper_interface_t *p);
cpiper_end_pose_t      cpiper_get_end_pose(cpiper_interface_t *p);
cpiper_joint_state_t   cpiper_get_joints(cpiper_interface_t *p);
cpiper_gripper_t       cpiper_get_gripper(cpiper_interface_t *p);
cpiper_motor_high_spd_t cpiper_get_motor_high_spd(cpiper_interface_t *p, int motor);
cpiper_motor_low_spd_t  cpiper_get_motor_low_spd(cpiper_interface_t *p, int motor);
double cpiper_get_can_fps(cpiper_interface_t *p);
bool   cpiper_is_ok(cpiper_interface_t *p);
char*  cpiper_get_firmware_version(cpiper_interface_t *p, char *buf, int buflen);

/* ─── Configuration ──────────────────────────────────────────── */

void cpiper_enable_fk(cpiper_interface_t *p, bool enable);
void cpiper_set_filter_abnormal(cpiper_interface_t *p, bool enable);
void cpiper_set_joint_limit(cpiper_interface_t *p, const char *joint,
                            float min_rad, float max_rad);
void cpiper_set_gripper_range(cpiper_interface_t *p, float min_mm, float max_mm);

/* ─── Debug: send a raw CAN frame ────────────────────────────── */
int cpiper_send_raw(cpiper_interface_t *p, uint32_t can_id,
                    const uint8_t data[8]);

#ifdef __cplusplus
}
#endif

#endif /* CPIPER_INTERFACE_H */
