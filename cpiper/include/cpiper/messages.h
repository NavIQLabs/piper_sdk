/**
 * @file messages.h
 * @brief All message structs for Piper robot arm (V2 protocol)
 */
#ifndef CPIPER_MESSAGES_H
#define CPIPER_MESSAGES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Message type enum ──────────────────────────────────────── */
typedef enum {
    /* Feedback (arm → PC) */
    CPPIER_MSG_STATUS_FEEDBACK = 0,
    CPPIER_MSG_END_POSE_FEEDBACK,
    CPPIER_MSG_JOINT_FEEDBACK,
    CPPIER_MSG_GRIPPER_FEEDBACK,
    CPPIER_MSG_HIGH_SPD_MOTOR_FEEDBACK,
    CPPIER_MSG_LOW_SPD_MOTOR_FEEDBACK,
    CPPIER_MSG_FIRMWARE,
    CPPIER_MSG_RESP_SET_INSTRUCTION,
    CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD,
    CPPIER_MSG_END_VEL_ACC_PARAM,
    CPPIER_MSG_CRASH_PROTECTION_FEEDBACK,
    CPPIER_MSG_MOTOR_MAX_ACC_LIMIT,
    CPPIER_MSG_GRIPPER_TEACHING_PARAM_FEEDBACK,

    /* Transmit (PC → arm) */
    CPPIER_MSG_MOTION_CTRL_1,
    CPPIER_MSG_MOTION_CTRL_2,
    CPPIER_MSG_MOTION_CTRL_CARTESIAN_1,
    CPPIER_MSG_MOTION_CTRL_CARTESIAN_2,
    CPPIER_MSG_MOTION_CTRL_CARTESIAN_3,
    CPPIER_MSG_JOINT_CTRL_12,
    CPPIER_MSG_JOINT_CTRL_34,
    CPPIER_MSG_JOINT_CTRL_56,
    CPPIER_MSG_CIRCULAR_CTRL,
    CPPIER_MSG_GRIPPER_CTRL,
    CPPIER_MSG_MASTER_SLAVE_CONFIG,
    CPPIER_MSG_MOTOR_ENABLE_DISABLE,
    CPPIER_MSG_SEARCH_MOTOR_MAX_ANGLE_SPD,
    CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD_SET,
    CPPIER_MSG_JOINT_CONFIG,
    CPPIER_MSG_INSTRUCTION_RESPONSE_CONFIG,
    CPPIER_MSG_PARAM_ENQUIRY_AND_CONFIG,
    CPPIER_MSG_END_VEL_ACC_PARAM_CONFIG,
    CPPIER_MSG_CRASH_PROTECTION_CONFIG,
    CPPIER_MSG_REQ_MASTER_ARM_HOME,
    CPPIER_MSG_GRIPPER_TEACHING_PARAM_CONFIG,
    CPPIER_MSG_MIT_CTRL_1,
    CPPIER_MSG_MIT_CTRL_2,
    CPPIER_MSG_MIT_CTRL_3,
    CPPIER_MSG_MIT_CTRL_4,
    CPPIER_MSG_MIT_CTRL_5,
    CPPIER_MSG_MIT_CTRL_6,

    CPPIER_MSG_UNKNOWN = 0xFF
} cpiper_msg_type_t;

/* ─── Feedback structs ───────────────────────────────────────── */

typedef struct {
    uint8_t  ctrl_mode;      /* 0-2: servo ready, 3-6: CAN master, 7: teach */
    uint8_t  arm_status;     /* 0-2: servo ready, 3-6: CAN master, 7: teach */
    uint8_t  mode_feed;      /* control mode feedback */
    uint8_t  teach_status;   /* 0=control, 1=teach */
    uint8_t  motion_status;  /* 0=idle, 1=moving */
    uint8_t  trajectory_num; /* current trajectory number */
    uint16_t err_code;       /* error code */
} cpiper_status_t;

typedef struct {
    int32_t X_axis;   /* 0.001mm */
    int32_t Y_axis;   /* 0.001mm */
    int32_t Z_axis;   /* 0.001mm */
    int32_t RX_axis;  /* 0.001° */
    int32_t RY_axis;  /* 0.001° */
    int32_t RZ_axis;  /* 0.001° */
} cpiper_end_pose_t;

typedef struct {
    int32_t joint_1;  /* 0.001° */
    int32_t joint_2;  /* 0.001° */
    int32_t joint_3;  /* 0.001° */
    int32_t joint_4;  /* 0.001° */
    int32_t joint_5;  /* 0.001° */
    int32_t joint_6;  /* 0.001° */
} cpiper_joint_state_t;

typedef struct {
    int32_t angle;       /* 0.001mm */
    int16_t effort;      /* 0.001N/m */
    uint8_t status_code;
} cpiper_gripper_t;

typedef struct {
    uint32_t can_id;
    int16_t  motor_speed;  /* 0.001 rad/s */
    int16_t  current;      /* 0.001A */
    int32_t  pos;          /* 0.001 rad */
} cpiper_motor_high_spd_t;

typedef struct {
    uint32_t can_id;
    uint16_t vol;              /* mV */
    int16_t  foc_temp;         /* 0.1°C */
    int8_t   motor_temp;       /* °C */
    uint8_t  foc_status_code;
    uint16_t bus_current;      /* mA */
} cpiper_motor_low_spd_t;

typedef struct {
    uint8_t firmware_data[8];
} cpiper_firmware_t;

typedef struct {
    uint8_t instruction_index;
    uint8_t is_set_zero_successfully;
} cpiper_resp_set_instruction_t;

typedef struct {
    uint8_t  motor_num;
    int16_t  max_angle_limit;  /* 0.1° */
    int16_t  min_angle_limit;  /* 0.1° */
    uint16_t max_joint_spd;    /* 0.001 rad/s */
} cpiper_motor_angle_limit_t;

typedef struct {
    uint16_t end_max_linear_vel;    /* 0.001 m/s */
    uint16_t end_max_angular_vel;   /* 0.001 rad/s */
    uint16_t end_max_linear_acc;    /* 0.001 m/s² */
    uint16_t end_max_angular_acc;   /* 0.001 rad/s² */
} cpiper_end_vel_acc_param_t;

typedef struct {
    uint8_t joint_1_protection_level;
    uint8_t joint_2_protection_level;
    uint8_t joint_3_protection_level;
    uint8_t joint_4_protection_level;
    uint8_t joint_5_protection_level;
    uint8_t joint_6_protection_level;
} cpiper_crash_protection_t;

typedef struct {
    uint8_t joint_motor_num;
    uint16_t max_joint_acc;  /* 0.001 rad/s² */
} cpiper_motor_max_acc_t;

typedef struct {
    uint8_t teaching_range_per;    /* % */
    uint8_t max_range_config;      /* mm */
    uint8_t teaching_friction;     /* N/m */
} cpiper_gripper_teaching_param_t;

/* ─── Transmit structs ───────────────────────────────────────── */

typedef struct {
    uint8_t emergency_stop;
    uint8_t track_ctrl;
    uint8_t grag_teach_ctrl;
} cpiper_motion_ctrl_1_t;

typedef struct {
    uint8_t ctrl_mode;
    uint8_t move_mode;
    uint8_t move_spd_rate_ctrl;
    uint8_t mit_mode;
    uint8_t residence_time;
    uint8_t installation_pos;
} cpiper_motion_ctrl_2_t;

typedef struct {
    int32_t X_axis;   /* 0.001mm */
    int32_t Y_axis;   /* 0.001mm */
    int32_t Z_axis;   /* 0.001mm */
    int32_t RX_axis;  /* 0.001° */
    int32_t RY_axis;  /* 0.001° */
    int32_t RZ_axis;  /* 0.001° */
} cpiper_cartesian_ctrl_t;

typedef struct {
    int32_t joint_1;  /* 0.001° */
    int32_t joint_2;
    int32_t joint_3;
    int32_t joint_4;
    int32_t joint_5;
    int32_t joint_6;
} cpiper_joint_ctrl_t;

typedef struct {
    uint8_t instruction_num;
} cpiper_circular_ctrl_t;

typedef struct {
    int32_t angle;       /* 0.001mm */
    int16_t effort;      /* 0.001N/m */
    uint8_t status_code;
    uint8_t set_zero;
} cpiper_gripper_ctrl_t;

typedef struct {
    uint8_t linkage_config;
    uint8_t feedback_offset;
    uint8_t ctrl_offset;
    uint8_t linkage_offset;
} cpiper_master_slave_config_t;

typedef struct {
    uint8_t motor_num;
    uint8_t enable_flag;
} cpiper_motor_enable_t;

typedef struct {
    uint8_t motor_num;
    uint8_t search_content;
} cpiper_search_motor_t;

typedef struct {
    uint8_t  motor_num;
    int16_t  max_angle_limit;
    int16_t  min_angle_limit;
    uint16_t max_joint_spd;
} cpiper_motor_angle_limit_set_t;

typedef struct {
    uint8_t  joint_motor_num;
    uint8_t  set_motor_current_pos_as_zero;
    uint8_t  acc_param_config_is_effective_or_not;
    uint16_t max_joint_acc;
    uint8_t  clear_joint_err;
} cpiper_joint_config_t;

typedef struct {
    uint8_t instruction_index;
    uint8_t zero_config_success_flag;
} cpiper_instruction_response_t;

typedef struct {
    uint8_t param_enquiry;
    uint8_t param_setting;
    uint8_t data_feedback_0x48x;
    uint8_t end_load_param_setting_effective;
    uint8_t set_end_load;
} cpiper_param_enquiry_t;

typedef struct {
    uint16_t end_max_linear_vel;
    uint16_t end_max_angular_vel;
    uint16_t end_max_linear_acc;
    uint16_t end_max_angular_acc;
} cpiper_end_vel_acc_config_t;

typedef struct {
    uint8_t joint_1_protection_level;
    uint8_t joint_2_protection_level;
    uint8_t joint_3_protection_level;
    uint8_t joint_4_protection_level;
    uint8_t joint_5_protection_level;
    uint8_t joint_6_protection_level;
} cpiper_crash_protection_config_t;

typedef struct {
    uint8_t teaching_range_per;
    uint8_t max_range_config;
    uint8_t teaching_friction;
} cpiper_gripper_teaching_config_t;

typedef struct {
    uint16_t pos_ref;     /* uint16 float mapped */
    uint16_t vel_ref;     /* 12-bit */
    uint16_t kp;          /* 12-bit */
    uint16_t kd;          /* 12-bit */
    uint16_t t_ref;       /* 12-bit */
    uint8_t  crc;         /* computed */
} cpiper_mit_ctrl_t;

/* ─── Aggregate message ──────────────────────────────────────── */
typedef struct {
    cpiper_msg_type_t type_;
    double time_stamp;

    /* Feedback */
    cpiper_status_t                status;
    cpiper_end_pose_t              end_pose;
    cpiper_joint_state_t           joint_state;
    cpiper_gripper_t               gripper;
    cpiper_motor_high_spd_t        motor_high_spd[6];
    cpiper_motor_low_spd_t         motor_low_spd[6];
    cpiper_firmware_t              firmware;
    cpiper_resp_set_instruction_t  resp_instruction;
    cpiper_motor_angle_limit_t     motor_angle_limit;
    cpiper_end_vel_acc_param_t     end_vel_acc_param;
    cpiper_crash_protection_t      crash_protection;
    cpiper_motor_max_acc_t         motor_max_acc;
    cpiper_gripper_teaching_param_t gripper_teaching_param;

    /* Transmit */
    cpiper_motion_ctrl_1_t         motion_ctrl_1;
    cpiper_motion_ctrl_2_t         motion_ctrl_2;
    cpiper_cartesian_ctrl_t        cartesian;
    cpiper_joint_ctrl_t            joint_ctrl;
    cpiper_circular_ctrl_t         circular_ctrl;
    cpiper_gripper_ctrl_t          gripper_ctrl;
    cpiper_master_slave_config_t   ms_config;
    cpiper_motor_enable_t          motor_enable;
    cpiper_search_motor_t          search_motor;
    cpiper_motor_angle_limit_set_t motor_angle_limit_set;
    cpiper_joint_config_t          joint_config;
    cpiper_instruction_response_t  instruction_response;
    cpiper_param_enquiry_t         param_enquiry;
    cpiper_end_vel_acc_config_t    end_vel_acc_config;
    cpiper_crash_protection_config_t crash_config;
    cpiper_gripper_teaching_config_t gripper_teaching_config;
    cpiper_mit_ctrl_t              mit_ctrl;
} cpiper_message_t;

/* ─── Message type ↔ CAN ID mapping ──────────────────────────── */
uint32_t cpiper_msg_type_to_can_id(cpiper_msg_type_t type);
cpiper_msg_type_t cpiper_can_id_to_msg_type(uint32_t can_id);

/* ─── Initialize a message to defaults ───────────────────────── */
void cpiper_msg_init(cpiper_message_t *msg);

#ifdef __cplusplus
}
#endif

#endif /* CPIPER_MESSAGES_H */
