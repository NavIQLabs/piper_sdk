/**
 * @file protocol.c
 * @brief CAN frame encode/decode for Piper V2 protocol
 */
#include "cpiper/protocol.h"
#include "cpiper/can_ids.h"
#include <string.h>

/* ─── Message type ↔ CAN ID mapping ──────────────────────────── */

uint32_t cpiper_msg_type_to_can_id(cpiper_msg_type_t type) {
    switch (type) {
        /* Feedback */
        case CPPIER_MSG_STATUS_FEEDBACK:          return CAN_ID_ARM_STATUS_FEEDBACK;
        case CPPIER_MSG_END_POSE_FEEDBACK:        return CAN_ID_ARM_END_POSE_FEEDBACK_1;
        case CPPIER_MSG_JOINT_FEEDBACK:           return CAN_ID_ARM_JOINT_FEEDBACK_12;
        case CPPIER_MSG_GRIPPER_FEEDBACK:         return CAN_ID_ARM_GRIPPER_FEEDBACK;
        case CPPIER_MSG_HIGH_SPD_MOTOR_FEEDBACK:  return CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_1;
        case CPPIER_MSG_LOW_SPD_MOTOR_FEEDBACK:   return CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_1;
        case CPPIER_MSG_FIRMWARE:                 return CAN_ID_ARM_FIRMWARE_READ;
        case CPPIER_MSG_RESP_SET_INSTRUCTION:     return CAN_ID_ARM_FEEDBACK_RESP_SET_INSTRUCTION;
        case CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD:return CAN_ID_ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD;
        case CPPIER_MSG_END_VEL_ACC_PARAM:        return CAN_ID_ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM;
        case CPPIER_MSG_CRASH_PROTECTION_FEEDBACK:return CAN_ID_ARM_CRASH_PROTECTION_RATING_FEEDBACK;
        case CPPIER_MSG_MOTOR_MAX_ACC_LIMIT:      return CAN_ID_ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT;
        case CPPIER_MSG_GRIPPER_TEACHING_PARAM_FEEDBACK: return CAN_ID_ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK;
        /* Transmit */
        case CPPIER_MSG_MOTION_CTRL_1:            return CAN_ID_ARM_MOTION_CTRL_1;
        case CPPIER_MSG_MOTION_CTRL_2:            return CAN_ID_ARM_MOTION_CTRL_2;
        case CPPIER_MSG_MOTION_CTRL_CARTESIAN_1:  return CAN_ID_ARM_MOTION_CTRL_CARTESIAN_1;
        case CPPIER_MSG_MOTION_CTRL_CARTESIAN_2:  return CAN_ID_ARM_MOTION_CTRL_CARTESIAN_2;
        case CPPIER_MSG_MOTION_CTRL_CARTESIAN_3:  return CAN_ID_ARM_MOTION_CTRL_CARTESIAN_3;
        case CPPIER_MSG_JOINT_CTRL_12:            return CAN_ID_ARM_JOINT_CTRL_12;
        case CPPIER_MSG_JOINT_CTRL_34:            return CAN_ID_ARM_JOINT_CTRL_34;
        case CPPIER_MSG_JOINT_CTRL_56:            return CAN_ID_ARM_JOINT_CTRL_56;
        case CPPIER_MSG_CIRCULAR_CTRL:            return CAN_ID_ARM_JOINT_CTRL_12; /* placeholder */
        case CPPIER_MSG_GRIPPER_CTRL:             return CAN_ID_ARM_GRIPPER_CTRL;
        case CPPIER_MSG_MASTER_SLAVE_CONFIG:      return CAN_ID_ARM_MASTER_SLAVE_MODE_CONFIG;
        case CPPIER_MSG_MOTOR_ENABLE_DISABLE:     return CAN_ID_ARM_MOTOR_ENABLE_DISABLE_CONFIG;
        case CPPIER_MSG_SEARCH_MOTOR_MAX_ANGLE_SPD: return CAN_ID_ARM_SEARCH_MOTOR_MAX_ANGLE_SPD;
        case CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD_SET: return CAN_ID_ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET;
        case CPPIER_MSG_JOINT_CONFIG:             return CAN_ID_ARM_JOINT_CONFIG;
        case CPPIER_MSG_INSTRUCTION_RESPONSE_CONFIG: return CAN_ID_ARM_INSTRUCTION_RESPONSE_CONFIG;
        case CPPIER_MSG_PARAM_ENQUIRY_AND_CONFIG: return CAN_ID_ARM_PARAM_ENQUIRY_AND_CONFIG;
        case CPPIER_MSG_END_VEL_ACC_PARAM_CONFIG: return CAN_ID_ARM_END_VEL_ACC_PARAM_CONFIG;
        case CPPIER_MSG_CRASH_PROTECTION_CONFIG:  return CAN_ID_ARM_CRASH_PROTECTION_RATING_CONFIG;
        case CPPIER_MSG_REQ_MASTER_ARM_HOME:      return CAN_ID_ARM_REQ_MASTER_ARM_HOME;
        case CPPIER_MSG_GRIPPER_TEACHING_PARAM_CONFIG: return CAN_ID_ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG;
        case CPPIER_MSG_MIT_CTRL_1:               return CAN_ID_ARM_JOINT_MIT_CTRL_1;
        case CPPIER_MSG_MIT_CTRL_2:               return CAN_ID_ARM_JOINT_MIT_CTRL_2;
        case CPPIER_MSG_MIT_CTRL_3:               return CAN_ID_ARM_JOINT_MIT_CTRL_3;
        case CPPIER_MSG_MIT_CTRL_4:               return CAN_ID_ARM_JOINT_MIT_CTRL_4;
        case CPPIER_MSG_MIT_CTRL_5:               return CAN_ID_ARM_JOINT_MIT_CTRL_5;
        case CPPIER_MSG_MIT_CTRL_6:               return CAN_ID_ARM_JOINT_MIT_CTRL_6;
        default: return 0;
    }
}

cpiper_msg_type_t cpiper_can_id_to_msg_type(uint32_t can_id) {
    switch (can_id) {
        case CAN_ID_ARM_STATUS_FEEDBACK:          return CPPIER_MSG_STATUS_FEEDBACK;
        case CAN_ID_ARM_END_POSE_FEEDBACK_1:      return CPPIER_MSG_END_POSE_FEEDBACK;
        case CAN_ID_ARM_JOINT_FEEDBACK_12:        return CPPIER_MSG_JOINT_FEEDBACK;
        case CAN_ID_ARM_GRIPPER_FEEDBACK:         return CPPIER_MSG_GRIPPER_FEEDBACK;
        case CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_1: return CPPIER_MSG_HIGH_SPD_MOTOR_FEEDBACK;
        case CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_1:  return CPPIER_MSG_LOW_SPD_MOTOR_FEEDBACK;
        case CAN_ID_ARM_FIRMWARE_READ:            return CPPIER_MSG_FIRMWARE;
        case CAN_ID_ARM_FEEDBACK_RESP_SET_INSTRUCTION: return CPPIER_MSG_RESP_SET_INSTRUCTION;
        case CAN_ID_ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD: return CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD;
        case CAN_ID_ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM: return CPPIER_MSG_END_VEL_ACC_PARAM;
        case CAN_ID_ARM_CRASH_PROTECTION_RATING_FEEDBACK: return CPPIER_MSG_CRASH_PROTECTION_FEEDBACK;
        case CAN_ID_ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT: return CPPIER_MSG_MOTOR_MAX_ACC_LIMIT;
        case CAN_ID_ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK: return CPPIER_MSG_GRIPPER_TEACHING_PARAM_FEEDBACK;
        case CAN_ID_ARM_MOTION_CTRL_1:            return CPPIER_MSG_MOTION_CTRL_1;
        case CAN_ID_ARM_MOTION_CTRL_2:            return CPPIER_MSG_MOTION_CTRL_2;
        case CAN_ID_ARM_MOTION_CTRL_CARTESIAN_1:  return CPPIER_MSG_MOTION_CTRL_CARTESIAN_1;
        case CAN_ID_ARM_MOTION_CTRL_CARTESIAN_2:  return CPPIER_MSG_MOTION_CTRL_CARTESIAN_2;
        case CAN_ID_ARM_MOTION_CTRL_CARTESIAN_3:  return CPPIER_MSG_MOTION_CTRL_CARTESIAN_3;
        case CAN_ID_ARM_JOINT_CTRL_12:            return CPPIER_MSG_JOINT_CTRL_12;
        case CAN_ID_ARM_JOINT_CTRL_34:            return CPPIER_MSG_JOINT_CTRL_34;
        case CAN_ID_ARM_JOINT_CTRL_56:            return CPPIER_MSG_JOINT_CTRL_56;
        case CAN_ID_ARM_GRIPPER_CTRL:             return CPPIER_MSG_GRIPPER_CTRL;
        case CAN_ID_ARM_MASTER_SLAVE_MODE_CONFIG: return CPPIER_MSG_MASTER_SLAVE_CONFIG;
        case CAN_ID_ARM_MOTOR_ENABLE_DISABLE_CONFIG: return CPPIER_MSG_MOTOR_ENABLE_DISABLE;
        case CAN_ID_ARM_SEARCH_MOTOR_MAX_ANGLE_SPD: return CPPIER_MSG_SEARCH_MOTOR_MAX_ANGLE_SPD;
        case CAN_ID_ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET: return CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD_SET;
        case CAN_ID_ARM_JOINT_CONFIG:             return CPPIER_MSG_JOINT_CONFIG;
        case CAN_ID_ARM_INSTRUCTION_RESPONSE_CONFIG: return CPPIER_MSG_INSTRUCTION_RESPONSE_CONFIG;
        case CAN_ID_ARM_PARAM_ENQUIRY_AND_CONFIG: return CPPIER_MSG_PARAM_ENQUIRY_AND_CONFIG;
        case CAN_ID_ARM_END_VEL_ACC_PARAM_CONFIG: return CPPIER_MSG_END_VEL_ACC_PARAM_CONFIG;
        case CAN_ID_ARM_CRASH_PROTECTION_RATING_CONFIG: return CPPIER_MSG_CRASH_PROTECTION_CONFIG;
        case CAN_ID_ARM_REQ_MASTER_ARM_HOME:      return CPPIER_MSG_REQ_MASTER_ARM_HOME;
        case CAN_ID_ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG: return CPPIER_MSG_GRIPPER_TEACHING_PARAM_CONFIG;
        case CAN_ID_ARM_JOINT_MIT_CTRL_1:         return CPPIER_MSG_MIT_CTRL_1;
        case CAN_ID_ARM_JOINT_MIT_CTRL_2:         return CPPIER_MSG_MIT_CTRL_2;
        case CAN_ID_ARM_JOINT_MIT_CTRL_3:         return CPPIER_MSG_MIT_CTRL_3;
        case CAN_ID_ARM_JOINT_MIT_CTRL_4:         return CPPIER_MSG_MIT_CTRL_4;
        case CAN_ID_ARM_JOINT_MIT_CTRL_5:         return CPPIER_MSG_MIT_CTRL_5;
        case CAN_ID_ARM_JOINT_MIT_CTRL_6:         return CPPIER_MSG_MIT_CTRL_6;
        default: return CPPIER_MSG_UNKNOWN;
    }
}

void cpiper_msg_init(cpiper_message_t *msg) {
    memset(msg, 0, sizeof(cpiper_message_t));
    msg->type_ = CPPIER_MSG_UNKNOWN;
}

/* ─── Decode ─────────────────────────────────────────────────── */

bool cpiper_decode(uint32_t can_id, const uint8_t data[8],
                   cpiper_message_t *msg, double timestamp) {
    msg->time_stamp = timestamp;

    switch (can_id) {

    /* ── Feedback: status ── */
    case CAN_ID_ARM_STATUS_FEEDBACK:
        msg->type_ = CPPIER_MSG_STATUS_FEEDBACK;
        msg->status.ctrl_mode      = cpiper_get_u8(data, 0);
        msg->status.arm_status     = cpiper_get_u8(data, 1);
        msg->status.mode_feed      = cpiper_get_u8(data, 2);
        msg->status.teach_status   = cpiper_get_u8(data, 3);
        msg->status.motion_status  = cpiper_get_u8(data, 4);
        msg->status.trajectory_num = cpiper_get_u8(data, 5);
        msg->status.err_code       = cpiper_get_u16_be(data, 6);
        break;

    /* ── Feedback: end pose (split across 3 frames) ── */
    case CAN_ID_ARM_END_POSE_FEEDBACK_1:
        msg->type_ = CPPIER_MSG_END_POSE_FEEDBACK;
        msg->end_pose.X_axis = cpiper_get_i32_be(data, 0);
        msg->end_pose.Y_axis = cpiper_get_i32_be(data, 4);
        break;
    case CAN_ID_ARM_END_POSE_FEEDBACK_2:
        msg->type_ = CPPIER_MSG_END_POSE_FEEDBACK;
        msg->end_pose.Z_axis = cpiper_get_i32_be(data, 0);
        msg->end_pose.RX_axis = cpiper_get_i32_be(data, 4);
        break;
    case CAN_ID_ARM_END_POSE_FEEDBACK_3:
        msg->type_ = CPPIER_MSG_END_POSE_FEEDBACK;
        msg->end_pose.RY_axis = cpiper_get_i32_be(data, 0);
        msg->end_pose.RZ_axis = cpiper_get_i32_be(data, 4);
        break;

    /* ── Feedback: joint angles (split across 3 frames) ── */
    case CAN_ID_ARM_JOINT_FEEDBACK_12:
        msg->type_ = CPPIER_MSG_JOINT_FEEDBACK;
        msg->joint_state.joint_1 = cpiper_get_i32_be(data, 0);
        msg->joint_state.joint_2 = cpiper_get_i32_be(data, 4);
        break;
    case CAN_ID_ARM_JOINT_FEEDBACK_34:
        msg->type_ = CPPIER_MSG_JOINT_FEEDBACK;
        msg->joint_state.joint_3 = cpiper_get_i32_be(data, 0);
        msg->joint_state.joint_4 = cpiper_get_i32_be(data, 4);
        break;
    case CAN_ID_ARM_JOINT_FEEDBACK_56:
        msg->type_ = CPPIER_MSG_JOINT_FEEDBACK;
        msg->joint_state.joint_5 = cpiper_get_i32_be(data, 0);
        msg->joint_state.joint_6 = cpiper_get_i32_be(data, 4);
        break;

    /* ── Feedback: gripper ── */
    case CAN_ID_ARM_GRIPPER_FEEDBACK:
        msg->type_ = CPPIER_MSG_GRIPPER_FEEDBACK;
        msg->gripper.angle       = cpiper_get_i32_be(data, 0);
        msg->gripper.effort      = cpiper_get_i16_be(data, 4);
        msg->gripper.status_code = cpiper_get_u8(data, 6);
        break;

    /* ── Feedback: high-speed motor (6 motors) ── */
    case CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_1:
    case CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_2:
    case CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_3:
    case CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_4:
    case CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_5:
    case CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_6: {
        msg->type_ = CPPIER_MSG_HIGH_SPD_MOTOR_FEEDBACK;
        int idx = (can_id - CAN_ID_ARM_INFO_HIGH_SPD_FEEDBACK_1);
        msg->motor_high_spd[idx].can_id      = can_id;
        msg->motor_high_spd[idx].motor_speed = cpiper_get_i16_be(data, 0);
        msg->motor_high_spd[idx].current     = cpiper_get_i16_be(data, 2);
        msg->motor_high_spd[idx].pos         = cpiper_get_i32_be(data, 4);
        break;
    }

    /* ── Feedback: low-speed motor (6 motors) ── */
    case CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_1:
    case CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_2:
    case CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_3:
    case CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_4:
    case CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_5:
    case CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_6: {
        msg->type_ = CPPIER_MSG_LOW_SPD_MOTOR_FEEDBACK;
        int idx = (can_id - CAN_ID_ARM_INFO_LOW_SPD_FEEDBACK_1);
        msg->motor_low_spd[idx].can_id         = can_id;
        msg->motor_low_spd[idx].vol            = cpiper_get_u16_be(data, 0);
        msg->motor_low_spd[idx].foc_temp       = cpiper_get_i16_be(data, 2);
        msg->motor_low_spd[idx].motor_temp     = cpiper_get_i8(data, 4);
        msg->motor_low_spd[idx].foc_status_code = cpiper_get_u8(data, 5);
        msg->motor_low_spd[idx].bus_current    = cpiper_get_u16_be(data, 6);
        break;
    }

    /* ── Feedback: firmware ── */
    case CAN_ID_ARM_FIRMWARE_READ:
        msg->type_ = CPPIER_MSG_FIRMWARE;
        memcpy(msg->firmware.firmware_data, data, 8);
        break;

    /* ── Feedback: response set instruction ── */
    case CAN_ID_ARM_FEEDBACK_RESP_SET_INSTRUCTION:
        msg->type_ = CPPIER_MSG_RESP_SET_INSTRUCTION;
        msg->resp_instruction.instruction_index       = cpiper_get_u8(data, 0);
        msg->resp_instruction.is_set_zero_successfully = cpiper_get_u8(data, 1);
        break;

    /* ── Feedback: motor angle limit max speed ── */
    case CAN_ID_ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD:
        msg->type_ = CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD;
        msg->motor_angle_limit.motor_num       = cpiper_get_u8(data, 0);
        msg->motor_angle_limit.max_angle_limit = cpiper_get_i16_be(data, 1);
        msg->motor_angle_limit.min_angle_limit = cpiper_get_i16_be(data, 3);
        msg->motor_angle_limit.max_joint_spd   = cpiper_get_u16_be(data, 5);
        break;

    /* ── Feedback: end vel/acc param ── */
    case CAN_ID_ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM:
        msg->type_ = CPPIER_MSG_END_VEL_ACC_PARAM;
        msg->end_vel_acc_param.end_max_linear_vel  = cpiper_get_u16_be(data, 0);
        msg->end_vel_acc_param.end_max_angular_vel = cpiper_get_u16_be(data, 2);
        msg->end_vel_acc_param.end_max_linear_acc  = cpiper_get_u16_be(data, 4);
        msg->end_vel_acc_param.end_max_angular_acc = cpiper_get_u16_be(data, 6);
        break;

    /* ── Feedback: crash protection ── */
    case CAN_ID_ARM_CRASH_PROTECTION_RATING_FEEDBACK:
        msg->type_ = CPPIER_MSG_CRASH_PROTECTION_FEEDBACK;
        msg->crash_protection.joint_1_protection_level = cpiper_get_u8(data, 0);
        msg->crash_protection.joint_2_protection_level = cpiper_get_u8(data, 1);
        msg->crash_protection.joint_3_protection_level = cpiper_get_u8(data, 2);
        msg->crash_protection.joint_4_protection_level = cpiper_get_u8(data, 3);
        msg->crash_protection.joint_5_protection_level = cpiper_get_u8(data, 4);
        msg->crash_protection.joint_6_protection_level = cpiper_get_u8(data, 5);
        break;

    /* ── Feedback: motor max acc limit ── */
    case CAN_ID_ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT:
        msg->type_ = CPPIER_MSG_MOTOR_MAX_ACC_LIMIT;
        msg->motor_max_acc.joint_motor_num = cpiper_get_u8(data, 0);
        msg->motor_max_acc.max_joint_acc   = cpiper_get_u16_be(data, 1);
        break;

    /* ── Feedback: gripper/teaching pendant param ── */
    case CAN_ID_ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK:
        msg->type_ = CPPIER_MSG_GRIPPER_TEACHING_PARAM_FEEDBACK;
        msg->gripper_teaching_param.teaching_range_per = cpiper_get_u8(data, 0);
        msg->gripper_teaching_param.max_range_config   = cpiper_get_u8(data, 1);
        msg->gripper_teaching_param.teaching_friction   = cpiper_get_u8(data, 2);
        break;

    /* ── Decode feedback mirror of control msgs (for slave→master) ── */
    case CAN_ID_ARM_MOTION_CTRL_2:
        msg->type_ = CPPIER_MSG_MOTION_CTRL_2;
        msg->motion_ctrl_2.ctrl_mode          = cpiper_get_u8(data, 0);
        msg->motion_ctrl_2.move_mode          = cpiper_get_u8(data, 1);
        msg->motion_ctrl_2.move_spd_rate_ctrl = cpiper_get_u8(data, 2);
        msg->motion_ctrl_2.mit_mode           = cpiper_get_u8(data, 3);
        msg->motion_ctrl_2.residence_time     = cpiper_get_u8(data, 4);
        break;
    case CAN_ID_ARM_JOINT_CTRL_12:
        msg->type_ = CPPIER_MSG_JOINT_CTRL_12;
        msg->joint_ctrl.joint_1 = cpiper_get_i32_be(data, 0);
        msg->joint_ctrl.joint_2 = cpiper_get_i32_be(data, 4);
        break;
    case CAN_ID_ARM_JOINT_CTRL_34:
        msg->type_ = CPPIER_MSG_JOINT_CTRL_34;
        msg->joint_ctrl.joint_3 = cpiper_get_i32_be(data, 0);
        msg->joint_ctrl.joint_4 = cpiper_get_i32_be(data, 4);
        break;
    case CAN_ID_ARM_JOINT_CTRL_56:
        msg->type_ = CPPIER_MSG_JOINT_CTRL_56;
        msg->joint_ctrl.joint_5 = cpiper_get_i32_be(data, 0);
        msg->joint_ctrl.joint_6 = cpiper_get_i32_be(data, 4);
        break;
    case CAN_ID_ARM_GRIPPER_CTRL:
        msg->type_ = CPPIER_MSG_GRIPPER_CTRL;
        msg->gripper_ctrl.angle        = cpiper_get_i32_be(data, 0);
        msg->gripper_ctrl.effort       = cpiper_get_i16_be(data, 4);
        msg->gripper_ctrl.status_code  = cpiper_get_u8(data, 6);
        msg->gripper_ctrl.set_zero     = cpiper_get_u8(data, 7);
        break;

    default:
        return false;
    }
    return true;
}

/* ─── Encode ─────────────────────────────────────────────────── */

uint32_t cpiper_float_to_uint(float x_float, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (uint32_t)((x_float - offset) * (float)((1 << bits) - 1) / span);
}

bool cpiper_encode(const cpiper_message_t *msg, uint32_t *can_id,
                   uint8_t data[8]) {
    memset(data, 0, 8);
    *can_id = cpiper_msg_type_to_can_id(msg->type_);
    if (*can_id == 0) return false;

    switch (msg->type_) {

    case CPPIER_MSG_MOTION_CTRL_1:
        cpiper_put_u8(data, 0, msg->motion_ctrl_1.emergency_stop);
        cpiper_put_u8(data, 1, msg->motion_ctrl_1.track_ctrl);
        cpiper_put_u8(data, 2, msg->motion_ctrl_1.grag_teach_ctrl);
        /* bytes 3-7 = 0x00 */
        break;

    case CPPIER_MSG_MOTION_CTRL_2:
        cpiper_put_u8(data, 0, msg->motion_ctrl_2.ctrl_mode);
        cpiper_put_u8(data, 1, msg->motion_ctrl_2.move_mode);
        cpiper_put_u8(data, 2, msg->motion_ctrl_2.move_spd_rate_ctrl);
        cpiper_put_u8(data, 3, msg->motion_ctrl_2.mit_mode);
        cpiper_put_u8(data, 4, msg->motion_ctrl_2.residence_time);
        cpiper_put_u8(data, 5, msg->motion_ctrl_2.installation_pos);
        /* bytes 6-7 = 0x00 */
        break;

    case CPPIER_MSG_MOTION_CTRL_CARTESIAN_1:
        cpiper_put_i32_be(data, 0, msg->cartesian.X_axis);
        cpiper_put_i32_be(data, 4, msg->cartesian.Y_axis);
        break;
    case CPPIER_MSG_MOTION_CTRL_CARTESIAN_2:
        cpiper_put_i32_be(data, 0, msg->cartesian.Z_axis);
        cpiper_put_i32_be(data, 4, msg->cartesian.RX_axis);
        break;
    case CPPIER_MSG_MOTION_CTRL_CARTESIAN_3:
        cpiper_put_i32_be(data, 0, msg->cartesian.RY_axis);
        cpiper_put_i32_be(data, 4, msg->cartesian.RZ_axis);
        break;

    case CPPIER_MSG_JOINT_CTRL_12:
        cpiper_put_i32_be(data, 0, msg->joint_ctrl.joint_1);
        cpiper_put_i32_be(data, 4, msg->joint_ctrl.joint_2);
        break;
    case CPPIER_MSG_JOINT_CTRL_34:
        cpiper_put_i32_be(data, 0, msg->joint_ctrl.joint_3);
        cpiper_put_i32_be(data, 4, msg->joint_ctrl.joint_4);
        break;
    case CPPIER_MSG_JOINT_CTRL_56:
        cpiper_put_i32_be(data, 0, msg->joint_ctrl.joint_5);
        cpiper_put_i32_be(data, 4, msg->joint_ctrl.joint_6);
        break;

    case CPPIER_MSG_CIRCULAR_CTRL:
        cpiper_put_u8(data, 0, msg->circular_ctrl.instruction_num);
        /* bytes 1-7 = 0 */
        break;

    case CPPIER_MSG_GRIPPER_CTRL:
        cpiper_put_i32_be(data, 0, msg->gripper_ctrl.angle);
        cpiper_put_u16_be(data, 4, (uint16_t)msg->gripper_ctrl.effort);
        cpiper_put_u8(data, 6, msg->gripper_ctrl.status_code);
        cpiper_put_u8(data, 7, msg->gripper_ctrl.set_zero);
        break;

    case CPPIER_MSG_MASTER_SLAVE_CONFIG:
        cpiper_put_u8(data, 0, msg->ms_config.linkage_config);
        cpiper_put_u8(data, 1, msg->ms_config.feedback_offset);
        cpiper_put_u8(data, 2, msg->ms_config.ctrl_offset);
        cpiper_put_u8(data, 3, msg->ms_config.linkage_offset);
        /* bytes 4-7 = 0 */
        break;

    case CPPIER_MSG_MOTOR_ENABLE_DISABLE:
        cpiper_put_u8(data, 0, msg->motor_enable.motor_num);
        cpiper_put_u8(data, 1, msg->motor_enable.enable_flag);
        /* bytes 2-7 = 0 */
        break;

    case CPPIER_MSG_SEARCH_MOTOR_MAX_ANGLE_SPD:
        cpiper_put_u8(data, 0, msg->search_motor.motor_num);
        cpiper_put_u8(data, 1, msg->search_motor.search_content);
        /* bytes 2-7 = 0 */
        break;

    case CPPIER_MSG_MOTOR_ANGLE_LIMIT_MAX_SPD_SET:
        cpiper_put_u8(data, 0, msg->motor_angle_limit_set.motor_num);
        cpiper_put_i16_be(data, 1, msg->motor_angle_limit_set.max_angle_limit);
        cpiper_put_i16_be(data, 3, msg->motor_angle_limit_set.min_angle_limit);
        cpiper_put_u16_be(data, 5, msg->motor_angle_limit_set.max_joint_spd);
        /* byte 7 = 0 */
        break;

    case CPPIER_MSG_JOINT_CONFIG:
        cpiper_put_u8(data, 0, msg->joint_config.joint_motor_num);
        cpiper_put_u8(data, 1, msg->joint_config.set_motor_current_pos_as_zero);
        cpiper_put_u8(data, 2, msg->joint_config.acc_param_config_is_effective_or_not);
        cpiper_put_u16_be(data, 3, msg->joint_config.max_joint_acc);
        cpiper_put_u8(data, 5, msg->joint_config.clear_joint_err);
        /* bytes 6-7 = 0 */
        break;

    case CPPIER_MSG_INSTRUCTION_RESPONSE_CONFIG:
        cpiper_put_u8(data, 0, msg->instruction_response.instruction_index);
        cpiper_put_u8(data, 1, msg->instruction_response.zero_config_success_flag);
        /* bytes 2-7 = 0 */
        break;

    case CPPIER_MSG_PARAM_ENQUIRY_AND_CONFIG:
        cpiper_put_u8(data, 0, msg->param_enquiry.param_enquiry);
        cpiper_put_u8(data, 1, msg->param_enquiry.param_setting);
        cpiper_put_u8(data, 2, msg->param_enquiry.data_feedback_0x48x);
        cpiper_put_u8(data, 3, msg->param_enquiry.end_load_param_setting_effective);
        cpiper_put_u8(data, 4, msg->param_enquiry.set_end_load);
        /* bytes 5-7 = 0 */
        break;

    case CPPIER_MSG_END_VEL_ACC_PARAM_CONFIG:
        cpiper_put_u16_be(data, 0, msg->end_vel_acc_config.end_max_linear_vel);
        cpiper_put_u16_be(data, 2, msg->end_vel_acc_config.end_max_angular_vel);
        cpiper_put_u16_be(data, 4, msg->end_vel_acc_config.end_max_linear_acc);
        cpiper_put_u16_be(data, 6, msg->end_vel_acc_config.end_max_angular_acc);
        break;

    case CPPIER_MSG_CRASH_PROTECTION_CONFIG:
        cpiper_put_u8(data, 0, msg->crash_config.joint_1_protection_level);
        cpiper_put_u8(data, 1, msg->crash_config.joint_2_protection_level);
        cpiper_put_u8(data, 2, msg->crash_config.joint_3_protection_level);
        cpiper_put_u8(data, 3, msg->crash_config.joint_4_protection_level);
        cpiper_put_u8(data, 4, msg->crash_config.joint_5_protection_level);
        cpiper_put_u8(data, 5, msg->crash_config.joint_6_protection_level);
        /* bytes 6-7 = 0 */
        break;

    case CPPIER_MSG_GRIPPER_TEACHING_PARAM_CONFIG:
        cpiper_put_u8(data, 0, msg->gripper_teaching_config.teaching_range_per);
        cpiper_put_u8(data, 1, msg->gripper_teaching_config.max_range_config);
        cpiper_put_u8(data, 2, msg->gripper_teaching_config.teaching_friction);
        /* bytes 3-7 = 0 */
        break;

    case CPPIER_MSG_MIT_CTRL_1:
    case CPPIER_MSG_MIT_CTRL_2:
    case CPPIER_MSG_MIT_CTRL_3:
    case CPPIER_MSG_MIT_CTRL_4:
    case CPPIER_MSG_MIT_CTRL_5:
    case CPPIER_MSG_MIT_CTRL_6: {
        /* 16-bit pos_ref, 12-bit vel, 12-bit kp, 12-bit kd, 8-bit torque → 7 bytes + CRC */
        uint16_t pos = msg->mit_ctrl.pos_ref;
        uint16_t vel = msg->mit_ctrl.vel_ref;
        uint16_t kp  = msg->mit_ctrl.kp;
        uint16_t kd  = msg->mit_ctrl.kd;
        uint16_t t   = msg->mit_ctrl.t_ref;

        cpiper_put_u16_be(data, 0, pos);
        data[2] = (uint8_t)((vel >> 4) & 0xFF);
        data[3] = (uint8_t)(((vel & 0x0F) << 4) | ((kp >> 8) & 0x0F));
        data[4] = (uint8_t)(kp & 0xFF);
        data[5] = (uint8_t)((kd >> 4) & 0xFF);
        data[6] = (uint8_t)(((kd & 0x0F) << 4) | ((t >> 4) & 0x0F));

        /* CRC: XOR bytes 0-6, masked to 4 bits */
        uint8_t crc = (data[0] ^ data[1] ^ data[2] ^ data[3] ^
                       data[4] ^ data[5] ^ data[6]) & 0x0F;
        data[7] = (uint8_t)(((t & 0x0F) << 4) | crc);
        break;
    }

    default:
        return false;
    }
    return true;
}
