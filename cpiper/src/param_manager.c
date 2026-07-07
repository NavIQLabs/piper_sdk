/**
 * @file param_manager.c
 * @brief Joint/gripper limit parameter manager (singleton)
 */
#include "cpiper/param_manager.h"
#include <math.h>

static cpiper_param_manager_t g_param_manager;
static int g_initialized = 0;

cpiper_param_manager_t* cpiper_param_manager_get(void) {
    if (!g_initialized) {
        cpiper_param_manager_init();
    }
    return &g_param_manager;
}

void cpiper_param_manager_init(void) {
    /* Default joint limits: ±3.14159 rad (±180°) */
    for (int i = 0; i < 6; i++) {
        g_param_manager.joint_min[i] = -3.14159f;
        g_param_manager.joint_max[i] =  3.14159f;
    }
    /* Override J2/J3 to ±2.0944 rad (±120°) like Python SDK */
    g_param_manager.joint_min[1] = -2.0944f;
    g_param_manager.joint_max[1] =  2.0944f;
    g_param_manager.joint_min[2] = -2.0944f;
    g_param_manager.joint_max[2] =  2.0944f;

    g_param_manager.gripper_min = 0.0f;
    g_param_manager.gripper_max = 150.0f;

    g_param_manager.sdk_joint_limit_enabled = true;
    g_param_manager.sdk_gripper_limit_enabled = true;
    g_initialized = 1;
}

void cpiper_param_set_joint_limit(int joint, float min_val, float max_val) {
    if (joint < 1 || joint > 6) return;
    g_param_manager.joint_min[joint - 1] = min_val;
    g_param_manager.joint_max[joint - 1] = max_val;
}

void cpiper_param_get_joint_limit(int joint, float *min_val, float *max_val) {
    if (joint < 1 || joint > 6) return;
    *min_val = g_param_manager.joint_min[joint - 1];
    *max_val = g_param_manager.joint_max[joint - 1];
}

void cpiper_param_set_gripper_range(float min_val, float max_val) {
    g_param_manager.gripper_min = min_val;
    g_param_manager.gripper_max = max_val;
}

void cpiper_param_get_gripper_range(float *min_val, float *max_val) {
    *min_val = g_param_manager.gripper_min;
    *max_val = g_param_manager.gripper_max;
}

float cpiper_param_clamp_joint(int joint, float value) {
    if (!g_param_manager.sdk_joint_limit_enabled) return value;
    if (joint < 1 || joint > 6) return value;
    float min_v = g_param_manager.joint_min[joint - 1];
    float max_v = g_param_manager.joint_max[joint - 1];
    if (value < min_v) return min_v;
    if (value > max_v) return max_v;
    return value;
}

float cpiper_param_clamp_gripper(float value) {
    if (!g_param_manager.sdk_gripper_limit_enabled) return value;
    if (value < g_param_manager.gripper_min) return g_param_manager.gripper_min;
    if (value > g_param_manager.gripper_max) return g_param_manager.gripper_max;
    return value;
}
