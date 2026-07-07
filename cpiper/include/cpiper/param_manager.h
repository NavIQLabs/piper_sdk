/**
 * @file param_manager.h
 * @brief Joint/gripper limit parameter manager (singleton)
 */
#ifndef CPIPER_PARAM_MANAGER_H
#define CPIPER_PARAM_MANAGER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Joint limits: [min, max] in radians */
    float joint_min[6];
    float joint_max[6];
    /* Gripper range: [min, max] in mm */
    float gripper_min;
    float gripper_max;
    /* SDK limit enforcement enabled flags */
    bool sdk_joint_limit_enabled;
    bool sdk_gripper_limit_enabled;
} cpiper_param_manager_t;

/**
 * Get the singleton parameter manager instance.
 */
cpiper_param_manager_t* cpiper_param_manager_get(void);

/**
 * Initialize the singleton with default values.
 */
void cpiper_param_manager_init(void);

/**
 * Set joint limits for a specific joint (1-6).
 */
void cpiper_param_set_joint_limit(int joint, float min_val, float max_val);

/**
 * Get joint limits for a specific joint (1-6).
 */
void cpiper_param_get_joint_limit(int joint, float *min_val, float *max_val);

/**
 * Set gripper range limits.
 */
void cpiper_param_set_gripper_range(float min_val, float max_val);

/**
 * Get gripper range limits.
 */
void cpiper_param_get_gripper_range(float *min_val, float *max_val);

/**
 * Clamp joint value to SDK limits.
 * @return clamped value
 */
float cpiper_param_clamp_joint(int joint, float value);

/**
 * Clamp gripper value to SDK limits.
 * @return clamped value
 */
float cpiper_param_clamp_gripper(float value);

#ifdef __cplusplus
}
#endif

#endif /* CPIPER_PARAM_MANAGER_H */
