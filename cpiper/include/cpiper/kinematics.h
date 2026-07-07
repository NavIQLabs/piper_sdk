/**
 * @file kinematics.h
 * @brief Forward kinematics for Piper robot arm (DH-based)
 */
#ifndef CPIPER_KINEMATICS_H
#define CPIPER_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * DH parameters table (set once at init).
 */
typedef struct {
    float a[6];       /* link lengths, mm */
    float alpha[6];   /* link twists, rad */
    float theta[6];   /* joint angle offsets, rad */
    float d[6];       /* link offsets, mm */
    float init_pos[6]; /* initial pose [x,y,z,rx,ry,rz], mm/deg */
    int dh_is_offset;  /* 1 = apply 2° J2/J3 offset for firmware >= S-V1.6-3 */
} cpiper_fk_dh_t;

/**
 * Initialize DH parameters.
 * @param dh_is_offset  1 = apply firmware offset, 0 = no offset
 */
void cpiper_fk_init(cpiper_fk_dh_t *dh, int dh_is_offset);

/**
 * Calculate forward kinematics for given joint angles.
 * @param dh     DH parameters
 * @param joints 6 joint angles in radians
 * @param pos    Output: [x, y, z, roll, pitch, yaw] - xyz in mm, rpy in degrees
 */
void cpiper_fk_calculate(cpiper_fk_dh_t *dh, const float joints[6], float pos[6]);

#ifdef __cplusplus
}
#endif

#endif /* CPIPER_KINEMATICS_H */
