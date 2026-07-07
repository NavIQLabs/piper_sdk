/**
 * @file kinematics.c
 * @brief Forward kinematics for Piper robot arm (DH-based)
 */
#include "cpiper/kinematics.h"
#include <math.h>

#define RAD2DEG  (180.0f / 3.14159265358979f)

void cpiper_fk_init(cpiper_fk_dh_t *dh, int dh_is_offset) {
    dh->dh_is_offset = dh_is_offset;

    /* Link lengths (mm) */
    dh->a[0] = 0.0f;  dh->a[1] = 0.0f;  dh->a[2] = 285.03f;
    dh->a[3] = -21.98f; dh->a[4] = 0.0f;  dh->a[5] = 0.0f;

    /* Link twists (rad) */
    dh->alpha[0] = 0.0f;
    dh->alpha[1] = -3.14159265f / 2.0f;
    dh->alpha[2] = 0.0f;
    dh->alpha[3] =  3.14159265f / 2.0f;
    dh->alpha[4] = -3.14159265f / 2.0f;
    dh->alpha[5] =  3.14159265f / 2.0f;

    /* Joint angle offsets (rad) */
    if (dh_is_offset == 0x01) {
        dh->theta[0] = 0.0f;
        dh->theta[1] = -3.14159265f * 172.22f / 180.0f;
        dh->theta[2] = -102.78f / 180.0f * 3.14159265f;
        dh->theta[3] = 0.0f;
        dh->theta[4] = 0.0f;
        dh->theta[5] = 0.0f;

        dh->init_pos[0] = 56.128f; dh->init_pos[1] = 0.0f;
        dh->init_pos[2] = 213.266f; dh->init_pos[3] = 0.0f;
        dh->init_pos[4] = 85.0f;  dh->init_pos[5] = 0.0f;
    } else {
        dh->theta[0] = 0.0f;
        dh->theta[1] = -3.14159265f * 174.22f / 180.0f;
        dh->theta[2] = -100.78f / 180.0f * 3.14159265f;
        dh->theta[3] = 0.0f;
        dh->theta[4] = 0.0f;
        dh->theta[5] = 0.0f;

        dh->init_pos[0] = 55.0f;  dh->init_pos[1] = 0.0f;
        dh->init_pos[2] = 205.0f; dh->init_pos[3] = 0.0f;
        dh->init_pos[4] = 85.0f;  dh->init_pos[5] = 0.0f;
    }

    /* Link offsets (mm) */
    dh->d[0] = 123.0f; dh->d[1] = 0.0f; dh->d[2] = 0.0f;
    dh->d[3] = 250.75f; dh->d[4] = 0.0f; dh->d[5] = 91.0f;
}

static void mat_mul(const float *a, const float *b, float *out,
                    int m, int l, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            float tmp = 0.0f;
            for (int k = 0; k < l; k++) {
                tmp += a[l * i + k] * b[n * k + j];
            }
            out[n * i + j] = tmp;
        }
    }
}

static void link_transform(float alpha, float a, float theta, float d,
                           float *T) {
    float ca = cosf(alpha), sa = sinf(alpha);
    float ct = cosf(theta), st = sinf(theta);

    T[0]  = ct;          T[1]  = -st;         T[2]  = 0.0f;  T[3]  = a;
    T[4]  = st * ca;     T[5]  = ct * ca;     T[6]  = -sa;   T[7]  = -sa * d;
    T[8]  = st * sa;     T[9]  = ct * sa;     T[10] = ca;     T[11] = ca * d;
    T[12] = 0.0f;        T[13] = 0.0f;        T[14] = 0.0f;  T[15] = 1.0f;
}

static void matrix_to_euler(const float *T, float *pos) {
    /* Extract position */
    pos[0] = T[3];   /* x */
    pos[1] = T[7];   /* y */
    pos[2] = T[11];  /* z */

    /* Euler angles (roll, pitch, yaw) in degrees */
    if (T[8] < -1.0f + 0.0001f) {
        pos[4] = 90.0f;   /* pitch */
        pos[5] = 0.0f;    /* yaw */
        pos[3] = atan2f(T[1], T[5]) * RAD2DEG; /* roll */
    } else if (T[8] > 1.0f - 0.0001f) {
        pos[4] = -90.0f;
        pos[5] = 0.0f;
        pos[3] = -atan2f(T[1], T[5]) * RAD2DEG;
    } else {
        float bt = atan2f(-T[8], sqrtf(T[0] * T[0] + T[4] * T[4]));
        pos[4] = bt * RAD2DEG;
        pos[5] = atan2f(T[4] / cosf(bt), T[0] / cosf(bt)) * RAD2DEG;
        pos[3] = atan2f(T[9] / cosf(bt), T[10] / cosf(bt)) * RAD2DEG;
    }
}

void cpiper_fk_calculate(cpiper_fk_dh_t *dh, const float joints[6],
                         float pos[6]) {
    float Rt[6][16];
    float R02[16], R03[16], R04[16], R05[16], R06[16];

    /* Compute individual link transforms */
    for (int i = 0; i < 6; i++) {
        float c_theta = joints[i] + dh->theta[i];
        link_transform(dh->alpha[i], dh->a[i], c_theta, dh->d[i], Rt[i]);
    }

    /* Chain multiply: T06 = T01 * T12 * T23 * T34 * T45 * T56 */
    mat_mul(Rt[0], Rt[1], R02, 4, 4, 4);
    mat_mul(R02, Rt[2], R03, 4, 4, 4);
    mat_mul(R03, Rt[3], R04, 4, 4, 4);
    mat_mul(R04, Rt[4], R05, 4, 4, 4);
    mat_mul(R05, Rt[5], R06, 4, 4, 4);

    /* Extract Euler angles for end-effector */
    matrix_to_euler(R06, pos);
}
