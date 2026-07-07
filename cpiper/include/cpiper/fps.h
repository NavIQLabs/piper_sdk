/**
 * @file fps.h
 * @brief FPS (frames per second) counter
 */
#ifndef CPIPER_FPS_H
#define CPIPER_FPS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t frame_count;
    double   last_time;
    double   hz;
} cpiper_fps_t;

void cpiper_fps_init(cpiper_fps_t *fps);
void cpiper_fps_update(cpiper_fps_t *fps, double current_time);
double cpiper_fps_get(cpiper_fps_t *fps);

#ifdef __cplusplus
}
#endif

#endif /* CPIPER_FPS_H */
