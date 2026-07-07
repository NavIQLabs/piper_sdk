/**
 * @file fps.c
 * @brief FPS (frames per second) counter
 */
#include "cpiper/fps.h"

void cpiper_fps_init(cpiper_fps_t *fps) {
    fps->frame_count = 0;
    fps->last_time = 0.0;
    fps->hz = 0.0;
}

void cpiper_fps_update(cpiper_fps_t *fps, double current_time) {
    fps->frame_count++;
    if (fps->last_time == 0.0) {
        fps->last_time = current_time;
        return;
    }
    double elapsed = current_time - fps->last_time;
    if (elapsed >= 1.0) {
        fps->hz = (double)fps->frame_count / elapsed;
        fps->frame_count = 0;
        fps->last_time = current_time;
    }
}

double cpiper_fps_get(cpiper_fps_t *fps) {
    return fps->hz;
}
