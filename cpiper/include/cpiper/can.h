/**
 * @file can.h
 * @brief CAN bus abstraction for Linux socketcan
 */
#ifndef CPIPER_CAN_H
#define CPIPER_CAN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fd;                  /* socketcan file descriptor, -1 if closed */
    char name[16];           /* e.g. "can0", "vcan0" */
} cpiper_can_t;

/**
 * Initialize and open a CAN socket.
 * @param can    Output: CAN handle
 * @param name   Interface name, e.g. "can0" or "vcan0"
 * @param judge  If true, validate socket exists and is UP with correct bitrate
 * @return 0 on success, negative error code on failure
 */
int cpiper_can_init(cpiper_can_t *can, const char *name, bool judge);

/**
 * Close the CAN socket.
 */
void cpiper_can_close(cpiper_can_t *can);

/**
 * Non-blocking send of a CAN frame.
 * @return 0 on success, negative on failure
 */
int cpiper_can_send(cpiper_can_t *can, uint32_t can_id, const uint8_t data[8]);

/**
 * Non-blocking receive of a CAN frame.
 * @param can_id  Output: arbitration ID
 * @param data    Output: 8-byte payload
 * @param timestamp Output: timestamp in seconds (from kernel)
 * @return 1 if a frame was received, 0 if no data available, negative on error
 */
int cpiper_can_recv(cpiper_can_t *can, uint32_t *can_id, uint8_t data[8],
                    double *timestamp);

#ifdef __cplusplus
}
#endif

#endif /* CPIPER_CAN_H */
