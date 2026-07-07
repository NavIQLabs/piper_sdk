/**
 * @file can.c
 * @brief CAN bus abstraction for Linux socketcan (non-blocking)
 */
#include "cpiper/can.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <linux/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int cpiper_can_init(cpiper_can_t *can, const char *name, bool judge) {
    if (!can || !name) return -1;

    can->fd = -1;
    strncpy(can->name, name, sizeof(can->name) - 1);
    can->name[sizeof(can->name) - 1] = '\0';

    /* Create CAN socket */
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        fprintf(stderr, "cpiper_can: socket() failed: %s\n", strerror(errno));
        return -2;
    }

    /* Validation mode */
    if (judge) {
        /* Check interface exists */
        char path[64];
        snprintf(path, sizeof(path), "/sys/class/net/%s", name);
        /* Try to open the interface */
        struct ifreq ifr;
        memset(&ifr, 0, sizeof(ifr));
        strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
        if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
            fprintf(stderr, "cpiper_can: interface %s not found\n", name);
            close(s);
            return -3;
        }
    }

    /* Bind to CAN interface */
    struct sockaddr_can addr;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        fprintf(stderr, "cpiper_can: SIOCGIFINDEX failed for %s: %s\n",
                name, strerror(errno));
        close(s);
        return -4;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "cpiper_can: bind() failed for %s: %s\n",
                name, strerror(errno));
        close(s);
        return -5;
    }

    /* Set non-blocking */
    int flags = fcntl(s, F_GETFL, 0);
    if (flags < 0) {
        fprintf(stderr, "cpiper_can: fcntl(F_GETFL) failed: %s\n", strerror(errno));
        close(s);
        return -6;
    }
    if (fcntl(s, F_SETFL, flags | O_NONBLOCK) < 0) {
        fprintf(stderr, "cpiper_can: fcntl(F_SETFL) failed: %s\n", strerror(errno));
        close(s);
        return -7;
    }

    can->fd = s;
    return 0;
}

void cpiper_can_close(cpiper_can_t *can) {
    if (can && can->fd >= 0) {
        close(can->fd);
        can->fd = -1;
    }
}

int cpiper_can_send(cpiper_can_t *can, uint32_t can_id, const uint8_t data[8]) {
    if (!can || can->fd < 0) return -1;

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id;
    frame.can_dlc = 8;
    memcpy(frame.data, data, 8);

    ssize_t n = write(can->fd, &frame, sizeof(frame));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -2;
    }
    return 0;
}

int cpiper_can_recv(cpiper_can_t *can, uint32_t *can_id, uint8_t data[8],
                    double *timestamp) {
    if (!can || can->fd < 0) return -1;

    struct can_frame frame;
    ssize_t n = read(can->fd, &frame, sizeof(frame));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -2;
    }
    if ((size_t)n < sizeof(struct can_frame)) return -3;

    *can_id = frame.can_id & CAN_ERR_MASK;  /* strip EFF/RTR/ERR flags */
    memcpy(data, frame.data, 8);

    /* Get timestamp from kernel via ioctl */
    struct timeval tv;
    if (ioctl(can->fd, SIOCGSTAMP, &tv) == 0) {
        *timestamp = (double)tv.tv_sec + (double)tv.tv_usec / 1e6;
    } else {
        /* Fallback: use current time */
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        *timestamp = (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
    }

    return 1;
}
