/**
 * @file send_zeros.c
 * @brief Send all joints to 0 position
 *
 * Usage: sudo ./send_zeros [can_name]
 * Default: vcan0
 *
 * Verify with: candump vcan0
 */
#include <cpiper/cpiper.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

static volatile int running = 1;

void sig_handler(int sig) {
    (void)sig;
    running = 0;
}

int main(int argc, char *argv[]) {
    const char *can_name = "vcan0";
    if (argc > 1) can_name = argv[1];

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    cpiper_interface_t arm;
    cpiper_init(&arm, can_name);

    printf("Connecting to %s...\n", can_name);
    int ret = cpiper_connect(&arm, false, false);
    if (ret < 0) {
        fprintf(stderr, "Failed to connect: %d\n", ret);
        return 1;
    }
    printf("Connected.\n");

    /* Enable all motors */
    cpiper_enable_piper(&arm);
    usleep(100000);

    /* Set motion mode: joint control */
    cpiper_motion_ctrl_2(&arm, 0x01, 0x00, 50, 0x00, 0);

    printf("Sending all joints to 0 (Ctrl+C to stop)...\n");

    while (running) {
        cpiper_joint_ctrl(&arm,
                          0.0f, 0.0f, 0.0f,
                          0.0f, 0.0f, 0.0f);

        cpiper_poll(&arm);

        cpiper_joint_state_t j = cpiper_get_joints(&arm);
        printf("\rJ1:%.2f J2:%.2f J3:%.2f J4:%.2f J5:%.2f J6:%.2f",
               j.joint_1 * 0.001, j.joint_2 * 0.001,
               j.joint_3 * 0.001, j.joint_4 * 0.001,
               j.joint_5 * 0.001, j.joint_6 * 0.001);
        fflush(stdout);

        usleep(2000); /* ~500 Hz */
    }

    printf("\nDisconnecting...\n");
    cpiper_disable_piper(&arm);
    cpiper_destroy(&arm);
    printf("Done.\n");
    return 0;
}
