#include "logging.h"
#include "udp_broadcast.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define BUFFER_SIZE 8192
static char log_buffer[BUFFER_SIZE];
static ring_buffer_t rb;
static udp_broadcast_config_t udp_config;
void *logging_thread(void *arg) {
    (void) arg;
    int counter = 0;
    while (1) {
        pr_info("Hello world %d", counter++);
        sleep(1);
    }
    return NULL;
}

void *udp_server_thread(void *arg) {
    (void) arg;
    while (1) {
        int result = process_log_message(&udp_config);
        if (result == 0) {
            sleep(0);
        }
    }
    return NULL;
}

int main() {
    printf("Starting UDP broadcast test server...\n");
    if (ring_buffer_initialize(&rb, log_buffer, BUFFER_SIZE) != 0) {
        printf("Failed to initialize ring buffer\n");
        return 1;
    }
    if (logging_initialize(&rb) != 0) {
        printf("Failed to initialize logging\n");
        return 1;
    }
    logging_set_level(LOG_INFO);
    if (udp_broadcast_initialize(&udp_config, &rb, UDP_BROADCAST_DEFAULT_PORT, NULL) != 0) {
        printf("Failed to initialize UDP broadcast\n");
        return 1;
    }
    pthread_t log_thread, udp_thread;
    if (pthread_create(&log_thread, NULL, logging_thread, NULL) != 0) {
        printf("Failed to create logging thread\n");
        return 1;
    }
    if (pthread_create(&udp_thread, NULL, udp_server_thread, NULL) != 0) {
        printf("Failed to create UDP server thread\n");
        return 1;
    }
    printf("UDP broadcast server running on port %d\n", UDP_BROADCAST_DEFAULT_PORT);
    printf("Press Ctrl+C to stop...\n");
    pthread_join(log_thread, NULL);
    pthread_join(udp_thread, NULL);
    udp_broadcast_shutdown(&udp_config);
    logging_shutdown();
    ring_buffer_destroy(&rb);
    return 0;
}
