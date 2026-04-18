#define _GNU_SOURCE
#include "logging.h"
#include "mock_robot.h"
#include "platform_api.h"
#include "protocol.h"
#include "pthread.h"
#include "robot_protocol.h"
#include "udp_broadcast.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
static int g_running = 1;
static mock_robot_t g_robot;
static robot_server_ctx_t *g_ctx = NULL;

// Structure to pass additional data through user_data
typedef struct {
    mock_robot_t *robot;
    char response_payload[512];
    int has_response_payload;
} server_context_t;

static server_context_t g_server_context;

extern int64_t get_timestamp_ms();

static void signal_handler(int sig) {
    pr_info("[SERVER] Received signal %d, shutting down...", sig);
    g_running = 0;
}

static void status_loop() {
    status_payload_t status;
    robot_status_payload_t robot_status;
    int64_t last_status_time = 0;
    int64_t last_robot_status_time = 0;
    const int64_t status_interval_ms = HEARTBEAT_INTERVAL_MS; // 10ms
    const int64_t robot_status_interval_ms = 100;             // 100ms
    while (g_running && robot_protocol_is_running(g_ctx)) {
        int64_t now = get_timestamp_ms();

        // Send position/velocity/torque status updates at regular intervals
        if (now - last_status_time >= status_interval_ms) {
            mock_robot_get_status(&g_robot, &status, now, 0);
            mock_robot_update(&g_robot, now);
            int sent = robot_protocol_send_position_velocity_torque(g_ctx, &status);
            if (sent > 0) {
                static int status_count = 0;
                status_count++;
                if (status_count % 100 == 0) { // Print every second
                    pr_info("[SERVER] Sent status #%d to %d clients", status_count, sent);
                }
            }
            last_status_time = now;
        }
        // Send robot status updates at a slower interval
        if (now - last_robot_status_time >= robot_status_interval_ms) {
            mock_robot_get_robot_status(&g_robot, &robot_status, now, 0);
            int sent = robot_protocol_send_robot_status(g_ctx, &robot_status);
            if (sent > 0) {
                static int robot_status_count = 0;
                robot_status_count++;
                if (robot_status_count % 10 == 0) { // Print every second
                    pr_info("[SERVER] Sent robot status #%d to %d clients", robot_status_count, sent);
                }
            }
            last_robot_status_time = now;
        }

        platform_usleep(1000); // 1ms sleep to prevent busy waiting
    }
}

#define BUFFER_SIZE 8192

static char log_buffer[BUFFER_SIZE];
static ring_buffer_t rb;
// static udp_broadcast_config_t udp_config;

void *fn_print_log_thread(void *arg) {
    (void) (arg);
    while (1) {
        log_to_stdout();
        // process_log_message(&udp_config);
        platform_usleep(100000);
    }
    return NULL;
}

int main() {
    if (ring_buffer_initialize(&rb, log_buffer, BUFFER_SIZE) != 0) {
        printf("Failed to initialize ring buffer\n");
        return 1;
    }

    if (logging_initialize(&rb) != 0) {
        printf("Failed to initialize logging\n");
        return 1;
    }
    // bzero(&udp_config, sizeof(udp_config));
    // if (udp_broadcast_initialize(&udp_config, &rb, UDP_BROADCAST_DEFAULT_PORT, "wlan0") != 0) {
    ///  printf("Failed to initialize udp broadcast logging\n");
    //  return 1;
    //}

    logging_set_level(LOG_INFO);
    pthread_t print_log_thread;

    if (pthread_create(&print_log_thread, NULL, fn_print_log_thread, NULL) != 0) {
        printf("Failed to create log thread\n");
        return 1;
    }
    pr_info("[SERVER] Robot Server with Network Library");

    // Setup signal handling
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize mock robot and server context
    mock_robot_init(&g_robot);
    g_server_context.robot = &g_robot;
    g_server_context.has_response_payload = 0;
    memset(g_server_context.response_payload, 0, sizeof(g_server_context.response_payload));

    // Configure network
    robot_server_config_t config = {.tcp_port = TCP_PORT,
                                    .udp_port = UDP_PORT,
                                    .connection_timeout_ms = 1000, // 1 second timeout
                                    .callbacks = {.on_connection = mock_robot_on_connection,
                                                  .on_disconnection = mock_robot_on_disconnection,
                                                  .handle_command = mock_robot_handle_command,
                                                  .get_error_info = mock_robot_get_error_info_callback},
                                    .user_data = g_server_context.robot};

    // Create and start network server
    g_ctx = robot_protocol_create(&config);
    if (!g_ctx) {
        pr_error("[SERVER] Failed to create network server");
        return 1;
    }
    if (robot_protocol_start(g_ctx) < 0) {
        pr_error("[SERVER] Failed to start network server");
        robot_protocol_destroy(g_ctx);
        return 1;
    }
    pr_info("[SERVER] Server ready. Press Ctrl+C to stop.");

    // Main status loop
    status_loop();
    pr_info("[SERVER] Shutting down...");

    // Cleanup
    robot_protocol_stop(g_ctx);
    robot_protocol_destroy(g_ctx);
    pr_info("[SERVER] Server shutdown complete");
    pthread_cancel(print_log_thread);
    logging_shutdown();
    return 0;
}
