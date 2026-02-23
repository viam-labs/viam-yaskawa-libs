#ifndef ROBOT_NETWORK_H
#define ROBOT_NETWORK_H

#include "lock_api.h"
#include "platform_api.h"
#include "protocol.h"
#include <stdint.h>

// Callback function types
typedef struct {
    // Called when a new TCP client connects
    void (*on_connection)(const char *client_ip, uint16_t client_port, void *user_data);

    // Called when a TCP client disconnects
    void (*on_disconnection)(const char *client_ip, uint16_t client_port, void *user_data);

    // Called when a command is received from TCP client
    // Returns: >= 0 for success (sends OK), < 0 for error (sends ERROR with error code)
    command_response_context_t *(*handle_command)(protocol_header_t *header, void *payload, void *user_data);

    // Called when MSG_GET_ERROR_INFO is received
    // Returns: 0 if no error info, 1 if error info available (fills error_code and error_message)
    command_response_context_t *(*get_error_info)(int32_t *error_code, void *user_data);
} robot_network_callbacks_t;

// Configuration structure
typedef struct {
    uint16_t tcp_port;
    uint16_t udp_port;
    uint32_t connection_timeout_ms; // Timeout for idle connections (default 1000ms)
    robot_network_callbacks_t callbacks;
    void *user_data; // Passed to all callbacks
} robot_server_config_t;

// Single client connection info
typedef struct {
    int socket;
    struct sockaddr_in addr;
    time_t last_command;
    _Bool connected;
    char peer_ip[INET_ADDRSTRLEN];
    uint16_t peer_port;
    uint16_t registered_udp_port; // Port registered by client for UDP status messages (0 = not registered)
    _Bool udp_port_registered;    // Flag indicating if UDP port has been registered
} client_info_t;

// Network server structure
typedef struct {
    int tcp_socket;
    int udp_socket;
    int running;
    THREAD_TYPE tcp_thread;
    THREAD_TYPE timeout_thread;
    LOCK_ID client_mutex;
    robot_server_config_t config;
    client_info_t client; // Single client only
} robot_server_ctx_t;

// Create and initialize network server
robot_server_ctx_t *robot_protocol_create(const robot_server_config_t *config);

// Start the network server (blocking call)
int robot_protocol_start(robot_server_ctx_t *ctx);

// Stop the network server
void robot_protocol_stop(robot_server_ctx_t *ctx);

// Send UDP status message to connected clients
int robot_protocol_send_position_velocity_torque(robot_server_ctx_t *ctx, const status_payload_t *status);

// Send UDP robot status message to connected clients
int robot_protocol_send_robot_status(robot_server_ctx_t *ctx, const robot_status_payload_t *status);

// Cleanup and destroy network server
void robot_protocol_destroy(robot_server_ctx_t *ctx);

// Check if server is running
int robot_protocol_is_running(robot_server_ctx_t *ctx);

#endif // ROBOT_NETWORK_H
