#include "robot_protocol.h"
#include "lock_api.h"
#include "logging.h"
#include "platform_api.h"
#include "protocol.h"
#include <errno.h>

#define MAX_PAYLOAD_SIZE 40016

int64_t get_timestamp_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_TYPE, &ts);
    return (int64_t) ts.tv_sec * 1000 + (int64_t) ts.tv_nsec / 1000000;
}

move_goal_payload_t *move_goal_from_payload(void *data, uint32_t size) {
    // Validate minimum size for header
    if (size < sizeof(move_goal_payload_t)) {
        pr_error("[PROTO] Payload too small for move_goal header: %u bytes (need %zu)", size,
                 sizeof(move_goal_payload_t));
        return NULL;
    }

    // Cast payload to move_goal structure (zero-copy)
    move_goal_payload_t *goal = (move_goal_payload_t *) data;
    pr_info("go moved goal %d %d %d", goal->number_of_axes_controlled, goal->group_index, goal->trajectory_size);
    // Validate trajectory size
    if (goal->trajectory_size == 0) {
        pr_error("[PROTO] Invalid trajectory_size: 0");
        return NULL;
    }

    // Calculate required size up to tolerance_size field
    uint32_t size_to_tol_size = sizeof(move_goal_payload_t) + goal->trajectory_size * sizeof(trajectory_point_t);

    if (size < size_to_tol_size + sizeof(uint32_t)) {
        pr_error("[PROTO] Payload too small for trajectory data: %u bytes (need %u)", size,
                 (uint32_t) (size_to_tol_size + sizeof(uint32_t)));
        return NULL;
    }

    // Get tolerance size
    uint32_t *tolerance_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(goal);
    uint32_t tolerance_size = *tolerance_size_ptr;

    // Validate tolerance size matches trajectory size (or is zero)
    if (tolerance_size != 0 && tolerance_size != goal->trajectory_size) {
        pr_error("[PROTO] Tolerance size mismatch: traj=%u, tol=%u (must match or be 0)", goal->trajectory_size,
                 tolerance_size);
        return NULL;
    }

    // Calculate total required size
    uint32_t required_size = MOVE_GOAL_CALC_SIZE(goal->trajectory_size, tolerance_size);

    if (size < required_size) {
        pr_error("[PROTO] Payload too small: %u bytes (need %u for %u trajectory + %u tolerance points)", size,
                 required_size, goal->trajectory_size, tolerance_size);
        return NULL;
    }

    pr_debug("[PROTO] move_goal validated: axes=%u, group=%u, traj_pts=%u, tol_pts=%u, size=%u bytes",
             goal->number_of_axes_controlled, goal->group_index, goal->trajectory_size, tolerance_size, required_size);

    return goal;
}

move_goal_payload_t *move_goal_deep_copy(const move_goal_payload_t *src, void *dest_buffer, uint32_t dest_size) {
    if (!src || !dest_buffer) {
        return NULL;
    }

    // Get tolerance size from source
    const uint32_t *src_tol_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(src);
    uint32_t tolerance_size = *src_tol_size_ptr;

    // Calculate required size
    uint32_t required_size = MOVE_GOAL_CALC_SIZE(src->trajectory_size, tolerance_size);

    if (dest_size < required_size) {
        pr_error("[PROTO] Destination buffer too small for deep copy: %u bytes (need %u)", dest_size, required_size);
        return NULL;
    }

    // Copy entire payload (header + trajectory + tolerance_size + tolerance data)
    memcpy(dest_buffer, src, required_size);

    pr_debug("[PROTO] move_goal deep copied: %u bytes (%u traj + %u tol points)", required_size, src->trajectory_size,
             tolerance_size);

    return (move_goal_payload_t *) dest_buffer;
}

command_response_context_t *allocate_response_context(uint32_t length, uint8_t type) {
    command_response_context_t *ctx = platform_malloc(sizeof(command_response_context_t));
    if (ctx == NULL)
        return NULL;
    bzero(ctx, sizeof(command_response_context_t));
    ctx->header = platform_malloc(sizeof(protocol_header_t));

    if (ctx->header == NULL)
        goto ctx_cleanup;

    ctx->header->magic_number = PROTOCOL_MAGIC_NUMBER;
    ctx->header->payload_length = length;
    ctx->header->version = PROTOCOL_VERSION;
    ctx->header->message_type = type;
    ctx->header->timestamp_ms = get_timestamp_ms();

    if (length > 0) {
        ctx->payload = platform_malloc(length);
        if (ctx->payload == NULL)
            goto header_cleanup;
    }
    return ctx;

header_cleanup:
    platform_free(ctx->header);
ctx_cleanup:
    platform_free(ctx);

    return NULL;
}

void free_command_response_context(command_response_context_t *ctx) {
    if (ctx->payload)
        platform_free(ctx->payload);

    platform_free(ctx->header);
    platform_free(ctx);
}

static int send_command_response_context(int socket, command_response_context_t *ctx) {
    unsigned long int sent = platform_send(socket, (const char *) ctx->header, sizeof(protocol_header_t), 0);
    if (ctx->payload && ctx->header->payload_length) {
        sent += platform_send(socket, (const char *) ctx->payload, ctx->header->payload_length, 0);
    }
    int ret = sent == sizeof(protocol_header_t) + ctx->header->payload_length ? 0 : -1;
    free_command_response_context(ctx);
    return ret;
}

// Helper function to send ERROR_INFO response message

static int add_client(robot_server_ctx_t *ctx, int socket, struct sockaddr_in *addr) {
    lock_take(ctx->client_mutex);

    // Check if we already have an active client
    if (ctx->client.connected) {
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &addr->sin_addr, ip_str, INET_ADDRSTRLEN);
        pr_info("[NET] Connection rejected - robot already controlled by %s:%d (new client: %s:%d)",
                ctx->client.peer_ip, ctx->client.peer_port, ip_str, platform_ntohs(addr->sin_port));
        lock_give(ctx->client_mutex);
        return -1; // Reject connection
    }
    // Accept the new client
    ctx->client.socket = socket;
    ctx->client.addr = *addr;
    ctx->client.last_command = time(NULL);
    ctx->client.connected = true;
    ctx->client.peer_port = platform_ntohs(addr->sin_port);
    ctx->client.registered_udp_port = 0; // No UDP port registered initially
    ctx->client.udp_port_registered = false;
    inet_ntop(AF_INET, &addr->sin_addr, ctx->client.peer_ip, INET_ADDRSTRLEN);
    pr_info("[NET] Client connected: %s:%d (exclusive control granted)", ctx->client.peer_ip, ctx->client.peer_port);

    // Callback
    if (ctx->config.callbacks.on_connection) {
        ctx->config.callbacks.on_connection(ctx->client.peer_ip, ctx->client.peer_port, ctx->config.user_data);
    }
    lock_give(ctx->client_mutex);
    return 0; // Success
}

static void remove_client(robot_server_ctx_t *ctx) {
    lock_take(ctx->client_mutex);
    if (ctx->client.connected) {
        pr_info("[NET] Client disconnected: %s:%d (exclusive control released)", ctx->client.peer_ip,
                ctx->client.peer_port);

        // Callback
        if (ctx->config.callbacks.on_disconnection) {
            ctx->config.callbacks.on_disconnection(ctx->client.peer_ip, ctx->client.peer_port, ctx->config.user_data);
        }
        platform_close(ctx->client.socket);
        ctx->client.connected = false;
        ctx->client.udp_port_registered = false;
        memset(&ctx->client, 0, sizeof(client_info_t));
    }
    lock_give(ctx->client_mutex);
}

static int read_header(int socket, protocol_header_t *header) {
    uint8_t needle[] = {0x52, 0x41, 0x49, 0x56};
    uint32_t offset = 0;
    int32_t to_read = sizeof(protocol_header_t);
    while (1) {
        int bytes_read = platform_recv(socket, (char *) (header + offset), to_read, MSG_DONTWAIT);
        if (bytes_read <= 0) {
            int err = errno;
            if (err == EWOULDBLOCK || err == EAGAIN) {
                return 1;
            }
            return -1;
        }
        if (bytes_read != to_read)
            return bytes_read;
        if (header->magic_number == PROTOCOL_MAGIC_NUMBER) {
            return sizeof(protocol_header_t);
        } else {
            void *begin = memchr((void *) header, needle[0], sizeof(protocol_header_t));
            if (begin != NULL) {
                memmove((void *) header, begin, sizeof(protocol_header_t) - (begin - (void *) header));
                offset = sizeof(protocol_header_t) - (begin - (void *) header);
                to_read = (begin - (void *) header);
            } else {
                offset = 0;
                to_read = sizeof(protocol_header_t);
            }
        }
    }
}

static void *tcp_thread_func(void *arg) {
    robot_server_ctx_t *ctx = (robot_server_ctx_t *) arg;

    struct sockaddr_in client_addr;
    SOCK_LEN_TYPE client_len = sizeof(client_addr);
    while (ctx->running) {
        fd_set read_fds;

        FD_ZERO(&read_fds);
        FD_SET(ctx->tcp_socket, &read_fds);
        int nfd = ctx->tcp_socket;
        if (ctx->client.connected) {
            FD_SET(ctx->client.socket, &read_fds);
            if (ctx->client.socket > nfd)
                nfd = ctx->client.socket;
        }
        struct timeval timeout = {1, 0}; // 1 second timeout
        int activity = platform_select(nfd + 1, &read_fds, NULL, NULL, &timeout);
        if (activity < 0) {
            switch (errno) {
            case EINTR:
                continue;
            case EBADF:
                if (ctx->client.connected) {
                    remove_client(ctx);
                }
                continue;
            default:
                pr_error("[TCP] Server error %d - quitting", errno);
            }
            break;
        }
        if (activity == 0) {
            continue; // Timeout, check running flag
        }
        // Check for new connections
        if (FD_ISSET(ctx->tcp_socket, &read_fds)) {
            int client_socket = platform_accept(ctx->tcp_socket, (struct sockaddr *) &client_addr, &client_len);
            if (client_socket >= 0) {
                if (add_client(ctx, client_socket, &client_addr) == 0) {
                } else {
                    // Client rejected - close socket immediately
                    platform_close(client_socket);
                }
            }
        }
        // Check existing client connection
        if (ctx->client.connected && FD_ISSET(ctx->client.socket, &read_fds)) {
            protocol_header_t header;
            int bytes_read = read_header(ctx->client.socket, &header);
            if (bytes_read <= 0) {
                remove_client(ctx);
                continue;
            }
            if (bytes_read == sizeof(header)) {
                void *payload = NULL;
                if (header.payload_length <= MAX_PAYLOAD_SIZE) {
                    payload = platform_malloc(header.payload_length);
                    bzero(payload, header.payload_length);
                    if (payload) {
                        uint32_t payload_received_bytes = 0;
                        while (payload_received_bytes != header.payload_length) {
                            int ret_recv = platform_recv(ctx->client.socket, (char *) payload + payload_received_bytes,
                                                         header.payload_length - payload_received_bytes, MSG_WAITALL);
                            if (ret_recv <= 0) {
                                pr_error("Failed to read entire payload have %d expected %d errno %d",
                                         payload_received_bytes, header.payload_length, errno);
                            }
                            payload_received_bytes += ret_recv;
                        }

                    } else {
                        send_command_response_context(ctx->client.socket, MSG_ERR_RESPONSE_INT(-1));
                        continue;
                    }
                } else {
                    pr_error("message too large expected less than %d got %d", MAX_PAYLOAD_SIZE, header.payload_length);
                    send_command_response_context(ctx->client.socket, MSG_ERR_RESPONSE_INT(-1));
                    continue;
                }
                // Update activity and handle command
                ctx->client.last_command = time(NULL);
                // Special handling for MSG_REGISTER_UDP_PORT (handled directly in protocol layer)
                if (header.message_type == MSG_REGISTER_UDP_PORT) {
                    if (header.payload_length >= sizeof(udp_port_registration_payload_t) && payload != NULL) {
                        udp_port_registration_payload_t *port_payload = (udp_port_registration_payload_t *) payload;

                        // Check protocol version if client sent v2 payload (3+ bytes)
                        if (header.payload_length >= sizeof(udp_port_registration_v2_payload_t)) {
                            udp_port_registration_v2_payload_t *v2 = (udp_port_registration_v2_payload_t *) payload;
                            if (v2->protocol_version != PROTOCOL_VERSION) {
                                pr_error("[NET] Client %s:%d protocol version mismatch: client=%u server=%u",
                                         ctx->client.peer_ip, ctx->client.peer_port, v2->protocol_version,
                                         PROTOCOL_VERSION);
                                error_payload_t err;
                                memset(&err, 0, sizeof(err));
                                err.error_code = -1;
                                snprintf(err.message, sizeof(err.message),
                                         "protocol version mismatch: client=%u server=%u", v2->protocol_version,
                                         PROTOCOL_VERSION);
                                send_command_response_context(ctx->client.socket, MSG_ERR_RESPONSE(err));
                                platform_free(payload);
                                payload = NULL;
                                remove_client(ctx);
                                continue;
                            }
                        } else {
                            pr_info("[NET] WARNING: Client %s:%d registered UDP port without protocol version. "
                                    "Update client to include protocol version in registration.",
                                    ctx->client.peer_ip, ctx->client.peer_port);
                        }

                        if (port_payload->udp_port > 0) {
                            // Register the UDP port
                            lock_take(ctx->client_mutex);
                            ctx->client.registered_udp_port = port_payload->udp_port;
                            ctx->client.udp_port_registered = true;
                            lock_give(ctx->client_mutex);
                            pr_info("[NET] Client %s:%d registered UDP port %u for status messages",
                                    ctx->client.peer_ip, ctx->client.peer_port, port_payload->udp_port);
                            send_command_response_context(ctx->client.socket, MSG_OK_RESPONSE());
                        } else {
                            pr_info("[NET] Client %s:%d sent invalid UDP port %u (must be > 0)", ctx->client.peer_ip,
                                    ctx->client.peer_port, port_payload->udp_port);
                            send_command_response_context(ctx->client.socket, MSG_ERR_RESPONSE_INT(-1));
                        }
                    } else {
                        pr_info("[NET] Client %s:%d sent invalid UDP port registration payload (expected >= %zu "
                                "bytes, got %u)",
                                ctx->client.peer_ip, ctx->client.peer_port, sizeof(udp_port_registration_payload_t),
                                header.payload_length);
                        send_command_response_context(ctx->client.socket, MSG_ERR_RESPONSE_INT(-1));
                    }
                }
                // Special handling for MSG_GET_ERROR_INFO
                else if (header.message_type == MSG_GET_ERROR_INFO) {
                    if (ctx->config.callbacks.get_error_info) {
                        int32_t error_code = 0;
                        command_response_context_t *rsp =
                            ctx->config.callbacks.get_error_info(&error_code, ctx->config.user_data);
                        if (rsp) {
                            // Send MSG_ERROR_INFO response with error details
                            send_command_response_context(ctx->client.socket, rsp);
                        } else {
                            // No error information available - send OK
                            send_command_response_context(ctx->client.socket, MSG_ERR_RESPONSE_INT(-1));
                        }
                    } else {
                        // No callback provided - send OK (no error info available)
                        send_command_response_context(ctx->client.socket, MSG_ERR_RESPONSE_INT(-1));
                    }
                } else {
                    // Handle regular commands
                    if (ctx->config.callbacks.handle_command) {
                        command_response_context_t *rsp =
                            ctx->config.callbacks.handle_command(&header, payload, ctx->config.user_data);
                        if (rsp)
                            send_command_response_context(ctx->client.socket, rsp);
                        else
                            send_command_response_context(ctx->client.socket, MSG_ERR_RESPONSE_INT(-1));
                    } else {
                        send_command_response_context(ctx->client.socket, MSG_ERR_RESPONSE_INT(-1));
                    }
                }
                if (payload) {
                    platform_free(payload);
                    payload = NULL;
                }
            }
        }
    }
    return NULL;
}

static void *timeout_thread_func(void *arg) {
    robot_server_ctx_t *ctx = (robot_server_ctx_t *) arg;
    while (ctx->running) {
        time_t now = time(NULL);
        double timeout_seconds = ctx->config.connection_timeout_ms / 1000.0;
        lock_take(ctx->client_mutex);
        if (ctx->client.connected) {
            double idle_time = difftime(now, ctx->client.last_command);
            if (idle_time > timeout_seconds) {
                pr_info("[NET] Client %s:%d timed out after %.1fs", ctx->client.peer_ip, ctx->client.peer_port,
                        idle_time);
                lock_give(ctx->client_mutex);
                remove_client(ctx);
            } else {
                lock_give(ctx->client_mutex);
            }
        } else {
            lock_give(ctx->client_mutex);
        }
        platform_usleep(100000); // Check every 100ms
    }
    return NULL;
}

robot_server_ctx_t *robot_protocol_create(const robot_server_config_t *config) {
    robot_server_ctx_t *ctx = (robot_server_ctx_t *) platform_malloc(sizeof(robot_server_ctx_t));
    bzero(ctx, sizeof(robot_server_ctx_t));
    if (!ctx) {
        return NULL;
    }
    ctx->config = *config;
    ctx->running = 0;
    ctx->tcp_socket = -1;
    ctx->udp_socket = -1;
    memset(&ctx->client, 0, sizeof(client_info_t));
    ctx->client_mutex = lock_create();
    if (ctx->client_mutex == NULL) {
        return NULL;
    }
    return ctx;
}

int robot_protocol_start(robot_server_ctx_t *ctx) {
    if (!ctx) {
        return -1;
    }
    struct sockaddr_in tcp_addr, udp_addr;
    int reuse = 1;

    // Create TCP socket
    ctx->tcp_socket = platform_socket(AF_INET, SOCK_STREAM, 0);
    if (ctx->tcp_socket < 0) {
        pr_error("[NET] Failed to create TCP socket");
        return -1;
    }
    platform_setsockopt(ctx->tcp_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    memset(&tcp_addr, 0, sizeof(tcp_addr));
    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_addr.s_addr = INADDR_ANY;
    tcp_addr.sin_port = platform_htons(ctx->config.tcp_port);
    if (platform_bind(ctx->tcp_socket, (struct sockaddr *) &tcp_addr, sizeof(tcp_addr)) < 0) {
        pr_error("[NET] TCP bind failed");
        platform_close(ctx->tcp_socket);
        return -1;
    }
    if (platform_listen(ctx->tcp_socket, 5) < 0) {
        pr_error("[NET] TCP listen failed");
        platform_close(ctx->tcp_socket);
        return -1;
    }
    // Create UDP socket
    ctx->udp_socket = platform_socket(AF_INET, SOCK_DGRAM, 0);
    if (ctx->udp_socket < 0) {
        pr_error("[NET] Failed to create UDP socket");
        platform_close(ctx->tcp_socket);
        return -1;
    }
    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_addr.s_addr = INADDR_ANY;
    udp_addr.sin_port = platform_htons(ctx->config.udp_port);
    if (platform_bind(ctx->udp_socket, (struct sockaddr *) &udp_addr, sizeof(udp_addr)) < 0) {
        pr_error("[NET] UDP bind failed");
        platform_close(ctx->tcp_socket);
        platform_close(ctx->udp_socket);
        return -1;
    }
    ctx->running = 1;
    pr_info("[NET] Server started on TCP:%d UDP:%d (timeout: %dms)", ctx->config.tcp_port, ctx->config.udp_port,
            ctx->config.connection_timeout_ms);

    // Start threads
    platform_pthread_create(&ctx->tcp_thread, NULL, tcp_thread_func, ctx);
    platform_pthread_create(&ctx->timeout_thread, NULL, timeout_thread_func, ctx);
    return 0;
}

void robot_protocol_stop(robot_server_ctx_t *ctx) {
    if (!ctx) {
        return;
    }
    ctx->running = 0;

    // Close client connection if active
    lock_take(ctx->client_mutex);
    if (ctx->client.connected) {
        platform_close(ctx->client.socket);
        ctx->client.connected = false;
        memset(&ctx->client, 0, sizeof(client_info_t));
    }
    lock_give(ctx->client_mutex);
    if (ctx->tcp_socket >= 0) {
        platform_shutdown(ctx->tcp_socket, SHUT_RDWR);
        platform_close(ctx->tcp_socket);
        ctx->tcp_socket = -1;
    }
    if (ctx->udp_socket >= 0) {
        platform_shutdown(ctx->udp_socket, SHUT_RDWR);
        platform_close(ctx->udp_socket);
        ctx->udp_socket = -1;
    }
    // Wait for threads
    if (ctx->tcp_thread != 0) {
        platform_pthread_join(ctx->tcp_thread, NULL);
    }
    if (ctx->timeout_thread != 0) {
        platform_pthread_join(ctx->timeout_thread, NULL);
    }
    pr_info("[NET] Network server stopped");
}

int robot_protocol_send_position_velocity_torque(robot_server_ctx_t *ctx, const status_payload_t *status) {
    if (!ctx || !status || !ctx->client.connected) {
        return -1;
    }
    if (!ctx->client.udp_port_registered) {
        // UDP port not registered - do not send status messages
        return 0; // Return 0 to indicate no messages sent (not an error, just no registered port)
    }

    protocol_header_t header;
    header.magic_number = PROTOCOL_MAGIC_NUMBER;
    header.version = PROTOCOL_VERSION;
    header.message_type = MSG_ROBOT_POSITION_VELOCITY_TORQUE;
    header.timestamp_ms = get_timestamp_ms();
    header.payload_length = sizeof(status_payload_t);
    char buffer[sizeof(header) + sizeof(status_payload_t)];
    memcpy(buffer, &header, sizeof(header));
    memcpy(buffer + sizeof(header), status, sizeof(status_payload_t));
    struct sockaddr_in udp_addr = ctx->client.addr;
    udp_addr.sin_port = platform_htons(ctx->client.registered_udp_port); // Use registered port instead of config port
    if (platform_sendto(ctx->udp_socket, buffer, sizeof(buffer), 0, (struct sockaddr *) &udp_addr, sizeof(udp_addr)) >
        0) {
        return 1; // Successfully sent to the single client
    }

    return 0; // Send failed
}

int robot_protocol_send_robot_status(robot_server_ctx_t *ctx, const robot_status_payload_t *status) {
    if (!ctx || !status || !ctx->client.connected) {
        return -1;
    }

    if (!ctx->client.udp_port_registered) {
        // UDP port not registered - do not send status messages
        return 0; // Return 0 to indicate no messages sent (not an error, just no registered port)
    }

    protocol_header_t header;
    header.magic_number = PROTOCOL_MAGIC_NUMBER;
    header.version = PROTOCOL_VERSION;
    header.message_type = MSG_ROBOT_STATUS;
    header.timestamp_ms = get_timestamp_ms();
    header.payload_length = sizeof(robot_status_payload_t);
    char buffer[sizeof(header) + sizeof(robot_status_payload_t)];
    memcpy(buffer, &header, sizeof(header));
    memcpy(buffer + sizeof(header), status, sizeof(robot_status_payload_t));
    struct sockaddr_in udp_addr = ctx->client.addr;
    udp_addr.sin_port = platform_htons(ctx->client.registered_udp_port); // Use registered port instead of config port
    if (platform_sendto(ctx->udp_socket, buffer, sizeof(buffer), 0, (struct sockaddr *) &udp_addr, sizeof(udp_addr)) >
        0) {
        return 1; // Successfully sent to the single client
    }

    return 0; // Send failed
}

int robot_protocol_is_running(robot_server_ctx_t *ctx) {
    return ctx ? ctx->running : 0;
}

void robot_protocol_destroy(robot_server_ctx_t *ctx) {
    if (!ctx) {
        return;
    }
    robot_protocol_stop(ctx);
    if (ctx->client_mutex) {
        lock_destroy(ctx->client_mutex);
    }
    platform_free(ctx);
}
