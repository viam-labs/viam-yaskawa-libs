#define _GNU_SOURCE
#include "fault_inject.h"
#include "logging.h"
#include "mock_robot.h"
#include "platform_api.h"
#include "protocol.h"
#include "robot_protocol.h"
#include "unity.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <unity_config.h>

// Test configuration
#define TEST_TCP_PORT 27654
#define TEST_UDP_PORT 27655
#define TEST_TIMEOUT_MS 2000

// Global test state
static robot_server_ctx_t *test_network = NULL;
static mock_robot_t test_robot;
static fault_inject_ctx_t test_fault_ctx;

// Helper: create a TCP test client connected to the server
static int create_test_client(void) {
    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0)
        return -1;
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TEST_TCP_PORT);
    inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr);
    if (connect(socket_fd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        close(socket_fd);
        return -1;
    }
    return socket_fd;
}

extern int64_t get_timestamp_ms();

// Helper: send a protocol message with no payload
static int send_protocol_message(int socket_fd, message_type_t msg_type) {
    protocol_header_t header;
    header.magic_number = PROTOCOL_MAGIC_NUMBER;
    header.version = PROTOCOL_VERSION;
    header.message_type = (uint8_t) msg_type;
    header.timestamp_ms = get_timestamp_ms();
    header.payload_length = 0;
    return send(socket_fd, &header, sizeof(header), 0);
}

// Helper: send protocol message with payload
static int send_protocol_message_with_payload(int socket_fd, message_type_t msg_type, const void *payload,
                                              size_t payload_size) {
    protocol_header_t header;
    header.magic_number = PROTOCOL_MAGIC_NUMBER;
    header.version = PROTOCOL_VERSION;
    header.message_type = (uint8_t) msg_type;
    header.timestamp_ms = get_timestamp_ms();
    header.payload_length = payload_size;
    if (send(socket_fd, &header, sizeof(header), 0) < 0) {
        return -1;
    }
    if (payload_size > 0 && payload != NULL) {
        return send(socket_fd, payload, payload_size, 0);
    }
    return sizeof(header);
}

// Helper: receive a response header and optional payload
static int receive_response(int socket_fd, message_type_t *response_type, error_payload_t *error_payload) {
    protocol_header_t header;
    ssize_t bytes_read = recv(socket_fd, &header, sizeof(header), 0);
    if (bytes_read != sizeof(header)) {
        return -1;
    }
    if (header.magic_number != PROTOCOL_MAGIC_NUMBER) {
        return -1;
    }
    *response_type = (message_type_t) header.message_type;
    if (header.payload_length > 0) {
        if (error_payload) {
            bytes_read = recv(socket_fd, error_payload, header.payload_length, 0);
            if (header.payload_length != (uint32_t) bytes_read) {
                return -1;
            }
        } else {
            // Drain payload bytes we don't need
            char drain[256];
            uint32_t remaining = header.payload_length;
            while (remaining > 0) {
                uint32_t chunk = remaining < sizeof(drain) ? remaining : sizeof(drain);
                ssize_t n = recv(socket_fd, drain, chunk, 0);
                if (n <= 0)
                    break;
                remaining -= (uint32_t) n;
            }
        }
    }
    return 0;
}

// Helper: receive response with poll timeout (returns -1 on timeout)
static int receive_response_with_timeout(int socket_fd, message_type_t *response_type, error_payload_t *error_payload,
                                         int timeout_sec) {
    struct pollfd pfd = {.fd = socket_fd, .events = POLLIN, .revents = 0};
    int activity = poll(&pfd, 1, timeout_sec * 1000);
    if (activity <= 0) {
        return -1; // Timeout or error
    }
    return receive_response(socket_fd, response_type, error_payload);
}

// Fault-injecting command handler wrapper.
// Checks fault rules before delegating to mock_robot_handle_command.
static command_response_context_t *fault_injecting_handle_command(protocol_header_t *header, void *payload,
                                                                  void *user_data) {
    fault_type_t fault = fault_inject_check(&test_fault_ctx, header);
    if (fault != FAULT_NONE) {
        fault_inject_execute(&test_fault_ctx, fault);
        if (fault == FAULT_UNRESPONSIVE) {
            // After the sleep, return a valid OK so the server doesn't
            // allocate a wasted error response on a possibly-dead connection.
            return MSG_OK_RESPONSE();
        }
        // For DISCONNECT/CLOSE, the connection is already dead.
        return NULL;
    }
    return mock_robot_handle_command(header, payload, user_data);
}

// --- setUp / tearDown ---

void setUp(void) {
    signal(SIGPIPE, SIG_IGN);
    mock_robot_init(&test_robot);
    fault_inject_init(&test_fault_ctx, NULL); // server pointer set after create

    robot_server_config_t config = {.tcp_port = TEST_TCP_PORT,
                                    .udp_port = TEST_UDP_PORT,
                                    .connection_timeout_ms = TEST_TIMEOUT_MS,
                                    .callbacks = {.on_connection = mock_robot_on_connection,
                                                  .on_disconnection = mock_robot_on_disconnection,
                                                  .handle_command = fault_injecting_handle_command,
                                                  .get_error_info = mock_robot_get_error_info_callback},
                                    .user_data = &test_robot};

    test_network = robot_protocol_create(&config);
    TEST_ASSERT_NOT_NULL(test_network);

    // Now that the server context exists, wire it into the fault context
    test_fault_ctx.server = test_network;

    int result = robot_protocol_start(test_network);
    TEST_ASSERT_EQUAL(0, result);
    platform_usleep(100000); // 100ms for server to start
}

void tearDown(void) {
    if (test_network) {
        robot_protocol_stop(test_network);
        robot_protocol_destroy(test_network);
        test_network = NULL;
    }
    platform_usleep(100000); // 100ms for cleanup
}

// ==========================================================================
// Test 1: FAULT_DISCONNECT after N heartbeats
// ==========================================================================
void test_fault_disconnect_after_n_messages(void) {
    // Configure: disconnect after 3 messages
    fault_inject_disconnect_after(&test_fault_ctx, 3);

    int client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send 2 heartbeats - should get OK responses
    message_type_t response_type;
    for (int i = 0; i < 2; i++) {
        int result = send_protocol_message(client, MSG_HEARTBEAT);
        TEST_ASSERT_NOT_EQUAL(-1, result);
        int rc = receive_response(client, &response_type, NULL);
        TEST_ASSERT_EQUAL(0, rc);
        TEST_ASSERT_EQUAL(MSG_OK, response_type);
    }

    // 3rd heartbeat triggers the fault
    int result = send_protocol_message(client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(200000); // Wait for disconnect to propagate

    // Connection should be dead - recv should return 0 or error
    char buf[1];
    ssize_t bytes = recv(client, buf, sizeof(buf), MSG_DONTWAIT);
    TEST_ASSERT_TRUE(bytes <= 0);

    close(client);
    platform_usleep(500000); // Wait for server to process disconnection
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);
}

// ==========================================================================
// Test 2: FAULT_UNRESPONSIVE on MSG_MOVE_GOAL
// ==========================================================================
void test_fault_server_unresponsive(void) {
    // Configure: become unresponsive when MSG_TURN_SERVO_POWER_ON is received
    // (using a simple command to avoid building move_goal payload)
    fault_inject_unresponsive_on(&test_fault_ctx, MSG_TURN_SERVO_POWER_ON);

    int client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send the command that triggers unresponsive behavior
    int result = send_protocol_message(client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    // Try to receive a response with 1 second timeout - should time out
    message_type_t response_type;
    int rc = receive_response_with_timeout(client, &response_type, NULL, 1);
    TEST_ASSERT_EQUAL(-1, rc); // Timeout: no response within 1 second

    close(client);
    platform_usleep(100000);
}

// ==========================================================================
// Test 3: Client disconnect during active goal (no fault injection)
// ==========================================================================
void test_fault_client_disconnect_during_goal(void) {
    int client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Power on
    message_type_t response_type;
    int result = send_protocol_message(client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(client, &response_type, NULL);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Build a simple move goal with 3 trajectory points
    uint32_t traj_size = 3;
    uint32_t tol_size = 0;
    size_t payload_size = MOVE_GOAL_CALC_SIZE(traj_size, tol_size);
    uint8_t *payload_data = (uint8_t *) malloc(payload_size);
    TEST_ASSERT_NOT_NULL(payload_data);
    memset(payload_data, 0, payload_size);

    move_goal_payload_t *move_goal = (move_goal_payload_t *) payload_data;
    move_goal->number_of_axes_controlled = 6;
    move_goal->group_index = 0;
    move_goal->trajectory_size = traj_size;

    trajectory_point_t *traj = MOVE_GOAL_GET_TRAJECTORY(move_goal);
    for (uint32_t i = 0; i < traj_size; i++) {
        for (int j = 0; j < NUMBER_OF_DOF; j++) {
            traj[i].positions[j] = (double) (i + 1) * 10.0;
            traj[i].velocities[j] = 5.0;
            traj[i].accelerations[j] = 2.0;
            traj[i].torque[j] = 1.0;
        }
        traj[i].time_from_start.sec = i + 1;
        traj[i].time_from_start.nanos = 0;
    }
    uint32_t *tolerance_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(move_goal);
    *tolerance_size_ptr = tol_size;

    // Send move goal
    result = send_protocol_message_with_payload(client, MSG_MOVE_GOAL, payload_data, payload_size);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    // Receive GOAL_ACCEPTED
    protocol_header_t goal_header;
    ssize_t bytes_read = recv(client, &goal_header, sizeof(goal_header), 0);
    TEST_ASSERT_EQUAL(sizeof(goal_header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_GOAL_ACCEPTED, goal_header.message_type);

    goal_accepted_payload_t goal_accepted;
    bytes_read = recv(client, &goal_accepted, sizeof(goal_accepted), 0);
    TEST_ASSERT_EQUAL(sizeof(goal_accepted), bytes_read);
    TEST_ASSERT_NOT_EQUAL(0, goal_accepted.goal_id);

    // Abruptly close the client socket (simulating client crash)
    close(client);
    free(payload_data);

    // Wait for server to detect disconnect via timeout
    platform_usleep((TEST_TIMEOUT_MS + 2000) * 1000);
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);

    // Reconnect a new client - should succeed
    int client2 = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client2);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(2, test_robot.connection_count);

    close(client2);
    platform_usleep(100000);
}

// ==========================================================================
// Test 4: FAULT_CLOSE_CONNECTION (server alarm / abrupt close)
// ==========================================================================
void test_fault_server_alarm_close(void) {
    // Configure: immediately close connection on first message (abrupt, no error payload)
    fault_inject_close_connection(&test_fault_ctx);

    int client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send any command to trigger the fault
    int result = send_protocol_message(client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(200000); // Wait for fault to execute

    // Connection should be abruptly closed — recv returns 0 (EOF) or error
    char buf[1];
    ssize_t bytes = recv(client, buf, sizeof(buf), MSG_DONTWAIT);
    TEST_ASSERT_TRUE(bytes <= 0);

    close(client);
    platform_usleep(500000); // Wait for server to process disconnection
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);
}

// ==========================================================================
// Test 5: Reconnect after one-shot fault clears
// ==========================================================================
void test_reconnect_after_fault(void) {
    // Configure: one-shot disconnect after 1 message
    fault_rule_t rule = {
        .fault = FAULT_DISCONNECT,
        .trigger = TRIGGER_AFTER_N_MESSAGES,
        .trigger_message_type = 0,
        .trigger_count = 1,
        .one_shot = true,
    };
    fault_inject_add_rule(&test_fault_ctx, &rule);

    // First connection: send 1 message, expect disconnect
    int client1 = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client1);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    int result = send_protocol_message(client1, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(200000);

    // Connection should be dead
    char buf[1];
    ssize_t bytes = recv(client1, buf, sizeof(buf), MSG_DONTWAIT);
    TEST_ASSERT_TRUE(bytes <= 0);
    close(client1);

    // Wait for server to fully clean up
    platform_usleep((TEST_TIMEOUT_MS + 2000) * 1000);
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);

    // Second connection: fault already fired (one-shot), should work normally
    int client2 = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client2);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(2, test_robot.connection_count);

    // Send heartbeat and expect normal OK response
    result = send_protocol_message(client2, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    message_type_t response_type;
    int rc = receive_response(client2, &response_type, NULL);
    TEST_ASSERT_EQUAL(0, rc);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    close(client2);
    platform_usleep(100000);
}

// ==========================================================================
// Logging setup and test runner
// ==========================================================================

#define BUFFER_SIZE 8192

static char log_buffer[BUFFER_SIZE];
static ring_buffer_t rb;

void *fn_print_log_thread(void *arg) {
    (void) (arg);
    while (1) {
        log_to_stdout();
        platform_usleep(100000);
    }
    return NULL;
}

int main(void) {
    if (ring_buffer_initialize(&rb, log_buffer, BUFFER_SIZE) != 0) {
        printf("Failed to initialize ring buffer\n");
        return 1;
    }
    if (logging_initialize(&rb) != 0) {
        printf("Failed to initialize logging\n");
        return 1;
    }
    logging_set_level(LOG_INFO);

    pthread_t print_log_thread;
    if (pthread_create(&print_log_thread, NULL, fn_print_log_thread, NULL) != 0) {
        printf("Failed to create log thread\n");
        return 1;
    }

    UNITY_BEGIN();
    RUN_TEST(test_fault_disconnect_after_n_messages);
    RUN_TEST(test_fault_server_unresponsive);
    RUN_TEST(test_fault_client_disconnect_during_goal);
    RUN_TEST(test_fault_server_alarm_close);
    RUN_TEST(test_reconnect_after_fault);
    pthread_cancel(print_log_thread);
    pthread_join(print_log_thread, NULL);
    logging_shutdown();
    return UNITY_END();
}
