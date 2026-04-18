#define _GNU_SOURCE
#include "logging.h"
#include "mock_robot.h"
#include "platform_api.h"
#include "protocol.h"
#include "robot_protocol.h"
#include "unity.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
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
// Multi-group test state (used by dual-arm tests)
static robot_server_ctx_t *test_multi_network = NULL;
static mock_robot_t test_multi_robot;
// Helper function to create a test client socket
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
// Helper function to send a protocol message
static int send_protocol_message(int socket_fd, message_type_t msg_type) {
    protocol_header_t header;
    header.magic_number = PROTOCOL_MAGIC_NUMBER;
    header.version = PROTOCOL_VERSION;
    header.message_type = (uint8_t) msg_type;
    header.timestamp_ms = get_timestamp_ms();
    header.payload_length = 0;
    return send(socket_fd, &header, sizeof(header), 0);
}

// Helper function to send protocol message with payload
static int send_protocol_message_with_payload(int socket_fd, message_type_t msg_type, const void *payload,
                                              size_t payload_size) {
    protocol_header_t header;
    header.magic_number = PROTOCOL_MAGIC_NUMBER;
    header.version = PROTOCOL_VERSION;
    header.message_type = (uint8_t) msg_type;
    header.timestamp_ms = get_timestamp_ms();
    header.payload_length = payload_size;

    // Send header first
    if (send(socket_fd, &header, sizeof(header), 0) < 0) {
        return -1;
    }
    // Send payload if present
    if (payload_size > 0 && payload != NULL) {
        return send(socket_fd, payload, payload_size, 0);
    }

    return sizeof(header);
}

// Helper function to send garbage data
static int send_garbage_data(int socket_fd, const uint8_t *data, size_t length) {
    return send(socket_fd, data, length, 0);
}

// Helper function to receive response message
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
        // Use a stack buffer large enough for any response payload to avoid overflow.
        // The caller's error_payload (error_payload_t, 132 bytes) may be smaller than
        // the actual payload (e.g. goal_status_payload_t at 157 bytes).
        uint8_t recv_buf[1024];
        if (header.payload_length > sizeof(recv_buf)) {
            return -1;
        }
        bytes_read = recv(socket_fd, recv_buf, header.payload_length, 0);
        if (header.payload_length != bytes_read) {
            return -1;
        }
        if (error_payload) {
            size_t copy_len = header.payload_length < sizeof(*error_payload) ? header.payload_length : sizeof(*error_payload);
            memcpy(error_payload, recv_buf, copy_len);
        }
    }
    return 0;
}

// Test setup and teardown
void setUp(void) {
    // Reset counters
    signal(SIGPIPE, SIG_IGN);

    // Initialize mock robot
    mock_robot_init(&test_robot);

    // Configure network
    robot_server_config_t config = {.tcp_port = TEST_TCP_PORT,
                                    .udp_port = TEST_UDP_PORT,
                                    .connection_timeout_ms = TEST_TIMEOUT_MS,
                                    .callbacks = {.on_connection = mock_robot_on_connection,
                                                  .on_disconnection = mock_robot_on_disconnection,
                                                  .handle_command = mock_robot_handle_command,
                                                  .get_error_info = mock_robot_get_error_info_callback},
                                    .user_data = &test_robot};

    // Create network server
    test_network = robot_protocol_create(&config);
    TEST_ASSERT_NOT_NULL(test_network);

    // Start network server
    int result = robot_protocol_start(test_network);
    TEST_ASSERT_EQUAL(0, result);

    // Give server time to start
    platform_usleep(100000); // 100ms
}

void tearDown(void) {
    if (test_network) {
        robot_protocol_destroy(test_network); // calls robot_protocol_stop internally
        test_network = NULL;
    }
    // Also clean up multi-group server if a multi-group test failed mid-way
    if (test_multi_network) {
        robot_protocol_destroy(test_multi_network);
        test_multi_network = NULL;
    }
    // Give time for cleanup
    platform_usleep(100000); // 100ms
}

// Test 1: Single client connection restriction
void test_single_client_connection_restriction(void) {
    // Connect first client
    int client1 = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client1);
    platform_usleep(50000); // Wait for connection to be processed
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Try to connect second client - should fail
    int client2 = create_test_client();
    if (client2 >= 0) {
        // Connection succeeded at TCP level, but server should close it immediately
        // platform_usleep(50000); // Wait for server to process and close connection

        // Try to send data to verify connection is actually closed
        char test_byte = 0x42;
        ssize_t result = send(client2, &test_byte, 1, MSG_DONTWAIT);
        if (result > 0) {
            // If send succeeded, wait a bit more for server to close
            platform_usleep(100000);
            result = send(client2, &test_byte, 1, MSG_DONTWAIT);
        }
        close(client2);

        // The key test: connection count should still be 1 (second client rejected)
        TEST_ASSERT_EQUAL(1, test_robot.connection_count);
    } else {
        // Connection failed immediately (also acceptable)
        TEST_ASSERT_EQUAL(1, test_robot.connection_count);
    }

    // Close first client
    close(client1);
    platform_usleep(100000); // Wait for disconnection to be processed
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);
}

// Test 2: Client reconnection after disconnection
void test_client_reconnection_after_disconnection(void) {
    // Connect first client
    int client1 = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client1);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Disconnect first client
    close(client1);
    platform_usleep(100000); // Wait for disconnection
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);

    // Now second client should be able to connect
    int client2 = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client2);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(2, test_robot.connection_count); // Should increment
    close(client2);
    platform_usleep(50000);
}

// Test 3: Heartbeat timeout disconnection
void test_heartbeat_timeout_disconnection(void) {
    // Connect client
    int client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send initial heartbeat to establish connection
    int result = send_protocol_message(client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.command_count);
    // Sleep long enough to guarantee timeout fires. Since time() has 1-second
    // granularity, difftime returns whole seconds. We need difftime > timeout_seconds,
    // so we must sleep at least ceil(timeout_seconds) + 1 seconds.
    platform_usleep((TEST_TIMEOUT_MS + 2000) * 1000);

    // Should be disconnected due to timeout
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);

    // Try to send another message - should fail
    result = send_protocol_message(client, MSG_HEARTBEAT);

    // Note: This might succeed at socket level but connection is dead server-side
    close(client);
}

// Test 4: Garbage data handling and packet recovery
void test_garbage_data_and_packet_recovery(void) {
    // Connect client
    int client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send some garbage data
    uint8_t garbage[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFF, 0x00, 0x42, 0x13};
    int result = send_garbage_data(client, garbage, sizeof(garbage));
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(5000);

    // Command count should still be 0 (garbage ignored)
    TEST_ASSERT_EQUAL(0, test_robot.command_count);

    // Send a valid message after garbage
    result = send_protocol_message(client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(5000);

    // This should increment command count (valid packet processed)
    TEST_ASSERT_EQUAL(1, test_robot.command_count);

    // Send more garbage mixed with valid header fragments
    uint8_t mixed_data[] = {
        0xFF,
        0xFF, // Garbage
        0x52,
        0x41,
        0x49,
        0x56,                    // Valid magic number
        PROTOCOL_VERSION,        // Version
        MSG_TURN_SERVO_POWER_ON, // Message type
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x07,
        0xD0, // Timestamp (2000ms)
        0x00,
        0x00,
        0x00,
        0x00 // Payload length
    };
    result = send_garbage_data(client, mixed_data, sizeof(mixed_data));
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(5000);

    // Should still process the valid message
    TEST_ASSERT_EQUAL(2, test_robot.command_count);
    close(client);
    platform_usleep(5000);
}

// Test 5: Multiple commands from single client
void test_multiple_commands_single_client(void) {
    // Connect client
    int client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send multiple commands
    int result;
    result = send_protocol_message(client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(5000);
    TEST_ASSERT_EQUAL(1, test_robot.command_count);
    result = send_protocol_message(client, MSG_TEST_TRAJECTORY_COMMAND);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(5000);
    TEST_ASSERT_EQUAL(2, test_robot.command_count);
    result = send_protocol_message(client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(5000);
    TEST_ASSERT_EQUAL(3, test_robot.command_count);
    close(client);
    platform_usleep(5000);
}

// Test 6: UDP status message transmission
void test_udp_status_transmission(void) {
    // Connect TCP client to establish connection
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(5000); // Wait for connection
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    udp_port_registration_v2_payload_t port_payload;
    message_type_t response_type;
    port_payload.udp_port = 12345;
    port_payload.protocol_version = PROTOCOL_VERSION;
    int result =
        send_protocol_message_with_payload(tcp_client, MSG_REGISTER_UDP_PORT, &port_payload, sizeof(port_payload));
    TEST_ASSERT_EQUAL(sizeof(udp_port_registration_v2_payload_t), result);

    // Receive response - should be OK since it's handled in protocol layer
    result = receive_response(tcp_client, &response_type, NULL);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);
    // Send a heartbeat to establish activity
    result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    platform_usleep(5000);
    TEST_ASSERT_EQUAL(1, test_robot.command_count);

    // Create a test status payload
    status_payload_t test_status;
    mock_robot_get_status(&test_robot, &test_status, time(NULL) * 1000, 0);

    // Test 1: Send status via the network library - should succeed with connected
    // client
    result = robot_protocol_send_position_velocity_torque(test_network, &test_status);
    TEST_ASSERT_EQUAL(1, result); // Should return 1 for successful send to connected client

    // Test 2: Send multiple status messages
    for (int i = 0; i < 3; i++) {
        test_status.timestamp = (time(NULL) + i) * 1000;
        result = robot_protocol_send_position_velocity_torque(test_network, &test_status);
        TEST_ASSERT_EQUAL(1, result); // Each send should succeed
    }
    // Test 3: Verify status structure content is reasonable
    TEST_ASSERT_GREATER_THAN(0, test_status.timestamp);
    TEST_ASSERT_GREATER_OR_EQUAL(0, test_status.num_axes);
    TEST_ASSERT_LESS_OR_EQUAL(MAX_AXES, test_status.num_axes);

    // Close TCP client
    close(tcp_client);
    platform_usleep(100000); // Wait for disconnection
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);

    // Test 4: Send status with no connected client - should fail
    result = robot_protocol_send_position_velocity_torque(test_network, &test_status);
    TEST_ASSERT_EQUAL(-1, result); // Should return 0 for no client connected

    // Test 5: Send with NULL parameters - should fail
    result = robot_protocol_send_position_velocity_torque(NULL, &test_status);
    TEST_ASSERT_EQUAL(-1, result); // Should return -1 for invalid network
    result = robot_protocol_send_position_velocity_torque(test_network, NULL);
    TEST_ASSERT_EQUAL(-1, result); // Should return -1 for NULL status
}

// Test 7: Response message handling (OK and ERROR)
void test_response_message_handling(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000); // Wait for connection
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Test 1: Send successful command and expect OK response
    int result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    message_type_t response_type;
    error_payload_t error_payload;
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 2: Send error command and expect ERROR response
    result = send_protocol_message(tcp_client, MSG_TEST_ERROR_COMMAND);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR, response_type);
    TEST_ASSERT_EQUAL(-42, error_payload.error_code); // Mock returns -42

    // Test 3: Send trajectory command when power is off (should succeed now since power was turned on)
    result = send_protocol_message(tcp_client, MSG_TEST_TRAJECTORY_COMMAND);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 4: Send heartbeat and expect OK response
    result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);
    close(tcp_client);
    platform_usleep(600000);
}
void test_get_error_info_command() {
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);

    // Test 1: Get error info when no error is available (should return empty OK)
    int result = send_protocol_message(tcp_client, MSG_GET_ERROR_INFO);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    message_type_t response_type;
    error_payload_t error_payload;
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 2: Trigger an error, then get error info
    result = send_protocol_message(tcp_client, MSG_TEST_ERROR_COMMAND);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR, response_type);
    TEST_ASSERT_EQUAL(-42, error_payload.error_code);

    // Now get error info - should return OK with no payload in simplified test
    result = send_protocol_message(tcp_client, MSG_GET_ERROR_INFO);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR_INFO, response_type);

    // Test 3: Cause trajectory error and check error info
    result = send_protocol_message(tcp_client, MSG_TEST_TRAJECTORY_COMMAND);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR, response_type); // Should fail because servo power is off

    // Get error info after trajectory error
    result = send_protocol_message(tcp_client, MSG_GET_ERROR_INFO);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR_INFO, response_type);
    close(tcp_client);
    platform_usleep(100000);
}

// Test 10: UDP port registration is mandatory for status messages
void test_udp_port_registration_mandatory(void) {
    message_type_t response_type;
    error_payload_t error_payload;
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000); // Wait for connection
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send a heartbeat to establish connection
    int result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);
    TEST_ASSERT_EQUAL(1, test_robot.command_count);

    // Test 1: Try to send status without port registration - should fail (return 0 = no clients sent to)
    status_payload_t test_status;
    mock_robot_get_status(&test_robot, &test_status, time(NULL) * 1000, 0);
    result = robot_protocol_send_position_velocity_torque(test_network, &test_status);
    TEST_ASSERT_EQUAL(0, result); // Should return 0 (no messages sent due to no registered port)

    // Test 2: Register UDP port (use v2 payload with protocol version for compatibility with all mismatch policies)
    udp_port_registration_v2_payload_t port_payload;
    port_payload.udp_port = 12345;
    port_payload.protocol_version = PROTOCOL_VERSION;
    result = send_protocol_message_with_payload(tcp_client, MSG_REGISTER_UDP_PORT, &port_payload, sizeof(port_payload));
    TEST_ASSERT_EQUAL(sizeof(udp_port_registration_v2_payload_t), result);

    // Receive response - should be OK since it's handled in protocol layer
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 3: Now send status after registration - should succeed
    result = robot_protocol_send_position_velocity_torque(test_network, &test_status);
    TEST_ASSERT_EQUAL(1, result); // Should return 1 (successfully sent to 1 client)

    // Test 4: Test robot status as well
    robot_status_payload_t robot_status;
    mock_robot_get_robot_status(&test_robot, &robot_status, time(NULL) * 1000, 0);
    result = robot_protocol_send_robot_status(test_network, &robot_status);
    TEST_ASSERT_EQUAL(1, result); // Should return 1 (successfully sent to 1 client)

    // Test 5: Change UDP port registration to a different port
    port_payload.udp_port = 54321;
    result = send_protocol_message_with_payload(tcp_client, MSG_REGISTER_UDP_PORT, &port_payload, sizeof(port_payload));
    TEST_ASSERT_EQUAL(sizeof(udp_port_registration_v2_payload_t), result);

    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 6: Status should still work with new port
    result = robot_protocol_send_position_velocity_torque(test_network, &test_status);
    TEST_ASSERT_EQUAL(1, result); // Should return 1 (successfully sent to 1 client with new port)

    // Test 7: Verify UDP data is actually received on configured socket
    // Create UDP socket to receive status messages on the registered port
    int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
    TEST_ASSERT_NOT_EQUAL(-1, udp_socket);

    struct sockaddr_in udp_addr;
    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_addr.s_addr = INADDR_ANY;
    udp_addr.sin_port = htons(54321); // Use the last registered port

    result = bind(udp_socket, (struct sockaddr *) &udp_addr, sizeof(udp_addr));
    TEST_ASSERT_EQUAL(0, result);

    // Set socket to non-blocking for testing
    int flags = fcntl(udp_socket, F_GETFL, 0);
    fcntl(udp_socket, F_SETFL, flags | O_NONBLOCK);

    // Send status message and verify it's received
    result = robot_protocol_send_position_velocity_torque(test_network, &test_status);
    TEST_ASSERT_EQUAL(1, result); // Should successfully send to 1 client

    // Give time for UDP message to be sent and received
    platform_usleep(10000);

    // Try to receive UDP message
    char buffer[1024];
    ssize_t bytes_received = recv(udp_socket, buffer, sizeof(buffer), 0);
    TEST_ASSERT_GREATER_THAN(0, bytes_received); // Should have received data

    // Verify the received message has correct header
    if (bytes_received >= (ssize_t) sizeof(protocol_header_t)) {
        protocol_header_t *header = (protocol_header_t *) buffer;
        TEST_ASSERT_EQUAL(PROTOCOL_MAGIC_NUMBER, header->magic_number);
        TEST_ASSERT_EQUAL(PROTOCOL_VERSION, header->version);
        TEST_ASSERT_EQUAL(MSG_ROBOT_POSITION_VELOCITY_TORQUE, header->message_type);
        TEST_ASSERT_EQUAL(sizeof(status_payload_t), header->payload_length);
    }

    close(udp_socket);
    close(tcp_client);
    platform_usleep(50000);
}

// Test 12: Reset errors command
void test_reset_errors_command(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000); // Wait for connection
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send a heartbeat to establish connection
    message_type_t response_type;
    error_payload_t error_payload;
    int result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);
    TEST_ASSERT_EQUAL(1, test_robot.command_count);

    // Step 1: Trigger an error with test error command
    result = send_protocol_message(tcp_client, MSG_TEST_ERROR_COMMAND);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR, response_type);
    TEST_ASSERT_EQUAL(-42, error_payload.error_code); // Mock returns -42

    // Step 2: Verify we can get error info (should have error info available)
    result = send_protocol_message(tcp_client, MSG_GET_ERROR_INFO);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR_INFO, response_type);

    // Step 3: Reset errors
    result = send_protocol_message(tcp_client, MSG_RESET_ERRORS);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type); // Should succeed

    // Step 5: Verify robot status shows no errors
    // Turn on servo power first so we can send trajectory command
    result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Send trajectory command - should work without errors now
    result = send_protocol_message(tcp_client, MSG_TEST_TRAJECTORY_COMMAND);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type); // Should succeed (no errors)

    close(tcp_client);
    platform_usleep(50000);
}

// Test 13: Move goal command with trajectory and tolerance
void test_move_goal_command(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000); // Wait for connection
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send a heartbeat to establish connection
    message_type_t response_type;
    error_payload_t error_payload;
    int result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);
    TEST_ASSERT_EQUAL(1, test_robot.command_count);

    // Turn on servo power first
    result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Create a simple trajectory with 90 points
    uint32_t traj_size = 90;
    uint32_t tol_size = 0;

    // Calculate required buffer size
    size_t payload_size = MOVE_GOAL_CALC_SIZE(traj_size, tol_size);

    uint8_t *payload_data = (uint8_t *) malloc(payload_size);
    TEST_ASSERT_NOT_NULL(payload_data);

    // Cast buffer to move_goal structure
    move_goal_payload_t *move_goal = (move_goal_payload_t *) payload_data;
    move_goal->number_of_axes_controlled = 6;
    move_goal->group_index = 0;
    move_goal->trajectory_size = traj_size;

    // Get trajectory data pointer
    trajectory_point_t *trajectory_data = MOVE_GOAL_GET_TRAJECTORY(move_goal);

    // Fill trajectory points
    for (uint32_t i = 0; i < traj_size; i++) {
        for (int j = 0; j < NUMBER_OF_DOF; j++) {
            trajectory_data[i].positions[j] = (double) (i + 1) * 10.0; // 10, 20, 30 degrees
            trajectory_data[i].velocities[j] = 5.0;                    // 5 deg/s
            trajectory_data[i].accelerations[j] = 2.0;                 // 2 deg/s²
            trajectory_data[i].torque[j] = 1.0;                        // 1 Nm
        }
        trajectory_data[i].time_from_start.sec = i + 1; // 1, 2, 3 seconds
        trajectory_data[i].time_from_start.nanos = 0;
    }

    // Set tolerance size
    uint32_t *tolerance_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(move_goal);
    *tolerance_size_ptr = tol_size;

    // Send move goal with payload
    result = send_protocol_message_with_payload(tcp_client, MSG_MOVE_GOAL, payload_data, payload_size);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    // Should get MSG_GOAL_ACCEPTED response with goal_id
    protocol_header_t goal_header;
    ssize_t bytes_read;
    bytes_read = recv(tcp_client, &goal_header, sizeof(goal_header), 0);
    TEST_ASSERT_EQUAL(sizeof(goal_header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_GOAL_ACCEPTED, goal_header.message_type);
    TEST_ASSERT_EQUAL(sizeof(goal_accepted_payload_t), goal_header.payload_length);

    goal_accepted_payload_t goal_accepted;
    bytes_read = recv(tcp_client, &goal_accepted, sizeof(goal_accepted), 0);
    TEST_ASSERT_EQUAL(sizeof(goal_accepted), bytes_read);
    TEST_ASSERT_NOT_EQUAL(0, goal_accepted.goal_id); // Goal ID should be non-zero

    // Test echo trajectory command
    result = send_protocol_message(tcp_client, MSG_ECHO_TRAJECTORY);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    // Should receive the echoed trajectory
    protocol_header_t echo_header;
    bytes_read = recv(tcp_client, &echo_header, sizeof(echo_header), 0);
    TEST_ASSERT_EQUAL(sizeof(echo_header), bytes_read);
    TEST_ASSERT_EQUAL(PROTOCOL_MAGIC_NUMBER, echo_header.magic_number);
    TEST_ASSERT_EQUAL(PROTOCOL_VERSION, echo_header.version);
    TEST_ASSERT_EQUAL(MSG_ECHO_TRAJECTORY, echo_header.message_type);
    TEST_ASSERT_GREATER_THAN(0, echo_header.payload_length);

    // Read the payload
    uint8_t *echo_payload = (uint8_t *) malloc(echo_header.payload_length);
    TEST_ASSERT_NOT_NULL(echo_payload);
    bytes_read = recv(tcp_client, echo_payload, echo_header.payload_length, 0);
    TEST_ASSERT_EQUAL(echo_header.payload_length, bytes_read);

    move_goal_payload_t *echoed = move_goal_from_payload(echo_payload, echo_header.payload_length);
    TEST_ASSERT_NOT_NULL(echoed);

    TEST_ASSERT_EQUAL(traj_size, echoed->trajectory_size);

    // Get tolerance size from echoed
    uint32_t *echoed_tol_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(echoed);
    uint32_t echoed_tol_size = *echoed_tol_size_ptr;
    TEST_ASSERT_EQUAL(tol_size, echoed_tol_size);

    // Get trajectory arrays
    trajectory_point_t *echoed_trajectory = MOVE_GOAL_GET_TRAJECTORY(echoed);

    // Validate trajectory points match
    for (uint32_t i = 0; i < traj_size; i++) {
        for (int j = 0; j < NUMBER_OF_DOF; j++) {
            TEST_ASSERT_EQUAL_DOUBLE(trajectory_data[i].positions[j], echoed_trajectory[i].positions[j]);
            TEST_ASSERT_EQUAL_DOUBLE(trajectory_data[i].velocities[j], echoed_trajectory[i].velocities[j]);
            TEST_ASSERT_EQUAL_DOUBLE(trajectory_data[i].accelerations[j], echoed_trajectory[i].accelerations[j]);
            TEST_ASSERT_EQUAL_DOUBLE(trajectory_data[i].torque[j], echoed_trajectory[i].torque[j]);
        }
        TEST_ASSERT_EQUAL(trajectory_data[i].time_from_start.sec, echoed_trajectory[i].time_from_start.sec);
        TEST_ASSERT_EQUAL(trajectory_data[i].time_from_start.nanos, echoed_trajectory[i].time_from_start.nanos);
    }

    // No tolerance data to check since tol_size == 0

    // Clean up
    free(payload_data);
    free(echo_payload);
    close(tcp_client);
    platform_usleep(50000);
}

// Test 14: Set motion mode command
void test_set_motion_mode_command(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000); // Wait for connection
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send a heartbeat to establish connection
    message_type_t response_type;
    error_payload_t error_payload;
    int result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);
    TEST_ASSERT_EQUAL(1, test_robot.command_count);

    // Test 1: Set motion mode to 0
    motion_mode_payload_t mode_payload;
    mode_payload.motion_mode = 0;
    result = send_protocol_message_with_payload(tcp_client, MSG_SET_MOTION_MODE, &mode_payload, sizeof(mode_payload));
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 2: Set motion mode to 1
    mode_payload.motion_mode = 1;
    result = send_protocol_message_with_payload(tcp_client, MSG_SET_MOTION_MODE, &mode_payload, sizeof(mode_payload));
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 3: Set motion mode to 5
    mode_payload.motion_mode = 5;
    result = send_protocol_message_with_payload(tcp_client, MSG_SET_MOTION_MODE, &mode_payload, sizeof(mode_payload));
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 4: Set motion mode to maximum value (255)
    mode_payload.motion_mode = 255;
    result = send_protocol_message_with_payload(tcp_client, MSG_SET_MOTION_MODE, &mode_payload, sizeof(mode_payload));
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    close(tcp_client);
    platform_usleep(50000);
}

// Test 15: IS_IN_MOTION command
void test_is_in_motion(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000); // Wait for connection
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send a heartbeat to establish connection
    message_type_t response_type;
    error_payload_t error_payload;
    int result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 1: Check is_in_motion when robot is initially stopped
    result = send_protocol_message(tcp_client, MSG_IS_IN_MOTION);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    // Receive response with boolean payload
    protocol_header_t header;
    ssize_t bytes_read = recv(tcp_client, &header, sizeof(header), 0);
    TEST_ASSERT_EQUAL(sizeof(header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_IS_IN_MOTION, header.message_type);
    TEST_ASSERT_EQUAL(sizeof(boolean_payload_t), header.payload_length);

    boolean_payload_t is_motion_payload;
    bytes_read = recv(tcp_client, &is_motion_payload, sizeof(is_motion_payload), 0);
    TEST_ASSERT_EQUAL(sizeof(is_motion_payload), bytes_read);
    TEST_ASSERT_EQUAL(0, is_motion_payload.value); // Should be false (not in motion)

    // Test 2: Turn on servo power and start trajectory
    result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    result = send_protocol_message(tcp_client, MSG_TEST_TRAJECTORY_COMMAND);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 3: Check is_in_motion when robot is in motion
    result = send_protocol_message(tcp_client, MSG_IS_IN_MOTION);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    bytes_read = recv(tcp_client, &header, sizeof(header), 0);
    TEST_ASSERT_EQUAL(sizeof(header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_IS_IN_MOTION, header.message_type);
    TEST_ASSERT_EQUAL(sizeof(boolean_payload_t), header.payload_length);

    bytes_read = recv(tcp_client, &is_motion_payload, sizeof(is_motion_payload), 0);
    TEST_ASSERT_EQUAL(sizeof(is_motion_payload), bytes_read);
    TEST_ASSERT_EQUAL(1, is_motion_payload.value); // Should be true (in motion)

    close(tcp_client);
    platform_usleep(50000);
}

// Test 17: Goal tracking and cancellation
void test_goal_tracking_and_cancellation(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000); // Wait for connection
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send heartbeat to establish connection
    message_type_t response_type;
    error_payload_t error_payload;
    int result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Turn on servo power
    result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Create a simple trajectory with 3 points and 3 tolerance points
    uint32_t traj_size = 3;
    uint32_t tol_size = 3;

    // Calculate required buffer size
    size_t payload_size = MOVE_GOAL_CALC_SIZE(traj_size, tol_size);
    uint8_t *payload_data = (uint8_t *) malloc(payload_size);
    TEST_ASSERT_NOT_NULL(payload_data);

    // Cast buffer to move_goal structure
    move_goal_payload_t *move_goal = (move_goal_payload_t *) payload_data;
    move_goal->number_of_axes_controlled = 6;
    move_goal->group_index = 0;
    move_goal->trajectory_size = traj_size;

    // Get trajectory data pointer
    trajectory_point_t *trajectory_data = MOVE_GOAL_GET_TRAJECTORY(move_goal);

    // Fill trajectory points
    for (uint32_t i = 0; i < traj_size; i++) {
        for (int j = 0; j < NUMBER_OF_DOF; j++) {
            trajectory_data[i].positions[j] = (double) (i + 1) * 10.0;
            trajectory_data[i].velocities[j] = 5.0;
            trajectory_data[i].accelerations[j] = 2.0;
            trajectory_data[i].torque[j] = 1.0;
        }
        trajectory_data[i].time_from_start.sec = i + 1;
        trajectory_data[i].time_from_start.nanos = 0;
    }

    // Set tolerance size
    uint32_t *tolerance_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(move_goal);
    *tolerance_size_ptr = tol_size;

    // Get tolerance data pointer
    tolerance_t *tolerance_data = MOVE_GOAL_GET_TOLERANCE(move_goal);

    // Fill tolerance
    for (uint32_t i = 0; i < tol_size; i++) {
        for (int j = 0; j < NUMBER_OF_DOF; j++) {
            tolerance_data[i].positions[j] = 0.1;
            tolerance_data[i].velocities[j] = 0.5;
            tolerance_data[i].accelerations[j] = 0.2;
            tolerance_data[i].torque[j] = 0.05;
        }
    }

    // Test 1: Send MSG_MOVE_GOAL and receive MSG_GOAL_ACCEPTED
    result = send_protocol_message_with_payload(tcp_client, MSG_MOVE_GOAL, payload_data, payload_size);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    protocol_header_t header;
    ssize_t bytes_read = recv(tcp_client, &header, sizeof(header), 0);
    TEST_ASSERT_EQUAL(sizeof(header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_GOAL_ACCEPTED, header.message_type);
    TEST_ASSERT_EQUAL(sizeof(goal_accepted_payload_t), header.payload_length);

    goal_accepted_payload_t goal_accepted;
    bytes_read = recv(tcp_client, &goal_accepted, sizeof(goal_accepted), 0);
    TEST_ASSERT_EQUAL(sizeof(goal_accepted), bytes_read);
    TEST_ASSERT_NOT_EQUAL(0, goal_accepted.goal_id); // Goal ID should be non-zero
    int32_t goal_id = goal_accepted.goal_id;
    pr_info("[TEST] Goal accepted with ID: %d", goal_id);

    // Test 2: Query goal status using MSG_GET_GOAL_STATUS
    cancel_goal_payload_t status_query;
    status_query.goal_id = goal_id;
    result = send_protocol_message_with_payload(tcp_client, MSG_GET_GOAL_STATUS, &status_query, sizeof(status_query));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    bytes_read = recv(tcp_client, &header, sizeof(header), 0);
    TEST_ASSERT_EQUAL(sizeof(header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_GOAL_STATUS, header.message_type);
    TEST_ASSERT_EQUAL(sizeof(goal_status_payload_t), header.payload_length);

    goal_status_payload_t goal_status;
    bytes_read = recv(tcp_client, &goal_status, sizeof(goal_status), 0);
    TEST_ASSERT_EQUAL(sizeof(goal_status), bytes_read);
    TEST_ASSERT_EQUAL(goal_id, goal_status.goal_id);
    TEST_ASSERT_EQUAL(GOAL_STATE_ACTIVE, goal_status.state);
    TEST_ASSERT_GREATER_OR_EQUAL(0.0, goal_status.progress);
    pr_info("[TEST] Goal status: state=%d, progress=%.2f", goal_status.state, goal_status.progress);

    // Test 3: Cancel the goal using MSG_CANCEL_GOAL
    cancel_goal_payload_t cancel_request;
    cancel_request.goal_id = goal_id;
    result = send_protocol_message_with_payload(tcp_client, MSG_CANCEL_GOAL, &cancel_request, sizeof(cancel_request));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);
    pr_info("[TEST] Goal cancelled successfully");

    // Send heartbeat to keep connection alive
    result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);

    // Test 4: Verify goal status is now CANCELLED
    result = send_protocol_message_with_payload(tcp_client, MSG_GET_GOAL_STATUS, &status_query, sizeof(status_query));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    bytes_read = recv(tcp_client, &header, sizeof(header), 0);
    TEST_ASSERT_EQUAL(sizeof(header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_GOAL_STATUS, header.message_type);

    bytes_read = recv(tcp_client, &goal_status, sizeof(goal_status), 0);
    TEST_ASSERT_EQUAL(sizeof(goal_status), bytes_read);
    TEST_ASSERT_EQUAL(GOAL_STATE_CANCELLED, goal_status.state);
    pr_info("[TEST] Goal status after cancellation: state=%d", goal_status.state);

    // Send heartbeat to keep connection alive
    result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);

    // Test 5: Verify robot is no longer in motion after cancellation
    result = send_protocol_message(tcp_client, MSG_IS_IN_MOTION);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    bytes_read = recv(tcp_client, &header, sizeof(header), 0);
    TEST_ASSERT_EQUAL(sizeof(header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_IS_IN_MOTION, header.message_type);

    boolean_payload_t is_motion_payload;
    bytes_read = recv(tcp_client, &is_motion_payload, sizeof(is_motion_payload), 0);
    TEST_ASSERT_EQUAL(sizeof(is_motion_payload), bytes_read);
    TEST_ASSERT_EQUAL(0, is_motion_payload.value); // Should be false (not in motion)

    // Send heartbeat to keep connection alive
    result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);

    // Test 6: Try to cancel already cancelled goal (should fail)
    result = send_protocol_message_with_payload(tcp_client, MSG_CANCEL_GOAL, &cancel_request, sizeof(cancel_request));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR, response_type); // Should return error
    pr_info("[TEST] Cannot cancel already cancelled goal (expected behavior)");

    // Test 7: Try to query non-existent goal ID (should fail)
    cancel_goal_payload_t invalid_query;
    invalid_query.goal_id = 999999; // Non-existent goal
    result = send_protocol_message_with_payload(tcp_client, MSG_GET_GOAL_STATUS, &invalid_query, sizeof(invalid_query));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR, response_type); // Should return error
    pr_info("[TEST] Cannot query non-existent goal (expected behavior)");

    // Clean up
    free(payload_data); // This frees move_goal too since it points into payload_data
    close(tcp_client);
    platform_usleep(50000);
}

// Test 16: STOP_MOTION command
void test_stop_motion(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000); // Wait for connection
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send a heartbeat to establish connection
    message_type_t response_type;
    error_payload_t error_payload;
    int result = send_protocol_message(tcp_client, MSG_HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 1: Turn on servo power and start trajectory
    result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    result = send_protocol_message(tcp_client, MSG_TEST_TRAJECTORY_COMMAND);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    // Test 2: Stop motion while in motion (group_id=-1 means stop all)
    group_id_t stop_all = {.group_id = GROUP_ID_ALL};
    result = send_protocol_message_with_payload(tcp_client, MSG_STOP_MOTION, &stop_all, sizeof(stop_all));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    protocol_header_t header;
    ssize_t bytes_read = recv(tcp_client, &header, sizeof(header), 0);
    TEST_ASSERT_EQUAL(sizeof(header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_STOP_MOTION, header.message_type);
    TEST_ASSERT_EQUAL(sizeof(boolean_payload_t), header.payload_length);

    boolean_payload_t stop_payload;
    bytes_read = recv(tcp_client, &stop_payload, sizeof(stop_payload), 0);
    TEST_ASSERT_EQUAL(sizeof(stop_payload), bytes_read);
    TEST_ASSERT_EQUAL(1, stop_payload.value); // Should be true (motion was stopped)

    // Test 3: Verify is_in_motion returns false after stopping
    result = send_protocol_message(tcp_client, MSG_IS_IN_MOTION);
    TEST_ASSERT_NOT_EQUAL(-1, result);

    bytes_read = recv(tcp_client, &header, sizeof(header), 0);
    TEST_ASSERT_EQUAL(sizeof(header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_IS_IN_MOTION, header.message_type);

    boolean_payload_t is_motion_payload;
    bytes_read = recv(tcp_client, &is_motion_payload, sizeof(is_motion_payload), 0);
    TEST_ASSERT_EQUAL(sizeof(is_motion_payload), bytes_read);
    TEST_ASSERT_EQUAL(0, is_motion_payload.value); // Should be false (not in motion)

    // Test 4: Try to stop motion when already stopped
    result = send_protocol_message_with_payload(tcp_client, MSG_STOP_MOTION, &stop_all, sizeof(stop_all));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    bytes_read = recv(tcp_client, &header, sizeof(header), 0);
    TEST_ASSERT_EQUAL(sizeof(header), bytes_read);
    TEST_ASSERT_EQUAL(MSG_STOP_MOTION, header.message_type);

    bytes_read = recv(tcp_client, &stop_payload, sizeof(stop_payload), 0);
    TEST_ASSERT_EQUAL(sizeof(stop_payload), bytes_read);
    TEST_ASSERT_EQUAL(0, stop_payload.value); // Should be false (no motion was active)

    close(tcp_client);
    platform_usleep(50000);
}

// Test: Version info request
void test_udp_registration_with_matching_version(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send v2 registration with correct protocol version
    udp_port_registration_v2_payload_t v2_payload;
    v2_payload.udp_port = 12345;
    v2_payload.protocol_version = PROTOCOL_VERSION;

    int result = send_protocol_message_with_payload(tcp_client, MSG_REGISTER_UDP_PORT, &v2_payload, sizeof(v2_payload));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    message_type_t response_type;
    result = receive_response(tcp_client, &response_type, NULL);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);

    close(tcp_client);
    platform_usleep(50000);
}

void test_udp_registration_version_mismatch(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send v2 registration with wrong protocol version
    udp_port_registration_v2_payload_t v2_payload;
    v2_payload.udp_port = 12345;
    v2_payload.protocol_version = PROTOCOL_VERSION + 1; // Wrong version

    int result = send_protocol_message_with_payload(tcp_client, MSG_REGISTER_UDP_PORT, &v2_payload, sizeof(v2_payload));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    message_type_t response_type;
#if COMMS_VERSION_MISMATCH >= 2
    // REJECT mode: should receive error response and be disconnected
    error_payload_t error_payload;
    result = receive_response(tcp_client, &response_type, &error_payload);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR, response_type);

    // Client should be disconnected
    platform_usleep(100000);
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);
#else
    // WARN or IGNORE mode: should receive OK response (connection allowed)
    result = receive_response(tcp_client, &response_type, NULL);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);
#endif

    close(tcp_client);
    platform_usleep(50000);
}

void test_udp_registration_v1_payload(void) {
    // Connect TCP client
    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000);
    TEST_ASSERT_EQUAL(1, test_robot.connection_count);

    // Send v1 registration (no protocol version field)
    udp_port_registration_payload_t v1_payload;
    v1_payload.udp_port = 12345;

    int result = send_protocol_message_with_payload(tcp_client, MSG_REGISTER_UDP_PORT, &v1_payload, sizeof(v1_payload));
    TEST_ASSERT_NOT_EQUAL(-1, result);

    message_type_t response_type;
#if COMMS_VERSION_MISMATCH >= 2
    // REJECT mode: v1 payload (missing version) should be rejected
    result = receive_response(tcp_client, &response_type, NULL);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_ERROR, response_type);

    // Client should be disconnected
    platform_usleep(100000);
    TEST_ASSERT_EQUAL(1, test_robot.disconnection_count);
#else
    // WARN or IGNORE mode: v1 payload should be accepted
    result = receive_response(tcp_client, &response_type, NULL);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, response_type);
#endif

    close(tcp_client);
    platform_usleep(50000);
}

// ---- Multi-group helpers ----

// Build and send a move goal for the given group, return the accepted goal_id (or -1 on error)
static int32_t send_move_goal_and_get_id(int tcp_client, uint32_t group_index, uint32_t traj_size, uint8_t num_axes) {
    uint32_t tol_size = 0;
    size_t payload_size = MOVE_GOAL_CALC_SIZE(traj_size, tol_size);
    uint8_t *payload_data = (uint8_t *) malloc(payload_size);
    if (!payload_data)
        return -1;

    move_goal_payload_t *move_goal = (move_goal_payload_t *) payload_data;
    move_goal->number_of_axes_controlled = num_axes;
    move_goal->group_index = group_index;
    move_goal->trajectory_size = traj_size;

    trajectory_point_t *traj = MOVE_GOAL_GET_TRAJECTORY(move_goal);
    for (uint32_t i = 0; i < traj_size; i++) {
        for (int j = 0; j < (int) num_axes; j++) {
            traj[i].positions[j] = (double) (i + 1) * 10.0;
            traj[i].velocities[j] = 5.0;
            traj[i].accelerations[j] = 2.0;
            traj[i].torque[j] = 1.0;
        }
        traj[i].time_from_start.sec = i + 1;
        traj[i].time_from_start.nanos = 0;
    }
    uint32_t *tol_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(move_goal);
    *tol_ptr = tol_size;

    int result = send_protocol_message_with_payload(tcp_client, MSG_MOVE_GOAL, payload_data, payload_size);
    free(payload_data);
    if (result < 0)
        return -1;

    protocol_header_t hdr;
    ssize_t bytes = recv(tcp_client, &hdr, sizeof(hdr), 0);
    if (bytes != sizeof(hdr))
        return -1;
    if (hdr.message_type != MSG_GOAL_ACCEPTED)
        return -1;

    goal_accepted_payload_t accepted;
    bytes = recv(tcp_client, &accepted, sizeof(accepted), 0);
    if (bytes != sizeof(accepted))
        return -1;
    return accepted.goal_id;
}

// Query goal status, returns goal_state_t or -1 on error
static int query_goal_state(int tcp_client, int32_t goal_id) {
    cancel_goal_payload_t query;
    query.goal_id = goal_id;
    int result = send_protocol_message_with_payload(tcp_client, MSG_GET_GOAL_STATUS, &query, sizeof(query));
    if (result < 0)
        return -1;

    protocol_header_t hdr;
    ssize_t bytes = recv(tcp_client, &hdr, sizeof(hdr), 0);
    if (bytes != sizeof(hdr))
        return -1;
    if (hdr.message_type != MSG_GOAL_STATUS)
        return -1;

    goal_status_payload_t status;
    bytes = recv(tcp_client, &status, sizeof(status), 0);
    if (bytes != sizeof(status))
        return -1;
    return (int) status.state;
}

// Poll until goal reaches target state or timeout (returns final state)
// robot parameter: if non-NULL, calls mock_robot_update_goal_progress each poll
static int poll_goal_until_ex(int tcp_client, int32_t goal_id, goal_state_t target, int max_polls,
                              mock_robot_t *robot) {
    int state = -1;
    for (int i = 0; i < max_polls; i++) {
        if (robot)
            mock_robot_update_goal_progress(robot);
        state = query_goal_state(tcp_client, goal_id);
        if (state == (int) target || state < 0)
            return state;
        platform_usleep(100000); // 100ms between polls
    }
    return state;
}

static int poll_goal_until(int tcp_client, int32_t goal_id, goal_state_t target, int max_polls) {
    return poll_goal_until_ex(tcp_client, goal_id, target, max_polls, NULL);
}

// ---- Dual-arm (multi-group) test scenarios ----

// Tear down the single-group server from global setUp and start a 2-group server instead
static void multi_setUp(void) {
    // Destroy the single-group server that the global setUp created
    if (test_network) {
        robot_protocol_destroy(test_network);
        test_network = NULL;
    }
    platform_usleep(100000);

    uint8_t group_types[] = {GROUP_TYPE_ROBOT, GROUP_TYPE_ROBOT};
    uint8_t group_axes[] = {6, 6};
    mock_robot_init_multi(&test_multi_robot, 2, group_types, group_axes);

    robot_server_config_t config = {.tcp_port = TEST_TCP_PORT,
                                    .udp_port = TEST_UDP_PORT,
                                    .connection_timeout_ms = TEST_TIMEOUT_MS,
                                    .callbacks = {.on_connection = mock_robot_on_connection,
                                                  .on_disconnection = mock_robot_on_disconnection,
                                                  .handle_command = mock_robot_handle_command,
                                                  .get_error_info = mock_robot_get_error_info_callback},
                                    .user_data = &test_multi_robot};

    test_multi_network = robot_protocol_create(&config);
    TEST_ASSERT_NOT_NULL(test_multi_network);
    int result = robot_protocol_start(test_multi_network);
    TEST_ASSERT_EQUAL(0, result);
    platform_usleep(100000);
}

static void multi_tearDown(void) {
    if (test_multi_network) {
        robot_protocol_destroy(test_multi_network);
        test_multi_network = NULL;
    }
    platform_usleep(100000);
}

// Test: Sequential goals on two groups
void test_multi_group_sequential_goals(void) {
    multi_setUp();

    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000);

    // Power on
    message_type_t rtype;
    error_payload_t ep;
    int result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &rtype, &ep);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, rtype);

    // Send goal to group 0
    int32_t goal0 = send_move_goal_and_get_id(tcp_client, 0, 5, 6);
    TEST_ASSERT_NOT_EQUAL(-1, goal0);
    pr_info("[TEST] Group 0 goal accepted: %d", goal0);

    // Poll until group 0 succeeds
    int state0 = poll_goal_until_ex(tcp_client, goal0, GOAL_STATE_SUCCEEDED, 20, &test_multi_robot);
    TEST_ASSERT_EQUAL(GOAL_STATE_SUCCEEDED, state0);

    // Send goal to group 1
    int32_t goal1 = send_move_goal_and_get_id(tcp_client, 1, 5, 6);
    TEST_ASSERT_NOT_EQUAL(-1, goal1);
    pr_info("[TEST] Group 1 goal accepted: %d", goal1);

    // Poll until group 1 succeeds
    int state1 = poll_goal_until_ex(tcp_client, goal1, GOAL_STATE_SUCCEEDED, 20, &test_multi_robot);
    TEST_ASSERT_EQUAL(GOAL_STATE_SUCCEEDED, state1);

    close(tcp_client);
    platform_usleep(50000);
    multi_tearDown();
}

// Test: Concurrent goals on two groups
void test_multi_group_concurrent_goals(void) {
    multi_setUp();

    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000);

    // Power on
    message_type_t rtype;
    error_payload_t ep;
    int result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &rtype, &ep);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, rtype);

    // Send goal to group 0
    int32_t goal0 = send_move_goal_and_get_id(tcp_client, 0, 5, 6);
    TEST_ASSERT_NOT_EQUAL(-1, goal0);
    pr_info("[TEST] Group 0 goal accepted: %d", goal0);

    // Immediately send goal to group 1 (before group 0 finishes)
    int32_t goal1 = send_move_goal_and_get_id(tcp_client, 1, 5, 6);
    TEST_ASSERT_NOT_EQUAL(-1, goal1);
    pr_info("[TEST] Group 1 goal accepted: %d (concurrent with group 0)", goal1);

    // Poll both — both should reach SUCCEEDED
    int state0 = poll_goal_until_ex(tcp_client, goal0, GOAL_STATE_SUCCEEDED, 20, &test_multi_robot);
    TEST_ASSERT_EQUAL(GOAL_STATE_SUCCEEDED, state0);

    int state1 = poll_goal_until_ex(tcp_client, goal1, GOAL_STATE_SUCCEEDED, 20, &test_multi_robot);
    TEST_ASSERT_EQUAL(GOAL_STATE_SUCCEEDED, state1);

    close(tcp_client);
    platform_usleep(50000);
    multi_tearDown();
}

// Test: Partial cancel — cancel group 0, group 1 still succeeds
void test_multi_group_partial_cancel(void) {
    multi_setUp();

    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000);

    // Power on
    message_type_t rtype;
    error_payload_t ep;
    int result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &rtype, &ep);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, rtype);

    // Send goals to both groups
    int32_t goal0 = send_move_goal_and_get_id(tcp_client, 0, 5, 6);
    TEST_ASSERT_NOT_EQUAL(-1, goal0);

    int32_t goal1 = send_move_goal_and_get_id(tcp_client, 1, 5, 6);
    TEST_ASSERT_NOT_EQUAL(-1, goal1);

    // Cancel group 0's goal
    cancel_goal_payload_t cancel;
    cancel.goal_id = goal0;
    result = send_protocol_message_with_payload(tcp_client, MSG_CANCEL_GOAL, &cancel, sizeof(cancel));
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &rtype, &ep);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, rtype);

    // Verify group 0 is CANCELLED
    int state0 = query_goal_state(tcp_client, goal0);
    TEST_ASSERT_EQUAL(GOAL_STATE_CANCELLED, state0);

    // Verify group 1 still reaches SUCCEEDED (not affected by group 0's cancel)
    int state1 = poll_goal_until_ex(tcp_client, goal1, GOAL_STATE_SUCCEEDED, 20, &test_multi_robot);
    TEST_ASSERT_EQUAL(GOAL_STATE_SUCCEEDED, state1);

    close(tcp_client);
    platform_usleep(50000);
    multi_tearDown();
}

// Test: Disconnect aborts all active goals
void test_multi_group_disconnect_aborts_all(void) {
    multi_setUp();

    int tcp_client = create_test_client();
    TEST_ASSERT_NOT_EQUAL(-1, tcp_client);
    platform_usleep(50000);

    // Power on
    message_type_t rtype;
    error_payload_t ep;
    int result = send_protocol_message(tcp_client, MSG_TURN_SERVO_POWER_ON);
    TEST_ASSERT_NOT_EQUAL(-1, result);
    result = receive_response(tcp_client, &rtype, &ep);
    TEST_ASSERT_EQUAL(0, result);
    TEST_ASSERT_EQUAL(MSG_OK, rtype);

    // Send goals to both groups
    int32_t goal0 = send_move_goal_and_get_id(tcp_client, 0, 5, 6);
    TEST_ASSERT_NOT_EQUAL(-1, goal0);

    int32_t goal1 = send_move_goal_and_get_id(tcp_client, 1, 5, 6);
    TEST_ASSERT_NOT_EQUAL(-1, goal1);

    // Close TCP connection (simulates disconnect)
    close(tcp_client);
    platform_usleep(200000); // Wait for server to detect disconnect and call on_disconnection

    // Verify both goals are ABORTED
    // (Check directly via mock robot state since we can't query via TCP after disconnect)
    TEST_ASSERT_EQUAL(GOAL_STATE_ABORTED, test_multi_robot.groups[0].current_goal_state);
    TEST_ASSERT_EQUAL(GOAL_STATE_ABORTED, test_multi_robot.groups[1].current_goal_state);
    TEST_ASSERT_EQUAL(0, test_multi_robot.groups[0].in_motion);
    TEST_ASSERT_EQUAL(0, test_multi_robot.groups[1].in_motion);

    // Verify disconnection was registered
    TEST_ASSERT_EQUAL(1, test_multi_robot.disconnection_count);

    multi_tearDown();
}

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

// Main test runner
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
    RUN_TEST(test_single_client_connection_restriction);
    RUN_TEST(test_client_reconnection_after_disconnection);
    RUN_TEST(test_heartbeat_timeout_disconnection);
    RUN_TEST(test_garbage_data_and_packet_recovery);
    RUN_TEST(test_multiple_commands_single_client);
    RUN_TEST(test_udp_status_transmission);
    RUN_TEST(test_response_message_handling);
    RUN_TEST(test_get_error_info_command);
    RUN_TEST(test_udp_port_registration_mandatory);
    RUN_TEST(test_reset_errors_command);
    RUN_TEST(test_move_goal_command);
    RUN_TEST(test_set_motion_mode_command);
    RUN_TEST(test_is_in_motion);
    RUN_TEST(test_stop_motion);
    RUN_TEST(test_goal_tracking_and_cancellation);
    RUN_TEST(test_udp_registration_with_matching_version);
    RUN_TEST(test_udp_registration_version_mismatch);
    RUN_TEST(test_udp_registration_v1_payload);
    // Multi-group dual-arm tests (use their own setUp/tearDown)
    RUN_TEST(test_multi_group_sequential_goals);
    RUN_TEST(test_multi_group_concurrent_goals);
    RUN_TEST(test_multi_group_partial_cancel);
    RUN_TEST(test_multi_group_disconnect_aborts_all);
    pthread_cancel(print_log_thread);
    logging_shutdown();
    return UNITY_END();
}
