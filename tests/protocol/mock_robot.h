#ifndef MOCK_ROBOT_H
#define MOCK_ROBOT_H

#include "protocol.h"
#include <stddef.h>
#include <stdint.h>
typedef struct {
    int servo_power_on;
    int trajectory_active;
    int64_t last_heartbeat;
    uint32_t sequence_counter;
    uint32_t connection_count;
    uint32_t disconnection_count;
    uint32_t command_count;

    // Simple zero positions for testing
    double positions[MAX_AXES];
    double velocities[MAX_AXES];
    double torques[MAX_AXES];
    double corrected_positions[MAX_AXES];

    // Error information storage
    int has_error_info;
    char last_error_message[256];

    // Robot status data
    int mode;
    uint8_t motion_mode; // Current motion mode setting
    _Bool e_stopped;
    _Bool drives_powered;
    _Bool motion_possible;
    _Bool in_motion;
    _Bool in_error;
    int error_codes[MAX_ALARM_COUNT + 1];
    int error_count;

    // Trajectory storage for echo functionality
    // Dynamically allocated buffer containing complete move_goal payload
    void *stored_trajectory_buffer;
    uint32_t stored_trajectory_buffer_size;

    // Goal tracking
    int32_t current_goal_id;         // Current goal ID (0 = no active goal)
    goal_state_t current_goal_state; // Current goal state
    double current_goal_progress;    // Current goal progress (0.0 to 1.0)
    int64_t goal_start_time;         // When goal execution started
} mock_robot_t;

// Initialize mock robot
void mock_robot_init(mock_robot_t *robot);

// Handle commands (return 0 for success, negative for error)
int mock_robot_power_on(mock_robot_t *robot);
int mock_robot_start_trajectory(mock_robot_t *robot);
int mock_robot_heartbeat(mock_robot_t *robot, int64_t timestamp);
int mock_robot_test_error_command(mock_robot_t *robot);                                     // Always returns error
int mock_robot_get_error_info(mock_robot_t *robot, char *error_buffer, size_t buffer_size); // Get last error info
int mock_robot_reset_errors(mock_robot_t *robot); // Reset/clear all error conditions
int mock_robot_move_goal(mock_robot_t *robot,
                         const move_goal_payload_t *trajectory);          // Execute move goal and store trajectory
move_goal_payload_t *mock_robot_echo_trajectory(mock_robot_t *robot);     // Echo back stored trajectory
int mock_robot_set_motion_mode(mock_robot_t *robot, uint8_t motion_mode); // Set motion mode
uint8_t mock_robot_is_in_motion(mock_robot_t *robot);                     // Check if robot is currently in motion
uint8_t mock_robot_stop_motion(mock_robot_t *robot);                      // Stop current motion

// Goal tracking functions
int32_t mock_robot_accept_goal(mock_robot_t *robot,
                               const move_goal_payload_t *trajectory); // Accept and start goal, returns goal_id
int mock_robot_cancel_goal(mock_robot_t *robot,
                           int32_t goal_id); // Cancel a goal (returns 0 on success, -1 if not found/not owner)
int mock_robot_get_goal_status(mock_robot_t *robot, int32_t goal_id, goal_status_payload_t *status); // Get goal status
void mock_robot_update_goal_progress(mock_robot_t *robot); // Update goal progress (call periodically)

// Get status for UDP transmission
void mock_robot_get_status(mock_robot_t *robot, status_payload_t *status, int64_t timestamp);

// Get robot status for UDP transmission
void mock_robot_get_robot_status(mock_robot_t *robot, robot_status_payload_t *status, int64_t timestamp);

// Update simulation (call periodically)
void mock_robot_update(mock_robot_t *robot, int64_t timestamp);
void mock_robot_on_connection(const char *client_ip, uint16_t client_port, void *user_data);
void mock_robot_on_disconnection(const char *client_ip, uint16_t client_port, void *user_data);
command_response_context_t *mock_robot_handle_command(protocol_header_t *header, void *payload, void *user_data);
command_response_context_t *mock_robot_get_error_info_callback(int32_t *error_code, void *user_data);
#endif // MOCK_ROBOT_H
