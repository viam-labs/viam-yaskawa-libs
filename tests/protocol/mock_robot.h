#ifndef MOCK_ROBOT_H
#define MOCK_ROBOT_H

#include "protocol.h"
#include <stddef.h>
#include <stdint.h>

#define MOCK_MAX_GROUPS MAX_GROUPS

// Per-group state
typedef struct {
    double positions[MAX_AXES];
    double velocities[MAX_AXES];
    double torques[MAX_AXES];
    double corrected_positions[MAX_AXES];

    int trajectory_active;
    _Bool in_motion;

    // Trajectory storage for echo functionality
    void *stored_trajectory_buffer;
    uint32_t stored_trajectory_buffer_size;

    // Goal tracking
    int32_t current_goal_id;
    goal_state_t current_goal_state;
    double current_goal_progress;
    int64_t goal_start_time;

    // Group metadata
    uint8_t group_type;              // group_type_t
    uint8_t num_axes;                // active axes in this group
    uint8_t axis_types[MAX_AXES];    // axis_type_protocol_t per axis
    int8_t base_axis_motion[MAX_AXES]; // for base groups
} mock_group_state_t;

typedef struct {
    // Per-group state
    uint8_t num_groups;
    mock_group_state_t groups[MOCK_MAX_GROUPS];

    // Global state
    int servo_power_on;
    int64_t last_heartbeat;
    uint32_t sequence_counter;
    uint32_t connection_count;
    uint32_t disconnection_count;
    uint32_t command_count;

    // Error information storage
    int has_error_info;
    char last_error_message[256];

    // Robot status data
    int mode;
    uint8_t motion_mode;
    _Bool e_stopped;
    _Bool drives_powered;
    _Bool motion_possible;
    _Bool in_error;
    int error_codes[MAX_ALARM_COUNT + 1];
    int error_count;
} mock_robot_t;

// Initialize mock robot (default: 1 robot group with 6 axes)
void mock_robot_init(mock_robot_t *robot);

// Initialize mock robot with configurable groups
void mock_robot_init_multi(mock_robot_t *robot, uint8_t num_groups, const uint8_t group_types[],
                           const uint8_t group_axes[]);

// Handle commands (return 0 for success, negative for error)
int mock_robot_power_on(mock_robot_t *robot);
int mock_robot_start_trajectory(mock_robot_t *robot);
int mock_robot_heartbeat(mock_robot_t *robot, int64_t timestamp);
int mock_robot_test_error_command(mock_robot_t *robot);
int mock_robot_get_error_info(mock_robot_t *robot, char *error_buffer, size_t buffer_size);
int mock_robot_reset_errors(mock_robot_t *robot);
int mock_robot_move_goal(mock_robot_t *robot, const move_goal_payload_t *trajectory);
move_goal_payload_t *mock_robot_echo_trajectory(mock_robot_t *robot, uint8_t group_index);
int mock_robot_set_motion_mode(mock_robot_t *robot, uint8_t motion_mode);
uint8_t mock_robot_is_in_motion(mock_robot_t *robot);
uint8_t mock_robot_stop_motion(mock_robot_t *robot);

// Goal tracking functions
int32_t mock_robot_accept_goal(mock_robot_t *robot, const move_goal_payload_t *trajectory);
int mock_robot_cancel_goal(mock_robot_t *robot, int32_t goal_id);
int mock_robot_get_goal_status(mock_robot_t *robot, int32_t goal_id, goal_status_payload_t *status);
void mock_robot_update_goal_progress(mock_robot_t *robot);

// Get status for UDP transmission (per-group)
void mock_robot_get_status(mock_robot_t *robot, status_payload_t *status, int64_t timestamp, uint8_t group_index);

// Get robot status for UDP transmission (per-group)
void mock_robot_get_robot_status(mock_robot_t *robot, robot_status_payload_t *status, int64_t timestamp,
                                 uint8_t group_index);

// Update simulation (call periodically)
void mock_robot_update(mock_robot_t *robot, int64_t timestamp);
void mock_robot_on_connection(const char *client_ip, uint16_t client_port, void *user_data);
void mock_robot_on_disconnection(const char *client_ip, uint16_t client_port, void *user_data);
command_response_context_t *mock_robot_handle_command(protocol_header_t *header, void *payload, void *user_data);
command_response_context_t *mock_robot_get_error_info_callback(int32_t *error_code, void *user_data);
#endif // MOCK_ROBOT_H
