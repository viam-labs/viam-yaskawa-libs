#include "mock_robot.h"
#include "logging.h"
#include "protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern int64_t get_timestamp_ms();

// Helper function to set error information
static void set_error_info(mock_robot_t *robot, const char *error_message) {
    robot->has_error_info = 1;
    strncpy(robot->last_error_message, error_message, sizeof(robot->last_error_message) - 1);
    robot->last_error_message[sizeof(robot->last_error_message) - 1] = '\0';
}

void mock_robot_init(mock_robot_t *robot) {
    memset(robot, 0, sizeof(mock_robot_t));
    robot->servo_power_on = 0;
    robot->trajectory_active = 0;
    robot->last_heartbeat = 0;
    robot->sequence_counter = 1;
    robot->has_error_info = 0;
    robot->disconnection_count = 0;
    robot->command_count = 0;
    robot->connection_count = 0;

    robot->stored_trajectory_buffer = NULL;
    robot->stored_trajectory_buffer_size = 0;
    robot->motion_mode = 0; // Default motion mode

    // Initialize all positions to zero for testing
    for (int i = 0; i < MAX_AXES; i++) {
        robot->positions[i] = 0.0;
        robot->velocities[i] = 0.0;
        robot->torques[i] = 0.0;
        robot->corrected_positions[i] = 0.0;
    }

    // Initialize robot status
    robot->mode = 1;            // Automatic mode
    robot->e_stopped = 0;       // Not e-stopped
    robot->drives_powered = 0;  // Drives not powered initially
    robot->motion_possible = 0; // Motion not possible initially
    robot->in_motion = 0;       // Not in motion
    robot->in_error = 0;        // No error
    robot->error_count = 0;     // No active errors
    memset(robot->error_codes, 0, sizeof(robot->error_codes));

    // Initialize goal tracking
    robot->current_goal_id = 0; // No active goal
    robot->current_goal_state = GOAL_STATE_PENDING;
    robot->current_goal_progress = 0.0;
    robot->goal_start_time = 0;
}

int mock_robot_power_on(mock_robot_t *robot) {
    robot->servo_power_on = 1;
    robot->drives_powered = 1;  // Drives are now powered
    robot->motion_possible = 1; // Motion is now possible

    // Set a reasonable initial heartbeat time to avoid immediate timeout
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    robot->last_heartbeat = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    pr_info("[MOCK] Servo power ON");
    return 0; // Success
}

int mock_robot_start_trajectory(mock_robot_t *robot) {
    if (robot->servo_power_on) {
        robot->trajectory_active = 1;
        robot->in_motion = 1; // Robot is now in motion

        // Set some simple test positions (small values for testing)
        for (int i = 0; i < MAX_AXES; i++) {
            robot->positions[i] = i * 1.0;                              // 0, 1, 2, 3, 4, 5, 6, 7
            robot->velocities[i] = i * 0.1;                             // 0, 0.1, 0.2, etc.
            robot->torques[i] = i * 0.01;                               // Small torque values
            robot->corrected_positions[i] = robot->positions[i] + 0.01; // Small correction
        }
        pr_info("[MOCK] Trajectory started");
        return 0; // Success
    } else {
        pr_info("[MOCK] Cannot start trajectory - servo power is OFF");
        set_error_info(robot, "Cannot start trajectory: servo power must be enabled first. Use TURN_SERVO_POWER_ON "
                              "command to enable servo power.");
        robot->in_error = 1;
        robot->error_codes[0] = 100; // Error code for servo power not enabled
        robot->error_count = 1;
        return -1; // Error - servo power not on
    }
}

int mock_robot_heartbeat(mock_robot_t *robot, int64_t timestamp) {
    robot->last_heartbeat = timestamp;
    return 0; // Success
}

int mock_robot_test_error_command(mock_robot_t *robot) {
    pr_info("[MOCK] Test error command - always returns error");
    set_error_info(robot, "Test error command executed. This command is designed to always fail for testing purposes. "
                          "Error details: simulated "
                          "fault condition in axis controller #3.");
    robot->in_error = 1;
    robot->error_codes[0] = 42; // Test error code
    robot->error_count = 1;
    return -42; // Always return error code -42
}

void mock_robot_get_status(mock_robot_t *robot, status_payload_t *status, int64_t timestamp) {
    // Check heartbeat timeout
    /*if (robot->servo_power_on && (timestamp - robot->last_heartbeat) > (HEARTBEAT_INTERVAL_MS * 3)) {
       robot->servo_power_on = 0;
       robot->trajectory_active = 0;
       } */
    status->timestamp = timestamp;
    status->num_axes = MAX_AXES;

    // Copy current robot state
    for (int i = 0; i < MAX_AXES; i++) {
        status->position[i] = robot->positions[i];
        status->velocity[i] = robot->velocities[i];
        status->torque[i] = robot->torques[i];
        status->position_corrected[i] = robot->corrected_positions[i];
    }
}

void mock_robot_update(mock_robot_t *robot, int64_t timestamp) {
    (void) timestamp; // Not used in simple version
    if (!robot->servo_power_on) {
        // When servo is off, gradually zero out velocities and torques
        for (int i = 0; i < MAX_AXES; i++) {
            robot->velocities[i] *= 0.95;
            robot->torques[i] *= 0.95;
        }
    }
    // Update goal progress if we have an active goal
    mock_robot_update_goal_progress(robot);
}

int mock_robot_get_error_info(mock_robot_t *robot, char *error_buffer, size_t buffer_size) {
    if (!robot->has_error_info) {
        // No error information available
        pr_info("[MOCK] No error information available");
        return 0; // Success - no error info (empty OK message should be sent)
    }
    // Copy error message to buffer
    strncpy(error_buffer, robot->last_error_message, buffer_size - 1);
    error_buffer[buffer_size - 1] = '\0';

    // Clear the error info after retrieving it
    robot->has_error_info = 0;
    memset(robot->last_error_message, 0, sizeof(robot->last_error_message));
    pr_info("[MOCK] Retrieved error info: %s", error_buffer);
    return 0; // Success
}

void mock_robot_get_robot_status(mock_robot_t *robot, robot_status_payload_t *status, int64_t timestamp) {
    // Check heartbeat timeout and update robot status accordingly
    /*if (robot->servo_power_on && (timestamp - robot->last_heartbeat) > (HEARTBEAT_INTERVAL_MS * 3)) {
       robot->servo_power_on = 0;
       robot->trajectory_active = 0;
       robot->drives_powered = 0;
       robot->motion_possible = 0;
       robot->in_motion = 0;
       } */
    // Fill robot status payload
    status->ts = timestamp;
    status->mode = robot->mode;
    status->e_stopped = robot->e_stopped;
    status->drives_powered = robot->drives_powered;
    status->motion_possible = robot->motion_possible;
    status->in_motion = robot->in_motion;
    status->in_error = robot->in_error;
    status->size = robot->error_count;

    // Copy error codes
    for (int i = 0; i < MAX_ALARM_COUNT + 1 && i < robot->error_count; i++) {
        status->error_codes[i] = robot->error_codes[i];
    }
    // Clear remaining error code slots
    for (int i = robot->error_count; i < MAX_ALARM_COUNT + 1; i++) {
        status->error_codes[i] = 0;
    }
}

int mock_robot_reset_errors(mock_robot_t *robot) {
    pr_info("[MOCK] Resetting all errors");

    // Clear error state
    robot->in_error = 0;
    robot->error_count = 0;

    // Clear error info
    robot->has_error_info = 0;
    memset(robot->last_error_message, 0, sizeof(robot->last_error_message));

    // Clear all error codes
    memset(robot->error_codes, 0, sizeof(robot->error_codes));

    return 0; // Success
}

int mock_robot_move_goal(mock_robot_t *robot, const move_goal_payload_t *trajectory) {
    // Get tolerance size to calculate total size needed
    const uint32_t *tol_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(trajectory);
    uint32_t tolerance_size = *tol_size_ptr;

    pr_info("[MOCK] Executing move goal with %u trajectory points and %u tolerance points", trajectory->trajectory_size,
            tolerance_size);

    // Calculate required buffer size
    uint32_t required_size = MOVE_GOAL_CALC_SIZE(trajectory->trajectory_size, tolerance_size);

    // Free previous trajectory buffer if it exists
    if (robot->stored_trajectory_buffer) {
        free(robot->stored_trajectory_buffer);
        robot->stored_trajectory_buffer = NULL;
        robot->stored_trajectory_buffer_size = 0;
    }

    // Allocate new buffer for deep copy
    robot->stored_trajectory_buffer = malloc(required_size);
    if (!robot->stored_trajectory_buffer) {
        pr_error("[MOCK] Failed to allocate %u bytes for trajectory storage", required_size);
        return -1;
    }
    robot->stored_trajectory_buffer_size = required_size;

    // Deep copy the entire move_goal payload
    if (move_goal_deep_copy(trajectory, robot->stored_trajectory_buffer, required_size) == NULL) {
        pr_error("[MOCK] Failed to deep copy trajectory");
        free(robot->stored_trajectory_buffer);
        robot->stored_trajectory_buffer = NULL;
        robot->stored_trajectory_buffer_size = 0;
        return -1;
    }

    // Simulate trajectory execution
    robot->trajectory_active = 1;
    robot->in_motion = 1;

    pr_info("[MOCK] Move goal stored (%u bytes) and trajectory started", required_size);
    return 0; // Success
}

move_goal_payload_t *mock_robot_echo_trajectory(mock_robot_t *robot) {
    if (!robot->stored_trajectory_buffer) {
        pr_info("[MOCK] No trajectory to echo - send a move goal first");
        return NULL;
    }

    move_goal_payload_t *stored_goal = (move_goal_payload_t *) robot->stored_trajectory_buffer;
    pr_info("[MOCK] Echoing trajectory with %u points", stored_goal->trajectory_size);

    // Return a pointer to the stored trajectory
    // Note: The caller should NOT free this pointer as it's owned by the robot
    return stored_goal;
}

int mock_robot_set_motion_mode(mock_robot_t *robot, uint8_t motion_mode) {
    pr_info("[MOCK] Setting motion mode to %u", motion_mode);
    robot->motion_mode = motion_mode;
    return 0; // Success
}

uint8_t mock_robot_is_in_motion(mock_robot_t *robot) {
    pr_info("[MOCK] Checking motion status: %s", robot->in_motion ? "in motion" : "stopped");
    return robot->in_motion ? 1 : 0;
}

uint8_t mock_robot_stop_motion(mock_robot_t *robot) {
    if (robot->in_motion) {
        pr_info("[MOCK] Stopping motion");
        robot->in_motion = 0;
        robot->trajectory_active = 0;
        return 1; // Motion was stopped
    } else {
        pr_info("[MOCK] No motion to stop");
        return 0; // No motion was active
    }
}

// Goal tracking implementation
int32_t mock_robot_accept_goal(mock_robot_t *robot, const move_goal_payload_t *trajectory) {
    // Check if robot is ready
    if (!robot->servo_power_on) {
        pr_info("[MOCK] Cannot accept goal - servo power is OFF");
        return -1;
    }
    // Check if another goal is active (PENDING or ACTIVE)
    // If the current goal is in a terminal state (SUCCEEDED, CANCELLED, ABORTED), we can accept a new goal
    if (robot->current_goal_id != 0 &&
        (robot->current_goal_state == GOAL_STATE_PENDING || robot->current_goal_state == GOAL_STATE_ACTIVE)) {
        pr_info("[MOCK] Cannot accept goal - another goal (ID: %d) is already active (state: %d)",
                robot->current_goal_id, robot->current_goal_state);
        return -1;
    }
    // If there's a previous goal in terminal state, we can accept a new goal
    if (robot->current_goal_id != 0) {
        pr_info("[MOCK] Previous goal %d in terminal state %d, accepting new goal", robot->current_goal_id,
                robot->current_goal_state);
    }

    // Get tolerance size to calculate total size needed
    const uint32_t *tol_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(trajectory);
    uint32_t tolerance_size = *tol_size_ptr;
    uint32_t required_size = MOVE_GOAL_CALC_SIZE(trajectory->trajectory_size, tolerance_size);

    // Free previous trajectory buffer if it exists
    if (robot->stored_trajectory_buffer) {
        free(robot->stored_trajectory_buffer);
        robot->stored_trajectory_buffer = NULL;
        robot->stored_trajectory_buffer_size = 0;
    }

    // Allocate new buffer for deep copy
    robot->stored_trajectory_buffer = malloc(required_size);
    if (!robot->stored_trajectory_buffer) {
        pr_error("[MOCK] Failed to allocate %u bytes for trajectory storage", required_size);
        return -1;
    }
    robot->stored_trajectory_buffer_size = required_size;

    // Deep copy the entire move_goal payload
    if (move_goal_deep_copy(trajectory, robot->stored_trajectory_buffer, required_size) == NULL) {
        pr_error("[MOCK] Failed to deep copy trajectory");
        free(robot->stored_trajectory_buffer);
        robot->stored_trajectory_buffer = NULL;
        robot->stored_trajectory_buffer_size = 0;
        return -1;
    }

    // Generate random goal_id (int32_t)
    int32_t goal_id;
    do {
        goal_id = (int32_t) rand();
    } while (goal_id == 0); // Ensure goal_id is never 0

    robot->current_goal_id = goal_id;
    robot->current_goal_state = GOAL_STATE_ACTIVE;
    robot->current_goal_progress = 0.0;
    robot->goal_start_time = get_timestamp_ms();

    // Start motion
    robot->trajectory_active = 1;
    robot->in_motion = 1;

    // Get stored trajectory for logging
    move_goal_payload_t *stored_goal = (move_goal_payload_t *) robot->stored_trajectory_buffer;
    trajectory_point_t *traj_data = MOVE_GOAL_GET_TRAJECTORY(stored_goal);

    pr_info("[MOCK] Goal accepted with ID: %d with size %d", goal_id, stored_goal->trajectory_size);
    for (uint32_t i = 0; i < stored_goal->trajectory_size; i++) {
        pr_info("[GRP %d][TS = %d.%d][ID=%d] p = (%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f) v = "
                "(%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f)",
                stored_goal->group_index, traj_data[i].time_from_start.sec, traj_data[i].time_from_start.nanos, i,
                traj_data[i].positions[0], traj_data[i].positions[1], traj_data[i].positions[2],
                traj_data[i].positions[3], traj_data[i].positions[4], traj_data[i].positions[5],
                traj_data[i].velocities[0], traj_data[i].velocities[1], traj_data[i].velocities[2],
                traj_data[i].velocities[3], traj_data[i].velocities[4], traj_data[i].velocities[5]);
    }
    pr_info("done");
    return goal_id;
}

int mock_robot_cancel_goal(mock_robot_t *robot, int32_t goal_id) {
    // Check if the goal matches
    if (robot->current_goal_id != goal_id) {
        pr_info("[MOCK] Cannot cancel goal - goal ID %d not found (current: %d)", goal_id, robot->current_goal_id);
        return -1;
    }
    // Check if goal is cancellable
    if (robot->current_goal_state != GOAL_STATE_PENDING && robot->current_goal_state != GOAL_STATE_ACTIVE) {
        pr_info("[MOCK] Cannot cancel goal - goal already in terminal state: %d", robot->current_goal_state);
        return -1;
    }
    // Cancel the goal
    robot->current_goal_state = GOAL_STATE_CANCELLED;
    robot->in_motion = 0;
    robot->trajectory_active = 0;

    pr_info("[MOCK] Goal %d cancelled", goal_id);
    return 0;
}

int mock_robot_get_goal_status(mock_robot_t *robot, int32_t goal_id, goal_status_payload_t *status) {
    if (robot->current_goal_id != goal_id) {
        pr_info("[MOCK] Goal ID %d not found (current: %d)", goal_id, robot->current_goal_id);
        return -1;
    }

    status->goal_id = robot->current_goal_id;
    status->state = robot->current_goal_state;
    status->progress = robot->current_goal_progress;
    status->timestamp_ms = get_timestamp_ms();

    return 0;
}

void mock_robot_update_goal_progress(mock_robot_t *robot) {
    // Only update if goal is active
    if (robot->current_goal_id == 0 || robot->current_goal_state != GOAL_STATE_ACTIVE) {
        return;
    }

    int64_t elapsed = get_timestamp_ms() - robot->goal_start_time;

    // Simulate progress: 1% per 100ms (complete in 10 seconds)
    robot->current_goal_progress = (double) elapsed / 1000.0;

    if (robot->current_goal_progress >= 1.0) {
        robot->current_goal_progress = 1.0;
        robot->current_goal_state = GOAL_STATE_SUCCEEDED;
        robot->in_motion = 0;
        robot->trajectory_active = 0;
        pr_info("[MOCK] Goal %d succeeded", robot->current_goal_id);
    }
}

// Network callback implementations
void mock_robot_on_connection(const char *client_ip, uint16_t client_port, void *user_data) {
    (void) user_data;
    mock_robot_t *robot = (mock_robot_t *) user_data;
    robot->connection_count++;
    pr_info("[SERVER] Client connected: %s:%d", client_ip, client_port);
}

void mock_robot_on_disconnection(const char *client_ip, uint16_t client_port, void *user_data) {
    (void) user_data;
    mock_robot_t *robot = (mock_robot_t *) user_data;
    robot->disconnection_count++;
    pr_info("[SERVER] Client disconnected: %s:%d", client_ip, client_port);
}

command_response_context_t *mock_robot_handle_command(protocol_header_t *header, void *payload, void *user_data) {
    mock_robot_t *robot = (mock_robot_t *) user_data;
    robot->command_count++;
    int err = 0;
    switch (header->message_type) {
    case MSG_TEST_TRAJECTORY_COMMAND:
        pr_info("[SERVER] Received TEST_TRAJECTORY_COMMAND");
        err = mock_robot_start_trajectory(robot);
        if (err != 0) {
            return MSG_ERR_RESPONSE(err);
        }
        return MSG_OK_RESPONSE();
    case MSG_TURN_SERVO_POWER_ON:
        pr_info("[SERVER] Received TURN_SERVO_POWER_ON");
        err = mock_robot_power_on(robot);
        if (err != 0) {
            return MSG_ERR_RESPONSE(err);
        }
        return MSG_OK_RESPONSE();
    case MSG_HEARTBEAT:
        err = mock_robot_heartbeat(robot, get_timestamp_ms());
        if (err != 0) {
            return MSG_ERR_RESPONSE(err);
        }
        return MSG_OK_RESPONSE();
    case MSG_TEST_ERROR_COMMAND:
        pr_info("[SERVER] Received TEST_ERROR_COMMAND");
        err = mock_robot_test_error_command(robot);
        if (err != 0) {
            return MSG_ERR_RESPONSE(err);
        }
        return MSG_OK_RESPONSE();
    case MSG_RESET_ERRORS:
        pr_info("[SERVER] Received RESET_ERRORS");
        err = mock_robot_reset_errors(robot);
        if (err != 0) {
            return MSG_ERR_RESPONSE(err);
        }
        return MSG_OK_RESPONSE();
    case MSG_MOVE_GOAL:
        pr_info("[SERVER] Received MOVE_GOAL");
        // Validate and cast move_goal payload (zero-copy)
        move_goal_payload_t *move_goal = move_goal_from_payload(payload, header->payload_length);
        if (!move_goal) {
            pr_info("[SERVER] Failed to validate move goal payload");
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        // Accept goal and get goal_id (mock_robot will deep copy the data)
        int32_t goal_id = mock_robot_accept_goal(robot, move_goal);
        if (goal_id < 0) {
            return MSG_ERR_RESPONSE(goal_id);
        }
        // Return MSG_GOAL_ACCEPTED with goal_id
        command_response_context_t *goal_ctx =
            allocate_response_context(sizeof(goal_accepted_payload_t), MSG_GOAL_ACCEPTED);
        if (goal_ctx == NULL) {
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        goal_accepted_payload_t *goal_accepted = (goal_accepted_payload_t *) goal_ctx->payload;
        goal_accepted->goal_id = goal_id;
        goal_accepted->timestamp_ms = get_timestamp_ms();
        return goal_ctx;
    case MSG_ECHO_TRAJECTORY:
        pr_info("[SERVER] Received ECHO_TRAJECTORY");
        {
            move_goal_payload_t *stored_traj = mock_robot_echo_trajectory(robot);
            if (!stored_traj) {
                err = -1; // No trajectory to echo
                return MSG_ERR_RESPONSE(err);
            }

            // Get tolerance size
            uint32_t *tol_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(stored_traj);
            uint32_t tolerance_size = *tol_size_ptr;

            // Calculate response size (entire move_goal payload)
            size_t response_size = MOVE_GOAL_CALC_SIZE(stored_traj->trajectory_size, tolerance_size);

            command_response_context_t *ctx = allocate_response_context(response_size, MSG_ECHO_TRAJECTORY);
            if (ctx == NULL) {
                err = -1;
                return MSG_ERR_RESPONSE(err);
            }

            // Copy entire move_goal payload to response
            memcpy(ctx->payload, stored_traj, response_size);

            return ctx;
        }
    case MSG_SET_MOTION_MODE:
        pr_info("[SERVER] Received SET_MOTION_MODE");
        if (header->payload_length != sizeof(motion_mode_payload_t)) {
            pr_info("[SERVER] SET_MOTION_MODE payload size mismatch");
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        motion_mode_payload_t *mode_payload = (motion_mode_payload_t *) payload;
        err = mock_robot_set_motion_mode(robot, mode_payload->motion_mode);
        if (err != 0) {
            return MSG_ERR_RESPONSE(err);
        }
        return MSG_OK_RESPONSE();
    case MSG_IS_IN_MOTION:
        pr_info("[SERVER] Received IS_IN_MOTION");
        if (header->payload_length != 0) {
            pr_info("[SERVER] IS_IN_MOTION should have no payload");
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        uint8_t is_in_motion = mock_robot_is_in_motion(robot);
        command_response_context_t *motion_ctx = allocate_response_context(sizeof(boolean_payload_t), MSG_IS_IN_MOTION);
        if (motion_ctx == NULL) {
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        boolean_payload_t *motion_payload = (boolean_payload_t *) motion_ctx->payload;
        motion_payload->value = is_in_motion;
        return motion_ctx;
    case MSG_STOP_MOTION:
        pr_info("[SERVER] Received STOP_MOTION");
        if (header->payload_length != 0) {
            pr_info("[SERVER] STOP_MOTION should have no payload");
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        uint8_t motion_stopped = mock_robot_stop_motion(robot);
        command_response_context_t *stop_ctx = allocate_response_context(sizeof(boolean_payload_t), MSG_STOP_MOTION);
        if (stop_ctx == NULL) {
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        boolean_payload_t *stop_payload = (boolean_payload_t *) stop_ctx->payload;
        stop_payload->value = motion_stopped;
        return stop_ctx;
    case MSG_CANCEL_GOAL:
        pr_info("[SERVER] Received CANCEL_GOAL");
        if (header->payload_length != sizeof(cancel_goal_payload_t)) {
            pr_info("[SERVER] CANCEL_GOAL payload size mismatch");
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        cancel_goal_payload_t *cancel_payload = (cancel_goal_payload_t *) payload;
        err = mock_robot_cancel_goal(robot, cancel_payload->goal_id);
        if (err != 0) {
            return MSG_ERR_RESPONSE(err);
        }
        return MSG_OK_RESPONSE();
    case MSG_GET_GOAL_STATUS:
        if (header->payload_length != sizeof(cancel_goal_payload_t)) { // Same structure as cancel
            pr_info("[SERVER] GET_GOAL_STATUS payload size mismatch");
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        cancel_goal_payload_t *status_req = (cancel_goal_payload_t *) payload;
        command_response_context_t *status_ctx =
            allocate_response_context(sizeof(goal_status_payload_t), MSG_GOAL_STATUS);
        if (status_ctx == NULL) {
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        goal_status_payload_t *goal_status = (goal_status_payload_t *) status_ctx->payload;
        err = mock_robot_get_goal_status(robot, status_req->goal_id, goal_status);
        if (err != 0) {
            free_command_response_context(status_ctx);
            return MSG_ERR_RESPONSE(err);
        }
        return status_ctx;
    case MSG_CHECK_GROUP:
        if (header->payload_length != sizeof(group_id_t)) { // Same structure as cancel
            pr_info("[SERVER] MSG_CHECK_GROUP payload size mismatch");
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        group_id_t *gid = (group_id_t *) payload;
        command_response_context_t *check_group_ctx =
            allocate_response_context(sizeof(boolean_payload_t), MSG_CHECK_GROUP);
        if (check_group_ctx == NULL) {
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        boolean_payload_t *check_group_payload = (boolean_payload_t *) check_group_ctx->payload;
        check_group_payload->value = gid->group_id < 1; // only one arm is configured
        return check_group_ctx;
    default:
        pr_info("[SERVER] Unknown message type: 0x%02x", header->message_type);
        err = -1;
        return MSG_ERR_RESPONSE(err); // Unknown command is an error
    }
}

command_response_context_t *mock_robot_get_error_info_callback(int32_t *error_code, void *user_data) {
    mock_robot_t *robot = (mock_robot_t *) user_data;
    pr_info("[SERVER] GET_ERROR_INFO callback called");

    // Use mock robot's get_error_info function
    char temp_buffer[512] = {0};
    mock_robot_get_error_info(robot, temp_buffer, sizeof(temp_buffer));

    if (strlen(temp_buffer) > 0) {
        command_response_context_t *ctx = allocate_response_context(strlen(temp_buffer), MSG_ERROR_INFO);
        if (ctx == NULL)
            return NULL;

        // We have error information
        *error_code = -1; // Generic error code for error info
        strcpy(ctx->payload, temp_buffer);
        ((char *) (ctx->payload))[strlen(temp_buffer)] = '\0';
        return ctx; // Has error info
    } else {
        return MSG_OK_RESPONSE(); // No error info
    }
}
