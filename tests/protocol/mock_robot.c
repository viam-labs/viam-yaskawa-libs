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

// Helper to find group by goal_id
static mock_group_state_t *find_group_with_goal(mock_robot_t *robot, int32_t goal_id) {
    for (int i = 0; i < robot->num_groups; i++) {
        if (robot->groups[i].current_goal_id == goal_id) {
            return &robot->groups[i];
        }
    }
    return NULL;
}

static void init_group(mock_group_state_t *grp, uint8_t group_type, uint8_t num_axes) {
    memset(grp, 0, sizeof(mock_group_state_t));
    grp->group_type = group_type;
    grp->num_axes = num_axes;
    grp->current_goal_state = GOAL_STATE_PENDING;
    for (int i = 0; i < MAX_AXES; i++) {
        grp->axis_types[i] = (i < num_axes) ? AXIS_TYPE_ROTARY : AXIS_TYPE_INVALID;
        grp->base_axis_motion[i] = -1; // NOT_USED
    }
    // For base groups, default first axis to linear
    if (group_type == GROUP_TYPE_BASE) {
        for (int i = 0; i < num_axes; i++) {
            grp->axis_types[i] = AXIS_TYPE_LINEAR;
            grp->base_axis_motion[i] = i; // X=0, Y=1, Z=2...
        }
    }
}

void mock_robot_init(mock_robot_t *robot) {
    memset(robot, 0, sizeof(mock_robot_t));
    robot->servo_power_on = 0;
    robot->last_heartbeat = 0;
    robot->sequence_counter = 1;
    robot->has_error_info = 0;
    robot->disconnection_count = 0;
    robot->command_count = 0;
    robot->connection_count = 0;
    robot->motion_mode = 0;

    // Default: 1 robot group with 6 axes
    robot->num_groups = 1;
    init_group(&robot->groups[0], GROUP_TYPE_ROBOT, 6);

    // Initialize robot status
    robot->mode = 1;
    robot->e_stopped = 0;
    robot->drives_powered = 0;
    robot->motion_possible = 0;
    robot->in_error = 0;
    robot->error_count = 0;
    memset(robot->error_codes, 0, sizeof(robot->error_codes));
}

void mock_robot_init_multi(mock_robot_t *robot, uint8_t num_groups, const uint8_t group_types[],
                           const uint8_t group_axes[]) {
    memset(robot, 0, sizeof(mock_robot_t));
    robot->servo_power_on = 0;
    robot->last_heartbeat = 0;
    robot->sequence_counter = 1;
    robot->has_error_info = 0;
    robot->disconnection_count = 0;
    robot->command_count = 0;
    robot->connection_count = 0;
    robot->motion_mode = 0;

    robot->num_groups = (num_groups <= MOCK_MAX_GROUPS) ? num_groups : MOCK_MAX_GROUPS;
    for (int i = 0; i < robot->num_groups; i++) {
        init_group(&robot->groups[i], group_types[i], group_axes[i]);
    }

    robot->mode = 1;
    robot->e_stopped = 0;
    robot->drives_powered = 0;
    robot->motion_possible = 0;
    robot->in_error = 0;
    robot->error_count = 0;
    memset(robot->error_codes, 0, sizeof(robot->error_codes));
}

int mock_robot_power_on(mock_robot_t *robot) {
    robot->servo_power_on = 1;
    robot->drives_powered = 1;
    robot->motion_possible = 1;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    robot->last_heartbeat = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    pr_info("[MOCK] Servo power ON");
    return 0;
}

int mock_robot_start_trajectory(mock_robot_t *robot) {
    if (robot->servo_power_on) {
        mock_group_state_t *grp = &robot->groups[0];
        grp->trajectory_active = 1;
        grp->in_motion = 1;

        for (int i = 0; i < grp->num_axes; i++) {
            grp->positions[i] = i * 1.0;
            grp->velocities[i] = i * 0.1;
            grp->torques[i] = i * 0.01;
            grp->corrected_positions[i] = grp->positions[i] + 0.01;
        }
        pr_info("[MOCK] Trajectory started");
        return 0;
    } else {
        pr_info("[MOCK] Cannot start trajectory - servo power is OFF");
        set_error_info(robot, "Cannot start trajectory: servo power must be enabled first. Use TURN_SERVO_POWER_ON "
                              "command to enable servo power.");
        robot->in_error = 1;
        robot->error_codes[0] = 100;
        robot->error_count = 1;
        return -1;
    }
}

int mock_robot_heartbeat(mock_robot_t *robot, int64_t timestamp) {
    robot->last_heartbeat = timestamp;
    return 0;
}

int mock_robot_test_error_command(mock_robot_t *robot) {
    pr_info("[MOCK] Test error command - always returns error");
    set_error_info(robot, "Test error command executed. This command is designed to always fail for testing purposes. "
                          "Error details: simulated "
                          "fault condition in axis controller #3.");
    robot->in_error = 1;
    robot->error_codes[0] = 42;
    robot->error_count = 1;
    return -42;
}

void mock_robot_get_status(mock_robot_t *robot, status_payload_t *status, int64_t timestamp, uint8_t group_index) {
    if (group_index >= robot->num_groups) {
        memset(status, 0, sizeof(status_payload_t));
        return;
    }
    mock_group_state_t *grp = &robot->groups[group_index];

    status->group_index = group_index;
    status->group_type = grp->group_type;
    status->timestamp = timestamp;
    status->num_axes = grp->num_axes;

    for (int i = 0; i < grp->num_axes; i++) {
        status->position[i] = grp->positions[i];
        status->velocity[i] = grp->velocities[i];
        status->torque[i] = grp->torques[i];
        status->position_corrected[i] = grp->corrected_positions[i];
    }
}

void mock_robot_update(mock_robot_t *robot, int64_t timestamp) {
    (void) timestamp;
    if (!robot->servo_power_on) {
        for (int g = 0; g < robot->num_groups; g++) {
            mock_group_state_t *grp = &robot->groups[g];
            for (int i = 0; i < grp->num_axes; i++) {
                grp->velocities[i] *= 0.95;
                grp->torques[i] *= 0.95;
            }
        }
    }
    mock_robot_update_goal_progress(robot);
}

int mock_robot_get_error_info(mock_robot_t *robot, char *error_buffer, size_t buffer_size) {
    if (!robot->has_error_info) {
        pr_info("[MOCK] No error information available");
        return 0;
    }
    strncpy(error_buffer, robot->last_error_message, buffer_size - 1);
    error_buffer[buffer_size - 1] = '\0';

    robot->has_error_info = 0;
    memset(robot->last_error_message, 0, sizeof(robot->last_error_message));
    pr_info("[MOCK] Retrieved error info: %s", error_buffer);
    return 0;
}

void mock_robot_get_robot_status(mock_robot_t *robot, robot_status_payload_t *status, int64_t timestamp,
                                 uint8_t group_index) {
    status->ts = timestamp;
    status->group_index = group_index;
    status->mode = robot->mode;
    status->e_stopped = robot->e_stopped;
    status->drives_powered = robot->drives_powered;
    status->motion_possible = robot->motion_possible;
    status->in_error = robot->in_error;
    status->size = robot->error_count;

    // Per-group in_motion
    if (group_index < robot->num_groups) {
        status->in_motion = robot->groups[group_index].in_motion;
    } else {
        status->in_motion = 0;
    }

    for (int i = 0; i < MAX_ALARM_COUNT + 1 && i < robot->error_count; i++) {
        status->error_codes[i] = robot->error_codes[i];
    }
    for (int i = robot->error_count; i < MAX_ALARM_COUNT + 1; i++) {
        status->error_codes[i] = 0;
    }
}

int mock_robot_reset_errors(mock_robot_t *robot) {
    pr_info("[MOCK] Resetting all errors");
    robot->in_error = 0;
    robot->error_count = 0;
    robot->has_error_info = 0;
    memset(robot->last_error_message, 0, sizeof(robot->last_error_message));
    memset(robot->error_codes, 0, sizeof(robot->error_codes));
    return 0;
}

int mock_robot_move_goal(mock_robot_t *robot, const move_goal_payload_t *trajectory) {
    uint32_t grp_idx = trajectory->group_index;
    if (grp_idx >= robot->num_groups) {
        pr_error("[MOCK] Invalid group_index %u (have %u groups)", grp_idx, robot->num_groups);
        return -1;
    }
    mock_group_state_t *grp = &robot->groups[grp_idx];

    const uint32_t *tol_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(trajectory);
    uint32_t tolerance_size = *tol_size_ptr;

    pr_info("[MOCK] Executing move goal group=%u with %u trajectory points and %u tolerance points", grp_idx,
            trajectory->trajectory_size, tolerance_size);

    uint32_t required_size = MOVE_GOAL_CALC_SIZE(trajectory->trajectory_size, tolerance_size);

    if (grp->stored_trajectory_buffer) {
        free(grp->stored_trajectory_buffer);
        grp->stored_trajectory_buffer = NULL;
        grp->stored_trajectory_buffer_size = 0;
    }

    grp->stored_trajectory_buffer = malloc(required_size);
    if (!grp->stored_trajectory_buffer) {
        pr_error("[MOCK] Failed to allocate %u bytes for trajectory storage", required_size);
        return -1;
    }
    grp->stored_trajectory_buffer_size = required_size;

    if (move_goal_deep_copy(trajectory, grp->stored_trajectory_buffer, required_size) == NULL) {
        pr_error("[MOCK] Failed to deep copy trajectory");
        free(grp->stored_trajectory_buffer);
        grp->stored_trajectory_buffer = NULL;
        grp->stored_trajectory_buffer_size = 0;
        return -1;
    }

    grp->trajectory_active = 1;
    grp->in_motion = 1;

    pr_info("[MOCK] Move goal stored (%u bytes) and trajectory started for group %u", required_size, grp_idx);
    return 0;
}

move_goal_payload_t *mock_robot_echo_trajectory(mock_robot_t *robot, uint8_t group_index) {
    if (group_index >= robot->num_groups) {
        return NULL;
    }
    mock_group_state_t *grp = &robot->groups[group_index];
    if (!grp->stored_trajectory_buffer) {
        pr_info("[MOCK] No trajectory to echo for group %u - send a move goal first", group_index);
        return NULL;
    }

    move_goal_payload_t *stored_goal = (move_goal_payload_t *) grp->stored_trajectory_buffer;
    pr_info("[MOCK] Echoing trajectory for group %u with %u points", group_index, stored_goal->trajectory_size);
    return stored_goal;
}

int mock_robot_set_motion_mode(mock_robot_t *robot, uint8_t motion_mode) {
    pr_info("[MOCK] Setting motion mode to %u", motion_mode);
    robot->motion_mode = motion_mode;
    return 0;
}

uint8_t mock_robot_is_in_motion(mock_robot_t *robot) {
    for (int i = 0; i < robot->num_groups; i++) {
        if (robot->groups[i].in_motion) {
            pr_info("[MOCK] Group %d is in motion", i);
            return 1;
        }
    }
    pr_info("[MOCK] No groups in motion");
    return 0;
}

uint8_t mock_robot_stop_motion(mock_robot_t *robot) {
    uint8_t stopped = 0;
    for (int i = 0; i < robot->num_groups; i++) {
        _Bool has_active_goal = robot->groups[i].current_goal_id != 0 &&
                                (robot->groups[i].current_goal_state == GOAL_STATE_ACTIVE ||
                                 robot->groups[i].current_goal_state == GOAL_STATE_PENDING);
        if (robot->groups[i].in_motion || has_active_goal) {
            robot->groups[i].in_motion = 0;
            robot->groups[i].trajectory_active = 0;
            if (has_active_goal) {
                robot->groups[i].current_goal_state = GOAL_STATE_ABORTED;
            }
            stopped = 1;
        }
    }
    pr_info("[MOCK] %s", stopped ? "All motion stopped" : "No motion to stop");
    return stopped;
}

uint8_t mock_robot_stop_group(mock_robot_t *robot, int group_index) {
    if (group_index < 0 || group_index >= robot->num_groups) {
        pr_error("[MOCK] Invalid group_index %d for stop", group_index);
        return 0;
    }
    mock_group_state_t *grp = &robot->groups[group_index];
    grp->in_motion = 0;
    grp->trajectory_active = 0;
    if (grp->current_goal_id != 0 &&
        (grp->current_goal_state == GOAL_STATE_ACTIVE || grp->current_goal_state == GOAL_STATE_PENDING)) {
        grp->current_goal_state = GOAL_STATE_ABORTED;
    }
    pr_info("[MOCK] Stopped motion for group %d", group_index);
    return 1;
}

// Goal tracking implementation
int32_t mock_robot_accept_goal(mock_robot_t *robot, const move_goal_payload_t *trajectory) {
    if (!robot->servo_power_on) {
        pr_info("[MOCK] Cannot accept goal - servo power is OFF");
        return -1;
    }

    uint32_t grp_idx = trajectory->group_index;
    if (grp_idx >= robot->num_groups) {
        pr_error("[MOCK] Invalid group_index %u", grp_idx);
        return -1;
    }
    mock_group_state_t *grp = &robot->groups[grp_idx];

    // Check if another goal is active on this group
    if (grp->current_goal_id != 0 &&
        (grp->current_goal_state == GOAL_STATE_PENDING || grp->current_goal_state == GOAL_STATE_ACTIVE)) {
        pr_info("[MOCK] Cannot accept goal - group %u already has active goal (ID: %d, state: %d)", grp_idx,
                grp->current_goal_id, grp->current_goal_state);
        return -1;
    }
    if (grp->current_goal_id != 0) {
        pr_info("[MOCK] Previous goal %d in terminal state %d, accepting new goal", grp->current_goal_id,
                grp->current_goal_state);
    }

    const uint32_t *tol_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(trajectory);
    uint32_t tolerance_size = *tol_size_ptr;
    uint32_t required_size = MOVE_GOAL_CALC_SIZE(trajectory->trajectory_size, tolerance_size);

    if (grp->stored_trajectory_buffer) {
        free(grp->stored_trajectory_buffer);
        grp->stored_trajectory_buffer = NULL;
        grp->stored_trajectory_buffer_size = 0;
    }

    grp->stored_trajectory_buffer = malloc(required_size);
    if (!grp->stored_trajectory_buffer) {
        pr_error("[MOCK] Failed to allocate %u bytes for trajectory storage", required_size);
        return -1;
    }
    grp->stored_trajectory_buffer_size = required_size;

    if (move_goal_deep_copy(trajectory, grp->stored_trajectory_buffer, required_size) == NULL) {
        pr_error("[MOCK] Failed to deep copy trajectory");
        free(grp->stored_trajectory_buffer);
        grp->stored_trajectory_buffer = NULL;
        grp->stored_trajectory_buffer_size = 0;
        return -1;
    }

    int32_t goal_id;
    do {
        goal_id = (int32_t) rand();
    } while (goal_id == 0);

    grp->current_goal_id = goal_id;
    grp->current_goal_state = GOAL_STATE_ACTIVE;
    grp->current_goal_progress = 0.0;
    grp->goal_start_time = get_timestamp_ms();

    grp->trajectory_active = 1;
    grp->in_motion = 1;

    move_goal_payload_t *stored_goal = (move_goal_payload_t *) grp->stored_trajectory_buffer;
    trajectory_point_t *traj_data = MOVE_GOAL_GET_TRAJECTORY(stored_goal);
    uint8_t num_axes = grp->num_axes;

    pr_info("[MOCK] Goal accepted with ID: %d for group %u with size %d", goal_id, grp_idx,
            stored_goal->trajectory_size);
    for (uint32_t i = 0; i < stored_goal->trajectory_size; i++) {
        // Log first two axes for any group size
        if (num_axes >= 2) {
            pr_info("[GRP %d][TS = %d.%d][ID=%d] p[0]=%0.5f p[1]=%0.5f v[0]=%0.5f v[1]=%0.5f",
                    stored_goal->group_index, traj_data[i].time_from_start.sec, traj_data[i].time_from_start.nanos, i,
                    traj_data[i].positions[0], traj_data[i].positions[1], traj_data[i].velocities[0],
                    traj_data[i].velocities[1]);
        }
    }
    pr_info("done");
    return goal_id;
}

int mock_robot_cancel_goal(mock_robot_t *robot, int32_t goal_id) {
    mock_group_state_t *grp = find_group_with_goal(robot, goal_id);
    if (!grp) {
        pr_info("[MOCK] Cannot cancel goal - goal ID %d not found", goal_id);
        return -1;
    }
    if (grp->current_goal_state != GOAL_STATE_PENDING && grp->current_goal_state != GOAL_STATE_ACTIVE) {
        pr_info("[MOCK] Cannot cancel goal - goal already in terminal state: %d", grp->current_goal_state);
        return -1;
    }
    grp->current_goal_state = GOAL_STATE_CANCELLED;
    grp->in_motion = 0;
    grp->trajectory_active = 0;

    pr_info("[MOCK] Goal %d cancelled", goal_id);
    return 0;
}

int mock_robot_get_goal_status(mock_robot_t *robot, int32_t goal_id, goal_status_payload_t *status) {
    mock_group_state_t *grp = find_group_with_goal(robot, goal_id);
    if (!grp) {
        pr_info("[MOCK] Goal ID %d not found", goal_id);
        return -1;
    }

    status->goal_id = grp->current_goal_id;
    status->state = grp->current_goal_state;
    status->progress = grp->current_goal_progress;
    status->timestamp_ms = get_timestamp_ms();

    return 0;
}

void mock_robot_update_goal_progress(mock_robot_t *robot) {
    for (int g = 0; g < robot->num_groups; g++) {
        mock_group_state_t *grp = &robot->groups[g];
        if (grp->current_goal_id == 0 || grp->current_goal_state != GOAL_STATE_ACTIVE) {
            continue;
        }

        int64_t elapsed = get_timestamp_ms() - grp->goal_start_time;
        grp->current_goal_progress = (double) elapsed / 1000.0;

        if (grp->current_goal_progress >= 1.0) {
            grp->current_goal_progress = 1.0;
            grp->current_goal_state = GOAL_STATE_SUCCEEDED;
            grp->in_motion = 0;
            grp->trajectory_active = 0;
            pr_info("[MOCK] Goal %d succeeded (group %d)", grp->current_goal_id, g);
        }
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
    // Stop all motion on disconnect — matches real controller on_disconnection behavior
    mock_robot_stop_motion(robot);
    pr_info("[SERVER] Client disconnected: %s:%d (all motion stopped)", client_ip, client_port);
}

static command_response_context_t *handle_get_capabilities(mock_robot_t *robot) {
    command_response_context_t *ctx = allocate_response_context(sizeof(capabilities_payload_t), MSG_CAPABILITIES);
    if (!ctx)
        return NULL;

    capabilities_payload_t *caps = (capabilities_payload_t *) ctx->payload;
    memset(caps, 0, sizeof(capabilities_payload_t));
    caps->protocol_version = PROTOCOL_VERSION;
    caps->num_groups = robot->num_groups;

    for (int i = 0; i < robot->num_groups && i < MAX_GROUPS; i++) {
        mock_group_state_t *grp = &robot->groups[i];
        caps->groups[i].group_id = i;
        caps->groups[i].group_type = grp->group_type;
        caps->groups[i].group_sub_index = i;
        caps->groups[i].num_axes = grp->num_axes;
        caps->groups[i].interpolation_period_us = 4000; // 4ms default
        memcpy(caps->groups[i].axis_types, grp->axis_types, MAX_AXES);
        memcpy(caps->groups[i].base_axis_motion, grp->base_axis_motion, MAX_AXES);
    }

    return ctx;
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
        move_goal_payload_t *move_goal = move_goal_from_payload(payload, header->payload_length);
        if (!move_goal) {
            pr_info("[SERVER] Failed to validate move goal payload");
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        int32_t goal_id = mock_robot_accept_goal(robot, move_goal);
        if (goal_id < 0) {
            return MSG_ERR_RESPONSE(goal_id);
        }
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
            // Echo trajectory from group 0 by default
            move_goal_payload_t *stored_traj = mock_robot_echo_trajectory(robot, 0);
            if (!stored_traj) {
                err = -1;
                return MSG_ERR_RESPONSE(err);
            }

            uint32_t *tol_size_ptr = MOVE_GOAL_GET_TOLERANCE_SIZE_PTR(stored_traj);
            uint32_t tolerance_size = *tol_size_ptr;

            size_t response_size = MOVE_GOAL_CALC_SIZE(stored_traj->trajectory_size, tolerance_size);

            command_response_context_t *ctx = allocate_response_context(response_size, MSG_ECHO_TRAJECTORY);
            if (ctx == NULL) {
                err = -1;
                return MSG_ERR_RESPONSE(err);
            }

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
        if (header->payload_length != sizeof(group_id_t)) {
            pr_info("[SERVER] STOP_MOTION payload size mismatch");
            err = -1;
            return MSG_ERR_RESPONSE(err);
        }
        group_id_t *stop_gid = (group_id_t *) payload;
        uint8_t motion_stopped;
        if (stop_gid->group_id == GROUP_ID_ALL) {
            motion_stopped = mock_robot_stop_motion(robot);
        } else {
            motion_stopped = mock_robot_stop_group(robot, stop_gid->group_id);
        }
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
        if (header->payload_length != sizeof(cancel_goal_payload_t)) {
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
        if (header->payload_length != sizeof(group_id_t)) {
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
        check_group_payload->value = gid->group_id < robot->num_groups;
        return check_group_ctx;
    case MSG_GET_CAPABILITIES:
        pr_info("[SERVER] Received GET_CAPABILITIES");
        return handle_get_capabilities(robot);
    default:
        pr_info("[SERVER] Unknown message type: 0x%02x", header->message_type);
        err = -1;
        return MSG_ERR_RESPONSE(err);
    }
}

command_response_context_t *mock_robot_get_error_info_callback(int32_t *error_code, void *user_data) {
    mock_robot_t *robot = (mock_robot_t *) user_data;
    pr_info("[SERVER] GET_ERROR_INFO callback called");

    char temp_buffer[512] = {0};
    mock_robot_get_error_info(robot, temp_buffer, sizeof(temp_buffer));

    if (strlen(temp_buffer) > 0) {
        command_response_context_t *ctx = allocate_response_context(sizeof(error_payload_t), MSG_ERROR_INFO);
        if (ctx == NULL)
            return NULL;

        *error_code = -1;
        error_payload_t *ep = (error_payload_t *) ctx->payload;
        ep->error_code = -1;
        strncpy(ep->message, temp_buffer, VIAM_ERROR_MESSAGE_MAX_LEN - 1);
        ep->message[VIAM_ERROR_MESSAGE_MAX_LEN - 1] = '\0';
        return ctx;
    } else {
        return MSG_OK_RESPONSE();
    }
}
