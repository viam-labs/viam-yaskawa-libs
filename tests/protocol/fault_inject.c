#include "fault_inject.h"
#include "logging.h"
#include "platform_api.h"
#include <string.h>

// Thread safety precondition: all rules must be added before the server starts
// accepting connections. Do not add or modify rules while the server is running.

void fault_inject_init(fault_inject_ctx_t *ctx, robot_server_ctx_t *server) {
    memset(ctx, 0, sizeof(fault_inject_ctx_t));
    ctx->server = server;
}

int fault_inject_add_rule(fault_inject_ctx_t *ctx, const fault_rule_t *rule) {
    for (int i = 0; i < FAULT_MAX_RULES; i++) {
        if (ctx->rules[i].fault == FAULT_NONE) {
            ctx->rules[i] = *rule;
            ctx->rules[i].current_count = 0;
            ctx->rule_fired[i] = false;
            return i;
        }
    }
    return -1; // No free slot
}

void fault_inject_reset(fault_inject_ctx_t *ctx) {
    robot_server_ctx_t *server = ctx->server;
    memset(ctx, 0, sizeof(fault_inject_ctx_t));
    ctx->server = server;
}

fault_type_t fault_inject_check(fault_inject_ctx_t *ctx, const protocol_header_t *header) {
    // Update global counter
    ctx->total_message_count++;

    // Evaluate rules in order; first match wins
    for (int i = 0; i < FAULT_MAX_RULES; i++) {
        fault_rule_t *rule = &ctx->rules[i];
        if (rule->fault == FAULT_NONE) {
            continue;
        }
        // Skip one-shot rules that have already fired
        if (rule->one_shot && ctx->rule_fired[i]) {
            continue;
        }

        bool triggered = false;
        switch (rule->trigger) {
        case TRIGGER_IMMEDIATE:
            triggered = true;
            break;
        case TRIGGER_AFTER_N_MESSAGES:
            triggered = (ctx->total_message_count >= rule->trigger_count);
            break;
        case TRIGGER_ON_MESSAGE_TYPE:
            triggered = (header->message_type == rule->trigger_message_type);
            break;
        case TRIGGER_AFTER_N_OF_TYPE:
            if (header->message_type == rule->trigger_message_type) {
                rule->current_count++;
                triggered = (rule->current_count >= rule->trigger_count);
            }
            break;
        }

        if (triggered) {
            if (rule->one_shot) {
                ctx->rule_fired[i] = true;
            }
            pr_info("[FAULT] Rule %d triggered: fault=%d trigger=%d (msg_type=0x%02x, total=%u)", i, rule->fault,
                    rule->trigger, header->message_type, ctx->total_message_count);
            return rule->fault;
        }
    }
    return FAULT_NONE;
}

int fault_inject_execute(fault_inject_ctx_t *ctx, fault_type_t fault) {
    if (!ctx->server) {
        return -1;
    }

    switch (fault) {
    case FAULT_NONE:
        return 0;

    case FAULT_DISCONNECT:
        // Cleanly shut down the client socket. The next select() / recv() in
        // tcp_thread_func will detect the closed fd and call remove_client().
        // Do NOT call platform_close() here — that would cause a double-close
        // when remove_client() later closes the same fd.
        pr_info("[FAULT] Executing DISCONNECT");
        if (ctx->server->client.connected) {
            platform_shutdown(ctx->server->client.socket, SHUT_RDWR);
        }
        return 0;

    case FAULT_UNRESPONSIVE:
        // Blocks the TCP thread, preventing all message processing.
        // Uses a loop of short sleeps so that tearDown (robot_protocol_stop)
        // can join the TCP thread without hanging — the loop exits once
        // the server's running flag is cleared.
        pr_info("[FAULT] Executing UNRESPONSIVE (blocking up to 5s)");
        for (int ms = 0; ms < 5000 && ctx->server->running; ms += 50) {
            platform_usleep(50000); // 50ms per iteration
        }
        return 0;

    case FAULT_CLOSE_CONNECTION:
        // Abrupt server-side close — represents a controller alarm/crash.
        // Same mechanism as DISCONNECT (platform_shutdown only, no error payload).
        // The separate enum exists for test readability.
        pr_info("[FAULT] Executing CLOSE_CONNECTION");
        if (ctx->server->client.connected) {
            platform_shutdown(ctx->server->client.socket, SHUT_RDWR);
        }
        return 0;
    }
    return -1;
}

// --- Convenience helpers ---

int fault_inject_disconnect_after(fault_inject_ctx_t *ctx, uint32_t n_messages) {
    fault_rule_t rule = {
        .fault = FAULT_DISCONNECT,
        .trigger = TRIGGER_AFTER_N_MESSAGES,
        .trigger_message_type = 0,
        .trigger_count = n_messages,
        .one_shot = true,
        .current_count = 0,
    };
    return fault_inject_add_rule(ctx, &rule);
}

int fault_inject_unresponsive_on(fault_inject_ctx_t *ctx, uint8_t message_type) {
    fault_rule_t rule = {
        .fault = FAULT_UNRESPONSIVE,
        .trigger = TRIGGER_ON_MESSAGE_TYPE,
        .trigger_message_type = message_type,
        .trigger_count = 0,
        .one_shot = false,
        .current_count = 0,
    };
    return fault_inject_add_rule(ctx, &rule);
}

int fault_inject_close_connection(fault_inject_ctx_t *ctx) {
    fault_rule_t rule = {
        .fault = FAULT_CLOSE_CONNECTION,
        .trigger = TRIGGER_IMMEDIATE,
        .trigger_message_type = 0,
        .trigger_count = 0,
        .one_shot = true,
        .current_count = 0,
    };
    return fault_inject_add_rule(ctx, &rule);
}
