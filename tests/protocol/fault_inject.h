#ifndef FAULT_INJECT_H
#define FAULT_INJECT_H

#include "protocol.h"
#include "robot_protocol.h"
#include <stdbool.h>
#include <stdint.h>

// Maximum number of fault rules that can be active simultaneously
#define FAULT_MAX_RULES 8

// Types of faults that can be injected
typedef enum {
    FAULT_NONE = 0,         // No fault (rule is inactive)
    FAULT_DISCONNECT,       // Cleanly disconnect the client via shutdown(SHUT_RDWR)
    FAULT_UNRESPONSIVE,     // Block the TCP thread (no response sent)
    FAULT_CLOSE_CONNECTION, // Abruptly close the TCP socket (same mechanism as DISCONNECT;
                            // separate enum for test readability — represents server-side alarm/crash)
} fault_type_t;

// Trigger conditions that determine when a fault fires
typedef enum {
    TRIGGER_IMMEDIATE = 0,    // Fire on the very next message
    TRIGGER_AFTER_N_MESSAGES, // Fire after N total messages received
    TRIGGER_ON_MESSAGE_TYPE,  // Fire when a specific message type is received
    TRIGGER_AFTER_N_OF_TYPE,  // Fire after N messages of a specific type
} fault_trigger_type_t;

// A single fault injection rule
typedef struct {
    fault_type_t fault;               // What fault to inject
    fault_trigger_type_t trigger;     // When to inject it
    uint8_t trigger_message_type;     // Message type to match (TRIGGER_ON_MESSAGE_TYPE / TRIGGER_AFTER_N_OF_TYPE)
    uint32_t trigger_count;           // N threshold (TRIGGER_AFTER_N_MESSAGES / TRIGGER_AFTER_N_OF_TYPE)
    bool one_shot;                    // If true, rule deactivates after firing once
    uint32_t current_count;           // Internal: per-rule match counter (used by TRIGGER_AFTER_N_OF_TYPE)
} fault_rule_t;

// Fault injection context
// Not thread-safe. Rules must be configured before server starts accepting connections.
typedef struct {
    fault_rule_t rules[FAULT_MAX_RULES]; // Active fault rules
    uint32_t total_message_count;        // Total messages received since init/reset
    bool rule_fired[FAULT_MAX_RULES];    // Tracks whether each one-shot rule has already fired
    robot_server_ctx_t *server;          // Pointer to the server context (for executing disconnect/close)
} fault_inject_ctx_t;

// Initialize the fault injection context
// server may be NULL if fault execution is handled externally
void fault_inject_init(fault_inject_ctx_t *ctx, robot_server_ctx_t *server);

// Add a fault rule. Returns the rule index (0..FAULT_MAX_RULES-1) on success, -1 if full.
int fault_inject_add_rule(fault_inject_ctx_t *ctx, const fault_rule_t *rule);

// Reset all rules and counters
void fault_inject_reset(fault_inject_ctx_t *ctx);

// Check whether any rule triggers for the given message.
// Updates internal counters. Returns the fault_type_t to execute, or FAULT_NONE.
fault_type_t fault_inject_check(fault_inject_ctx_t *ctx, const protocol_header_t *header);

// Execute a fault action against the server context.
// Returns 0 on success, -1 on error (e.g., no server pointer).
int fault_inject_execute(fault_inject_ctx_t *ctx, fault_type_t fault);

// --- Convenience helpers ---

// Disconnect the client after N total messages
int fault_inject_disconnect_after(fault_inject_ctx_t *ctx, uint32_t n_messages);

// Become unresponsive whenever a specific message type is received
int fault_inject_unresponsive_on(fault_inject_ctx_t *ctx, uint8_t message_type);

// Abruptly close the connection (immediate, one-shot)
int fault_inject_close_connection(fault_inject_ctx_t *ctx);

#endif // FAULT_INJECT_H
