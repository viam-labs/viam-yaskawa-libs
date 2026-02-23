
#include "../include/logging.h"
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* Global logging configuration */
static logging_config_t g_logging_config = {NULL, LOG_DEBUG};

int logging_initialize(ring_buffer_t *rb) {
    if (rb == NULL) {
        return -1;
    }

    g_logging_config.ring_buffer = rb;
    g_logging_config.current_level = LOG_DEBUG;

    return 0;
}

void logging_set_level(log_level_t level) {
    g_logging_config.current_level = level;
}

log_level_t logging_get_level(void) {
    return g_logging_config.current_level;
}

int write_log_message(log_level_t level, const char *fmt, ...) {
    /* Check if logging is initialized */
    if (g_logging_config.ring_buffer == NULL) {
        return -1;
    }

    /* Filter out messages above current log level */
    if (level > g_logging_config.current_level) {
        return 0;
    }

    /* Initialize temporary buffer */
    char temp_buffer[1024];
    size_t offset = 0;

    /* Get current time and format it */
    struct timespec ts;
    struct tm tm_info;

    if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
        if (localtime_r(&ts.tv_sec, &tm_info) != NULL) {
            offset = strftime(temp_buffer, sizeof(temp_buffer), "%Y-%m-%d %H:%M:%S ", &tm_info);
        }
    }

    /* Format the user message using variadic arguments */
    va_list args;
    va_start(args, fmt);

    size_t remaining_space = sizeof(temp_buffer) - offset;
    int formatted_len = vsnprintf(temp_buffer + offset, remaining_space, fmt, args);

    va_end(args);

    size_t total_len = offset;
    if (formatted_len > 0) {
        total_len = (size_t) formatted_len < remaining_space ? formatted_len + offset : sizeof(temp_buffer) - 1;
    }
    /* Write to ring buffer and return bytes not written */
    return ring_buffer_write_n(g_logging_config.ring_buffer, temp_buffer, total_len);
}

void logging_shutdown(void) {
    g_logging_config.ring_buffer = NULL;
    g_logging_config.current_level = LOG_DEBUG;
}

void log_to_stdout(void) {
    char buffer[1024];

    size_t bytes_read = ring_buffer_read_n(g_logging_config.ring_buffer, buffer, sizeof(buffer) - 1);
    if (bytes_read == 0) {
        return;
    }

    buffer[bytes_read] = '\0';

    size_t message_len = strlen(buffer);
    if (message_len == 0) {
        return;
    }

    printf("%s", buffer);
}
