#ifndef LOGGING_H
#define LOGGING_H
#include "ring_buffer.h"

/* Log levels */
typedef enum {
    LOG_CRITICAL = 0,
    LOG_ERROR = 1,
    LOG_WARNING = 2,
    LOG_INFO = 3,
    LOG_DEBUG = 4
} log_level_t;

/* Global logging configuration */
typedef struct {
    ring_buffer_t *ring_buffer;
    log_level_t current_level;
} logging_config_t;

/* Logging API functions */

/*
 * Initialize the logging system
 * @param rb: Ring buffer to use for logging
 * @return: 0 on success, -1 on error
 */
int logging_initialize(ring_buffer_t *rb);

/*
 * Set the current log level
 * @param level: New log level (messages above this level will be filtered)
 */
void logging_set_level(log_level_t level);

/*
 * Get the current log level
 * @return: Current log level
 */
log_level_t logging_get_level(void);

/*
 * Write a log message
 * @param level: Log level of the message
 * @param fmt: Format string (like printf)
 * @param ...: Variable arguments for format string
 * @return: Number of bytes that were NOT written (0 on complete success)
 */
int write_log_message(log_level_t level, const char *fmt, ...);

/*
 * Shutdown the logging system
 */
void logging_shutdown(void);

void log_to_stdout(void);

/* Logging macros */
#define pr_crit(format, ...) write_log_message(LOG_CRITICAL, "-C-" format "\r\n", ##__VA_ARGS__)
#define pr_error(format, ...) write_log_message(LOG_ERROR, "-E-" format "\r\n", ##__VA_ARGS__)
#define pr_warn(format, ...) write_log_message(LOG_WARNING, "-W-" format "\r\n", ##__VA_ARGS__)
#define pr_info(format, ...) write_log_message(LOG_INFO, "-I-" format "\r\n", ##__VA_ARGS__)
#define pr_debug(format, ...) write_log_message(LOG_DEBUG, "-D-" format "\r\n", ##__VA_ARGS__)

#endif /* LOGGING_H */
