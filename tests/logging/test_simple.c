#define _POSIX_C_SOURCE 200809L
#include "logging.h"
#include "ring_buffer.h"
#include "unity.h"
#include <string.h>

/* Test buffer */
static char test_buffer[RING_BUFFER_DEFAULT_SIZE];
static ring_buffer_t rb;

void setUp(void) {
    ring_buffer_initialize(&rb, test_buffer, sizeof(test_buffer));
}

void tearDown(void) {
    ring_buffer_destroy(&rb);
}

void test_ring_buffer_basic(void) {
    const char *test_data = "Hello, World!";
    char read_buffer[64];
    size_t written, read;

    written = ring_buffer_write_n(&rb, test_data, strlen(test_data));
    TEST_ASSERT_EQUAL(strlen(test_data), written);

    TEST_ASSERT_EQUAL(strlen(test_data), ring_buffer_used_space(&rb));

    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer));
    TEST_ASSERT_EQUAL(strlen(test_data), read);
    TEST_ASSERT_EQUAL_STRING_LEN(test_data, read_buffer, strlen(test_data));
}

void test_logging_basic(void) {
    char read_buffer[512];
    size_t read;

    /* Initialize logging */
    logging_initialize(&rb);
    logging_set_level(LOG_INFO);

    /* Write test message */
    TEST_ASSERT_GREATER_THAN(0, write_log_message(LOG_INFO, "Test message %d", 42));

    /* Read message */
    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer) - 1);
    read_buffer[read] = '\0';

    /* Should contain the message */
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "Test message 42"));

    logging_shutdown();
}

void test_log_macros(void) {
    char read_buffer[1024];
    size_t read;

    logging_initialize(&rb);
    logging_set_level(LOG_DEBUG);

    pr_crit("Critical message");
    pr_warn("Warning message");
    pr_info("Info message");
    pr_debug("Debug message");

    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer) - 1);
    read_buffer[read] = '\0';

    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "Critical message"));
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "Warning message"));
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "Info message"));
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "Debug message"));

    logging_shutdown();
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_ring_buffer_basic);
    RUN_TEST(test_logging_basic);
    RUN_TEST(test_log_macros);

    return UNITY_END();
}
