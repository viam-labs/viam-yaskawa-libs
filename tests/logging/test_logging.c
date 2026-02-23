#define _DEFAULT_SOURCE
#include "logging.h"
#include "unity.h"
#include <pthread.h>
#include <string.h>
#include <unistd.h>

/* Test buffer and ring buffer */
static char log_buffer[RING_BUFFER_DEFAULT_SIZE];
static ring_buffer_t rb;

/* Thread data for logging tests */
typedef struct {
    int thread_id;
    int message_count;
} log_thread_data_t;

/* Consumer thread data */
typedef struct {
    ring_buffer_t *rb;
    int messages_consumed;
    int should_stop;
} log_consumer_data_t;

void setUp(void) {
    /* Initialize ring buffer and logging system */
    ring_buffer_initialize(&rb, log_buffer, sizeof(log_buffer));
    logging_initialize(&rb);
    logging_set_level(LOG_DEBUG); /* Allow all messages */
}

void tearDown(void) {
    /* Clean up */
    logging_shutdown();
    ring_buffer_destroy(&rb);
}

void test_logging_initialization(void) {
    ring_buffer_t test_rb;
    char buffer[1024];

    ring_buffer_initialize(&test_rb, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL(0, logging_initialize(&test_rb));
    TEST_ASSERT_EQUAL(LOG_DEBUG, logging_get_level());

    logging_shutdown();
    ring_buffer_destroy(&test_rb);
}

void test_log_level_filtering(void) {
    char read_buffer[256];
    size_t read;

    /* Set log level to WARNING */
    logging_set_level(LOG_WARNING);
    TEST_ASSERT_EQUAL(LOG_WARNING, logging_get_level());

    /* These should be logged (level <= WARNING) */
    TEST_ASSERT_NOT_EQUAL(0, pr_crit("Critical message"));
    TEST_ASSERT_NOT_EQUAL(0, pr_warn("Warning message"));

    /* These should be filtered out (level > WARNING) */
    TEST_ASSERT_EQUAL(0, pr_info("Info message"));
    TEST_ASSERT_EQUAL(0, pr_debug("Debug message"));

    /* Read messages from buffer */
    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer) - 1);
    read_buffer[read] = '\0';

    /* Should contain critical and warning messages */
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "Critical message"));
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "Warning message"));

    /* Should NOT contain info and debug messages */
    TEST_ASSERT_NULL(strstr(read_buffer, "Info message"));
    TEST_ASSERT_NULL(strstr(read_buffer, "Debug message"));
}

void test_log_message_format(void) {
    char read_buffer[512];
    size_t read;

    /* Write a test message */
    TEST_ASSERT_NOT_EQUAL(0, pr_info("Test message %d: %s", 42, "hello"));

    /* Read the message */
    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer) - 1);
    read_buffer[read] = '\0';

    /* Check timestamp format (YYYY-MM-DD HH:MM:SS) */
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "Test message 42: hello"));

    /* Check that timestamp is present (basic format check) */
    char *first_space = strchr(read_buffer, ' ');
    TEST_ASSERT_NOT_NULL(first_space);

    /* Should contain date format */
    TEST_ASSERT_NOT_NULL(strchr(read_buffer, '-')); /* Date separator */
    TEST_ASSERT_NOT_NULL(strchr(read_buffer, ':')); /* Time separator */
}

void test_log_macros(void) {
    char read_buffer[1024];
    size_t read;

    /* Use all log macros */
    pr_crit("Critical: %s", "system failure");
    pr_warn("Warning: %s", "low memory");
    pr_info("Info: %s", "system started");
    pr_debug("Debug: variable x = %d", 123);

    /* Read all messages */
    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer) - 1);
    read_buffer[read] = '\0';

    /* Verify all messages are present */
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "system failure"));
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "low memory"));
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "system started"));
    TEST_ASSERT_NOT_NULL(strstr(read_buffer, "variable x = 123"));
}

/* Producer thread for multithreaded logging test */
void *logging_producer_thread(void *arg) {
    log_thread_data_t *data = (log_thread_data_t *) arg;
    int i;

    for (i = 0; i < data->message_count; i++) {
        switch (i % 4) {
        case 0:
            pr_crit("Thread %d: Critical message %d", data->thread_id, i);
            break;
        case 1:
            pr_warn("Thread %d: Warning message %d", data->thread_id, i);
            break;
        case 2:
            pr_info("Thread %d: Info message %d", data->thread_id, i);
            break;
        case 3:
            pr_debug("Thread %d: Debug message %d", data->thread_id, i);
            break;
        }

        /* Small delay to allow interleaving */
        usleep(500);
    }

    return NULL;
}

/* Consumer thread that reads from ring buffer and writes to stdout */
void *logging_consumer_thread(void *arg) {
    log_consumer_data_t *data = (log_consumer_data_t *) arg;
    char read_buffer[512];

    while (!data->should_stop) {
        size_t read = ring_buffer_read_n(data->rb, read_buffer, sizeof(read_buffer) - 1);
        if (read > 0) {
            read_buffer[read] = '\0';
            printf("%s", read_buffer);
            fflush(stdout);
            data->messages_consumed++;
        } else {
            usleep(10000); /* Wait for data */
        }
    }

    /* Final read to get any remaining data */
    size_t read = ring_buffer_read_n(data->rb, read_buffer, sizeof(read_buffer) - 1);
    if (read > 0) {
        read_buffer[read] = '\0';
        printf("%s", read_buffer);
        fflush(stdout);
    }

    return NULL;
}

void test_multithreaded_logging(void) {
    const int num_producers = 2;
    /* const int messages_per_producer = 10; */ /* Unused variable */
    pthread_t producers[2], consumer;
    log_thread_data_t producer_data[2];
    log_consumer_data_t consumer_data = {&rb, 0, 0};
    int i;

    printf("\n=== Multithreaded Logging Test ===\n");

    /* Initialize producer data */
    for (i = 0; i < num_producers; i++) {
        producer_data[i].thread_id = i + 1;
        producer_data[i].message_count = 10;
    }

    /* Start consumer thread first */
    TEST_ASSERT_EQUAL(0, pthread_create(&consumer, NULL, logging_consumer_thread, &consumer_data));

    /* Start producer threads */
    for (i = 0; i < num_producers; i++) {
        TEST_ASSERT_EQUAL(0, pthread_create(&producers[i], NULL, logging_producer_thread, &producer_data[i]));
    }

    /* Wait for producers to complete */
    for (i = 0; i < num_producers; i++) {
        TEST_ASSERT_EQUAL(0, pthread_join(producers[i], NULL));
    }

    /* Give consumer time to read remaining messages */
    sleep(1);

    /* Signal consumer to stop */
    consumer_data.should_stop = 1;
    TEST_ASSERT_EQUAL(0, pthread_join(consumer, NULL));

    printf("\n=== Test completed: Consumer processed messages ===\n");
    TEST_ASSERT_GREATER_THAN(0, consumer_data.messages_consumed);
}

void test_buffer_overflow_handling(void) {
    int i;
    int messages_written = 0;
    int messages_failed = 0;

    /* Fill the buffer with many large messages */
    for (i = 0; i < 1000; i++) {
        int result = pr_info("Large message %d: This is a relatively long log message that will "
                             "help fill up the ring buffer quickly to test overflow handling",
                             i);
        if (result == 0) {
            messages_written++;
        } else {
            messages_failed++;
        }
    }

    /* Should have written some messages and failed some due to buffer being
     * full */
    TEST_ASSERT_GREATER_THAN(0, messages_written);
    printf("Messages written: %d, Messages failed: %d\n", messages_written, messages_failed);

    /* Buffer should be close to full */
    TEST_ASSERT_LESS_THAN(100, ring_buffer_free_space(&rb)); /* Less than 100 bytes free */
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_logging_initialization);
    RUN_TEST(test_log_level_filtering);
    RUN_TEST(test_log_message_format);
    RUN_TEST(test_log_macros);
    RUN_TEST(test_multithreaded_logging);
    RUN_TEST(test_buffer_overflow_handling);

    return UNITY_END();
}
