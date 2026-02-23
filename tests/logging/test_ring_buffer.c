#define _DEFAULT_SOURCE
#include "ring_buffer.h"
#include "unity.h"
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

/* Test buffer */
static char test_buffer[RING_BUFFER_DEFAULT_SIZE];
static ring_buffer_t rb;

/* Thread test data */
typedef struct {
    ring_buffer_t *rb;
    int thread_id;
    int iterations;
    int bytes_written;
    int bytes_read;
    int usleep;
    volatile sig_atomic_t *done;
} thread_data_t;

typedef struct {
    uint32_t header;
    uint8_t tid;
    uint32_t cnt;
} data_test_n;

void setUp(void) {
    /* Initialize ring buffer before each test */
    ring_buffer_initialize(&rb, test_buffer, sizeof(test_buffer));
}

void tearDown(void) {
    /* Clean up after each test */
    ring_buffer_destroy(&rb);
}

/* Basic functionality tests */
void test_ring_buffer_initialize(void) {
    ring_buffer_t test_rb;
    char buffer[1024];

    TEST_ASSERT_EQUAL(0, ring_buffer_initialize(&test_rb, buffer, sizeof(buffer)));
    TEST_ASSERT_EQUAL_PTR(buffer, test_rb.buffer);
    TEST_ASSERT_EQUAL(sizeof(buffer), test_rb.size);
    TEST_ASSERT_EQUAL(0, ring_buffer_used_space(&test_rb));

    ring_buffer_destroy(&test_rb);
}

void test_ring_buffer_write_read(void) {
    const char *test_data = "Hello, World!";
    char read_buffer[64];
    size_t written, read;

    written = ring_buffer_write_n(&rb, test_data, strlen(test_data));
    TEST_ASSERT_EQUAL(strlen(test_data), written); /* All bytes should be written */

    TEST_ASSERT_EQUAL(strlen(test_data), ring_buffer_used_space(&rb));
    TEST_ASSERT_EQUAL(sizeof(test_buffer) - strlen(test_data) - 1, ring_buffer_free_space(&rb));

    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer));
    TEST_ASSERT_EQUAL(strlen(test_data), read);
    TEST_ASSERT_EQUAL_STRING_LEN(test_data, read_buffer, strlen(test_data));

    TEST_ASSERT_EQUAL(0, ring_buffer_used_space(&rb));
    TEST_ASSERT_EQUAL(sizeof(test_buffer) - 1, ring_buffer_free_space(&rb));
}

void test_ring_buffer_wrap_around(void) {
    char write_data[100];
    char read_data[50];
    size_t i, written, read;

    /* Fill with pattern */
    for (i = 0; i < sizeof(write_data); i++) {
        write_data[i] = 'A' + (i % 26);
    }

    /* Fill the buffer multiple times to test wrap-around */
    for (i = 0; i < 100; i++) {
        written = ring_buffer_write_n(&rb, write_data, sizeof(write_data));
        if (written < sizeof(write_data)) {
            /* Buffer is full, read some data */
            read = ring_buffer_read_n(&rb, read_data, sizeof(read_data));
            TEST_ASSERT_GREATER_THAN(0, read);
        }
    }
}

void test_partial_write_read(void) {
    char large_data[RING_BUFFER_DEFAULT_SIZE + 1000];
    char read_buffer[100];
    size_t written, read;

    /* Fill with test pattern */
    memset(large_data, 'X', sizeof(large_data));

    /* Try to write more than buffer size */
    written = ring_buffer_write_n(&rb, large_data, sizeof(large_data));
    TEST_ASSERT_GREATER_THAN(0, written); /* Some bytes should not be written */

    /* Buffer should be full */
    TEST_ASSERT_EQUAL(0, ring_buffer_free_space(&rb));
    TEST_ASSERT_EQUAL(sizeof(test_buffer) - 1, ring_buffer_used_space(&rb));

    /* Try to read more than available */
    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer));
    TEST_ASSERT_EQUAL(sizeof(read_buffer), read);

    /* Try to read more than what's left */
    size_t remaining = ring_buffer_used_space(&rb);
    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer));
    TEST_ASSERT_EQUAL((remaining < sizeof(read_buffer)) ? remaining : sizeof(read_buffer), read);
}

void test_empty_buffer_read(void) {
    char read_buffer[64];
    size_t read;

    /* Try to read from empty buffer */
    read = ring_buffer_read_n(&rb, read_buffer, sizeof(read_buffer));
    TEST_ASSERT_EQUAL(0, read);

    /* Buffer should still be empty */
    TEST_ASSERT_EQUAL(0, ring_buffer_used_space(&rb));
    TEST_ASSERT_EQUAL(sizeof(test_buffer) - 1, ring_buffer_free_space(&rb));
}

void *producer_thread2(void *arg) {
    thread_data_t *data = (thread_data_t *) arg;
    for (int i = 0; i < data->iterations; i++) {
        data_test_n obj = {0xAAAAAAAA, data->thread_id, i + 1};
        size_t written = 0;
        do {
            written = ring_buffer_write_exact_n(data->rb, (const char *) (&obj), sizeof(obj));
            data->bytes_written += written;
            usleep(data->usleep);
            if (written == 0) {
                data->bytes_read++;
            }
        } while (written == 0);
    }
    *data->done = (*data->done) | (1 << data->thread_id);
    return NULL;
}

void test_multithreaded_access2(void) {
    const int workers = 30;
    volatile sig_atomic_t sync = 0;
    pthread_t hnds[workers];
    int thread_cnts[workers];
    thread_data_t th_data[workers];
    int bits = (1 << (workers)) - 1;

    for (int i = 0; i < workers; i++) {
        bzero(&th_data[i], sizeof(thread_data_t));
        th_data[i].rb = &rb;
        th_data[i].iterations = 30000;
        th_data[i].usleep = 0;
        th_data[i].done = &sync;
        th_data[i].thread_id = i;
        thread_cnts[i] = 0;
    }

    for (int i = 0; i < workers; ++i) {
        TEST_ASSERT_EQUAL(0, pthread_create(&hnds[i], NULL, producer_thread2, &th_data[i]));
    }

    char buf[1024];
    bzero(buf, sizeof(buf));
    int needle = 0xAAAAAAAA;
    int remaining = 0;
    do {
        bzero(buf + remaining, sizeof(buf) - remaining);
        int read = ring_buffer_read_n(&rb, buf + remaining, sizeof(buf) - remaining) + remaining;
        char *start = buf;
        char *end = memmem(start + sizeof(needle), read - (start - buf), &needle, sizeof(needle));
        while (end != NULL) {
            data_test_n *obj = (data_test_n *) (start);
            TEST_ASSERT_EQUAL_HEX(needle, obj->header);
            TEST_ASSERT_LESS_THAN(sizeof(thread_cnts) / sizeof(int), obj->tid);
            TEST_ASSERT_GREATER_THAN(thread_cnts[obj->tid], obj->cnt);
            TEST_ASSERT_EQUAL(1, obj->cnt - thread_cnts[obj->tid]);
            thread_cnts[obj->tid] = obj->cnt;
            start = end;
            end = memmem(start + sizeof(needle), read - (start - buf), &needle, sizeof(needle));
        }
        memmove(buf, start, read - (start - buf));
        remaining = read - (start - buf);
        usleep(800);
    } while (sync != bits);

    for (int i = 0; i < workers; ++i) {
        TEST_ASSERT_GREATER_THAN(0, th_data[i].bytes_written);
        TEST_ASSERT_GREATER_THAN(0, th_data[i].bytes_read);
    }
}

/* Stress test */
void *stress_producer(void *arg) {
    thread_data_t *data = (thread_data_t *) arg;
    char message[32];
    int i;

    for (i = 0; i < data->iterations; i++) {
        snprintf(message, sizeof(message), "T%d_M%d ", data->thread_id, i);
        ring_buffer_write_n(data->rb, message, strlen(message));
    }

    return NULL;
}

void *stress_consumer(void *arg) {
    thread_data_t *data = (thread_data_t *) arg;
    char read_buffer[1024];
    int total_reads = 0;
    int empty_reads = 0;
    int max_empty_reads = 1000; /* Maximum consecutive empty reads before giving up */

    while (total_reads < data->iterations && empty_reads < max_empty_reads) {
        size_t read = ring_buffer_read_n(data->rb, read_buffer, sizeof(read_buffer));
        if (read > 0) {
            total_reads++;
            empty_reads = 0; /* Reset empty read counter */
        } else {
            empty_reads++;
            usleep(100);
        }
    }

    return NULL;
}

void test_stress_test(void) {
    const int num_producers = 3;
    const int iterations = 100;
    pthread_t producers[3], consumer;
    thread_data_t producer_data[3];
    thread_data_t consumer_data = {&rb, 0, iterations * num_producers / 10, 0, 0, 0, 0};
    int i;

    /* Initialize producer data */
    for (i = 0; i < num_producers; i++) {
        producer_data[i].rb = &rb;
        producer_data[i].thread_id = i;
        producer_data[i].iterations = iterations;
    }

    /* Start threads */
    for (i = 0; i < num_producers; i++) {
        TEST_ASSERT_EQUAL(0, pthread_create(&producers[i], NULL, stress_producer, &producer_data[i]));
    }
    TEST_ASSERT_EQUAL(0, pthread_create(&consumer, NULL, stress_consumer, &consumer_data));

    /* Wait for completion */
    for (i = 0; i < num_producers; i++) {
        TEST_ASSERT_EQUAL(0, pthread_join(producers[i], NULL));
    }
    TEST_ASSERT_EQUAL(0, pthread_join(consumer, NULL));

    printf("Stress test completed successfully\n");
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_ring_buffer_initialize);
    RUN_TEST(test_ring_buffer_write_read);
    RUN_TEST(test_ring_buffer_wrap_around);
    RUN_TEST(test_partial_write_read);
    RUN_TEST(test_empty_buffer_read);
    RUN_TEST(test_multithreaded_access2);
    RUN_TEST(test_stress_test);

    return UNITY_END();
}
