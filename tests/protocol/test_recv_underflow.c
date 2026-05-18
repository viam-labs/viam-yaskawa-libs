/*
 * Regression test for task #6: infinite recv-loop / payload_received_bytes
 * underflow in tcp_thread_func.
 *
 * The buggy payload-read loop in robot_protocol.c looked like:
 *
 *     while (payload_received_bytes != header.payload_length) {
 *         int ret_recv = platform_recv(sock,
 *                                      (char *) payload + payload_received_bytes,
 *                                      header.payload_length - payload_received_bytes,
 *                                      MSG_WAITALL);
 *         if (ret_recv <= 0) {
 *             pr_error("Failed to read entire payload ...");
 *             // NO break — falls through!
 *         }
 *         payload_received_bytes += ret_recv;  // unguarded; ret_recv == -1 wraps uint32
 *     }
 *
 * If platform_recv returns -1 (e.g. ECONNRESET), the uint32_t
 * payload_received_bytes is incremented by -1 and wraps to 0xFFFFFFFF.
 * The next recv is then called with
 *     buf = original_payload + 0xFFFFFFFF
 *     len = payload_length - 0xFFFFFFFF
 * and the != loop condition never holds, so the thread spins forever
 * smashing memory at wild addresses.
 *
 * This test uses -Wl,--wrap=platform_recv to substitute platform_recv at link
 * time. Header reads (MSG_DONTWAIT) are passed through to the real recv so the
 * server actually parses the header we send. Payload reads (MSG_WAITALL) are
 * intercepted: every call records (buf, len, flags) and returns -1 with
 * errno=ECONNRESET. After MAX_UNDERFLOW_CALLS the stub calls pthread_exit() to
 * break the (otherwise infinite) spin so the test can join and inspect state.
 *
 * Assertions encode the *correct* behavior, so this test FAILS while the bug
 * is present and PASSES once the fix is re-applied:
 *
 *   - Exactly one payload-path recv call should occur: the loop must break on
 *     the first ret_recv <= 0. Under the bug, the stub will be hit
 *     MAX_UNDERFLOW_CALLS times before pthread_exit breaks the spin.
 *   - The buf pointer offset between successive calls must stay <= the
 *     announced payload length. Under the bug, the second call's buf is
 *     offset by 0xFFFFFFFF (~4 GiB) — the uint32_t wrap of -1.
 *
 * Diagnostics are printed before the assertions so the actual observed
 * underflow value is visible in the test log on failure.
 */

#define _GNU_SOURCE
#include "logging.h"
#include "platform_api.h"
#include "protocol.h"
#include "ring_buffer.h"
#include "robot_protocol.h"
#include "unity.h"
#include <arpa/inet.h>
#include <errno.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <unity_config.h>

#define TEST_TCP_PORT 27754
#define TEST_UDP_PORT 27755
#define TEST_TIMEOUT_MS 5000

#define MAX_UNDERFLOW_CALLS 10
#define RECORDED_CALL_SLOTS 32

typedef struct {
    char *buf;
    int len;
    int flags;
} recv_call_record_t;

static recv_call_record_t g_recv_calls[RECORDED_CALL_SLOTS];
static volatile int g_payload_recv_count = 0;
static volatile int g_thread_exited = 0;

// Linker --wrap=platform_recv redirects every reference (including inside
// libcomms) to this function; __real_platform_recv is the original.
extern int __real_platform_recv(int sockfd, char *buf, int len, int flags);

int __wrap_platform_recv(int sockfd, char *buf, int len, int flags) {
    if (flags & MSG_WAITALL) {
        // Payload-read path in the buggy loop. Record and force a -1 return.
        int idx = g_payload_recv_count;
        if (idx < RECORDED_CALL_SLOTS) {
            g_recv_calls[idx].buf = buf;
            g_recv_calls[idx].len = len;
            g_recv_calls[idx].flags = flags;
        }
        g_payload_recv_count++;
        if (g_payload_recv_count >= MAX_UNDERFLOW_CALLS) {
            // Break the spin so the test can finish. The buggy loop never
            // exits on its own; without this the test would hang.
            g_thread_exited = 1;
            pthread_exit(NULL);
        }
        errno = ECONNRESET;
        return -1;
    }
    return __real_platform_recv(sockfd, buf, len, flags);
}

static robot_server_ctx_t *g_server = NULL;

static command_response_context_t *dummy_handle_command(protocol_header_t *h, void *p, void *u) {
    (void) h;
    (void) p;
    (void) u;
    return NULL;
}

void setUp(void) {
    signal(SIGPIPE, SIG_IGN);

    g_payload_recv_count = 0;
    g_thread_exited = 0;
    memset(g_recv_calls, 0, sizeof(g_recv_calls));

    robot_server_config_t config = {
        .tcp_port = TEST_TCP_PORT,
        .udp_port = TEST_UDP_PORT,
        .connection_timeout_ms = TEST_TIMEOUT_MS,
        .callbacks =
            {
                .on_connection = NULL,
                .on_disconnection = NULL,
                .handle_command = dummy_handle_command,
                .get_error_info = NULL,
            },
        .user_data = NULL,
    };
    g_server = robot_protocol_create(&config);
    TEST_ASSERT_NOT_NULL(g_server);
    TEST_ASSERT_EQUAL(0, robot_protocol_start(g_server));
    platform_usleep(100000);
}

void tearDown(void) {
    if (g_server) {
        robot_protocol_stop(g_server);
        robot_protocol_destroy(g_server);
        g_server = NULL;
    }
    platform_usleep(100000);
    // Final drain so any logs emitted during shutdown make it to stdout.
    log_to_stdout();
}

static int connect_client(void) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    TEST_ASSERT_TRUE(s >= 0);
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(TEST_TCP_PORT);
    inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    int rc = connect(s, (struct sockaddr *) &addr, sizeof(addr));
    TEST_ASSERT_EQUAL(0, rc);
    return s;
}

void test_recv_loop_underflows_on_minus_one(void) {
    int c = connect_client();

    // Let the server accept the connection.
    platform_usleep(50000);

    // Send a header announcing a 64-byte payload, then DO NOT send the payload.
    // The server's tcp_thread_func will read the header (pass-through), enter
    // the buggy payload-read loop, and our stub will return -1 for every call.
    protocol_header_t hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.magic_number = PROTOCOL_MAGIC_NUMBER;
    hdr.version = PROTOCOL_VERSION;
    hdr.message_type = (uint8_t) MSG_HEARTBEAT;
    hdr.timestamp_ms = 0;
    hdr.payload_length = 64;

    ssize_t sent = send(c, &hdr, sizeof(hdr), 0);
    TEST_ASSERT_EQUAL((ssize_t) sizeof(hdr), sent);

    // Wait up to ~2 seconds for the stub to hit MAX_UNDERFLOW_CALLS and
    // pthread_exit out of the spin. The CTest TIMEOUT also bounds the run.
    for (int i = 0; i < 200 && !g_thread_exited; i++) {
        platform_usleep(10000);
    }

    printf("\n[underflow] payload_recv_count=%d  thread_exited=%d\n", g_payload_recv_count, g_thread_exited);
    for (int i = 0; i < g_payload_recv_count && i < RECORDED_CALL_SLOTS; i++) {
        printf("[underflow] call[%d]  buf=%p  len=%d  flags=0x%x\n", i, (void *) g_recv_calls[i].buf,
               g_recv_calls[i].len, g_recv_calls[i].flags);
    }
    if (g_payload_recv_count >= 2) {
        intptr_t off = (intptr_t) g_recv_calls[1].buf - (intptr_t) g_recv_calls[0].buf;
        printf("[underflow] buf offset between call[0] and call[1] = 0x%" PRIxPTR " (= %" PRIdPTR
               " bytes; expected <= payload_length=%u)\n",
               (uintptr_t) off, off, (unsigned) hdr.payload_length);
        printf("[underflow] reconstructed payload_received_bytes after first -1 = 0x%" PRIxPTR
               " (uint32 wrap of -1 == 0xFFFFFFFF)\n",
               (uintptr_t) off);
    }

    // Correct behavior: the loop must break on the first ret_recv <= 0, so
    // exactly one payload-path recv call is made.
    TEST_ASSERT_EQUAL_MESSAGE(1, g_payload_recv_count,
                              "payload-read loop did NOT break on ret_recv <= 0 — it kept calling recv "
                              "after a failure. payload_received_bytes is incremented by ret_recv (== -1) "
                              "and underflows the uint32_t.");

    // Correct behavior: no underflow — payload_received_bytes (i.e. the buf
    // offset between successive calls) must never exceed the announced payload
    // length.
    if (g_payload_recv_count >= 2) {
        intptr_t offset = (intptr_t) g_recv_calls[1].buf - (intptr_t) g_recv_calls[0].buf;
        TEST_ASSERT_TRUE_MESSAGE((uintptr_t) offset <= (uintptr_t) hdr.payload_length,
                                 "payload_received_bytes underflowed: the second recv's buf pointer is "
                                 "offset far past the end of the allocated payload buffer.");
    }

    close(c);
}

// =============================================================================
// Logging plumbing — mirrors test_fault_injection.c / robot_server_example.c
// so pr_info / pr_error / pr_warn etc. from the server are visible in the test
// output. A background pump thread drains the ring buffer to stdout every
// 100 ms; setUp/tearDown also drain at key points so we don't lose the last
// entries if the test exits between ticks.
// =============================================================================

#define LOG_BUFFER_SIZE 8192

static char g_log_buffer[LOG_BUFFER_SIZE];
static ring_buffer_t g_rb;
static pthread_t g_log_pump_thread;
static volatile int g_log_pump_running = 0;

static void *log_pump_fn(void *arg) {
    (void) arg;
    while (g_log_pump_running) {
        log_to_stdout();
        platform_usleep(50000); // 50ms — tighter than the 100ms used elsewhere
                                // because our whole test runs in ~300ms.
    }
    // Final drain on shutdown.
    log_to_stdout();
    return NULL;
}

int main(void) {
    if (ring_buffer_initialize(&g_rb, g_log_buffer, LOG_BUFFER_SIZE) != 0) {
        fprintf(stderr, "Failed to initialize log ring buffer\n");
        return 1;
    }
    if (logging_initialize(&g_rb) != 0) {
        fprintf(stderr, "Failed to initialize logging\n");
        return 1;
    }
    logging_set_level(LOG_DEBUG); // capture everything; the bug log is pr_error

    g_log_pump_running = 1;
    if (pthread_create(&g_log_pump_thread, NULL, log_pump_fn, NULL) != 0) {
        fprintf(stderr, "Failed to create log pump thread\n");
        return 1;
    }

    UNITY_BEGIN();
    RUN_TEST(test_recv_loop_underflows_on_minus_one);
    int unity_rc = UNITY_END();

    // Stop the pump cleanly, then drain once more in case anything raced in
    // between the last tick and unity teardown.
    g_log_pump_running = 0;
    pthread_join(g_log_pump_thread, NULL);
    log_to_stdout();

    return unity_rc;
}
