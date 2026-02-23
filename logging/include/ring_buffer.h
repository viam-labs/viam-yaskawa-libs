#ifndef RING_BUFFER_H
#define RING_BUFFER_H
#include "lock_api.h"
#include <stddef.h>

#define RING_BUFFER_DEFAULT_SIZE 8192

/* Ring buffer structure */
typedef struct {
    char *buffer; /* User-provided buffer */
    size_t size;  /* Size of the buffer */
    size_t head;  /* Current write position */
    size_t tail;  /* Current read position */
    LOCK_ID lock; /* Lock for thread safety */
} ring_buffer_t;

/* Ring buffer API functions */

/*
 * Initialize a ring buffer
 * @param rb: Ring buffer object to initialize
 * @param buffer: User-provided buffer (must be statically allocated)
 * @param size: Size of the buffer
 * @return: 0 on success, -1 on error
 */
int ring_buffer_initialize(ring_buffer_t *rb, char *buffer, size_t size);

/*
 * Get free space in the ring buffer
 * @param rb: Ring buffer object
 * @return: Number of bytes that can be written
 */
size_t ring_buffer_free_space(ring_buffer_t *rb);

/*
 * Get used space in the ring buffer
 * @param rb: Ring buffer object
 * @return: Number of bytes that can be read
 */
size_t ring_buffer_used_space(ring_buffer_t *rb);

/*
 * Write data to the ring buffer
 * @param rb: Ring buffer object
 * @param data: Buffer containing data to write
 * @param size: Number of bytes to write
 * @return: Number of bytes that were NOT written (0 on complete success)
 */
size_t ring_buffer_write_n(ring_buffer_t *rb, const char *data, size_t size);

size_t ring_buffer_write_exact_n(ring_buffer_t *rb, const char *data, size_t size);
/*
 * Read data from the ring buffer
 * @param rb: Ring buffer object
 * @param data: Buffer to store read data
 * @param size: Maximum number of bytes to read
 * @return: Number of bytes actually read
 */
size_t ring_buffer_read_n(ring_buffer_t *rb, char *data, size_t size);

/*
 * Destroy a ring buffer and cleanup resources
 * @param rb: Ring buffer object to destroy
 * @return: 0 on success, -1 on error
 */
int ring_buffer_destroy(ring_buffer_t *rb);

#endif /* RING_BUFFER_H */
