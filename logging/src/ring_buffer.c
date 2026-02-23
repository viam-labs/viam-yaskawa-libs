#include "../include/ring_buffer.h"
#include <stddef.h>
#include <string.h>

int ring_buffer_initialize(ring_buffer_t *rb, char *buffer, size_t size) {
    if (rb == NULL || buffer == NULL || size == 0) {
        return -1;
    }

    /* Reset buffer to 0 */
    memset(buffer, 0, size);

    /* Initialize ring buffer structure */
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;

    /* Create lock */
    rb->lock = lock_create();
    if (rb->lock == NULL) {
        return -1;
    }

    return 0;
}

size_t ring_buffer_free_space_internal(ring_buffer_t *rb) {
    size_t free_space;
    size_t head = rb->head;
    if (head >= rb->tail) {
        free_space = rb->size - (head - rb->tail) - 1;
    } else {
        free_space = rb->tail - head - 1;
    }
    return free_space;
}

size_t ring_buffer_free_space(ring_buffer_t *rb) {
    if (rb == NULL) {
        return 0;
    }

    size_t free_space;

    if (lock_take(rb->lock) != LOCK_SUCCESS) {
        return 0;
    }
    free_space = ring_buffer_free_space_internal(rb);

    lock_give(rb->lock);

    return free_space;
}

size_t ring_buffer_used_space_inner(ring_buffer_t *rb) {
    size_t used_space;
    if (rb->head >= rb->tail) {
        used_space = rb->head - rb->tail;
    } else {
        used_space = rb->size - rb->tail + rb->head;
    }
    return used_space;
}

size_t ring_buffer_used_space(ring_buffer_t *rb) {
    if (rb == NULL) {
        return 0;
    }

    size_t used_space;

    if (lock_take(rb->lock) != LOCK_SUCCESS) {
        return 0;
    }

    used_space = ring_buffer_used_space_inner(rb);

    lock_give(rb->lock);

    return used_space;
}

size_t ring_buffer_write_inner(ring_buffer_t *rb, const char *data, size_t size) {
    size_t first_chunk = size;
    if (rb->head + size > rb->size) {
        first_chunk = rb->size - rb->head;
    }

    memcpy(&rb->buffer[rb->head], data, first_chunk);

    if (first_chunk < size) {
        /* Wrap around */
        memcpy(&rb->buffer[0], &data[first_chunk], size - first_chunk);
    }

    /* Update head position */
    rb->head = (rb->head + size) % rb->size;

    return size;
}

size_t ring_buffer_write_exact_n(ring_buffer_t *rb, const char *data, size_t size) {
    if (rb == NULL || data == NULL || size == 0) {
        return 0;
    }

    if (lock_take(rb->lock) != LOCK_SUCCESS) {
        return 0;
    }
    size_t free_space = ring_buffer_free_space_internal(rb);

    size_t bytes_to_write = (size <= free_space) ? size : 0;

    if (bytes_to_write == 0) {
        lock_give(rb->lock);
        return 0;
    }
    size_t written = ring_buffer_write_inner(rb, data, bytes_to_write);
    lock_give(rb->lock);
    return written;
}

size_t ring_buffer_write_n(ring_buffer_t *rb, const char *data, size_t size) {
    if (rb == NULL || data == NULL || size == 0) {
        return 0;
    }

    if (lock_take(rb->lock) != LOCK_SUCCESS) {
        return 0;
    }

    size_t free_space = ring_buffer_free_space_internal(rb);

    size_t bytes_to_write = (size <= free_space) ? size : free_space;

    if (bytes_to_write == 0) {
        lock_give(rb->lock);
        return 0;
    }
    size_t written = ring_buffer_write_inner(rb, data, bytes_to_write);
    lock_give(rb->lock);
    return written;
}

size_t ring_buffer_read_n(ring_buffer_t *rb, char *data, size_t size) {
    if (rb == NULL || data == NULL || size == 0) {
        return 0;
    }

    /* if (lock_take(rb->lock) != LOCK_SUCCESS) */
    /* { */
    /*     return 0; */
    /* } */

    size_t used_space = ring_buffer_used_space_inner(rb);

    size_t bytes_to_read = (size <= used_space) ? size : used_space;

    if (bytes_to_read == 0) {
        lock_give(rb->lock);
        return 0; /* Buffer empty */
    }

    /* Read data in two parts if wrapping around */
    size_t first_chunk = bytes_to_read;
    if (rb->tail + bytes_to_read > rb->size) {
        first_chunk = rb->size - rb->tail;
    }

    memcpy(data, &rb->buffer[rb->tail], first_chunk);

    if (first_chunk < bytes_to_read) {
        /* Wrap around */
        memcpy(&data[first_chunk], &rb->buffer[0], bytes_to_read - first_chunk);
    }

    /* Update tail position */
    rb->tail = (rb->tail + bytes_to_read) % rb->size;

    /* lock_give(rb->lock); */

    return bytes_to_read;
}

int ring_buffer_destroy(ring_buffer_t *rb) {
    if (rb == NULL) {
        return -1;
    }

    /* Reset buffer if provided */
    if (rb->buffer != NULL && rb->size > 0) {
        memset(rb->buffer, 0, rb->size);
    }

    /* Destroy lock */
    int result = 0;
    if (rb->lock != NULL) {
        result = lock_destroy(rb->lock);
    }

    /* Reset structure */
    rb->buffer = NULL;
    rb->size = 0;
    rb->head = 0;
    rb->tail = 0;
    rb->lock = NULL;

    return result;
}
