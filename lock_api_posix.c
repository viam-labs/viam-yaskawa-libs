#include "lock_api.h"
#include <stdlib.h>

LOCK_ID lock_create(void) {
    sem_t *sem = (sem_t *) malloc(sizeof(sem_t));
    if (sem == NULL)
        return NULL;
    if (sem_init(sem, 0, 1) != 0) {
        free(sem);
        return NULL;
    }
    return sem;
}

int lock_take(LOCK_ID id) {
    if (id == NULL)
        return LOCK_ERROR;
    return (sem_wait(id) == 0) ? LOCK_SUCCESS : LOCK_ERROR;
}

int lock_give(LOCK_ID id) {
    if (id == NULL)
        return LOCK_ERROR;
    return (sem_post(id) == 0) ? LOCK_SUCCESS : LOCK_ERROR;
}

int lock_destroy(LOCK_ID id) {
    if (id == NULL)
        return LOCK_ERROR;
    sem_destroy(id);
    free(id);
    return LOCK_SUCCESS;
}
