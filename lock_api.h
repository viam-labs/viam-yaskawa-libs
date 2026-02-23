#ifndef LOCK_API_H
#define LOCK_API_H

/* Lock abstraction — types and declarations only.
   POSIX: link lock_api_posix.c
   VxWorks: force-include vxworks_compat.h to pre-define LOCK_TYPES_DEFINED */

#ifndef LOCK_TYPES_DEFINED
#include <semaphore.h>
typedef sem_t *LOCK_ID;
#define LOCK_SUCCESS 0
#define LOCK_ERROR -1
#endif

LOCK_ID lock_create(void);
int lock_take(LOCK_ID id);
int lock_give(LOCK_ID id);
int lock_destroy(LOCK_ID id);

#endif /* LOCK_API_H */
