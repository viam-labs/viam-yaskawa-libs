#ifndef PLATFORM_API_H
#define PLATFORM_API_H

/* Platform abstraction — types and declarations only.
   POSIX: link platform_api_posix.c
   VxWorks: force-include vxworks_compat.h to pre-define PLATFORM_TYPES_DEFINED */

#include <stdint.h>

#ifndef PLATFORM_TYPES_DEFINED
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

typedef pthread_t THREAD_TYPE;
typedef socklen_t SOCK_LEN_TYPE;
#define CLOCK_TYPE CLOCK_MONOTONIC
#endif /* PLATFORM_TYPES_DEFINED */

/* Socket functions */
int platform_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout);
int platform_recv(int sockfd, char *buf, int len, int flags);
int platform_send(int sockfd, const char *buf, int len, int flags);
int platform_sendto(int sockfd, char *buf, int len, int flags, struct sockaddr *dest_addr, int addrlen);
int platform_socket(int domain, int type, int protocol);
int platform_setsockopt(int sockfd, int level, int optname, const void *optval, int optlen);
int platform_bind(int sockfd, struct sockaddr *addr, int addrlen);
int platform_listen(int sockfd, int backlog);
int platform_accept(int sockfd, struct sockaddr *addr, SOCK_LEN_TYPE *addrlen);
int platform_close(int fd);
void platform_shutdown(int sockfd, int how);
uint16_t platform_htons(uint16_t hostshort);
uint16_t platform_ntohs(uint16_t netshort);

/* Threading */
int platform_pthread_create(THREAD_TYPE *thread, void *attr, void *(*start_routine)(void *), void *arg);
int platform_pthread_join(THREAD_TYPE thread, void **retval);
void platform_usleep(unsigned int microseconds);

/* Memory */
void *platform_malloc(int len);
void platform_free(void *ptr);

#endif /* PLATFORM_API_H */
