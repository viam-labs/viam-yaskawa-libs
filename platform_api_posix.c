#include "platform_api.h"

int platform_select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout) {
    return select(nfds, readfds, writefds, exceptfds, timeout);
}

int platform_recv(int sockfd, char *buf, int len, int flags) {
    return recv(sockfd, buf, len, flags);
}

int platform_send(int sockfd, const char *buf, int len, int flags) {
    return send(sockfd, buf, len, flags);
}

int platform_sendto(int sockfd, char *buf, int len, int flags, struct sockaddr *dest_addr, int addrlen) {
    return sendto(sockfd, buf, len, flags, dest_addr, addrlen);
}

int platform_socket(int domain, int type, int protocol) {
    return socket(domain, type, protocol);
}

int platform_setsockopt(int sockfd, int level, int optname, const void *optval, int optlen) {
    return setsockopt(sockfd, level, optname, optval, optlen);
}

int platform_bind(int sockfd, struct sockaddr *addr, int addrlen) {
    return bind(sockfd, addr, addrlen);
}

int platform_listen(int sockfd, int backlog) {
    return listen(sockfd, backlog);
}

int platform_accept(int sockfd, struct sockaddr *addr, SOCK_LEN_TYPE *addrlen) {
    return accept(sockfd, addr, addrlen);
}

int platform_close(int fd) {
    return close(fd);
}

void platform_shutdown(int sockfd, int how) {
    shutdown(sockfd, how);
}

uint16_t platform_htons(uint16_t hostshort) {
    return htons(hostshort);
}

uint16_t platform_ntohs(uint16_t netshort) {
    return ntohs(netshort);
}

int platform_pthread_create(THREAD_TYPE *thread, void *attr, void *(*start_routine)(void *), void *arg) {
    return pthread_create(thread, attr, start_routine, arg);
}

int platform_pthread_join(THREAD_TYPE thread, void **retval) {
    return pthread_join(thread, retval);
}

void platform_usleep(unsigned int microseconds) {
    usleep(microseconds);
}

void *platform_malloc(int len) {
    return malloc(len);
}

void platform_free(void *ptr) {
    free(ptr);
}
