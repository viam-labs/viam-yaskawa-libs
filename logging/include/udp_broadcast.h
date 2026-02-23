#ifndef UDP_BROADCAST_H
#define UDP_BROADCAST_H

#ifndef UDP_BROADCAST_TYPES_DEFINED
#include <netinet/in.h>
#define UDP_BROADCAST_MAX_INTERFACES 16
#endif

#include "../include/ring_buffer.h"

#define UDP_BROADCAST_DEFAULT_PORT 21789

#define UDP_BROADCAST_BUFFER_SIZE 1024
#define MAX_NETWORK_PORTS 2
typedef struct {
    struct sockaddr_in dest_addr;
    int socket_fd;
    char interface_name[16];
} udp_interface_t;
typedef struct {
    udp_interface_t interfaces[UDP_BROADCAST_MAX_INTERFACES];
    int interface_count;
    int port;
    ring_buffer_t *ring_buffer;
} udp_broadcast_config_t;
/* Platform-specific: gather broadcast interfaces.
   POSIX: implemented in udp_broadcast.c
   VxWorks: consumer links udp_broadcast_yrc1000.c (or equivalent) */
int udp_broadcast_gather_interfaces(udp_broadcast_config_t *config, const char *specific_interface);

/* Platform-specific: create a UDP broadcast socket.
   POSIX: implemented in udp_broadcast.c
   VxWorks: consumer links udp_broadcast_yrc1000.c (or equivalent) */
int udp_broadcast_create_socket(void);

int udp_broadcast_initialize(udp_broadcast_config_t *config, ring_buffer_t *rb, int port,
                             const char *specific_interface);
int process_log_message(udp_broadcast_config_t *config);
void udp_broadcast_shutdown(udp_broadcast_config_t *config);
#endif /* UDP_BROADCAST_H */
