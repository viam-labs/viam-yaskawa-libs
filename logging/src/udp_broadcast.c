#include "../include/udp_broadcast.h"
#include "../include/logging.h"
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

int udp_broadcast_create_socket(void) {
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        return -1;
    }
    int broadcast_enable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        close(sockfd);
        return -1;
    }
    return sockfd;
}

int udp_broadcast_gather_interfaces(udp_broadcast_config_t *config, const char *specific_interface) {
    struct ifaddrs *ifaddrs_ptr, *ifa;
    config->interface_count = 0;
    if (getifaddrs(&ifaddrs_ptr) == -1) {
        printf("UDP broadcast: Failed to get interface addresses\n");
        return -1;
    }
    for (ifa = ifaddrs_ptr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL)
            continue;
        printf("processing %p internet ifa %s fam %x  flags %x fam %s \r\n", ifa, ifa->ifa_name,
               ifa->ifa_addr->sa_family, ifa->ifa_flags, ifa->ifa_addr->sa_data);
        if (ifa->ifa_addr->sa_family == AF_INET) {
            if (!(ifa->ifa_flags & 0x2))
                continue; // IFF_BROADCAST
            if (!(ifa->ifa_flags & 0x1))
                continue; // IFF_UP
            if (ifa->ifa_flags & 0x8)
                continue; // IFF_LOOPBACK
            if (specific_interface && strcmp(ifa->ifa_name, specific_interface) != 0) {
                continue;
            }

            struct sockaddr_in *broadcast_addr = (struct sockaddr_in *) ifa->ifa_broadaddr;
            if (broadcast_addr == NULL)
                continue;
            udp_interface_t *interface = &config->interfaces[config->interface_count];
            interface->socket_fd = udp_broadcast_create_socket();
            if (interface->socket_fd < 0) {
                printf("UDP broadcast: Failed to create socket for interface %s\n", ifa->ifa_name);
                continue;
            }
            interface->dest_addr = *broadcast_addr;
            interface->dest_addr.sin_port = htons(config->port);
            strncpy(interface->interface_name, ifa->ifa_name, sizeof(interface->interface_name) - 1);
            interface->interface_name[sizeof(interface->interface_name) - 1] = '\0';
            config->interface_count++;
        }
    }
    freeifaddrs(ifaddrs_ptr);
    if (specific_interface && config->interface_count == 0) {
        printf("UDP broadcast: Specified interface '%s' not found or not "
               "suitable\n",
               specific_interface);
        return -1;
    }
    if (config->interface_count == 0) {
        printf("UDP broadcast: No suitable broadcast interfaces found\n");
        return -1;
    }
    printf("UDP broadcast: Configured %d interface(s)\n", config->interface_count);
    for (int i = 0; i < config->interface_count; i++) {
        printf("  Interface: %s, Broadcast to: %s:%d\n", config->interfaces[i].interface_name,
               inet_ntoa(config->interfaces[i].dest_addr.sin_addr), ntohs(config->interfaces[i].dest_addr.sin_port));
    }
    return 0;
}

int udp_broadcast_initialize(udp_broadcast_config_t *config, ring_buffer_t *rb, int port,
                             const char *specific_interface) {
    if (config == NULL || rb == NULL) {
        return -1;
    }
    memset(config, 0, sizeof(udp_broadcast_config_t));
    config->ring_buffer = rb;
    config->port = (port > 0) ? port : UDP_BROADCAST_DEFAULT_PORT;
    return udp_broadcast_gather_interfaces(config, specific_interface);
}

int process_log_message(udp_broadcast_config_t *config) {
    if (config == NULL || config->ring_buffer == NULL) {
        return -1;
    }
    char buffer[UDP_BROADCAST_BUFFER_SIZE];
    size_t bytes_read = ring_buffer_read_n(config->ring_buffer, buffer, UDP_BROADCAST_BUFFER_SIZE - 1);
    if (bytes_read == 0) {
        return 0;
    }
    buffer[bytes_read] = '\0';
    size_t message_len = strlen(buffer);
    if (message_len == 0) {
        return 0;
    }
    puts(buffer);
    for (int i = 0; i < config->interface_count; i++) {
        udp_interface_t *interface = &config->interfaces[i];
        int32_t sent = sendto(interface->socket_fd, buffer, message_len, 0, (struct sockaddr *) &interface->dest_addr,
                              sizeof(interface->dest_addr));
        if (sent < 0) {
            pr_error("UDP broadcast: Failed to send on interface %s", interface->interface_name);
        }
    }
    return bytes_read;
}

void udp_broadcast_shutdown(udp_broadcast_config_t *config) {
    if (config == NULL) {
        return;
    }
    for (int i = 0; i < config->interface_count; i++) {
        if (config->interfaces[i].socket_fd >= 0) {
            close(config->interfaces[i].socket_fd);
            config->interfaces[i].socket_fd = -1;
        }
    }
    config->interface_count = 0;
    config->ring_buffer = NULL;
}
