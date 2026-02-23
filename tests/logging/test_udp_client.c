#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define UDP_BROADCAST_DEFAULT_PORT 21789
#define BUFFER_SIZE 1024
int main(int argc, char *argv[]) {
    int port = UDP_BROADCAST_DEFAULT_PORT;
    if (argc > 1) {
        port = atoi(argv[1]);
        if (port <= 0) {
            printf("Invalid port number\n");
            return 1;
        }
    }
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return 1;
    }
    int broadcast_enable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        perror("setsockopt SO_BROADCAST");
        close(sockfd);
        return 1;
    }
    int reuse_addr = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)) < 0) {
        perror("setsockopt SO_REUSEADDR");
        close(sockfd);
        return 1;
    }
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port);
    if (bind(sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        perror("bind");
        close(sockfd);
        return 1;
    }
    printf("UDP broadcast client listening on port %d\n", port);
    printf("Press Ctrl+C to stop...\n");
    char buffer[BUFFER_SIZE];
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    while (1) {
        ssize_t bytes_received =
            recvfrom(sockfd, buffer, BUFFER_SIZE - 1, 0, (struct sockaddr *) &client_addr, &client_len);
        if (bytes_received > 0) {
            buffer[bytes_received] = '\0';
            printf("[%s:%d] %s", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port), buffer);
        }
    }
    close(sockfd);
    return 0;
}
