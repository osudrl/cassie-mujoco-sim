/*
 * Copyright (c) 2018 Agility Robotics
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifdef _WIN32

// Winsock compatibility
#define _WIN32_WINNT 0x0601
#include <winsock2.h>

#define poll WSAPoll
#define close closesocket
#define ioctl ioctlsocket
#define perror(str) fprintf(stderr, str ": %d\n", WSAGetLastError())
#define SOCKETS_INIT                                    \
    do {                                                \
        WSADATA wsaData;                                \
        int res = WSAStartup(MAKEWORD(2, 2), &wsaData); \
        if (res) {                                      \
            printf("WSAStartup failed: %d\n", res);     \
            return -1;                                  \
        }                                               \
    } while (0)
#define SOCKETS_CLEANUP WSACleanup()
typedef u_long ioctl_arg_t;

#else

// Linux sockets
#include <unistd.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <netdb.h>
#include <fcntl.h>

#define SOCKETS_INIT
#define SOCKETS_CLEANUP
typedef int ioctl_arg_t;

#endif // _WIN32

#include "udp.h"
#include <stdio.h>


void process_packet_header(packet_header_info_t *info,
                           const unsigned char *header_in,
                           unsigned char *header_out)
{
    // Increment outgoing packet sequence number
    ++info->seq_num_out;

    // header_in[0]: sequence number of incoming packet
    // header_in[1]: sequence number of previous outgoing packet, looped back
    char seq_num_in = (char) header_in[0];
    char loopback = (char) header_in[1];

    // Compute round-trip delay and incoming sequence number diff
    info->delay = info->seq_num_out - loopback;
    info->seq_num_in_diff = seq_num_in - info->seq_num_in_last;
    info->seq_num_in_last = seq_num_in;

    // Write outgoing packet header
    header_out[0] = (unsigned char) info->seq_num_out;
    header_out[1] = (unsigned char) seq_num_in;
}


int udp_init_host(const char *local_addr_str, const char *local_port_str)
{
    // Platform-specific socket library initialization
    SOCKETS_INIT;

    int err;

    // Get address info
    struct addrinfo *local;
    struct addrinfo hints = {0};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    err = getaddrinfo(local_addr_str, local_port_str, &hints, &local);
    if (err) {
        printf("%s\n", gai_strerror(err));
        SOCKETS_CLEANUP;
        return -1;
    }

    // Create socket
    int sock = socket(local->ai_family, local->ai_socktype, local->ai_protocol);
    if (-1 == sock) {
        perror("Error creating socket");
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Bind to interface address
    if (bind(sock, (struct sockaddr *) local->ai_addr,
             local->ai_addrlen)) {
        perror("Error binding to interface address");
        close(sock);
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Free addrinfo struct
    freeaddrinfo(local);

    // Make socket non-blocking
    ioctl_arg_t mode = 1;
    ioctl(sock, FIONBIO, &mode);

    return sock;
}


int udp_init_client(const char *remote_addr_str, const char *remote_port_str,
                    const char *local_addr_str, const char *local_port_str)
{
    // Platform-specific socket library initialization
    SOCKETS_INIT;

    int err;

    // Get remote address info
    struct addrinfo *remote;
    struct addrinfo hints = {0};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    err = getaddrinfo(remote_addr_str, remote_port_str, &hints, &remote);
    if (err) {
        printf("%s\n", gai_strerror(err));
        SOCKETS_CLEANUP;
        return -1;
    }

    // Get remote address info
    struct addrinfo *local;
    err = getaddrinfo(local_addr_str, local_port_str, &hints, &local);
    if (err) {
        printf("%s\n", gai_strerror(err));
        SOCKETS_CLEANUP;
        return -1;
    }

    // Create socket
    int sock = socket(remote->ai_family, remote->ai_socktype,
                      remote->ai_protocol);
    if (-1 == sock) {
        perror("Error creating socket");
        freeaddrinfo(remote);
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Bind to interface address
    if (bind(sock, (struct sockaddr *) local->ai_addr, local->ai_addrlen)) {
        perror("Error binding to interface address");
        close(sock);
        freeaddrinfo(remote);
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Connect to remote address
    if (connect(sock, (struct sockaddr *) remote->ai_addr,
                remote->ai_addrlen)) {
        perror("Error connecting to remote address");
        close(sock);
        freeaddrinfo(remote);
        freeaddrinfo(local);
        SOCKETS_CLEANUP;
        return -1;
    }

    // Free addrinfo structs
    freeaddrinfo(remote);
    freeaddrinfo(local);

    // Make socket non-blocking
    ioctl_arg_t mode = 1;
    ioctl(sock, FIONBIO, &mode);

    return sock;
}


void udp_close(int sock)
{
    close(sock);
    SOCKETS_CLEANUP;
}


ssize_t get_newest_packet(int sock, void *recvbuf, size_t recvlen,
                          struct sockaddr *src_addr, socklen_t *addrlen)
{
    // Does not use sequence number for determining newest packet
    ssize_t nbytes = -1;
    struct pollfd fd = {.fd = sock, .events = POLLIN, .revents = 0};

    // Loop through RX buffer, copying data if packet is correct size
    while (poll(&fd, 1, 0)) {
        ioctl_arg_t nbytes_avail;
        ioctl(sock, FIONREAD, &nbytes_avail);
        if (recvlen == (size_t) nbytes_avail)
            nbytes = recvfrom(sock, recvbuf, recvlen, 0, src_addr, addrlen);
        else
            recv(sock, recvbuf, 0, 0); // Discard packet
    }

    // Return the copied packet size, or -1 if no data was copied
    return nbytes;
}


ssize_t wait_for_packet(int sock, void *recvbuf, size_t recvlen,
                        struct sockaddr *src_addr, socklen_t *addrlen)
{
    ssize_t nbytes;

    do {
        // Wait if no packets are available
        struct pollfd fd = {.fd = sock, .events = POLLIN, .revents = 0};
        while (!poll(&fd, 1, 0)) {}

        // Get the newest available packet
        nbytes = get_newest_packet(sock, recvbuf, recvlen, src_addr, addrlen);
    } while ((ssize_t) recvlen != nbytes);

    // Return the copied packet size
    return nbytes;
}


ssize_t send_packet(int sock, void *sendbuf, size_t sendlen,
                    struct sockaddr *dst_addr, socklen_t addrlen)
{
    ssize_t nbytes;

    // Send packet, retrying if busy
    do {
        nbytes = sendto(sock, sendbuf, sendlen, 0, dst_addr, addrlen);
    } while (-1 == nbytes);

    // Return the sent packet size
    return nbytes;
}
