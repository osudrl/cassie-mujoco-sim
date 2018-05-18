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

#ifndef UDP_H
#define UDP_H

#define PACKET_HEADER_LEN 2

// Data and results for processing packet header
typedef struct {
    char seq_num_out;
    char seq_num_in_last;
    char delay;
    char seq_num_in_diff;
} packet_header_info_t;


// Process packet header used to measure delay and skipped packets
void process_packet_header(packet_header_info_t *info,
                           const unsigned char *header_in,
                           unsigned char *header_out);

#ifndef _WIN32
#include <sys/socket.h>

enum udp_init_mode {
    UDP_SERVER,
    UDP_CLIENT
};

// Create a UDP socket bound or connected to a specific address
int udp_init(const char *addr_str, const char *port_str,
             enum udp_init_mode mode);

// Get newest valid packet in RX buffer
ssize_t get_newest_packet(int sock, void *recvbuf, size_t recvlen,
                          struct sockaddr *src_addr, socklen_t *addrlen);

// Wait for a new valid packet
ssize_t wait_for_packet(int sock, void *recvbuf, size_t recvlen,
                        struct sockaddr *src_addr, socklen_t *addrlen);

#endif // _WIN32
#endif // UDP_H
