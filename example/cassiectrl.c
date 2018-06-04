/*
 * Copyright (c) 2018 Dynamic Robotics Laboratory
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

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include "cassie_out_t.h"
#include "cassie_user_in_t.h"
#include "state_out_t.h"
#include "pd_in_t.h"
#include "udp.h"


enum mode {
    MODE_STANDARD,
    MODE_PD
};


int main(int argc, char *argv[])
{
    // Option variables and flags
    char *remote_addr_str = "127.0.0.1";
    char *remote_port_str = "25000";
    char *iface_addr_str = "0.0.0.0";
    char *iface_port_str = "25001";
    int mode = MODE_STANDARD;

    // Parse arguments
    int c;
    while ((c = getopt(argc, argv, "a:p:b:c:x")) != -1) {
        switch (c) {
        case 'a':
            // Remote address to connect to
            remote_addr_str = optarg;
            break;
        case 'p':
            // Port to connect to
            remote_port_str = optarg;
            break;
        case 'b':
            // Remote address to connect to
            iface_addr_str = optarg;
            break;
        case 'c':
            // Port to connect to
            iface_port_str = optarg;
            break;
        case 'x':
            // Run in PD mode
            mode = MODE_PD;
            break;
        default:
            // Print usage
            printf(
"Usage: cassiectrl [OPTION]...\n"
"Controls the Cassie robot over UDP.\n"
"\n"
"  -a [ADDRESS]   Specify the local interface to bind to.\n"
"  -p [PORT]      Specify the port to listen on.\n"
"  -b [ADDRESS]   Specify the remote address to connect to.\n"
"  -c [PORT]      Specify the remote port to connect to.\n"
"  -x             Run in PD mode, taking state estimates and sending PD targets.\n"
"\n"
"By default, the controller connects to localhost over IPv4 to port %s"
"from port %s.\n\n",
remote_port_str, iface_port_str);
            exit(EXIT_SUCCESS);
        }
    }

    // Bind to network interface
    int sock = udp_init_client(remote_addr_str, remote_port_str,
                               iface_addr_str, iface_port_str);
    if (-1 == sock)
        exit(EXIT_FAILURE);

    // Create packet input/output buffers
    int dinlen, doutlen;
    switch (mode) {
    case MODE_PD:
        dinlen = STATE_OUT_T_PACKED_LEN;
        doutlen = PD_IN_T_PACKED_LEN;
        break;
    default:
        dinlen = CASSIE_OUT_T_PACKED_LEN;
        doutlen = CASSIE_USER_IN_T_PACKED_LEN;
    }
    const int recvlen = PACKET_HEADER_LEN + dinlen;
    const int sendlen = PACKET_HEADER_LEN + doutlen;
    unsigned char *recvbuf = malloc(recvlen);
    unsigned char *sendbuf = malloc(sendlen);

    // Separate input/output buffers into header and payload
    const unsigned char *header_in = recvbuf;
    const unsigned char *data_in = &recvbuf[PACKET_HEADER_LEN];
    unsigned char *header_out = sendbuf;
    unsigned char *data_out = &sendbuf[PACKET_HEADER_LEN];

    // Create standard input/output structs
    cassie_user_in_t cassie_user_in = {0};
    cassie_out_t cassie_out;

    // Create PD input/output structs
    pd_in_t pd_in = {0};
    state_out_t state_out;

    // Create header information struct
    packet_header_info_t header_info = {0};

    // Prepare initial null command packet to start communication
    printf("Connecting to cassie...\n");
    memset(sendbuf, 0, sendlen);
    bool received_data = false;

    // Listen/respond loop
    while (true) {
        if (!received_data) {
            // Send null commands until the simulator responds
            ssize_t nbytes;
            do {
                send_packet(sock, sendbuf, sendlen, NULL, 0);
                usleep(1000);
                nbytes = get_newest_packet(sock, recvbuf, recvlen, NULL, NULL);
            } while (recvlen != nbytes);
            received_data = true;
            printf("Connected!\n\n");
        } else {
            // Wait for a new packet
            wait_for_packet(sock, recvbuf, recvlen, NULL, NULL);
        }

        // Process incoming header and write outgoing header
        process_packet_header(&header_info, header_in, header_out);
        printf("\033[F\033[Jdelay: %d, diff: %d\n",
               header_info.delay, header_info.seq_num_in_diff);

        switch (mode) {
        case MODE_PD:
            // Unpack received data into cassie user input struct
            unpack_state_out_t(data_in, &state_out);

            // Run controller
            // Do nothing in this example

            // Pack cassie out struct into outgoing packet
            pack_pd_in_t(&pd_in, data_out);
            break;
        default:
            // Unpack received data into cassie user input struct
            unpack_cassie_out_t(data_in, &cassie_out);

            // Run controller
            // Do nothing in this example

            // Pack cassie out struct into outgoing packet
            pack_cassie_user_in_t(&cassie_user_in, data_out);
        }

        // Send response
        send_packet(sock, sendbuf, sendlen, NULL, 0);
    }
}
