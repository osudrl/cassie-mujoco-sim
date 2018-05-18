#include "udp.h"
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


enum mode {
    MODE_STANDARD,
    MODE_RL
};


int main(int argc, char *argv[])
{
    // Option variables and flags
    char *remote_addr_str = "127.0.0.1";
    char *port_str = "25000";
    int mode = MODE_STANDARD;

    // Parse arguments
    int c;
    while ((c = getopt(argc, argv, "a:p:x")) != -1) {
        switch (c) {
        case 'a':
            // Remote address to connect to
            remote_addr_str = optarg;
            break;
        case 'p':
            // Port to connect to
            port_str = optarg;
            break;
        case 'x':
            // Run in RL mode
            mode = MODE_RL;
            break;
        default:
            // Print usage
            printf(
"Usage: cassiectrl [OPTION]...\n"
"Controlls the Cassie robot over UDP.\n"
"\n"
"  -a [ADDRESS]   Specify the local interface to bind to.\n"
"  -p [PORT]      Specify the port to listen on.\n"
"  -x             Run in RL mode, taking state estimates and sending PD targets.\n"
"\n"
"By default, the simulator connects to localhost over IPv4 at port %s.\n\n",
port_str);
            exit(EXIT_SUCCESS);
        }
    }

    // Bind to network interface
    int sock = udp_init(remote_addr_str, port_str, UDP_CLIENT);
    if (-1 == sock)
        exit(EXIT_FAILURE);

    // Create packet input/output buffers
    int dinlen, doutlen;
    switch (mode) {
    case MODE_RL:
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

    // Create RL input/output structs
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
        // Try to get a new packet
        ssize_t nbytes = get_newest_packet(sock, recvbuf, recvlen, NULL, NULL);

        // If no data has been received from the simulator, send null command
        if (!received_data) {
            while (-1 == send(sock, sendbuf, sendlen, 0)) {}
            usleep(1000);
        }

        // If no new cassie output packets were received, do nothing
        if (recvlen != nbytes)
            continue;

        // If execution reaches here, data has been received from the simulator
        received_data = true;

        // Process incoming header and write outgoing header
        process_packet_header(&header_info, header_in, header_out);
        printf("\033[F\033[Jdelay: %d, diff: %d\n",
               header_info.delay, header_info.seq_num_in_diff);

        switch (mode) {
        case MODE_RL:
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

        // Send response, retry if busy
        while (-1 == send(sock, sendbuf, sendlen, 0)) {}
    }
}
