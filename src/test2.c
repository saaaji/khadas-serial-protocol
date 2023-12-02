#define _DEFAULT_SOURCE
#include <stdlib.h>
#include <stdint.h> // integer types
#include <stdio.h> // printf and family
#include <fcntl.h> // file IO
#include <unistd.h> // more file IO
#include <termios.h> // serial config
#include <errno.h> // error tracking
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <string.h>
#include <signal.h>
#define termios asm_termios
#include <asm/termbits.h>
#include <asm/ioctls.h>
#undef termios



const int TEENSY_BITRATE = 480000000;
const int S_TO_MICROS = 1000000;
const int MEGA = 1000000;

int elapsed_micros(const struct timeval st) {
    struct timeval et;
    gettimeofday(&et, NULL);
    return (et.tv_sec - st.tv_sec) * S_TO_MICROS + (et.tv_usec - st.tv_usec);
}

#define SECONDS(st) (elapsed_micros(st) / (float)MEGA)
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// check errno when return is -1
int init_uart(int port) {
    struct termios tty;

    if (tcgetattr(port, &tty) < 0) {
        return -1;
    }

    // "raw" mode
    tty.c_cflag |= (CLOCAL | CREAD); // ignore modem control lines, enable receiver
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8; // 1 "char" = 8 bits
    tty.c_cflag &= ~PARENB; // disable bit parity check
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    // tty.c_cflag &= ~CRTSCTS; // disable HW flow control (RTS)
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST; // disable implementation-based output processing

    // blocking reads
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(port, TCSANOW, &tty) < 0) {
        return -1;
    }

    /*
    man page: "tcsetattr() returns success if any of the requested changes could 
    successfully be carried out...when making multiple changes...follow this call with a
    further call to tcgetattr to check that all changes have been performed successfully"
    */
    if (tcgetattr(port, &tty) < 0) {
        return -1;
    }

    return 0;
}

// if return value is less than 0, check errno
// https://man7.org/linux/man-pages/man2/ioctl_tty.2.html
int config_uart_bitrate(int port, int bitrate) {
    struct termios2 tty;

    // ioctl equivalent to "tc(get/set)attr"
    if (ioctl(port, TCGETS2, &tty) < 0) {
        return -1;
    }

    /*
    man page: "If...c_cflag contains the flag BOTHER, then the baud rate is
    stored in the structure members c_ispeed and c_ospeed as integer values"
    */
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ispeed = bitrate;
    tty.c_ospeed = bitrate;

    if (ioctl(port, TCSETS2, &tty) < 0) {
        return -1;
    }

    if (ioctl(port, TCGETS2, &tty) < 0) {
        return -1;
    }

    return 0;
}

unsigned char *write_buffer = NULL;
void cleanup() {
    printf("cleaning write buffer...\n");
    free(write_buffer);
    write_buffer = NULL;
    exit(EXIT_SUCCESS);
}

typedef enum {
    SEEK_SOF,
    READ_HEADER,
    READ_BODY,
} State;

typedef struct {
    State state;

    size_t offset;
    unsigned char header[7];
    unsigned char body[1024];
} FSM;

int main(int argc, char **argv) {
    int port = open(argv[1], O_RDWR | O_NOCTTY);

    if (port < 0) {
        printf("could not open port \"%s\": %d\n", argv[1], errno);
        exit(EXIT_FAILURE);
    }

    if (init_uart(port) < 0) {
        printf("could not configure UART settings: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    if (config_uart_bitrate(port, TEENSY_BITRATE) < 0) {
        printf("could not configure UART bitrate: %d\n", errno);
        exit(EXIT_FAILURE);
    }

    struct sigaction act;
    memset(&act, 0, sizeof(act));
    act.sa_handler = cleanup;
    act.sa_flags = 0;
    if (sigaction(SIGINT, &act, NULL) < 0) {
        printf("could not configure cleanup handler: %d\n", errno);
    }

    int freq = 2000;
    int cycle_time_micros = S_TO_MICROS / freq;
    int transfer_speed = TEENSY_BITRATE / 8;
    int bitrate_throttle_limit = transfer_speed / freq;

    size_t read_buffer_size = 4095;
    unsigned char read_buffer[read_buffer_size];

    struct timeval prog_st;
    gettimeofday(&prog_st, NULL);

    unsigned char test_packet[] = {0xa5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int packet_len = 15;

    write_buffer = (unsigned char *) calloc(sizeof(unsigned char), bitrate_throttle_limit);
    int write_idx = 0;
    int total_sent = 0;
    int total_recv = 0;

    FSM fsm;
    fsm.state = SEEK_SOF;
    fsm.offset = 0;


    FILE *logf = fopen("log.txt", "w");

    for (int counter = 0;; counter++) {
        struct timeval st;
        gettimeofday(&st, NULL);

        write_idx = 0;
        int num_packets = bitrate_throttle_limit / packet_len;
        for (int i = 0; i < num_packets; i++) {
            for (int j = 0; j < packet_len; j++) {
                write_buffer[write_idx++] = test_packet[j];
            }
        }

        ssize_t num_written = write(port, write_buffer, bitrate_throttle_limit);

        int recv = 0;
        while (recv < num_written) {
            // break;
            int res = read(port, read_buffer, MIN(read_buffer_size, num_written - recv));
            if (res > 0) {
                recv += res;
                for (int i = 0; i < res; i++) {
                    unsigned char byte = read_buffer[i];
                    switch (fsm.state) {
                        case SEEK_SOF:
                            if (byte == 0xA5) {
                                fsm.state = READ_HEADER;
                            }
                            break;
                        case READ_HEADER:
                            // if (fsm.offset < )
                            break;
                        case READ_BODY:
                            break;
                    }
                }
            }
        }

        total_sent += num_written;
        total_recv += recv;
        
        if (counter % 100 == 0) {
          float elapsed_s = elapsed_micros(prog_st) / (float) MEGA;
          fprintf(logf, "%0.2f\t%0.2f\t%0.2f\n",
              elapsed_s,
              (total_recv * 8.0) / MEGA / elapsed_s,
              (total_sent * 8.0) / MEGA / elapsed_s);
          // if (elapsed_s > 10.0) break;
        }

        while (elapsed_micros(st) < cycle_time_micros) {
            continue;
        }
    }

    return 0;
}