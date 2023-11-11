#include <poll.h>
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
#include <linux/serial.h>
#include <string.h>
#define termios asm_termios
#include <asm/termbits.h>
#include <asm/ioctls.h>
#undef termios

#define MIN(a, b) ((a) < (b) ? (a) : (b))

const int TEENSY_BITRATE = 480000000;
const int S_TO_MICROS = 1000000;

int elapsed_micros(const struct timeval st) {
    struct timeval et;
    gettimeofday(&et, NULL);
    return (et.tv_sec - st.tv_sec) * S_TO_MICROS + (et.tv_usec - st.tv_usec);
}

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

    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

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

    struct serial_struct kernel_serial_settings;
    
    if (ioctl(port, TIOCGSERIAL, &kernel_serial_settings) < 0) {
        return -1;
    }

    kernel_serial_settings.flags |= ASYNC_LOW_LATENCY;

    if (ioctl(port, TIOCSSERIAL, &kernel_serial_settings) < 0) {
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

int main(int argc, char **argv) {
    if (argc < 5) {
        printf("expected <exe> <port> <freq> <%%bps> <size>\n");
        exit(EXIT_FAILURE);
    }

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

    int freq = atoi(argv[2]);
    double bitrate_util = atof(argv[3]);
    int payload_size = atoi(argv[4]);

    size_t alloc_size = 50000; // arbitrary 50K
    unsigned char *data = (char *)calloc(sizeof(unsigned char), alloc_size); // 50K allocation

    for (int i = 0; i < alloc_size; i++) {
        data[i] = 'A' + i % ('Z' - 'A' + 1); // mark each
    }

    int bitrate_limit = (int)(bitrate_util * TEENSY_BITRATE / 8) / freq;

    printf(
        "freq: %d\nbitrate_util: %lf\npayload_size: %d\nbitrate_limit: %d\n", 
        freq, 
        bitrate_util, 
        payload_size,
        bitrate_limit
    );

    int cycle_time_micros = S_TO_MICROS / freq;

    struct pollfd pfd;
    pfd.fd = port;
    pfd.events = POLLIN;
    pfd.revents = 0;

    size_t read_buf_size = 4095;
    unsigned char read_buf[read_buf_size];

    ssize_t bytes_sent = 0;
    ssize_t bytes_recv = 0;

    struct timeval global_st;
    gettimeofday(&global_st, NULL);

    for (int counter = 0;; counter++) {
        struct timeval st;
        gettimeofday(&st, NULL);

        // write
        if (bytes_sent < payload_size) {         
            int write_limit = MIN(bitrate_limit, alloc_size);
            int num_written = write(port, data, write_limit);
            // tcdrain(port);

            if (num_written < 0 && errno != EAGAIN) {
                printf("write failed: %d\n", errno);
            } else {
                bytes_sent += num_written;
            }
        } else {
            if (data != NULL) {
                free(data);
                data = NULL;
                printf("full payload sent, freeing allocation...\n");
                break;
            }
        }

        while (elapsed_micros(st) < cycle_time_micros) {
            continue;
        }

        // if (counter % 10 == 0) {
        //     printf(
        //         "TX(%ld / %ld) : %.2f Hz\n", 
        //         bytes_recv, 
        //         bytes_sent,
        //         counter / (elapsed_micros(global_st) / (float)S_TO_MICROS)
        //     );
        // }

        // if ((float)elapsed_micros(global_st) / S_TO_MICROS > 5.0f) {
        //     break;
        // }
    }

    float ts = elapsed_micros(global_st) / (float) S_TO_MICROS;
    printf("TIME: %f (%f)\n", ts, ((8 * bytes_sent) / ts));

    if (data != NULL) {
        free(data);
        data = NULL;
        printf("full payload sent, freeing allocation...\n");
    }

    return 0;
}