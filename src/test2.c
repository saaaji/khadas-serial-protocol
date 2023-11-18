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
const int MEGA = 1000000;

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

    int freq = 2000;
    int cycle_time_micros = S_TO_MICROS / freq;
    int transfer_speed = TEENSY_BITRATE / 8;
    int bitrate_throttle_limit = transfer_speed / freq;

    size_t read_buf_size = 4095;
    unsigned char read_buf[read_buf_size];

    struct timeval global_st;
    gettimeofday(&global_st, NULL);

    unsigned char test_packet[] = {0xa5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    int packet_len = 15;

    unsigned char *write_buffer = (unsigned char *) calloc(sizeof(unsigned char), bitrate_throttle_limit);
    int write_idx = 0;

    int global_sent = 0;
    int global_recv = 0;
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
        while (recv < num_written)
            recv += read(port, read_buf, MIN(read_buf_size, num_written - recv));

        global_sent += num_written;
        global_recv += recv;
        
        float elapsed_s = elapsed_micros(global_st) / 1000000.0;
        printf("THROUGHPUT: %.2fMbps R / %.2fMbps W\n", 
            (global_recv * 8.0) / MEGA / elapsed_s,
            (global_sent * 8.0) / MEGA / elapsed_s);

        while (elapsed_micros(st) < cycle_time_micros) {
            continue;
        }
    }

    return 0;
}