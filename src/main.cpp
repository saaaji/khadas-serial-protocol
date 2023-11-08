#define termios asm_termios
#define winsize asm_winsize
#define termio asm_termio
#include <asm/termios.h>
#undef termios
#undef winsize
#undef termio

#include <poll.h>
#include <sys/time.h>
#include "SerialComms.hpp"
#include <chrono>
#include <string>

constexpr int CYCLE_FREQ_HZ = 2000; // Hz
constexpr int PACKET_DUMP_COUNT = 150;

uint32_t bit_cast(float f) {
  union {
    float f32;
    uint32_t u32;
  } caster;
  caster.f32 = f;
  return caster.u32;
}

typedef std::chrono::high_resolution_clock sys_time;
typedef std::chrono::duration<uint32_t, std::micro> micros;

// if return value is less than 0, check errno
// https://www.gnu.org/software/libc/manual/html_node/Setting-Modes.html
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
    // tty.c_cflag |= CSTOPB; // set 2 stop bits
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS; // disable HW flow control (RTS)
    // tty.c_cflag |= CRTSCTS; // enable HW flow control (RTS)
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST; // disable implementation-based output processing

    tty.c_cc[VMIN] = 0;
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

int elapsed_us(struct timeval st) {
    struct timeval et;
    gettimeofday(&et, NULL);
    return (et.tv_sec - st.tv_sec) * 1'000'000 + (et.tv_usec - st.tv_usec);
}

int main(int argc, char **argv) {
    int bitrate = 480'000'000; // 480 Mbps

    if (argc < 3) {
        printf("expected <payload_size> <divisor>\n");
        return 1;
    }

    // initialize UART
    int port = open("/dev/ttyACM0", O_RDWR | O_NOCTTY);

    if (port < 0) {
        printf("open failed: %d\n", errno);
        return 1;
    }

    if (init_uart(port) < 0) {
        printf("init_uart failed\n");
        return 1;
    }

    if (config_uart_bitrate(port, bitrate) < 0) {
        printf("config_uart_bitrate failed\n");
        return 1;
    }

    // allocate payload
    int payload_size = std::atoi(argv[1]);
    unsigned char *raw_bytes = reinterpret_cast<unsigned char*>(calloc(sizeof(unsigned char), 100000)); // n bytes
    raw_bytes[0] = 1;
    
    int size = 4095;
    int recv = 0;
    unsigned char buffer[size];

    int sent = 0;
    int freq = 2000;
    int cycle_time_us = 1'000'000 / freq;
    struct timeval gst;
    gettimeofday(&gst, NULL);

    int theoretical_limit = 60'000'000 / freq;
    int limit = theoretical_limit / std::atoi(argv[2]);
    int prev_recv = -1;

    for (int i = 0;;i++) {
        struct timeval st, et;
        gettimeofday(&st, NULL);

        if (sent < payload_size) {
            // teensy can send back data while we are writing, causing potential loss
            int res = write(port, raw_bytes, limit);
            raw_bytes[0] = 0;
            if (res < limit) {
                printf("write failed: %d\n", errno);
                return 1;
            } else {
                sent += res;
            }
        } else if (raw_bytes != nullptr) {
            free(raw_bytes);
            raw_bytes = nullptr;
        }

        int available = 0, tot = 0, n = 0;
        int res;
        do {
            res = read(port, buffer, size);
            if (res > 0) {
                tot += res;
                recv += res;
                n++;
            }
        } while (res > 0);

        /*ioctl(port, FIONREAD, &available);
        while (available > 0 && elapsed_us(st) < cycle_time_us) {
            int res = read(port, buffer, size);
            if (res > 0) {
                recv += res;
                tot += res;
                n++;
            }
            ioctl(port, FIONREAD, &available);
        }*/

        if (i % 5 == 0 && recv != prev_recv) {
            printf("RECV: %d / %d (%.2f) (%d) : ", recv, sent, float(recv)/sent, n);
        }

        if (recv == sent) break;

        while (elapsed_us(st) < cycle_time_us) {
            continue;
        }

        if (i % 5 == 0 && recv != prev_recv) {
            float elapsed = float(elapsed_us(gst)) / 1'000'000;
            printf("%.0f : %.3f\n", float(i) / elapsed, (8.0 * recv / elapsed)/1'000'000);
        }

        if (i % 50 == 0 && i != 0 && recv == prev_recv) {
            break;
        }

        if (i % 50 == 0) prev_recv = recv;
    }

    int elapsed = elapsed_us(gst);
    printf("TIME: %.0f", float(elapsed) / 1'000'000);
    
    free(raw_bytes);
    return 0;
}

//for linux machine
/*int main(int argc, char **argv) {
    SerialComms serial_comms("/dev/ttyACM0");
    SerialPacket packet;

    uint32_t cycle_time_us = 1'000'000 / CYCLE_FREQ_HZ;

    int dumpc = PACKET_DUMP_COUNT;
    if (argc > 1) {
        dumpc = std::stoi(argv[1]);
    }

    for (int i = 0; i < 10; i++) {
        serial_comms.stats.crc8_fails = 0;
        serial_comms.stats.crc16_fails = 0;
        serial_comms.stats.incomplete_packet = 0;
        serial_comms.stats.read_attempts = 0;
        

        for (int i = 0; i < dumpc; i++) {
            serial_comms.enqueue_data_request(SerialPacket::DR16, 0);
        }

        int recv_count = 0;
        int prev_count = 0;
        int streak = 0;
        while (true) {
            // printf("\033[H\033[J");

            auto start = sys_time::now();

            // auto perf_start = sys_time::now();
            int res = serial_comms.flush_queue(CYCLE_FREQ_HZ);
            bool recv = serial_comms.read_packet(packet);

            // printf("[DUMP: %d]\n", res);
            // printf("%s\n", recv ? "recv" : "(!) not recv");
            // printf("[RECV: %d]\n", recv_count);

            if (recv) recv_count++;
            if (recv_count == prev_count)
                streak++;
            else
                streak = 0;

            prev_count = recv_count;

            if (streak > 100) break;

            while(
              std::chrono::duration_cast<micros>(sys_time::now() - start).count() < cycle_time_us
            );
        }

        NetStats s = serial_comms.stats;
        float crc8_packet = (float) s.crc8_fails / recv_count;
        float crc16_packet = (float) s.crc16_fails / recv_count;
        float incomplete_packet = (float) s.incomplete_packet / recv_count;
        float expected_throughput = float(dumpc - dumpc * (crc8_packet + crc16_packet + incomplete_packet)) / dumpc;

        printf("\n[TRIAL: %d]\n", i);
        printf("payload size: %d\n", (5 + 2 + 29 + 2) * dumpc);
        printf("actual throughput: %.2f%% (net %d)]\n", 100 * (float) recv_count / dumpc, recv_count);
        printf("expected throughput: %.2f%% (net %d)]\n", 100 * expected_throughput, (int)(expected_throughput*dumpc));
        printf("\tstats/read_attempts: %d\n", serial_comms.stats.read_attempts);
        printf("\tstats/crc8_fails: %d (%.2f /p)\n", serial_comms.stats.crc8_fails, crc8_packet);
        printf("\tstats/crc16_fails: %d (%.2f /p)\n", serial_comms.stats.crc16_fails, crc16_packet);
        printf("\tstats/incomplete_packet: %d (%.2f /p)\n", serial_comms.stats.incomplete_packet, incomplete_packet);
    }
    
    return 0;
}*/
