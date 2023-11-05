#define termios asm_termios
#define winsize asm_winsize
#define termio asm_termio
#include <asm/termios.h>
#undef termios
#undef winsize
#undef termio

#include "SerialComms.hpp"
#include <chrono>
#include <string>

constexpr int CYCLE_FREQ_HZ = 1000; // Hz
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

int main(int argc, char **argv) {
    int cycle_time_us = 1'000'000 / CYCLE_FREQ_HZ;

    if (argc < 2)
        return 1;

    int payload_size = std::atoi(argv[1]);
    char *payload = new char[payload_size]; // n bytes

    for (int i = 0; i < payload_size; i++) {
        payload[i] = static_cast<char>(0);
    }
    payload[0] = 1;

    int port = open("/dev/ttyACM0", O_RDWR);

    struct termios tty;
    speed_t brate = B38400;
    tcgetattr(port, &tty);
    
    cfsetospeed(&tty, brate);
    cfsetispeed(&tty, brate);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;    // No parity
    tty.c_cflag &= ~CSTOPB;    // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;        // 8 data bits
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0; // NOBLOCK
    
    tcsetattr(port, TCSANOW, &tty);

    struct termios2 tty2;
    ioctl(port, TCGETS2, &tty2);
    tty2.c_cflag &= ~CBAUD;
    tty2.c_cflag |= BOTHER;
    tty2.c_ispeed = 480000000;
    tty2.c_ospeed = 480000000;
    // tty2.c_cc[VMIN] = 0;
    // tty2.c_cc[VTIME] = 0;
    // tty2.c_cflag |= (CLOCAL | CREAD);
    // tty2.c_cflag &= ~PARENB;    // No parity
    // tty2.c_cflag &= ~CSTOPB;    // 1 stop bit
    // tty2.c_cflag &= ~CSIZE;
    // tty2.c_cflag |= CS8;        // 8 data bits
    // tty2.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // tty2.c_oflag &= ~OPOST;
    ioctl(port, TCSETS2, &tty2);
    int blen = 4096;
    char bytes[blen];

    for (int i = 0; i < 1; i++) {
        // send n bytes
        int res = write(port, payload, payload_size);
        // tcdrain(port);
        printf("WBUF: %d\n", res);

        int recv = 0;
        int streak = 0;

        // printf("RBUF:");
        int j = 0;
        while (true) {
            j++;
            auto start = sys_time::now();
            int res = read(port, bytes, blen);
            if (res > 0) {
                // printf(" %d", res);
                streak = 0;
                recv += res;
            } else if (res == 0) {
                streak++;
            }
            
            if (j%1000==0) printf("R: %d\n", recv);
            if (recv >= payload_size) break;
            while (std::chrono::duration_cast<micros>(sys_time::now() - start).count() < cycle_time_us);
        }

        printf("\nTRIAL %d > recv: %d / %d\n\n", i, recv, payload_size);
    }

    delete payload;

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
