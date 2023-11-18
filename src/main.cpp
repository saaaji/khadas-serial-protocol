#include "SerialComms.hpp"
#include <chrono>
#include <string>
#include <cstdio>

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

//for linux machine
int main(int argc, char **argv) {
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
            auto start = sys_time::now();

            int res = serial_comms.flush_queue(CYCLE_FREQ_HZ);
            bool recv = serial_comms.read_packet(packet);

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
}
