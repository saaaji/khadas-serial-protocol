#include "SerialComms.hpp"
#include <chrono>

// Hz
#define CYCLE_FREQ_HZ 10

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
int main() {
    SerialComms serial_comms("/dev/ttyACM0");
    SerialPacket packet;

    uint32_t cycle_time_us = 1'000'000 / CYCLE_FREQ_HZ;

    while (true) {
        auto start = sys_time::now();

        serial_comms.enqueue_data_request(SerialPacket::DR16, 0);
        int res = serial_comms.flush_queue(CYCLE_FREQ_HZ);

        printf("[DEQ: %d]\n", res);

        serial_comms.read_packet(packet);

        while(
          std::chrono::duration_cast<micros>(sys_time::now() - start).count() < cycle_time_us
        );
    }
    
    return 0;
}