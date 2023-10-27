#include "SerialComms.hpp"

uint32_t bit_cast(float f) {
  union {
    float f32;
    uint32_t u32;
  } caster;
  caster.f32 = f;
  return caster.u32;
}

//for linux machine
int main() {
    SerialComms serial_comms("/dev/ttyACM0");

    printf("[ARDUINO READBACK]\n");

    // Continuously read and print bytes from the serial port
    int counter = 0;
    while (counter++ < 50) {
        uint8_t dr16_data[28];
        float dr16_floats[5] = {0.1, 0.2, 0.3, 0.4, 0.5};
        uint32_t dr16_ints[2] = {1, 2};

        SerialPacket packet;

        for (int i = 0; i < 5; i++) {
            uint32_t u32 = bit_cast(dr16_floats[i]);
            dr16_data[i * 4 + 0] = u32 & 0xFF;
            dr16_data[i * 4 + 1] = (u32 >> 8) & 0xFF;
            dr16_data[i * 4 + 2] = (u32 >> 16) & 0xFF;
            dr16_data[i * 4 + 3] = (u32 >> 24) & 0xFF;
        }

        for (int i = 0; i < 2; i++) {
            dr16_data[20 + i * 4 + 0] = dr16_ints[i] & 0xFF;
            dr16_data[20 + i * 4 + 1] = (dr16_ints[i] >> 8) & 0xFF;
            dr16_data[20 + i * 4 + 2] = (dr16_ints[i] >> 24) & 0xFF;
            dr16_data[20 + i * 4 + 3] = (dr16_ints[i] >> 16) & 0xFF;
        }
        
        serial_comms.send_packet(SerialPacket::DR16, dr16_data, 28);
        serial_comms.read_packet(packet);
    }
    
    return 0;
}