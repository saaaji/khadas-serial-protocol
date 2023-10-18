#include "SerialComms.hpp"
#include <cstring>

uint32_t bit_cast(float f) {
    uint32_t target;
    std::memcpy(&target, &f, sizeof(float));
    return target;
}

//for linux machine
int main() {
    const char *port_name = "/dev/ttyACM0"; // Update with your serial port name
    const speed_t baud_rate = B9600;         // Adjust the baud rate as needed

    SerialComms serial_comms(port_name, baud_rate);

    std::cout << "[ARDUINO READBACK]" << std::endl;

    // Continuously read and print bytes from the serial port
    while (true) {
        // std::string input;
        // std::cin >> input;

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
            // printf("sent: %u\n", u32);
        }

        for (int i = 0; i < 2; i++) {
            dr16_data[20 + i * 4 + 0] = dr16_ints[i] & 0xFF;
            dr16_data[20 + i * 4 + 1] = (dr16_ints[i] >> 8) & 0xFF;
            dr16_data[20 + i * 4 + 2] = (dr16_ints[i] >> 24) & 0xFF;
            dr16_data[20 + i * 4 + 3] = (dr16_ints[i] >> 16) & 0xFF;
            // printf("sent: %d\n", dr16_ints[i]);
        }
        
        serial_comms.send_packet(SerialPacket::DR16, dr16_data, 28);
        sleep(1);
        serial_comms.read_packet(packet);
    }
    
    return 0;
}