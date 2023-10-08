#ifndef SERIAL_COMMS
#define SERIAL_COMMS

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <cstring>

#define DEBUG_PACKETS // log packet debug data
#define READ_BUFFER_MAX_SIZE 1024 // max size (in bytes) of the read buffer
#define WRITE_BUFFER_MAX_SIZE 1024 // max size (in bytes) of the write buffer
#define START_BYTE 0xA5 // start byte of packet

/// @brief Simple struct to store packet data
struct SerialPacket {
    uint16_t cmd_id;
    struct {
        float l_stick_x, l_stick_y, r_stick_x, r_stick_y, wheel;
        uint32_t l_switch, r_switch;
    } dr16;
};

/// @brief Exposes basic functions for sending and receiving packets from the Teensy
class SerialComms {
    private:
        int port;
        uint8_t seq = 0;
        uint8_t prev_seq = 0;
        SerialPacket packet;
        
        int read_buffer_current_size = 0;
        uint8_t read_buffer[READ_BUFFER_MAX_SIZE];
        uint8_t write_buffer[WRITE_BUFFER_MAX_SIZE];
        int read_buffer_offset = 0;
        int write_buffer_offset = 0;
        
        uint64_t decode_uint(int offset, int size, uint8_t *data);
        uint16_t decode_short(int offset, uint8_t *data);
        float decode_float(int offset, uint8_t *data);
        double decode_double(int offset, uint8_t *data);

        int stage_byte(uint8_t byte);
        int stage_uint(uint64_t u64, int size);

        int read_byte();
        int read_bytes(uint8_t *data, int count);
        
        void stage_header(uint16_t length);
        void stage_footer();

        void log_err(const std::string& func_name, const std::string& msg);
        
    public:
        SerialComms(const char *port_name, const speed_t& baud_rate);
        void send_packet(uint16_t cmd_id, uint8_t *data, uint16_t length);
        SerialPacket read_packet();
        ~SerialComms();
};

#endif