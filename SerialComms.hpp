#ifndef SERIAL_COMMS
#define SERIAL_COMMS

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#define DEBUG_PACKETS // log packet debug data

constexpr int READ_BUFFER_MAX_SIZE = 1024; // max size (in bytes) of the read buffer
constexpr int WRITE_BUFFER_MAX_SIZE = 1024; // max size (in bytes) of the write buffer
constexpr int START_BYTE = 0xA5; // start byte of packet

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
        /// @brief FD of the serial port
        int port;
        
        /// @brief Sequence number which may be used to track packet loss; resets when it reaches 255
        uint8_t seq = 0;
        uint8_t prev_seq = 0;
        
        uint8_t read_buffer[READ_BUFFER_MAX_SIZE];
        uint8_t write_buffer[WRITE_BUFFER_MAX_SIZE];
        
        int read_buffer_current_size = 0;
        int write_buffer_current_size = 0;
        int read_buffer_offset = 0;
        int write_buffer_offset = 0;
        
        /// @brief Decodes unsigned-integer value of arbitrary size from byte array
        /// @note Bytes must be arranged in Little-Endian order
        /// @param offset Offset into the byte array
        /// @param size Size of the desired integer
        /// @param data Pointer to the byte array
        /// @return A 64-bit unsigned integer with the decoded value 
        uint64_t decode_uint(int offset, int size, uint8_t *data);
        
        /// @brief Decodes an unsigned short from byte array
        /// @note Bytes must be arranged in Little-Endian order
        /// @param offset Offset into the byte array
        /// @param data Pointer to the byte array
        /// @return An unsigned short with the decoded value
        uint16_t decode_short(int offset, uint8_t *data);

        /// @brief Decodes a float from byte array
        /// @note Bytes must be arranged in Little-Endian order
        /// @param offset Offset into the byte array
        /// @param data Pointer to the byte array
        /// @return A float with the decoded value
        float decode_float(int offset, uint8_t *data);

        /// @brief Decodes a double from byte array
        /// @note Bytes must be arranged in Little-Endian order
        /// @param offset Offset into the byte array
        /// @param data Pointer to the byte array
        /// @return A double with the decoded value
        double decode_double(int offset, uint8_t *data);

        /// @brief Read single byte from the stream
        /// @return Either -1 if no bytes are available or the byte value
        int read_byte();

        /// @brief  Read multiple bytes from the stream
        /// @param data Pointer to the data array that you want to read into
        /// @param count Desired number of bytes to be read
        /// @return The number of bytes successfully read from the stream
        int read_bytes(uint8_t *data, int count);

        int flush_write_buffer();

        /// @brief Write single byte to the write buffer
        /// @param byte Data byte
        /// @return -1 if write was unsuccessful
        int write_byte(uint8_t byte);

        /// @brief Write unsigned integer of arbitrary size to the write buffer
        /// @param u64 Data value
        /// @param size Size of the integer
        /// @return -1 if write was unsuccessful
        int write_uint(uint64_t u64, int size);

        /// @brief Write a full header to the write buffer
        /// @param length Length of the data section (check spec for details)
        void write_header(uint16_t length);

        /// @brief Write a full footer to the write buffer
        void write_footer();

        /// @brief Log error with message
        void log_err(const std::string& func_name, const std::string& msg);
        
        /// @brief Log error without message
        void log_err(const std::string& func_name);
        
    public:
        SerialComms(const char *port_name, const speed_t& baud_rate);
        ~SerialComms();

        /// @brief Send packet to the Teensy
        /// @param cmd_id Command ID of the packet
        /// @param data Pointer to the byte array with packet data
        /// @param length Length of the data section
        void send_packet(uint16_t cmd_id, uint8_t *data, uint16_t length);
        
        /// @brief Attempt to read packet from 
        /// @return -1 if unable to detect packet in the stream
        int read_packet(SerialPacket& packet);
};

#endif