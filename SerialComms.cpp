#include "SerialComms.hpp"

SerialComms::SerialComms(const char *port_name, const speed_t& baud_rate) 
{
    // attempt to open the serial port
    port = open(port_name, O_RDWR);

    // check if Teensy is connected
    if (port < 0) {
        log_err("open", "Teensy may be disconnected");
    } else {
        // Configure the serial port settings
        struct termios tty;

        if (tcgetattr(port, &tty) < 0) {
            log_err("tcgetattr");
        }
        
        if (cfsetospeed(&tty, baud_rate) < 0) {
            log_err("cfsetospeed");
        }

        if (cfsetispeed(&tty, baud_rate) < 0) {
            log_err("cfsetispeed");
        }

        // setting "raw mode" attributes according to man7.org/linux/man-pages/man3/termios3.html
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;    // No parity
        tty.c_cflag &= ~CSTOPB;    // 1 stop bit
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;        // 8 data bits
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_oflag &= ~OPOST;
        
        // set VMIN and VTIME to 0 (for total non-blocking reads)
        // man page warns against this so alternatives are welcomed
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        
        if (tcsetattr(port, TCSANOW, &tty) < 0) {
            log_err("tcsetattr");
        }
    }
}

SerialComms::~SerialComms() 
{
    // attempt to close the serial port
    if (close(this->port) < 0) {
        log_err("close", "Teensy may be disconnected");
    }
}

void SerialComms::log_err(const std::string& func_name, const std::string& msg)
{
    std::cerr << "\"" << func_name << "\"" << " call failed (" << msg << "): errno = " << errno << std::endl;
}

void SerialComms::log_err(const std::string& func_name)
{
    std::cerr << "\"" << func_name << "\"" << " call failed: errno = " << errno << std::endl;
}

uint8_t SerialComms::generate_crc8(const uint8_t *data, int size)
{
    uint8_t crc8 = 0xFF;
    for (int i = 0; i < size; i++) {
        uint8_t n = crc8 ^ data[i];
        crc8 = CRC8_LOOKUP[n];
    }
    return crc8;
}

uint16_t SerialComms::generate_crc16(const uint8_t *data, int size)
{
    uint16_t crc16 = 0xFFFF;
    for (int i = 0; i < size; i++) {
        uint8_t n = data[i];
        crc16 = (crc16 >> 8) ^ CRC16_LOOKUP[(crc16 ^ static_cast<uint16_t>(n)) & 0x00FF];
    }
    return crc16;
}

int SerialComms::flush_write_buffer()
{
    if (write(port, write_buffer, write_buffer_offset) < 0) {
        log_err("write");
        return -1;
    }
    write_buffer_offset = 0;
    return 0;
}

int SerialComms::write_byte(uint8_t byte)
{
    // check to make sure we don't write off the end of the buffer
    if (write_buffer_offset >= WRITE_BUFFER_MAX_SIZE) {
        if (flush_write_buffer() < 0) {
            log_err("flush_write_buffer");
            return -1;
        }
    }
    
    write_buffer[write_buffer_offset++] = byte;
    return 0;
}

int SerialComms::write_uint(uint64_t u64, int size)
{
    // if requesting 1 byte, just stage it directly
    if (size == 1) {
        return write_byte((uint8_t) u64);
    } else {
        // serialize integer into respective bytes in little endian order
        for (int i = 0; i < size; i++) {
            uint8_t byte = (u64 >> i * 8) & 0xFF;
            if (write_byte(byte) < 0) {
                return -1;
            }
        }
        return 0;
    }
}

void SerialComms::write_header(uint16_t length)
{
    // stage all components of header
    write_byte(START_BYTE);
    write_uint(length, 2);
    write_byte(seq);

    // update packet sequence number if must wrap around
    if (seq == 255) {
        seq = 0;
    } else {
        seq++;
    }

    // TODO: implement crc8
    write_byte(generate_crc8(write_buffer, write_buffer_offset));
}

void SerialComms::write_footer()
{
    uint16_t crc16 = generate_crc16(write_buffer, write_buffer_offset);
    write_uint(crc16, 2);
}

void SerialComms::send_packet(SerialPacket::CommandID cmd_id, uint8_t *data, uint16_t length)
{
    // stage all components of packet
    write_header(length);
    write_uint((uint16_t) cmd_id, 2);

    for (int i = 0; i < length; i++) {
        write_byte(data[i]);
    }

    write_footer();
    flush_write_buffer();
}

// send DATA_REQUEST
void SerialComms::request_data(SerialPacket::CommandID cmd_id, uint8_t sensor_id)
{
    uint8_t data[3];
    
    data[0] = (uint16_t) cmd_id & 0xFF;
    data[1] = (uint16_t) cmd_id >> 8;
    data[2] = sensor_id;

    send_packet(SerialPacket::DATA_REQUEST, data, 3);
}

int SerialComms::read_byte()
{
    // if we are at the end of the read buffer, overwrite from beginning
    if (read_buffer_offset == read_buffer_current_size) {
        int result = read(port, read_buffer, READ_BUFFER_MAX_SIZE);

        if (result >= 0) {
            read_buffer_current_size = result; 
            read_buffer_offset = 0;
        } else {
            log_err("read");
            read_buffer_current_size = 0;
            read_buffer_offset = 0;
        }
    }

    if (read_buffer_current_size > 0) {
        return (int) read_buffer[read_buffer_offset++];
    } else {
        return -1;
    }
}

int SerialComms::read_bytes(uint8_t *data, int count)
{
    int read_count;
    for (read_count = 0; read_count < count; read_count++) {
        int byte = read_byte();
        if (byte >= 0) {
            data[read_count] = (uint8_t) byte;
        } else {
            break;
        }
    }
    return read_count;
}

uint64_t SerialComms::decode_uint(int offset, int size, uint8_t *data)
{
    // if we just want 1 byte, pull from array directly
    if (size == 1) {
        return data[offset];
    } else {
        // must assemble integer from bytes in little endian order
        uint64_t result = 0;
        for (int i = 0; i < size; i++) {
            uint8_t byte = data[offset + i];
            result = result | (byte << i * 8);
        }
        return result;
    }
}

uint16_t SerialComms::decode_short(int offset, uint8_t *data)
{
    return static_cast<uint16_t>(decode_uint(offset, 2, data));
}

int SerialComms::read_packet(SerialPacket& packet)
{
    int byte;
    uint8_t temp[4];

    // try to pull bytes from the stream until a start byte is encountered
    while ((byte = read_byte()) >= 0) {
        if (byte == START_BYTE) {
            #ifdef DEBUG_PACKETS
            printf("\n[ATTEMPTING PACKET READ]\n");
            #endif

            // read the packet header
            if (read_bytes(temp, 4) < 4) {
                break;
            }

            uint16_t data_length = decode_short(0, temp);
            uint8_t seq = temp[2];
            uint8_t crc8 = temp[3];

            // check for dropped packet
            if (seq - prev_seq > 1) {
                std::cerr << "packet dropped" << std::endl;
            }
            prev_seq = seq;

            // read the packet data section
            if (read_bytes(temp, 2) < 2) {
                break;
            }

            uint16_t cmd_id = decode_short(0, temp);
            uint8_t packet_bytes[7 + data_length];
            
            packet_bytes[0] = START_BYTE;
            packet_bytes[1] = data_length & 0xFF;
            packet_bytes[2] = data_length >> 8;
            packet_bytes[3] = seq;
            packet_bytes[4] = crc8;
            packet_bytes[5] = cmd_id & 0xFF;
            packet_bytes[6] = cmd_id >> 8;

            uint8_t *data = &packet_bytes[7];
            uint8_t calc_crc8 = generate_crc8(packet_bytes, 4);

            #ifdef DEBUG_PACKETS
            printf("\tpacket sequence num = %u\n", seq);
            printf("\treceived crc8 = %u\n", crc8);
            printf("\tcalculated crc8 = %u\n", calc_crc8);
            #endif

            #ifdef CHECK_CRC
            if (crc8 != calc_crc8) {
                std::cerr << "CRC8 checksums do not match" << std::endl;
                break;
            }
            #endif
            
            if (read_bytes(data, data_length) < data_length) {
                break;
            }

            // read the packet footer
            if (read_bytes(temp, 2) < 2) {
                break;
            }

            uint16_t crc16 = decode_short(0, temp);
            uint16_t calc_crc16 = generate_crc16(packet_bytes, 7 + data_length);
            
            #ifdef DEBUG_PACKETS
            printf("\treceived crc16 = %u\n", crc16);
            printf("\tcalculated crc16 = %u\n", calc_crc16);
            #endif
            
            #ifdef CHECK_CRC
            if (crc16 != calc_crc16) {
                std::cerr << "CRC16 checksums do not match" << std::endl;
                break;
            }
            #endif

            // utility arrays for decoding packet data
            float float_arr[16] = {0.0};
            uint32_t uint32_arr[16] {0};
            union {
                uint32_t u32;
                float f32;
            } bitcaster;

            packet.cmd_id = static_cast<SerialPacket::CommandID>(cmd_id);
            switch (cmd_id) {
                // sensors and estimators
                case 0x0101: // estimator state
                    break;
                case 0x0102: // motor feedback
                    break;
                case SerialPacket::DR16: // DR16 data
                    for (int i = 0; i < 5; i++) {
                        bitcaster.u32 = static_cast<uint32_t>(decode_uint(i * 4, 4, data));
                        float_arr[i] = bitcaster.f32;
                    }

                    for (int i = 0; i < 2; i++) {
                        uint32_arr[i] = static_cast<uint32_t>(decode_uint(20 + i * 4, 4, data));
                    }

                    packet.dr16.l_stick_x = float_arr[0];
                    packet.dr16.l_stick_y = float_arr[1];
                    packet.dr16.r_stick_x = float_arr[2];
                    packet.dr16.r_stick_y = float_arr[3];
                    packet.dr16.wheel = float_arr[4];
                    packet.dr16.l_switch = uint32_arr[0];
                    packet.dr16.r_switch = uint32_arr[1];

                    #ifdef DEBUG_PACKETS
                    printf(
                        "\tdr16 packet data (%#02x):\n"
                        "\t\tl_stick_x: %f\n"
                        "\t\tl_stick_y: %f\n"
                        "\t\tr_stick_x: %f\n"
                        "\t\tr_stick_y: %f\n"
                        "\t\twheel: %f\n"
                        "\t\tl_switch: %u\n"
                        "\t\tr_switch: %u\n",
                        packet.cmd_id,
                        packet.dr16.l_stick_x,
                        packet.dr16.l_stick_y,
                        packet.dr16.r_stick_x,
                        packet.dr16.r_stick_y,
                        packet.dr16.wheel,
                        packet.dr16.l_switch,
                        packet.dr16.r_switch
                    );
                    #endif
                    
                    break;
                case SerialPacket::REV_ENCODER: // rev encoder
                    bitcaster.u32 = static_cast<uint32_t>(decode_uint(0, 4, data));
                    packet.rev_encoder.angle = bitcaster.f32;

                    #ifdef DEBUG_PACKETS
                    printf(
                        "\tRev Encoder packet data (%#02x):\n"
                        "\t\tangle: %f\n",
                        packet.cmd_id,
                        packet.rev_encoder.angle
                    );
                    #endif

                    break;
                case SerialPacket::ISM: // ISM
                    for (int i = 0; i < 3; i++) {
                        bitcaster.u32 = static_cast<uint32_t>(decode_uint(i * 4, 4, data));
                        float_arr[i] = bitcaster.f32;
                    }

                    packet.ism.psi = float_arr[0];
                    packet.ism.theta = float_arr[1];
                    packet.ism.phi = float_arr[2];

                    #ifdef DEBUG_PACKETS
                    printf(
                        "\tISM packet data (%#02x):\n"
                        "\t\tpsi: %f\n"
                        "\t\tpsi: %f\n"
                        "\t\tpsi: %f\n",
                        packet.cmd_id,
                        packet.ism.psi,
                        packet.ism.theta,
                        packet.ism.phi
                    );
                    #endif

                    break;

                // referee system
                case 0x0201: // referee system data
                    break;
                case 0x0202: // client draw command
                    break;
            }

            // successful packet read
            return 0;
        }
    }
    
    // while loop broke, packet was not found
    return -1;
}