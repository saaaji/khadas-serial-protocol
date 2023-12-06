#include "UartSerialComms.hpp"

/**
* UartPacketData
*/
int UartPacketData::calc_body_size(UartPacketData::CmdId cmd_id) {
  switch (cmd_id) {
    case DATA_REQUEST:
      return 3;

    case REV_ENCODER:
      return 5;
  }
  return 0;
}

int UartPacketData::calc_response_size() {
  switch (cmd_id) {
    case DATA_REQUEST:
      UartPacketData::CmdId req_cmd_id = static_cast<UartPacketData::CmdId>(req.cmd_id);
      switch (req_cmd_id) {
        case REV_ENCODER:
          return 9 + UartPacketData::calc_body_size(req_cmd_id);
      }
      break;
  }

  return 0;
}

UartPacketData UartPacketData::from_raw(const UartRawPacket& raw) {
  UartPacketData data;
  int read_idx = 0;

  data.cmd_id = static_cast<UartPacketData::CmdId>(raw.cmd_id);

  union {
    uint32_t u32;
    float f32;
  } bitcaster;

  switch (data.cmd_id) {
    case UartPacketData::DATA_REQUEST:
      data.req.sensor_id = UartUtil::decode<uint8_t>(raw.body, read_idx);
      data.req.cmd_id = UartUtil::decode<uint16_t>(raw.body, read_idx);
      break;

    case UartPacketData::REV_ENCODER:
      data.rev_encoder.sensor_id = UartUtil::decode<uint8_t>(raw.body, read_idx);
      bitcaster.u32 = UartUtil::decode<uint32_t>(raw.body, read_idx);
      data.rev_encoder.angle = bitcaster.f32;
      break;
  }

  return data;
}

/**
* UartRawPacket
*/
uint8_t UartRawPacket::reset_crc8() {
  calc_crc8 = 0xff;
  return calc_crc8;
}

uint8_t UartRawPacket::cycle_crc8(const uint8_t& byte) {
  uint8_t n = calc_crc8 ^ byte;
  calc_crc8 = CRC8_LOOKUP[n];
  return calc_crc8;
}

uint16_t UartRawPacket::reset_crc16() {
  calc_crc16 = 0xffff;
  return calc_crc16;
}

uint16_t UartRawPacket::cycle_crc16(const uint8_t& byte) {
  calc_crc16 = (calc_crc16 >> 8) ^ CRC16_LOOKUP[(calc_crc16 ^ static_cast<uint16_t>(byte)) & 0x00ff];
  return calc_crc16;
}

void UartRawPacket::clear() {
  reset_crc8();
  reset_crc16();

  length = 0;
  cmd_id = 0;
  crc16 = 0;

  body = &bytes[7];
}

UartRawPacket UartRawPacket::from_data(const UartPacketData& data, const uint8_t& seq) {
  UartRawPacket raw;
  raw.body = &raw.bytes[5];
  int write_idx = 0;

  raw.cmd_id = static_cast<uint16_t>(data.cmd_id);
  raw.length = UartPacketData::calc_body_size(data.cmd_id);

  UartUtil::encode<uint8_t>(raw.bytes, write_idx, UartFsm::MAGIC);
  UartUtil::encode<uint16_t>(raw.bytes, write_idx, raw.length);
  UartUtil::encode<uint8_t>(raw.bytes, write_idx, seq);

  raw.reset_crc8();
  for (int i = 0; i < 4; i++) {
    raw.cycle_crc8(raw.bytes[i]);
  }

  UartUtil::encode<uint8_t>(raw.bytes, write_idx, raw.calc_crc8);
  UartUtil::encode<uint16_t>(raw.bytes, write_idx, raw.cmd_id);

  union {
    uint32_t u32;
    float f32;
  } bitcaster;

  switch (data.cmd_id) {
    case UartPacketData::REV_ENCODER:
      bitcaster.f32 = data.rev_encoder.angle;
      UartUtil::encode<uint32_t>(raw.bytes, write_idx, bitcaster.u32);
      break;
  }

  raw.reset_crc16();
  for (int i = 0; i < write_idx; i++) {
    raw.cycle_crc16(raw.bytes[i]);
  }

  UartUtil::encode<uint16_t>(raw.bytes, write_idx, raw.calc_crc16);
  return raw;
}

int UartRawPacket::get_size() {
  return 9 + length;
}

/**
* UartFsm
*/
UartFsm::UartFsm() {
  read_state = UartFsm::SCAN_MAGIC;
}

int z;
uint8_t hist[20];

bool UartFsm::cycle(const uint8_t& byte) {
  hist[z++ % 20] = byte;
  switch (read_state) {
    case UartFsm::SCAN_MAGIC:
      if (byte == MAGIC) {
        packet_state.clear();
        counter = 0;
        read_state = UartFsm::SCAN_LENGTH;
      }
      break;

    case UartFsm::SCAN_LENGTH:
      UartUtil::append_byte_le(packet_state.length, counter, byte);
      if (counter == 2) {
        read_state = UartFsm::SCAN_SEQ;
      }
      break;

    case UartFsm::SCAN_SEQ:
      packet_state.seq = byte;
      if (packet_state.length != 5) {
        for (int i = 0; i < 20; i++) {
          printf("%4d | ", i);
        }printf("\n");
        for (int i = 0; i < 20; i++) {
          printf("%4u | ", hist[i]);
        }
        printf("\nZ / l: %d / %u\n", z%20, packet_state.length);
        // printf("LEN[%u, %d]: %#x\n", byte, gseq, packet_state.length);
      }
      // gseq++;
      read_state = UartFsm::SCAN_CRC8;
      break;

    case UartFsm::SCAN_CRC8:
      packet_state.crc8 = byte;
      counter = 0;
      read_state = UartFsm::SCAN_CMD_ID;

    #ifdef ENABLE_CRC_CHECK
      if (packet_state.calc_crc8 != byte) {
        read_state = UartFsm::SCAN_MAGIC;
      }  
    #endif

      break;

    case UartFsm::SCAN_CMD_ID:
      UartUtil::append_byte_le(packet_state.cmd_id, counter, byte);
      if (counter == 2) {
        counter = 0;
        read_state = UartFsm::SCAN_DATA;
      }
      break;
    
    case UartFsm::SCAN_DATA:
      packet_state.body[counter++] = byte;
      if (counter == packet_state.length) {
        counter = 0;
        read_state = UartFsm::SCAN_CRC16;
      }
      break;

    case UartFsm::SCAN_CRC16:
      UartUtil::append_byte_le(packet_state.crc16, counter, byte);
      if (counter == 2) {
        read_state = UartFsm::SCAN_MAGIC;

      #ifdef ENABLE_CRC_CHECK
        if (packet_state.calc_crc16 == packet_state.crc16) {
          return true;
        }
      #else
        return true;
      #endif

      }
      break;
  }

  if (read_state < UartFsm::SCAN_CMD_ID) {
    packet_state.cycle_crc8(byte);
  }

  if (read_state < UartFsm::SCAN_CRC16) {
    packet_state.cycle_crc16(byte);
  }

  return false;
}

// output of the finite state machine is its state
UartRawPacket UartFsm::get_state() {
  return packet_state;
}



/**
* UartSerialHost
*/
#ifdef UART_LINUX_DETECTED

UartSerialHost::UartSerialHost(const char *_port_name, int bitrate, int _loop_freq) {
  loop_freq = _loop_freq;
  cycle_time_micros = 1000000 / _loop_freq;
  transfer_rate_bps = bitrate;
  transfer_rate_bytes = bitrate / 8;
  throttle_limit_bytes = transfer_rate_bytes / _loop_freq;

  // IO CONFIGURATION
  port_name = _port_name;
  
  port = open(port_name, O_RDWR | O_NOCTTY);
  assert(port >= 0);

  FD_ZERO(&port_set);
  FD_SET(port, &port_set);

  struct termios tty;
  struct termios2 tty2;

  assert(tcgetattr(port, &tty) >= 0);

  // "raw" mode
  tty.c_cflag |= (CLOCAL | CREAD); // ignore modem control lines, enable receiver
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8; // 1 "char" = 8 bits
  tty.c_cflag &= ~PARENB; // disable bit parity check
  tty.c_cflag &= ~CSTOPB; // 1 stop bit
  tty.c_cflag &= ~CRTSCTS; // disable HW flow control (RTS)
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST; // disable implementation-based output processing

  // blocking reads
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  assert(tcsetattr(port, TCSANOW, &tty) >= 0);

  /*
  man page: "tcsetattr() returns success if any of the requested changes could 
  successfully be carried out...when making multiple changes...follow this call with a
  further call to tcgetattr to check that all changes have been performed successfully"
  */
  assert(tcgetattr(port, &tty) >= 0);

  // ioctl equivalent to "tc(get/set)attr"
  assert(ioctl(port, TCGETS2, &tty2) >= 0);

  /*
  man page: "If...c_cflag contains the flag BOTHER, then the baud rate is
  stored in the structure members c_ispeed and c_ospeed as integer values"
  */
  tty2.c_cflag &= ~CBAUD;
  tty2.c_cflag |= BOTHER;
  tty2.c_ispeed = bitrate;
  tty2.c_ospeed = bitrate;

  assert(ioctl(port, TCSETS2, &tty2) >= 0);
  assert(ioctl(port, TCGETS2, &tty2) >= 0);

  // ALLOC
  write_buffer = new unsigned char[throttle_limit_bytes];
  write_idx = 0;
  seq = 0;

  stats.total_recv = stats.total_sent = 0;
}

UartSerialHost::~UartSerialHost() {
  delete write_buffer;
}

bool UartSerialHost::write_packet(UartRawPacket packet) {
#ifdef ENABLE_SAFE_WRITE
  if (throttle_limit_bytes - write_idx < packet.get_size()) {
    return false;
  }
#endif

  std::memcpy(write_buffer + write_idx, packet.bytes, packet.get_size());
  write_idx += packet.get_size();
  return true;
}

void UartSerialHost::handle_cycle_io() {
  // flush write queue
  int bytes_remaining = throttle_limit_bytes;
  int expected_response_size = 0;

  write_idx = 0;
  while (!to_write.empty()) {
    UartPacketData packet = to_write.front();
    UartRawPacket raw = UartRawPacket::from_data(packet, static_cast<uint8_t>(seq++ % 255));
    
    if (bytes_remaining >= raw.get_size()) {
      bytes_remaining -= raw.get_size();
      expected_response_size += packet.calc_response_size();
      
      write_packet(raw);
      to_write.pop();
    } else break;
  }

  ssize_t num_written = write(port, write_buffer, write_idx);

  // read
  int bytes_recv = 0;
  while (bytes_recv < expected_response_size) {
 
  #ifdef ENABLE_UART_READ_TIMEOUT
    FD_ZERO(&port_set);
    FD_SET(port, &port_set);
    read_timeout.tv_sec = 0;
    read_timeout.tv_usec = USB_BULK_LATENCY;

    int result = select(port + 1, &port_set, NULL, NULL, &read_timeout);
  
    if (result <= 0) {
      break;
    }
  #endif

    int read_count = read(port, read_buffer, MIN(READ_BUFFER_SIZE, expected_response_size - bytes_recv));
    if (read_count > 0) {
      bytes_recv += read_count;
      for (int i = 0; i < read_count; i++) {
        if (fsm.cycle(read_buffer[i])) {
          to_read.push(fsm.get_state());
        }
      }
    } else break;
  }

  stats.total_sent += num_written;
  stats.total_recv += bytes_recv;
}

int UartSerialHost::reqs_in_queue() {
  return to_write.size();
}

void UartSerialHost::send(const UartPacketData& packet) {
  to_write.push(packet);
}

bool UartSerialHost::recv(UartPacketData& dst) {
  if (!to_read.empty()) {
    UartRawPacket raw = to_read.front();
    dst = UartPacketData::from_raw(raw);
    to_read.pop();
    return true;
  }
  return false;
}

#else // UART_LINUX_DETECTED

/**
* UartSerialClient
*/

UartSerialClient::UartSerialClient(int bitrate, int _loop_freq) {
  loop_freq = _loop_freq;
  cycle_time_micros = 1000000 / _loop_freq;
  transfer_rate_bps = bitrate;
  transfer_rate_bytes = bitrate / 8;
  throttle_limit_bytes = transfer_rate_bytes / _loop_freq;

  Serial.setTimeout(0);
}

bool UartSerialClient::gen_response(UartRawPacket request, UartPacketData& response) {
  UartPacketData data = UartPacketData::from_raw(request);

  switch (data.cmd_id) {
    case UartPacketData::DATA_REQUEST:
      response.cmd_id = UartPacketData::REV_ENCODER;
      response.rev_encoder.sensor_id = data.req.sensor_id;
      response.rev_encoder.angle = 3.14159;
      return true;

    default:
      break;
  }

  return false;
}

void UartSerialClient::handle_cycle_io() {
  UartPacketData response;
  UartRawPacket raw;
  
  int bytes_recv = 0;
  do {
    int num_read = Serial.readBytes(reinterpret_cast<char*>(read_buffer), READ_BUFFER_SIZE);
    if (num_read > 0) {
      bytes_recv += num_read;
      for (int i = 0; i < num_read; i++) {
        if (fsm.cycle(read_buffer[i])) {
          // need to send a response?
          if (gen_response(fsm.get_state(), response)) {
            raw = UartRawPacket::from_data(response, static_cast<uint8_t>(seq++ % 255));
            Serial.write(raw.bytes, raw.get_size());
          }
        }
      }
    } else {
      break;
    }
  } while (bytes_recv < throttle_limit_bytes);
}

#endif // UART_LINUX_DETECTED