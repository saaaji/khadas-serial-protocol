/**
* UartPacketData
*/
int UartPacketData::calc_data_response_size(UartPacketData::CmdId cmd_id) {
  switch (cmd_id) {
    case REV_ENCODER:
      return 4;
  }
  return 0;
}

UartPacketData UartPacketData::from_raw(const UartRawPacket& raw, uint8_t seq) {
  UartPacketData data;
  int read_idx = 0;

  data.cmd_id = static_cast<UartPacketData::CmdId>(raw.cmd_id);

  union {
    uint32_t u32;
    float f32;
  } bitcaster;

  switch (data.cmd_id) {
    case UartPacketData::REV_ENCODER:
      bitcaster.u32 = decode<uint32_t>(raw.data, read_idx);
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
  // TODO: crc8
  return calc_crc8;
}

uint16_t UartRawPacket::reset_crc16() {
  calc_crc16 = 0xffff;
  return calc_crc16;
}

uint16_t UartRawPacket::cycle_crc16(const uint8_t& byte) {
  // TODO: crc16
  return calc_crc16;
}

void UartRawPacket::clear() {
  reset_crc8();
  reset_crc16();

  length = 0;
  cmd_id = 0;
  crc16 = 0;
}

UartRawPacket UartRawPacket::from_data(const UartPacketData& data, const uint8_t& seq) {
  UartRawPacket raw;
  raw.body = &raw.bytes[5];
  int write_idx = 0;

  raw.cmd_id = static_cast<uint16_t>(data.cmd_id);
  raw.length = UartPacketData::calc_data_response_size(data.cmd_id);

  encode<uint8_t>(raw.bytes, write_idx, UartFsm::MAGIC);
  encode<uint16_t>(raw.bytes, write_idx, raw.length);
  encode<uint8_t>(raw.bytes, write_idx, seq);

  raw.reset_crc8();
  for (int i = 0; i < 4; i++) {
    raw.cycle_crc8(raw.bytes[i]);
  }

  encode<uint8_t>(raw.bytes, write_idx, raw.calc_crc8);
  encode<uint16_t>(raw.bytes, write_idx, raw.cmd_id);

  union {
    uint32_t u32;
    float f32;
  } bitcaster;

  switch (data.cmd_id) {
    case UartPacketData::REV_ENCODER:
      bitcaster.f32 = data.rev_encoder.angle;
      encode<uint32_t>(raw.bytes, write_idx, bitcaster.u32);
      break;
  }

  raw.reset_crc16();
  for (int i = 0; i < write_idx; i++) {
    raw.cycle_crc16(raw.bytes[i]);
  }

  encode<uint16_t>(raw.bytes, write_idx, raw.calc_crc16);
  return raw;
}

int UartRawPacket::get_size() {
  return 7 + length;
}

/**
* UartFsm
*/
UartFsm::UartFsm() {
  read_state = UartFsm::SCAN_MAGIC;
}

bool UartFsm::cycle(const uint8_t& byte) {
  switch (read_state) {
    case UartFsm::SCAN_MAGIC:
      if (byte == MAGIC) {
        packet_state.clear();
        counter = 0;
        read_state = UartFsm::SCAN_LENGTH;
      }
      break;

    case UartFsm::SCAN_LENGTH:
      append_byte_le(packet_state.length, counter, byte);
      if (counter == 2) {
        read_state = UartFsm::SCAN_SEQ;
      }
      break;

    case UartFsm::SCAN_SEQ:
      packet_state.seq = byte;
      read_state = UartFsm::SCAN_CRC8;
      break;

    case UartFsm::SCAN_CRC8:
      packet_state.crc8 = byte;
      counter = 0;
      state = UartFsm::SCAN_CMD_ID;

    #ifdef ENABLE_CRC_CHECK
      if (packet_state.calc_crc8 != byte) {
        read_state = UartFsm::SCAN_MAGIC;
      }  
    #endif

      break;

    case UartFsm::SCAN_CMD_ID:
      append_byte_le(packet_state.cmd_id, counter, byte);
      if (counter == 2) {
        counter = 0;
        read_state = UartFsm::SCAN_DATA;
      }
      break;
    
    case UartFsm::SCAN_DATA:
      packet_state.data[counter++] = byte;
      if (counter == packet_state.length) {
        counter = 0;
        read_state = UartFsm::SCAN_CRC16;
      }
      break;

    case UartFsm::SCAN_CRC16:
      append_byte_le(packet_state.crc16, counter, byte);
      if (counter == 2) {
      
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
    packet_state.cycle_crc8();
  }

  if (read_state < UartFsm::SCAN_CRC16) {
    packet_state.cycle_crc16();
  }

  return false;
}

// output of the finite state machine is its state
UartRawPacket UartFsm::get_state() {
  return packet_state;
}

/**
* UartSerialComms
*/
UartSerialComms::UartSerialComms(const char *_port_name, int bitrate, int _loop_freq) {
  loop_freq = _loop_freq;
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
  struct termios2 tty;

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
  assert(ioctl(port, TCGETS2, &tty) >= 0);

  /*
  man page: "If...c_cflag contains the flag BOTHER, then the baud rate is
  stored in the structure members c_ispeed and c_ospeed as integer values"
  */
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ispeed = bitrate;
  tty.c_ospeed = bitrate;

  assert(ioctl(port, TCSETS2, &tty) >= 0);
  assert(ioctl(port, TCGETS2, &tty) >= 0);

  // ALLOC
  write_buffer = new unsigned char[throttle_limit_bytes];
  write_idx = 0;
  seq = 0;
}

UartSerialComms::~UartSerialComms() {
  delete write_buffer;
}

bool UartSerialComms::write_packet(UartRawPacket packet) {
#ifdef ENABLE_SAFE_WRITE
  if (throttle_limit_bytes - write_idx < packet.get_size()) {
    return false;
  }
#endif

  std::memcpy(write_buffer, packet.bytes, packet.get_size());
  write_idx += packet.get_size();
  return true;
}

void UartSerialComms::handle_cycle_io() {
  // TODO
}

void UartSerialComms::send(const UartPacketData& packet) {
  UartRawPacket raw = UartRawPacket::from_data(packet);
  to_write.push(raw);
}

bool UartSerialComms::recv(UartPacketData& dst) {
  if (!to_read.empty()) {
    UartRawPacket raw = to_read.front();
    dst = UartPacketData::from_raw(raw);
    to_read.pop();
    return true;
  }
  return false;
}