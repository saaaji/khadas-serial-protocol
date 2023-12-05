#define _DEFAULT_SOURCE
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <cstring>
#define termios asm_termios
#include <asm/termbits.h>
#include <asm/ioctls.h>
#undef termios
#include <cstdlib>
#include <cstdio>
#include <queue>
#include <cassert>

#define MAX_DATA_SIZE 1024

#define ENABLE_SAFE_WRITE
#define ENABLE_CRC_CHECK
#define ENABLE_UART_READ_TIMEOUT

template<typename T>
void append_byte_le(T& n, int& ord, const uint8_t& byte) {
  n |= byte << (8 * ord++);
}

template<typename T>
T decode(const uint8_t*& data, int& offset) {
  T res = 0;
  size_t n = sizeof(T);

  for (int i = 0; i < n; i++) {
    uint8_t byte = data[offset + i];
    res |= (byte << i * 8);
  }
  return res;
}

template<typename T>
void encode(unsigned char* data, int& write_idx, T val) {
  size_t n = sizeof(T);

  if (n == 1) {
    data[write_idx++] = static_cast<unsigned char>(val);
  }

  for (int i = 0; i < n; i++) {
    unsigned char byte = (val >> i * 8) & 0xff;
    data[write_idx++] = byte;
  }
}

struct UartRawPacket;

struct UartPacketData {
  enum CmdId: uint16_t {
    DATA_REQUEST = 0x0ff,
    DR16         = 0x0103,
    REV_ENCODER  = 0x0104,
    ISM          = 0x0105
  };

  CmdId cmd_id;
  union {
    struct {
      float angle;
    } rev_encoder;
    // ...
  };

  static int calc_data_response_size(CmdId cmd_id);
  static UartPacketData from_raw(const UartRawPacket& raw);
};

struct UartRawPacket {
  uint8_t bytes[MAX_DATA_SIZE];
  uint16_t length;
  uint8_t seq;
  uint8_t crc8, calc_crc8;
  uint16_t cmd_id;
  uint8_t *body;
  uint16_t crc16, calc_crc16;

  uint8_t reset_crc8();
  uint8_t cycle_crc8(const uint8_t& byte);
  uint16_t reset_crc16();
  uint16_t cycle_crc16(const uint8_t& byte);
  void clear();
  int get_size();

  static UartRawPacket from_data(const UartPacketData& data, const uint8_t& seq);
};

class UartFsm {
  private:
    enum State {    
      SCAN_MAGIC,
      SCAN_LENGTH,
      SCAN_SEQ,
      SCAN_CRC8,
      SCAN_CMD_ID,
      SCAN_DATA,
      SCAN_CRC16,
    };

    int counter;
    UartRawPacket packet_state;
    State read_state;

    bool cycle(const uint8_t& byte);

  public:
    static const uint8_t MAGIC = 0xa5;

    UartFsm();
    ~UartFsm();

    UartRawPacket get_state();
};

class UartSerialComms {
  private:
    static const int USB_BULK_LATENCY = 15000 // micros
    static const int READ_BUFFER_SIZE = 4095; // bytes

    // IO
    const char *port_name;
    int port;
    fd_set port_set;

    int loop_freq;
    int transfer_rate_bps, transfer_rate_bytes, throttle_limit_bytes;

    struct timeval read_timeout;
    unsigned char read_buffer[READ_BUFFER_SIZE];
    unsigned char *write_buffer;
    int write_idx;

    // parsing
    UartFsm fsm;
    UartSerde usd;

    // API
    int seq;
    std::queue<UartRawPacket> to_read;
    std::queue<UartRawPacket> to_write;

    bool write_packet(UartRawPacket packet);

  public:
    UartSerialComms(const char *port_name);
    ~UartSerialComms();

    void handle_cycle_io();
    void send(const UartPacketData& packet);
    bool recv(UartPacketData& dst);
};