#define _DEFAULT_SOURCE
#include <cstdlib>
#include <stdint.h> // integer types
#include <cstdio> // printf and family
#include <fcntl.h> // file IO
#include <unistd.h> // more file IO
#include <termios.h> // serial config
#include <errno.h> // error tracking
#include <sys/ioctl.h>
#include <sys/time.h>
#include <signal.h>
#include <string.h>
#include <queue>
#define termios asm_termios
#include <asm/termbits.h>
#include <asm/ioctls.h>
#undef termios

// constants
#define MAX_DATA_SIZE 1024

const int TARGET_BITRATE = 1000000 * 8; // bps
const int MEGA = 1000000; // unit conversions
const int KILO = 1000;
const int USB_BULK_LATENCY = 15 /* ms */ * KILO; // micros
const int LOOP_FREQ = 1000;
const int CYCLE_TIME_MICROS = MEGA / LOOP_FREQ;
const int TRANSFER_SPEED_BYTES = TARGET_BITRATE / 8; // bytes/s
const int BITRATE_THROTTLE_LIMIT = TRANSFER_SPEED_BYTES / LOOP_FREQ; // bytes/cycle
const int READ_BUFFER_SIZE = 4095; // bytes
const int MAGIC = 0xa5;

int elapsed_micros(const struct timeval st) {
  struct timeval et;
  gettimeofday(&et, NULL);
  return (et.tv_sec - st.tv_sec) * MEGA + (et.tv_usec - st.tv_usec);
}

#define SECONDS(st) (elapsed_micros(st) / (float) MEGA)
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// check errno when return is -1
int init_uart(int port) {
  struct termios tty;

  if (tcgetattr(port, &tty) < 0) {
    return -1;
  }

  // "raw" mode
  tty.c_cflag |= (CLOCAL | CREAD); // ignore modem control lines, enable receiver
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8; // 1 "char" = 8 bits
  tty.c_cflag &= ~PARENB; // disable bit parity check
  tty.c_cflag &= ~CSTOPB; // 1 stop bit
  // tty.c_cflag &= ~CRTSCTS; // disable HW flow control (RTS)
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST; // disable implementation-based output processing

  // blocking reads
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(port, TCSANOW, &tty) < 0) {
    return -1;
  }

  /*
  man page: "tcsetattr() returns success if any of the requested changes could 
  successfully be carried out...when making multiple changes...follow this call with a
  further call to tcgetattr to check that all changes have been performed successfully"
  */
  if (tcgetattr(port, &tty) < 0) {
    return -1;
  }

  return 0;
}

// if return value is less than 0, check errno
// https://man7.org/linux/man-pages/man2/ioctl_tty.2.html
int config_uart_bitrate(int port, int bitrate) {
  struct termios2 tty;

  // ioctl equivalent to "tc(get/set)attr"
  if (ioctl(port, TCGETS2, &tty) < 0) {
    return -1;
  }

  /*
  man page: "If...c_cflag contains the flag BOTHER, then the baud rate is
  stored in the structure members c_ispeed and c_ospeed as integer values"
  */
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ispeed = bitrate;
  tty.c_ospeed = bitrate;

  if (ioctl(port, TCSETS2, &tty) < 0) {
    return -1;
  }

  if (ioctl(port, TCGETS2, &tty) < 0) {
    return -1;
  }

  return 0;
}

// create global write buffer pointer so that exit handler can free memory
unsigned char *write_buffer = NULL;

void cleanup() {
  printf("cleaning write buffer...\n");
  free(write_buffer);
  write_buffer = NULL;
  exit(EXIT_SUCCESS);
}

/*
the state machine
*/
typedef enum {
  CMD_DATA_REQUEST    = 0x00ff,
  CMD_KINEMATIC_STATE = 0x0001,
  CMD_ROBOT_STATE     = 0x0002,
  CMD_CLIENT_DRAW     = 0x0003,
  CMD_ESTIMATOR_STATE = 0x0101,
  CMD_MOTOR_FEEDBACK  = 0x0102,
  CMD_DR16            = 0x0103,
  CMD_REV_ENCODER     = 0x0104,
  CMD_ISM             = 0x0105,
  CMD_REF_SYS         = 0x0107
} UartCmdId;

const UartCmdId NORM_CMD[] = { 
  CMD_DATA_REQUEST,
  CMD_KINEMATIC_STATE,
  CMD_ROBOT_STATE,
  CMD_CLIENT_DRAW,
  CMD_ESTIMATOR_STATE,
  CMD_MOTOR_FEEDBACK,
  CMD_DR16,
  CMD_REV_ENCODER,
  CMD_ISM,
  CMD_REF_SYS
};

const int RESPONSE_SIZE_LOOKUP[] = {
  0, // DATA_REQUEST
  0, // KINEMATIC_STATE
  0, // ROBOT_STATE
  0, // CLIENT_DRAW
  0, // ESTIMATOR_STATE
  0, // MOTOR_FEEDBACK
  28, // DR16
  0, // REV_ENCODER
  0, // ISM
  0 // REF_SYS
};

typedef enum {
  SCAN_MAGIC,
  SCAN_LENGTH,
  SCAN_SEQ,
  SCAN_CRC8,
  SCAN_CMD_ID,
  SCAN_DATA,
  SCAN_CRC16,
} UartState;

// state machine packet state
typedef struct {
  uint16_t length;
  uint8_t seq;
  uint8_t crc8, calc_crc8;
  uint16_t cmd_id;
  uint8_t data[MAX_DATA_SIZE];
  uint16_t crc16, calc_crc16;
} UartRawPacket;

// packet
typedef struct {
  uint16_t cmd_id;
  struct {
    float angle;
  } rev_encoder;
} UartPacket;



typedef struct {
  int counter;
  UartRawPacket packet;
  UartState state;
} UartFsm;

void fsm_init(UartFsm *fsm) {
  fsm->state = SCAN_MAGIC;
}

void reset_crc8(uint8_t *crc8) {
  *crc8 = 0xff;
}

void reset_crc16(uint16_t *crc16) {
  *crc16 = 0xffff;
}

void cycle_crc8(uint8_t *crc8, const uint8_t next_byte) {}
void cycle_crc16(uint16_t *crc16, const uint8_t next_byte) {}

void fsm_cycle(UartFsm *fsm, uint8_t byte) {
  switch (fsm->state) {
    case SCAN_MAGIC:
      if (byte == MAGIC) {
        // reset checksums
        reset_crc8(&fsm->packet.calc_crc8);
        reset_crc16(&fsm->packet.calc_crc16);

        fsm->packet.length = 0;
        fsm->packet.cmd_id = 0;
        fsm->packet.crc16 = 0;

        fsm->counter = 0;
        fsm->state = SCAN_LENGTH;
      }
      break;

    case SCAN_LENGTH:
      fsm->packet.length |= byte << (fsm->counter++ * 8);
      if (fsm->counter == 2) {
        fsm->state = SCAN_SEQ;
      }
      break;

    case SCAN_SEQ:
      fsm->packet.seq = byte;
      fsm->state = SCAN_CRC8;
      break;

    case SCAN_CRC8:
      fsm->packet.crc8 = byte;
      fsm->counter = 0;
      fsm->state = SCAN_CMD_ID;

      #ifdef CRC_CHECK
      if (fsm->packet.calc_crc8 != byte) {
        fsm->state = SCAN_MAGIC;
      }
      #endif // CRC_CHECK

      break;

    case SCAN_CMD_ID:
      fsm->packet.cmd_id |= byte << (fsm->counter++ * 8);
      if (fsm->counter == 2) {
        fsm->counter = 0;
        fsm->state = SCAN_DATA;
      }
      break;

    case SCAN_DATA:
      fsm->packet.data[fsm->counter++] = byte;
      if (fsm->counter == fsm->packet.length) {
        fsm->counter = 0;
        fsm->state = SCAN_CRC16;
      }
      break;

    case SCAN_CRC16:
      fsm->packet.crc16 |= byte << (fsm->counter++ * 8);
      if (fsm->counter == 2) {
        // print
        /*printf(
          "UartRawPacket\n"
          "\tlen: %u\n"
          "\tseq: %u\n"
          "\tcrc8: %u\n"
          "\tcmd_id: %#04x\n"
          "\tcrc16: %u\n",
          fsm->packet.length,
          fsm->packet.seq,
          fsm->packet.crc8,
          fsm->packet.cmd_id,
          fsm->packet.crc16
        );*/
        
        if (fsm->packet.calc_crc16 == fsm->packet.crc16) {
          // ENQUEUE  
        }

        fsm->state = SCAN_MAGIC; // find next packet
      }
      break;
  }

  // update checksums
  if (fsm->state < SCAN_CMD_ID) {
    cycle_crc8(&fsm->packet.calc_crc8, byte);
  }

  if (fsm->state < SCAN_CRC16) {
    cycle_crc16(&fsm->packet.calc_crc16, byte);
  }
}

int main(int argc, char **argv) {
  printf(
    "stats:\n"
    "\tfreq: %d\n"
    "\ttarget (bps): %d (%d byte/s)\n"
    "\tthrottle limit (byte/s): %d\n",
    LOOP_FREQ,
    TARGET_BITRATE,
    TRANSFER_SPEED_BYTES,
    BITRATE_THROTTLE_LIMIT
  );

  // initialize UART port
  int port = open(argv[1], O_RDWR | O_NOCTTY);
  fd_set port_set;
  FD_ZERO(&port_set);
  FD_SET(port, &port_set);

  if (port < 0) {
    printf("could not open port \"%s\": %d\n", argv[1], errno);
    exit(EXIT_FAILURE);
  }

  if (init_uart(port) < 0) {
    printf("could not configure UART settings: %d\n", errno);
    exit(EXIT_FAILURE);
  }

  if (config_uart_bitrate(port, TARGET_BITRATE) < 0) {
    printf("could not configure UART bitrate: %d\n", errno);
    exit(EXIT_FAILURE);
  }

  // create exit handler to free write buffer
  struct sigaction act;
  memset(&act, 0, sizeof(act));
  act.sa_handler = reinterpret_cast<void (*)(int)>(cleanup);
  act.sa_flags = 0;
  if (sigaction(SIGINT, &act, NULL) < 0) {
    printf("could not configure cleanup handler: %d\n", errno);
  }
  
  // dummy packet
  unsigned char dummy_packet[] = {
    MAGIC, // magic
    6, 0, // length
    0, // seq
    1, // crc8
    10, 0, // cmd_id
    0, 0, 0, 0, 0, 0, // data 
    2, 0 // crc16
  };
  int packet_len = 15;

  // FSM
  UartFsm fsm;
  fsm_init(&fsm);
  
  // read and write buffers (max read of 4905 bytes on Linux)
  unsigned char read_buffer[READ_BUFFER_SIZE];
  write_buffer = (unsigned char *) calloc(sizeof(unsigned char), BITRATE_THROTTLE_LIMIT);
  int write_idx = 0;

  // read timeout
  struct timeval read_timeout;
  read_timeout.tv_sec = 0;
  read_timeout.tv_usec = USB_BULK_LATENCY;

  // bitrate statistics
  int total_sent = 0;
  int total_recv = 0;
  
  // main loop
  struct timeval prog_st; // program start time
  gettimeofday(&prog_st, NULL);

  for (int counter = 0; /* loop forever */; counter++) {
    struct timeval st; // cycle start time
    gettimeofday(&st, NULL);

    write_idx = 0; // reset write index
    int num_packets = BITRATE_THROTTLE_LIMIT / packet_len; // throttle the serial port bitrate
    // int expected_response_size = 

    for (int i = 0; i < num_packets; i++) {
      // copy dummy packet into write buffer
      for (int j = 0; j < packet_len; j++) {
        write_buffer[write_idx++] = dummy_packet[j];
      }
    }

    // push to USB driver
    ssize_t num_written = write(port, write_buffer, write_idx);
    if (num_written < 0) {
      printf("could not write to serial port: %d\n", errno);
      exit(EXIT_FAILURE);
    }
    printf("np: %d, wi: %d, #w: %ld\n", num_packets, write_idx, num_written);

    // a specified payload is expected in response, so read until it is received
    int num_recv = 0;
    while (num_recv < num_written) {
      #define READ_TIMEOUT_ABORT_LOSS
      #ifdef READ_TIMEOUT_ABORT_LOSS

      /*
      man page (https://man7.org/linux/man-pages/man2/select.2.html)
      nfds "...should be set to the highest-numbered file descriptor
      in any of the three sets, plus 1"
      */

      // give sys call enough time to read data (account USB bulk transfer latency... ~10-20ms)
      int result = select(port + 1, &port_set, NULL, NULL, &read_timeout);
      
      // must reset timeout after select call
      FD_ZERO(&port_set);
      FD_SET(port, &port_set);
      read_timeout.tv_sec = 0;
      read_timeout.tv_usec = USB_BULK_LATENCY; // needs tweaking

      if (result <= 0) {
        printf("(!) killing read loop...\n");
        break;
      }

      #endif // READ_TIMEOUT_ABORT_LOSS

      // we want to read however many bytes that have been sent that have not been read back again
      int read_count = read(port, read_buffer, MIN(READ_BUFFER_SIZE, num_written - num_recv));

      if (read_count > 0) {
        num_recv += read_count;

        for (int i = 0; i < read_count; i++) {
          fsm_cycle(&fsm, read_buffer[i]);
        }
      } else {
        printf("could not read serial port: %d\n", errno);
        exit(EXIT_FAILURE);
      }
    }

    // bitrate metrics
    total_sent += num_written;
    total_recv += num_recv;
    
    if (counter % 1 == 0) {
      float elapsed_s = elapsed_micros(prog_st) / MEGA;
      printf(
        "THROUGHPUT: %.5fMbps R / %.5fMbps W\n", 
        (total_recv * 8.0) / MEGA / elapsed_s,
        (total_sent * 8.0) / MEGA / elapsed_s
      );
    }

    // maintain consistent loop frequency
    while (elapsed_micros(st) < CYCLE_TIME_MICROS) {
      continue;
    }
  }

  return 0;
}