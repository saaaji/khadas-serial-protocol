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

#include "UartSerialComms.hpp"

const int TARGET_BITRATE = 10'000000 * 8; // bps
const int MEGA = 1000000; // unit conversions
const int KILO = 1000;
const int LOOP_FREQ = 1000;
const int CYCLE_TIME_MICROS = MEGA / LOOP_FREQ;

int elapsed_micros(const struct timeval st) {
  struct timeval et;
  gettimeofday(&et, NULL);
  return (et.tv_sec - st.tv_sec) * MEGA + (et.tv_usec - st.tv_usec);
}

int main() {
  UartSerialComms comms("/dev/ttyACM0", TARGET_BITRATE, LOOP_FREQ);

  UartPacketData dst;
  UartPacketData test;
  test.cmd_id = UartPacketData::REV_ENCODER;
  test.rev_encoder.angle = 3.14159;
  
  for (int i = 0; i < 500000; i++) {
    comms.send(test);
  }

  printf("WAITING: %d\n", comms.reqs_in_queue());

  struct timeval prog_st; // program start time
  gettimeofday(&prog_st, NULL);

  int recv = 0;
  for (int counter = 0; /* loop forever */; counter++) {
    struct timeval st; // cycle start time
    gettimeofday(&st, NULL);

    comms.handle_cycle_io();
    while (comms.recv(dst)) {
      recv++;
    }

    if (comms.reqs_in_queue() == 0) break;

    // maintain consistent loop frequency
    while (elapsed_micros(st) < CYCLE_TIME_MICROS) {
      continue;
    }
  }

  printf("G: %f\n", elapsed_micros(prog_st) / (float)MEGA);
  printf("r: %d / %d\n", comms.stats.total_recv, comms.stats.total_sent);
}