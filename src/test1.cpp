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
  UartSerialHost host("/dev/ttyACM0", TARGET_BITRATE, LOOP_FREQ);

  UartPacketData dst;
  UartPacketData test;
  test.cmd_id = UartPacketData::DATA_REQUEST;
  test.req.cmd_id = static_cast<uint16_t>(UartPacketData::REV_ENCODER);
  test.req.sensor_id = 1;

  int z = 1000000;
  
  for (int i = 0; i < z; i++) {
    host.send(test);
  }

  printf("waiting on %d packets...\n", host.reqs_in_queue());

  struct timeval prog_st; // program start time
  gettimeofday(&prog_st, NULL);

  int recv = 0;
  for (int counter = 0; /* loop forever */; counter++) {
    struct timeval st; // cycle start time
    gettimeofday(&st, NULL);

    host.handle_cycle_io();
    while (host.recv(dst)) {
      // printf("ANG: %f\n", dst.rev_encoder.angle);
      recv++;
    }

    if (host.reqs_in_queue() == 0) break;

    // maintain consistent loop frequency
    while (elapsed_micros(st) < CYCLE_TIME_MICROS) {
      continue;
    }
  }

  float elapsed_s = elapsed_micros(prog_st) / (float) MEGA;
  printf("transfer time: %f\n", elapsed_s);
  printf("bitrate (R/W, Mbps): %0.2f\t%0.2f\n",
              (host.stats.total_recv * 8.0) / MEGA / elapsed_s,
              (host.stats.total_sent * 8.0) / MEGA / elapsed_s);
  printf("packets sent/recv: %d / %d\n", z, recv);
  printf("bytes sent/recv: %d / %d\n", host.stats.total_sent, host.stats.total_recv);
}