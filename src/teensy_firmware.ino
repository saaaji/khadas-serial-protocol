/*// SPDX-License-Identifier: CC0-1.0

// Size of outgoing buffer
constexpr size_t  packet_size = 32;

constexpr size_t  packet_words = packet_size / 4;
uint32_t          packet[packet_words];

// Xorshift64* pseudo-random number generator state
static uint64_t  prng_state;

// Return only the upper 32 bits; this passes BigCrunch tests
static inline uint32_t  prng_u32(void)
{
  uint64_t  x = prng_state;
  x ^= x >> 12;
  x ^= x << 25;
  x ^= x >> 27;
  prng_state = x;
  return (x * UINT64_C(2685821657736338717)) >> 32;
}

void setup() {
  // Zero state is invalid, causing only zeroes to be generated.
  prng_state = 0;
}

void loop() {
  if (!Serial) {
    // No serial connection.  Abort anything ongoing.
    prng_state = 0;
  
  } else
  if (prng_state) {
    size_t  n = Serial.availableForWrite() / 4;
#if 0
    while (n-->0) {
      uint32_t  u = prng_u32();
      Serial.write(u & 255);
      u >>= 8;
      Serial.write(u & 255);
      u >>= 8;
      Serial.write(u & 255);
      u >>= 8;
      Serial.write(u & 255);
    }
#else
    if (n > packet_words)
      n = packet_words;
    if (n > 0) {
      for (size_t i = 0; i < n; i++)
        packet[i] = prng_u32();
      Serial.write((char *)packet, n * 4);
    }
#endif
  } else
  if (Serial.available() >= 8) {
    char  buf[8];
    if (Serial.readBytes(buf, 8) == 8) {
      prng_state = ((uint64_t)((unsigned char)buf[0])      )
                 | ((uint64_t)((unsigned char)buf[1]) <<  8)
                 | ((uint64_t)((unsigned char)buf[2]) << 16)
                 | ((uint64_t)((unsigned char)buf[3]) << 24)
                 | ((uint64_t)((unsigned char)buf[4]) << 32)
                 | ((uint64_t)((unsigned char)buf[5]) << 40)
                 | ((uint64_t)((unsigned char)buf[6]) << 48)
                 | ((uint64_t)((unsigned char)buf[7]) << 56);
    }
  }
}*/

void setup() {
  while (!Serial) {
    continue;
  }
}

void loop() {
  // Serial.println(Serial.baud());
  
  int freq = 1000;
  int transfer_speed = 60000000;
  unsigned long cycle_time_us = 1000000 / freq;
  unsigned long start = micros();
  
  int len = 512;
  char buf[len];

  // testing?
  int total_received = 0;
  do {
    int count = Serial.readBytes(buf, len);
    if (count > 0) {
      Serial.write(buf, count);
    }
    total_received += count;
  } while (total_received < (transfer_speed / freq) && 
           (micros() - start) < cycle_time_us);

  // Serial.readBytes(buf, len);

  // int count = 0;
  // while (count < len) {
  //   if (Serial.available()) {
  //     buf[count++] = Serial.read();
  //   }
  // }

  // if (Serial.availableForWrite()) {
  //   Serial.write(buf, len);
  // }

  while ((micros() - start) < cycle_time_us) {
    continue;
  }
}