

// SPDX-License-Identifier: CC0-1.0
// gcc -Wall -O2 this.c -lrt -o this && ./this /dev/ttyACM0
#define  _POSIX_C_SOURCE  200809L
#define  _DEFAULT_SOURCE
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

/* Input buffer size.  Use a power of two larger than 512; I'd recommend 4096 - 65536.
*/
#ifndef  BUFFER_SIZE
#define  BUFFER_SIZE  512
#endif

/* Units of MB: 1000000 or 1048576
*/
#ifndef  MEGA
#define  MEGA  1000000
#endif

/* Xorshift64* pseudo-random number generator.  Zero state is invalid.
*/
static uint64_t  prng_state = 0;

/* Return only the upper 32 bits of XorShift64*. This passes BigCrunch tests.
*/
static inline uint32_t  prng_u32(void)
{
  uint64_t  x = prng_state;
  x ^= x >> 12;
  x ^= x << 25;
  x ^= x >> 27;
  prng_state = x;
  return (x * UINT64_C(2685821657736338717)) >> 32;
}

/* Generate a 64-bit seed for the pseudo-random number generator.
*/
static uint64_t  prng_seed(void)
{
    struct timespec  now, boot;
    pid_t            pid;
    uint64_t         seed;

    pid = getpid();

    do {
        clock_gettime(CLOCK_REALTIME, &now);
        clock_gettime(CLOCK_BOOTTIME, &boot);

        seed = (uint64_t)pid * UINT64_C(21272683)
             + (uint64_t)now.tv_sec * UINT64_C(113090255381)
             + (uint64_t)now.tv_nsec * UINT64_C(2470424306258081)
             + (uint64_t)boot.tv_sec * UINT64_C(9526198845596401)
             + (uint64_t)boot.tv_nsec * UINT64_C(9454161880954729);
    } while (!seed);

    prng_state = seed;
    return seed;
}

/* When 'done' becomes nonzero, it is time to stop the measurement.
*/
static volatile sig_atomic_t  done = 0;

/* Signal handler for 'done'.
*/
static void handle_done(int signum)
{
    // Silence unused variable warning; generates no code.
    (void)signum;

    done = 1;
}

static int install_done(int signum)
{
    struct sigaction  act;
    memset(&act, 0, sizeof act);
    sigemptyset(&act.sa_mask);
    act.sa_handler = handle_done;
    act.sa_flags = 0;  // Specifically, NO SA_RESTART flag.
    return sigaction(signum, &act, NULL);
}

/* One-second interval timer.  Simply sets 'update' to nonzero.
*/
#ifndef  UPDATE_SIGNAL
#define  UPDATE_SIGNAL  (SIGRTMIN+0)
#endif

static timer_t                update_timer;
static volatile sig_atomic_t  update = 0;

static void handle_update(int signum)
{
    (void)signum;
    update = 1;
}

static int install_update(void)
{
    struct itimerspec spec;
    struct sigevent   ev;
    struct sigaction  act;

    memset(&act, 0, sizeof act);
    sigemptyset(&act.sa_mask);
    act.sa_handler = handle_update;
    act.sa_flags = SA_RESTART;
    if (sigaction(UPDATE_SIGNAL, &act, NULL) == -1)
        return -1;

    ev.sigev_notify = SIGEV_SIGNAL;
    ev.sigev_signo  = UPDATE_SIGNAL;
    ev.sigev_value.sival_ptr = NULL;
    if (timer_create(CLOCK_BOOTTIME, &ev, &update_timer) == -1)
        return -1;

    spec.it_value.tv_sec = 1;       // One second to first update
    spec.it_value.tv_nsec = 0;
    spec.it_interval.tv_sec = 1;    // Repeat at one second intervals
    spec.it_interval.tv_nsec = 0;
    if (timer_settime(update_timer, 0, &spec, NULL) == -1)
        return -1;

    return 0;
}

/* USB serial port device handling.
*/
static struct termios   tty_settings;
static int              tty_descriptor = -1;
static const char      *tty_path = NULL;

static void tty_cleanup(void)
{
    if (tty_descriptor != -1) {
        if (tcsetattr(tty_descriptor, TCSANOW, &tty_settings) == -1)
            fprintf(stderr, "Warning: %s: Cannot reset original termios settings: %s.\n", tty_path, strerror(errno));

        tcflush(tty_descriptor, TCIOFLUSH);

        if (close(tty_descriptor) == -1)
            fprintf(stderr, "Warning: %s: Error closing device: %s.\n", tty_path, strerror(errno));

        tty_descriptor = -1;
    }
}

static int tty_open(const char *path)
{
    struct termios  raw;
    int             fd;

    // NULL or empty path is invalid, and yields "no such file or directory" error.
    if (!path || !*path) {
        errno = ENOENT;
        return -1;
    }

    // Fail if tty is already open.
    if (tty_descriptor != -1) {
        errno = EALREADY;
        return -1;
    }

    // Open the tty device.
    do {
        fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
    } while (fd == -1 && errno == EINTR);
    if (fd == -1)
        return -1;

    // Set exclusive mode, so that others cannot open the device while we have it open.
    if (ioctl(fd, TIOCEXCL) == -1)
        fprintf(stderr, "Warning: %s: Cannot get exclusive access on tty device: %s.\n", path, strerror(errno));

    // Drop any already pending data.
    tcflush(fd, TCIOFLUSH);

    // Obtain current termios settings.
    if (tcgetattr(fd, &raw) == -1 || tcgetattr(fd, &tty_settings) == -1) {
        fprintf(stderr, "%s: Cannot get termios settings: %s.\n", path, strerror(errno));
        close(fd);
        errno = 0; // Already reported
        return -1;
    }

    // Raw 8-bit mode: no post-processing or special characters, 8-bit data.
    raw.c_iflag &= ~( IGNBRK | BRKINT | PARMRK | INPCK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IUCLC | IUTF8 );
    raw.c_oflag &= ~( OPOST );
    raw.c_lflag &= ~( ECHO | ECHONL | ICANON | ISIG | IEXTEN );
    raw.c_cflag &= ~( CSIZE | PARENB | CLOCAL );
    raw.c_cflag |= CS8 | CREAD | HUPCL;
    // Blocking reads.
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &raw) == -1) {
        fprintf(stderr, "%s: Cannot set termios settings: %s.\n", path, strerror(errno));
        close(fd);
        errno = 0; // Already reported
        return -1;
    }

    // Drop any already pending data, again.  Just to make sure.
    tcflush(fd, TCIOFLUSH);

    // Everything seems to be in order.  Update state for tty_cleanup(), and return success.
    tty_descriptor = fd;
    tty_path = path;
    return 0;
}

static inline double  seconds_between(const struct timespec after, const struct timespec before)
{
    return (double)(after.tv_sec - before.tv_sec)
         + (double)(after.tv_nsec - before.tv_nsec) / 1000000000.0;
}

int main(int argc, char *argv[])
{
    const size_t     buffer_size = BUFFER_SIZE;
    size_t           buffer_have = 0;
    unsigned char   *buffer_data = NULL;
    struct timespec  started, mark;
    uint64_t         received_before = 0;   // Received till mark
    uint64_t         received = 0;          // Received after mark
    uint64_t         sequence = 0;
    uint64_t         seed;

    if (argc != 2 || !strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
        const char *arg0 = (argc > 0 && argv && argv[0] && argv[0][0] != '\0') ? argv[0] : "(this)";
        fprintf(stderr, "\n");
        fprintf(stderr, "Usage: %s [ -h | --help ]\n", arg0);
        fprintf(stderr, "       %s DEVICE\n", arg0);
        fprintf(stderr, "\n");
        fprintf(stderr, "This writes a 64-bit XorShift64* seed to DEVICE, then reads the\n");
        fprintf(stderr, "sequence of the generated numbers, 32 high bits each, verifying\n");
        fprintf(stderr, "and reporting the transfer rate.  Press CTRL+C or send SIGHUP,\n");
        fprintf(stderr, "SIGINT, or SIGTERM signal to exit.\n");
        fprintf(stderr, "\n");
        return (argc == 1 || argc == 2) ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    if (install_done(SIGHUP) ||
        install_done(SIGINT) ||
        install_done(SIGTERM)) {
        fprintf(stderr, "Cannot install signal handlers: %s.\n", strerror(errno));
        return EXIT_FAILURE;
    }

    buffer_data = malloc(buffer_size + 4);
    if (!buffer_data) {
        fprintf(stderr, "Not enough memory available for a %zu-byte input buffer.\n", buffer_size);
        return EXIT_FAILURE;
    }

    seed = prng_seed();

    if (tty_open(argv[1])) {
        if (errno)
            fprintf(stderr, "%s: Cannot open device: %s.\n", argv[1], strerror(errno));
        return EXIT_FAILURE;
    }

    {
        unsigned char  request[8] = {   seed        & 255,
                                       (seed >>  8) & 255,
                                       (seed >> 16) & 255,
                                       (seed >> 24) & 255,
                                       (seed >> 32) & 255,
                                       (seed >> 40) & 255,
                                       (seed >> 48) & 255,
                                       (seed >> 56) & 255 };
        const unsigned char *const q = request + 8;
        const unsigned char       *p = request;
        ssize_t                    n;

        while (p < q) {
            n = write(tty_descriptor, p, (size_t)(q - p));
            if (n > 0) {
                p += n;
            } else
            if (n != -1) {
                fprintf(stderr, "%s: Invalid write (%zd)\n", tty_path, n);
                break;
            } else
            if (errno != EINTR) {
                fprintf(stderr, "%s: Write error: %s.\n", tty_path, strerror(errno));
                break;
            }
        }

        if (p != q) {
            tty_cleanup();
            return EXIT_FAILURE;
        }
    }

    if (install_update()) {
        fprintf(stderr, "Cannot create a periodic update signal: %s.\n", strerror(errno));
        tty_cleanup();
        return EXIT_FAILURE;
    }

    if (clock_gettime(CLOCK_BOOTTIME, &started) == -1) {
        fprintf(stderr, "Cannot read BOOTTIME clock: %s.\n", strerror(errno));
        tty_cleanup();
        return EXIT_FAILURE;
    } else
        mark = started;

    while (!done) {
        if (update) {
            struct timespec  now;

            if (clock_gettime(CLOCK_BOOTTIME, &now) == -1) {
                fprintf(stderr, "Cannot read BOOTTIME clock: %s.\n", strerror(errno));
                tty_cleanup();
                return EXIT_FAILURE;
            }

            const double  sec_last = seconds_between(now, mark);
            const double  mib_last = (double)received / (double)(MEGA);
            const double  sec_total = seconds_between(now, started);
            const double  mib_total = (double)(received + received_before) / (double)(MEGA);

            if (sec_last > 0.0 && sec_total > 0.0) {
                printf("%.3f MB in %.0f seconds (%.3f MB/s on average); %.3f MB in last %.3f seconds (%.3f MB/s); %" PRIu64 " numbers verified\n",
                       mib_total, sec_total, mib_total/sec_total,
                       mib_last, sec_last, mib_last/sec_last,
                       sequence);
                fflush(stdout);
            }

            received_before += received;
            received = 0;
            mark = now;
            update = 0;
        }

        // Receive more data?
        if (buffer_have < buffer_size) {
            ssize_t  n = read(tty_descriptor, buffer_data + buffer_have, buffer_size - buffer_have);
            if (n > 0) {
                buffer_have += n;
                received += n;
            } else
            if (n != -1) {
                fprintf(stderr, "%s: Unexpected read error (%zd).\n", tty_path, n);
                tty_cleanup();
                return EXIT_FAILURE;
            } else
            if (errno != EINTR) {
                fprintf(stderr, "%s: Read error: %s.\n", tty_path, strerror(errno));
                tty_cleanup();
                return EXIT_FAILURE;
            }
        }

        // Verify all full words thus far received.
        if (buffer_have > 3) {
            const unsigned char       *next = buffer_data;
            const unsigned char *const ends = buffer_data + buffer_have;

            while (next + 4 <= ends) {
                uint32_t  u =  (uint32_t)(next[0])
                            | ((uint32_t)(next[1]) << 8)
                            | ((uint32_t)(next[2]) << 16)
                            | ((uint32_t)(next[3]) << 24);
                if (u == prng_u32()) {
                    sequence++;
                    next += 4;
                } else {
                    fprintf(stderr, "Data mismatch at %" PRIu64 ". generated number.\n", sequence + 1);
                    tty_cleanup();
                    return EXIT_FAILURE;
                }
            }

            if (next < ends) {
                memmove(buffer_data, next, (size_t)(ends - next));
                buffer_have = (size_t)(ends - next);
            } else {
                buffer_have = 0;
            }
        }
    }

    tty_cleanup();
    return EXIT_SUCCESS;
}