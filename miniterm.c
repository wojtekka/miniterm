/*
 * Copyright (c) 2006-2023, Wojtek Kaniewski <wojtekka@toxygen.net>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the author nor the names of its contributors
 *    may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <sys/ioctl.h>
#ifdef __linux__
#include <linux/serial.h>
#endif

#define ESCAPE_CHARACTER '~'    /**< Escape character */
#define HELP_CHARACTER '?'      /**< Help character */
#define BREAK_CHARACTER 'B'     /**< Break character */
#define DTR_ON_CHARACTER 'D'    /**< DTR on character */
#define DTR_OFF_CHARACTER 'd'   /**< DTR off character */
#define RTS_ON_CHARACTER 'R'    /**< RTS on character */
#define RTS_OFF_CHARACTER 'r'   /**< RTS off character */
#define SEND_CHARACTER 's'      /**< Send file character */
#define EXIT_CHARACTER '.'      /**< Exit character */

#define SLIP_END        0300    /**< End of packet marker */
#define SLIP_ESC        0333    /**< Escape sequence marker */
#define SLIP_ESC_END    0334    /**< Escaped END byte */
#define SLIP_ESC_ESC    0335    /**< Escaped ESC byte */

bool suspend = false;    /**< Suspension flag, the serial port is closed */

/** Terminal mode */
enum terminal_mode {
    MODE_TEXT,    /**< Text terminal */
    MODE_HEX,    /**< Hex terminal */
    MODE_SLIP    /**< SLIP terminal */
};

/**
 * Dumps data buffer in hex.
 *
 * \param buf Buffer pointer
 * \param len Buffer length
 */
static void dump(const char *buf, size_t len)
{
    char hextab[16] = "0123456789abcdef", line[80];
    size_t i;

    for (i = 0; i < len; i++) {

        if (i % 16 == 0)
            snprintf(line, sizeof(line), "%04x:                                                                   ", (unsigned int) i);

        line[6 + (i % 16) * 3 + ((i % 16) > 7 ? 1 : 0)] = hextab[(buf[i] >> 4) & 15];
        line[7 + (i % 16) * 3 + ((i % 16) > 7 ? 1 : 0)] = hextab[buf[i] & 15];
        line[56 + (i % 16)] = isprint(buf[i]) ? buf[i] : '.';

        if ((i % 16 == 15) || (i == len - 1))
            printf("%s\n", line);
    }
}

/**
 * SLIP packet parser
 *
 * \param fd Input descriptor
 * \param buf Output buffer pointer
 * \param len Output buffer length
 * \return Packet length
 */
static int slip_receive(int fd, char *buf, size_t len)
{
    unsigned char ch;
    size_t idx = 0;

    for (;;) {
        if (read(fd, &ch, 1) == -1) {
            if (errno == EINTR)
                continue;

            return -1;
        }

        switch (ch) {
            case SLIP_END:
                if (idx > 0)
                    return idx;
                break;

            case SLIP_ESC:
                if (read(fd, &ch, 1) == -1) {
                    if (errno == EINTR)
                        continue;

                    return -1;
                }

                if (ch == SLIP_ESC_END) {
                    ch = SLIP_END;
                    break;
                } else if (ch == SLIP_ESC_ESC) {
                    ch = SLIP_ESC;
                    break;
                }

            default:
                if (idx < len)
                    buf[idx++] = ch;
                break;
        }
    }
}

/**
 * Convert numeric baud rate to speed_t.
 *
 * \param baudrate Numberic baud rate
 * \return speed_t baud rate
 */
static speed_t convert_baudrate(unsigned int baudrate)
{
    switch (baudrate) {
        case 50: return B50;
        case 75: return B75;
        case 110: return B110;
        case 134: return B134;
        case 150: return B150;
        case 200: return B200;
        case 300: return B300;
        case 600: return B600;
        case 1200: return B1200;
        case 1800: return B1800;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B500000
        case 500000: return B500000;
#endif
#ifdef B576000
        case 576000: return B576000;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
#ifdef B1000000
        case 1000000: return B1000000;
#endif
#ifdef __linux__
        default: return -1;
#else
        default:
            fprintf(stderr, "Unknown baud rate %d\n", baudrate);
            exit(1);
#endif
    }
}


/**
 * Opens serial port
 *
 * \param device Device path
 * \param baudrate Baud rate
 * \param rtscts RTS/CTS hardware flowcontrol flag
 * \param old Pointer to termios structure for storing previous settings (may be \c NULL)
 * \return File descriptor
 */
static int serial_open(const char *device, int baudrate, bool rtscts, struct termios *old)
{
    struct termios new;
    int fd;
    int b;

    fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (fd == -1)
        return -1;

    if (old != NULL)
        tcgetattr(fd, old);

    b = convert_baudrate(baudrate);

    if (b == -1) {
#ifdef __linux__
        struct serial_struct ss;

        if (ioctl(fd, TIOCGSERIAL, &ss) == -1) {
            close(fd);
            return -1;
        }

        ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
        ss.custom_divisor = (ss.baud_base + (baudrate / 2)) / baudrate;

        if (ioctl(fd, TIOCSSERIAL, &ss) == -1) {
            close(fd);
            return -1;
        }

        if (ss.baud_base / ss.custom_divisor != baudrate)
            fprintf(stderr, "Baud rate set to %d\n", ss.baud_base / ss.custom_divisor);

        b = B38400;
#else
        fprintf(stderr, "Invalid baud rate\n");
        close(fd);
        return -1;
#endif
    }

    new.c_cflag = b | CS8 | CREAD;

    if (rtscts)
        new.c_cflag |= CRTSCTS;
    else
        new.c_cflag |= CLOCAL;

    new.c_iflag = IGNPAR;
    new.c_oflag = 0;
    new.c_lflag = 0;
    new.c_cc[VMIN] = 1;
    new.c_cc[VTIME] = 0;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &new);

    return fd;
}

/**
 * Close serial port
 *
 * \param fd File descriptor
 * \param old Pointer to termios structure with old settings (may be \c NULL)
 */
static void serial_close(int fd, struct termios *old)
{
#ifdef __linux__
    struct serial_struct ss;

    if (ioctl(fd, TIOCGSERIAL, &ss) != -1) {
        ss.flags = ss.flags & ~ASYNC_SPD_MASK;
        ioctl(fd, TIOCSSERIAL, &ss);
    }
#endif

    if (old != NULL)
        tcsetattr(fd, TCSANOW, old);

    close(fd);
}

/**
 * Print program usage
 *
 * \param argv0 Program name (\c argv[0])
 */
static void usage(const char *argv0)
{
    fprintf(stderr, "usage: %s [OPTIONS] PORT\n\n"
        "  -s BAUD     set baud rate (default: 9600)\n"
        "  -r          enable RTS/CTS hardware flow control (default: disable)\n"
        "  -D          enable DTR (default: disable)\n"
        "  -R          enable RTS when flow control disabled (default: disable)\n"
        "  -x          print received data in hex (read-only)\n"
        "  -S          print received data as SLIP packets (read-only)\n"
        "  -f PATH     file to be sent on request\n"
        "  -p          enable persistent mode, don't quit when port is not available\n"
        "  -h          print this message\n"
        "\n", argv0);
}

/**
 * SIGUSR1 handler. Tells main loop to close the serial port.
 *
 * \param sig Signal number
 *
 */
static void sigusr1(int sig)
{
    suspend = true;
}

/**
 * SIGUSR2 handler. Tells main loop to reopen the serial port.
 *
 * \param sig Signal number
 */
static void sigusr2(int sig)
{
    suspend = false;
}

/**
 * Print help on first connection.
 */
static void print_help(void)
{
    fprintf(stderr, "Press '%c%c' to exit, '%c%c' for help.\r\n\r\n", ESCAPE_CHARACTER, EXIT_CHARACTER, ESCAPE_CHARACTER, HELP_CHARACTER);
}

/**
 * Main routine.
 *
 * \param argc Argument count
 * \param argv Argument vector
 * \return Exit code
 */
int main(int argc, char **argv)
{
    struct termios stdin_termio;
    struct termios stdout_termio;
    struct termios serial_termio;
    int fd = -1;
    int retval = 0;
    int ch;
    int baudrate = 9600;
    enum terminal_mode mode = MODE_TEXT;
    bool escape = false;
    bool rtscts = false;
    bool enable_rts = false;
    bool enable_dtr = false;
    const char *device = NULL;
    const char *send_file = NULL;
    int flags;
    char *write_buf = NULL;
    size_t write_len = 0;
    bool persistent = false;
    bool first_open = true;

    while ((ch = getopt(argc, argv, "s:f:SrDRxhp")) != -1) {
        switch (ch) {
            case 's':
                baudrate = atoi(optarg);
                break;
            case 'r':
                rtscts = true;
                break;
            case 'D':
                enable_dtr = true;
                break;
            case 'R':
                enable_rts = true;
                break;
            case 'x':
                mode = MODE_HEX;
                break;
            case 'S':
                mode = MODE_SLIP;
                break;
            case 'f':
                send_file = optarg;
                break;
            case 'p':
                persistent = true;
                break;
            case 'h':
                usage(argv[0]);
                exit(0);
            default:
                usage(argv[0]);
                exit(1);
        }
    }

    if (optind < argc)
        device = argv[optind];

    if (!device) {
        usage(argv[0]);
        exit(1);
    }

    signal(SIGUSR1, sigusr1);
    signal(SIGUSR2, sigusr2);

    if (mode == MODE_TEXT) {
        struct termios new;

        new.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
        new.c_iflag = IGNPAR;
        new.c_oflag = 0;
        new.c_lflag = 0;
        new.c_cc[VMIN] = 1;
        new.c_cc[VTIME] = 0;

        tcgetattr(1, &stdout_termio);
        tcsetattr(1, TCSANOW, &new);
 
        tcgetattr(0, &stdin_termio);
        tcgetattr(0, &new);
        new.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(0, TCSANOW, &new);
    }

    for (;;) {
        fd_set rds, wds;
        int res, max = 0;

        if (suspend && fd != -1) {
            printf("\r\nSuspending...\r\n");
            serial_close(fd, &serial_termio);
            fd = -1;
        }

        if (!suspend && fd == -1) {
            if ((fd = serial_open(device, baudrate, rtscts, &serial_termio)) == -1) {
                if (first_open)
                    fprintf(stderr, "Failed to open %s: %s\r\n", device, strerror(errno));

                if (!persistent) {
                    retval = 1;
                    break;
                } else {
                    if (first_open) {
                        print_help();
                        first_open = false;
                    }
                }
            } else {
                if (ioctl(fd, TIOCMGET, &flags) == -1) {
                    perror(device);
                    retval = 1;
                    break;
                }

                if (!rtscts) {
                    if (enable_rts)
                        flags |= TIOCM_RTS;
                    else
                        flags &= ~TIOCM_RTS;
                }

                if (enable_dtr)
                    flags |= TIOCM_DTR;
                else
                    flags &= ~TIOCM_DTR;

                if (ioctl(fd, TIOCMSET, &flags) == -1) {
                    perror(device);
                    retval = 1;
                    break;
                }

                fprintf(stderr, "Connected to %s at %dbps.\r\n", device, baudrate);
                if (first_open) {
                    print_help();
                    first_open = false;
                }

                if (mode == MODE_SLIP) {
                    char buf[4096];
                    int len;

                    for (;;) {
                        len = slip_receive(fd, buf, sizeof(buf));
                        if (len == -1)
                            break;
                        dump(buf, len);
                        printf("\n");
                    }
                    close(fd);
                    fd = -1;
                    if (!persistent) {
                        retval = 1;
                        break;
                    }
                }
            }
        }

        FD_ZERO(&rds);
        FD_ZERO(&wds);

        if (mode == MODE_TEXT) {
            FD_SET(0, &rds);
            max = 0;
        }

        if (fd != -1) {
            FD_SET(fd, &rds);
            if (write_buf != NULL && write_len > 0)
                FD_SET(fd, &wds);
            max = fd;
        }

        struct timeval timeout_tmp;
        struct timeval *timeout = NULL;
        if (persistent && fd == -1) {
            timeout = &timeout_tmp;
            timeout->tv_sec = 0;
            timeout->tv_usec = 100000;
        }

        res = select(max + 1, &rds, &wds, NULL, timeout);

        if (res < 0) {
            if (errno == EINTR)
                continue;

            perror("select");
            retval = 1;
            break;
        }

        if (mode == MODE_TEXT && FD_ISSET(0, &rds)) {
            char ibuf[4096], obuf[4096];
            int i, ilen, olen = 0;
            bool quit = false;

            if ((ilen = read(0, ibuf, sizeof(ibuf))) < 1)
                break;

            for (i = 0; i < ilen; i++) {
                if (escape) {
                    if (ibuf[i] == ESCAPE_CHARACTER)
                        obuf[olen++] = ESCAPE_CHARACTER;
                    else if (ibuf[i] == HELP_CHARACTER) {
                        printf("\r\n%c%c  This help message", ESCAPE_CHARACTER, HELP_CHARACTER);
                        printf("\r\n%c%c  Send break", ESCAPE_CHARACTER, BREAK_CHARACTER);
                        printf("\r\n%c%c  Enable DTR", ESCAPE_CHARACTER, DTR_ON_CHARACTER);
                        printf("\r\n%c%c  Disable DTR", ESCAPE_CHARACTER, DTR_OFF_CHARACTER);
                        printf("\r\n%c%c  Enable RTS", ESCAPE_CHARACTER, RTS_ON_CHARACTER);
                        printf("\r\n%c%c  Disable RTS", ESCAPE_CHARACTER, RTS_OFF_CHARACTER);
                        printf("\r\n%c%c  Send file", ESCAPE_CHARACTER, SEND_CHARACTER);
                        printf("\r\n%c%c  Exit program\r\n", ESCAPE_CHARACTER, EXIT_CHARACTER);
                    } else if (ibuf[i] == BREAK_CHARACTER) {
                        tcsendbreak(fd, 0);
                    } else if (ibuf[i] == DTR_ON_CHARACTER) {
                        int flags;
                        ioctl(fd, TIOCMGET, &flags);
                        flags |= TIOCM_DTR;
                        ioctl(fd, TIOCMSET, &flags);
                    } else if (ibuf[i] == DTR_OFF_CHARACTER) {
                        int flags;
                        ioctl(fd, TIOCMGET, &flags);
                        flags &= ~TIOCM_DTR;
                        ioctl(fd, TIOCMSET, &flags);
                    } else if (ibuf[i] == RTS_ON_CHARACTER) {
                        int flags;
                        ioctl(fd, TIOCMGET, &flags);
                        flags |= TIOCM_RTS;
                        ioctl(fd, TIOCMSET, &flags);
                    } else if (ibuf[i] == RTS_OFF_CHARACTER) {
                        int flags;
                        ioctl(fd, TIOCMGET, &flags);
                        flags &= ~TIOCM_RTS;
                        ioctl(fd, TIOCMSET, &flags);
                    } else if (ibuf[i] == SEND_CHARACTER) {
                        if (send_file == NULL) {
                            fprintf(stderr, "\r\nFile not provided\r\n");
                        } else {
                            int file_fd = open(send_file, O_RDONLY);
                            if (file_fd == -1) {
                                fprintf(stderr, "\r\nFailed to open '%s': %s\r\n", send_file, strerror(errno));
                            } else {
                                off_t file_len = lseek(file_fd, 0, SEEK_END);
                                if (file_len == (off_t)-1) {
                                    fprintf(stderr, "\r\nFailed to check file size. Is it a regular file?\r\n");
                                } else {
                                    lseek(file_fd, 0, SEEK_SET);
                                    char *tmp = realloc(write_buf, write_len + file_len);
                                    if (tmp == NULL) {
                                        fprintf(stderr, "\r\nOut of memory\r\n");
                                        quit = true;
                                        retval = 1;
                                        close(file_fd);
                                        break;
                                    } else {
                                        write_buf = tmp;
                                        int res = read(file_fd, write_buf + write_len, file_len);
                                        if (res != file_len) {
                                            fprintf(stderr, "\r\nFailed to read %zu bytes from '%s'\r\n", file_len, send_file);
                                        } else {
                                            fprintf(stderr, "\r\nRead %zu bytes, sending...\r\n", file_len);
                                            write_len += file_len;
                                        }
                                    }
                                }
                                close(file_fd);
                            }
                        }
                    } else if (ibuf[i] == EXIT_CHARACTER) {
                        quit = true;
                        break;
                    } else {
                        obuf[olen++] = ESCAPE_CHARACTER;
                        obuf[olen++] = ibuf[i];
                    }
                    escape = false;
                } else {
                    if (ibuf[i] == ESCAPE_CHARACTER)
                        escape = true;
                    else {
                        obuf[olen++] = ibuf[i];
                        escape = false;
                    }
                }
            }

            if (olen > 0) {
                char *tmp = realloc(write_buf, write_len + olen);
                if (tmp == NULL) {
                    fprintf(stderr, "\r\nOut of memory\r\n");
                    retval = 1; 
                    break;
                } else {
                    write_buf = tmp;
                    memcpy(write_buf + write_len, obuf, olen);
                    write_len += olen;
                }
            }

            if (quit)
                break;
        }

        if (fd != -1 && FD_ISSET(fd, &wds)) {
            int res = write(fd, write_buf, write_len);

            if (res == -1) {
                if (!persistent) {
                    retval = 1;
                    break;
                } else {
                    perror(device);
                    close(fd);
                    fd = -1;
                }
            }

            if (res == write_len) {
                free(write_buf);
                write_buf = NULL;
                write_len = 0;
            } else {
                memmove(write_buf, write_buf + res, write_len - res);
                write_len -= res;
            }
        }

        if (fd != -1 && FD_ISSET(fd, &rds)) {
            char buf[4096];
            int len, wrote, res;

            if ((len = read(fd, buf, sizeof(buf))) < 1) {
                if (len == 0)
                    fprintf(stderr, "\r\nDisconnected from %s\r\n", device);
                else if (len == -1)
                    fprintf(stderr, "\r\n%s: %s\r\n", device, strerror(errno));

                if (!persistent) {
                    break;
                } else {
                    close(fd);
                    fd = -1;
                }
            }

            if (mode == MODE_TEXT) {
                for (wrote = 0; wrote < len; ) {
                    res = write(1, buf + wrote, len - wrote);

                    if (res < 1) {
                        retval = 1;
                        break;
                    }

                    wrote += res;
                }
            } else {
                printf("Read %d byte(s):\n", len);
                dump(buf, len);
                printf("\n");
            }
        }
    }

    if (mode == MODE_TEXT) {
        tcsetattr(0, TCSANOW, &stdin_termio);
        tcsetattr(1, TCSANOW, &stdout_termio);
    }

    if (fd != -1) {
        serial_close(fd, &serial_termio);
        fprintf(stderr, "\nConnection closed.\n");
    }

    return retval;
}

