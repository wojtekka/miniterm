/*
 * Copyright (c) 2006-2014, Wojtek Kaniewski <wojtekka@toxygen.net>
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
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>

#define END	0300    /**< End of packet marker */
#define ESC	0333    /**< Escape sequence marker */
#define ESC_END	0334    /**< Escaped END byte */
#define ESC_ESC	0335    /**< Escaped ESC byte */

int suspend = 0;	/**< Suspension flag, the serial port is closed */

/**
 * Dumps data buffer in hex.
 *
 * \param buf Buffer pointer
 * \param len Buffer length
 */
void dump(const char *buf, int len)
{
	char hextab[16] = "0123456789abcdef", line[80];
	int i;

	for (i = 0; i < len; i++) {
		
		if (i % 16 == 0)
			snprintf(line, sizeof(line), "%04x:                                                                   ", i);
						
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
int slip_receive(int fd, char *buf, int len)
{
	unsigned char ch;
	int idx = 0;

	for (;;) {
		if (read(fd, &ch, 1) == -1) {
			if (errno == EINTR)
				continue;
			
			return -1;
		}

		switch (ch) {
			case END:
				if (idx > 0)
					return idx;
				break;

			case ESC:
				if (read(fd, &ch, 1) == -1) {
					if (errno == EINTR)
						continue;
				
					return -1;
				}

				if (ch == ESC_END) {
					ch = END;
					break;
				} else if (ch == ESC_ESC) {
					ch = ESC;
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
 * Opens serial port
 *
 * \param device Device path
 * \param baudrate Baud rate (Bxxx constant, not decimal)
 * \param rtscts RTS/CTS hardware flowcontrol flag
 * \param old Pointer to termios structure for storing previous settings (may be \c NULL)
 * \return File descriptor
 */
int serial_open(const char *device, int baudrate, int rtscts, struct termios *old)
{
	struct termios new;
	int fd;

	if ((fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1)
		return -1;

	fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);

	if (old != NULL)
		tcgetattr(fd, old);

	new.c_cflag = baudrate | CS8 | CREAD;

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
void serial_close(int fd, struct termios *old)
{
	if (old != NULL)
		tcsetattr(fd, TCSANOW, old);

	close(fd);
}

/**
 * Print program usage
 *
 * \param argv0 Program name (\c argv[0])
 */
void usage(const char *argv0)
{
	fprintf(stderr, "usage: %s [OPTIONS] PORT\n\n"
		"  -s BAUD     set baud rate (default: 9600)\n"
		"  -r          enable RTS/CTS hardware flow control (default: disable)\n"
		"  -x          print received data in hex (read-only)\n"
		"  -S          print received data as SLIP packets (read-only)\n"
		"  -h          print this message\n"
		"\n", argv0);
}

/**
 * SIGUSR1 handler. Tells main loop to close the serial port.
 *
 * \param sig Signal number
 *
 */
void sigusr1(int sig)
{
	suspend = 1;
}

/**
 * SIGUSR2 handler. Tells main loop to reopen the serial port.
 *
 * \param sig Signal number
 */
void sigusr2(int sig)
{
	suspend = 0;
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
	struct termios stdin_termio, stdout_termio, serial_termio;
	int fd, baudrate = 9600, b, retval = 0, tilde = 0, ch, rtscts = 0, mode = 0;
	const char *device = NULL;

	while ((ch = getopt(argc, argv, "s:Srxh")) != -1) {
		switch (ch) {
			case 's':
				baudrate = atoi(optarg);
				break;
			case 'r':
				rtscts = 1;
				break;
			case 'x':
				mode = 1;
				break;
			case 'S':
				mode = 2;
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

	switch (baudrate) {
		case 50: b = B50; break;
		case 75: b = B75; break;
		case 110: b = B110; break;
		case 134: b = B134; break;
		case 150: b = B150; break;
		case 200: b = B200; break;
		case 300: b = B300; break;
		case 600: b = B600; break;
		case 1200: b = B1200; break;
		case 1800: b = B1800; break;
		case 2400: b = B2400; break;
		case 4800: b = B4800; break;
		case 9600: b = B9600; break;
		case 19200: b = B19200; break;
		case 38400: b = B38400; break;
		case 57600: b = B57600; break;
		case 115200: b = B115200; break;
#ifdef B230400
		case 230400: b = B230400; break;
#endif
#ifdef B460800
		case 460800: b = B460800; break;
#endif
#ifdef B500000
		case 500000: b = B500000; break;
#endif
#ifdef B576000
		case 576000: b = B576000; break;
#endif
#ifdef B921600
		case 921600: b = B921600; break;
#endif
#ifdef B1000000
		case 1000000: b = B1000000; break;
#endif
		default:
			fprintf(stderr, "Unknown baud rate %d\n", baudrate);
			exit(1);
	}
	
	signal(SIGUSR1, sigusr1);
	signal(SIGUSR2, sigusr2);

	if ((fd = serial_open(device, b, rtscts, &serial_termio)) == -1) {
		perror(device);
		exit(1);
	}

	fprintf(stderr, "Connected to %s at %dbps. Press '~.' to exit, '~B' to send break.\n\n", device, baudrate);
	
	if (mode == 0) {
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

	if (mode == 2) {
		char buf[4096];
		int len;
		
		for (;;) {
			if ((len = slip_receive(fd, buf, sizeof(buf))) == -1)
				break;
			dump(buf, len);
			printf("\n");
		}
	}

	for (;;) {
		fd_set rds;
		int res, max = 0;

		if (suspend && fd != -1) {
			printf("Suspending...\n");
			serial_close(fd, &serial_termio);
			fd = -1;
		}

		if (!suspend && fd == -1) {
			printf("Resuming...\n");
			if ((fd = serial_open(device, baudrate, rtscts, &serial_termio)) == -1) {
				perror(device);
				exit(1);
			}
		}

		FD_ZERO(&rds);

		if (mode == 0) {
			FD_SET(0, &rds);
			max = 0;
		}

		if (fd != -1) {
			FD_SET(fd, &rds);
			max = fd;
		}

		res = select(max + 1, &rds, NULL, NULL, NULL);

		if (res < 0) {
			if (errno == EINTR)
				continue;

			perror("select");
			retval = 1;
			break;
		}

		if (mode == 0 && FD_ISSET(0, &rds)) {
			char ibuf[4096], obuf[4096];
			int i, wrote, ilen, olen = 0, quit = 0;

			if ((ilen = read(0, ibuf, sizeof(ibuf))) < 1)
				break;

			for (i = 0; i < ilen; i++) {
				if (tilde) {
					if (ibuf[i] == '~')
						obuf[olen++] = '~';
					else if (ibuf[i] == 'B')
						tcsendbreak(fd, 0);
					else if (ibuf[i] == '.') {
						quit = 1;
						break;
					} else {
						obuf[olen++] = '~';
						obuf[olen++] = ibuf[i];
					}
					tilde = 0;
				} else {
					if (ibuf[i] == '~')
						tilde = 1;
					else {
						obuf[olen++] = ibuf[i];
						tilde = 0;
					}
				}
			}
			
			for (wrote = 0; olen && wrote < olen; ) {
				res = write(fd, obuf + wrote, olen - wrote);

				if (res < 1) {
					retval = 1;
					break;
				}

				wrote += res;
			}

			if (quit)
				break;
		}

		if (fd != -1 && FD_ISSET(fd, &rds)) {
			char buf[4096];
			int len, wrote, res;

			if ((len = read(fd, buf, sizeof(buf))) < 1)
				break;

			if (mode == 0) {
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

	if (mode == 0) {
		tcsetattr(0, TCSANOW, &stdin_termio);
		tcsetattr(1, TCSANOW, &stdout_termio);
	}

	if (fd != -1)
		serial_close(fd, &serial_termio);

	fprintf(stderr, "\nConnection closed.\n");

	return 0;
}
