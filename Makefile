CC = gcc
CFLAGS = -Wall -O3

all:	miniterm

clean:
	rm -f miniterm core *~

.PHONY:	all install clean
