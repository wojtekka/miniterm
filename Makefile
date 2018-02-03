CC = gcc
CFLAGS += -Wall -O3

all:	miniterm

clean:
	rm -f miniterm core *~

install:	miniterm
	cp miniterm /usr/local/bin

.PHONY:	all install clean
