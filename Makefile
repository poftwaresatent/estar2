CC = gcc
#CFLAGS = -Wall -O2 -pipe
CFLAGS = -Wall -O0 -g -pipe

all: estar-mini2d

estar-mini2d: estar-mini2d.c cell.h pqueue.o Makefile
	$(CC) $(CFLAGS) -o estar-mini2d estar-mini2d.c `pkg-config --cflags gtk+-2.0` `pkg-config --libs gtk+-2.0`

pqueue.o: pqueue.c pqueue.h Makefile

clean:
	rm -rf *~ *.o *.dSYM estar-mini2d
