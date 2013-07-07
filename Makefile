CC = gcc
#CFLAGS = -Wall -O2 -pipe
CFLAGS = -Wall -O0 -g -pipe

all: estar-mini2d test-pqueue

estar-mini2d: estar-mini2d.c pqueue.o grid.o estar.o Makefile
	$(CC) $(CFLAGS) -o estar-mini2d estar-mini2d.c `pkg-config --cflags gtk+-2.0` `pkg-config --libs gtk+-2.0` pqueue.o grid.o estar.o

test-pqueue: test-pqueue.c pqueue.o grid.o estar.o Makefile
	$(CC) $(CFLAGS) -o test-pqueue test-pqueue.c pqueue.o grid.o estar.o

pqueue.o: cell.h pqueue.c pqueue.h Makefile
grid.o: cell.h grid.c grid.h Makefile
estar.o: cell.h grid.h pqueue.h estar.c estar.h Makefile

clean:
	rm -rf *~ *.o *.dSYM estar-mini2d test-pqueue
