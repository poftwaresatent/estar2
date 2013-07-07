CC = gcc
#CFLAGS = -Wall -O2 -pipe
CFLAGS = -Wall -O0 -g -pipe

HDRS= pqueue.h grid.h estar.h cell.h
SRCS= pqueue.c grid.c estar.c
OBJS= $(SRCS:.c=.o)

all: estar-mini2d test-pqueue

estar-mini2d: estar-mini2d.c $(OBJS) Makefile
	$(CC) $(CFLAGS) -o estar-mini2d estar-mini2d.c `pkg-config --cflags gtk+-2.0` `pkg-config --libs gtk+-2.0` $(OBJS)

test-pqueue: test-pqueue.c $(OBJS) Makefile
	$(CC) $(CFLAGS) -o test-pqueue test-pqueue.c $(OBJS)

pqueue.o: $(HDRS) pqueue.c Makefile
grid.o: $(HDRS) grid.c Makefile
estar.o: $(HDRS) estar.c Makefile

clean:
	rm -rf *~ *.o *.dSYM estar-mini2d test-pqueue
