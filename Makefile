INSTALL	?= install
MKDIR	?= mkdir
CP		?= cp
CHMOD	?= chmod

PREFIX	?= usr/
DESTDIR ?= /

CFLAGS += -std=gnu99

EXE		= earlyboot
SRCS	= $(wildcard src/*.c)
OBJS	= $(patsubst %.c,%.o,$(SRCS))

all: build

info:
	@echo $(SRCS)

.c.o: $(SRCS)
	$(CC) $(CFLAGS) -c -o $@ $<

build: $(OBJS)
	$(CC) $(CFLAGS) $^ -o $(EXE)

install:
	$(INSTALL) -D -m 755 earlyboot $(DESTDIR)/$(PREFIX)/bin/earlyboot

clean:
	rm -rf $(OBJS) $(EXE)
