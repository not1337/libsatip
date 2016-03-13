#
#   This file is part of the satip library example programs.
#
#   (c) 2016 Andreas Steinmetz ast@domdv.de
#
#   This program is free software; you can redistribute it and/or modify it
#   under the terms of the GNU General Public License as published by the
#   Free Software Foundation, version 2.
#
PFX?=/usr/local
#
CC?=gcc
CFLAGS?=-I`pwd`/../include -Wall -Wno-strict-aliasing -Wno-pointer-sign -D_GNU_SOURCE -O3 -fomit-frame-pointer -fno-stack-protector
LDFLAGS?=-L`pwd`/../lib
#
ifdef NOCDK
CFLAGS+=-DNOCDK
CDKLIBS=
else
CDKLIBS=-lcdkw -lncursesw
endif
#
ifdef DEBUG
ifdef STACKCHECK
CFLAGS+=-fno-omit-frame-pointer -fstack-protector -fstack-usage -g
else
CFLAGS+=-fno-omit-frame-pointer -fstack-protector -g
endif
MYLIB=satip
else
ifdef PROFILE
CFLAGS+=-fno-omit-frame-pointer -fstack-protector -g -pg -DPROFILE
LDFLAGS+=-pg
MYLIB=satip_gpl
else
ifdef GCOV
CFLAGS+=-g -fno-omit-frame-pointer -fstack-protector -fprofile-arcs -ftest-coverage -DPROFILE
LDFLAGS+=-fprofile-arcs -ftest-coverage
MYLIB=satip_gpl
else
LDFLAGS+=-s
MYLIB=satip
endif
endif
endif
#
all: satipscan satipprog

satipscan: satipscan.o ../lib/libsatip.so
	$(CC) $(LDFLAGS) -o $@ $< -l$(MYLIB) -lpthread

satipprog: satipprog.o ../lib/libsatip.so
	$(CC) $(LDFLAGS) -o $@ $< -l$(MYLIB) $(CDKLIBS)

install: all
	@[ `id -u` = 0 ] || sh -c "echo 'must be root to install' ; exit 1"
	mkdir -p $(PFX)/sbin
	install -m 0755 -o root -g root satipscan $(PFX)/sbin/satipscan
	install -m 0755 -o root -g root satipprog $(PFX)/sbin/satipprog
	-setcap CAP_SYS_NICE,CAP_SYS_RESOURCE=ep $(PFX)/sbin/satipscan
	[ -f /etc/satipscan.conf ] || install -m 0644 -o root -g root satipscan.conf /etc/satipscan.conf

clean:
	rm -f satipscan satipprog *.o *.gcno *.su

.c.o:
	$(CC) -c $(CFLAGS) $<

satipscan.o: satipscan.c ../include/satip.h
satipprog.o: satipscan.c ../include/satip.h
