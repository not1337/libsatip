#
#   This file is part of the satip library.
#
#   The satip Library is free software; you can redistribute it and/or
#   modify it under the terms of the GNU Lesser General Public
#   License as published by the Free Software Foundation; either
#   version 2.1 of the License, or (at your option) any later version.
#
#   (c) 2016 Andreas Steinmetz ast@domdv.de
#
PFX?=/usr/local
CC?=gcc
AR?=ar
CFLAGS?=-I`pwd`/../include -Wall -Wno-strict-aliasing -Wno-pointer-sign -D_GNU_SOURCE -O3 -fomit-frame-pointer -fno-stack-protector
LDFLAGS?=
#
ifdef DEBUG
ifdef STACKCHECK
CFLAGS+=-fno-omit-frame-pointer -fstack-protector -fstack-usage -g
else
CFLAGS+=-fno-omit-frame-pointer -fstack-protector -g
endif
else
ifdef PROFILE
CFLAGS+=-fno-omit-frame-pointer -fstack-protector -g -pg -DPROFILE
LDFLAGS+=-pg
else
ifdef GCOV
CFLAGS+=-g -fno-omit-frame-pointer -fstack-protector -fprofile-arcs -ftest-coverage -DPROFILE
LDFLAGS+=-fprofile-arcs -ftest-coverage
else
LDFLAGS+=-s
endif
endif
endif
#
SOVERSION=0
SOMINOR=0

all: libsatip.so libsatip.so.$(SOVERSION) libsatip_gpl.a

libsatip.so: libsatip.so.$(SOVERSION).$(SOMINOR)
	ln -sf $< $@

libsatip.so.$(SOVERSION): libsatip.so.$(SOVERSION).$(SOMINOR)
	ln -sf $< $@

libsatip.so.$(SOVERSION).$(SOMINOR): hw.O remap.O server.O client.O util.O scan.O
	$(CC) $(LDFLAGS) -shared -Wl,-soname,libsatip.so.$(SOVERSION) -o $@ $^ -lpthread

libsatip_gpl.a: hw.o remap.o server.o client.o util.o scan.o
	$(AR) rcu $@ $^

install: all
	@[ `id -u` != 0 ] && sh -c "echo 'must be root to install' ; exit 1"
	mkdir -p $(PFX)/lib
	install -m 0644 -o root -g root libsatip.so.$(SOVERSION).$(SOMINOR) $(PFX)/lib/libsatip.so.$(SOVERSION).$(SOMINOR)
	ln -sf libsatip.so.$(SOVERSION).$(SOMINOR) $(PFX)/lib/libsatip.so.$(SOVERSION)
	ln -sf libsatip.so.$(SOVERSION).$(SOMINOR) $(PFX)/lib/libsatip.so
	install -m 0644 -o root -g root libsatip_gpl.a $(PFX)/lib/libsatip_gpl.a

clean:
	rm -f *.O *.o *.gcno *.su libsatip.so libsatip.so.$(SOVERSION) libsatip.so.$(SOVERSION).$(SOMINOR) libsatip_gpl.a

%.O: %.c
	$(CC) -c -o $@ $(CFLAGS) -fPIC $<

%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

hw.o hw.O: hw.c ../include/satip.h
remap.o remap.O: ../include/satip.h
server.o server.O: server.c ../include/satip.h common.h
client.o client.O: client.c ../include/satip.h common.h
util.o util.O: util.c ../include/satip.h common.h
scan.o scan.O: scan.c ../include/satip.h
