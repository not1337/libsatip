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
all: satipd satipdctl sample-plugin.so

satipdctl: satipdctl.o
	$(CC) $(LDFLAGS) -o $@ $<

satipd: satipd.o ../lib/libsatip.so
	$(CC) $(LDFLAGS) -o $@ $< -l$(MYLIB) -lpthread -ldl

sample-plugin.so: sample-plugin.O ../lib/libsatip.so
	$(CC) $(LDFLAGS) -shared -o $@ $< -lsatip

icons.h: icons/tux-icon-48.png icons/tux-icon-48.jpg icons/tux-icon-120.png \
	icons/tux-icon-120.jpg
	./mkiconh.sh

install: all
	@[ `id -u` != 0 ] && sh -c "echo 'must be root to install' ; exit 1"
	mkdir -p $(PFX)/sbin
	install -m 0755 -o root -g root satipd $(PFX)/sbin/satipd
	install -m 0755 -o root -g root satipdctl $(PFX)/sbin/satipdctl
	-setcap CAP_SETUID,CAP_SETGID,CAP_IPC_LOCK,CAP_NET_ADMIN,CAP_NET_RAW,CAP_NET_BIND_SERVICE,CAP_SYS_NICE,CAP_SYS_RESOURCE=ep $(PFX)/sbin/satipd
	[ -f /etc/satipd.conf ] || install -m 0644 -o root -g root satipd.conf /etc/satipd.conf

clean:
	rm -f satipd satipdctl sample-plugin.so *.o *.O *.gcno *.su icons.h

%.O: %.c
	$(CC) -c -o $@ $(CFLAGS) -fPIC $<

%.o: %.c
	$(CC) -c $(CFLAGS) $<

satipd.o: satipd.c ../include/satip.h ctlsocket.h icons.h
satipdctl.o: satipdctl.c ../include/satip.h ctlsocket.h
sample-plugin.O: sample-plugin.c ../include/satip.h
