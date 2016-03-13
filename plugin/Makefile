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
CFLAGS+=`pkg-config --cflags fuse`
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
all: satiploopd-cam-plugin.so satipd-cam-plugin.so satiploopd-dvbapi-plugin.so satipd-dvbapi-plugin.so

libffdecsa/libffdecsa.a:
	$(MAKE) -C libffdecsa

satiploopd-cam-plugin.so: satiploopd-cam-plugin.O ../lib/libsatip.so
	$(CC) $(LDFLAGS) -shared -o $@ $< -lsatip

satipd-cam-plugin.so: satipd-cam-plugin.O ../lib/libsatip.so
	$(CC) $(LDFLAGS) -shared -o $@ $< -lsatip

satiploopd-dvbapi-plugin.so: satiploopd-dvbapi-plugin.O ../lib/libsatip.so libffdecsa/libffdecsa.a
	$(CC) $(LDFLAGS) -shared -o $@ $< -lsatip -Llibffdecsa -Wl,-Bstatic -lffdecsa -Wl,-Bdynamic

satipd-dvbapi-plugin.so: satipd-dvbapi-plugin.O ../lib/libsatip.so libffdecsa/libffdecsa.a
	$(CC) $(LDFLAGS) -shared -o $@ $< -lsatip -Llibffdecsa -Wl,-Bstatic -lffdecsa -Wl,-Bdynamic

install: all
	@[ `id -u` = 0 ] || sh -c "echo 'must be root to install' ; exit 1"
	mkdir -p $(PFX)/lib/satip
	install -m 0755 -o root -g root satiploopd-cam-plugin.so $(PFX)/lib/satip/satiploopd-cam-plugin.so
	install -m 0755 -o root -g root satipd-cam-plugin.so $(PFX)/lib/satip/satipd-cam-plugin.so
	install -m 0755 -o root -g root satiploopd-dvbapi-plugin.so $(PFX)/lib/satip/satiploopd-dvbapi-plugin.so
	install -m 0755 -o root -g root satipd-dvbapi-plugin.so $(PFX)/lib/satip/satipd-dvbapi-plugin.so

clean:
	rm -f satiploopd-cam-plugin.so satipd-cam-plugin.so satiploopd-dvbapi-plugin.so satipd-dvbapi-plugin.so *.o *.O *.gcno *.su
	-make -C libffdecsa clean

%.O: %.c
	$(CC) -c -o $@ $(CFLAGS) -fPIC $<

%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

satiploopd-cam-plugin.O: satiploopd-cam-plugin.c ../include/satip.h
satipd-cam-plugin.O: satipd-cam-plugin.c ../include/satip.h
satiploopd-dvbapi-plugin.O: satiploopd-dvbapi-plugin.c ../include/satip.h libffdecsa/ffdecsa.h
satipd-dvbapi-plugin.O: satipd-dvbapi-plugin.c ../include/satip.h libffdecsa/ffdecsa.h
