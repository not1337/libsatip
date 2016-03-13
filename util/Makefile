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
CFLAGS?=-I`pwd`/../include -Wall -Wno-strict-aliasing -Wno-pointer-sign -D_GNU_SOURCE -O3 -fomit-frame-pointer
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
all: satipdetect satippipe satipzap satipmcast satipinfo satipsink

satipdetect: satipdetect.o ../lib/libsatip.so
	$(CC) $(LDFLAGS) -o $@ $< -l$(MYLIB) -lpthread

satippipe: satippipe.o ../lib/libsatip.so
	$(CC) $(LDFLAGS) -o $@ $< -l$(MYLIB) -lpthread

satipzap: satipzap.o ../lib/libsatip.so
	$(CC) $(LDFLAGS) -o $@ $< -l$(MYLIB) -lpthread

satipmcast: satipmcast.o ../lib/libsatip.so
	$(CC) $(LDFLAGS) -o $@ $< -l$(MYLIB) -lpthread

satipinfo: satipinfo.o ../lib/libsatip.so
	$(CC) $(LDFLAGS) -o $@ $< -l$(MYLIB) -lpthread

satipsink: satipsink.o
	$(CC) $(LDFLAGS) -o $@ $< -lpthread

install: all
	@[ `id -u` = 0 ] || sh -c "echo 'must be root to install' ; exit 1"
	mkdir -p $(PFX)/bin
	install -m 0755 -o root -g root satippipe $(PFX)/bin/satippipe
	install -m 0755 -o root -g root satipdetect $(PFX)/bin/satipdetect
	install -m 0755 -o root -g root satipzap $(PFX)/bin/satipzap
	install -m 0755 -o root -g root satipzap $(PFX)/bin/satipmcast
	install -m 0755 -o root -g root satipinfo $(PFX)/bin/satipinfo
	install -m 0755 -o root -g root satipsink $(PFX)/bin/satipsink

clean:
	rm -f satipdetect satippipe satipzap satipmcast satipinfo satipsink *.o *.gcno *.su

%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

satipdetect.o: satipdetect.c ../include/satip.h
satippipe.o: satippipe.c ../include/satip.h
satipzap.o: satipzap.c ../include/satip.h
satipmcast.o: satipmcast.c ../include/satip.h
satipinfo.o: satipinfo.c ../include/satip.h
satipsink.o: satipsink.c ../include/satip.h
