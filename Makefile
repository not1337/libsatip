#
#   This file is part of the satip library.
#
#   (c) 2016 Andreas Steinmetz ast@domdv.de
#
#   This program is free software; you can redistribute it and/or modify it
#   under the terms of the GNU General Public License as published by the
#   Free Software Foundation, version 2.
#
#set the installation prefix here
PFX=/usr/local
#
#define, if you don't have the cdk library (optional anyway)
#NOCDK=1
#
#define, if you want usable core dumps or gdb information
#DEBUG=1
#
#define, if you want to check stack usage (needs DEBUG)
#STACKCHECK=1
#
#define, if you want to run gprof
#PROFILE=1
#
#define, if you want to run gcov
#GCOV=1
#
#define, if you don't want to use spinlocks
#NO_SPINLOCKS=1
#
TOPDIR=$(shell pwd)
CC=gcc
AR=ar
CFLAGS=-I$(TOPDIR)/include -Wall -Wno-strict-aliasing -Wno-pointer-sign -D_GNU_SOURCE -O3 -fomit-frame-pointer -fno-stack-protector
LDFLAGS=-L$(TOPDIR)/lib
#
DIRS=include lib server scan loop util plugin
export PFX
export CC
export AR
export CFLAGS
export LDFLAGS
export DEBUG
export STACKCHECK
export PROFILE
export GCOV

all:
	for i in $(DIRS) ; do $(MAKE) -C $$i || exit 1 ; done

install: all
	@[ `id -u` = 0 ] || sh -c "echo 'must be root to install' ; exit 1"
	for i in $(DIRS) ; do $(MAKE) -C $$i install || exit 1 ; done

clean:
	for i in $(DIRS) ; do $(MAKE) -C $$i clean || exit 1 ; done
