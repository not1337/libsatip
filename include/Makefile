#
#   This file is part of the satip library.
#
#   When building or linking against the shared library:
#
#   The satip Library is free software; you can redistribute it and/or
#   modify it under the terms of the GNU Lesser General Public
#   License as published by the Free Software Foundation; either
#   version 2.1 of the License, or (at your option) any later version.
#
#   When building not for or not linking against the shared library:
#
#   The satip Library is free software; you can redistribute it and/or modify it
#   under the terms of the GNU General Public License as published by the
#   Free Software Foundation, version 2.
#
#   (c) 2016 Andreas Steinmetz ast@domdv.de
#
PFX?=/usr/local

all:

install: all
	@[ `id -u` != 0 ] && sh -c "echo 'must be root to install' ; exit 1"
	mkdir -p $(PFX)/include
	install -m 0644 -o root -g root satip.h $(PFX)/include/satip.h

clean:

