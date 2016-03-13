#!/bin/sh
#
#   This file is part of the satip library example programs.
#
#   (c) 2016 Andreas Steinmetz ast@domdv.de
#
#   This program is free software; you can redistribute it and/or modify it
#   under the terms of the GNU General Public License as published by the
#   Free Software Foundation, version 2.
#
file2array()
{
size=`stat -c %s icons/$1`
echo "#define $3 $size"
echo ""
echo "unsigned char $2[$size]="
echo "{"
od -t x1 -v -w8 icons/$1 | sed -e 's/^......../ /' -e 's/^.......$//' \
	-e 's/ /,0x/g' -e 's/^,/\t/' -e 's/$/,/' -e 's/^,$//' | grep -v '^$'
echo "};"
echo ""
}
(
file2array tux-icon-48.png png48 PNG48
file2array tux-icon-120.png png120 PNG120
file2array tux-icon-48.jpg jpg48 JPG48
file2array tux-icon-120.jpg jpg120 JPG120
) > icons.h
