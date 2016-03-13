/*
 * example SAT>IP server daemon control socket header
 *
 * Copyright (c) 2016 Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 * License extemption:
 * If you only require to include this header to create your own control
 * application, this header is licensed as:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef _CTLSOCKET_H
#define _CTLSOCKET_H

#ifdef __cplusplus
extern "C" {
#endif

#define CTL_MAX_REQUEST	8192
#define CTL_MAX_ANSWER	1048576

#define CTLRTSP		0
#define CTLHTTP		1
#define CTLCLRALL	2
#define CTLCLRRTSP	3
#define CTLCLRHTTP	4
#define CTLCLRSTREAM	5
#define CTLCLRDEV	6
#define CTLADDMCST	7
#define CTLDELMCST	8
#define CTLADDLIST	9
#define CTLDELLIST	10
#define CTLADDRLIST	11
#define CTLDELRLIST	12
#define CTLADDHLIST	13
#define CTLDELHLIST	14
#define CTLADDMLIST	15
#define CTLDELMLIST	16
#define CTLTUNER	17
#define CTLINQ		18
#define CTLSVRSTATS	19
#define CTLTUNERSTATS	20
#define CTLRTSPSTATS	21
#define CTLHTTPSTATS	22
#define CTLRESTART	255

#define CTLOK		0
#define CTLERR		-1

typedef struct
{
	int reqans;
	int datalen;
	unsigned char data[0];
} CTLMSG;

typedef struct
{
	int val;
} CTLINT1;

typedef struct
{
	int val1;
	int val2;
} CTLINT2;

typedef struct
{
	int val1;
	int val2;
	int val3;
	int val4;
	int val5;
} CTLINT5;

typedef struct
{
	int srvid;
	int ttl;
	int play;
	char url[0];
} CTLMCST;

typedef struct
{
	int access;
	int open;
	int streams;
	int lock;
	int level;
	int quality;
	int groupstreams;
	unsigned long long streambytes;
	unsigned long long byteupdates;
	char msys[16];
	char caps[112];
	char tune[384];
} CTLHWSTATS;

typedef struct
{
	int instance;
	int streamid;
	int playing;
	int port;
	int lock;
	int level;
	int quality;
	char addr[64];
	char tune[384];
} CTLSVSTATS;

#ifdef __cplusplus
}
#endif

#endif
