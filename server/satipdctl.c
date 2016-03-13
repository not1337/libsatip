/*
 * example SAT>IP server daemon control socket client
 *
 * Copyright (c) 2016 Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 */

#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <unistd.h>
#include <stdlib.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include "ctlsocket.h"

#ifndef __GNUC__
#define __attribute__(x)
#endif

static void usage(void) __attribute__ ((noreturn));

static void usage(void)
{
fprintf(stderr,
"Usage:\n"
"satipdctl -p socket -a net|-A net|-r net|-R net|-h net|-H net|-m net|-M net\n"
"satipdctl -p socket [-i instance] [-x ttl] [-P] -S setup-url\n"
"satipdctl -p socket [-i instance] -s streamid|-c streamid\n"
"satipdctl -p socket -d deviceid\n"
"satipdctl -p socket -u deviceid|-U deviceid\n"
"satipdctl -p socket -c|-l|-L|-e|-E|-t|-T|-X|-I\n"
"-a net         remove host/network (CDIR format) from global access\n"
"-A net         add host/network (CDIR format) to global access\n"
"-r net         remove host/network (CDIR format) from RTSP access\n"
"-R net         add host/network (CDIR format) to RTSP access\n"
"-h net         remove host/network (CDIR format) from HTTP access\n"
"-H net         add host/network (CDIR format) to HTTP access\n"
"-m net         remove host/network (CDIR format) from multicast setup access\n"
"-M net         add host/network (CDIR format) to multicast setup access\n"
"-S setup-url   setup multicast, setup-url is a SAT>IP RTP url with a query\n"
"-s streamid    remove multicast, streamid ist thre value returned from '-S'\n"
"-c streamid    kill stream from server\n"
"-d deviceid    kill all streams using tuner 'deviceid' from server\n"
"-u deviceid    disable tuner 'deviceid' from further usage\n"
"-U deviceid    enable tuner 'deviceid'\n"
"-c             kill all streams\n"
"-l             kill all RTSP streams\n"
"-L             kill all HTTP streams\n"
"-e             disable further RTSP access\n"
"-E             enable RTSP access\n"
"-t             disable further HTTP access\n"
"-T             enable HTTP access\n"
"-X             restart server gracefully\n"
"-I             inquire running status\n"
"-o             show server statistics\n"
"-O             show tuner statistics\n"
"-q             show RTSP statistics\n"
"-Q             show HTTP statistics\n"
"-p socket      pathname of satipd control socket\n"
"-i instance    instance of server if more than one is defined, default is 1\n"
"-x ttl         multicast ttl from 1 to 255, default is 2\n"
"-P             instant multicast stream play, default is defer until joined\n"
);
	exit(1);
}

static int doconnect(char *fn,int waitms)
{
	int s;
	int x;
	int y;
	struct sockaddr_un a;
	struct pollfd p;
	struct stat stb;

	if(waitms<0||waitms>10000)return -1;
	if(strlen(fn)>=sizeof(a.sun_path))return -1;
	if(lstat(fn,&stb))return -1;
	if(!S_ISSOCK(stb.st_mode))return -1;
	if(access(fn,R_OK|W_OK))return -1;

	memset(&a,0,sizeof(a));
	a.sun_family=AF_UNIX;
	strcpy(a.sun_path,fn);

	if((s=socket(PF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0))==-1)
		return -1;

	if(connect(s,(struct sockaddr *)(&a),sizeof(a)))
	{
		if(errno!=EINPROGRESS)
		{
			close(s);
			return -1;
		}

		p.fd=s;
		p.events=POLLOUT;

		if(poll(&p,1,waitms)<=0)
		{
			close(s);
			return -1;
		}

		y=sizeof(x);
		if(getsockopt(s,SOL_SOCKET,SO_ERROR,&x,&y))
		{
			close(s);
			return -1;
		}

		if(x)
		{
			close(s);
			return -1;
		}
	}

	return s;
}

static int sendreq(int s,int req,void *data,int len)
{
	CTLMSG m;
	struct iovec iov[2];

	if((!data&&len)||len<0||len>CTL_MAX_REQUEST)return -1;

	m.reqans=req;
	m.datalen=len;

	iov[0].iov_base=&m;
	iov[0].iov_len=sizeof(CTLMSG);
	iov[1].iov_base=data;
	iov[1].iov_len=len;

	if(writev(s,iov,len?2:1)!=sizeof(CTLMSG)+len)return -1;
	return 0;
}

static int recvans(int s,int *ans,void **data,int *len,int waitms)
{
	int l;
	unsigned char *ptr;
	CTLMSG m;
	struct pollfd p;

	if(!ans||!data||!len||waitms<0||waitms>10000)return -1;

	p.fd=s;
	p.events=POLLIN;

	if(poll(&p,1,waitms)<=0)return -1;
	if(!(p.revents&POLLIN))return -1;

	if(read(s,&m,sizeof(CTLMSG))!=sizeof(CTLMSG))return -1;

	if(m.datalen<0||m.datalen>CTL_MAX_ANSWER)return -1;

	*ans=m.reqans;
	*len=m.datalen;
	*data=NULL;

	if(m.datalen)
	{
		if(!(*data=ptr=malloc(m.datalen)))return -1;

		while(m.datalen)
		{
			if(poll(&p,1,100)<=0||!(p.revents&POLLIN))
			{
				free(*data);
				*data=NULL;
				return -1;
			}
			if((l=read(s,ptr,m.datalen))<=0)
			{
				free(*data);
				*data=NULL;
				return -1;
			}
			ptr+=l;
			m.datalen-=l;
		}
	}

	return 0;
}

int main(int argc,char *argv[])
{
	int c;
	int s;
	int ans;
	int len;
	int res=1;
	int req=-1;
	int instance=0;
	int ttl=0;
	int play=0;
	char *fn=NULL;
	void *data=NULL;
	union
	{
		CTLINT1 i1;
		CTLINT2 i2;
		CTLINT5 i5;
		CTLMCST m;
		unsigned char bfr[CTL_MAX_REQUEST];
		CTLHWSTATS *h;
		CTLSVSTATS *s;
	}u;

	while((c=getopt(argc,argv,
	    "a:A:r:R:h:H:m:M:s:S:e:E:t:T:cx:C:d:lLXIuUi:p:PoOqQ"))!=-1)switch(c)
	{
	case 'e':
		if(req!=-1)usage();
		req=CTLRTSP;
		u.i1.val=0;
		break;

	case 'E':
		if(req!=-1)usage();
		req=CTLRTSP;
		u.i1.val=1;
		break;

	case 't':
		if(req!=-1)usage();
		req=CTLHTTP;
		u.i1.val=0;
		break;

	case 'T':
		if(req!=-1)usage();
		req=CTLHTTP;
		u.i1.val=1;
		break;

	case 'c':
		if(req!=-1)usage();
		req=CTLCLRALL;
		break;

	case 'l':
		if(req!=-1)usage();
		req=CTLCLRRTSP;
		break;

	case 'L':
		if(req!=-1)usage();
		req=CTLCLRHTTP;
		break;

	case 'C':
		if(req!=-1)usage();
		req=CTLCLRSTREAM;
		if((u.i2.val2=atoi(optarg))==-1)usage();
		break;

	case 'd':
		if(req!=-1)usage();
		req=CTLCLRDEV;
		if((u.i1.val=atoi(optarg))<1||u.i1.val>65535)usage();
		break;

	case 'S':
		if(req!=-1)usage();
		req=CTLADDMCST;
		strncpy(u.bfr+sizeof(CTLMCST),optarg,
			sizeof(u.bfr)-sizeof(CTLMCST));
		u.bfr[sizeof(u.bfr)-1]=0;
		break;

	case 's':
		if(req!=-1)usage();
		req=CTLDELMCST;
		if((u.i2.val2=atoi(optarg))==-1)usage();
		break;

	case 'a':
		if(req!=-1)usage();
		req=CTLDELLIST;
		strncpy(u.bfr,optarg,sizeof(u.bfr));
		u.bfr[sizeof(u.bfr)-1]=0;
		break;

	case 'A':
		if(req!=-1)usage();
		req=CTLADDLIST;
		strncpy(u.bfr,optarg,sizeof(u.bfr));
		u.bfr[sizeof(u.bfr)-1]=0;
		break;

	case 'r':
		if(req!=-1)usage();
		req=CTLDELRLIST;
		strncpy(u.bfr,optarg,sizeof(u.bfr));
		u.bfr[sizeof(u.bfr)-1]=0;
		break;

	case 'R':
		if(req!=-1)usage();
		req=CTLADDRLIST;
		strncpy(u.bfr,optarg,sizeof(u.bfr));
		u.bfr[sizeof(u.bfr)-1]=0;
		break;

	case 'h':
		if(req!=-1)usage();
		req=CTLDELHLIST;
		strncpy(u.bfr,optarg,sizeof(u.bfr));
		u.bfr[sizeof(u.bfr)-1]=0;
		break;

	case 'H':
		if(req!=-1)usage();
		req=CTLADDHLIST;
		strncpy(u.bfr,optarg,sizeof(u.bfr));
		u.bfr[sizeof(u.bfr)-1]=0;
		break;

	case 'm':
		if(req!=-1)usage();
		req=CTLDELMLIST;
		strncpy(u.bfr,optarg,sizeof(u.bfr));
		u.bfr[sizeof(u.bfr)-1]=0;
		break;

	case 'M':
		if(req!=-1)usage();
		req=CTLADDMLIST;
		strncpy(u.bfr,optarg,sizeof(u.bfr));
		u.bfr[sizeof(u.bfr)-1]=0;
		break;

	case 'u':
		if(req!=-1)usage();
		req=CTLTUNER;
		u.i2.val1=0;
		if((u.i2.val2=atoi(optarg))<1||u.i2.val2>65535)usage();
		break;

	case 'U':
		if(req!=-1)usage();
		req=CTLTUNER;
		u.i2.val1=1;
		if((u.i2.val2=atoi(optarg))<1||u.i2.val2>65535)usage();
		break;

	case 'X':
		if(req!=-1)usage();
		req=CTLRESTART;
		break;

	case 'I':
		if(req!=-1)usage();
		req=CTLINQ;
		break;

	case 'i':
		if(instance)usage();
		if((instance=atoi(optarg))<1||instance>32)usage();
		break;

	case 'x':
		if(ttl)usage();
		if((ttl=atoi(optarg))<1||ttl>255)usage();
		break;

	case 'P':
		if(play)usage();
		play=1;
		break;

	case 'p':
		if(fn)usage();
		fn=optarg;
		break;

	case 'o':
		if(req!=-1)usage();
		req=CTLSVRSTATS;
		break;

	case 'O':
		if(req!=-1)usage();
		req=CTLTUNERSTATS;
		break;

	case 'q':
		if(req!=-1)usage();
		req=CTLRTSPSTATS;
		break;

	case 'Q':
		if(req!=-1)usage();
		req=CTLHTTPSTATS;
		break;

	default:usage();
	}

	if(!fn||req==-1)usage();

	if(instance)instance--;

	if((s=doconnect(fn,1000))==-1)
	{
		fprintf(stderr,"Connect to %s failed\n",fn);
		goto err1;
	}

	switch(req)
	{
	case CTLRESTART:
	case CTLCLRALL:
	case CTLCLRRTSP:
	case CTLCLRHTTP:
	case CTLINQ:
	case CTLSVRSTATS:
	case CTLTUNERSTATS:
	case CTLRTSPSTATS:
	case CTLHTTPSTATS:
		if(sendreq(s,req,NULL,0))goto err2;
		break;

	case CTLRTSP:
	case CTLHTTP:
	case CTLCLRDEV:
		if(sendreq(s,req,&u,sizeof(CTLINT1)))goto err2;
		break;

	case CTLCLRSTREAM:
	case CTLDELMCST:
		u.i2.val1=instance;
	case CTLTUNER:
		if(sendreq(s,req,&u,sizeof(CTLINT2)))goto err2;
		break;

	case CTLADDRLIST:
	case CTLDELRLIST:
	case CTLADDHLIST:
	case CTLDELHLIST:
	case CTLADDMLIST:
	case CTLDELMLIST:
		if(sendreq(s,req,&u,strlen(u.bfr)))goto err2;
		break;

	case CTLADDMCST:
		u.m.srvid=instance;
		u.m.ttl=ttl;
		u.m.play=play;
		if(sendreq(s,req,&u,sizeof(CTLMCST)+strlen(u.m.url)))goto err2;
		break;
	}

	if(recvans(s,&ans,&data,&len,1000))goto err2;

	if(ans!=CTLOK)goto err3;

	switch(req)
	{
	case CTLADDMCST:
	case CTLINQ:
		if(len!=sizeof(CTLINT1))goto err3;
		memcpy(&u,data,sizeof(CTLINT1));
		printf("%d\n",u.i1.val);
		break;

	case CTLSVRSTATS:
		if(len!=sizeof(CTLINT5))goto err3;
		memcpy(&u,data,sizeof(CTLINT5));
		printf("Total RTSP Sessions: %d\n",u.i5.val1);
		printf("Client RTSP Sessions: %d\n",u.i5.val2);
		printf("Playing RTSP Sessions: %d\n",u.i5.val3);
		printf("Total HTTP Sessions: %d\n",u.i5.val4);
		printf("Playing HTTP Sessions: %d\n",u.i5.val5);
		break;

	case CTLTUNERSTATS:
		if(len%sizeof(CTLHWSTATS))goto err3;
		len/=sizeof(CTLHWSTATS);
		for(u.h=data,c=0;c<len;c++)
		{
			printf("Tuner %d\n",c+1);
			printf("System: %s\n",u.h[c].msys);
			printf("Capabilites: %s\n",u.h[c].caps);
			printf("Access: %d\n",u.h[c].access);
			printf("Streams: %d\n",u.h[c].streams);
			printf("Total Streams: %d\n",u.h[c].groupstreams);
			printf("Bytes: %llu\n",u.h[c].streambytes);
			printf("Time (ms):%llu00\n",u.h[c].byteupdates);
			if(u.h[c].open)
			{
				printf("Lock: %d\n",u.h[c].lock);
				printf("Level: %d\n",u.h[c].level);
				printf("Quality: %d\n",u.h[c].quality);
				printf("Tune Data: %s\n",u.h[c].tune);
			}
			printf("\n");
		}
		break;

	case CTLRTSPSTATS:
	case CTLHTTPSTATS:
		if(len%sizeof(CTLSVSTATS))goto err3;
		len/=sizeof(CTLSVSTATS);
		for(u.s=data,c=0;c<len;c++)
		{
			printf("Server: %d\n",u.s[c].instance+1);
			printf("Stream: %d\n",u.s[c].streamid);
			printf("Playing: %d\n",u.s[c].playing);
			if(u.s[c].port)
			{
				printf("Address: %s\n",u.s[c].addr);
				printf("Port: %d\n",u.s[c].port);
			}
			printf("Lock: %d\n",u.s[c].lock);
			printf("Level: %d\n",u.s[c].level);
			printf("Quality: %d\n",u.s[c].quality);
			printf("Tune Data: %s\n",u.s[c].tune);
			printf("\n");
		}
		break;

	default:if(len)goto err3;
	}

	res=0;

err3:	if(data)free(data);
err2:	close(s);
err1:	if(res)fprintf(stderr,"Operation failed\n");
	return res;
}
