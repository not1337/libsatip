/*
 * example SAT>IP UPNP client
 *
 * Copyright (c) 2016 Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "satip.h"

#ifndef __GNUC__
#define __attribute__(x)
#endif

static void usage(void) __attribute__ ((noreturn));

static void usage(void)
{
	fprintf(stderr,"Usage:\n"
"satipinfo -h host|-u uuid [-p port] [-i interface] [-l level] [-m mttl]\n"
"-u uuid         UUID of SAT>IP host\n"
"-h host         host name or IP of SAT>IP host\n"
"-p port         port of SAT>IP host, default 554\n"
"-m mttl         multicast ttl, default 2\n"
"-i interface    network interface to use, default eth0\n"
"-l level        0=IPv4,1=0+IPv6 Link Layer,2=1+ IPv6 "
	"ULA addresses, default 0\n"
"-I              enforce strict SAT>IP query syntax compliance\n");
	exit(1);
}

int main(int argc,char *argv[])
{
	int res=1;
	int c;
	int port=0;
	int level=0;
	int mttl=0;
	int strict=0;
	char *dev=NULL;
	char *host=NULL;
	char *uuid=NULL;
	void *ch;
	SATIP_CLN_UPNPLIST *ulist=NULL;
	SATIP_CLN_UPNPLIST *ue;
	SATIP_CLN_UPNPLIST *uf;
	SATIP_CLN_STREAMINFO *info;
	SATIP_CLN_STREAMINFO *e;
	SATIP_CLN_CONFIG conf;
	char tmp[1024];

	while((c=getopt(argc,argv,"h:p:i:l:u:m:I"))!=-1)switch(c)
	{
	case 'h':
		if(host||uuid)usage();
		host=optarg;
		break;

	case 'p':
		if(port)usage();
		if((port=atoi(optarg))<1||port>65535)usage();
		break;

	case 'i':
		if(dev||!*optarg)usage();
		dev=optarg;
		break;

	case 'l':
		if(level)usage();
		if((level=atoi(optarg))<0||level>2)usage();
		level++;
		break;

	case 'u':
		if(host||uuid)usage();
		uuid=optarg;
		break;

	case 'm':
		if(mttl)usage();
		if((mttl=atoi(optarg))<1||mttl>255)usage();
		break;

	case 'I':
		if(strict)usage();
		strict=1;
		break;
	}

	if(!dev)dev="eth0";
	if(!port)port=SATIP_RTSP_PORT;
	if(!mttl)mttl=SATIP_MCST_TTL;

	if(!host&&!uuid)usage();

	memset(&conf,0,sizeof(conf));

	conf.level=level?level-1:0;
	strncpy(conf.dev,dev,sizeof(conf.dev)-1);
	conf.mttl=mttl;
	conf.interval=uuid?1:SATIP_UPNP_OFF;

	if(!(ch=satip_cln_init(&conf)))
	{
		fprintf(stderr,"satip_cln_init failure\n");
		goto err1;
	}

	if(uuid)for(c=0;c<80&&!host;c++)
	{
		usleep(25000);
		if(!satip_cln_upnplist(ch,&ulist))for(ue=ulist;ue;ue=ue->next)
			if(!strcmp(ue->uuid,uuid))
		{
			for(uf=ue,ue=ue->same;ue;ue=ue->same)
				if(ue->level>uf->level)uf=ue;
			host=uf->addr;
			break;
		}
		if(!host)satip_cln_freelist(ch,ulist);
	}

	if(!host)
	{
		fprintf(stderr,"no device found for uuid %s\n",uuid);
		goto err2;
	}

	if(satip_cln_streaminfo(ch,host,port,&info))
	{
		fprintf(stderr,"satip_cln_streaminfo failure\n");
		goto err2;
	}

	if(info)printf("Stream Info:\n");

	for(e=info;e;e=e->next)
	{
		if(satip_util_create(SATIP_TYPE_QRY,strict?SATIP_STRICTQRY:0,
			NULL,0,0,&e->info.tune,&e->info.set,NULL,NULL,NULL,tmp,
			sizeof(tmp))<=0)strcpy(tmp," <unknown>");

		switch(e->type)
		{
		case SATIP_TYPE_RTSP:
			printf("\nStream %d (RTSP, %splaying):\n"
				"Signal Lock %d    Signal Level %d     "
				"Signal Quality %d\nTuning Data: %s\n",
				e->stream,e->inactive?"not ":"",e->info.lock,
				e->info.level,e->info.quality,tmp+1);
			break;

		case SATIP_TYPE_RTP:
			printf("\nStream %d (RTP, IP: %s Port: %d TTL: %d, "
				"%splaying):\nSignal Lock %d    Signal Level "
				"%d     Signal Quality %d\nTuning Data: %s\n",
				e->stream,e->addr,e->port,e->ttl,
				e->inactive?"not ":"",e->info.lock,
				e->info.level,e->info.quality,tmp+1);
			break;

		default:printf("\nStream %d (unknown, %splaying):\n"
				"Signal Lock %d    Signal Level %d     "
				"Signal Quality %d\nTuning Data: %s\n",
				e->stream,e->inactive?"not ":"",e->info.lock,
				e->info.level,e->info.quality,tmp+1);
			break;
		}
	}

	res=0;

	satip_cln_streaminfo_free(info);
err2:	if(ulist)satip_cln_freelist(ch,ulist);
	satip_cln_fini(ch);
err1:	return res;
}
