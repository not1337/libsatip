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
"satipmcast -a -h host|-u uuid [-p port] [-l level] [-m ttl] [-i interface]\n"
"           [-P] URL\n"
"satipmcast -d -h host|-u uuid [-p port] [-l level] [-m ttl] [-i interface]\n"
"           -s stream -S session\n"
"-a               add multicast stream\n"
"-d               delete multicast stream\n"
"-u uuid          UUID of SAT>IP server\n"
"-h host          SAT>IP server host name or address\n"
"-p port          SAT>IP server port, default 554\n"
"-m ttl           multicast ttl, default 2\n"
"-P               instant multicast stream start\n"
"-l level         0=IPv4,1=0+IPv6 Link Layer,2=1+ IPv6 ULA addresses, default 0\n"
"-i interface     device to be used, default eth0\n"
"-s stream        stream returned when the multicast was created\n"
"-S session       session returned when the multicast was created\n"
"-I               enforce strict SAT>IP query syntax compliance\n"
"URL              a SAT>IP rtp url including channel query data\n");
	exit(1);
}

int main(int argc,char *argv[])
{
	int c;
	int res=1;
	int mttl=0;
	int port=0;
	int play=0;
	int mode=0;
	int mport;
	int stream=-1;
	int type;
	char *host=NULL;
	char *uuid=NULL;
	void *ch;
	SATIP_CLN_UPNPLIST *ulist=NULL;
	SATIP_CLN_UPNPLIST *ue;
	SATIP_CLN_UPNPLIST *uf;
	char maddr[128];
	char sess[128];
	SATIP_TUNE tune;
	SATIP_PIDS set;
	SATIP_PIDS add;
	SATIP_PIDS del;
	SATIP_CLN_CONFIG conf;

	memset(&conf,0,sizeof(conf));
	*sess=0;

	while((c=getopt(argc,argv,"adh:p:m:Pl:i:s:S:u:I"))!=-1)switch(c)
	{
	case 'a':
		if(mode)usage();
		mode=1;
		break;

	case 'd':
		if(mode)usage();
		mode=2;
		break;

	case 'h':
		if(host||uuid)usage();
		host=optarg;
		if(!*host)usage();
		break;

	case 'p':
		if(port)usage();
		if((port=atoi(optarg))<1||port>65535)usage();
		break;

	case 'm':
		if(mttl)usage();
		if((mttl=atoi(optarg))<1||mttl>255)usage();
		break;

	case 'P':
		if(play)usage();
		play=1;
		break;

	case 'l':
		if(conf.level)usage();
		if((conf.level=atoi(optarg))<0||conf.level>2)usage();
		conf.level++;
		break;

	case 'i':
		if(*conf.dev||!*optarg)usage();
		strncpy(conf.dev,optarg,sizeof(conf.dev)-1);
		break;

	case 's':
		stream=atoi(optarg);
		break;

	case 'S':
		snprintf(sess,sizeof(sess),"%s",optarg);
		break;

	case 'u':
		if(host||uuid)usage();
		uuid=optarg;
		if(!*uuid)usage();
		break;

	case 'I':
		if(conf.strict)usage();
		conf.strict=1;
		break;

	default:usage();
	}

	if(!port)port=SATIP_RTSP_PORT;
	if(!mttl)mttl=SATIP_MCST_TTL;
	if(conf.level)conf.level--;
	if(!*conf.dev)strcpy(conf.dev,"eth0");
	conf.mttl=mttl;
	conf.interval=uuid?1:SATIP_UPNP_OFF;

	if(!mode||(!host&&!uuid))usage();

	switch(mode)
	{
	case 1:
		if(optind>=argc)usage();

		satip_util_init_tune(&tune);
		satip_util_init_pids(&set);
		satip_util_init_pids(&add);
		satip_util_init_pids(&del);

		if(satip_util_parse(SATIP_PARSE_URL,0,
			SATIP_IGNCAPS|SATIP_RTPQUERY,
			argv[optind],&type,maddr,sizeof(maddr),&mport,
			&stream,&tune,&set,&add,&del,NULL)||add.numpids||
			del.numpids||set.numpids<1||stream!=SATIP_UNDEF||
			type!=SATIP_TYPE_RTP)
		{
			fprintf(stderr,"Bad URL\n");
			return 1;
		}
		break;

	case 2:
		if(stream==-1||!*sess)usage();
		break;
	}

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

	switch(mode)
	{
	case 1:	if(satip_cln_setup_multicast(ch,host,port,maddr,mport,mttl,play,
			sess,sizeof(sess),&stream,&tune,&set,NULL))
		{
			fprintf(stderr,"satip_cln_setup_multicast failure\n");
			goto err2;
		}
		printf("session=%s streamid=%d\n",sess,stream);
		break;

	case 2:	if(satip_cln_end_multicast(ch,host,port,sess,stream))
		{
			fprintf(stderr,"satip_cln_end_multicast failure\n");
			goto err2;
		}
		break;
	}

	res=0;

err2:	if(ulist)satip_cln_freelist(ch,ulist);
	satip_cln_fini(ch);
err1:	return res;
}
