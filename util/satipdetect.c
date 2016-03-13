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

#include <limits.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "satip.h"

static void printout(void *h,SATIP_CLN_UPNPLIST *list)
{
	SATIP_CLN_UPNPLIST *e;
	SATIP_CLN_UPNPLIST *s;
	SATIP_CLN_UPNPINFO *info;

	for(e=list;e;e=e->next)
	{
		printf("uuid: %s\n",e->uuid);
		for(s=e;s;s=s->same)
		{
			printf("    addr=%s boot=%d id=%d\n    url=%s\n",
				s->addr,s->bootcount,s->configid,s->location);
			if(!(satip_cln_upnpinfo(h,s,&info)))
			{
				printf("\tDVBS2: %d DVBT: %d DVBT2: "
					"%d DVBC: %d DVBC2: %d\n",
					info->totals[0],info->totals[1],
					info->totals[2],info->totals[3],
					info->totals[4]);
				printf("\tfriendlyname: %s\n",
					info->friendlyname);
				printf("\tpresentationurl: %s\n",
					info->presentationurl);
				printf("\tplaylisturl: %s\n",
					info->playlisturl);
				printf("\tpng48url: %s\n",info->png48url);
				printf("\tpng120url: %s\n",info->png120url);
				printf("\tjpg48url: %s\n",info->jpg48url);
				printf("\tjpg120url: %s\n",info->jpg120url);
				printf("\tmanufacturername: %s\n",
					info->manufacturername);
				printf("\tmanufacturerurl: %s\n",
					info->manufacturerurl);
				printf("\tmodelname: %s\n",info->modelname);
				printf("\tmodelnumber: %s\n",
					info->modelnumber);
				printf("\tmodelurl: %s\n",info->modelurl);
				printf("\tmodeldescription: %s\n",
					info->modeldescription);
				printf("\tserial: %s\n",info->serial);
				printf("\tupc: %s\n",info->upc);
				satip_cln_freeinfo(h,info);
			}
			else printf("\t<no data>\n");
		}
	}
}

static void fetcher(void *h,SATIP_CLN_UPNPLIST *list)
{
	SATIP_CLN_UPNPLIST *e;
	SATIP_CLN_UPNPLIST *s;
	SATIP_CLN_UPNPINFO *info;
	FILE *fp;
	void *data;
	int size;
	char bfr[PATH_MAX];

	for(e=list;e;e=e->next)for(s=e;s;s=s->same)
		if(!(satip_cln_upnpinfo(h,s,&info)))
	{
		if(info->playlisturl[0])
			if(!satip_cln_upnpitem(h,info->playlisturl,&data,&size))
		{
			snprintf(bfr,sizeof(bfr),"%s.%d.m3u",e->uuid,s->level);
			if((fp=fopen(bfr,"we")))
			{
				fwrite(data,size,1,fp);
				fclose(fp);
			}
			free(data);
		}
		satip_cln_freeinfo(h,info);
	}
}

int main(int argc,char *argv[])
{
	int c;
	int level=0;
	int m3u=0;
	int mttl=SATIP_MCST_TTL;
	char *ifc="eth0";
	void *h;
	SATIP_CLN_CONFIG conf;
	SATIP_CLN_UPNPLIST *list;

	while((c=getopt(argc,argv,"i:m:l:hM"))!=-1)switch(c)
	{
	case 'i':
		ifc=optarg;
		break;
	case 'm':
		mttl=atoi(optarg);
		if(mttl<=0||mttl>255)mttl=SATIP_MCST_TTL;
		break;
	case 'l':
		level=atoi(optarg);
		if(level>=0&&level<=2)break;
	case 'M':
		m3u=1;
		break;
	case 'h':
	default:fprintf(stderr,
"Usage: satipdetect [-l level] [-m multicast-ttl] [-i interface] [-h]\n"
"-l level           0=IPv4, 1=0+IPv6 link layer, 2=1+IPv6 ULA addresses\n"
"-m multicast-ttl   multicast ttl to use, default %d\n"
"-i interface       interface to use, default eth0\n"
"-M                 fetch M3U lists as UTF-8 from server, if available\n"
"-h                 this help text\n",SATIP_MCST_TTL);
		return 1;
	}

	memset(&conf,0,sizeof(conf));
	strncpy(conf.dev,ifc,sizeof(conf.dev)-1);
	conf.level=level;
	conf.mttl=mttl;
	conf.interval=1;

	if(!(h=satip_cln_init(&conf)))
	{
		fprintf(stderr,"satip_cln_init failed\n");
		return 1;
	}

	printf("Please wait...");
	fflush(stdout);
	sleep(2);
	printf("\r              \r");
	fflush(stdout);

	if(satip_cln_upnplist(h,&list))
	{
		fprintf(stderr,"satip_cln_upnplist failed\n");
		satip_cln_fini(h);
		return 1;
	}

	printout(h,list);

	if(m3u)fetcher(h,list);

	satip_cln_freelist(h,list);
	satip_cln_fini(h);

	return 0;
}
