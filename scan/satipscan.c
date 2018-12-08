/*
 * example SAT>IP transponder scanner
 *
 * Copyright (c) 2016 Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 */

#include <pthread.h>
#include <sys/signalfd.h>
#include <sys/capability.h>
#include <sys/resource.h>
#include <sys/prctl.h>
#include <sys/uio.h>
#include <iconv.h>
#include <signal.h>
#include <stddef.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include "satip.h"

#define MAX_TUNERS 32
#define MAX_TRANSPONDERS 128

#define TESTCAP(a,cap) \
	(cap>31?(a[1].effective&(1<<(cap-32))):(a[0].effective&(1<<cap)))

#define SETCAP(a,cap) \
do \
{ \
	int idx=(cap>31?1:0); \
	int bit=(cap>31?(1<<(cap-32)):(1<<cap)); \
	a[idx].effective|=bit; \
	a[idx].permitted|=bit; \
	a[idx].inheritable|=bit; \
} while(0)

typedef struct
{
	int portmin;
	int portmax;
	int tnum;
	int dvbsnum;
	int dvbtnum;
	int dvbcnum;
	SATIP_SCAN_PARAMS cfg;
	SATIP_HW_CONFIG global;
	SATIP_HW_TUNERCFG tuner[MAX_TUNERS];
	SATIP_TUNE dvbs[MAX_TRANSPONDERS];
	SATIP_TUNE dvbt[MAX_TRANSPONDERS];
	SATIP_TUNE dvbc[MAX_TRANSPONDERS];
} CONFIG;

typedef struct
{
	char *name;
	int min;
	int max;
	unsigned long offset;
	int (*func)(void *s,char *val);
} CFGFILE;

typedef struct
{
	int mode;
	int header;
	int tty;
	pthread_mutex_t mtx;
} STATE;

typedef struct
{
	int tv;
	int radio;
	int fta;
	int nonfta;
	int num;
	int eit;
	int idx;
	int size;
	SATIP_UTIL_PROGRAM **list;
} PROGS;

#ifndef __GNUC__
#define __attribute__(x)
#endif

#ifndef PROFILE

static int setpriv(int drop) __attribute__ ((cold));
static int doinv(void *s,char *val) __attribute__ ((cold));
static int dosrc(void *s,char *val) __attribute__ ((cold));
static int doparam(CFGFILE *p,void *s,char *name,char *val)
	__attribute__ ((cold));
static int parse_config(char *fn,CONFIG *c,int sfd) __attribute__ ((cold));
static void usage(void) __attribute__ ((cold)) __attribute__ ((noreturn));

#else

static int doinv(void *s,char *val);
static int dosrc(void *s,char *val);
static void usage(void) __attribute__ ((noreturn));

#endif

static CFGFILE tparams[]=
{
	{"deviceid",1,65535,offsetof(SATIP_HW_TUNERCFG,deviceid),NULL},
	{"adapter",0,255,offsetof(SATIP_HW_TUNERCFG,adapter),NULL},
	{"frontend",0,255,offsetof(SATIP_HW_TUNERCFG,frontend),NULL},
	{"demux",0,255,offsetof(SATIP_HW_TUNERCFG,demux),NULL},
	{"ca",0,255,offsetof(SATIP_HW_TUNERCFG,ca),NULL},
	{"inversion",0,0,offsetof(SATIP_HW_TUNERCFG,inversion),doinv},
	{"lna",0,1,offsetof(SATIP_HW_TUNERCFG,lna),NULL},
	{"snrshift",0,15,offsetof(SATIP_HW_TUNERCFG,snrshift),NULL},
	{"snrspeed",0,2,offsetof(SATIP_HW_TUNERCFG,snrspeed),NULL},
	{"lnbtype",0,4,offsetof(SATIP_HW_TUNERCFG,lnbtype),NULL},
	{"dvbctype",0,0,offsetof(SATIP_HW_TUNERCFG,dvbctype),NULL},
	{"diseqc",0,9,offsetof(SATIP_HW_TUNERCFG,diseqc),NULL},
	{"dscforce",0,1,offsetof(SATIP_HW_TUNERCFG,dscforce),NULL},
	{"dscwait",15,100,offsetof(SATIP_HW_TUNERCFG,dscwait),NULL},
	{"dscreplywait",50,250,offsetof(SATIP_HW_TUNERCFG,dscreplywait),NULL},
	{"source",0,0,offsetof(SATIP_HW_TUNERCFG,src),dosrc},
	{NULL,0,0,0L,NULL}
};

static CFGFILE scparams[]=
{
	{"locktimeout",0,60,offsetof(SATIP_SCAN_PARAMS,locktimeout),NULL},
	{"mintimeout",0,60,offsetof(SATIP_SCAN_PARAMS,mintimeout),NULL},
	{"basetimeout",0,60,offsetof(SATIP_SCAN_PARAMS,basetimeout),NULL},
	{"maxtimeout",0,60,offsetof(SATIP_SCAN_PARAMS,maxtimeout),NULL},
	{"pmttimeout",0,60,offsetof(SATIP_SCAN_PARAMS,pmttimeout),NULL},
	{"pmtparallel",0,32,offsetof(SATIP_SCAN_PARAMS,pmtparallel),NULL},
	{NULL,0,0,0L,NULL}
};

#define TESTCAP(a,cap) \
	(cap>31?(a[1].effective&(1<<(cap-32))):(a[0].effective&(1<<cap)))

#define SETCAP(a,cap) \
do \
{ \
	int idx=(cap>31?1:0); \
	int bit=(cap>31?(1<<(cap-32)):(1<<cap)); \
	a[idx].effective|=bit; \
	a[idx].permitted|=bit; \
	a[idx].inheritable|=bit; \
} while(0)

static int setpriv(int drop)
{
	int csn=1;
	struct rlimit r;
	struct __user_cap_header_struct hdr;
	struct __user_cap_data_struct cap[2];

#ifdef PROFILE
	fprintf(stderr,"warning: profiling, privilege setting disabled\n");
	return 0;
#endif
	memset(&hdr,0,sizeof(hdr));

	hdr.version=_LINUX_CAPABILITY_VERSION_3;
	hdr.pid=getpid();

	if(drop)
	{
		memset(cap,0,sizeof(cap));
		if(capset(&hdr,cap))
		{
			perror("capset");
			return -1;
		}
		return 0;
	}

	if(capget(&hdr,cap))
	{
		perror("capget");
		return -1;
	}

	if(!TESTCAP(cap,CAP_SYS_NICE))
	{
		csn=0;
		fprintf(stderr,"warning: may need CAP_SYS_NICE\n");
	}

	if(TESTCAP(cap,CAP_SYS_RESOURCE))
	{
		r.rlim_cur=RLIM_INFINITY;
		r.rlim_max=RLIM_INFINITY;

		if(setrlimit(RLIMIT_NICE,&r))
		{
			perror("setrlimit(RLIMIT_NICE)");
			return -1;
		}

		if(setrlimit(RLIMIT_RTPRIO,&r))
		{
			perror("setrlimit(RLIMIT_RTPRIO)");
			return -1;
		}

		if(setrlimit(RLIMIT_RTTIME,&r))
		{
			perror("setrlimit(RLIMIT_RTTIME)");
			return -1;
		}
	}

	if(prctl(PR_SET_KEEPCAPS,1,0,0,0))
	{
		perror("prctl(PR_SET_KEEPCAPS)");
		return -1;
	}

	memset(cap,0,sizeof(cap));

	if(csn)SETCAP(cap,CAP_SYS_NICE);

	if(capset(&hdr,cap))
	{
		perror("capset");
		return -1;
	}

	if(prctl(PR_SET_NO_NEW_PRIVS,1,0,0,0))
		fprintf(stderr,
			"warning: PR_SET_NO_NEW_PRIVS fail, old kernel?\n");

	return 0;
}

static int doinv(void *s,char *val)
{
	int inv;

	inv=atoi(val);
	if(inv<0||inv>1)return -1;
	if(!inv)inv=SATIP_SPI_OFF;
	else inv=SATIP_SPI_ON;
	*((int *)s)=inv;
	return 0;
}

static int dosrc(void *s,char *val)
{
	int i;
	char *mem;
	int *src=(int *)s;

	for(i=0,val=strtok_r(val,",",&mem);val;i++,val=strtok_r(NULL,",",&mem))
	{
		if(!*val||i>=32)return -1;
		src[i]=atoi(val);
		if(src[i]<1||src[i]>255)return -1;
	}
	if(!i)return -1;
	src[-1]=i;
	return 0;
}

static int doparam(CFGFILE *p,void *s,char *name,char *val)
{
	int i=0;
	int v;

	for(i=0;p[i].name;i++)if(!strcmp(p[i].name,name))
	{
		if(p[i].func)
			return p[i].func(((unsigned char *)s)+p[i].offset,val);
		else
		{
			v=atoi(val);
			if(v<p[i].min||v>p[i].max)return -1;
			*((int *)(((unsigned char *)s)+p[i].offset))=v;
			return 0;
		}
	}

	return -1;
}

static int parse_config(char *fn,CONFIG *c,int sfd)
{
	int i;
	int j;
	int k;
	int l;
	int line=0;
	int glob=0;
	char *name;
	char *val;
	char *mem;
	FILE *fp;
	SATIP_HW_TUNERCFG *tuner=NULL;
	SATIP_HW_TUNERCFG *cmp;
	SATIP_TUNE *dvbs=NULL;
	SATIP_TUNE *dvbt=NULL;
	SATIP_TUNE *dvbc=NULL;
	SATIP_PIDS set;
	SATIP_PIDS add;
	SATIP_PIDS del;
	char bfr[8192];

	memset(c,0,sizeof(CONFIG));

	if(!(fp=fopen(fn,"re")))
	{
		fprintf(stderr,"Can't access %s\n",fn);
		goto err1;
	}

	while(fgets(bfr,sizeof(bfr),fp))
	{
		line++;
		name=strtok_r(bfr," \t\r\n",&mem);
		if(!name||!*name||*name=='#')continue;

		if(!strcmp(name,"[global]"))
		{
			if(glob==-1)goto err3;
			glob=1;
			tuner=NULL;
			dvbs=NULL;
			dvbt=NULL;
			dvbc=NULL;
			continue;
		}

		if(!strcmp(name,"[tuner]"))
		{
			glob=-1;
			dvbs=NULL;
			dvbt=NULL;
			dvbc=NULL;
			if(c->tnum==MAX_TUNERS)
			{
				fprintf(stderr,"Out of memory\n");
				goto err2;
			}
			tuner=&c->tuner[c->tnum++];
			tuner->inversion=SATIP_SPI_OFF;
			tuner->lna=SATIP_LNA_AUTO;
			continue;
		}

		if(!strcmp(name,"[dvbs]"))
		{
			glob=-1;
			tuner=NULL;
			dvbt=NULL;
			dvbc=NULL;
			dvbs=&c->dvbs[c->dvbsnum];
			continue;
		}

		if(!strcmp(name,"[dvbt]"))
		{
			glob=-1;
			tuner=NULL;
			dvbs=NULL;
			dvbc=NULL;
			dvbt=&c->dvbt[c->dvbtnum];
			continue;
		}

		if(!strcmp(name,"[dvbc]"))
		{
			glob=-1;
			tuner=NULL;
			dvbs=NULL;
			dvbt=NULL;
			dvbc=&c->dvbc[c->dvbcnum];
			continue;
		}

		if(*name=='?')
		{
			val=name;
			name="tune";
		}
		else
		{
			if(!(val=strchr(name,'=')))goto err3;
			*val++=0;

			name=strtok_r(name," \t",&mem);
			val=strtok_r(val," \t",&mem);
		}

		if(!name||!*name||!val||!*val)goto err3;

		if(glob==1)
		{
			if(!strcmp(name,"rtprio"))
			{
				c->global.rtprio=atoi(val);
				if(c->global.rtprio<0||c->global.rtprio>99)
					goto err3;
			}
			else if(!strcmp(name,"portmin"))
			{
				c->portmin=atoi(val);
				if(c->portmin<1024||c->portmin>65534)goto err3;
			}
			else if(!strcmp(name,"portmax"))
			{
				c->portmax=atoi(val);
				if(c->portmax<1025||c->portmax>65535)goto err3;
			}
			else if(doparam(scparams,&c->cfg,name,val))goto err3;
		}
		else if(tuner)
		{
			if(doparam(tparams,tuner,name,val))goto err3;
		}
		else if(dvbs)
		{
			if(strcmp(name,"tune"))goto err3;
			if(c->dvbsnum==MAX_TRANSPONDERS)
			{
				fprintf(stderr,"Out of memory\n");
				goto err2;
			}
			satip_util_init_tune(&c->dvbs[c->dvbsnum]);
			if(satip_util_parse(SATIP_PARSE_QRY,0,SATIP_IGNCAPS,
				val+1,NULL,NULL,0,NULL,NULL,
				&c->dvbs[c->dvbsnum++],&set,&add,&del,NULL)||
				set.numpids!=SATIP_NOPIDS||add.numpids||
				del.numpids)goto err3;
		}
		else if(dvbt)
		{
			if(strcmp(name,"tune"))goto err3;
			if(c->dvbtnum==MAX_TRANSPONDERS)
			{
				fprintf(stderr,"Out of memory\n");
				goto err2;
			}
			satip_util_init_tune(&c->dvbt[c->dvbtnum]);
			if(satip_util_parse(SATIP_PARSE_QRY,0,
				SATIP_IGNCAPS|SATIP_IGNPLPETC,
				val+1,NULL,NULL,0,NULL,NULL,
				&c->dvbt[c->dvbtnum++],&set,&add,&del,NULL)||
				set.numpids!=SATIP_NOPIDS||add.numpids||
				del.numpids)goto err3;
		}
		else if(dvbc)
		{
			if(strcmp(name,"tune"))goto err3;
			if(c->dvbcnum==MAX_TRANSPONDERS)
			{
				fprintf(stderr,"Out of memory\n");
				goto err2;
			}
			satip_util_init_tune(&c->dvbc[c->dvbcnum]);
			if(satip_util_parse(SATIP_PARSE_QRY,0,
				SATIP_IGNCAPS|SATIP_IGNPLPETC,
				val+1,NULL,NULL,0,NULL,NULL,
				&c->dvbc[c->dvbcnum++],&set,&add,&del,NULL)||
				set.numpids!=SATIP_NOPIDS||add.numpids||
				del.numpids)goto err3;
		}
		else goto err3;
	}

	for(k=0;k<c->tnum;k++)
	{
		tuner=&c->tuner[k];

		if(!tuner->deviceid)goto err4;

		if(tuner->srcnum)
		{
			for(i=tuner->srcnum-1;i;i--)for(j=i-1;j>=0;j--)
				if(tuner->src[i]==tuner->src[j])goto err4;

			for(j=(tuner->src[0]-1)>>2,i=1;i<tuner->srcnum;i++)
				if(((tuner->src[i]-1)>>2)!=j)goto err4;
		}

		for(l=k+1;l<c->tnum;l++)
		{
			cmp=&c->tuner[l];

			if(tuner->frontend==cmp->frontend||
				tuner->demux==cmp->demux||tuner->ca==cmp->ca)
					if(tuner->adapter==cmp->adapter)
						goto err4;
			if(tuner->deviceid==cmp->deviceid)goto err4;
		}
	}

	c->cfg.termfd=sfd;

	posix_fadvise(fileno(fp),0,0,POSIX_FADV_DONTNEED);
	fclose(fp);
	return 0;

err4:	fprintf(stderr,"Syntax error in tuner definition\n");
	goto err2;
err3:	fprintf(stderr,"Syntax error in line %d of %s\n",line,fn);
err2:	fclose(fp);
err1:	return -1;
}

static int next(SATIP_UTIL_PROGRAM **p,int *user_prognum,void *priv)
{
	PROGS *prg=(PROGS *)priv;

	for(;prg->idx<prg->size;prg->idx++)
	{
		if(!prg->list[prg->idx]->streams)continue;
		if(prg->tv&&!prg->list[prg->idx]->tv)continue;
		if(prg->radio&&!prg->list[prg->idx]->radio)continue;
		if(prg->fta&&
			(prg->list[prg->idx]->fta||prg->list[prg->idx]->sdtfta))
				goto work;
		if(prg->nonfta&&!prg->list[prg->idx]->fta&&
			!prg->list[prg->idx]->sdtfta)goto work;
		continue;

work:		*user_prognum=prg->num++;
		*p=prg->list[prg->idx++];
		if(!prg->eit)(*p)->m3u_eit_exclude=1;
		return 1;
	}

	return 0;
}

static void scanstate(SATIP_TUNE *t,SATIP_SCAN_STATUS *st,void *priv)
{
	int fh;
	int fl;
	char pol=0;
	char *msys;
	STATE *state=(STATE *)priv;

	pthread_mutex_lock(&state->mtx);

	if(state->mode)
	{
		fh=t->freq/1000000000ULL;
		fl=(t->freq%1000000000ULL)/1000000ULL;

		switch(t->pol)
		{
		case SATIP_POL_H:
			pol='H';
			break;
		case SATIP_POL_V:
			pol='V';
			break;
		case SATIP_POL_L:
			pol='L';
			break;
		case SATIP_POL_R:
			pol='R';
		}

		if(t->msys==SATIP_DVBS)msys="DVB-S ";
		else msys="DVB-S2";
	}
	else
	{
		fh=t->freq/1000000ULL;
		fl=(t->freq%1000000ULL)/1000ULL;
	}

	if(!state->header)
	{
		state->header=1;
		if(state->mode)fprintf(stderr,"Frequency  Pol. Symbols  System "
			"Lock Level Quality Device Done/Items Done/Total\n");
		else fprintf(stderr," Frequency   Lock Level Quality Device "
			"Done/Items Done/Total\n");
	}

	if(!state->tty&&st->info!=SATIP_SCAN_CBF);
	else if(state->mode)fprintf(stderr,"\r%2d.%03d GHz %c    %8d %s %d    "
		"%3d    %3d    %3d    %3d /  %3d %3d / %3d\t",fh,fl,pol,
		t->sr,msys,st->lock,st->level,st->quality,st->deviceid,st->done,
		st->total,st->globdone,st->globtotal);
	else fprintf(stderr,"\r%4d.%03d MHz %d    %3d   %3d     %3d    %3d /  "
		"%3d %3d /  %3d\t",fh,fl,st->lock,st->level,st->quality,
		st->deviceid,st->done,st->total,st->globdone,st->globtotal);

	if(st->info==SATIP_SCAN_CBF)
	{
		if(!st->lock)fprintf(stderr,"#");
		else if(st->done!=st->total)fprintf(stderr,"*");
		fprintf(stderr,"\n");
	}

	fflush(stdout);

	pthread_mutex_unlock(&state->mtx);
}

static void usage(void)
{
fprintf(stderr,
"Usage:\n\n"
"satipscan [-c config-file] -S|-T|-C [-s source] [-P parallel-tuners]\n"
"	[-t tuner-list] [-L|-R|-H] [-u uuid|-h host] [-p port] [-i interface]\n"
"	[-X mttl] [-l network-level] [-O|-F] [-m|-M] [-o locale] [-x] [-v]\n"
"	[-r] [-f] [-n] [-N program-number] [-b file]\n"
"\n"
"-c config-file      configuration file, default /etc/satipscan.conf\n"
"-S                  do a DVB-S scan\n"
"-T                  do a DVB-T scan\n"
"-C                  do a DVB-C scan\n"
"-s source           SAT>IP source number (DVB-S only)\n"
"-P parallel-tuners  number of parallel tuners to use, default 1\n"
"-t tuner-list       comma separated tuner device list, amount must match -P\n"
"-L                  use local hardware\n"
"-R                  scan via SAT>IP in RTSP mode\n"
"-H                  scan via SAT>IP in HTTP mode "
				"(only against libsatip server)\n"
"-u uuid             SAT>IP host UUID\n"
"-I                  enforce strict SAT>IP query syntax compliance\n"
"-h host             SAT>IP host name or IP\n"
"-p port             SAT>IP port number, default 554 (RTSP) or 80 (HTTP)\n"
"-i interface        network device to use, default eth0\n"
"-l network-level    0=IPv4, 1=0+IPv6 link layer, 2=1+IPv6 ULA address range\n"
"-X mttl             multicast ttl, default 2\n"
"-O                  output optimized transponder list\n"
"-F                  output full transponder list\n"
"-m                  output RTSP m3u\n"
"-M                  output HTTP m3u\n"
"-o locale           locale, default is de_DE.ISO-8859-15\n"
"-x                  output x_sid for libsatip based servers (pid tracking)\n"
"-e                  add eit pid to output if eit is available\n"
"-d                  add pmt pid to output (e.g. for Digital Devices)\n"
"-D                  add 'x_pmt=<pmt>' to output (e.g. for Digital Devices)\n"
"-v                  list tv programs\n"
"-r                  list radio programs\n"
"-f                  list fta programs\n"
"-n                  list non-fta programs\n"
"-N program-number   initial program number, default 1\n"
"-b file             output scan result in binary format to file\n"
);
	exit(1);
}

int main(int argc,char *argv[])
{
	int res=1;
	int l;
	int i;
	int sfd;
	int bfd;
	int r;
	int total;
	int m3usize;
	int scanmode;
	int src=0;
	int tuners[MAX_TUNERS];
	int tnum=0;
	int para=0;
	int mode=0;
	int lmode=0;
	int mmode=0;
	int level=0;
	int flags=0;
	int net=0;
	int port=0;
	int tv=0;
	int radio=0;
	int fta=0;
	int nonfta=0;
	int num=0;
	int eit=0;
	int mttl=0;
	char *host=NULL;
	char *uuid=NULL;
	char *dev=NULL;
	char *locale=NULL;
	char *cfgfile=NULL;
	char *m3umem=NULL;
	char *binfile=NULL;
	void *h;
	char *p;
	struct iovec iov[4];
	SATIP_CLN_UPNPLIST *ulst=NULL;
	SATIP_CLN_UPNPLIST *e;
	SATIP_CLN_UPNPLIST *f;
	SATIP_SCAN_RESULT *lst;
	SATIP_CFGINFO hwinfo;
	SATIP_CLN_CONFIG clnconf;
	SATIP_TUNE tune[MAX_TRANSPONDERS];
	CONFIG conf;
	STATE state;
	PROGS prg;
	sigset_t set;
	char bfr[1024];

	if(setpriv(0))goto err1;

	while((l=getopt(argc,argv,
		"c:STCs:t:P:LRHh:p:i:l:OFo:MmxvrfnN:edDb:u:X:I"))!=-1)switch(l)
	{
	case 'c':
		if(cfgfile)usage();
		cfgfile=optarg;
		break;

	case 'S':
		if(mode)usage();
		mode=1;
		break;

	case 'T':
		if(mode)usage();
		mode=2;
		break;

	case 'C':
		if(mode)usage();
		mode=3;
		break;

	case 's':
		if(src)usage();
		if((src=atoi(optarg))<1||src>255)usage();
		break;

	case 't':
		if(tnum)usage();
		for(tnum=0,p=strtok(optarg,",");p;tnum++,p=strtok(NULL,","))
		{
			if(!*p||tnum==MAX_TUNERS)usage();
			if((tuners[tnum]=atoi(p))<1||tuners[tnum]>65535)
				usage();
		}
		if(!tnum)usage();
		break;

	case 'P':
		if(para)usage();
		if((para=atoi(optarg))<1||para>MAX_TUNERS)usage();
		break;

	case 'L':
		if(net)usage();
		net=1;
		break;

	case 'R':
		if(net)usage();
		net=2;
		break;

	case 'H':
		if(net)usage();
		net=3;
		break;

	case 'h':
		if(host||uuid)usage();
		if(!*(host=optarg))usage();
		break;

	case 'p':
		if(port)usage();
		if((port=atoi(optarg))<1||port>65535)usage();
		break;

	case 'i':
		if(dev)usage();
		if(!*(dev=optarg))usage();
		break;

	case 'l':
		if(level)usage();
		if((level=atoi(optarg))<0||level>2)usage();
		level++;
		break;

	case 'O':
		if(lmode)usage();
		lmode=1;
		break;

	case 'F':
		if(lmode)usage();
		lmode=2;
		break;

	case 'o':
		if(locale)usage();
		if(!*(locale=optarg))usage();
		break;

	case 'm':
		if(mmode)usage();
		mmode=1;
		break;

	case 'M':
		if(mmode)usage();
		mmode=2;
		break;

	case 'x':
		if(flags&SATIP_ADDSID)usage();
		flags|=SATIP_ADDSID;
		break;

	case 'v':
		if(tv)usage();
		tv=1;
		break;

	case 'r':
		if(radio)usage();
		radio=1;
		break;

	case 'f':
		if(fta)usage();
		fta=1;
		break;

	case 'n':
		if(nonfta)usage();
		nonfta=1;
		break;

	case 'N':
		if(num)usage();
		if((num=atoi(optarg))<1)usage();
		break;

	case 'e':
		if(eit)usage();
		eit=1;
		break;

	case 'd':
		if(flags&SATIP_ADDPMT)usage();
		flags|=SATIP_ADDPMT;
		break;

	case 'D':
		if(flags&SATIP_ADDXPMT)usage();
		flags|=SATIP_ADDXPMT;
		break;

	case 'b':
		if(binfile)usage();
		binfile=optarg;
		break;

	case 'u':
		if(uuid||host)usage();
		uuid=optarg;
		break;

	case 'X':
		if(mttl)usage();
		if((mttl=atoi(optarg))<1||mttl>255)usage();
		break;

	case 'I':
		if(flags&SATIP_STRICTQRY)usage();
		flags|=SATIP_STRICTQRY;
		break;

	default:usage();
	}

	if(!cfgfile)cfgfile="/etc/satipscan.conf";
	if(!locale)locale="de_DE.ISO-8859-15";
	if(!dev)dev="eth0";
	if(!para)para=1;
	if(!num)num=1;
	if(net)net--;
	if(level)level--;

	if(!port)switch(net)
	{
	case 1:	port=SATIP_RTSP_PORT;
		break;
	case 2:	port=SATIP_HTTP_PORT;
		break;
	}

	if(!mode||(mode==1&&!src))usage();
	if(tnum&&tnum!=para)usage();
	if(net)
	{
		if(!host&&!uuid)usage();
	}
	else if(mmode&&(!host||!port))usage();
	if(mmode&&!tv&&!radio)usage();
	if(mmode&&!fta&&!nonfta)usage();
	if(lmode==1&&mode!=1)usage();

	sigfillset(&set);
	if((sfd=signalfd(-1,&set,SFD_NONBLOCK|SFD_CLOEXEC))==-1)
	{
		perror("signalfd");
		goto err1;
	}
	sigprocmask(SIG_BLOCK,&set,NULL);

	if(parse_config(cfgfile,&conf,sfd))goto err2;

	memset(&state,0,sizeof(state));
	pthread_mutex_init(&state.mtx,NULL);
	state.tty=isatty(fileno(stderr));

	memset(&prg,0,sizeof(prg));
	prg.tv=tv;
	prg.radio=radio;
	prg.fta=fta;
	prg.nonfta=nonfta;
	prg.num=num;
	prg.eit=eit;

	switch(mode)
	{
	case 1:	state.mode=1;

		for(l=0,r=0;r<conf.dvbsnum;r++)
			if(conf.dvbs[r].src==src)tune[l++]=conf.dvbs[r];
		break;

	case 2:	state.mode=0;
		for(l=0;l<conf.tnum;l++)tune[l]=conf.dvbt[l];
		break;

	case 3:	state.mode=0;
		for(l=0;l<conf.dvbcnum;l++)tune[l]=conf.dvbc[l];
		break;
	}

	if(!l)
	{
		fprintf(stderr,"no transponder(s) defined\n");
		goto err2;
	}

	if(net)
	{
		if(net>1)scanmode=SATIP_SCAN_HTTP;
		else scanmode=SATIP_SCAN_RTSP;

		memset(&clnconf,0,sizeof(clnconf));
		strncpy(clnconf.dev,dev,sizeof(clnconf.dev)-1);
		clnconf.level=level;
		clnconf.mttl=mttl?mttl:SATIP_MCST_TTL;
		clnconf.portmin=conf.portmin;
		clnconf.portmax=conf.portmax;
		clnconf.interval=uuid?1:SATIP_UPNP_OFF;
		clnconf.strict=(flags&SATIP_STRICTQRY)?1:0;

		if(!(h=satip_cln_init(&clnconf)))goto err2;

		if(uuid)for(i=0;i<80;i++)
		{
			usleep(25000);
			if(!satip_cln_upnplist(h,&ulst))
			{
				for(e=ulst;e;e=e->next)if(!strcmp(e->uuid,uuid))
				{
					for(f=e,e=e->same;e;e=e->same)
						if(e->level>f->level)f=e;
					host=f->addr;
					break;
				}
				if(host)break;
				satip_cln_freelist(h,ulst);
				ulst=NULL;
			}
		}

		if(!host)
		{
			fprintf(stderr,"device with uuid %s not found\n",uuid);
			goto err3;
		}
	}
	else
	{
		scanmode=SATIP_SCAN_HW;

		if(!conf.tnum)
		{
			fprintf(stderr,"no tuners defined\n");
			goto err2;
		}

		if(!(h=satip_hw_init(&conf.global,&hwinfo)))
		{
			fprintf(stderr,"satip_hw_init failure\n");
			goto err2;
		}

		for(r=0;r<conf.tnum;r++)if(satip_hw_add(h,&conf.tuner[r],
			&hwinfo))
		{
			fprintf(stderr,"satip_hw_add failure\n");
			goto err3;
		}
	}

	if(setpriv(1))goto err3;

	switch(mode)
	{
	case 1:	r=satip_scan_dvbs(h,tune,l,&conf.cfg,para,
			SATIP_SCAN_STD|scanmode,tnum?tuners:NULL,scanstate,
			&state,&lst,&total,net?host:NULL,net?port:0);
		break;

	case 2:	r=satip_scan_dvbt(h,conf.dvbt,conf.dvbtnum,&conf.cfg,para,
			scanmode,tnum?tuners:NULL,scanstate,
			&state,&lst,&total,net?host:NULL,net?port:0);
		break;

	case 3:	r=satip_scan_dvbc(h,conf.dvbc,conf.dvbcnum,&conf.cfg,para,
			scanmode,NULL,0,tnum?tuners:NULL,scanstate,
			&state,&lst,&total,net?host:NULL,net?port:0);
		break;
	}

	if(r)
	{
		fprintf(stderr,"scan failure %d\n",r);
		goto err3;
	}

	if(satip_util_program_list(lst,total,locale,0,&prg.list,&prg.size))
	{
		fprintf(stderr,"satip_util_program_list failure\n");
		goto err4;
	}

	if(binfile)
	{
		if((bfd=open(binfile,O_WRONLY|O_TRUNC|O_CREAT|O_CLOEXEC,0666))
			==-1)
		{
			fprintf(stderr,"can't create %s\n",binfile);
			goto err5;
		}
		for(l=0;l<prg.size;l++)
		{
			iov[0].iov_base=&r;
			iov[0].iov_len=sizeof(r);
			iov[1].iov_base=prg.list[l];
			iov[1].iov_len=sizeof(prg.list[0][0])+
				prg.list[l]->streams*
				sizeof(prg.list[0]->stream[0]);
			iov[2].iov_base=prg.list[l]->progname;
			iov[2].iov_len=strlen(prg.list[l]->progname)+1;
			iov[3].iov_base=prg.list[l]->provider;
			iov[3].iov_len=strlen(prg.list[l]->provider)+1;
			r=iov[1].iov_len+iov[2].iov_len+iov[3].iov_len;
			if(writev(bfd,iov,4)!=r+iov[0].iov_len)
			{
				fprintf(stderr,"write error\n");
				close(bfd);
				goto err5;

			}
		}
		close(bfd);
	}

	if(mmode)if((r=satip_util_create_unicast_m3u(host,port,
		mmode==1?SATIP_TYPE_RTSP:SATIP_TYPE_HTTP,flags,
		0,(p=strchr(locale,'.'))?p+1:"ISO-8859-15",next,&prg,
		&m3umem,&m3usize)))
	{
		fprintf(stderr,"satip_util_create_unicast_m3u failure %d\n",r);
		goto err5;
			
	}

	res=0;

	switch(lmode)
	{
	case 1:	printf("# Optimized initial transponder list:\n");
		for(l=0;l<total;l++)if(lst[l].fastfind)
			if(satip_util_create(SATIP_TYPE_QRY,flags,NULL,0,0,
				&lst[l].actual,NULL,NULL,NULL,NULL,bfr,
				sizeof(bfr))>0)printf("%s\n",bfr);
		break;

	case 2:	printf("# Full transponder list:\n");
		for(l=0;l<total;l++)if(satip_util_create(SATIP_TYPE_QRY,flags,
			NULL,0,0,&lst[l].actual,NULL,NULL,NULL,NULL,bfr,
			sizeof(bfr))>0)printf("%s\n",bfr);
		break;
	}

	if(m3umem)
	{
		fwrite(m3umem,1,m3usize,stdout);
		free(m3umem);
	}

err5:	satip_util_program_list_free(prg.list,prg.size);
err4:	satip_scan_free(h,lst,total);
err3:	if(net)
	{
		if(ulst)satip_cln_freelist(h,ulst);
		satip_cln_fini(h);
	}
	else satip_hw_fini(h);
err2:	close(sfd);
err1:	return res;
}
