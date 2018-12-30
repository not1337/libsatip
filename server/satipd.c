/*
 * example SAT>IP server daemon
 *
 * Copyright (c) 2016 Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 */

#include <pthread.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/signalfd.h>
#include <sys/eventfd.h>
#include <sys/prctl.h>
#include <sys/mman.h>
#include <sys/capability.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/uio.h>
#include <dlfcn.h>
#include <grp.h>
#include <pwd.h>
#include <limits.h>
#include <stddef.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <poll.h>
#include "satip.h"
#include "ctlsocket.h"
#include "icons.h"

#define MAX_TUNERS 32
#define MAX_SERVERS 8
#define MAX_MSGS (MAX_TUNERS+2*MAX_SERVERS)
#define PLUGGER_MAXQUEUE 800
#define STACKSIZE 131072

#ifndef MCL_ONFAULT
#define MCL_ONFAULT 0
#endif

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

#define CLRCAP(a,cap) \
do \
{ \
	int idx=(cap>31?1:0); \
	int bit=(cap>31?(1<<(cap-32)):(1<<cap)); \
	a[idx].effective&=~bit; \
	a[idx].permitted&=~bit; \
	a[idx].inheritable&=~bit; \
} while(0)

typedef struct
{
	int type;
	union
	{
		SATIP_HW_CAM_IO msg;
		struct
		{
			int hint;
			int idx;
		};
	};
	unsigned char data[0];
} MSG;

typedef struct
{
	void *hw;
	void *list;
	void *hlist;
	void *rlist;
	void *mlist;
	void *plugin;
	void *plugctx;
	void (*pluginit)(void *hw,void **glob,char *config);
	void (*plugfini)(void *hw,void *glob);
	void (*plugstream)(void *hw,void *glob,void *ctx,SATIP_STREAM *stream);
	void (*plugmulti)(void *hw,void *glob,void *ctx,SATIP_STREAM **stream,
		int total,int totlen,int *done,int flush);
	void (*plugprep)(void *hw,void *glob,void **ctx,SATIP_TUNE *tune,
		SATIP_PIDS *set);
	void (*plugplay)(void *hw,void *glob,void *ctx,SATIP_TUNE *tune,
		SATIP_PIDS *set,int success);
	void (*plugend)(void *hw,void *glob,void *ctx);
	void (*plugpids)(void *hw,void *glob,void *ctx,SATIP_PIDS *set);
	void (*plugcam)(void *hw,void *glob,SATIP_HW_CAM_IO *msg);
	char *m3umem;
	int m3usize;
	int remap;
	int qlen;
	int qlim;
	int head;
	int tail;
	int fill;
	int efd;
	int verbose;
	int nostats;
	int proxyport;
	MSG *queue[MAX_MSGS];
	char datadir[PATH_MAX];
	char proxyhost[128];
	pthread_mutex_t mtx;
	pthread_rwlock_t plugmtx;
} COMMON;

typedef struct
{
	int idx;
	void *h;
	struct _config *conf;
	char addr[3][64];
	int ports[2];
} PRIV;

typedef struct
{
	int port;
	char addr[128];
	SATIP_TUNE tune;
	SATIP_PIDS set;
} MCAST;

typedef struct _config
{
	int tnum;
	int snum;
	int onum;
	int pnum;
	int restart;
	int running;
	MCAST *ondemand;
	MCAST *stream;
	SATIP_HW_CONFIG global;
	int disable[MAX_TUNERS];
	SATIP_HW_TUNERCFG tuner[MAX_TUNERS];
	SATIP_SRV_CONFIG conf[MAX_SERVERS];
	PRIV priv[MAX_SERVERS];
	COMMON cmn;
	char plugin[PATH_MAX];
	char plugcfg[PATH_MAX];
	char control[PATH_MAX];
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
	PRIV *priv;
	void *srvhandle;
	void *strhandle;
	void *hwhandle;
	void (*stream)(void *id,SATIP_STREAM *stream);
	void (*status)(void *id,SATIP_STATUS *status);
	void *plugctx;
	int head;
	int tail;
	int fill;
	int bypass:1;
	SATIP_STREAM queue[0];
} PLUGGER;

#ifndef __GNUC__
#define __attribute__(x)
#endif

#ifndef PROFILE

static void plugger_stream(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void plugger_multi(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void plugger_status(void *id,SATIP_STATUS *status) __attribute__ ((hot));

static int droppriv(void) __attribute__ ((cold));
static int setpriv(char *user,char *group) __attribute__ ((cold));
static int dodev(void *s,char *val) __attribute__ ((cold));
static int doull(void *s,char *val) __attribute__ ((cold));
static int doinv(void *s,char *val) __attribute__ ((cold));
static int dosrc(void *s,char *val) __attribute__ ((cold));
static int doparam(CFGFILE *p,void *s,char *name,char *val)
	__attribute__ ((cold));
static int parse_config(char *fn,CONFIG *c,void *scb,void *ccb,int verbose)
	__attribute__ ((cold));

#else

static int dodev(void *s,char *val);
static int doull(void *s,char *val);
static int doinv(void *s,char *val);
static int dosrc(void *s,char *val);

#endif

static CFGFILE sparams[]=
{
	{"dev",0,0,offsetof(SATIP_SRV_CONFIG,dev),dodev},
	{"level",0,2,offsetof(SATIP_SRV_CONFIG,level),NULL},
	{"rtspport",1,65535,offsetof(SATIP_SRV_CONFIG,rtspport),NULL},
	{"httpport",1,65535,offsetof(SATIP_SRV_CONFIG,httpport),NULL},
	{"mdftport",1,65535,offsetof(SATIP_SRV_CONFIG,mdftport),NULL},
	{"igmpv3",0,3,offsetof(SATIP_SRV_CONFIG,igmpv3),NULL},
	{"mldv2",0,3,offsetof(SATIP_SRV_CONFIG,mldv2),NULL},
	{"upnpage",60,86400,offsetof(SATIP_SRV_CONFIG,upnpage),NULL},
	{"mttl",1,255,offsetof(SATIP_SRV_CONFIG,mttl),NULL},
	{"rtsplimit",0,65535,offsetof(SATIP_SRV_CONFIG,rtsplimit),NULL},
	{"httplimit",0,65535,offsetof(SATIP_SRV_CONFIG,httplimit),NULL},
	{"httpnostream",0,65535,offsetof(SATIP_SRV_CONFIG,httpnostream),NULL},
	{"bytespersec",0,0,offsetof(SATIP_SRV_CONFIG,bytespersec),doull},
	{"strict",0,1,offsetof(SATIP_SRV_CONFIG,strict),NULL},
	{"timeout",10,180,offsetof(SATIP_SRV_CONFIG,timeout),NULL},
	{"burst",1,4,offsetof(SATIP_SRV_CONFIG,burst),NULL},
	{"portmin",1024,65534,offsetof(SATIP_SRV_CONFIG,portmin),NULL},
	{"portmax",1025,65535,offsetof(SATIP_SRV_CONFIG,portmax),NULL},
	{NULL,0,0,0L,NULL}
};

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
	{"prefer",0,255,offsetof(SATIP_HW_TUNERCFG,prefer),NULL},
	{"explicit",0,1,offsetof(SATIP_HW_TUNERCFG,explicit),NULL},
	{"noshare",0,1,offsetof(SATIP_HW_TUNERCFG,noshare),NULL},
	{"fast",0,1,offsetof(SATIP_HW_TUNERCFG,fast),NULL},
	{"source",0,0,offsetof(SATIP_HW_TUNERCFG,src),dosrc},
	{NULL,0,0,0L,NULL}
};

static int droppriv(void)
{
	struct __user_cap_header_struct hdr;
	struct __user_cap_data_struct cap[2];

	memset(&hdr,0,sizeof(hdr));
	hdr.version=_LINUX_CAPABILITY_VERSION_3;

	if(capget(&hdr,cap))
	{
		perror("capget");
		return -1;
	}

	CLRCAP(cap,CAP_SYS_NICE);
	CLRCAP(cap,CAP_IPC_LOCK);

	if(capset(&hdr,cap))
	{
		perror("capset");
		return -1;
	}

	if(prctl(PR_SET_DUMPABLE,1,0,0,0))
	{
		perror("prctl(PR_SET_DUMPABLE)");
		return -1;
	}

	return 0;
}

static int setpriv(char *user,char *group)
{
	int cnbs=1;
	int csn=1;
	int csr=1;
	int f=0;
	struct group *g;
	struct passwd *u;
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

	if(capget(&hdr,cap))
	{
		perror("capget");
		return -1;
	}

	if(user&&!TESTCAP(cap,CAP_SETUID))
	{
		fprintf(stderr,"need CAP_SETUID\n");
		f=1;
	}

	if(group&&!TESTCAP(cap,CAP_SETGID))
	{
		fprintf(stderr,"need CAP_SETGID\n");
		f=1;
	}

	if(!TESTCAP(cap,CAP_IPC_LOCK))
	{
		fprintf(stderr,"need CAP_IPC_LOCK\n");
		f=1;
	}

	if(!TESTCAP(cap,CAP_NET_ADMIN))
	{
		fprintf(stderr,"need CAP_NET_ADMIN\n");
		f=1;
	}

	if(!TESTCAP(cap,CAP_NET_RAW))
	{
		fprintf(stderr,"need CAP_NET_RAW\n");
		f=1;
	}

	if(!TESTCAP(cap,CAP_NET_BIND_SERVICE))
	{
		cnbs=0;
		fprintf(stderr,"warning: may need CAP_NET_BIND_SERVICE\n");
	}

	if(!TESTCAP(cap,CAP_SYS_NICE))
	{
		csn=0;
		fprintf(stderr,"warning: may need CAP_SYS_NICE\n");
	}

	if(!TESTCAP(cap,CAP_SYS_RESOURCE))
	{
		csr=0;
		fprintf(stderr,"warning: may need CAP_SYS_RESOURCE\n");
	}

	if(f)
	{
		fprintf(stderr,"\nYou can either run:\n");
		fprintf(stderr,"- satipd as root\n");
		fprintf(stderr,
			"- 'chown root satipd' and 'chmod u+s satipd'\n");
		fprintf(stderr,"- 'capset <cap>[,<cap>,...]=ep satipd'\n\n");
		return -1;
	}

	if(csr)
	{
		r.rlim_cur=RLIM_INFINITY;
		r.rlim_max=RLIM_INFINITY;

		if(setrlimit(RLIMIT_CPU,&r))
		{
			perror("setrlimit(RLIMIT_CPU)");
			return -1;
		}

		if(setrlimit(RLIMIT_AS,&r))
		{
			perror("setrlimit(RLIMIT_AS)");
			return -1;
		}

		if(setrlimit(RLIMIT_DATA,&r))
		{
			perror("setrlimit(RLIMIT_DATA)");
			return -1;
		}

		if(setrlimit(RLIMIT_RSS,&r))
		{
			perror("setrlimit(RLIMIT_RSS)");
			return -1;
		}

		if(setrlimit(RLIMIT_STACK,&r))
		{
			perror("setrlimit(RLIMIT_STACK)");
			return -1;
		}

		if(setrlimit(RLIMIT_MEMLOCK,&r))
		{
			perror("setrlimit(RLIMIT_MEMLOCK)");
			return -1;
		}

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

		if(setrlimit(RLIMIT_NPROC,&r))
		{
			perror("setrlimit(RLIMIT_NPROC)");
			return -1;
		}

		r.rlim_cur=1000000;
		r.rlim_max=1000000;

		if(setrlimit(RLIMIT_NOFILE,&r))
		{
			perror("setrlimit(RLIMIT_NOFILE)");
			return -1;
		}
	}

	if(mlockall(MCL_CURRENT|MCL_FUTURE|MCL_ONFAULT))
	{
		perror("mlockall");
		return -1;
	}

	if(prctl(PR_SET_KEEPCAPS,1,0,0,0))
	{
		perror("prctl(PR_SET_KEEPCAPS)");
		return -1;
	}

	if(group)
	{
		if(!(g=getgrnam(group)))
		{
			perror("getgrnam");
			return -1;
		}

		if(setresgid(g->gr_gid,g->gr_gid,g->gr_gid))
		{
			perror("setresgid");
			return -1;
		}
	}

	if(user)
	{
		if(!(u=getpwnam(user)))
		{
			perror("getpwnam");
			return -1;
		}

		if(!u->pw_uid||!u->pw_gid)
		{
			fprintf(stderr,"will not run as root\n");
			return -1;
		}

		if(initgroups(user,u->pw_gid))
		{
			perror("initgroups");
			return -1;
		}

		if(setresuid(u->pw_uid,u->pw_uid,u->pw_uid))
		{
			perror("setresuid");
			return -1;
		}
	}

	if(prctl(PR_SET_DUMPABLE,1,0,0,0))
	{
		perror("prctl(PR_SET_DUMPABLE)");
		return -1;
	}

	if(prctl(PR_SET_KEEPCAPS,0,0,0,0))
	{
		perror("prctl(PR_SET_KEEPCAPS)");
		return -1;
	}

	memset(cap,0,sizeof(cap));

	SETCAP(cap,CAP_NET_ADMIN);
	SETCAP(cap,CAP_NET_RAW);
	SETCAP(cap,CAP_IPC_LOCK);
	if(cnbs)SETCAP(cap,CAP_NET_BIND_SERVICE);
	if(csn)SETCAP(cap,CAP_SYS_NICE);

	if(capset(&hdr,cap))
	{
		perror("capset");
		return -1;
	}

	return 0;
}

static int dodev(void *s,char *val)
{
	SATIP_SRV_CONFIG *dummy __attribute__ ((unused));

	if(!*val)return -1;
	strncpy(s,val,sizeof(dummy->dev)-1);
	return 0;
}

static int doull(void *s,char *val)
{
	*((unsigned long long *)s)=(unsigned long long)atoi(val);
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

static int parse_config(char *fn,CONFIG *c,void *scb,void *ccb,int verbose)
{
	int i;
	int j;
	int k;
	int l;
	int fd;
	int line=0;
	int glob=0;
	int ondemand=0;
	int play=0;
	int type;
	int port;
	int stream;
	char *name;
	char *val;
	char *mem;
	FILE *fp;
	SATIP_SRV_CONFIG *server=NULL;
	SATIP_HW_TUNERCFG *tuner=NULL;
	SATIP_HW_TUNERCFG *cmp;
	MCAST *m;
	SATIP_TUNE tune;
	SATIP_PIDS set;
	SATIP_PIDS add;
	SATIP_PIDS del;
	struct stat stb;
	char bfr[8192];
	char addr[128];

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
			if(ondemand==1)ondemand=-1;
			if(play==1)play=-1;
			tuner=NULL;
			server=NULL;
			continue;
		}

		if(!strcmp(name,"[ondemand]"))
		{
			if(ondemand)goto err3;
			ondemand=1;
			glob=-1;
			if(play==1)play=-1;
			tuner=NULL;
			server=NULL;
			continue;
		}

		if(!strcmp(name,"[stream]"))
		{
			if(play)goto err3;
			play=1;
			glob=-1;
			if(ondemand==1)ondemand=-1;
			tuner=NULL;
			server=NULL;
			continue;
		}

		if(!strcmp(name,"[tuner]"))
		{
			glob=-1;
			if(ondemand==1)ondemand=-1;
			if(play==1)play=-1;
			server=NULL;
			if(c->tnum==MAX_TUNERS)
			{
				fprintf(stderr,"Out of memory\n");
				goto err2;
			}
			tuner=&c->tuner[c->tnum++];
			tuner->inversion=SATIP_SPI_OFF;
			tuner->lna=SATIP_LNA_AUTO;
			tuner->idleflush=1;
			continue;
		}

		if(!strcmp(name,"[server]"))
		{
			glob=-1;
			if(ondemand==1)ondemand=-1;
			if(play==1)play=-1;
			tuner=NULL;
			if(c->snum==MAX_SERVERS)
			{
				fprintf(stderr,"Out of memory\n");
				goto err2;
			}
			server=&c->conf[c->snum++];
			server->stack=STACKSIZE;
			continue;
		}

		if(!strncmp(name,"rtp://",6))
		{
			val=name;
			name="tune";
			val=strtok_r(val," \t",&mem);
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
			if(!strcmp(name,"streamlimit"))
			{
				c->global.streamlimit=atoi(val);
				if(c->global.streamlimit<0||
					c->global.streamlimit>65535)goto err3;
			}
			else if(!strcmp(name,"rtprio"))
			{
				c->global.rtprio=atoi(val);
				if(c->global.rtprio<0||c->global.rtprio>99)
					goto err3;
			}
			else if(!strcmp(name,"remap"))
			{
				c->cmn.remap=atoi(val);
				if(c->cmn.remap<0||c->cmn.remap>1)goto err3;
			}
			else if(!strcmp(name,"pktqueue"))
			{
				c->cmn.qlen=atoi(val);
				if(c->cmn.qlen<50||c->cmn.qlen>PLUGGER_MAXQUEUE)
					goto err3;
			}
			else if(!strcmp(name,"datadir"))
			{
				if(!*val)goto err3;
				strncpy(c->cmn.datadir,val,
					sizeof(c->cmn.datadir)-7);
			}
			else if(!strcmp(name,"proxyhost"))
			{
				if(!*val)goto err3;
				strncpy(c->cmn.proxyhost,val,
					sizeof(c->cmn.proxyhost)-1);
			}
			else if(!strcmp(name,"proxyport"))
			{
				c->cmn.proxyport=atoi(val);
				if(c->cmn.proxyport<1||c->cmn.proxyport>65535)
					goto err3;
			}
			else if(!strcmp(name,"nostats"))
			{
				c->cmn.nostats=atoi(val);
				if(c->cmn.nostats<0||c->cmn.nostats>1)goto err3;
			}
			else if(!strcmp(name,"m3u"))
			{
				if(c->cmn.m3umem)goto err3;
				if(stat(val,&stb)||!S_ISREG(stb.st_mode)||
					!stb.st_size)goto err3;
				if(!(c->cmn.m3umem=malloc(stb.st_size)))
				{
					fprintf(stderr,"Out of memory\n");
					goto err2;
				}
				c->cmn.m3usize=stb.st_size;
				if((fd=open(val,O_RDONLY|O_CLOEXEC))==-1)
				{
					free(c->cmn.m3umem);
					c->cmn.m3umem=NULL;
					goto err3;
				}
				if(read(fd,c->cmn.m3umem,stb.st_size)!=
					stb.st_size)
				{
					close(fd);
					free(c->cmn.m3umem);
					c->cmn.m3umem=NULL;
					goto err3;
				}
				close(fd);
			}
			else if(!strcmp(name,"allow"))
			{
				satip_util_list_init(&c->cmn.list);
				for(i=0,val=strtok_r(val,",",&mem);val;i++,
					val=strtok_r(NULL,",",&mem))
				{
					if(!val)goto err3;
					if(satip_util_list_add(c->cmn.list,
						val))goto err3;
				}
				if(!i)goto err3;
			}
			else if(!strcmp(name,"httpallow"))
			{
				satip_util_list_init(&c->cmn.hlist);
				for(i=0,val=strtok_r(val,",",&mem);val;i++,
					val=strtok_r(NULL,",",&mem))
				{
					if(!val)goto err3;
					if(satip_util_list_add(c->cmn.hlist,
						val))goto err3;
				}
				if(!i)goto err3;
			}
			else if(!strcmp(name,"rtspallow"))
			{
				satip_util_list_init(&c->cmn.rlist);
				for(i=0,val=strtok_r(val,",",&mem);val;i++,
					val=strtok_r(NULL,",",&mem))
				{
					if(!val)goto err3;
					if(satip_util_list_add(c->cmn.rlist,
						val))goto err3;
				}
				if(!i)goto err3;
			}
			else if(!strcmp(name,"mcstallow"))
			{
				satip_util_list_init(&c->cmn.mlist);
				for(i=0,val=strtok_r(val,",",&mem);val;i++,
					val=strtok_r(NULL,",",&mem))
				{
					if(!val)goto err3;
					if(satip_util_list_add(c->cmn.mlist,
						val))goto err3;
				}
				if(!i)goto err3;
			}
			else if(!strcmp(name,"plugin"))
			{
				if(!*val||strlen(val)>=sizeof(c->plugin))
					goto err3;
				strcpy(c->plugin,val);
			}
			else if(!strcmp(name,"plugcfg"))
			{
				if(!*val||strlen(val)>=sizeof(c->plugcfg))
					goto err3;
				strcpy(c->plugcfg,val);
			}
			else if(!strcmp(name,"control"))
			{
				if(!*val||strlen(val)>=sizeof(c->control))
					goto err3;
				strcpy(c->control,val);
			}
			else goto err3;
		}
		else if(ondemand==1)
		{
			if(strcmp(name,"tune"))goto err3;
			if(satip_util_parse(SATIP_PARSE_URL,0,
				SATIP_IGNCAPS|SATIP_RTPQUERY,val,
				&type,addr,sizeof(addr),&port,
				&stream,&tune,&set,&add,&del,NULL)||
				add.numpids||del.numpids||set.numpids<1||
				stream!=SATIP_UNDEF||type!=SATIP_TYPE_RTP)
					goto err3;
			if(!(m=realloc(c->ondemand,(c->onum+1)*sizeof(MCAST))))
			{
				fprintf(stderr,"out of memory\n");
				goto err2;
			}
			c->ondemand=m;
			m->port=port;
			strcpy(m->addr,addr);
			m->tune=tune;
			m->set=set;
			c->onum++;
		}
		else if(play==1)
		{
			if(strcmp(name,"tune"))goto err3;
			satip_util_init_tune(&tune);
			satip_util_init_pids(&set);
			satip_util_init_pids(&add);
			satip_util_init_pids(&del);
			if(satip_util_parse(SATIP_PARSE_URL,0,
				SATIP_IGNCAPS|SATIP_RTPQUERY,val,
				&type,addr,sizeof(addr),&port,
				&stream,&tune,&set,&add,&del,NULL)||
				add.numpids||del.numpids||set.numpids<1||
				stream!=SATIP_UNDEF||type!=SATIP_TYPE_RTP)
					goto err3;
			if(!(m=realloc(c->stream,(c->pnum+1)*sizeof(MCAST))))
			{
				fprintf(stderr,"out of memory\n");
				goto err2;
			}
			c->stream=m;
			m->port=port;
			strcpy(m->addr,addr);
			m->tune=tune;
			m->set=set;
			c->pnum++;
		}
		else if(server)
		{
			if(doparam(sparams,server,name,val))goto err3;
		}
		else if(tuner)
		{
			if(!strcmp(name,"disable"))
			{
				if(!strcmp(val,"0"))c->disable[c->tnum-1]=0;
				else if(!strcmp(val,"1"))
					c->disable[c->tnum-1]=1;
				else goto err3;
			}
			else if(doparam(tparams,tuner,name,val))goto err3;
		}
		else goto err3;
	}

	if(!c->snum)
	{
		fprintf(stderr,"No servers specified\n");
		goto err2;
	}

	for(k=0;k<c->snum;k++)
	{
		if(c->cmn.m3umem)c->conf[k].havem3u=1;

		c->conf[k].png48depth=32;
		c->conf[k].png120depth=32;
		c->conf[k].jpg48depth=24;
		c->conf[k].jpg120depth=24;

		strcpy(c->conf[k].friendlyname,"open source SAT>IP server");
		strcpy(c->conf[k].manufacturerurl,"http://senseless.info/");
		strcpy(c->conf[k].modelname,"satipd");
		strcpy(c->conf[k].modelnumber,"v1.0");

		c->conf[k].callback=scb;
		c->conf[k].priv=&c->priv[k];

		c->conf[k].locked=1;

		c->priv[k].idx=k;
		c->priv[k].conf=c;
	}

	if(!c->tnum)
	{
		fprintf(stderr,"No tuners specified\n");
		goto err2;
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

	c->global.camfunc=ccb;
	c->global.priv=&c->cmn;
	c->global.stack=STACKSIZE;

	c->cmn.verbose=verbose;

	if(!c->cmn.qlen)c->cmn.qlen=49;
	c->cmn.qlim=(c->cmn.qlen>>1)+(c->cmn.qlen>>2);

	posix_fadvise(fileno(fp),0,0,POSIX_FADV_DONTNEED);
	fclose(fp);
	return 0;

err4:	fprintf(stderr,"Syntax error in tuner definition\n");
	goto err2;
err3:	fprintf(stderr,"Syntax error in line %d of %s\n",line,fn);
err2:	fclose(fp);
err1:	return -1;
}

static void plugger_stream(void *id,SATIP_STREAM *stream)
{
	PLUGGER *p=(PLUGGER *)id;
	COMMON *cmn=&p->priv->conf->cmn;

	if(stream&&cmn->plugstream)
		cmn->plugstream(p->hwhandle,cmn->plugctx,p->plugctx,stream);

	p->stream(p->srvhandle,stream);
}

static void plugger_multi(void *id,SATIP_STREAM *stream)
{
	int i;
	int j;
	int done;
	int len;
	SATIP_STREAM *list[PLUGGER_MAXQUEUE];
	PLUGGER *p=(PLUGGER *)id;
	COMMON *cmn=&p->priv->conf->cmn;

	if(!stream||p->fill>=cmn->qlim)
	{
		if(!p->fill)return;

		for(i=0,len=0,j=p->tail,done=0;i<p->fill;i++)
		{
			list[i]=&p->queue[j++];
			len+=list[i]->fill;
			if(j==cmn->qlen)j=0;
		}

		cmn->plugmulti(p->hwhandle,cmn->plugctx,p->plugctx,list,
			p->fill,len,&done,1);

		for(i=0,p->fill-=done;i<done;i++)
		{
			p->stream(p->srvhandle,&p->queue[p->tail++]);
			if(p->tail==cmn->qlen)p->tail=0;
		}

		if(!stream)return;
	}

	if(p->fill==cmn->qlen)
	{
		p->fill--;
		p->stream(p->srvhandle,&p->queue[p->tail++]);
		if(p->tail==cmn->qlen)p->tail=0;
	}

	p->queue[p->head++]=*stream;
	if(p->head==cmn->qlen)p->head=0;
	p->fill++;

	for(i=0,len=0,j=p->tail,done=0;i<p->fill;i++)
	{
		list[i]=&p->queue[j++];
		len+=list[i]->fill;
		if(j==cmn->qlen)j=0;
	}

	cmn->plugmulti(p->hwhandle,cmn->plugctx,p->plugctx,list,
		p->fill,len,&done,stream->fill==sizeof(stream->data)?0:1);

	for(i=0,p->fill-=done;i<done;i++)
	{
		p->stream(p->srvhandle,&p->queue[p->tail++]);
		if(p->tail==cmn->qlen)p->tail=0;
	}
}

static void plugger_status(void *id,SATIP_STATUS *status)
{
	PLUGGER *p=(PLUGGER *)id;

	p->status(p->srvhandle,status);
}

static int plugger_play(void *handle,SATIP_STRDATA *params,
	void (*stream)(void *id,SATIP_STREAM *stream),
	void (*status)(void *id,SATIP_STATUS *status),
	void *user,int access)
{
	int r=SATIP_SYSFAIL;
	PLUGGER *p;
	COMMON *cmn=&((PRIV *)user)->conf->cmn;
	int qlen=0;
	int bypass=(access==SATIP_HW_LOCAL?1:0);

	if(cmn->plugmulti&&!bypass&&!((PRIV *)user)->conf
		->conf[((PRIV *)user)->idx].strict)qlen=cmn->qlen;
	if(!(p=malloc(sizeof(PLUGGER)+qlen*sizeof(SATIP_STREAM))))goto err1;
	p->priv=(PRIV *)user;
	p->stream=stream;
	p->status=status;
	p->srvhandle=params->handle;
	p->hwhandle=handle;
	p->bypass=bypass;
	p->plugctx=NULL;
	p->head=0;
	p->tail=0;
	p->fill=0;

	if(!p->bypass)
	{
		params->handle=p;
		pthread_rwlock_rdlock(&cmn->plugmtx);
		if(cmn->plugprep)cmn->plugprep(p->hwhandle,cmn->plugctx,
			&p->plugctx,params->tune,params->set);
	}
	if((r=satip_hw_play(handle,params,
		p->bypass?stream:(qlen?plugger_multi:plugger_stream),
		p->bypass?status:plugger_status,user,access)))
	{
		if(!p->bypass&&cmn->plugplay)cmn->plugplay(p->hwhandle,
			cmn->plugctx,p->plugctx,params->tune,params->set,0);
		pthread_rwlock_unlock(&cmn->plugmtx);
		goto err2;
	}
	if(!p->bypass&&cmn->plugplay)cmn->plugplay(p->hwhandle,cmn->plugctx,
		p->plugctx,params->tune,params->set,1);
	pthread_rwlock_unlock(&cmn->plugmtx);
	p->strhandle=params->handle;
	params->handle=p;
	return 0;

err2:	params->handle=p->srvhandle;
	free(p);
err1:	return r;
}

static int plugger_end(void *handle,SATIP_STRDATA *params)
{
	int r;
	PLUGGER *p=(PLUGGER *)params->handle;
	COMMON *cmn=&p->priv->conf->cmn;
	SATIP_STRDATA data;

	data.handle=p->strhandle;
	pthread_rwlock_rdlock(&cmn->plugmtx);
	r=satip_hw_end(handle,&data);
	if(!p->bypass&&cmn->plugend)
		cmn->plugend(p->hwhandle,cmn->plugctx,p->plugctx);
	pthread_rwlock_unlock(&cmn->plugmtx);
	free(p);
	return r;
}

static int plugger_setpids(void *handle,SATIP_STRDATA *params)
{
	int r;
	PLUGGER *p=(PLUGGER *)params->handle;
	COMMON *cmn=&p->priv->conf->cmn;
	SATIP_STRDATA data;

	data.handle=p->strhandle;
	data.set=params->set;
	r=satip_hw_setpids(handle,&data);
	if(!p->bypass&&cmn->plugpids)cmn->plugpids(p->hwhandle,cmn->plugctx,
		p->plugctx,params->set);
	return r;
}

static int ctlsocket(char *fn)
{
	int s;
	mode_t mask;
	struct sockaddr_un a;
	struct stat stb;

	if(strlen(fn)>=sizeof(a.sun_path))goto err1;

	if((s=socket(PF_UNIX,SOCK_STREAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0))==-1)
		goto err1;

	memset(&a,0,sizeof(a));
	a.sun_family=AF_UNIX;
	strcpy(a.sun_path,fn);

	if(!lstat(fn,&stb))
	{
		if(!S_ISSOCK(stb.st_mode))goto err2;
		if(unlink(fn))goto err2;
	}

	mask=umask(0);
	
	if(bind(s,(struct sockaddr *)(&a),sizeof(a)))goto err3;

	if(listen(s,5))goto err4;

	umask(mask);

	return s;

err4:	unlink(fn);
err3:	umask(mask);
err2:	close(s);
err1:	return -1;
}

static int ctlread(int s,int *req,void *data,int *size)
{
	CTLMSG m;
	struct pollfd p;

	p.fd=s;
	p.events=POLLIN;
	if(poll(&p,1,100)<=0||!(p.revents&POLLIN))return -1;
	if(read(s,&m,sizeof(CTLMSG))!=sizeof(CTLMSG))return -1;

	if(m.datalen<0||m.datalen>CTL_MAX_REQUEST||m.datalen>*size)return -1;

	*req=m.reqans;
	*size=m.datalen;

	if(m.datalen)if(read(s,data,m.datalen)!=m.datalen)return -1;

	return 0;
}

static int ctlwrite(int s,int ans,void *data,int size)
{
	int l;
	CTLMSG m;
	struct iovec iov[2];
	struct pollfd p;

	if(size<0||size>CTL_MAX_ANSWER)return -1;

	m.reqans=ans;
	m.datalen=size;

	iov[0].iov_base=&m;
	iov[0].iov_len=sizeof(CTLMSG);
	iov[1].iov_base=data;
	iov[1].iov_len=size;

	if((l=writev(s,iov,size?2:1))<sizeof(CTLMSG))return -1;

	l-=sizeof(CTLMSG);
	size-=l;
	data+=l;

	p.fd=s;
	p.events=POLLOUT;

	while(size)
	{
		if(poll(&p,1,100)<=0||!(p.revents&POLLOUT))return -1;
		if((l=write(s,data,size))<=0)return -1;
		size-=l;
		data+=l;
	}

	return 0;
}

static void ctlhandler(CONFIG *c,int s)
{
	int i;
	int j;
	int val;
	int req;
	int ans;
	int mport;
	int size=CTL_MAX_REQUEST;
	CTLSVSTATS *st;
	SATIP_TUNE tune;
	SATIP_PIDS set;
	SATIP_PIDS add;
	SATIP_PIDS del;
	SATIP_SRV_STATS stats;
	SATIP_HW_STATUS hwstats;
	char maddr[64];
	union
	{
		void *ptr;
		CTLINT1 *i1;
		CTLINT2	*i2;
		CTLINT5	*i5;
		CTLMCST	*m;
		SATIP_SRV_INFO *e;
	}u;
	union
	{
		char bfr[CTL_MAX_REQUEST];
		CTLHWSTATS t[MAX_TUNERS];
		SATIP_SRV_INFO *list[MAX_SERVERS];
	}v;

	if(ctlread(s,&req,v.bfr,&size))return;

	u.ptr=v.bfr;

	switch(req)
	{
	case CTLRTSP:
		if(size!=sizeof(CTLINT1))break;
		ans=CTLOK;
		if(u.i1->val)val=1;
		else val=0;
		for(i=0;i<c->snum;i++)
			if(satip_srv_set_rtsp_lock(c->priv[i].h,val))ans=CTLERR;
		ctlwrite(s,ans,NULL,0);
		return;

	case CTLHTTP:
		if(size!=sizeof(CTLINT1))break;
		ans=CTLOK;
		if(u.i1->val)val=1;
		else val=0;
		for(i=0;i<c->snum;i++)
			if(satip_srv_set_http_lock(c->priv[i].h,val))ans=CTLERR;
		ctlwrite(s,ans,NULL,0);
		return;

	case CTLCLRALL:
		if(size)break;
		ans=CTLOK;
		for(i=0;i<c->snum;i++)
			if(satip_srv_clr_all(c->priv[i].h))ans=CTLERR;
		ctlwrite(s,ans,NULL,0);
		return;

	case CTLCLRRTSP:
		if(size)break;
		ans=CTLOK;
		for(i=0;i<c->snum;i++)
			if(satip_srv_clr_all(c->priv[i].h))ans=CTLERR;
		ctlwrite(s,ans,NULL,0);
		return;

	case CTLCLRHTTP:
		if(size)break;
		ans=CTLOK;
		for(i=0;i<c->snum;i++)
			if(satip_srv_clr_all(c->priv[i].h))ans=CTLERR;
		ctlwrite(s,ans,NULL,0);
		return;

	case CTLCLRSTREAM:
		if(size!=sizeof(CTLINT2))break;
		i=u.i2->val1;
		val=u.i2->val2;
		if(i<0||i>=c->snum)break;
		if(satip_srv_clr_stream(c->priv[i].h,val))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLCLRDEV:
		if(size!=sizeof(CTLINT1))break;
		val=u.i1->val;
		ans=CTLOK;
		for(i=0;i<c->snum;i++)
			if(satip_srv_clr_device(c->priv[i].h,val))ans=CTLERR;
		ctlwrite(s,ans,NULL,0);
		return;

	case CTLADDMCST:
		if(size<=sizeof(CTLMCST)||size>=CTL_MAX_REQUEST)break;
		i=u.m->srvid;
		if(i<0||i>=c->snum)break;
		if(!u.m->ttl)u.m->ttl=c->conf[i].mttl;
		if(u.m->ttl<1||u.m->ttl>255)break;

		v.bfr[size]=0;
		satip_util_init_tune(&tune);
		satip_util_init_pids(&set);
		satip_util_init_pids(&add);
		satip_util_init_pids(&del);

		if(satip_util_parse(SATIP_PARSE_URL,0,
			SATIP_IGNCAPS|SATIP_RTPQUERY,
			u.m->url,&val,maddr,sizeof(maddr),&mport,
			&j,&tune,&set,&add,&del,NULL)||add.numpids||
			del.numpids||set.numpids<1||j!=SATIP_UNDEF||
			val!=SATIP_TYPE_RTP||!mport||(mport&1))break;

		if((u.i1->val=satip_srv_add_multicast(c->priv[i].h,maddr,mport,
			u.m->ttl,u.m->play?1:0,&tune,&set))==-1)break;

		ctlwrite(s,CTLOK,u.ptr,sizeof(CTLINT1));
		return;

	case CTLDELMCST:
		if(size!=sizeof(CTLINT2))break;
		i=u.i2->val1;
		val=u.i2->val2;
		if(i<0||i>=c->snum)break;
		if(satip_srv_del_multicast(c->priv[i].h,val))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLINQ:
		if(size)break;
		u.i1->val=c->running;
		ctlwrite(s,CTLOK,u.ptr,sizeof(CTLINT1));
		return;

	case CTLADDLIST:
		if(!size||size>=CTL_MAX_REQUEST)break;
		v.bfr[size]=0;
		if(satip_util_list_total(c->cmn.list)==SATIP_SYSFAIL)break;
		if(satip_util_list_add(c->cmn.list,v.bfr))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLDELLIST:
		if(!size||size>=CTL_MAX_REQUEST)break;
		v.bfr[size]=0;
		if(satip_util_list_total(c->cmn.list)==SATIP_SYSFAIL)break;
		if(satip_util_list_del(c->cmn.list,v.bfr))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLADDRLIST:
		if(!size||size>=CTL_MAX_REQUEST)break;
		v.bfr[size]=0;
		if(satip_util_list_total(c->cmn.rlist)==SATIP_SYSFAIL)break;
		if(satip_util_list_add(c->cmn.rlist,v.bfr))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLDELRLIST:
		if(!size||size>=CTL_MAX_REQUEST)break;
		v.bfr[size]=0;
		if(satip_util_list_total(c->cmn.rlist)==SATIP_SYSFAIL)break;
		if(satip_util_list_del(c->cmn.rlist,v.bfr))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLADDHLIST:
		if(!size||size>=CTL_MAX_REQUEST)break;
		v.bfr[size]=0;
		if(satip_util_list_total(c->cmn.hlist)==SATIP_SYSFAIL)break;
		if(satip_util_list_add(c->cmn.hlist,v.bfr))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLDELHLIST:
		if(!size||size>=CTL_MAX_REQUEST)break;
		v.bfr[size]=0;
		if(satip_util_list_total(c->cmn.hlist)==SATIP_SYSFAIL)break;
		if(satip_util_list_del(c->cmn.hlist,v.bfr))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLADDMLIST:
		if(!size||size>=CTL_MAX_REQUEST)break;
		v.bfr[size]=0;
		if(satip_util_list_total(c->cmn.mlist)==SATIP_SYSFAIL)break;
		if(satip_util_list_add(c->cmn.mlist,v.bfr))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLDELMLIST:
		if(!size||size>=CTL_MAX_REQUEST)break;
		v.bfr[size]=0;
		if(satip_util_list_total(c->cmn.mlist)==SATIP_SYSFAIL)break;
		if(satip_util_list_del(c->cmn.mlist,v.bfr))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLRESTART:
		if(size)break;
		c->restart=1;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLTUNER:
		if(size!=sizeof(CTLINT2))break;
		i=u.i2->val1;
		val=u.i2->val2;
		for(j=0;j<c->tnum;j++)if(c->tuner[j].deviceid==i)break;
		if(j==c->tnum)break;
		if(satip_hw_access(c->cmn.hw,i,
			val?SATIP_HW_REMOTE:SATIP_HW_LOCAL))break;
		ctlwrite(s,CTLOK,NULL,0);
		return;

	case CTLSVRSTATS:
		if(size)break;
		memset(u.ptr,0,sizeof(CTLINT5));
		ans=CTLOK;
		for(i=0;i<c->snum;i++)
			if(satip_srv_statistics(c->priv[i].h,&stats))ans=CTLERR;
		else
		{
			u.i5->val1+=stats.rtsp_all_sessions;
			u.i5->val2+=stats.rtsp_client_sessions;
			u.i5->val3+=stats.rtsp_playing_sessions;
			u.i5->val4+=stats.http_running;
			u.i5->val5+=stats.http_playing;
		}
		ctlwrite(s,ans,u.i5,sizeof(CTLINT5));
		return;

	case CTLTUNERSTATS:
		if(size)break;
		memset(v.t,0,sizeof(v.t));
		ans=CTLOK;
		for(j=0,i=0;i<c->tnum;i++)if(satip_hw_info(c->cmn.hw,
			c->tuner[i].deviceid,&hwstats))ans=CTLERR;
		else
		{
			switch(hwstats.msys)
			{
			case SATIP_DVBS:
				strcpy(v.t[j].msys,"DVB-S");
				break;
			case SATIP_DVBS2:
				strcpy(v.t[j].msys,"DVB-S2");
				break;
			case SATIP_DVBT:
				strcpy(v.t[j].msys,"DVB-T");
				break;
			case SATIP_DVBT2:
				strcpy(v.t[j].msys,"DVB-T2");
				break;
			case SATIP_DVBC:
				strcpy(v.t[j].msys,"DVB-C");
				break;
			case SATIP_DVBC2:
				strcpy(v.t[j].msys,"DVB-C2");
				break;
			default:strcpy(v.t[j].msys,"<unknown>");
				break;
			}

			if(hwstats.caps&SATIP_ROFF_AUTO)
				strcat(v.t[j].caps,"ro ");
			if(hwstats.caps&SATIP_AUTOQ)
				strcat(v.t[j].caps,"mtype ");
			if(hwstats.caps&SATIP_PLTS_AUTO)
				strcat(v.t[j].caps,"plts ");
			if(hwstats.caps&SATIP_FEC_AUTO)
				strcat(v.t[j].caps,"fec ");
			if(hwstats.caps&SATIP_BW_AUTO)
				strcat(v.t[j].caps,"bw ");
			if(hwstats.caps&SATIP_TMOD_AUTO)
				strcat(v.t[j].caps,"tmode ");
			if(hwstats.caps&SATIP_GI_AUTO)
				strcat(v.t[j].caps,"gi ");
			if(hwstats.caps&SATIP_SM_AUTO)
				strcat(v.t[j].caps,"sm ");
			if(hwstats.caps&SATIP_HIER_AUTO)
				strcat(v.t[j].caps,"x_hier ");
			if(hwstats.caps&SATIP_TFT_AUTO)
				strcat(v.t[j].caps,"c2tft ");
			if(hwstats.caps&SATIP_SPI_AUTO)
				strcat(v.t[j].caps,"specinv ");
			if(!*v.t[j].caps)strcpy(v.t[j].caps,"<none>");
			else v.t[j].caps[strlen(v.t[j].caps)-1]=0;

			v.t[j].access=hwstats.access;
			v.t[j].open=hwstats.open;
			v.t[j].streams=hwstats.streams;
			v.t[j].lock=hwstats.lock;
			v.t[j].level=hwstats.level;
			v.t[j].quality=hwstats.quality;
			v.t[j].groupstreams=hwstats.groupstreams;
			v.t[j].streambytes=hwstats.streambytes;
			v.t[j].byteupdates=hwstats.byteupdates;

			if(hwstats.open)
			{
				if(satip_util_create(SATIP_TYPE_QRY,0,NULL,0,0,
					&hwstats.tune,NULL,NULL,NULL,NULL,
					v.t[j].tune,sizeof(v.t[0].tune))<=0)
					    strcpy(v.t[j].tune," <unknown>");
			}

			j++;
		}

		ctlwrite(s,ans,&v,j*sizeof(CTLHWSTATS));
		return;

	case CTLRTSPSTATS:
		if(size)break;
		memset(v.list,0,sizeof(v.list));
		for(i=0;i<c->snum;i++)
			v.list[i]=satip_srv_list_rtsp(c->priv[i].h);
		for(val=0,i=0;i<c->snum;i++)
			for(u.e=v.list[i];u.e;u.e=u.e->next)val++;

		if(!(st=malloc(val*sizeof(CTLSVSTATS))))
		{
			for(i=0;i<c->snum;i++)if(v.list[i])
				satip_srv_free_list(c->priv[i].h,v.list[i]);
			break;
		}
		memset(st,0,val*sizeof(CTLSVSTATS));

		for(j=0,i=0;i<c->snum;i++)
			for(u.e=v.list[i];u.e;u.e=u.e->next,j++)
		{
			st[j].instance=i;
			st[j].streamid=u.e->streamid;
			st[j].playing=u.e->playing;
			st[j].port=u.e->port;
			st[j].lock=u.e->state.lock;
			st[j].level=u.e->state.level;
			st[j].quality=u.e->state.quality;
			strcpy(st[j].addr,u.e->addr);
			if(satip_util_create(SATIP_TYPE_QRY,
				c->conf[i].strict?SATIP_STRICTQRY:0,NULL,0,0,
				&u.e->state.tune,&u.e->state.set,NULL,NULL,
				NULL,st[j].tune,sizeof(st[0].tune))<=0)
					strcpy(st[j].tune," <unknown>");
		}

		ctlwrite(s,CTLOK,st,val*sizeof(CTLSVSTATS));
		free(st);
		return;

	case CTLHTTPSTATS:
		if(size)break;
		memset(v.list,0,sizeof(v.list));
		for(i=0;i<c->snum;i++)
			v.list[i]=satip_srv_list_http(c->priv[i].h);
		for(val=0,i=0;i<c->snum;i++)
			for(u.e=v.list[i];u.e;u.e=u.e->next)val++;

		if(!(st=malloc(val*sizeof(CTLSVSTATS))))
		{
			for(i=0;i<c->snum;i++)if(v.list[i])
				satip_srv_free_list(c->priv[i].h,v.list[i]);
			break;
		}
		memset(st,0,val*sizeof(CTLSVSTATS));

		for(j=0,i=0;i<c->snum;i++)
			for(u.e=v.list[i];u.e;u.e=u.e->next,j++)
		{
			st[j].instance=i;
			st[j].streamid=u.e->streamid;
			st[j].playing=u.e->playing;
			st[j].port=u.e->port;
			st[j].lock=u.e->state.lock;
			st[j].level=u.e->state.level;
			st[j].quality=u.e->state.quality;
			strcpy(st[j].addr,u.e->addr);
			if(satip_util_create(SATIP_TYPE_QRY,
				c->conf[i].strict?SATIP_STRICTQRY:0,NULL,0,0,
				&u.e->state.tune,&u.e->state.set,NULL,NULL,
				NULL,st[j].tune,sizeof(st[0].tune))<=0)
					strcpy(st[j].tune," <unknown>");
		}

		ctlwrite(s,CTLOK,st,val*sizeof(CTLSVSTATS));
		free(st);
		break;
	}

	ctlwrite(s,CTLERR,NULL,0);
}

static void enqueue(MSG *msg,COMMON *p)
{
	uint64_t dummy=1;

	pthread_mutex_lock(&p->mtx);
	if(p->fill==MAX_MSGS)
	{
		free(msg);
		goto out;
	}
	p->queue[p->head++]=msg;
	if(p->head==MAX_MSGS)p->head=0;
	p->fill++;
	dummy=write(p->efd,&dummy,sizeof(dummy));
out:	pthread_mutex_unlock(&p->mtx);
}

static MSG *dequeue(COMMON *p)
{
	MSG *m;
	uint64_t dummy;

	pthread_mutex_lock(&p->mtx);
	if(read(p->efd,&dummy,sizeof(dummy))!=sizeof(dummy))goto err;
	if(!p->fill)
	{
err:		m=NULL;
		goto out;
	}
	m=p->queue[p->tail++];
	if(p->tail==MAX_MSGS)p->tail=0;
	p->fill--;
out:	pthread_mutex_unlock(&p->mtx);
	return m;
}

static int webstats(int fd,char *request,PRIV *p)
{
	int i;
	int len;
	char *mem;
	SATIP_SRV_INFO *list;
	SATIP_SRV_INFO *e;
	SATIP_SRV_STATS stats;
	SATIP_HW_STATUS hwstats;
	struct pollfd pp;
	char *bfr;
	char tmp[1024];

	if(!(bfr=malloc(65536)))goto err1;

	if(p->conf->cmn.nostats)goto err2;

	strncpy(bfr,request,65536);
	bfr[65536-1]=0;

	if(!(request=strtok_r(bfr,"\r\n",&mem)))goto err2;

	if(!(request=strtok_r(request," \t",&mem)))goto err2;
	if(strcmp(request,"GET"))goto err2;

	if(!(request=strtok_r(NULL," \t",&mem)))goto err2;
	if(strcmp(request,"/stats.html"))goto err2;

	len=snprintf(bfr,65536,"HTTP/1.1 200 OK\r\nConnection: close\r\n"
		"\r\n"
		"<html><head><title>Statistics</title></head><body><pre>\n");

	if(p->h&&!satip_srv_statistics(p->h,&stats))
	{
		len+=snprintf(bfr+len,65536-len,
			"\nServer Statistics:\n\nTotal RTSP sessions: %d\n"
			"Client RTSP Sessions: %d\nPlaying RTSP sessions: %d\n"
			"Active HTTP sessions: %d\nPlaying HTTP sessions: %d\n",
			stats.rtsp_all_sessions,stats.rtsp_client_sessions,
			stats.rtsp_playing_sessions,stats.http_running,
			stats.http_playing);
	}

	if(p->conf->cmn.hw)
	{
		len+=snprintf(bfr+len,65536-len,"\nTuner Statistics:\n");
		for(i=0;i<p->conf->tnum;i++)if(satip_hw_info(p->conf->cmn.hw,
			p->conf->tuner[i].deviceid,&hwstats))
			len+=snprintf(bfr+len,65536-len,
				"\nTuner %d: no data\n",
				p->conf->tuner[i].deviceid);
		else
		{
			switch(hwstats.msys)
			{
			case SATIP_DVBS:
				mem="DVB-S";
				break;
			case SATIP_DVBS2:
				mem="DVB-S2";
				break;
			case SATIP_DVBT:
				mem="DVB-T";
				break;
			case SATIP_DVBT2:
				mem="DVB-T2";
				break;
			case SATIP_DVBC:
				mem="DVB-C";
				break;
			case SATIP_DVBC2:
				mem="DVB-C2";
				break;
			default:mem="<unknown>";
				break;
			}

			*tmp=0;
			if(hwstats.caps&SATIP_ROFF_AUTO)strcat(tmp," ro");
			if(hwstats.caps&SATIP_AUTOQ)strcat(tmp," mtype");
			if(hwstats.caps&SATIP_PLTS_AUTO)strcat(tmp," plts");
			if(hwstats.caps&SATIP_FEC_AUTO)strcat(tmp," fec");
			if(hwstats.caps&SATIP_BW_AUTO)strcat(tmp," bw");
			if(hwstats.caps&SATIP_TMOD_AUTO)strcat(tmp," tmode");
			if(hwstats.caps&SATIP_GI_AUTO)strcat(tmp," gi");
			if(hwstats.caps&SATIP_SM_AUTO)strcat(tmp," sm");
			if(hwstats.caps&SATIP_HIER_AUTO)strcat(tmp," x_hier");
			if(hwstats.caps&SATIP_TFT_AUTO)strcat(tmp," c2tft");
			if(hwstats.caps&SATIP_SPI_AUTO)strcat(tmp," specinv");
			if(!*tmp)strcpy(tmp," <none>");

			len+=snprintf(bfr+len,65536-len,
				"\nTuner %d (%s):\n%s for clients and "
				"serving %d stream(s) of %d total stream(s).\n"
				"Auto configuration for:%s\n"
				"Served total of %llu bytes (%llu MB) in "
				"%llu.%01d seconds (%llu:%02d:%02d.%01d).\n",
				p->conf->tuner[i].deviceid,mem,
				hwstats.access==SATIP_HW_REMOTE?
					"Enabled":"Disabled",
				hwstats.streams,hwstats.groupstreams,
				tmp,hwstats.streambytes,
				hwstats.streambytes/1048576ULL,
				hwstats.byteupdates/10ULL,
				(int)(hwstats.byteupdates%10ULL),
				hwstats.byteupdates/36000ULL,
				(int)((hwstats.byteupdates/600ULL)%60ULL),
				(int)((hwstats.byteupdates/10ULL)%60ULL),
				(int)(hwstats.byteupdates%10ULL));

			if(!hwstats.open)continue;

			if(satip_util_create(SATIP_TYPE_QRY,0,NULL,0,0,
				&hwstats.tune,NULL,NULL,NULL,NULL,tmp,
				sizeof(tmp))<=0)strcpy(tmp," <unknown>");
			len+=snprintf(bfr+len,65536-len,"Signal Lock %d"
				"    Signal Level %d     Signal Quality %d\n"
				"Tuning Data (no PID details): %s\n",
				hwstats.lock,hwstats.level,hwstats.quality,
				tmp+1);
		}

		if((list=satip_srv_list_rtsp(p->h)))
		{
			len+=snprintf(bfr+len,65536-len,
				"\nRTSP Statistics:\n");

			for(e=list;e;e=e->next)
			{
				if(satip_util_create(SATIP_TYPE_QRY,
					p->conf->conf[p->idx].strict?
					SATIP_STRICTQRY:0,NULL,0,0,
					&e->state.tune,&e->state.set,NULL,NULL,
					NULL,tmp,sizeof(tmp))<=0)
					strcpy(tmp," <unknown>");
				len+=snprintf(bfr+len,65536-len,
					"\nStream %d (IP: %s Port: %d, %s"
					"playing):\nSignal Lock %d    "
					"Signal Level %d     "
					"Signal Quality %d\nTuning Data: %s\n",
					e->streamid,e->addr,e->port,
					e->playing?"":"not ",e->state.lock,
					e->state.level,e->state.quality,tmp+1);
			}

			satip_srv_free_list(p->h,list);
		}

		if((list=satip_srv_list_http(p->h)))
		{
			len+=snprintf(bfr+len,65536-len,
				"\nHTTP Statistics:\n");

			for(e=list;e;e=e->next)
			{
				if(satip_util_create(SATIP_TYPE_QRY,
					p->conf->conf[p->idx].strict?
					SATIP_STRICTQRY:0,NULL,0,0,
					&e->state.tune,&e->state.set,NULL,NULL,
					NULL,tmp,sizeof(tmp))<=0)
					strcpy(tmp," <unknown>");
				len+=snprintf(bfr+len,65536-len,
					"\nStream %d (IP: %s Port: %d):\n"
					"Signal Lock %d    Signal Level %d     "
					"Signal Quality %d\nTuning Data: %s\n",
					e->streamid,e->addr,e->port,
					e->state.lock,e->state.level,
					e->state.quality,tmp+1);
			}

			satip_srv_free_list(p->h,list);
		}
	}

	len+=snprintf(bfr+len,65536-len,"</pre></body></html>\n");

	pp.fd=fd;
	pp.events=POLLOUT;
	mem=bfr;

	while(len)
	{
		if(poll(&pp,1,500)<=0||!(pp.revents&POLLOUT))break;
		if((i=write(fd,mem,len))<=0)break;
		mem+=i;
		len-=i;
	}

	free(bfr);
	return 0;

err2:	free(bfr);
err1:	return -1;
}

static int save(char *dir,char *file,void *data,int len)
{
	int fd;
	char fn[PATH_MAX];

	snprintf(fn,sizeof(fn),"%s/%s",dir,file);
	if((fd=open(fn,O_WRONLY|O_TRUNC|O_CREAT|O_CLOEXEC,0666))==-1)return -1;
	if(write(fd,data,len)!=len)
	{
		close(fd);
		return -1;
	}
	if(close(fd))return -1;
	return 0;
}

static int load(char *dir,char *file,void *data,int len)
{
	int fd;
	char fn[PATH_MAX];

	snprintf(fn,sizeof(fn),"%s/%s",dir,file);
	if((fd=open(fn,O_RDONLY|O_CLOEXEC))==-1)return -1;
	if(read(fd,data,len)!=len)
	{
		close(fd);
		return -1;
	}
	if(close(fd))return -1;
	return 0;
}

static int copym3u(PRIV *p,int level,void **data,int *size)
{
	int i;
	int j;
	int len;
	int l;
	int hlen;
	int rlen;
	char *src;
	char *dst;
	char *tmp;
	char http[128];
	char rtsp[128];

	if(p->ports[0]==SATIP_HTTP_PORT)
	{
		if(!level)snprintf(http,sizeof(http),"%s",p->addr[level]);
		else snprintf(http,sizeof(http),"[%s]",p->addr[level]);
	}
	else if(!level)
		snprintf(http,sizeof(http),"%s:%d",p->addr[level],p->ports[0]);
	else snprintf(http,sizeof(http),"[%s]:%d",p->addr[level],p->ports[0]);
	hlen=strlen(http);

	if(p->ports[1]==SATIP_RTSP_PORT)
	{
		if(!level)snprintf(rtsp,sizeof(rtsp),"%s",p->addr[level]);
		else snprintf(rtsp,sizeof(rtsp),"[%s]",p->addr[level]);
	}
	else if(!level)
		snprintf(rtsp,sizeof(rtsp),"%s:%d",p->addr[level],p->ports[1]);
	else snprintf(rtsp,sizeof(rtsp),"[%s]:%d",p->addr[level],p->ports[1]);
	rlen=strlen(rtsp);

	src=p->conf->cmn.m3umem;
	len=p->conf->cmn.m3usize;
	dst=NULL;
	l=0;

	for(i=0,j=0;i<len;)
	{
		if(i+8192>=l)
		{
			l+=32768;
			if(!(tmp=realloc(dst,l)))
			{
				if(dst)free(dst);
				return -1;
			}
			dst=tmp;
		}

		if(src[i]!='_'||i+8>len)dst[j++]=src[i++];
		else if(!strncmp(&src[i],"__HTTP__",8))
		{
			memcpy(dst+j,http,hlen);
			i+=8;
			j+=hlen;
		}
		else if(!strncmp(&src[i],"__RTSP__",8))
		{
			memcpy(dst+j,rtsp,rlen);
			i+=8;
			j+=rlen;
		}
		else dst[j++]=src[i++];
	}

	if((tmp=realloc(dst,j)))dst=tmp;

	*data=dst;
	*size=j;

	return 0;
}

static void camcb(SATIP_HW_CAM_IO *msg,void *priv)
{
	MSG *m;

	switch(msg->type)
	{
	case SATIP_CAM_STATE:
		if(!(m=malloc(sizeof(MSG))))break;
		m->type=0;
		m->msg=*msg;
		enqueue(m,(COMMON *)priv);
		break;

	case SATIP_CAM_READ:
		if(!(m=malloc(sizeof(MSG)+msg->len)))break;
		m->type=0;
		m->msg=*msg;
		m->msg.data=m->data;
		memcpy(m->data,msg->data,msg->len);
		enqueue(m,(COMMON *)priv);
		break;
	}
}

static int servercb(int code,void *data,void *priv)
{
	int r;
	PRIV *p;
	union
	{
		SATIP_DATA *d;
		SATIP_STRDATA *s;
	}u;
	union
	{
		int i;
		void *ptr;
		MSG *msg;
		char addr[64];
	}v;

	switch(code)
	{
	case SATIP_GETRANDOM:
		u.d=(SATIP_DATA *)data;
		return satip_util_random(u.d->ptrval,u.d->intval);

	case SATIP_PEEROK:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(satip_util_list_total(p->conf->cmn.list)==SATIP_SYSFAIL)
			return 0;
		if(!satip_util_list_match_data(p->conf->cmn.list,data))return 0;
		if(p->conf->cmn.verbose)
		{
			if(satip_util_data2addr(u.d,v.addr,sizeof(v.addr)))
				strcpy(v.addr,"<unknown>");
			printf("host %s access denied\n",v.addr);
		}
		break;

	case SATIP_HTTPOK:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(satip_util_list_total(p->conf->cmn.hlist)==SATIP_SYSFAIL)
			return 0;
		if(!satip_util_list_match_data(p->conf->cmn.hlist,data))
			return 0;
		if(p->conf->cmn.verbose)
		{
			if(satip_util_data2addr(u.d,v.addr,sizeof(v.addr)))
				strcpy(v.addr,"<unknown>");
			printf("host %s http access denied\n",v.addr);
		}
		break;

	case SATIP_RTSPOK:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(satip_util_list_total(p->conf->cmn.rlist)==SATIP_SYSFAIL)
			return 0;
		if(!satip_util_list_match_data(p->conf->cmn.rlist,data))
			return 0;
		if(p->conf->cmn.verbose)
		{
			if(satip_util_data2addr(u.d,v.addr,sizeof(v.addr)))
				strcpy(v.addr,"<unknown>");
			printf("host %s rtsp access denied\n",v.addr);
		}
		break;

	case SATIP_MCSTOK:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(satip_util_list_total(p->conf->cmn.mlist)==SATIP_SYSFAIL)
			return 0;
		if(!satip_util_list_match_data(p->conf->cmn.mlist,data))
			return 0;
		if(p->conf->cmn.verbose)
		{
			if(satip_util_data2addr(u.d,v.addr,sizeof(v.addr)))
				strcpy(v.addr,"<unknown>");
			printf("host %s multicast config access denied\n",
				v.addr);
		}
		break;

	case SATIP_PNG48:
		u.d=(SATIP_DATA *)data;
		u.d->intval=PNG48;
		u.d->ptrval=png48;
		return 0;

	case SATIP_PNG120:
		u.d=(SATIP_DATA *)data;
		u.d->intval=PNG120;
		u.d->ptrval=png120;
		return 0;

	case SATIP_JPG48:
		u.d=(SATIP_DATA *)data;
		u.d->intval=JPG48;
		u.d->ptrval=jpg48;
		return 0;

	case SATIP_JPG120:
		u.d=(SATIP_DATA *)data;
		u.d->intval=JPG120;
		u.d->ptrval=jpg120;
		return 0;

	case SATIP_SAVEBCNT:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(p->conf->cmn.datadir[0])return save(p->conf->cmn.datadir,
			"bcnt",&u.d[0].intval,sizeof(int));
		break;

	case SATIP_SAVEUUID:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(p->conf->cmn.datadir[0])return save(p->conf->cmn.datadir,
			"uuid",u.d[0].ptrval,u.d[0].intval);
		break;

	case SATIP_SAVEDEVID:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(p->conf->cmn.datadir[0])return save(p->conf->cmn.datadir,
			"devid",&u.d[0].intval,sizeof(int));
		break;

	case SATIP_LOADBCNT:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(p->conf->cmn.datadir[0])return load(p->conf->cmn.datadir,
			"bcnt",&u.d[0].intval,sizeof(int));
		break;

	case SATIP_LOADUUID:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(p->conf->cmn.datadir[0])return load(p->conf->cmn.datadir,
			"uuid",u.d[0].ptrval,u.d[0].intval);
		break;

	case SATIP_LOADDEVID:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(p->conf->cmn.datadir[0])return load(p->conf->cmn.datadir,
			"devid",&u.d[0].intval,sizeof(int));
		break;

	case SATIP_HTTPREQ:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		switch(webstats(u.d[1].intval,u.d[0].ptrval,p))
		{
		case 0:	return 0;
		case -1:if(p->conf->cmn.proxyhost[0]&&p->conf->cmn.proxyport)
				return satip_srv_forward(p->h,
						p->conf->cmn.proxyhost,
						p->conf->cmn.proxyport,data);
		default:return -1;
		}

	case SATIP_COPYM3U:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		if(!p->conf->cmn.m3umem||
			copym3u(p,u.d->intval,&u.d->ptrval,&u.d->intval))break;
		return 0;

	case SATIP_STRPLAY:
		u.s=(SATIP_STRDATA *)data;
		p=(PRIV *)priv;
		if(p->conf->cmn.plugin)
		{
			if(!p->conf->cmn.remap)r=plugger_play(p->conf->cmn.hw,
				u.s,satip_srv_stream,satip_srv_status,p,
				SATIP_HW_REMOTE);
			else r=satip_remap_play(p->conf->cmn.hw,u.s,
				plugger_play,plugger_end,plugger_setpids,
				satip_srv_stream,satip_srv_status,p,
				SATIP_HW_REMOTE);
				
		}
		else if(!p->conf->cmn.remap)r=satip_hw_play(p->conf->cmn.hw,u.s,
			satip_srv_stream,satip_srv_status,p,SATIP_HW_REMOTE);
		else r=satip_remap_play(p->conf->cmn.hw,u.s,satip_hw_play,
			satip_hw_end,satip_hw_setpids,satip_srv_stream,
			satip_srv_status,p,SATIP_HW_REMOTE);
		if(r&&p->conf->cmn.verbose)printf("play error %d\n",r);
		return r;

	case SATIP_STREND:
		u.s=(SATIP_STRDATA *)data;
		p=(PRIV *)priv;
		if(p->conf->cmn.plugin)
		{
			if(!p->conf->cmn.remap)
				r=plugger_end(p->conf->cmn.hw,u.s);
			else r=satip_remap_end(p->conf->cmn.hw,u.s);
		}
		else if(!p->conf->cmn.remap)r=satip_hw_end(p->conf->cmn.hw,u.s);
		else r=satip_remap_end(p->conf->cmn.hw,u.s);
		if(r&&p->conf->cmn.verbose)printf("end error %d\n",r);
		return r;

	case SATIP_STRPIDS:
		u.s=(SATIP_STRDATA *)data;
		p=(PRIV *)priv;
		if(p->conf->cmn.plugin)
		{
			if(!p->conf->cmn.remap)
				r=plugger_setpids(p->conf->cmn.hw,u.s);
			else r=satip_remap_setpids(p->conf->cmn.hw,u.s);
		}
		else if(!p->conf->cmn.remap)
			r=satip_hw_setpids(p->conf->cmn.hw,u.s);
		else r=satip_remap_setpids(p->conf->cmn.hw,u.s);
		if(r&&p->conf->cmn.verbose)printf("setpids error %d\n",r);
		return r;

	case SATIP_RUNNING:
		u.d=(SATIP_DATA *)data;
		p=(PRIV *)priv;
		p->ports[0]=u.d[0].intval;
		p->ports[1]=u.d[1].intval;
		for(v.i=0;v.i<=p->conf->conf[p->idx].level;v.i++)
			satip_util_data2addr(&u.d[v.i+2],p->addr[v.i],
				sizeof(p->addr[0]));
		if(p->conf->cmn.verbose)printf("running\n");
		if(!(v.msg=malloc(sizeof(MSG))))
		{
			if(p->conf->cmn.verbose)printf("malloc error\n");
			break;
		}
		v.msg->type=1;
		v.msg->hint=0;
		v.msg->idx=p->idx;
		enqueue(v.msg,&p->conf->cmn);
		return 0;

	case SATIP_STOPPING:
		p=(PRIV *)priv;
		if(p->conf->cmn.verbose)printf("stopping\n");
		if(!(v.msg=malloc(sizeof(MSG))))
		{
			if(p->conf->cmn.verbose)printf("malloc error\n");
			break;
		}
		v.msg->type=1;
		v.msg->hint=1;
		v.msg->idx=p->idx;
		enqueue(v.msg,&p->conf->cmn);
		return 0;
	}

	return -1;
}

int main(int argc,char *argv[])
{
	int sfd;
	int r=1;
	int fg=1;
	int ctl=-1;
	int fdcnt=2;
	char *user=NULL;
	char *group=NULL;
	char *conffile="/etc/satipd.conf";
	char *pidfile=NULL;
	FILE *fp;
	struct pollfd p[3];
	union
	{
		MSG *m;
		sigset_t set;
		struct signalfd_siginfo sig;
		SATIP_CFGINFO hwinfo;
	}u;
	union
	{
		int c;
		char data[256];
	}v;
	CONFIG cfg;

	while((v.c=getopt(argc,argv,"c:u:g:dp:h"))!=-1)switch(v.c)
	{
	case 'c':
		conffile=optarg;
		break;

	case 'u':
		user=optarg;
		break;

	case 'g':
		group=optarg;
		break;

	case 'd':
		fg=0;
		break;

	case 'p':
		pidfile=optarg;
		break;

	default:fprintf(stderr,"Usage: satipd [<options>]\n");
		fprintf(stderr,"-c file   config file (/etc/satipd.conf)\n");
		fprintf(stderr,"-u user   user to run as\n");
		fprintf(stderr,"-g group  group to run as\n");
		fprintf(stderr,"-p file   pid file in daemon mode\n");
		fprintf(stderr,"-d        daemonize\n");
		fprintf(stderr,"-h        this help text\n");
		goto err1;
	}

	if(setpriv(user,group))goto err1;

	if(parse_config(conffile,&cfg,servercb,camcb,fg))goto err2;

	if(pthread_mutex_init(&cfg.cmn.mtx,NULL))
	{
		perror("pthread_mutex_init");
		goto err2;
	}

	if(pthread_rwlock_init(&cfg.cmn.plugmtx,NULL))
	{
		perror("pthread_rwlock_init");
		goto err3;
	}

	if((cfg.cmn.efd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK|EFD_SEMAPHORE))==-1)
	{
		perror("eventfd");
		goto err4;
	}

	sigfillset(&u.set);
	if((sfd=signalfd(-1,&u.set,SFD_NONBLOCK|SFD_CLOEXEC))==-1)
	{
		perror("signalfd");
		goto err5;
	}
	sigprocmask(SIG_BLOCK,&u.set,NULL);

	if(cfg.control[0])
	{
		if((ctl=ctlsocket(cfg.control))==-1)
		{
			fprintf(stderr,"control socket creation failure\n");
			goto err6;
		}
		fdcnt++;
	}

	if(!fg)if(daemon(0,0))
	{
		perror("daemon");
		goto err7;
	}

	if(!(cfg.cmn.hw=satip_hw_init(&cfg.global,&u.hwinfo)))
	{
		fprintf(stderr,"satip_hw_init failed\n");
		goto err7;
	}

	if(droppriv())goto err8;

	for(v.c=0;v.c<cfg.tnum;v.c++)
		if(satip_hw_add(cfg.cmn.hw,&cfg.tuner[v.c],&u.hwinfo))
	{
		fprintf(stderr,"satip_hw_add failed\n");
		goto err8;
	}

	for(v.c=0;v.c<cfg.tnum;v.c++)if(cfg.disable[v.c])
		if(satip_hw_access(cfg.cmn.hw,cfg.tuner[v.c].deviceid,
			SATIP_HW_LOCAL))
	{
		fprintf(stderr,"satip_hw_access failed\n");
		goto err8;
	}

	if(cfg.plugin[0])
	{
		if(!(cfg.cmn.plugin=dlopen(cfg.plugin,RTLD_NOW)))
		{
			fprintf(stderr,"plugin not found\n");
			goto err8;
		}

		cfg.cmn.pluginit=dlsym(cfg.cmn.plugin,"plugin_init");
		cfg.cmn.plugfini=dlsym(cfg.cmn.plugin,"plugin_fini");
		cfg.cmn.plugstream=dlsym(cfg.cmn.plugin,"plugin_stream");
		cfg.cmn.plugmulti=dlsym(cfg.cmn.plugin,"plugin_multi");
		cfg.cmn.plugprep=dlsym(cfg.cmn.plugin,"plugin_prepare");
		cfg.cmn.plugplay=dlsym(cfg.cmn.plugin,"plugin_play");
		cfg.cmn.plugend=dlsym(cfg.cmn.plugin,"plugin_end");
		cfg.cmn.plugpids=dlsym(cfg.cmn.plugin,"plugin_setpids");
		cfg.cmn.plugcam=dlsym(cfg.cmn.plugin,"plugin_cam");

		if(cfg.cmn.pluginit)cfg.cmn.pluginit(cfg.cmn.hw,
			&cfg.cmn.plugctx,cfg.plugcfg[0]?cfg.plugcfg:NULL);
	}

	for(v.c=0;v.c<cfg.snum;v.c++)
		if(!(cfg.priv[v.c].h=satip_srv_init(&cfg.conf[v.c],&u.hwinfo)))
	{
		while(--v.c>=0)satip_srv_fini(cfg.priv[v.c].h);

		fprintf(stderr,"satip_srv_init failed\n");
		goto err9;
	}

	if(cfg.cmn.verbose)printf("starting\n");

	if(!fg&&pidfile)if((fp=fopen(pidfile,"we")))
	{
		fprintf(fp,"%d\n",getpid());
		fclose(fp);
	}

	r=0;

	p[0].fd=sfd;
	p[0].events=POLLIN;
	p[1].fd=cfg.cmn.efd;
	p[1].events=POLLIN;
	p[2].fd=ctl;
	p[2].events=POLLIN;
	p[2].revents=0;

	while(!cfg.restart)
	{
		if(poll(p,fdcnt,-1)<=0)continue;
		if(p[0].revents&POLLIN)
		{
			if(read(sfd,&u.sig,sizeof(u.sig))!=sizeof(u.sig))
				continue;
			switch(u.sig.ssi_signo)
			{
			case SIGINT:
			case SIGHUP:
			case SIGTERM:
				break;
			default:continue;
			}
			break;
		}

		if(p[1].revents&POLLIN)if((u.m=dequeue(&cfg.cmn)))
			switch(u.m->type)
		{
		case 0:	v.c=0;
			if(cfg.cmn.plugcam)
			{
				pthread_rwlock_wrlock(&cfg.cmn.plugmtx);
				cfg.cmn.plugcam(cfg.cmn.hw,cfg.cmn.plugctx,
					&u.m->msg);
				pthread_rwlock_unlock(&cfg.cmn.plugmtx);
				v.c=1;
			}
			if(v.c);
			else if(u.m->msg.type==SATIP_CAM_READ)
			{
			}
			else if(u.m->msg.type==SATIP_CAM_STATE)if(u.m->msg.state
				==(SATIP_CAM_AVAIL|SATIP_CAM_READY))
			{
				u.m->msg.type=SATIP_CAM_WRITE;
				u.m->msg.len=3;
				u.m->msg.data=v.data;
				u.m->msg.tsid=u.m->msg.slot+1;
				v.data[0]=0x82;
				v.data[1]=1;
				v.data[2]=(unsigned char)u.m->msg.tsid;
				satip_hw_cam_io(cfg.cmn.hw,&u.m->msg);
			}
			free(u.m);
			break;

		case 1:	if(u.m->hint)
			{
				cfg.running=0;
				satip_srv_set_all_locks(cfg.priv[u.m->idx].h,1);
				satip_srv_clr_multicast(cfg.priv[u.m->idx].h);
			}
			else
			{
				for(v.c=0;v.c<cfg.onum;v.c++)
					satip_srv_add_multicast(
						cfg.priv[u.m->idx].h,
						cfg.ondemand[v.c].addr,
						cfg.ondemand[v.c].port,
						cfg.conf[u.m->idx].mttl?
						  cfg.conf[u.m->idx].mttl:
						  SATIP_MCST_TTL,0,
						&cfg.ondemand[v.c].tune,
						&cfg.ondemand[v.c].set);
				for(v.c=0;v.c<cfg.pnum;v.c++)
					satip_srv_add_multicast(
						cfg.priv[u.m->idx].h,
						cfg.stream[v.c].addr,
						cfg.stream[v.c].port,
						cfg.conf[u.m->idx].mttl?
						  cfg.conf[u.m->idx].mttl:
						  SATIP_MCST_TTL,1,
						&cfg.stream[v.c].tune,
						&cfg.stream[v.c].set);
				satip_srv_set_all_locks(cfg.priv[u.m->idx].h,0);
				cfg.running=1;
			}
			free(u.m);
			break;
		}

		if(p[2].revents&POLLIN)
		{
			v.c=accept4(ctl,NULL,NULL,SOCK_NONBLOCK|SOCK_CLOEXEC);
			if(v.c!=-1)
			{
				ctlhandler(&cfg,v.c);
				close(v.c);
			}
		}
	}

	if(!fg&&pidfile)unlink(pidfile);

	for(v.c=0;v.c<cfg.snum;v.c++)satip_srv_fini(cfg.priv[v.c].h);
err9:	if(cfg.cmn.plugin)
	{
		if(cfg.cmn.plugfini)
			cfg.cmn.plugfini(cfg.cmn.hw,cfg.cmn.plugctx);
		dlclose(cfg.cmn.plugin);
	}
err8:	satip_hw_fini(cfg.cmn.hw);

err7:	if(ctl!=-1)
	{
		close(ctl);
		unlink(cfg.control);
	}

err6:	close(sfd);
err5:	close(cfg.cmn.efd);
err4:	pthread_rwlock_destroy(&cfg.cmn.plugmtx);
err3:	pthread_mutex_destroy(&cfg.cmn.mtx);

err2:	if(cfg.cmn.m3umem)free(cfg.cmn.m3umem);
	if(cfg.ondemand)free(cfg.ondemand);
	if(cfg.stream)free(cfg.stream);
	if(cfg.cmn.list)satip_util_list_free(cfg.cmn.list);
	if(cfg.restart)execvp(argv[0],argv);

err1:	return r;
}
