/*
 * example SAT>IP DVB loop driver based on CUSE (vanilla kernel is sufficient)
 *
 * Copyright (c) 2016 Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 */

#define FUSE_USE_VERSION 29

#include <linux/dvb/frontend.h>
#include <linux/dvb/dmx.h>

#include <cuse_lowlevel.h>
#include <fuse_lowlevel.h>
#include <fuse_opt.h>

#include <sys/signalfd.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <sys/capability.h>
#include <sys/resource.h>
#include <sys/prctl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/fsuid.h>
#include <dlfcn.h>
#include <stddef.h>
#include <limits.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <poll.h>
#include <pthread.h>

#include "satip.h"

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#define LIKELY(a)	__builtin_expect((a),1)
#define UNLIKELY(a)	__builtin_expect((a),0)
static __inline__ int sync_lock_init(int *a) {*a=0;return 0;}
#define pthread_spin_init(a,b)	sync_lock_init(a)
#define pthread_spin_destroy(a)
#define pthread_spin_lock(a)	while(__sync_lock_test_and_set(a,1))
#define pthread_spin_unlock(a)	__sync_lock_release(a)
#define pthread_spinlock_t	int
#else
#define __attribute__(x)
#define LIKELY(a)	a
#define UNLIKELY(a)	a
#ifdef NO_SPINLOCKS
#define pthread_spin_init(a,b)	pthread_mutex_init(a,NULL)
#define pthread_spin_destroy	pthread_mutex_destroy
#define pthread_spin_lock	pthread_mutex_lock
#define pthread_spin_unlock	pthread_mutex_unlock
#define pthread_spinlock_t	pthread_mutex_t
#endif
#endif

#define MAX_DEVICES 32
#define MAX_TUNERS 32
#define FE_LIST_MAX 16
#define DMX_BUFSIZE (4*SATIP_MAX_BURST*7*188)
#define DVR_BUFSIZE (12*SATIP_MAX_BURST*7*188)
#define STACKSIZE 131072

#ifndef MCL_ONFAULT
#define MCL_ONFAULT 0
#endif

#define FAIL(a) do { err=a; goto fail; } while(0)

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

typedef struct _dmx
{
	struct _dmx *next;
	struct _dmxdta *dmx;
} DMX;

typedef struct _pid
{
	struct _pid *next;
	DMX *dmx;
	int pid;
} PID;

typedef struct
{
	int adapter;
	int major;
	int minbase;

	int owner;
	int group;
	int perms;

	int src;
	int diseqc;
	int dscbits;
	int mode;
	int lnb;
	int maxpids;
	int lnbpower;
	int filtertotal;
	int local;
	int port;
	int fast;
	int udevwait;
	int forcemode;
	int bufsize;
	int fetotal;
	int felist[FE_LIST_MAX];
	char uuid[64];
	char host[64];
	char plugin[PATH_MAX];
	char plugcfg[PATH_MAX];
	SATIP_TUNE *tunefilter;
	void *h;
} CONFIG;

typedef struct
{
	void *(*init)(char *config,void *filter_ctx,
		void (*plugin_pids)(void *ctx,SATIP_PIDS *set,
			int from_notify_hook),void *ctx);
	void (*exit)(void *plugin_ctx);
	void (*notify_pids)(void *plugin_ctx,short *pids,int total);
	void (*pre_pids)(void *plugin_ctx,SATIP_EXTRA **extra);
	void (*post_pids)(void *plugin_ctx,int success);
	void (*pre_tune)(void *plugin_ctx,SATIP_TUNE *tune,
		SATIP_EXTRA **extra);
	void (*post_tune)(void *plugin_ctx,SATIP_TUNE *tune,int success);
	void (*no_tune)(void *plugin_ctx);
	void (*strinit)(void *plugin_ctx,void **strctx);
	void (*strexit)(void *plugin_ctx,void *strctx);
	void (*stream)(void *plugin_ctx,void *strctx,unsigned char *ts188);
	void (*multi)(void *plugin_ctx,void *strctx,unsigned char *ts188b1,
		int len1,unsigned char *ts188b2,int len2,int *rlen,int flush);
	void (*cam)(void *plugin_ctx,void *handle,SATIP_HW_CAM_IO *msg);
	void *ctx;
	SATIP_PIDS set;
} PLUGIN;

typedef struct
{
	unsigned long long sec;
	unsigned long long pes;
	unsigned long long dmx;
	unsigned long long dvr;
	unsigned long long net;
} XRUN;

typedef struct
{
	void *filter;
	void *sh;
	struct _stream *s;
	struct _stream *list;
	struct _stream *fe;
	struct _stream *ehd;
	struct _stream *etl;
	unsigned char *dvrbfr;
	struct fuse_session *fese;
	struct fuse_session *dmxse;
	struct fuse_session *dvrse;
	void *dlh;
	PID *pids;
	void *plugdta;
	int dvrok:1;
	int dmxok:1;
	int feok:1;
	int hl:1;
	int ffd;
	int nfd;
	int tfd;
	int cfd;
	int ifd;
	int xfd;
	int dvrbufsize;
	int dvrthres;
	int todvr;
	int dvropen;
	int dvrrun;
	int dvrhead;
	int dvrtail;
	int dvrfill;
	int plughead;
	int plugfill;
	int plugwork;
	int feopen;
	int fecnt;
	int running;
	int enabled;
	int event;
	int src;
	int signal;
	int sigcnt;
	clock_t ticks;
	clock_t start;
	int timeouts;
	int reply;
	int feid;
	unsigned int tunecnt;
	unsigned int freq;
	XRUN xrun;
	pthread_spinlock_t spin;
	pthread_rwlock_t mtx;
	pthread_mutex_t tmtx;
	pthread_mutex_t cmtx;
	pthread_t th[5];
	SATIP_STATUS status;
	SATIP_TUNE tune;
	SATIP_TUNE feedertune;
	SATIP_PIDS set;
	SATIP_PIDS feederset;
	PLUGIN plug;
	CONFIG conf;
} DEVICE;

typedef struct _dmxdta
{
	void *usrflt;
	void *secflt;
	void *plugdta;
	DEVICE *dev;
	struct _stream *s;
	unsigned char *bfr;
	int bufsize;
	int thres;
	int open;
	int ftype;
	int running;
	int head;
	int tail;
	int fill;
	int plughead;
	int plugfill;
	int plugwork;
	int oneshot;
	clock_t timeout;
	SATIP_UTIL_SECFILTER filter;
} DMXDTA;

typedef struct _stream
{
	struct _stream *next;
	struct _stream *enx;
	DEVICE *dev;
	DMXDTA *dmx;
	struct fuse_pollhandle *ph;
	struct fuse_req *req;
	clock_t start;
	int flags;
	int notify;
	int event;
	int type;
	int size;
	pthread_mutex_t mtx;
} STREAM;

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
	int dnum;
	int tnum;
	int nnum;
	int rtprio;
	SATIP_CLN_CONFIG cln;
	CONFIG dev[MAX_DEVICES];
	SATIP_HW_TUNERCFG tuner[MAX_TUNERS];
} GLOBAL;

typedef struct
{
	SATIP_HW_CAM_IO msg;
	unsigned char data[0];
} MSG;

typedef struct
{
	int head;
	int tail;
	int fill;
	int efd;
	MSG *queue[MAX_TUNERS];
	pthread_mutex_t mtx;
} CAMQUEUE;

#ifndef PROFILE

static void secstream(void *data,int len,void *priv) __attribute__ ((hot));
static void pesstream(void *data,int len,void *priv) __attribute__ ((hot));
static void dmxstream(void *data,int len,void *priv) __attribute__ ((hot));
static void dmxmulti(void *data,int len,void *priv) __attribute__ ((hot));
static void dvrstream(void *data,int len,void *priv) __attribute__ ((hot));
static void dvrmulti(void *data,int len,void *priv) __attribute__ ((hot));
static void nextpacket(void *priv) __attribute__ ((hot));
static void streamer(SATIP_DATA *data,void *priv) __attribute__ ((hot));
static void hwstream(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void hwstatus(void *id,SATIP_STATUS *status) __attribute__ ((hot));
static void dvr_read(fuse_req_t req,size_t size,off_t off,
	struct fuse_file_info *fi) __attribute__ ((hot));
static void dvr_poll(fuse_req_t req,struct fuse_file_info *fi,
	struct fuse_pollhandle *ph) __attribute__ ((hot));
static void dmx_read(fuse_req_t req,size_t size,off_t off,
	struct fuse_file_info *fi) __attribute__ ((hot));
static void dmx_poll(fuse_req_t req,struct fuse_file_info *fi,
	struct fuse_pollhandle *ph) __attribute__ ((hot));
static void *notifier(void *data) __attribute__ ((hot));

static void dvr_post(void *userdata) __attribute__ ((cold));
static void dmx_post(void *userdata) __attribute__ ((cold));
static void fe_post(void *userdata) __attribute__ ((cold));
static void *dvrworker(void *data) __attribute__ ((cold));
static void *dmxworker(void *data) __attribute__ ((cold));
static void *feworker(void *data) __attribute__ ((cold));
static int dummy_args(void *data,const char *arg,int key,
	struct fuse_args *outargs) __attribute__ ((cold));
static int doinv(void *s,char *val) __attribute__ ((cold));
static int dosrc(void *s,char *val) __attribute__ ((cold));
static int dooctal(void *s,char *val) __attribute__ ((cold));
static int dohost(void *s,char *val) __attribute__ ((cold));
static int dodev(void *s,char *val) __attribute__ ((cold));
static int dopath(void *s,char *val) __attribute__ ((cold));
static int dofelist(void *s,char *val) __attribute__ ((cold));
static int doparam(CFGFILE *p,void *s,char *name,char *val)
	__attribute__ ((cold));
static int parse_config(char *fn,GLOBAL *c) __attribute__ ((cold));
static int setpriv(int rtprio)  __attribute__ ((cold));
static void *create(CONFIG *config) __attribute__ ((cold));
static void destroy(void *ctx) __attribute__ ((cold));
static void handler(int unused) __attribute__ ((cold));
int main(int argc,char *argv[]) __attribute__ ((cold));

#else

static int doinv(void *s,char *val);
static int dosrc(void *s,char *val);
static int dooctal(void *s,char *val);
static int dohost(void *s,char *val);
static int dodev(void *s,char *val);
static int dopath(void *s,char *val);
static int dofelist(void *s,char *val);

#endif

static CFGFILE gparams[]=
{
	{"rtprio",1,99,offsetof(GLOBAL,rtprio),NULL},
	{NULL,0,0,0L,NULL}
};

static CFGFILE cparams[]=
{
	{"level",0,2,offsetof(SATIP_CLN_CONFIG,level),NULL},
	{"dev",0,0,offsetof(SATIP_CLN_CONFIG,dev),dodev},
	{"mttl",1,255,offsetof(SATIP_CLN_CONFIG,mttl),NULL},
	{"portmin",1024,65534,offsetof(SATIP_CLN_CONFIG,portmin),NULL},
	{"portmax",1025,65535,offsetof(SATIP_CLN_CONFIG,portmax),NULL},
	{"rtpbuffer",32768,1048576,offsetof(SATIP_CLN_CONFIG,rtpbuffer),NULL},
	{"strict",0,1,offsetof(SATIP_CLN_CONFIG,strict),NULL},
	{"fast",0,1,offsetof(SATIP_CLN_CONFIG,fast),NULL},
	{NULL,0,0,0L,NULL}
};

static CFGFILE dparams[]=
{
	{"adapter",0,255,offsetof(CONFIG,adapter),NULL},
	{"major",0,8191,offsetof(CONFIG,major),NULL},
	{"minbase",0,252,offsetof(CONFIG,minbase),NULL},
	{"owner",0,65535,offsetof(CONFIG,owner),NULL},
	{"group",0,65535,offsetof(CONFIG,group),NULL},
	{"perms",0,0,offsetof(CONFIG,perms),dooctal},
	{"src",1,255,offsetof(CONFIG,src),NULL},
	{"diseqc",0,9,offsetof(CONFIG,diseqc),NULL},
	{"dscbits",0,4,offsetof(CONFIG,dscbits),NULL},
	{"mode",1,6,offsetof(CONFIG,mode),NULL},
	{"lnb",0,4,offsetof(CONFIG,lnb),NULL},
	{"felist",0,0,offsetof(CONFIG,felist),dofelist},
	{"maxpids",0,SATIP_MAX_PIDS,offsetof(CONFIG,maxpids),NULL},
	{"lnbpower",0,1,offsetof(CONFIG,lnbpower),NULL},
	{"local",0,1,offsetof(CONFIG,local),NULL},
	{"port",1,65535,offsetof(CONFIG,port),NULL},
	{"uuid",0,0,offsetof(CONFIG,uuid),dohost},
	{"host",0,0,offsetof(CONFIG,host),dohost},
	{"plugin",0,0,offsetof(CONFIG,plugin),dopath},
	{"plugcfg",0,0,offsetof(CONFIG,plugcfg),dopath},
	{"fast",0,1,offsetof(CONFIG,fast),NULL},
	{"udevwait",0,500,offsetof(CONFIG,udevwait),NULL},
	{"forcemode",0,1,offsetof(CONFIG,forcemode),NULL},
	{"bufferkb",64,1024,offsetof(CONFIG,bufsize),NULL},
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

static const struct fuse_opt dummy_opts[]=
{
	FUSE_OPT_END
};

static int dummy_args(void *data,const char *arg,int key,
	struct fuse_args *outargs)
{
	return 1;
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

static int dooctal(void *s,char *val)
{
	int num;

	if(!*val)return -1;
	for(num=0;*val;val++)
	{
		if(*val<'0'||*val>'7')return -1;
		num<<=3;
		num+=*val-'0';
		if(num>0777)return -1;
	}
	num&=0666;
	*((int *)s)=num;
	return 0;
}

static int dohost(void *s,char *val)
{
	CONFIG *unused __attribute__ ((unused));

	if(!*val||strlen(val)>=sizeof(unused->host))return -1;
	strcpy(s,val);
	return 0;
}

static int dodev(void *s,char *val)
{
	if(!*val||strlen(val)>IFNAMSIZ)return -1;
	strcpy(s,val);
	return 0;
}

static int dopath(void *s,char *val)
{
	if(!*val||strlen(val)>=PATH_MAX)return -1;
	strcpy(s,val);
	return 0;
}

static int dofelist(void *s,char *val)
{
	int i;
	char *mem;
	int *felist=(int *)s;

	for(i=0,val=strtok_r(val,",",&mem);val;i++,val=strtok_r(NULL,",",&mem))
	{
		if(!*val||i>=FE_LIST_MAX)return -1;
		felist[i]=atoi(val);
		if(felist[i]<1||felist[i]>65535)return -1;
	}
	if(!i)return -1;
	felist[-1]=i;
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

static int parse_config(char *fn,GLOBAL *c)
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
	SATIP_TUNE *t;
	CONFIG *dev=NULL;
	SATIP_HW_TUNERCFG *tuner=NULL;
	SATIP_HW_TUNERCFG *cmp;
	SATIP_PIDS set;
	SATIP_PIDS add;
	SATIP_PIDS del;
	char bfr[8192];

	memset(c,0,sizeof(GLOBAL));

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
			dev=NULL;
			tuner=NULL;
			continue;
		}

		if(!strcmp(name,"[device]"))
		{
			glob=-1;
			tuner=NULL;
			if(c->dnum==MAX_DEVICES)
			{
				fprintf(stderr,"Out of memory\n");
				goto err2;
			}
			dev=&c->dev[c->dnum++];
			dev->udevwait=15;
			continue;
		}

		if(!strcmp(name,"[tuner]"))
		{
			glob=-1;
			dev=NULL;
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
			if(doparam(gparams,c,name,val))
				if(doparam(cparams,&c->cln,name,val))goto err3;
		}
		else if(dev)
		{
			if(!strcmp(name,"tune"))
			{
				if(!(t=realloc(dev->tunefilter,
				    (dev->filtertotal+1)*sizeof(SATIP_TUNE))))
				{
					fprintf(stderr,"Out of memory\n");
					goto err2;
				}
				dev->tunefilter=t;
				t=&t[dev->filtertotal++];
				satip_util_init_tune(t);
				if(satip_util_parse(SATIP_PARSE_QRY,0,
					SATIP_IGNCAPS|SATIP_IGNPLPETC,
					val+1,NULL,NULL,0,NULL,
					NULL,t,&set,&add,&del,NULL)||
					set.numpids!=SATIP_NOPIDS||add.numpids||
					del.numpids)goto err3;
			}
			else if(doparam(dparams,dev,name,val))goto err3;
		}
		else if (tuner)
		{
			if(doparam(tparams,tuner,name,val))goto err3;
		}
		else goto err3;
	}

	if(!c->dnum)
	{
		fprintf(stderr,"No device(s) defined.\n");
		goto err2;
	}

	if(!c->cln.dev[0])
	{
		fprintf(stderr,"No network device specified.\n");
		goto err2;
	}

	if(!c->cln.mttl)c->cln.mttl=SATIP_MCST_TTL;
	c->cln.interval=SATIP_UPNP_OFF;
	c->cln.idleflush=1;
	c->cln.stack=STACKSIZE;

	for(k=0;k<c->dnum;k++)
	{
		dev=&c->dev[k];
		if(!dev->mode)goto err4;
		if(dev->minbase%4)goto err4;
		if(dev->mode==SATIP_DVBS||dev->mode==SATIP_DVBS2)
		{
			if(!dev->src)dev->src=1;
		}
		else if(dev->src||dev->diseqc||dev->dscbits||dev->lnb||
			dev->lnbpower)goto err4;
		if(!dev->maxpids)dev->maxpids=SATIP_MAX_PIDS;
		if(dev->local)
		{
			if(dev->port||dev->host[0]||dev->uuid[0])goto err4;
			if(!c->tnum)
			{
				fprintf(stderr,"No tuner(s) defined.\n");
				goto err2;
			}
		}
		else
		{
			c->nnum++;
			if(!dev->port)dev->port=SATIP_RTSP_PORT;
			if(dev->uuid[0])c->cln.interval=1;
			else if(!dev->host[0])goto err4;
		}
		for(j=0;j<c->dnum;j++)if(j==k)continue;
		else if(dev->minbase==c->dev[j].minbase)goto err4;
		if(!dev->bufsize)dev->bufsize=DVR_BUFSIZE;
		else
		{
			dev->bufsize*=1024;
			dev->bufsize/=188;
			dev->bufsize*=188;
		}
	}

	for(k=0;k<c->tnum;k++)
	{
		tuner=&c->tuner[k];

		if(!tuner->deviceid)goto err5;

		if(tuner->srcnum)
		{
			for(i=tuner->srcnum-1;i;i--)for(j=i-1;j>=0;j--)
				if(tuner->src[i]==tuner->src[j])goto err5;

			for(j=(tuner->src[0]-1)>>2,i=1;i<tuner->srcnum;i++)
				if(((tuner->src[i]-1)>>2)!=j)goto err5;
		}

		for(l=k+1;l<c->tnum;l++)
		{
			cmp=&c->tuner[l];

			if(tuner->frontend==cmp->frontend||
				tuner->demux==cmp->demux||tuner->ca==cmp->ca)
					if(tuner->adapter==cmp->adapter)
						goto err5;
			if(tuner->deviceid==cmp->deviceid)goto err5;
		}
	}

	posix_fadvise(fileno(fp),0,0,POSIX_FADV_DONTNEED);
	fclose(fp);
	return 0;

err5:	fprintf(stderr,"Syntax error in tuner definition.\n");
	goto err2;
err4:	fprintf(stderr,"Syntax error in device definition.\n");
	goto err2;
err3:	fprintf(stderr,"Syntax error in line %d of %s\n",line,fn);
	for(k=0;k<=c->dnum;k++)if(c->dev[k].tunefilter)
		free(c->dev[k].tunefilter);
err2:	fclose(fp);
err1:	return -1;
}

static int setpriv(int rtprio)
{
	int csn=1;
	struct rlimit r;
	struct __user_cap_header_struct hdr;
	struct __user_cap_data_struct cap[2];
	struct sched_param s;
	uid_t uid;
	gid_t gid;

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

	if(!TESTCAP(cap,CAP_SETUID))
	{
		fprintf(stderr,"error: need CAP_SETUID\n");
		return -1;
	}

	if(prctl(PR_SET_KEEPCAPS,1,0,0,0))
	{
		perror("prctl(PR_SET_KEEPCAPS)");
		return -1;
	}

	uid=getuid();
	gid=getgid();

	if(setresuid(0,0,0))
	{
		perror("setresuid");
		return -1;
	}

	if(setresgid(gid,gid,gid))
	{
		perror("setresgid");
		return -1;
	}

	if(!TESTCAP(cap,CAP_SYS_NICE)&&rtprio)
	{
		csn=0;
		fprintf(stderr,"warning: may need CAP_SYS_NICE\n");
	}

	if(TESTCAP(cap,CAP_SYS_RESOURCE))
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

	if(mlockall(MCL_CURRENT|MCL_FUTURE|MCL_ONFAULT))fprintf(stderr,
		"warning: mlockall failed, missing CAP_IPC_LOCK?\n");

	if(rtprio)
	{
		s.sched_priority=rtprio;
		if(sched_setscheduler(0,SCHED_RR,&s))fprintf(stderr,
			"warning: sched_setscheduler failed, missing "
			"CAP_SYS_NICE?\n");
	}

	if(setresuid(uid,uid,0))
	{
		perror("setresuid");
		return -1;
	}

	if(prctl(PR_SET_DUMPABLE,1,0,0,0))
	{
		perror("prctl(PR_SET_DUMPABLE)");
		return -1;
	}

	memset(cap,0,sizeof(cap));

	if(csn)SETCAP(cap,CAP_SYS_NICE);

	if(capset(&hdr,cap))
	{
		perror("capset");
		return -1;
	}

	if(prctl(PR_SET_KEEPCAPS,0,0,0,0))
	{
		perror("prctl(PR_SET_KEEPCAPS)");
		return -1;
	}

	if(prctl(PR_SET_NO_NEW_PRIVS,1,0,0,0))
		fprintf(stderr,
			"warning: PR_SET_NO_NEW_PRIVS fail, old kernel?\n");

	return 0;
}

static void secstream(void *data,int len,void *priv)
{
	DMXDTA *dmx=(DMXDTA *)priv;

	if(UNLIKELY(!data))return;

	pthread_spin_lock(&dmx->dev->spin);

	if(!dmx->dev->fecnt||!dmx->running||dmx->oneshot==-1)goto out;

	if(dmx->bufsize-dmx->fill>len+2)
	{
		if(dmx->tail)
		{
			memmove(dmx->bfr,dmx->bfr+dmx->tail,dmx->fill);
			dmx->head-=dmx->tail;
			dmx->tail=0;
		}
		dmx->bfr[dmx->head]=(unsigned char)(len>>8);
		dmx->bfr[dmx->head+1]=(unsigned char)len;
		memcpy(dmx->bfr+dmx->head+2,data,len);
		dmx->head+=len+2;
		dmx->fill+=len+2;
		if(dmx->oneshot)dmx->oneshot=-1;
		if(dmx->s->notify&&!dmx->s->event)
		{
			dmx->s->event=1;
			dmx->dev->signal=1;
			dmx->s->enx=NULL;
			if(!dmx->dev->etl)dmx->dev->ehd=dmx->dev->etl=dmx->s;
			else dmx->dev->etl=dmx->dev->etl->enx=dmx->s;
		}
	}
	else dmx->dev->xrun.sec++;

out:	pthread_spin_unlock(&dmx->dev->spin);
}

static void pesstream(void *data,int len,void *priv)
{
	DMXDTA *dmx=(DMXDTA *)priv;
	unsigned char *ptr=(unsigned char *)data;
	int l;

	if(UNLIKELY(!data))return;

	pthread_spin_lock(&dmx->dev->spin);

	if(!dmx->dev->fecnt||!dmx->running||(len&0xffff)!=188)goto out;

	if(dmx->dev->plug.stream)
		dmx->dev->plug.stream(dmx->dev->plug.ctx,dmx->plugdta,data);

	len=184;
	if(!(ptr[3]&0x10))goto out;
	if(ptr[3]&0x20)
	{
		if(ptr[4]>182)goto out;
		len-=ptr[4]+1;
	}

	if(dmx->bufsize-dmx->fill>=len)
	{
		l=len;
		if(l>dmx->bufsize-dmx->head)l=dmx->bufsize-dmx->head;
		memcpy(dmx->bfr+dmx->head,ptr+4,l);
		if((dmx->head+=l)==dmx->bufsize)dmx->head=0;
		dmx->fill+=l;
		if((len-=l))
		{
			memcpy(dmx->bfr+dmx->head,ptr+4+l,len);
			dmx->head+=len;
			dmx->fill+=len;
		}
		if(dmx->s->notify&&!dmx->s->event)
		{
			dmx->s->event=1;
			dmx->dev->signal=1;
			dmx->s->enx=NULL;
			if(!dmx->dev->etl)dmx->dev->ehd=dmx->dev->etl=dmx->s;
			else dmx->dev->etl=dmx->dev->etl->enx=dmx->s;
		}
	}
	else dmx->dev->xrun.pes++;

out:	pthread_spin_unlock(&dmx->dev->spin);
}

static void dmxstream(void *data,int len,void *priv)
{
	DMXDTA *dmx=(DMXDTA *)priv;
	DEVICE *dev=dmx->dev;

	if(UNLIKELY(!data))return;

	pthread_spin_lock(&dev->spin);

	if(!dev->fecnt||!dmx->running||(len&0xffff)!=188)goto out;

	if(dev->plug.stream)dev->plug.stream(dev->plug.ctx,dmx->plugdta,data);

	if(dmx->bufsize-dmx->fill>=188)
	{
		memcpy(dmx->bfr+dmx->head,data,188);
		if((dmx->head+=188)==dmx->bufsize)dmx->head=0;
		dmx->fill+=188;
		if(dmx->s->notify&&!dmx->s->event)
		{
			dmx->s->event=1;
			dev->signal=1;
			dmx->s->enx=NULL;
			if(!dev->etl)dev->ehd=dev->etl=dmx->s;
			else dev->etl=dev->etl->enx=dmx->s;
		}
	}
	else dev->xrun.dmx++;

out:	pthread_spin_unlock(&dev->spin);
}

static void dmxmulti(void *data,int len,void *priv)
{
	DMXDTA *dmx=(DMXDTA *)priv;
	DEVICE *dev=dmx->dev;
	unsigned char *ptr1;
	unsigned char *ptr2;
	int rlen;
	int len1;
	int len2;

	pthread_spin_lock(&dev->spin);

	if(!dev->fecnt||!dmx->running||(len&0xffff)!=188)goto out;

	if(UNLIKELY(!data))
	{
		if(!dmx->plugfill)goto out;

		if(dmx->head+dmx->plugfill>dmx->bufsize)
		{
			ptr1=dmx->bfr+dmx->head;
			len1=dmx->bufsize-dmx->head;
			ptr2=dmx->bfr;
			len2=dmx->plugfill-len1;
		}
		else
		{
			ptr1=dmx->bfr+dmx->head;
			len1=dmx->plugfill;
			ptr2=NULL;
			len2=0;
		}
		rlen=0;

		dmx->plugwork=1;
		pthread_spin_unlock(&dev->spin);

		dev->plug.multi(dev->plug.ctx,dmx->plugdta,ptr1,len1,ptr2,len2,
			&rlen,1);

		pthread_spin_lock(&dev->spin);
		dmx->plugwork=0;

		if(!dev->fecnt||!dmx->running||!rlen)goto out;

		dmx->plugfill-=rlen;
		dmx->fill+=rlen;
		dmx->head+=rlen;
		if(dmx->head>dmx->bufsize)dmx->head-=dmx->bufsize;

		if(dmx->s->notify&&!dmx->s->event)
		{
			dmx->s->event=1;
			dev->signal=1;
			dmx->s->enx=NULL;
			if(!dev->etl)dev->ehd=dev->etl=dmx->s;
			else dev->etl=dev->etl->enx=dmx->s;
		}

		goto out;
	}

	if(dmx->bufsize-dmx->fill-dmx->plugfill>=188)
	{
		memcpy(dmx->bfr+dmx->plughead,data,188);
		if((dmx->plughead+=188)==dmx->bufsize)dmx->plughead=0;
		dmx->plugfill+=188;

		if(dmx->head+dmx->plugfill>dmx->bufsize)
		{
			ptr1=dmx->bfr+dmx->head;
			len1=dmx->bufsize-dmx->head;
			ptr2=dmx->bfr;
			len2=dmx->plugfill-len1;
		}
		else
		{
			ptr1=dmx->bfr+dmx->head;
			len1=dmx->plugfill;
			ptr2=NULL;
			len2=0;
		}
		rlen=0;

		dmx->plugwork=1;
		pthread_spin_unlock(&dev->spin);

		dev->plug.multi(dev->plug.ctx,dmx->plugdta,ptr1,len1,ptr2,len2,
			&rlen,dmx->plugfill>=dmx->thres?1:0);

		pthread_spin_lock(&dev->spin);
		dmx->plugwork=0;

		if(!dev->fecnt||!dmx->running||!rlen)goto out;

		dmx->plugfill-=rlen;
		dmx->fill+=rlen;
		dmx->head+=rlen;
		if(dmx->head>dmx->bufsize)dmx->head-=dmx->bufsize;

		if(dmx->s->notify&&!dmx->s->event)
		{
			dmx->s->event=1;
			dev->signal=1;
			dmx->s->enx=NULL;
			if(!dev->etl)dev->ehd=dev->etl=dmx->s;
			else dev->etl=dev->etl->enx=dmx->s;
		}
	}
	else dev->xrun.dmx++;

out:	pthread_spin_unlock(&dev->spin);
}

static void dvrstream(void *data,int len,void *priv)
{
	DMXDTA *dmx=(DMXDTA *)priv;
	DEVICE *dev=dmx->dev;

	if(UNLIKELY(!data))return;

	pthread_spin_lock(&dev->spin);

	if(!dmx->running||!dev->fecnt||!dev->dvropen||!dev->dvrrun||dev->todvr||
		(len&0xffff)!=188)goto out;

	dev->todvr=1;

	if(dev->plug.stream)dev->plug.stream(dev->plug.ctx,dev->plugdta,data);

	if(dev->dvrbufsize-dev->dvrfill>=188)
	{
		memcpy(dev->dvrbfr+dev->dvrhead,data,188);
		if((dev->dvrhead+=188)==dev->dvrbufsize)dev->dvrhead=0;
		dev->dvrfill+=188;
		if(dev->s->notify&&!dev->s->event)
		{
			dev->s->event=1;
			dev->signal=1;
			dev->s->enx=NULL;
			if(!dev->etl)dev->ehd=dev->etl=dev->s;
			else dev->etl=dev->etl->enx=dev->s;
		}
	}
	else dev->xrun.dvr++;

out:	pthread_spin_unlock(&dev->spin);
}

static void dvrmulti(void *data,int len,void *priv)
{
	DMXDTA *dmx=(DMXDTA *)priv;
	DEVICE *dev=dmx->dev;
	unsigned char *ptr1;
	unsigned char *ptr2;
	int rlen;
	int len1;
	int len2;

	pthread_spin_lock(&dev->spin);

	if(!dmx->running||!dev->fecnt||!dev->dvropen||!dev->dvrrun||dev->todvr||
		(len&0xffff)!=188)goto out;

	dev->todvr=1;

	if(UNLIKELY(!data))
	{
		if(!dev->plugfill)goto out;

		if(dev->dvrhead+dev->plugfill>dev->dvrbufsize)
		{
			ptr1=dev->dvrbfr+dev->dvrhead;
			len1=dev->dvrbufsize-dev->dvrhead;
			ptr2=dev->dvrbfr;
			len2=dev->plugfill-len1;
		}
		else
		{
			ptr1=dev->dvrbfr+dev->dvrhead;
			len1=dev->plugfill;
			ptr2=NULL;
			len2=0;
		}
		rlen=0;

		dev->plugwork=1;
		pthread_spin_unlock(&dev->spin);

		dev->plug.multi(dev->plug.ctx,dev->plugdta,ptr1,len1,ptr2,len2,
			&rlen,1);

		pthread_spin_lock(&dev->spin);
		dev->plugwork=0;

		if(!dmx->running||!dev->fecnt||!dev->dvropen||!dev->dvrrun||
			!rlen)goto out;

		dev->plugfill-=rlen;
		dev->dvrfill+=rlen;
		dev->dvrhead+=rlen;
		if(dev->dvrhead>dev->dvrbufsize)dev->dvrhead-=dev->dvrbufsize;

		if(dev->s->notify&&!dev->s->event)
		{
			dev->s->event=1;
			dev->signal=1;
			dev->s->enx=NULL;
			if(!dev->etl)dev->ehd=dev->etl=dev->s;
			else dev->etl=dev->etl->enx=dev->s;
		}

		goto out;
	}

	if(dev->dvrbufsize-dev->dvrfill-dev->plugfill>=188)
	{
		memcpy(dev->dvrbfr+dev->plughead,data,188);
		if((dev->plughead+=188)==dev->dvrbufsize)dev->plughead=0;
		dev->plugfill+=188;

		if(dev->dvrhead+dev->plugfill>dev->dvrbufsize)
		{
			ptr1=dev->dvrbfr+dev->dvrhead;
			len1=dev->dvrbufsize-dev->dvrhead;
			ptr2=dev->dvrbfr;
			len2=dev->plugfill-len1;
		}
		else
		{
			ptr1=dev->dvrbfr+dev->dvrhead;
			len1=dev->plugfill;
			ptr2=NULL;
			len2=0;
		}
		rlen=0;

		dev->plugwork=1;
		pthread_spin_unlock(&dev->spin);

		dev->plug.multi(dev->plug.ctx,dev->plugdta,ptr1,len1,ptr2,len2,
			&rlen,dev->plugfill>=dev->dvrthres?1:0);

		pthread_spin_lock(&dev->spin);
		dev->plugwork=0;

		if(!dmx->running||!dev->fecnt||!dev->dvropen||!dev->dvrrun||
			!rlen)goto out;

		dev->plugfill-=rlen;
		dev->dvrfill+=rlen;
		dev->dvrhead+=rlen;
		if(dev->dvrhead>dev->dvrbufsize)dev->dvrhead-=dev->dvrbufsize;

		if(dev->s->notify&&!dev->s->event)
		{
			dev->s->event=1;
			dev->signal=1;
			dev->s->enx=NULL;
			if(!dev->etl)dev->ehd=dev->etl=dev->s;
			else dev->etl=dev->etl->enx=dev->s;
		}
	}
	else dev->xrun.dvr++;

out:	pthread_spin_unlock(&dev->spin);
}

static void nextpacket(void *priv)
{
	DEVICE *dev=(DEVICE *)priv;

	dev->todvr=0;
}

static void streamer(SATIP_DATA *data,void *priv)
{
	DEVICE *dev=(DEVICE *)priv;
	uint64_t dummy=1;
	clock_t curr;
	int flag=0;

	dev->signal=0;

	if(UNLIKELY(!data))
	{
		satip_util_filter_packets(dev->filter,NULL,0);

		if(dev->signal||dev->sigcnt)
		{
			dummy=write(dev->nfd,&dummy,sizeof(dummy));
			dev->sigcnt=0;
		}

		return;
	}

	switch(data->intval&(SATIP_RTP|SATIP_RTCP))
	{
	case SATIP_RTP:
		if(data->intval&SATIP_LOSTPKT)
		{
			pthread_spin_lock(&dev->spin);
			dev->xrun.net++;
			pthread_spin_unlock(&dev->spin);
		}
		if(UNLIKELY((data->intval&0xffff)!=1316))flag=1;
		satip_util_filter_packets_cb(dev->filter,
			data->ptrval,data->intval,nextpacket,dev);
		if(flag)satip_util_filter_packets(dev->filter,NULL,0);
		break;

	case SATIP_RTCP:
		pthread_spin_lock(&dev->spin);
		dev->status=*((SATIP_STATUS *)data->ptrval);
		if(dev->feopen&&dev->status.lock&&!dev->event)
		{
			dev->event=1;
			if(dev->fe->notify&&!dev->fe->event)
			{
				flag=1;
				dev->fe->event=1;
				dev->signal=1;
				dev->fe->enx=NULL;
				if(!dev->etl)dev->ehd=dev->etl=dev->fe;
				else dev->etl=dev->etl->enx=dev->fe;
			}
		}
		pthread_spin_unlock(&dev->spin);
		break;
	}

	if(dev->signal||dev->sigcnt)
	{
		if(!dev->sigcnt++)curr=dev->start=times(NULL);
		else curr=times(NULL);

		if(flag||dev->conf.fast||dev->sigcnt>=2*SATIP_MAX_BURST||
			curr-dev->start>=dev->ticks)
		{
			dummy=write(dev->nfd,&dummy,sizeof(dummy));
			dev->sigcnt=0;
		}
	}
}

static void hwstream(void *id,SATIP_STREAM *stream)
{
	DEVICE *dev=(DEVICE *)id;
	uint64_t dummy=1;
	clock_t curr;
	int flag=0;

	dev->signal=0;

	if(UNLIKELY(!stream))
	{
		satip_util_filter_packets(dev->filter,NULL,0);

		if(dev->signal||dev->sigcnt)
		{
			dummy=write(dev->nfd,&dummy,sizeof(dummy));
			dev->sigcnt=0;
		}

		return;
	}

	switch(stream->fill)
	{
	case 1316:
		goto work;
	case 1128:
	case 940:
	case 752:
	case 564:
	case 376:
	case 188:
		flag=1;
work:		satip_util_filter_packets_cb(dev->filter,
			stream->data,stream->fill,nextpacket,dev);
		if(flag)satip_util_filter_packets(dev->filter,NULL,0);
		break;
	case 0:	flag=1;
		satip_util_filter_packets(dev->filter,NULL,0);
		break;
	}

	if(dev->signal||dev->sigcnt)
	{
		if(!dev->sigcnt++)curr=dev->start=times(NULL);
		else curr=times(NULL);

		if(flag||dev->conf.fast||dev->sigcnt>=2*SATIP_MAX_BURST||
			curr-dev->start>=dev->ticks)
		{
			dummy=write(dev->nfd,&dummy,sizeof(dummy));
			dev->sigcnt=0;
		}
	}
}

static void hwstatus(void *id,SATIP_STATUS *status)
{
	DEVICE *dev=(DEVICE *)id;
	int signal=0;
	uint64_t dummy=1;

	pthread_spin_lock(&dev->spin);
	dev->status=*status;
	if(dev->feopen&&dev->status.lock&&!dev->event)
	{
		dev->event=1;
		if(dev->fe->notify&&!dev->fe->event)
		{
			dev->fe->event=1;
			signal=1;
			dev->fe->enx=NULL;
			if(!dev->etl)dev->ehd=dev->etl=dev->fe;
			else dev->etl=dev->etl->enx=dev->fe;
		}
	}
	pthread_spin_unlock(&dev->spin);

	if(signal)dummy=write(dev->nfd,&dummy,sizeof(dummy));
}

static void enqueue(MSG *msg,CAMQUEUE *p)
{
	uint64_t dummy=1;

	pthread_mutex_lock(&p->mtx);
	if(p->fill==MAX_TUNERS)
	{
		free(msg);
		goto out;
	}
	p->queue[p->head++]=msg;
	if(p->head==MAX_TUNERS)p->head=0;
	p->fill++;
	dummy=write(p->efd,&dummy,sizeof(dummy));
out:	pthread_mutex_unlock(&p->mtx);
}

static MSG *dequeue(CAMQUEUE *p)
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
	if(p->tail==MAX_TUNERS)p->tail=0;
	p->fill--;
out:	pthread_mutex_unlock(&p->mtx);
	return m;
}

static void camcb(SATIP_HW_CAM_IO *msg,void *priv)
{
	MSG *m;

	switch(msg->type)
	{
	case SATIP_CAM_STATE:
		if(!(m=malloc(sizeof(MSG))))break;
		m->msg=*msg;
		enqueue(m,(CAMQUEUE *)priv);
		break;

	case SATIP_CAM_READ:
		if(!(m=malloc(sizeof(MSG)+msg->len)))break;
		m->msg=*msg;
		m->msg.data=m->data;
		memcpy(m->data,msg->data,msg->len);
		enqueue(m,(CAMQUEUE *)priv);
		break;
	}
}

static unsigned int satfreq(SATIP_TUNE *tune,int lnb)
{
	unsigned long long freq=0ULL;

	switch(lnb)
	{
	case SATIP_LNB_UNIV:
		if(tune->freq<10700000000ULL||tune->freq>12750000000ULL)break;
		if(tune->freq>=11700000000ULL)freq=tune->freq-10600000000ULL;
		else freq=tune->freq-9750000000ULL;
		break;

	case SATIP_LNB_DBS:
		if(tune->freq<12200000000ULL||tune->freq>12700000000ULL)break;
		freq=tune->freq-11250000000ULL;
		break;

	case SATIP_LNB_CMONO:
		if(tune->freq<3700000000ULL||tune->freq>4200000000ULL)break;
		freq=5150000000ULL-tune->freq;
		break;

	case SATIP_LNB_CMULT:
		if(tune->freq<3700000000ULL||tune->freq>4200000000ULL)break;
		if(tune->pol==SATIP_POL_V)freq=5150000000ULL-tune->freq;
		else freq=5750000000ULL-tune->freq;
		break;

	case SATIP_LNB_AUS:
		if(tune->freq<11700000000ULL||tune->freq>12750000000ULL)break;
		freq=tune->freq-10700000000ULL;
		break;
	}

	return (unsigned int)freq;
}

static int checkfreq(unsigned int satfreq,int lnb)
{
	switch(lnb)
	{
	case SATIP_LNB_UNIV:
		if(satfreq<950000000||satfreq>2150000000)return -1;
		return 0;
	case SATIP_LNB_DBS:
		if(satfreq<950000000||satfreq>1450000000)return -1;
		return 0;
	case SATIP_LNB_CMONO:
		if(satfreq<950000000||satfreq>1450000000)return -1;
		return 0;
	case SATIP_LNB_CMULT:
		if(satfreq<950000000||satfreq>2050000000)return -1;
		if(satfreq>1450000000&&satfreq<1550000000)return -1;
		return 0;
	case SATIP_LNB_AUS:
		if(satfreq<1000000000||satfreq>2050000000)return -1;
		return 0;
	default:return -1;
	}
}

static unsigned long long satipfreq(unsigned int satfreq,int lnb,int pol,int hl)
{
	unsigned long long freq=0ULL;

	switch(lnb)
	{
	case SATIP_LNB_UNIV:
		if(hl)freq=(unsigned long long)satfreq+10600000000ULL;
		else freq=(unsigned long long)satfreq+9750000000ULL;
		break;

	case SATIP_LNB_DBS:
		freq=(unsigned long long)satfreq+11250000000ULL;
		break;

	case SATIP_LNB_CMONO:
		freq=5150000000ULL-(unsigned long long)satfreq;
		break;

	case SATIP_LNB_CMULT:
		if(pol==SATIP_POL_V)
			freq=5150000000ULL-(unsigned long long)satfreq;
		else freq=5750000000ULL-(unsigned long long)satfreq;
		break;

	case SATIP_LNB_AUS:
		if(satfreq<1000000000||satfreq>2050000000)break;
		freq=(unsigned long long)satfreq+10700000000ULL;
		break;
	}

	return freq;
}

static int fec(int fec)
{
	switch(fec)
	{
	case SATIP_FEC_12:
		return FEC_1_2;
	case SATIP_FEC_23:
		return FEC_2_3;
	case SATIP_FEC_34:
		return FEC_3_4;
	case SATIP_FEC_35:
		return FEC_3_5;
	case SATIP_FEC_45:
		return FEC_4_5;
	case SATIP_FEC_56:
		return FEC_5_6;
	case SATIP_FEC_78:
		return FEC_7_8;
	case SATIP_FEC_89:
		return FEC_8_9;
	case SATIP_FEC_910:
		return FEC_9_10;
	default:return FEC_AUTO;
	}
}

static int satipfec(int fec)
{
	switch(fec)
	{
	case FEC_1_2:
		return SATIP_FEC_12;
	case FEC_2_3:
		return SATIP_FEC_23;
	case FEC_3_4:
		return SATIP_FEC_34;
	case FEC_3_5:
		return SATIP_FEC_35;
	case FEC_4_5:
		return SATIP_FEC_45;
	case FEC_5_6:
		return SATIP_FEC_56;
	case FEC_7_8:
		return SATIP_FEC_78;
	case FEC_8_9:
		return SATIP_FEC_89;
	case FEC_9_10:
		return SATIP_FEC_910;
	default:return SATIP_FEC_AUTO;
	}
}

static int bw(int bw)
{
	switch(bw)
	{
	case SATIP_BW_1712:
		return BANDWIDTH_1_712_MHZ;
	case SATIP_BW_5:
		return BANDWIDTH_5_MHZ;
	case SATIP_BW_6:
		return BANDWIDTH_6_MHZ;
	case SATIP_BW_7:
		return BANDWIDTH_7_MHZ;
	case SATIP_BW_8:
		return BANDWIDTH_8_MHZ;
	case SATIP_BW_10:
		return BANDWIDTH_10_MHZ;
	default:return BANDWIDTH_AUTO;
	}
}

static int satipbw(int bw)
{
	switch(bw)
	{
	case BANDWIDTH_1_712_MHZ:
		return SATIP_BW_1712;
	case BANDWIDTH_5_MHZ:
		return SATIP_BW_5;
	case BANDWIDTH_6_MHZ:
		return SATIP_BW_6;
	case BANDWIDTH_7_MHZ:
		return SATIP_BW_7;
	case BANDWIDTH_8_MHZ:
		return SATIP_BW_8;
	case BANDWIDTH_10_MHZ:
		return SATIP_BW_10;
	default:return SATIP_BW_AUTO;
	}
}

static int mtype(int mtype)
{
	switch(mtype)
	{
	case SATIP_QPSK:
		return QPSK;
	case SATIP_8PSK:
		return PSK_8;
	case SATIP_16APSK:
		return APSK_16;
	case SATIP_32APSK:
		return APSK_32;
	case SATIP_16Q:
		return QAM_16;
	case SATIP_32Q:
		return QAM_32;
	case SATIP_64Q:
		return QAM_64;
	case SATIP_128Q:
		return QAM_128;
	case SATIP_256Q:
		return QAM_256;
	default:return QAM_AUTO;
	}
}

static int satipmtype(int mtype)
{
	switch(mtype)
	{
	case QPSK:
		return SATIP_QPSK;
	case PSK_8:
		return SATIP_8PSK;
	case APSK_16:
		return SATIP_16APSK;
	case APSK_32:
		return SATIP_32APSK;
	case QAM_16:
		return SATIP_16Q;
	case QAM_32:
		return SATIP_32Q;
	case QAM_64:
		return SATIP_64Q;
	case QAM_128:
		return SATIP_128Q;
	case QAM_256:
		return SATIP_256Q;
	default:return SATIP_AUTOQ;
	}
}

static int tmode(int tmode)
{
	switch(tmode)
	{
	case SATIP_TMOD_1K:
		return TRANSMISSION_MODE_1K;
	case SATIP_TMOD_2K:
		return TRANSMISSION_MODE_2K;
	case SATIP_TMOD_4K:
		return TRANSMISSION_MODE_4K;
	case SATIP_TMOD_8K:
		return TRANSMISSION_MODE_8K;
	case SATIP_TMOD_16K:
		return TRANSMISSION_MODE_16K;
	case SATIP_TMOD_32K:
		return TRANSMISSION_MODE_32K;
	default:return TRANSMISSION_MODE_AUTO;
	}
}

static int satiptmode(int tmode)
{
	switch(tmode)
	{
	case TRANSMISSION_MODE_1K:
		return SATIP_TMOD_1K;
	case TRANSMISSION_MODE_2K:
		return SATIP_TMOD_2K;
	case TRANSMISSION_MODE_4K:
		return SATIP_TMOD_4K;
	case TRANSMISSION_MODE_8K:
		return SATIP_TMOD_8K;
	case TRANSMISSION_MODE_16K:
		return SATIP_TMOD_16K;
	case TRANSMISSION_MODE_32K:
		return SATIP_TMOD_32K;
	default:return SATIP_TMOD_AUTO;
	}
}

static int gi(int gi)
{
	switch(gi)
	{
	case SATIP_GI_14:
		return GUARD_INTERVAL_1_4;
	case SATIP_GI_18:
		return GUARD_INTERVAL_1_8;
	case SATIP_GI_116:
		return GUARD_INTERVAL_1_16;
	case SATIP_GI_132:
		return GUARD_INTERVAL_1_32;
	case SATIP_GI_1128:
		return GUARD_INTERVAL_1_128;
	case SATIP_GI_19128:
		return GUARD_INTERVAL_19_128;
	case SATIP_GI_19256:
		return GUARD_INTERVAL_19_256;
	default: return GUARD_INTERVAL_AUTO;
	}
}

static int satipgi(int gi)
{
	switch(gi)
	{
	case GUARD_INTERVAL_1_4:
		return SATIP_GI_14;
	case GUARD_INTERVAL_1_8:
		return SATIP_GI_18;
	case GUARD_INTERVAL_1_16:
		return SATIP_GI_116;
	case GUARD_INTERVAL_1_32:
		return SATIP_GI_132;
	case GUARD_INTERVAL_1_128:
		return SATIP_GI_1128;
	case GUARD_INTERVAL_19_128:
		return SATIP_GI_19128;
	case GUARD_INTERVAL_19_256:
		return SATIP_GI_19256;
	default:return SATIP_GI_AUTO;
	}
}

static int pidcmp(const void *p1,const void *p2)
{
	short v1=*((short *)p1);
	short v2=*((short *)p2);
        return v1-v2;
}

static void pidmerge(SATIP_PIDS *result,SATIP_PIDS *set1,SATIP_PIDS *set2)
{
	int i;
	int j;
	SATIP_PIDS wrk;

	if(set1->numpids==SATIP_ALLPIDS||set2->numpids==SATIP_ALLPIDS)
	{
		result->numpids=SATIP_ALLPIDS;
		return;
	}

	if(set1->numpids==SATIP_NOPIDS&&set2->numpids==SATIP_NOPIDS)
	{
		result->numpids=SATIP_NOPIDS;
		return;
	}

	if(set1->numpids==SATIP_NOPIDS)
	{
		if(result!=set2)*result=*set2;
		return;
	}

	if(set2->numpids==SATIP_NOPIDS)
	{
		if(result!=set1)*result=*set1;
		return;
	}

	if(set1->numpids>1)qsort(set1->pids,set1->numpids,sizeof(short),pidcmp);
	if(set2->numpids>1)qsort(set2->pids,set2->numpids,sizeof(short),pidcmp);

	for(i=0,j=0;i<set1->numpids||j<set2->numpids;)
	{
		if(i<set1->numpids&&j<set2->numpids)
		{
			if(wrk.numpids==SATIP_MAX_PIDS)
			{
				result->numpids=SATIP_ALLPIDS;
				return;
			}

			if(set1->pids[i]<set2->pids[j])
				wrk.pids[wrk.numpids++]=set1->pids[i++];
			else if(set2->pids[j]<set1->pids[i])
				wrk.pids[wrk.numpids++]=set2->pids[j++];
			else
			{
				wrk.pids[wrk.numpids++]=set1->pids[i++];
				j++;
			}
		}
		else if(i<set1->numpids)
		{
			if(wrk.numpids==SATIP_MAX_PIDS)
			{
				result->numpids=SATIP_ALLPIDS;
				return;
			}
			wrk.pids[wrk.numpids++]=set1->pids[i++];
		}
		else
		{
			if(wrk.numpids==SATIP_MAX_PIDS)
			{
				result->numpids=SATIP_ALLPIDS;
				return;
			}
			wrk.pids[wrk.numpids++]=set2->pids[j++];
		}
	}

	*result=wrk;
}

static void update_set(DEVICE *dev,int notify)
{
	int i;
	int total;
	PID *p;
	uint64_t dummy=1;
	short plist[8192];

	if(notify&&dev->plug.notify_pids)
	{
		for(total=0,p=dev->pids;p;p=p->next)if(p->pid<0x2000)
			plist[total++]=p->pid;
		dev->plug.notify_pids(dev->plug.ctx,plist,total);
	}

	for(i=0,dev->set.numpids=SATIP_NOPIDS,p=dev->pids;p;p=p->next)
	{
		if((p->pid&0x3fff)==0x2000)
		{
			dev->set.numpids=SATIP_ALLPIDS;
			i=0;
			break;
		}
		else if(i<SATIP_MAX_PIDS)
		{
			dev->set.pids[i++]=(p->pid&0x1fff);
		}
	}
	if(i)dev->set.numpids=i;

	if(dev->plug.notify_pids)
	{
		i=dev->set.prognum;
		pidmerge(&dev->set,&dev->set,&dev->plug.set);
		dev->set.prognum=i;
	}

	if(dev->conf.maxpids&&dev->set.numpids>dev->conf.maxpids)
		dev->set.numpids=SATIP_ALLPIDS;

	dummy=write(dev->ffd,&dummy,sizeof(dummy));
}

static void addpid(DEVICE *dev,DMXDTA *dmx,int pid,int ftype)
{
	PID **p;
	DMX **d;
	void *e;
	int flag=0;

	if(ftype>2)pid|=0x4000;

	for(p=&dev->pids;*p;p=&(*p)->next)if((*p)->pid==pid)break;
	if(!*p)
	{
		if(!(e=malloc(sizeof(PID))))goto out;
		((PID *)e)->next=NULL;
		if(!(((PID *)e)->dmx=malloc(sizeof(DMX))))
		{
			free(e);
			goto out;
		}
		((PID *)e)->dmx->next=NULL;
		((PID *)e)->dmx->dmx=dmx;
		((PID *)e)->pid=pid;
		*p=e;
		flag=1;
		goto out;
	}
	for(d=&(*p)->dmx;*d;d=&(*d)->next)if((*d)->dmx==dmx)goto out;
	if(!(e=malloc(sizeof(DMX))))goto out;
	((DMX *)e)->next=NULL;
	((DMX *)e)->dmx=dmx;
	*d=e;

out:	if(flag)update_set(dev,1);
}

static void delpid(DEVICE *dev,DMXDTA *dmx,int pid,int ftype)
{
	PID **p;
	DMX **d;
	void *e;
	int flag=0;

	if(ftype>2)pid|=0x4000;

	for(p=&dev->pids;*p;)if((*p)->pid==pid)
	{
		for(d=&(*p)->dmx;*d;)
		{
			if((*d)->dmx==dmx)
			{
				e=*d;
				*d=(*d)->next;
				free(e);
			}
			else d=&(*d)->next;
		}
		if(!(*p)->dmx)
		{
			e=*p;
			*p=(*p)->next;
			free(e);
			flag=1;
		}
		else p=&(*p)->next;
		break;
	}
	else p=&(*p)->next;

	if(flag)update_set(dev,1);
}

static void clrpid(DEVICE *dev,DMXDTA *dmx)
{
	PID **p;
	DMX **d;
	void *e;
	int flag=0;

	for(p=&dev->pids;*p;)
	{
		for(d=&(*p)->dmx;*d;)
		{
			if((*d)->dmx==dmx)
			{
				e=*d;
				*d=(*d)->next;
				free(e);
			}
			else d=&(*d)->next;
		}
		if(!(*p)->dmx)
		{
			e=*p;
			*p=(*p)->next;
			free(e);
			flag=1;
		}
		else p=&(*p)->next;
	}

	if(flag)update_set(dev,1);
}

static void freepid(DEVICE *dev)
{
	void *e;
	int flag=0;

	while(dev->pids)
	{
		while(dev->pids->dmx)
		{
			e=dev->pids->dmx;
			dev->pids->dmx=dev->pids->dmx->next;
			free(e);
		}
		e=dev->pids;
		dev->pids=dev->pids->next;
		free(e);
		flag=1;
	}

	if(flag)update_set(dev,1);
}

static void add_timeout(DEVICE *dev)
{
	struct itimerspec itm;

	pthread_mutex_lock(&dev->tmtx);

	if(dev->timeouts++)goto out;

	itm.it_value.tv_sec=0;
	itm.it_value.tv_nsec=250000000;
	itm.it_interval.tv_sec=0;
	itm.it_interval.tv_nsec=250000000;

	timerfd_settime(dev->tfd,0,&itm,NULL);

out:	pthread_mutex_unlock(&dev->tmtx);
}

static void del_timeout(DEVICE *dev)
{
	struct itimerspec itm;

	pthread_mutex_lock(&dev->tmtx);

	if(--(dev->timeouts))goto out;

	itm.it_value.tv_sec=0;
	itm.it_value.tv_nsec=0;
	itm.it_interval.tv_sec=0;
	itm.it_interval.tv_nsec=0;

	timerfd_settime(dev->tfd,0,&itm,NULL);

out:	pthread_mutex_unlock(&dev->tmtx);
}

static int setprop(DEVICE *dev,struct dtv_property *p)
{
	int pol;
	uint64_t dummy=1;

	switch(p->cmd)
	{
	case DTV_CLEAR:
		pol=dev->tune.pol;
		satip_util_init_tune(&dev->tune);
		dev->tune.msys=dev->conf.mode;
		switch(dev->conf.mode)
		{
		case SATIP_DVBS:
		case SATIP_DVBS2:
			dev->freq=950000000;
			dev->tune.mtype=SATIP_QPSK;
			dev->tune.ro=SATIP_ROFF_035;
			dev->tune.pol=pol;
			break;
		default:dev->freq=54000000;
			break;
		}
		break;

	case DTV_TUNE:
		dev->enabled=1;
		dev->tunecnt++;
		dummy=write(dev->ffd,&dummy,sizeof(dummy));
		break;

	case DTV_FREQUENCY:
		switch(dev->conf.mode)
		{
		case SATIP_DVBS:
		case SATIP_DVBS2:
			if(checkfreq(p->u.data*1000,dev->conf.lnb))
				return EINVAL;
			dev->freq=p->u.data*1000;
			break;
		default:dev->freq=p->u.data;
			break;
		}
		break;

	case DTV_MODULATION:
		dev->tune.mtype=satipmtype(p->u.data);
		break;

	case DTV_BANDWIDTH_HZ:
		switch(p->u.data)
		{
		case 1712000:
			dev->tune.bw=SATIP_BW_1712;
			break;
		case 5000000:
			dev->tune.bw=SATIP_BW_5;
			break;
		case 6000000:
			dev->tune.bw=SATIP_BW_6;
			break;
		case 7000000:
			dev->tune.bw=SATIP_BW_7;
			break;
		case 8000000:
			dev->tune.bw=SATIP_BW_8;
			break;
		case 10000000:
			dev->tune.bw=SATIP_BW_10;
			break;
		default:dev->tune.bw=SATIP_BW_AUTO;
			break;
		}
		break;

	case DTV_INVERSION:
		switch(p->u.data)
		{
		case INVERSION_OFF:
			dev->tune.specinv=SATIP_SPI_OFF;
			break;
		case INVERSION_ON:
			dev->tune.specinv=SATIP_SPI_ON;
			break;
		default:dev->tune.specinv=SATIP_SPI_AUTO;
			break;
		}
		break;

	case DTV_SYMBOL_RATE:
		dev->tune.sr=p->u.data;
		break;

	case DTV_INNER_FEC:
	case DTV_CODE_RATE_HP:
		dev->tune.fec=satipfec(p->u.data);
		break;

	case DTV_PILOT:
		switch(p->u.data)
		{
		case PILOT_OFF:
			dev->tune.plts=SATIP_PLTS_OFF;
			break;
		case PILOT_ON:
			dev->tune.plts=SATIP_PLTS_ON;
			break;
		default:dev->tune.plts=SATIP_PLTS_AUTO;
			break;
		}
		break;

	case DTV_ROLLOFF:
		switch(p->u.data)
		{
		case ROLLOFF_35:
			dev->tune.ro=SATIP_ROFF_035;
			break;
		case ROLLOFF_25:
			dev->tune.ro=SATIP_ROFF_025;
			break;
		case ROLLOFF_20:
			dev->tune.ro=SATIP_ROFF_020;
			break;
		default:dev->tune.ro=SATIP_ROFF_AUTO;
			break;
		}
		break;

	case DTV_DELIVERY_SYSTEM:
		switch(p->u.data)
		{
		case SYS_DVBS:
			if(dev->conf.mode!=SATIP_DVBS&&
				dev->conf.mode!=SATIP_DVBS2)return EINVAL;
			if(dev->conf.forcemode)
				dev->tune.msys=dev->conf.mode;
			else
				dev->tune.msys=SATIP_DVBS;
			break;
		case SYS_DVBS2:
			if(dev->conf.mode!=SATIP_DVBS&&
				dev->conf.mode!=SATIP_DVBS2)return EINVAL;
			if(dev->conf.forcemode)
				dev->tune.msys=dev->conf.mode;
			else
				dev->tune.msys=SATIP_DVBS2;
			break;
		case SYS_DVBT:
			if(dev->conf.mode!=SATIP_DVBT&&
				dev->conf.mode!=SATIP_DVBT2)return EINVAL;
			if(dev->conf.forcemode)
				dev->tune.msys=dev->conf.mode;
			else
				dev->tune.msys=SATIP_DVBT;
			break;
		case SYS_DVBT2:
			if(dev->conf.mode!=SATIP_DVBT&&
				dev->conf.mode!=SATIP_DVBT2)return EINVAL;
			if(dev->conf.forcemode)
				dev->tune.msys=dev->conf.mode;
			else
				dev->tune.msys=SATIP_DVBT2;
			break;
		case SYS_DVBC_ANNEX_A:
			if(dev->conf.mode!=SATIP_DVBC&&
				dev->conf.mode!=SATIP_DVBC2)return EINVAL;
			if(dev->conf.forcemode)
				dev->tune.msys=dev->conf.mode;
			else
				dev->tune.msys=SATIP_DVBC;
			break;
		default:dev->tune.msys=dev->conf.mode;
			break;
		}
		break;

	case DTV_VOLTAGE:
		switch(p->u.data)
		{
		case SEC_VOLTAGE_18:
			dev->tune.pol=SATIP_POL_H;
			break;
		case SEC_VOLTAGE_OFF:
			if(dev->enabled&&dev->conf.lnbpower)
			{
				dev->enabled=0;
				dummy=write(dev->ffd,&dummy,sizeof(dummy));
			}
		case SEC_VOLTAGE_13:
		default:dev->tune.pol=SATIP_POL_V;
			break;
		}
		break;

	case DTV_TONE:
		switch(p->u.data)
		{
		case SEC_TONE_ON:
			dev->hl=1;
			break;
		case SEC_TONE_OFF:
		default:dev->hl=0;
			break;
		}
		break;

	case DTV_GUARD_INTERVAL:
		dev->tune.gi=satipgi(p->u.data);
		break;

	case DTV_TRANSMISSION_MODE:
		dev->tune.tmode=satiptmode(p->u.data);
		break;

	case DTV_STREAM_ID:
	case DTV_DVBT2_PLP_ID_LEGACY:
		dev->tune.plp=p->u.data>=0?p->u.data:SATIP_UNDEF;
		break;

	case DTV_HIERARCHY:
	case DTV_LNA:
	case DTV_CODE_RATE_LP:
		break;

	default:p->result=EOPNOTSUPP;
	}

	p->result=0;
	return 0;
}

static int getprop(DEVICE *dev,struct dtv_property *p)
{
	switch(p->cmd)
	{
	case DTV_ENUM_DELSYS:
		switch(dev->conf.mode)
		{
		case SATIP_DVBS:
		case SATIP_DVBS2:
			p->u.buffer.len=2;
			p->u.buffer.data[0]=SYS_DVBS;
			p->u.buffer.data[1]=SYS_DVBS2;
			break;
		case SATIP_DVBT:
		case SATIP_DVBT2:
			p->u.buffer.len=2;
			p->u.buffer.data[0]=SYS_DVBT;
			p->u.buffer.data[1]=SYS_DVBT2;
			break;
		case SATIP_DVBC:
		case SATIP_DVBC2:
			p->u.buffer.len=1;
			p->u.buffer.data[0]=SYS_DVBC_ANNEX_A;
			break;
		}
		break;

	case DTV_FREQUENCY:
		switch(dev->conf.mode)
		{
		case SATIP_DVBS:
		case SATIP_DVBS2:
			p->u.data=dev->freq/1000;
			break;
		default:p->u.data=dev->freq;
			break;
		}
		break;

	case DTV_MODULATION:
		p->u.data=mtype(dev->tune.mtype);
		break;

	case DTV_BANDWIDTH_HZ:
		switch(dev->tune.bw)
		{
		case SATIP_BW_1712:
			p->u.data=1712000;
			break;
		case SATIP_BW_5:
			p->u.data=5000000;
			break;
		case SATIP_BW_6:
			p->u.data=6000000;
			break;
		case SATIP_BW_7:
			p->u.data=7000000;
			break;
		case SATIP_BW_8:
			p->u.data=8000000;
			break;
		case SATIP_BW_10:
			p->u.data=10000000;
			break;
		default:p->u.data=0;
			break;
		}
		break;

	case DTV_INVERSION:
		switch(dev->tune.specinv)
		{
		case SATIP_SPI_OFF:
			p->u.data=INVERSION_OFF;
			break;
		case SATIP_SPI_ON:
			p->u.data=INVERSION_ON;
			break;
		default:p->u.data=INVERSION_AUTO;
			break;
		}
		break;

	case DTV_SYMBOL_RATE:
		p->u.data=dev->tune.sr;
		break;

	case DTV_INNER_FEC:
	case DTV_CODE_RATE_HP:
	case DTV_CODE_RATE_LP:
		p->u.data=fec(dev->tune.fec);
		break;

	case DTV_PILOT:
		switch(dev->tune.plts)
		{
		case SATIP_PLTS_OFF:
			p->u.data=PILOT_OFF;
			break;
		case SATIP_PLTS_ON:
			p->u.data=PILOT_ON;
			break;
		default:p->u.data=PILOT_AUTO;
		}
		break;

	case DTV_ROLLOFF:
		switch(dev->tune.ro)
		{
		case SATIP_ROFF_035:
			p->u.data=ROLLOFF_35;
			break;
		case SATIP_ROFF_025:
			p->u.data=ROLLOFF_25;
			break;
		case SATIP_ROFF_020:
			p->u.data=ROLLOFF_20;
			break;
		default:p->u.data=ROLLOFF_AUTO;
			break;
		}
		break;

	case DTV_DELIVERY_SYSTEM:
		switch(dev->tune.msys)
		{
		case SATIP_DVBS:
			p->u.data=SYS_DVBS;
			break;
		case SATIP_DVBS2:
			p->u.data=SYS_DVBS2;
			break;
		case SATIP_DVBT:
			p->u.data=SYS_DVBT;
			break;
		case SATIP_DVBT2:
			p->u.data=SYS_DVBT2;
			break;
		case SATIP_DVBC:
		case SATIP_DVBC2:
			p->u.data=SYS_DVBC_ANNEX_A;
			break;
		}
		break;

	case DTV_VOLTAGE:
		switch(dev->tune.pol)
		{
		case SATIP_POL_H:
		case SATIP_POL_L:
			p->u.data=SEC_VOLTAGE_18;
			break;
		case SATIP_POL_V:
		case SATIP_POL_R:
			p->u.data=SEC_VOLTAGE_13;
			break;
		}
		break;

	case DTV_TONE:
		p->u.data=dev->hl?SEC_TONE_ON:SEC_TONE_OFF;
		break;

	case DTV_API_VERSION:
		p->u.data=0x050a;
		break;

	case DTV_GUARD_INTERVAL:
		p->u.data=gi(dev->tune.gi);
		break;

	case DTV_TRANSMISSION_MODE:
		p->u.data=tmode(dev->tune.tmode);
		break;

	case DTV_HIERARCHY:
		p->u.data=HIERARCHY_AUTO;
		break;

	case DTV_STREAM_ID:
	case DTV_DVBT2_PLP_ID_LEGACY:
		p->u.data=dev->tune.plp;
		break;

	case DTV_LNA:
		p->u.data=LNA_AUTO;
		break;

	default:memset(&p->u,0,sizeof(p->u));
		p->result=EOPNOTSUPP;
		break;
	}

	p->result=0;
	return 0;
}

static void dvr_notify(fuse_req_t req,void *data)
{
	DEVICE *dev=(DEVICE *)data;
	uint64_t dummy=1;

	pthread_spin_lock(&dev->spin);

	if(dev->dvropen&&dev->s->notify&&!dev->s->event)
	{
		dev->s->event=1;
		dev->s->enx=NULL;
		if(!dev->etl)dev->ehd=dev->etl=dev->s;
		else dev->etl=dev->etl->enx=dev->s;
	}

	pthread_spin_unlock(&dev->spin);

	dummy=write(dev->nfd,&dummy,sizeof(dummy));
}

static void dvr_post(void *userdata)
{
	DEVICE *dev=(DEVICE *)userdata;
	char devpath[PATH_MAX];
	int unused __attribute__ ((unused));
	uint64_t dummy=1;

	int owner=dev->conf.owner?dev->conf.owner:-1;
	int group=dev->conf.group?dev->conf.group:-1;
	int perms=dev->conf.perms?dev->conf.perms:0666;

	sprintf(devpath,"/dev/dvb/adapter%d/dvr0",dev->conf.adapter);
	usleep(dev->conf.udevwait*1000);
	setfsuid(0);
	unused=chown(devpath,owner,group);
	unused=chmod(devpath,perms);
	setfsuid(getuid());
	pthread_spin_lock(&dev->spin);
	dev->dvrok=1;
	pthread_spin_unlock(&dev->spin);
	dummy=write(dev->ifd,&dummy,sizeof(dummy));
}

static void dvr_open(fuse_req_t req,struct fuse_file_info *fi)
{
	DEVICE *dev=fuse_req_userdata(req);
	STREAM *s;
	unsigned char *ptr;
	int err=EINVAL;

	if(!dev)goto err;
	pthread_rwlock_wrlock(&dev->mtx);

	if(!dev->running||dev->dvropen)FAIL(EBUSY);
	if(!(s=malloc(sizeof(STREAM))))FAIL(EMFILE);

	if(pthread_mutex_init(&s->mtx,NULL))
	{
		free(s);
		FAIL(ENOMEM);
	}

	memset(s,0,sizeof(STREAM));
	s->flags=fi->flags;
	s->dev=dev;
	dev->s=s;

	if(dev->dvrbufsize!=dev->conf.bufsize&&!dev->dvrrun)
		if((ptr=realloc(dev->dvrbfr,dev->conf.bufsize)))
	{
		dev->dvrbfr=ptr;
		dev->dvrbufsize=dev->conf.bufsize;
		dev->dvrthres=(dev->dvrbufsize>>1)+(dev->dvrbufsize>>2);
		dev->dvrhead=0;
		dev->dvrtail=0;
		dev->dvrfill=0;
		dev->plughead=0;
		dev->plugfill=0;
	}

	if(dev->dvrrun&&dev->plug.strinit)
		dev->plug.strinit(dev->plug.ctx,&dev->plugdta);

	pthread_spin_lock(&dev->spin);
	dev->dvropen=1;
	pthread_spin_unlock(&dev->spin);

	s->next=dev->list;
	dev->list=s;

	fi->direct_io=1;
	fi->keep_cache=0;
	fi->nonseekable=1;
	fi->fh=(uint64_t)s;

	pthread_rwlock_unlock(&dev->mtx);
	fuse_reply_open(req,fi);
	return;

fail:	pthread_rwlock_unlock(&dev->mtx);
err:	fuse_reply_err(req,err);
}

static void dvr_read(fuse_req_t req,size_t size,off_t off,
	struct fuse_file_info *fi)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	int err=0;
	ssize_t len=0;

	pthread_rwlock_rdlock(&dev->mtx);
	pthread_mutex_lock(&s->mtx);

	if(!dev->running)FAIL(EBUSY);
	if((s->flags&O_ACCMODE)==O_WRONLY)FAIL(EPERM);
	if(s->req)FAIL(EAGAIN);

	if(!dev->dvrrun)goto nonblk;

	pthread_spin_lock(&dev->spin);
	len=(dev->dvrfill>size?size:dev->dvrfill);
	if(dev->dvrtail>=dev->dvrhead)
		if(len>dev->dvrbufsize-dev->dvrtail)
			len=dev->dvrbufsize-dev->dvrtail;
	pthread_spin_unlock(&dev->spin);

	if(!len)
	{
nonblk:		if(s->flags&O_NONBLOCK)FAIL(EAGAIN);
		else
		{
			s->req=req;
			s->type=1;
			s->size=size;
			pthread_spin_lock(&dev->spin);
			s->notify++;
			pthread_spin_unlock(&dev->spin);
			fuse_req_interrupt_func(req,dvr_notify,dev);
			goto fail;
		}
	}

	fuse_reply_buf(req,dev->dvrbfr+dev->dvrtail,len);

	pthread_spin_lock(&dev->spin);
	if((dev->dvrtail+=len)==dev->dvrbufsize)dev->dvrtail=0;
	dev->dvrfill-=len;
	pthread_spin_unlock(&dev->spin);

fail:	pthread_mutex_unlock(&s->mtx);
	pthread_rwlock_unlock(&dev->mtx);
	if(err)fuse_reply_err(req,err);
}

static void dvr_write(fuse_req_t req,const char *buf,size_t size,off_t off,
	struct fuse_file_info *fi)
{
	STREAM *s=(STREAM *)fi->fh;

	if((s->flags&O_ACCMODE)==O_RDONLY)fuse_reply_err(req,EPERM);
	else fuse_reply_err(req,EOPNOTSUPP);
}

static void dvr_flush(fuse_req_t req,struct fuse_file_info *fi)
{
	fuse_reply_err(req,EOPNOTSUPP);
}

static void dvr_release(fuse_req_t req,struct fuse_file_info *fi)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	STREAM **e;
	unsigned char *ptr;
	int err=0;

	pthread_rwlock_wrlock(&dev->mtx);

	if(!dev->running)FAIL(EBUSY);

	pthread_spin_lock(&dev->spin);

	dev->dvropen=0;

	while(dev->plugwork)
	{
		pthread_spin_unlock(&dev->spin);
		usleep(1000);
		pthread_spin_lock(&dev->spin);
	}

	if(dev->dvrrun&&dev->plug.strexit)
		dev->plug.strexit(dev->plug.ctx,dev->plugdta);

	dev->s=NULL;
	dev->dvrfill=0;
	dev->dvrhead=0;
	dev->dvrtail=0;
	dev->plughead=0;
	dev->plugfill=0;

	pthread_spin_unlock(&dev->spin);

	pthread_mutex_destroy(&s->mtx);

	if(s->ph)fuse_pollhandle_destroy(s->ph);

	if(s->req)fuse_reply_err(s->req,EBADF);

	if(dev->dvrbufsize!=dev->conf.bufsize&&!dev->dvrrun)
		if((ptr=realloc(dev->dvrbfr,dev->conf.bufsize)))
	{
		dev->dvrbfr=ptr;
		dev->dvrbufsize=dev->conf.bufsize;
		dev->dvrthres=(dev->dvrbufsize>>1)+(dev->dvrbufsize>>2);
	}

	for(e=&dev->ehd;*e;e=&(*e)->enx)if(*e==s)
	{
		*e=s->enx;
		if(!dev->ehd)dev->etl=NULL;
		break;
	}

	for(e=&dev->list;*e;e=&(*e)->next)if(*e==s)
	{
		*e=s->next;
		free(s);
		break;
	}

fail:	pthread_rwlock_unlock(&dev->mtx);
	fuse_reply_err(req,err);
}

static void dvr_fsync(fuse_req_t req,int datasync,struct fuse_file_info *fi)
{
	fuse_reply_err(req,EOPNOTSUPP);
}

static void dvr_ioctl(fuse_req_t req,int cmd,void *arg,
	struct fuse_file_info *fi,unsigned flags,const void *in_buf,
	size_t in_bufsz,size_t out_bufsz)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	unsigned char *ptr;
	int err=EINVAL;

	pthread_rwlock_wrlock(&dev->mtx);

	if(!dev->running)FAIL(EBUSY);
	if(flags&FUSE_IOCTL_COMPAT)FAIL(ENOSYS);

	switch(cmd)
	{
	case DMX_SET_BUFFER_SIZE:
		if(dev->dvrrun)FAIL(EBUSY);
		if((long)arg<8192||(long)arg>1073741824)FAIL(EINVAL);
		err=(long)arg/188;
		err+=1;
		err*=188;
		if(err<dev->conf.bufsize)err=dev->conf.bufsize;
		else
		{
			err+=SATIP_MAX_BURST*7*188-1;
			err/=SATIP_MAX_BURST*7*188;
			err*=SATIP_MAX_BURST*7*188;
		}
		if(err!=dev->dvrbufsize)
		{
			if(!(ptr=realloc(dev->dvrbfr,err)))FAIL(ENOMEM);
			dev->dvrbfr=ptr;
			dev->dvrbufsize=err;
			dev->dvrthres=(dev->dvrbufsize>>1)+(dev->dvrbufsize>>2);
		}
		dev->dvrhead=0;
		dev->dvrtail=0;
		dev->dvrfill=0;
		dev->plughead=0;
		dev->plugfill=0;
		pthread_rwlock_unlock(&dev->mtx);
		fuse_reply_ioctl(req,0,NULL,0);
		return;
	}

fail:	pthread_rwlock_unlock(&dev->mtx);
	fuse_reply_err(req,err);
}

static void dvr_poll(fuse_req_t req,struct fuse_file_info *fi,
	struct fuse_pollhandle *ph)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	struct fuse_pollhandle *pd=NULL;
	int err=0;
	int revents=0;

	pthread_rwlock_rdlock(&dev->mtx);
	pthread_mutex_lock(&s->mtx);

	if(!dev->running)FAIL(EBUSY);

	pthread_spin_lock(&dev->spin);

	if(dev->dvrrun&&dev->dvrfill)revents=POLLIN;

	if(ph)
	{
		if(revents)
		{
			if(s->ph)
			{
				pd=s->ph;
				s->ph=NULL;
				s->notify--;
			}
		}
		else
		{
			if(s->ph)pd=s->ph;
			else s->notify++;
			s->ph=ph;
		}
	}
	else if(s->ph&&revents)
	{
		ph=s->ph;
		s->ph=NULL;
		s->notify--;
	}

	pthread_spin_unlock(&dev->spin);

fail:	pthread_mutex_unlock(&s->mtx);
	pthread_rwlock_unlock(&dev->mtx);
	if(err)fuse_reply_err(req,err);
	else
	{
		if(pd)fuse_pollhandle_destroy(pd);
		fuse_reply_poll(req,revents);
		if(revents&&ph)
		{
			fuse_lowlevel_notify_poll(ph);
			fuse_pollhandle_destroy(ph);
		}
	}
}

static const struct cuse_lowlevel_ops dvr_ops=
{
	.init_done=dvr_post,
	.open=dvr_open,
	.read=dvr_read,
	.write=dvr_write,
	.flush=dvr_flush,
	.release=dvr_release,
	.fsync=dvr_fsync,
	.ioctl=dvr_ioctl,
	.poll=dvr_poll,
};

static void *dvrworker(void *data)
{
	struct cuse_info ci;
	DEVICE *dev=(DEVICE *)data;
	char devpath[PATH_MAX+9];
	const char *devarg[1]={devpath};
	int unused;
	int dummy_argc=1;
	char *dummy_argv[66]={""};
	struct fuse_args args=FUSE_ARGS_INIT(dummy_argc,dummy_argv);
	uint64_t dummy=1;
	sigset_t set;

	sigfillset(&set);
	sigdelset(&set,SIGQUIT);
	pthread_sigmask(SIG_SETMASK,&set,NULL);

	pthread_setname_np(pthread_self(),"loopd dvrworker");

	if(fuse_opt_parse(&args,NULL,dummy_opts,dummy_args))goto out;
	if(fuse_opt_add_arg(&args, "-f"))goto out;

	sprintf(devpath,"DEVNAME=dvb/adapter%d/dvr0",dev->conf.adapter);
	memset(&ci,0,sizeof(ci));
	ci.dev_major=dev->conf.major;
	ci.dev_minor=dev->conf.minbase+2;
	ci.dev_info_argc=1;
	ci.dev_info_argv=devarg;
	ci.flags=CUSE_UNRESTRICTED_IOCTL;

	setfsuid(0);
	if(!(dev->dvrse=cuse_lowlevel_setup(args.argc,args.argv,&ci,&dvr_ops,
		&unused,data)))
	{
		setfsuid(getuid());
		goto out;
	}
	setfsuid(getuid());
	fuse_session_loop_mt(dev->dvrse);
	fuse_session_destroy(dev->dvrse);

out:	if(!dev->dvrok)dummy=write(dev->ifd,&dummy,sizeof(dummy));
	fuse_opt_free_args(&args);

	pthread_exit(NULL);
}

static void dmx_notify(fuse_req_t req,void *data)
{
	DMXDTA *dmx=(DMXDTA *)data;
	uint64_t dummy=1;

	pthread_spin_lock(&dmx->dev->spin);

	if(dmx->open&&dmx->s->notify&&!dmx->s->event)
	{
		dmx->s->event=1;
		dmx->s->enx=NULL;
		if(!dmx->dev->etl)dmx->dev->ehd=dmx->dev->etl=dmx->s;
		else dmx->dev->etl=dmx->dev->etl->enx=dmx->s;
	}

	pthread_spin_unlock(&dmx->dev->spin);

	dummy=write(dmx->dev->nfd,&dummy,sizeof(dummy));
}

static void dmx_post(void *userdata)
{
	DEVICE *dev=(DEVICE *)userdata;
	char devpath[PATH_MAX];
	int unused __attribute__ ((unused));
	uint64_t dummy=1;

	int owner=dev->conf.owner?dev->conf.owner:-1;
	int group=dev->conf.group?dev->conf.group:-1;
	int perms=dev->conf.perms?dev->conf.perms:0666;

	sprintf(devpath,"/dev/dvb/adapter%d/demux0",dev->conf.adapter);
	usleep(dev->conf.udevwait*1000);
	setfsuid(0);
	unused=chown(devpath,owner,group);
	unused=chmod(devpath,perms);
	setfsuid(getuid());
	pthread_spin_lock(&dev->spin);
	dev->dmxok=1;
	pthread_spin_unlock(&dev->spin);
	dummy=write(dev->ifd,&dummy,sizeof(dummy));
}

static void dmx_open(fuse_req_t req,struct fuse_file_info *fi)
{
	DEVICE *dev=fuse_req_userdata(req);
	STREAM *s;
	int err=EINVAL;

	if(!dev)goto err;
	pthread_rwlock_wrlock(&dev->mtx);

	if(!dev->running)FAIL(EBUSY);

	if(!(s=malloc(sizeof(STREAM)+sizeof(DMXDTA))))FAIL(ENOMEM);
	memset(s,0,sizeof(STREAM));
	s->flags=fi->flags;
	s->dev=dev;
	s->dmx=(DMXDTA *)(((unsigned char *)s)+sizeof(STREAM));
	memset(s->dmx,0,sizeof(DMXDTA));
	s->dmx->dev=dev;
	s->dmx->s=s;
	s->dmx->open=1;

	if(pthread_mutex_init(&s->mtx,NULL))
	{
		free(s);
		FAIL(ENOMEM);
	}

	if(!(s->dmx->bfr=malloc(DMX_BUFSIZE)))
	{
		pthread_mutex_destroy(&s->mtx);
		free(s);
		FAIL(ENOMEM);
	}
	s->dmx->bufsize=DMX_BUFSIZE;
	s->dmx->thres=(s->dmx->bufsize>>1)+(s->dmx->bufsize>>2);

	s->next=dev->list;
	dev->list=s;

	fi->direct_io=1;
	fi->keep_cache=0;
	fi->nonseekable=1;
	fi->fh=(uint64_t)s;

	pthread_rwlock_unlock(&dev->mtx);
	fuse_reply_open(req,fi);
	return;

fail:	pthread_rwlock_unlock(&dev->mtx);
err:	fuse_reply_err(req,err);
}

static void dmx_read(fuse_req_t req,size_t size,off_t off,
	struct fuse_file_info *fi)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	DMXDTA *dmx=s->dmx;
	int err=0;
	int add=0;
	ssize_t len=0;

	pthread_rwlock_rdlock(&dev->mtx);
	pthread_mutex_lock(&s->mtx);

	if(!dev->running)FAIL(EBUSY);
	if((s->flags&O_ACCMODE)==O_WRONLY)FAIL(EPERM);
	if(s->req)FAIL(EAGAIN);

	if(!dmx->running)goto nonblk;

	pthread_spin_lock(&dev->spin);
	if(dmx->ftype==4)
	{
		add=2;
		if(dmx->fill)len=(dmx->bfr[dmx->tail]<<8)|dmx->bfr[dmx->tail+1];
		if(len>size)
		{
			if((dmx->tail+=len+add)==dmx->bufsize)dmx->tail=0;
			dmx->fill-=len+add;
			pthread_spin_unlock(&dev->spin);
			FAIL(EOVERFLOW);
		}
	}
	else
	{
		len=(dmx->fill>size?size:dmx->fill);
		if(dmx->tail>=dmx->head)if(len>dmx->bufsize-dmx->tail)
			len=dmx->bufsize-dmx->tail;
	}
	pthread_spin_unlock(&dev->spin);

	if(!len)
	{
nonblk:		if(s->flags&O_NONBLOCK)FAIL(EAGAIN);
		else
		{
			s->req=req;
			s->size=size;
			s->enx=NULL;
			if(dmx->ftype==4&&dmx->timeout)
			{
				s->start=times(NULL);
				s->type=2;
				add_timeout(dev);
			}
			else s->type=3;
			pthread_spin_lock(&dev->spin);
			s->notify++;
			pthread_spin_unlock(&dev->spin);
			fuse_req_interrupt_func(req,dmx_notify,dmx);
			goto fail;
		}
	}

	fuse_reply_buf(req,dmx->bfr+dmx->tail+add,len);

	pthread_spin_lock(&dev->spin);
	if((dmx->tail+=len+add)==dmx->bufsize)dmx->tail=0;
	dmx->fill-=len+add;
	pthread_spin_unlock(&dev->spin);

fail:	pthread_mutex_unlock(&s->mtx);
	pthread_rwlock_unlock(&dev->mtx);
	if(err)fuse_reply_err(req,err);
}

static void dmx_write(fuse_req_t req,const char *buf,size_t size,off_t off,
	struct fuse_file_info *fi)
{
	STREAM *s=(STREAM *)fi->fh;

	if((s->flags&O_ACCMODE)==O_RDONLY)fuse_reply_err(req,EPERM);
	else fuse_reply_err(req,EOPNOTSUPP);
}

static void dmx_flush(fuse_req_t req,struct fuse_file_info *fi)
{
	fuse_reply_err(req,EOPNOTSUPP);
}

static void dmx_release(fuse_req_t req,struct fuse_file_info *fi)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	DMXDTA *dmx=s->dmx;
	STREAM **e;
	unsigned char *ptr;
	int err=0;

	pthread_rwlock_wrlock(&dev->mtx);

	if(!dev->running)FAIL(EBUSY);

	pthread_spin_lock(&dev->spin);
	dmx->open=0;
	pthread_spin_unlock(&dev->spin);

	if(dmx->usrflt)satip_util_filter_del_user(dev->filter,dmx->usrflt);
	if(dmx->secflt)satip_util_section_free(dmx->secflt);

	switch(dmx->ftype)
	{
	case 1:	pthread_spin_lock(&dev->spin);

		if(dmx->running)
		{
			dev->dvrrun--;
			if(dev->dvropen&&!dev->dvrrun)
			{
				pthread_spin_unlock(&dev->spin);

				if(dmx->dev->plug.strexit)
				    dmx->dev->plug.strexit(dmx->dev->plug.ctx,
					dmx->dev->plugdta);

				dev->dvrhead=0;
				dev->dvrtail=0;
				dev->dvrfill=0;
				dev->plughead=0;
				dev->plugfill=0;
			}
			else pthread_spin_unlock(&dev->spin);
		}
		else pthread_spin_unlock(&dev->spin);

		if(!dev->dvrrun&&!dev->dvropen)
		{
			if(dev->dvrbufsize!=dev->conf.bufsize)
				if((ptr=realloc(dev->dvrbfr,dev->conf.bufsize)))
			{
				dev->dvrbfr=ptr;
				dev->dvrbufsize=dev->conf.bufsize;
				dev->dvrthres=(dev->dvrbufsize>>1)+
					(dev->dvrbufsize>>2);
			}

			dev->dvrhead=0;
			dev->dvrtail=0;
			dev->dvrfill=0;
			dev->plughead=0;
			dev->plugfill=0;
		}
		break;

	case 2:
	case 3:	if(dmx->running&&dmx->dev->plug.strexit)
			dmx->dev->plug.strexit(dmx->dev->plug.ctx,
				dmx->plugdta);
		break;
	}

	free(dmx->bfr);
	pthread_mutex_destroy(&s->mtx);

	if(s->ph)fuse_pollhandle_destroy(s->ph);

	if(s->req)
	{
		if(s->type==2)del_timeout(dev);
		fuse_reply_err(s->req,EBADF);
	}

	clrpid(dev,dmx);

	for(e=&dev->ehd;*e;e=&(*e)->enx)if(*e==s)
	{
		*e=s->enx;
		if(!dev->ehd)dev->etl=NULL;
		break;
	}

	for(e=&dev->list;*e;e=&(*e)->next)if(*e==s)
	{
		*e=s->next;
		free(s);
		break;
	}

fail:	pthread_rwlock_unlock(&dev->mtx);
	fuse_reply_err(req,err);
}

static void dmx_fsync(fuse_req_t req,int datasync,struct fuse_file_info *fi)
{
	fuse_reply_err(req,EOPNOTSUPP);
}

static void dmx_ioctl(fuse_req_t req,int cmd,void *arg,
	struct fuse_file_info *fi,unsigned flags,const void *in_buf,
	size_t in_bufsz,size_t out_bufsz)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	DMXDTA *dmx=s->dmx;
	STREAM **e;
	int err=EINVAL;
	struct iovec iov;
	union
	{
		void *ptr;
		uint16_t *pid;
		struct dmx_sct_filter_params *sctflt;
		struct dmx_pes_filter_params *pesflt;
	}u;

	pthread_rwlock_wrlock(&dev->mtx);

	if(!dev->running)FAIL(EBUSY);
	if(flags&FUSE_IOCTL_COMPAT)FAIL(ENOSYS);

	switch(cmd)
	{
	case DMX_START:
		switch(dmx->ftype)
		{
		case 1:	if(dmx->running)break;
			if(!dmx->dev->dvrrun&&dmx->dev->dvropen&&
				dmx->dev->plug.strinit)
				dmx->dev->plug.strinit(dmx->dev->plug.ctx,
					&dmx->dev->plugdta);
			pthread_spin_lock(&dev->spin);
			dmx->dev->dvrrun++;
			dmx->running=1;
			pthread_spin_unlock(&dev->spin);
			break;

		case 2:
		case 3:	if(dmx->running)break;
			if(dmx->dev->plug.strinit)	
				dmx->dev->plug.strinit(dmx->dev->plug.ctx,
					&dmx->plugdta);
			pthread_spin_lock(&dev->spin);
			dmx->running=1;
			pthread_spin_unlock(&dev->spin);
			break;

		case 4:	if(dmx->running)break;
			pthread_spin_lock(&dev->spin);
			dmx->running=1;
			pthread_spin_unlock(&dev->spin);
			break;

		default:FAIL(EINVAL);
		}

		pthread_rwlock_unlock(&dev->mtx);
		fuse_reply_ioctl(req,0,NULL,0);
		return;

	case DMX_STOP:
		switch(dmx->ftype)
		{
		case 1:	if(!dmx->running)break;

			pthread_spin_lock(&dev->spin);
			dmx->running=0;
			if(!--(dmx->dev->dvrrun))
			{
				while(dev->plugwork)
				{
					pthread_spin_unlock(&dev->spin);
					usleep(1000);
					pthread_spin_lock(&dev->spin);
				}
				pthread_spin_unlock(&dev->spin);

				if(dmx->dev->dvropen&&dmx->dev->plug.strexit)
				    dmx->dev->plug.strexit(dmx->dev->plug.ctx,
					dmx->dev->plugdta);

				dmx->dev->dvrhead=0;
				dmx->dev->dvrtail=0;
				dmx->dev->dvrfill=0;
				dmx->dev->plughead=0;
				dmx->dev->plugfill=0;

				for(e=&dev->ehd;*e;e=&(*e)->enx)if(*e==dev->s)
				{
					dev->s->event=0;
					*e=dev->s->enx;
					if(!dev->ehd)dev->etl=NULL;
					break;
				}
			}
			else pthread_spin_unlock(&dev->spin);
			break;

		case 2:
		case 3:
		case 4:	if(!dmx->running)break;

			pthread_spin_lock(&dev->spin);
			dmx->running=0;
			while(dmx->plugwork)
			{
				pthread_spin_unlock(&dev->spin);
				usleep(1000);
				pthread_spin_lock(&dev->spin);
			}
			pthread_spin_unlock(&dev->spin);

			if(dmx->ftype!=4&&dmx->dev->plug.strexit)
				dmx->dev->plug.strexit(dmx->dev->plug.ctx,
					dmx->plugdta);

			dmx->head=0;
			dmx->tail=0;
			dmx->fill=0;
			dmx->plughead=0;
			dmx->plugfill=0;

			for(e=&dev->ehd;*e;e=&(*e)->enx)if(*e==s)
			{
				s->event=0;
				*e=s->enx;
				if(!dev->ehd)dev->etl=NULL;
				break;
			}

			if(dmx->oneshot)dmx->oneshot=0;
			break;

		default:FAIL(EINVAL);
		}

		pthread_rwlock_unlock(&dev->mtx);
		fuse_reply_ioctl(req,0,NULL,0);
		return;

	case DMX_SET_BUFFER_SIZE:
		if(dmx->running)FAIL(EBUSY);
		if((long)arg<8192||(long)arg>1073741824)FAIL(EINVAL);
		err=(long)arg/188;
		err+=1;
		err*=188;
		if(err<(dmx->ftype==2?dev->conf.bufsize:DMX_BUFSIZE))
			err=(dmx->ftype==2?dev->conf.bufsize:DMX_BUFSIZE);
		else
		{
			err+=SATIP_MAX_BURST*7*188-1;
			err/=SATIP_MAX_BURST*7*188;
			err*=SATIP_MAX_BURST*7*188;
		}
		if(err!=dmx->bufsize)
		{
			if(!(u.ptr=realloc(dmx->bfr,err)))FAIL(ENOMEM);
			dmx->bfr=u.ptr;
			dmx->bufsize=err;
			dmx->thres=(dmx->bufsize>>1)+(dmx->bufsize>>2);
		}
		dmx->head=0;
		dmx->tail=0;
		dmx->fill=0;
		dmx->plughead=0;
		dmx->plugfill=0;
		pthread_rwlock_unlock(&dev->mtx);
		fuse_reply_ioctl(req,0,NULL,0);
		return;

	case DMX_ADD_PID:
		if(!in_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(uint16_t);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,NULL,0);
		}
		else
		{
			u.pid=(uint16_t *)in_buf;
			switch(dmx->ftype)
			{
			case 2:	if(!satip_util_user_addpid(dmx->usrflt,*u.pid))
				{
					addpid(dev,dmx,*u.pid,dmx->ftype);
					break;
				}
			default:FAIL(EINVAL);
			}
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,NULL,0);
		}
		return;

	case DMX_REMOVE_PID:
		if(!in_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(uint16_t);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,NULL,0);
		}
		else
		{
			u.pid=(uint16_t *)in_buf;
			switch(dmx->ftype)
			{
			case 2:	if(!satip_util_user_delpid(dmx->usrflt,*u.pid))
				{
					delpid(dev,dmx,*u.pid,dmx->ftype);
					break;
				}
			default:FAIL(EINVAL);
			}
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,NULL,0);
		}
		return;

	case DMX_SET_PES_FILTER:
		if(!in_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dmx_pes_filter_params);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,NULL,0);
		}
		else
		{
			u.pesflt=(struct dmx_pes_filter_params *)in_buf;
			if(u.pesflt->pid>0x2000||
				u.pesflt->input!=DMX_IN_FRONTEND
				||u.pesflt->pes_type>DMX_PES_OTHER)
				FAIL(EINVAL);

			switch(u.pesflt->output)
			{
			case DMX_OUT_TAP:
				if(u.pesflt->pid==0x2000)FAIL(EINVAL);
			case DMX_OUT_TS_TAP:
			case DMX_OUT_TSDEMUX_TAP:
				break;
			default:FAIL(EINVAL);
			}

			switch(dmx->ftype)
			{
			case 1:	satip_util_filter_del_user(dev->filter,
					dmx->usrflt);	
				if(dmx->running)
				{
					pthread_spin_lock(&dev->spin);
					if(!--(dev->dvrrun))
					{
						pthread_spin_unlock(&dev->spin);

						if(dev->dvropen&&
						    dev->plug.strexit)
						    dev->plug.strexit(
						    dev->plug.ctx,dev->plugdta);

						dev->dvrhead=0;
						dev->dvrtail=0;
						dev->dvrfill=0;
						dev->plughead=0;
						dev->plugfill=0;

						for(e=&dev->ehd;*e;
							e=&(*e)->enx)
								if(*e==dev->s)
						{
							dev->s->event=0;
							*e=dev->s->enx;
							if(!dev->ehd)
								dev->etl=NULL;
							break;
						}
					}
					else pthread_spin_unlock(&dev->spin);
				}
				break;

			case 2:
			case 3:
			case 4:	satip_util_filter_del_user(dev->filter,
					dmx->usrflt);
				if(dmx->secflt)
					satip_util_section_free(dmx->secflt);
				dmx->secflt=NULL;

				if(dmx->ftype!=4&&dmx->running&&
				    dev->plug.strexit)
					dev->plug.strexit(dev->plug.ctx,
						dev->plugdta);

				for(e=&dev->ehd;*e;e=&(*e)->enx)if(*e==dev->s)
				{
					dev->s->event=0;
					*e=dev->s->enx;
					if(!dev->ehd)dev->etl=NULL;
					break;
				}
				break;
			}

			dmx->usrflt=NULL;
			dmx->ftype=0;
			dmx->running=0;
			dmx->head=0;
			dmx->tail=0;
			dmx->fill=0;
			dmx->plughead=0;
			dmx->plugfill=0;

			clrpid(dev,dmx);

			switch(u.pesflt->output)
			{
			case DMX_OUT_TS_TAP:
				if(satip_util_filter_add_user(dev->filter,
					&dmx->usrflt,dev->plug.multi?
					dvrmulti:dvrstream,dmx))FAIL(ENOMEM);
				dmx->ftype=1;
				break;
			case DMX_OUT_TSDEMUX_TAP:
				if(dmx->bufsize<dev->conf.bufsize)
				{
					if(!(e=realloc(dmx->bfr,
						dev->conf.bufsize)))
						FAIL(ENOMEM);
					dmx->bfr=(void *)e;
					dmx->bufsize=dev->conf.bufsize;
					dmx->thres=(dmx->bufsize>>1)+
						(dmx->bufsize>>2);
				}
				if(satip_util_filter_add_user(dev->filter,
					&dmx->usrflt,dev->plug.multi?
					dmxmulti:dmxstream,dmx))FAIL(ENOMEM);
				dmx->ftype=2;
				break;
			case DMX_OUT_TAP:
				if(satip_util_filter_add_user(dev->filter,
					&dmx->usrflt,pesstream,dmx))
					FAIL(ENOMEM);
				dmx->ftype=3;
			default:break;
			}

			if(satip_util_user_addpid(dmx->usrflt,u.pesflt->pid))
			{
				satip_util_filter_del_user(dev->filter,
					dmx->usrflt);
				dmx->usrflt=NULL;
				dmx->ftype=0;
				FAIL(EINVAL);
			}

			addpid(dev,dmx,u.pesflt->pid,dmx->ftype);

			if(u.pesflt->flags&DMX_IMMEDIATE_START)
			{
				switch(dmx->ftype)
				{
				case 1:	if(!dev->dvrrun&&dev->dvropen&&
					    dev->plug.strinit)
						dev->plug.strinit(dev->plug.ctx,
							&dev->plugdta);
					break;
				case 2:
				case 3:	if(dev->plug.strinit)
						dev->plug.strinit(dev->plug.ctx,
							&dmx->plugdta);
					break;
				}

				pthread_spin_lock(&dev->spin);
				if(dmx->ftype==1)dev->dvrrun++;
				dmx->running=1;
				pthread_spin_unlock(&dev->spin);
			}

			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,NULL,0);
		}
		return;

	case DMX_GET_STC:
	case DMX_GET_PES_PIDS:
		FAIL(EINVAL);

	case DMX_SET_FILTER:
		if(!in_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dmx_sct_filter_params);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,NULL,0);
		}
		else
		{
			u.sctflt=(struct dmx_sct_filter_params *)in_buf;

			if(u.sctflt->pid>=0x2000)FAIL(EINVAL);
			if(u.sctflt->timeout<0||u.sctflt->timeout>1000)
				FAIL(EINVAL);

			switch(dmx->ftype)
			{
			case 1:	satip_util_filter_del_user(dev->filter,
					dmx->usrflt);	
				if(dmx->running)
				{
					pthread_spin_lock(&dev->spin);
					if(!--(dev->dvrrun))
					{
						pthread_spin_unlock(&dev->spin);

						if(dev->dvropen&&
						    dev->plug.strexit)
						    dev->plug.strexit(
						    dev->plug.ctx,dev->plugdta);

						dev->dvrhead=0;
						dev->dvrtail=0;
						dev->dvrfill=0;
						dev->plughead=0;
						dev->plugfill=0;

						for(e=&dev->ehd;*e;
							e=&(*e)->enx)
								if(*e==dev->s)
						{
							dev->s->event=0;
							*e=dev->s->enx;
							if(!dev->ehd)
								dev->etl=NULL;
							break;
						}
					}
					else pthread_spin_unlock(&dev->spin);
				}
				break;

			case 2:
			case 3:
			case 4:	satip_util_filter_del_user(dev->filter,
					dmx->usrflt);
				if(dmx->secflt)
					satip_util_section_free(dmx->secflt);
				dmx->secflt=NULL;

				if(dmx->ftype!=4&&dmx->running&&
				    dev->plug.strexit)
					dev->plug.strexit(dev->plug.ctx,
						dev->plugdta);

				for(e=&dev->ehd;*e;e=&(*e)->enx)if(*e==dev->s)
				{
					dev->s->event=0;
					*e=dev->s->enx;
					if(!dev->ehd)dev->etl=NULL;
					break;
				}
				break;
			}

			dmx->usrflt=NULL;
			dmx->ftype=0;
			dmx->running=0;
			dmx->head=0;
			dmx->tail=0;
			dmx->fill=0;
			dmx->plughead=0;
			dmx->plugfill=0;

			clrpid(dev,dmx);

			memcpy(dmx->filter.filter,u.sctflt->filter.filter,
				SATIP_FILTERSZ);
			memcpy(dmx->filter.mask,u.sctflt->filter.mask,
				SATIP_FILTERSZ);
			memcpy(dmx->filter.mode,u.sctflt->filter.mode,
				SATIP_FILTERSZ);

			if(satip_util_section_create(&dmx->secflt,&dmx->filter,
				u.sctflt->flags&DMX_CHECK_CRC,secstream,dmx))
				FAIL(ENOMEM);

			if(satip_util_filter_add_user(dev->filter,&dmx->usrflt,
				satip_util_section_packet,dmx->secflt))
			{
				satip_util_section_free(dmx->secflt);
				dmx->secflt=NULL;
				dmx->ftype=0;
				FAIL(ENOMEM);
			}

			dmx->ftype=4;
			dmx->oneshot=(u.sctflt->flags&DMX_ONESHOT)?1:0;
			dmx->timeout=u.sctflt->timeout*sysconf(_SC_CLK_TCK);
			if(dmx->timeout)dmx->timeout+=sysconf(_SC_CLK_TCK)>>1;

			if(satip_util_user_addpid(dmx->usrflt,u.sctflt->pid))
			{
				satip_util_filter_del_user(dev->filter,
					dmx->usrflt);
				satip_util_section_free(dmx->secflt);
				dmx->usrflt=NULL;
				dmx->secflt=NULL;
				dmx->ftype=0;
				FAIL(EINVAL);
			}

			addpid(dev,dmx,u.sctflt->pid,dmx->ftype);

			if(u.sctflt->flags&DMX_IMMEDIATE_START)
			{
				pthread_spin_lock(&dev->spin);
				dmx->running=1;
				pthread_spin_unlock(&dev->spin);
			}

			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,NULL,0);
		}
		return;
	}

fail:	pthread_rwlock_unlock(&dev->mtx);
	fuse_reply_err(req,err);
}

static void dmx_poll(fuse_req_t req,struct fuse_file_info *fi,
	struct fuse_pollhandle *ph)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	DMXDTA *dmx=s->dmx;
	struct fuse_pollhandle *pd=NULL;
	int err=0;
	int revents=0;

	pthread_rwlock_rdlock(&dev->mtx);
	pthread_mutex_lock(&s->mtx);

	if(!dev->running)FAIL(EBUSY);

	pthread_spin_lock(&dev->spin);

	if(dmx->running&&dmx->fill)revents=POLLIN;

	if(ph)
	{
		if(revents)
		{
			if(s->ph)
			{
				pd=s->ph;
				s->ph=NULL;
				s->notify--;
			}
		}
		else
		{
			if(s->ph)pd=s->ph;
			else s->notify++;
			s->ph=ph;
		}
	}
	else if(s->ph&&revents)
	{
		ph=s->ph;
		s->ph=NULL;
		s->notify--;
	}

	pthread_spin_unlock(&dev->spin);

fail:	pthread_mutex_unlock(&s->mtx);
	pthread_rwlock_unlock(&dev->mtx);
	if(err)fuse_reply_err(req,err);
	else
	{
		if(pd)fuse_pollhandle_destroy(pd);
		fuse_reply_poll(req,revents);
		if(revents&&ph)
		{
			fuse_lowlevel_notify_poll(ph);
			fuse_pollhandle_destroy(ph);
		}
	}
}

static const struct cuse_lowlevel_ops dmx_ops=
{
	.init_done=dmx_post,
	.open=dmx_open,
	.read=dmx_read,
	.write=dmx_write,
	.flush=dmx_flush,
	.release=dmx_release,
	.fsync=dmx_fsync,
	.ioctl=dmx_ioctl,
	.poll=dmx_poll,
};

static void *dmxworker(void *data)
{
	struct cuse_info ci;
	DEVICE *dev=(DEVICE *)data;
	char devpath[PATH_MAX+9];
	const char *devarg[1]={devpath};
	int unused;
	int dummy_argc=1;
	char *dummy_argv[66]={""};
	struct fuse_args args=FUSE_ARGS_INIT(dummy_argc,dummy_argv);
	uint64_t dummy=1;
	sigset_t set;

	sigfillset(&set);
	sigdelset(&set,SIGQUIT);
	pthread_sigmask(SIG_SETMASK,&set,NULL);

	pthread_setname_np(pthread_self(),"loopd dmxworker");

	if(fuse_opt_parse(&args,NULL,dummy_opts,dummy_args))goto out;
	if(fuse_opt_add_arg(&args, "-f"))goto out;

	sprintf(devpath,"DEVNAME=dvb/adapter%d/demux0",dev->conf.adapter);
	memset(&ci,0,sizeof(ci));
	ci.dev_major=dev->conf.major;
	ci.dev_minor=dev->conf.minbase+1;
	ci.dev_info_argc=1;
	ci.dev_info_argv=devarg;
	ci.flags=CUSE_UNRESTRICTED_IOCTL;

	setfsuid(0);
	if(!(dev->dmxse=cuse_lowlevel_setup(args.argc,args.argv,&ci,&dmx_ops,
		&unused,data)))
	{
		setfsuid(getuid());
		goto out;
	}
	setfsuid(getuid());
	fuse_session_loop_mt(dev->dmxse);
	fuse_session_destroy(dev->dmxse);

out:	if(!dev->dmxok)dummy=write(dev->ifd,&dummy,sizeof(dummy));
	fuse_opt_free_args(&args);

	pthread_exit(NULL);
}

static void fe_notify(fuse_req_t req,void *data)
{
	DEVICE *dev=(DEVICE *)data;
	uint64_t dummy=1;

	pthread_spin_lock(&dev->spin);

	if(dev->feopen&&dev->fe->notify&&!dev->fe->event)
	{
		dev->fe->event=1;
		dev->fe->enx=NULL;
		if(!dev->etl)dev->ehd=dev->etl=dev->fe;
		else dev->etl=dev->etl->enx=dev->fe;
	}

	pthread_spin_unlock(&dev->spin);

	dummy=write(dev->nfd,&dummy,sizeof(dummy));
}

static void fe_post(void *userdata)
{
	DEVICE *dev=(DEVICE *)userdata;
	char devpath[PATH_MAX];
	int unused __attribute__ ((unused));
	uint64_t dummy=1;

	int owner=dev->conf.owner?dev->conf.owner:-1;
	int group=dev->conf.group?dev->conf.group:-1;
	int perms=dev->conf.perms?dev->conf.perms:0666;

	sprintf(devpath,"/dev/dvb/adapter%d/frontend0",dev->conf.adapter);
	usleep(dev->conf.udevwait*1000);
	setfsuid(0);
	unused=chown(devpath,owner,group);
	unused=chmod(devpath,perms);
	setfsuid(getuid());
	pthread_spin_lock(&dev->spin);
	dev->feok=1;
	pthread_spin_unlock(&dev->spin);
	dummy=write(dev->ifd,&dummy,sizeof(dummy));
}

static void fe_open(fuse_req_t req,struct fuse_file_info *fi)
{
	DEVICE *dev=fuse_req_userdata(req);
	STREAM *s;
	int err=EINVAL;
	uint64_t dummy=1;

	if(!dev)goto err;
	pthread_rwlock_wrlock(&dev->mtx);

	if(!dev->running)FAIL(EBUSY);
	if((fi->flags&O_ACCMODE)!=O_RDONLY&&dev->feopen)FAIL(EBUSY);
	if(!(s=malloc(sizeof(STREAM))))FAIL(EMFILE);

	if(pthread_mutex_init(&s->mtx,NULL))
	{
		free(s);
		FAIL(ENOMEM);
	}

	memset(s,0,sizeof(STREAM));
	s->flags=fi->flags;
	s->dev=dev;

	s->next=dev->list;
	dev->list=s;

	pthread_spin_lock(&dev->spin);
	if((s->flags&O_ACCMODE)!=O_RDONLY)
	{
		dev->feopen=1;
		dev->fe=s;
		dev->reply=0;
	}
	dev->fecnt++;
	pthread_spin_unlock(&dev->spin);

	if(dev->fecnt==1)
	{
		if(dev->enabled)
		{
			dev->enabled=0;
			dummy=write(dev->ffd,&dummy,sizeof(dummy));
		}
		else
		{
			pthread_spin_lock(&dev->spin);
			dev->event=0;
			pthread_spin_unlock(&dev->spin);
		}
		dev->hl=0;
		dev->src=0;
		satip_util_init_pids(&dev->set);
		satip_util_init_tune(&dev->tune);
		dev->tune.msys=dev->conf.mode;
		switch(dev->conf.mode)
		{
		case SATIP_DVBS:
		case SATIP_DVBS2:
			dev->freq=1000000000;
			if(dev->enabled&&dev->conf.lnbpower)
			{
				dev->enabled=0;
				dummy=write(dev->ffd,&dummy,sizeof(dummy));
			}
			dev->tune.pol=SATIP_POL_V;
			dev->tune.mtype=SATIP_QPSK;
			dev->tune.ro=SATIP_ROFF_035;
			break;
		default:dev->freq=54000000;
			break;
		}
		pthread_spin_lock(&dev->spin);
		memset(&dev->status,0,sizeof(dev->status));
		pthread_spin_unlock(&dev->spin);
	}

	fi->direct_io=1;
	fi->keep_cache=0;
	fi->nonseekable=1;
	fi->fh=(uint64_t)s;

	pthread_rwlock_unlock(&dev->mtx);
	fuse_reply_open(req,fi);
	return;

fail:	pthread_rwlock_unlock(&dev->mtx);
err:	fuse_reply_err(req,err);
}

static void fe_read(fuse_req_t req,size_t size,off_t off,
	struct fuse_file_info *fi)
{
	STREAM *s=(STREAM *)fi->fh;

	if((s->flags&O_ACCMODE)==O_WRONLY)fuse_reply_err(req,EPERM);
	else fuse_reply_err(req,EOPNOTSUPP);
}

static void fe_write(fuse_req_t req,const char *buf,size_t size,off_t off,
	struct fuse_file_info *fi)
{
	STREAM *s=(STREAM *)fi->fh;

	if((s->flags&O_ACCMODE)==O_RDONLY)fuse_reply_err(req,EPERM);
	else fuse_reply_err(req,EOPNOTSUPP);
}

static void fe_flush(fuse_req_t req,struct fuse_file_info *fi)
{
	fuse_reply_err(req,EOPNOTSUPP);
}

static void fe_release(fuse_req_t req,struct fuse_file_info *fi)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	STREAM **e;
	int err=0;
	uint64_t dummy;

	pthread_rwlock_wrlock(&dev->mtx);

	if(!dev->running)FAIL(EBUSY);

	pthread_mutex_destroy(&s->mtx);

	pthread_spin_lock(&dev->spin);
	dev->fecnt--;

	if((s->flags&O_ACCMODE)!=O_RDONLY)
	{
		dev->feopen=0;
		dev->fe=NULL;
		dev->event=0;
		pthread_spin_unlock(&dev->spin);

		if(s->ph)fuse_pollhandle_destroy(s->ph);

		if(s->req)fuse_reply_err(s->req,EBADF);

		for(e=&dev->ehd;*e;e=&(*e)->enx)if(*e==s)
		{
			*e=s->enx;
			if(!dev->ehd)dev->etl=NULL;
			break;
		}
	}
	else pthread_spin_unlock(&dev->spin);

	if(!dev->fecnt)
	{
		if(dev->enabled)
		{
			dev->enabled=0;
			dummy=1;
			dummy=write(dev->ffd,&dummy,sizeof(dummy));
		}
		freepid(dev);
	}

	for(e=&dev->list;*e;e=&(*e)->next)if(*e==s)
	{
		*e=s->next;
		free(s);
		break;
	}

fail:	pthread_rwlock_unlock(&dev->mtx);
	fuse_reply_err(req,err);
}

static void fe_fsync(fuse_req_t req,int datasync,struct fuse_file_info *fi)
{
	fuse_reply_err(req,EOPNOTSUPP);
}

static void fe_ioctl(fuse_req_t req,int cmd,void *arg,
	struct fuse_file_info *fi,unsigned flags,const void *in_buf,
	size_t in_bufsz,size_t out_bufsz)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	int err=EINVAL;
	int i;
	uint64_t dummy=1;
	struct iovec iov;
	union
	{
		struct dvb_frontend_info info;
		fe_status_t status;
		uint32_t u32;
		uint16_t u16;
		struct dvb_diseqc_master_cmd *cmd;
		struct dvb_diseqc_slave_reply reply;
		struct dvb_frontend_parameters *pin;
		struct dvb_frontend_parameters pout;
		struct dvb_frontend_event event;
	}u;
	struct dtv_properties props;

	pthread_rwlock_wrlock(&dev->mtx);

	if(!dev->running)FAIL(EBUSY);
	if((s->flags&O_ACCMODE)==O_RDONLY&&(_IOC_DIR(cmd)!=_IOC_READ||
		cmd==FE_GET_EVENT||cmd==FE_DISEQC_RECV_SLAVE_REPLY))FAIL(EPERM);
	if(flags&FUSE_IOCTL_COMPAT)FAIL(ENOSYS);

	switch(cmd)
	{
	case FE_SET_PROPERTY:
		if(!in_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dtv_properties);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,NULL,0);
		}
		else if(in_bufsz==sizeof(struct dtv_properties))
		{
			props=*(struct dtv_properties *)in_buf;
			if(!props.num||props.num>DTV_IOCTL_MAX_MSGS)
				FAIL(EINVAL);
			iov.iov_base=props.props;
			iov.iov_len=sizeof(struct dtv_property)*props.num;
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,NULL,0);
		}
		else
		{
			props.props=(struct dtv_property *)in_buf;
			props.num=in_bufsz/sizeof(struct dtv_property);
			for(i=0;i<props.num;i++)
				if((err=setprop(dev,&props.props[i])))FAIL(err);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,NULL,0);
		}
		return;

	case FE_GET_PROPERTY:
		if(!in_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dtv_properties);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,NULL,0);
		}
		else if(in_bufsz==sizeof(struct dtv_properties)&&!out_bufsz)
		{
			props=*(struct dtv_properties *)in_buf;
			if(!props.num||props.num>DTV_IOCTL_MAX_MSGS)
				FAIL(EINVAL);
			iov.iov_base=props.props;
			iov.iov_len=sizeof(struct dtv_property)*props.num;
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,&iov,1);
		}
		else if(in_bufsz&&in_bufsz==out_bufsz)
		{
			props.props=(struct dtv_property *)in_buf;
			props.num=out_bufsz/sizeof(struct dtv_property);
			for(i=0;i<props.num;i++)
				if((err=getprop(dev,&props.props[i])))FAIL(err);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,in_buf,out_bufsz);
		}
		else FAIL(ENODATA);
		return;

	case FE_GET_INFO:
		if(!out_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dvb_frontend_info);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,NULL,0,&iov,1);
		}
		else
		{
			strcpy(u.info.name,"SAT>IP Network Tuner");
			u.info.frequency_stepsize=0;
			u.info.frequency_tolerance=0;
			u.info.symbol_rate_min=1000000;
			u.info.symbol_rate_max=50000000;
			u.info.symbol_rate_tolerance=0;
			u.info.notifier_delay=0;
			u.info.caps=FE_CAN_INVERSION_AUTO|FE_CAN_FEC_1_2|
				FE_CAN_FEC_2_3|FE_CAN_FEC_3_4|FE_CAN_FEC_4_5|
				FE_CAN_FEC_5_6|FE_CAN_FEC_7_8|FE_CAN_FEC_8_9|
				FE_CAN_FEC_AUTO|FE_CAN_QPSK|FE_CAN_QAM_16|
				FE_CAN_QAM_32|FE_CAN_QAM_64|FE_CAN_QAM_128|
				FE_CAN_QAM_256|FE_CAN_QAM_AUTO|
				FE_CAN_TRANSMISSION_MODE_AUTO|
				FE_CAN_BANDWIDTH_AUTO|
				FE_CAN_GUARD_INTERVAL_AUTO|
				FE_CAN_HIERARCHY_AUTO|FE_HAS_EXTENDED_CAPS|
				FE_CAN_MULTISTREAM|
				FE_CAN_RECOVER|FE_CAN_MUTE_TS;

			switch(dev->conf.mode)
			{
			case SATIP_DVBS2:
				u.info.caps|=FE_CAN_2G_MODULATION;
			case SATIP_DVBS:
				u.info.type=FE_QPSK;
				switch(dev->conf.lnb)
				{
				case SATIP_LNB_UNIV:
					u.info.frequency_min=950000;
					u.info.frequency_max=2150000;
					break;
				case SATIP_LNB_DBS:
					u.info.frequency_min=950000;
					u.info.frequency_max=1450000;
					break;
				case SATIP_LNB_CMONO:
					u.info.frequency_min=950000;
					u.info.frequency_max=1450000;
					break;
				case SATIP_LNB_CMULT:
					u.info.frequency_min=950000;
					u.info.frequency_max=2050000;
					break;
				case SATIP_LNB_AUS:
					u.info.frequency_min=1000000;
					u.info.frequency_max=2050000;
					break;
        			}
				break;

			case SATIP_DVBT2:
				u.info.caps|=FE_CAN_2G_MODULATION;
			case SATIP_DVBT:
				u.info.type=FE_OFDM;
				u.info.frequency_min=54000000;
				u.info.frequency_max=957000000;
				break;

			case SATIP_DVBC2:
				u.info.caps|=FE_CAN_2G_MODULATION;
			case SATIP_DVBC:
				u.info.type=FE_QAM;
				u.info.frequency_min=54000000;
				u.info.frequency_max=957000000;
				break;
			}

			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,&u.info,
				sizeof(struct dvb_frontend_info));
		}
		return;

	case FE_READ_STATUS:
		if(!out_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(fe_status_t);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,NULL,0,&iov,1);
		}
		else
		{
			pthread_spin_lock(&dev->spin);
			if(dev->status.lock)u.status=FE_HAS_SIGNAL|
				FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC|
				FE_HAS_LOCK;
			else u.status=0;
			pthread_spin_unlock(&dev->spin);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,&u.status,sizeof(fe_status_t));
		}
		return;

	case FE_READ_BER:
	case FE_READ_UNCORRECTED_BLOCKS:
		if(!out_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(uint32_t);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,NULL,0,&iov,1);
		}
		else
		{
			u.u32=0;
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,&u.u32,sizeof(uint32_t));
		}
		return;

	case FE_READ_SIGNAL_STRENGTH:
		if(!out_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(uint16_t);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,NULL,0,&iov,1);
		}
		else
		{
			pthread_spin_lock(&dev->spin);
			u.u16=dev->status.level<<8;
			pthread_spin_unlock(&dev->spin);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,&u.u16,sizeof(uint16_t));
		}
		return;

	case FE_READ_SNR:
		if(!out_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(uint16_t);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,NULL,0,&iov,1);
		}
		else
		{
			pthread_spin_lock(&dev->spin);
			u.u16=dev->status.quality<<12;
			pthread_spin_unlock(&dev->spin);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,&u.u16,sizeof(uint16_t));
		}
		return;

	case FE_DISEQC_SEND_BURST:
		switch(dev->conf.diseqc)
		{
		case SATIP_DSC_TONE:
		case SATIP_DSC_1_0_T:
		case SATIP_DSC_2_0_T:
		case SATIP_DSC_1_1_T:
		case SATIP_DSC_2_1_T:
			switch((int)((long)arg))
			{
			case SEC_MINI_A:
				dev->src&=~1;
				break;
			case SEC_MINI_B:
				dev->src|=1;
				break;
			}
		break;
		}
		pthread_rwlock_unlock(&dev->mtx);
		fuse_reply_ioctl(req,0,NULL,0);
		return;

	case FE_SET_FRONTEND_TUNE_MODE:
	case FE_DISEQC_RESET_OVERLOAD:
		pthread_rwlock_unlock(&dev->mtx);
		fuse_reply_ioctl(req,0,NULL,0);
		return;

	case FE_ENABLE_HIGH_LNB_VOLTAGE:
	case FE_DISHNETWORK_SEND_LEGACY_CMD:
		FAIL(EOPNOTSUPP);

	case FE_DISEQC_SEND_MASTER_CMD:
		if(!in_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dvb_diseqc_master_cmd);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,NULL,0);
		}
		else
		{
			u.cmd=(struct dvb_diseqc_master_cmd *)in_buf;

			if(u.cmd->msg_len<3||u.cmd->msg_len>6)FAIL(EINVAL);

			switch(dev->conf.diseqc)
			{
			case SATIP_DSC_1_0:
				if((u.cmd->msg[0]&0xfe)!=0xe0)break;
			case SATIP_DSC_2_0:
				dev->reply=0x00;
				if((u.cmd->msg[0]&0xfc)!=0xe0)break;
				switch(u.cmd->msg[2])
				{
				case 0x00:
					dev->reply=0xe4;
					break;
				case 0x22:
					dev->src|=1;
					dev->reply=0xe4;
					break;
				case 0x23:
					dev->src|=2;
					dev->reply=0xe4;
					break;
				case 0x26:
					dev->src&=~1;
					dev->reply=0xe4;
					break;
				case 0x27:
					dev->src&=~2;
					dev->reply=0xe4;
					break;
				case 0x38:
					if(u.cmd->msg_len==4)
					{
						dev->src&=~(u.cmd->msg[3]>>6);
						dev->src|=(u.cmd->msg[3]>>2)&3;
						dev->reply=0xe4;
						break;
					}
				default:dev->reply=0xe5;
					break;
				}
				if(!(u.cmd->msg[0]&2))dev->reply=0x00;
				break;

			case SATIP_DSC_1_1:
				if((u.cmd->msg[0]&0xfe)!=0xe0)break;
			case SATIP_DSC_2_1:
				if((u.cmd->msg[0]&0xfc)!=0xe0)break;
				switch(u.cmd->msg[2])
				{
				case 0x00:
				case 0x22:
				case 0x23:
				case 0x26:
				case 0x27:
				case 0x38:
					dev->reply=0xe4;
					break;
				case 0x39:
					if(u.cmd->msg_len==4)
					{
						dev->src&=~(u.cmd->msg[3]>>4);
						dev->src|=u.cmd->msg[3]&0xf;
						dev->reply=0xe4;
						break;
					}
				default:dev->reply=0xe5;
					break;
				}
				if(!(u.cmd->msg[0]&2))dev->reply=0x00;
				break;

			case SATIP_DSC_1_0_T:
				if((u.cmd->msg[0]&0xfe)!=0xe0)break;
			case SATIP_DSC_2_0_T:
				dev->reply=0x00;
				if((u.cmd->msg[0]&0xfc)!=0xe0)break;
				switch(u.cmd->msg[2])
				{
				case 0x00:
					dev->reply=0xe4;
					break;
				case 0x22:
					dev->src|=2;
					dev->reply=0xe4;
					break;
				case 0x23:
					dev->src|=4;
					dev->reply=0xe4;
					break;
				case 0x26:
					dev->src&=~2;
					dev->reply=0xe4;
					break;
				case 0x27:
					dev->src&=~4;
					dev->reply=0xe4;
					break;
				case 0x38:
					if(u.cmd->msg_len==4)
					{
						dev->src&=
							~((u.cmd->msg[3]>>5)&6);
						dev->src|=(u.cmd->msg[3]>>1)&6;
						dev->reply=0xe4;
						break;
					}
				default:dev->reply=0xe5;
					break;
				}
				if(!(u.cmd->msg[0]&2))dev->reply=0x00;
				break;

			case SATIP_DSC_1_1_T:
				if((u.cmd->msg[0]&0xfe)!=0xe0)break;
			case SATIP_DSC_2_1_T:
				if((u.cmd->msg[0]&0xfc)!=0xe0)break;
				switch(u.cmd->msg[2])
				{
				case 0x00:
				case 0x22:
				case 0x23:
				case 0x26:
				case 0x27:
				case 0x38:
					dev->reply=0xe4;
					break;
				case 0x39:
					if(u.cmd->msg_len==4)
					{
						dev->src&=
						    ~((u.cmd->msg[3]>>3)&0x1e);
						dev->src|=
						    (u.cmd->msg[3]<<1)&0x1e;
						dev->reply=0xe4;
						break;
					}
				default:dev->reply=0xe5;
					break;
				}
				if(!(u.cmd->msg[0]&2))dev->reply=0x00;
				break;
			}

			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,NULL,0);
		}
		return;

	case FE_SET_TONE:
		switch((int)((long)arg))
		{
		case SEC_TONE_ON:
			dev->hl=1;
			break;
		case SEC_TONE_OFF:
			dev->hl=0;
			break;
		default:FAIL(EINVAL);
		}
		pthread_rwlock_unlock(&dev->mtx);
		fuse_reply_ioctl(req,0,NULL,0);
		return;

	case FE_SET_VOLTAGE:
		switch((int)((long)arg))
		{
		case SEC_VOLTAGE_13:
		case SEC_VOLTAGE_OFF:
			dev->tune.pol=SATIP_POL_V;
			break;
		case SEC_VOLTAGE_18:
			dev->tune.pol=SATIP_POL_H;
			break;
		default:FAIL(EINVAL);
		}
		pthread_rwlock_unlock(&dev->mtx);
		fuse_reply_ioctl(req,0,NULL,0);
		return;

	case FE_DISEQC_RECV_SLAVE_REPLY:
		if(!out_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dvb_diseqc_slave_reply);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,NULL,0,&iov,1);
		}
		else switch(dev->conf.diseqc)
		{
		case SATIP_DSC_2_0:
		case SATIP_DSC_2_1:
		case SATIP_DSC_2_0_T:
		case SATIP_DSC_2_1_T:
			if(dev->reply)
			{
				u.reply.msg_len=1;
				u.reply.msg[0]=dev->reply;
				dev->reply=0;
				pthread_rwlock_unlock(&dev->mtx);
				fuse_reply_ioctl(req,0,&u.reply,
					sizeof(struct dvb_diseqc_slave_reply));
				break;
			}
		default:if(u.reply.timeout)FAIL(ETIMEDOUT);
			u.reply.msg_len=0;
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,&u.reply,
				sizeof(struct dvb_diseqc_slave_reply));
			break;
		}
		return;

	case FE_SET_FRONTEND:
		if(!in_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dvb_frontend_parameters);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,&iov,1,NULL,0);
		}
		else
		{
			u.pin=(struct dvb_frontend_parameters *)in_buf;

			switch(dev->conf.mode)
			{
			case SATIP_DVBS:
			case SATIP_DVBS2:
				if(checkfreq(u.pin->frequency*1000,
					dev->conf.lnb))FAIL(EINVAL);
				dev->freq=u.pin->frequency*1000;
				dev->tune.sr=u.pin->u.qpsk.symbol_rate;
				dev->tune.fec=satipfec(u.pin->u.qpsk.fec_inner);
				break;

			case SATIP_DVBT:
			case SATIP_DVBT2:
				dev->freq=u.pin->frequency;
				dev->tune.bw=satipbw(u.pin->u.ofdm.bandwidth);
				dev->tune.fec=
					satipfec(u.pin->u.ofdm.code_rate_HP);
				dev->tune.mtype=
					satipmtype(u.pin->u.ofdm.constellation);
				dev->tune.tmode=satiptmode(u.pin->u.ofdm.
					transmission_mode);
				dev->tune.gi=
					satipgi(u.pin->u.ofdm.guard_interval);
				break;

			case SATIP_DVBC:
			case SATIP_DVBC2:
				dev->freq=u.pin->frequency;
				dev->tune.sr=u.pin->u.qam.symbol_rate;
				dev->tune.fec=satipfec(u.pin->u.qam.fec_inner);
				dev->tune.mtype=
					satipmtype(u.pin->u.qam.modulation);
				break;
			}

			dev->enabled=1;
			dev->tunecnt++;
			dummy=write(dev->ffd,&dummy,sizeof(dummy));

			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,NULL,0);
		}
		return;

	case FE_GET_FRONTEND:
		if(!out_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dvb_frontend_parameters);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,NULL,0,&iov,1);
		}
		else
		{
			switch(dev->tune.specinv)
			{
			case SATIP_SPI_OFF:
				u.pout.inversion=INVERSION_OFF;
				break;
			case SATIP_SPI_ON:
				u.pout.inversion=INVERSION_ON;
				break;
			default:u.pout.inversion=INVERSION_AUTO;
				break;
			}

			switch(dev->conf.mode)
			{
			case SATIP_DVBS:
			case SATIP_DVBS2:
				u.pout.frequency=dev->freq/1000;
				u.pout.u.qpsk.symbol_rate=dev->tune.sr;
				u.pout.u.qpsk.fec_inner=fec(dev->tune.fec);
				break;

			case SATIP_DVBT:
			case SATIP_DVBT2:
				u.pout.frequency=(unsigned int)dev->freq;
				u.pout.u.ofdm.bandwidth=bw(dev->tune.bw);
				u.pout.u.ofdm.code_rate_HP=fec(dev->tune.fec);
				u.pout.u.ofdm.code_rate_LP=fec(dev->tune.fec);
				u.pout.u.ofdm.constellation=
					mtype(dev->tune.mtype);
				u.pout.u.ofdm.transmission_mode=
					tmode(dev->tune.tmode);
				u.pout.u.ofdm.guard_interval=gi(dev->tune.gi);
				u.pout.u.ofdm.hierarchy_information=
					HIERARCHY_AUTO;
				break;

			case SATIP_DVBC:
			case SATIP_DVBC2:
				u.pout.frequency=(unsigned int)dev->freq;
				u.pout.u.qam.symbol_rate=dev->tune.sr;
				u.pout.u.qam.fec_inner=fec(dev->tune.fec);
				u.pout.u.qam.modulation=mtype(dev->tune.mtype);
				break;
			}

			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,&u.pout,
				sizeof(struct dvb_frontend_parameters));
		}
		return;

	case FE_GET_EVENT:
		if(!out_bufsz)
		{
			iov.iov_base=arg;
			iov.iov_len=sizeof(struct dvb_frontend_event);
			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl_retry(req,NULL,0,&iov,1);
		}
		else
		{
			pthread_spin_lock(&dev->spin);
			if(dev->event<=0)
			{
				pthread_spin_unlock(&dev->spin);
				if((s->flags&O_NONBLOCK)||s->req)FAIL(EAGAIN);
				s->req=req;
				s->type=0;
				pthread_spin_lock(&dev->spin);
				s->notify++;
				pthread_spin_unlock(&dev->spin);
				fuse_req_interrupt_func(req,fe_notify,dev);
				return;
			}
			dev->event=-1;

			u.event.status=FE_HAS_SIGNAL|FE_HAS_CARRIER|
				FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;

			switch(dev->status.tune.specinv)
			{
			case SATIP_SPI_OFF:
				u.event.parameters.inversion=INVERSION_OFF;
				break;
			case SATIP_SPI_ON:
				u.event.parameters.inversion=INVERSION_ON;
				break;
			default:u.event.parameters.inversion=INVERSION_AUTO;
				break;
			}

			switch(dev->conf.mode)
			{
			case SATIP_DVBS:
			case SATIP_DVBS2:
				u.event.parameters.frequency=
					satfreq(&dev->status.tune,
						dev->conf.lnb)/1000;
				u.event.parameters.u.qpsk.symbol_rate=
					dev->status.tune.sr;
				u.event.parameters.u.qpsk.fec_inner=
					fec(dev->status.tune.fec);
				break;

			case SATIP_DVBT:
			case SATIP_DVBT2:
				u.event.parameters.frequency=(unsigned int)
					dev->status.tune.freq;
				u.event.parameters.u.ofdm.bandwidth=
					bw(dev->status.tune.bw);
				u.event.parameters.u.ofdm.code_rate_HP=
					fec(dev->status.tune.fec);
				u.event.parameters.u.ofdm.code_rate_LP=
					fec(dev->status.tune.fec);
				u.event.parameters.u.ofdm.constellation=
					mtype(dev->status.tune.mtype);
				u.event.parameters.u.ofdm.transmission_mode=
					tmode(dev->status.tune.tmode);
				u.event.parameters.u.ofdm.guard_interval=
					gi(dev->status.tune.gi);
				u.event.parameters.u.ofdm.hierarchy_information=
					HIERARCHY_AUTO;
				break;

			case SATIP_DVBC:
			case SATIP_DVBC2:
				u.event.parameters.frequency=(unsigned int)
					dev->status.tune.freq;
				u.event.parameters.u.qam.symbol_rate=
					dev->status.tune.sr;
				u.event.parameters.u.qam.fec_inner=
					fec(dev->status.tune.fec);
				u.event.parameters.u.qam.modulation=
					mtype(dev->status.tune.mtype);
				break;
			}

			pthread_spin_unlock(&dev->spin);

			pthread_rwlock_unlock(&dev->mtx);
			fuse_reply_ioctl(req,0,&u.event,
				sizeof(struct dvb_frontend_event));
		}
		return;
	}

fail:	pthread_rwlock_unlock(&dev->mtx);
	if(err==EAGAIN)usleep(2000);
	fuse_reply_err(req,err);
}

static void fe_poll(fuse_req_t req,struct fuse_file_info *fi,
	struct fuse_pollhandle *ph)
{
	STREAM *s=(STREAM *)fi->fh;
	DEVICE *dev=s->dev;
	struct fuse_pollhandle *pd=NULL;
	int err=0;
	int revents=0;

	pthread_rwlock_rdlock(&dev->mtx);
	pthread_mutex_lock(&s->mtx);

	if(!dev->running)FAIL(EBUSY);

	pthread_spin_lock(&dev->spin);

	if((s->flags&O_ACCMODE)!=O_RDONLY&&dev->event>0)revents=POLLIN;

	if(ph)
	{
		if(revents)
		{
			if(s->ph)
			{
				pd=s->ph;
				s->ph=NULL;
				s->notify--;
			}
		}
		else
		{
			if(s->ph)pd=s->ph;
			else s->notify++;
			s->ph=ph;
		}
	}
	else if(s->ph&&revents)
	{
		ph=s->ph;
		s->ph=NULL;
		s->notify--;
	}

	pthread_spin_unlock(&dev->spin);

fail:	pthread_mutex_unlock(&s->mtx);
	pthread_rwlock_unlock(&dev->mtx);
	if(err)fuse_reply_err(req,err);
	else
	{
		if(pd)fuse_pollhandle_destroy(pd);
		fuse_reply_poll(req,revents);
		if(revents&&ph)
		{
			fuse_lowlevel_notify_poll(ph);
			fuse_pollhandle_destroy(ph);
		}
	}
}

static const struct cuse_lowlevel_ops fe_ops=
{
	.init_done=fe_post,
	.open=fe_open,
	.read=fe_read,
	.write=fe_write,
	.flush=fe_flush,
	.release=fe_release,
	.fsync=fe_fsync,
	.ioctl=fe_ioctl,
	.poll=fe_poll,
};

static void *feworker(void *data)
{
	struct cuse_info ci;
	DEVICE *dev=(DEVICE *)data;
	char devpath[PATH_MAX+9];
	const char *devarg[1]={devpath};
	int unused;
	int dummy_argc=1;
	char *dummy_argv[66]={""};
	struct fuse_args args=FUSE_ARGS_INIT(dummy_argc,dummy_argv);
	uint64_t dummy=1;
	sigset_t set;

	sigfillset(&set);
	sigdelset(&set,SIGQUIT);
	pthread_sigmask(SIG_SETMASK,&set,NULL);

	pthread_setname_np(pthread_self(),"loopd feworker");

	if(fuse_opt_parse(&args,NULL,dummy_opts,dummy_args))goto out;
	if(fuse_opt_add_arg(&args, "-f"))goto out;

	sprintf(devpath,"DEVNAME=dvb/adapter%d/frontend0",dev->conf.adapter);
	memset(&ci,0,sizeof(ci));
	ci.dev_major=dev->conf.major;
	ci.dev_minor=dev->conf.minbase;
	ci.dev_info_argc=1;
	ci.dev_info_argv=devarg;
	ci.flags=CUSE_UNRESTRICTED_IOCTL;

	setfsuid(0);
	if(!(dev->fese=cuse_lowlevel_setup(args.argc,args.argv,&ci,&fe_ops,
		&unused,data)))
	{
		setfsuid(getuid());
		goto out;
	}
	setfsuid(getuid());
	fuse_session_loop_mt(dev->fese);
	fuse_session_destroy(dev->fese);

out:	if(!dev->dmxok)dummy=write(dev->ifd,&dummy,sizeof(dummy));
	fuse_opt_free_args(&args);

	pthread_exit(NULL);
}

static void *notifier(void *data)
{
	DEVICE *dev=(DEVICE *)data;
	DMXDTA *dmx;
	STREAM *s;
	STREAM **e;
	uint64_t dummy;
	struct pollfd p[3];
	int add;
	union
	{
		int l;
		sigset_t set;
		struct dvb_frontend_event e;
		struct fuse_pollhandle *pd;
	}u;

	sigfillset(&u.set);
	pthread_sigmask(SIG_BLOCK,&u.set,NULL);

	pthread_setname_np(pthread_self(),"loopd notifier");

	p[0].fd=dev->nfd;
	p[0].events=POLLIN;
	p[1].fd=dev->tfd;
	p[1].events=POLLIN;
	p[2].fd=dev->xfd;
	p[2].events=POLLIN;

	while(1)
	{
		if(poll(p,3,-1)<=0)continue;

		if(UNLIKELY(p[2].revents&POLLIN))break;

		pthread_rwlock_rdlock(&dev->mtx);

		if(UNLIKELY(p[1].revents&POLLIN))
		{
			dummy=read(dev->tfd,&dummy,sizeof(dummy));

			for(s=dev->list;s;s=s->next)
			{
				pthread_mutex_lock(&s->mtx);

				if(s->req&&s->type==2&&
					times(NULL)-s->start>s->dmx->timeout)
				{
					pthread_spin_lock(&dev->spin);
					s->notify--;
					pthread_spin_unlock(&dev->spin);
					del_timeout(dev);
					fuse_reply_err(s->req,ETIMEDOUT);
					s->req=NULL;
					pthread_mutex_unlock(&s->mtx);
				}
				else pthread_mutex_unlock(&s->mtx);
			}
		}

		if(UNLIKELY(!(p[0].revents&POLLIN)))
		{
			pthread_rwlock_unlock(&dev->mtx);
			pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);
			continue;
		}

		dummy=read(dev->nfd,&dummy,sizeof(dummy));

		pthread_spin_lock(&dev->spin);

		for(e=&dev->ehd;*e;)
		{
			s=*e;
			pthread_spin_unlock(&dev->spin);

			pthread_mutex_lock(&s->mtx);

			pthread_spin_lock(&dev->spin);

			if(s->ph)
			{
				s->notify--;
				pthread_spin_unlock(&dev->spin);
				fuse_lowlevel_notify_poll(s->ph);
				fuse_pollhandle_destroy(s->ph);
				s->ph=NULL;
			}
			else pthread_spin_unlock(&dev->spin);

			if(!s->req);
			else if(fuse_req_interrupted(s->req))
			{
				pthread_spin_lock(&dev->spin);
				s->notify--;
				pthread_spin_unlock(&dev->spin);
				fuse_reply_err(s->req,EINTR);
				s->req=NULL;
			}
			else switch(s->type)
			{
			case 0:	pthread_spin_lock(&dev->spin);
				dev->event=-1;
				s->notify--;

				u.e.status=FE_HAS_SIGNAL|FE_HAS_CARRIER|
					FE_HAS_VITERBI|FE_HAS_SYNC|FE_HAS_LOCK;

				switch(dev->status.tune.specinv)
				{
				case SATIP_SPI_OFF:
					u.e.parameters.inversion=INVERSION_OFF;
					break;
				case SATIP_SPI_ON:
					u.e.parameters.inversion=INVERSION_ON;
					break;
				default:u.e.parameters.inversion=INVERSION_AUTO;
					break;
				}

				switch(dev->conf.mode)
				{
				case SATIP_DVBS:
				case SATIP_DVBS2:
					u.e.parameters.frequency=
						satfreq(&dev->status.tune,
							dev->conf.lnb)/1000;
					u.e.parameters.u.qpsk.symbol_rate=
						dev->status.tune.sr;
					u.e.parameters.u.qpsk.fec_inner=
						fec(dev->status.tune.fec);
					break;

				case SATIP_DVBT:
				case SATIP_DVBT2:
					u.e.parameters.frequency=(unsigned int)
						dev->status.tune.freq;
					u.e.parameters.u.ofdm.bandwidth=
						bw(dev->status.tune.bw);
					u.e.parameters.u.ofdm.code_rate_HP=
						fec(dev->status.tune.fec);
					u.e.parameters.u.ofdm.code_rate_LP=
						fec(dev->status.tune.fec);
					u.e.parameters.u.ofdm.constellation=
						mtype(dev->status.tune.mtype);
					u.e.parameters.u.ofdm.transmission_mode=
						tmode(dev->status.tune.tmode);
					u.e.parameters.u.ofdm.guard_interval=
						gi(dev->status.tune.gi);
					u.e.parameters.u.ofdm.
						hierarchy_information=
							HIERARCHY_AUTO;
					break;

				case SATIP_DVBC:
				case SATIP_DVBC2:
					u.e.parameters.frequency=(unsigned int)
						dev->status.tune.freq;
					u.e.parameters.u.qam.symbol_rate=
						dev->status.tune.sr;
					u.e.parameters.u.qam.fec_inner=
						fec(dev->status.tune.fec);
					u.e.parameters.u.qam.modulation=
						mtype(dev->status.tune.mtype);
					break;
				}
				pthread_spin_unlock(&dev->spin);

				fuse_reply_ioctl(s->req,0,&u.e,
					sizeof(struct dvb_frontend_event));
				s->req=NULL;
				break;

			case 1:	pthread_spin_lock(&dev->spin);
				s->notify--;

				u.l=(dev->dvrfill>s->size?s->size:dev->dvrfill);
				if(dev->dvrtail>=dev->dvrhead)
					if(u.l>dev->dvrbufsize-dev->dvrtail)
					    u.l=dev->dvrbufsize-dev->dvrtail;

				pthread_spin_unlock(&dev->spin);
				fuse_reply_buf(s->req,dev->dvrbfr+dev->dvrtail,
					u.l);
				s->req=NULL;
				pthread_spin_lock(&dev->spin);

				if((dev->dvrtail+=u.l)==dev->dvrbufsize)
					dev->dvrtail=0;
				dev->dvrfill-=u.l;
				goto next;

			case 2:
			case 3:	dmx=s->dmx;

				pthread_spin_lock(&dev->spin);
				s->notify--;

				if(dmx->ftype==4)
				{
					add=2;
					u.l=(dmx->bfr[dmx->tail]<<8)|
						dmx->bfr[dmx->tail+1];
					if(u.l>s->size)
					{
						if((dmx->tail+=u.l+add)==
						    dmx->bufsize)dmx->tail=0;
						dmx->fill-=u.l+add;
						pthread_spin_unlock(&dev->spin);
						fuse_reply_err(s->req,
							EOVERFLOW);
						s->req=NULL;
						break;
					}
				}
				else
				{
					add=0;
					u.l=(dmx->fill>s->size?
						s->size:dmx->fill);
					if(dmx->tail>=dmx->head)
						if(u.l>dmx->bufsize-dmx->tail)
						    u.l=dmx->bufsize-dmx->tail;
				}

				pthread_spin_unlock(&dev->spin);
				if(s->type==2)del_timeout(dev);
				fuse_reply_buf(s->req,dmx->bfr+dmx->tail+add,
					u.l);
				s->req=NULL;
				pthread_spin_lock(&dev->spin);

				if((dmx->tail+=u.l+add)==dmx->bufsize)
					dmx->tail=0;
				dmx->fill-=u.l+add;
				goto next;
			}

			pthread_spin_lock(&dev->spin);
next:			if(!s->notify)
			{
				s->event=0;
				*e=s->enx;
				if(!dev->ehd)dev->etl=NULL;
			}
			else e=&s->enx;
			pthread_mutex_unlock(&s->mtx);
		}

		pthread_spin_unlock(&dev->spin);
		pthread_rwlock_unlock(&dev->mtx);
	}

	pthread_exit(NULL);
}

static int dotune(DEVICE *dev)
{
	int i;
	int curr;
	void *sh;
	char *host=NULL;
	SATIP_CLN_UPNPLIST *l=NULL;
	SATIP_CLN_UPNPLIST *e;
	SATIP_CLN_UPNPLIST *r;
	SATIP_EXTRA *extra=NULL;
	SATIP_STRDATA std;

	curr=dev->feedertune.fe;

	dev->feedertune.fe=SATIP_UNSPEC;

	if(dev->plug.pre_tune)dev->plug.pre_tune(dev->plug.ctx,&dev->feedertune,
		&extra);

	if(!dev->conf.local)
	{
		if(dev->conf.uuid[0])
		{
			if(!satip_cln_upnplist(dev->conf.h,&l))
				for(e=l;e;e=e->next)
					if(!strcmp(e->uuid,dev->conf.uuid))
			{
				for(r=e,e=e->same;e;e=e->same)
					if(e->level>r->level)r=e;
				host=r->addr;
				break;
			}
		}
		else host=dev->conf.host;

		if(!host)goto fail;
	}

	if(curr!=SATIP_UNSPEC||!dev->conf.fetotal)
	{
		dev->feedertune.fe=curr;

		if(dev->conf.local)
		{
			std.tune=&dev->feedertune;
			std.set=&dev->feederset;
			std.terminate=dev->xfd;
			std.handle=dev;
			pthread_mutex_lock(&dev->cmtx);
			if(!satip_hw_play(dev->conf.h,&std,hwstream,hwstatus,
				NULL,SATIP_HW_REMOTE))
			{
				dev->feid=dev->feedertune.fe;
				pthread_mutex_unlock(&dev->cmtx);
				sh=std.handle;
				goto out;
			}
			pthread_mutex_unlock(&dev->cmtx);
		}
		else if(!satip_cln_stream_unicast(dev->conf.h,host,
			dev->conf.port,1,&dev->feedertune,&dev->feederset,extra,
			streamer,dev,&sh))goto out;
	}

	for(i=0;i<dev->conf.fetotal;i++)if(dev->conf.felist[i]!=curr)
	{
		dev->feedertune.fe=dev->conf.felist[i];

		if(dev->conf.local)
		{
			std.tune=&dev->feedertune;
			std.set=&dev->feederset;
			std.terminate=dev->xfd;
			std.handle=dev;
			pthread_mutex_lock(&dev->cmtx);
			if(!satip_hw_play(dev->conf.h,&std,hwstream,hwstatus,
				NULL,SATIP_HW_REMOTE))
			{
				dev->feid=dev->feedertune.fe;
				pthread_mutex_unlock(&dev->cmtx);
				sh=std.handle;
				goto out;
			}
			pthread_mutex_unlock(&dev->cmtx);
		}
		else if(!satip_cln_stream_unicast(dev->conf.h,host,
			dev->conf.port,1,&dev->feedertune,&dev->feederset,extra,
			streamer,dev,&sh))goto out;
	}

fail:	dev->feedertune.fe=SATIP_UNSPEC;
	if(l)satip_cln_freelist(dev->conf.h,l);
	if(dev->plug.post_tune)dev->plug.post_tune(dev->plug.ctx,
		&dev->feedertune,0);
	return -1;

out:	dev->sh=sh;
	if(l)satip_cln_freelist(dev->conf.h,l);
	if(dev->plug.post_tune)dev->plug.post_tune(dev->plug.ctx,
		&dev->feedertune,1);
	return 0;
}

static void chgtune(DEVICE *dev)
{
	SATIP_EXTRA *extra;
	SATIP_STRDATA std;

	extra=NULL;
	if(dev->plug.pre_pids)dev->plug.pre_pids(dev->plug.ctx,&extra);
	if(dev->conf.local)
	{
		std.tune=&dev->feedertune;
		std.set=&dev->feederset;
		std.terminate=dev->xfd;
		std.handle=dev->sh;
		if(satip_hw_setpids(dev->conf.h,&std))goto fail;
	}
	else if(satip_cln_change_unicast(dev->conf.h,dev->sh,&dev->feederset,
		NULL,NULL,extra))
	{
fail:		if(dev->plug.post_pids)dev->plug.post_pids(dev->plug.ctx,0);
		return;
	}
	else if(dev->plug.post_pids)dev->plug.post_pids(dev->plug.ctx,1);
}

static void endtune(DEVICE *dev)
{
	SATIP_STRDATA std;

	if(dev->conf.local)
	{
		std.tune=&dev->feedertune;
		std.set=&dev->feederset;
		std.terminate=dev->xfd;
		std.handle=dev->sh;
		satip_hw_end(dev->conf.h,&std);
	}
	else satip_cln_stream_stop(dev->conf.h,dev->sh);
	dev->sh=NULL;
	if(dev->plug.no_tune)dev->plug.no_tune(dev->plug.ctx);
}

static void *feeder(void *data)
{
	DEVICE *dev=(DEVICE *)data;
	STREAM **e;
	unsigned int tunecnt=0;
	int op;
	int i;
	int delay=0;
	int leaky=0;
	int enabled=0;
	int currfe=SATIP_UNSPEC;
	uint64_t dummy;
	struct pollfd p[3];
	struct itimerspec it;
	sigset_t sset;

	sigfillset(&sset);
	pthread_sigmask(SIG_BLOCK,&sset,NULL);

	pthread_setname_np(pthread_self(),"loopd feeder");

	p[0].fd=dev->ffd;
	p[0].events=POLLIN;
	p[1].fd=dev->xfd;
	p[1].events=POLLIN;
	p[2].fd=dev->cfd;
	p[2].events=POLLIN;

	satip_util_init_pids(&dev->feederset);
	memset(&it,0,sizeof(it));

	while(1)
	{
		op=0;

		while(poll(p,3,-1)<=0);

		if(UNLIKELY(p[1].revents&POLLIN))break;

		if(p[2].revents&POLLIN)
		{
			memset(&it,0,sizeof(it));
			timerfd_settime(dev->cfd,0,&it,NULL);
			dummy=read(dev->cfd,&dummy,sizeof(dummy));
			if(delay==1)op=4;
			else if(delay==2)delay=0;
		}

		if(p[0].revents&POLLIN)
			dummy=read(p[0].fd,&dummy,sizeof(dummy));

		pthread_rwlock_wrlock(&dev->mtx);

		switch(dev->enabled)
		{
		case 0:	if(dev->sh)op=1;
			else op=0;
			delay=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(dev->cfd,0,&it,NULL);
			dummy=read(dev->cfd,&dummy,sizeof(dummy));
			break;

		case 1:	switch(dev->conf.mode)
			{
			case SATIP_DVBS:
			case SATIP_DVBS2:
				dev->tune.freq=satipfreq(dev->freq,
					dev->conf.lnb,dev->tune.pol,dev->hl);
				dev->tune.src=dev->src;
				switch(dev->conf.diseqc)
				{
				case SATIP_DSC_NONE:
					dev->tune.src=0;
					break;
				case SATIP_DSC_TONE:
					dev->tune.src&=1;
					break;
				case SATIP_DSC_1_0:
				case SATIP_DSC_2_0:
					if(dev->conf.dscbits==1)
						dev->tune.src&=1;
					else dev->tune.src&=3;
					break;
				case SATIP_DSC_1_0_T:
				case SATIP_DSC_2_0_T:
					if(dev->conf.dscbits==1)
						dev->tune.src&=3;
					else dev->tune.src&=7;
					break;
				case SATIP_DSC_1_1:
				case SATIP_DSC_2_1:
					switch(dev->conf.dscbits)
					{
					case 1:	dev->tune.src&=1;
						break;
					case 2:	dev->tune.src&=3;
						break;
					case 3:	dev->tune.src&=7;
						break;
					default:dev->tune.src&=15;
						break;
					}
					break;
				case SATIP_DSC_1_1_T:
				case SATIP_DSC_2_1_T:
					switch(dev->conf.dscbits)
					{
					case 1:	dev->tune.src&=3;
						break;
					case 2:	dev->tune.src&=7;
						break;
					case 3:	dev->tune.src&=15;
						break;
					default:dev->tune.src&=31;
						break;
					}
					break;
				default:dev->tune.src=0;
					break;
				}
				dev->tune.src+=dev->conf.src;
				if(dev->tune.src>255)dev->tune.src=255;
				break;

			default:dev->tune.freq=dev->freq;
				break;
			}

			if(delay==2)
			{
				pthread_rwlock_unlock(&dev->mtx);
				continue;
			}

			if(!dev->sh)
			{
				op=2;
				dev->feedertune=dev->tune;
				dev->feederset=dev->set;
				tunecnt=dev->tunecnt;
			}
			else if(satip_util_tunecmp(&dev->feedertune,&dev->tune))
			{
				op=3;
				dev->feedertune=dev->tune;
				dev->feederset=dev->set;
				tunecnt=dev->tunecnt;
			}
			else
			{
				if(dev->feederset.numpids==SATIP_NOPIDS&&
					dev->set.numpids==SATIP_NOPIDS)
						goto chktune;
				if(dev->feederset.numpids==SATIP_ALLPIDS&&
					dev->set.numpids==SATIP_ALLPIDS)
						goto chktune;

				if(++leaky==SATIP_MAX_PIDS)op=4;
				else if(!op)op=5;
				dev->feederset=dev->set;

chktune:			if(!op&&delay==1)op=4;
				if(tunecnt!=dev->tunecnt)
				{
					tunecnt=dev->tunecnt;
					pthread_spin_lock(&dev->spin);
					dev->event=0;
					if(dev->feopen)for(e=&dev->ehd;*e;
						e=&(*e)->enx)if(*e==dev->fe)
					{
						dev->fe->event=0;
						*e=dev->fe->enx;
						if(!dev->ehd)dev->etl=NULL;
						break;
					}
					pthread_spin_unlock(&dev->spin);
				}
			}
			break;
		}

		pthread_rwlock_unlock(&dev->mtx);

		switch(op)
		{
		case 2:
		case 3:
		case 4:	for(i=0;i<dev->conf.filtertotal;i++)
				if(!satip_util_tunecmp(&dev->feedertune,
					&dev->conf.tunefilter[i]))
			{
				dev->feederset.numpids=SATIP_NOPIDS;
				break;
			}
			break;
		}

		switch(op)
		{
		case 1:	endtune(dev);
			currfe=dev->feedertune.fe=SATIP_UNSPEC;
			enabled=0;
			delay=0;
			leaky=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(dev->cfd,0,&it,NULL);
			dummy=read(dev->cfd,&dummy,sizeof(dummy));
			pthread_rwlock_wrlock(&dev->mtx);
			pthread_spin_lock(&dev->spin);
			dev->event=0;
			if(dev->feopen)for(e=&dev->ehd;*e;e=&(*e)->enx)
				if(*e==dev->fe)
			{
				dev->fe->event=0;
				*e=dev->fe->enx;
				if(!dev->ehd)dev->etl=NULL;
				break;
			}
			memset(&dev->status,0,sizeof(dev->status));
			pthread_spin_unlock(&dev->spin);
			pthread_rwlock_unlock(&dev->mtx);
			break;

		case 2:	delay=0;
			leaky=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(dev->cfd,0,&it,NULL);
			dummy=read(dev->cfd,&dummy,sizeof(dummy));
			pthread_rwlock_wrlock(&dev->mtx);
			pthread_spin_lock(&dev->spin);
			dev->event=0;
			if(dev->feopen)for(e=&dev->ehd;*e;e=&(*e)->enx)
				if(*e==dev->fe)
			{
				dev->fe->event=0;
				*e=dev->fe->enx;
				if(!dev->ehd)dev->etl=NULL;
				break;
			}
			memset(&dev->status,0,sizeof(dev->status));
			pthread_spin_unlock(&dev->spin);
			pthread_rwlock_unlock(&dev->mtx);

			currfe=dev->feedertune.fe=SATIP_UNSPEC;
			if(!dotune(dev))
			{
				currfe=dev->feedertune.fe;
				enabled=1;
			}
			else
			{
				delay=2;
				it.it_value.tv_sec=5;
				timerfd_settime(dev->cfd,0,&it,NULL);
			}
			dev->feedertune.fe=SATIP_UNSPEC;
			break;

		case 3:	endtune(dev);
			dev->feedertune.fe=currfe;
			currfe=SATIP_UNSPEC;
			enabled=0;
			delay=0;
			leaky=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(dev->cfd,0,&it,NULL);
			dummy=read(dev->cfd,&dummy,sizeof(dummy));
			pthread_rwlock_wrlock(&dev->mtx);
			pthread_spin_lock(&dev->spin);
			dev->event=0;
			if(dev->feopen)for(e=&dev->ehd;*e;e=&(*e)->enx)
				if(*e==dev->fe)
			{
				dev->fe->event=0;
				*e=dev->fe->enx;
				if(!dev->ehd)dev->etl=NULL;
				break;
			}
			memset(&dev->status,0,sizeof(dev->status));
			pthread_spin_unlock(&dev->spin);
			pthread_rwlock_unlock(&dev->mtx);
			if(!dotune(dev))
			{
				currfe=dev->feedertune.fe;
				enabled=1;
			}
			else
			{
				delay=2;
				it.it_value.tv_sec=5;
				timerfd_settime(dev->cfd,0,&it,NULL);
			}
			dev->feedertune.fe=SATIP_UNSPEC;
			break;

		case 4:	delay=0;
			leaky=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(dev->cfd,0,&it,NULL);
			dummy=read(dev->cfd,&dummy,sizeof(dummy));
			if(!enabled)break;
			chgtune(dev);
			break;

		case 5:	delay=1;
			memset(&it,0,sizeof(it));
			timerfd_settime(dev->cfd,0,&it,NULL);
			dummy=read(dev->cfd,&dummy,sizeof(dummy));
			it.it_value.tv_nsec=10000000;
			timerfd_settime(dev->cfd,0,&it,NULL);
			break;
		}
	}

	pthread_exit(NULL);
}

static void set_plugin_pids(void *ctx,SATIP_PIDS *set,int from_notify_hook)
{
	DEVICE *dev=(DEVICE *)ctx;

	if(from_notify_hook)
	{
		dev->plug.set=*set;
		return;
	}

	pthread_rwlock_wrlock(&dev->mtx);

	dev->plug.set=*set;
	if(!dev->plug.set.numpids)dev->plug.set.numpids=SATIP_NOPIDS;
	update_set(dev,0);

	pthread_rwlock_unlock(&dev->mtx);
}

static int camhandler(void *ctx,void *handle,SATIP_HW_CAM_IO *msg)
{
	DEVICE *dev=(DEVICE *)ctx;

	pthread_mutex_lock(&dev->cmtx);
	if(dev->feid==msg->deviceid)
	{
		if(msg->type==SATIP_CAM_STATE&&!msg->state)dev->feid=0;
		pthread_mutex_unlock(&dev->cmtx);
		if(dev->plug.cam)dev->plug.cam(dev->plug.ctx,handle,msg);
		return 0;
	}
	else pthread_mutex_unlock(&dev->cmtx);

	return -1;
}

static void xruncpy(XRUN *dst,void *ctx)
{
	DEVICE *dev=(DEVICE *)ctx;

	pthread_spin_unlock(&dev->spin);
	*dst=dev->xrun;
	pthread_spin_unlock(&dev->spin);
}

static void tunedump(void *ctx,int idx,int strict)
{
	int enabled;
	DEVICE *dev=(DEVICE *)ctx;
	SATIP_TUNE tune;
	SATIP_PIDS set;
	char bfr[512];

	pthread_rwlock_wrlock(&dev->mtx);
	enabled=dev->enabled;
	tune=dev->tune;
	set=dev->set;
	pthread_rwlock_unlock(&dev->mtx);

	if(!enabled)fprintf(stderr,"Device %d: not tuning\n",idx+1);
	else if(satip_util_create(SATIP_TYPE_QRY,strict?SATIP_STRICTQRY:0,NULL,
		0,0,&tune,&set,NULL,NULL,NULL,bfr,sizeof(bfr))<=0)
			fprintf(stderr,"Device %d: <unknown>\n",idx+1);
	else fprintf(stderr,"Device %d: %s\n",idx+1,bfr+1);
}

static void *create(CONFIG *config)
{
	DEVICE *dev;
	int i;
	uint64_t dummy;
	struct stat stb;
	pthread_attr_t attr;
	char bfr[PATH_MAX];

	if(!config)goto err1;

	if(config->adapter<0||config->adapter>255||config->major<0||
		config->major>0x7fff||config->minbase<0||config->minbase>0x7fff)
			goto err1;

	if(config->minbase&3)goto err1;

	if(stat("/dev/cuse",&stb)||!S_ISCHR(stb.st_mode))goto err1;

	sprintf(bfr,"/dev/dvb/adapter%d/frontend0",config->adapter);
	if(!stat(bfr,&stb))goto err1;

	sprintf(bfr,"/dev/dvb/adapter%d/demux0",config->adapter);
	if(!stat(bfr,&stb))goto err1;

	sprintf(bfr,"/dev/dvb/adapter%d/dvr0",config->adapter);
	if(!stat(bfr,&stb))goto err1;

	if(!(dev=malloc(sizeof(DEVICE))))goto err1;
	memset(dev,0,sizeof(DEVICE));
	satip_util_init_pids(&dev->plug.set);
	dev->conf=*config;
	dev->ticks=(sysconf(_SC_CLK_TCK)*2)/100;

	if(dev->conf.plugin[0])
	{
		if(!(dev->dlh=dlopen(dev->conf.plugin,RTLD_NOW)))goto err2;
		dev->plug.init=dlsym(dev->dlh,"plugin_init");
		dev->plug.exit=dlsym(dev->dlh,"plugin_exit");
		dev->plug.notify_pids=dlsym(dev->dlh,"plugin_notify_pids");
		dev->plug.pre_pids=dlsym(dev->dlh,"plugin_pre_pids");
		dev->plug.post_pids=dlsym(dev->dlh,"plugin_post_pids");
		dev->plug.pre_tune=dlsym(dev->dlh,"plugin_pre_tune");
		dev->plug.post_tune=dlsym(dev->dlh,"plugin_post_tune");
		dev->plug.no_tune=dlsym(dev->dlh,"plugin_no_tune");
		dev->plug.strinit=dlsym(dev->dlh,"plugin_strinit");
		dev->plug.strexit=dlsym(dev->dlh,"plugin_strexit");
		dev->plug.stream=dlsym(dev->dlh,"plugin_stream");
		dev->plug.multi=dlsym(dev->dlh,"plugin_multi");
		dev->plug.cam=dlsym(dev->dlh,"plugin_cam");
	}

	if(!(dev->dvrbfr=malloc(dev->conf.bufsize)))goto err2;
	dev->dvrbufsize=dev->conf.bufsize;
	dev->dvrthres=(dev->dvrbufsize>>1)+(dev->dvrbufsize>>2);

	if(pthread_rwlock_init(&dev->mtx,NULL))goto err3;

	if(pthread_mutex_init(&dev->tmtx,NULL))goto err4;

	if(pthread_mutex_init(&dev->cmtx,NULL))goto err5;

	if(pthread_spin_init(&dev->spin,PTHREAD_PROCESS_PRIVATE))goto err6;

	if((dev->nfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err7;

	if((dev->ffd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err8;

	if((dev->xfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err9;

	if((dev->tfd=timerfd_create(CLOCK_MONOTONIC,TFD_CLOEXEC|TFD_NONBLOCK))
		==-1)goto err10;

	if((dev->cfd=timerfd_create(CLOCK_MONOTONIC,TFD_CLOEXEC|TFD_NONBLOCK))
		==-1)goto err11;

	if((dev->ifd=eventfd(0,EFD_CLOEXEC|EFD_SEMAPHORE))==-1)goto err12;

	if(satip_util_filter_create(&dev->filter))goto err13;

	if(pthread_attr_init(&attr))goto err14;

	if(pthread_attr_setstacksize(&attr,STACKSIZE))goto err15;

	for(i=0;i<5;i++)switch(i)
	{
	case 0:	if(pthread_create(&dev->th[0],&attr,feeder,dev))goto err16;
		break;

	case 1:	if(pthread_create(&dev->th[1],&attr,notifier,dev))goto err16;
		break;

	case 2:	if(pthread_create(&dev->th[2],&attr,feworker,dev))goto err16;
		break;

	case 3:	if(pthread_create(&dev->th[3],&attr,dmxworker,dev))goto err16;
		break;

	case 4:	if(pthread_create(&dev->th[4],&attr,dvrworker,dev))goto err16;
		break;
	}

	for(i=0;i<3;i++)dummy=read(dev->ifd,&dummy,sizeof(dummy));
	if(!dev->dvrok||!dev->dmxok||!dev->feok)goto err16;

	if(dev->plug.init)if(!(dev->plug.ctx=dev->plug.init(
		dev->conf.plugcfg[0]?dev->conf.plugcfg:NULL,
		dev->filter,set_plugin_pids,dev)))goto err16;

	close(dev->ifd);

	pthread_attr_destroy(&attr);

	pthread_rwlock_wrlock(&dev->mtx);
	dev->running=1;
	pthread_rwlock_unlock(&dev->mtx);

	return dev;

err16:	for(i--;i>=0;i--)switch(i)
	{
	case 4:	fuse_session_exit(dev->dvrse);
		pthread_kill(dev->th[4],SIGHUP);
		break;
	case 3:	fuse_session_exit(dev->dmxse);
		pthread_kill(dev->th[3],SIGHUP);
		break;
	case 2:	fuse_session_exit(dev->fese);
		pthread_kill(dev->th[2],SIGHUP);
		break;
	case 1:
	case 0:	pthread_cancel(dev->th[i]);
		pthread_join(dev->th[i],NULL);
		break;
	}
	if(dev->sh)endtune(dev);
err15:	pthread_attr_destroy(&attr);
err14:	satip_util_filter_free(dev->filter);
err13:	close(dev->ifd);
err12:	close(dev->cfd);
err11:	close(dev->tfd);
err10:	close(dev->xfd);
err9:	close(dev->ffd);
err8:	close(dev->nfd);
err7:	pthread_spin_destroy(&dev->spin);
err6:	pthread_mutex_destroy(&dev->cmtx);
err5:	pthread_mutex_destroy(&dev->tmtx);
err4:	pthread_rwlock_destroy(&dev->mtx);
err3:	free(dev->dvrbfr);
err2:	if(dev->dlh)dlclose(dev->dlh);
	free(dev);
err1:	return NULL;
}

static void destroy(void *ctx)
{
	int i;
	DEVICE *dev=(DEVICE *)ctx;
	STREAM *s;
	uint64_t dummy=1;

	if(!dev)return;

	pthread_rwlock_wrlock(&dev->mtx);
	dev->running=0;
	pthread_rwlock_unlock(&dev->mtx);

	dummy=write(dev->xfd,&dummy,sizeof(dummy));
	for(i=1;i>=0;i--)pthread_join(dev->th[i],NULL);

	if(dev->sh)endtune(dev);

	pthread_rwlock_wrlock(&dev->mtx);

	for(s=dev->s;s;s=s->next)
	{
		if(s->ph)fuse_pollhandle_destroy(s->ph);
		if(s->req)fuse_reply_err(s->req,EBUSY);
		if(s->dmx)
		{
			if(s->dmx->usrflt)
				satip_util_filter_del_user(dev->filter,
					s->dmx->usrflt);
			if(s->dmx->secflt)
				satip_util_section_free(s->dmx->secflt);
			if(s->dmx->open&&s->dmx->running&&dev->plug.strexit)
				switch(s->dmx->ftype)
			{
			case 2:
			case 3:	dev->plug.strexit(dev->plug.ctx,
					s->dmx->plugdta);
			}
		}
	}

	if(dev->dvropen&&dev->dvrrun&&dev->plug.strexit)
			dev->plug.strexit(dev->plug.ctx,dev->plugdta);

	pthread_rwlock_unlock(&dev->mtx);

	if(dev->plug.exit)dev->plug.exit(dev->plug.ctx);

	fuse_session_exit(dev->dvrse);
	fuse_session_exit(dev->dmxse);
	fuse_session_exit(dev->fese);
	for(i=4;i>=2;i--)pthread_kill(dev->th[i],SIGQUIT);
	for(i=4;i>=2;i--)pthread_join(dev->th[i],NULL);

	while(dev->s)
	{
		s=dev->s;
		dev->s=s->next;
		if(s->dmx&&s->dmx->bfr)free(s->dmx->bfr);
		pthread_mutex_destroy(&s->mtx);
		free(s);
	}
	if(dev->dvrbfr)free(dev->dvrbfr);
	freepid(dev);
	satip_util_filter_free(dev->filter);

	close(dev->cfd);
	close(dev->tfd);
	close(dev->xfd);
	close(dev->ffd);
	close(dev->nfd);
	pthread_spin_destroy(&dev->spin);
	pthread_mutex_destroy(&dev->cmtx);
	pthread_mutex_destroy(&dev->tmtx);
	pthread_rwlock_destroy(&dev->mtx);

	if(dev->dlh)dlclose(dev->dlh);
	free(dev);
}

static void handler(int unused)
{
}

int main(int argc,char *argv[])
{
	int err=1;
	int f=1;
	char *cfg="/etc/satiploopd.conf";
	void *h=NULL;
	void *hh=NULL;
	struct pollfd p[3];
	void *dh[MAX_DEVICES];
	union
	{
		struct itimerspec ic;
		struct tm tm;
		uint64_t dummy;
		struct signalfd_siginfo sig;
		struct sigaction a;
		sigset_t set;
		SATIP_CFGINFO hwinfo;
		MSG *m;
		XRUN xcpy;
	}u;
	union
	{
		int c;
		char data[256];
		struct
		{
			union
			{
				time_t tme;
				char bfr[128];
			};
			XRUN mem;
		}xrun;
	}v;
	GLOBAL g;
	CAMQUEUE q;
	SATIP_HW_CONFIG hwconf;
	XRUN xrun[MAX_DEVICES];
	char fusestack[32];

	while((v.c=getopt(argc,argv,"c:dxh"))!=-1)switch(v.c)
	{
	case 'c':
		cfg=optarg;
		break;

	case 'd':
		f&=~1;
		break;

	case 'x':
		f|=2;
		break;

	case 'h':
	default:fprintf(stderr,"Usage: satiploopd [-c <config-file>] [-d] [-x]"
			" [-h]\n-c <config-file>   configuration file, default"
			" /etc/satiploopd.conf\n-d                 daemonize\n"
			"-x                 print overrun statistics (disables"
			" '-d')\n-h                 this help\n");
		return 1;
	}

	memset(&g,0,sizeof(GLOBAL));
	memset(&q,0,sizeof(CAMQUEUE));

	if(parse_config(cfg,&g))goto err1;

	if(setpriv(g.rtprio))goto err1;

	snprintf(fusestack,sizeof(fusestack),"FUSE_THREAD_STACK=%d",STACKSIZE);
	putenv(fusestack);

	if(pthread_mutex_init(&q.mtx,NULL))
	{
		perror("pthread_mutex_init");
		goto err1;
	}

	if((q.efd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK|EFD_SEMAPHORE))==-1)
	{
		perror("eventfd");
		goto err2;
	}

	if((p[2].fd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))
		==-1)
	{
		perror("timerfd");
		goto err3;
	}

	sigfillset(&u.set);
	if((p[0].fd=signalfd(-1,&u.set,SFD_NONBLOCK|SFD_CLOEXEC))==-1)
	{
		perror("signalfd");
		goto err4;
	}
	sigprocmask(SIG_BLOCK,&u.set,NULL);
	memset(&u.a,0,sizeof(u.a));
	u.a.sa_handler=handler;
	sigaction(SIGQUIT,&u.a,NULL);

	p[0].events=POLLIN;
	p[1].fd=q.efd;
	p[1].events=POLLIN;
	p[2].events=POLLIN;

	if(!f)if(daemon(0,0))
	{
		perror("daemon");
		goto err5;
	}

	if(g.nnum)if(!(h=satip_cln_init(&g.cln)))
	{
		fprintf(stderr,"satip_cln_init failed\n");
		goto err6;
	}

	if(g.tnum)
	{
		memset(&hwconf,0,sizeof(SATIP_HW_CONFIG));
		hwconf.camfunc=camcb;
		hwconf.priv=&q;
		hwconf.rtprio=g.rtprio;
		hwconf.stack=STACKSIZE;

		if(!(hh=satip_hw_init(&hwconf,&u.hwinfo)))
		{
			fprintf(stderr,"satip_hw_init failure\n");
			goto err6;
		}

		for(v.c=0;v.c<g.tnum;v.c++)
			if(satip_hw_add(hh,&g.tuner[v.c],&u.hwinfo))
		{
			fprintf(stderr,"satip_hw_add failure\n");
			goto err7;
		}
	}

	for(v.c=0;v.c<g.dnum;v.c++)
	{
		if(g.dev[v.c].local)g.dev[v.c].h=hh;
		else g.dev[v.c].h=h;

		if(!(dh[v.c]=create(&g.dev[v.c])))
		{
			while(--v.c>=0)destroy(dh[v.c]);
			fprintf(stderr,"device creation failed\n");
			goto err7;
		}
	}

	if(setresuid(-1,-1,getuid()))
	{
		perror("setresuid");
		goto err7;
	}

	if(prctl(PR_SET_DUMPABLE,1,0,0,0))
	{
		perror("prctl(PR_SET_DUMPABLE)");
		goto err7;
	}

	if(f&2)
	{
		memset(xrun,0,sizeof(xrun));
		memset(&u.ic,0,sizeof(u.ic));
		u.ic.it_interval.tv_sec=1;
		u.ic.it_value.tv_sec=1;
		timerfd_settime(p[2].fd,0,&u.ic,NULL);
	}

	err=0;

	while(1)
	{
		if(poll(p,3,-1)<=0)continue;

		if(p[0].revents&POLLIN)
		{
			if(read(p[0].fd,&u.sig,sizeof(u.sig))!=sizeof(u.sig))
				continue;
			switch(u.sig.ssi_signo)
			{
			case SIGHUP:
			case SIGINT:
			case SIGTERM:
				break;
			case SIGUSR1:
				for(f=0;f<g.dnum;f++)
					tunedump(dh[f],f,g.cln.strict);
			default:continue;
			}
			break;
		}

		if(p[1].revents&POLLIN)if((u.m=dequeue(&q)))
		{
			for(f=0,v.c=0;v.c<g.dnum;v.c++)if(g.dev[v.c].local)
				if(!camhandler(dh[v.c],hh,&u.m->msg))
			{
				f=1;
				break;
			}

			if(f);
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
				satip_hw_cam_io(hh,&u.m->msg);
			}
			free(u.m);
		}

		if(p[2].revents&POLLIN)
		{
			u.dummy=read(p[2].fd,&u.dummy,sizeof(u.dummy));
			for(v.xrun.bfr[0]=0,f=0;f<g.dnum;f++)
			{
				xruncpy(&u.xcpy,dh[f]);
				if(!memcmp(&xrun[f],&u.xcpy,sizeof(XRUN)))
					continue;
				v.xrun.mem.sec=u.xcpy.sec-xrun[f].sec;
				v.xrun.mem.pes=u.xcpy.pes-xrun[f].pes;
				v.xrun.mem.dmx=u.xcpy.dmx-xrun[f].dmx;
				v.xrun.mem.dvr=u.xcpy.dvr-xrun[f].dvr;
				v.xrun.mem.net=u.xcpy.net-xrun[f].net;
				xrun[f]=u.xcpy;
				if(!v.xrun.bfr[0])
				{
					v.xrun.tme=time(NULL);
					localtime_r(&v.xrun.tme,&u.tm);
					strftime(v.xrun.bfr,sizeof(v.xrun.bfr),
						"%F %T",&u.tm);
				}
				fprintf(stderr,"XRUN %s Device %d: SEC %llu PES"
					" %llu DMX %llu DVR %llu NET %llu\n",
					v.xrun.bfr,f+1,v.xrun.mem.sec,
					v.xrun.mem.pes,v.xrun.mem.dmx,
					v.xrun.mem.dvr,v.xrun.mem.net);
			}
		}
	}

	for(v.c=0;v.c<g.dnum;v.c++)destroy(dh[v.c]);
err7:	if(g.tnum)satip_hw_fini(hh);
err6:	if(g.nnum)satip_cln_fini(h);
err5:	close(p[0].fd);
err4:	close(p[2].fd);
err3:	close(q.efd);
err2:	pthread_mutex_destroy(&q.mtx);
err1:	return err;
}
