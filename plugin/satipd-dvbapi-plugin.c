/*
 * example plugin for SAT>IP DVB loop driver
 *
 * 2016 by Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 */

#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <signal.h>
#include <poll.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include "satip.h"
#include "libffdecsa/ffdecsa.h"

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
#define pthread_spin_lock	pthread_rwlock_wrlock
#define pthread_spin_unlock	pthread_rwlock_unlock
#define pthread_spinlock_t	pthread_mutex_t
#else
#endif
#endif

#define MAXPIDS		16
#define MAX_CAM_MSGLEN	2048
#define MAX_CAIDS	64
#define MAX_SIDS	64
#define MAX_CONFIGS	32
#define MAXDECODE	8
#define MAX_ENTRIES	140
#define STACKSIZE	131072

#define PMT_LIST_MORE	0x00
#define PMT_LIST_FIRST	0x01
#define PMT_LIST_LAST	0x02
#define PMT_LIST_ONLY	0x03
#define PMT_LIST_ADD	0x04
#define PMT_LIST_UPDATE	0x05

#define OK_DESCRAMBLE	0x01
#define OK_MMI		0x02
#define QUERY		0x03
#define NOT_SELECTED	0x04

#define DVBAPI_PROTOCOL_VERSION		2
#define DVBAPI_CA_SET_PID		0x40086f87
#define DVBAPI_CA_SET_DESCR		0x40106f86                         
#define DVBAPI_CA_SET_DESCR_MODE	0x400c6f88
#define DVBAPI_DMX_SET_FILTER		0x403c6f2b
#define DVBAPI_DMX_STOP			0x00006f2a
#define DVBAPI_AOT_CA			0x9F803000
#define DVBAPI_AOT_CA_PMT		0x9F803200
#define DVBAPI_AOT_CA_STOP		0x9F803F04
#define DVBAPI_FILTER_DATA		0xFFFF0000
#define DVBAPI_CLIENT_INFO		0xFFFF0001
#define DVBAPI_SERVER_INFO		0xFFFF0002
#define DVBAPI_ECM_INFO			0xFFFF0003
#define DVBAPI_INDEX_DISABLE		0xFFFFFFFF

#pragma pack(push,1)

typedef struct
{
	uint8_t adapter;
	uint8_t demux;
	uint8_t filter;
	uint16_t pid;
	uint8_t data[16];
	uint8_t mask[16];
	uint8_t mode[16];
	uint32_t timeout;
	uint32_t flags;
} DOFILTER_MSG;

typedef struct
{
	uint8_t adapter;
	uint32_t pid;
	uint32_t index;
} CA_SET_PID_MSG;

typedef struct
{
	uint8_t adapter;
	uint32_t index;
	uint32_t parity;
	uint8_t cw[8];
} CA_SET_DESCR_MSG;

typedef struct
{
	uint8_t adapter;
	uint32_t index;
	uint32_t algo;
	uint32_t mode;
} CA_SET_DESCR_MODE_MSG;

typedef struct
{
	uint8_t adapter;
	uint8_t demux;
	uint8_t filter;
	uint16_t pid;
} NOFILTER_MSG;

typedef struct
{
	uint8_t adapter;
	uint16_t serviceid;
	uint16_t caid;
	uint16_t pid;
	uint32_t provid;
	uint32_t ecmtime;
} ECM_INFO_MSG;

typedef struct
{
	uint32_t opcode;
	uint8_t demux;
	uint8_t filter;
} FILTERDATA_MSG;

#pragma pack(pop)

typedef struct
{
	union
	{
		struct sockaddr_in s4;
		struct sockaddr_in6 s6;
	} sock;
	int port;
	int deviceid;
	int adapter;
	int maxdecode;
	int pmtfilter;
	int csamode;
	unsigned short igntotal;
	unsigned short ignore[MAX_SIDS];
	unsigned short catotal;
	unsigned short caids[MAX_CAIDS];
} CONFIG;

typedef struct
{
	void *hw;
	struct _piddata *data;
	SATIP_TUNE *tunefilter;
	int filtertotal;
	int xfd;
	int cnum;
	pthread_attr_t attr;
	CONFIG conf[MAX_CONFIGS];
} GLOBAL;

typedef struct
{
	unsigned short prognum;
	unsigned short total;
	int catotal;
	unsigned char *raw[SATIP_PMTCA_MAX];
	struct
	{
		unsigned short pid;
		unsigned short type;
		int catotal;
		unsigned char *raw[SATIP_PMTCA_MAX];
	} data[0];
} PMT;

typedef struct
{
	int total;
	PMT *list[0];
} PMTDATA;

typedef struct
{
	void *handle;
	void *res;
	short fpid;
	int *fd;
	struct _piddata *data;
	FILTERDATA_MSG msg;
} FILTER;

typedef struct
{
	void *keys;
	int flags;
	int csaflags;
	int oe;
	int numpids;
	int idx;
	unsigned char even[8];
	unsigned char odd[8];
	unsigned short pids[SATIP_MAX_PIDS];
	unsigned char *csa[(MAX_ENTRIES+1)<<1];
} DESCRAMBLE;

typedef struct _piditem
{
	struct _piditem *next;
	SATIP_PIDS set;
} PIDITEM;

typedef struct _piddata
{
	unsigned char *new;
	unsigned char *curr;
	void *csactx;
	PIDITEM *plist;
	CONFIG *conf;
	GLOBAL *g;
	PMTDATA *msg;
	SATIP_UTIL_PAT *pat;
	FILTER f[SATIP_MAX_PIDS];
	SATIP_UTIL_PMT *list[MAXPIDS];
	short pids[SATIP_MAX_PIDS];
	SATIP_TUNE tune;
	pthread_mutex_t mtx;
	pthread_mutex_t secmtx;
	pthread_spinlock_t stx;
	pthread_t th;
	pthread_t nw;
	pthread_t cw;
	int usecnt;
	int running:1;
	int term:1;
	int skip:1;
	int fini:1;
	int pidtotal;
	int total;
	int efd;
	int rfd;
	int afd;
	int cfd;
	int dfd;
	int xfd;
	int tfd;
	int csabusy;
	int stage;
	int optnum;
	int optlen;
	unsigned char pidmap[8192];
	unsigned char pmtidx[65536];
	DESCRAMBLE desc[MAXDECODE];
} PIDDATA;

typedef struct
{
	PIDDATA *data;
	int fill;
	int gotcsa;
	PIDITEM p;
} STREAM;

typedef struct
{
	char *name;
	int min;
	int max;
	unsigned long offset;
	int (*func)(void *s,char *val);
} CFGFILE;

#ifndef PROFILE

static void rawstream(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void stream(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void nostat(void *id,SATIP_STATUS *status) __attribute__ ((hot));
static void *csaworker(void *dta)  __attribute__ ((hot));
void plugin_multi(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_STREAM **stream,int total,int totlen,int *done,int flush)
	__attribute__ ((hot));

static int docaids(void *s,char *val) __attribute__ ((cold));
static int doignore(void *s,char *val) __attribute__ ((cold));
static int dohost(void *s,char *val) __attribute__ ((cold));
static int doparam(CFGFILE *p,void *s,char *name,char *val)
	__attribute__ ((cold));
static int parse_config(char *fn,GLOBAL *g) __attribute__ ((cold));
static int instance_init(PIDDATA *data) __attribute__ ((cold));
static void instance_exit(PIDDATA *data) __attribute__ ((cold));
void plugin_init(void *hwhandle,void **globalctx,char *config)
	__attribute__ ((cold));
void plugin_fini(void *hwhandle,void *globalctx) __attribute__ ((cold));

#else

static int docaids(void *s,char *val);
static int doignore(void *s,char *val);
static int dohost(void *s,char *val);

#endif

static CFGFILE cparams[]=
{
	{"host",0,0,offsetof(CONFIG,sock),dohost},
	{"port",1,65535,offsetof(CONFIG,port),NULL},
	{"deviceid",1,65535,offsetof(CONFIG,deviceid),NULL},
	{"adapter",0,255,offsetof(CONFIG,adapter),NULL},
	{"caids",0,0,offsetof(CONFIG,caids),docaids},
	{"maxdecode",1,MAXDECODE,offsetof(CONFIG,maxdecode),NULL},
	{"pmtfilter",0,1,offsetof(CONFIG,pmtfilter),NULL},
#if defined(__x86_64) || defined(__i386)
	{"csamode",0,3,offsetof(CONFIG,csamode),NULL},
#else
	{"csamode",0,1,offsetof(CONFIG,csamode),NULL},
#endif
	{"ignore",0,0,offsetof(CONFIG,ignore),doignore},
	{NULL,0,0,0L,NULL}
};

static int pidcmp(const void *p1,const void *p2)
{
	short v1=*((short *)p1);
	short v2=*((short *)p2);
	return v1-v2;
}

static int intcmp(const void *p1, const void *p2)
{
	int v1=*((int *)p1);
	int v2=*((int *)p2);
	return v1-v2;
}

static int confcmp(const void *p1,const void *p2)
{
	CONFIG *c1=(CONFIG *)p1;
	CONFIG *c2=(CONFIG *)p2;

	if(c1->deviceid>c2->deviceid)return -1;
	else if(c1->deviceid>c2->deviceid)return 1;
	else return 0;
}

static int docaids(void *s,char *val)
{
	int i;
	unsigned long tmp;
	char *mem=val;
	char *end;
	unsigned short *caids=(unsigned short *)s;

	for(i=0,val=strtok_r(mem,",",&mem);val;i++,val=strtok_r(NULL,",",&mem))
	{
		if(!*val||i>=MAX_CAIDS)return -1;
		tmp=strtoul(val,&end,16);
		if(*end||tmp<1||tmp>65535)return -1;
		caids[i]=(unsigned short)tmp;
	}
	if(!i)return -1;
	caids[-1]=i;
	return 0;
}

static int doignore(void *s,char *val)
{
	int i;
	int tmp;
	char *mem;
	unsigned short *ignore=(unsigned short *)s;

	for(i=0,val=strtok_r(val,",",&mem);val;i++,val=strtok_r(NULL,",",&mem))
	{
		if(!*val||i>=MAX_SIDS)return -1;
		tmp=atoi(val);
		if(tmp<1||tmp>65535)return -1;
		ignore[i]=(unsigned short)tmp;
	}
	if(!i)return -1;
	ignore[-1]=i;
	return 0;
}

static int dohost(void *s,char *val)
{
	struct sockaddr_in *s4=(struct sockaddr_in *)s;
	struct sockaddr_in6 *s6=(struct sockaddr_in6 *)s;

	if(inet_pton(AF_INET,val,&s4->sin_addr.s_addr)==1)
	{
		s4->sin_family=AF_INET;
		return 0;
	}
	if(inet_pton(AF_INET6,val,s6->sin6_addr.s6_addr)==1)
	{
		s6->sin6_family=AF_INET6;
		return 0;
	}
	return -1;
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

static int parse_config(char *fn,GLOBAL *g)
{
	int i;
	int j;
	int line=0;
	int freq=0;
	char *name;
	char *val;
	char *mem;
	FILE *fp;
	CONFIG *c=NULL;
	SATIP_TUNE *t;
	SATIP_PIDS set;
	SATIP_PIDS add;
	SATIP_PIDS del;
	char bfr[8192];

	if(!fn)
	{
		fprintf(stderr,"No configuration file specified\n");
		goto err1;
	}

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

		if(!strcmp(name,"[softcam]"))
		{
			if(g->cnum==MAX_CONFIGS)
			{
				fprintf(stderr,"Out of memory\n");
				goto err2;
			}
			c=&g->conf[g->cnum++];
			if(freq)freq=-1;
			continue;
		}
		else if(!strcmp(name,"[transponders]"))
		{
			if(freq)goto err3;
			c=NULL;
			freq=1;
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

		if(c)
		{
			if(doparam(cparams,c,name,val))goto err3;
		}
		else if(freq==1)
		{
			if(!strcmp(name,"tune"))
			{
				if(!(t=realloc(g->tunefilter,
				    (g->filtertotal+1)*sizeof(SATIP_TUNE))))
				{
					fprintf(stderr,"Out of memory\n");
					goto err2;
				}
				g->tunefilter=t;
				t=&t[g->filtertotal++];
				satip_util_init_tune(t);
				if(satip_util_parse(SATIP_PARSE_QRY,0,
					SATIP_IGNCAPS|SATIP_IGNPLPETC,
					val+1,NULL,NULL,0,NULL,
					NULL,t,&set,&add,&del,NULL)||
					set.numpids!=SATIP_NOPIDS||add.numpids||
					del.numpids)goto err3;
			}
			else goto err3;
		}
		else goto err3;
	}

	if(!g->cnum)
	{
		fprintf(stderr,"No softcam(s) defined.\n");
		goto err2;
	}

	qsort(g->conf,g->cnum,sizeof(CONFIG),confcmp);

	for(i=0,j=0;i<g->cnum;i++)
	{
		if(i&&g->conf[i-1].deviceid==g->conf[i].deviceid)goto err4;
		if(!g->conf[i].sock.s4.sin_family||!g->conf[i].port||
			!g->conf[i].maxdecode||!g->conf[i].deviceid)goto err4;

		for(j=0;j<i-1;j++)if(g->conf[i].adapter==g->conf[j].adapter)
			goto err4;

		switch(g->conf[i].sock.s4.sin_family)
		{
		case AF_INET:
			g->conf[i].sock.s4.sin_port=htons(g->conf[i].port);
			break;
		case AF_INET6:
			g->conf[i].sock.s6.sin6_port=htons(g->conf[i].port);
			break;
		}
	}

	posix_fadvise(fileno(fp),0,0,POSIX_FADV_DONTNEED);
	fclose(fp);
	return 0;

err4:	fprintf(stderr,"Syntax error in softcam definition.\n");
	goto err2;
err3:	fprintf(stderr,"Syntax error in line %d of %s\n",line,fn);
err2:	fclose(fp);
err1:	return -1;
}

static int pmtsend(int fd,PMT *pmt,int adapter,int type)
{
	int i;
	int j;
	int l;
	int pos;
	int mem;
	int off=2;
	unsigned char *ptr;
	unsigned char bfr[MAX_CAM_MSGLEN];

	ptr=bfr+6;

	ptr[0]=type;
	ptr[1]=pmt->prognum>>8;
	ptr[2]=pmt->prognum&0xff;
	ptr[3]=0x01;

	ptr[7]=0x82;
	ptr[8]=0x02;
	ptr[9]=0x00;
	ptr[10]=adapter;

	for(pos=11,i=0;i<pmt->catotal;i++)
	{
		l=satip_util_get_raw_len(pmt->raw[i]);
		if(l>sizeof(bfr)-pos-6)return -1;
		memcpy(ptr+pos,pmt->raw[i],l);
		pos+=l;
	}

	ptr[4]=(pos-6)>>8;
	ptr[5]=(pos-6)&0xff;
	ptr[6]=OK_DESCRAMBLE;

	for(i=0;i<pmt->total;i++)
	{
		if(pos+5+(pmt->data[i].catotal?1:0)>sizeof(bfr)-6)return -1;
		ptr[pos++]=pmt->data[i].type;
		ptr[pos++]=pmt->data[i].pid>>8;
		ptr[pos++]=pmt->data[i].pid&0xff;
		mem=pos;
		pos+=2;
		if(pmt->data[i].catotal)ptr[pos++]=OK_DESCRAMBLE;

		for(j=0;j<pmt->data[i].catotal;j++)
		{
			l=satip_util_get_raw_len(pmt->data[i].raw[j]);
			if(l>sizeof(bfr)-pos-6)return -1;
			memcpy(ptr+pos,pmt->data[i].raw[j],l);
			pos+=l;
		}

		ptr[mem]=(pos-mem-2)>>8;
		ptr[mem+1]=(pos-mem-2)&0xff;
	}

	if(pos>127&&pos<256)
	{
		off-=1;
		bfr[off+3]=0x81;
		bfr[off+4]=pos;
		mem=5;
	}
	else if(pos>=256)
	{
		off-=2;
		bfr[off+3]=0x82;
		bfr[off+4]=pos>>8;
		bfr[off+5]=pos&0xff;
		mem=6;
	}
	else
	{
		bfr[off+3]=pos;
		mem=4;
	}

	bfr[off]=0x9f;
	bfr[off+1]=0x80;
	bfr[off+2]=0x32;

	if(write(fd,bfr+off,pos+mem)!=pos+mem)return -1;

	return 0;
}

static void rawstream(void *id,SATIP_STREAM *stream)
{
	FILTER *f=(FILTER *)id;
	struct iovec io[2];
	struct itimerspec it;

	if(!stream||!stream->fill||!(stream->flags&SATIP_FLGSECT))return;

	io[0].iov_base=&f->msg;
	io[0].iov_len=sizeof(f->msg);
	io[1].iov_base=stream->section;
	io[1].iov_len=stream->fill;

	pthread_mutex_lock(&f->data->secmtx);
	if(f->data->stage==6)
		if(writev(*(f->fd),io,2)!=io[0].iov_len+io[1].iov_len)
	{
		f->data->stage=7;
		memset(&it,0,sizeof(it));
		it.it_value.tv_nsec=1;
		timerfd_settime(f->data->tfd,0,&it,NULL);
	}
	pthread_mutex_unlock(&f->data->secmtx);
}

static void stream(void *id,SATIP_STREAM *stream)
{
	int i;
	int idx;
	uint64_t dummy=1;
	PIDDATA *data=(PIDDATA *)id;
	SATIP_UTIL_PAT *pat=(SATIP_UTIL_PAT *)data->f[0].res;
	union
	{       
		SATIP_UTIL_PAT *pat;
		SATIP_UTIL_PMT *pmt;
	} u;

	if(!stream||!stream->fill||!(stream->flags&SATIP_FLGSECT))return;

	switch(stream->section[0])
	{
	case 0x00:
		if(!(u.pat=satip_util_unpack_pat_section(stream->section,
			stream->fill)))break;;
		if(!u.pat->cnind||u.pat->secnum)goto nopat;
		pthread_spin_lock(&data->stx);
		if(!data->f[0].res)
		{
			data->f[0].res=u.pat;
			pthread_spin_unlock(&data->stx);
			dummy=write(data->efd,&dummy,sizeof(dummy));
			return;
		}
		pthread_spin_unlock(&data->stx);
nopat:		free(u.pat);
		return;

	case 0x02:
		if(!(u.pmt=satip_util_unpack_pmt_section(stream->section,
			stream->fill)))break;
		if(!u.pmt->cnind||u.pmt->secnum)goto nopmt;
		pthread_spin_lock(&data->stx);
		if((idx=data->pmtidx[u.pmt->prognum]))
			if(!data->f[idx].res)for(i=0;i<pat->total;i++)
				if(u.pmt->prognum==pat->data[i].prognum)
		{
			pat->data[i].pmt=u.pmt;
			data->f[idx].res=u.pmt;
			pthread_spin_unlock(&data->stx);
			dummy=write(data->efd,&dummy,sizeof(dummy));
			return;
		}
		pthread_spin_unlock(&data->stx);
nopmt:		free(u.pmt);
		return;
	}
}

static void nostat(void *id,SATIP_STATUS *status)
{
}

static void *hwplay(PIDDATA *data,SATIP_PIDS *set,void *rawflt)
{
	SATIP_STRDATA s;

	s.tune=&data->tune;
	s.set=set;
	s.terminate=data->g->xfd;
	s.handle=rawflt?rawflt:data;

	if(satip_hw_play(data->g->hw,&s,rawflt?rawstream:stream,nostat,NULL,
		SATIP_HW_LOCAL))return NULL;
	return s.handle;
}

static int hwend(PIDDATA *data,void *handle)
{
	SATIP_STRDATA s;

	s.handle=handle;
	return satip_hw_end(data->g->hw,&s)?-1:0;
}

static int hwpids(PIDDATA *data,void *handle,SATIP_PIDS *set)
{
	SATIP_STRDATA s;

	s.handle=handle;
	s.set=set;
	return satip_hw_setpids(data->g->hw,&s)?-1:0;
}

static void *networker(void *dta)
{
	int fd=-1;
	int tmo=0;
	int max=0;
	int r;
	int x;
	int reuse;
	unsigned char *mem;
	unsigned char *old;
	PIDDATA *data=(PIDDATA *)dta;
	PMTDATA *newmsg=NULL;
	PMTDATA *curr=NULL;
	uint64_t dummy;
	struct pollfd p[3];
	sigset_t sset;
	struct itimerspec it;
	struct iovec io[3];
	uint32_t opcode;
	uint16_t protovers;
	unsigned char bfr[256];
	DOFILTER_MSG dofilter;
	CA_SET_PID_MSG capid;
	CA_SET_DESCR_MSG cadescr;
	CA_SET_DESCR_MODE_MSG camode;
	NOFILTER_MSG nofilter;
	ECM_INFO_MSG ecminfo;
	SATIP_PIDS set;

	sigfillset(&sset);
	pthread_sigmask(SIG_SETMASK,&sset,NULL);

	pthread_setname_np(pthread_self(),"plugin network");

	p[0].fd=data->rfd;
	p[0].events=POLLIN;
	p[1].fd=data->tfd;
	p[1].events=POLLIN;

	for(x=0;x<SATIP_MAX_PIDS;x++)data->f[x].fd=&fd;

	data->stage=0;

	satip_util_init_pids(&set);
	set.numpids=SATIP_SECTION;
	set.filterraw=1;

	while(1)
	{
		if(fd==-1)
		{
			p[2].revents=0;
			r=poll(p,2,-1);
		}
		else r=poll(p,3,-1);

		if(data->fini)break;
		if(r<0)continue;

		if(p[0].revents&POLLIN)
		{
			dummy=read(data->rfd,&dummy,sizeof(dummy));

			if(newmsg)
			{
				for(r=0;r<newmsg->total;r++)
					free(newmsg->list[r]);
				free(newmsg);
			}
			newmsg=data->msg;

			pthread_mutex_lock(&data->secmtx);
			if(newmsg&&!data->stage)
			{
				data->stage=1;
				pthread_mutex_unlock(&data->secmtx);
				dummy=1;
				dummy=write(data->afd,&dummy,sizeof(dummy));
			}
			else if(!newmsg&&data->stage)
			{
				data->stage=8;
				pthread_mutex_unlock(&data->secmtx);
			}
			else
			{
				pthread_mutex_unlock(&data->secmtx);
				dummy=1;
				dummy=write(data->afd,&dummy,sizeof(dummy));
			}
		}

		if(p[1].revents&POLLIN)
		{
			dummy=read(data->tfd,&dummy,sizeof(dummy));
			tmo=1;
			memset(&it,0,sizeof(it));
			timerfd_settime(data->tfd,0,&it,NULL);
		}

repeat:		pthread_mutex_lock(&data->secmtx);
		switch(data->stage)
		{
		case 2:	pthread_mutex_unlock(&data->secmtx);
			if(!tmo)break;
			tmo=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(data->tfd,0,&it,NULL);
			dummy=read(data->tfd,&dummy,sizeof(dummy));
			goto stage1;

		case 1:	pthread_mutex_unlock(&data->secmtx);
stage1:			if((fd=socket(data->conf->sock.s4.sin_family,
				SOCK_STREAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0))==-1)
			{
				data->stage=2;
				tmo=0;
				memset(&it,0,sizeof(it));
				timerfd_settime(data->tfd,0,&it,NULL);
				dummy=read(data->tfd,&dummy,sizeof(dummy));
				it.it_value.tv_sec=2;
				timerfd_settime(data->tfd,0,&it,NULL);
				break;
			}

			p[2].fd=fd;

			if((connect(fd,(struct sockaddr *)&data->conf->sock,
				sizeof(data->conf->sock)))<0)
			{
				if(errno==EINPROGRESS)
				{
					data->stage=3;
					p[2].events=POLLOUT;
					it.it_value.tv_nsec=500000000;
					timerfd_settime(data->tfd,0,&it,NULL);
				}
				else
				{
					close(fd);
					fd=-1;
					data->stage=2;
					it.it_value.tv_sec=2;
					timerfd_settime(data->tfd,0,&it,NULL);
				}
				break;
			}

			data->stage=4;
			tmo=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(data->tfd,0,&it,NULL);
			dummy=read(data->tfd,&dummy,sizeof(dummy));
			goto repeat;

		case 3:	pthread_mutex_unlock(&data->secmtx);
			if(tmo&&!(p[2].events&POLLOUT))goto connfail;
			if(!(p[2].events&POLLOUT))break;
			r=sizeof(x);
			if(!getsockopt(fd,SOL_SOCKET,SO_ERROR,&x,&r)&&!x)
			{
				data->stage=4;
				tmo=0;
				memset(&it,0,sizeof(it));
				timerfd_settime(data->tfd,0,&it,NULL);
				dummy=read(data->tfd,&dummy,sizeof(dummy));
				goto repeat;
			}
connfail:		close(fd);
			fd=-1;
			data->stage=2;
			tmo=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(data->tfd,0,&it,NULL);
			dummy=read(data->tfd,&dummy,sizeof(dummy));
			it.it_value.tv_sec=2;
			timerfd_settime(data->tfd,0,&it,NULL);
			break;

		case 4:	pthread_mutex_unlock(&data->secmtx);
			r=1;
			if(setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&r,sizeof(r)))
			{
				close(fd);
				fd=-1;
				data->stage=2;
				it.it_value.tv_sec=2;
				timerfd_settime(data->tfd,0,&it,NULL);
				break;
			}
			opcode=htonl(DVBAPI_CLIENT_INFO);
			protovers=htons(DVBAPI_PROTOCOL_VERSION);
			strcpy(bfr+1,"satiploopd dvbapi plugin");
			bfr[0]=strlen(&bfr[1]);
			io[0].iov_base=&opcode;
			io[0].iov_len=sizeof(opcode);
			io[1].iov_base=&protovers;
			io[1].iov_len=sizeof(protovers);
			io[2].iov_base=bfr;
			io[2].iov_len=bfr[0]+1;
			if(writev(fd,io,3)!=io[0].iov_len+io[1].iov_len+
				io[2].iov_len)
			{
				close(fd);
				fd=-1;
				data->stage=2;
				it.it_value.tv_sec=2;
				timerfd_settime(data->tfd,0,&it,NULL);
				break;
			}
			data->stage=5;
			p[2].events=POLLIN;
			it.it_value.tv_nsec=500000000;
			timerfd_settime(data->tfd,0,&it,NULL);
			break;

		case 5:	pthread_mutex_unlock(&data->secmtx);
			if(tmo&&!(p[2].revents&POLLIN))goto connfail;
			if(!(p[2].revents&POLLIN))break;
			if(read(fd,&opcode,sizeof(opcode))!=sizeof(opcode)
				||ntohl(opcode)!=DVBAPI_SERVER_INFO)
					goto connfail;
			if(poll(&p[2],1,10)!=1||!(p[2].revents&POLLIN))
					goto connfail;
			if(read(fd,&protovers,sizeof(protovers))!=
				sizeof(protovers)||
				ntohs(protovers)<DVBAPI_PROTOCOL_VERSION)
					goto connfail;
			if(poll(&p[2],1,10)!=1||!(p[2].revents&POLLIN))
					goto connfail;
			if(read(fd,bfr,1)!=1)goto connfail;
			if(bfr[0])
			{
				if(poll(&p[2],1,10)!=1||!(p[2].revents&POLLIN))
					goto connfail;
				if(read(fd,bfr+1,bfr[0])!=bfr[0])goto connfail;
			}
			tmo=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(data->tfd,0,&it,NULL);
			data->stage=6;
			if(newmsg&&curr)
			{
				for(r=0;r<curr->total;r++)free(curr->list[r]);
				free(curr);
				curr=NULL;
			}
			else if(!newmsg&&curr)
			{
				newmsg=curr;
				curr=NULL;
			}
			p[2].revents=0;
			goto stage6;

		case 6:	pthread_mutex_unlock(&data->secmtx);
stage6:			if(newmsg)
			{
				if(curr)
				{
					for(r=0;r<curr->total;r++)
						free(curr->list[r]);
					free(curr);
				}
				curr=newmsg;
				newmsg=NULL;
				if(curr->total==1)r=pmtsend(fd,curr->list[0],
					data->conf->adapter,PMT_LIST_ONLY);
				else for(x=0,r=0;x<curr->total&&!r;x++)
				{
					if(!x)r=pmtsend(fd,curr->list[0],
						data->conf->adapter,
						PMT_LIST_FIRST);
					else if(x+1==curr->total)
						r=pmtsend(fd,curr->list[x],
							data->conf->adapter,
							PMT_LIST_LAST);
					else r=pmtsend(fd,curr->list[x],
						data->conf->adapter,
						PMT_LIST_MORE);
				}
				if(r)goto datafail;
			}
			if(!(p[2].revents&POLLIN))break;
			if(read(fd,&opcode,sizeof(opcode))!=sizeof(opcode))
				goto datafail;
			switch(ntohl(opcode))
			{
			case DVBAPI_CA_SET_PID:
				if(read(fd,&capid,sizeof(capid))!=
					sizeof(capid))goto datafail;
				if(capid.adapter!=data->conf->adapter)break;
				capid.pid=ntohl(capid.pid);
				capid.index=ntohl(capid.index);
				if(capid.index!=-1&&
					capid.index>=data->conf->maxdecode)
						break;
				if(capid.pid<0||capid.pid>8191)break;
				data->pidmap[capid.pid]=
					(unsigned char)capid.index;
				if((mem=malloc(sizeof(data->pidmap))))
				{
					memcpy(mem,data->pidmap,
						sizeof(data->pidmap));
					pthread_spin_lock(&data->stx);
					old=data->new;
					data->new=mem;
					pthread_spin_unlock(&data->stx);
					if(old)free(old);
				}
				break;

			case DVBAPI_CA_SET_DESCR:
				if(read(fd,&cadescr,sizeof(cadescr))!=
					sizeof(cadescr))goto datafail;
				if(cadescr.adapter!=data->conf->adapter)break;
				cadescr.index=ntohl(cadescr.index);
				cadescr.parity=ntohl(cadescr.parity);
				if(cadescr.index>=data->conf->maxdecode)break;
				pthread_spin_lock(&data->stx);
				if(cadescr.parity)
				{
				    memcpy(data->desc[cadescr.index].odd,
					cadescr.cw,8);
				    data->desc[cadescr.index].flags|=0x01;
				    data->desc[cadescr.index].csaflags|=0x01;
				}
				else
				{
				    memcpy(data->desc[cadescr.index].even,
					cadescr.cw,8);
				    data->desc[cadescr.index].flags|=0x02;
				    data->desc[cadescr.index].csaflags|=0x02;
				}
				pthread_spin_unlock(&data->stx);
				break;

			case DVBAPI_CA_SET_DESCR_MODE:
				if(read(fd,&camode,sizeof(camode))!=
					sizeof(camode))goto datafail;
				if(camode.adapter!=data->conf->adapter)break;
				camode.index=ntohl(camode.index);
				camode.algo=ntohl(camode.algo);
				camode.mode=ntohl(camode.mode);
				if(camode.index>=data->conf->maxdecode)break;
				break;

			case DVBAPI_DMX_SET_FILTER:
				if(read(fd,&dofilter,sizeof(dofilter))!=
					sizeof(dofilter))goto datafail;
				if(dofilter.adapter!=data->conf->adapter)break;
				dofilter.pid=ntohs(dofilter.pid);
				dofilter.timeout=ntohl(dofilter.timeout);
				dofilter.flags=ntohl(dofilter.flags);
				reuse=0;
				for(x=0;x<=max;x++)if(data->f[x].handle&&
					data->f[x].msg.demux==dofilter.demux&&
					data->f[x].msg.filter==dofilter.filter&&
					data->f[x].fpid==dofilter.pid)
				{
					reuse=1;
					break;
				}
				if(x>max)for(x=0;x<SATIP_MAX_PIDS;x++)
					if(!data->f[x].handle)break;
				if(x==SATIP_MAX_PIDS)break;
				if(!reuse&&x>max)max=x;
				data->f[x].msg.demux=dofilter.demux;
				data->f[x].msg.filter=dofilter.filter;
				data->f[x].fpid=dofilter.pid;
				set.pid=dofilter.pid;
				set.filtercrc=dofilter.flags&1;
				memcpy(set.filter,dofilter.data,SATIP_FILTERSZ);
				memcpy(set.mask,dofilter.mask,SATIP_FILTERSZ);
				memcpy(set.mode,dofilter.mode,SATIP_FILTERSZ);
				if(reuse)hwpids(data,data->f[x].handle,&set);
				else data->f[x].handle=hwplay(data,&set,
					&data->f[x]);
				break;

			case DVBAPI_DMX_STOP:
				if(read(fd,&nofilter,sizeof(nofilter))!=
					sizeof(nofilter))goto datafail;
				if(nofilter.adapter!=data->conf->adapter)break;
				nofilter.pid=ntohs(nofilter.pid);
				for(x=0;x<=max;x++)if(data->f[x].handle)
				    if(data->f[x].msg.demux==nofilter.demux&&
					data->f[x].msg.filter==nofilter.filter&&
					data->f[x].fpid==nofilter.pid)
				{
					hwend(data,data->f[x].handle);
					data->f[x].handle=NULL;
					break;
				}
				break;

			case DVBAPI_ECM_INFO:
				if(read(fd,&ecminfo,sizeof(ecminfo))!=
					sizeof(ecminfo))goto datafail;
				if(read(fd,bfr,1)!=1)goto datafail;
				if(bfr[0])if(read(fd,bfr+1,bfr[0])!=bfr[0])
					goto datafail;
				if(read(fd,bfr,1)!=1)goto datafail;
				if(bfr[0])if(read(fd,bfr+1,bfr[0])!=bfr[0])
					goto datafail;
				if(read(fd,bfr,1)!=1)goto datafail;
				if(bfr[0])if(read(fd,bfr+1,bfr[0])!=bfr[0])
					goto datafail;
				if(read(fd,bfr,1)!=1)goto datafail;
				if(bfr[0])if(read(fd,bfr+1,bfr[0])!=bfr[0])
					goto datafail;
				if(read(fd,bfr,1)!=1)goto datafail;
				if(ecminfo.adapter!=data->conf->adapter)break;
				ecminfo.serviceid=ntohs(ecminfo.serviceid);
				ecminfo.caid=ntohs(ecminfo.caid);
				ecminfo.pid=ntohs(ecminfo.pid);
				ecminfo.provid=ntohl(ecminfo.provid);
				ecminfo.ecmtime=ntohl(ecminfo.ecmtime);
				break;

			default:goto datafail;
			}
			break;
datafail:		pthread_mutex_lock(&data->secmtx);

		case 7:	data->stage=2;
			goto downcmn;

		case 8:	data->stage=0;
downcmn:		pthread_mutex_unlock(&data->secmtx);
			if((mem=malloc(sizeof(data->pidmap))))
			{
				memset(mem,0xff,sizeof(data->pidmap));
				pthread_spin_lock(&data->stx);
				old=data->new;
				data->new=mem;
			}
			else
			{
				old=NULL;
				pthread_spin_lock(&data->stx);
			}
			for(x=0;x<data->conf->maxdecode;x++)
				data->desc[x].flags=0x04;
			pthread_spin_unlock(&data->stx);
			if(old)free(old);
			for(x=0;x<=max;x++)if(data->f[x].handle)
			{
				hwend(data,data->f[x].handle);
				data->f[x].handle=NULL;
			}
			close(fd);
			fd=-1;
			tmo=0;
			max=0;
			memset(&it,0,sizeof(it));
			timerfd_settime(data->tfd,0,&it,NULL);
			dummy=read(data->tfd,&dummy,sizeof(dummy));
			memset(data->pidmap,0xff,sizeof(data->pidmap));
			if(!data->stage)
			{
				dummy=1;
				dummy=write(data->afd,&dummy,sizeof(dummy));
			}
			else
			{
				it.it_value.tv_nsec=500000000;
				timerfd_settime(data->tfd,0,&it,NULL);
			}
			break;
		}
        }

	for(x=0;x<=max;x++)if(data->f[x].handle)
	{
		hwend(data,data->f[x].handle);
		data->f[x].handle=NULL;
	}
	if(fd!=-1)close(fd);

	if(newmsg)
	{
		for(r=0;r<newmsg->total;r++)free(newmsg->list[r]);
		free(newmsg);
	}
	if(curr)
	{
		for(r=0;r<curr->total;r++)free(curr->list[r]);
		free(curr);
	}

	pthread_exit(NULL);
}

static PMT *pmtcopy(PIDDATA *data,SATIP_UTIL_PMT *pmt)
{
	int i;
	int j;
	int k;
	int l;
	PMT *p;
	unsigned char *ptr;
	unsigned char *raw;

	for(i=0,l=0;i<pmt->catotal;i++)
	{
		if(data->conf->pmtfilter&&data->conf->catotal)
		{
			for(k=0;k<data->conf->catotal;k++)
				if(data->conf->caids[k]==pmt->caid[i])
					goto prglen;
			continue;
		}
prglen:		l+=satip_util_get_raw_len(
			satip_util_get_raw_pmt(pmt,pmt->raw[i]));
	}

	for(i=0;i<pmt->total;i++)for(j=0;j<pmt->data[i].catotal;j++)
	{
		if(data->conf->pmtfilter&&data->conf->catotal)
		{
			for(k=0;k<data->conf->catotal;k++)
				if(data->conf->caids[k]==
					pmt->data[i].caid[j])goto peslen;
			continue;
		}
peslen:		l+=satip_util_get_raw_len(
			satip_util_get_raw_pmt(pmt,pmt->data[i].raw[j]));
	}

	if(!(p=malloc(sizeof(PMT)+pmt->total*sizeof(p->data[0])+l)))return NULL;
	ptr=((unsigned char *)p)+sizeof(PMT)+pmt->total*sizeof(p->data[0]);

	p->prognum=pmt->prognum;
	p->total=pmt->total;
	p->catotal=0;

	for(i=0;i<pmt->catotal;i++)
	{
		if(data->conf->pmtfilter&&data->conf->catotal)
		{
			for(k=0;k<data->conf->catotal;k++)
				if(data->conf->caids[k]==pmt->caid[i])
					goto prgadd;
			continue;
		}
prgadd:		raw=satip_util_get_raw_pmt(pmt,pmt->raw[i]);
		l=satip_util_get_raw_len(raw);
		memcpy(ptr,raw,l);
		p->raw[p->catotal++]=ptr;
		ptr+=l;
	}

	for(i=0;i<pmt->total;i++)
	{
	    p->data[i].pid=pmt->data[i].pid;
	    p->data[i].type=pmt->data[i].type1;
	    p->data[i].catotal=0;
	    for(j=0;j<pmt->data[i].catotal;j++)
	    {
		if(data->conf->pmtfilter&&data->conf->catotal)
		{
			for(k=0;k<data->conf->catotal;k++)
				if(data->conf->caids[k]==
					pmt->data[i].caid[j])goto pesadd;
			continue;
		}
pesadd:		raw=satip_util_get_raw_pmt(pmt,pmt->data[i].raw[j]);
		l=satip_util_get_raw_len(raw);
		memcpy(ptr,raw,l);
		p->data[i].raw[p->data[i].catotal++]=ptr;
		ptr+=l;
	    }
	}

	return p;
}

static int newlist(PIDDATA *data,SATIP_UTIL_PMT **list,int total)
{
	int i;
	struct pollfd p;
	uint64_t dummy=1;

	data->msg=NULL;

	if(!total)goto send;

	if(!(data->msg=malloc(sizeof(PMTDATA)+total*
		sizeof(data->msg->list[0]))))return -1;
	data->msg->total=total;

	for(i=0;i<total;i++)if(!(data->msg->list[i]=pmtcopy(data,list[i])))
	{
		while(--(i)>=0)free(data->msg->list[i]);
		free(data->msg);
		return -1;
	}

send:	dummy=write(data->rfd,&dummy,sizeof(dummy));
	p.fd=data->afd;
	p.events=POLLIN;
	while(poll(&p,1,250)<=0||!(p.revents&POLLIN))
		if(data->term||data->fini)return -1;
	dummy=read(data->afd,&dummy,sizeof(dummy));

	return 0;
}

static int cafilter(PIDDATA *data,SATIP_UTIL_PMT *pmt)
{
	int i;
	int j;
	int k;

	for(i=0;i<data->conf->igntotal;i++)
		if(data->conf->ignore[i]==pmt->prognum)return -1;

	if(!data->conf->catotal)return 0;

	for(i=0;i<data->conf->catotal;i++)
	{
		for(k=0;k<pmt->catotal;k++)
			if(data->conf->caids[i]==pmt->caid[k])return 0;
		for(j=0;j<pmt->total;j++)for(k=0;k<pmt->data[j].catotal;j++)
			if(data->conf->caids[i]==pmt->data[j].caid[k])
				return 0;
	}

	return -1;
}

static void *pidworker(void *dta)
{
	PIDDATA *data=(PIDDATA *)dta;
	SATIP_UTIL_PAT *pat=NULL;
	SATIP_UTIL_PMT *pmt;
	int i;
	int j;
	int k;
	int state=0;
	int totcnt=0;
	int pmttot=0;
	int tmo=-1;
	int total;
	uint64_t dummy;
	SATIP_PIDS set;
	SATIP_UTIL_PMT *list[MAXPIDS];
	struct pollfd p;
	sigset_t sset;

	sigfillset(&sset);
	pthread_sigmask(SIG_SETMASK,&sset,NULL);

	pthread_setname_np(pthread_self(),"plugin pidwork");

	satip_util_init_pids(&set);
	set.numpids=SATIP_SECTION;
	set.table=0x02;

	p.fd=data->efd;
	p.events=POLLIN;

	while(1)
	{
		switch(poll(&p,1,tmo))
		{
		case -1:if(data->term)goto out;
			continue;

		case 0:	if(data->term)goto out;
			if(state==1)
			{
				for(i=1;i<pmttot;i++)if(data->f[i].handle)
				{
					hwend(data,data->f[i].handle);
					data->f[i].handle=NULL;
				}
				tmo=-1;
				state=2;
				goto work;
			}
			continue;

		default:if(data->term)goto out;
			if(!(p.revents&POLLIN))continue;
			break;
		}

		dummy=read(data->efd,&dummy,sizeof(dummy));

work:		switch(state)
		{
		case 0:	pthread_spin_lock(&data->stx);
			if(!data->f[0].res)
			{
				pthread_spin_unlock(&data->stx);
				break;
			}
			pthread_spin_unlock(&data->stx);
			hwend(data,data->f[0].handle);
			data->f[0].handle=NULL;
			pat=(SATIP_UTIL_PAT *)data->f[0].res;
			memset(data->pmtidx,0,sizeof(data->pmtidx));
			for(i=1,j=0,totcnt=1;i<SATIP_MAX_PIDS&&j<pat->total;j++)
				if(pat->data[j].prognum)
			{
				data->pmtidx[pat->data[j].prognum]=i;
				for(k=1;k<i;k++)
					if(data->f[k].fpid==pat->data[j].pmtpid)
						break;
				if(k<i)
				{
					totcnt++;
					continue;
				}

				set.pid=pat->data[j].pmtpid;
				if(!(data->f[i].handle=hwplay(data,&set,NULL)))
					continue;

				data->f[i++].fpid=pat->data[j].pmtpid;
				totcnt++;
			}
			pmttot=i;
			tmo=2000;
			state=1;

		case 1:	for(i=1;i<pmttot;i++)
			{
				pthread_spin_lock(&data->stx);
				if(!data->f[i].res||!data->pmtidx[
				 ((SATIP_UTIL_PMT *)(data->f[i].res))->prognum])
				{
					pthread_spin_unlock(&data->stx);
					continue;
				}
				data->pmtidx[
					((SATIP_UTIL_PMT *)(data->f[i].res))
					->prognum]=0;
				pthread_spin_unlock(&data->stx);
				totcnt--;
			}

			if(totcnt)break;

			for(i=1;i<pmttot;i++)
			{
				hwend(data,data->f[i].handle);
				data->f[i].handle=NULL;
			}
			tmo=-1;
			state=2;

		case 2:	qsort(&pat->data[0],pat->total,sizeof(pat->data[0]),
				intcmp);

			for(i=0;i<pat->total;i++)if(pat->data[i].pmt)
				qsort(&pat->data[i].pmt->data[0],
					pat->data[i].pmt->total,
					sizeof(pat->data[i].pmt->data[0]),
					intcmp);

			pthread_mutex_lock(&data->mtx);
			data->pat=pat;
			state=3;
			pthread_mutex_unlock(&data->mtx);

		case 3:	pthread_mutex_lock(&data->mtx);
			for(total=0,i=0;i<pat->total;i++)if(pat->data[i].pmt)
			{
				pmt=pat->data[i].pmt;
				for(j=0;j<pmt->total;j++)
				{
				    for(k=0;k<data->pidtotal;k++)
				    {
					if(data->pids[k]==pmt->data[j].pid)
						goto hit;
					if(data->pids[k]>pmt->data[j].pid)
						goto fail;
				    }
				    goto fail;
hit:;
				}
				if(cafilter(data,pmt))goto fail;
				if(total!=MAXPIDS)list[total++]=pmt;
fail:;
			}
			pthread_mutex_unlock(&data->mtx);
			if(data->total!=total||
				memcmp(data->list,list,total*sizeof(list[0])))
					if(!newlist(data,list,total))
			{
				memcpy(data->list,list,total*sizeof(list[0]));
				data->total=total;
			}
			break;
		}
	}

out:	pthread_exit(NULL);
}

static void pidupdate(PIDDATA *data)
{
	int i;
	int j;
	PIDITEM *e;

	for(e=data->plist,data->pidtotal=0;e&&data->pidtotal<SATIP_MAX_PIDS;
	  e=e->next)for(i=0;i<e->set.numpids&&data->pidtotal<SATIP_MAX_PIDS;i++)
	{
		for(j=0;j<data->pidtotal;j++)
			if(e->set.pids[i]==data->pids[j])break;
		if(j==data->pidtotal)
			data->pids[data->pidtotal++]=e->set.pids[i];
	}
}

static void *csaworker(void *dta)
{
	PIDDATA *data=(PIDDATA *)dta;
	int i;
	int flags;
	uint64_t dummy;
	unsigned char even[8];
	unsigned char odd[8];
	struct pollfd p[2];
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_SETMASK,&set,NULL);

	pthread_setname_np(pthread_self(),"plugin csawork");

	p[0].fd=data->cfd;
	p[0].events=POLLIN;
	p[1].fd=data->xfd;
	p[1].events=POLLIN;

	while(1)
	{
		if(poll(p,2,-1)<1)continue;

		if(UNLIKELY(p[1].revents&POLLIN))break;
		if(UNLIKELY(!(p[0].revents&POLLIN)))continue;

		dummy=read(data->cfd,&dummy,sizeof(dummy));

		for(i=0;i<data->conf->maxdecode;i++)
		{
			pthread_spin_lock(&data->stx);
			if(!(flags=data->desc[i].csaflags))
			{
				pthread_spin_unlock(&data->stx);
				goto process;
			}
			data->desc[i].csaflags=0x00;
			if(flags&0x01)
				memcpy(odd,data->desc[i].odd,sizeof(odd));
			if(flags&0x02)
				memcpy(even,data->desc[i].even,sizeof(even));
			pthread_spin_unlock(&data->stx);

			switch(flags)
			{
			case 0x01:
				ffdecsa_set_odd_control_word(data->csactx,
					data->desc[i].keys,odd);
				break;
			case 0x02:
				ffdecsa_set_even_control_word(data->csactx,
					data->desc[i].keys,even);
				break;
			case 0x03:
				ffdecsa_set_control_words(data->csactx,
					data->desc[i].keys,even,odd);
				break;
			}

process:		if(data->desc[i].idx)
			{
				data->desc[i].csa[data->desc[i].idx]=NULL;
				ffdecsa_decrypt_packets(data->csactx,
					data->desc[i].keys,data->desc[i].csa);
				data->desc[i].idx=0;
			}
		}

		dummy=1;
		dummy=write(data->dfd,&dummy,sizeof(dummy));

	}

	pthread_exit(NULL);
}

static int instance_init(PIDDATA *data)
{
	int i;
	uint64_t dummy=1;
	pthread_attr_t attr;

	if(!(data->curr=malloc(sizeof(data->pidmap))))goto err1;
	memset(data->pidmap,0xff,sizeof(data->pidmap));
	memset(data->curr,0xff,sizeof(data->pidmap));
	for(i=0;i<SATIP_MAX_PIDS;i++)
	{
		data->f[i].msg.opcode=htonl(DVBAPI_FILTER_DATA);
		data->f[i].data=data;
	}
	ffdecsa_select_mode(&data->csactx,data->conf->csamode);
	data->optnum=ffdecsa_get_suggested_cluster_size(data->csactx);
	if(data->optnum>MAX_ENTRIES)data->optnum=MAX_ENTRIES;
	data->optlen=data->optnum*188;
	data->optnum<<=1;
	if(pthread_spin_init(&data->stx,PTHREAD_PROCESS_PRIVATE))goto err2;
	if(pthread_mutex_init(&data->mtx,NULL))goto err3;
	if(pthread_mutex_init(&data->secmtx,NULL))goto err4;
	if((data->efd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err5;
	if((data->rfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err6;
	if((data->afd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err7;
	if((data->cfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err8;
	if((data->dfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err9;
	if((data->xfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err10;
	if((data->tfd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))
		==-1)goto err11;
	for(i=0;i<data->conf->maxdecode;i++)
		if(!(data->desc[i].keys=ffdecsa_get_key_struct(data->csactx)))
			goto err12;
	if(pthread_attr_init(&attr))goto err12;
	if(pthread_attr_setstacksize(&attr,STACKSIZE))goto err13;
	if(pthread_create(&data->cw,&attr,csaworker,data))goto err13;
	if(pthread_create(&data->nw,&attr,networker,data))goto err14;
	pthread_attr_destroy(&attr);
	return 0;

err14:	dummy=write(data->xfd,&dummy,sizeof(dummy));
	pthread_join(data->cw,NULL);
err13:	pthread_attr_destroy(&attr);
err12:	for(i=0;i<data->conf->maxdecode;i++)
		if(data->desc[i].keys)ffdecsa_free_key_struct(data->csactx,
			data->desc[i].keys);
	close(data->tfd);
err11:	close(data->xfd);
err10:	close(data->dfd);
err9:	close(data->cfd);
err8:	close(data->afd);
err7:	close(data->rfd);
err6:	close(data->efd);
err5:	pthread_mutex_destroy(&data->secmtx);
err4:	pthread_mutex_destroy(&data->mtx);
err3:	pthread_spin_destroy(&data->stx);
err2:	free(data->curr);
err1:	return -1;
}

static void instance_exit(PIDDATA *data)
{
	int i;
	uint64_t dummy=1;

	if(!data)return;

	if(data->running)
	{
		data->term=1;
		dummy=write(data->efd,&dummy,sizeof(dummy));
		pthread_cancel(data->th);
		pthread_join(data->th,NULL);
	}
	for(i=0;i<SATIP_MAX_PIDS;i++)
		if(data->f[i].handle)hwend(data,data->f[i].handle);
	newlist(data,NULL,0);
	for(i=0;i<SATIP_MAX_PIDS;i++)if(data->f[i].res)free(data->f[i].res);
	data->fini=1;
	dummy=1;
	dummy=write(data->rfd,&dummy,sizeof(dummy));
	pthread_cancel(data->nw);
	pthread_join(data->nw,NULL);
	dummy=1;
	dummy=write(data->xfd,&dummy,sizeof(dummy));
	pthread_join(data->cw,NULL);
	close(data->tfd);
	close(data->xfd);
	close(data->dfd);
	close(data->cfd);
	close(data->afd);
	close(data->rfd);
	close(data->efd);
	pthread_mutex_destroy(&data->secmtx);
	pthread_mutex_destroy(&data->mtx);
	pthread_spin_destroy(&data->stx);
	if(data->new)free(data->new);
	if(data->curr)free(data->curr);
	for(i=0;i<data->conf->maxdecode;i++)if(data->desc[i].keys)
		ffdecsa_free_key_struct(data->csactx,data->desc[i].keys);
}

void plugin_init(void *hwhandle,void **globalctx,char *config)
{
	int i;
	GLOBAL *g;

	if(!(g=malloc(sizeof(GLOBAL))))goto err1;
	memset(g,0,sizeof(GLOBAL));
	g->hw=hwhandle;

	if(parse_config(config,g))goto err2;

	if(!(g->data=malloc(sizeof(PIDDATA)*g->cnum)))goto err2;
	memset(g->data,0,sizeof(PIDDATA)*g->cnum);

	if(pthread_attr_init(&g->attr))goto err2;
	if(pthread_attr_setstacksize(&g->attr,STACKSIZE))goto err3;

	for(i=0;i<g->cnum;i++)
	{
		g->data[i].g=g;
		g->data[i].conf=&g->conf[i];
		if(instance_init(&g->data[i]))
		{
			while(--i>=0)instance_exit(&g->data[i]);
			goto err3;
		}
	}

	*globalctx=g;
	return;

err3:	pthread_attr_destroy(&g->attr);
err2:	if(g->tunefilter)free(g->tunefilter);
	if(g->data)free(g->data);
	free(g);
err1:	*globalctx=NULL;
}

void plugin_fini(void *hwhandle,void *globalctx)
{
	int i;
	GLOBAL *g=(GLOBAL *)globalctx;

	if(!g)return;

	for(i=0;i<g->cnum;i++)instance_exit(&g->data[i]);
	pthread_attr_destroy(&g->attr);
	if(g->tunefilter)free(g->tunefilter);
	if(g->data)free(g->data);
	free(g);
}

void plugin_setpids(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_PIDS *set)
{
	int i;
	uint64_t dummy=1;
	GLOBAL *g=(GLOBAL *)globalctx;
	STREAM *s=(STREAM *)streamctx;

	if(!g||!s||!s->data)return;

	s->p.set=*set;
	if(s->p.set.numpids<0)s->p.set.numpids=0;
	else if(s->p.set.numpids>0)qsort(s->p.set.pids,s->p.set.numpids,
		sizeof(short),pidcmp);
	for(i=0;i<s->p.set.numpids;i++)if(s->p.set.pids[i]>=0x20)break;
	if(i)
	{
		memmove(s->p.set.pids,&s->p.set.pids[i],
			(s->p.set.numpids-i)*sizeof(short));
		s->p.set.numpids-=i;
	}

	pthread_mutex_lock(&s->data->mtx);
	pidupdate(s->data);
	if(s->data->pat)dummy=write(s->data->efd,&dummy,sizeof(dummy));
	pthread_mutex_unlock(&s->data->mtx);
}

void plugin_prepare(void *hwhandle,void *globalctx,void **streamctx,
	SATIP_TUNE *tune,SATIP_PIDS *set)
{
	int i;
	GLOBAL *g=(GLOBAL *)globalctx;
	STREAM *s;

	if(!g)goto err1;

	if(g->filtertotal)
	{
		for(i=0;i<g->filtertotal;i++)
			if(!satip_util_tunecmp(tune,&g->tunefilter[i]))
				goto work;
		goto err1;
	}

work:	if(!(s=malloc(sizeof(STREAM))))goto err1;
	memset(s,0,sizeof(STREAM));

	*streamctx=s;
	return;

err1:	*streamctx=NULL;
}

void plugin_play(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_TUNE *tune,SATIP_PIDS *set,int success)
{
	int i;
	GLOBAL *g=(GLOBAL *)globalctx;
	STREAM *s=(STREAM *)streamctx;
	SATIP_PIDS secset;

	if(!g||!s||!success)return;

	for(i=0;i<g->cnum;i++)if(g->conf[i].deviceid==tune->fe)goto work;
	return;

work:	s->data=&g->data[i];

	s->p.set=*set;
	if(s->p.set.numpids<0)s->p.set.numpids=0;
	else if(s->p.set.numpids>0)qsort(s->p.set.pids,s->p.set.numpids,
		sizeof(short),pidcmp);
	for(i=0;i<s->p.set.numpids;i++)if(s->p.set.pids[i]>=0x20)break;
	if(i)
	{
		memmove(s->p.set.pids,&s->p.set.pids[i],
			(s->p.set.numpids-i)*sizeof(short));
		s->p.set.numpids-=i;
	}

	s->p.next=s->data->plist;
	s->data->plist=&s->p;
	pidupdate(s->data);

	if(!s->data->usecnt)
	{
		s->data->tune=*tune;
		satip_util_init_pids(&secset);

		secset.numpids=SATIP_SECTION;
		secset.pid=0;
		secset.table=0;
		if(!(s->data->f[0].handle=hwplay(s->data,&secset,NULL)))
			goto err1;

		if(pthread_create(&s->data->th,&g->attr,pidworker,s->data))
			goto err2;

		s->data->running=1;
	}
	s->data->usecnt++;

	return;

err2:	hwend(s->data,s->data->f[0].handle);
	s->data->f[0].handle=NULL;
err1:	s->data->plist=s->p.next;
	pidupdate(s->data);
	s->data=NULL;
	return;
}

void plugin_end(void *hwhandle,void *globalctx,void *streamctx)
{
	int i;
	uint64_t dummy;
	GLOBAL *g=(GLOBAL *)globalctx;
	STREAM *s=(STREAM *)streamctx;
	PIDITEM **d;
	struct pollfd p[2];

	if(!g||!s||!s->data)return;

	if(s->gotcsa)
	{
		p[0].fd=s->data->dfd;
		p[0].events=POLLIN;
		p[1].fd=s->data->xfd;
		p[1].events=POLLIN;

		while(poll(p,2,-1)<1);

		if(p[0].events&POLLIN)
		{
			dummy=read(s->data->dfd,&dummy,sizeof(dummy));
			pthread_spin_lock(&s->data->stx);
			s->data->csabusy=0;
			pthread_spin_unlock(&s->data->stx);
		}
	}

	if(s->data->running)
	{
		dummy=1;
		pthread_mutex_lock(&s->data->mtx);
		pidupdate(s->data);
		if(s->data->pat)dummy=write(s->data->efd,&dummy,sizeof(dummy));
		pthread_mutex_unlock(&s->data->mtx);
	}

	if(!--(s->data->usecnt)&&s->data->running)
	{
		s->data->term=1;
		dummy=1;
		dummy=write(s->data->efd,&dummy,sizeof(dummy));
		pthread_join(s->data->th,NULL);
		dummy=read(s->data->efd,&dummy,sizeof(dummy));

		for(i=0;i<SATIP_MAX_PIDS;i++)if(s->data->f[i].handle)
		{
			hwend(s->data,s->data->f[i].handle);
			s->data->f[i].handle=NULL;
		}

		if(s->data->total)newlist(s->data,NULL,0);

		for(i=0;i<SATIP_MAX_PIDS;i++)if(s->data->f[i].res)
		{
			free(s->data->f[i].res);
			s->data->f[i].res=NULL;
		}

		s->data->term=0;
		s->data->total=0;
		s->data->running=0;
		s->data->pat=NULL;
	}

	for(d=&s->data->plist;*d;d=&(*d)->next)if(*d==&s->p)
	{
		*d=s->p.next;
		break;
	}

	free(s);
}

void plugin_multi(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_STREAM **stream,int total,int totlen,int *done,int flush)
{
	GLOBAL *g=(GLOBAL *)globalctx;
	STREAM *s=(STREAM *)streamctx;
	unsigned char *mem;
	uint64_t dummy;
	int i;
	int j;
	int r;
	int pid;
	int idx;

	if(!g||!s||!s->data)
	{
		*done=total;
		return;
	}

	if(UNLIKELY(flush&&s->fill))goto ret;

	pthread_spin_lock(&s->data->stx);
	if(s->data->new)
	{
		mem=s->data->curr;
		s->data->curr=s->data->new;
		s->data->new=NULL;
		pthread_spin_unlock(&s->data->stx);
		if(mem)free(mem);
	}
	else pthread_spin_unlock(&s->data->stx);

	for(i=0,r=0;i<s->data->conf->maxdecode;i++)
	{
		pthread_spin_lock(&s->data->stx);
		if(s->data->desc[i].flags&0x04)s->data->desc[i].oe=0x00;
		s->data->desc[i].oe|=s->data->desc[i].flags&0x03;
		s->data->desc[i].flags=0x00;
		pthread_spin_unlock(&s->data->stx);
		if(s->data->desc[i].oe==0x03)r+=s->data->optlen;
	}

	if(!r)
	{
		s->fill=0;
		*done=total;
		return;
	}
	else if(LIKELY(!flush&&totlen-s->fill*1316<r))goto ret;

	if(s->gotcsa)
	{
		if(read(s->data->dfd,&dummy,sizeof(dummy))!=sizeof(dummy))
			goto ret;

		pthread_spin_lock(&s->data->stx);
		s->data->csabusy=0;
		pthread_spin_unlock(&s->data->stx);
		s->gotcsa=0;

		r=0;
		for(i=s->fill,r=0;i<total;i++,r++)
			for(j=0;j<stream[i]->fill;j+=188)
		{
			if(((unsigned char)stream[i]->data[j+3])&0xc0)
			{
				pid=((unsigned char)stream[i]->data[j+1])&0x1f;
				pid<<=8;
				pid|=(unsigned char)stream[i]->data[j+2];
				if((idx=s->data->curr[pid])!=0xff&&
					s->data->desc[idx].oe==0x03)goto out;
			}
		}
	}
	else
	{
		pthread_spin_lock(&s->data->stx);
		if(s->data->csabusy)
		{
			pthread_spin_unlock(&s->data->stx);
			goto ret;
		}
		s->data->csabusy=1;
		pthread_spin_unlock(&s->data->stx);

		s->gotcsa=1;

		for(i=s->fill;i<total;i++)for(r=0;r<stream[i]->fill;r+=188)
		{
			pid=((unsigned char)stream[i]->data[r+1])&0x1f;
			pid<<=8;
			pid|=(unsigned char)stream[i]->data[r+2];
			if((idx=s->data->curr[pid])!=0xff&&
				s->data->desc[idx].oe==0x03&&
				s->data->desc[idx].idx<s->data->optnum)
			{
				s->data->desc[idx].csa[s->data->desc[idx].idx++]
					=stream[i]->data+r;
				s->data->desc[idx].csa[s->data->desc[idx].idx++]
					=stream[i]->data+r+188;
			}
		}

		dummy=1;
		dummy=write(s->data->cfd,&dummy,sizeof(dummy));

		goto ret;
	}

out:	if(LIKELY(s->fill+=r))
	{
ret:		if(UNLIKELY(flush))
		{
			*done=s->fill;
			s->fill=0;
		}
		else switch(s->fill)
		{
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			*done=s->fill;
			s->fill=0;
			break;

		default:
			*done=8;
			s->fill-=8;
		case 0: break;

		}
	}
}
