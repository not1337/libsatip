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
	void *flt;
	void *sec;
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

typedef struct _piddata
{
	void (*plugin_pids)(void *ctx,SATIP_PIDS *set,int from_notify_hook);
	void *ctx;
	void *filter;
	unsigned char *new;
	unsigned char *curr;
	void *csactx;
	PMTDATA *msg;
	SATIP_UTIL_PAT *pat;
	SATIP_TUNE *tunefilter;
	FILTER f[SATIP_MAX_PIDS];
	SATIP_UTIL_PMT *list[MAXPIDS];
	short pids[SATIP_MAX_PIDS];
	SATIP_PIDS set;
	pthread_mutex_t mtx;
	pthread_mutex_t secmtx;
	pthread_spinlock_t stx;
	pthread_t th;
	pthread_t nw;
	pthread_t cw;
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
	int filtertotal;
	int optnum;
	int optlen;
	unsigned char pidmap[8192];
	unsigned char pmtidx[65536];
	CONFIG conf;
	DESCRAMBLE desc[MAXDECODE];
} PIDDATA;

typedef struct
{
	int fill;
	int gotcsa;
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

static void seccb(void *dta,int len,void *priv) __attribute__ ((hot));
static void patcb(void *dta,int len,void *priv) __attribute__ ((hot));
static void pmtcb(void *dta,int len,void *priv) __attribute__ ((hot));
static void *csaworker(void *dta)  __attribute__ ((hot));
void plugin_multi(void *plugin_ctx,void *strctx,unsigned char *ts188b1,int len1,
	unsigned char *ts188b2,int len2,int *rlen,int flush)
	__attribute__ ((hot));

static int docaids(void *s,char *val) __attribute__ ((cold));
static int doignore(void *s,char *val) __attribute__ ((cold));
static int dohost(void *s,char *val) __attribute__ ((cold));
static int doparam(CFGFILE *p,void *s,char *name,char *val)
	__attribute__ ((cold));
static int parse_config(char *fn,PIDDATA *data) __attribute__ ((cold));
void *plugin_init(char *config,void *filter_ctx,
	void (*plugin_pids)(void *ctx,SATIP_PIDS *set,int from_notify_hook),
	void *ctx) __attribute__ ((cold));
void plugin_exit(void *plugin_ctx) __attribute__ ((cold));

#else

static int docaids(void *s,char *val);
static int doignore(void *s,char *val);
static int dohost(void *s,char *val);

#endif

static CFGFILE cparams[]=
{
	{"host",0,0,offsetof(CONFIG,sock),dohost},
	{"port",1,65535,offsetof(CONFIG,port),NULL},
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

static int parse_config(char *fn,PIDDATA *data)
{
	int line=0;
	int freq=0;
	int sc=0;
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
			if(sc)goto err3;
			sc=1;
			c=&data->conf;
			if(freq)freq=-1;
			continue;
		}
		else if(!strcmp(name,"[transponders]"))
		{
			if(freq)goto err3;
			c=NULL;
			if(sc)sc=-1;
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
				if(!(t=realloc(data->tunefilter,
				    (data->filtertotal+1)*sizeof(SATIP_TUNE))))
				{
					fprintf(stderr,"Out of memory\n");
					goto err2;
				}
				data->tunefilter=t;
				t=&t[data->filtertotal++];
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

	if(!sc)
	{
		fprintf(stderr,"No softcam defined.\n");
		goto err2;
	}

	if(!data->conf.sock.s4.sin_family||!data->conf.port||
		!data->conf.maxdecode)goto err4;

	switch(data->conf.sock.s4.sin_family)
	{
	case AF_INET:
		data->conf.sock.s4.sin_port=htons(data->conf.port);
		break;
	case AF_INET6:
		data->conf.sock.s6.sin6_port=htons(data->conf.port);
		break;
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

static void seccb(void *dta,int len,void *priv)
{
	FILTER *f=(FILTER *)priv;
	struct iovec io[2];
	struct itimerspec it;

	io[0].iov_base=&f->msg;
	io[0].iov_len=sizeof(f->msg);
	io[1].iov_base=dta;
	io[1].iov_len=len;

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
	sigset_t set;
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
	SATIP_UTIL_SECFILTER sec;

	sigfillset(&set);
	pthread_sigmask(SIG_SETMASK,&set,NULL);

	pthread_setname_np(pthread_self(),"plugin network");

	p[0].fd=data->rfd;
	p[0].events=POLLIN;
	p[1].fd=data->tfd;
	p[1].events=POLLIN;

	for(x=0;x<SATIP_MAX_PIDS;x++)data->f[x].fd=&fd;

	data->stage=0;

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
stage1:			if((fd=socket(data->conf.sock.s4.sin_family,
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

			if((connect(fd,(struct sockaddr *)&data->conf.sock,
				sizeof(data->conf.sock)))<0)
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
					data->conf.adapter,PMT_LIST_ONLY);
				else for(x=0,r=0;x<curr->total&&!r;x++)
				{
					if(!x)r=pmtsend(fd,curr->list[0],
						data->conf.adapter,
						PMT_LIST_FIRST);
					else if(x+1==curr->total)
						r=pmtsend(fd,curr->list[x],
							data->conf.adapter,
							PMT_LIST_LAST);
					else r=pmtsend(fd,curr->list[x],
						data->conf.adapter,
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
				if(capid.adapter!=data->conf.adapter)break;
				capid.pid=ntohl(capid.pid);
				capid.index=ntohl(capid.index);
				if(capid.index!=-1&&
					capid.index>=data->conf.maxdecode)break;
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
				if(cadescr.adapter!=data->conf.adapter)break;
				cadescr.index=ntohl(cadescr.index);
				cadescr.parity=ntohl(cadescr.parity);
				if(cadescr.index>=data->conf.maxdecode)break;
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
				if(camode.adapter!=data->conf.adapter)break;
				camode.index=ntohl(camode.index);
				camode.algo=ntohl(camode.algo);
				camode.mode=ntohl(camode.mode);
				if(camode.index>=data->conf.maxdecode)break;
				break;

			case DVBAPI_DMX_SET_FILTER:
				if(read(fd,&dofilter,sizeof(dofilter))!=
					sizeof(dofilter))goto datafail;
				if(dofilter.adapter!=data->conf.adapter)break;
				dofilter.pid=ntohs(dofilter.pid);
				dofilter.timeout=ntohl(dofilter.timeout);
				dofilter.flags=ntohl(dofilter.flags);
				reuse=0;
				for(x=0;x<=max;x++)if(data->f[x].flt&&
					data->f[x].msg.demux==dofilter.demux&&
					data->f[x].msg.filter==dofilter.filter&&
					data->f[x].fpid==dofilter.pid)
				{
					reuse=1;
					break;
				}
				if(x>max)for(x=0;x<SATIP_MAX_PIDS;x++)
					if(!data->f[x].flt)break;
				if(x==SATIP_MAX_PIDS)break;
				if(reuse)
				{
					satip_util_filter_del_user(data->filter,
						data->f[x].flt);
					data->f[x].flt=NULL;
					satip_util_section_free(data->f[x].sec);
					data->f[x].sec=NULL;
				}
				else if(x>max)max=x;
				data->f[x].msg.demux=dofilter.demux;
				data->f[x].msg.filter=dofilter.filter;
				data->f[x].fpid=dofilter.pid;
				memcpy(sec.filter,dofilter.data,SATIP_FILTERSZ);
				memcpy(sec.mask,dofilter.mask,SATIP_FILTERSZ);
				memcpy(sec.mode,dofilter.mode,SATIP_FILTERSZ);
				if(satip_util_section_create(&data->f[x].sec,
					&sec,dofilter.flags&1,seccb,
					&data->f[x]))break;
				if(satip_util_filter_add_user(data->filter,
					&data->f[x].flt,
					satip_util_section_packet,
					data->f[x].sec))
				{
					satip_util_section_free(data->f[x].sec);
					data->f[x].sec=NULL;
					break;
				}
				if(satip_util_user_addpid(data->f[x].flt,
					dofilter.pid))
				{
					satip_util_filter_del_user(data->filter,
						data->f[x].flt);
					data->f[x].flt=NULL;
					satip_util_section_free(data->f[x].sec);
					data->f[x].sec=NULL;
					break;
				}
				if(reuse)break;
				data->set.numpids=0;
				for(x=0;x<=max;x++)if(data->f[x].flt)
				{
					for(r=0;r<data->set.numpids;r++)
						if(data->f[x].fpid==
						    data->set.pids[r])
							goto addskip;
					data->set.pids[data->set.numpids++]=
						data->f[x].fpid;
addskip:;
				}
				data->plugin_pids(data->ctx,&data->set,0);
				break;

			case DVBAPI_DMX_STOP:
				if(read(fd,&nofilter,sizeof(nofilter))!=
					sizeof(nofilter))goto datafail;
				if(nofilter.adapter!=data->conf.adapter)break;
				nofilter.pid=ntohs(nofilter.pid);
				for(x=0;x<=max;x++)if(data->f[x].flt)
				    if(data->f[x].msg.demux==nofilter.demux&&
					data->f[x].msg.filter==nofilter.filter&&
					data->f[x].fpid==nofilter.pid)
				{
					satip_util_filter_del_user(data->filter,
						data->f[x].flt);
					data->f[x].flt=NULL;
					satip_util_section_free(data->f[x].sec);
					data->f[x].sec=NULL;
					data->set.numpids=0;
					for(x=0;x<=max;x++)if(data->f[x].flt)
					{
						for(r=0;r<data->set.numpids;r++)
							if(data->f[x].fpid==
							    data->set.pids[r])
								goto delskip;
						data->set.pids[
							data->set.numpids++]=
								data->f[x].fpid;
delskip:;
					}
					data->plugin_pids(data->ctx,
						&data->set,0);
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
				if(ecminfo.adapter!=data->conf.adapter)break;
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
			data->set.numpids=0;
			data->plugin_pids(data->ctx,&data->set,0);
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
			for(x=0;x<data->conf.maxdecode;x++)
				data->desc[x].flags=0x04;
			pthread_spin_unlock(&data->stx);
			if(old)free(old);
			for(x=0;x<=max;x++)if(data->f[x].flt)
			{
				satip_util_filter_del_user(data->filter,
					data->f[x].flt);
				data->f[x].flt=NULL;
				satip_util_section_free(data->f[x].sec);
				data->f[x].sec=NULL;
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

	for(x=0;x<=max;x++)if(data->f[x].flt)
	{
		satip_util_filter_del_user(data->filter,data->f[x].flt);
		data->f[x].flt=NULL;
		satip_util_section_free(data->f[x].sec);
		data->f[x].sec=NULL;
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
		if(data->conf.pmtfilter&&data->conf.catotal)
		{
			for(k=0;k<data->conf.catotal;k++)
				if(data->conf.caids[k]==pmt->caid[i])
					goto prglen;
			continue;
		}
prglen:		l+=satip_util_get_raw_len(
			satip_util_get_raw_pmt(pmt,pmt->raw[i]));
	}

	for(i=0;i<pmt->total;i++)for(j=0;j<pmt->data[i].catotal;j++)
	{
		if(data->conf.pmtfilter&&data->conf.catotal)
		{
			for(k=0;k<data->conf.catotal;k++)
				if(data->conf.caids[k]==
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
		if(data->conf.pmtfilter&&data->conf.catotal)
		{
			for(k=0;k<data->conf.catotal;k++)
				if(data->conf.caids[k]==pmt->caid[i])
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
		if(data->conf.pmtfilter&&data->conf.catotal)
		{
			for(k=0;k<data->conf.catotal;k++)
				if(data->conf.caids[k]==
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

	for(i=0;i<data->conf.igntotal;i++)
		if(data->conf.ignore[i]==pmt->prognum)return -1;

	if(!data->conf.catotal)return 0;

	for(i=0;i<data->conf.catotal;i++)
	{
		for(k=0;k<pmt->catotal;k++)
			if(data->conf.caids[i]==pmt->caid[k])return 0;
		for(j=0;j<pmt->total;j++)for(k=0;k<pmt->data[j].catotal;j++)
			if(data->conf.caids[i]==pmt->data[j].caid[k])
				return 0;
	}

	return -1;
}

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

static void patcb(void *dta,int len,void *priv)
{
	PIDDATA *data=(PIDDATA *)priv;
	SATIP_UTIL_PAT *pat;
	uint64_t dummy=1;

	if(UNLIKELY(((unsigned char *)dta)[0]))return;
	if(UNLIKELY(!(pat=satip_util_unpack_pat_section(dta,len))))return;
	if(UNLIKELY(!pat->cnind||pat->secnum))goto out;

	pthread_spin_lock(&data->stx);
	if(!data->f[0].res)
	{
		data->f[0].res=pat;
		pthread_spin_unlock(&data->stx);
		dummy=write(data->efd,&dummy,sizeof(dummy));
		return;
	}
	pthread_spin_unlock(&data->stx);

out:	free(pat);
}

static void pmtcb(void *dta,int len,void *priv)
{
	int i;
	int idx;
	PIDDATA *data=(PIDDATA *)priv;
	SATIP_UTIL_PAT *pat=(SATIP_UTIL_PAT *)data->f[0].res;
	SATIP_UTIL_PMT *pmt;
	uint64_t dummy=1;

	if(UNLIKELY(((unsigned char *)dta)[0]!=0x02))return;
	if(UNLIKELY(!(pmt=satip_util_unpack_pmt_section(dta,len))))return;
	if(UNLIKELY(!pmt->cnind||pmt->secnum))goto out;

	pthread_spin_lock(&data->stx);
	if((idx=data->pmtidx[pmt->prognum]))
		if(!data->f[idx].res)for(i=0;i<pat->total;i++)
			if(pmt->prognum==pat->data[i].prognum)
	{
		pat->data[i].pmt=pmt;
		data->f[idx].res=pmt;
		pthread_spin_unlock(&data->stx);
		dummy=write(data->efd,&dummy,sizeof(dummy));
		return;
	}
	pthread_spin_unlock(&data->stx);

out:	free(pmt);
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
	SATIP_UTIL_PMT *list[MAXPIDS];
	SATIP_UTIL_SECFILTER flt;
	struct pollfd p;
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_SETMASK,&set,NULL);

	pthread_setname_np(pthread_self(),"plugin pidwork");

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
				for(i=1;i<pmttot;i++)if(data->f[i].flt)
				{
					satip_util_filter_del_user(data->filter,
						data->f[i].flt);
					data->f[i].flt=NULL;
					satip_util_section_free(data->f[i].sec);
					data->f[i].sec=NULL;
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
			satip_util_filter_del_user(data->filter,data->f[0].flt);
			data->f[0].flt=NULL;
			satip_util_section_free(data->f[0].sec);
			data->f[0].sec=NULL;
			pat=(SATIP_UTIL_PAT *)data->f[0].res;
			data->set.numpids=1;
			data->set.pids[0]=1;
			memset(data->pmtidx,0,sizeof(data->pmtidx));
			memset(&flt,0,sizeof(flt));
			flt.mask[0]=0xff;
			flt.filter[0]=0x02;
			for(i=1,j=0,totcnt=1;i<SATIP_MAX_PIDS&&j<pat->total;j++)
				if(pat->data[j].prognum)
			{
				data->pmtidx[pat->data[j].prognum]=i;
				for(k=1;k<i;k++)
					if(data->f[k].fpid==pat->data[j].pmtpid)
						break;
				if(k<i)
				{
					if(satip_util_user_addpid(
						data->f[k].flt,
						pat->data[j].pmtpid))continue;
					totcnt++;
					continue;
				}

				if(satip_util_section_create(&data->f[i].sec,
					&flt,1,pmtcb,data))continue;

				if(satip_util_filter_add_user(data->filter,
					&data->f[i].flt,
					satip_util_section_packet,
					data->f[i].sec))
				{
					satip_util_section_free(data->f[i].sec);
					data->f[i].sec=NULL;
					continue;
				}

				if(satip_util_user_addpid(data->f[i].flt,
					pat->data[j].pmtpid))
				{
					satip_util_filter_del_user(data->filter,
						data->f[i].flt);
					data->f[i].flt=NULL;
					satip_util_section_free(data->f[i].sec);
					data->f[i].sec=NULL;
					continue;
				}

				data->set.pids[data->set.numpids++]=
					pat->data[j].pmtpid;
				data->f[i++].fpid=pat->data[j].pmtpid;
				totcnt++;
			}
			pmttot=i;
			data->plugin_pids(data->ctx,&data->set,0);
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
				satip_util_filter_del_user(data->filter,
					data->f[i].flt);
				data->f[i].flt=NULL;
				satip_util_section_free(data->f[i].sec);
				data->f[i].sec=NULL;
			}
			tmo=-1;
			state=2;

		case 2:	data->set.numpids=0;
			data->plugin_pids(data->ctx,&data->set,0);

			qsort(&pat->data[0],pat->total,sizeof(pat->data[0]),
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

		for(i=0;i<data->conf.maxdecode;i++)
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

void *plugin_init(char *config,void *filter_ctx,
	void (*plugin_pids)(void *ctx,SATIP_PIDS *set,int from_notify_hook),
	void *ctx)
{
	int i;
	uint64_t dummy=1;
	PIDDATA *data;
	pthread_attr_t attr;

	if(!(data=malloc(sizeof(PIDDATA))))goto err1;
	memset(data,0,sizeof(PIDDATA));
	if(!(data->curr=malloc(sizeof(data->pidmap))))goto err2;
	memset(data->pidmap,0xff,sizeof(data->pidmap));
	memset(data->curr,0xff,sizeof(data->pidmap));
	for(i=0;i<SATIP_MAX_PIDS;i++)
	{
		data->f[i].msg.opcode=htonl(DVBAPI_FILTER_DATA);
		data->f[i].data=data;
	}
	if(parse_config(config,data))goto err2;
	ffdecsa_select_mode(&data->csactx,data->conf.csamode);
	data->optnum=ffdecsa_get_suggested_cluster_size(data->csactx);
	if(data->optnum>MAX_ENTRIES)data->optnum=MAX_ENTRIES;
	data->optlen=data->optnum*188;
	data->optnum<<=1;
	data->plugin_pids=plugin_pids;
	data->ctx=ctx;
	data->filter=filter_ctx;
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
	for(i=0;i<data->conf.maxdecode;i++)
		if(!(data->desc[i].keys=ffdecsa_get_key_struct(data->csactx)))
			goto err12;
	if(pthread_attr_init(&attr))goto err12;
	if(pthread_attr_setstacksize(&attr,STACKSIZE))goto err13;
	if(pthread_create(&data->cw,&attr,csaworker,data))goto err13;
	if(pthread_create(&data->nw,&attr,networker,data))goto err14;
	pthread_attr_destroy(&attr);
	return data;

err14:	dummy=write(data->xfd,&dummy,sizeof(dummy));
	pthread_join(data->cw,NULL);
err13:	pthread_attr_destroy(&attr);
err12:	for(i=0;i<data->conf.maxdecode;i++)
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
err2:	if(data->curr)free(data->curr);
	if(data->tunefilter)free(data->tunefilter);
	free(data);
err1:	return NULL;
}

void plugin_exit(void *plugin_ctx)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
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
	{
		if(data->f[i].flt)
			satip_util_filter_del_user(data->filter,data->f[i].flt);
		if(data->f[i].sec)satip_util_section_free(data->f[i].sec);
	}
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
	if(data->tunefilter)free(data->tunefilter);
	if(data->new)free(data->new);
	if(data->curr)free(data->curr);
	for(i=0;i<data->conf.maxdecode;i++)if(data->desc[i].keys)
		ffdecsa_free_key_struct(data->csactx,data->desc[i].keys);
	free(data);
}

void plugin_notify_pids(void *plugin_ctx,short *pids,int total)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	int i;
	uint64_t dummy=1;

	if(!data)return;

	qsort(pids,total,sizeof(short),pidcmp);

	for(i=0;i<total;i++)if(pids[i]>=0x20)break;

	pthread_mutex_lock(&data->mtx);
	memcpy(data->pids,&pids[i],(total-i)*sizeof(short));
	data->pidtotal=total-i;
	if(data->pat)dummy=write(data->efd,&dummy,sizeof(dummy));
	pthread_mutex_unlock(&data->mtx);
}

void plugin_pre_tune(void *plugin_ctx,SATIP_TUNE *tune,SATIP_EXTRA **extra)
{
	int i;
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	SATIP_UTIL_SECFILTER flt;
	uint64_t dummy=1;

	if(!data)return;

	memset(&flt,0,sizeof(flt));
	flt.mask[0]=0xff;
	flt.filter[0]=0x00;

	if(data->running)
	{
		data->term=1;
		dummy=write(data->efd,&dummy,sizeof(dummy));
		pthread_join(data->th,NULL);
		data->running=0;
	}

	data->skip=0;

	if(data->filtertotal)
	{
		for(i=0;i<data->filtertotal;i++)
			if(!satip_util_tunecmp(tune,&data->tunefilter[i]))
				goto work;
		goto err1;
	}

work:	satip_util_init_pids(&data->set);

	data->set.numpids=2;
	data->set.pids[0]=0;
	data->set.pids[1]=1;

	if(satip_util_section_create(&data->f[0].sec,&flt,1,patcb,data))
		goto err1;

	if(satip_util_filter_add_user(data->filter,&data->f[0].flt,
		satip_util_section_packet,data->f[0].sec))goto err2;

	if(satip_util_user_addpid(data->f[0].flt,0))goto err3;

	data->plugin_pids(data->ctx,&data->set,0);

	if(data->total)
	{
		newlist(data,NULL,0);
		data->total=0;
	}
	data->term=0;
	data->pat=NULL;
	dummy=read(data->efd,&dummy,sizeof(dummy));
	return;

err3:	satip_util_filter_del_user(data->filter,data->f[0].flt);
	data->f[0].flt=NULL;
err2:	satip_util_section_free(data->f[0].sec);
	data->f[0].sec=NULL;
err1:	data->skip=1;
	return;
}

void plugin_post_tune(void *plugin_ctx,SATIP_TUNE *tune,int success)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	pthread_attr_t attr;

	if(!data||data->skip)goto err1;

	if(pthread_attr_init(&attr))goto err1;
	if(pthread_attr_setstacksize(&attr,STACKSIZE))goto err2;

	if(!success||pthread_create(&data->th,&attr,pidworker,data))
	{
		satip_util_filter_del_user(data->filter,data->f[0].flt);
		data->f[0].flt=NULL;
		satip_util_section_free(data->f[0].sec);
		data->f[0].sec=NULL;
		if(data->f[0].res)free(data->f[0].res);
		data->f[0].res=NULL;
		data->set.numpids=0;
		data->plugin_pids(data->ctx,&data->set,0);
err2:		pthread_attr_destroy(&attr);
err1:		return;
	}
	pthread_attr_destroy(&attr);
	data->running=1;
}

void plugin_no_tune(void *plugin_ctx)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	int i;
	uint64_t dummy=1;

	if(!data)return;

	if(data->running)
	{
		data->term=1;
		dummy=write(data->efd,&dummy,sizeof(dummy));
		pthread_join(data->th,NULL);
		data->running=0;
	}
	for(i=0;i<SATIP_MAX_PIDS;i++)
	{
		if(data->f[i].flt)
		{
			satip_util_filter_del_user(data->filter,data->f[i].flt);
			data->f[i].flt=NULL;
		}
		if(data->f[i].sec)
		{
			satip_util_section_free(data->f[i].sec);
			data->f[i].sec=NULL;
		}
	}

	if(data->total)
	{
		newlist(data,NULL,0);
		data->total=0;
	}
	else
	{
		data->set.numpids=0;
		data->plugin_pids(data->ctx,&data->set,0);
	}

	for(i=0;i<SATIP_MAX_PIDS;i++)if(data->f[i].res)
	{
		free(data->f[i].res);
		data->f[i].res=NULL;
	}

	data->term=0;
	data->total=0;
	data->pat=NULL;
}

void plugin_strinit(void *plugin_ctx,void **strctx)
{
	if(!(*strctx=malloc(sizeof(STREAM))))return;
	memset(*strctx,0,sizeof(STREAM));
}

void plugin_strexit(void *plugin_ctx,void *strctx)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	STREAM *s=(STREAM *)strctx;
	uint64_t dummy;
	struct pollfd p[2];

	if(!data||!s)return;

	if(s->gotcsa)
	{
		p[0].fd=data->dfd;
		p[0].events=POLLIN;
		p[1].fd=data->xfd;
		p[1].events=POLLIN;

		while(poll(p,2,-1)<1);

		if(p[0].events&POLLIN)
		{
			dummy=read(data->dfd,&dummy,sizeof(dummy));
			pthread_spin_lock(&data->stx);
			data->csabusy=0;
			pthread_spin_unlock(&data->stx);
		}
	}

	free(s);
}

void plugin_multi(void *plugin_ctx,void *strctx,unsigned char *ts188b1,int len1,
	unsigned char *ts188b2,int len2,int *rlen,int flush)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	STREAM *s=(STREAM *)strctx;
	unsigned char *mem;
	uint64_t dummy;
	int i;
	int r;
	int pid;
	int idx;

	if(UNLIKELY(!data||!s))
	{
		*rlen=len1+len2;
		return;
	}

	if(UNLIKELY(flush&&s->fill))goto ret;

	pthread_spin_lock(&data->stx);
	if(data->new)
	{
		mem=data->curr;
		data->curr=data->new;
		data->new=NULL;
		pthread_spin_unlock(&data->stx);
		if(mem)free(mem);
	}
	else pthread_spin_unlock(&data->stx);

	for(i=0,r=0;i<data->conf.maxdecode;i++)
	{
		pthread_spin_lock(&data->stx);
		if(data->desc[i].flags&0x04)data->desc[i].oe=0x00;
		data->desc[i].oe|=data->desc[i].flags&0x03;
		data->desc[i].flags=0x00;
		pthread_spin_unlock(&data->stx);
		if(data->desc[i].oe==0x03)r+=data->optlen;
	}

	if(!r)
	{
		s->fill=0;
		*rlen=len1+len2;
		return;
	}
	else if(LIKELY(!flush&&len1+len2-s->fill<r))goto ret;

	if(s->gotcsa)
	{
		if(read(data->dfd,&dummy,sizeof(dummy))!=sizeof(dummy))goto ret;

		pthread_spin_lock(&data->stx);
		data->csabusy=0;
		pthread_spin_unlock(&data->stx);
		s->gotcsa=0;

		r=0;
		if(s->fill<len1)for(i=s->fill;i<len1;i+=188)
		{
			if(!(ts188b1[i+3]&0xc0))r+=188;
			else
			{
				pid=ts188b1[i+1]&0x1f;
				pid<<=8;
				pid|=ts188b1[i+2];
				if((idx=data->curr[pid])!=0xff&&
					data->desc[idx].oe==0x03)goto out;
				r+=188;
			}
		}

		if(s->fill>=len1)for(i=s->fill-len1;i<len2;i+=188)
		{
			if(!(ts188b2[i+3]&0xc0))r+=188;
			else
			{
				pid=ts188b2[i+1]&0x1f;
				pid<<=8;
				pid|=ts188b2[i+2];
				if((idx=data->curr[pid])!=0xff&&
					data->desc[idx].oe==0x03)goto out;
				r+=188;
			}
		}
	}
	else
	{
		pthread_spin_lock(&data->stx);
		if(data->csabusy)
		{
			pthread_spin_unlock(&data->stx);
			goto ret;
		}
		data->csabusy=1;
		pthread_spin_unlock(&data->stx);

		s->gotcsa=1;

		if(s->fill<len1)for(i=s->fill;i<len1;i+=188)
		{
			pid=ts188b1[i+1]&0x1f;
			pid<<=8;
			pid|=ts188b1[i+2];
			if((idx=data->curr[pid])!=0xff&&
				data->desc[idx].oe==0x03&&
				data->desc[idx].idx<data->optnum)
			{
				data->desc[idx].csa[data->desc[idx].idx++]=
					ts188b1+i;
				data->desc[idx].csa[data->desc[idx].idx++]=
					ts188b1+i+188;
			}
		}

		if(s->fill>=len1)for(i=s->fill-len1;i<len2;i+=188)
		{
			pid=ts188b2[i+1]&0x1f;
			pid<<=8;
			pid|=ts188b2[i+2];
			if((idx=data->curr[pid])!=0xff&&
				data->desc[idx].oe==0x03&&
				data->desc[idx].idx<data->optnum)
			{
				data->desc[idx].csa[data->desc[idx].idx++]=
					ts188b2+i;
				data->desc[idx].csa[data->desc[idx].idx++]=
					ts188b2+i+188;
			}
		}

		dummy=1;
		dummy=write(data->cfd,&dummy,sizeof(dummy));

		goto ret;
	}

out:	if(LIKELY(s->fill+=r))
	{
ret:		if(UNLIKELY(flush))
		{
			*rlen=s->fill;
			s->fill=0;
		}
		else switch(s->fill)
		{
		case 188:
		case 376:
		case 564:
		case 752:
		case 940:
		case 1128:
			*rlen=s->fill;
			s->fill=0;
			break;

		default:
			*rlen=1316;
			s->fill-=1316;
		case 0:	break;

		}
	}
}
