/*
   This file is part of the satip library.

   When building the shared library:

   The satip Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   When building not for the shared library:

   The satip Library is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation, version 2.

   (c) 2016 Andreas Steinmetz ast@domdv.de
 */

#include <pthread.h>
#include <sys/signalfd.h>
#include <sys/timerfd.h>
#include <sys/eventfd.h>
#include <signal.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include "satip.h"
#include "common.h"

#ifdef __GNUC__
#pragma GCC diagnostic ignored "-Wmisleading-indentation"
#endif

#define MAXPMT		32
#define DVBCTMHZ	2048
#define DVBSMHZ		24576
#define TRANSPONDERS	1024
#define STACKSIZE	131072

#define SCAN_OK		0x00000001
#define NO_MORE		0x00000002
#define MATCHED		0x00000004
#define GET_NIT		0x00000008
#define INITIAL		0x00000010
#define NEEDTUNE	0x00000020
#define TUNETEST	0x00000040
#define AUTOBW		0x00000080
#define AUTOTMODE	0x00000100
#define AUTOMTYPE	0x00000200
#define AUTOGI		0x00000400
#define AUTOFEC		0x00000800
#define AUTOSPI		0x00001000
#define CAN2G		0x00002000
#define AUTOFECLP	0x00004000
#define AUTOHIER	0x00008000

typedef struct
{
	int level;
	int lock;
	int quality;
	int items;
	int done;
	int work;
	int totpmt;
	int prognum[MAXPMT];
	unsigned int flags;
	void *filter;
	pthread_spinlock_t mtx;
	SATIP_SCAN_RESULT *tp;
	SATIP_UTIL_PMT *pmt[MAXPMT];
	SATIP_UTIL_PMT **pmtdest[MAXPMT];
	SATIP_TUNE tune;
} TRANSPONDER;

typedef struct
{
	pthread_spinlock_t mtx;
	int done;
	int items;
	void (*status)(SATIP_TUNE *t,SATIP_SCAN_STATUS *st,void *priv);
	void *priv;
} GLOBALS;

typedef struct _dvbsfreq
{
	struct _dvbsfreq *next;
	struct _dvbsfreq *job;
	struct _dvbsfreq *chain;
	unsigned long long low;
	unsigned long long high;
	GLOBALS *g;
	SATIP_SCAN_RESULT tp;
	SATIP_TUNE tune;
	int tuner;
	int flags;
} DVBSFREQ;

typedef struct _dvbctfreq
{
	struct _dvbctfreq *next;
	GLOBALS *g;
	SATIP_SCAN_RESULT tp;
	SATIP_TUNE tune;
	int bits;
	int tuner;
} DVBCTFREQ;

typedef struct
{
	pthread_spinlock_t mtx;
	SATIP_SCAN_PARAMS cfg;
	union
	{
		DVBSFREQ *e;
        	DVBCTFREQ *f;
	};
	void *h;
	char *host;
	int port;
	int mode;
	int r;
	int workfd;
	int donefd;
	int termfd;
	int state;
	int totsym;
	int *symtab;
	pthread_t th;
} SCANJOB;

#ifndef PROFILE

static void rtspwrk(SATIP_DATA *data,void *priv) __attribute__ ((hot));
static void rtspsec(void *data,int len,void *priv) __attribute__ ((hot));
static void httpstr(SATIP_DATA *data,void *priv) __attribute__ ((hot));
static void hwstream(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void dosection(TRANSPONDER *t,unsigned char *data,int size)
	__attribute__ ((hot));
static void nullstream(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void nullhttp(SATIP_DATA *data,void *priv) __attribute__ ((hot));
static void nullrtsp(SATIP_DATA *data,void *priv) __attribute__ ((hot));

#endif

static void nullstream(void *id,SATIP_STREAM *stream)
{
}

static void nullstatus(void *id,SATIP_STATUS *status)
{
}

static void nullhttp(SATIP_DATA *data,void *priv)
{
}

static void nullrtsp(SATIP_DATA *data,void *priv)
{
}

static int lookup(SATIP_TUNE *tune,DVBSFREQ **list,int total,int base)
{
	int offset;
	int delta;

	if(!base)return -1;

	offset=delta=base>>1;

	while(1)
	{
		if(offset<total)
		{
			if(tune->freq<list[offset]->low)offset-=delta;
			else if(tune->freq>list[offset]->high)offset+=delta;
			else return offset;
		}
		else offset-=delta;

		if(!delta||offset<0)return -1;
		delta>>=1;
	}
}

static void dvbctscancb(SATIP_SCAN_STATUS *st,void *priv)
{
	DVBCTFREQ *e=(DVBCTFREQ *)priv;

	if(!e->g->status)return;
	st->globdone=e->g->done+1;
	st->globtotal=e->g->items;
	st->tuner=e->tuner;
	e->g->status(&e->tune,st,e->g->priv);
}

static void dvbctmultiscancb(SATIP_SCAN_STATUS *st,void *priv)
{
	DVBCTFREQ *e=(DVBCTFREQ *)priv;

	if(!e->g->status)return;
	pthread_spin_lock(&e->g->mtx);
	st->globdone=e->g->done+1;
	st->globtotal=e->g->items;
	st->tuner=e->tuner;
	pthread_spin_unlock(&e->g->mtx);
	e->g->status(&e->tune,st,e->g->priv);
}

static void dvbsscancb(SATIP_SCAN_STATUS *st,void *priv)
{
	DVBSFREQ *e=(DVBSFREQ *)priv;

	if(!e->g->status)return;
	st->globdone=e->g->done+1;
	st->globtotal=e->g->items;
	st->tuner=e->tuner;
	e->g->status(&e->tune,st,e->g->priv);
}

static void dvbsmultiscancb(SATIP_SCAN_STATUS *st,void *priv)
{
	DVBSFREQ *e=(DVBSFREQ *)priv;

	if(!e->g->status)return;
	pthread_spin_lock(&e->g->mtx);
	st->globdone=e->g->done+1;
	st->globtotal=e->g->items;
	st->tuner=e->tuner;
	pthread_spin_unlock(&e->g->mtx);
	e->g->status(&e->tune,st,e->g->priv);
}

static void *dvbsslave(void *data)
{
	int n=1;
	uint64_t dummy;
	SCANJOB *job=(SCANJOB *)data;
	struct pollfd p[2];
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_BLOCK,&set,NULL);

	pthread_setname_np(pthread_self(),"scan dvbsslave");

	p[0].fd=job->workfd;
	p[0].events=POLLIN;
	p[1].fd=job->termfd;
	p[1].events=POLLIN;
	p[1].revents=0;

	if(job->termfd!=-1)n=2;

	while(1)
	{
		if(poll(p,n,-1)<1)continue;
		if(p[1].revents&POLLIN)break;
		if(!(p[0].revents&POLLIN))break;
		if(read(job->workfd,&dummy,sizeof(dummy))!=sizeof(dummy))
			continue;
		if(!job->e)break;

		switch(job->mode)
		{
		case SATIP_SCAN_HW:
			job->r=satip_scan_transponder_hw(job->h,&job->e->tune,
				&job->cfg,&job->e->tp,dvbsmultiscancb,job->e);
			break;
		case SATIP_SCAN_HTTP:
			job->r=satip_scan_transponder_http(job->h,&job->e->tune,
				&job->cfg,&job->e->tp,dvbsmultiscancb,job->e,
				job->host,job->port);
			break;
		case SATIP_SCAN_RTSP:
			job->r=satip_scan_transponder_rtsp(job->h,&job->e->tune,
				&job->cfg,&job->e->tp,dvbsmultiscancb,job->e,
				job->host,job->port);
			break;
		default:job->r=SATIP_SYSFAIL;
			break;
		}

		pthread_spin_lock(&job->mtx);
		job->state=2;
		pthread_spin_unlock(&job->mtx);

		dummy=1;
		dummy=write(job->donefd,&dummy,sizeof(dummy));
	}

	pthread_spin_lock(&job->mtx);
	job->state=-1;
	pthread_spin_unlock(&job->mtx);

	dummy=1;
	dummy=write(job->donefd,&dummy,sizeof(dummy));

	pthread_exit(NULL);
}

static int dvbcttunetest(void *h,SATIP_TUNE *t,int mode,int termfd,char *host,
	int port)
{
	SATIP_STRDATA param;
	SATIP_PIDS set;
	void *handle;

	switch(mode&0xf0)
	{
	case SATIP_SCAN_HW:
		set.prognum=0;
		set.numpids=SATIP_NOPIDS;
		param.tune=t;
		param.set=&set;
		param.handle=NULL;
		param.terminate=termfd;
		switch(satip_hw_play(h,&param,nullstream,nullstatus,NULL,
			SATIP_HW_LOCAL))
		{
		case 0:	satip_hw_end(h,&param);
			return 0;
		case SATIP_TUNERERR:
			return 1;
		default:return -1;
		}
	case SATIP_SCAN_HTTP:
		set.numpids=SATIP_SIGNAL;
		switch(satip_cln_stream_http(h,host,port,t,&set,NULL,nullhttp,
			NULL,&handle))
		{
		case 0:	satip_cln_stream_stop(h,handle);
			return 0;
		case 400:
		case 403:
		case 500:
		case 503:
			return 1;
		default:return -1;
		}
	case SATIP_SCAN_RTSP:
		set.numpids=SATIP_NOPIDS;
		switch(satip_cln_stream_unicast(h,host,port,1,t,&set,NULL,
			nullrtsp,NULL,&handle))
		{
		case 0:	satip_cln_stream_stop(h,handle);
			return 0;
		case 400:
		case 403:
		case 500:
		case 503:
			return 1;
		default:return -1;
		}
	default:return -1;
	}
}

static int dvbtscanchan(void *h,SATIP_TUNE *t,SATIP_SCAN_PARAMS *cfg,
	SATIP_SCAN_RESULT *tp,void (*status)(SATIP_SCAN_STATUS *st,void *priv),
	void *priv,int mode,int bits,char *host,int port)
{
	int j;
	int k;
	int l;
	int m;
	int n;
	int o;
	int p;
	int q;
	int r;
	SATIP_TUNE tune;

	tune=*t;
	for(j=0;j<6;j++)
	{
	  if((bits&AUTOBW)||tune.bw!=SATIP_BW_AUTO)j=6;
	  else switch(j)
	  {
	  case 0:tune.bw=SATIP_BW_1712;
		 break;
	  case 1:tune.bw=SATIP_BW_5;
		 break;
	  case 2:tune.bw=SATIP_BW_6;
		 break;
	  case 3:tune.bw=SATIP_BW_7;
		 break;
	  case 4:tune.bw=SATIP_BW_8;
		 break;
	  case 5:tune.bw=SATIP_BW_10;
		 break;
	  }
	  for(k=0;k<6;k++)
	  {
	    if((bits&AUTOTMODE)||tune.tmode!=SATIP_TMOD_AUTO)k=6;
	    else switch(k)
	    {
	    case 0:tune.tmode=SATIP_TMOD_1K;
		   break;
	    case 1:tune.tmode=SATIP_TMOD_2K;
		   break;
	    case 2:tune.tmode=SATIP_TMOD_4K;
		   break;
	    case 3:tune.tmode=SATIP_TMOD_8K;
		   break;
	    case 4:tune.tmode=SATIP_TMOD_16K;
		   break;
	    case 5:tune.tmode=SATIP_TMOD_32K;
		   break;
	    }
	    for(l=0;l<4;l++)
	    {
	      if((bits&AUTOMTYPE)||tune.mtype!=SATIP_AUTOQ)l=4;
	      else switch(l)
	      {
	      case 0:tune.mtype=SATIP_QPSK;
		     break;
	      case 1:tune.mtype=SATIP_16Q;
		     break;
	      case 2:tune.mtype=SATIP_64Q;
		     break;
	      case 3:tune.mtype=SATIP_256Q;
		     break;
	      }
	      for(m=0;m<8;m++)
	      {
		if((bits&AUTOGI)||tune.gi!=SATIP_GI_AUTO)m=8;
		else switch(m)
		{
		case 0:tune.gi=SATIP_GI_14;
		       break;
		case 1:tune.gi=SATIP_GI_18;
		       break;
		case 2:tune.gi=SATIP_GI_116;
		       break;
		case 3:tune.gi=SATIP_GI_132;
		       break;
		case 4:tune.gi=SATIP_GI_164;
		       break;
		case 5:tune.gi=SATIP_GI_1128;
		       break;
		case 6:tune.gi=SATIP_GI_19128;
		       break;
		case 7:tune.gi=SATIP_GI_19256;
		       break;
		}
		for(n=0;n<7;n++)
		{
		  if((bits&AUTOFEC)||tune.fec!=SATIP_FEC_AUTO)n=7;
		  else switch(n)
		  {
		  case 0:tune.fec=SATIP_FEC_12;
			 break;
		  case 1:tune.fec=SATIP_FEC_23;
			 break;
		  case 2:tune.fec=SATIP_FEC_34;
			 break;
		  case 3:tune.fec=SATIP_FEC_35;
			 break;
		  case 4:tune.fec=SATIP_FEC_45;
			 break;
		  case 5:tune.fec=SATIP_FEC_56;
			 break;
		  case 6:tune.fec=SATIP_FEC_78;
			 break;
		  }
		  for(p=0;p<7;p++)
		  {
		    if((bits&AUTOFECLP)||tune.feclp!=SATIP_FEC_AUTO)p=7;
		    else switch(p)
		    {
		    case 0:tune.feclp=SATIP_FEC_12;
			   break;
		    case 1:tune.feclp=SATIP_FEC_23;
			   break;
		    case 2:tune.feclp=SATIP_FEC_34;
			   break;
		    case 3:tune.feclp=SATIP_FEC_35;
			   break;
		    case 4:tune.feclp=SATIP_FEC_45;
			   break;
		    case 5:tune.feclp=SATIP_FEC_56;
			   break;
		    case 6:tune.feclp=SATIP_FEC_78;
			   break;
		    }
		    for(q=0;q<4;q++)
		    {
		      if((bits&AUTOHIER)||tune.hier!=SATIP_HIER_AUTO)q=4;
		      else switch(q)
		      {
		      case 0:tune.hier=SATIP_HIER_NONE;
			     break;
		      case 1:tune.hier=SATIP_HIER_1;
			     break;
		      case 2:tune.hier=SATIP_HIER_2;
			     break;
		      case 3:tune.hier=SATIP_HIER_4;
			     break;
		      }
		      for(o=0;o<2;o++)
		      {
			if(!(bits&CAN2G)||tune.msys==SATIP_DVBT2)o=2;
			else switch(o)
			{
			case 0:tune.msys=SATIP_DVBT;
			       break;
			case 1:tune.msys=SATIP_DVBT2;
			       break;
			}

			switch(mode&0xf0)
			{
			case SATIP_SCAN_HW:
			  r=satip_scan_transponder_hw(h,&tune,cfg,tp,status,
				priv);
			  if(!r||r==SATIP_PARTIAL)return r;
			  if(r!=SATIP_TUNERERR)return r;
			  break;
			case SATIP_SCAN_HTTP:
			  r=satip_scan_transponder_http(h,&tune,cfg,tp,status,
				priv,host,port);
			  if(!r||r==SATIP_PARTIAL)return r;
			  if(r!=403&&r!=500&&r!=503)return r;
			  break;
			case SATIP_SCAN_RTSP:
			  r=satip_scan_transponder_rtsp(h,&tune,cfg,tp,status,
				priv,host,port);
			  if(!r||r==SATIP_PARTIAL)return r;
			  if(r!=403&&r!=500&&r!=503)return r;
			  break;
			default:
			  return SATIP_SYSFAIL;
			}
		      }
		    }
		  }
		}
	      }
	    }
	  }
	}

	return SATIP_NODATA;
}

static void *dvbtslave(void *data)
{
	int n=1;
	uint64_t dummy;
	SCANJOB *job=(SCANJOB *)data;
	struct pollfd p[2];
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_BLOCK,&set,NULL);

	pthread_setname_np(pthread_self(),"scan dvbtslave");

	p[0].fd=job->workfd;
	p[0].events=POLLIN;
	p[1].fd=job->termfd;
	p[1].events=POLLIN;
	p[1].revents=0;

	if(job->termfd!=-1)n=2;
	while(1)
	{
		if(poll(p,n,-1)<1)continue;
		if(p[1].revents&POLLIN)break;
		if(!(p[0].revents&POLLIN))break;
		if(read(job->workfd,&dummy,sizeof(dummy))!=sizeof(dummy))
			continue;
		if(!job->f)break;

		job->r=dvbtscanchan(job->h,&job->f->tune,&job->cfg,&job->f->tp,
			dvbctmultiscancb,job->f,job->mode,job->f->bits,
			job->host,job->port);

		pthread_spin_lock(&job->mtx);
		job->state=2;
		pthread_spin_unlock(&job->mtx);
		
		dummy=1;
		dummy=write(job->donefd,&dummy,sizeof(dummy));
	}       
	
	pthread_spin_lock(&job->mtx);
	job->state=-1;
	pthread_spin_unlock(&job->mtx);

	dummy=1;
	dummy=write(job->donefd,&dummy,sizeof(dummy));

	pthread_exit(NULL);
}

static int dvbcscanchan(void *h,SATIP_TUNE *t,SATIP_SCAN_PARAMS *cfg,
	SATIP_SCAN_RESULT *tp,void (*status)(SATIP_SCAN_STATUS *st,void *priv),
	void *priv,int mode,int bits,int *symtab,int totsym,char *host,int port)
{
	int j;
	int k;
	int l;
	int r;
	SATIP_TUNE tune;

	tune=*t;
	for(j=0;j<2;j++)
	{
	  if((bits&AUTOSPI)||tune.specinv!=SATIP_SPI_AUTO)j=2;
	  else switch(j)
	  {
	  case 0:tune.specinv=SATIP_SPI_OFF;
		 break;
	  case 1:tune.specinv=SATIP_SPI_ON;
		 break;
	  }
          for(k=0;k<5;k++)
	  {
	    if((bits&AUTOMTYPE)||tune.mtype!=SATIP_AUTOQ)k=5;
	    else switch(k)
	    {
	    case 0:tune.mtype=SATIP_16Q;
		   break;
	    case 1:tune.mtype=SATIP_32Q;
		   break;
	    case 2:tune.mtype=SATIP_64Q;
		   break;
	    case 3:tune.mtype=SATIP_128Q;
		   break;
	    case 4:tune.mtype=SATIP_256Q;
		   break;
	    }
	    for(l=-1;l<totsym;l++)
	    {
	      if(tune.sr)l=totsym;
	      else
	      {
		if(l<0)continue;
		tune.sr=symtab[l];
	      }
	      switch(mode&0xf0)
	      {
	      case SATIP_SCAN_HW:
		r=satip_scan_transponder_hw(h,&tune,cfg,tp,status,priv);
		if(!r||r==SATIP_PARTIAL)return r;
		if(r!=SATIP_TUNERERR)return r;
		break;
	      case SATIP_SCAN_HTTP:
		r=satip_scan_transponder_http(h,&tune,cfg,tp,status,priv,
			host,port);
		if(!r||r==SATIP_PARTIAL)return r;
		if(r!=403&&r!=500&&r!=503)return r;
		break;
	      case SATIP_SCAN_RTSP:
		r=satip_scan_transponder_rtsp(h,&tune,cfg,tp,status,priv,
			host,port);
		if(!r||r==SATIP_PARTIAL)return r;
		if(r!=403&&r!=500&&r!=503)return r;
		break;
	      default:
		return SATIP_SYSFAIL;
	      }
	    }
	  }
	}

	return SATIP_NODATA;
}

static void *dvbcslave(void *data)
{
	int n=1;
	uint64_t dummy;
	SCANJOB *job=(SCANJOB *)data;
	struct pollfd p[2];
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_BLOCK,&set,NULL);

	pthread_setname_np(pthread_self(),"scan dvbcslave");

	p[0].fd=job->workfd;
	p[0].events=POLLIN;
	p[1].fd=job->termfd;
	p[1].events=POLLIN;
	p[1].revents=0;

	if(job->termfd!=-1)n=2;
	while(1)
	{
		if(poll(p,n,-1)<1)continue;
		if(p[1].revents&POLLIN)break;
		if(!(p[0].revents&POLLIN))break;
		if(read(job->workfd,&dummy,sizeof(dummy))!=sizeof(dummy))
			continue;
		if(!job->f)break;

		job->r=dvbcscanchan(job->h,&job->f->tune,&job->cfg,&job->f->tp,
			dvbctmultiscancb,job->f,job->mode,job->f->bits,
			job->symtab,job->totsym,job->host,job->port);

		pthread_spin_lock(&job->mtx);
		job->state=2;
		pthread_spin_unlock(&job->mtx);
		
		dummy=1;
		dummy=write(job->donefd,&dummy,sizeof(dummy));
	}       
	
	pthread_spin_lock(&job->mtx);
	job->state=-1;
	pthread_spin_unlock(&job->mtx);

	dummy=1;
	dummy=write(job->donefd,&dummy,sizeof(dummy));

	pthread_exit(NULL);
}

static void dosection(TRANSPONDER *t,unsigned char *data,int size)
{
	int i;
	int j;
	int k;
	union
	{
		SATIP_UTIL_PAT *pat;
		SATIP_UTIL_SDT *sdt;
		SATIP_UTIL_NIT *nit;
		SATIP_UTIL_PMT *pmt;
		SATIP_UTIL_NIT *ont;
		void *ptr;
	}u;
	union
	{
		SATIP_UTIL_PAT *pat;
		SATIP_UTIL_SDT *sdt;
		SATIP_UTIL_NIT *nit;
		SATIP_UTIL_PMT *pmt;
		SATIP_UTIL_NIT *ont;
	} v;
	union
	{
		SATIP_UTIL_PAT **pat;
		SATIP_UTIL_SDT **sdt;
		SATIP_UTIL_NIT **nit;
		SATIP_UTIL_PMT **pmt;
		SATIP_UTIL_NIT **ont;
	} w;

	switch(data[0])
	{
	case 0x00:
		if(!(u.pat=satip_util_unpack_pat_section(data,size)))return;

		pthread_spin_lock(&t->mtx);

		t->flags|=0x00030000;
		if(!u.pat->cnind)goto out2;
		if(t->tp->pat)if(t->tp->pat->tsid!=u.pat->tsid)
			t->flags|=0x80000000;
		if(t->flags&0x00000001)goto out2;

		if(t->tp->pat)if(u.pat->vernum!=t->tp->pat->vernum)
		{
			while(t->tp->pat)
			{
				v.pat=t->tp->pat;
				t->tp->pat=v.pat->next;
				free(v.pat);
			}
			goto out2;
		}

		if(u.pat->secnum>u.pat->lastsec)goto out2;

		for(w.pat=&t->tp->pat;*w.pat;w.pat=&(*w.pat)->next)
		{
			if(u.pat->secnum==(*w.pat)->secnum)goto out2;
			if((*w.pat)->secnum>u.pat->secnum)break;
		}
		u.pat->next=*w.pat;
		*w.pat=u.pat;

		for(i=0,v.pat=t->tp->pat;v.pat;v.pat=v.pat->next,i++)
		{
			if(v.pat->secnum!=i)break;
			if(v.pat->secnum==v.pat->lastsec)
			{
				t->flags|=0x00000001;
				break;
			}
		}

		goto out1;

	case 0x42:
		if(!(u.sdt=satip_util_unpack_sdt_section(data,size)))return;

		pthread_spin_lock(&t->mtx);

		t->flags|=0x00040000;
		if(!u.sdt->cnind)goto out2;
		if(t->tp->sdt)if(t->tp->sdt->tsid!=u.sdt->tsid)
			t->flags|=0x40000000;
		if(t->flags&0x00000004)goto out2;

		if(t->tp->sdt)if(u.sdt->vernum!=t->tp->sdt->vernum)
		{
			while(t->tp->sdt)
			{
				v.sdt=t->tp->sdt;
				t->tp->sdt=v.sdt->next;
				free(v.sdt);
			}
			goto out2;
		}

		if(u.sdt->secnum>u.sdt->lastsec)goto out2;

		for(w.sdt=&t->tp->sdt;*w.sdt;w.sdt=&(*w.sdt)->next)
		{
			if(u.sdt->secnum==(*w.sdt)->secnum)goto out2;
			if((*w.sdt)->secnum>u.sdt->secnum)break;
		}
		u.sdt->next=*w.sdt;
		*w.sdt=u.sdt;

		for(i=0,v.sdt=t->tp->sdt;v.sdt;v.sdt=v.sdt->next,i++)
		{
			if(v.sdt->secnum!=i)break;
			if(v.sdt->secnum==v.sdt->lastsec)
			{
				t->flags|=0x00000004;
				t->done++;
				break;
			}
		}

		goto out1;

	case 0x40:
		if(!(u.nit=satip_util_unpack_nit_section(data,size)))return;

		pthread_spin_lock(&t->mtx);

		if(!(t->flags&0x00080000))t->items++;
		t->flags|=0x00080000;
		if(!u.nit->cnind)goto out2;
		for(k=-1,j=0;j<SATIP_NIT_MAX;j++)if(!t->tp->nit[j])
		{
			if(k==-1)k=j;
		}
		else if(t->tp->nit[j]->netid==u.nit->netid)break;
		if(j==SATIP_NIT_MAX)
		{
			if(k!=-1)
			{
				j=k;
				if(t->flags&0x00000008)
				{
					t->flags&=~0x00000008;
					t->done--;
				}
			}
			else
			{
				t->flags|=0x20000000;
				goto out2;
			}
		}
		if(t->flags&0x00000008)goto out2;

		if(t->tp->nit[j])if(u.nit->vernum!=t->tp->nit[j]->vernum)
		{
			while(t->tp->nit[j])
			{
				v.nit=t->tp->nit[j];
				t->tp->nit[j]=v.nit->next;
				free(v.nit);
			}
			goto out2;
		}

		if(u.nit->secnum>u.nit->lastsec)goto out2;

		for(w.nit=&t->tp->nit[j];*w.nit;w.nit=&(*w.nit)->next)
		{
			if(u.nit->secnum==(*w.nit)->secnum)goto out2;
			if((*w.nit)->secnum>u.nit->secnum)break;
		}
		u.nit->next=*w.nit;
		*w.nit=u.nit;

		for(k=0,j=0;j<SATIP_NIT_MAX;j++)if(t->tp->nit[j])
		{
			for(k=0,i=0,v.nit=t->tp->nit[j];v.nit;
				v.nit=v.nit->next,i++)
			{
				if(v.nit->secnum!=i)goto out1;
				if(v.nit->secnum==v.nit->lastsec)
				{
					k=1;
					break;
				}
			}
			if(!k)break;
		}
		if(k)
		{
			t->flags|=0x00000008;
			t->done++;
		}

		goto out1;

	case 0x41:
		if(!(u.ont=satip_util_unpack_nit_section(data,size)))return;

		pthread_spin_lock(&t->mtx);

		if(!(t->flags&0x00400000))t->items++;
		t->flags|=0x00400000;
		if(!u.ont->cnind)goto out2;
		for(k=-1,j=0;j<SATIP_ONT_MAX;j++)if(!t->tp->ont[j])
		{
			if(k==-1)k=j;
		}
		else if(t->tp->ont[j]->netid==u.ont->netid)break;
		if(j==SATIP_ONT_MAX)
		{
			if(k!=-1)
			{
				j=k;
				if(t->flags&0x00000040)
				{
					t->flags&=~0x00000040;
					t->done--;
				}
			}
			else
			{
				t->flags|=0x10000000;
				goto out2;
			}
		}
		if(t->flags&0x00000040)goto out2;

		if(t->tp->ont[j])if(u.ont->vernum!=t->tp->ont[j]->vernum)
		{
			while(t->tp->ont)
			{
				v.ont=t->tp->ont[j];
				t->tp->ont[j]=v.ont->next;
				free(v.ont);
			}
			goto out2;
		}

		if(u.ont->secnum>u.ont->lastsec)goto out2;

		for(w.ont=&t->tp->ont[j];*w.ont;w.ont=&(*w.ont)->next)
		{
			if(u.ont->secnum==(*w.ont)->secnum)goto out2;
			if((*w.ont)->secnum>u.ont->secnum)break;
		}
		u.ont->next=*w.ont;
		*w.ont=u.ont;

		for(k=0,j=0;j<SATIP_ONT_MAX;j++)if(t->tp->ont[j])
		{
			for(k=0,i=0,v.ont=t->tp->ont[j];v.ont;
				v.ont=v.ont->next,i++)
			{
				if(v.ont->secnum!=i)goto out1;
				if(v.ont->secnum==v.ont->lastsec)
				{
					k=1;
					break;
				}
			}
			if(!k)break;
		}
		if(k)
		{
			t->flags|=0x00000040;
			t->done++;
		}

		goto out1;

	case 0x02:
		if(!(u.pmt=satip_util_unpack_pmt_section(data,size)))return;

		pthread_spin_lock(&t->mtx);

		t->flags|=0x00300000;
		for(j=0;j<t->totpmt;j++)if(u.pmt->prognum==t->prognum[j])break;
		if(j==t->totpmt||(t->flags&0x00000010)||!u.pmt->cnind)goto out2;

		if(t->pmt[j])if(u.pmt->vernum!=t->pmt[j]->vernum)
		{
			while(t->pmt[j])
			{
				v.pmt=t->pmt[j];
				t->pmt[j]=v.pmt->next;
				free(v.pmt);
			}
			goto out2;
		}

		if(u.pmt->secnum>u.pmt->lastsec)goto out2;

		for(w.pmt=&t->pmt[j];*w.pmt;w.pmt=&(*w.pmt)->next)
		{
			if(u.pmt->secnum==(*w.pmt)->secnum)goto out2;
			if((*w.pmt)->secnum>u.pmt->secnum)break;
		}
		u.pmt->next=*w.pmt;
		*w.pmt=u.pmt;

		for(i=0,v.pmt=t->pmt[j];v.pmt;v.pmt=v.pmt->next,i++)
		{
			if(v.pmt->secnum!=i)break;
			if(v.pmt->secnum==v.pmt->lastsec)
			{
				t->prognum[j]=-1;
				break;
			}
		}

		if(t->prognum[j]==-1)
		{
			for(j=0;j<t->totpmt;j++)if(t->prognum[j]!=-1)break;
			if(j==t->totpmt)t->flags|=0x00000010;
		}

		goto out1;
	}

	return;

out2:	free(u.ptr);
out1:	pthread_spin_unlock(&t->mtx);
}

static void hwstream(void *id,SATIP_STREAM *stream)
{
	TRANSPONDER *t=(TRANSPONDER *)id;

	if(!stream->fill||!(stream->flags&SATIP_FLGSECT))return;

	dosection(t,stream->section,stream->fill);
}

static void hwstatus(void *id,SATIP_STATUS *status)
{
	TRANSPONDER *t=(TRANSPONDER *)id;

	pthread_spin_lock(&t->mtx);
	t->level=status->level;
	t->work|=t->lock=status->lock;
	t->quality=status->quality;
	t->tp->actual=status->tune;
	pthread_spin_unlock(&t->mtx);
}

static void httpstr(SATIP_DATA *data,void *priv)
{
	TRANSPONDER *t=(TRANSPONDER *)priv;

	if(!data->intval)return;

	dosection(t,data->ptrval,data->intval);
}

static void httpstat(SATIP_DATA *data,void *priv)
{
	TRANSPONDER *t=(TRANSPONDER *)priv;
	SATIP_STATUS *status=(SATIP_STATUS *)(data->ptrval);

	if(!(data->intval&SATIP_RTCP))return;

	pthread_spin_lock(&t->mtx);
	t->level=status->level;
	t->work|=t->lock=status->lock;
	t->quality=status->quality;
	t->tune.fe=status->tune.fe;
	t->tp->actual=status->tune;
	pthread_spin_unlock(&t->mtx);
}

static void rtspsec(void *data,int len,void *priv)
{
	TRANSPONDER *t=(TRANSPONDER *)priv;

	dosection(t,data,len);
}

static void rtspwrk(SATIP_DATA *data,void *priv)
{
	TRANSPONDER *t=(TRANSPONDER *)priv;
	SATIP_STATUS *status=(SATIP_STATUS *)(data->ptrval);

	if(data->intval&SATIP_RTCP)
	{
		pthread_spin_lock(&t->mtx);
		t->level=status->level;
		t->work|=t->lock=status->lock;
		t->quality=status->quality;
		t->tune.fe=status->tune.fe;
		t->tp->actual=status->tune;
		pthread_spin_unlock(&t->mtx);
		return;
	}
	else if(data->intval&SATIP_RTP)
		satip_util_filter_packets(t->filter,data->ptrval,data->intval);
}

static void pidscan(unsigned char *packet,SATIP_SCAN_PIDINFO *info)
{
	int pid;
	int len=188;

	if(packet[0]!=0x47||(packet[1]&0x80))return;
	pid=packet[1]&0x1f;
	pid<<=8;
	pid|=packet[2];
	info[pid].present=1;
	if(pid==0x1fff)return;
	if(packet[3]&0xc0)
	{
		info[pid].ts_scramble=1;
		return;
	}
	if(!(packet[1]&0x40))return;
	switch(packet[3]&0x30)
	{
	case 0x00:
	case 0x20:
		return;
	case 0x30:
		len-=packet[4]+1;
		packet+=packet[4]+1;
	case 0x10:
		len-=4;
		packet+=4;
		if(len<3)return;
		break;
	}
	if(!packet[0]&&!packet[1]&&packet[2]==0x01)
	{
		info[pid].pes=1;
		if(len<4)return;
		switch(packet[3])
		{
		case 0xf0:
		case 0xf1:
			info[pid].ecmemm=1;
		case 0xbc:
		case 0xbe:
		case 0xbf:
		case 0xff:
		case 0xf2:
		case 0xf8:
			return;
		case 0xc0 ... 0xdf:
			info[pid].audio=1;
			break;
		case 0xe0 ... 0xef:
			info[pid].video=1;
			break;
		}
		if(len<7)return;
		if(packet[6]&0x30)info[pid].pes_scramble=1;
	}
	else info[pid].psi=1;
}

static void rtspscan(SATIP_DATA *data,void *priv)
{
	int i;
	int len;
	TRANSPONDER *t=(TRANSPONDER *)priv;

	if(!(data->intval&SATIP_RTP))return;

	for(i=0,len=data->intval&0xffff;i<len&&len-i>=188;i+=188)
		pidscan(((unsigned char *)(data->ptrval))+i,t->tp->pidinfo);
}

static void httpscan(SATIP_DATA *data,void *priv)
{
	int i;
	int len;
	TRANSPONDER *t=(TRANSPONDER *)priv;

	if(!data->intval)return;

	for(i=0,len=data->intval;i<len&&len-i>=188;i+=188)
		pidscan(((unsigned char *)(data->ptrval))+i,t->tp->pidinfo);
}

static void hwscan(void *id,SATIP_STREAM *stream)
{
	int i;
	int len;
	TRANSPONDER *t=(TRANSPONDER *)id;

	if(!stream->fill||(stream->flags&SATIP_FLGSECT))return;

	for(i=0,len=stream->fill;i<len&&len-i>=188;i+=188)
		pidscan(((unsigned char *)(stream->data))+i,t->tp->pidinfo);
}

static void shift(SATIP_UTIL_NIT **list,int total)
{
	int i;
	int j;

	for(i=0;i<total;i++)if(!list[i])break;
	for(j=i+1;j<total;j++)if(list[j])
	{
		list[i]=list[j];
		list[j]=NULL;
		for(i++;i<total;i++)if(!list[i])break;
		j=i;
	}
}

int satip_scan_transponder_hw(void *h,SATIP_TUNE *tune,SATIP_SCAN_PARAMS *cfg,
	SATIP_SCAN_RESULT *tp,void (*status)(SATIP_SCAN_STATUS *status,
	void *priv),void *priv)
{
	int i=0;
	int n=1;
	int j;
	int r=SATIP_SYSFAIL;
	int tfd;
	int ticks=0;
	int pmtstart=0;
	int pmtwork=0;
	int missing=0;
	int started=0;
	int repeat=0;
	int timeout;
	int timeadd;
	int locktmo;
	int timemin;
	int pmttmo;
	int pidtmo;
	int pidticks=0;
	int duration=0;
	int maxpmt;
	unsigned int match=0;
	uint64_t dummy;
	TRANSPONDER t;
	SATIP_UTIL_PAT *pat=NULL;
	SATIP_UTIL_PMT *pmt;
	SATIP_UTIL_SDT *sdt;
	struct pollfd p[2];
	struct itimerspec ts;
	SATIP_PIDS set[5+MAXPMT];
	SATIP_STRDATA params[5+MAXPMT];
	SATIP_STRDATA pidparam;
	SATIP_PIDS set2;
	SATIP_SCAN_STATUS st;
	unsigned long long map[1024];

	if(!h||!cfg||!tp||!tune)return r;

	if(cfg->fe)
	{
		if(tune->fe)return r;
		tune->fe=cfg->fe;
	}

	memset(tp,0,sizeof(SATIP_SCAN_RESULT));

	if(cfg->locktimeout<=0||cfg->mintimeout<=cfg->locktimeout||
		cfg->basetimeout<=cfg->mintimeout||
		cfg->maxtimeout<cfg->basetimeout||
		cfg->maxtimeout>60||cfg->pidscan<0||
		cfg->pidscan>cfg->mintimeout||
		cfg->pmtparallel<0||cfg->pmttimeout<=0||
		cfg->pmttimeout>cfg->mintimeout)return r;

	t.tp=tp;
	tp->requested=t.tune=*tune;

	maxpmt=(!cfg->pmtparallel||cfg->pmtparallel>MAXPMT)?
		MAXPMT:cfg->pmtparallel;

	locktmo=cfg->locktimeout*10;
	timemin=cfg->mintimeout*10;
	timeout=cfg->basetimeout*10;
	timeadd=(cfg->maxtimeout-cfg->basetimeout)*10;
	pmttmo=cfg->pmttimeout*10;
	pidtmo=cfg->pidscan*10;

	set[0].prognum=0;
	set[0].numpids=SATIP_NOPIDS;
	params[0].tune=&t.tune;
	params[0].set=&set[0];
	params[0].handle=&t;
	params[0].terminate=cfg->termfd;

	set[1].prognum=0;
	set[1].numpids=SATIP_SECTION;
	set[1].pid=0x0000;
	set[1].table=0x00;
	set[1].extra.bits=0;
	params[1].tune=&t.tune;
	params[1].set=&set[1];
	params[1].handle=&t;
	params[1].terminate=cfg->termfd;

	set[2].prognum=0;
	set[2].numpids=SATIP_SECTION;
	set[2].pid=0x0011;
	set[2].table=0x42;
	set[2].extra.bits=0;
	params[2].tune=&t.tune;
	params[2].set=&set[2];
	params[2].handle=&t;
	params[2].terminate=cfg->termfd;

	set[3].prognum=0;
	set[3].numpids=SATIP_SECTION;
	set[3].pid=0x0010;
	set[3].table=0x40;
	set[3].extra.bits=0;
	params[3].tune=&t.tune;
	params[3].set=&set[3];
	params[3].handle=&t;
	params[3].terminate=cfg->termfd;

	set[4].prognum=0;
	set[4].numpids=SATIP_SECTION;
	set[4].pid=0x0010;
	set[4].table=0x41;
	set[4].extra.bits=0;
	params[4].tune=&t.tune;
	params[4].set=&set[4];
	params[4].handle=&t;
	params[4].terminate=cfg->termfd;

	for(i=5;i<5+MAXPMT;i++)
	{
		set[i].prognum=0;
		set[i].numpids=SATIP_SECTION;
		set[i].pid=0x0000;
		set[i].table=0x02;
		set[i].extra.bits=0;
		params[i].tune=&t.tune;
		params[i].set=&set[i];
		params[i].handle=&t;
		params[i].terminate=cfg->termfd;
	}

	set2.prognum=0;
	set2.numpids=SATIP_ALLPIDS;
	pidparam.tune=&t.tune;
	pidparam.set=&set2;
	pidparam.handle=&t;
	pidparam.terminate=cfg->termfd;

	ts.it_interval.tv_sec=0;
	ts.it_interval.tv_nsec=100000000;
	ts.it_value.tv_sec=0;
	ts.it_value.tv_nsec=100000000;

	t.level=0;
	t.lock=0;
	t.quality=0;
	t.flags=0x00000000;
	t.items=0;
	t.done=0;
	t.work=0;
	t.totpmt=0;
	memset(t.pmt,0,sizeof(t.pmt));

	st.lock=0;
	st.level=0;
	st.quality=0;
	st.deviceid=0;
	st.total=0;
	st.done=0;
	st.info=SATIP_SCAN_CBI;
	st.globdone=0;
	st.globtotal=0;
	st.tuner=0;

	if(status)status(&st,priv);

	st.info=SATIP_SCAN_CBU;

	if(cfg->pidscan)
	{
		if(!(tp->pidinfo=malloc(sizeof(SATIP_SCAN_PIDINFO)*8192)))
			goto err1;
		memset(tp->pidinfo,0,sizeof(SATIP_SCAN_PIDINFO)*8192);
	}

	if(pthread_spin_init(&t.mtx,PTHREAD_PROCESS_PRIVATE))goto err1;

	if((tfd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))==-1)
		goto err2;
	if(timerfd_settime(tfd,0,&ts,NULL))goto err3;

	if((r=satip_hw_play(h,&params[0],hwstream,hwstatus,NULL,
		SATIP_HW_LOCAL)))goto err4;

	p[0].fd=tfd;
	p[0].events=POLLIN;
	if(cfg->termfd!=-1)
	{
		p[1].fd=cfg->termfd;
		p[1].events=POLLIN;
		n=2;
	}
	else p[1].revents=0;

	r=SATIP_SYSFAIL;

	while(1)
	{
		if(poll(p,n,-1)<1)continue;

		if(p[1].revents&POLLIN)break;
		if(!(p[0].revents&POLLIN))continue;
		if(read(tfd,&dummy,sizeof(dummy))!=sizeof(dummy))continue;
		duration+=(int)dummy;

		pthread_spin_lock(&t.mtx);
		if(t.flags&0xc0000000)
		{
			pthread_spin_unlock(&t.mtx);
			if(repeat++)
			{
				r=SATIP_NODATA;
				goto err9;
			}
			for(i=3;i<5+MAXPMT;i++)
			{
				if(params[i].handle!=&t)
					satip_hw_end(h,&params[i]);
				params[i].handle=&t;
			}
			set[4].pid=0x0010;
			set[3].pid=0x0010;
			pthread_spin_lock(&t.mtx);
			for(i=0;i<MAXPMT;i++)while(t.pmt[i])
			{
				pmt=t.pmt[i];
				t.pmt[i]=pmt->next;
				free(pmt);
			}
			satip_scan_transponder_free(h,tp);
			timeadd=(cfg->maxtimeout-cfg->basetimeout)*10;
			ticks=locktmo>>1;
			pmtstart=0;
			pmtwork=0;
			missing=0;
			match=0;
			t.flags=0x00000000;
			t.items=2;
			t.done=0;
			t.totpmt=0;
		}
		if(t.work&&!started)
		{
			if(cfg->tuneonly)
			{
				r=0;
				goto err9;
			}
			pthread_spin_unlock(&t.mtx);
			started=1;
			if((r=satip_hw_play(h,&params[1],hwstream,hwstatus,
				NULL,SATIP_HW_LOCAL)))goto err5;
			if((r=satip_hw_play(h,&params[2],hwstream,hwstatus,
				NULL,SATIP_HW_LOCAL)))goto err6;
			r=SATIP_SYSFAIL;
			pthread_spin_lock(&t.mtx);
			t.items=2;
		}
		if((t.flags&0x00000003)==0x00000001)
		{
			t.done++;
			for(pat=tp->pat;pat;pat=pat->next)
				for(i=0;i<pat->total;i++)
					if(pat->data[i].prognum)t.items++;
			else
			{
				if(pat->data[i].netpid!=0x0010&&
					set[3].pid==0x0010)
				{
					set[3].pid=pat->data[i].netpid;
					set[4].pid=pat->data[i].netpid;
				}
			}
			if(cfg->getnit)
			{
				pthread_spin_unlock(&t.mtx);
				if((r=satip_hw_play(h,&params[3],hwstream,
					hwstatus,NULL,SATIP_HW_LOCAL)))
						goto err7;
				if(cfg->getnit==SATIP_SCAN_NITS)
					if((r=satip_hw_play(h,&params[4],
						hwstream,hwstatus,NULL,
						SATIP_HW_LOCAL)))goto err8;
				r=SATIP_SYSFAIL;
				pthread_spin_lock(&t.mtx);
			}
			t.flags|=0x00000002;
			if(cfg->pidscan)
			{
				pthread_spin_unlock(&t.mtx);
				if((r=satip_hw_play(h,&pidparam,hwscan,
					nullstatus,NULL,SATIP_HW_LOCAL)))
						goto err9;
				r=SATIP_SYSFAIL;
				pthread_spin_lock(&t.mtx);
			}
			pat=tp->pat;
			i=0;
nextpmt:		for(j=0;j<t.totpmt;j++)params[5+j].handle=&t;
			memset(map,0,sizeof(map));
			pmtstart=ticks;
			for(t.totpmt=0;pat;pat=pat->next,i=0)while(i<pat->total)
				if(pat->data[i].prognum)
			{
				j=pat->data[i].prognum;
				if(map[j>>6]&(1ULL<<(j&0x3f)))goto dopmt;
				map[j>>6]|=(1ULL<<(j&0x3f));
				t.prognum[t.totpmt]=j;
				set[5+t.totpmt].pid=pat->data[i].pmtpid;
				t.pmtdest[t.totpmt++]=&pat->data[i++].pmt;
				if(t.totpmt>=maxpmt||t.totpmt==MAXPMT)
					goto dopmt;
			}
			else i++;
dopmt:			if(!t.totpmt)t.flags|=0x00000030;
			else 
			{
				pthread_spin_unlock(&t.mtx);
				for(j=0;j<t.totpmt;j++)
					if((r=satip_hw_play(h,&params[5+j],
						hwstream,hwstatus,NULL,
						SATIP_HW_LOCAL)))goto err9;
				r=SATIP_SYSFAIL;
				pthread_spin_lock(&t.mtx);
			}
		}
		if((ticks>=locktmo&&!t.lock)||
			(ticks>=timemin&&!(t.flags&0x00000003)))
		{
			pthread_spin_unlock(&t.mtx);
			r=SATIP_NODATA;
			goto err9;
		}
		if((t.flags&0x00000033)==0x00000013)pmtwork=1;
		if(params[5].handle!=&t&&ticks>=pmtstart+pmttmo)pmtwork=1;
		if((t.flags&(cfg->getnit?0x0000007f:0x00000037))==
			(cfg->getnit?0x0000007f:0x00000037))r=0;
		else if(cfg->getnit&&(t.flags&0x0040003f)==0x0000003f)r=0;
		else if(match&&(t.flags&match)==match)r=0;
		pthread_spin_unlock(&t.mtx);

		if(pmtwork)
		{
			for(j=0;j<t.totpmt;j++)satip_hw_end(h,&params[5+j]);
			pmtwork=0;
			pthread_spin_lock(&t.mtx);
			for(j=0;j<t.totpmt;j++)
			{
				*t.pmtdest[j]=t.pmt[j];
				if(t.pmt[j])t.done++;
				else missing++;
			}
			memset(t.pmt,0,sizeof(t.pmt));
			t.flags&=~0x00000010;
			goto nextpmt;
		}

		if(!r)if(!cfg->pidscan||pidticks>pidtmo)break;

		if(pidparam.handle!=&t)pidticks++;

		if(++ticks==timeout)
		{
			pthread_spin_lock(&t.mtx);
			if(t.flags&0xffff0000)
			{
				timeout+=timeadd;
				timeadd=0;
				match=(t.flags>>16)&0x0fff;
			}
			pthread_spin_unlock(&t.mtx);
			if(ticks==timeout)
			{
				r=SATIP_PARTIAL;
				break;
			}
		}

		if(ticks&3)continue;

		pthread_spin_lock(&t.mtx);
		st.lock=t.lock;
		st.level=t.level;
		st.quality=t.quality;
		st.deviceid=t.tune.fe;
		st.total=t.items;
		st.done=t.done;
		pthread_spin_unlock(&t.mtx);

		if(status)status(&st,priv);
	}

	if(!r)
	{
		if(tp->pat->total&&(t.flags&0x00000030)!=0x00000030)
			r=SATIP_PARTIAL;
		else if(missing||(t.flags&0x30000000))r=SATIP_PARTIAL;
	}

err9:	for(i=5;i<5+MAXPMT;i++)if(params[i].handle!=&t)satip_hw_end(h,&params[i]);
	for(i=0;i<MAXPMT;i++)while(t.pmt[i])
	{
		pmt=t.pmt[i];
		t.pmt[i]=pmt->next;
		free(pmt);
	}
	if(pidparam.handle!=&t)satip_hw_end(h,&pidparam);
	if(params[4].handle!=&t)satip_hw_end(h,&params[4]);
err8:	if(params[3].handle!=&t)satip_hw_end(h,&params[3]);
err7:	if(params[2].handle!=&t)satip_hw_end(h,&params[2]);
err6:	if(params[1].handle!=&t)satip_hw_end(h,&params[1]);
err5:	if(params[0].handle!=&t)satip_hw_end(h,&params[0]);
err4:	memset(&ts,0,sizeof(ts));
	timerfd_settime(tfd,0,&ts,NULL);
err3:	close(tfd);
err2:	pthread_spin_destroy(&t.mtx);
err1:	if(r&&r!=SATIP_PARTIAL&&tp->pidinfo)
	{
		free(tp->pidinfo);
		tp->pidinfo=NULL;
	}
	if(r==SATIP_NODATA)satip_scan_transponder_free(h,tp);
	else if(tp->pat&&tp->sdt)for(pat=tp->pat;pat;pat=pat->next)
		for(i=0;i<pat->total;i++)
			for(pmt=pat->data[i].pmt;pmt;pmt=pmt->next)
	{
		for(sdt=tp->sdt;sdt;sdt=sdt->next)for(n=0;n<sdt->total;n++)
			if(pmt->prognum==sdt->data[n].prognum)
		{
			pmt->sdt=sdt;
			pmt->sdtindex=n;
			goto loopend;
		}
loopend:;
	}

	shift(tp->nit,SATIP_NIT_MAX);
	shift(tp->ont,SATIP_ONT_MAX);

	tp->duration=duration;
	tp->partial=(r==SATIP_PARTIAL?1:0);
	tp->fastfind=0;

	st.lock=t.lock;
	st.level=t.level;
	st.quality=t.quality;
	st.deviceid=t.tune.fe;
	st.total=t.items;
	st.done=t.done;
	st.info=SATIP_SCAN_CBF;

	if(status)status(&st,priv);

	return r;
}

int satip_scan_transponder_http(void *h,SATIP_TUNE *tune,SATIP_SCAN_PARAMS *cfg,
	SATIP_SCAN_RESULT *tp,void (*status)(SATIP_SCAN_STATUS *status,
	void *priv),void *priv,char *host,int port)
{
	int i=0;
	int n=1;
	int j;
	int r=SATIP_SYSFAIL;
	int tfd;
	int ticks=0;
	int pmtstart=0;
	int pmtwork=0;
	int missing=0;
	int started=0;
	int repeat=0;
	int timeout;
	int timeadd;
	int locktmo;
	int timemin;
	int pmttmo;
	int pidtmo;
	int pidticks=0;
	int duration=0;
	int maxpmt;
	unsigned int match=0;
	uint64_t dummy;
	TRANSPONDER t;
	SATIP_UTIL_PAT *pat=NULL;
	SATIP_UTIL_PMT *pmt;
	SATIP_UTIL_SDT *sdt;
	void *handle[5+MAXPMT];
	void *pidhandle=NULL;
	struct pollfd p[2];
	struct itimerspec ts;
	SATIP_PIDS set[5+MAXPMT];
	SATIP_PIDS set2;
	SATIP_SCAN_STATUS st;
	unsigned long long map[1024];

	if(!h||!cfg||!tp||!tune||!host)return r;

	if(cfg->fe)
	{
		if(tune->fe)return r;
		tune->fe=cfg->fe;
	}

	memset(tp,0,sizeof(SATIP_SCAN_RESULT));

	if(cfg->locktimeout<=0||cfg->mintimeout<=cfg->locktimeout||
		cfg->basetimeout<=cfg->mintimeout||
		cfg->maxtimeout<cfg->basetimeout||
		cfg->maxtimeout>60||cfg->pidscan<0||
		cfg->pidscan>cfg->mintimeout||cfg->pmtparallel<0||
		cfg->pmttimeout<=0||cfg->pmttimeout>cfg->mintimeout||
		port<1||port>65535)return r;

	memset(handle,0,sizeof(handle));

	t.tp=tp;
	tp->requested=t.tune=*tune;

	maxpmt=(!cfg->pmtparallel||cfg->pmtparallel>MAXPMT)?
		MAXPMT:cfg->pmtparallel;

	locktmo=cfg->locktimeout*10;
	timemin=cfg->mintimeout*10;
	timeout=cfg->basetimeout*10;
	timeadd=(cfg->maxtimeout-cfg->basetimeout)*10;
	pmttmo=cfg->pmttimeout*10;
	pidtmo=cfg->pidscan*10;

	set[0].prognum=0;
	set[0].numpids=SATIP_SIGNAL;

	set[1].prognum=0;
	set[1].numpids=SATIP_SECTION;
	set[1].pid=0x0000;
	set[1].table=0x00;

	set[2].prognum=0;
	set[2].numpids=SATIP_SECTION;
	set[2].pid=0x0011;
	set[2].table=0x42;

	set[3].prognum=0;
	set[3].numpids=SATIP_SECTION;
	set[3].pid=0x0010;
	set[3].table=0x40;

	set[4].prognum=0;
	set[4].numpids=SATIP_SECTION;
	set[4].pid=0x0010;
	set[4].table=0x41;

	for(i=5;i<5+MAXPMT;i++)
	{
		set[i].prognum=0;
		set[i].numpids=SATIP_SECTION;
		set[i].pid=0x0000;
		set[i].table=0x02;
	}

	set2.prognum=0;

	ts.it_interval.tv_sec=0;
	ts.it_interval.tv_nsec=100000000;
	ts.it_value.tv_sec=0;
	ts.it_value.tv_nsec=100000000;

	t.level=0;
	t.lock=0;
	t.quality=0;
	t.flags=0x00000000;
	t.items=0;
	t.done=0;
	t.work=0;
	t.totpmt=0;
	memset(t.pmt,0,sizeof(t.pmt));

	st.lock=0;
	st.level=0;
	st.quality=0;
	st.deviceid=0;
	st.total=0;
	st.done=0;
	st.info=SATIP_SCAN_CBI;
	st.globdone=0;
	st.globtotal=0;
	st.tuner=0;

	if(status)status(&st,priv);

	st.info=SATIP_SCAN_CBU;

	if(cfg->pidscan)
	{
		if(!(tp->pidinfo=malloc(sizeof(SATIP_SCAN_PIDINFO)*8192)))
			goto err1;
		memset(tp->pidinfo,0,sizeof(SATIP_SCAN_PIDINFO)*8192);
	}

	if(pthread_spin_init(&t.mtx,PTHREAD_PROCESS_PRIVATE))goto err1;

	if((tfd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))==-1)
		goto err2;
	if(timerfd_settime(tfd,0,&ts,NULL))goto err3;

	if((r=satip_cln_stream_http(h,host,port,&t.tune,&set[0],NULL,httpstat,
		&t,&handle[0])))goto err4;

	p[0].fd=tfd;
	p[0].events=POLLIN;
	if(cfg->termfd!=-1)
	{
		p[1].fd=cfg->termfd;
		p[1].events=POLLIN;
		n=2;
	}
	else p[1].revents=0;

	r=SATIP_SYSFAIL;

	while(1)
	{
		if(poll(p,n,-1)<1)continue;

		if(p[1].revents&POLLIN)break;
		if(!(p[0].revents&POLLIN))continue;
		if(read(tfd,&dummy,sizeof(dummy))!=sizeof(dummy))continue;
		duration+=(int)dummy;

		pthread_spin_lock(&t.mtx);
		if(t.flags&0xc0000000)
		{
			pthread_spin_unlock(&t.mtx);
			if(repeat++)
			{
				r=SATIP_NODATA;
				goto err9;
			}
			for(i=3;i<5+MAXPMT;i++)
			{
				if(handle[i])satip_cln_stream_stop(h,handle[i]);
				handle[i]=NULL;
			}
			set[4].pid=0x0010;
			set[3].pid=0x0010;
			pthread_spin_lock(&t.mtx);
			for(i=0;i<MAXPMT;i++)while(t.pmt[i])
			{
				pmt=t.pmt[i];
				t.pmt[i]=pmt->next;
				free(pmt);
			}
			satip_scan_transponder_free(h,tp);
			timeadd=(cfg->maxtimeout-cfg->basetimeout)*10;
			ticks=locktmo>>1;
			pmtstart=0;
			pmtwork=0;
			missing=0;
			match=0;
			t.flags=0x00000000;
			t.items=2;
			t.done=0;
			t.totpmt=0;
		}
		if(t.work&&!started)
		{
			if(cfg->tuneonly)
			{
				r=0;
				goto err9;
			}
			pthread_spin_unlock(&t.mtx);
			started=1;
			if((r=satip_cln_stream_http(h,host,port,&t.tune,&set[1],
				NULL,httpstr,&t,&handle[1])))goto err5;
			if((r=satip_cln_stream_http(h,host,port,&t.tune,&set[2],
				NULL,httpstr,&t,&handle[2])))goto err6;
			r=SATIP_SYSFAIL;
			pthread_spin_lock(&t.mtx);
			t.items=2;
		}
		if((t.flags&0x00000003)==0x00000001)
		{
			t.done++;
			for(pat=tp->pat;pat;pat=pat->next)
				for(i=0;i<pat->total;i++)
					if(pat->data[i].prognum)t.items++;
			else
			{
				if(pat->data[i].netpid!=0x0010&&
					set[3].pid==0x0010)
				{
					set[3].pid=pat->data[i].netpid;
					set[4].pid=pat->data[i].netpid;
				}
			}
			if(cfg->getnit)
			{
				pthread_spin_unlock(&t.mtx);
				if((r=satip_cln_stream_http(h,host,port,&t.tune,
					&set[3],NULL,httpstr,&t,&handle[3])))
						goto err7;
				if(cfg->getnit==SATIP_SCAN_NITS)
					if((r=satip_cln_stream_http(h,host,port,
						&t.tune,&set[4],NULL,httpstr,&t,
						&handle[4])))goto err8;
				r=SATIP_SYSFAIL;
				pthread_spin_lock(&t.mtx);
			}
			t.flags|=0x00000002;
			if(cfg->pidscan)
			{
				pthread_spin_unlock(&t.mtx);
				set2.numpids=SATIP_ALLPIDS;
				if((r=satip_cln_stream_http(h,host,port,&t.tune,
					&set2,NULL,httpscan,&t,&pidhandle)))
						goto err9;
				r=SATIP_SYSFAIL;
				pthread_spin_lock(&t.mtx);
			}
			pat=tp->pat;
			i=0;
nextpmt:		for(j=0;j<t.totpmt;j++)handle[5+j]=NULL;
			memset(map,0,sizeof(map));
			pmtstart=ticks;
			for(t.totpmt=0;pat;pat=pat->next,i=0)while(i<pat->total)
				if(pat->data[i].prognum)
			{
				j=pat->data[i].prognum;
				if(map[j>>6]&(1ULL<<(j&0x3f)))goto dopmt;
				map[j>>6]|=(1ULL<<(j&0x3f));
				t.prognum[t.totpmt]=j;
				set[5+t.totpmt].pid=pat->data[i].pmtpid;
				t.pmtdest[t.totpmt++]=&pat->data[i++].pmt;
				if(t.totpmt>=maxpmt||t.totpmt==MAXPMT)
					goto dopmt;
			}
			else i++;
dopmt:			if(!t.totpmt)t.flags|=0x00000030;
			else 
			{
				pthread_spin_unlock(&t.mtx);
				for(j=0;j<t.totpmt;j++)
					if((r=satip_cln_stream_http(h,host,port,
						&t.tune,&set[5+j],NULL,httpstr,
						&t,&handle[5+j])))goto err9;
				r=SATIP_SYSFAIL;
				pthread_spin_lock(&t.mtx);
			}
		}
		if((ticks>=locktmo&&!t.lock)||
			(ticks>=timemin&&!(t.flags&0x00000003)))
		{
			pthread_spin_unlock(&t.mtx);
			r=SATIP_NODATA;
			goto err9;
		}
		if((t.flags&0x00000033)==0x00000013)pmtwork=1;
		if(handle[5]&&ticks>=pmtstart+pmttmo)pmtwork=1;
		if((t.flags&(cfg->getnit?0x0000007f:0x00000037))==
			(cfg->getnit?0x0000007f:0x00000037))r=0;
		else if(cfg->getnit&&(t.flags&0x0040003f)==0x0000003f)r=0;
		else if(match&&(t.flags&match)==match)r=0;
		pthread_spin_unlock(&t.mtx);

		if(pmtwork)
		{
			for(j=0;j<t.totpmt;j++)
				satip_cln_stream_stop(h,handle[5+j]);
			pmtwork=0;
			pthread_spin_lock(&t.mtx);
			for(j=0;j<t.totpmt;j++)
			{
				*t.pmtdest[j]=t.pmt[j];
				if(t.pmt[j])t.done++;
				else missing++;
			}
			memset(t.pmt,0,sizeof(t.pmt));
			t.flags&=~0x00000010;
			goto nextpmt;
		}

		if(!r)if(!cfg->pidscan||pidticks>pidtmo)break;

		if(pidhandle)pidticks++;

		if(++ticks==timeout)
		{
			pthread_spin_lock(&t.mtx);
			if(t.flags&0xffff0000)
			{
				timeout+=timeadd;
				timeadd=0;
				match=(t.flags>>16)&0x0fff;
			}
			pthread_spin_unlock(&t.mtx);
			if(ticks==timeout)
			{
				r=SATIP_PARTIAL;
				break;
			}
		}

		if(ticks&3)continue;

		pthread_spin_lock(&t.mtx);
		st.lock=t.lock;
		st.level=t.level;
		st.quality=t.quality;
		st.deviceid=t.tune.fe;
		st.total=t.items;
		st.done=t.done;
		pthread_spin_unlock(&t.mtx);

		if(status)status(&st,priv);
	}

	if(!r)
	{
		if(tp->pat->total&&(t.flags&0x00000030)!=0x00000030)
			r=SATIP_PARTIAL;
		else if(missing||(t.flags&0x30000000))r=SATIP_PARTIAL;
	}

err9:	for(i=5;i<5+MAXPMT;i++)if(handle[i])satip_cln_stream_stop(h,handle[i]);
	for(i=0;i<MAXPMT;i++)while(t.pmt[i])
	{
		pmt=t.pmt[i];
		t.pmt[i]=pmt->next;
		free(pmt);
	}
	if(pidhandle)satip_cln_stream_stop(h,pidhandle);
	if(handle[4])satip_cln_stream_stop(h,handle[4]);
err8:	if(handle[3])satip_cln_stream_stop(h,handle[3]);
err7:	if(handle[2])satip_cln_stream_stop(h,handle[2]);
err6:	if(handle[1])satip_cln_stream_stop(h,handle[1]);
err5:	if(handle[0])satip_cln_stream_stop(h,handle[0]);
err4:	memset(&ts,0,sizeof(ts));
	timerfd_settime(tfd,0,&ts,NULL);
err3:	close(tfd);
err2:	pthread_spin_destroy(&t.mtx);
err1:	if(r&&r!=SATIP_PARTIAL&&tp->pidinfo)
	{
		free(tp->pidinfo);
		tp->pidinfo=NULL;
	}
	if(r==SATIP_NODATA)satip_scan_transponder_free(h,tp);
	else if(tp->pat&&tp->sdt)for(pat=tp->pat;pat;pat=pat->next)
		for(i=0;i<pat->total;i++)
			for(pmt=pat->data[i].pmt;pmt;pmt=pmt->next)
	{
		for(sdt=tp->sdt;sdt;sdt=sdt->next)for(n=0;n<sdt->total;n++)
			if(pmt->prognum==sdt->data[n].prognum)
		{
			pmt->sdt=sdt;
			pmt->sdtindex=n;
			goto loopend;
		}
loopend:;
	}

	shift(tp->nit,SATIP_NIT_MAX);
	shift(tp->ont,SATIP_ONT_MAX);

	tp->duration=duration;
	tp->partial=(r==SATIP_PARTIAL?1:0);
	tp->fastfind=0;

	st.lock=t.lock;
	st.level=t.level;
	st.quality=t.quality;
	st.deviceid=t.tune.fe;
	st.total=t.items;
	st.done=t.done;
	st.info=SATIP_SCAN_CBF;

	if(status)status(&st,priv);

	return r;
}

int satip_scan_transponder_rtsp(void *h,SATIP_TUNE *tune,SATIP_SCAN_PARAMS *cfg,
	SATIP_SCAN_RESULT *tp,void (*status)(SATIP_SCAN_STATUS *status,
	void *priv),void *priv,char *host,int port)
{
	int i=0;
	int n=1;
	int r=SATIP_SYSFAIL;
	int j;
	int k;
	int tfd;
	int ticks=0;
	int pmtstart=0;
	int pmtwork=0;
	int missing=0;
	int started=0;
	int repeat=0;
	int timeout;
	int timeadd;
	int locktmo;
	int timemin;
	int pmttmo;
	int pidtmo;
	int pidticks=0;
	int duration=0;
	int maxpmt;
	int busy=0;
	unsigned int match=0;
	uint64_t dummy;
	TRANSPONDER t;
	SATIP_UTIL_PAT *pat=NULL;
	SATIP_UTIL_PMT *pmt;
	SATIP_UTIL_SDT *sdt;
	void *handle;
	void *pidhandle=NULL;
	void *section[4+MAXPMT];
	void *user[4+MAXPMT];
	struct pollfd p[2];
	struct itimerspec ts;
	SATIP_PIDS set;
	SATIP_PIDS set2;
	SATIP_UTIL_SECFILTER flt;
	SATIP_SCAN_STATUS st;
	unsigned long long map[1024];

	if(!h||!cfg||!tp||!tune||!host)return r;

	if(cfg->fe)
	{
		if(tune->fe)return r;
		tune->fe=cfg->fe;
	}

	memset(tp,0,sizeof(SATIP_SCAN_RESULT));

	if(cfg->locktimeout<=0||cfg->mintimeout<=cfg->locktimeout||
		cfg->basetimeout<=cfg->mintimeout||
		cfg->maxtimeout<cfg->basetimeout||
		cfg->maxtimeout>60||cfg->pidscan<0||
		cfg->pidscan>cfg->mintimeout||cfg->pmtparallel<0||
		cfg->pmttimeout<=0||cfg->pmttimeout>cfg->mintimeout||
		port<1||port>65535)return r;

	memset(&flt,0,sizeof(flt));
	flt.mask[0]=0xff;

	t.tp=tp;
	tp->requested=t.tune=*tune;

	maxpmt=(!cfg->pmtparallel||cfg->pmtparallel>MAXPMT)?
		MAXPMT:cfg->pmtparallel;

	locktmo=cfg->locktimeout*10;
	timemin=cfg->mintimeout*10;
	timeout=cfg->basetimeout*10;
	timeadd=(cfg->maxtimeout-cfg->basetimeout)*10;
	pmttmo=cfg->pmttimeout*10;
	pidtmo=cfg->pidscan*10;

	set.prognum=0;
	set.numpids=2;
	set.pids[0]=0x0000;
	set.pids[1]=0x0011;
	set.pids[2]=0x0010;

	set2.prognum=0;

	ts.it_interval.tv_sec=0;
	ts.it_interval.tv_nsec=100000000;
	ts.it_value.tv_sec=0;
	ts.it_value.tv_nsec=100000000;

	t.level=0;
	t.lock=0;
	t.quality=0;
	t.flags=0x00000000;
	t.items=0;
	t.done=0;
	t.work=0;
	memset(t.pmt,0,sizeof(t.pmt));

	st.lock=0;
	st.level=0;
	st.quality=0;
	st.deviceid=0;
	st.total=0;
	st.done=0;
	st.info=SATIP_SCAN_CBI;
	st.globdone=0;
	st.globtotal=0;
	st.tuner=0;

	if(status)status(&st,priv);

	st.info=SATIP_SCAN_CBU;

	if(cfg->pidscan)
	{
		if(!(tp->pidinfo=malloc(sizeof(SATIP_SCAN_PIDINFO)*8192)))
			goto err1;
		memset(tp->pidinfo,0,sizeof(SATIP_SCAN_PIDINFO)*8192);
	}

	if(pthread_spin_init(&t.mtx,PTHREAD_PROCESS_PRIVATE))goto err1;

	if((tfd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))==-1)
		goto err2;
	if(timerfd_settime(tfd,0,&ts,NULL))goto err3;

	if(satip_util_filter_create(&t.filter))goto err4;

	flt.filter[0]=0x00;
	if(satip_util_section_create(&section[0],&flt,1,rtspsec,&t))goto err5;
	if(satip_util_filter_add_user(t.filter,&user[0],
		satip_util_section_packet,section[0]))goto err6;

	flt.filter[0]=0x42;
	if(satip_util_section_create(&section[1],&flt,1,rtspsec,&t))goto err7;
	if(satip_util_filter_add_user(t.filter,&user[1],
		satip_util_section_packet,section[1]))goto err8;

	flt.filter[0]=0x40;
	if(satip_util_section_create(&section[2],&flt,1,rtspsec,&t))goto err9;
	if(satip_util_filter_add_user(t.filter,&user[2],
		satip_util_section_packet,section[2]))goto err10;

	flt.filter[0]=0x41;
	if(satip_util_section_create(&section[3],&flt,1,rtspsec,&t))goto err11;
	if(satip_util_filter_add_user(t.filter,&user[3],
		satip_util_section_packet,section[3]))goto err12;

	for(flt.filter[0]=0x02,i=0;i<MAXPMT;i++)
	{
		if(satip_util_section_create(&section[i+4],&flt,1,rtspsec,&t))
			goto fail;
		if(satip_util_filter_add_user(t.filter,&user[i+4],
			satip_util_section_packet,section[i+4]))
		{
			satip_util_section_free(section[i+4]);
fail:			while(i--)
			{
				satip_util_filter_del_user(t.filter,user[i+4]);
				satip_util_section_free(section[i+4]);
			}
			goto err13;
		}
	}

	if((r=satip_cln_stream_unicast(h,host,port,1,&t.tune,&set,NULL,rtspwrk,
		&t,&handle)))goto err14;

	p[0].fd=tfd;
	p[0].events=POLLIN;
	if(cfg->termfd!=-1)
	{
		p[1].fd=cfg->termfd;
		p[1].events=POLLIN;
		n=2;
	}
	else p[1].revents=0;

	r=SATIP_SYSFAIL;

	while(1)
	{
		if(poll(p,n,-1)<1)continue;

		if(p[1].revents&POLLIN)break;
		if(!(p[0].revents&POLLIN))continue;
		if(read(tfd,&dummy,sizeof(dummy))!=sizeof(dummy))continue;
		duration+=(int)dummy;

		pthread_spin_lock(&t.mtx);
		if(t.flags&0xc0000000)
		{
			pthread_spin_unlock(&t.mtx);
			if(repeat++)
			{
				r=SATIP_NODATA;
				goto err15;
			}
			for(i=2;i<4+MAXPMT;i++)
			{
				satip_util_user_clrpid(user[i]);
				satip_util_section_reset(section[i]);
			}
			set.numpids=2;
			set.pids[0]=0x0000;
			set.pids[1]=0x0011;
			set.pids[2]=0x0010;
			if((r=satip_cln_change_unicast(h,handle,&set,NULL,NULL,
				NULL)))goto err15;
			r=SATIP_SYSFAIL;
			pthread_spin_lock(&t.mtx);
			for(j=0;j<MAXPMT;j++)while(t.pmt[j])
			{
				pmt=t.pmt[j];
				t.pmt[j]=pmt->next;
				free(pmt);
			}
			satip_scan_transponder_free(h,tp);
			timeadd=(cfg->maxtimeout-cfg->basetimeout)*10;
			ticks=locktmo>>1;
			pmtstart=0;
			pmtwork=0;
			missing=0;
			busy=0;
			match=0;
			t.flags=0x00000000;
			t.items=2;
			t.done=0;
		}
		if(t.work&&!started)
		{
			if(cfg->tuneonly)
			{
				r=0;
				goto err15;
			}
			pthread_spin_unlock(&t.mtx);
			started=1;
			if((r=satip_util_user_addpid(user[0],0x0000)))
				goto err15;
			if((r=satip_util_user_addpid(user[1],0x0011)))
				goto err15;
			r=SATIP_SYSFAIL;
			pthread_spin_lock(&t.mtx);
			t.items=2;
		}
		if((t.flags&0x00000003)==0x00000001)
		{
			t.done++;
			for(pat=tp->pat;pat;pat=pat->next)
				for(i=0;i<pat->total;i++)
					if(pat->data[i].prognum)t.items++;
			else
			{
				if(pat->data[i].netpid!=0x0010&&
					set.pids[2]==0x0010)
						set.pids[2]=pat->data[i].netpid;
			}
			if(cfg->getnit)
			{
				pthread_spin_unlock(&t.mtx);
				set.numpids=3;
				for(set2.numpids=0,j=0;j<set.numpids;j++)
				{
					for(k=0;k<j;k++)
						if(set.pids[j]==set.pids[k])
							break;
					if(j==k)set2.pids[set2.numpids++]=
						set.pids[j];
				}
				if(satip_util_user_addpid(user[2],set.pids[2]))
					goto err15;
				if(cfg->getnit==SATIP_SCAN_NITS)
					if(satip_util_user_addpid(user[3],
						set.pids[2]))goto err15;
				if((r=satip_cln_change_unicast(h,handle,&set2,
					NULL,NULL,NULL)))goto err15;
				r=SATIP_SYSFAIL;
				pthread_spin_lock(&t.mtx);
			}
			set.numpids++;
			t.flags|=0x00000002;
			if(cfg->pidscan)
			{
				pthread_spin_unlock(&t.mtx);
				set2.numpids=SATIP_ALLPIDS;
				if((r=satip_cln_stream_unicast(h,host,port,1,
					&t.tune,&set2,NULL,rtspscan,&t,
					&pidhandle)))goto err15;
				r=SATIP_SYSFAIL;
				pthread_spin_lock(&t.mtx);
			}
			pat=tp->pat;
			i=0;
nextpmt:		busy=0;
			memset(map,0,sizeof(map));
			pmtstart=ticks;
			for(t.totpmt=0;pat;pat=pat->next,i=0)while(i<pat->total)
				if(pat->data[i].prognum)
			{
				j=pat->data[i].prognum;
				if(map[j>>6]&(1ULL<<(j&0x3f)))goto dopmt;
				map[j>>6]|=(1ULL<<(j&0x3f));
				t.prognum[t.totpmt]=j;
				set.pids[set.numpids-1+t.totpmt]=
					pat->data[i].pmtpid;
				t.pmtdest[t.totpmt++]=&pat->data[i++].pmt;
				if(t.totpmt>=maxpmt||t.totpmt==MAXPMT)
					goto dopmt;
			}
			else i++;
dopmt:			if(!t.totpmt)t.flags|=0x00000030;
			else 
			{
				pthread_spin_unlock(&t.mtx);
				for(set2.numpids=0,j=0;j<set.numpids-1+t.totpmt;
					j++)
				{
					for(k=0;k<j;k++)if(set.pids[j]==
						set.pids[k])break;
					if(j==k)set2.pids[set2.numpids++]=
						set.pids[j];
				}
				if((r=satip_cln_change_unicast(h,handle,&set2,
					NULL,NULL,NULL)))goto err15;
				for(j=set.numpids-1;j<set.numpids-1+t.totpmt;
					j++)if(satip_util_user_addpid(
						user[5+j-set.numpids],
						set.pids[j]))goto err15;
				busy=1;
				r=SATIP_SYSFAIL;
				pthread_spin_lock(&t.mtx);
			}
		}
		if((ticks>=locktmo&&!t.lock)||
			(ticks>=timemin&&!(t.flags&0x00000003)))
		{
			pthread_spin_unlock(&t.mtx);
			r=SATIP_NODATA;
			goto err15;
		}
		if((t.flags&0x00000033)==0x00000013)pmtwork=1;
		if(busy&&ticks>=pmtstart+pmttmo)pmtwork=1;
		if((t.flags&(cfg->getnit?0x0000007f:0x00000037))==
			(cfg->getnit?0x0000007f:0x00000037))r=0;
		else if(cfg->getnit&&(t.flags&0x0040003f)==0x0000003f)r=0;
		else if(match&&(t.flags&match)==match)r=0;
		pthread_spin_unlock(&t.mtx);

		if(pmtwork)
		{
			for(j=0;j<t.totpmt;j++)
			{
				satip_util_user_clrpid(user[j+4]);
				satip_util_section_reset(section[j+4]);
			}
			pmtwork=0;
			pthread_spin_lock(&t.mtx);
			for(j=0;j<t.totpmt;j++)
			{
				*t.pmtdest[j]=t.pmt[j];
				if(t.pmt[j])t.done++;
				else missing++;
			}
			memset(t.pmt,0,sizeof(t.pmt));
			t.flags&=~0x00000010;
			goto nextpmt;
		}

		if(!r)if(!cfg->pidscan||pidticks>pidtmo)break;

		if(pidhandle)pidticks++;

		if(++ticks==timeout)
		{
			pthread_spin_lock(&t.mtx);
			if(t.flags&0xffff0000)
			{
				timeout+=timeadd;
				timeadd=0;
				match=(t.flags>>16)&0x0fff;
			}
			pthread_spin_unlock(&t.mtx);
			if(ticks==timeout)
			{
				r=SATIP_PARTIAL;
				break;
			}
		}

		if(ticks&3)continue;

		pthread_spin_lock(&t.mtx);
		st.lock=t.lock;
		st.level=t.level;
		st.quality=t.quality;
		st.deviceid=t.tune.fe;
		st.total=t.items;
		st.done=t.done;
		pthread_spin_unlock(&t.mtx);

		if(status)status(&st,priv);
	}

	if(!r)
	{
		if(tp->pat->total&&(t.flags&0x00000030)!=0x00000030)
			r=SATIP_PARTIAL;
		else if(missing||(t.flags&0x30000000))r=SATIP_PARTIAL;
	}

err15:	for(i=0;i<MAXPMT;i++)satip_util_user_clrpid(user[i+4]);
	satip_util_user_clrpid(user[3]);
	satip_util_user_clrpid(user[2]);
	satip_util_user_clrpid(user[1]);
	satip_util_user_clrpid(user[0]);
	satip_cln_stream_stop(h,handle);
	if(pidhandle)satip_cln_stream_stop(h,pidhandle);
	for(j=0;j<MAXPMT;j++)while(t.pmt[j])
	{
		pmt=t.pmt[j];
		t.pmt[j]=pmt->next;
		free(pmt);
	}
err14:	for(i=0;i<MAXPMT;i++)
	{
		satip_util_filter_del_user(t.filter,user[i+4]);
		satip_util_section_free(section[i+4]);
	}
err13:	satip_util_filter_del_user(t.filter,user[3]);
err12:	satip_util_section_free(section[3]);
err11:	satip_util_filter_del_user(t.filter,user[2]);
err10:	satip_util_section_free(section[2]);
err9:	satip_util_filter_del_user(t.filter,user[1]);
err8:	satip_util_section_free(section[1]);
err7:	satip_util_filter_del_user(t.filter,user[0]);
err6:	satip_util_section_free(section[0]);
err5:	satip_util_filter_free(t.filter);
err4:	memset(&ts,0,sizeof(ts));
	timerfd_settime(tfd,0,&ts,NULL);
err3:	close(tfd);
err2:	pthread_spin_destroy(&t.mtx);
err1:	if(r&&r!=SATIP_PARTIAL&&tp->pidinfo)
	{
		free(tp->pidinfo);
		tp->pidinfo=NULL;
	}
	if(r==SATIP_NODATA)satip_scan_transponder_free(h,tp);
	else if(tp->pat&&tp->sdt)for(pat=tp->pat;pat;pat=pat->next)
		for(i=0;i<pat->total;i++)
			for(pmt=pat->data[i].pmt;pmt;pmt=pmt->next)
	{
		for(sdt=tp->sdt;sdt;sdt=sdt->next)for(n=0;n<sdt->total;n++)
			if(pmt->prognum==sdt->data[n].prognum)
		{
			pmt->sdt=sdt;
			pmt->sdtindex=n;
			goto loopend;
		}
loopend:;
	}

	shift(tp->nit,SATIP_NIT_MAX);
	shift(tp->ont,SATIP_ONT_MAX);

	tp->duration=duration;
	tp->partial=(r==SATIP_PARTIAL?1:0);
	tp->fastfind=0;

	st.lock=t.lock;
	st.level=t.level;
	st.quality=t.quality;
	st.deviceid=t.tune.fe;
	st.total=t.items;
	st.done=t.done;
	st.info=SATIP_SCAN_CBF;

	if(status)status(&st,priv);

	return r;
}

int satip_scan_transponder_free(void *h,SATIP_SCAN_RESULT *tp)
{
	int i;
	SATIP_UTIL_PAT *pat;
	SATIP_UTIL_PMT *pmt;
	SATIP_UTIL_SDT *sdt;
	SATIP_UTIL_NIT *nit;

	if(!h||!tp)return SATIP_SYSFAIL;

	while(tp->pat)
	{
		pat=tp->pat;
		tp->pat=pat->next;
		for(i=0;i<pat->total;i++)while(pat->data[i].pmt)
		{
			pmt=pat->data[i].pmt;
			pat->data[i].pmt=pmt->next;
			free(pmt);
		}
		free(pat);
	}
	while(tp->sdt)
	{
		sdt=tp->sdt;
		tp->sdt=sdt->next;
		free(sdt);
	}
	for(i=0;i<SATIP_NIT_MAX&&tp->nit[i];i++)while(tp->nit[i])
	{
		nit=tp->nit[i];
		tp->nit[i]=nit->next;
		free(nit);
	}
	for(i=0;i<SATIP_ONT_MAX&&tp->ont[i];i++)while(tp->ont[i])
	{
		nit=tp->ont[i];
		tp->ont[i]=nit->next;
		free(nit);
	}
	if(tp->pidinfo)
	{
		free(tp->pidinfo);
		tp->pidinfo=NULL;
	}

	return 0;
}

int satip_scan_dvbs(void *h,SATIP_TUNE *l,int n,SATIP_SCAN_PARAMS *cfg,
	int tuners,int mode,int *felist,
	void (*status)(SATIP_TUNE *t,SATIP_SCAN_STATUS *st,void *priv),
	void *priv,SATIP_SCAN_RESULT **result,int *total,char *host,int port)
{
	int i;
	int j;
	int r;
	int idx;
	int hits;
	int miss;
	int besthits;
	int bestmiss;
	int res=SATIP_SYSFAIL;
	int fe=-1;
	int clrfe=0;
	int src=-1;
	int hltot=0;
	int vrtot=0;
	int hlbase=0;
	int vrbase=0;
	int donefd=-1;
	int busy=0;
	int mem[2];
	uint64_t dummy;
	SATIP_UTIL_NIT *nit;
	DVBSFREQ *e;
	DVBSFREQ *x;
	DVBSFREQ *z;
	DVBSFREQ **a;
	DVBSFREQ **b=NULL;
	DVBSFREQ *job=NULL;
	DVBSFREQ *chain=NULL;
	DVBSFREQ **list;
	DVBSFREQ **hllist;
	DVBSFREQ **vrlist;
	DVBSFREQ *hlres[TRANSPONDERS];
	DVBSFREQ *vrres[TRANSPONDERS];
	GLOBALS glob;
	SCANJOB jobs[SATIP_SCAN_PARA];
	struct pollfd p;
	pthread_attr_t attr;

	if(!h||!l||!cfg||n<1)return SATIP_SYSFAIL;

	if(tuners<1||tuners>SATIP_SCAN_PARA)return SATIP_SYSFAIL;

	switch(mode&0x0f)
	{
	case SATIP_SCAN_STD:
	case SATIP_SCAN_FAST:
	case SATIP_SCAN_LIST:
		break;
	default:return SATIP_SYSFAIL;
	}

	switch(mode&0xf0)
	{
	case SATIP_SCAN_HW:
	case SATIP_SCAN_HTTP:
	case SATIP_SCAN_RTSP:
		break;
	default:return SATIP_SYSFAIL;
	}

	if(pthread_attr_init(&attr))return SATIP_SYSFAIL;
	if(pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE)||
		pthread_attr_setstacksize(&attr,STACKSIZE))
	{
		pthread_attr_destroy(&attr);
		return SATIP_SYSFAIL;
	}

	if(!(hllist=malloc(sizeof(DVBSFREQ *)*DVBSMHZ*2)))
	{
		pthread_attr_destroy(&attr);
		return SATIP_SYSFAIL;
	}
	vrlist=&hllist[DVBSMHZ];

	mem[0]=cfg->getnit;
	mem[1]=cfg->tuneonly;

	memset(hllist,0,sizeof(DVBSFREQ *)*DVBSMHZ);
	memset(vrlist,0,sizeof(DVBSFREQ *)*DVBSMHZ);
	memset(&glob,0,sizeof(glob));

	glob.status=status;
	glob.priv=priv;
	a=&job;

	for(i=0;i<n;i++)
	{
		switch(l[i].msys)
		{
		case SATIP_DVBS:
		case SATIP_DVBS2:
			break;
		default:goto fail;
		}
		idx=l[i].freq/1000000;
		if(idx<SATIP_SEARCHMHZ||idx>=DVBSMHZ-SATIP_SEARCHMHZ)goto fail;
		if(fe!=-1)
		{
			if(l[i].fe>0)if(l[i].fe!=fe)goto fail;
		}
		else if(l[i].fe>0)fe=l[i].fe;
		if(src!=-1)
		{
			if(l[i].src>0)if(l[i].src!=src)goto fail;
		}
		else if(l[i].src>0)src=l[i].src;
		if(!(e=malloc(sizeof(DVBSFREQ))))goto fail;
		e->next=NULL;
		e->job=NULL;
		e->flags=INITIAL|((mode&0x0f)!=SATIP_SCAN_LIST?
			GET_NIT|NEEDTUNE:0);
		e->g=&glob;
		e->tune=l[i];

		switch(e->tune.pol)
		{
		case SATIP_POL_H:
		case SATIP_POL_L:
			if(hllist[idx])
			{
				free(e);
				goto fail;
			}
			hllist[idx]=e;
			break;
		case SATIP_POL_V:
		case SATIP_POL_R:
			if(vrlist[idx])
			{
				free(e);
				goto fail;
			}
			vrlist[idx]=e;
			break;
		default:free(e);
			goto fail;
		}

		e->chain=chain;
		chain=e;

		*a=e;
		a=&e->job;
		glob.items++;
	}

	if(src==-1)src=1;

	if(fe!=-1&&tuners>1)goto fail;

	if(felist)
	{
		if(fe!=-1)goto fail;
		for(i=0;i<tuners;i++)
		{
			if(felist[i]<0||felist[i]>255)goto fail;
			for(j=i+1;j<tuners;j++)
				if(felist[i]==felist[j])goto fail;
		}
		if(tuners==1)
		{
			fe=felist[0];
			felist=NULL;
		}
		clrfe=1;
	}
	else if(fe==-1)clrfe=1;

	for(e=job;e;e=e->job)
	{
		e->tune.src=src;
		if(fe!=-1)e->tune.fe=fe;
	}

	if(tuners>1)
	{
		if(pthread_spin_init(&glob.mtx,PTHREAD_PROCESS_PRIVATE))
			goto fail;
		if((donefd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK|EFD_SEMAPHORE))
			==-1)
		{
			pthread_spin_destroy(&glob.mtx);
			goto fail;
		}

		p.fd=donefd;
		p.events=POLLIN;

		for(i=0;i<tuners;i++)
		{
			jobs[i].cfg=*cfg;
			jobs[i].e=NULL;
			jobs[i].h=h;
			jobs[i].host=host;
			jobs[i].port=port;
			jobs[i].mode=mode&0xf0;
			jobs[i].donefd=donefd;
			jobs[i].termfd=cfg->termfd;
			jobs[i].state=0;
			if(felist)jobs[i].cfg.fe=felist[i];
			if(pthread_spin_init(&jobs[i].mtx,
				PTHREAD_PROCESS_PRIVATE))goto f1;
			if((jobs[i].workfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))
				==-1)goto f2;
			if(pthread_create(&jobs[i].th,&attr,dvbsslave,&jobs[i]))
			{
				close(jobs[i].workfd);
f2:				pthread_spin_destroy(&jobs[i].mtx);
f1:				while(i--)
				{
					pthread_cancel(jobs[i].th);
					pthread_join(jobs[i].th,NULL);
					pthread_spin_destroy(&jobs[i].mtx);
					close(jobs[i].workfd);
				}
				pthread_spin_destroy(&glob.mtx);
				close(donefd);
				donefd=-1;
				goto fail;
			}
		}
	}

	b=&job;

	while(*b||busy)
	{
		if(tuners>1)
		{
			while(busy<tuners&&*b)
			{
				for(i=0;i<tuners;i++)
				{
					pthread_spin_lock(&jobs[i].mtx);
					if(!jobs[i].state)
					{
						pthread_spin_unlock(
							&jobs[i].mtx);
						break;
					}
					pthread_spin_unlock(&jobs[i].mtx);
				}

				jobs[i].cfg.getnit=
					((*b)->flags&GET_NIT)?SATIP_SCAN_NIT:0;
				jobs[i].cfg.tuneonly=((*b)->flags&TUNETEST)?1:0;
				(*b)->tuner=i+1;
				jobs[i].e=*b;
				jobs[i].state=1;
				dummy=1;
				dummy=write(jobs[i].workfd,&dummy,
					sizeof(dummy));
				b=&(*b)->job;
				busy++;
			}

repeat:			while(poll(&p,1,-1)<1);
			if(!(p.revents&POLLIN))goto repeat;
			if(read(donefd,&dummy,sizeof(dummy))!=sizeof(dummy))
				goto repeat;
			busy--;

			for(i=0;i<tuners;i++)
			{
				pthread_spin_lock(&jobs[i].mtx);
				if(jobs[i].state==2)
				{
					jobs[i].state=0;
					pthread_spin_unlock(&jobs[i].mtx);
					break;
				}
				pthread_spin_unlock(&jobs[i].mtx);
			}
			r=jobs[i].r;
			x=jobs[i].e;
		}
		else
		{
			x=*b;
			b=&x->job;

			cfg->getnit=(x->flags&GET_NIT)?SATIP_SCAN_NIT:0;
			cfg->tuneonly=(x->flags&TUNETEST)?1:0;
			x->tuner=1;

			switch(mode&0xf0)
			{
			case SATIP_SCAN_HW:
				r=satip_scan_transponder_hw(h,&x->tune,cfg,
					&x->tp,dvbsscancb,x);
				break;
			case SATIP_SCAN_HTTP:
				r=satip_scan_transponder_http(h,&x->tune,cfg,
					&x->tp,dvbsscancb,x,host,port);
				break;
			case SATIP_SCAN_RTSP:
				r=satip_scan_transponder_rtsp(h,&x->tune,cfg,
					&x->tp,dvbsscancb,x,host,port);
				break;
			default:r=SATIP_SYSFAIL;
				break;
			}
		}

		if(tuners>1)pthread_spin_lock(&glob.mtx);
		glob.done++;
		if(tuners>1)pthread_spin_unlock(&glob.mtx);

		if(!x)goto fail;

		switch(r)
		{
		case SATIP_PARTIAL:
		case 0:	x->flags|=SCAN_OK;
			break;
		case SATIP_NODATA:
			continue;
		default:goto fail;
		}

		for(j=0;j<SATIP_NIT_MAX&&x->tp.nit[j];j++)
			for(nit=x->tp.nit[j];nit;nit=nit->next)
				for(i=0;i<nit->total;i++)
					if(nit->data[i].mis==-1)
		{
			idx=nit->data[i].tune.freq/1000000;
			if(idx<SATIP_SEARCHMHZ||idx>=DVBSMHZ-SATIP_SEARCHMHZ)
				continue;

			if(!(e=malloc(sizeof(DVBSFREQ))))goto fail;
			e->next=NULL;
			e->job=NULL;
			e->flags=((mode&0x0f)==SATIP_SCAN_STD?GET_NIT:0);
			e->g=&glob;
			e->tune=nit->data[i].tune;
			e->tune.src=src;
			if(fe!=-1)e->tune.fe=fe;

			switch(e->tune.pol)
			{
			case SATIP_POL_H:
			case SATIP_POL_L:
				list=hllist;
				break;
			case SATIP_POL_V:
			case SATIP_POL_R:
				list=vrlist;
				break;
			default:free(e);
				goto fail;
			}

			e->chain=chain;
			chain=e;

			for(r=idx-SATIP_SEARCHMHZ;r<=idx+SATIP_SEARCHMHZ;r++)
			{
				for(z=list[r];z;z=z->next)
					if(e->tune.sr==z->tune.sr)break;

				if(z)
				{
					idx=r;
					break;
				}
			}

			if(list[idx])
			{
				for(r=0,z=list[idx];z;z=z->next)
					if(e->tune.sr==z->tune.sr)
				{
					r=1;
					break;
				}

				for(z=list[idx];z->next;z=z->next);
				z->next=e;

				if(r)
				{
					if(list[idx]->flags&NEEDTUNE)
					{
						list[idx]->flags&=~NEEDTUNE;
						e->flags|=TUNETEST;
					}
					else continue;
				}
			}
			else list[idx]=e;

			*a=e;
			a=&e->job;

			if(tuners>1)pthread_spin_lock(&glob.mtx);
			glob.items++;
			if(tuners>1)pthread_spin_unlock(&glob.mtx);
		}
	}

	for(r=0;r<DVBSMHZ&&hltot<TRANSPONDERS;r++)if(hllist[r])
	{
		for(e=NULL,x=hllist[r];x;x=x->next)if(x->flags&SCAN_OK)
		{
			if(!e)e=x;
			else if(e->tune.sr<x->tune.sr)e=x;
		}
		if(e)
		{
			if((e->flags&(INITIAL|NEEDTUNE))==INITIAL)
				for(x=hllist[r];x;x=x->next)if(x->flags&SCAN_OK)
					if(x->flags&TUNETEST)
			{
				e->tp.actual=x->tp.actual;
				break;
			}
			e->low=(r-SATIP_SEARCHMHZ)*1000000ULL;
			e->high=(r+SATIP_SEARCHMHZ)*1000000ULL;
			hlres[hltot++]=e;
		}
	}

	for(r=0;r<DVBSMHZ&&vrtot<TRANSPONDERS;r++)if(vrlist[r])
	{
		for(e=NULL,x=vrlist[r];x;x=x->next)if(x->flags&SCAN_OK)
		{
			if(!e)e=x;
			else if(e->tune.sr<x->tune.sr)e=x;
		}
		if(e)
		{
			if((e->flags&(INITIAL|NEEDTUNE))==INITIAL)
				for(x=vrlist[r];x;x=x->next)if(x->flags&SCAN_OK)
					if(x->flags&TUNETEST)
			{
				e->tp.actual=x->tp.actual;
				break;
			}
			e->low=(r-SATIP_SEARCHMHZ)*1000000ULL;
			e->high=(r+SATIP_SEARCHMHZ)*1000000ULL;
			vrres[vrtot++]=e;
		}
	}

	if(hltot)for(hlbase=1;hlbase<hltot;hlbase<<=1);
	if(vrtot)for(vrbase=1;vrbase<hltot;vrbase<<=1);

	if((mode&0x0f)!=SATIP_SCAN_LIST)while(1)
	{
		e=NULL;
		besthits=0;
		bestmiss=0;

		for(r=0;r<hltot;r++)
		{
			if(hlres[r]->flags&NO_MORE)continue;

			miss=0;
			hits=0;
			for(j=0;j<SATIP_NIT_MAX&&hlres[r]->tp.nit[j];j++)
				for(nit=hlres[r]->tp.nit[j];nit;nit=nit->next)
					for(i=0;i<nit->total;i++)
					    if(nit->data[i].mis==-1)
						switch(nit->data[i].tune.pol)
			{
			case SATIP_POL_H:
			case SATIP_POL_L:
				if((idx=lookup(&nit->data[i].tune,hlres,hltot,
					hlbase))==-1)miss++;
				else if(!(hlres[idx]->flags&MATCHED))hits++;
				break;
			case SATIP_POL_V:
			case SATIP_POL_R:
				if((idx=lookup(&nit->data[i].tune,vrres,vrtot,
					vrbase))==-1)miss++;
				else if(!(vrres[idx]->flags&MATCHED))hits++;
				break;
			}

			if(hits>besthits||(hits==besthits&&miss<bestmiss)||
				(hits==besthits&&miss==bestmiss&&e&&
					hlres[r]->tp.duration<e->tp.duration))
			{
				e=hlres[r];
				besthits=hits;
				bestmiss=miss;
			}

			if(!hits)hlres[r]->flags|=NO_MORE;
		}

		for(r=0;r<vrtot;r++)
		{
			if(vrres[r]->flags&NO_MORE)continue;

			miss=0;
			hits=0;
			for(j=0;j<SATIP_NIT_MAX&&vrres[r]->tp.nit[j];j++)
				for(nit=vrres[r]->tp.nit[j];nit;nit=nit->next)
					for(i=0;i<nit->total;i++)
					    if(nit->data[i].mis==-1)
						switch(nit->data[i].tune.pol)
			{
			case SATIP_POL_H:
			case SATIP_POL_L:
				if((idx=lookup(&nit->data[i].tune,hlres,hltot,
					hlbase))==-1)miss++;
				else if(!(hlres[idx]->flags&MATCHED))hits++;
				break;
			case SATIP_POL_V:
			case SATIP_POL_R:
				if((idx=lookup(&nit->data[i].tune,vrres,vrtot,
					vrbase))==-1)miss++;
				else if(!(vrres[idx]->flags&MATCHED))hits++;
				break;
			}

			if(hits>besthits||(hits==besthits&&miss<bestmiss)||
				(hits==besthits&&miss==bestmiss&&e&&
					vrres[r]->tp.duration<e->tp.duration))
			{
				e=vrres[r];
				besthits=hits;
				bestmiss=miss;
			}

			if(!hits)vrres[r]->flags|=NO_MORE;
		}

		if(!e)
		{
			for(r=0;r<hltot;r++)if(!(hlres[r]->flags&MATCHED))
				hlres[r]->tp.fastfind=1;
			for(r=0;r<vrtot;r++)if(!(vrres[r]->flags&MATCHED))
				vrres[r]->tp.fastfind=1;
			break;
		}

		e->flags|=NO_MORE|MATCHED;
		e->tp.fastfind=1;

		for(j=0;j<SATIP_NIT_MAX&&e->tp.nit[j];j++)
			for(nit=e->tp.nit[j];nit;nit=nit->next)
				for(i=0;i<nit->total;i++)
					if(nit->data[i].mis==-1)
						switch(nit->data[i].tune.pol)
		{
		case SATIP_POL_H:
		case SATIP_POL_L:
			if((idx=lookup(&nit->data[i].tune,hlres,hltot,hlbase))
				!=-1)hlres[idx]->flags|=MATCHED;
			break;
		case SATIP_POL_V:
		case SATIP_POL_R:
			if((idx=lookup(&nit->data[i].tune,vrres,vrtot,vrbase))
				!=-1)vrres[idx]->flags|=MATCHED;
			break;
		}
	}

	if(!hltot&&!vrtot)
	{
		*result=NULL;
		*total=0;
	}
	else
	{
		if(!(*result=malloc((hltot+vrtot)*sizeof(SATIP_SCAN_RESULT))))
			goto fail;
		*total=hltot+vrtot;
	}

	for(i=0,j=0,r=0;i<hltot||j<vrtot;)if(i<hltot&&j<vrtot)
	{
		if(hlres[i]->tune.freq<vrres[j]->tune.freq)
			(*result)[r++]=hlres[i++]->tp;
		else if(vrres[j]->tune.freq<hlres[i]->tune.freq)
			(*result)[r++]=vrres[j++]->tp;
		else
		{
			(*result)[r++]=hlres[i++]->tp;
			(*result)[r++]=vrres[j++]->tp;
		}
	}
	else if(i<hltot)(*result)[r++]=hlres[i++]->tp;
	else if(j<vrtot)(*result)[r++]=vrres[j++]->tp;

	for(i=0,r=0;i<*total;i++)
	{
		if((*result)[i].fastfind)r++;
		if(clrfe)(*result)[i].actual.fe=SATIP_UNSPEC;
		switch((*result)[i].actual.ro)
		{
		case SATIP_ROFF_AUTO:
		case SATIP_ROFF_035:
			break;
		default:(*result)[i].actual.msys=SATIP_DVBS2;
		}
		switch((*result)[i].actual.fec)
		{
		case SATIP_FEC_35:
		case SATIP_FEC_45:
		case SATIP_FEC_89:
		case SATIP_FEC_910:
			(*result)[i].actual.msys=SATIP_DVBS2;
			break;
		}
		switch((*result)[i].actual.mtype)
		{
		case SATIP_8PSK:
		case SATIP_16APSK:
		case SATIP_32APSK:
			(*result)[i].actual.msys=SATIP_DVBS2;
			break;
		}
		if((*result)[i].actual.msys==SATIP_DVBS)
			(*result)[i].actual.ro=SATIP_ROFF_AUTO;
		else if((*result)[i].actual.msys==SATIP_DVBS2)
		{
			if(!(*result)[i].actual.ro)
				(*result)[i].actual.ro=SATIP_ROFF_035;
			if(!(*result)[i].actual.plts)
				(*result)[i].actual.plts=SATIP_PLTS_AUTO;
		}
	}

	if(r>*total>>1)for(i=0;i<*total;i++)(*result)[i].fastfind=0;

	res=0;

fail:	if(donefd!=-1)
	{
		while(busy)
		{
			if(poll(&p,1,-1)<1)continue;
			if(!(p.revents&POLLIN))continue;
			if(read(donefd,&dummy,sizeof(dummy))!=sizeof(dummy))
				continue;
			busy--;
		}
		for(i=0;i<tuners;i++)
		{
			if(jobs[i].state!=-1)
			{
				dummy=1;
				jobs[i].e=NULL;
				dummy=write(jobs[i].workfd,&dummy,
					sizeof(dummy));
			}
			pthread_join(jobs[i].th,NULL);
			pthread_spin_destroy(&jobs[i].mtx);
			close(jobs[i].workfd);
		}
		close(donefd);
		pthread_spin_destroy(&glob.mtx);
	}

	while(chain)
	{
		x=chain;
		chain=x->chain;
		if((x->flags&SCAN_OK)&&res)
			satip_scan_transponder_free(h,&x->tp);
		free(x);
	}

	cfg->getnit=mem[0];
	cfg->tuneonly=mem[1];

	free(hllist);
	pthread_attr_destroy(&attr);

	return res;
}

int satip_scan_dvbt(void *h,SATIP_TUNE *t,int tot,SATIP_SCAN_PARAMS *cfg,
	int tuners,int mode,int *felist,
	void (*status)(SATIP_TUNE *t,SATIP_SCAN_STATUS *st,void *priv),
	void *priv,SATIP_SCAN_RESULT **result,int *total,char *host,int port)
{
	int i;
	int j;
	int k;
	int l;
	int m;
	int n;
	int o;
	int fe=-1;
	int clrfe=0;
	int bits=0;
	int donefd=-1;
	int busy=0;
	int mem[2];
	int res=SATIP_SYSFAIL;
	uint64_t dummy;
	SATIP_TUNE tune;
	DVBCTFREQ *e;
	DVBCTFREQ *x;
	DVBCTFREQ *chain=NULL;
	DVBCTFREQ **a=&chain;
	DVBCTFREQ *list[DVBCTMHZ];
	GLOBALS glob;
	SCANJOB jobs[SATIP_SCAN_PARA];
	struct pollfd p;
	pthread_attr_t attr;

	if(!h||!t||!cfg||!result||!total||tot<1)return SATIP_SYSFAIL;

	if(tuners<1||tuners>SATIP_SCAN_PARA)return SATIP_SYSFAIL;

	switch(mode&0xf0)
	{
	case SATIP_SCAN_HTTP:
	case SATIP_SCAN_RTSP:
		if(!host||port<1||port>65535)return SATIP_SYSFAIL;
	case SATIP_SCAN_HW:
		break;
	default:return SATIP_SYSFAIL;
	}

	if(pthread_attr_init(&attr))return SATIP_SYSFAIL;
	if(pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE)||
		pthread_attr_setstacksize(&attr,STACKSIZE))
	{
		pthread_attr_destroy(&attr);
		return SATIP_SYSFAIL;
	}

	mem[0]=cfg->getnit;
	mem[1]=cfg->tuneonly;

	cfg->getnit=0;
	cfg->tuneonly=0;

	memset(&glob,0,sizeof(glob));
	glob.status=status;
	glob.priv=priv;

	satip_util_init_tune(&tune);

	tune.msys=SATIP_DVBT;
	tune.freq=t[0].freq;

	for(n=0;n<2;n++)for(o=0;o<2;o++)for(i=0;i<2;i++)for(j=0;j<2;j++)
		for(k=0;k<2;k++)for(l=0;l<2;l++)for(m=0;m<2;m++)
	{
		tune.bw=i?SATIP_BW_8:SATIP_BW_AUTO;
		tune.tmode=j?SATIP_TMOD_8K:SATIP_TMOD_AUTO;
		tune.mtype=k?SATIP_16Q:SATIP_AUTOQ;
		tune.gi=l?SATIP_GI_14:SATIP_GI_AUTO;
		tune.fec=m?SATIP_FEC_23:SATIP_FEC_AUTO;
		tune.feclp=n?SATIP_FEC_23:SATIP_FEC_AUTO;
		tune.hier=o?SATIP_HIER_NONE:SATIP_HIER_AUTO;

		switch(dvbcttunetest(h,&tune,mode,cfg->termfd,host,port))
		{
		case 0:	tune.msys=SATIP_DVBT2;
			if(dvbcttunetest(h,&tune,mode,cfg->termfd,host,port))
				tune.msys=SATIP_DVBT;
			goto found;
		case -1:goto fail;
		}
	}
	goto fail;

found:	if(tune.bw==SATIP_BW_AUTO)bits|=AUTOBW;
	if(tune.tmode==SATIP_TMOD_AUTO)bits|=AUTOTMODE;
	if(tune.mtype==SATIP_AUTOQ)bits|=AUTOMTYPE;
	if(tune.gi==SATIP_GI_AUTO)bits|=AUTOGI;
	if(tune.fec==SATIP_FEC_AUTO)bits|=AUTOFEC;
	if(tune.feclp==SATIP_FEC_AUTO)bits|=AUTOFECLP;
	if(tune.hier==SATIP_HIER_AUTO)bits|=AUTOHIER;
	if(tune.msys==SATIP_DVBT2)bits|=CAN2G;

	memset(list,0,sizeof(list));

	for(k=0,i=0;i<tot;i++)
	{
		j=cfg->locktimeout;

		switch(t[i].msys)
		{
		case SATIP_DVBT:
			if(bits&CAN2G)j*=2;
			break;
		case SATIP_DVBT2:
			if(!(bits&CAN2G))goto fail;
			break;
		default:goto fail;
		}

		if(!(bits&AUTOBW)&&tune.bw==SATIP_BW_AUTO)j*=6;
		if(!(bits&AUTOTMODE)&&tune.tmode==SATIP_TMOD_AUTO)j*=6;
		if(!(bits&AUTOMTYPE)&&tune.mtype==SATIP_AUTOQ)j*=4;
		if(!(bits&AUTOGI)&&tune.gi==SATIP_GI_AUTO)j*=8;
		if(!(bits&AUTOFEC)&&tune.fec==SATIP_FEC_AUTO)j*=7;
		if(!(bits&AUTOFECLP)&&tune.feclp==SATIP_FEC_AUTO)j*=7;
		if(!(bits&AUTOHIER)&&tune.hier==SATIP_HIER_AUTO)j*=4;

		k+=j;

		if(k>7200)goto fail;

		if(t[i].freq>=(DVBCTMHZ*1000000ULL))goto fail;
		l=t[i].freq/1000000ULL;
		if(list[l])goto fail;
		if(t[i].src)goto fail;
		if(fe!=-1)
		{
			if(t[i].fe>0)if(t[i].fe!=fe)goto fail;
		}
		else if(t[i].fe>0)fe=t[i].fe;
		if(!(e=malloc(sizeof(DVBCTFREQ))))goto fail;
		e->next=NULL;
		e->g=&glob;
		e->tune=t[i];
		e->bits=bits;
		memset(&e->tp,0,sizeof(e->tp));
		list[l]=e;
		*a=e;
		a=&e->next;
		glob.items++;
	}

	if(fe!=-1&&tuners>1)goto fail;

	if(felist)
	{
		if(fe!=-1)goto fail;
		for(i=0;i<tuners;i++)
		{
			if(felist[i]<0||felist[i]>255)goto fail;
			for(j=i+1;j<tuners;j++)
				if(felist[i]==felist[j])goto fail;
		}
		if(tuners==1)
		{
			fe=felist[0];
			felist=NULL;
		}
		clrfe=1;
	}
	else if(fe==-1)clrfe=1;

	for(e=chain;e;e=e->next)if(fe!=-1)e->tune.fe=fe;

	if(tuners>1)
	{
		if(pthread_spin_init(&glob.mtx,PTHREAD_PROCESS_PRIVATE))
			goto fail;
		if((donefd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK|EFD_SEMAPHORE))
			==-1)
		{
			pthread_spin_destroy(&glob.mtx);
			goto fail;
		}

		p.fd=donefd;
		p.events=POLLIN;

		for(i=0;i<tuners;i++)
		{
			jobs[i].cfg=*cfg;
			jobs[i].f=NULL;
			jobs[i].h=h;
			jobs[i].host=host;
			jobs[i].port=port;
			jobs[i].mode=mode&0xf0;
			jobs[i].donefd=donefd;
			jobs[i].termfd=cfg->termfd;
			jobs[i].state=0;
			if(felist)jobs[i].cfg.fe=felist[i];
			if(pthread_spin_init(&jobs[i].mtx,
				PTHREAD_PROCESS_PRIVATE))goto f1;
			if((jobs[i].workfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))
				==-1)goto f2;
			if(pthread_create(&jobs[i].th,&attr,dvbtslave,&jobs[i]))
			{
				close(jobs[i].workfd);
f2:				pthread_spin_destroy(&jobs[i].mtx);
f1:				while(i--)
				{
					pthread_cancel(jobs[i].th);
					pthread_join(jobs[i].th,NULL);
					pthread_spin_destroy(&jobs[i].mtx);
					close(jobs[i].workfd);
				}
				pthread_spin_destroy(&glob.mtx);
				close(donefd);
				donefd=-1;
				goto fail;
			}
		}
	}

	e=chain;
	k=0;

	while(e||busy)
	{
		if(tuners>1)
		{
			while(busy<tuners&&e)
			{
				for(i=0;i<tuners;i++)
				{
					pthread_spin_lock(&jobs[i].mtx);
					if(!jobs[i].state)
					{
						pthread_spin_unlock(
							&jobs[i].mtx);
						break;
					}
					pthread_spin_unlock(&jobs[i].mtx);
				}

				e->tuner=i+1;
				jobs[i].f=e;
				jobs[i].state=1;
				dummy=1;
				dummy=write(jobs[i].workfd,&dummy,
					sizeof(dummy));
				e=e->next;
				busy++;
			}

repeat:			while(poll(&p,1,-1)<1);
			if(!(p.revents&POLLIN))goto repeat;
			if(read(donefd,&dummy,sizeof(dummy))!=sizeof(dummy))
				goto repeat;
			busy--;

			for(i=0;i<tuners;i++)
			{
				pthread_spin_lock(&jobs[i].mtx);
				if(jobs[i].state==2)
				{
					jobs[i].state=0;
					pthread_spin_unlock(&jobs[i].mtx);
					break;
				}
				pthread_spin_unlock(&jobs[i].mtx);
			}
			l=jobs[i].r;
			x=jobs[i].f;
		}
		else
		{
			e->tuner=1;
			l=dvbtscanchan(h,&e->tune,cfg,&e->tp,dvbctscancb,e,mode,
				e->bits,host,port);
			x=e;
			e=e->next;
		}

		if(tuners>1)pthread_spin_lock(&glob.mtx);
		glob.done++;
		if(tuners>1)pthread_spin_unlock(&glob.mtx);

		if(!x)goto fail;

		switch(l)
		{
		case 0:
		case SATIP_PARTIAL:
			x->bits|=SCAN_OK;
			k++;
			break;
		case SATIP_NODATA:
			continue;
			default:goto fail;
		}
	}

	if(!k)
	{
		*result=NULL;
		*total=0;
	}
	else
	{
		if(!(*result=malloc(k*sizeof(SATIP_SCAN_RESULT))))goto fail;
		*total=k;
	}

	for(k=0,i=0;i<DVBCTMHZ;i++)if(list[i])if(list[i]->bits&SCAN_OK)
		(*result)[k++]=list[i]->tp;

	if(clrfe)for(i=0;i<*total;i++)(*result)[i].actual.fe=SATIP_UNSPEC;

	res=0;

fail:	if(donefd!=-1)
	{
		while(busy)
		{
			if(poll(&p,1,-1)<1)continue;
			if(!(p.revents&POLLIN))continue;
			if(read(donefd,&dummy,sizeof(dummy))!=sizeof(dummy))
				continue;
			busy--;
		}
		for(i=0;i<tuners;i++)
		{
			if(jobs[i].state!=-1)
			{
				dummy=1;
				jobs[i].f=NULL;
				dummy=write(jobs[i].workfd,&dummy,
					sizeof(dummy));
			}
			pthread_join(jobs[i].th,NULL);
			pthread_spin_destroy(&jobs[i].mtx);
			close(jobs[i].workfd);
		}
		close(donefd);
		pthread_spin_destroy(&glob.mtx);
	}

	while(chain)
	{
		e=chain;
		chain=e->next;
		if((e->bits&SCAN_OK)&&res)satip_scan_transponder_free(h,&e->tp);
	}

	cfg->getnit=mem[0];
	cfg->tuneonly=mem[1];

	pthread_attr_destroy(&attr);

	return res;
}

int satip_scan_dvbc(void *h,SATIP_TUNE *t,int tot,SATIP_SCAN_PARAMS *cfg,
	int tuners,int mode,int *symtab,int totsym,int *felist,
	void (*status)(SATIP_TUNE *t,SATIP_SCAN_STATUS *st,void *priv),
	void *priv,SATIP_SCAN_RESULT **result,int *total,char *host,int port)
{
	int i;
	int j;
	int k;
	int l;
	int fe=-1;
	int clrfe=0;
	int bits=0;
	int donefd=-1;
	int busy=0;
	int mem[2];
	int res=SATIP_SYSFAIL;
	uint64_t dummy;
	SATIP_TUNE tune;
	DVBCTFREQ *e;
	DVBCTFREQ *x;
	DVBCTFREQ *chain=NULL;
	DVBCTFREQ **a=&chain;
	DVBCTFREQ *list[DVBCTMHZ];
	GLOBALS glob;
	SCANJOB jobs[SATIP_SCAN_PARA];
	struct pollfd p;
	pthread_attr_t attr;

	if(!h||!t||!cfg||!result||!total||tot<1||totsym<0)return SATIP_SYSFAIL;
	if(totsym&&!symtab)return SATIP_SYSFAIL;
	if(tuners<1||tuners>SATIP_SCAN_PARA)return SATIP_SYSFAIL;

	switch(mode&0xf0)
	{
	case SATIP_SCAN_HTTP:
	case SATIP_SCAN_RTSP:
		if(!host||port<1||port>65535)return SATIP_SYSFAIL;
	case SATIP_SCAN_HW:
		break;
	default:return SATIP_SYSFAIL;
	}

	if(pthread_attr_init(&attr))return SATIP_SYSFAIL;
	if(pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE)||
		pthread_attr_setstacksize(&attr,STACKSIZE))
	{
		pthread_attr_destroy(&attr);
		return SATIP_SYSFAIL;
	}

	mem[0]=cfg->getnit;
	mem[1]=cfg->tuneonly;

	cfg->getnit=0;
	cfg->tuneonly=0;

	memset(&glob,0,sizeof(glob));
	glob.status=status;
	glob.priv=priv;

	satip_util_init_tune(&tune);

	tune.msys=SATIP_DVBC;
	tune.freq=t[0].freq;

	if(!totsym&&!t[0].sr)goto fail;

	for(i=0;i<2;i++)for(j=0;j<2;j++)
	{
		tune.specinv=i?SATIP_SPI_OFF:SATIP_SPI_AUTO;
		tune.mtype=j?SATIP_16Q:SATIP_AUTOQ;
		tune.sr=totsym?symtab[0]:t[0].sr;

		switch(dvbcttunetest(h,&tune,mode,cfg->termfd,host,port))
		{
		case 0:	goto found;
		case -1:goto fail;
		}
	}
	goto fail;

found:	if(tune.specinv==SATIP_SPI_AUTO)bits|=AUTOSPI;
	if(tune.mtype==SATIP_AUTOQ)bits|=AUTOMTYPE;

	memset(list,0,sizeof(list));

	for(k=0,i=0;i<tot;i++)
	{
		j=cfg->locktimeout;

		switch(t[i].msys)
		{
		case SATIP_DVBC:
			break;
		default:goto fail;
		}

		if(!t[i].sr)
		{
			if(!totsym)goto fail;
			else j*=totsym;
		}

		if(!(bits&AUTOSPI)&&tune.specinv==SATIP_SPI_AUTO)j*=2;
		if(!(bits&AUTOMTYPE)&&tune.mtype==SATIP_AUTOQ)j*=5;

		k+=j;

		if(k>7200)goto fail;

		if(t[i].freq>=(DVBCTMHZ*1000000ULL))goto fail;
		l=t[i].freq/1000000ULL;
		if(list[l])goto fail;
		if(t[i].src)goto fail;
		if(fe!=-1)
		{
			if(t[i].fe>0)if(t[i].fe!=fe)goto fail;
		}
		else if(t[i].fe>0)fe=t[i].fe;
		if(!(e=malloc(sizeof(DVBCTFREQ))))goto fail;
		e->next=NULL;
		e->g=&glob;
		e->tune=t[i];
		e->bits=bits;
		memset(&e->tp,0,sizeof(e->tp));
		list[l]=e;
		*a=e;
		a=&e->next;
		glob.items++;
	}

	if(fe!=-1&&tuners>1)goto fail;

	if(felist)
	{
		if(fe!=-1)goto fail;
		for(i=0;i<tuners;i++)
		{
			if(felist[i]<0||felist[i]>255)goto fail;
			for(j=i+1;j<tuners;j++)
				if(felist[i]==felist[j])goto fail;
		}
		if(tuners==1)
		{
			fe=felist[0];
			felist=NULL;
		}
		clrfe=1;
	}
	else if(fe==-1)clrfe=1;

	for(e=chain;e;e=e->next)if(fe!=-1)e->tune.fe=fe;

	if(tuners>1)
	{
		if(pthread_spin_init(&glob.mtx,PTHREAD_PROCESS_PRIVATE))
			goto fail;
		if((donefd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK|EFD_SEMAPHORE))
			==-1)
		{
			pthread_spin_destroy(&glob.mtx);
			goto fail;
		}

		p.fd=donefd;
		p.events=POLLIN;

		for(i=0;i<tuners;i++)
		{
			jobs[i].cfg=*cfg;
			jobs[i].f=NULL;
			jobs[i].h=h;
			jobs[i].host=host;
			jobs[i].port=port;
			jobs[i].mode=mode&0xf0;
			jobs[i].donefd=donefd;
			jobs[i].termfd=cfg->termfd;
			jobs[i].state=0;
			jobs[i].symtab=symtab;
			jobs[i].totsym=totsym;
			if(felist)jobs[i].cfg.fe=felist[i];
			if(pthread_spin_init(&jobs[i].mtx,
				PTHREAD_PROCESS_PRIVATE))goto f1;
			if((jobs[i].workfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))
				==-1)goto f2;
			if(pthread_create(&jobs[i].th,&attr,dvbcslave,&jobs[i]))
			{
				close(jobs[i].workfd);
f2:				pthread_spin_destroy(&jobs[i].mtx);
f1:				while(i--)
				{
					pthread_cancel(jobs[i].th);
					pthread_join(jobs[i].th,NULL);
					pthread_spin_destroy(&jobs[i].mtx);
					close(jobs[i].workfd);
				}
				pthread_spin_destroy(&glob.mtx);
				close(donefd);
				donefd=-1;
				goto fail;
			}
		}
	}

	e=chain;
	k=0;

	while(e||busy)
	{
		if(tuners>1)
		{
			while(busy<tuners&&e)
			{
				for(i=0;i<tuners;i++)
				{
					pthread_spin_lock(&jobs[i].mtx);
					if(!jobs[i].state)
					{
						pthread_spin_unlock(
							&jobs[i].mtx);
						break;
					}
					pthread_spin_unlock(&jobs[i].mtx);
				}

				e->tuner=i+1;
				jobs[i].f=e;
				jobs[i].state=1;
				dummy=1;
				dummy=write(jobs[i].workfd,&dummy,
					sizeof(dummy));
				e=e->next;
				busy++;
			}

repeat:			while(poll(&p,1,-1)<1);
			if(!(p.revents&POLLIN))goto repeat;
			if(read(donefd,&dummy,sizeof(dummy))!=sizeof(dummy))
				goto repeat;
			busy--;

			for(i=0;i<tuners;i++)
			{
				pthread_spin_lock(&jobs[i].mtx);
				if(jobs[i].state==2)
				{
					jobs[i].state=0;
					pthread_spin_unlock(&jobs[i].mtx);
					break;
				}
				pthread_spin_unlock(&jobs[i].mtx);
			}
			l=jobs[i].r;
			x=jobs[i].f;
		}
		else
		{
			e->tuner=1;
			l=dvbcscanchan(h,&e->tune,cfg,&e->tp,dvbctscancb,e,mode,
				e->bits,symtab,totsym,host,port);
			x=e;
			e=e->next;
		}

		if(tuners>1)pthread_spin_lock(&glob.mtx);
		glob.done++;
		if(tuners>1)pthread_spin_unlock(&glob.mtx);

		if(!x)goto fail;

		switch(l)
		{
		case 0:
		case SATIP_PARTIAL:
			x->bits|=SCAN_OK;
			k++;
			break;
		case SATIP_NODATA:
			continue;
			default:goto fail;
		}
	}

	if(!k)
	{
		*result=NULL;
		*total=0;
	}
	else
	{
		if(!(*result=malloc(k*sizeof(SATIP_SCAN_RESULT))))goto fail;
		*total=k;
	}

	for(k=0,i=0;i<DVBCTMHZ;i++)if(list[i])if(list[i]->bits&SCAN_OK)
		(*result)[k++]=list[i]->tp;

	if(clrfe)for(i=0;i<*total;i++)(*result)[i].actual.fe=SATIP_UNSPEC;

	res=0;

fail:	if(donefd!=-1)
	{
		while(busy)
		{
			if(poll(&p,1,-1)<1)continue;
			if(!(p.revents&POLLIN))continue;
			if(read(donefd,&dummy,sizeof(dummy))!=sizeof(dummy))
				continue;
			busy--;
		}
		for(i=0;i<tuners;i++)
		{
			if(jobs[i].state!=-1)
			{
				dummy=1;
				jobs[i].f=NULL;
				dummy=write(jobs[i].workfd,&dummy,
					sizeof(dummy));
			}
			pthread_join(jobs[i].th,NULL);
			pthread_spin_destroy(&jobs[i].mtx);
			close(jobs[i].workfd);
		}
		close(donefd);
		pthread_spin_destroy(&glob.mtx);
	}

	while(chain)
	{
		e=chain;
		chain=e->next;
		if((e->bits&SCAN_OK)&&res)satip_scan_transponder_free(h,&e->tp);
	}

	cfg->getnit=mem[0];
	cfg->tuneonly=mem[1];

	pthread_attr_destroy(&attr);

	return res;
}

int satip_scan_free(void *h,SATIP_SCAN_RESULT *result,int total)
{
	int i;

	if(!h||total<0)return SATIP_SYSFAIL;
	if(!total)return 0;
	if(!result)return SATIP_SYSFAIL;

	for(i=0;i<total;i++)satip_scan_transponder_free(h,&result[i]);
	free(result);

	return 0;
}
