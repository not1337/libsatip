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
#include <stdlib.h>
#include <string.h>
#include "satip.h"
#include "common.h"

typedef struct _remap
{
	struct _remap *next;
	pthread_spinlock_t mtx;
	void *srvhandle;
	void *strhandle;
	void *pathandle;
	void *pmthandle;
	void *hwpriv;
	int (*play)(void *handle,SATIP_STRDATA *params,
		void (*stream)(void *id,SATIP_STREAM *stream),
		void (*status)(void *id,SATIP_STATUS *status),
		void *user,int access);
	int (*end)(void *handle,SATIP_STRDATA *params);
	int (*setpids)(void *handle,SATIP_STRDATA *params);
        void (*stream)(void *id,SATIP_STREAM *stream);
        void (*status)(void *id,SATIP_STATUS *status);
	int terminate;
	int pmtpid;
	SATIP_TUNE tune;
	SATIP_PIDS set;
	int bypass:1;
	int hold:1;
	int remap:1;
	int video[3];
	int audio[3];
	int ttx[3];
	int subs[3];
} REMAP;

#ifndef PROFILE

static void remap_stream(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void remap_nostat(void *id,SATIP_STATUS *status) __attribute__ ((hot));
static void remap_status(void *id,SATIP_STATUS *status) __attribute__ ((hot));

#endif

static REMAP *mlist=NULL;

static void remap_stream(void *id,SATIP_STREAM *stream)
{
	int i;
	int pid;
	REMAP *m=(REMAP *)id;

	pthread_spin_lock(&m->mtx);
	if(UNLIKELY(!stream))
	{
		pthread_spin_unlock(&m->mtx);
		m->stream(m->srvhandle,NULL);
	}
	else if(LIKELY(!m->hold))
	{
		if(m->remap&&!(stream->flags&SATIP_FLGSECT))
		{
		    pthread_spin_unlock(&m->mtx);

		    for(i=0;i<stream->fill;i+=188)if(stream->data[i]==0x47)
		    {
			pid=(unsigned char)stream->data[i+1]&0x1f;
			pid<<=8;
			pid|=(unsigned char)stream->data[i+2];

			pthread_spin_lock(&m->mtx);

			if(m->video[2]&&pid==m->video[2])
			{
				stream->data[i+1]=(stream->data[i+1]&0xe0)|
					m->video[1]>>8;
				stream->data[i+2]=m->video[1]&0xff;
				pthread_spin_unlock(&m->mtx);
				continue;
			}

			if(m->audio[2]&&pid==m->audio[2])
			{
				stream->data[i+1]=(stream->data[i+1]&0xe0)|
					m->audio[1]>>8;
				stream->data[i+2]=m->audio[1]&0xff;
				pthread_spin_unlock(&m->mtx);
				continue;
			}

			if(m->ttx[2]&&pid==m->ttx[2])
			{
				stream->data[i+1]=(stream->data[i+1]&0xe0)|
					m->ttx[1]>>8;
				stream->data[i+2]=m->ttx[1]&0xff;
				pthread_spin_unlock(&m->mtx);
				continue;
			}

			if(m->subs[2]&&pid==m->subs[2])
			{
				stream->data[i+1]=(stream->data[i+1]&0xe0)|
					m->subs[1]>>8;
				stream->data[i+2]=m->subs[1]&0xff;
				pthread_spin_unlock(&m->mtx);
				continue;
			}

			pthread_spin_unlock(&m->mtx);
		    }
		}
		else pthread_spin_unlock(&m->mtx);

		m->stream(m->srvhandle,stream);
	}
	else
	{
		pthread_spin_unlock(&m->mtx);
		i=stream->fill;
		stream->fill=0;
		m->stream(m->srvhandle,stream);
		stream->fill=i;
	}
}

static void remap_status(void *id,SATIP_STATUS *status)
{
	REMAP *m=(REMAP *)id;

	m->status(m->srvhandle,status);
}

static void remap_nostat(void *id,SATIP_STATUS *status)
{
}

static void remap_pmt(void *id,SATIP_STREAM *stream)
{
	int i;
	REMAP *m=(REMAP *)id;
	SATIP_UTIL_PMT *pmt;
	SATIP_PIDS set;
	SATIP_STRDATA data;

	if(!stream->fill||!(stream->flags&SATIP_FLGSECT)||!m->set.prognum)
		return;

	if(!(pmt=satip_util_unpack_pmt_section(stream->section,stream->fill)))
		return;

	if(pmt->prognum==m->set.prognum&&!pmt->secnum)
	{
		pthread_spin_lock(&m->mtx);
		m->remap=0;
		m->video[2]=m->audio[2]=m->ttx[2]=m->subs[2]=0;
		pthread_spin_unlock(&m->mtx);

		for(i=0;i<pmt->total;i++)switch(pmt->data[i].type)
		{
		case SATIP_PMT_VIDEO:
		case SATIP_PMT_AVC:
		case SATIP_PMT_HEVC:
			if(!m->video[0])
			{
				pthread_spin_lock(&m->mtx);
				m->video[0]=pmt->data[i].type;
				m->video[2]=m->video[1]=pmt->data[i].pid;
				pthread_spin_unlock(&m->mtx);
			}
			else
			if(m->video[0]==pmt->data[i].type&&!m->video[2])
			{
				pthread_spin_lock(&m->mtx);
				m->video[2]=pmt->data[i].pid;
				if(m->video[1]!=m->video[2])m->remap=1;
				pthread_spin_unlock(&m->mtx);
			}
			break;

		case SATIP_PMT_AUDIO:
		case SATIP_PMT_ADTS:
		case SATIP_PMT_AC3:
		case SATIP_PMT_EAC3:
		case SATIP_PMT_LATM:
			if(!m->audio[0])
			{
				pthread_spin_lock(&m->mtx);
				m->audio[0]=pmt->data[i].type;
				m->audio[2]=m->audio[1]=pmt->data[i].pid;
				pthread_spin_unlock(&m->mtx);
			}
			else
			if(m->audio[0]==pmt->data[i].type&&!m->audio[2])
			{
				pthread_spin_lock(&m->mtx);
				m->audio[2]=pmt->data[i].pid;
				if(m->audio[1]!=m->audio[2])m->remap=1;
				pthread_spin_unlock(&m->mtx);
			}
			break;

		case SATIP_PMT_TTX:
			if(!m->ttx[0])
			{
				pthread_spin_lock(&m->mtx);
				m->ttx[0]=pmt->data[i].type;
				m->ttx[2]=m->ttx[1]=pmt->data[i].pid;
				pthread_spin_unlock(&m->mtx);
			}
			else
			if(m->ttx[0]==pmt->data[i].type&&!m->ttx[2])
			{
				pthread_spin_lock(&m->mtx);
				m->ttx[2]=pmt->data[i].pid;
				if(m->ttx[1]!=m->ttx[2])m->remap=1;
				pthread_spin_unlock(&m->mtx);
			}
			break;

		case SATIP_PMT_SUBS:
			if(!m->subs[0])
			{
				pthread_spin_lock(&m->mtx);
				m->subs[0]=pmt->data[i].type;
				m->subs[2]=m->subs[1]=pmt->data[i].pid;
				pthread_spin_unlock(&m->mtx);
			}
			else
			if(m->subs[0]==pmt->data[i].type&&!m->subs[2])
			{
				pthread_spin_lock(&m->mtx);
				m->subs[2]=pmt->data[i].pid;
				if(m->subs[1]!=m->subs[2])m->remap=1;
				pthread_spin_unlock(&m->mtx);
			}
			break;
		}

		set.prognum=0;
		for(set.numpids=0;set.numpids<pmt->total&&
			set.numpids<SATIP_MAX_PIDS;set.numpids++)
			    set.pids[set.numpids]=pmt->data[set.numpids].pid;
		for(i=0;i<m->set.numpids&&set.numpids<SATIP_MAX_PIDS;i++)
			if(m->set.pids[i]<0x0020)
				set.pids[set.numpids++]=m->set.pids[i];
		data.set=&set;
		data.handle=m->strhandle;
		data.terminate=m->terminate;
		if(m->setpids(m->hwpriv,&data))goto out;

		pthread_spin_lock(&m->mtx);
		m->hold=0;
		pthread_spin_unlock(&m->mtx);

		set.numpids=SATIP_SECTION;
		set.pid=m->pmtpid;
		set.table=0x02;
		set.extra.bits=0;
		set.filtercurrent=1;
		set.version=pmt->vernum;
		set.filterversion=1;
		data.set=&set;
		data.handle=m->pmthandle;
		satip_hw_setpids(m->hwpriv,&data);
	}

out:	free(pmt);
}

static void remap_pat(void *id,SATIP_STREAM *stream)
{
	int i;
	REMAP *m=(REMAP *)id;
	SATIP_UTIL_PAT *pat;
	SATIP_PIDS set;
	SATIP_STRDATA data;

	if(!stream->fill||!(stream->flags&SATIP_FLGSECT)||!m->set.prognum)
		return;

	if(!(pat=satip_util_unpack_pat_section(stream->section,stream->fill)))
		return;

	for(i=0;i<pat->total;i++)if(pat->data[i].prognum==m->set.prognum)
	{
		if(m->pmtpid!=pat->data[i].pmtpid)
		{
			if(m->pmtpid)
			{
				pthread_spin_lock(&m->mtx);
				m->hold=1;
				pthread_spin_unlock(&m->mtx);
			}

			set.prognum=0;
			set.numpids=SATIP_SECTION;
			set.pid=pat->data[i].pmtpid;
			set.table=0x02;
			set.extra.bits=0;
			set.filtercurrent=1;
			data.set=&set;
			data.handle=m->pmthandle;
			data.terminate=m->terminate;
			if(satip_hw_setpids(m->hwpriv,&data))break;

			m->pmtpid=pat->data[i].pmtpid;
		}

		set.prognum=0;
		set.numpids=SATIP_SECTION;
		set.pid=0x0000;
		set.table=0x00;
		set.extra.bits=0;
		set.filtercurrent=1;
		set.version=pat->vernum;
		set.filterversion=1;
		data.set=&set;
		data.handle=m->pathandle;
		data.terminate=m->terminate;
		satip_hw_setpids(m->hwpriv,&data);
		break;
	}

	free(pat);
}

int satip_remap_play(void *handle,SATIP_STRDATA *params,
	int (*play)(void *handle,SATIP_STRDATA *params,
		void (*stream)(void *id,SATIP_STREAM *stream),
		void (*status)(void *id,SATIP_STATUS *status),
		void *user,int access),
	int (*end)(void *handle,SATIP_STRDATA *params),
	int (*setpids)(void *handle,SATIP_STRDATA *params),
	void (*stream)(void *id,SATIP_STREAM *stream),
	void (*status)(void *id,SATIP_STATUS *status),
	void *user,int access)
{
	int r=SATIP_SYSFAIL;
	int mem;
	SATIP_PIDS set;
	SATIP_STRDATA data;
	REMAP *m;

	if(!(m=malloc(sizeof(REMAP))))goto err1;
	memset(m,0,sizeof(REMAP));
	if(pthread_spin_init(&m->mtx,PTHREAD_PROCESS_PRIVATE))goto err2;
	m->play=play;
	m->end=end;
	m->setpids=setpids;
	m->stream=stream;
	m->status=status;
	m->srvhandle=params->handle;
	m->hwpriv=handle;
	m->terminate=params->terminate;
	m->tune=*params->tune;
	m->set=*params->set;
	m->bypass=(access==SATIP_HW_LOCAL?1:0);
	if(params->set->prognum)m->hold=1;
	if(!m->bypass)params->handle=m;
	if((r=play(handle,params,m->bypass?stream:remap_stream,
		m->bypass?status:remap_status,user,access)))goto err3;
	m->strhandle=params->handle;
	params->handle=m;
	if(!m->bypass&&params->set->prognum)
	{
		set.prognum=0;
		set.numpids=SATIP_SECTION;
		set.pid=0x1fff;
		set.table=0x02;
		set.extra.bits=0;
		set.filtercurrent=1;
		data.tune=&m->tune;
		data.handle=m;
		data.terminate=params->terminate;
		data.set=&set;
		mem=m->tune.fe;
		r=satip_hw_play(handle,&data,remap_pmt,remap_nostat,NULL,
			SATIP_HW_LOCAL);
		m->tune.fe=mem;
		if(r)goto err4;
		m->pmthandle=data.handle;

		set.pid=0x0000;
		set.table=0x00;
		data.handle=m;
		mem=m->tune.fe;
		r=satip_hw_play(handle,&data,remap_pat,remap_nostat,NULL,
			SATIP_HW_LOCAL);
		m->tune.fe=mem;
		if(r)goto err5;
		m->pathandle=data.handle;
	}
	m->next=mlist;
	mlist=m;
	return 0;

err5:	params->handle=m->pmthandle;
	satip_hw_end(handle,params);
err4:	params->handle=m->strhandle;
	end(handle,params);
err3:	params->handle=m->srvhandle;
	pthread_spin_destroy(&m->mtx);
err2:	free(m);
err1:	return r;
}

int satip_remap_end(void *handle,SATIP_STRDATA *params)
{
	int r;
	REMAP *m;
	REMAP **d;
	SATIP_STRDATA data;

	for(d=&mlist;*d;d=&(*d)->next)if(*d==params->handle)
	{
		m=*d;
		*d=m->next;
		if(m->pathandle)
		{
			data.handle=m->pathandle;
			satip_hw_end(handle,&data);
		}
		if(m->pmthandle)
		{
			data.handle=m->pmthandle;
			satip_hw_end(handle,&data);
		}
		data.handle=m->strhandle;
		r=m->end(handle,&data);
		pthread_spin_destroy(&m->mtx);
		free(m);
		return r;
	}
	return SATIP_SYSFAIL;
}

int satip_remap_setpids(void *handle,SATIP_STRDATA *params)
{
	int r;
	int mem;
	REMAP *m;
	SATIP_STRDATA data;
	SATIP_PIDS set;

	for(m=mlist;m;m=m->next)if(m==params->handle)
	{
		if(m->pathandle)
		{
			data.handle=m->pathandle;
			satip_hw_end(handle,&data);
			m->pathandle=NULL;
		}
		if(m->pmthandle)
		{
			data.handle=m->pmthandle;
			satip_hw_end(handle,&data);
			m->pmthandle=NULL;
		}
		m->pmtpid=0;
		m->set=*params->set;
		if(m->bypass||!m->set.prognum)
		{
			data.handle=m->strhandle;
			data.set=params->set;
			if((r=m->setpids(handle,&data)))return r;
			pthread_spin_lock(&m->mtx);
			m->hold=0;
			pthread_spin_unlock(&m->mtx);
			return 0;
		}
		pthread_spin_lock(&m->mtx);
		m->hold=1;
		pthread_spin_unlock(&m->mtx);

		data.handle=m->strhandle;
		data.set=params->set;
		if((r=m->setpids(handle,&data)))return r;

		set.prognum=0;
		set.numpids=SATIP_SECTION;
		set.pid=0x1fff;
		set.table=0x02;
		set.extra.bits=0;
		set.filtercurrent=1;
		data.tune=&m->tune;
		data.handle=m;
		data.terminate=m->terminate;
		data.set=&set;
		mem=m->tune.fe;
		r=satip_hw_play(handle,&data,remap_pmt,remap_nostat,NULL,
			SATIP_HW_LOCAL);
		m->tune.fe=mem;
		if(r)return r;

		m->pmthandle=data.handle;

		set.prognum=0;
		set.numpids=SATIP_SECTION;
		set.pid=0x0000;
		set.table=0x00;
		set.extra.bits=0;
		set.filtercurrent=1;
		data.tune=&m->tune;
		data.handle=m;
		data.terminate=m->terminate;
		data.set=&set;
		mem=m->tune.fe;
		r=satip_hw_play(handle,&data,remap_pat,remap_nostat,NULL,
			SATIP_HW_LOCAL);
		m->tune.fe=mem;
		if(r)return r;

		m->pathandle=data.handle;
		return 0;
	}
	return SATIP_SYSFAIL;
}
