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
#include <linux/dvb/frontend.h>
#include <linux/dvb/dmx.h>
#include <linux/dvb/ca.h>
#include <sys/ioctl.h>
#include <sys/timerfd.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <sys/resource.h>
#include <sys/capability.h>
#include <sys/uio.h>
#include <sched.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <poll.h>
#include "satip.h"
#include "common.h"

#define TUNER_IDLE_SECS	15
#define MAX_EPOLL	32
#define DEMUX_BUFFER	131036
#define SECTION_BUFFER	8192
#define MIN_API		0x050a
#define MAX_CAM_MSGLEN	2048

#define DVBS		(1<<(SATIP_DVBS-1))
#define DVBS2		(1<<(SATIP_DVBS2-1))
#define DVBT		(1<<(SATIP_DVBT-1))
#define DVBT2		(1<<(SATIP_DVBT2-1))
#define DVBC		(1<<(SATIP_DVBC-1))
#define DVBC2		(1<<(SATIP_DVBC2-1))

#define COPY		-1
#define DONE		0
#define CANCEL		1
#define FAILED		2
#define PREPARE		3
#define WAIT		4
#define TONEOFF		5
#define SENDCMD		6
#define SENDRECV	7
#define TONEBURST	8
#define TONE		9
#define TUNE		10
#define LOCKWAIT	11
#define STATUS		12

typedef struct
{
	unsigned int msys;
	unsigned int caps;
	unsigned int fmin;
	unsigned int fmax;
	unsigned int step;
	unsigned int smin;
	unsigned int smax;
} FE_INFO;

typedef struct
{
	struct dtv_properties props;
	struct dtv_property prop[16];
	struct dvb_diseqc_master_cmd dcmd;
	unsigned char *seq;
	int dburst;
	int dtone;
	int dsrc;
} HWTUNE;

typedef struct _tuner
{
	struct _tuner *next;
	struct _group *group;
	struct _stream *item;
	struct _stream *slist;
	SATIP_HW_CAM_IO *camsg;
	unsigned char *dseq;
	unsigned long long bytes;
	unsigned long long txbytes;
	unsigned long long txticks;
	SATIP_TUNE tune;
	SATIP_TUNE initial;
	FE_INFO info;
	HWTUNE hwtune;
	pthread_t tuningmgr;
	pthread_t streamer;
	pthread_t camgr;
	pthread_spinlock_t mtx;
	pthread_mutex_t dtx;
	int adapter;
	int frontend;
	int demux;
	int ca;
	int snrshift;
	int snrspeed;
	int inversion;
	int lna;
	int idleflush;
	int fast;
	int prefer;
	int explicit;
	int noshare;
	int access;
	int tuning;
	int use;
	int idle;
	int ticks;
	int lock;
	int level;
	int quality;
	int tunefd;
	int cafd;
	int catfd;
	int epfd;
	int tfd;
	int carfd;
	int caafd;
	int ocrfd;
	int ocafd;
	int dsfd;
	int diseqc;
	int dstate;
	int dwait;
	int d2wait;
	int dforce;
	int currsrc;
	int lnb;
	int annex;
	int feid;
	int caans;
	int req;
	int ans;
	int reqfd;
	int ansfd;
	int busy;
	int srcnum;
	int src[0];
} TUNER;

typedef struct
{
#pragma pack(push,1)
	void *unused;
	void (*func)();
#pragma pack(pop)
} FUNC;

typedef struct
{
#pragma pack(push,1)
	struct _stream *stream;
	void (*func)();
#pragma pack(pop)
} TIMER;

typedef struct _stream
{
#pragma pack(push,1)
	struct _stream *next;
	void (*func)();
#pragma pack(pop)
	void (*streamer)(void *id,SATIP_STREAM *stream);
	void (*stater)(void *id,SATIP_STATUS *status);
	TUNER *tuner;
	void *did;
	TIMER *timer;
	int fd;
	int tfd;
	int idle;
	int rtcp;
	int index;
	int ramp;
	SATIP_STATUS status;
	SATIP_STREAM *stream;
	SATIP_STREAM queue[SATIP_MAX_BURST];
} STREAM;

typedef struct _group
{
	int limit;
	int streams;
	int termfd;
	void (*camcb)(SATIP_HW_CAM_IO *msg,void *campriv);
	void *campriv;
	TUNER *tlist;
	pthread_attr_t attr;
	pthread_mutex_t stx;
	pthread_mutex_t ctx;
} GROUP;

#ifndef PROFILE

static void streamtrigger(TUNER *t,void *data) __attribute__ ((hot));
static void dostream(TUNER *t,void *data) __attribute__ ((hot));
static void dosection(TUNER *t,void *data) __attribute__ ((hot));
static void timetick(TUNER *t,void *data) __attribute__ ((hot));
static void *streamer(void *data) __attribute__ ((hot));
static unsigned int khz90(void) __attribute__ ((hot));

int satip_hw_add(void *handle,SATIP_HW_TUNERCFG *tuner,SATIP_CFGINFO *info)
	__attribute__ ((cold));
void *satip_hw_init(SATIP_HW_CONFIG *config,SATIP_CFGINFO *info)
	__attribute__ ((cold));
void satip_hw_fini(void *handle) __attribute__ ((cold));

#endif

static unsigned char nosat[]=
{
	DONE,
	CANCEL,
	FAILED,
	PREPARE,
	TUNE,
	LOCKWAIT,
	STATUS
};

static unsigned char nodiseqc[]=
{
	DONE,
	CANCEL,
	FAILED,
	PREPARE,
	TONE,
	TUNE,
	LOCKWAIT,
	STATUS
};

static unsigned char toneburst[]=
{
	DONE,
	CANCEL,
	FAILED,
	PREPARE,
	WAIT,
	TONEOFF,
	TONEBURST,
	TONE,
	TUNE,
	LOCKWAIT,
	STATUS
};

static unsigned char diseqc1x[]=
{
	DONE,
	CANCEL,
	FAILED,
	PREPARE,
	WAIT,
	TONEOFF,
	SENDCMD,
	TONE,
	TUNE,
	LOCKWAIT,
	STATUS
};

static unsigned char diseqc1xt[]=
{
	DONE,
	CANCEL,
	FAILED,
	PREPARE,
	WAIT,
	TONEOFF,
	SENDCMD,
	TONEBURST,
	TONE,
	TUNE,
	LOCKWAIT,
	STATUS
};

static unsigned char diseqc2x[]=
{
	DONE,
	CANCEL,
	FAILED,
	PREPARE,
	WAIT,
	TONEOFF,
	SENDRECV,
	TONE,
	TUNE,
	LOCKWAIT,
	STATUS
};

static unsigned char diseqc2xt[]=
{
	DONE,
	CANCEL,
	FAILED,
	PREPARE,
	WAIT,
	TONEOFF,
	SENDRECV,
	TONEBURST,
	TONE,
	TUNE,
	LOCKWAIT,
	STATUS
};

static int fe_open(int adapter,int id,int mode)
{
	char bfr[64];
	char *d;

	switch(mode>>1)
	{
	case 0:	d="frontend";
		break;
	case 1:	d="demux";
		break;
	case 2:	d="ca";
		break;
	default:return -1;
	}

	if(adapter<0||adapter>255||id<0||id>255)return -1;

	sprintf(bfr,"/dev/dvb/adapter%d/%s%d",adapter,d,id);
	return open(bfr,((mode&1)?O_RDWR:O_RDONLY)|O_NONBLOCK|O_CLOEXEC);
}

static int dmx_open(int adapter,int id,int bufsize)
{
	int fd;

	if((fd=fe_open(adapter,id,3))==-1)return -1;
	if(ioctl(fd,DMX_SET_BUFFER_SIZE,bufsize))
	{
		close(fd);
		return -2;
	}
	return fd;
}

static int dmx_set_filter(int fd,int pid)
{
	struct dmx_pes_filter_params filter;

	if(pid<-1||pid>8191)return -1;

	memset(&filter,0,sizeof(filter));
	filter.pid=(pid==-1)?0x2000:pid;
	filter.input=DMX_IN_FRONTEND;
	filter.output=DMX_OUT_TSDEMUX_TAP;
	filter.pes_type=DMX_PES_OTHER;

	if(ioctl(fd,DMX_SET_PES_FILTER,&filter))return -2;
	return 0;
}

static int dmx_add_filter(int fd,int pid)
{
	unsigned short val=(unsigned short)pid;

	if(pid<0||pid>8191)return -1;

	return ioctl(fd,DMX_ADD_PID,&val)?-2:0;
}

static int dmx_del_filter(int fd,int pid)
{
	unsigned short val=(unsigned short)pid;

	if(pid<0||pid>8191)return -1;

	return ioctl(fd,DMX_REMOVE_PID,&val)?-2:0;
}

static int dmx_set_section(int fd,int pid,int table,int filtercurrent,
	int filterversion,int version)
{
	struct dmx_sct_filter_params filter;

	if(pid<0||pid>8191)return -1;

	memset(&filter,0,sizeof(filter));
	filter.pid=pid;
	filter.flags=DMX_CHECK_CRC;
	if(table>=0&&table<256)
	{
		filter.filter.filter[0]=table;
		filter.filter.mask[0]=0xff;
	}
	if(filtercurrent)
	{
		filter.filter.filter[3]=0x01;
		filter.filter.mask[3]=0x01;
	}
	if(filterversion)
	{
		filter.filter.filter[3]|=(version&0x1f)<<1;
		filter.filter.mask[3]|=0x3e;
		filter.filter.mode[3]=0x3e;
	}
	if(ioctl(fd,DMX_SET_FILTER,&filter))return -2;
	return 0;
}

static int dmx_set_secfilter(int fd,int pid,int crc,unsigned char *filter,
	unsigned char *mask,unsigned char *mode)
{
	struct dmx_sct_filter_params flt;

	if(pid<0||pid>8191)return -1;

	memset(&flt,0,sizeof(flt));
	flt.pid=pid;
	if(crc)flt.flags=DMX_CHECK_CRC;
	memcpy(flt.filter.filter,filter,SATIP_FILTERSZ);
	memcpy(flt.filter.mask,mask,SATIP_FILTERSZ);
	memcpy(flt.filter.mode,mode,SATIP_FILTERSZ);
	if(ioctl(fd,DMX_SET_FILTER,&flt))return -2;
	return 0;
}

static int dmx_start(int fd)
{
	if(ioctl(fd,DMX_START))return -1;
	return 0;
}

static int dmx_stop(int fd)
{
	if(ioctl(fd,DMX_STOP))return -1;
	return 0;
}

static int fe_info(TUNER *t)
{
	int i;
	int fd;
	struct dvb_frontend_info inf;
	struct dtv_properties props;
	struct dtv_property prop[2];

	if((fd=fe_open(t->adapter,t->frontend,0))==-1)goto err1;
	memset(&t->info,0,sizeof(FE_INFO));

	props.num=2;
	props.props=prop;
	prop[0].cmd=DTV_API_VERSION;
	prop[1].cmd=DTV_ENUM_DELSYS;
	if(ioctl(fd,FE_GET_PROPERTY,&props))goto err2;

	if(prop[0].u.data<MIN_API)goto err2;

	for(i=0;i<prop[1].u.buffer.len;i++)switch(prop[1].u.buffer.data[i])
	{
	case SYS_DVBS:
		t->info.msys|=DVBS;
		break;
	case SYS_DVBS2:
		t->info.msys|=DVBS2;
		break;
	case SYS_DVBT:
		t->info.msys|=DVBT;
		break;
	case SYS_DVBT2:
		t->info.msys|=DVBT2;
		break;
	case SYS_DVBC_ANNEX_A:
		t->info.msys|=DVBC;
		break;
	}

	switch(t->info.msys)
	{
	case DVBS:
	case DVBS2:
	case DVBS|DVBS2:
	case DVBT:
	case DVBT2:
	case DVBT|DVBT2:
	case DVBC:
	case DVBC2:
	case DVBC|DVBC2:
		break;
	default:goto err2;
	}

	if(ioctl(fd,FE_GET_INFO,&inf))goto err2;

	if(inf.caps&FE_CAN_INVERSION_AUTO)t->info.caps|=SATIP_SPI_AUTO;
	if(inf.caps&FE_CAN_FEC_AUTO)t->info.caps|=SATIP_FEC_AUTO;
	if(inf.caps&FE_CAN_QAM_AUTO)t->info.caps|=SATIP_AUTOQ;
	if(inf.caps&FE_CAN_TRANSMISSION_MODE_AUTO)t->info.caps|=SATIP_TMOD_AUTO;
	if(inf.caps&FE_CAN_BANDWIDTH_AUTO)t->info.caps|=SATIP_BW_AUTO;
	if(inf.caps&FE_CAN_GUARD_INTERVAL_AUTO)t->info.caps|=SATIP_GI_AUTO;
	if(t->info.msys&(DVBS|DVBS2))
		t->info.caps|=SATIP_ROFF_AUTO|SATIP_PLTS_AUTO|SATIP_AUTOQ;
	if(inf.caps&FE_CAN_HIERARCHY_AUTO)t->info.caps|=SATIP_HIER_AUTO;

	t->info.fmin=inf.frequency_min;
	t->info.fmax=inf.frequency_max;
	t->info.step=inf.frequency_stepsize;

	if(t->info.msys&(DVBS|DVBS2))
	{
		t->info.fmin*=1000;
		t->info.fmax*=1000;
		t->info.step*=1000;
	}

	if(!t->info.step)t->info.step=1;

	if(t->info.msys&(DVBS|DVBS2|DVBC))
	{
		t->info.smin=inf.symbol_rate_min;
		t->info.smax=inf.symbol_rate_max;
	}

	close(fd);
	return 0;

err2:	close(fd);
err1:	return -1;
}

static int fe_dvbs2_tune(TUNER *tuner)
{
	int idx;
	int hl;
	int hv;
	int val;
	unsigned int s;
	unsigned int l;
	unsigned int h;
	unsigned long long freq;

	if(!(tuner->info.msys&(DVBS|DVBS2)))return -1;
	if(tuner->tune.src<1||tuner->tune.src>255)return -1;

	switch(tuner->lnb)
	{
	case SATIP_LNB_UNIV:
		if(tuner->tune.freq<10700000000ULL||
			tuner->tune.freq>12750000000ULL)return -1;
		if((hl=(tuner->tune.freq>=11700000000ULL)?1:0))
			freq=tuner->tune.freq-10600000000ULL;
		else freq=tuner->tune.freq-9750000000ULL;
		break;

	case SATIP_LNB_DBS:
		if(tuner->tune.freq<12200000000ULL||
			tuner->tune.freq>12700000000ULL)return -1;
		hl=0;
		freq=tuner->tune.freq-11250000000ULL;
		break;

	case SATIP_LNB_CMONO:
		if(tuner->tune.freq<3700000000ULL||
			tuner->tune.freq>4200000000ULL)return -1;
		hl=0;
		freq=5150000000ULL-tuner->tune.freq;
		break;

	case SATIP_LNB_CMULT:
		if(tuner->tune.freq<3700000000ULL||
			tuner->tune.freq>4200000000ULL)return -1;
		hl=0;
		if(tuner->tune.pol==SATIP_POL_V)
			freq=5150000000ULL-tuner->tune.freq;
		else freq=5750000000ULL-tuner->tune.freq;
		break;

	case SATIP_LNB_AUS:
		if(tuner->tune.freq<11700000000ULL||
			tuner->tune.freq>12750000000ULL)return -1;
		hl=0;
		freq=tuner->tune.freq-10700000000ULL;
		break;

	default:return -1;
	}

	if(freq<tuner->info.fmin||freq>tuner->info.fmax||
		tuner->tune.sr<tuner->info.smin||
		tuner->tune.sr>tuner->info.smax)return -1;
	if(freq%1000)return -1;

	if(tuner->info.step>1)
	{
		s=(freq-tuner->info.fmin)/tuner->info.step;
		l=tuner->info.fmin+s*tuner->info.step;
		h=l+tuner->info.step;
		if(freq!=l)
		{
			if(freq-l<h-freq)freq=l;
			else freq=h;
			if(freq<tuner->info.fmin)freq=tuner->info.fmin;
			else if(freq>tuner->info.fmax)freq=tuner->info.fmax;
		}
	}

	switch(tuner->tune.pol)
	{
	case SATIP_POL_H:
	case SATIP_POL_L:
		hv=1;
		break;
	case SATIP_POL_V:
	case SATIP_POL_R:
		hv=0;
		break;
	default:return -1;
	}

	memset(&tuner->hwtune.dcmd,0,sizeof(tuner->hwtune.dcmd));
	tuner->hwtune.dcmd.msg_len=4;

	switch(tuner->diseqc)
	{
	case SATIP_DSC_TONE:
		idx=(tuner->tune.src-1)&1;
		tuner->hwtune.dburst=idx?SEC_MINI_B:SEC_MINI_A;
		break;

	case SATIP_DSC_1_0:
		idx=(tuner->tune.src-1)&3;
		tuner->hwtune.dcmd.msg[0]=0xe0;
		tuner->hwtune.dcmd.msg[1]=0x10;
		tuner->hwtune.dcmd.msg[2]=0x38;
		tuner->hwtune.dcmd.msg[3]=0xf0|(idx<<2)|(hv?2:0)|(hl?1:0);
		break;

	case SATIP_DSC_2_0:
		idx=(tuner->tune.src-1)&3;
		tuner->hwtune.dcmd.msg[0]=0xe2;
		tuner->hwtune.dcmd.msg[1]=0x10;
		tuner->hwtune.dcmd.msg[2]=0x38;
		tuner->hwtune.dcmd.msg[3]=0xf0|(idx<<2)|(hv?2:0)|(hl?1:0);
		break;

	case SATIP_DSC_1_1:
		idx=(tuner->tune.src-1)&15;
		tuner->hwtune.dcmd.msg[0]=0xe0;
		tuner->hwtune.dcmd.msg[1]=0x10;
		tuner->hwtune.dcmd.msg[2]=0x39;
		tuner->hwtune.dcmd.msg[3]=0xf0|idx;
		break;

	case SATIP_DSC_2_1:
		idx=(tuner->tune.src-1)&15;
		tuner->hwtune.dcmd.msg[0]=0xe2;
		tuner->hwtune.dcmd.msg[1]=0x10;
		tuner->hwtune.dcmd.msg[2]=0x39;
		tuner->hwtune.dcmd.msg[3]=0xf0|idx;
		break;

	case SATIP_DSC_1_0_T:
		idx=(tuner->tune.src-1)&7;
		tuner->hwtune.dburst=(idx&1)?SEC_MINI_B:SEC_MINI_A;
		idx>>=1;
		tuner->hwtune.dcmd.msg[0]=0xe0;
		tuner->hwtune.dcmd.msg[1]=0x10;
		tuner->hwtune.dcmd.msg[2]=0x38;
		tuner->hwtune.dcmd.msg[3]=0xf0|(idx<<2)|(hv?2:0)|(hl?1:0);
		break;

	case SATIP_DSC_2_0_T:
		idx=(tuner->tune.src-1)&7;
		tuner->hwtune.dburst=(idx&1)?SEC_MINI_B:SEC_MINI_A;
		idx>>=1;
		tuner->hwtune.dcmd.msg[0]=0xe2;
		tuner->hwtune.dcmd.msg[1]=0x10;
		tuner->hwtune.dcmd.msg[2]=0x38;
		tuner->hwtune.dcmd.msg[3]=0xf0|(idx<<2)|(hv?2:0)|(hl?1:0);
		break;

	case SATIP_DSC_1_1_T:
		idx=(tuner->tune.src-1)&31;
		tuner->hwtune.dburst=(idx&1)?SEC_MINI_B:SEC_MINI_A;
		idx>>=1;
		tuner->hwtune.dcmd.msg[0]=0xe0;
		tuner->hwtune.dcmd.msg[1]=0x10;
		tuner->hwtune.dcmd.msg[2]=0x39;
		tuner->hwtune.dcmd.msg[3]=0xf0|idx;
		break;

	case SATIP_DSC_2_1_T:
		idx=(tuner->tune.src-1)&31;
		tuner->hwtune.dburst=(idx&1)?SEC_MINI_B:SEC_MINI_A;
		idx>>=1;
		tuner->hwtune.dcmd.msg[0]=0xe2;
		tuner->hwtune.dcmd.msg[1]=0x10;
		tuner->hwtune.dcmd.msg[2]=0x39;
		tuner->hwtune.dcmd.msg[3]=0xf0|idx;
		break;

	default:idx=0;
	}

	tuner->hwtune.dtone=hl?SEC_TONE_ON:SEC_TONE_OFF;
	tuner->hwtune.dsrc=tuner->tune.src;

	tuner->hwtune.props.props=tuner->hwtune.prop;

	tuner->hwtune.prop[0].cmd=DTV_CLEAR;
	tuner->hwtune.prop[0].u.data=0;

	tuner->hwtune.prop[1].cmd=DTV_DELIVERY_SYSTEM;
	tuner->hwtune.prop[1].u.data=
		(tuner->tune.msys==SATIP_DVBS?SYS_DVBS:SYS_DVBS2);

	if(tuner->tune.msys==SATIP_DVBS)val=QPSK;
	else switch(tuner->tune.mtype)
	{
	case SATIP_QPSK:
	case SATIP_AUTOQ:
		val=QPSK;
		break;
	case SATIP_8PSK:
		val=PSK_8;
		break;
	case SATIP_16APSK:
		val=APSK_16;
		break;
	case SATIP_32APSK:
		val=APSK_32;
		break;
	default:return -1;
	}
	tuner->hwtune.prop[2].cmd=DTV_MODULATION;
	tuner->hwtune.prop[2].u.data=val;

	tuner->hwtune.prop[3].cmd=DTV_FREQUENCY;
	tuner->hwtune.prop[3].u.data=freq/1000;

	tuner->hwtune.prop[4].cmd=DTV_SYMBOL_RATE;
	tuner->hwtune.prop[4].u.data=tuner->tune.sr;

	tuner->hwtune.prop[5].cmd=DTV_INVERSION;
	tuner->hwtune.prop[5].u.data=
	    ((tuner->info.caps&SATIP_SPI_AUTO)?INVERSION_AUTO:tuner->inversion);

	switch(tuner->tune.fec)
	{
	case SATIP_FEC_12:
		val=FEC_1_2;
		break;
	case SATIP_FEC_23:
		val=FEC_2_3;
		break;
	case SATIP_FEC_34:
		val=FEC_3_4;
		break;
	case SATIP_FEC_35:
		val=FEC_3_5;
		break;
	case SATIP_FEC_45:
		val=FEC_4_5;
		break;
	case SATIP_FEC_56:
		val=FEC_5_6;
		break;
	case SATIP_FEC_78:
		val=FEC_7_8;
		break;
	case SATIP_FEC_89:
		val=FEC_8_9;
		break;
	case SATIP_FEC_910:
		val=FEC_9_10;
		break;
	case SATIP_FEC_AUTO:
		if(tuner->info.caps&SATIP_FEC_AUTO)
		{
			val=FEC_AUTO;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[6].cmd=DTV_INNER_FEC;
	tuner->hwtune.prop[6].u.data=val;

	if(tuner->tune.msys==SATIP_DVBS2)
	{
		tuner->hwtune.prop[7].cmd=DTV_STREAM_ID;
		tuner->hwtune.prop[7].u.data=NO_STREAM_ID_FILTER;

		switch(tuner->tune.plts)
		{
		case SATIP_PLTS_OFF:
			val=PILOT_OFF;
			break;
		case SATIP_PLTS_ON:
			val=PILOT_ON;
			break;
		case SATIP_PLTS_AUTO:
			val=PILOT_AUTO;
			break;
		default:return -1;
		}
		tuner->hwtune.prop[8].cmd=DTV_PILOT;
		tuner->hwtune.prop[8].u.data=val;

		switch(tuner->tune.ro)
		{
		case SATIP_ROFF_035:
			val=ROLLOFF_35;
			break;
		case SATIP_ROFF_025:
			val=ROLLOFF_25;
			break;
		case SATIP_ROFF_020:
			val=ROLLOFF_20;
			break;
		case SATIP_ROFF_AUTO:
			val=ROLLOFF_AUTO;
			break;
		default:return -1;
		}
		tuner->hwtune.prop[9].cmd=DTV_ROLLOFF;
		tuner->hwtune.prop[9].u.data=val;

		tuner->hwtune.props.num=10;
	}
	else tuner->hwtune.props.num=7;

	tuner->hwtune.prop[tuner->hwtune.props.num].cmd=DTV_VOLTAGE;
	tuner->hwtune.prop[tuner->hwtune.props.num].u.data=
		(hv?SEC_VOLTAGE_18:SEC_VOLTAGE_13);
	tuner->hwtune.props.num++;

	if(tuner->diseqc==SATIP_DSC_NONE||tuner->currsrc==tuner->tune.src)
		tuner->hwtune.seq=nodiseqc;
	else tuner->hwtune.seq=tuner->dseq;

	return 0;
}

static int fe_dvbt2_tune(TUNER *tuner)
{
	unsigned long long freq;
	int val;
	unsigned int s;
	unsigned int l;
	unsigned int h;

	if(!(tuner->info.msys&(DVBT|DVBT2)))return -1;

	if(tuner->tune.freq<tuner->info.fmin||tuner->tune.freq>tuner->info.fmax)
		return -1;

	if(tuner->info.step>1)
	{
		s=(tuner->tune.freq-tuner->info.fmin)/tuner->info.step;
		l=tuner->info.fmin+s*tuner->info.step;
		h=l+tuner->info.step;
		if(tuner->tune.freq!=l)
		{
			if(tuner->tune.freq-l<h-tuner->tune.freq)freq=l;
			else freq=h;
			if(freq<tuner->info.fmin)freq=tuner->info.fmin;
			else if(freq>tuner->info.fmax)freq=tuner->info.fmax;
		}
		else freq=tuner->tune.freq;
	}
	else freq=tuner->tune.freq;

	tuner->hwtune.props.props=tuner->hwtune.prop;

	tuner->hwtune.prop[0].cmd=DTV_CLEAR;
	tuner->hwtune.prop[0].u.data=0;
	tuner->hwtune.prop[1].cmd=DTV_FREQUENCY;
	tuner->hwtune.prop[1].u.data=freq;

	switch(tuner->tune.hier)
	{
	case SATIP_HIER_NONE:
		val=HIERARCHY_NONE;
		break;
	case SATIP_HIER_1:
		val=HIERARCHY_1;
		break;
	case SATIP_HIER_2:
		val=HIERARCHY_2;
		break;
	case SATIP_HIER_4:
		val=HIERARCHY_4;
		break;
	case SATIP_HIER_AUTO:
		if(tuner->info.caps&SATIP_HIER_AUTO)
		{
			val=HIERARCHY_AUTO;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[2].cmd=DTV_HIERARCHY;
	tuner->hwtune.prop[2].u.data=val;

	tuner->hwtune.prop[3].cmd=DTV_LNA;
	tuner->hwtune.prop[3].u.data=tuner->lna;
	tuner->hwtune.prop[4].cmd=DTV_DELIVERY_SYSTEM;
	tuner->hwtune.prop[4].u.data=
		(tuner->tune.msys==SATIP_DVBT?SYS_DVBT:SYS_DVBT2);

	switch(tuner->tune.bw)
	{
	case SATIP_BW_1712:
		val=1712000;
		break;
	case SATIP_BW_5:
		val=5000000;
		break;
	case SATIP_BW_6:
		val=6000000;
		break;
	case SATIP_BW_7:
		val=7000000;
		break;
	case SATIP_BW_8:
		val=8000000;
		break;
	case SATIP_BW_10:
		val=10000000;
		break;
	case SATIP_BW_AUTO:
		if(tuner->info.caps&SATIP_BW_AUTO)
		{
			val=0;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[5].cmd=DTV_BANDWIDTH_HZ;
	tuner->hwtune.prop[5].u.data=val;

	switch(tuner->tune.fec)
	{
	case SATIP_FEC_12:
		val=FEC_1_2;
		break;
	case SATIP_FEC_23:
		val=FEC_2_3;
		break;
	case SATIP_FEC_34:
		val=FEC_3_4;
		break;
	case SATIP_FEC_35:
		val=FEC_3_5;
		break;
	case SATIP_FEC_45:
		val=FEC_4_5;
		break;
	case SATIP_FEC_56:
		val=FEC_5_6;
		break;
	case SATIP_FEC_78:
		val=FEC_7_8;
		break;
	case SATIP_FEC_AUTO:
		if(tuner->info.caps&SATIP_FEC_AUTO)
		{
			val=FEC_AUTO;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[6].cmd=DTV_CODE_RATE_HP;
	tuner->hwtune.prop[6].u.data=val;

	switch(tuner->tune.feclp)
	{
	case SATIP_FEC_12:
		val=FEC_1_2;
		break;
	case SATIP_FEC_23:
		val=FEC_2_3;
		break;
	case SATIP_FEC_34:
		val=FEC_3_4;
		break;
	case SATIP_FEC_35:
		val=FEC_3_5;
		break;
	case SATIP_FEC_45:
		val=FEC_4_5;
		break;
	case SATIP_FEC_56:
		val=FEC_5_6;
		break;
	case SATIP_FEC_78:
		val=FEC_7_8;
		break;
	case SATIP_FEC_AUTO:
		if(tuner->info.caps&SATIP_FEC_AUTO)
		{
			val=FEC_AUTO;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[7].cmd=DTV_CODE_RATE_LP;
	tuner->hwtune.prop[7].u.data=val;

	switch(tuner->tune.mtype)
	{
	case SATIP_QPSK:
		val=QPSK;
		break;
	case SATIP_16Q:
		val=QAM_16;
		break;
	case SATIP_64Q:
		val=QAM_64;
		break;
	case SATIP_256Q:
		val=QAM_256;
		break;
	case SATIP_AUTOQ:
		if(tuner->info.caps&SATIP_AUTOQ)
		{
			val=QAM_AUTO;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[8].cmd=DTV_MODULATION;
	tuner->hwtune.prop[8].u.data=val;

	switch(tuner->tune.tmode)
	{
	case SATIP_TMOD_1K:
		val=TRANSMISSION_MODE_1K;
		break;
	case SATIP_TMOD_2K:
		val=TRANSMISSION_MODE_2K;
		break;
	case SATIP_TMOD_4K:
		val=TRANSMISSION_MODE_4K;
		break;
	case SATIP_TMOD_8K:
		val=TRANSMISSION_MODE_8K;
		break;
	case SATIP_TMOD_16K:
		val=TRANSMISSION_MODE_16K;
		break;
	case SATIP_TMOD_32K:
		val=TRANSMISSION_MODE_32K;
		break;
	case SATIP_TMOD_AUTO:
		if(tuner->info.caps&SATIP_TMOD_AUTO)
		{
			val=TRANSMISSION_MODE_AUTO;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[9].cmd=DTV_TRANSMISSION_MODE;
	tuner->hwtune.prop[9].u.data=val;

	switch(tuner->tune.gi)
	{
	case SATIP_GI_14:
		val=GUARD_INTERVAL_1_4;
		break;
	case SATIP_GI_18:
		val=GUARD_INTERVAL_1_8;
		break;
	case SATIP_GI_116:
		val=GUARD_INTERVAL_1_16;
		break;
	case SATIP_GI_132:
		val=GUARD_INTERVAL_1_32;
		break;
	case SATIP_GI_1128:
		val=GUARD_INTERVAL_1_128;
		break;
	case SATIP_GI_19128:
		val=GUARD_INTERVAL_19_128;
		break;
	case SATIP_GI_19256:
		val=GUARD_INTERVAL_19_256;
		break;
	case SATIP_GI_AUTO:
		if(tuner->info.caps&SATIP_GI_AUTO)
		{
			val=GUARD_INTERVAL_AUTO;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[10].cmd=DTV_GUARD_INTERVAL;
	tuner->hwtune.prop[10].u.data=val;

	if(tuner->tune.msys==SATIP_DVBT2)
	{
		if(!(tuner->info.msys&DVBT2))return -1;

		tuner->hwtune.prop[11].cmd=DTV_STREAM_ID;
		tuner->hwtune.prop[11].u.data=(tuner->tune.plp!=SATIP_UNDEF?
			tuner->tune.plp:NO_STREAM_ID_FILTER);
		tuner->hwtune.props.num=12;
	}
	else tuner->hwtune.props.num=11;

	tuner->hwtune.seq=nosat;

	return 0;
}

static int fe_dvbc_tune(TUNER *tuner)
{
	unsigned long long freq;
	int val;
	unsigned int s;
	unsigned int l;
	unsigned int h;

	if(!(tuner->info.msys&DVBC))return -1;

	if(tuner->tune.freq<tuner->info.fmin||
		tuner->tune.freq>tuner->info.fmax||
		tuner->tune.sr<tuner->info.smin||
		tuner->tune.sr>tuner->info.smax)return -1;

	if(tuner->info.step>1)
	{
		s=(tuner->tune.freq-tuner->info.fmin)/tuner->info.step;
		l=tuner->info.fmin+s*tuner->info.step;
		h=l+tuner->info.step;
		if(tuner->tune.freq!=l)
		{
			if(tuner->tune.freq-l<h-tuner->tune.freq)freq=l;
			else freq=h;
			if(freq<tuner->info.fmin)freq=tuner->info.fmin;
			else if(freq>tuner->info.fmax)freq=tuner->info.fmax;
		}
		else freq=tuner->tune.freq;
	}
	else freq=tuner->tune.freq;

	tuner->hwtune.props.props=tuner->hwtune.prop;

	tuner->hwtune.prop[0].cmd=DTV_CLEAR;
	tuner->hwtune.prop[0].u.data=0;
	tuner->hwtune.prop[1].cmd=DTV_FREQUENCY;
	tuner->hwtune.prop[1].u.data=freq;
	tuner->hwtune.prop[2].cmd=DTV_DELIVERY_SYSTEM;
	tuner->hwtune.prop[2].u.data=SYS_DVBC_ANNEX_A;
	tuner->hwtune.prop[3].cmd=DTV_SYMBOL_RATE;
	tuner->hwtune.prop[3].u.data=tuner->tune.sr;
	tuner->hwtune.prop[4].cmd=DTV_INNER_FEC;
	tuner->hwtune.prop[4].u.data=FEC_AUTO;
	tuner->hwtune.prop[5].cmd=DTV_LNA;
	tuner->hwtune.prop[5].u.data=tuner->lna;

	switch(tuner->tune.mtype)
	{
	case SATIP_16Q:
		val=QAM_16;
		break;
	case SATIP_32Q:
		val=QAM_32;
		break;
	case SATIP_64Q:
		val=QAM_64;
		break;
	case SATIP_128Q:
		val=QAM_128;
		break;
	case SATIP_256Q:
		val=QAM_256;
		break;
	case SATIP_AUTOQ:
		if(tuner->info.caps&SATIP_AUTOQ)
		{
			val=QAM_AUTO;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[6].cmd=DTV_MODULATION;
	tuner->hwtune.prop[6].u.data=val;

	switch(tuner->tune.specinv)
	{
	case SATIP_SPI_OFF:
		val=INVERSION_OFF;
		break;
	case SATIP_SPI_ON:
		val=INVERSION_ON;
		break;
	case SATIP_SPI_AUTO:
		if(tuner->info.caps&SATIP_SPI_AUTO)
		{
			val=INVERSION_AUTO;
			break;
		}
	default:return -1;
	}
	tuner->hwtune.prop[7].cmd=DTV_INVERSION;
	tuner->hwtune.prop[7].u.data=val;

	tuner->hwtune.props.num=8;

	tuner->hwtune.seq=nosat;

	return 0;
}

static unsigned int khz90(void)
{
	unsigned long tme;
	struct timespec ts;

	if(UNLIKELY(clock_gettime(CLOCK_MONOTONIC_RAW,&ts)))return 0;
	tme=((unsigned long)ts.tv_sec)*10000UL+
		(unsigned long)(ts.tv_nsec/100000);
	return (unsigned int)(tme*9UL)+(unsigned int)(ts.tv_nsec%100000/11112);
}

static void timetick(TUNER *t,void *data)
{
	int l;
	int j;
	STREAM *s;
	SATIP_STREAM *ss;
	uint64_t dummy;

	if(UNLIKELY(read(t->tfd,&dummy,sizeof(dummy))!=sizeof(dummy)))return;

	for(s=t->slist;s;s=s->next)if(s->idle!=-1)switch(++(s->idle))
	{
	case 2:	if(LIKELY((ss=s->stream)->fill<188))
		{
			if(t->idleflush)s->streamer(s->did,NULL);
			break;
		}

		for(l=0;l<ss->fill;l++)if(LIKELY(ss->data[l]==0x47))
		{
			for(j=l+188;j<ss->fill;j+=188)
				if(UNLIKELY(ss->data[j]!=0x47))break;
			if(LIKELY(j>=ss->fill))break;
		}

		if(UNLIKELY(l))
		{
			memmove(ss->data,ss->data+l,ss->fill-l);
			ss->fill-=l;
			if(ss->fill<188)break;
		}

		for(l=188;l<=ss->fill;l+=188);
		l-=188;
		j=ss->fill-l;
		ss->fill=l;
		t->bytes+=l;

		s->streamer(s->did,ss);
		ss->flags&=~SATIP_FLGSTART;

		if(UNLIKELY(j))memmove(ss->data,ss->data+ss->fill,j);
		ss->fill=j;

		break;

	case 10:s->idle=0;
		s->stream->fill=0;
		s->stream->khz90=khz90();
		s->streamer(s->did,s->stream);
		s->stream->flags&=~SATIP_FLGSTART;
		break;
	}

	if(++t->ticks<10)return;
	t->ticks=0;

	pthread_spin_lock(&t->mtx);
	t->txbytes=t->bytes;
	t->txticks++;
	pthread_spin_unlock(&t->mtx);

	for(s=t->slist;s;s=s->next)if((s->rtcp^=1))
	{
	    switch(s->status.tune.msys)
	    {
	    case SATIP_DVBS:
	    case SATIP_DVBS2:
		s->status.lock=s->status.level=s->status.quality=0;
		pthread_spin_lock(&t->mtx);
		if(!s->tuner->lock)
		{
			pthread_spin_unlock(&t->mtx);
			break;
		}

		s->status.lock=s->tuner->lock;
		s->status.level=s->tuner->level;
		s->status.quality=s->tuner->quality;

		if(s->status.tune.mtype==SATIP_AUTOQ)
		{
			if(s->status.tune.msys==SATIP_DVBS)
				s->status.tune.mtype=SATIP_QPSK;
			else s->status.tune.mtype=s->tuner->tune.mtype;
		}

		s->status.tune.freq=s->tuner->tune.freq;
		s->status.tune.pol=s->tuner->tune.pol;

		if(s->status.tune.msys==SATIP_DVBS2)
		{
			if(s->status.tune.plts==SATIP_PLTS_AUTO)
				s->status.tune.plts=s->tuner->tune.plts;
			if(s->status.tune.ro==SATIP_ROFF_AUTO)
				s->status.tune.ro=s->tuner->tune.ro;
		}

		s->status.tune.sr=s->tuner->tune.sr;

		if(s->status.tune.fec==SATIP_FEC_AUTO)
			s->status.tune.fec=s->tuner->tune.fec;

		pthread_spin_unlock(&t->mtx);
		break;

	    case SATIP_DVBT:
	    case SATIP_DVBT2:
		s->status.lock=s->status.level=s->status.quality=0;
		pthread_spin_lock(&t->mtx);
		if(!s->tuner->lock)
		{
			pthread_spin_unlock(&t->mtx);
			break;
		}

		s->status.lock=s->tuner->lock;
		s->status.level=s->tuner->level;
		s->status.quality=s->tuner->quality;

		s->status.tune.freq=s->tuner->tune.freq;
		if(s->status.tune.bw==SATIP_BW_AUTO)
			s->status.tune.bw=s->tuner->tune.bw;
		if(s->status.tune.tmode==SATIP_TMOD_AUTO)
			s->status.tune.tmode=s->tuner->tune.tmode;
		if(s->status.tune.mtype==SATIP_AUTOQ)
			s->status.tune.mtype=s->tuner->tune.mtype;
		if(s->status.tune.gi==SATIP_GI_AUTO)
			s->status.tune.gi=s->tuner->tune.gi;
		if(s->status.tune.fec==SATIP_FEC_AUTO)
			s->status.tune.fec=s->tuner->tune.fec;

		if(s->status.tune.msys==SATIP_DVBT2)
		{
			s->status.tune.plp=s->tuner->tune.plp;
			s->status.tune.t2id=s->tuner->tune.t2id;
			if(s->status.tune.sm==SATIP_SM_AUTO)
				s->status.tune.sm=s->tuner->tune.sm;
		}

		pthread_spin_unlock(&t->mtx);
		break;

	    case SATIP_DVBC:
		s->status.lock=s->status.level=s->status.quality=0;
		pthread_spin_lock(&t->mtx);
		if(!s->tuner->lock)
		{
			pthread_spin_unlock(&t->mtx);
			break;
		}

		s->status.lock=s->tuner->lock;
		s->status.level=s->tuner->level;
		s->status.quality=s->tuner->quality;

		s->status.tune.freq=s->tuner->tune.freq;
		if(s->status.tune.mtype==SATIP_AUTOQ)
			s->status.tune.mtype=s->tuner->tune.mtype;
		s->status.tune.sr=s->tuner->tune.sr;
		if(s->status.tune.specinv==SATIP_SPI_AUTO)
			s->status.tune.specinv=s->tuner->tune.specinv;

		pthread_spin_unlock(&t->mtx);
		break;

	    case SATIP_DVBC2:
		s->status.lock=s->status.level=s->status.quality=0;
		pthread_spin_lock(&t->mtx);
		if(!s->tuner->lock)
		{
			pthread_spin_unlock(&t->mtx);
			break;
		}

		s->status.lock=s->tuner->lock;
		s->status.level=s->tuner->level;
		s->status.quality=s->tuner->quality;

		s->status.tune.freq=s->tuner->tune.freq;
		if(s->status.tune.c2tft==SATIP_TFT_AUTO)
			s->status.tune.c2tft=s->tuner->tune.c2tft;
		if(s->status.tune.bw==SATIP_BW_AUTO)
			s->status.tune.bw=s->tuner->tune.bw;
		s->status.tune.ds=s->tuner->tune.ds;
		s->status.tune.plp=s->tuner->tune.plp;

		pthread_spin_unlock(&t->mtx);
		break;

	    default:goto out;
	    }

	    s->stater(s->did,&s->status);
out:;	}
}

static void streamtrigger(TUNER *t,void *data)
{
	STREAM *s=((TIMER *)data)->stream;
	struct epoll_event e;
	struct itimerspec it;

	e.events=EPOLLIN|EPOLLONESHOT;
	e.data.ptr=s;
	if(UNLIKELY(epoll_ctl(t->epfd,EPOLL_CTL_MOD,s->fd,&e)))
	{
		it.it_interval.tv_sec=0;
		it.it_interval.tv_nsec=0;
		it.it_value.tv_sec=0;
		it.it_value.tv_nsec=1000000;
		timerfd_settime(s->tfd,0,&it,NULL);
	}
}

static void dostream(TUNER *t,void *data)
{
	int l;
	int j;
	STREAM *s=(STREAM *)data;
	SATIP_STREAM *ss=s->stream;
	struct epoll_event e;
	struct itimerspec it;
	struct iovec io[SATIP_MAX_BURST];

	for(l=0,j=s->index;l<SATIP_MAX_BURST;l++,j++)
	{
		if(j==SATIP_MAX_BURST)j=0;
		io[l].iov_base=s->queue[j].data+s->queue[j].fill;
		io[l].iov_len=sizeof(ss->data)-s->queue[j].fill;
	}

	l=readv(s->fd,io,SATIP_MAX_BURST);

	if(UNLIKELY(l>=sizeof(ss->data)*(SATIP_MAX_BURST-1)))switch(s->ramp++)
	{
	case 0:	it.it_interval.tv_sec=0;
		it.it_interval.tv_nsec=0;
		it.it_value.tv_sec=0;
		it.it_value.tv_nsec=1000000;
		timerfd_settime(s->tfd,0,&it,NULL);
		break;

	case 2:	it.it_interval.tv_sec=0;
		it.it_interval.tv_nsec=0;
		it.it_value.tv_sec=0;
		it.it_value.tv_nsec=500000;
		timerfd_settime(s->tfd,0,&it,NULL);
		break;

	case 4:	it.it_interval.tv_sec=0;
		it.it_interval.tv_nsec=0;
		it.it_value.tv_sec=0;
		it.it_value.tv_nsec=250000;
		timerfd_settime(s->tfd,0,&it,NULL);
		break;

	case 6:	s->ramp--;
		e.events=EPOLLIN|EPOLLONESHOT;
		e.data.ptr=s;
		if(UNLIKELY(epoll_ctl(t->epfd,EPOLL_CTL_MOD,s->fd,&e)))
		{
			it.it_interval.tv_sec=0;
			it.it_interval.tv_nsec=0;
			it.it_value.tv_sec=0;
			it.it_value.tv_nsec=1000000;
			timerfd_settime(s->tfd,0,&it,NULL);
		}
	}
	else
	{
		s->ramp=0;
		if(UNLIKELY(l<0))switch(errno)
		{
		case EAGAIN:
		case EINTR:
		case EOVERFLOW:
			break;
		default:return;
		}

		if(t->fast)
		{
			e.events=EPOLLIN|EPOLLONESHOT;
			e.data.ptr=s;
			if(UNLIKELY(epoll_ctl(t->epfd,EPOLL_CTL_MOD,s->fd,&e)))
			{
				it.it_interval.tv_sec=0;
				it.it_interval.tv_nsec=0;
				it.it_value.tv_sec=0;
				it.it_value.tv_nsec=1000000;
				timerfd_settime(s->tfd,0,&it,NULL);
			}
		}
		else
		{
			it.it_interval.tv_sec=0;
			it.it_interval.tv_nsec=0;
			it.it_value.tv_sec=0;
			it.it_value.tv_nsec=
				(l>=sizeof(ss->data)*(SATIP_MAX_BURST-2)?
					2000000:4000000);
			timerfd_settime(s->tfd,0,&it,NULL);
		}

		if(UNLIKELY(l<=0))return;
	}

	for(j=s->index;;)
	{
		if(l>sizeof(ss->data)-s->queue[j].fill)
		{
			l-=sizeof(ss->data)-s->queue[j].fill;
			s->queue[j].fill=sizeof(ss->data);
		}
		else
		{
			s->queue[j].fill+=l;
			break;
		}
		if(++j>=SATIP_MAX_BURST)j=0;
	}

	while(1)
	{
		for(l=0;l<ss->fill;l++)if(LIKELY(ss->data[l]==0x47))
		{
			for(j=l+188;j<ss->fill;j+=188)
				if(UNLIKELY(ss->data[j]!=0x47))break;
			if(LIKELY(j>=ss->fill))break;
		}

		if(UNLIKELY(l))for(j=0;j<SATIP_MAX_BURST;j++)s->queue[j].fill=0;

		if(LIKELY(ss->fill==sizeof(ss->data)))
		{
			t->bytes+=sizeof(ss->data);
			s->streamer(s->did,ss);
			ss->flags&=~SATIP_FLGSTART;
			ss->fill=0;
			s->idle=0;

			if(++(s->index)>=SATIP_MAX_BURST)s->index=0;
			ss=s->stream=&s->queue[s->index];
			continue;
		}
		else if(ss->fill)s->idle=0;
		break;
	}
}

static void dosection(TUNER *t,void *data)
{
	int l;
	STREAM *s=(STREAM *)data;
	SATIP_STREAM *ss=s->stream;
	struct epoll_event e;
	struct itimerspec it;
	unsigned char bfr[SECTION_BUFFER];

	l=read(s->fd,bfr,sizeof(bfr));

	if(UNLIKELY(l<0))switch(errno)
	{
	case EAGAIN:
	case EINTR:
	case EOVERFLOW:
		break;
	default:return;
	}

	e.events=EPOLLIN|EPOLLONESHOT;
	e.data.ptr=s;
	if(UNLIKELY(epoll_ctl(t->epfd,EPOLL_CTL_MOD,s->fd,&e)))
	{
		it.it_interval.tv_sec=0;
		it.it_interval.tv_nsec=0;
		it.it_value.tv_sec=0;
		it.it_value.tv_nsec=1000000;
		timerfd_settime(s->tfd,0,&it,NULL);
	}

	if(UNLIKELY(l<=0))return;

	ss->khz90=0;
	ss->flags=SATIP_FLGSECT;
	ss->fill=l;
	ss->section=bfr;
	s->streamer(s->did,ss);
}

static void *camgr(void *data)
{
	int n=4;
	int val;
	TUNER *t;
	uint64_t dummy;
	struct itimerspec ts;
	ca_caps_t caps;
	ca_slot_info_t info;
	struct pollfd p[5];
	unsigned char state[SATIP_CAM_SLOTS];
	SATIP_HW_CAM_IO msg;
	union
	{
		sigset_t set;
		unsigned char bfr[MAX_CAM_MSGLEN];
	} u;

	sigfillset(&u.set);
	pthread_sigmask(SIG_BLOCK,&u.set,NULL);

	pthread_setname_np(pthread_self(),"hw camgr");

	t=(TUNER *)data;

	p[0].fd=t->group->termfd;
	p[0].events=POLLIN;
	p[1].fd=t->ocrfd;
	p[1].events=POLLIN;
	p[2].fd=t->catfd;
	p[2].events=POLLIN;
	p[3].fd=t->carfd;
	p[3].events=POLLIN;
	p[4].events=POLLIN;
	p[4].revents=0;

	while(1)
	{
		if(poll(p,n,-1)<1)continue;

		if(p[0].revents&POLLIN)break;

		if(p[1].revents&POLLIN)
			if(read(t->ocrfd,&dummy,sizeof(dummy))==sizeof(dummy))
		{
			if(t->tunefd!=-1&&t->cafd==-1)
			{
				if((t->cafd=fe_open(t->adapter,t->ca,5))==-1);
				else if(ioctl(t->cafd,CA_GET_CAP,&caps)||
					!caps.slot_num)goto fail;
				else if(ioctl(t->cafd,CA_RESET))
				{
fail:					close(t->cafd);
					t->cafd=-1;
				}
				else
				{
					memset(state,0,sizeof(state));
					if(caps.slot_num>sizeof(state))
						caps.slot_num=sizeof(state);
					p[4].fd=t->cafd;
					n=5;
					memset(&ts,0,sizeof(ts));
					ts.it_value.tv_nsec=1;
					ts.it_interval.tv_nsec=250000000;
					timerfd_settime(t->catfd,0,&ts,NULL);
				}
			}
			else if(t->tunefd==-1&&t->cafd!=-1)
			{
				memset(&ts,0,sizeof(ts));
				timerfd_settime(t->catfd,0,&ts,NULL);
				dummy=read(t->catfd,&dummy,sizeof(dummy));
				close(t->cafd);
				t->cafd=-1;
				p[4].revents=0;
				n=4;
				for(val=0;val<caps.slot_num;val++)if(state[val])
				{
					msg.type=SATIP_CAM_STATE;
					msg.deviceid=t->feid;
					msg.slot=val;
					msg.state=0;
					msg.len=0;
					msg.data=NULL;
					pthread_mutex_lock(&t->group->ctx);
					t->group->camcb(&msg,t->group->campriv);
					pthread_mutex_unlock(&t->group->ctx);
				}
			}

			dummy=1;
			dummy=write(t->ocafd,&dummy,sizeof(dummy));
		}

		if(p[2].revents&POLLIN)
			if(read(t->catfd,&dummy,sizeof(dummy))==sizeof(dummy))
		{
			for(info.num=0;info.num<caps.slot_num;info.num++)
				if(!ioctl(t->cafd,CA_GET_SLOT_INFO,&info))
			{
				val=0;
				if(info.type&CA_CI_LINK)
				{
					if(info.flags&CA_CI_MODULE_PRESENT)
						val|=SATIP_CAM_AVAIL;
					if(info.flags&CA_CI_MODULE_READY)
						val|=SATIP_CAM_READY;
				}
				if(val!=state[info.num])
				{
					state[info.num]=val;
					msg.type=SATIP_CAM_STATE;
					msg.deviceid=t->feid;
					msg.slot=info.num;
					msg.state=val;
					msg.len=0;
					msg.data=NULL;
					pthread_mutex_lock(&t->group->ctx);
					t->group->camcb(&msg,t->group->campriv);
					pthread_mutex_unlock(&t->group->ctx);
				}
			}
		}

		if(p[3].revents&POLLIN)
			if(read(t->carfd,&dummy,sizeof(dummy))==sizeof(dummy))
		{
			t->caans=-1;
			if(t->cafd!=-1)switch(t->camsg->type)
			{
			case SATIP_CAM_RESET:
				if(ioctl(t->cafd,CA_RESET))break;
				for(val=0;val<caps.slot_num;val++)if(state[val])
				{
					msg.type=SATIP_CAM_STATE;
					msg.deviceid=t->feid;
					msg.slot=val;
					msg.state=0;
					msg.len=0;
					msg.data=NULL;
					pthread_mutex_lock(&t->group->ctx);
					t->group->camcb(&msg,t->group->campriv);
					pthread_mutex_unlock(&t->group->ctx);
				}
				memset(state,0,sizeof(state));
				memset(&ts,0,sizeof(ts));
				ts.it_value.tv_nsec=1;
				ts.it_interval.tv_sec=1;
				timerfd_settime(t->catfd,0,&ts,NULL);
				t->caans=0;
				break;

			case SATIP_CAM_WRITE:
				u.bfr[0]=(unsigned char)t->camsg->slot;
				u.bfr[1]=(unsigned char)t->camsg->tsid;
				memcpy(u.bfr+2,t->camsg->data,t->camsg->len);
				if(write(t->cafd,u.bfr,t->camsg->len+2)!=
					t->camsg->len+2)break;
				t->caans=0;
				break;
			}

			dummy=1;
			dummy=write(t->caafd,&dummy,sizeof(dummy));
		}

		if((p[4].revents&POLLIN)&&t->cafd!=-1)
			if((val=read(t->cafd,u.bfr,sizeof(u.bfr)))>2)
		{
			msg.type=SATIP_CAM_READ;
			msg.deviceid=t->feid;
			msg.slot=u.bfr[0];
			msg.tsid=u.bfr[1];
			msg.len=val-2;
			msg.data=u.bfr+2;
			pthread_mutex_lock(&t->group->ctx);
			t->group->camcb(&msg,t->group->campriv);
			pthread_mutex_unlock(&t->group->ctx);
		}
	}

	if(t->cafd!=-1)for(val=0;val<caps.slot_num;val++)if(state[val])
	{
		msg.type=SATIP_CAM_STATE;
		msg.deviceid=t->feid;
		msg.slot=val;
		msg.state=0;
		msg.len=0;
		msg.data=NULL;
		pthread_mutex_lock(&t->group->ctx);
		t->group->camcb(&msg,t->group->campriv);
		pthread_mutex_unlock(&t->group->ctx);
	}

	pthread_exit(NULL);
}

static void *streamer(void *data)
{
	union
	{
		struct
		{
			int i;
			int n;
			int f;
			union
			{
				uint64_t dummy;
				struct epoll_event e;
				STREAM **ds;
			} u;
			TUNER *t;
			struct epoll_event evt[MAX_EPOLL];
		} s;
		sigset_t set;
	} u;

	sigfillset(&u.set);
	pthread_sigmask(SIG_BLOCK,&u.set,NULL);

	pthread_setname_np(pthread_self(),"hw streamer");

	u.s.t=(TUNER *)data;
	u.s.f=0;

	while(1)
	{
		if(UNLIKELY((u.s.n=epoll_wait(u.s.t->epfd,u.s.evt,MAX_EPOLL,-1))
			<=0))continue;

		for(u.s.i=0;u.s.i<u.s.n;u.s.i++)
			if(LIKELY(u.s.evt[u.s.i].events&EPOLLIN))
			switch((long)(((FUNC *)u.s.evt[u.s.i].data.ptr)->func))

		{
		case 0:	goto out;

		case -1:if(UNLIKELY(read(u.s.t->reqfd,&u.s.u.dummy,
				sizeof(u.s.u.dummy))!=sizeof(u.s.u.dummy)))
					break;
			u.s.f=1;
			u.s.t->ans=-1;
			break;

		default:((FUNC *)u.s.evt[u.s.i].data.ptr)
				->func(u.s.t,u.s.evt[u.s.i].data.ptr);
			break;
		}

		if(u.s.f)
		{
			switch(u.s.t->req)
			{
			case 0:	for(u.s.u.ds=&u.s.t->slist;*u.s.u.ds;
					u.s.u.ds=&((*u.s.u.ds)->next))
						if(*u.s.u.ds==u.s.t->item)
				{
					*u.s.u.ds=u.s.t->item->next;
					if(u.s.t->item->status.set.numpids
					    !=SATIP_NOPIDS)
						dmx_stop(u.s.t->item->fd);
					epoll_ctl(u.s.t->epfd,EPOLL_CTL_DEL,
						u.s.t->item->fd,NULL);
					u.s.t->ans=0;
					break;
				}
				break;

			case 1:	u.s.u.e.events=EPOLLIN|EPOLLONESHOT;
				u.s.u.e.data.ptr=u.s.t->item;
				if(!epoll_ctl(u.s.t->epfd,EPOLL_CTL_ADD,
					u.s.t->item->fd,&u.s.u.e))
				{
					u.s.t->item->next=u.s.t->slist;
					u.s.t->slist=u.s.t->item;
					u.s.t->ans=0;
				}
				break;

			case 2:	epoll_ctl(u.s.t->epfd,EPOLL_CTL_DEL,
					u.s.t->item->tfd,NULL);
				u.s.t->ans=0;
				break;

			case 3:	u.s.u.e.events=EPOLLIN;
				u.s.u.e.data.ptr=u.s.t->item->timer;
				if(!epoll_ctl(u.s.t->epfd,EPOLL_CTL_ADD,
					u.s.t->item->tfd,&u.s.u.e))
						u.s.t->ans=0;
				break;
			}
			u.s.u.dummy=1;
			u.s.u.dummy=write(u.s.t->ansfd,&u.s.u.dummy,
				sizeof(u.s.u.dummy));
			u.s.f=0;
		}
	}

out:	pthread_exit(NULL);
}

static void cadev(TUNER *t)
{
	uint64_t dummy=1;
	struct pollfd p[2];

	if(!t->group->camcb)return;

	p[0].fd=t->group->termfd;
	p[0].events=POLLIN;
	p[1].fd=t->ocafd;
	p[1].events=POLLIN;

	if(write(t->ocrfd,&dummy,sizeof(dummy))!=sizeof(dummy))return;
	while(poll(p,2,-1)<1);
	if(p[0].revents&POLLIN)return;
	if(!(p[1].revents&POLLIN))return;
	dummy=read(t->ocafd,&dummy,sizeof(dummy));
}

static void *tuningmgr(void *data)
{
	int i;
	int lock=0;
	int strength=0;
	int snr=0;
	int snrticker=0;
	uint64_t dummy;
	TUNER *t=(TUNER *)data;
	STREAM *s;
	union
	{
		struct
		{
			unsigned short s;
			fe_status_t st;
			struct pollfd p[2];
			struct itimerspec ts;
			struct dvb_diseqc_slave_reply reply;
			struct dtv_properties props;
			struct dtv_property prop[18];
			struct dvb_frontend_event ev;
			HWTUNE hwtune;
		}s;
		sigset_t set;
	}u;

	sigfillset(&u.set);
	pthread_sigmask(SIG_BLOCK,&u.set,NULL);

	pthread_setname_np(pthread_self(),"hw tuningmgr");

	u.s.p[0].fd=t->group->termfd;
	u.s.p[0].events=POLLIN;
	u.s.p[1].fd=t->dsfd;
	u.s.p[1].events=POLLIN;

	while(1)
	{
		if(poll(u.s.p,2,-1)<1)continue;
		if(u.s.p[0].revents&POLLIN)break;
		if(!(u.s.p[1].revents&POLLIN))continue;

		pthread_mutex_lock(&t->dtx);

		memset(&u.s.ts,0,sizeof(u.s.ts));

		dummy=read(t->dsfd,&dummy,sizeof(dummy));

		switch(t->dstate)
		{
		case COPY:
			t->dstate=PREPARE;
			t->idle=0;
			u.s.hwtune=t->hwtune;
			lock=strength=snr=snrticker=0;
			break;
		case CANCEL:
			u.s.hwtune.seq=nosat;
			break;
		}

		switch(u.s.hwtune.seq[t->dstate])
		{
		case CANCEL:
			if(!t->idle)
			{
				u.s.ts.it_value.tv_nsec=250000000;
				u.s.ts.it_interval.tv_nsec=250000000;
				timerfd_settime(t->dsfd,0,&u.s.ts,NULL);
			}
			if(t->idle++==TUNER_IDLE_SECS*4)
			{
				t->dstate=DONE;
				close(t->tunefd);
				t->tunefd=-1;
				t->currsrc=0;
				t->idle=0;
				timerfd_settime(t->dsfd,0,&u.s.ts,NULL);
				cadev(t);
			}
		case DONE:
		case FAILED:
			break;

		case PREPARE:
			pthread_mutex_unlock(&t->dtx);
			if(ioctl(t->tunefd,FE_SET_PROPERTY,&u.s.hwtune.props))
				goto fail;
			pthread_mutex_lock(&t->dtx);
			if(t->dstate==PREPARE)t->dstate++;
			goto instant;

		case TONEOFF:
			u.s.props.num=1;
			u.s.props.props=u.s.prop;
			u.s.prop[0].cmd=DTV_TONE;
			u.s.prop[0].u.data=SEC_TONE_OFF;
			pthread_mutex_unlock(&t->dtx);
			if(ioctl(t->tunefd,FE_SET_PROPERTY,&u.s.props))
				goto fail;
			pthread_mutex_lock(&t->dtx);
			if(t->dstate<PREPARE)goto instant;
			goto timer;

		case SENDCMD:
			pthread_mutex_unlock(&t->dtx);
			if(ioctl(t->tunefd,FE_DISEQC_SEND_MASTER_CMD,
				&u.s.hwtune.dcmd))goto fail;
			pthread_mutex_lock(&t->dtx);
			if(t->dstate<PREPARE)goto instant;
			goto timer;

		case SENDRECV:
			memset(&u.s.reply,0,sizeof(u.s.reply));
			u.s.reply.timeout=t->d2wait?t->d2wait:150;
			pthread_mutex_unlock(&t->dtx);
			if(ioctl(t->tunefd,FE_DISEQC_SEND_MASTER_CMD,
				&u.s.hwtune.dcmd))goto fail;
			if(ioctl(t->tunefd,FE_DISEQC_RECV_SLAVE_REPLY,
				&u.s.reply)||u.s.reply.msg_len!=1||
				u.s.reply.msg[0]!=0xe4)goto fail;
			pthread_mutex_lock(&t->dtx);
			if(t->dstate<PREPARE)goto instant;
			goto timer;

		case TONEBURST:
			pthread_mutex_unlock(&t->dtx);
			if(ioctl(t->tunefd,FE_DISEQC_SEND_BURST,
				u.s.hwtune.dburst))goto fail;
			pthread_mutex_lock(&t->dtx);
			if(t->dstate<PREPARE)goto instant;
		case WAIT:
timer:			u.s.ts.it_value.tv_nsec=(t->dwait?t->dwait:15)*1000000;
			timerfd_settime(t->dsfd,0,&u.s.ts,NULL);
			t->dstate++;
			break;

		case TONE:
			u.s.props.num=1;
			u.s.props.props=u.s.prop;
			u.s.prop[0].cmd=DTV_TONE;
			u.s.prop[0].u.data=u.s.hwtune.dtone;
			pthread_mutex_unlock(&t->dtx);
			if(ioctl(t->tunefd,FE_SET_PROPERTY,&u.s.props))
				goto fail;
			pthread_mutex_lock(&t->dtx);
			if(t->dstate>=PREPARE)t->dstate++;
			goto instant;

		case TUNE:
			u.s.props.num=1;
			u.s.props.props=u.s.prop;
			u.s.prop[0].cmd=DTV_TUNE;
			u.s.prop[0].u.data=0;
			pthread_mutex_unlock(&t->dtx);
			if(ioctl(t->tunefd,FE_SET_PROPERTY,&u.s.props))
			{
fail:				pthread_mutex_lock(&t->dtx);
				t->currsrc=0;
				t->dstate=FAILED;
				break;
			}
			pthread_mutex_lock(&t->dtx);
			if(t->dstate<PREPARE)goto instant;

			for(s=t->slist;s;s=s->next)switch(s->status.set.numpids)
			{
			case SATIP_SECTION:
				if(s->status.set.filterraw)
					dmx_set_secfilter(s->fd,
						s->status.set.pid,
						s->status.set.filtercrc,
						s->status.set.filter,
						s->status.set.mask,
						s->status.set.mode);
				else dmx_set_section(s->fd,s->status.set.pid,
					s->status.set.table,
					s->status.set.filtercurrent,
					s->status.set.filterversion,
					s->status.set.version);
				dmx_start(s->fd);
				break;

			case SATIP_ALLPIDS:
				dmx_set_filter(s->fd,-1);
				dmx_start(s->fd);
			case SATIP_NOPIDS:
				break;

			default:dmx_set_filter(s->fd,s->status.set.pids[0]);
				for(i=1;i<s->status.set.numpids;i++)
					dmx_add_filter(s->fd,
						s->status.set.pids[i]);
				dmx_start(s->fd);
				break;
			}

			if(!t->dforce)t->currsrc=u.s.hwtune.dsrc;
			t->tuning=0;
			t->dstate++;

instant:		u.s.ts.it_value.tv_nsec=1;
			timerfd_settime(t->dsfd,0,&u.s.ts,NULL);
			break;

		case LOCKWAIT:
			pthread_mutex_unlock(&t->dtx);
			i=ioctl(t->tunefd,FE_GET_EVENT,&u.s.ev);
			pthread_mutex_lock(&t->dtx);
			if(t->dstate<PREPARE)goto instant;

			if(!i&&u.s.ev.status&FE_HAS_LOCK)
			{
				t->dstate++;
				u.s.ts.it_value.tv_nsec=1;
				u.s.ts.it_interval.tv_nsec=250000000;
			}
			else u.s.ts.it_value.tv_nsec=100000000;
			timerfd_settime(t->dsfd,0,&u.s.ts,NULL);
			break;

		case STATUS:
			pthread_mutex_unlock(&t->dtx);
			i=ioctl(t->tunefd,FE_READ_STATUS,&u.s.st);
			pthread_mutex_lock(&t->dtx);
			if(t->dstate<PREPARE)goto instant;

			if(i)lock=0;
			else if(u.s.st&(FE_TIMEDOUT|FE_REINIT))lock=0;
			else lock=(u.s.st&FE_HAS_LOCK)?1:0;

			if(lock)
			{
				pthread_mutex_unlock(&t->dtx);
				i=ioctl(t->tunefd,FE_READ_SIGNAL_STRENGTH,
					&u.s.s);
				pthread_mutex_lock(&t->dtx);
				if(t->dstate<PREPARE)goto instant;
				if(i)strength=0;
				else strength=u.s.s;
				strength=(strength*255)/0xff00;

				if(!snr||!(++snrticker&t->snrspeed))
				{
					pthread_mutex_unlock(&t->dtx);
					i=ioctl(t->tunefd,FE_READ_SNR,&u.s.s);
					pthread_mutex_lock(&t->dtx);
					if(t->dstate<PREPARE)goto instant;

					if(i)snr=0;
					else snr=(u.s.s<<t->snrshift)|
						((1<<t->snrshift)-1);
					snr=(snr*15)/0xff00;
				}
			}
			else strength=snr=snrticker=0;

			u.s.props.num=18;
			u.s.props.props=u.s.prop;
			u.s.prop[0].cmd=DTV_DELIVERY_SYSTEM;
			u.s.prop[1].cmd=DTV_FREQUENCY;
			u.s.prop[2].cmd=DTV_MODULATION;
			u.s.prop[3].cmd=DTV_BANDWIDTH_HZ;
			u.s.prop[4].cmd=DTV_INVERSION;
			u.s.prop[5].cmd=DTV_SYMBOL_RATE;
			u.s.prop[6].cmd=DTV_INNER_FEC;
			u.s.prop[7].cmd=DTV_VOLTAGE;
			u.s.prop[8].cmd=DTV_PILOT;
			u.s.prop[9].cmd=DTV_ROLLOFF;
			u.s.prop[10].cmd=DTV_TRANSMISSION_MODE;
			u.s.prop[11].cmd=DTV_CODE_RATE_HP;
			u.s.prop[12].cmd=DTV_GUARD_INTERVAL;
			u.s.prop[13].cmd=DTV_STREAM_ID;
			u.s.prop[14].cmd=DTV_STREAM_ID;
			u.s.prop[15].cmd=DTV_TONE;
			u.s.prop[16].cmd=DTV_CODE_RATE_LP;
			u.s.prop[17].cmd=DTV_HIERARCHY;

			pthread_mutex_unlock(&t->dtx);
			i=ioctl(t->tunefd,FE_GET_PROPERTY,&u.s.props);
			pthread_mutex_lock(&t->dtx);
			if(t->dstate<PREPARE)goto instant;

			if(i)
			{
				pthread_spin_lock(&t->mtx);
				t->lock=lock;
				t->level=strength;
				t->quality=snr;
				pthread_spin_unlock(&t->mtx);
				break;
			}

			pthread_spin_lock(&t->mtx);

			t->lock=lock;
			t->level=strength;
			t->quality=snr;

			switch(u.s.prop[0].u.data)
			{
			case SYS_DVBS2:
			case SYS_DVBS:
				if(u.s.prop[0].u.data==SYS_DVBS2)
					t->tune.msys=SATIP_DVBS2;
				else t->tune.msys=SATIP_DVBS;
				t->tune.freq=u.s.prop[1].u.data;
				t->tune.freq*=1000;
				if(u.s.prop[15].u.data==SEC_TONE_ON)
					t->tune.freq+=10600000000ULL;
				else t->tune.freq+=9750000000ULL;
				if(u.s.prop[7].u.data==SEC_VOLTAGE_18)
					t->tune.pol=SATIP_POL_H;
				else t->tune.pol=SATIP_POL_V;
				switch(u.s.prop[2].u.data)
				{
				case QPSK:
					t->tune.mtype=SATIP_QPSK;
					break;
				case PSK_8:
					t->tune.mtype=SATIP_8PSK;
					break;
				case APSK_16:
					t->tune.mtype=SATIP_16APSK;
					break;
				case APSK_32:
					t->tune.mtype=SATIP_32APSK;
					break;
				default:t->tune.mtype=SATIP_AUTOQ;
					break;
				}
				switch(u.s.prop[8].u.data)
				{
				case PILOT_OFF:
					t->tune.plts=SATIP_PLTS_OFF;
					break;
				case PILOT_ON:
					t->tune.plts=SATIP_PLTS_ON;
					break;
				default:t->tune.plts=SATIP_PLTS_AUTO;
					break;
				}
				switch(u.s.prop[9].u.data)
				{
				case ROLLOFF_35:
					t->tune.ro=SATIP_ROFF_035;
					break;
				case ROLLOFF_25:
					t->tune.ro=SATIP_ROFF_025;
					break;
				case ROLLOFF_20:
					t->tune.ro=SATIP_ROFF_020;
					break;
				default:t->tune.ro=SATIP_ROFF_AUTO;
					break;
				}
				t->tune.sr=u.s.prop[5].u.data;
				switch(u.s.prop[6].u.data)
				{
				case FEC_1_2:
					t->tune.fec=SATIP_FEC_12;
					break;
				case FEC_2_3:
					t->tune.fec=SATIP_FEC_23;
					break;
				case FEC_3_4:
					t->tune.fec=SATIP_FEC_34;
					break;
				case FEC_3_5:
					t->tune.fec=SATIP_FEC_35;
					break;
				case FEC_4_5:
					t->tune.fec=SATIP_FEC_45;
					break;
				case FEC_5_6:
					t->tune.fec=SATIP_FEC_56;
					break;
				case FEC_7_8:
					t->tune.fec=SATIP_FEC_78;
					break;
				case FEC_8_9:
					t->tune.fec=SATIP_FEC_89;
					break;
				case FEC_9_10:
					t->tune.fec=SATIP_FEC_910;
					break;
				default:t->tune.fec=SATIP_FEC_AUTO;
					break;
				}
				break;

			case SYS_DVBT2:
			case SYS_DVBT:
				if(u.s.prop[0].u.data==SYS_DVBT2)
					t->tune.msys=SATIP_DVBT2;
				else t->tune.msys=SATIP_DVBT;
				t->tune.freq=u.s.prop[1].u.data;
				switch(u.s.prop[3].u.data)
				{
				case 1712000:
					t->tune.bw=SATIP_BW_1712;
					break;
				case 5000000:
					t->tune.bw=SATIP_BW_5;
					break;
				case 6000000:
					t->tune.bw=SATIP_BW_6;
					break;
				case 7000000:
					t->tune.bw=SATIP_BW_7;
					break;
				case 8000000:
					t->tune.bw=SATIP_BW_8;
					break;
				case 10000000:
					t->tune.bw=SATIP_BW_10;
					break;
				default:t->tune.bw=SATIP_BW_AUTO;
					break;
				}
				switch(u.s.prop[10].u.data)
				{
				case TRANSMISSION_MODE_1K:
					t->tune.tmode=SATIP_TMOD_1K;
					break;
				case TRANSMISSION_MODE_2K:
					t->tune.tmode=SATIP_TMOD_2K;
					break;
				case TRANSMISSION_MODE_4K:
					t->tune.tmode=SATIP_TMOD_4K;
					break;
				case TRANSMISSION_MODE_8K:
					t->tune.tmode=SATIP_TMOD_8K;
					break;
				case TRANSMISSION_MODE_16K:
					t->tune.tmode=SATIP_TMOD_16K;
					break;
				case TRANSMISSION_MODE_32K:
					t->tune.tmode=SATIP_TMOD_32K;
					break;
				default:t->tune.tmode=SATIP_TMOD_AUTO;
					break;
				}
				switch(u.s.prop[2].u.data)
				{
				case QPSK:
					t->tune.mtype=SATIP_QPSK;
					break;
				case QAM_16:
					t->tune.mtype=SATIP_16Q;
					break;
				case QAM_64:
					t->tune.mtype=SATIP_64Q;
					break;
				case QAM_256:
					t->tune.mtype=SATIP_256Q;
					break;
				default:t->tune.mtype=SATIP_AUTOQ;
					break;
				}
				switch(u.s.prop[12].u.data)
				{
				case GUARD_INTERVAL_1_4:
					t->tune.gi=SATIP_GI_14;
					break;
				case GUARD_INTERVAL_1_8:
					t->tune.gi=SATIP_GI_18;
					break;
				case GUARD_INTERVAL_1_16:
					t->tune.gi=SATIP_GI_116;
					break;
				case GUARD_INTERVAL_1_32:
					t->tune.gi=SATIP_GI_132;
					break;
				case GUARD_INTERVAL_1_128:
					t->tune.gi=SATIP_GI_1128;
					break;
				case GUARD_INTERVAL_19_128:
					t->tune.gi=SATIP_GI_19128;
					break;
				case GUARD_INTERVAL_19_256:
					t->tune.gi=SATIP_GI_19256;
					break;
				default:t->tune.gi=SATIP_GI_AUTO;
					break;
				}
				switch(u.s.prop[11].u.data)
				{
				case FEC_1_2:
					t->tune.fec=SATIP_FEC_12;
					break;
				case FEC_2_3:
					t->tune.fec=SATIP_FEC_23;
					break;
				case FEC_3_4:
					t->tune.fec=SATIP_FEC_34;
					break;
				case FEC_3_5:
					t->tune.fec=SATIP_FEC_35;
					break;
				case FEC_4_5:
					t->tune.fec=SATIP_FEC_45;
					break;
				case FEC_5_6:
					t->tune.fec=SATIP_FEC_56;
					break;
				case FEC_7_8:
					t->tune.fec=SATIP_FEC_78;
					break;
				default:t->tune.fec=SATIP_FEC_AUTO;
					break;
				}
				switch(u.s.prop[16].u.data)
				{
				case FEC_1_2:
					t->tune.feclp=SATIP_FEC_12;
					break;
				case FEC_2_3:
					t->tune.feclp=SATIP_FEC_23;
					break;
				case FEC_3_4:
					t->tune.feclp=SATIP_FEC_34;
					break;
				case FEC_3_5:
					t->tune.feclp=SATIP_FEC_35;
					break;
				case FEC_4_5:
					t->tune.feclp=SATIP_FEC_45;
					break;
				case FEC_5_6:
					t->tune.feclp=SATIP_FEC_56;
					break;
				case FEC_7_8:
					t->tune.feclp=SATIP_FEC_78;
					break;
				default:t->tune.feclp=SATIP_FEC_AUTO;
					break;
				}
				switch(u.s.prop[17].u.data)
				{
				case HIERARCHY_NONE:
					t->tune.hier=SATIP_HIER_NONE;
					break;
				case HIERARCHY_1:
					t->tune.hier=SATIP_HIER_1;
					break;
				case HIERARCHY_2:
					t->tune.hier=SATIP_HIER_2;
					break;
				case HIERARCHY_4:
					t->tune.hier=SATIP_HIER_4;
					break;
				default:t->tune.hier=SATIP_HIER_AUTO;
					break;
				}
				if(u.s.prop[0].u.data==SYS_DVBT2)
				{
					if(u.s.prop[13].u.data>=0&&
						u.s.prop[13].u.data<=255)
							t->tune.plp=
							    u.s.prop[13].u.data;
					else t->tune.plp=SATIP_UNDEF;
				}
				break;

			case SYS_DVBC_ANNEX_A:
				t->tune.freq=u.s.prop[1].u.data;
				t->tune.msys=SATIP_DVBC;
				switch(u.s.prop[2].u.data)
				{
				case QAM_16:
					t->tune.mtype=SATIP_16Q;
					break;
				case QAM_32:
					t->tune.mtype=SATIP_32Q;
					break;
				case QAM_64:
					t->tune.mtype=SATIP_64Q;
					break;
				case QAM_128:
					t->tune.mtype=SATIP_128Q;
					break;
				case QAM_256:
					t->tune.mtype=SATIP_256Q;
					break;
				default:t->tune.mtype=SATIP_AUTOQ;
					break;
				}
				t->tune.sr=u.s.prop[5].u.data;
				switch(u.s.prop[4].u.data)
				{
				case INVERSION_OFF:
					t->tune.specinv=SATIP_SPI_OFF;
					break;
				case INVERSION_ON:
					t->tune.specinv=SATIP_SPI_ON;
					break;
				default:t->tune.specinv=SATIP_SPI_AUTO;
					break;
				}
				break;
			}

			pthread_spin_unlock(&t->mtx);
			break;
		}

		pthread_mutex_unlock(&t->dtx);
	}

	pthread_exit(NULL);
}

static int streamcmd(TUNER *t,STREAM *s,int cmd)
{
	uint64_t dummy=1;
	struct pollfd p[2];

	p[0].fd=t->group->termfd;
	p[0].events=POLLIN;
	p[1].fd=t->ansfd;
	p[1].events=POLLIN;
	t->item=s;
	t->req=cmd;
	dummy=write(t->reqfd,&dummy,sizeof(dummy));
	while(1)if(poll(p,2,-1)>0)break;
	if(p[0].revents&POLLIN)return -1;
	if(!(p[1].revents&POLLIN))return -1;
	if(read(t->ansfd,&dummy,sizeof(dummy))!=sizeof(dummy))return -1;
	return t->ans;
}

int satip_hw_cam_io(void *handle,SATIP_HW_CAM_IO *msg)
{
	int r=SATIP_SYSFAIL;
	uint64_t dummy=1;
	GROUP *g=(GROUP *)handle;
	TUNER *t;
	struct pollfd p[2];

	pthread_mutex_lock(&g->stx);

	if(!g||!g->camcb||!msg)goto err;

	for(t=g->tlist;t;t=t->next)if(t->feid==msg->deviceid)break;
	if(!t)goto err;
	if(pthread_equal(t->camgr,pthread_self()))goto err;

	if(msg->slot<0||msg->slot>=SATIP_CAM_SLOTS)goto err;
	if(msg->tsid<0||msg->tsid>255)goto err;
	if(msg->len<=0||msg->len>=MAX_CAM_MSGLEN-2)goto err;
	if(!msg->data)goto err;

	p[0].fd=g->termfd;
	p[0].events=POLLIN;
	p[1].fd=t->caafd;
	p[1].events=POLLIN;

	t->caans=-1;
	t->camsg=msg;
	if(write(t->carfd,&dummy,sizeof(dummy))!=sizeof(dummy))goto err;
	while(poll(p,2,-1)<1);
	if(p[0].revents&POLLIN)goto err;
	if(!(p[1].revents&POLLIN))goto err;
	if(read(t->caafd,&dummy,sizeof(dummy))!=sizeof(dummy))goto err;
	if(!t->caans)r=0;

err:	pthread_mutex_unlock(&g->stx);
	return r;
}

int satip_hw_play(void *handle,SATIP_STRDATA *params,
	void (*stream)(void *id,SATIP_STREAM *stream),
	void (*status)(void *id,SATIP_STATUS *status),
	void *user,int access)
{
	int i;
	GROUP *g=(GROUP *)handle;
	STREAM *s=NULL;
	TUNER *x=NULL;
	TUNER *t;
	struct itimerspec ts;

	if(!handle||!params||!stream||!status||
		!params->tune||!params->set)return SATIP_SYSFAIL;

	switch(access)
	{
	case SATIP_HW_REMOTE:
	case SATIP_HW_LOCAL:
			break;
	default:return SATIP_SYSFAIL;
	}

	pthread_mutex_lock(&g->stx);

	if(access==SATIP_HW_REMOTE&&g->limit&&g->streams>=g->limit)
	{
		pthread_mutex_unlock(&g->stx);
		return SATIP_STRLIMIT;
	}

	for(t=g->tlist;t;t=t->next)t->busy=0;

retry:	for(t=g->tlist;t;t=t->next)if(!t->busy)
	{
		if(params->tune->fe)
		{
			if(params->tune->fe!=t->feid)continue;
		}
		else if(t->explicit)continue;

		if(t->access>access)continue;

		switch(params->tune->msys)
		{
		case SATIP_DVBS:
		case SATIP_DVBS2:
			if(!(t->info.msys&(DVBS|DVBS2)))continue;

			for(i=0;i<t->srcnum;i++)if(params->tune->src==t->src[i])
				goto check;
			continue;

check:			if(!t->use)
			{
				if(!x)x=t;
				else if(x->prefer>t->prefer)x=t;
				continue;
			}
			else if(t->noshare&&access!=SATIP_HW_LOCAL)continue;

			if(t->initial.freq!=params->tune->freq)continue;
			if(t->initial.pol!=params->tune->pol)continue;

			x=t;
			pthread_mutex_lock(&x->dtx);
			goto share;

		case SATIP_DVBT:
		case SATIP_DVBT2:
			if(!(t->info.msys&(DVBT|DVBT2)))continue;

			if(!t->use)
			{
				if(!x)x=t;
				else if(x->prefer>t->prefer)x=t;
				continue;
			}
			else if(t->noshare&&access!=SATIP_HW_LOCAL)continue;

			if(t->initial.freq!=params->tune->freq)continue;

			x=t;
			pthread_mutex_lock(&x->dtx);
			goto share;

		case SATIP_DVBC:
			if(!(t->info.msys&DVBC))continue;

			if(!t->use)
			{
				if(!x)x=t;
				else if(x->prefer>t->prefer)x=t;
				continue;
			}
			else if(t->noshare&&access!=SATIP_HW_LOCAL)continue;

			if(t->initial.freq!=params->tune->freq)continue;

			x=t;
			pthread_mutex_lock(&x->dtx);
			goto share;

		case SATIP_DVBC2:
			if(!(t->info.msys&DVBC2))continue;

			if(!t->use)
			{
				if(!x)x=t;
				else if(x->prefer>t->prefer)x=t;
				continue;
			}
			else if(t->noshare&&access!=SATIP_HW_LOCAL)continue;

			if(t->initial.freq!=params->tune->freq)continue;

			x=t;
			pthread_mutex_lock(&x->dtx);
			goto share;

		default:continue;
		}
	}

	if(!x)
	{
		pthread_mutex_unlock(&g->stx);
		return SATIP_DEVLIMIT;
	}

	pthread_mutex_lock(&x->dtx);

	if(x->tunefd==-1)
	{
		if((x->tunefd=fe_open(x->adapter,x->frontend,1))==-1)
		{
			pthread_mutex_unlock(&x->dtx);
			x->busy=1;
			x=NULL;
			goto retry;
		}
		x->initial=*(params->tune);
		x->tune=*(params->tune);
		x->lock=0;
		x->level=0;
		x->quality=0;
		x->currsrc=0;
		x->idle=0;
		cadev(x);
	}
	else
	{
		x->initial=*(params->tune);
		x->tune=*(params->tune);
		x->lock=0;
		x->level=0;
		x->quality=0;
	}

	switch(params->tune->msys)
	{
	case SATIP_DVBS:
	case SATIP_DVBS2:
		i=fe_dvbs2_tune(x);
		break;

	case SATIP_DVBT:
	case SATIP_DVBT2:
		i=fe_dvbt2_tune(x);
		break;

	case SATIP_DVBC:
		i=fe_dvbc_tune(x);
		break;

	default:i=-1;
		break;
	}

	if(i)
	{
		pthread_mutex_unlock(&x->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_TUNERERR;
	}
	else
	{
		x->dstate=COPY;
		x->tuning=1;
		memset(&ts,0,sizeof(ts));
		ts.it_value.tv_nsec=1;
		timerfd_settime(x->dsfd,0,&ts,NULL);
	}

share:	if(!(s=malloc(sizeof(STREAM)+sizeof(TIMER))))
	{
		pthread_mutex_unlock(&x->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_SYSFAIL;
	}

	memset(s,0,sizeof(STREAM));
	s->did=params->handle;
	s->tuner=x;
	s->status.tune=*(params->tune);
	s->status.tune.fe=s->tuner->feid;
	s->status.set.numpids=SATIP_NOPIDS;
	s->streamer=stream;
	s->stater=status;
	s->stream=s->queue;
	s->stream->flags|=SATIP_FLGSTART;
	s->func=(params->set->numpids==SATIP_SECTION?dosection:dostream);
	s->idle=(params->set->numpids==SATIP_SECTION?-1:0);
	s->timer=(TIMER *)&s[1];
	s->timer->stream=s;
	s->timer->func=streamtrigger;

	if((s->tfd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))
		==-1)
	{
		free(s);
		pthread_mutex_unlock(&x->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_SYSFAIL;
	}

	if((s->fd=dmx_open(x->adapter,x->demux,
	  params->set->numpids==SATIP_SECTION?SECTION_BUFFER:DEMUX_BUFFER))==-1)
	{
		close(s->tfd);
		free(s);
		pthread_mutex_unlock(&x->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_SYSFAIL;
	}

	switch(params->set->numpids)
	{
	default:memcpy(s->status.set.pids,params->set->pids,
		params->set->numpids*sizeof(int));
	case SATIP_NOPIDS:
	case SATIP_ALLPIDS:
		s->status.set.numpids=params->set->numpids;
		break;
	case SATIP_SECTION:
		s->status.set.numpids=params->set->numpids;
		s->status.set.pid=params->set->pid;
		s->status.set.table=params->set->table;
		s->status.set.extra=params->set->extra;
		break;
	}

	if(streamcmd(x,s,3))
	{
		close(s->tfd);
		close(s->fd);
		free(s);
		pthread_mutex_unlock(&x->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_SYSFAIL;
	}

	if(streamcmd(x,s,1))
	{
		streamcmd(x,s,2);
		close(s->tfd);
		close(s->fd);
		free(s);
		pthread_mutex_unlock(&x->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_SYSFAIL;
	}

	params->tune->fe=s->tuner->feid;
	g->streams++;
	if(!x->use++)
	{
		x->ticks=0;
		ts.it_value.tv_sec=0;
		ts.it_value.tv_nsec=1;
		ts.it_interval.tv_sec=0;
		ts.it_interval.tv_nsec=10000000;
		timerfd_settime(x->tfd,0,&ts,NULL);
	}

	switch(params->set->numpids)
	{
	case SATIP_SECTION:
		if(s->status.set.filterraw)dmx_set_secfilter(s->fd,
			s->status.set.pid,s->status.set.filtercrc,
			s->status.set.filter,s->status.set.mask,
			s->status.set.mode);
		else dmx_set_section(s->fd,s->status.set.pid,
			s->status.set.table,s->status.set.filtercurrent,
			s->status.set.filterversion,s->status.set.version);
		if(!x->tuning)dmx_start(s->fd);
		break;

	case SATIP_ALLPIDS:
		dmx_set_filter(s->fd,-1);
		if(!x->tuning)dmx_start(s->fd);
	case SATIP_NOPIDS:
		break;

	default:dmx_set_filter(s->fd,params->set->pids[0]);
		for(i=1;i<params->set->numpids;i++)
			dmx_add_filter(s->fd,params->set->pids[i]);
		if(!x->tuning)dmx_start(s->fd);
		break;
	}

	pthread_mutex_unlock(&x->dtx);
	pthread_mutex_unlock(&g->stx);

	params->handle=s;

	return 0;
}

int satip_hw_end(void *handle,SATIP_STRDATA *params)
{
	GROUP *g=(GROUP *)handle;
	STREAM *s=(STREAM *)params->handle;
	struct itimerspec ts;

	if(!g||!params||!params->handle)return SATIP_SYSFAIL;
	s=(STREAM *)params->handle;

	pthread_mutex_lock(&g->stx);
	pthread_mutex_lock(&s->tuner->dtx);

	streamcmd(s->tuner,s,2);

	if(streamcmd(s->tuner,s,0))
	{
		pthread_mutex_unlock(&s->tuner->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_SYSFAIL;
	}

	close(s->tfd);
	close(s->fd);
	g->streams--;

	if(!--s->tuner->use)
	{
		memset(&ts,0,sizeof(ts));
		timerfd_settime(s->tuner->tfd,0,&ts,NULL);
	}

	if(!s->tuner->use)
	{
		switch(s->tuner->dstate)
		{
		case DONE:
		case CANCEL:
			break;
		default:s->tuner->dstate=CANCEL;
			memset(&ts,0,sizeof(ts));
			ts.it_value.tv_nsec=1;
			timerfd_settime(s->tuner->dsfd,0,&ts,NULL);
			break;
		}
	}

	pthread_mutex_unlock(&s->tuner->dtx);
	pthread_mutex_unlock(&g->stx);

	free(s);

	return 0;
}

int satip_hw_setpids(void *handle,SATIP_STRDATA *params)
{
	int i;
	int j;
	GROUP *g=(GROUP *)handle;
	STREAM *s;
	uint64_t dummy;
	struct pollfd p;
	struct itimerspec it;
	char unused[4096];

	if(!g||!params||!params->set||!params->handle)return SATIP_SYSFAIL;

	s=(STREAM *)params->handle;

	if(s->status.set.numpids==SATIP_SECTION)
		if(params->set->numpids!=SATIP_SECTION)return SATIP_SYSFAIL;

	pthread_mutex_lock(&g->stx);
	pthread_mutex_lock(&s->tuner->dtx);

	if(s->status.set.numpids==SATIP_SECTION)
	{
		if(!s->tuner->tuning)dmx_stop(s->fd);
		s->status.set.numpids=params->set->numpids;
		s->status.set.pid=params->set->pid;
		s->status.set.table=params->set->table;
		s->status.set.extra=params->set->extra;
		if(s->status.set.filterraw)dmx_set_secfilter(s->fd,
			s->status.set.pid,s->status.set.filtercrc,
			s->status.set.filter,s->status.set.mask,
			s->status.set.mode);
		else dmx_set_section(s->fd,s->status.set.pid,
			s->status.set.table,s->status.set.filtercurrent,
			s->status.set.filterversion,s->status.set.version);
		if(!s->tuner->tuning)dmx_start(s->fd);

		pthread_mutex_unlock(&s->tuner->dtx);
		pthread_mutex_unlock(&g->stx);
		return 0;
	}

	if(s->status.set.numpids>0&&params->set->numpids>0&&!s->tuner->tuning)
	{
		for(i=0,j=0;i<s->status.set.numpids||j<params->set->numpids;)
			if(i<s->status.set.numpids&&j<params->set->numpids)
		{
			if(s->status.set.pids[i]<params->set->pids[j])
				dmx_del_filter(s->fd,s->status.set.pids[i++]);
			else if(params->set->pids[j]<s->status.set.pids[i])
				dmx_add_filter(s->fd,params->set->pids[j++]);
			else
			{
				i++;
				j++;
			}
		}
		else if(i<s->status.set.numpids)
			dmx_del_filter(s->fd,s->status.set.pids[i++]);
		else if(j<params->set->numpids)
			dmx_add_filter(s->fd,params->set->pids[j++]);

		memcpy(s->status.set.pids,params->set->pids,
			params->set->numpids*sizeof(int));
		s->status.set.numpids=params->set->numpids;

		pthread_mutex_unlock(&s->tuner->dtx);
		pthread_mutex_unlock(&g->stx);
		return 0;
	}

	streamcmd(s->tuner,s,2);

	if(streamcmd(s->tuner,s,0))
	{
		pthread_mutex_unlock(&s->tuner->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_SYSFAIL;
	}
	else pthread_mutex_unlock(&s->tuner->dtx);

	memset(&it,0,sizeof(it));
	timerfd_settime(s->tfd,0,&it,NULL);
	dummy=read(s->tfd,&dummy,sizeof(dummy));

	p.events=POLLIN;
	p.fd=s->fd;
	if(poll(&p,1,0)==1)if(p.revents&POLLIN)
	{
		if((i=open("/dev/null",O_WRONLY|O_NONBLOCK|O_CLOEXEC))!=-1)
		{
			splice(s->fd,NULL,i,NULL,DEMUX_BUFFER,
				SPLICE_F_NONBLOCK);
			close(i);
		}
		else do
		{
			i=read(s->fd,unused,sizeof(unused));
			if(poll(&p,1,0)!=1)break;
		} while(p.revents&POLLIN);
	}
	s->stream->flags|=SATIP_FLGSTART;
	s->stream->fill=0;
	s->idle=0;

	switch(params->set->numpids)
	{
	default:memcpy(s->status.set.pids,params->set->pids,
		params->set->numpids*sizeof(int));
	case SATIP_ALLPIDS:
	case SATIP_NOPIDS:
		s->status.set.numpids=params->set->numpids;
		break;
	}

	pthread_mutex_lock(&s->tuner->dtx);

	if(streamcmd(s->tuner,s,3))
	{
		pthread_mutex_unlock(&s->tuner->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_SYSFAIL;
	}

	if(streamcmd(s->tuner,s,1))
	{
		streamcmd(s->tuner,s,2);
		pthread_mutex_unlock(&s->tuner->dtx);
		pthread_mutex_unlock(&g->stx);
		return SATIP_SYSFAIL;
	}

	switch(params->set->numpids)
	{
	case SATIP_ALLPIDS:
		dmx_set_filter(s->fd,-1);
		if(!s->tuner->tuning)dmx_start(s->fd);
	case SATIP_NOPIDS:
		break;

	default:dmx_set_filter(s->fd,params->set->pids[0]);
		for(i=1;i<params->set->numpids;i++)
			dmx_add_filter(s->fd,params->set->pids[i]);
		if(!s->tuner->tuning)dmx_start(s->fd);
		break;
	}

	pthread_mutex_unlock(&s->tuner->dtx);

	pthread_mutex_unlock(&g->stx);

	return 0;
}

int satip_hw_access(void *handle,int deviceid,int access)
{
	GROUP *g=(GROUP *)handle;
	TUNER *t;

	if(!g)return SATIP_SYSFAIL;

	switch(access)
	{
	case SATIP_HW_REMOTE:
	case SATIP_HW_LOCAL:
			break;
	default:return SATIP_SYSFAIL;
	}

	pthread_mutex_lock(&g->stx);

	for(t=g->tlist;t;t=t->next)if(t->feid==deviceid)
	{
		t->access=access;
		break;
	}

	pthread_mutex_unlock(&g->stx);

	return t?0:SATIP_SYSFAIL;
}

int satip_hw_info(void *handle,int deviceid,SATIP_HW_STATUS *s)
{
	GROUP *g=(GROUP *)handle;
	TUNER *t;

	if(!g||!s)return SATIP_SYSFAIL;

	pthread_mutex_lock(&g->stx);

	for(t=g->tlist;t;t=t->next)if(t->feid==deviceid)
	{
		s->caps=t->info.caps;
		s->access=t->access;

		switch(t->info.msys)
		{
		case DVBS2:
		case DVBS|DVBS2:
			s->msys=SATIP_DVBS2;
			break;
		case DVBS:
			s->msys=SATIP_DVBS;
			break;
		case DVBT2:
		case DVBT|DVBT2:
			s->msys=SATIP_DVBT2;
			break;
		case DVBT:
			s->msys=SATIP_DVBT;
			break;
		case DVBC2:
		case DVBC|DVBC2:
			s->msys=SATIP_DVBC2;
			break;
		case DVBC:
			s->msys=SATIP_DVBC;
			break;
		default:s->msys=SATIP_UNSPEC;
		}

		pthread_mutex_lock(&t->dtx);
		if(t->tunefd==-1)
		{
			pthread_mutex_unlock(&t->dtx);
			s->open=s->streams=s->lock=s->level=s->quality=0;
			memset(&s->tune,0,sizeof(SATIP_TUNE));
			pthread_spin_lock(&t->mtx);
			s->streambytes=t->txbytes;
			s->byteupdates=t->txticks;
			pthread_spin_unlock(&t->mtx);
		}
		else
		{
			pthread_mutex_unlock(&t->dtx);
			s->open=1;
			s->streams=t->use;
			s->lock=s->level=s->quality=0;
			pthread_spin_lock(&t->mtx);
			s->tune=t->tune;
			s->streambytes=t->txbytes;
			s->byteupdates=t->txticks;
			if(t->use)
			{
				s->lock=t->lock;
				s->level=t->level;
				s->quality=t->quality;
			}
			pthread_spin_unlock(&t->mtx);
		}

		s->groupstreams=t->group->streams;
		break;
	}

	pthread_mutex_unlock(&g->stx);

	return t?0:SATIP_SYSFAIL;
}

int satip_hw_add(void *handle,SATIP_HW_TUNERCFG *tuner,SATIP_CFGINFO *info)
{
	int i;
	int j;
	int inversion;
	int lna;
	GROUP *g=(GROUP *)handle;
	TUNER *t;
	struct epoll_event e;
	static FUNC ttick={NULL,timetick};
	static FUNC tterm={NULL,(void *)0};
	static FUNC twork={NULL,(void *)-1};

	if(!g||!tuner||!info)goto err1;

	if(tuner->deviceid<1||tuner->deviceid>65535)goto err1;
	if(tuner->adapter<0||tuner->adapter>255)goto err1;
	if(tuner->frontend<0||tuner->frontend>255)goto err1;
	if(tuner->demux<0||tuner->demux>255)goto err1;
	if(tuner->ca<0||tuner->ca>255)goto err1;
	if(tuner->inversion)switch(tuner->inversion)
	{
	case SATIP_SPI_OFF:
	case SATIP_SPI_ON:
		inversion=tuner->inversion;
		break;
	default:goto err1;
	}
	else inversion=SATIP_SPI_OFF;
	if(tuner->lna)switch(tuner->lna)
	{
	case SATIP_LNA_OFF:
		lna=0;
		break;
	case SATIP_LNA_ON:
		lna=1;
		break;
	default:goto err1;
	}
	else lna=LNA_AUTO;
	if(tuner->snrshift<0||tuner->snrshift>15)goto err1;
	switch(tuner->snrspeed)
	{
	case SATIP_SNR_SLOW:
	case SATIP_SNR_MED:
	case SATIP_SNR_FAST:
		break;
	default:goto err1;
	}
	if(tuner->dvbctype!=SATIP_ANNEX_A)goto err1;
	if(tuner->dscwait&&(tuner->dscwait<15||tuner->dscwait>100))goto err1;
	if(tuner->dscreplywait&&(tuner->dscreplywait<50||
		tuner->dscreplywait>250))goto err1;
	if(tuner->idleflush<0||tuner->idleflush>1)goto err1;
	if(tuner->fast<0||tuner->fast>1)goto err1;
	if(tuner->explicit<0||tuner->explicit>1)goto err1;
	if(tuner->prefer<0||tuner->prefer>255)goto err1;
	if(tuner->noshare<0||tuner->noshare>1)goto err1;
	if(tuner->srcnum<0||tuner->srcnum>32)goto err1;
	for(i=0;i<tuner->srcnum;i++)if(tuner->src[i]<1||tuner->src[i]>255)
		goto err1;

	switch(tuner->lnbtype)
	{
	case SATIP_LNB_UNIV:
	case SATIP_LNB_DBS:
	case SATIP_LNB_CMONO:
	case SATIP_LNB_CMULT:
	case SATIP_LNB_AUS:
		break;
	default:goto err1;
	}

	switch(tuner->diseqc)
	{
	case SATIP_DSC_NONE:
		if(tuner->srcnum>1)goto err1;
		break;

	case SATIP_DSC_TONE:
		if(!tuner->srcnum||tuner->srcnum>2)goto err1;
		if(tuner->srcnum==1)break;
		if(tuner->src[0]==tuner->src[1])goto err1;
		if((tuner->src[0]>>1)!=(tuner->src[1]>>1))goto err1;
		break;

	case SATIP_DSC_1_0:
	case SATIP_DSC_2_0:
		if(!tuner->srcnum||tuner->srcnum>4)goto err1;
		for(i=tuner->srcnum-1;i;i--)for(j=i-1;j>=0;j--)
			if(tuner->src[i]==tuner->src[j])goto err1;
		for(j=(tuner->src[0]-1)>>2,i=1;i<tuner->srcnum;i++)
			if(((tuner->src[i]-1)>>2)!=j)goto err1;
		break;

	case SATIP_DSC_1_1:
	case SATIP_DSC_2_1:
		if(!tuner->srcnum||tuner->srcnum>16)goto err1;
		for(i=tuner->srcnum-1;i;i--)for(j=i-1;j>=0;j--)
			if(tuner->src[i]==tuner->src[j])goto err1;
		for(j=(tuner->src[0]-1)>>4,i=1;i<tuner->srcnum;i++)
			if(((tuner->src[i]-1)>>4)!=j)goto err1;
		break;

	case SATIP_DSC_1_0_T:
	case SATIP_DSC_2_0_T:
		if(!tuner->srcnum||tuner->srcnum>8)goto err1;
		for(i=tuner->srcnum-1;i;i--)for(j=i-1;j>=0;j--)
			if(tuner->src[i]==tuner->src[j])goto err1;
		for(j=(tuner->src[0]-1)>>3,i=1;i<tuner->srcnum;i++)
			if(((tuner->src[i]-1)>>3)!=j)goto err1;
		break;

	case SATIP_DSC_1_1_T:
	case SATIP_DSC_2_1_T:
		if(!tuner->srcnum||tuner->srcnum>32)goto err1;
		for(i=tuner->srcnum-1;i;i--)for(j=i-1;j>=0;j--)
			if(tuner->src[i]==tuner->src[j])goto err1;
		for(j=(tuner->src[0]-1)>>5,i=1;i<tuner->srcnum;i++)
			if(((tuner->src[i]-1)>>5)!=j)goto err1;
		break;

	default:goto err1;
	}

	for(t=g->tlist;t;t=t->next)
	{
		if(tuner->deviceid==t->feid)goto err1;
		if(tuner->frontend==t->frontend||tuner->demux==t->demux)
			if(tuner->adapter==t->adapter)goto err1;
	}

	if(!(t=malloc(sizeof(TUNER)+tuner->srcnum*sizeof(int))))goto err1;
	memset(t,0,sizeof(TUNER));
	t->group=g;
	t->tunefd=-1;
	t->cafd=-1;
	t->dsfd=-1;
	t->feid=tuner->deviceid;
	t->adapter=tuner->adapter;
	t->frontend=tuner->frontend;
	t->demux=tuner->demux;
	t->ca=tuner->ca;
	t->inversion=inversion;
	t->lna=lna;
	t->lnb=tuner->lnbtype;
	t->annex=tuner->dvbctype;
	t->diseqc=tuner->diseqc;
	t->dwait=tuner->dscwait;
	t->d2wait=tuner->dscreplywait;
	t->dforce=tuner->dscforce;
	t->snrshift=tuner->snrshift;
	t->idleflush=tuner->idleflush;
	t->fast=tuner->fast;
	t->explicit=tuner->explicit;
	t->prefer=tuner->prefer;
	t->noshare=tuner->noshare;
	t->srcnum=tuner->srcnum;
	memcpy(t->src,tuner->src,tuner->srcnum*sizeof(int));

	switch(tuner->snrspeed)
	{
	case SATIP_SNR_SLOW:
		t->snrspeed=0x1f;
		break;
	case SATIP_SNR_MED:
		t->snrspeed=0x07;
		break;
	case SATIP_SNR_FAST:
		t->snrspeed=0x01;
		break;
	}

	switch(tuner->diseqc)
	{
	case SATIP_DSC_TONE:
		t->dseq=toneburst;
		break;

	case SATIP_DSC_1_0:
	case SATIP_DSC_1_1:
		t->dseq=diseqc1x;
		break;

	case SATIP_DSC_1_0_T:
	case SATIP_DSC_1_1_T:
		t->dseq=diseqc1xt;
		break;

	case SATIP_DSC_2_0:
	case SATIP_DSC_2_1:
		t->dseq=diseqc2x;
		break;

	case SATIP_DSC_2_0_T:
	case SATIP_DSC_2_1_T:
		t->dseq=diseqc2xt;
		break;
	}

	if(fe_info(t))goto err2;

	if(t->info.msys&(DVBS|DVBS2))
	{
		if(!t->srcnum)goto err2;
	}
	else if(t->srcnum||t->diseqc!=SATIP_DSC_NONE)goto err2;

	if(pthread_spin_init(&t->mtx,PTHREAD_PROCESS_PRIVATE))goto err2;

	if(pthread_mutex_init(&t->dtx,NULL))goto err3;

	if((t->epfd=epoll_create1(EPOLL_CLOEXEC))==-1)goto err4;

	if((t->reqfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err5;
	if((t->ansfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err6;

	if(g->camcb)
	{
		if((t->ocrfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err7;
		if((t->ocafd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err8;
		if((t->carfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err9;
		if((t->caafd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)
			goto err10;
	}

	if((t->tfd=timerfd_create(CLOCK_MONOTONIC,TFD_CLOEXEC|TFD_NONBLOCK))
		==-1)goto err11;
	if(g->camcb)if((t->catfd=timerfd_create(CLOCK_MONOTONIC,
		TFD_CLOEXEC|TFD_NONBLOCK))==-1)goto err12;

	if((t->dsfd=timerfd_create(CLOCK_MONOTONIC,TFD_CLOEXEC|TFD_NONBLOCK))
		==-1)goto err13;

	e.events=EPOLLIN;
	e.data.ptr=&tterm;
	if(epoll_ctl(t->epfd,EPOLL_CTL_ADD,t->group->termfd,&e))goto err14;

	e.events=EPOLLIN;
	e.data.ptr=&ttick;
	if(epoll_ctl(t->epfd,EPOLL_CTL_ADD,t->tfd,&e))goto err15;

	e.events=EPOLLIN;
	e.data.ptr=&twork;
	if(epoll_ctl(t->epfd,EPOLL_CTL_ADD,t->reqfd,&e))goto err16;

	if(pthread_create(&t->tuningmgr,&g->attr,tuningmgr,t))goto err17;

	if(g->camcb)if(pthread_create(&t->camgr,&g->attr,camgr,t))goto err18;

	if(pthread_create(&t->streamer,&g->attr,streamer,t))goto err19;

	info->caps|=t->info.caps;

	if(t->info.msys&(DVBS|DVBS2))
	{
		if(info->totals[SATIP_TOT_DVBS2]<9)
			info->totals[SATIP_TOT_DVBS2]++;
	}
	else if(t->info.msys&DVBT)
	{
		if(info->totals[SATIP_TOT_DVBT]<9)
			info->totals[SATIP_TOT_DVBT]++;
	}
	else if(t->info.msys&DVBT2)
	{
		if(info->totals[SATIP_TOT_DVBT2]<9)
			info->totals[SATIP_TOT_DVBT2]++;
	}
	else if(t->info.msys&DVBC)
	{
		if(info->totals[SATIP_TOT_DVBC]<9)
			info->totals[SATIP_TOT_DVBC]++;
	}
	else if(t->info.msys&DVBC2)
	{
		if(info->totals[SATIP_TOT_DVBC2]<9)
			info->totals[SATIP_TOT_DVBC2]++;
	}

	pthread_mutex_lock(&g->stx);
	t->next=g->tlist;
	g->tlist=t;
	pthread_mutex_unlock(&g->stx);

	return 0;

err19:	if(g->camcb)
	{
		pthread_cancel(t->camgr);
		pthread_join(t->camgr,NULL);
	}
err18:	pthread_cancel(t->tuningmgr);
	pthread_join(t->tuningmgr,NULL);
err17:	epoll_ctl(t->epfd,EPOLL_CTL_DEL,t->reqfd,NULL);
err16:	epoll_ctl(t->epfd,EPOLL_CTL_DEL,t->tfd,NULL);
err15:	epoll_ctl(t->epfd,EPOLL_CTL_DEL,t->group->termfd,NULL);
err14:	close(t->dsfd);
err13:	if(g->camcb)close(t->catfd);
err12:	close(t->tfd);
err11:	if(g->camcb)close(t->caafd);
err10:	if(g->camcb)close(t->carfd);
err9:	if(g->camcb)close(t->ocafd);
err8:	if(g->camcb)close(t->ocrfd);
err7:	close(t->ansfd);
err6:	close(t->reqfd);
err5:	close(t->epfd);
err4:	pthread_mutex_destroy(&t->dtx);
err3:	pthread_spin_destroy(&t->mtx);
err2:	free(t);
err1:	return SATIP_SYSFAIL;
}

void *satip_hw_init(SATIP_HW_CONFIG *config,SATIP_CFGINFO *info)
{
	GROUP *g;
	struct __user_cap_header_struct h;
	struct __user_cap_data_struct c[2];
	struct sched_param p;
	struct rlimit l;

	if(!config||!info)goto err1;
	if(config->streamlimit<0||config->streamlimit>65535)goto err1;

	if(config->rtprio)
	{
		if(config->rtprio<sched_get_priority_min(SCHED_RR))goto err1;
		if(config->rtprio>sched_get_priority_max(SCHED_RR))goto err1;
		if(geteuid())
		{
			memset(&h,0,sizeof(h));
			h.version=_LINUX_CAPABILITY_VERSION_3;
			h.pid=0;
			memset(c,0,sizeof(c));
			if(capget(&h,c))goto err1;
			if(!(c[0].effective&(1<<CAP_SYS_NICE)))
			{
				if(getrlimit(RLIMIT_RTPRIO,&l))goto err1;
				if(l.rlim_cur<config->rtprio)goto err1;
			}
		}
	}

	if(!(g=malloc(sizeof(GROUP))))goto err1;
	memset(g,0,sizeof(GROUP));
	g->limit=config->streamlimit;
	g->camcb=config->camfunc;
	g->campriv=config->priv;

	if(pthread_mutex_init(&g->stx,NULL))goto err2;
	if(g->camcb)if(pthread_mutex_init(&g->ctx,NULL))goto err3;

	if(pthread_attr_init(&g->attr))goto err4;
	if(pthread_attr_setdetachstate(&g->attr,PTHREAD_CREATE_JOINABLE))
		goto err5;
	if(config->stack)if(pthread_attr_setstacksize(&g->attr,config->stack))
		goto err5;
	if(config->rtprio)
	{
		memset(&p,0,sizeof(p));
		p.sched_priority=config->rtprio;
		if(pthread_attr_setschedpolicy(&g->attr,SCHED_RR))goto err5;
		if(pthread_attr_setschedparam(&g->attr,&p))goto err5;
		if(pthread_attr_setinheritsched(&g->attr,
			PTHREAD_EXPLICIT_SCHED))goto err5;
	}

	if((g->termfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err5;

	memset(info,0,sizeof(SATIP_CFGINFO));
	return g;

err5:	pthread_attr_destroy(&g->attr);
err4:	if(g->camcb)pthread_mutex_destroy(&g->ctx);
err3:	pthread_mutex_destroy(&g->stx);
err2:	free(g);
err1:	return NULL;
}

void satip_hw_fini(void *handle)
{
	GROUP *g=(GROUP *)handle;
	TUNER *t;
	STREAM *s;
	uint64_t dummy=1;
	struct itimerspec ts;

	if(!g)return;

	pthread_mutex_lock(&g->stx);

	dummy=write(g->termfd,&dummy,sizeof(dummy));

	memset(&ts,0,sizeof(ts));

	while(g->tlist)
	{
		t=g->tlist;
		g->tlist=t->next;

		pthread_join(t->streamer,NULL);

		if(g->camcb)pthread_join(t->camgr,NULL);

		pthread_mutex_lock(&t->dtx);
		if(t->tunefd!=-1)close(t->tunefd);
		if(t->cafd!=-1)close(t->cafd);
		pthread_mutex_unlock(&t->dtx);

		pthread_join(t->tuningmgr,NULL);

		if(g->camcb)timerfd_settime(t->catfd,0,&ts,0);
		timerfd_settime(t->tfd,0,&ts,0);

		while(t->slist)
		{
			s=t->slist;
			t->slist=s->next;
			close(s->fd);
			free(s);
		}

		timerfd_settime(t->dsfd,0,&ts,0);
		close(t->dsfd);

		epoll_ctl(t->epfd,EPOLL_CTL_DEL,t->reqfd,NULL);
		epoll_ctl(t->epfd,EPOLL_CTL_DEL,t->tfd,NULL);
		epoll_ctl(t->epfd,EPOLL_CTL_DEL,t->group->termfd,NULL);

		if(g->camcb)close(t->catfd);
		close(t->tfd);
		if(g->camcb)
		{
			close(t->caafd);
			close(t->carfd);
			close(t->ocafd);
			close(t->ocrfd);
		}
		close(t->ansfd);
		close(t->reqfd);
		close(t->epfd);
		pthread_mutex_destroy(&t->dtx);
		pthread_spin_destroy(&t->mtx);

		free(t);
	}
	close(g->termfd);
	pthread_attr_destroy(&g->attr);
	if(g->camcb)pthread_mutex_destroy(&g->ctx);
	pthread_mutex_unlock(&g->stx);
	pthread_mutex_destroy(&g->stx);
	free(g);
}
