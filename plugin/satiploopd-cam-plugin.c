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
#include <pthread.h>
#include <signal.h>
#include <poll.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include "satip.h"

#ifdef __GNUC__
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
#define CAMQ		16
#define MAX_CAM_MSGLEN	2048
#define MAX_CAIDS	64
#define MAX_SIDS	64
#define CAMWAIT_STEP	9
#define STACKSIZE	131072

#define RSRC_MGR	0
#define RSRC_APPINFO	1
#define RSRC_CASUPP	2
#define RSRC_HOSTCTL	3
#define RSRC_DATIM	4
#define RSRC_MMI	5
#define RSRC_UNKNOWN	6

#define T_PROFILE_ENQ	0
#define T_PROFILE	1
#define T_PROFILE_CHG	2
#define T_APPL_INFO_ENQ	3
#define T_APPL_INFO	4
#define T_ENTER_MENU	5
#define T_CA_INFO_REQ	6
#define T_CA_INFO	7
#define T_CA_PMT	8
#define T_CA_PMT_REPLY	9
#define T_TUNE		10
#define T_REPLACE	11
#define T_CLEAR_REPLACE	12
#define T_ASK_RELEASE	13
#define T_DATIM_ENQ	14
#define T_DATIM		15
#define T_CLOSE_MMI	16
#define T_DISPLAY_CTL	17
#define T_DISPLAY_REPLY	18
#define T_TEXT_LAST	19
#define T_TEXT_MORE	20
#define T_KEYPAD_CTL	21
#define T_KEYPRESS	22
#define T_ENQ		23
#define T_ANSW		24
#define T_MENU_LAST	25
#define T_MENU_MORE	26
#define T_MENU_ANSW	27
#define T_LIST_LAST	28
#define T_LIST_MORE	29
#define T_SUB_SEG_LAST	30
#define T_SUB_SEG_MORE	31
#define T_DISPLAY_MSG	32
#define T_SC_END_MARK	33
#define T_SC_DONE	34
#define T_SC_CONTROL	35
#define T_SUB_DOWN_LAST	36
#define T_SUB_DOWN_MORE	37
#define T_FLUSH_DOWNLD	38
#define T_DOWLNLD_REPLY	39
#define T_COMMS_CMD	40
#define T_CONN_DESCR	41
#define T_COMMS_REPLY	42
#define T_COMMS_SNDLAST	43
#define T_COMMS_SNDMORE	44
#define T_COMMS_RCVLAST	45
#define T_COMMS_RCVMORE	46

#define T_SB		0x80
#define T_RCV		0x81
#define CREATE_T_C	0x82
#define C_T_C_REPLY	0x83
#define DELETE_T_C	0x84
#define D_T_C_REPLY	0x85
#define REQUEST_T_C	0x86
#define NEW_T_C		0x87
#define T_C_ERROR	0x88
#define DATA_LAST	0xa0
#define DATA_MORE	0xa1

#define IDLE		0
#define IN_CREATION	1
#define ACTIVE		2
#define IN_DELETION	3

#define OPEN_SESS_REQ	0x91
#define OPEN_SESS_RSP	0x92
#define CREATE_SESS	0x93
#define CREATE_SESS_RSP	0x94
#define CLOSE_SESS_REQ	0x95
#define CLOSE_SESS_RSP	0x96
#define SESS_NUMBER	0x90

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

typedef struct
{
	int slot;
	int maxdecode;
	int pmtfilter;
	int streampids;
	unsigned short preftotal;
	unsigned short prefer[MAX_SIDS];
	unsigned short igntotal;
	unsigned short ignore[MAX_SIDS];
	unsigned short catotal;
	unsigned short caids[MAX_CAIDS];
} CONFIG;

typedef struct
{
	void (*plugin_pids)(void *ctx,SATIP_PIDS *set,int from_notify_hook);
	void *ctx;
	void *filter;
	void *handle;
	SATIP_UTIL_PAT *pat;
	SATIP_UTIL_CAT *cat;
	SATIP_TUNE *tunefilter;
	void *flt[SATIP_MAX_PIDS];
	void *sec[SATIP_MAX_PIDS];
	void *res[SATIP_MAX_PIDS];
	SATIP_UTIL_PMT *list[MAXPIDS];
	int map[MAXPIDS];
	short pids[SATIP_MAX_PIDS];
	short fpid[SATIP_MAX_PIDS];
	SATIP_PIDS set;
	pthread_mutex_t mtx;
	pthread_spinlock_t stx;
	pthread_t th;
	int running:1;
	int term:1;
	int skip:1;
	int pidtotal;
	int total;
	int efd;
	int fe;
	int cnum;
	int cards;
	int filtertotal;
	unsigned char pmtidx[65536];
	unsigned char slotidx[SATIP_CAM_SLOTS];
	CONFIG conf[SATIP_CAM_SLOTS];
} PIDDATA;

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
	SATIP_HW_CAM_IO msg;
} CAMDATA;

typedef struct _camdata
{
	PMTDATA *new;
	PMTDATA *curr;
	void *handle;
	int fe;
	int slot;
	int nfd;
	int xfd;
	int tfd;
	int running;
	int head;
	int tail;
	int fill;
	int newpmt;
	int rstate;
	int astate;
	int cstate;
	int num[6];
	int apptype;
	int mfctr;
	int mcode;
	int idtotal;
	char apptxt[128];
	short caid[MAX_CAIDS];
	CAMDATA *queue[CAMQ];
	pthread_mutex_t mtx;
	pthread_t th;
} CAMSLOT;

typedef struct _camwrk
{
	struct _camwrk *next;
	int fe;
	CAMSLOT *slot[SATIP_CAM_SLOTS];
} CAMWRK;

typedef struct
{
	char *name;
	int min;
	int max;
	unsigned long offset;
	int (*func)(void *s,char *val);
} CFGFILE;

#ifndef PROFILE

static void patcb(void *dta,int len,void *priv) __attribute__ ((hot));
static void catcb(void *dta,int len,void *priv) __attribute__ ((hot));
static void pmtcb(void *dta,int len,void *priv) __attribute__ ((hot));

static int docaids(void *s,char *val) __attribute__ ((cold));
static int doprefign(void *s,char *val) __attribute__ ((cold));
static int doparam(CFGFILE *p,void *s,char *name,char *val)
	__attribute__ ((cold));
static int parse_config(char *fn,PIDDATA *data) __attribute__ ((cold));
static int global_init(void) __attribute__ ((cold));
static void global_exit(void) __attribute__ ((cold));
void *plugin_init(char *config,void *filter_ctx,
	void (*plugin_pids)(void *ctx,SATIP_PIDS *set,int from_notify_hook),
	void *ctx) __attribute__ ((cold));
void plugin_exit(void *plugin_ctx) __attribute__ ((cold));

#else

static int docaids(void *s,char *val);
static int doprefign(void *s,char *val);

#endif

static pthread_t th;
static int running;
static int rfd;
static int afd;
static int xfd;
static int type;
static int fe;
static int slot;
static void *dptr;
static pthread_mutex_t mtx=PTHREAD_MUTEX_INITIALIZER;
static pthread_attr_t attr;

static unsigned char rsrc[6][4]=
{
	{0x00,0x01,0x00,0x41},
	{0x00,0x02,0x00,0x41},
	{0x00,0x03,0x00,0x41},
	{0x00,0x20,0x00,0x41},
	{0x00,0x24,0x00,0x41},
	{0x00,0x40,0x00,0x41}
};

static unsigned char tags[47][3]=
{
	{0x9f,0x80,0x10},
	{0x9f,0x80,0x11},
	{0x9f,0x80,0x12},
	{0x9f,0x80,0x20},
	{0x9f,0x80,0x21},
	{0x9f,0x80,0x22},
	{0x9f,0x80,0x30},
	{0x9f,0x80,0x31},
	{0x9f,0x80,0x32},
	{0x9f,0x80,0x33},
	{0x9f,0x84,0x00},
	{0x9f,0x84,0x01},
	{0x9f,0x84,0x02},
	{0x9f,0x84,0x03},
	{0x9f,0x84,0x40},
	{0x9f,0x84,0x41},
	{0x9f,0x88,0x00},
	{0x9f,0x88,0x01},
	{0x9f,0x88,0x02},
	{0x9f,0x88,0x03},
	{0x9f,0x88,0x04},
	{0x9f,0x88,0x05},
	{0x9f,0x88,0x06},
	{0x9f,0x88,0x07},
	{0x9f,0x88,0x08},
	{0x9f,0x88,0x09},
	{0x9f,0x88,0x0a},
	{0x9f,0x88,0x0b},
	{0x9f,0x88,0x0c},
	{0x9f,0x88,0x0d},
	{0x9f,0x88,0x0e},
	{0x9f,0x88,0x0f},
	{0x9f,0x88,0x00},
	{0x9f,0x88,0x01},
	{0x9f,0x88,0x02},
	{0x9f,0x88,0x03},
	{0x9f,0x88,0x04},
	{0x9f,0x88,0x05},
	{0x9f,0x88,0x06},
	{0x9f,0x88,0x07},
	{0x9f,0x8c,0x00},
	{0x9f,0x8c,0x01},
	{0x9f,0x8c,0x02},
	{0x9f,0x8c,0x03},
	{0x9f,0x8c,0x04},
	{0x9f,0x8c,0x05},
	{0x9f,0x8c,0x06}
};

static CFGFILE cparams[]=
{
	{"slot",1,8,offsetof(CONFIG,slot),NULL},
	{"caids",0,0,offsetof(CONFIG,caids),docaids},
	{"maxdecode",1,8,offsetof(CONFIG,maxdecode),NULL},
	{"pmtfilter",0,1,offsetof(CONFIG,pmtfilter),NULL},
	{"streampids",0,1,offsetof(CONFIG,streampids),NULL},
	{"prefer",0,0,offsetof(CONFIG,prefer),doprefign},
	{"ignore",0,0,offsetof(CONFIG,ignore),doprefign},
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

static int doprefign(void *s,char *val)
{
	int i;
	int tmp;
	char *mem;
	unsigned short *prefign=(unsigned short *)s;

	for(i=0,val=strtok_r(val,",",&mem);val;i++,val=strtok_r(NULL,",",&mem))
	{
		if(!*val||i>=MAX_SIDS)return -1;
		tmp=atoi(val);
		if(tmp<1||tmp>65535)return -1;
		prefign[i]=(unsigned short)tmp;
	}
	if(!i)return -1;
	prefign[-1]=i;
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

static int parse_config(char *fn,PIDDATA *data)
{
	int i;
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

		if(!strcmp(name,"[card]"))
		{
			if(data->cnum==SATIP_CAM_SLOTS)
			{
				fprintf(stderr,"Out of memory\n");
				goto err2;
			}
			c=&data->conf[data->cnum++];
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
					SATIP_IGNCAPS,val+1,NULL,NULL,0,NULL,
					NULL,t,&set,&add,&del,NULL)||
					set.numpids!=SATIP_NOPIDS||add.numpids||
					del.numpids)goto err3;
			}
			else goto err3;
		}
		else goto err3;
	}

	if(!data->cnum)
	{
		fprintf(stderr,"No card(s) defined.\n");
		goto err2;
	}

	for(i=0;i<data->cnum;i++)
	{
		if(!data->conf[i].slot||!data->conf[i].maxdecode)goto err4;
		data->conf[i].slot-=1;
		if(data->cards&(1<<data->conf[i].slot))goto err4;
		data->cards|=(1<<data->conf[i].slot);
		data->slotidx[data->conf[i].slot]=i;
	}

	posix_fadvise(fileno(fp),0,0,POSIX_FADV_DONTNEED);
	fclose(fp);
	return 0;

err4:	fprintf(stderr,"Syntax error in card(s) definition(s).\n");
	goto err2;
err3:	fprintf(stderr,"Syntax error in line %d of %s\n",line,fn);
err2:	fclose(fp);
err1:	return -1;
}

static void pmtsend(CAMSLOT *e,PMT *pmt,int type,int mode,int tsid)
{
	int i;
	int j;
	int l;
	int pos;
	int mem;
	int off=7;
	unsigned char *ptr;
	SATIP_HW_CAM_IO m;
	unsigned char bfr[MAX_CAM_MSGLEN];

	m.deviceid=e->fe;
	m.type=SATIP_CAM_WRITE;
	m.slot=e->slot;
	m.tsid=tsid;

	ptr=bfr+15;

	ptr[0]=type;
	ptr[1]=pmt->prognum>>8;
	ptr[2]=pmt->prognum&0xff;
	ptr[3]=0x01;

	if(pmt->catotal)
	{
		for(pos=7,i=0;i<pmt->catotal;i++)
		{
			l=satip_util_get_raw_len(pmt->raw[i]);
			if(l>sizeof(bfr)-pos-15)return;
			memcpy(ptr+pos,pmt->raw[i],l);
			pos+=l;
		}

		ptr[4]=(pos-6)>>8;
		ptr[5]=(pos-6)&0xff;
		ptr[6]=mode;
	}
	else
	{
		pos=6;
		ptr[4]=0;
		ptr[5]=0;
	}

	for(i=0;i<pmt->total;i++)
	{
		if(pos+13+(pmt->data[i].catotal?1:0)>sizeof(bfr)-7)return;
		ptr[pos++]=pmt->data[i].type;
		ptr[pos++]=pmt->data[i].pid>>8;
		ptr[pos++]=pmt->data[i].pid&0xff;
		mem=pos;
		pos+=2;
		if(pmt->data[i].catotal)ptr[pos++]=mode;

		for(j=0;j<pmt->data[i].catotal;j++)
		{
			l=satip_util_get_raw_len(pmt->data[i].raw[j]);
			if(l>sizeof(bfr)-pos-15)return;
			memcpy(ptr+pos,pmt->data[i].raw[j],l);
			pos+=l;
		}

		ptr[mem]=(pos-mem-2)>>8;
		ptr[mem+1]=(pos-mem-2)&0xff;
	}

	if(pos>127&&pos<256)
	{
		off-=1;
		bfr[off+7]=0x81;
		bfr[off+8]=pos;
		mem=1;
	}
	else if(pos>=256)
	{
		off-=2;
		bfr[off+7]=0x82;
		bfr[off+8]=pos>>8;
		bfr[off+9]=pos&0xff;
		mem=2;
	}
	else
	{
		bfr[off+7]=pos;
		mem=0;
	}

	bfr[off]=SESS_NUMBER;
	bfr[off+1]=2;
	bfr[off+2]=e->num[RSRC_CASUPP]>>8;
	bfr[off+3]=e->num[RSRC_CASUPP]&0xff;

	memcpy(bfr+off+4,tags[T_CA_PMT],3);

	mem=pos+mem+9;
	m.len=mem+2;

	if(mem>127&&mem<256)
	{
		m.len+=1;
		off-=4;
		bfr[off]=DATA_LAST;
		bfr[off+1]=0x81;
		bfr[off+2]=mem;
		bfr[off+3]=tsid;
	}
	else if(mem>256)
	{
		m.len+=2;
		off-=5;
		bfr[off]=DATA_LAST;
		bfr[off+1]=0x82;
		bfr[off+2]=mem>>8;
		bfr[off+3]=mem&0xff;
		bfr[off+4]=tsid;
	}
	else
	{
		off-=3;
		bfr[off]=DATA_LAST;
		bfr[off+1]=mem;
		bfr[off+2]=tsid;
	}

	m.data=bfr+off;
	satip_hw_cam_io(e->handle,&m);
}

static void pmthandler(CAMSLOT *e,int tsid)
{
	int i;
	int j;
	PMTDATA *prev;

	prev=e->curr;
	e->curr=e->new;
	e->new=NULL;

	if(!prev&&!e->curr)goto out;

	if(prev&&!e->curr)
	{
		pmtsend(e,prev->list[0],PMT_LIST_ONLY,NOT_SELECTED,tsid);
		goto out;
	}

	if(e->curr&&!prev)
	{
		for(i=0;i<e->curr->total;i++)
		{
			pmtsend(e,e->curr->list[i],i?PMT_LIST_ADD:PMT_LIST_ONLY,
				OK_DESCRAMBLE,tsid);
		}
		goto out;
	}

	for(i=0,j=0;i<e->curr->total&&j<prev->total;)
	{
		if(e->curr->list[i]->prognum==prev->list[j]->prognum)
			goto match;
		else if(e->curr->list[i]->prognum<prev->list[j]->prognum)i++;
		else j++;
	}

	for(i=0;i<e->curr->total;i++)
	{
		pmtsend(e,e->curr->list[i],i?PMT_LIST_ADD:PMT_LIST_ONLY,
			OK_DESCRAMBLE,tsid);
	}
	goto out;

match:	for(i=0,j=0;i<e->curr->total&&j<prev->total;)
	{
		if(e->curr->list[i]->prognum==prev->list[j]->prognum)
		{
			i++;
			j++;
		}
		else if(e->curr->list[i]->prognum<prev->list[j]->prognum)i++;
		else
		{
			pmtsend(e,prev->list[j],PMT_LIST_UPDATE,NOT_SELECTED,
				tsid);
			j++;
		}
	}

	for(;j<prev->total;j++)pmtsend(e,prev->list[j],PMT_LIST_UPDATE,
		NOT_SELECTED,tsid);

	for(i=0,j=0;i<e->curr->total&&j<prev->total;)
	{
		if(e->curr->list[i]->prognum==prev->list[j]->prognum)
		{
			pmtsend(e,e->curr->list[i],PMT_LIST_UPDATE,
				OK_DESCRAMBLE,tsid);
			i++;
			j++;
		}
		else if(e->curr->list[i]->prognum<prev->list[j]->prognum)
		{
			pmtsend(e,e->curr->list[i],PMT_LIST_ADD,OK_DESCRAMBLE,
				tsid);
			i++;
		}
		else j++;
	}

	for(;i<e->curr->total;i++)pmtsend(e,e->curr->list[i],PMT_LIST_ADD,
		OK_DESCRAMBLE,tsid);

out:	if(prev)
	{
		for(i=0;i<prev->total;i++)free(prev->list[i]);
		free(prev);
	}

}

static int sesswrk(CAMSLOT *e,unsigned char *data,int len,unsigned char *ptr)
{
	int i;
	int j;
	int off;
	int err=0xf0;
	uint64_t dummy;

	if(!len)
	{
		if(e->rstate==1)
		{
			ptr[0]=SESS_NUMBER;
			ptr[1]=2;
			ptr[2]=e->num[RSRC_MGR]>>8;
			ptr[3]=e->num[RSRC_MGR]&0xff;
			memcpy(ptr+4,tags[T_PROFILE_ENQ],3);
			ptr[7]=0;
			e->rstate++;
			return 8;
		}
		else if(e->astate==1)
		{
			ptr[0]=SESS_NUMBER;
			ptr[1]=2;
			ptr[2]=e->num[RSRC_APPINFO]>>8;
			ptr[3]=e->num[RSRC_APPINFO]&0xff;
			memcpy(ptr+4,tags[T_APPL_INFO_ENQ],3);
			ptr[7]=0;
			e->astate++;
			return 8;
		}
		else if(e->cstate==1)
		{
			ptr[0]=SESS_NUMBER;
			ptr[1]=2;
			ptr[2]=e->num[RSRC_CASUPP]>>8;
			ptr[3]=e->num[RSRC_CASUPP]&0xff;
			memcpy(ptr+4,tags[T_CA_INFO_REQ],3);
			ptr[7]=0;
			e->cstate++;
			return 8;
		}

		return 0;
	}

	switch(data[0])
	{
	case OPEN_SESS_REQ:
		if(len<6||data[1]!=4)break;
		for(i=0;i<6;i++)if(!memcmp(data+2,rsrc[i],4))break;
		switch(i)
		{
		case RSRC_MGR:
		case RSRC_APPINFO:
		case RSRC_CASUPP:
		case RSRC_DATIM:
		case RSRC_MMI:
			if(e->num[i])err=0xf3;
			else
			{
				e->num[i]=i+1;
				err=0x00;
			}
		default:ptr[0]=OPEN_SESS_RSP;
			ptr[1]=7;
			ptr[2]=err;
			memcpy(ptr+3,data+2,4);
			if(!err)
			{
				ptr[7]=e->num[i]>>8;
				ptr[8]=e->num[i]&0xff;
				switch(i)
				{
				case RSRC_MGR:
					if(!e->rstate)e->rstate=1;
					break;
				case RSRC_APPINFO:
					if(!e->astate)e->astate=1;
					break;
				case RSRC_CASUPP:
					if(!e->cstate)e->cstate=1;
					break;
				}
			}
			else ptr[7]=ptr[8]=0x00;
			return 9;
		}
		break;

	case SESS_NUMBER:
		if(len<4||data[1]!=2)break;
		i=data[2];
		i<<=8;
		i|=data[3];
		if(i<1||i>=RSRC_UNKNOWN||e->num[i-1]!=i)break;
		switch(--i)
		{
		case RSRC_MGR:
			if(len<7)break;
			if(!memcmp(data+4,tags[T_PROFILE],3)&&e->rstate==2)
			{
				ptr[0]=SESS_NUMBER;
				ptr[1]=2;
				ptr[2]=e->num[RSRC_MGR]>>8;
				ptr[3]=e->num[RSRC_MGR]&0xff;
				memcpy(ptr+4,tags[T_PROFILE_CHG],3);
				ptr[7]=0;
				e->rstate++;
				return 8;
			}
			if(!memcmp(data+4,tags[T_PROFILE_ENQ],3)&&e->rstate>=3)
			{
				ptr[0]=SESS_NUMBER;
				ptr[1]=2;
				ptr[2]=e->num[RSRC_MGR]>>8;
				ptr[3]=e->num[RSRC_MGR]&0xff;
				memcpy(ptr+4,tags[T_PROFILE],3);
				ptr[7]=20;
				memcpy(ptr+8,rsrc[RSRC_MGR],4);
				memcpy(ptr+12,rsrc[RSRC_APPINFO],4);
				memcpy(ptr+16,rsrc[RSRC_CASUPP],4);
				memcpy(ptr+20,rsrc[RSRC_DATIM],4);
				memcpy(ptr+24,rsrc[RSRC_MMI],4);
				return 28;
			}
			break;

		case RSRC_APPINFO:
			if(len<7)break;
			if(!memcmp(data+4,tags[T_APPL_INFO],3))
			{
				if(len<8||len<data[7]+8)break;
				if(e->astate==2)e->astate=3;
				if(len<9)break;
				e->apptype=data[8];
				if(len<11)break;
				e->mfctr=data[9];
				e->mfctr<<=8;
				e->mfctr|=data[10];
				if(len<13)break;
				e->mcode=data[11];
				e->mcode<<=8;
				e->mcode|=data[12];
				if(len<14)break;
				if(len<data[13]+14)break;
				memcpy(e->apptxt,data+14,data[13]);
				e->apptxt[data[13]]=0;
				return 0;
			}
			break;

		case RSRC_CASUPP:
			if(len<7)break;
			if(!memcmp(data+4,tags[T_CA_INFO],3))
			{
				if(len<8||data[7]==0x80||data[7]>0x81||
					(len<9&&data[7]==0x81))break;
				if(data[7]==0x81)
				{
					if(len<data[8]+9)break;
					off=1;
				}
				else
				{
					if(len<data[7]+8)break;
					off=0;
				}
				for(i=8+off,j=0;i<len-1&&j<MAX_CAIDS;i+=2,j++)
				{
					e->caid[j]=data[i];
					e->caid[j]<<=8;
					e->caid[j]|=data[i+1];
				}
				e->idtotal=j;
				if(e->cstate==2)e->cstate=3;
				else if(e->cstate>=3&&
						e->cstate<CAMWAIT_STEP)
				{
					e->cstate=CAMWAIT_STEP+1;
					dummy=1;
					dummy=write(e->nfd,&dummy,
						sizeof(dummy));
				}
				return 0;
			}
			break;
		}
		break;
	}

	return 0;
}

static void *camwrk(void *dta)
{
	CAMSLOT *e=(CAMSLOT *)dta;
	SATIP_HW_CAM_IO *m;
	int timeout;
	int notify;
	int state=IDLE;
	int tsid=0;
	int waiting=0;
	int expect=0;
	int len;
	int ofs;
	struct pollfd p[3];
	uint64_t dummy;
	sigset_t set;
	SATIP_HW_CAM_IO msg;
	unsigned char bfr[31];
	struct itimerspec on;
	struct itimerspec off;

	sigfillset(&set);
	pthread_sigmask(SIG_SETMASK,&set,NULL);

	pthread_setname_np(pthread_self(),"plugin camwrk");

	memset(&off,0,sizeof(off));
	memset(&on,0,sizeof(on));
	on.it_interval.tv_nsec=300000000;
	on.it_value.tv_nsec=300000000;

	msg.deviceid=e->fe;
	msg.type=SATIP_CAM_WRITE;
	msg.slot=e->slot;
	msg.data=bfr;

	p[0].fd=e->xfd;
	p[0].events=POLLIN;
	p[1].fd=e->nfd;
	p[1].events=POLLIN;
	p[2].fd=e->tfd;
	p[2].events=POLLIN;

	dummy=1;
	dummy=write(e->nfd,&dummy,sizeof(dummy));

	while(1)
	{
		if(poll(p,3,-1)<=0)continue;

		if(p[0].revents&POLLIN)break;

		if(p[2].revents&POLLIN)
		{
			dummy=read(e->tfd,&dummy,sizeof(dummy));
			timeout=1;
		}
		else timeout=0;

		if(p[1].revents&POLLIN)
		{
			dummy=read(e->nfd,&dummy,sizeof(dummy));
			notify=1;
		}
		else notify=0;

		switch(state)
		{
		case IDLE:
			timerfd_settime(e->tfd,0,&off,NULL);
			dummy=read(e->tfd,&dummy,sizeof(dummy));
			pthread_mutex_lock(&e->mtx);
			while(e->fill)
			{
				e->fill--;
				free(e->queue[e->tail++]);
				if(e->tail==CAMQ)e->tail=0;
			}
			pthread_mutex_unlock(&e->mtx);
			waiting=0;
			expect=0;
			if(++tsid>16)tsid=0;
			msg.tsid=(unsigned char)tsid;
			msg.len=3;
			bfr[0]=CREATE_T_C;
			bfr[1]=1;
			bfr[2]=(unsigned char)tsid;
			satip_hw_cam_io(e->handle,&msg);
			timerfd_settime(e->tfd,0,&on,NULL);
			state=IN_CREATION;
			break;

		case IN_CREATION:
			if(timeout)
			{
				state=IDLE;
				break;
			}
			if(!notify)break;
			pthread_mutex_lock(&e->mtx);
crerep:			if(!e->fill)
			{
				pthread_mutex_unlock(&e->mtx);
				break;
			}
			m=&e->queue[e->tail]->msg;
			if(m->len!=7||m->data[0]!=C_T_C_REPLY||
				m->data[1]!=1||m->data[2]!=tsid||
				m->data[3]!=T_SB||m->data[4]!=2||
				m->data[5]!=tsid)
			{
				e->fill--;
				free(e->queue[e->tail++]);
				if(e->tail==CAMQ)e->tail=0;
				goto crerep;
			}
			else
			{
				waiting=(m->data[6]&0x80)?1:0;
				e->fill--;
				free(e->queue[e->tail++]);
				if(e->tail==CAMQ)e->tail=0;
			}
			pthread_mutex_unlock(&e->mtx);
			if(waiting)
			{
				dummy=1;
				dummy=write(e->nfd,&dummy,sizeof(dummy));
			}
			e->rstate=e->astate=e->cstate=e->num[0]=e->num[1]=
				e->num[2]=e->num[3]=e->num[4]=e->num[5]=
				e->apptype=e->mfctr=e->mcode=e->idtotal=0;
			e->apptxt[0]=0;
			state=ACTIVE;
			break;

		case ACTIVE:
			if(timeout)
			{
				if(e->cstate>=3&&
					e->cstate<CAMWAIT_STEP)
				{
					e->cstate++;
					if(e->cstate==CAMWAIT_STEP)
					{
						dummy=1;
						dummy=write(e->nfd,&dummy,
							sizeof(dummy));
						e->cstate++;
					}
				}
				if(expect)
				{
					msg.len=3;
					bfr[0]=DELETE_T_C;
					bfr[1]=1;
					bfr[2]=tsid;
					satip_hw_cam_io(e->handle,&msg);
					state=IDLE;
					break;
				}

				msg.len=3;
				bfr[0]=DATA_LAST;
				bfr[1]=1;
				bfr[2]=tsid;
				satip_hw_cam_io(e->handle,&msg);
			}

			if(!notify)break;

			pthread_mutex_lock(&e->mtx);

			if(e->cstate>CAMWAIT_STEP&&e->newpmt)
			{
				pmthandler(e,tsid);
				e->newpmt=0;
			}

			if(e->fill)expect=0;

			while(e->fill)
			{
				m=&e->queue[e->tail]->msg;
				if(m->len>=1)switch(m->data[0])
				{
				case T_SB:
					if(m->len!=4||m->data[1]!=2||
						m->data[2]!=tsid)break;
					waiting|=(m->data[3]&0x80)?1:0;
					break;

				case DELETE_T_C:
					if(m->len<3||m->data[1]!=1||
						m->data[2]!=tsid)break;
					state=IDLE;
					break;

				case REQUEST_T_C:
					if(m->len<3||m->data[1]!=1||
						 m->data[2]!=tsid)break;
					msg.len=4;
					bfr[0]=T_C_ERROR;
					bfr[1]=2;
					bfr[2]=tsid;
					bfr[3]=1;
					satip_hw_cam_io(e->handle,&msg);
					break;

				case DATA_MORE:
				case DATA_LAST:
					if(m->len<3)break;
					ofs=0;
					if(m->data[1]&0x80)
					{
						if((ofs=m->data[1]&0x7f)>2||
							!ofs||m->len<3+ofs)
								break;
						switch(ofs)
						{
						case 1:	len=m->data[2];
							break;
						case 2:	len=m->data[2];
							len<<=8;
							len|=m->data[3];
							break;
						}
					}
					else len=m->data[1];
					if(!len||m->data[2+ofs]!=tsid||
						m->len<len+2+ofs+4||
						m->data[len+2+ofs]!=T_SB||
						m->data[len+3+ofs]!=2||
						m->data[len+4+ofs]!=tsid)break;
					waiting|=(m->data[len+5+ofs]&0x80)?1:0;
					if(len-1>0)
						len=sesswrk(e,&m->data[3+ofs],
							len-1,bfr+3);
					while(len)
					{
						msg.len=len+3;
						bfr[0]=DATA_LAST;
						bfr[1]=len+1;
						bfr[2]=tsid;
						satip_hw_cam_io(e->handle,&msg);
						len=sesswrk(e,NULL,0,bfr+3);
					}
					break;
				}

				e->fill--;
				free(e->queue[e->tail++]);
				if(e->tail==CAMQ)e->tail=0;
			}

			if(state!=ACTIVE)goto out;

			if(waiting)
			{
				timerfd_settime(e->tfd,0,&off,NULL);
				dummy=read(e->tfd,&dummy,sizeof(dummy));
				msg.len=3;
				bfr[0]=T_RCV;
				bfr[1]=1;
				bfr[2]=tsid;
				satip_hw_cam_io(e->handle,&msg);
				timerfd_settime(e->tfd,0,&on,NULL);
				expect=1;
				waiting=0;
			}

out:			pthread_mutex_unlock(&e->mtx);
			break;

		case IN_DELETION:
			if(timeout)
			{
				state=IDLE;
				break;
			}
			if(!notify)break;
			pthread_mutex_lock(&e->mtx);
delrep:			if(!e->fill)
			{
				pthread_mutex_unlock(&e->mtx);
				break;
			}
			m=&e->queue[e->tail]->msg;
			if(m->len<3||m->data[0]!=D_T_C_REPLY||
				m->data[1]!=1||m->data[2]!=tsid)
			{
				e->fill--;
				free(e->queue[e->tail++]);
				if(e->tail==CAMQ)e->tail=0;
				goto delrep;
			}
			else
			{
				free(e->queue[e->tail++]);
				e->fill=0;
				e->head=0;
				e->tail=0;
				state=IDLE;
			}
			pthread_mutex_unlock(&e->mtx);
			break;
		}

	}

	pthread_exit(NULL);
}

static int startcam(CAMSLOT *e,void *handle)
{
	if(e->running)return 0;
	e->handle=handle;
	if((e->nfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err1;
	if((e->xfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err2;
	if((e->tfd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))
		==-1)goto err3;
	if(pthread_mutex_init(&e->mtx,NULL))goto err4;
	if(pthread_create(&e->th,&attr,camwrk,e))goto err5;
	e->running=1;
	return 0;

err5:	pthread_mutex_destroy(&e->mtx);
err4:	close(e->tfd);
err3:	close(e->xfd);
err2:	close(e->nfd);
err1:	return -1;
}

static void stopcam(CAMSLOT *e)
{
	int i;
	uint64_t dummy=1;

	if(!e->running)return;
	dummy=write(e->xfd,&dummy,sizeof(dummy));
	pthread_join(e->th,NULL);
	pthread_mutex_destroy(&e->mtx);
	close(e->tfd);
	close(e->xfd);
	close(e->nfd);
	e->running=0;
	if(e->curr)
	{
		for(i=0;i<e->curr->total;i++)free(e->curr->list[i]);
		free(e->curr);
		e->curr=NULL;
	}
}

static void *manager(void *unused)
{
	int i;
	CAMWRK *list=NULL;
	CAMWRK *w;
	CAMSLOT *e;
	CAMDATA *c;
	struct pollfd p[2];
	uint64_t dummy;
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_SETMASK,&set,NULL);

	pthread_setname_np(pthread_self(),"plugin manager");

	p[0].fd=xfd;
	p[0].events=POLLIN;
	p[1].fd=rfd;
	p[1].events=POLLIN;

	while(1)
	{
		if(poll(p,2,-1)<=0)continue;

		if(p[0].revents&POLLIN)break;

		if(p[1].revents&POLLIN)
		{
			dummy=read(rfd,&dummy,sizeof(dummy));

			switch(type)
			{
			case 0:	type=1;
				if(!fe)goto out;
				for(w=list;w;w=w->next)
					if(w->fe==fe)break;
				if(!w)
				{
					if(!(w=malloc(sizeof(CAMWRK))))goto out;
					memset(w,0,sizeof(CAMWRK));
					w->fe=fe;
					w->next=list;
					list=w;
				}
				if(!(e=w->slot[slot]))
				{
					if(!(e=malloc(sizeof(CAMSLOT))))
						goto out;
					memset(e,0,sizeof(CAMSLOT));
					e->fe=fe;
					e->slot=slot;
					w->slot[slot]=e;
				}
				if(e->running)pthread_mutex_lock(&e->mtx);
				if(e->new)
				{
					for(i=0;i<e->new->total;i++)
						free(e->new->list[i]);
					free(e->new);
				}
				e->new=(PMTDATA *)dptr;
				e->newpmt=1;
				if(e->running)
				{
					dummy=1;
					dummy=write(e->nfd,&dummy,
						sizeof(dummy));
					pthread_mutex_unlock(&e->mtx);
				}
				type=0;
out:				if(type)free(dptr);
				break;

			case 1:	c=(CAMDATA *)dptr;
				for(w=list;w;w=w->next)
					if(w->fe==c->msg.deviceid)break;
				if(!w)
				{
					if(!(w=malloc(sizeof(CAMWRK))))goto out;
					memset(w,0,sizeof(CAMWRK));
					w->fe=c->msg.deviceid;
					w->next=list;
					list=w;
				}
				if(!(e=w->slot[c->msg.slot]))
				{
					if(!(e=malloc(sizeof(CAMSLOT))))
						goto out;
					memset(e,0,sizeof(CAMSLOT));
					e->fe=c->msg.deviceid;
					e->slot=c->msg.slot;
					w->slot[c->msg.slot]=e;
				}
				switch(c->msg.type)
				{
				case SATIP_CAM_STATE:
					if(c->msg.state==
					    (SATIP_CAM_AVAIL|SATIP_CAM_READY))
					{
						if(!startcam(e,c->handle))
							type=0;
					}
					else
					{
						stopcam(e);
						type=0;
					}
					free(dptr);
					break;

				case SATIP_CAM_READ:
					if(!e->running)goto out;
					pthread_mutex_lock(&e->mtx);
					if(e->fill<CAMQ)
					{
						e->fill++;
						e->queue[e->head++]=c;
						if(e->head==CAMQ)e->head=0;
						dummy=1;
						dummy=write(e->nfd,&dummy,
							sizeof(dummy));
						type=0;
					}
					else free(dptr);
					pthread_mutex_unlock(&e->mtx);
					break;
				}
				break;
			}

			dummy=1;
			dummy=write(afd,&dummy,sizeof(dummy));
		}
	}

	while(list)
	{
		w=list;
		list=w->next;
		for(i=0;i<SATIP_CAM_SLOTS;i++)if((e=w->slot[i]))
		{
			if(e->running)stopcam(e);
			if(e->new)
			{
				for(i=0;i<e->new->total;i++)
					free(e->new->list[i]);
				free(e->new);
			}
			if(e->curr)
			{
				for(i=0;i<e->curr->total;i++)
					free(e->curr->list[i]);
				free(e->curr);
			}
			while(e->fill--)
			{
				free(e->queue[e->tail++]);
				if(e->tail==CAMQ)e->tail=0;
			}
			free(e);
		}
		free(w);
	}

	pthread_exit(NULL);
}

static int global_init(void)
{
	pthread_mutex_lock(&mtx);
	if(!running++)
	{
		if((rfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err1;
		if((afd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err2;
		if((xfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err3;
		if(pthread_attr_init(&attr))goto err4;
		if(pthread_attr_setstacksize(&attr,STACKSIZE))goto err5;
		if(pthread_create(&th,&attr,manager,NULL))goto err5;
	}
	pthread_mutex_unlock(&mtx);
	return 0;

err5:	pthread_attr_destroy(&attr);
err4:	close(xfd);
err3:	close(afd);
err2:	close(rfd);
err1:	pthread_mutex_unlock(&mtx);
	return -1;
}

static void global_exit(void)
{
	uint64_t dummy=1;

	pthread_mutex_lock(&mtx);
	if(!(--running))
	{
		running=0;
		dummy=write(xfd,&dummy,sizeof(dummy));
		pthread_join(th,NULL);
		pthread_attr_destroy(&attr);
		close(xfd);
		close(afd);
		close(rfd);
	}
	pthread_mutex_unlock(&mtx);
}

static void addpid(PIDDATA *data,int pid)
{
	int i;

	for(i=0;i<data->set.numpids;i++)if(data->set.pids[i]==pid)return;
	if(data->set.numpids==SATIP_MAX_PIDS)return;
	data->set.pids[data->set.numpids++]=pid;
}

static PMT *pmtcopy(PIDDATA *data,int slot,SATIP_UTIL_PMT *pmt)
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
		if(data->conf[slot].pmtfilter&&data->conf[slot].catotal)
		{
			for(k=0;k<data->conf[slot].catotal;k++)
				if(data->conf[slot].caids[k]==pmt->caid[i])
					goto prglen;
			continue;
		}
prglen:		l+=satip_util_get_raw_len(
			satip_util_get_raw_pmt(pmt,pmt->raw[i]));
	}

	for(i=0;i<pmt->total;i++)for(j=0;j<pmt->data[i].catotal;j++)
	{
		if(data->conf[slot].pmtfilter&&data->conf[slot].catotal)
		{
			for(k=0;k<data->conf[slot].catotal;k++)
				if(data->conf[slot].caids[k]==
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
		if(data->conf[slot].pmtfilter&&data->conf[slot].catotal)
		{
			for(k=0;k<data->conf[slot].catotal;k++)
				if(data->conf[slot].caids[k]==pmt->caid[i])
					goto prgadd;
			continue;
		}
prgadd:		if(data->conf[slot].streampids)addpid(data,pmt->capid[i]);
		raw=satip_util_get_raw_pmt(pmt,pmt->raw[i]);
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
		if(data->conf[slot].pmtfilter&&data->conf[slot].catotal)
		{
			for(k=0;k<data->conf[slot].catotal;k++)
				if(data->conf[slot].caids[k]==
					pmt->data[i].caid[j])goto pesadd;
			continue;
		}
pesadd:		if(data->conf[slot].streampids)
			addpid(data,pmt->data[i].capid[j]);
		raw=satip_util_get_raw_pmt(pmt,pmt->data[i].raw[j]);
		l=satip_util_get_raw_len(raw);
		memcpy(ptr,raw,l);
		p->data[i].raw[p->data[i].catotal++]=ptr;
		ptr+=l;
	    }
	}

	return p;
}

static int newlist(PIDDATA *data,SATIP_UTIL_PMT **list,int *map,int total)
{
	int res=0;
	int i;
	int j;
	int k;
	struct pollfd p;
	uint64_t dummy=1;
	CONFIG *c;
	int dest[MAXPIDS];
	int sum[SATIP_CAM_SLOTS];
	PMTDATA *msg[SATIP_CAM_SLOTS];

	memset(msg,0,sizeof(msg));
	data->set.numpids=0;

	if(!total)goto send;

	memset(sum,0,sizeof(sum));
	memset(dest,0xff,sizeof(dest));

	for(i=0;i<total;i++)for(j=0;j<SATIP_CAM_SLOTS;j++)if(map[i]&(1<<j))
		if(sum[j]<data->conf[data->slotidx[j]].maxdecode)
	{
		sum[j]++;
		dest[i]=j;
		break;
	}

	for(i=0;i<SATIP_CAM_SLOTS;i++)if(sum[i])
	{
		if(!(msg[i]=malloc(sizeof(PMTDATA)+
			sum[i]*sizeof(msg[0]->list[0]))))
		{
			while(--i>=0)if(msg[i])free(msg[i]);
			return -1;
		}
		msg[i]->total=0;

		c=&data->conf[data->slotidx[i]];
		if(c->streampids&&data->cat)
		{
			addpid(data,1);
			for(j=0;j<data->cat->catotal;j++)
			{
				if(c->pmtfilter&&c->catotal)
				{
					for(k=0;k<c->catotal;k++)
					    if(data->cat->caid[j]==c->caids[k])
						goto addcat;
					continue;
				}
addcat:				addpid(data,data->cat->capid[j]);
			}
		}
	}

	for(i=0;i<total;i++)if(dest[i]!=-1)
	{
		if(!(msg[dest[i]]->list[msg[dest[i]]->total]=
			pmtcopy(data,data->slotidx[dest[i]],list[i])))
		{
			for(i=0;i<SATIP_CAM_SLOTS;i++)if(msg[i])
			{
				while(--(msg[i]->total)>=0)
					free(msg[i]->list[msg[i]->total]);
				free(msg[i]);
			}
			return -1;
		}
		msg[dest[i]]->total++;
	}

send:	data->plugin_pids(data->ctx,&data->set,0);
	for(i=0;i<SATIP_CAM_SLOTS;i++)if(data->cards&(1<<i))
	{
		pthread_mutex_lock(&mtx);
		type=0;
		fe=data->fe;
		slot=i;
		dptr=msg[i];
		dummy=write(rfd,&dummy,sizeof(dummy));
		p.fd=afd;
		p.events=POLLIN;
		while(poll(&p,1,-1)<=0||!(p.revents&POLLIN));
		dummy=read(afd,&dummy,sizeof(dummy));
		fe=0;
		if(type)res=-1;
		pthread_mutex_unlock(&mtx);
	}

	return res;
}

static int cafilter(PIDDATA *data,SATIP_UTIL_PMT *pmt)
{
	int i;
	int j;
	int k;
	int l;
	int map=0;
	int pref=0;

	for(l=0;l<data->cnum;l++)
	{
	    for(k=0;k<data->conf[l].igntotal;k++)
		if(pmt->prognum==data->conf[l].ignore[k])goto next;

	    if(!data->conf[l].catotal)goto hit;

	    for(i=0;i<data->conf[l].catotal;i++)
	    {
		for(k=0;k<pmt->catotal;k++)
			if(data->conf[l].caids[i]==pmt->caid[k])goto hit;
		for(j=0;j<pmt->total;j++)for(k=0;k<pmt->data[j].catotal;j++)
			if(data->conf[l].caids[i]==pmt->data[j].caid[k])
		{
hit:			map|=(1<<data->conf[l].slot);
			goto next;
		}
	    }
next:;
	}

	if(map)
	{
		for(l=0;l<data->cnum;l++)
			for(k=0;k<data->conf[l].preftotal;k++)
				if(pmt->prognum==data->conf[l].prefer[k])
		{
			pref|=(1<<data->conf[l].slot);
			break;
		}
		if(map&pref)map&=pref;
	}

	return map;
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
	if(!data->res[0])
	{
		data->res[0]=pat;
		pthread_spin_unlock(&data->stx);
		dummy=write(data->efd,&dummy,sizeof(dummy));
		return;
	}
	pthread_spin_unlock(&data->stx);

out:	free(pat);
}

static void catcb(void *dta,int len,void *priv)
{
	PIDDATA *data=(PIDDATA *)priv;
	SATIP_UTIL_CAT *cat;
	uint64_t dummy=1;

	if(UNLIKELY(((unsigned char *)dta)[0]!=0x01))return;
	if(UNLIKELY(!(cat=satip_util_unpack_cat_section(dta,len))))return;
	if(UNLIKELY(!cat->cnind||cat->secnum))goto out;

	pthread_spin_lock(&data->stx);
	if(!data->res[1])
	{
		data->res[1]=cat;
		pthread_spin_unlock(&data->stx);
		dummy=write(data->efd,&dummy,sizeof(dummy));
		return;
	}
	pthread_spin_unlock(&data->stx);

out:	free(cat);
}

static void pmtcb(void *dta,int len,void *priv)
{
	int i;
	int idx;
	PIDDATA *data=(PIDDATA *)priv;
	SATIP_UTIL_PAT *pat=(SATIP_UTIL_PAT *)data->res[0];
	SATIP_UTIL_PMT *pmt;
	uint64_t dummy=1;

	if(UNLIKELY(((unsigned char *)dta)[0]!=0x02))return;
	if(UNLIKELY(!(pmt=satip_util_unpack_pmt_section(dta,len))))return;
	if(UNLIKELY(!pmt->cnind||pmt->secnum))goto out;

	pthread_spin_lock(&data->stx);
	if((idx=data->pmtidx[pmt->prognum]))
		if(!data->res[idx])for(i=0;i<pat->total;i++)
			if(pmt->prognum==pat->data[i].prognum)
	{
		pat->data[i].pmt=pmt;
		data->res[idx]=pmt;
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
	int m;
	int state=0;
	int totcnt=0;
	int pmttot=0;
	int tmo=-1;
	int total;
	uint64_t dummy;
	SATIP_UTIL_PMT *list[MAXPIDS];
	int map[MAXPIDS];
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
				for(i=1;i<pmttot;i++)if(data->flt[i])
				{
					satip_util_filter_del_user(data->filter,
						data->flt[i]);
					data->flt[i]=NULL;
					satip_util_section_free(data->sec[i]);
					data->sec[i]=NULL;
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
			if(!data->res[0])
			{
				pthread_spin_unlock(&data->stx);
				break;
			}
			pthread_spin_unlock(&data->stx);
			satip_util_filter_del_user(data->filter,data->flt[0]);
			data->flt[0]=NULL;
			satip_util_section_free(data->sec[0]);
			data->sec[0]=NULL;
			pat=(SATIP_UTIL_PAT *)data->res[0];
			data->set.numpids=1;
			data->set.pids[0]=1;
			memset(data->pmtidx,0,sizeof(data->pmtidx));
			memset(&flt,0,sizeof(flt));
			flt.mask[0]=0xff;
			flt.filter[0]=0x02;
			for(i=2,j=0,totcnt=1;i<SATIP_MAX_PIDS&&j<pat->total;j++)
				if(pat->data[j].prognum)
			{
				data->pmtidx[pat->data[j].prognum]=i;
				for(k=2;k<i;k++)
					if(data->fpid[k]==pat->data[j].pmtpid)
						break;
				if(k<i)
				{
					if(satip_util_user_addpid(data->flt[k],
						pat->data[j].pmtpid))continue;
					totcnt++;
					continue;
				}

				if(satip_util_section_create(&data->sec[i],
					&flt,1,pmtcb,data))continue;

				if(satip_util_filter_add_user(data->filter,
					&data->flt[i],
					satip_util_section_packet,data->sec[i]))
				{
					satip_util_section_free(data->sec[i]);
					data->sec[i]=NULL;
					continue;
				}

				if(satip_util_user_addpid(data->flt[i],
					pat->data[j].pmtpid))
				{
					satip_util_filter_del_user(data->filter,
						data->flt[i]);
					data->flt[i]=NULL;
					satip_util_section_free(data->sec[i]);
					data->sec[i]=NULL;
					continue;
				}

				data->set.pids[data->set.numpids++]=
					pat->data[j].pmtpid;
				data->fpid[i++]=pat->data[j].pmtpid;
				totcnt++;
			}
			pmttot=i;
			data->plugin_pids(data->ctx,&data->set,0);
			tmo=2000;
			state=1;

		case 1:	pthread_spin_lock(&data->stx);
			if(data->res[1]&&data->flt[1])
			{
				pthread_spin_unlock(&data->stx);
				satip_util_filter_del_user(data->filter,
					data->flt[1]);
				data->flt[1]=NULL;
				satip_util_section_free(data->sec[1]);
				data->sec[1]=NULL;
				totcnt--;
			}
			else pthread_spin_unlock(&data->stx);

			for(i=2;i<pmttot;i++)
			{
				pthread_spin_lock(&data->stx);
				if(!data->res[i]||!data->pmtidx[
				   ((SATIP_UTIL_PMT *)(data->res[i]))->prognum])
				{
					pthread_spin_unlock(&data->stx);
					continue;
				}
				data->pmtidx[((SATIP_UTIL_PMT *)(data->res[i]))
					->prognum]=0;
				pthread_spin_unlock(&data->stx);
				totcnt--;
			}

			if(totcnt)break;

			for(i=2;i<pmttot;i++)
			{
				satip_util_filter_del_user(data->filter,
					data->flt[i]);
				data->flt[i]=NULL;
				satip_util_section_free(data->sec[i]);
				data->sec[i]=NULL;
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
			data->cat=data->res[1];
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
				if(!(m=cafilter(data,pmt)))goto fail;
				if(total!=MAXPIDS)
				{
					map[total]=m;
					list[total++]=pmt;
				}
fail:;
			}
			pthread_mutex_unlock(&data->mtx);
			if(data->total!=total||
				memcmp(data->list,list,total*sizeof(list[0]))||
				memcmp(data->map,map,total*sizeof(map[0])))
					if(!newlist(data,list,map,total))
			{
				memcpy(data->list,list,total*sizeof(list[0]));
				memcpy(data->map,map,total*sizeof(map[0]));
				data->total=total;
			}
			break;
		}
	}

out:	pthread_exit(NULL);
}

void *plugin_init(char *config,void *filter_ctx,
	void (*plugin_pids)(void *ctx,SATIP_PIDS *set,int from_notify_hook),
	void *ctx)
{
	PIDDATA *data;

	if(!(data=malloc(sizeof(PIDDATA))))goto err1;
	memset(data,0,sizeof(PIDDATA));
	if(parse_config(config,data))goto err2;
	if(global_init())goto err2;
	data->plugin_pids=plugin_pids;
	data->ctx=ctx;
	data->filter=filter_ctx;
	if(pthread_spin_init(&data->stx,PTHREAD_PROCESS_PRIVATE))goto err2;
	if(pthread_mutex_init(&data->mtx,NULL))goto err3;
	if((data->efd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err4;
	return data;

err4:	pthread_mutex_destroy(&data->mtx);
err3:	pthread_spin_destroy(&data->stx);
err2:	if(data->tunefilter)free(data->tunefilter);
	free(data);
err1:	return NULL;
}

void plugin_exit(void *plugin_ctx)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	int i;
	uint64_t dummy=1;

	if(!running||!data)return;

	if(data->running)
	{
		data->term=1;
		dummy=write(data->efd,&dummy,sizeof(dummy));
		pthread_cancel(data->th);
		pthread_join(data->th,NULL);
	}
	for(i=0;i<SATIP_MAX_PIDS;i++)
	{
		if(data->flt[i])
			satip_util_filter_del_user(data->filter,data->flt[i]);
		if(data->sec[i])satip_util_section_free(data->sec[i]);
	}
	newlist(data,NULL,NULL,0);
	global_exit();
	for(i=0;i<SATIP_MAX_PIDS;i++)if(data->res[i])free(data->res[i]);
	close(data->efd);
	pthread_mutex_destroy(&data->mtx);
	pthread_spin_destroy(&data->stx);
	if(data->tunefilter)free(data->tunefilter);
	free(data);
}

void plugin_notify_pids(void *plugin_ctx,short *pids,int total)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	int i;
	uint64_t dummy=1;

	if(!running||!data)return;

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

	if(!running||!data)return;

	memset(&flt,0,sizeof(flt));
	flt.mask[0]=0xff;

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

	flt.filter[0]=0x00;
	if(satip_util_section_create(&data->sec[0],&flt,1,patcb,data))goto err1;

	if(satip_util_filter_add_user(data->filter,&data->flt[0],
		satip_util_section_packet,data->sec[0]))goto err2;

	if(satip_util_user_addpid(data->flt[0],0))goto err3;

	flt.filter[0]=0x01;
	if(satip_util_section_create(&data->sec[1],&flt,1,catcb,data))goto err3;

	if(satip_util_filter_add_user(data->filter,&data->flt[1],
		satip_util_section_packet,data->sec[1]))goto err4;

	if(satip_util_user_addpid(data->flt[1],1))goto err5;

	data->plugin_pids(data->ctx,&data->set,0);

	if(data->total)
	{
		newlist(data,NULL,NULL,0);
		data->total=0;
	}
	data->term=0;
	data->fe=0;
	data->pat=NULL;
	dummy=read(data->efd,&dummy,sizeof(dummy));
	return;

err5:	satip_util_filter_del_user(data->filter,data->flt[1]);
	data->flt[1]=NULL;
err4:	satip_util_section_free(data->sec[1]);
	data->sec[1]=NULL;
err3:	satip_util_filter_del_user(data->filter,data->flt[0]);
	data->flt[0]=NULL;
err2:	satip_util_section_free(data->sec[0]);
	data->sec[0]=NULL;
err1:	data->skip=1;
	return;
}

void plugin_post_tune(void *plugin_ctx,SATIP_TUNE *tune,int success)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;

	if(!running||!data||data->skip)return;

	if(!success||pthread_create(&data->th,&attr,pidworker,data))
	{
		satip_util_filter_del_user(data->filter,data->flt[1]);
		data->flt[1]=NULL;
		satip_util_section_free(data->sec[1]);
		data->sec[1]=NULL;
		if(data->res[1])free(data->res[1]);
		data->res[1]=NULL;
		satip_util_filter_del_user(data->filter,data->flt[0]);
		data->flt[0]=NULL;
		satip_util_section_free(data->sec[0]);
		data->sec[0]=NULL;
		if(data->res[0])free(data->res[0]);
		data->res[0]=NULL;
		data->set.numpids=0;
		data->plugin_pids(data->ctx,&data->set,0);
		return;
	}
	data->running=1;
	data->fe=tune->fe;
}

void plugin_no_tune(void *plugin_ctx)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	int i;
	uint64_t dummy=1;

	if(!running||!data)return;

	if(data->running)
	{
		data->term=1;
		dummy=write(data->efd,&dummy,sizeof(dummy));
		pthread_join(data->th,NULL);
		data->running=0;
	}
	for(i=0;i<SATIP_MAX_PIDS;i++)
	{
		if(data->flt[i])
		{
			satip_util_filter_del_user(data->filter,data->flt[i]);
			data->flt[i]=NULL;
		}
		if(data->sec[i])
		{
			satip_util_section_free(data->sec[i]);
			data->sec[i]=NULL;
		}
	}

	if(data->total)newlist(data,NULL,NULL,0);
	else
	{
		data->set.numpids=0;
		data->plugin_pids(data->ctx,&data->set,0);
	}

	for(i=0;i<SATIP_MAX_PIDS;i++)if(data->res[i])
	{
		free(data->res[i]);
		data->res[i]=NULL;
	}

	data->term=0;
	data->fe=0;
	data->total=0;
	data->pat=NULL;
}

void plugin_cam(void *plugin_ctx,void *handle,SATIP_HW_CAM_IO *msg)
{
	PIDDATA *data=(PIDDATA *)plugin_ctx;
	CAMDATA *m;
	struct pollfd p[2];
	uint64_t dummy=1;

	if(!running||!data)return;

	if(!(m=malloc(sizeof(CAMDATA)+msg->len)))return;
	m->handle=handle;
	m->msg=*msg;
	if(msg->len)memcpy(m->msg.data,msg->data,msg->len);

	pthread_mutex_lock(&mtx);
	type=1;
	dptr=m;
	dummy=write(rfd,&dummy,sizeof(dummy));
	p[0].fd=xfd;
	p[0].events=POLLIN;
	p[1].fd=afd;
	p[1].events=POLLIN;
	while(1)
	{
		if(poll(p,2,-1)<=0)continue;
		if(p[0].revents&(POLLIN||POLLNVAL))goto out;
		if(p[1].revents&POLLIN)break;
	}
	dummy=read(afd,&dummy,sizeof(dummy));
out:	pthread_mutex_unlock(&mtx);
}
