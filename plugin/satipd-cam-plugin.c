/*
 * example plugin for SAT>IP DVB server
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
#define MAX_CONFIGS	64
#define MAX_DEVICES	32
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
	int deviceid;
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
	int deviceid;
	int cnum;
	int cards;
	unsigned char slotidx[SATIP_CAM_SLOTS];
	CONFIG *conf[SATIP_CAM_SLOTS];
} DEVICE;

typedef struct _pmt
{
	struct _pmt *next;
	void *id;
	unsigned short prognum;
	unsigned short total;
	int catotal;
	int alloc;
	unsigned char *raw[SATIP_PMTCA_MAX];
	struct
	{
		unsigned short pid;
		unsigned short type;
		int catotal;
		unsigned char *raw[SATIP_PMTCA_MAX];
	} data[0];
} PMT;

typedef struct _pmtlist
{
	struct _pmtlist *next;
	CONFIG *conf;
	PMT *list;
	int deviceid;
} PMTLIST;

typedef struct _piditem
{
	struct _piditem *next;
	void *id;
	SATIP_PIDS set;
} PIDITEM;

typedef struct _auxlist
{
	struct _auxlist *next;
	PIDITEM *list;
	void *handle;
	int deviceid;
} AUXLIST;

typedef struct
{
	int rfd;
	int afd;
	int xfd;
	int type;
	int fe;
	int slot;
	int cnum;
	int dnum;
	int filtertotal;
	void *dptr;
	void *hw;
	void *id;
	AUXLIST *aux;
	PMTLIST *list[SATIP_CAM_SLOTS];
	SATIP_TUNE *tunefilter;
	pthread_attr_t attr;
	pthread_t th;
	pthread_mutex_t mtx;
	pthread_mutex_t auxmtx;
	CONFIG conf[MAX_CONFIGS];
	DEVICE dev[MAX_DEVICES];
} GLOBAL;

typedef struct
{
	GLOBAL *g;
	DEVICE *dev;
	SATIP_UTIL_PAT *pat;
	SATIP_UTIL_CAT *cat;
	void *handle[SATIP_MAX_PIDS];
	void *res[SATIP_MAX_PIDS];
	SATIP_UTIL_PMT *list[MAXPIDS];
	int map[MAXPIDS];
	short fpid[SATIP_MAX_PIDS];
	SATIP_TUNE tune;
	SATIP_PIDS set;
	SATIP_PIDS streamset;
	pthread_mutex_t mtx;
	pthread_spinlock_t stx;
	pthread_t th;
	int running:1;
	int term:1;
	int skip:1;
	int total;
	int efd;
	unsigned char pmtidx[65536];
} PIDDATA;

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


static void stream(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void nostat(void *id,SATIP_STATUS *status) __attribute__ ((hot));

static int docaids(void *s,char *val) __attribute__ ((cold));
static int doprefign(void *s,char *val) __attribute__ ((cold));
static int doparam(CFGFILE *p,void *s,char *name,char *val)
	__attribute__ ((cold));
static int parse_config(char *fn,GLOBAL *g) __attribute__ ((cold));
void plugin_init(void *hwhandle,void **globalctx,char *config)
	 __attribute__ ((cold));
void plugin_fini(void *hwhandle,void *globalctx) __attribute__ ((cold));

#else

static int docaids(void *s,char *val);
static int doprefign(void *s,char *val);

#endif

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
	{"deviceid",1,65535,offsetof(CONFIG,deviceid),NULL},
	{"slot",1,8,offsetof(CONFIG,slot),NULL},
	{"caids",0,0,offsetof(CONFIG,caids),docaids},
	{"maxdecode",1,8,offsetof(CONFIG,maxdecode),NULL},
	{"pmtfilter",0,1,offsetof(CONFIG,pmtfilter),NULL},
	{"streampids",0,1,offsetof(CONFIG,streampids),NULL},
	{"prefer",0,0,offsetof(CONFIG,prefer),doprefign},
	{"ignore",0,0,offsetof(CONFIG,ignore),doprefign},
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
	else if(c1->slot<c2->slot)return -1;
	else if(c1->slot>c2->slot)return 1;
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

		if(!strcmp(name,"[card]"))
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
					SATIP_IGNCAPS,val+1,NULL,NULL,0,NULL,
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
		fprintf(stderr,"No card(s) defined.\n");
		goto err2;
	}

	qsort(g->conf,g->cnum,sizeof(CONFIG),confcmp);

	for(i=0,j=0;i<g->cnum;i++)
	{
		if(!g->conf[i].deviceid||!g->conf[i].slot||
			!g->conf[i].maxdecode)goto err4;
		g->conf[i].slot-=1;
		if(i&&g->conf[i-1].deviceid!=g->conf[i].deviceid)
			if(++j==MAX_DEVICES)
		{
			fprintf(stderr,"Out of memory\n");
			goto err2;
		}
		if(!g->dev[j].cnum)g->dev[j].deviceid=g->conf[i].deviceid;
		if(g->dev[j].cards&(1<<g->conf[i].slot))goto err4;
		g->dev[j].cards|=(1<<g->conf[i].slot);
		if(g->cnum==SATIP_CAM_SLOTS)
		{
			fprintf(stderr,"Out of memory\n");
			goto err2;
		}
		g->dev[j].slotidx[g->conf[i].slot]=g->dev[j].cnum;
		g->dev[j].conf[g->dev[j].cnum++]=&g->conf[i];
	}
	g->dnum=j+1;

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

static int startcam(CAMSLOT *e,void *handle,pthread_attr_t *attr)
{
	if(e->running)return 0;
	e->handle=handle;
	if((e->nfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err1;
	if((e->xfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err2;
	if((e->tfd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))
		==-1)goto err3;
	if(pthread_mutex_init(&e->mtx,NULL))goto err4;
	if(pthread_create(&e->th,attr,camwrk,e))goto err5;
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

static PMTDATA *pmtprocess(GLOBAL *g,PMTDATA *p,void *id,int deviceid,int slot)
{
	int i;
	int n;
	PMTLIST *e;
	PMT *m;
	PMT **d;
	PMTDATA *t;

	for(e=g->list[slot];e;e=e->next)if(e->deviceid==deviceid)break;
	if(!e)
	{
		if(!(e=malloc(sizeof(PMTLIST))))goto fail;
		e->next=g->list[slot];
		e->deviceid=deviceid;
		for(i=0;i<g->dnum;i++)if(g->dev[i].deviceid==deviceid)
		{
			e->conf=g->dev[i].conf[g->dev[i].slotidx[slot]];
			goto ready;
		}
		free(e);
fail:		if(p)
		{
			for(i=0;i<p->total;i++)free(p->list[i]);
			free(p);
		}
		return NULL;
ready:		g->list[slot]=e;
	}

	if(!p)
	{
		for(d=&e->list;*d;)if((*d)->id==id)
		{
			m=*d;
			*d=m->next;
			free(m);
		}
		else d=&(*d)->next;
	}
	else
	{
		for(i=0;i<p->total;i++)
		{
			p->list[i]->id=id;
			for(d=&e->list;*d;d=&(*d)->next)
				if((*d)->prognum==p->list[i]->prognum)
			{
				m=*d;
				p->list[i]->next=m->next;
				*d=p->list[i];
				free(m);
				break;
			}
			if(!*d)
			{
				p->list[i]->next=NULL;
				*d=p->list[i];
			}
		}
		free(p);
	}

	for(n=0,m=e->list;m&&n<e->conf->maxdecode;m=m->next,n++);

	if(!n)return NULL;

	if(!(t=malloc(sizeof(sizeof(PMTDATA)+n*sizeof(p->list[0])))))
		return NULL;

	for(t->total=0,m=e->list;m&&t->total<n;m=m->next)
	{
		for(i=0;i<t->total;i++)if(m->prognum==t->list[i]->prognum)break;
		if(i==t->total)t->list[t->total++]=m;
	}

	for(i=0;i<t->total;i++)
	{
		if(!(m=malloc(t->list[i]->alloc)))
		{
			while(--i>=0)free(t->list[i]);
			free(t);
			return NULL;
		}
		memcpy(m,t->list[i],t->list[i]->alloc);
		t->list[i]=m;
	}

	return t;
}

static void *manager(void *glob)
{
	int i;
	GLOBAL *g=(GLOBAL *)glob;
	CAMWRK *list=NULL;
	CAMWRK *w;
	CAMSLOT *e;
	CAMDATA *c;
	PMTLIST *l;
	PMT *d;
	struct pollfd p[2];
	uint64_t dummy;
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_SETMASK,&set,NULL);

	pthread_setname_np(pthread_self(),"plugin manager");

	p[0].fd=g->xfd;
	p[0].events=POLLIN;
	p[1].fd=g->rfd;
	p[1].events=POLLIN;

	while(1)
	{
		if(poll(p,2,-1)<=0)continue;

		if(p[0].revents&POLLIN)break;

		if(p[1].revents&POLLIN)
		{
			dummy=read(g->rfd,&dummy,sizeof(dummy));

			switch(g->type)
			{
			case 0:	g->type=1;
				if(!g->fe)goto out;
				for(w=list;w;w=w->next)
					if(w->fe==g->fe)break;
				if(!w)
				{
					if(!(w=malloc(sizeof(CAMWRK))))goto out;
					memset(w,0,sizeof(CAMWRK));
					w->fe=g->fe;
					w->next=list;
					list=w;
				}
				if(!(e=w->slot[g->slot]))
				{
					if(!(e=malloc(sizeof(CAMSLOT))))
						goto out;
					memset(e,0,sizeof(CAMSLOT));
					e->fe=g->fe;
					e->slot=g->slot;
					w->slot[g->slot]=e;
				}
				if(e->running)pthread_mutex_lock(&e->mtx);
				if(e->new)
				{
					for(i=0;i<e->new->total;i++)
						free(e->new->list[i]);
					free(e->new);
				}
				e->new=pmtprocess(g,(PMTDATA *)g->dptr,g->id,
					g->fe,g->slot);
				e->newpmt=1;
				if(e->running)
				{
					dummy=1;
					dummy=write(e->nfd,&dummy,
						sizeof(dummy));
					pthread_mutex_unlock(&e->mtx);
				}
				g->type=0;
out:				if(g->type)free(g->dptr);
				break;

			case 1:	c=(CAMDATA *)g->dptr;
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
						if(!startcam(e,c->handle,
							&g->attr))g->type=0;
					}
					else
					{
						stopcam(e);
						g->type=0;
					}
					free(g->dptr);
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
						g->type=0;
					}
					else free(g->dptr);
					pthread_mutex_unlock(&e->mtx);
					break;
				}
				break;
			}

			dummy=1;
			dummy=write(g->afd,&dummy,sizeof(dummy));
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

	for(i=0;i<SATIP_CAM_SLOTS;i++)while(g->list[i])
	{
		l=g->list[i];
		g->list[i]=l->next;
		while(l->list)
		{
			d=l->list;
			l->list=d->next;
			free(d);
		}
		free(l);
	}

	pthread_exit(NULL);
}

static void stream(void *id,SATIP_STREAM *stream)
{
	int i;
	int idx;
	uint64_t dummy=1;
	PIDDATA *data=(PIDDATA *)id;
	SATIP_UTIL_PAT *pat=(SATIP_UTIL_PAT *)data->res[0];
	union
	{
		SATIP_UTIL_PAT *pat;
		SATIP_UTIL_CAT *cat;
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
		if(!data->res[0])
		{
			data->res[0]=u.pat;
			pthread_spin_unlock(&data->stx);
			dummy=write(data->efd,&dummy,sizeof(dummy));
			return;
		}
		pthread_spin_unlock(&data->stx);
nopat:		free(u.pat);
		return;

	case 0x01:
		if(!(u.cat=satip_util_unpack_cat_section(stream->section,
			stream->fill)))break;
		if(!u.cat->cnind||u.cat->secnum)goto nocat;
		if(!data->res[1])
		{
			data->res[1]=u.cat;
			pthread_spin_unlock(&data->stx);
			dummy=write(data->efd,&dummy,sizeof(dummy));
			return;
		}
		pthread_spin_unlock(&data->stx);
nocat:		free(u.cat);
		return;

	case 0x02:
		if(!(u.pmt=satip_util_unpack_pmt_section(stream->section,
			stream->fill)))break;
		if(!u.pmt->cnind||u.pmt->secnum)goto nopmt;
		pthread_spin_lock(&data->stx);
		if((idx=data->pmtidx[u.pmt->prognum]))
			if(!data->res[idx])for(i=0;i<pat->total;i++)
				if(u.pmt->prognum==pat->data[i].prognum)
		{
			pat->data[i].pmt=u.pmt;
			data->res[idx]=u.pmt;
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

static void *hwplay(PIDDATA *data,SATIP_PIDS *set)
{
	SATIP_STRDATA s;

	s.tune=&data->tune;
	s.set=set;
	s.terminate=data->g->xfd;
	s.handle=data;

	if(satip_hw_play(data->g->hw,&s,stream,nostat,NULL,SATIP_HW_LOCAL))
		return NULL;
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

static void auxprocess(PIDDATA *data,SATIP_PIDS *newset)
{
	int i;
	int j;
	AUXLIST *a;
	PIDITEM *e;
	PIDITEM **d;
	SATIP_PIDS set;

	pthread_mutex_lock(&data->g->auxmtx);

	for(a=data->g->aux;a;a=a->next)if(a->deviceid==data->tune.fe)break;
	if(!a)
	{
		if(!(a=malloc(sizeof(AUXLIST))))goto out;
		a->list=NULL;
		a->handle=NULL;
		a->deviceid=data->tune.fe;
		a->next=data->g->aux;
		data->g->aux=a;
	}

	for(d=&a->list;*d;d=&(*d)->next)if((*d)->id==data)break;
	if(*d)
	{
		if(!newset)
		{
			e=*d;
			*d=e->next;
			free(e);
		}
		else (*d)->set=*newset;
	}
	else if(newset)
	{
		if(!(e=malloc(sizeof(PIDITEM))))goto out;
		e->next=a->list;
		a->list=e;
		e->id=data;
		e->set=*newset;
	}

	for(e=a->list,set.numpids=0;e&&set.numpids<SATIP_MAX_PIDS;e=e->next)
		for(i=0;i<e->set.numpids&&set.numpids<SATIP_MAX_PIDS;i++)
	{
		for(j=0;j<set.numpids;j++)if(e->set.pids[i]==set.pids[j])break;
		if(j==set.numpids)set.pids[set.numpids++]=e->set.pids[i];
	}

	if(set.numpids||a->handle)
	{
		if(!set.numpids)
		{
			hwend(data,a->handle);
			a->handle=NULL;
		}
		else if(a->handle)hwpids(data,a->handle,&set);
		else a->handle=hwplay(data,&set);
	}

out:	pthread_mutex_unlock(&data->g->auxmtx);
}

static void addpid(PIDDATA *data,int pid)
{
	int i;

	for(i=0;i<data->streamset.numpids;i++)
		if(data->streamset.pids[i]==pid)return;
	if(data->streamset.numpids==SATIP_MAX_PIDS)return;
	data->streamset.pids[data->streamset.numpids++]=pid;
}

static PMT *pmtcopy(PIDDATA *data,int slot,SATIP_UTIL_PMT *pmt)
{
	int i;
	int j;
	int k;
	int l;
	PMT *p;
	CONFIG *c=data->dev->conf[slot];
	unsigned char *ptr;
	unsigned char *raw;

	for(i=0,l=0;i<pmt->catotal;i++)
	{
		if(c->pmtfilter&&c->catotal)
		{
			for(k=0;k<c->catotal;k++)
				if(c->caids[k]==pmt->caid[i])goto prglen;
			continue;
		}
prglen:		l+=satip_util_get_raw_len(
			satip_util_get_raw_pmt(pmt,pmt->raw[i]));
	}

	for(i=0;i<pmt->total;i++)for(j=0;j<pmt->data[i].catotal;j++)
	{
		if(c->pmtfilter&&c->catotal)
		{
			for(k=0;k<c->catotal;k++)
				if(c->caids[k]==pmt->data[i].caid[j])
					goto peslen;
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
	p->alloc=sizeof(PMT)+pmt->total*sizeof(p->data[0])+l;

	for(i=0;i<pmt->catotal;i++)
	{
		if(c->pmtfilter&&c->catotal)
		{
			for(k=0;k<c->catotal;k++)
				if(c->caids[k]==pmt->caid[i])goto prgadd;
			continue;
		}
prgadd:		if(c->streampids)addpid(data,pmt->capid[i]);
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
		if(c->pmtfilter&&c->catotal)
		{
			for(k=0;k<c->catotal;k++)
				if(c->caids[k]==pmt->data[i].caid[j])
					goto pesadd;
			continue;
		}
pesadd:		if(c->streampids)addpid(data,pmt->data[i].capid[j]);
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
	data->streamset.numpids=0;

	if(!total)goto send;

	memset(sum,0,sizeof(sum));
	memset(dest,0xff,sizeof(dest));

	for(i=0;i<total;i++)for(j=0;j<SATIP_CAM_SLOTS;j++)if(map[i]&(1<<j))
		if(sum[j]<data->dev->conf[data->dev->slotidx[j]]->maxdecode)
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

		c=data->dev->conf[data->dev->slotidx[i]];
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
			pmtcopy(data,data->dev->slotidx[dest[i]],list[i])))
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

send:	auxprocess(data,data->streamset.numpids?&data->streamset:NULL);

	for(i=0;i<SATIP_CAM_SLOTS;i++)if(data->dev->cards&(1<<i))
	{
		pthread_mutex_lock(&data->g->mtx);
		data->g->type=0;
		data->g->fe=data->tune.fe;
		data->g->slot=i;
		data->g->id=data;
		data->g->dptr=msg[i];
		dummy=write(data->g->rfd,&dummy,sizeof(dummy));
		p.fd=data->g->afd;
		p.events=POLLIN;
		while(poll(&p,1,-1)<=0||!(p.revents&POLLIN));
		dummy=read(data->g->afd,&dummy,sizeof(dummy));
		data->g->fe=0;
		if(data->g->type)res=-1;
		pthread_mutex_unlock(&data->g->mtx);
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

	for(l=0;l<data->dev->cnum;l++)
	{
	    for(k=0;k<data->dev->conf[l]->igntotal;k++)
		if(pmt->prognum==data->dev->conf[l]->ignore[k])goto next;

	    if(!data->dev->conf[l]->catotal)goto hit;

	    for(i=0;i<data->dev->conf[l]->catotal;i++)
	    {
		for(k=0;k<pmt->catotal;k++)
			if(data->dev->conf[l]->caids[i]==pmt->caid[k])goto hit;
		for(j=0;j<pmt->total;j++)for(k=0;k<pmt->data[j].catotal;j++)
			if(data->dev->conf[l]->caids[i]==pmt->data[j].caid[k])
		{
hit:			map|=(1<<data->dev->conf[l]->slot);
			goto next;
		}
	    }
next:;
	}

	if(map)
	{
		for(l=0;l<data->dev->cnum;l++)
			for(k=0;k<data->dev->conf[l]->preftotal;k++)
				if(pmt->prognum==data->dev->conf[l]->prefer[k])
		{
			pref|=(1<<data->dev->conf[l]->slot);
			break;
		}
		if(map&pref)map&=pref;
	}

	return map;
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
	SATIP_PIDS set;
	int map[MAXPIDS];
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
				for(i=1;i<pmttot;i++)if(data->handle[i])
				{
					hwend(data,data->handle[i]);
					data->handle[i]=NULL;
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
			hwend(data,data->handle[0]);
			data->handle[0]=NULL;
			pat=(SATIP_UTIL_PAT *)data->res[0];
			memset(data->pmtidx,0,sizeof(data->pmtidx));
			for(i=2,j=0,totcnt=1;i<SATIP_MAX_PIDS&&j<pat->total;j++)
				if(pat->data[j].prognum)
			{
				pthread_spin_lock(&data->stx);
				data->pmtidx[pat->data[j].prognum]=i;
				pthread_spin_unlock(&data->stx);

				for(k=2;k<i;k++)
					if(data->fpid[k]==pat->data[j].pmtpid)
						break;
				if(k<i)
				{
					totcnt++;
					continue;
				}

				set.pid=pat->data[j].pmtpid;
				if(!(data->handle[i]=hwplay(data,&set)))
					continue;

				data->fpid[i++]=pat->data[j].pmtpid;
				totcnt++;
			}
			pmttot=i;
			tmo=2000;
			state=1;

		case 1:	pthread_spin_lock(&data->stx);
			if(data->res[1]&&data->handle[1])
			{
				pthread_spin_unlock(&data->stx);
				hwend(data,data->handle[1]);
				data->handle[1]=NULL;
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
				hwend(data,data->handle[i]);
				data->handle[i]=NULL;
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
			data->cat=data->res[1];
			state=3;
			pthread_mutex_unlock(&data->mtx);

		case 3:	pthread_mutex_lock(&data->mtx);
			for(total=0,i=0;i<pat->total;i++)if(pat->data[i].pmt)
			{
				pmt=pat->data[i].pmt;
				for(j=0;j<pmt->total;j++)
				{
				    for(k=0;k<data->set.numpids;k++)
				    {
					if(data->set.pids[k]==pmt->data[j].pid)
						goto hit;
					if(data->set.pids[k]>pmt->data[j].pid)
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

void plugin_init(void *hwhandle,void **globalctx,char *config)
{
	GLOBAL *g;

	if(!(g=malloc(sizeof(GLOBAL))))goto err1;
	memset(g,0,sizeof(GLOBAL));
	g->hw=hwhandle;

	if(parse_config(config,g))goto err2;

	if(pthread_mutex_init(&g->mtx,NULL))goto err2;
	if(pthread_mutex_init(&g->auxmtx,NULL))goto err3;
	if((g->rfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err4;
	if((g->afd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err5;
	if((g->xfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err6;
	if(pthread_attr_init(&g->attr))goto err7;
	if(pthread_attr_setstacksize(&g->attr,STACKSIZE))goto err8;
	if(pthread_create(&g->th,&g->attr,manager,g))goto err8;

	*globalctx=g;
	return;

err8:	pthread_attr_destroy(&g->attr);
err7:	close(g->xfd);
err6:	close(g->afd);
err5:	close(g->rfd);
err4:	pthread_mutex_destroy(&g->auxmtx);
err3:	pthread_mutex_destroy(&g->mtx);
err2:	if(g->tunefilter)free(g->tunefilter);
	free(g);
err1:	*globalctx=NULL;
}

void plugin_fini(void *hwhandle,void *globalctx)
{
	GLOBAL *g=(GLOBAL *)globalctx;
	AUXLIST *a;
	PIDITEM *e;
	uint64_t dummy=1;

	if(!g)return;
	dummy=write(g->xfd,&dummy,sizeof(dummy));
	pthread_join(g->th,NULL);
	while(g->aux)
	{
		a=g->aux;
		g->aux=a->next;
		if(a->handle)hwend((PIDDATA *)&g,a->handle);
		while(a->list)
		{
			e=a->list;
			a->list=e->next;
			free(e);
		}
		free(a);
	}
	pthread_attr_destroy(&g->attr);
	close(g->xfd);
	close(g->afd);
	close(g->rfd);
	pthread_mutex_destroy(&g->auxmtx);
	pthread_mutex_destroy(&g->mtx);
	if(g->tunefilter)free(g->tunefilter);
	free(g);
}

void plugin_prepare(void *hwhandle,void *globalctx,void **streamctx,
	SATIP_TUNE *tune,SATIP_PIDS *set)
{
	int i;
	GLOBAL *g=(GLOBAL *)globalctx;
	PIDDATA *data;

	if(!g)goto err1;

	if(g->filtertotal)
	{
		for(i=0;i<g->filtertotal;i++)
			if(!satip_util_tunecmp(tune,&g->tunefilter[i]))
				goto work;
		goto err1;
	}

work:	if(!(data=malloc(sizeof(PIDDATA))))goto err1;
	memset(data,0,sizeof(PIDDATA));
	if(pthread_spin_init(&data->stx,PTHREAD_PROCESS_PRIVATE))goto err2;
	if(pthread_mutex_init(&data->mtx,NULL))goto err3;
	if((data->efd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err4;
	data->g=g;

	*streamctx=data;
	return;

err4:	pthread_mutex_destroy(&data->mtx);
err3:	pthread_spin_destroy(&data->stx);
err2:	free(data);
err1:	*streamctx=NULL;
}

void plugin_play(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_TUNE *tune,SATIP_PIDS *set,int success)
{
	int i;
	GLOBAL *g=(GLOBAL *)globalctx;
	PIDDATA *data=(PIDDATA *)streamctx;
	SATIP_PIDS secset;

	if(!g||!data||!success)return;

	for(i=0;i<g->dnum;i++)if(g->dev[i].deviceid==tune->fe)goto work;
	return;

work:	data->dev=&g->dev[i];
	data->tune=*tune;
	data->set=*set;
	data->set.prognum=0;
	if(data->set.numpids<0)data->set.numpids=0;
	else if(data->set.numpids>0)qsort(data->set.pids,data->set.numpids,
		sizeof(short),pidcmp);
	for(i=0;i<data->set.numpids;i++)if(data->set.pids[i]>=0x20)break;
	if(i)
	{
		memmove(data->set.pids,&data->set.pids[i],
			(data->set.numpids-i)*sizeof(short));
		data->set.numpids-=i;
	}
	satip_util_init_pids(&data->streamset);

	satip_util_init_pids(&secset);

	secset.numpids=SATIP_SECTION;
	secset.pid=0;
	secset.table=0;
	if(!(data->handle[0]=hwplay(data,&secset)))goto err1;

	secset.pid=1;
	secset.table=1;
	if(!(data->handle[1]=hwplay(data,&secset)))goto err2;

	if(pthread_create(&data->th,&g->attr,pidworker,data))goto err3;

	data->running=1;
	return;

err3:	hwend(data,data->handle[1]);
	data->handle[1]=NULL;
err2:	hwend(data,data->handle[0]);
	data->handle[0]=NULL;
err1:	return;
}

void plugin_end(void *hwhandle,void *globalctx,void *streamctx)
{
	int i;
	uint64_t dummy=1;
	GLOBAL *g=(GLOBAL *)globalctx;
	PIDDATA *data=(PIDDATA *)streamctx;

	if(!g||!data)return;

	if(data->running)
	{
		data->term=1;
		dummy=write(data->efd,&dummy,sizeof(dummy));
		pthread_join(data->th,NULL);

		for(i=0;i<SATIP_MAX_PIDS;i++)
			if(data->handle[i])hwend(data,data->handle[i]);

		if(data->total)newlist(data,NULL,NULL,0);

		for(i=0;i<SATIP_MAX_PIDS;i++)
			if(data->res[i])free(data->res[i]);
	}

	close(data->efd);
	pthread_mutex_destroy(&data->mtx);
	pthread_spin_destroy(&data->stx);
	free(data);
}

void plugin_setpids(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_PIDS *set)
{
	int i;
	uint64_t dummy=1;
	GLOBAL *g=(GLOBAL *)globalctx;
	PIDDATA *data=(PIDDATA *)streamctx;

	if(!g||!data)return;

	pthread_mutex_lock(&data->mtx);
	data->set=*set;
	data->set.prognum=0;
	if(data->set.numpids<0)data->set.numpids=0;
	else if(data->set.numpids>0)qsort(data->set.pids,data->set.numpids,
		sizeof(short),pidcmp);
	for(i=0;i<data->set.numpids;i++)if(data->set.pids[i]>=0x20)break;
	if(i)
	{
		memmove(data->set.pids,&data->set.pids[i],
			(data->set.numpids-i)*sizeof(short));
		data->set.numpids-=i;
	}
	if(data->pat)dummy=write(data->efd,&dummy,sizeof(dummy));
	pthread_mutex_unlock(&data->mtx);
}

void plugin_cam(void *hwhandle,void *globalctx,SATIP_HW_CAM_IO *msg)
{
	GLOBAL *g=(GLOBAL *)globalctx;
	CAMDATA *m;
	struct pollfd p[2];
	uint64_t dummy=1;

	if(!hwhandle||!g)return;

	if(!(m=malloc(sizeof(CAMDATA)+msg->len)))return;
	m->handle=hwhandle;
	m->msg=*msg;
	if(msg->len)memcpy(m->msg.data,msg->data,msg->len);

	pthread_mutex_lock(&g->mtx);
	g->type=1;
	g->dptr=m;
	dummy=write(g->rfd,&dummy,sizeof(dummy));
	p[0].fd=g->xfd;
	p[0].events=POLLIN;
	p[1].fd=g->afd;
	p[1].events=POLLIN;
	while(1)
	{
		if(poll(p,2,-1)<=0)continue;
		if(p[0].revents&(POLLIN||POLLNVAL))goto out;
		if(p[1].revents&POLLIN)break;
	}
	dummy=read(g->afd,&dummy,sizeof(dummy));
out:	pthread_mutex_unlock(&g->mtx);
}
