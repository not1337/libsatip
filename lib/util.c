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
#include <sys/times.h>
#include <arpa/inet.h>
#include <limits.h>
#include <iconv.h>
#include <wchar.h>
#include <wctype.h>
#include <locale.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "satip.h"
#include "common.h"

typedef struct _entry
{
	struct _entry *next;
	ADDR addr;
	ADDR mask;
} ENTRY;


typedef struct
{
	pthread_spinlock_t mtx;
	int total;
	ENTRY *list;
} LIST;

typedef struct _user
{
	struct _user *next;
	unsigned long long map[128];
	void (*cb)(void *data,int len,void *priv);
	void *priv;
	pthread_spinlock_t mtx;
	int all;
} USER;

typedef struct
{
	pthread_spinlock_t mtx;
	USER *user;
} FILTER;

typedef struct
{
	int fill;
	int init;
	int len;
	int cont;
	int dup;
	int crc;
	int flen;
	int neq;
	void (*cb)(void *data,int len,void *priv);
	void *priv;
	unsigned char flt[SATIP_FILTERSZ];
	unsigned char mm[SATIP_FILTERSZ];
	unsigned char mn[SATIP_FILTERSZ];
	unsigned char bfr[4286];
} SECTION;

typedef struct
{
	locale_t locale;
	iconv_t ic;
} PROGSORT;

static uint32_t crctab[256]=
{
	0x00000000,0x04c11db7,0x09823b6e,0x0d4326d9,
	0x130476dc,0x17c56b6b,0x1a864db2,0x1e475005,
	0x2608edb8,0x22c9f00f,0x2f8ad6d6,0x2b4bcb61,
	0x350c9b64,0x31cd86d3,0x3c8ea00a,0x384fbdbd,
	0x4c11db70,0x48d0c6c7,0x4593e01e,0x4152fda9,
	0x5f15adac,0x5bd4b01b,0x569796c2,0x52568b75,
	0x6a1936c8,0x6ed82b7f,0x639b0da6,0x675a1011,
	0x791d4014,0x7ddc5da3,0x709f7b7a,0x745e66cd,
	0x9823b6e0,0x9ce2ab57,0x91a18d8e,0x95609039,
	0x8b27c03c,0x8fe6dd8b,0x82a5fb52,0x8664e6e5,
	0xbe2b5b58,0xbaea46ef,0xb7a96036,0xb3687d81,
	0xad2f2d84,0xa9ee3033,0xa4ad16ea,0xa06c0b5d,
	0xd4326d90,0xd0f37027,0xddb056fe,0xd9714b49,
	0xc7361b4c,0xc3f706fb,0xceb42022,0xca753d95,
	0xf23a8028,0xf6fb9d9f,0xfbb8bb46,0xff79a6f1,
	0xe13ef6f4,0xe5ffeb43,0xe8bccd9a,0xec7dd02d,
	0x34867077,0x30476dc0,0x3d044b19,0x39c556ae,
	0x278206ab,0x23431b1c,0x2e003dc5,0x2ac12072,
	0x128e9dcf,0x164f8078,0x1b0ca6a1,0x1fcdbb16,
	0x018aeb13,0x054bf6a4,0x0808d07d,0x0cc9cdca,
	0x7897ab07,0x7c56b6b0,0x71159069,0x75d48dde,
	0x6b93dddb,0x6f52c06c,0x6211e6b5,0x66d0fb02,
	0x5e9f46bf,0x5a5e5b08,0x571d7dd1,0x53dc6066,
	0x4d9b3063,0x495a2dd4,0x44190b0d,0x40d816ba,
	0xaca5c697,0xa864db20,0xa527fdf9,0xa1e6e04e,
	0xbfa1b04b,0xbb60adfc,0xb6238b25,0xb2e29692,
	0x8aad2b2f,0x8e6c3698,0x832f1041,0x87ee0df6,
	0x99a95df3,0x9d684044,0x902b669d,0x94ea7b2a,
	0xe0b41de7,0xe4750050,0xe9362689,0xedf73b3e,
	0xf3b06b3b,0xf771768c,0xfa325055,0xfef34de2,
	0xc6bcf05f,0xc27dede8,0xcf3ecb31,0xcbffd686,
	0xd5b88683,0xd1799b34,0xdc3abded,0xd8fba05a,
	0x690ce0ee,0x6dcdfd59,0x608edb80,0x644fc637,
	0x7a089632,0x7ec98b85,0x738aad5c,0x774bb0eb,
	0x4f040d56,0x4bc510e1,0x46863638,0x42472b8f,
	0x5c007b8a,0x58c1663d,0x558240e4,0x51435d53,
	0x251d3b9e,0x21dc2629,0x2c9f00f0,0x285e1d47,
	0x36194d42,0x32d850f5,0x3f9b762c,0x3b5a6b9b,
	0x0315d626,0x07d4cb91,0x0a97ed48,0x0e56f0ff,
	0x1011a0fa,0x14d0bd4d,0x19939b94,0x1d528623,
	0xf12f560e,0xf5ee4bb9,0xf8ad6d60,0xfc6c70d7,
	0xe22b20d2,0xe6ea3d65,0xeba91bbc,0xef68060b,
	0xd727bbb6,0xd3e6a601,0xdea580d8,0xda649d6f,
	0xc423cd6a,0xc0e2d0dd,0xcda1f604,0xc960ebb3,
	0xbd3e8d7e,0xb9ff90c9,0xb4bcb610,0xb07daba7,
	0xae3afba2,0xaafbe615,0xa7b8c0cc,0xa379dd7b,
	0x9b3660c6,0x9ff77d71,0x92b45ba8,0x9675461f,
	0x8832161a,0x8cf30bad,0x81b02d74,0x857130c3,
	0x5d8a9099,0x594b8d2e,0x5408abf7,0x50c9b640,
	0x4e8ee645,0x4a4ffbf2,0x470cdd2b,0x43cdc09c,
	0x7b827d21,0x7f436096,0x7200464f,0x76c15bf8,
	0x68860bfd,0x6c47164a,0x61043093,0x65c52d24,
	0x119b4be9,0x155a565e,0x18197087,0x1cd86d30,
	0x029f3d35,0x065e2082,0x0b1d065b,0x0fdc1bec,
	0x3793a651,0x3352bbe6,0x3e119d3f,0x3ad08088,
	0x2497d08d,0x2056cd3a,0x2d15ebe3,0x29d4f654,
	0xc5a92679,0xc1683bce,0xcc2b1d17,0xc8ea00a0,
	0xd6ad50a5,0xd26c4d12,0xdf2f6bcb,0xdbee767c,
	0xe3a1cbc1,0xe760d676,0xea23f0af,0xeee2ed18,
	0xf0a5bd1d,0xf464a0aa,0xf9278673,0xfde69bc4,
	0x89b8fd09,0x8d79e0be,0x803ac667,0x84fbdbd0,
	0x9abc8bd5,0x9e7d9662,0x933eb0bb,0x97ffad0c,
	0xafb010b1,0xab710d06,0xa6322bdf,0xa2f33668,
	0xbcb4666d,0xb8757bda,0xb5365d03,0xb1f740b4
};

#ifndef PROFILE

static void section_finish(SECTION *s) __attribute__ ((hot));
void satip_util_section_packet(void *data,int len,void *section)
	__attribute__ ((hot));
int satip_util_filter_packets(void *filter,void *data,int len)
	__attribute__ ((hot));
int satip_util_filter_packets_cb(void *filter,void *data,int len,
	void (*next)(void *priv),void *priv) __attribute__ ((hot));
SATIP_UTIL_PAT *satip_util_unpack_pat_section(unsigned char *data,int len)
	__attribute__ ((hot));
SATIP_UTIL_CAT *satip_util_unpack_cat_section(unsigned char *data,int len)
	__attribute__ ((hot));
SATIP_UTIL_PMT *satip_util_unpack_pmt_section(unsigned char *data,int len)
	__attribute__ ((hot));
SATIP_UTIL_NIT *satip_util_unpack_nit_section(unsigned char *data,int len)
	__attribute__ ((hot));
SATIP_UTIL_SDT *satip_util_unpack_sdt_section(unsigned char *data,int len)
	__attribute__ ((hot));

#endif

static int msys2type(int msys)
{
	switch(msys)
	{
	case SATIP_DVBS:
	case SATIP_DVBS2:
		return 0;
	case SATIP_DVBT:
	case SATIP_DVBT2:
		return 1;
	case SATIP_DVBC:
	case SATIP_DVBC2:
		return 2;
	default:return 3;
	}
}

static int streamsort(const void *p1,const void *p2)
{
	int r;
	int t1;
	int t2;
#pragma pack(push,1)
	const struct
	{
		unsigned int pmttype:4;
		unsigned int pid:13;
		char lang[4];
	} *stream[2];
#pragma pack(pop)

	stream[0]=p1;
	stream[1]=p2;

	t1=stream[0]->pmttype?stream[0]->pmttype:16;
	t2=stream[1]->pmttype?stream[1]->pmttype:16;
	if(t1<t2)return -1;
	if(t1>t2)return 1;
	if((r=strcmp(stream[0]->lang,stream[1]->lang)))return r;
	if(stream[0]->pid<stream[1]->pid)return -1;
	if(stream[0]->pid>stream[1]->pid)return 1;
	return 0;
}

static int dupsort(const void *p1,const void *p2)
{
	int i;
	int j;
	SATIP_UTIL_PROGRAM *e1=*((SATIP_UTIL_PROGRAM **)p1);
	SATIP_UTIL_PROGRAM *e2=*((SATIP_UTIL_PROGRAM **)p2);

	i=msys2type(e1->tune.msys);
	j=msys2type(e2->tune.msys);

	if(i<j)return -1;
	if(i>j)return 1;

	if(!i)
	{
		if(e1->tune.src<e2->tune.src)return -1;
		if(e1->tune.src>e2->tune.src)return 1;
		if(e1->tune.pol<e2->tune.pol)return -1;
		if(e1->tune.pol>e2->tune.pol)return 1;
	}

	if(e1->index<e2->index)return -1;
	if(e1->index>e2->index)return 1;

	if(e1->streams<e2->streams)return -1;
	if(e1->streams>e2->streams)return 1;

	for(i=0;i<e1->streams;i++)
	{
		if(e1->stream[i].pid<e2->stream[i].pid)return -1;
		if(e1->stream[i].pid>e2->stream[i].pid)return 1;
	}

	if(e1->prognum<e2->prognum)return -1;
	if(e1->prognum>e2->prognum)return 1;

	return 0;
}

static int mergesort(const void *p1,const void *p2)
{
	int i;
	int j;
	SATIP_UTIL_PROGRAM *e1=*((SATIP_UTIL_PROGRAM **)p1);
	SATIP_UTIL_PROGRAM *e2=*((SATIP_UTIL_PROGRAM **)p2);

	i=msys2type(e1->tune.msys);
	j=msys2type(e2->tune.msys);

	if(i<j)return -1;
	if(i>j)return 1;

	if(!i)
	{
		if(e1->tune.src<e2->tune.src)return -1;
		if(e1->tune.src>e2->tune.src)return 1;
		if(e1->tune.pol<e2->tune.pol)return -1;
		if(e1->tune.pol>e2->tune.pol)return 1;
	}

	if(e1->index<e2->index)return -1;
	if(e1->index>e2->index)return 1;

	if(e1->prognum<e2->prognum)return -1;
	if(e1->prognum>e2->prognum)return 1;

	return 0;
}

static int progsort(const void *p1,const void *p2,void *param)
{
	int i;
	int j;
	int r;
	size_t ilen;
	size_t olen;
	char *in;
	char *out;
	SATIP_UTIL_PROGRAM *e1=*((SATIP_UTIL_PROGRAM **)p1);
	SATIP_UTIL_PROGRAM *e2=*((SATIP_UTIL_PROGRAM **)p2);
	PROGSORT *p=(PROGSORT *)param;
	wchar_t n1[64];
	wchar_t n2[64];

	iconv(p->ic,NULL,NULL,NULL,NULL);
	in=e1->progname;
	out=(char *)n1;
	ilen=strlen(in)+1;
	olen=sizeof(n1);
	if(iconv(p->ic,&in,&ilen,&out,&olen)==-1)goto fail;
	iconv(p->ic,NULL,NULL,NULL,NULL);
	in=e2->progname;
	out=(char *)n2;
	ilen=strlen(in)+1;
	olen=sizeof(n2);
	if(iconv(p->ic,&in,&ilen,&out,&olen)==-1)goto fail;
	for(r=0;n1[r];r++)n1[r]=towlower_l(n1[r],p->locale);
	for(r=0;n2[r];r++)n2[r]=towlower_l(n2[r],p->locale);
	if((r=wcscoll_l(n1,n2,p->locale)))return r;
	goto num;

fail:	if((r=strcmp(e1->progname,e2->progname)))return r;

num:	i=msys2type(e1->tune.msys);
	j=msys2type(e2->tune.msys);

	if(i<j)return -1;
	if(i>j)return 1;

	if(e1->prognum<e2->prognum)return -1;
	if(e1->prognum>e2->prognum)return 1;

	if(e1->tune.freq<e2->tune.freq)return -1;
	if(e1->tune.freq>e2->tune.freq)return 1;

	return 0;
}

static void section_finish(SECTION *s)
{
	int i;
	int j;
	uint32_t crc;
	uint32_t ref;

	if(s->crc)
	{
		crc=0xffffffff;
		ref=s->bfr[s->len-4];
		ref<<=8;
		ref|=s->bfr[s->len-3];
		ref<<=8;
		ref|=s->bfr[s->len-2];
		ref<<=8;
		ref|=s->bfr[s->len-1];
		for(i=0;i<s->len-4;i++)
			crc=(crc<<8)^crctab[((crc>>24)^s->bfr[i])&0xff];
		if(crc!=ref)return;
	}

	if(s->flen)
	{
		if(s->len<s->flen+(s->flen>1?2:0))return;
		for(i=0,j=0;i<s->flen;i++,j++)
		{
			if(j==1)j=3;
			if((s->bfr[j]^s->flt[i])&s->mn[i])return;
			if(s->neq&&!((s->bfr[j]^s->flt[i])&s->mm[i]))return;
		}
	}

	s->cb(s->bfr,s->len,s->priv);
}

static int chrcpy(char *dst,size_t dlen,char *src,size_t slen)
{
	char *cset;
	iconv_t h;

	if(!src||!dst||slen<0||dlen<1)return -1;

	if(!slen)
	{
		*dst=0;
		return 0;
	}

	switch(*src)
	{
	case 0x01:
		cset="ISO8859-5";
		src++;
		slen--;
		break;
	case 0x02:
		cset="ISO8859-6";
		src++;
		slen--;
		break;
	case 0x03:
		cset="ISO8859-7";
		src++;
		slen--;
		break;
	case 0x04:
		cset="ISO8859-8";
		src++;
		slen--;
		break;
	case 0x05:
		cset="ISO8859-9";
		src++;
		slen--;
		break;
	case 0x06:
		cset="ISO8859-10";
		src++;
		slen--;
		break;
	case 0x07:
		cset="ISO8859-11";
		src++;
		slen--;
		break;
	case 0x09:
		cset="ISO8859-13";
		src++;
		slen--;
		break;
	case 0x0a:
		cset="ISO8859-14";
		src++;
		slen--;
		break;
	case 0x0b:
		cset="ISO8859-15";
		src++;
		slen--;
		break;
	case 0x10:
		src++;
		slen--;
		if(*src)return -1;
		src++;
		slen--;
		switch(*src)
		{
		case 0x01:
			cset="ISO8859-1";
			src++;
			slen--;
			break;
		case 0x02:
			cset="ISO8859-2";
			src++;
			slen--;
			break;
		case 0x03:
			cset="ISO8859-3";
			src++;
			slen--;
			break;
		case 0x04:
			cset="ISO8859-4";
			src++;
			slen--;
			break;
		case 0x05:
			cset="ISO8859-5";
			src++;
			slen--;
			break;
		case 0x06:
			cset="ISO8859-6";
			src++;
			slen--;
			break;
		case 0x07:
			cset="ISO8859-7";
			src++;
			slen--;
			break;
		case 0x08:
			cset="ISO8859-8";
			src++;
			slen--;
			break;
		case 0x09:
			cset="ISO8859-9";
			src++;
			slen--;
			break;
		case 0x0a:
			cset="ISO8859-10";
			src++;
			slen--;
			break;
		case 0x0b:
			cset="ISO8859-11";
			src++;
			slen--;
			break;
		case 0x0d:
			cset="ISO8859-13";
			src++;
			slen--;
			break;
		case 0x0e:
			cset="ISO8859-14";
			src++;
			slen--;
			break;
		case 0x0f:
			cset="ISO8859-15";
			src++;
			slen--;
			break;
		default:return -1;
		}
		break;
	case 0x11:
		cset="ISO-10646";
		src++;
		slen--;
		break;
	case 0x12:
		return -1;
	case 0x13:
		cset="GB2312";
		src++;
		slen--;
		break;
	case 0x14:
		cset="BIG-5";
		src++;
		slen--;
		break;
	case 0x15:
		cset="UTF-8";
		src++;
		slen--;
		break;
	case 0x1f:
		return -1;
		break;
	default:cset="ISO6937";
		break;
	case 0x00:
	case 0x08:
	case 0x0c:
	case 0x0d:
	case 0x0e:
	case 0x0f:
	case 0x16:
	case 0x17:
	case 0x18:
	case 0x19:
	case 0x1a:
	case 0x1b:
	case 0x1c:
	case 0x1d:
	case 0x1e:
		return -1;
	}

	if((h=iconv_open("UTF-8//TRANSLIT",cset))==(iconv_t)-1)return -1;

	dlen--;
	if(iconv(h,&src,&slen,&dst,&dlen)==-1)if(errno!=E2BIG)
	{
		iconv_close(h);
		return -1;
	}
	*dst=0;

	iconv_close(h);
	return 0;
}

static int csv2arr(char *csv,SATIP_PIDS *p)
{
	int j;
	int l;
	char *ptr;
	char *mem;
	char *end;

	for(p->numpids=0,ptr=strtok_r(csv,",",&mem);ptr;p->numpids++,
		ptr=strtok_r(NULL,",",&mem))
	{
		if(p->numpids==SATIP_MAX_PIDS)return -3;
		l=strtol(ptr,&end,10);
		if(*end||end==csv)return -1;
		if(l<0||l>8191)return -2;
		for(j=0;j<p->numpids;j++)if(p->pids[j]==l)return -1;
		p->pids[p->numpids]=(int)l;
	}

	if(!p->numpids)return -1;
	return 0;
}

static int pidcmp(const void *p1,const void *p2)
{
	short v1=*((short *)p1);
	short v2=*((short *)p2);
	return v1-v2;
}

static int parsequery(char *cmd,unsigned int caps,int flags,SATIP_TUNE *tune,
	SATIP_PIDS *set,SATIP_PIDS *add,SATIP_PIDS *del,SATIP_EXTRA *extra)
{
	int i;
	int j;
	int section=-1;
	int table=-1;
	int signal=-1;
	long l;
	double d;
	char *ptr;
	char *val;
	char *end;
	char *mem=NULL;

	memset(tune,0,sizeof(SATIP_TUNE));
	tune->plp=SATIP_UNDEF;
	tune->t2id=SATIP_UNDEF;
	tune->ds=SATIP_UNDEF;
	set->prognum=0;
	add->prognum=0;
	del->prognum=0;
	set->numpids=0;
	add->numpids=0;
	del->numpids=0;
	set->extra.bits=0;
	add->extra.bits=0;
	del->extra.bits=0;
	if(extra)extra->total=0;
	if(flags&SATIP_IGNCAPS)caps=-1;

	for(ptr=strtok_r(cmd,"&",&mem);ptr;ptr=strtok_r(NULL,"&",&mem))
	{
		if(!(val=strchr(ptr,'=')))continue;
		*val++=0;

		if(!strcmp(ptr,"fe"))
		{
			if(tune->fe)goto err1;
			l=strtol(val,&end,10);
			if(l<1||l>65535||*end||end==val)goto err2;
			tune->fe=(int)l;
		}
		else if(!strcmp(ptr,"src"))
		{
			if(tune->src)goto err1;
			l=strtol(val,&end,10);
			if(l<1||l>255||*end||end==val)goto err2;
			tune->src=(int)l;
		}
		else if(!strcmp(ptr,"freq"))
		{
			if(tune->freq)goto err1;
			d=strtod(val,&end);
			if(d<1||d>15000||*end||end==val)goto err2;
			tune->freq=(unsigned long long)(d*1000000.0);
		}
		else if(!strcmp(ptr,"pol"))
		{
			if(tune->pol)goto err1;
			else if(!strcmp(val,"h"))tune->pol=SATIP_POL_H;
			else if(!strcmp(val,"v"))tune->pol=SATIP_POL_V;
			else if(!strcmp(val,"l"))tune->pol=SATIP_POL_L;
			else if(!strcmp(val,"r"))tune->pol=SATIP_POL_R;
			else goto err2;
		}
		else if(!strcmp(ptr,"ro"))
		{
			if(tune->ro)goto err1;
			else if(!strcmp(val,"0.35"))tune->ro=SATIP_ROFF_035;
			else if(!strcmp(val,"0.25"))tune->ro=SATIP_ROFF_025;
			else if(!strcmp(val,"0.20"))tune->ro=SATIP_ROFF_020;
			else goto err2;
		}
		else if(!strcmp(ptr,"msys"))
		{
			if(tune->msys)goto err1;
			else if(!strcmp(val,"dvbs"))tune->msys=SATIP_DVBS;
			else if(!strcmp(val,"dvbs2"))tune->msys=SATIP_DVBS2;
			else if(!strcmp(val,"dvbt"))tune->msys=SATIP_DVBT;
			else if(!strcmp(val,"dvbt2"))tune->msys=SATIP_DVBT2;
			else if(!strcmp(val,"dvbc"))tune->msys=SATIP_DVBC;
			else if(!strcmp(val,"dvbc2"))tune->msys=SATIP_DVBC2;
			else goto err2;
		}
		else if(!strcmp(ptr,"mtype"))
		{
			if(tune->mtype)goto err1;
			else if(!strcmp(val,"qpsk"))tune->mtype=SATIP_QPSK;
			else if(!strcmp(val,"8psk"))tune->mtype=SATIP_8PSK;
			else if(!strcmp(val,"x_16apsk"))
				tune->mtype=SATIP_16APSK;
			else if(!strcmp(val,"x_32apsk"))
				tune->mtype=SATIP_32APSK;
			else if(!strcmp(val,"16qam"))tune->mtype=SATIP_16Q;
			else if(!strcmp(val,"64qam"))tune->mtype=SATIP_64Q;
			else if(!strcmp(val,"256qam"))tune->mtype=SATIP_256Q;
			else if(!strcmp(val,"32qam"))tune->mtype=SATIP_32Q;
			else if(!strcmp(val,"128qam"))tune->mtype=SATIP_128Q;
			else goto err2;
		}
		else if(!strcmp(ptr,"plts"))
		{
			if(tune->plts)goto err1;
			else if(!strcmp(val,"off"))tune->plts=SATIP_PLTS_OFF;
			else if(!strcmp(val,"on"))tune->plts=SATIP_PLTS_ON;
			else goto err2;
		}
		else if(!strcmp(ptr,"sr"))
		{
			if(tune->sr)goto err1;
			l=strtol(val,&end,10);
			if(l<1||l>INT_MAX/1000||*end||end==val)goto err2;
			tune->sr=(int)l*1000;
		}
		else if(!strcmp(ptr,"fec"))
		{
			if(tune->fec)goto err1;
			else if(!strcmp(val,"12"))tune->fec=SATIP_FEC_12;
			else if(!strcmp(val,"23"))tune->fec=SATIP_FEC_23;
			else if(!strcmp(val,"34"))tune->fec=SATIP_FEC_34;
			else if(!strcmp(val,"56"))tune->fec=SATIP_FEC_56;
			else if(!strcmp(val,"78"))tune->fec=SATIP_FEC_78;
			else if(!strcmp(val,"89"))tune->fec=SATIP_FEC_89;
			else if(!strcmp(val,"35"))tune->fec=SATIP_FEC_35;
			else if(!strcmp(val,"45"))tune->fec=SATIP_FEC_45;
			else if(!strcmp(val,"910"))tune->fec=SATIP_FEC_910;
			else goto err2;
		}
		else if(!strcmp(ptr,"x_feclp"))
		{
			if(tune->feclp)goto err1;
			else if(!strcmp(val,"12"))tune->feclp=SATIP_FEC_12;
			else if(!strcmp(val,"23"))tune->feclp=SATIP_FEC_23;
			else if(!strcmp(val,"34"))tune->feclp=SATIP_FEC_34;
			else if(!strcmp(val,"56"))tune->feclp=SATIP_FEC_56;
			else if(!strcmp(val,"78"))tune->feclp=SATIP_FEC_78;
			else if(!strcmp(val,"89"))tune->feclp=SATIP_FEC_89;
			else if(!strcmp(val,"35"))tune->feclp=SATIP_FEC_35;
			else if(!strcmp(val,"45"))tune->feclp=SATIP_FEC_45;
			else if(!strcmp(val,"910"))tune->feclp=SATIP_FEC_910;
			else goto err2;
		}
		else if(!strcmp(ptr,"x_hier"))
		{
			if(tune->hier)goto err1;
			else if(!strcmp(val,"0"))tune->feclp=SATIP_HIER_NONE;
			else if(!strcmp(val,"1"))tune->feclp=SATIP_HIER_1;
			else if(!strcmp(val,"2"))tune->feclp=SATIP_HIER_2;
			else if(!strcmp(val,"4"))tune->feclp=SATIP_HIER_4;
			else goto err2;
		}
		else if(!strcmp(ptr,"x_psi"))
		{
			if(set->numpids||add->numpids||del->numpids||
				set->prognum)goto err1;
			if(section!=-1)goto err1;
			l=strtol(val,&end,10);
			if(l<0||l>8191||*end||end==val)goto err2;
			section=(int)l;
		}
		else if(!strcmp(ptr,"x_table"))
		{
			if(set->numpids||add->numpids||del->numpids||
				set->prognum)goto err1;
			if(table!=-1)goto err1;
			l=strtol(val,&end,10);
			if(l<0||l>INT_MAX||*end||end==val)goto err2;
			table=(int)l;
		}
		else if(!strcmp(ptr,"x_signal"))
		{
			if(set->numpids||add->numpids||del->numpids||
				set->prognum)goto err1;
			if(signal!=-1)goto err1;
			l=strtol(val,&end,10);
			if(l!=1||*end||end==val)goto err2;
			signal=(int)l;
		}
		else if(!strcmp(ptr,"x_sid"))
		{
			if(section!=-1||table!=-1||signal!=-1)goto err1;
			if(set->numpids==SATIP_NOPIDS||
				set->numpids==SATIP_ALLPIDS||
				add->numpids||del->numpids)goto err1;
			if(set->prognum)goto err1;
			l=strtol(val,&end,10);
			if(l<1||l>65535||*end||end==val)goto err2;
			set->prognum=(int)l;
		}
		else if(!strcmp(ptr,"pids"))
		{
			if(set->numpids)goto err1;
			if(!(flags&SATIP_SADOK))if(add->numpids||del->numpids)
				goto err1;
			if(section!=-1||table!=-1||signal!=-1)goto err1;
			if(!strcmp(val,"all"))
			{
				if(set->prognum)goto err1;
				set->numpids=SATIP_ALLPIDS;
			}
			else if(!strcmp(val,"none"))
			{
				if(set->prognum)goto err1;
				set->numpids=SATIP_NOPIDS;
			}
			else switch(csv2arr(val,set))
			{
			case -1:goto err1;
			case -2:goto err2;
			case -3:goto err3;
			}
		}
		else if(!strcmp(ptr,"addpids"))
		{
			if(add->numpids)goto err1;
			if(!(flags&SATIP_SADOK))if(set->numpids||set->prognum)
				goto err1;
			if(section!=-1||table!=-1||signal!=-1)goto err1;
			switch(csv2arr(val,add))
			{
			case -1:goto err1;
			case -2:goto err2;
			case -3:goto err3;
			}
		}
		else if(!strcmp(ptr,"delpids"))
		{
			if(del->numpids)goto err1;
			if(!(flags&SATIP_SADOK))if(set->numpids||set->prognum)
				goto err1;
			if(section!=-1||table!=-1||signal!=-1)goto err1;
			switch(csv2arr(val,del))
			{
			case -1:goto err1;
			case -2:goto err2;
			case -3:goto err3;
			}
		}
		else if(!strcmp(ptr,"bw"))
		{
			if(tune->bw)goto err1;
			else if(!strcmp(val,"5"))tune->bw=SATIP_BW_5;
			else if(!strcmp(val,"6"))tune->bw=SATIP_BW_6;
			else if(!strcmp(val,"7"))tune->bw=SATIP_BW_7;
			else if(!strcmp(val,"8"))tune->bw=SATIP_BW_8;
			else if(!strcmp(val,"10"))tune->bw=SATIP_BW_10;
			else if(!strcmp(val,"1.712"))tune->bw=SATIP_BW_1712;
			else goto err2;
		}
		else if(!strcmp(ptr,"tmode"))
		{
			if(tune->tmode)goto err1;
			else if(!strcmp(val,"2k"))tune->tmode=SATIP_TMOD_2K;
			else if(!strcmp(val,"4k"))tune->tmode=SATIP_TMOD_4K;
			else if(!strcmp(val,"8k"))tune->tmode=SATIP_TMOD_8K;
			else if(!strcmp(val,"1k"))tune->tmode=SATIP_TMOD_1K;
			else if(!strcmp(val,"16k"))tune->tmode=SATIP_TMOD_16K;
			else if(!strcmp(val,"32k"))tune->tmode=SATIP_TMOD_32K;
			else goto err2;
		}
		else if(!strcmp(ptr,"gi"))
		{
			if(tune->gi)goto err1;
			else if(!strcmp(val,"14"))tune->gi=SATIP_GI_14;
			else if(!strcmp(val,"18"))tune->gi=SATIP_GI_18;
			else if(!strcmp(val,"116"))tune->gi=SATIP_GI_116;
			else if(!strcmp(val,"132"))tune->gi=SATIP_GI_132;
			else if(!strcmp(val,"1128"))tune->gi=SATIP_GI_1128;
			else if(!strcmp(val,"19128"))tune->gi=SATIP_GI_19128;
			else if(!strcmp(val,"19256"))tune->gi=SATIP_GI_19256;
			else goto err2;
		}
		else if(!strcmp(ptr,"plp"))
		{
			if(tune->plp!=SATIP_UNDEF)goto err1;
			l=strtol(val,&end,10);
			if(l<0||l>255||*end||end==val)goto err2;
			tune->plp=(int)l;
		}
		else if(!strcmp(ptr,"t2id"))
		{
			if(tune->t2id!=SATIP_UNDEF)goto err1;
			l=strtol(val,&end,10);
			if(l<0||l>65535||*end||end==val)goto err2;
			tune->t2id=(int)l;
		}
		else if(!strcmp(ptr,"sm"))
		{
			if(tune->sm)goto err1;
			else if(!strcmp(val,"0"))tune->sm=SATIP_SM_SISO;
			else if(!strcmp(val,"1"))tune->sm=SATIP_SM_MISO;
			else goto err2;
		}
		else if(!strcmp(ptr,"c2tft"))
		{
			if(tune->c2tft)goto err1;
			else if(!strcmp(val,"0"))tune->c2tft=SATIP_TFT_DS;
			else if(!strcmp(val,"1"))tune->c2tft=SATIP_TFT_C2;
			else if(!strcmp(val,"2"))tune->c2tft=SATIP_TFT_IT;
			else goto err2;
		}
		else if(!strcmp(ptr,"ds"))
		{
			if(tune->ds!=SATIP_UNDEF)goto err1;
			l=strtol(val,&end,10);
			if(l<0||l>255||*end||end==val)goto err2;
			tune->ds=(int)l;
		}
		else if(!strcmp(ptr,"specinv"))
		{
			if(tune->specinv)goto err1;
			else if(!strcmp(val,"0"))tune->specinv=SATIP_SPI_OFF;
			else if(!strcmp(val,"1"))tune->specinv=SATIP_SPI_ON;
			else goto err2;
		}
		else if(extra&&strlen(ptr)<sizeof(extra->name[0])
			&&strlen(val)<sizeof(extra->value[0])
			&&extra->total<SATIP_EXTRA_TOT)
		{
			strcpy(extra->name[extra->total],ptr);
			strcpy(extra->value[extra->total++],val);
		}
	}

	if(!tune->freq)goto err1;
	if(!set->numpids&&set->prognum)goto err1;
	if(!set->numpids&&!add->numpids&&!del->numpids)
	{
		if(signal!=-1)
		{
			if(section!=-1||table!=-1)goto err1;
			set->numpids=SATIP_SIGNAL;
		}
		else if(section==-1)goto err1;
		else
		{
			if(signal!=-1)goto err1;
			set->numpids=SATIP_SECTION;
			set->pid=section;
			set->table=(table==-1?256:table);
		}
	}
	else if(section!=-1||table!=-1||signal!=-1)goto err1;

	if(set->numpids>0)qsort(set->pids,set->numpids,sizeof(short),pidcmp);
	if(add->numpids>0)qsort(add->pids,add->numpids,sizeof(short),pidcmp);
	if(del->numpids>0)qsort(del->pids,del->numpids,sizeof(short),pidcmp);

	for(i=0,j=0;i<add->numpids&&j<del->numpids;)
	{
		if(add->pids[i]<del->pids[j])i++;
		else if(add->pids[i]>del->pids[j])j++;
		else goto err1;
	}

	switch(tune->msys)
	{
	case SATIP_DVBS2:
		if(!tune->ro&&(caps&SATIP_ROFF_AUTO))
			tune->ro=SATIP_ROFF_AUTO;
		if(!tune->mtype&&(caps&SATIP_AUTOQ))tune->mtype=SATIP_AUTOQ;
		if(!tune->plts&&(caps&SATIP_PLTS_AUTO))
			tune->plts=SATIP_PLTS_AUTO;
		if(!tune->ro||!tune->mtype||!tune->plts)goto err1;
	case SATIP_DVBS:
		if(!tune->fec&&(caps&SATIP_FEC_AUTO))
			tune->fec=SATIP_FEC_AUTO;
		if(!tune->pol||!tune->sr||!tune->fec)goto err1;
		if(tune->bw||tune->tmode||tune->gi||tune->sm||tune->c2tft||
			tune->specinv)goto err1;
		if(tune->plp!=SATIP_UNDEF||tune->t2id!=SATIP_UNDEF||
			tune->ds!=SATIP_UNDEF)goto err1;
		if(!tune->src)tune->src=1;
		if(tune->msys==SATIP_DVBS&&!tune->mtype)tune->mtype=SATIP_QPSK;
		switch(tune->mtype)
		{
		case SATIP_QPSK:
		case SATIP_8PSK:
		case SATIP_16APSK:
		case SATIP_32APSK:
		case SATIP_AUTOQ:
			break;
		default:goto err2;
		}
		break;

	case SATIP_DVBT2:
		if(!tune->sm&&(caps&SATIP_SM_AUTO))tune->sm=SATIP_SM_AUTO;
		if(tune->plp==SATIP_UNDEF||tune->t2id==SATIP_UNDEF||!tune->sm)
			goto err1;
	case SATIP_DVBT:
		if(!tune->bw&&(caps&SATIP_BW_AUTO))tune->bw=SATIP_BW_AUTO;
		if(!tune->tmode&&(caps&SATIP_TMOD_AUTO))
			tune->tmode=SATIP_TMOD_AUTO;
		if(!tune->mtype&&(caps&SATIP_AUTOQ))tune->mtype=SATIP_AUTOQ;
		if(!tune->gi&&(caps&SATIP_GI_AUTO))tune->gi=SATIP_GI_AUTO;
		if(!tune->fec&&(caps&SATIP_FEC_AUTO))
			tune->fec=SATIP_FEC_AUTO;
		if(!tune->feclp&&(caps&SATIP_FEC_AUTO))
			tune->feclp=SATIP_FEC_AUTO;
		if(!tune->hier&&(caps&SATIP_HIER_AUTO))
			tune->hier=SATIP_HIER_AUTO;
		if(!tune->bw||!tune->tmode||!tune->mtype||!tune->gi||
			!tune->fec||!tune->feclp||!tune->hier)goto err1;
		if(tune->src||tune->pol||tune->ro||tune->plts||tune->sr||
			tune->c2tft||tune->specinv)goto err1;
		if(tune->ds!=SATIP_UNDEF)goto err1;
		switch(tune->mtype)
		{
		case SATIP_QPSK:
		case SATIP_16Q:
		case SATIP_64Q:
		case SATIP_256Q:
		case SATIP_AUTOQ:
			break;
		default:goto err2;
		}
		switch(tune->fec)
		{
		case SATIP_FEC_12:
		case SATIP_FEC_35:
		case SATIP_FEC_23:
		case SATIP_FEC_34:
		case SATIP_FEC_45:
		case SATIP_FEC_56:
		case SATIP_FEC_78:
		case SATIP_FEC_AUTO:
			break;
		default:goto err2;
		}
		switch(tune->feclp)
		{
		case SATIP_FEC_12:
		case SATIP_FEC_35:
		case SATIP_FEC_23:
		case SATIP_FEC_34:
		case SATIP_FEC_45:
		case SATIP_FEC_56:
		case SATIP_FEC_78:
		case SATIP_FEC_AUTO:
			break;
		default:goto err2;
		}
		switch(tune->hier)
		{
		case SATIP_HIER_NONE:
		case SATIP_HIER_1:
		case SATIP_HIER_2:
		case SATIP_HIER_4:
		case SATIP_HIER_AUTO:
			break;
		default:goto err2;
		}
		break;

	case SATIP_DVBC2:
		if(!tune->c2tft&&(caps&SATIP_TFT_AUTO))
			tune->c2tft=SATIP_TFT_AUTO;
		if(!tune->bw&&(caps&SATIP_BW_AUTO))tune->bw=SATIP_BW_AUTO;
		if(!tune->c2tft||!tune->bw||tune->ds==SATIP_UNDEF||
			tune->plp==SATIP_UNDEF)goto err1;
		if(tune->src||tune->pol||tune->ro||tune->mtype||tune->sr||
			tune->fec||tune->tmode||tune->gi)goto err1;
		if(tune->t2id!=SATIP_UNDEF)goto err1;
		if(tune->sm||tune->specinv||tune->plts)goto err1;
		switch(tune->bw)
		{
		case SATIP_BW_6:
		case SATIP_BW_8:
		case SATIP_BW_AUTO:
			break;
		default:goto err2;
		}
		break;

	case SATIP_DVBC:
		if(!tune->mtype&&(caps&SATIP_AUTOQ))tune->mtype=SATIP_AUTOQ;
		if(!tune->specinv&&(caps&SATIP_SPI_AUTO))
			tune->specinv=SATIP_SPI_AUTO;
		if(!tune->mtype||!tune->sr||!tune->specinv)goto err1;
		if(tune->src||tune->pol||tune->ro||tune->plts||tune->fec||
			tune->bw||tune->tmode||tune->gi)goto err1;
		if(tune->plp!=SATIP_UNDEF||tune->t2id!=SATIP_UNDEF||
			tune->ds!=SATIP_UNDEF)goto err1;
		if(tune->sm||tune->c2tft)goto err1;
		switch(tune->mtype)
		{
		case SATIP_16Q:
		case SATIP_32Q:
		case SATIP_64Q:
		case SATIP_128Q:
		case SATIP_256Q:
		case SATIP_AUTOQ:
			break;
		default:goto err2;
		}
		break;

	default:goto err1;
	}

	return 0;

err3:	return SATIP_ERR_OVER;
err2:	return SATIP_ERR_VALUE;
err1:	return SATIP_ERR_SYNTX;
}

static int mkquery(int flags,SATIP_TUNE *t,SATIP_PIDS *p,SATIP_PIDS *a,
	SATIP_PIDS *d,SATIP_EXTRA *extra,char *bfr,int size)
{
	int len=0;
	int fh;
	int fl;
	int i;

	if(!t)return SATIP_ERR_SYNTX;
	if(p&&(a||d))return SATIP_ERR_SYNTX;
	if(extra&&extra->total>SATIP_EXTRA_TOT)return SATIP_ERR_SYNTX;

	switch(t->msys)
	{
	case SATIP_DVBS:
	case SATIP_DVBS2:
		if(t->fe>0&&t->fe<65536)len+=snprintf(bfr+len,size-len,"&fe=%d",
			t->fe);
		else if(t->fe!=SATIP_UNSPEC)return SATIP_ERR_SYNTX;
		if(t->src==SATIP_UNSPEC)
			len+=snprintf(bfr+len,size-len,"&src=1");
		else if(t->src>0&&t->src<256)len+=snprintf(bfr+len,size-len,
			"&src=%d",t->src);
		else return SATIP_ERR_SYNTX;
		if(t->freq==SATIP_UNSPEC)return SATIP_ERR_SYNTX;
		fh=t->freq/1000000;
		fl=t->freq%1000000;
		fl/=1000;
		len+=snprintf(bfr+len,size-len,"&freq=%d.%03d",fh,fl);
		switch(t->pol)
		{
		case SATIP_POL_H:
			len+=snprintf(bfr+len,size-len,"&pol=h");
			break;
		case SATIP_POL_V:
			len+=snprintf(bfr+len,size-len,"&pol=v");
			break;
		case SATIP_POL_L:
			len+=snprintf(bfr+len,size-len,"&pol=l");
			break;
		case SATIP_POL_R:
			len+=snprintf(bfr+len,size-len,"&pol=r");
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->ro)
		{
		case SATIP_ROFF_035:
			len+=snprintf(bfr+len,size-len,"&ro=0.35");
			break;
		case SATIP_ROFF_025:
			len+=snprintf(bfr+len,size-len,"&ro=0.25");
			break;
		case SATIP_ROFF_020:
			len+=snprintf(bfr+len,size-len,"&ro=0.20");
		case SATIP_ROFF_AUTO:
			break;
		default:if(!t->ro&&t->msys==SATIP_DVBS)
			{
				if(flags&SATIP_STRICTQRY)len+=snprintf(bfr+len,
					size-len,"&ro=0.35");
				break;
			}
			return SATIP_ERR_SYNTX;
		}
		switch(t->msys)
		{
		case SATIP_DVBS:
			len+=snprintf(bfr+len,size-len,"&msys=dvbs");
			break;
		case SATIP_DVBS2:
			len+=snprintf(bfr+len,size-len,"&msys=dvbs2");
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->mtype)
		{
		case SATIP_QPSK:
			len+=snprintf(bfr+len,size-len,"&mtype=qpsk");
			break;
		case SATIP_8PSK:
			len+=snprintf(bfr+len,size-len,"&mtype=8psk");
			break;
		case SATIP_16APSK:
			len+=snprintf(bfr+len,size-len,"&mtype=x_16apsk");
			break;
		case SATIP_32APSK:
			len+=snprintf(bfr+len,size-len,"&mtype=x_32apsk");
		case SATIP_AUTOQ:
			break;
		default:if(!t->mtype&&t->msys==SATIP_DVBS)
			{
				if(flags&SATIP_STRICTQRY)len+=snprintf(bfr+len,
					size-len,"&mtype=qpsk");
				break;
			}
			return SATIP_ERR_SYNTX;
		}
		switch(t->plts)
		{
		case SATIP_PLTS_OFF:
			len+=snprintf(bfr+len,size-len,"&plts=off");
			break;
		case SATIP_PLTS_ON:
			len+=snprintf(bfr+len,size-len,"&plts=on");
		case SATIP_PLTS_AUTO:
			break;
		default:if(!t->plts&&t->msys==SATIP_DVBS)
			{
				if(flags&SATIP_STRICTQRY)len+=snprintf(bfr+len,
					size-len,"&plts=off");
				break;
			}
			return SATIP_ERR_SYNTX;
		}
		if(t->sr<=SATIP_UNSPEC||t->sr%1000)return SATIP_ERR_SYNTX;
		len+=snprintf(bfr+len,size-len,"&sr=%d",t->sr/1000);
		switch(t->fec)
		{
		case SATIP_FEC_12:
			len+=snprintf(bfr+len,size-len,"&fec=12");
			break;
		case SATIP_FEC_23:
			len+=snprintf(bfr+len,size-len,"&fec=23");
			break;
		case SATIP_FEC_34:
			len+=snprintf(bfr+len,size-len,"&fec=34");
			break;
		case SATIP_FEC_35:
			len+=snprintf(bfr+len,size-len,"&fec=35");
			break;
		case SATIP_FEC_45:
			len+=snprintf(bfr+len,size-len,"&fec=45");
			break;
		case SATIP_FEC_56:
			len+=snprintf(bfr+len,size-len,"&fec=56");
			break;
		case SATIP_FEC_78:
			len+=snprintf(bfr+len,size-len,"&fec=78");
			break;
		case SATIP_FEC_89:
			len+=snprintf(bfr+len,size-len,"&fec=89");
			break;
		case SATIP_FEC_910:
			len+=snprintf(bfr+len,size-len,"&fec=910");
		case SATIP_FEC_AUTO:
			break;
		default:return SATIP_ERR_SYNTX;
		}
		break;

	case SATIP_DVBT:
	case SATIP_DVBT2:
		if(t->freq==SATIP_UNSPEC)return SATIP_ERR_SYNTX;
		fh=t->freq/1000000;
		fl=t->freq%1000000;
		fl/=1000;
		len+=snprintf(bfr+len,size-len,"&freq=%d.%03d",fh,fl);
		switch(t->bw)
		{
		case SATIP_BW_1712:
			len+=snprintf(bfr+len,size-len,"&bw=1.712");
			break;
		case SATIP_BW_5:
			len+=snprintf(bfr+len,size-len,"&bw=5");
			break;
		case SATIP_BW_6:
			len+=snprintf(bfr+len,size-len,"&bw=6");
			break;
		case SATIP_BW_7:
			len+=snprintf(bfr+len,size-len,"&bw=7");
			break;
		case SATIP_BW_8:
			len+=snprintf(bfr+len,size-len,"&bw=8");
			break;
		case SATIP_BW_10:
			len+=snprintf(bfr+len,size-len,"&bw=10");
		case SATIP_BW_AUTO:
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->msys)
		{
		case SATIP_DVBT:
			len+=snprintf(bfr+len,size-len,"&msys=dvbt");
			break;
		case SATIP_DVBT2:
			len+=snprintf(bfr+len,size-len,"&msys=dvbt2");
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->tmode)
		{
		case SATIP_TMOD_1K:
			len+=snprintf(bfr+len,size-len,"&tmode=1k");
			break;
		case SATIP_TMOD_2K:
			len+=snprintf(bfr+len,size-len,"&tmode=2k");
			break;
		case SATIP_TMOD_4K:
			len+=snprintf(bfr+len,size-len,"&tmode=4k");
			break;
		case SATIP_TMOD_8K:
			len+=snprintf(bfr+len,size-len,"&tmode=8k");
			break;
		case SATIP_TMOD_16K:
			len+=snprintf(bfr+len,size-len,"&tmode=16k");
			break;
		case SATIP_TMOD_32K:
			len+=snprintf(bfr+len,size-len,"&tmode=32k");
		case SATIP_TMOD_AUTO:
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->mtype)
		{
		case SATIP_QPSK:
			len+=snprintf(bfr+len,size-len,"&mtype=qpsk");
			break;
		case SATIP_16Q:
			len+=snprintf(bfr+len,size-len,"&mtype=16qam");
			break;
		case SATIP_64Q:
			len+=snprintf(bfr+len,size-len,"&mtype=64qam");
			break;
		case SATIP_256Q:
			len+=snprintf(bfr+len,size-len,"&mtype=256qam");
			break;
		case SATIP_AUTOQ:
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->gi)
		{
		case SATIP_GI_14:
			len+=snprintf(bfr+len,size-len,"&gi=14");
			break;
		case SATIP_GI_18:
			len+=snprintf(bfr+len,size-len,"&gi=18");
			break;
		case SATIP_GI_116:
			len+=snprintf(bfr+len,size-len,"&gi=116");
			break;
		case SATIP_GI_132:
			len+=snprintf(bfr+len,size-len,"&gi=132");
			break;
		case SATIP_GI_1128:
			len+=snprintf(bfr+len,size-len,"&gi=1128");
			break;
		case SATIP_GI_19128:
			len+=snprintf(bfr+len,size-len,"&gi=19128");
			break;
		case SATIP_GI_19256:
			len+=snprintf(bfr+len,size-len,"&gi=19256");
		case SATIP_GI_AUTO:
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->fec)
		{
		case SATIP_FEC_12:
			len+=snprintf(bfr+len,size-len,"&fec=12");
			break;
		case SATIP_FEC_23:
			len+=snprintf(bfr+len,size-len,"&fec=23");
			break;
		case SATIP_FEC_34:
			len+=snprintf(bfr+len,size-len,"&fec=34");
			break;
		case SATIP_FEC_35:
			len+=snprintf(bfr+len,size-len,"&fec=35");
			break;
		case SATIP_FEC_45:
			len+=snprintf(bfr+len,size-len,"&fec=45");
			break;
		case SATIP_FEC_56:
			len+=snprintf(bfr+len,size-len,"&fec=56");
			break;
		case SATIP_FEC_78:
			len+=snprintf(bfr+len,size-len,"&fec=78");
		case SATIP_FEC_AUTO:
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->feclp)
		{
		case SATIP_FEC_12:
			len+=snprintf(bfr+len,size-len,"&x_feclp=12");
			break;
		case SATIP_FEC_23:
			len+=snprintf(bfr+len,size-len,"&x_feclp=23");
			break;
		case SATIP_FEC_34:
			len+=snprintf(bfr+len,size-len,"&x_feclp=34");
			break;
		case SATIP_FEC_35:
			len+=snprintf(bfr+len,size-len,"&x_feclp=35");
			break;
		case SATIP_FEC_45:
			len+=snprintf(bfr+len,size-len,"&x_feclp=45");
			break;
		case SATIP_FEC_56:
			len+=snprintf(bfr+len,size-len,"&x_feclp=56");
			break;
		case SATIP_FEC_78:
			len+=snprintf(bfr+len,size-len,"&x_feclp=78");
		case SATIP_FEC_AUTO:
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->hier)
		{
		case SATIP_HIER_NONE:
			len+=snprintf(bfr+len,size-len,"&x_hier=0");
			break;
		case SATIP_HIER_1:
			len+=snprintf(bfr+len,size-len,"&x_hier=1");
			break;
		case SATIP_HIER_2:
			len+=snprintf(bfr+len,size-len,"&x_hier=2");
			break;
		case SATIP_HIER_4:
			len+=snprintf(bfr+len,size-len,"&x_hier=4");
			break;
		case SATIP_HIER_AUTO:
			break;
		default:return SATIP_ERR_SYNTX;
		}
		if(t->msys==SATIP_DVBT)break;
		if(t->plp>=0&&t->plp<=255)len+=snprintf(bfr+len,size-len,
			"&plp=%d",t->plp);
		else if(t->plp!=SATIP_UNDEF)return SATIP_ERR_SYNTX;
		if(t->t2id>=0&&t->t2id<=65535)len+=snprintf(bfr+len,size-len,
			"&t2id=%d",t->t2id);
		else if(t->t2id!=SATIP_UNDEF)return SATIP_ERR_SYNTX;
		switch(t->sm)
		{
		case SATIP_SM_SISO:
			len+=snprintf(bfr+len,size-len,"&sm=0");
			break;
		case SATIP_SM_MISO:
			len+=snprintf(bfr+len,size-len,"&sm=1");
		case SATIP_SM_AUTO:
			break;
		default:if(!t->sm&&t->msys==SATIP_DVBT)break;
			return SATIP_ERR_SYNTX;
		}
		break;

	case SATIP_DVBC:
	case SATIP_DVBC2:
		if(t->freq==SATIP_UNSPEC)return SATIP_ERR_SYNTX;
		fh=t->freq/1000000;
		fl=t->freq%1000000;
		fl/=1000;
		len+=snprintf(bfr+len,size-len,"&freq=%d.%03d",fh,fl);
		switch(t->c2tft)
		{
		case SATIP_TFT_DS:
			len+=snprintf(bfr+len,size-len,"&c2tft=0");
			break;
		case SATIP_TFT_C2:
			len+=snprintf(bfr+len,size-len,"&c2tft=1");
			break;
		case SATIP_TFT_IT:
			len+=snprintf(bfr+len,size-len,"&c2tft=2");
		case SATIP_TFT_AUTO:
			break;
		default:if(!t->c2tft&&t->msys==SATIP_DVBC)break;
			return SATIP_ERR_SYNTX;
		}
		switch(t->bw)
		{
		case SATIP_BW_6:
			len+=snprintf(bfr+len,size-len,"&bw=6");
			break;
		case SATIP_BW_8:
			len+=snprintf(bfr+len,size-len,"&bw=8");
		case SATIP_BW_AUTO:
			break;
		default:if(!t->bw&&t->msys==SATIP_DVBC)break;
			return SATIP_ERR_SYNTX;
		}
		switch(t->msys)
		{
		case SATIP_DVBC:
			len+=snprintf(bfr+len,size-len,"&msys=dvbc");
			break;
		case SATIP_DVBC2:
			len+=snprintf(bfr+len,size-len,"&msys=dvbc2");
			break;
		default:return SATIP_ERR_SYNTX;
		}
		switch(t->mtype)
		{
		case SATIP_16Q:
			len+=snprintf(bfr+len,size-len,"&mtype=16qam");
			break;
		case SATIP_32Q:
			len+=snprintf(bfr+len,size-len,"&mtype=32qam");
			break;
		case SATIP_64Q:
			len+=snprintf(bfr+len,size-len,"&mtype=64qam");
			break;
		case SATIP_128Q:
			len+=snprintf(bfr+len,size-len,"&mtype=128qam");
			break;
		case SATIP_256Q:
			len+=snprintf(bfr+len,size-len,"&mtype=256qam");
			break;
		case SATIP_AUTOQ:
			break;
		default:if(!t->mtype&&t->msys==SATIP_DVBC2)break;
			return SATIP_ERR_SYNTX;
		}
		if(t->sr<=SATIP_UNSPEC)
		{
			if(t->msys==SATIP_DVBC)return SATIP_ERR_SYNTX;
		}
		else len+=snprintf(bfr+len,size-len,"&sr=%d",t->sr);
		if(t->ds>=0&&t->ds<=255)len+=snprintf(bfr+len,size-len,
			"&ds=%d",t->ds);
		else if(!t->ds&&t->msys==SATIP_DVBC);
		else if(t->ds!=SATIP_UNDEF)return SATIP_ERR_SYNTX;
		if(t->plp>=0&&t->plp<=255)len+=snprintf(bfr+len,size-len,
			"&plp=%d",t->plp);
		else if(!t->plp&&t->msys==SATIP_DVBC);
		else if(t->plp!=SATIP_UNDEF)return SATIP_ERR_SYNTX;
		switch(t->specinv)
		{
		case SATIP_SPI_OFF:
			len+=snprintf(bfr+len,size-len,"&specinv=0");
			break;
		case SATIP_SPI_ON:
			len+=snprintf(bfr+len,size-len,"&specinv=1");
		case SATIP_SPI_AUTO:
			break;
		default:if(!t->specinv&&t->msys==SATIP_DVBC2)break;
			return SATIP_ERR_SYNTX;
		}
		break;

	default:return SATIP_ERR_SYNTX;
	}

	if(!p&&!a&&!d)len+=snprintf(bfr+len,size-len,"&pids=none");
	else if(p)switch(p->numpids)
	{
	case SATIP_SIGNAL:
		if(p->prognum)return SATIP_ERR_SYNTX;
		len+=snprintf(bfr+len,size-len,"&x_signal=1");
		break;
	case SATIP_SECTION:
		if(p->prognum)return SATIP_ERR_SYNTX;
		if(p->pid<0||p->pid>8191||p->table<0)return SATIP_ERR_SYNTX;
		len+=snprintf(bfr+len,size-len,"&x_psi=%d",p->pid);
		if(p->table>0)len+=snprintf(bfr+len,size-len,"&x_table=%d",
			p->table);
		break;
	case SATIP_NOPIDS:
		if(p->prognum)return SATIP_ERR_SYNTX;
		len+=snprintf(bfr+len,size-len,"&pids=none");
		break;
	case SATIP_ALLPIDS:
		if(p->prognum)return SATIP_ERR_SYNTX;
		len+=snprintf(bfr+len,size-len,"&pids=all");
		break;
	default:if(p->numpids<1||p->numpids>SATIP_MAX_PIDS)
			return SATIP_ERR_SYNTX;
		len+=snprintf(bfr+len,size-len,"&pids");
		for(i=0;i<p->numpids;i++)if(p->pids[i]<0||p->pids[i]>8191)
			return SATIP_ERR_SYNTX;
		else len+=snprintf(bfr+len,size-len,"%c%d",i?',':'=',
			p->pids[i]);
		if(p->prognum>0&&p->prognum<65536)
			len+=snprintf(bfr+len,size-len,"&x_sid=%d",p->prognum);
		else if(p->prognum)return SATIP_ERR_SYNTX;
		break;
	}
	else
	{
		if(a)
		{
			if(a->prognum)return SATIP_ERR_SYNTX;
			if(a->numpids<1||a->numpids>SATIP_MAX_PIDS)
				return SATIP_ERR_SYNTX;
			len+=snprintf(bfr+len,size-len,"&addpids");
			for(i=0;i<a->numpids;i++)
				if(a->pids[i]<0||a->pids[i]>8191)
					return SATIP_ERR_SYNTX;
			else len+=snprintf(bfr+len,size-len,"%c%d",i?',':'=',
				a->pids[i]);
		}
		if(d)
		{
			if(d->prognum)return SATIP_ERR_SYNTX;
			if(d->numpids<1||d->numpids>SATIP_MAX_PIDS)
				return SATIP_ERR_SYNTX;
			len+=snprintf(bfr+len,size-len,"&delpids");
			for(i=0;i<d->numpids;i++)
				if(d->pids[i]<0||d->pids[i]>8191)
					return SATIP_ERR_SYNTX;
			else len+=snprintf(bfr+len,size-len,"%c%d",i?',':'=',
				d->pids[i]);
		}
	}

	if(extra)for(i=0;i<extra->total;i++)len+=snprintf(bfr+len,size-len,
		"&%s=%s",extra->name[i],extra->value[i]);

	*bfr='?';

	return len;
}

static int parsemaskaddr(char *addr,ADDR *a,ADDR *m)
{
	int i;
	long ll;
	unsigned char *ptr;
	char *end;
	ADDR c;
	unsigned char wrk[SATIP_ADDR_LEN+1];

	strncpy(wrk,addr,sizeof(wrk)-1);
	wrk[sizeof(wrk)-1]=0;
	if((ptr=strchr(wrk,'/')))*ptr=0;
	if(str2addr(wrk,a))return -1;

	if((ptr=strchr(addr,'/')))
	{
		ll=strtol(++ptr,&end,10);
		if(!*end)
		{
			if(ll<0||ptr==(unsigned char *)end)return -1;
			switch(a->family)
			{
			case AF_INET:
				if(ll>32)return -1;
				m->a4.s_addr=htonl(0xffffffff<<(32-ll));
				break;
			case AF_INET6:
				if(ll>128)return -1;
				for(i=0,ptr=m->a6.s6_addr;ll>=8;ll-=8,i++)
					ptr[i]=0xff;
				if(ll)ptr[i++]=0xff<<(8-ll);
				while(i<16)ptr[i++]=0;
				break;
			}
			m->family=a->family;
		}
		else
		{
			if(str2addr(ptr,m)||m->family!=a->family)return -1;
			switch(m->family)
			{
			case AF_INET:
				for(i=0;i<=32;i++)if(m->a4.s_addr==
					htonl(0xffffffff<<(32-i)))break;
				if(i>32)return -1;
				break;
			case AF_INET6:
				for(ptr=m->a6.s6_addr,i=0;i<16;i++)
					if(ptr[i]!=0xff)break;
				if(i==16)break;
				if(ptr[i])switch(ptr[i++])
				{
				case 0x00:
				case 0x80:
				case 0xc0:
				case 0xe0:
				case 0xf0:
				case 0xf8:
				case 0xfc:
				case 0xfe:
				case 0xff:
					break;
				default:return -1;
				}
				while(i<16)if(ptr[i++])return -1;
				break;
			}
		}
	}
	else if(a->family==AF_INET)
	{
		m->a4.s_addr=-1;
		m->family=AF_INET;
	}
	else
	{
		memset(m->a6.s6_addr,0xff,16);
		m->family=AF_INET6;
	}

	switch(a->family)
	{
	case AF_INET:
		c.family=AF_INET;
		c.a4.s_addr=a->a4.s_addr&m->a4.s_addr;
		break;
	case AF_INET6:
		c.family=AF_INET6;
		for(i=0;i<16;i++)
			c.a6.s6_addr[i]=a->a6.s6_addr[i]&m->a6.s6_addr[i];
		break;
	}
	if(aacmp(a,&c))return -1;

	return 0;
}

static int matchaddr(LIST *l,ADDR *a)
{
	int i;
	ENTRY *e;
	ADDR c;

	pthread_spin_lock(&l->mtx);

	for(e=l->list;e;e=e->next)
	{
		switch(a->family)
		{
		case AF_INET:
			c.family=AF_INET;
			c.a4.s_addr=a->a4.s_addr&e->mask.a4.s_addr;
			break;
		case AF_INET6:
			c.family=AF_INET6;
			for(i=0;i<16;i++)c.a6.s6_addr[i]=
				a->a6.s6_addr[i]&e->mask.a6.s6_addr[i];
			break;
		}
		if(!aacmp(&e->addr,&c))
		{
			pthread_spin_unlock(&l->mtx);
			return 0;
		}
	}

	pthread_spin_unlock(&l->mtx);
	return -1;
}

int satip_util_parse(int mode,unsigned int caps,int flags,char *input,int *type,
	char *addr,int size,int *port,int *stream,SATIP_TUNE *tune,
	SATIP_PIDS *set,SATIP_PIDS *add,SATIP_PIDS *del,SATIP_EXTRA *extra)
{
	int l;
	long ll;
	char *end;

	if(!input)return SATIP_SYSFAIL;

	switch(mode)
	{
	case SATIP_PARSE_HST:
	case SATIP_PARSE_URL:
		if(!type||!addr||!port)return SATIP_SYSFAIL;

		if(!strncmp(input,"http://",7))
		{
			*type=SATIP_TYPE_HTTP;
			*port=SATIP_HTTP_PORT;
			input+=7;
		}
		else if(!strncmp(input,"rtsp://",7))
		{
			*type=SATIP_TYPE_RTSP;
			*port=SATIP_RTSP_PORT;
			input+=7;
		}
		else if(!strncmp(input,"rtp://",6))
		{
			*type=SATIP_TYPE_RTP;
			*port=0;
			input+=6;
		}
		else return SATIP_ERR_SYNTX;

		if(*input=='[')
		{
			for(l=0,input++;input[l]&&input[l]!=']'&&input[l]!='/';
				l++);
			if(input[l]!=']')return SATIP_ERR_SYNTX;
			if(size<=l)return SATIP_SYSFAIL;
			memcpy(addr,input,l);
			addr[l]=0;
			input+=l+1;
		}
		else
		{
			for(l=0;input[l]&&input[l]!=':'&&input[l]!='/';l++);
			if(!input[l]&&*type!=SATIP_TYPE_RTP)
				return SATIP_ERR_SYNTX;
			if(size<=l)return SATIP_SYSFAIL;
			memcpy(addr,input,l);
			addr[l]=0;
			input+=l;
		}

		if(*input==':')
		{
			ll=strtol(++input,&end,10);
			if(ll<=0||ll>65535||input==end)return SATIP_ERR_SYNTX;
			*port=ll;
			switch(*type)
			{
			case SATIP_TYPE_HTTP:
			case SATIP_TYPE_RTSP:
				if(*end!='/')if(!(mode==SATIP_PARSE_HST&&*end))
					return SATIP_ERR_SYNTX;
				input=end;
				break;
			case SATIP_TYPE_RTP:
				if(ll&1)return SATIP_ERR_SYNTX;
				if((flags&SATIP_RTPQUERY)&&*end!='/')
					return SATIP_ERR_SYNTX;
				else if(!(flags&SATIP_RTPQUERY)&&*end)
					return SATIP_ERR_SYNTX;
				input=end;
				break;
			}
		}

		if(!*port)return SATIP_ERR_SYNTX;

		if(mode==SATIP_PARSE_HST)return 0;

		if(!stream)return SATIP_SYSFAIL;
		*stream=SATIP_UNDEF;

		if(*type==SATIP_TYPE_RTP&&!(flags&SATIP_RTPQUERY))return 0;

		if(!strncmp(++input,"stream=",7))
		{
			if(*type!=SATIP_TYPE_RTSP)return SATIP_ERR_SYNTX;
			input+=7;
			ll=strtol(input,&end,10);
			if(ll<=0||ll>65535||input==end||*end)
				return SATIP_ERR_SYNTX;
			*stream=ll;
			input=end;
			if(!*input)return 0;
			
		}

		if(*input++!='?')return SATIP_ERR_SYNTX;

	case SATIP_PARSE_QRY:
		if(!tune||!set||!add||!del)return SATIP_SYSFAIL;
		return parsequery(input,caps,flags,tune,set,add,del,extra);

	case SATIP_PARSE_PID:
		if(!set)return SATIP_SYSFAIL;
		if(!strcmp(input,"all"))set->numpids=SATIP_ALLPIDS;
		else if(!strcmp(input,"none"))set->numpids=SATIP_NOPIDS;
		else switch(csv2arr(input,set))
		{
		case 0:	qsort(set->pids,set->numpids,sizeof(short),pidcmp);
			set->prognum=0;
			break;
		case -1:return SATIP_ERR_SYNTX;
		case -2:return SATIP_ERR_VALUE;
		case -3:return SATIP_ERR_OVER;
		}
		return 0;

	default:return SATIP_SYSFAIL;
	}
}

int satip_util_create(int type,int flags,char *addr,int port,int stream,
	SATIP_TUNE *tune,SATIP_PIDS *set,SATIP_PIDS *add,SATIP_PIDS *del,
	SATIP_EXTRA *extra,char *output,int size)
{
	int len;
	ADDR a;

	if(!output||size<=0)return SATIP_SYSFAIL;

	switch(type)
	{
	case SATIP_TYPE_RTSP:
		if(!addr||port<1||port>65535)return SATIP_SYSFAIL;
		if(str2addr(addr,&a)==-1)a.family=0;
		else if(invalid_mcast(&a)!=-1)return SATIP_SYSFAIL;
		if(a.family==AF_INET6)
		{
			if(port==SATIP_RTSP_PORT)
				len=snprintf(output,size,"rtsp://[%s]/",addr);
			else len=snprintf(output,size,"rtsp://[%s]:%d/",addr,
				port);
		}
		else if(port==SATIP_RTSP_PORT)
			len=snprintf(output,size,"rtsp://%s/",addr);
		else len=snprintf(output,size,"rtsp://%s:%d/",addr,port);
		if(stream!=SATIP_UNDEF)
		{
			if(stream<0||stream>65535)return SATIP_ERR_SYNTX;
			len+=snprintf(output+len,size-len,"stream=%d",stream);
		}
		if(!tune)return len;
		else return mkquery(flags,tune,set,add,del,extra,output+len,
			size-len);

	case SATIP_TYPE_RTP:
		if(!addr||port<1||port>65535)return SATIP_SYSFAIL;
		if(tune||set||add||del)return SATIP_SYSFAIL;
		if(str2addr(addr,&a)==-1)a.family=0;
		else if(invalid_mcast(&a)==-1)return SATIP_SYSFAIL;
		if(a.family==AF_INET6)
			return snprintf(output,size,"rtp://[%s]:%d",addr,port);
		return snprintf(output,size,"rtp://%s:%d",addr,port);

	case SATIP_TYPE_HTTP:
		if(!tune||!addr||port<1||port>65535)return SATIP_SYSFAIL;
		if(str2addr(addr,&a)==-1)a.family=0;
		else if(invalid_mcast(&a)!=-1)return SATIP_SYSFAIL;
		if(a.family==AF_INET6)
		{
			if(port==SATIP_HTTP_PORT)
				len=snprintf(output,size,"http://[%s]/",addr);
			else len=snprintf(output,size,"http://[%s]:%d/",addr,
				port);
		}
		else if(port==SATIP_HTTP_PORT)
			len=snprintf(output,size,"http://%s/",addr);
		else len=snprintf(output,size,"http://%s:%d/",addr,port);
		return mkquery(flags,tune,set,add,del,extra,output+len,
			size-len);

	case SATIP_TYPE_QRY:
		return mkquery(flags,tune,set,add,del,extra,output,size);

	default:return SATIP_SYSFAIL;
	}
}

int satip_util_random(unsigned char *dst,int amount)
{
	uint64_t s0;
	uint64_t s1;
	static int p=-1;
	static uint64_t s[16];
	static pthread_mutex_t mtx=PTHREAD_MUTEX_INITIALIZER;

	pthread_mutex_lock(&mtx);

	if(p==-1)for(s0=(s0=((uint64_t)times(NULL))^
		(((uint64_t)time(NULL))<<32))?s0:1,p=16;p>0;)
	{
		s0^=s0>>12;
		s0^=s0<<25;
		s0^=s0>>27;
		s[--p]=s0*2685821657736338717ULL;
	}

	while(amount-->0)
	{
		s0=s[p];
		s1=s[(p=(p+1)&15)];
		s1^=s1<<31;
		s1^=s1>>11;
		s0^=s0>>30;
		*dst++=(unsigned char)((s[p]=s0^s1)*1181783497276652981ULL);
	}

	pthread_mutex_unlock(&mtx);

	return 0;
}

SATIP_UTIL_PAT *satip_util_unpack_pat_section(unsigned char *data,int len)
{
	int i;
	SATIP_UTIL_PAT *e;

	if(len<12)return NULL;

	i=data[1]&0xf;
	i<<=8;
	i|=data[2];
	if(len!=i+3)return NULL;

	if(!(e=malloc(sizeof(SATIP_UTIL_PAT)+sizeof(e->data[0])*((len-12)>>2))))
		return NULL;
	e->next=NULL;
	e->tsid=data[3];
	e->tsid<<=8;
	e->tsid|=data[4];
	e->vernum=(data[5]>>1)&0x1f;
	e->cnind=data[5]&0x01;
	e->secnum=data[6];
	e->lastsec=data[7];
	e->total=(len-12)>>2;
	for(data+=8,i=0;i<e->total;i++,data+=4)
	{
		e->data[i].prognum=data[0];
		e->data[i].prognum<<=8;
		e->data[i].prognum|=data[1];
		e->data[i].pmtpid=data[2]&0x1f;
		e->data[i].pmtpid<<=8;
		e->data[i].pmtpid|=data[3];
		e->data[i].pmt=NULL;
	}
	return e;
}

SATIP_UTIL_CAT *satip_util_unpack_cat_section(unsigned char *data,int len)
{
	int i;
	int j;
	int l;
	int m;
	int q;
	int r;
	unsigned char *ptr;
	SATIP_UTIL_CAT *e;

	if(len<12)return NULL;

	i=data[1]&0xf;
	i<<=8;
	i|=data[2];
	if(len!=i+3)return NULL;

	r=0;
	ptr=data+8;
	j=len-12;

	while(j>0)
	{
		if(j<2)return NULL;
		l=ptr[1];
		if(l==0x80||l>0x82)return NULL;
		switch(l)
		{
		case 0x81:
			if(j<3)return NULL;
			l=ptr[2]+3;
			break;
		case 0x82:
			if(j<4)return NULL;
			l=ptr[2];
			l<<=8;
			l|=ptr[3];
			l+=4;
			break;
		default:l+=2;
			break;
		}
		if(j<l)return NULL;
		if(ptr[0]==0x09)r+=l;
		ptr+=l;
		j-=l;
	}

	if(!(e=malloc(sizeof(SATIP_UTIL_CAT)+r)))return NULL;
	e->next=NULL;
	e->vernum=(data[5]>>1)&0x1f;
	e->cnind=data[5]&0x01;
	e->secnum=data[6];
	e->lastsec=data[7];
	e->catotal=0;

	ptr=(unsigned char *)e;
	r=sizeof(SATIP_UTIL_CAT);

	data+=8;
	len-=8;
	while(len>0)
	{
		l=data[1];
		switch(l)
		{
		case 0x81:
			l=data[2];
			q=1;
			break;
		case 0x82:
			l=data[2];
			l<<=8;
			l|=data[3];
			q=2;
			break;
		default:q=0;
			break;
		}
		l+=2;
		switch(data[0])
		{
		case 0x09:
			if(l<6)break;
			i=data[2+q];
			i<<=8;
			i|=data[3+q];
			m=data[4+q]&0x1f;
			m<<=8;
			m|=data[5+q];
			for(j=0;j<e->catotal;j++)
				if(e->caid[j]==i&&e->capid[j]==m)break;
			if(j<e->catotal)break;
			if(e->catotal==SATIP_CATCA_MAX)break;
			e->caid[e->catotal]=i;
			e->capid[e->catotal]=m;
			e->raw[e->catotal++]=r;
			memcpy(ptr+r,data,l+q);
			r+=l+q;
			break;
		}
		data+=l+q;
		len-=l+q;
	}

	return e;
}

SATIP_UTIL_PMT *satip_util_unpack_pmt_section(unsigned char *data,int len)
{
	int i;
	int j;
	int k;
	int l;
	int m;
	int n;
	int p;
	int q;
	int r;
	unsigned char *ptr;
	SATIP_UTIL_PMT *e;

	if(len<12)return NULL;

	i=data[1]&0xf;
	i<<=8;
	i|=data[2];
	if(len!=i+3)return NULL;

	r=0;
	i=0;
	ptr=data+8;
	j=len-12;
	k=ptr[2]&0x0f;
	k<<=8;
	k|=ptr[3];
	ptr+=4;
	j-=4;
	while(k>0)
	{
		if(j<2)return NULL;
		l=ptr[1];
		if(l==0x80||l>0x82)return NULL;
		switch(l)
		{
		case 0x81:
			if(j<3)return NULL;
			l=ptr[2]+3;
			break;
		case 0x82:
			if(j<4)return NULL;
			l=ptr[2];
			l<<=8;
			l|=ptr[3];
			l+=4;
			break;
		default:l+=2;
			break;
		}
		if(j<l)return NULL;
		if(ptr[0]==0x09)r+=l;
		ptr+=l;
		j-=l;
		k-=l;
	}
	while(j>0)
	{
		if(j<5)return NULL;
		i++;
		k=ptr[3]&0x0f;
		k<<=8;
		k|=ptr[4];
		ptr+=5;
		j-=5;
		while(k>0)
		{
			if(j<2)return NULL;
			l=ptr[1];
			if(l==0x80||l>0x82)return NULL;
			switch(l)
			{
			case 0x81:
				if(j<3)return NULL;
				l=ptr[2]+3;
				break;
			case 0x82:
				if(j<4)return NULL;
				l=ptr[2];
				l<<=8;
				l|=ptr[3];
				l+=4;
				break;
			default:l+=2;
				break;
			}
			if(j<l)return NULL;
			if(ptr[0]==0x09)r+=l;
			ptr+=l;
			j-=l;
			k-=l;
		}
	}

	if(!(e=malloc(sizeof(SATIP_UTIL_PMT)+r+sizeof(e->data[0])*i)))
		return NULL;
	e->next=NULL;
	e->sdt=NULL;
	e->prognum=data[3];
	e->prognum<<=8;
	e->prognum|=data[4];
	e->vernum=(data[5]>>1)&0x1f;
	e->cnind=data[5]&0x01;
	e->secnum=data[6];
	e->lastsec=data[7];
	e->sdtindex=-1;
	e->total=0;
	e->catotal=0;

	ptr=(unsigned char *)e;
	r=sizeof(SATIP_UTIL_PMT)+sizeof(e->data[0])*i;

	data+=8;
	len-=12;
	e->pcrpid=data[0]&0x1f;
	e->pcrpid<<=8;
	e->pcrpid|=data[1];
	k=data[2]&0x0f;
	k<<=8;
	k|=data[3];
	data+=4;
	len-=4;
	while(k>0)
	{
		l=data[1];
		switch(l)
		{
		case 0x81:
			l=data[2];
			q=1;
			break;
		case 0x82:
			l=data[2];
			l<<=8;
			l|=data[3];
			q=2;
			break;
		default:q=0;
			break;
		}
		l+=2;
		switch(data[0])
		{
		case 0x09:
			if(l<6)break;
			i=data[2+q];
			i<<=8;
			i|=data[3+q];
			m=data[4+q]&0x1f;
			m<<=8;
			m|=data[5+q];
			for(j=0;j<e->catotal;j++)
				if(e->caid[j]==i&&e->capid[j]==m)break;
			if(j<e->catotal)break;
			if(e->catotal==SATIP_PMTCA_MAX)break;
			e->caid[e->catotal]=i;
			e->capid[e->catotal]=m;
			e->raw[e->catotal++]=r;
			memcpy(ptr+r,data,l+q);
			r+=l+q;
			break;
		}
		data+=l+q;
		len-=l+q;
		k-=l+q;
	}
	while(len>0)
	{
		p=0;
		e->data[e->total].type=0;
		e->data[e->total].pid=data[1]&0x1f;
		e->data[e->total].pid<<=8;
		e->data[e->total].pid|=data[2];
		e->data[e->total].lang[0]=0;
		e->data[e->total].catotal=0;
		k=data[3]&0x0f;
		k<<=8;
		k|=data[4];
		i=data[0];
		len-=5;
		data+=5;
		switch(i)
		{
		case 0x01:
		case 0x02:
			e->data[e->total].type=SATIP_PMT_VIDEO;
			e->data[e->total].typenum=e->data[e->total].type1=i;
			break;

		case 0x1b:
			e->data[e->total].type=SATIP_PMT_AVC;
			e->data[e->total].typenum=e->data[e->total].type1=i;
			break;

		case 0x24:
			e->data[e->total].type=SATIP_PMT_HEVC;
			e->data[e->total].typenum=e->data[e->total].type1=i;
			break;

		case 0x03:
		case 0x04:
			e->data[e->total].type=SATIP_PMT_AUDIO;
			e->data[e->total].typenum=e->data[e->total].type1=i;
			break;

		case 0x0f:
			e->data[e->total].type=SATIP_PMT_ADTS;
			e->data[e->total].typenum=e->data[e->total].type1=i;
			break;

		case 0x11:
			e->data[e->total].type=SATIP_PMT_LATM;
			e->data[e->total].typenum=e->data[e->total].type1=i;
			break;

		case 0x81:
			e->data[e->total].type=SATIP_PMT_AC3;
			e->data[e->total].typenum=e->data[e->total].type1=i;
			break;

		case 0x05:
		case 0x06:
			e->data[e->total].typenum=e->data[e->total].type1=i;
			p=1;
			break;
		}
		while(k>0)
		{
			l=data[1];
			switch(l)
			{
			case 0x81:
				l=data[2];
				q=1;
				break;
			case 0x82:
				l=data[2];
				l<<=8;
				l|=data[3];
				q=2;
				break;
			default:q=0;
				break;
			}
			l+=2;
			switch(data[0])
			{
			case 0x56:
				if(p)
				{
					e->data[e->total].type=SATIP_PMT_TTX;
					e->data[e->total].typenum=data[0];
					if(l<7)break;
					memcpy(e->data[e->total].lang,data+2,3);
					e->data[e->total].lang[3]=0;
				}
				break;

			case 0x59:
				if(p)
				{
					e->data[e->total].type=SATIP_PMT_SUBS;
					e->data[e->total].typenum=data[0];
					if(l<10)break;
					memcpy(e->data[e->total].lang,data+2,3);
					e->data[e->total].lang[3]=0;
				}
				break;

			case 0x6a:
				if(p)
				{
					e->data[e->total].type=SATIP_PMT_AC3;
					e->data[e->total].typenum=data[0];
				}
				break;

			case 0x7a:
				if(p)
				{
					e->data[e->total].type=SATIP_PMT_EAC3;
					e->data[e->total].typenum=data[0];
				}
				break;

			case 0x0a:
				if(l<6)break;
				memcpy(e->data[e->total].lang,data+2+q,3);
				e->data[e->total].lang[3]=0;
				break;

			case 0x09:
				if(l<6)break;
				i=data[2+q];
				i<<=8;
				i|=data[3+q];
				n=data[4+q]&0x1f;
				n<<=8;
				n|=data[5+q];
				m=e->data[e->total].catotal;
				for(j=0;j<m;j++)if(e->data[e->total].caid[j]==i
					&&e->data[e->total].capid[j]==n)break;
				if(j<m)break;
				if(m==SATIP_PMTCA_MAX)break;
				e->data[e->total].caid[m]=i;
				e->data[e->total].capid[m]=n;
				e->data[e->total].catotal=m+1;
				e->data[e->total].raw[m]=r;
				memcpy(ptr+r,data,l+q);
				r+=l+q;
				break;
			}
			data+=l+q;
			len-=l+q;
			k-=l+q;
		}
		if(e->data[e->total].type)e->total++;
	}

	return e;
}

SATIP_UTIL_NIT *satip_util_unpack_nit_section(unsigned char *data,int len)
{
	int i;
	int j;
	int k;
	int l;
	unsigned char *ptr;
	SATIP_UTIL_NIT *e;

	if(len<12)return NULL;

	i=data[1]&0xf;
	i<<=8;
	i|=data[2];
	if(len!=i+3)return NULL;

	i=0;
	j=len-12;
	ptr=data+8;
	k=ptr[0]&0x0f;
	k<<=8;
	k|=ptr[1];
	ptr+=2;
	j-=2;
	while(k>0)
	{
		if(j<2)return NULL;
		l=ptr[1]+2;
		if(j<l)return NULL;
		ptr+=l;
		j-=l;
		k-=l;
	}
	ptr+=2;
	j-=2;
	while(j>0)
	{
		if(j<6)return NULL;
		i++;
		k=ptr[4]&0x0f;
		k<<=8;
		k|=ptr[5];
		ptr+=6;
		j-=6;
		while(k>0)
		{
			if(j<2)return NULL;
			l=ptr[1]+2;
			if(j<l)return NULL;
			ptr+=l;
			j-=l;
			k-=l;
		}
	}

	if(!(e=malloc(sizeof(SATIP_UTIL_NIT)+sizeof(e->data[0])*i)))
		return NULL;

	e->next=NULL;
	e->netid=data[3];
	e->netid<<=8;
	e->netid|=data[4];
	e->vernum=(data[5]>>1)&0x1f;
	e->cnind=data[5]&0x01;
	e->secnum=data[6];
	e->lastsec=data[7];
	e->total=0;
	e->netname[0]=0;

	data+=8;
	len-=12;
	k=data[0]&0x0f;
	k<<=8;
	k|=data[1];
	data+=2;
	len-=2;
	while(k>0)
	{
		l=data[1]+2;
		switch(data[0])
		{
		case 0x40:
			if(l<2)break;
			if(chrcpy(e->netname,sizeof(e->netname),data+2,l-2))
				e->netname[i]=0;
			break;
		}
		data+=l;
		len-=l;
		k-=l;
	}
	data+=2;
	len-=2;
	while(len>0)
	{
		e->data[e->total].tsid=data[0];
		e->data[e->total].tsid<<=8;
		e->data[e->total].tsid|=data[1];
		e->data[e->total].onid=data[2];
		e->data[e->total].onid<<=8;
		e->data[e->total].onid|=data[3];
		e->data[e->total].priority=-1;
		e->data[e->total].tslice=-1;
		e->data[e->total].mpefec=-1;
		e->data[e->total].feco=SATIP_FEC_AUTO;
		e->data[e->total].feci=SATIP_FEC_AUTO;
		e->data[e->total].c2gi=SATIP_GI_AUTO;
		e->data[e->total].othfreq=-1;
		e->data[e->total].cellid=0;
		e->data[e->total].alttotal=0;
		e->data[e->total].mis=-1;
		e->data[e->total].ssi=0;
		e->data[e->total].satpos[0]=0;
		e->data[e->total].tune.freq=SATIP_UNSPEC;
		e->data[e->total].tune.fe=SATIP_UNSPEC;
		e->data[e->total].tune.src=SATIP_UNSPEC;
		e->data[e->total].tune.bw=SATIP_BW_AUTO;
		e->data[e->total].tune.pol=SATIP_UNSPEC;
		e->data[e->total].tune.msys=SATIP_UNSPEC;
		e->data[e->total].tune.tmode=SATIP_TMOD_AUTO;
		e->data[e->total].tune.mtype=SATIP_AUTOQ;
		e->data[e->total].tune.plts=SATIP_PLTS_AUTO;
		e->data[e->total].tune.ro=SATIP_ROFF_AUTO;
		e->data[e->total].tune.sr=SATIP_UNSPEC;
		e->data[e->total].tune.gi=SATIP_GI_AUTO;
		e->data[e->total].tune.fec=SATIP_FEC_AUTO;
		e->data[e->total].tune.feclp=SATIP_FEC_AUTO;
		e->data[e->total].tune.hier=SATIP_HIER_AUTO;
		e->data[e->total].tune.c2tft=SATIP_TFT_AUTO;
		e->data[e->total].tune.ds=SATIP_UNDEF;
		e->data[e->total].tune.plp=SATIP_UNDEF;
		e->data[e->total].tune.t2id=SATIP_UNDEF;
		e->data[e->total].tune.sm=SATIP_SM_AUTO;
		e->data[e->total].tune.specinv=SATIP_SPI_AUTO;
		k=data[4]&0x0f;
		k<<=8;
		k|=data[5];
		data+=6;
		len-=6;
		while(k>0)
		{
			l=data[1]+2;
			if(data[0]==0x7f)i=data[2];
			else i=data[0];
			switch(i)
			{
			case 0x43:
				if(l<13)break;
				e->data[e->total].tune.freq=
					(data[2]>>4)*10+(data[2]&0x0f);
				e->data[e->total].tune.freq*=100ULL;
				e->data[e->total].tune.freq+=
					(data[3]>>4)*10+(data[3]&0x0f);
				e->data[e->total].tune.freq*=100ULL;
				e->data[e->total].tune.freq+=
					(data[4]>>4)*10+(data[4]&0x0f);
				e->data[e->total].tune.freq*=100ULL;
				e->data[e->total].tune.freq+=
					(data[5]>>4)*10+(data[5]&0x0f);
				e->data[e->total].tune.freq*=10000ULL;
				e->data[e->total].satpos[0]=(data[6]>>4)|0x30;
				e->data[e->total].satpos[1]=(data[6]&0x0f)|0x30;
				e->data[e->total].satpos[2]=(data[7]>>4)|0x30;
				e->data[e->total].satpos[3]=
					(data[8]&0x80)?'E':'W';
				e->data[e->total].satpos[4]=(data[7]&0x0f)|0x30;
				e->data[e->total].satpos[5]=0;
				switch(data[8]&0x60)
				{
				case 0x00:
					e->data[e->total].tune.pol=SATIP_POL_H;
					break;
				case 0x20:
					e->data[e->total].tune.pol=SATIP_POL_V;
					break;
				case 0x40:
					e->data[e->total].tune.pol=SATIP_POL_L;
					break;
				case 0x60:
					e->data[e->total].tune.pol=SATIP_POL_R;
					break;
				}
				switch(data[8]&0x18)
				{
				case 0x00:
					e->data[e->total].tune.ro=
						SATIP_ROFF_035;
					break;
				case 0x08:
					e->data[e->total].tune.ro=
						SATIP_ROFF_025;
					break;
				case 0x10:
					e->data[e->total].tune.ro=
						SATIP_ROFF_020;
					break;
				}
				if(data[8]&0x04)e->data[e->total].tune.msys=
					SATIP_DVBS2;
				else e->data[e->total].tune.msys=SATIP_DVBS;
				switch(data[8]&0x03)
				{
				case 0x01:
					e->data[e->total].tune.mtype=SATIP_QPSK;
					break;
				case 0x02:
					e->data[e->total].tune.mtype=SATIP_8PSK;
					break;
				case 0x03:
					e->data[e->total].tune.mtype=SATIP_16Q;
					break;
				}
				e->data[e->total].tune.sr=
					(data[9]>>4)*10+(data[9]&0x0f);
				e->data[e->total].tune.sr*=100;
				e->data[e->total].tune.sr+=
					(data[10]>>4)*10+(data[10]&0x0f);
				e->data[e->total].tune.sr*=100;
				e->data[e->total].tune.sr+=
					(data[11]>>4)*10+(data[11]&0x0f);
				e->data[e->total].tune.sr*=10;
				e->data[e->total].tune.sr+=(data[12]>>4);
				e->data[e->total].tune.sr*=100;
				switch(data[12]&0x0f)
				{
				case 0x01:
					e->data[e->total].tune.fec=SATIP_FEC_12;
					break;
				case 0x02:
					e->data[e->total].tune.fec=SATIP_FEC_23;
					break;
				case 0x03:
					e->data[e->total].tune.fec=SATIP_FEC_34;
					break;
				case 0x04:
					e->data[e->total].tune.fec=SATIP_FEC_56;
					break;
				case 0x05:
					e->data[e->total].tune.fec=SATIP_FEC_78;
					break;
				case 0x06:
					e->data[e->total].tune.fec=SATIP_FEC_89;
					break;
				case 0x07:
					e->data[e->total].tune.fec=SATIP_FEC_35;
					break;
				case 0x08:
					e->data[e->total].tune.fec=SATIP_FEC_45;
					break;
				case 0x09:
					e->data[e->total].tune.fec=
						SATIP_FEC_910;
					break;
				}
				break;

			case 0x44:
				if(l<13)break;
				e->data[e->total].tune.msys=SATIP_DVBC;
				e->data[e->total].tune.freq=
					(data[2]>>4)*10+(data[2]&0x0f);
				e->data[e->total].tune.freq*=100ULL;
				e->data[e->total].tune.freq+=
					(data[3]>>4)*10+(data[3]&0x0f);
				e->data[e->total].tune.freq*=100ULL;
				e->data[e->total].tune.freq+=
					(data[4]>>4)*10+(data[4]&0x0f);
				e->data[e->total].tune.freq*=100ULL;
				e->data[e->total].tune.freq+=
					(data[5]>>4)*10+(data[5]&0x0f);
				e->data[e->total].tune.freq*=100ULL;
				switch(data[7]&0x0f)
				{
				case 1:	e->data[e->total].feco=SATIP_FEC_NONE;
					break;
				case 2:	e->data[e->total].feco=SATIP_FEC_RS204;
					break;
				}
				switch(data[8])
				{
				case 0x01:
					e->data[e->total].tune.mtype=SATIP_16Q;
					break;
				case 0x02:
					e->data[e->total].tune.mtype=SATIP_32Q;
					break;
				case 0x03:
					e->data[e->total].tune.mtype=SATIP_64Q;
					break;
				case 0x04:
					e->data[e->total].tune.mtype=SATIP_128Q;
					break;
				case 0x05:
					e->data[e->total].tune.mtype=SATIP_256Q;
					break;
				}
				e->data[e->total].tune.sr=
					(data[9]>>4)*10+(data[9]&0x0f);
				e->data[e->total].tune.sr*=100;
				e->data[e->total].tune.sr+=
					(data[10]>>4)*10+(data[10]&0x0f);
				e->data[e->total].tune.sr*=100;
				e->data[e->total].tune.sr+=
					(data[11]>>4)*10+(data[11]&0x0f);
				e->data[e->total].tune.sr*=10;
				e->data[e->total].tune.sr+=(data[12]>>4);
				e->data[e->total].tune.sr*=100;
				switch(data[12]&0x0f)
				{
				case 0x01:
					e->data[e->total].feci=SATIP_FEC_12;
					break;
				case 0x02:
					e->data[e->total].feci=SATIP_FEC_23;
					break;
				case 0x03:
					e->data[e->total].feci=SATIP_FEC_34;
					break;
				case 0x04:
					e->data[e->total].feci=SATIP_FEC_56;
					break;
				case 0x05:
					e->data[e->total].feci=SATIP_FEC_78;
					break;
				case 0x06:
					e->data[e->total].feci=SATIP_FEC_89;
					break;
				case 0x07:
					e->data[e->total].feci=SATIP_FEC_35;
					break;
				case 0x08:
					e->data[e->total].feci=SATIP_FEC_45;
					break;
				case 0x09:
					e->data[e->total].feci=SATIP_FEC_910;
					break;
				}
				break;

			case 0x5a:
				if(l<13)break;
				e->data[e->total].tune.msys=SATIP_DVBT;
				e->data[e->total].tune.freq=data[2];
				e->data[e->total].tune.freq<<=8;
				e->data[e->total].tune.freq|=data[3];
				e->data[e->total].tune.freq<<=8;
				e->data[e->total].tune.freq|=data[4];
				e->data[e->total].tune.freq<<=8;
				e->data[e->total].tune.freq|=data[5];
				e->data[e->total].tune.freq*=10ULL;
				switch(data[6]&0xe0)
				{
				case 0x00:
					e->data[e->total].tune.bw=SATIP_BW_8;
					break;
				case 0x20:
					e->data[e->total].tune.bw=SATIP_BW_7;
					break;
				case 0x40:
					e->data[e->total].tune.bw=SATIP_BW_6;
					break;
				case 0x60:
					e->data[e->total].tune.bw=SATIP_BW_5;
					break;
				}
				e->data[e->total].priority=(data[6]&0x10)?1:0;
				e->data[e->total].tslice=(data[6]&0x08)?1:0;
				e->data[e->total].mpefec=(data[6]&0x04)?1:0;
				switch(data[7]&0xc0)
				{
				case 0x00:
					e->data[e->total].tune.mtype=SATIP_QPSK;
					break;
				case 0x40:
					e->data[e->total].tune.mtype=SATIP_16Q;
					break;
				case 0x80:
					e->data[e->total].tune.mtype=SATIP_64Q;
					break;
				}
				switch(data[7]&0x38)
				{
				case 0x00:
				case 0x20:
					e->data[e->total].tune.hier=
						SATIP_HIER_NONE;
					break;
				case 0x08:
				case 0x28:
					e->data[e->total].tune.hier=
						SATIP_HIER_1;
					break;
				case 0x10:
				case 0x30:
					e->data[e->total].tune.hier=
						SATIP_HIER_2;
					break;
				case 0x18:
				case 0x38:
					e->data[e->total].tune.hier=
						SATIP_HIER_4;
					break;
				}
				switch(data[7]&0x07)
				{
				case 0x00:
					e->data[e->total].tune.fec=SATIP_FEC_12;
					break;
				case 0x01:
					e->data[e->total].tune.fec=SATIP_FEC_23;
					break;
				case 0x02:
					e->data[e->total].tune.fec=SATIP_FEC_34;
					break;
				case 0x03:
					e->data[e->total].tune.fec=SATIP_FEC_56;
					break;
				case 0x04:
					e->data[e->total].tune.fec=SATIP_FEC_78;
					break;
				}
				switch(data[8]&0xe0)
				{
				case 0x00:
					e->data[e->total].tune.feclp=
						SATIP_FEC_12;
					break;
				case 0x20:
					e->data[e->total].tune.feclp=
						SATIP_FEC_23;
					break;
				case 0x40:
					e->data[e->total].tune.feclp=
						SATIP_FEC_34;
					break;
				case 0x60:
					e->data[e->total].tune.feclp=
						SATIP_FEC_56;
					break;
				case 0x80:
					e->data[e->total].tune.feclp=
						SATIP_FEC_78;
					break;
				}
				switch(data[8]&0x18)
				{
				case 0x00:
					e->data[e->total].tune.gi=SATIP_GI_132;
					break;
				case 0x08:
					e->data[e->total].tune.gi=SATIP_GI_116;
					break;
				case 0x10:
					e->data[e->total].tune.gi=SATIP_GI_18;
					break;
				case 0x18:
					e->data[e->total].tune.gi=SATIP_GI_14;
					break;
				}
				switch(data[8]&0x06)
				{
				case 0x00:
					e->data[e->total].tune.tmode=
						SATIP_TMOD_2K;
					break;
				case 0x02:
					e->data[e->total].tune.tmode=
						SATIP_TMOD_8K;
					break;
				case 0x04:
					e->data[e->total].tune.tmode=
						SATIP_TMOD_4K;
					break;
				}
				e->data[e->total].othfreq=(data[8]&0x01);
				break;

			case 0x62:
				if(l<7)break;
				if(!(i=data[2]&0x03))break;
				l-=3;
				data+=3;
				len-=3;
				k-=3;
				while(l>3)
				{
					j=e->data[e->total].alttotal;
					if(j<SATIP_FRALT_MAX)switch(i)
					{
					case 1:	e->data[e->total].alt[j]
							.cellid=0;
						e->data[e->total].alt[j]
							.istransposer=0;
						e->data[e->total].alt[j]
							.freq=(data[0]>>4)*10+
							(data[0]&0x0f);
						e->data[e->total].alt[j]
							.freq*=100ULL;
						e->data[e->total].alt[j]
							.freq+=(data[1]>>4)*10+
							(data[1]&0x0f);
						e->data[e->total].alt[j]
							.freq*=100ULL;
						e->data[e->total].alt[j]
							.freq+=(data[2]>>4)*10+
							(data[2]&0x0f);
						e->data[e->total].alt[j]
							.freq*=100ULL;
						e->data[e->total].alt[j]
							.freq+=(data[3]>>4)*10+
							(data[3]&0x0f);
						e->data[e->total].alt[j]
							.freq*=10000ULL;
						e->data[e->total].alttotal=j+1;
						break;

					case 2:	e->data[e->total].alt[j]
							.cellid=0;
						e->data[e->total].alt[j]
							.istransposer=0;
						e->data[e->total].alt[j]
							.freq=(data[0]>>4)*10+
							(data[0]&0x0f);
						e->data[e->total].alt[j]
							.freq*=100ULL;
						e->data[e->total].alt[j]
							.freq+=(data[1]>>4)*10+
							(data[1]&0x0f);
						e->data[e->total].alt[j]
							.freq*=100ULL;
						e->data[e->total].alt[j]
							.freq+=(data[2]>>4)*10+
							(data[2]&0x0f);
						e->data[e->total].alt[j]
							.freq*=100ULL;
						e->data[e->total].alt[j]
							.freq+=(data[3]>>4)*10+
							(data[3]&0x0f);
						e->data[e->total].alt[j]
							.freq*=100ULL;
						e->data[e->total].alttotal=j+1;
						break;

					case 3:	e->data[e->total].alt[j]
							.cellid=0;
						e->data[e->total].alt[j]
							.istransposer=0;
						e->data[e->total].alt[j]
							.freq=data[0];
						e->data[e->total].alt[j]
							.freq<<=8;
						e->data[e->total].alt[j]
							.freq|=data[1];
						e->data[e->total].alt[j]
							.freq<<=8;
						e->data[e->total].alt[j]
							.freq|=data[2];
						e->data[e->total].alt[j]
							.freq<<=8;
						e->data[e->total].alt[j]
							.freq|=data[3];
						e->data[e->total].alt[j]
							.freq*=10ULL;
						e->data[e->total].alttotal=j+1;
						break;
					}
					l-=4;
					data+=4;
					len-=4;
					k-=4;
				}
				break;

			case 0x79:
				if(l<3)break;
				if(l<3+((data[2]&0x80)?3:0)+
					((data[2]&0x40)?1:0))break;
				if(data[2]&0x80)
				{
					e->data[e->total].ssi=data[3]&0x03;
					e->data[e->total].ssi<<=8;
					e->data[e->total].ssi|=data[4];
					e->data[e->total].ssi<<=8;
					e->data[e->total].ssi|=data[5];
				}
				if(!(data[2]&0x40))break;
				e->data[e->total].mis=
					data[3+((data[2]&0x80)?3:0)];
				break;

			case 0x0d:
				if(l<10)break;
				e->data[e->total].tune.msys=SATIP_DVBC2;
				e->data[e->total].tune.plp=data[3];
				e->data[e->total].tune.ds=data[4];
				e->data[e->total].tune.freq=data[5];
				e->data[e->total].tune.freq<<=8;
				e->data[e->total].tune.freq|=data[6];
				e->data[e->total].tune.freq<<=8;
				e->data[e->total].tune.freq|=data[7];
				e->data[e->total].tune.freq<<=8;
				e->data[e->total].tune.freq|=data[8];
				switch(data[9]&0xc0)
				{
				case 0x00:
					e->data[e->total].tune.c2tft=
						SATIP_TFT_DS;
					break;
				case 0x40:
					e->data[e->total].tune.c2tft=
						SATIP_TFT_C2;
					break;
				case 0x80:
					e->data[e->total].tune.c2tft=
						SATIP_TFT_IT;
					break;
				}
				switch(data[9]&0x38)
				{
				case 0x00:
					e->data[e->total].tune.bw=SATIP_BW_8;
					break;
				case 0x08:
					e->data[e->total].tune.bw=SATIP_BW_6;
					break;
				}
				switch(data[9]&0x07)
				{
				case 0x00:
					e->data[e->total].c2gi=SATIP_GI_1128;
					break;
				case 0x01:
					e->data[e->total].c2gi=SATIP_GI_164;
					break;
				}
				break;

			case 0x04:
				if(l!=6)
				{
					if(l<15)break;
					if(data[7]&0x01)break;
				}
				e->data[e->total].tune.msys=SATIP_DVBT2;
				e->data[e->total].tune.plp=data[3];
				e->data[e->total].tune.t2id=data[4];
				e->data[e->total].tune.t2id<<=8;
				e->data[e->total].tune.t2id|=data[5];
				if(l==6)break;
				switch(data[6]&0xc0)
				{
				case 0x00:
					e->data[e->total].tune.sm=SATIP_SM_SISO;
					break;
				case 0x40:
					e->data[e->total].tune.sm=SATIP_SM_MISO;
					break;
				}
				switch(data[6]&0x3c)
				{
				case 0x00:
					e->data[e->total].tune.bw=SATIP_BW_8;
					break;
				case 0x04:
					e->data[e->total].tune.bw=SATIP_BW_7;
					break;
				case 0x08:
					e->data[e->total].tune.bw=SATIP_BW_6;
					break;
				case 0x0c:
					e->data[e->total].tune.bw=SATIP_BW_5;
					break;
				case 0x10:
					e->data[e->total].tune.bw=SATIP_BW_10;
					break;
				case 0x14:
					e->data[e->total].tune.bw=SATIP_BW_1712;
					break;
				}
				switch(data[7]&0xe0)
				{
				case 0x20:
					e->data[e->total].tune.gi=SATIP_GI_132;
					break;
				case 0x40:
					e->data[e->total].tune.gi=SATIP_GI_116;
					break;
				case 0x60:
					e->data[e->total].tune.gi=SATIP_GI_18;
					break;
				case 0x80:
					e->data[e->total].tune.gi=SATIP_GI_14;
					break;
				case 0xa0:
					e->data[e->total].tune.gi=
						SATIP_GI_19128;
					break;
				case 0xc0:
					e->data[e->total].tune.gi=
						SATIP_GI_19256;
					break;
				}
				switch(data[7]&0x1c)
				{
				case 0x00:
					e->data[e->total].tune.tmode=
						SATIP_TMOD_2K;
					break;
				case 0x04:
					e->data[e->total].tune.tmode=
						SATIP_TMOD_8K;
					break;
				case 0x08:
					e->data[e->total].tune.tmode=
						SATIP_TMOD_4K;
					break;
				case 0x0c:
					e->data[e->total].tune.tmode=
						SATIP_TMOD_1K;
					break;
				case 0x10:
					e->data[e->total].tune.tmode=
						SATIP_TMOD_16K;
					break;
				case 0x14:
					e->data[e->total].tune.tmode=
						SATIP_TMOD_32K;
					break;
				}
				e->data[e->total].othfreq=(data[7]&0x02)?1:0;
				e->data[e->total].cellid=data[8];
				e->data[e->total].cellid<<=8;
				e->data[e->total].cellid|=data[9];
				e->data[e->total].tune.freq=data[10];
				e->data[e->total].tune.freq<<=8;
				e->data[e->total].tune.freq|=data[11];
				e->data[e->total].tune.freq<<=8;
				e->data[e->total].tune.freq|=data[12];
				e->data[e->total].tune.freq<<=8;
				e->data[e->total].tune.freq|=data[13];
				e->data[e->total].tune.freq*=10ULL;
				i=data[14];
				l-=15;
				data+=15;
				len-=15;
				k-=15;
t2loop:				if(i<=l)for(;i>=6;i-=6)
				{
					j=e->data[e->total].alttotal;
					if(j<SATIP_FRALT_MAX)
					{
						e->data[e->total].alt[j]
							.cellid=data[0];
						e->data[e->total].alt[j]
							.cellid<<=8;
						e->data[e->total].alt[j]
							.cellid=data[1];
						e->data[e->total].alt[j]
							.freq=data[2];
						e->data[e->total].alt[j]
							.freq<<=8;
						e->data[e->total].alt[j]
							.freq|=data[3];
						e->data[e->total].alt[j]
							.freq<<=8;
						e->data[e->total].alt[j]
							.freq|=data[4];
						e->data[e->total].alt[j]
							.freq<<=8;
						e->data[e->total].alt[j]
							.freq|=data[5];
						e->data[e->total].alt[j]
							.freq*=10ULL;
						e->data[e->total].alt[j]
							.istransposer=1;
						e->data[e->total].alttotal=j+1;
					}
					l-=6;
					data+=6;
					len-=6;
					k-=6;
				}
				if(l<7)break;
				j=e->data[e->total].alttotal;
				if(j<SATIP_FRALT_MAX)
				{
					e->data[e->total].alt[j].cellid=data[0];
					e->data[e->total].alt[j].cellid<<=8;
					e->data[e->total].alt[j].cellid=data[1];
					e->data[e->total].alt[j].freq=data[2];
					e->data[e->total].alt[j].freq<<=8;
					e->data[e->total].alt[j].freq|=data[3];
					e->data[e->total].alt[j].freq<<=8;
					e->data[e->total].alt[j].freq|=data[4];
					e->data[e->total].alt[j].freq<<=8;
					e->data[e->total].alt[j].freq|=data[5];
					e->data[e->total].alt[j].freq*=10ULL;
					e->data[e->total].alt[j].istransposer=0;
					e->data[e->total].alttotal=j+1;
				}
				i=data[6];
				l-=7;
				data+=7;
				len-=7;
				k-=7;
				goto t2loop;
			}
			data+=l;
			len-=l;
			k-=l;
		}

		if(e->data[e->total].tune.msys==SATIP_DVBS)
			if(e->data[e->total].tune.mtype==SATIP_8PSK||
				e->data[e->total].tune.ro==SATIP_ROFF_025||
				e->data[e->total].tune.ro==SATIP_ROFF_020||
				e->data[e->total].tune.fec==SATIP_FEC_35||
				e->data[e->total].tune.fec==SATIP_FEC_910)
					e->data[e->total].tune.msys=SATIP_DVBS2;

		if(e->data[e->total].tune.msys!=SATIP_UNSPEC)e->total++;
	}
	return e;
}

SATIP_UTIL_SDT *satip_util_unpack_sdt_section(unsigned char *data,int len)
{
	int i;
	int j;
	int k;
	int l;
	unsigned char *ptr;
	SATIP_UTIL_SDT *e;

	if(len<15)return NULL;

	i=data[1]&0xf;
	i<<=8;
	i|=data[2];
	if(len!=i+3)return NULL;

	i=0;
	j=len-15;
	ptr=data+11;
	while(j>0)
	{
		if(j<5)return NULL;
		i++;
		k=ptr[3]&0x0f;
		k<<=8;
		k|=ptr[4];
		ptr+=5;
		j-=5;
		while(k>0)
		{
			if(j<2)return NULL;
			l=ptr[1]+2;
			if(j<l)return NULL;
			ptr+=l;
			j-=l;
			k-=l;
		}
	}

	if(!(e=malloc(sizeof(SATIP_UTIL_SDT)+sizeof(e->data[0])*i)))
		return NULL;

	e->next=NULL;
	e->tsid=data[3];
	e->tsid<<=8;
	e->tsid|=data[4];
	e->vernum=(data[5]>>1)&0x1f;
	e->cnind=data[5]&0x01;
	e->secnum=data[6];
	e->lastsec=data[7];
	e->onid=data[8];
	e->onid<<=8;
	e->tsid|=data[9];
	e->total=0;

	len-=15;
	data+=11;

	while(len>0)
	{
		e->data[e->total].prognum=data[0];
		e->data[e->total].prognum<<=8;
		e->data[e->total].prognum|=data[1];
		e->data[e->total].eitsched=(data[2]&0x02)?1:0;
		e->data[e->total].eitpres=(data[2]&0x01);
		switch(data[2]&0xe0)
		{
		case 0x20:
			e->data[e->total].running=SATIP_SDT_NORUN;
			break;
		case 0x40:
			e->data[e->total].running=SATIP_SDT_START;
			break;
		case 0x60:
			e->data[e->total].running=SATIP_SDT_PAUSE;
			break;
		case 0x80:
			e->data[e->total].running=SATIP_SDT_RUN;
			break;
		case 0xa0:
			e->data[e->total].running=SATIP_SDT_OFAIR;
			break;
		default:e->data[e->total].running=SATIP_UNSPEC;
		}
		e->data[e->total].fta=(data[3]&0x10)?0:1;
		e->data[e->total].type=0;
		e->data[e->total].catotal=0;
		e->data[e->total].provname[0]=0;
		e->data[e->total].servname[0]=0;
		k=data[3]&0x0f;
		k<<=8;
		k|=data[4];
		data+=5;
		len-=5;
		while(k>0)
		{
			l=data[1]+2;
			switch(data[0])
			{
			case 0x48:
				switch(data[2])
				{
				case 0x01:
				case 0x04:
				case 0x05:
					e->data[e->total].type=SATIP_SDT_MP2SD;
					break;
				case 0x11:
					e->data[e->total].type=SATIP_SDT_MP2HD;
					break;
				case 0x16:
				case 0x17:
				case 0x18:
					e->data[e->total].type=SATIP_SDT_ACSD;
					break;
				case 0x19:
				case 0x1a:
				case 0x1b:
					e->data[e->total].type=SATIP_SDT_ACHD;
					break;
				case 0x02:
					e->data[e->total].type=SATIP_SDT_MP1AU;
					break;
				case 0x0a:
					e->data[e->total].type=SATIP_SDT_ACAU;
					break;
				case 0x07:
					e->data[e->total].type=SATIP_SDT_FMAU;
					break;
				case 0x0d:
					e->data[e->total].type=SATIP_SDT_CI;
					break;
				case 0x03:
					e->data[e->total].type=SATIP_SDT_TTX;
					break;
				case 0x0c:
					e->data[e->total].type=SATIP_SDT_DATA;
					break;
				case 0x06:
				case 0x08:
				case 0x0b:
				case 0x0e:
				case 0x0f:
				case 0x10:
					e->data[e->total].type=SATIP_SDT_OTHER;
					break;
				}
				if(chrcpy(e->data[e->total].provname,
					sizeof(e->data[e->total].provname),
					data+4,data[3]))
						e->data[e->total].provname[0]=0;
				i=data[3]+4;
				if(chrcpy(e->data[e->total].servname,
					sizeof(e->data[e->total].servname),
					data+i+1,data[i]))
						e->data[e->total].servname[0]=0;
				break;

			case 0x53:
				l-=2;
				data+=2;
				len-=2;
				k-=2;
				while(l>=2)
				{
					j=e->data[e->total].catotal;
					if(j<SATIP_SDTCA_MAX)
					{
						e->data[e->total].caid[j]=
							data[0];
						e->data[e->total].caid[j]<<=8;
						e->data[e->total].caid[j]|=
							data[1];
						e->data[e->total].catotal=j+1;
					}
					l-=2;
					data+=2;
					len-=2;
					k-=2;
				}
				break;
			}
			data+=l;
			len-=l;
			k-=l;
		}
		e->total++;
	}

	return e;
}

unsigned char *satip_util_get_raw_cat(SATIP_UTIL_CAT *cat,int offset)
{
	return ((unsigned char *)cat)+offset;
}

unsigned char *satip_util_get_raw_pmt(SATIP_UTIL_PMT *pmt,int offset)
{
	return ((unsigned char *)pmt)+offset;
}

int satip_util_get_raw_len(unsigned char *ptr)
{
	if(ptr[1]==0x80||ptr[1]>0x82)return 0;
	switch(ptr[1])
	{
	case 0x81:
		return ((int)ptr[2])+3;
	case 0x82:
		return ((((int)ptr[2])<<8)|((int)ptr[3]))+4;
	default:return ((int)ptr[1])+2;
	}
}

int satip_util_list_init(void **list)
{
	LIST **l=(LIST **)list;

	if(!l)goto err1;
	if(!(*l=malloc(sizeof(LIST))))goto err1;
	memset(*l,0,sizeof(LIST));
	if(pthread_spin_init(&(*l)->mtx,PTHREAD_PROCESS_PRIVATE))goto err2;
	return 0;

err2:	free(*list);
err1:	return SATIP_SYSFAIL;
}

int satip_util_list_free(void *list)
{
	LIST *l=(LIST *)list;
	ENTRY *e;

	if(!l)return SATIP_SYSFAIL;

	while(l->list)
	{
		e=l->list;
		l->list=e->next;
		free(e);
	}
	pthread_spin_destroy(&l->mtx);
	free(l);

	return 0;
}

int satip_util_list_add(void *list,char *addr)
{
	LIST *l=(LIST *)list;
	ENTRY *e;
	ENTRY **x;
	ADDR a;
	ADDR m;

	if(!list||!addr)return SATIP_SYSFAIL;

	if(parsemaskaddr(addr,&a,&m))return SATIP_SYSFAIL;

	if(!(e=malloc(sizeof(ENTRY))))return SATIP_SYSFAIL;

	e->next=NULL;
	e->addr=a;
	e->mask=m;

	pthread_spin_lock(&l->mtx);
	for(x=&l->list;*x;x=&(*x)->next);
	*x=e;
	l->total++;
	pthread_spin_unlock(&l->mtx);

	return 0;
}

int satip_util_list_del(void *list,char *addr)
{
	LIST *l=(LIST *)list;
	ENTRY *e;
	ENTRY **x;
	ADDR a;
	ADDR m;

	if(!list||!addr)return SATIP_SYSFAIL;

	if(parsemaskaddr(addr,&a,&m))return SATIP_SYSFAIL;

	pthread_spin_lock(&l->mtx);
	for(x=&l->list;*x;x=&(*x)->next)
		if(!aacmp(&(*x)->addr,&a)&&!aacmp(&(*x)->mask,&m))
	{
		l->total--;
		e=*x;
		*x=e->next;
		free(e);
		pthread_spin_unlock(&l->mtx);
		return 0;
	}
	pthread_spin_unlock(&l->mtx);

	return SATIP_SYSFAIL;
}

int satip_util_list_match_data(void *list,SATIP_DATA *data)
{
	LIST *l=(LIST *)list;
	ADDR a;

	if(!list||!data||!data->ptrval)return SATIP_SYSFAIL;
	if(data->intval!=AF_INET&&data->intval!=AF_INET6)return SATIP_SYSFAIL;
	a.family=0;
	sock2addr(&a,(SOCK *)(data->ptrval));
	if(!a.family)return SATIP_SYSFAIL;

	return matchaddr(l,&a)?SATIP_NOMATCH:0;
}

int satip_util_list_match_addr(void *list,char *addr)
{
	LIST *l=(LIST *)list;
	ADDR a;

	if(!list||!addr)return SATIP_SYSFAIL;
	if(str2addr(addr,&a))return SATIP_SYSFAIL;

	return matchaddr(l,&a)?SATIP_NOMATCH:0;
}

int satip_util_list_total(void *list)
{
	int total;
	LIST *l=(LIST *)list;

	if(!l)return SATIP_SYSFAIL;

	pthread_spin_lock(&l->mtx);
	total=l->total;
	pthread_spin_unlock(&l->mtx);

	return total;
}

int satip_util_data2addr(SATIP_DATA *data,char *addr,int size)
{
	ADDR a;

	if(!addr||!data||!data->ptrval)return SATIP_SYSFAIL;
	if(data->intval!=AF_INET&&data->intval!=AF_INET6)return SATIP_SYSFAIL;
	a.family=0;
	sock2addr(&a,(SOCK *)(data->ptrval));
	if(!a.family)return SATIP_SYSFAIL;
	if(!addr2str(&a,addr,size))return SATIP_SYSFAIL;
	return 0;
}

int satip_util_addrinc(char *addr,int size)
{
	ADDR a;

	if(str2addr(addr,&a))return SATIP_SYSFAIL;
	if(aainc(&a))return SATIP_SYSFAIL;
	if(!addr2str(&a,addr,size))return SATIP_SYSFAIL;
	return 0;
}

int satip_util_filter_create(void **filter)
{
	FILTER *f;

	if(!filter)goto err1;
	if(!(f=malloc(sizeof(FILTER))))goto err1;
	if(pthread_spin_init(&f->mtx,PTHREAD_PROCESS_PRIVATE))goto err2;
	f->user=NULL;
	*filter=f;
	return 0;

err2:	free(f);
err1:	return SATIP_SYSFAIL;
}

int satip_util_filter_free(void *filter)
{
	FILTER *f=(FILTER *)filter;
	USER *u;

	if(!filter)return SATIP_SYSFAIL;
	pthread_spin_lock(&f->mtx);
	while(f->user)
	{
		u=f->user;
		f->user=u->next;
		pthread_spin_destroy(&u->mtx);
		free(u);
	}
	pthread_spin_unlock(&f->mtx);
	pthread_spin_destroy(&f->mtx);
	free(f);
	return 0;
}

int satip_util_filter_packets(void *filter,void *data,int len)
{
	int pid;
	int bits;
	unsigned long long bit;
	FILTER *f=(FILTER *)filter;
	unsigned char *ptr;
	USER *u;

	if(!filter||UNLIKELY((len&SATIP_RTCP)&&data))return SATIP_SYSFAIL;

	if(UNLIKELY(!data))
	{
		for(u=f->user;u;u=u->next)
		{
			pthread_spin_lock(&u->mtx);
			u->cb(NULL,0,u->priv);
			pthread_spin_unlock(&u->mtx);
		}
		return 0;
	}

	bits=(len&0xffff0000)|188;
	len&=0x0000ffff;
	pthread_spin_lock(&f->mtx);
	for(ptr=(unsigned char *)data;len>=188;ptr+=188,len-=188)
	{
		pid=(ptr[1]&0x1f)<<2;
		pid|=ptr[2]>>6;
		bit=1ULL<<(ptr[2]&0x3f);
		for(u=f->user;u;u=u->next)
		{
			pthread_spin_lock(&u->mtx);
			if((u->map[pid]&bit)||u->all)u->cb(ptr,bits,u->priv);
			pthread_spin_unlock(&u->mtx);
		}
		bits=SATIP_RTP|188;
	}
	pthread_spin_unlock(&f->mtx);
	return 0;
}

int satip_util_filter_packets_cb(void *filter,void *data,int len,
	void (*next)(void *priv),void *priv)
{
	int pid;
	int bits;
	unsigned long long bit;
	FILTER *f=(FILTER *)filter;
	unsigned char *ptr;
	USER *u;

	if(!filter||UNLIKELY((len&SATIP_RTCP)&&data))return SATIP_SYSFAIL;

	if(UNLIKELY(!data))
	{
		for(u=f->user;u;u=u->next)
		{
			pthread_spin_lock(&u->mtx);
			u->cb(NULL,0,u->priv);
			pthread_spin_unlock(&u->mtx);
		}
		return 0;
	}

	bits=(len&0xffff0000)|188;
	len&=0x0000ffff;
	pthread_spin_lock(&f->mtx);
	for(ptr=(unsigned char *)data;len>=188;ptr+=188,len-=188)
	{
		next(priv);
		pid=(ptr[1]&0x1f)<<2;
		pid|=ptr[2]>>6;
		bit=1ULL<<(ptr[2]&0x3f);
		for(u=f->user;u;u=u->next)
		{
			pthread_spin_lock(&u->mtx);
			if((u->map[pid]&bit)||u->all)u->cb(ptr,bits,u->priv);
			pthread_spin_unlock(&u->mtx);
		}
		bits=SATIP_RTP|188;
	}
	pthread_spin_unlock(&f->mtx);
	return 0;
}

int satip_util_filter_add_user(void *filter,void **user,
	void (*cb)(void *data,int len,void *priv),void *priv)
{
	FILTER *f=(FILTER *)filter;
	USER *u;

	if(!filter||!user||!cb)goto err1;
	if(!(u=malloc(sizeof(USER))))goto err1;
	memset(u,0,sizeof(USER));
	if(pthread_spin_init(&u->mtx,PTHREAD_PROCESS_PRIVATE))goto err2;
	u->cb=cb;
	u->priv=priv;
	pthread_spin_lock(&f->mtx);
	u->next=f->user;
	f->user=u;
	pthread_spin_unlock(&f->mtx);
	*user=u;
	return 0;

err2:	free(u);
err1:	return SATIP_SYSFAIL;
}

int satip_util_filter_del_user(void *filter,void *user)
{
	FILTER *f=(FILTER *)filter;
	USER *u=(USER *)user;
	USER **d;

	if(!filter||!user)return SATIP_SYSFAIL;
	pthread_spin_lock(&f->mtx);
	for(d=&f->user;*d;d=&(*d)->next)if(*d==u)
	{
		*d=u->next;
		pthread_spin_unlock(&f->mtx);
		pthread_spin_destroy(&u->mtx);
		free(u);
		return 0;
	}
	pthread_spin_unlock(&f->mtx);
	return SATIP_SYSFAIL;
}

int satip_util_user_addpid(void *user,int pid)
{
	USER *u=(USER *)user;
	unsigned long long bit;

	if(!user||pid<0||pid>8192)return SATIP_SYSFAIL;
	bit=1ULL<<(pid&0x3f);
	pid>>=6;
	pthread_spin_lock(&u->mtx);
	if(pid==8192)u->all=1;
	else u->map[pid]|=bit;
	pthread_spin_unlock(&u->mtx);
	return 0;
}

int satip_util_user_delpid(void *user,int pid)
{
	USER *u=(USER *)user;
	unsigned long long bit;

	if(!user||pid<0||pid>8192)return SATIP_SYSFAIL;
	bit=1ULL<<(pid&0x3f);
	bit^=-1;
	pid>>=6;
	pthread_spin_lock(&u->mtx);
	if(pid==8192)u->all=0;
	else u->map[pid]&=bit;
	pthread_spin_unlock(&u->mtx);
	return 0;
}

int satip_util_user_clrpid(void *user)
{
	USER *u=(USER *)user;

	if(!user)return SATIP_SYSFAIL;
	pthread_spin_lock(&u->mtx);
	memset(u->map,0,sizeof(u->map));
	pthread_spin_unlock(&u->mtx);
	return 0;
}

int satip_util_section_create(void **section,SATIP_UTIL_SECFILTER *filter,
	int checkcrc,void (*cb)(void *data,int len,void *priv),void *priv)
{
	int j;
	SECTION *s;

	if(!section||!cb)goto err1;

	if(!(s=malloc(sizeof(SECTION))))goto err1;
	s->fill=0;
	s->init=1;
	s->dup=0;
	s->len=0;
	s->crc=checkcrc;
	s->flen=0;
	s->neq=0;
	s->cb=cb;
	s->priv=priv;
	if(!filter)s->flen=0;
	else
	{
		for(j=SATIP_FILTERSZ-1;j>=0;j--)if(filter->mask[j])break;
		s->flen=j+1;
		if(s->flen)for(s->neq=0,j=0;j<s->flen;j++)
		{
			s->flt[j]=filter->filter[j];
			s->mn[j]=filter->mask[j]&~filter->mode[j];
			s->neq|=(s->mm[j]=filter->mask[j]&filter->mode[j])?1:0;
		}
	}
	*section=s;
	return 0;

err1:	return SATIP_SYSFAIL;
}

int satip_util_section_free(void *section)
{
	SECTION *s=(SECTION *)section;

	if(!section)return SATIP_SYSFAIL;

	free(s);
	return 0;
}

int satip_util_section_reset(void *section)
{
	SECTION *s=(SECTION *)section;

	if(!section)return SATIP_SYSFAIL;

	s->init=1;
	s->fill=0;
	s->dup=0;
	s->len=0;
	return 0;
}

void satip_util_section_packet(void *data,int len,void *section)
{
	SECTION *s=(SECTION *)section;
	unsigned char *ptr=(unsigned char *)data;

	len&=0x0000ffff;

	if(!data||!section||len!=188)return;

	if(ptr[1]&0x80)goto fail;

	if(!s->init)
	{
		if(s->cont==(ptr[3]&0xf))
		{
			if(++s->dup>1)goto fail;
			return;
		}
		s->cont=(s->cont+1)&0xf;
		if(s->cont!=(ptr[3]&0xf))goto fail;
		s->dup=0;
	}

	if(ptr[1]&0x40)
	{
		s->init=1;
		s->cont=ptr[3]&0xf;
	}
	else if(s->init)return;

	switch(ptr[3]&0x30)
	{
	case 0x00:
		goto fail;
	case 0x20:
		return;
	case 0x30:
		if(len<ptr[4]+5)goto fail;
		if(ptr[4])if(ptr[5]&0x80)goto fail;
		len-=ptr[4]+5;
		ptr+=ptr[4]+5;
		break;
	case 0x10:
		ptr+=4;
		len-=4;
		break;
	}

	if(s->init)
	{
		if(!len)goto fail;
		if(len<ptr[0]+1)goto fail;

		if(ptr[0]&&s->len)if(s->fill+ptr[0]<=sizeof(s->bfr))
		{
			memcpy(s->bfr+s->fill,ptr+1,ptr[0]);
			s->fill+=ptr[0];
			if(s->fill>=s->len)section_finish(s);
		}

		len-=ptr[0]+1;
		ptr+=ptr[0]+1;
		s->init=0;
		s->fill=0;
		s->dup=0;
		s->len=0;
	}

	if(s->fill+len>sizeof(s->bfr))goto fail;

	if(!len)return;

	memcpy(s->bfr+s->fill,ptr,len);
	s->fill+=len;

	if(!s->len&&s->fill>=3)
	{
		s->len=s->bfr[1]&0x0f;
		s->len<<=8;
		s->len|=s->bfr[2];
		s->len+=3;
		if(s->len<12)goto fail;
	}

	if(s->fill<s->len)return;

	section_finish(s);

fail:	s->init=1;
	s->fill=0;
	s->dup=0;
	s->len=0;
}

void satip_util_init_tune(SATIP_TUNE *tune)
{
	tune->freq=SATIP_UNSPEC;
	tune->fe=SATIP_UNSPEC;
	tune->src=SATIP_UNSPEC;
	tune->bw=SATIP_BW_AUTO;
	tune->pol=SATIP_UNSPEC;
	tune->msys=SATIP_UNSPEC;
	tune->tmode=SATIP_TMOD_AUTO;
	tune->mtype=SATIP_AUTOQ;
	tune->plts=SATIP_PLTS_AUTO;
	tune->ro=SATIP_ROFF_AUTO;
	tune->sr=SATIP_UNSPEC;
	tune->gi=SATIP_GI_AUTO;
	tune->fec=SATIP_FEC_AUTO;
	tune->feclp=SATIP_FEC_AUTO;
	tune->hier=SATIP_HIER_AUTO;
	tune->c2tft=SATIP_TFT_AUTO;
	tune->ds=SATIP_UNDEF;
	tune->plp=SATIP_UNDEF;
	tune->t2id=SATIP_UNDEF;
	tune->sm=SATIP_SM_AUTO;
	tune->specinv=SATIP_SPI_AUTO;
}

void satip_util_init_pids(SATIP_PIDS *set)
{
	memset(set,0,sizeof(SATIP_PIDS));
	set->numpids=SATIP_NOPIDS;
}

int satip_util_tunecmp(SATIP_TUNE *t1,SATIP_TUNE *t2)
{
	if(!t1||!t2)return SATIP_SYSFAIL;

	switch(t1->msys)
	{
	case SATIP_DVBS:
	case SATIP_DVBS2:
		if(t2->msys!=SATIP_DVBS&&t2->msys!=SATIP_DVBS2)
			return SATIP_NOMATCH;
		if(t1->freq-10000000ULL>t2->freq||
			t1->freq+10000000ULL<t2->freq)return SATIP_NOMATCH;
		if(t2->freq-10000000ULL>t1->freq||
			t2->freq+10000000ULL<t1->freq)return SATIP_NOMATCH;
		if(t1->fe!=t2->fe&&t1->fe!=SATIP_UNSPEC&&t2->fe!=SATIP_UNSPEC)
			return SATIP_NOMATCH;
		if(t1->src!=t2->src||t1->pol!=t2->pol)return SATIP_NOMATCH;
		if(t1->mtype!=t2->mtype&&t1->mtype!=SATIP_AUTOQ&&
			t2->mtype!=SATIP_AUTOQ)return SATIP_NOMATCH;
		if(t1->plts!=t2->plts&&t1->plts!=SATIP_PLTS_AUTO&&
			t2->plts!=SATIP_PLTS_AUTO)return SATIP_NOMATCH;
		if(t1->ro!=t2->ro&&t1->ro!=SATIP_ROFF_AUTO&&
			t2->ro!=SATIP_ROFF_AUTO)return SATIP_NOMATCH;
		if(t1->sr-300000>t2->sr||t1->sr+300000<t2->sr)
			return SATIP_NOMATCH;
		if(t2->sr-300000>t1->sr||t2->sr+300000<t1->sr)
			return SATIP_NOMATCH;
		if(t1->fec!=t2->fec&&t1->fec!=SATIP_FEC_AUTO&&
			t2->fec!=SATIP_FEC_AUTO)return SATIP_NOMATCH;
		return 0;
	case SATIP_DVBT:
	case SATIP_DVBT2:
		if(t2->msys!=SATIP_DVBT&&t2->msys!=SATIP_DVBT2)
			return SATIP_NOMATCH;
		if(t1->freq-2000000ULL>t2->freq||
			t1->freq+2000000ULL<t2->freq)return SATIP_NOMATCH;
		if(t2->freq-2000000ULL>t1->freq||
			t2->freq+2000000ULL<t1->freq)return SATIP_NOMATCH;
		if(t1->fe!=t2->fe&&t1->fe!=SATIP_UNSPEC&&t2->fe!=SATIP_UNSPEC)
			return SATIP_NOMATCH;
		if(t1->bw!=t2->bw&&t1->bw!=SATIP_BW_AUTO&&t2->bw!=SATIP_BW_AUTO)
			return SATIP_NOMATCH;
		if(t1->tmode!=t2->tmode&&t1->tmode!=SATIP_TMOD_AUTO&&
			t2->tmode!=SATIP_TMOD_AUTO)return SATIP_NOMATCH;
		if(t1->mtype!=t2->mtype&&t1->mtype!=SATIP_AUTOQ&&
			t2->mtype!=SATIP_AUTOQ)return SATIP_NOMATCH;
		if(t1->gi!=t2->gi&&t1->gi!=SATIP_GI_AUTO&&t2->gi!=SATIP_GI_AUTO)
			return SATIP_NOMATCH;
		if(t1->fec!=t2->fec&&t1->fec!=SATIP_FEC_AUTO&&
			t2->fec!=SATIP_FEC_AUTO)return SATIP_NOMATCH;
		if(t1->feclp!=t2->feclp&&t1->feclp!=SATIP_FEC_AUTO&&
			t2->feclp!=SATIP_FEC_AUTO)return SATIP_NOMATCH;
		if(t1->hier!=t2->hier&&t1->hier!=SATIP_HIER_AUTO&&
			t2->hier!=SATIP_HIER_AUTO)return SATIP_NOMATCH;
		if(t1->plp!=t2->plp&&t1->plp!=SATIP_UNDEF&&t2->plp!=SATIP_UNDEF)
			return SATIP_NOMATCH;
		if(t1->t2id!=t2->t2id&&t1->t2id!=SATIP_UNDEF&&
			t2->t2id!=SATIP_UNDEF)return SATIP_NOMATCH;
		if(t1->sm!=t2->sm&&t1->sm!=SATIP_SM_AUTO&&t2->sm!=SATIP_SM_AUTO)
			return SATIP_NOMATCH;
		return 0;
	case SATIP_DVBC:
	case SATIP_DVBC2:
		if(t2->msys!=SATIP_DVBC&&t2->msys!=SATIP_DVBC2)
			return SATIP_NOMATCH;
		if(t1->freq-SATIP_SEARCHMHZ*1000000ULL>t2->freq||
			t1->freq+SATIP_SEARCHMHZ*1000000ULL<t2->freq)
			return SATIP_NOMATCH;
		if(t2->freq-SATIP_SEARCHMHZ*1000000ULL>t1->freq||
			t2->freq+SATIP_SEARCHMHZ*1000000ULL<t1->freq)
			return SATIP_NOMATCH;
		if(t1->fe!=t2->fe&&t1->fe!=SATIP_UNSPEC&&t2->fe!=SATIP_UNSPEC)
			return SATIP_NOMATCH;
		if(t1->bw!=t2->bw&&t1->bw!=SATIP_BW_AUTO&&t2->bw!=SATIP_BW_AUTO)
			return SATIP_NOMATCH;
		if(t1->mtype!=t2->mtype&&t1->mtype!=SATIP_AUTOQ&&
			t2->mtype!=SATIP_AUTOQ)return SATIP_NOMATCH;
		if(t1->sr-SATIP_SEARCHMHZ*1000000ULL>t2->sr||
			t1->sr+SATIP_SEARCHMHZ*1000000ULL<t2->sr)
			return SATIP_NOMATCH;
		if(t2->sr-SATIP_SEARCHMHZ*1000000ULL>t1->sr||
			t2->sr+SATIP_SEARCHMHZ*1000000ULL<t1->sr)
			return SATIP_NOMATCH;
		if(t1->c2tft!=t2->c2tft&&t1->c2tft!=SATIP_TFT_AUTO&&
			t2->c2tft!=SATIP_TFT_AUTO)return SATIP_NOMATCH;
		if(t1->ds!=t2->ds&&t1->ds!=SATIP_UNDEF&&t2->ds!=SATIP_UNDEF)
			return SATIP_NOMATCH;
		if(t1->plp!=t2->plp&&t1->plp!=SATIP_UNDEF&&t2->plp!=SATIP_UNDEF)
			return SATIP_NOMATCH;
		if(t1->specinv!=t2->specinv&&t1->specinv!=SATIP_SPI_AUTO&&
			t2->specinv!=SATIP_SPI_AUTO)return SATIP_NOMATCH;
		return 0;
	}

	return SATIP_NOMATCH;
}

int satip_util_program_list_create(SATIP_UTIL_PROGRAM ***list,int *size)
{
	*list=NULL;
	*size=0;
	return 0;
}

int satip_util_program_list(SATIP_SCAN_RESULT *res,int tot,char *user_locale,
	int sdtcaid,SATIP_UTIL_PROGRAM ***list,int *size)
{
	int i;
	int j;
	int k;
	int m;
	int s;
	int c;
	int n;
	int pid;
	int pcr;
	int tsid;
	int nid;
	int total=0;
	int alloc=0;
	unsigned char *ptr;
	SATIP_UTIL_PAT *pat;
	SATIP_UTIL_PMT *pmt;
	SATIP_UTIL_PROGRAM *e;
	SATIP_UTIL_PROGRAM **l=NULL;
	SATIP_UTIL_PROGRAM **tmp;
	PROGSORT p={NULL,(iconv_t)-1};

	if(!res||tot<0||!user_locale||!list||!size)return SATIP_SYSFAIL;

	if(!(p.locale=newlocale(LC_ALL_MASK,user_locale,NULL)))goto fail;
	if((p.ic=iconv_open("WCHAR_T//TRANSLIT","UTF8"))==(iconv_t)-1)goto fail;

	for(i=0;i<tot;i++)for(pat=res[i].pat;pat;pat=pat->next)
		for(j=0;j<pat->total;j++)
	{
		for(s=0,pmt=pat->data[j].pmt;pmt;pmt=pmt->next)s+=pmt->total;
		if(s>511)s=511;
		if(!s)continue;
		pmt=pat->data[j].pmt;
		pid=pat->data[j].pmtpid;
		pcr=pmt->pcrpid;
		if(res->nit[0])nid=res->nit[0]->netid;
		else nid=0;
		if(pmt->sdt)
		{
			tsid=pmt->sdt->tsid;
			if(!nid)nid=pmt->sdt->onid;
		}
		else tsid=0;

		if(total==alloc)
		{
			alloc+=64;
			if(!(tmp=realloc(l,alloc*sizeof(e))))goto fail;
			l=tmp;
		}

		k=sizeof(SATIP_UTIL_PROGRAM)+s*sizeof(e->stream[0]);
		if(pmt->sdt)c=strlen(pmt->sdt->data[pmt->sdtindex].servname);
		else c=0;
		if(c)
		{
			for(m=0;pmt->sdt->data[pmt->sdtindex].servname[m];m++)
			    if(pmt->sdt->data[pmt->sdtindex].servname[m]!=' '&&
				pmt->sdt->data[pmt->sdtindex].servname[m]!='\t')
					break;
			if(!pmt->sdt->data[pmt->sdtindex].servname[m])c=0;
		}
		if(c)k+=c+1;
		else k+=7;
		if(pmt->sdt)n=strlen(pmt->sdt->data[pmt->sdtindex].provname);
		else n=0;
		if(n)
		{
			for(m=0;pmt->sdt->data[pmt->sdtindex].provname[m];m++)
			    if(pmt->sdt->data[pmt->sdtindex].provname[m]!=' '&&
				pmt->sdt->data[pmt->sdtindex].provname[m]!='\t')
					break;
			if(!pmt->sdt->data[pmt->sdtindex].provname[m])n=0;
		}
		if(n)k+=n+1;
		else k+=7;

		if(!(e=malloc(k)))goto fail;
		l[total++]=e;

		memset(e,0,k);
		e->prognum=pmt->prognum;
		e->pmtpid=pid;
		e->pcrpid=pcr;
		e->tsid=tsid;
		e->nid=nid;
		e->streams=s;

		e->catotal=(e->catotal>SATIP_SDTCA_MAX)?
			SATIP_SDTCA_MAX:pmt->catotal;
		for(m=0;m<e->catotal&&m<SATIP_SDTCA_MAX;m++)
			e->caid[m]=pmt->caid[m];

		if(pmt->sdt)
		{
			e->sdttype=pmt->sdt->data[pmt->sdtindex].type;
			e->sdtfta=pmt->sdt->data[pmt->sdtindex].fta;
			if(pmt->sdt->data[pmt->sdtindex].eitpres||
				pmt->sdt->data[pmt->sdtindex].eitsched)e->eit=1;
			if(!pmt->sdt->data[pmt->sdtindex].catotal)e->fta=1;

			if(sdtcaid)
			    for(m=0;m<pmt->sdt->data[pmt->sdtindex].catotal;m++)
			{
				if(e->catotal==SATIP_SDTCA_MAX)break;
				for(n=0;n<e->catotal;n++)if(e->caid[n]==
					pmt->sdt->data[pmt->sdtindex].caid[m])
						break;
				if(n!=e->catotal)continue;
				e->caid[e->catotal++]=
					pmt->sdt->data[pmt->sdtindex].caid[m];
			}
		}
		else e->fta=1;
		if(pmt->catotal)e->fta=0;
		e->progname=((char *)e)+sizeof(SATIP_UTIL_PROGRAM)+
			(pmt->total*sizeof(e->stream[0]));
		if(c)e->provider=e->progname+c+1;
		else e->provider=e->progname+7;
		if(c)
		{
			for(m=0,ptr=pmt->sdt->data[pmt->sdtindex].servname;
				*ptr;)switch(ptr[0])
			{
			case 0xee:
				switch(ptr[1])
				{
				case 0x82:
					switch(ptr[2])
					{
					case 0x8a:
						e->progname[m++]=' ';
					case 0x86:
					case 0x87:
						ptr+=3;
						continue;
					}
				}
				e->progname[m++]=*ptr++;
				break;
			case 0xc2:
				switch(ptr[1])
				{
				case 0x8a:
					e->progname[m++]=' ';
				case 0x86:
				case 0x87:
					ptr+=2;
					continue;
				}
			default:e->progname[m++]=*ptr++;
			}
			e->progname[m]=0;
		}
		else sprintf(e->progname,"%u.",pmt->prognum);
		if(n)
		{
			for(m=0,ptr=pmt->sdt->data[pmt->sdtindex].provname;*ptr;)
				switch(ptr[0])
			{
			case 0xee:
				switch(ptr[1])
				{
				case 0x82:
					switch(ptr[2])
					{
					case 0x8a:
						e->provider[m++]=' ';
					case 0x86:
					case 0x87:
						ptr+=3;
						continue;
					}
				}
				e->provider[m++]=*ptr++;
				break;
			case 0xc2:
				switch(ptr[1])
				{
				case 0x8a:
					e->provider[m++]=' ';
				case 0x86:
				case 0x87:
					ptr+=2;
					continue;
				}
			default:e->provider[m++]=*ptr++;
			}
			e->provider[m]=0;
		}
		else strcpy(e->provider,"(null)");
		e->tune=res[i].actual;

		for(c=0;pmt;pmt=pmt->next)for(k=0;k<pmt->total;k++)
		{
			if(pmt->data[k].catotal)e->fta=0;
			if(res[i].pidinfo)
				if(res[i].pidinfo[pmt->data[k].pid].present)
			{
				if(res[i].pidinfo[pmt->data[k].pid].ts_scramble)
					e->fta=0;
				if(res[i].pidinfo[pmt->data[k].pid].pes)
					if(res[i].pidinfo[pmt->data[k].pid]
						.pes_scramble)e->fta=0;
			}
			e->stream[c].pmttype=pmt->data[k].type;
			e->stream[c].pmttnum=pmt->data[k].typenum;
			e->stream[c].pid=pmt->data[k].pid;
			for(m=0;pmt->data[k].lang[m];m++)
				if(pmt->data[k].lang[m]>='A'&&
				    pmt->data[k].lang[m]<='Z')
					e->stream[c].lang[m]=
						pmt->data[k].lang[m]+0x20;
			else e->stream[c].lang[m]=pmt->data[k].lang[m];
			e->stream[c].lang[m]=0;
			if(!strcmp(e->stream[c].lang,"alb"))
					strcpy(e->stream[c].lang,"sqi");
			else if(!strcmp(e->stream[c].lang,"arm"))
					strcpy(e->stream[c].lang,"hye");
			else if(!strcmp(e->stream[c].lang,"baq"))
					strcpy(e->stream[c].lang,"eus");
			else if(!strcmp(e->stream[c].lang,"tib"))
					strcpy(e->stream[c].lang,"bod");
			else if(!strcmp(e->stream[c].lang,"bur"))
					strcpy(e->stream[c].lang,"mya");
			else if(!strcmp(e->stream[c].lang,"cze"))
					strcpy(e->stream[c].lang,"ces");
			else if(!strcmp(e->stream[c].lang,"chi"))
					strcpy(e->stream[c].lang,"zho");
			else if(!strcmp(e->stream[c].lang,"wel"))
					strcpy(e->stream[c].lang,"cym");
			else if(!strcmp(e->stream[c].lang,"ger"))
					strcpy(e->stream[c].lang,"deu");
			else if(!strcmp(e->stream[c].lang,"dut"))
					strcpy(e->stream[c].lang,"nld");
			else if(!strcmp(e->stream[c].lang,"gre"))
					strcpy(e->stream[c].lang,"ell");
			else if(!strcmp(e->stream[c].lang,"per"))
					strcpy(e->stream[c].lang,"fas");
			else if(!strcmp(e->stream[c].lang,"fre"))
					strcpy(e->stream[c].lang,"fra");
			else if(!strcmp(e->stream[c].lang,"geo"))
					strcpy(e->stream[c].lang,"kat");
			else if(!strcmp(e->stream[c].lang,"ice"))
					strcpy(e->stream[c].lang,"isl");
			else if(!strcmp(e->stream[c].lang,"mac"))
					strcpy(e->stream[c].lang,"mkd");
			else if(!strcmp(e->stream[c].lang,"mao"))
					strcpy(e->stream[c].lang,"mri");
			else if(!strcmp(e->stream[c].lang,"may"))
					strcpy(e->stream[c].lang,"msa");
			else if(!strcmp(e->stream[c].lang,"rum"))
					strcpy(e->stream[c].lang,"ron");
			else if(!strcmp(e->stream[c].lang,"slo"))
					strcpy(e->stream[c].lang,"slk");
			if(++c==s)goto sort;
		}

sort:		qsort(e->stream,s,sizeof(e->stream[0]),streamsort);
		switch(e->stream[0].pmttype)
		{
		case SATIP_PMT_VIDEO:
		case SATIP_PMT_AVC:
		case SATIP_PMT_HEVC:
			e->tv=1;
			break;
		case SATIP_PMT_AUDIO:
		case SATIP_PMT_ADTS:
		case SATIP_PMT_AC3:
		case SATIP_PMT_EAC3:
		case SATIP_PMT_LATM:
			e->radio=1;
			break;
		default:switch(e->sdttype)
			{
			case SATIP_SDT_MP2SD:
			case SATIP_SDT_MP2HD:
			case SATIP_SDT_ACSD:
			case SATIP_SDT_ACHD:
				e->tv=1;
				break;
			case SATIP_SDT_MP1AU:
			case SATIP_SDT_ACAU:
			case SATIP_SDT_FMAU:
				e->radio=1;
				break;
			}
			break;
		}
	}

	if(!total)
	{
		*list=NULL;
		*size=0;
	}
	else
	{
		if((tmp=realloc(l,total*sizeof(e))))l=tmp;

		for(i=0;i<total;i++)
		{
			l[i]->index=l[i]->tune.freq/1000000ULL;
			l[i]->pgroup=0;
		}
		qsort(l,total,sizeof(*l),dupsort);
		for(i=0;i<total;i++)if(!l[i]->pgroup)for(j=i+1;j<total;j++)
		{
			if(!msys2type(l[i]->tune.msys))
			{
				if(l[i]->tune.src!=l[j]->tune.src)break;
				if(l[i]->tune.pol!=l[j]->tune.pol)break;
				if(l[i]->index<l[j]->index-SATIP_SEARCHMHZ)
					break;
				if(l[i]->index>l[j]->index+SATIP_SEARCHMHZ)
					break;
			}
			else if(l[i]->index!=l[j]->index)break;
			if(l[i]->streams!=l[j]->streams)break;
			for(k=0;k<l[i]->streams;k++)
				if(l[i]->stream[k].pid!=l[j]->stream[k].pid)
					break;
			if(k!=l[i]->streams)break;
			l[j]->pgroup=1;
		}
		for(k=0,i=1;i<total;i++)if(l[i]->pgroup)
		{
			if(!l[i-1]->pgroup)l[i-1]->pgroup=++k;
			l[i]->pgroup=l[i-1]->pgroup;
		}

		qsort_r(l,total,sizeof(e),progsort,&p);
		for(i=0;i<total;i++)l[i]->index=i;

		*list=l;
		*size=total;
	}

	iconv_close(p.ic);
	freelocale(p.locale);

	return 0;

fail:	if(p.ic!=(iconv_t)-1)iconv_close(p.ic);
	if(p.locale)freelocale(p.locale);
	for(i=0;i<total;i++)free(l[i]);
	if(l)free(l);
	return SATIP_SYSFAIL;
}

int satip_util_program_list_free(SATIP_UTIL_PROGRAM **list,int size)
{
	int i;

	if(size<0)return SATIP_SYSFAIL;
	if(!size)return 0;
	if(!list)return SATIP_SYSFAIL;

	for(i=0;i<size;i++)free(list[i]);
	free(list);
	return 0;
}

int satip_util_program_merge(char *user_locale,SATIP_UTIL_PROGRAM **list1,
	int size1,SATIP_UTIL_PROGRAM **list2,int size2,
	SATIP_UTIL_PROGRAM ***result,int *size,
	int (*deletecallback)(SATIP_UTIL_PROGRAM *p1,SATIP_UTIL_PROGRAM *p2))
{
	int i;
	int j;
	int k;
	int total;
	SATIP_UTIL_PROGRAM **l;
	SATIP_UTIL_PROGRAM **tmp;
	PROGSORT p={NULL,(iconv_t)-1};

	if(!result||!size||size1<0||size2<0)return SATIP_SYSFAIL;
	if(size1&&!list1)return SATIP_SYSFAIL;
	if(size2&&!list2)return SATIP_SYSFAIL;

	if(!size1&&!size2)
	{
		*result=NULL;
		*size=0;
		return 0;
	}

	if(!(p.locale=newlocale(LC_ALL_MASK,user_locale,NULL)))goto fail;
	if((p.ic=iconv_open("WCHAR_T//TRANSLIT","UTF8"))==(iconv_t)-1)goto fail;

	if(!(l=malloc(sizeof(SATIP_UTIL_PROGRAM **)*(size1+size2))))goto fail;

	for(total=0,i=0;i<size1;i++)l[total++]=list1[i];
	for(i=0;i<size2;i++)l[total++]=list2[i];

	if(list1)free(list1);
	if(list2)free(list2);

	for(i=0;i<total;i++)
	{
		l[i]->index=l[i]->tune.freq/1000000ULL;
		l[i]->pgroup=0;
	}
	qsort(l,total,sizeof(*l),mergesort);
	for(i=0;i<total;i++)if(l[i])for(j=i+1;j<total;j++)
	{
		if((k=msys2type(l[i]->tune.msys))!=msys2type(l[j]->tune.msys))
			break;
		if(!k)
		{
			if(l[i]->tune.src!=l[j]->tune.src)break;
			if(l[i]->tune.pol!=l[j]->tune.pol)break;
			if(l[i]->index<l[j]->index-SATIP_SEARCHMHZ)break;
			if(l[i]->index>l[j]->index+SATIP_SEARCHMHZ)break;
		}
		else if(l[i]->index!=l[j]->index)break;
		if(l[i]->prognum!=l[j]->prognum)break;
		if(l[i]->pmtpid!=l[j]->pmtpid)break;
		switch(deletecallback(l[i],l[j]))
		{
		case -1:free(l[i]);
			l[i]=l[j];
			l[j]=NULL;
			break;
		case 1:	free(l[j]);
			l[j]=NULL;
			break;
		}
	}
	for(i=0,j=0;i<total;i++)if(l[i])l[j++]=l[i];
	if(i!=j)if((tmp=realloc(l,j*sizeof(*l))))l=tmp;

	total=j;
	qsort(l,total,sizeof(*l),dupsort);
	for(i=0;i<total;i++)if(!l[i]->pgroup)for(j=i+1;j<total;j++)
	{
		if((k=msys2type(l[i]->tune.msys))!=msys2type(l[j]->tune.msys))
			break;
		if(!k)
		{
			if(l[i]->tune.src!=l[j]->tune.src)break;
			if(l[i]->tune.pol!=l[j]->tune.pol)break;
			if(l[i]->index<l[j]->index-SATIP_SEARCHMHZ)break;
			if(l[i]->index>l[j]->index+SATIP_SEARCHMHZ)break;
		}
		else if(l[i]->index!=l[j]->index)break;
		if(l[i]->streams!=l[j]->streams)break;
		for(k=0;k<l[i]->streams;k++)
			if(l[i]->stream[k].pid!=l[j]->stream[k].pid)break;
		if(k!=l[i]->streams)break;
		l[j]->pgroup=1;
	}
	for(k=0,i=1;i<total;i++)if(l[i]->pgroup)
	{
		if(!l[i-1]->pgroup)l[i-1]->pgroup=++k;
		l[i]->pgroup=l[i-1]->pgroup;
	}

	qsort_r(l,total,sizeof(*l),progsort,&p);
	for(i=0;i<total;i++)l[i]->index=i;

	*result=l;
	*size=total;

	iconv_close(p.ic);
	freelocale(p.locale);

	return 0;

fail:	if(p.ic!=(iconv_t)-1)iconv_close(p.ic);
	if(p.locale)freelocale(p.locale);
	return SATIP_SYSFAIL;
}

int satip_util_program_create(SATIP_UTIL_PROGRAM **p,int streams,char *progname,
	char *provider)
{
	int i;
	SATIP_UTIL_PROGRAM *e;

	if(!p||streams<1||!progname||!*progname||!provider)return SATIP_SYSFAIL;
	for(i=0;progname[i];i++)if(progname[i]!=' '&&progname[i]!='\t')break;
	if(!progname[i])return SATIP_SYSFAIL;

	i=sizeof(SATIP_UTIL_PROGRAM)+streams*sizeof(e->stream[0])+
		strlen(progname)+1+strlen(provider)+1;
	if(!(e=malloc(i)))return SATIP_SYSFAIL;
	memset(e,0,i);

	e->progname=((char *)e)+sizeof(SATIP_UTIL_PROGRAM)+
		streams*sizeof(e->stream[0]);
	strcpy(e->progname,progname);
	e->provider=e->progname+strlen(e->progname)+1;
	strcpy(e->provider,provider);
	satip_util_init_tune(&e->tune);
	*p=e;
	return 0;
}

int satip_util_program_append(SATIP_UTIL_PROGRAM ***list,int *size,
	SATIP_UTIL_PROGRAM *p)
{
	SATIP_UTIL_PROGRAM **tmp;

	if(!list||!size||!p||*size<0)return SATIP_SYSFAIL;

	if(!(tmp=realloc(*list,(*size+1)*sizeof(**list))))return SATIP_SYSFAIL;
	tmp[(*size)++]=p;
	*list=tmp;
	return 0;
}

int satip_util_program_remove(SATIP_UTIL_PROGRAM ***list,int *size,int index)
{
	int i;
	SATIP_UTIL_PROGRAM **tmp;

	if(!list||!size||*size<0||index<0||index>=*size)return SATIP_SYSFAIL;

	free((*list)[index]);
	for(i=index+1;i<*size;i++)(*list)[i-1]=(*list)[i];
	if(!--(*size))
	{
		free(*list);
		*list=NULL;
	}
	else if((tmp=realloc(*list,(*size)*sizeof(**list))))*list=tmp;
	return 0;
}

int satip_util_program_extract(SATIP_UTIL_PROGRAM ***list,int *size,int index,
	SATIP_UTIL_PROGRAM **p)
{
	int i;
	SATIP_UTIL_PROGRAM **tmp;

	if(!list||!size||*size<0||index<0||*size<index||!p)return SATIP_SYSFAIL;

	*p=(*list)[index];
	for(i=index+1;i<*size;i++)(*list)[i-1]=(*list)[i];
	if(!--(*size))
	{
		free(*list);
		*list=NULL;
	}
	else if((tmp=realloc(*list,(*size)*sizeof(**list))))*list=tmp;
	return 0;
}

int satip_util_program_free(SATIP_UTIL_PROGRAM *p)
{
	if(!p)return SATIP_SYSFAIL;
	free(p);
	return 0;
}

int satip_util_cvt_utf8(char *charset,char *src,int slen,char **dst,int *dlen)
{
	int alloc;
	size_t ilen;
	size_t olen;
	iconv_t iv;
	char *in;
	char *out;
	char *res;
	char dest[128];

	if(!charset||slen<0||!dst||!dlen)return SATIP_SYSFAIL;
	if(slen&&!src)return SATIP_SYSFAIL;

	if(!slen)
	{
		*dst=NULL;
		*dlen=0;
		return 0;
	}

	alloc=slen<<1;
	snprintf(dest,sizeof(dest),"%s//TRANSLIT",charset);
	if((iv=iconv_open(dest,"UTF-8"))==(iconv_t)-1)return SATIP_SYSFAIL;
	if(!(res=malloc(alloc)))
	{
		iconv_close(iv);
		return SATIP_SYSFAIL;
	}
	in=src;
	ilen=slen;
	out=res;
	olen=alloc;
	if(iconv(iv,&in,&ilen,&out,&olen)==-1)
	{
		iconv_close(iv);
		free(res);
		return SATIP_SYSFAIL;
	}
	iconv_close(iv);
	if((out=realloc(res,alloc-olen)))*dst=out;
	else *dst=res;
	*dlen=alloc-olen;
	return 0;
}

int satip_util_create_rtp_setup(char *host,int port,int flags,
	SATIP_UTIL_PROGRAM *p,char *result,int size)
{
	int i;
	SATIP_PIDS set;
	SATIP_EXTRA extra;

	if(flags&SATIP_ADDXPMT)
	{
		extra.total=1;
		strcpy(extra.name[0],"x_pmt");
		sprintf(extra.value[0],"%d",p->pmtpid);
	}
	else extra.total=0;

	set.prognum=((flags&SATIP_ADDSID)?p->prognum:0);
	for(set.numpids=0,i=0;i<p->streams&&set.numpids<SATIP_MAX_PIDS;i++)
		if(!p->stream[i].m3u_exclude)
			set.pids[set.numpids++]=p->stream[i].pid;
	if(set.numpids&&p->eit&&!p->m3u_eit_exclude&&set.numpids<SATIP_MAX_PIDS)
		set.pids[set.numpids++]=0x12;
	if(!set.numpids)return SATIP_SYSFAIL;
	if(flags&SATIP_ADDPMT)
	{
		for(i=0;i<set.numpids;i++)if(set.pids[i]==p->pmtpid)break;
		if(i==set.numpids)
		{
			if(set.numpids==SATIP_MAX_PIDS)return SATIP_SYSFAIL;
			set.pids[set.numpids++]=p->pmtpid;
		}
	}

	if((i=satip_util_create(SATIP_TYPE_RTP,flags,host,port,0,NULL,NULL,NULL,
		NULL,NULL,result,size))<0)return i;
	if(i+1>=size)return SATIP_SYSFAIL;
	strcpy(result+i,"/");
	result+=i+1;
	size-=i+1;
	if((i=satip_util_create(SATIP_TYPE_QRY,flags,NULL,0,0,&p->tune,&set,
		NULL,NULL,&extra,result,size)<0))return i;

	return 0;
}

int satip_util_create_unicast_m3u(char *host,int port,int type,int flags,
	int crlf,char *charset,
	int (*nextprogram)(SATIP_UTIL_PROGRAM **p,int *user_prognum,void *priv),
	void *priv,char **result,int *size)
{
	int r=SATIP_SYSFAIL;
	int len=0;
	int i;
	int alloc=32768;
	int userprog;
	char *m3u=NULL;
	char *tmp;
	SATIP_UTIL_PROGRAM *p;
	SATIP_PIDS set;
	char baseurl[128];
	SATIP_EXTRA extra;

	if(!host||port<1||port>65535||!nextprogram||!result||!size)
		return SATIP_SYSFAIL;

	if(port==(type==SATIP_TYPE_HTTP?SATIP_HTTP_PORT:SATIP_RTSP_PORT))
		snprintf(baseurl,sizeof(baseurl),"%s://%s/",
		type==SATIP_TYPE_HTTP?"http":"rtsp",host);
	else if(!strchr(host,':'))snprintf(baseurl,sizeof(baseurl),
		"%s://%s:%d/",type==SATIP_TYPE_HTTP?"http":"rtsp",
		host,port);
	else snprintf(baseurl,sizeof(baseurl),"%s://[%s]:%d/",
		type==SATIP_TYPE_HTTP?"http":"rtsp",host,port);

	if(!(m3u=malloc(32768)))goto fail;
	len=snprintf(m3u,alloc,"#EXTM3U%s",crlf?"\r\n":"\n");

	while(nextprogram(&p,&userprog,priv))
	{
		if(!p||userprog<1)goto fail;
		if(p->streams<1)goto fail;

		if(alloc-len<1024)
		{
			alloc+=32768;
			if(!(tmp=realloc(m3u,alloc)))goto fail;
			m3u=tmp;
		}
		set.prognum=((flags&SATIP_ADDSID)?p->prognum:0);
		for(set.numpids=0,r=0;r<p->streams&&set.numpids<SATIP_MAX_PIDS;
			r++)if(!p->stream[r].m3u_exclude)
				set.pids[set.numpids++]=p->stream[r].pid;
		if(set.numpids&&p->eit&&!p->m3u_eit_exclude&&
			set.numpids<SATIP_MAX_PIDS)set.pids[set.numpids++]=0x12;
		if(!set.numpids)
		{
			r=SATIP_SYSFAIL;
			continue;
		}
		if(flags&SATIP_ADDPMT)
		{
			for(i=0;i<set.numpids;i++)if(set.pids[i]==p->pmtpid)
				break;
			if(i==set.numpids)
			{
				if(set.numpids==SATIP_MAX_PIDS)
				{
					r=SATIP_SYSFAIL;
					continue;
				}
				set.pids[set.numpids++]=p->pmtpid;
			}
		}
		if(flags&SATIP_ADDXPMT)
		{
			extra.total=1;
			strcpy(extra.name[0],"x_pmt");
			sprintf(extra.value[0],"%d",p->pmtpid);
		}
		else extra.total=0;
		len+=snprintf(m3u+len,alloc-len,"#EXTINF:0,%d. %s%s%s",userprog,
			p->progname,crlf?"\r\n":"\n",baseurl);
		if((r=satip_util_create(SATIP_TYPE_QRY,flags,NULL,0,0,&p->tune,
			&set,NULL,NULL,&extra,m3u+len,alloc-len))
			==SATIP_ERR_SYNTX)goto fail;
		len+=r;
		r=SATIP_SYSFAIL;
		len+=snprintf(m3u+len,alloc-len,"%s",crlf?"\r\n":"\n");
	}

	if(charset&&strcasecmp(charset,"UTF8")&&strcasecmp(charset,"UTF-8"))
	{
		if(satip_util_cvt_utf8(charset,m3u,len,result,size))goto fail;
		free(m3u);
	}
	else
	{
		if((tmp=realloc(m3u,len)))*result=tmp;
		else *result=m3u;
		*size=len;
	}

	return 0;

fail:	if(m3u)free(m3u);
	return r;
}

int satip_util_create_multicast_m3u(int port,int crlf,char *charset,
	int (*nextprogram)(SATIP_UTIL_PROGRAM **p,int *user_prognum,char *addr,
	int size,void *priv),void *priv,char **result,int *size)
{
	int r=SATIP_SYSFAIL;
	int len=0;
	int alloc=32768;
	int userprog;
	char *m3u=NULL;
	char *tmp;
	SATIP_UTIL_PROGRAM *p;
	char addr[SATIP_ADDR_LEN+1];

	if(port<1||port>65535||(port&1)||!nextprogram||!result||!size)
		return SATIP_SYSFAIL;

	if(!(m3u=malloc(32768)))goto fail;
	len=snprintf(m3u,alloc,"#EXTM3U%s",crlf?"\r\n":"\n");

	while(nextprogram(&p,&userprog,addr,sizeof(addr),priv))
	{
		if(!p||userprog<1)goto fail;
		if(p->streams<1)goto fail;

		if(alloc-len<1024)
		{
			alloc+=32768;
			if(!(tmp=realloc(m3u,alloc)))goto fail;
			m3u=tmp;
		}
		len+=snprintf(m3u+len,alloc-len,"#EXTINF:0,%d. %s%s"
			"rtp://%s:%d%s",userprog,p->progname,crlf?"\r\n":"\n",
			addr,port,crlf?"\r\n":"\n");
	}

	if(charset&&strcasecmp(charset,"UTF8")&&strcasecmp(charset,"UTF-8"))
	{
		if(satip_util_cvt_utf8(charset,m3u,len,result,size))goto fail;
		free(m3u);
	}
	else
	{
		if((tmp=realloc(m3u,len)))*result=tmp;
		else *result=m3u;
		*size=len;
	}

	return 0;

fail:	if(m3u)free(m3u);
	return r;
}
