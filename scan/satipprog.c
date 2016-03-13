/*
 * example SAT>IP program list tool
 *
 * Copyright (c) 2016 Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 */

#include <sys/uio.h>
#include <iconv.h>
#include <signal.h>
#include <stddef.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#ifndef NOCDK
#include <cdk.h>
#endif
#include "satip.h"

#ifndef __GNUC__
#define __attribute__(x)
#endif

static void usage(void) __attribute__ ((noreturn));

typedef struct
{
	SATIP_UTIL_PROGRAM **list;
	int size;
	int idx;
} UNI;

typedef struct
{
	SATIP_UTIL_PROGRAM **list;
	int size;
	int idx;
	void *mclist;
	char addr[64];
	int first;
} MULTI;

#ifndef NOCDK

typedef struct
{
	int size;
	SATIP_UTIL_PROGRAM **list;
	CDKSCREEN *scr;
	CDKSELECTION *s;
	WINDOW *nw;
	char **dlist;
} SEL;

#define CDKOPT "i:"
#define HLP1 "[-i] "
#define HLP2 "-i               interactive mode\n"

static char *choices[2]=
{
	"      ",
	"*     "
};

#else
#define CDKOPT
#define HLP1
#define HLP2
#endif

static void vdr(FILE *fp,SATIP_UTIL_PROGRAM *p,char *satid,char *charset)
{
	int len;
	int i;
	int j;
	size_t ilen;
	size_t olen;
	iconv_t ic;
	char *in;
	char *out;
	char bfr[1024];
	char tmp[1024];

	if(!p->streams)return;

	len=snprintf(bfr,sizeof(bfr),"%s",p->progname);
	for(i=0;i<len;i++)if(bfr[i]==':')bfr[i]='|';
	len+=snprintf(bfr+len,sizeof(bfr)-len,";%s:%llu:",p->provider,
		p->tune.freq/1000000ULL);

	if(charset)
	{
		snprintf(tmp,sizeof(tmp),"%s//TRANSLIT",charset);
		ic=iconv_open(tmp,"UTF-8");
		if(ic!=(iconv_t)-1)
		{
			ilen=strlen(bfr)+1;
			olen=sizeof(tmp);
			in=bfr;
			out=tmp;
			if(iconv(ic,&in,&ilen,&out,&olen)!=(size_t)-1)
				len=snprintf(bfr,sizeof(bfr),"%s",tmp);
			iconv_close(ic);
		}
	}

	if(p->tune.msys==SATIP_DVBS||p->tune.msys==SATIP_DVBS2)
		switch(p->tune.pol)
	{
	case SATIP_POL_H:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"H");
		break;
	case SATIP_POL_V:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"V");
		break;
	case SATIP_POL_L:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"L");
		break;
	case SATIP_POL_R:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"R");
		break;
	}

	if(p->tune.msys==SATIP_DVBT||p->tune.msys==SATIP_DVBT2)
		switch(p->tune.bw)
	{
	case SATIP_BW_1712:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"B1712");
		break;
	case SATIP_BW_5:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"B5");
		break;
	case SATIP_BW_6:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"B6");
		break;
	case SATIP_BW_7:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"B7");
		break;
	case SATIP_BW_8:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"B8");
		break;
	case SATIP_BW_10:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"B10");
		break;
	}

	switch(p->tune.fec)
	{
	case SATIP_FEC_12:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"C12");
		break;
	case SATIP_FEC_23:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"C23");
		break;
	case SATIP_FEC_34:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"C34");
		break;
	case SATIP_FEC_35:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"C35");
		break;
	case SATIP_FEC_45:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"C45");
		break;
	case SATIP_FEC_56:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"C56");
		break;
	case SATIP_FEC_78:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"C78");
		break;
	case SATIP_FEC_89:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"C89");
		break;
	case SATIP_FEC_910:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"C910");
		break;
	}

	if(p->tune.msys==SATIP_DVBT||p->tune.msys==SATIP_DVBT2)
		switch(p->tune.feclp)
	{
	case SATIP_FEC_12:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"D12");
		break;
	case SATIP_FEC_23:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"D23");
		break;
	case SATIP_FEC_34:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"D34");
		break;
	case SATIP_FEC_35:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"D35");
		break;
	case SATIP_FEC_45:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"D45");
		break;
	case SATIP_FEC_56:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"D56");
		break;
	case SATIP_FEC_78:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"D78");
		break;
	case SATIP_FEC_89:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"D89");
		break;
	case SATIP_FEC_910:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"D910");
		break;
	default:len+=snprintf(bfr+len,sizeof(bfr)-len,"D0");
		break;
	}

	if(p->tune.msys==SATIP_DVBT||p->tune.msys==SATIP_DVBT2)
		switch(p->tune.gi)
	{
	case SATIP_GI_14:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"G4");
		break;
	case SATIP_GI_18:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"G8");
		break;
	case SATIP_GI_116:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"G16");
		break;
	case SATIP_GI_132:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"G32");
		break;
	case SATIP_GI_164:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"G64");
		break;
	case SATIP_GI_1128:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"G128");
		break;
	case SATIP_GI_19128:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"G19128");
		break;
	case SATIP_GI_19256:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"G19256");
		break;
	}

	if(p->tune.msys!=SATIP_DVBS&&p->tune.msys!=SATIP_DVBS2)
		switch(p->tune.specinv)
	{
	case SATIP_SPI_OFF:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"I0");
		break;
	case SATIP_SPI_ON:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"I1");
		break;
	}

	switch(p->tune.mtype)
	{
	case SATIP_QPSK:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"M2");
		break;
	case SATIP_8PSK:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"M5");
		break;
	case SATIP_16APSK:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"M6");
		break;
	case SATIP_32APSK:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"M7");
		break;
	case SATIP_16Q:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"M16");
		break;
	case SATIP_32Q:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"M32");
		break;
	case SATIP_64Q:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"M64");
		break;
	case SATIP_128Q:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"M128");
		break;
	case SATIP_256Q:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"M256");
		break;
	default:len+=snprintf(bfr+len,sizeof(bfr)-len,"M999");
		break;
	}

	if(p->tune.msys==SATIP_DVBS||p->tune.msys==SATIP_DVBS2)
		switch(p->tune.ro)
	{
	case SATIP_ROFF_025:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"O25");
		break;
	case SATIP_ROFF_020:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"O20");
		break;
	default:len+=snprintf(bfr+len,sizeof(bfr)-len,"O35");
		break;
	}

	if(p->tune.msys==SATIP_DVBT2&&p->tune.plp!=SATIP_UNDEF)
		len+=snprintf(bfr+len,sizeof(bfr)-len,"P%d",p->tune.plp);

	if(p->tune.msys==SATIP_DVBT||p->tune.msys==SATIP_DVBT2)
		switch(p->tune.tmode)
	{
	case SATIP_TMOD_1K:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"T1");
		break;
	case SATIP_TMOD_2K:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"T2");
		break;
	case SATIP_TMOD_4K:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"T4");
		break;
	case SATIP_TMOD_8K:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"T8");
		break;
	case SATIP_TMOD_16K:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"T16");
		break;
	case SATIP_TMOD_32K:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"T32");
		break;
	}

	switch(p->tune.msys)
	{
	case SATIP_DVBS2:
	case SATIP_DVBT2:
	case SATIP_DVBC2:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"S1");
		break;
	default:len+=snprintf(bfr+len,sizeof(bfr)-len,"S0");
		break;
	}
	len+=snprintf(bfr+len,sizeof(bfr)-len,":%s:%d:",satid,p->tune.sr/1000);

	switch(p->stream[0].pmttype)
	{
	case SATIP_PMT_VIDEO:
	case SATIP_PMT_AVC:
	case SATIP_PMT_HEVC:
		if(p->pcrpid!=p->stream[0].pid) len+=snprintf(bfr+len,
			sizeof(bfr)-len,"%d+%d",p->stream[0].pid,
			p->pcrpid);
		else len+=snprintf(bfr+len,sizeof(bfr)-len,"%d",
			p->stream[0].pid);
		if(p->stream[0].pmttnum)len+=snprintf(bfr+len,sizeof(bfr)-len,
			"=%d:",p->stream[0].pmttnum);
		else len+=snprintf(bfr+len,sizeof(bfr)-len,":");
		break;
	default:len+=snprintf(bfr+len,sizeof(bfr)-len,"0:");
		break;
	}

	for(j=0,i=0;i<p->streams;i++)switch(p->stream[i].pmttype)
	{
	case SATIP_PMT_AUDIO:
	case SATIP_PMT_ADTS:
	case SATIP_PMT_LATM:
		if(j)len+=snprintf(bfr+len,sizeof(bfr)-len,",");
		len+=snprintf(bfr+len,sizeof(bfr)-len,"%d",p->stream[i].pid);
		if(p->stream[i].lang[0]||p->stream[i].pmttnum)
			len+=snprintf(bfr+len,sizeof(bfr)-len,"=");
		if(p->stream[i].lang[0])len+=snprintf(bfr+len,sizeof(bfr)-len,
			"%s",p->stream[i].lang);
		if(p->stream[i].pmttnum)len+=snprintf(bfr+len,sizeof(bfr)-len,
			"@%d",p->stream[i].pmttnum);
		j=1;
		break;
	}
	if(!j)len+=snprintf(bfr+len,sizeof(bfr)-len,"0");

	for(j=0,i=0;i<p->streams;i++)switch(p->stream[i].pmttype)
	{
	case SATIP_PMT_AC3:
	case SATIP_PMT_EAC3:
		if(!j)len+=snprintf(bfr+len,sizeof(bfr)-len,";");
		else len+=snprintf(bfr+len,sizeof(bfr)-len,",");
		len+=snprintf(bfr+len,sizeof(bfr)-len,"%d",p->stream[i].pid);
		if(p->stream[i].lang[0]||p->stream[i].pmttnum)
			len+=snprintf(bfr+len,sizeof(bfr)-len,"=");
		if(p->stream[i].lang[0])len+=snprintf(bfr+len,sizeof(bfr)-len,
			"%s",p->stream[i].lang);
		if(p->stream[i].pmttnum)len+=snprintf(bfr+len,sizeof(bfr)-len,
			"@%d",p->stream[i].pmttnum);
		j=1;
		break;
	}
	len+=snprintf(bfr+len,sizeof(bfr)-len,":");

	for(j=0,i=0;i<p->streams;i++)switch(p->stream[i].pmttype)
	{
	case SATIP_PMT_TTX:
		if(!j)len+=snprintf(bfr+len,sizeof(bfr)-len,"%d",
			p->stream[i].pid);
		j=1;
		break;
	}
	if(!j)len+=snprintf(bfr+len,sizeof(bfr)-len,"0");
	for(j=0,i=0;i<p->streams;i++)switch(p->stream[i].pmttype)
	{
	case SATIP_PMT_SUBS:
		if(!j)len+=snprintf(bfr+len,sizeof(bfr)-len,";");
		else len+=snprintf(bfr+len,sizeof(bfr)-len,",");
		if(p->stream[i].lang[0])len+=snprintf(bfr+len,sizeof(bfr)-len,
			"%d=%s",p->stream[i].pid,p->stream[i].lang);
		else len+=snprintf(bfr+len,sizeof(bfr)-len,"%d",
			p->stream[i].pid);
		j=1;
		break;
	}
	len+=snprintf(bfr+len,sizeof(bfr)-len,":");

	if(!p->catotal)len+=snprintf(bfr+len,sizeof(bfr)-len,"0");
	else for(i=0;i<p->catotal;i++)
	{
		if(i)len+=snprintf(bfr+len,sizeof(bfr)-len,",");
		len+=snprintf(bfr+len,sizeof(bfr)-len,"%x",p->caid[i]);
	}
	len+=snprintf(bfr+len,sizeof(bfr)-len,":");

	len+=snprintf(bfr+len,sizeof(bfr)-len,"%d:%d:%d:0",p->prognum,
		p->nid,p->tsid);

	fprintf(fp,"%s\n",bfr);
}

#ifndef NOCDK

static void okbutton(struct SButton *button)
{
	exitOKCDKScreenOf(&button->obj);
}

static int chpnum(EObjectType cdktype,void *obj,void *data,chtype key)
{
	SEL *sel=data;
	CDKENTRY *e;
	char *ptr;
	char *end;
	int i;
	int idx=getCDKSelectionCurrent(sel->s);
	unsigned long val;
	char tmp[32];

	if(sel->list[idx]->user[0])sprintf(tmp,"%d",sel->list[idx]->user[0]);
	else *tmp=0;

	if(!(e=newCDKEntry(sel->scr,CENTER,CENTER,"Enter Program Number:",NULL,
		A_NORMAL,'.',vMIXED,40,0,16,TRUE,FALSE)))return 0;

	refreshCDKScreen(sel->scr);
	setCDKEntry(e,tmp,0,16,TRUE);
	while(1)
	{
		ptr=activateCDKEntry(e,NULL);
		if(e->exitType==vNORMAL)
		{
			if(!*ptr)val=0;
			else
			{
				val=strtoul(ptr,&end,10);
				if(ptr==end||*end)continue;
				if(val<0||val>999999)continue;
				for(i=0;i<sel->size;i++)if(i!=idx)
					if(val==sel->list[i]->user[0])break;
				if(i!=sel->size)continue;
			}
			if(val==sel->list[idx]->user[0])break;
			sel->list[idx]->user[0]=(int)val;
			if(!val)strcpy(tmp,"       ");
			else sprintf(tmp,"%7d",(int)val);
			memcpy(sel->dlist[idx]+7,tmp,7);
			for(i=0;i<sel->size;i++)
				if(getCDKSelectionChoice(sel->s,i))
					sel->list[i]->user[1]|=1;
			else sel->list[i]->user[1]&=~1;
			setCDKSelectionItems(sel->s,sel->dlist,sel->size);
			setCDKSelectionCurrent(sel->s,idx);
			for(i=0;i<sel->size;i++)if(sel->list[i]->user[1]&1)
				setCDKSelectionChoice(sel->s,i,1);
		}
		break;
	}

	destroyCDKEntry(e);
	return 0;
}

static int rmidx(EObjectType cdktype,void *obj,void *data,chtype key)
{
	SEL *sel=data;
	int i;
	int idx=getCDKSelectionCurrent(sel->s);

	if(!sel->size)return 0;
	for(i=0;i<sel->size;i++)if(getCDKSelectionChoice(sel->s,i))
		sel->list[i]->user[1]|=1;
	for(i=idx+1;i<sel->size;i++)sel->dlist[i-1]=sel->dlist[i];
	satip_util_program_remove(&sel->list,&sel->size,idx);
	setCDKSelectionItems(sel->s,sel->dlist,sel->size);
	if(sel->size&&sel->size==idx)idx--;
	setCDKSelectionCurrent(sel->s,idx);
	for(i=0;i<sel->size;i++)if(sel->list[i]->user[1]&1)
		setCDKSelectionChoice(sel->s,i,1);
	return 0;
}

static void cdklist(char *charset,SATIP_UTIL_PROGRAM ***list,int *size)
{
	CDKSCREEN *cs;
	WINDOW *cw;
	CDKBUTTON *b;
	char *in;
	char *out;
	size_t ilen;
	size_t olen;
	int i;
	SEL sel;
	iconv_t ic;
	char tmp[64];
	char src[32];
	char type[8];
	char name[256];

	sel.list=*list;
	sel.size=*size;

	if(!(sel.dlist=malloc(256*sel.size+sizeof(char *)*sel.size)))goto err0;

	snprintf(tmp,sizeof(tmp),"%s//TRANSLIT",charset);

	if((ic=iconv_open(tmp,"UTF-8"))==(iconv_t)-1)goto err1;

	for(i=0;i<sel.size;i++)
	{
		in=sel.list[i]->progname;
		ilen=strlen(sel.list[i]->progname)+1;
		out=name;
		olen=sizeof(name);
		if(iconv(ic,&in,&ilen,&out,&olen)==(size_t)-1)
			snprintf(name,sizeof(name),"%s",sel.list[i]->progname);
		type[0]=sel.list[i]->tv?'V':' ';
		type[1]=sel.list[i]->radio?'R':' ';
		type[2]=sel.list[i]->fta||sel.list[i]->sdtfta?'F':' ';
		type[3]=sel.list[i]->eit?'E':' ';
		type[4]=0;
		if(!sel.list[i]->user[0])strcpy(tmp,"       ");
		else sprintf(tmp,"%7d",sel.list[i]->user[0]);
		if(sel.list[i]->tune.msys==SATIP_DVBT||
			sel.list[i]->tune.msys==SATIP_DVBT2)
				strcpy(src,"     T");
		else if(sel.list[i]->tune.msys==SATIP_DVBC||
			sel.list[i]->tune.msys==SATIP_DVBC2)
				strcpy(src,"     C");
		else if(sel.list[i]->tune.src<1)strcpy(src,"      ");
		else sprintf(src,"%6d",sel.list[i]->tune.src);
		sel.dlist[i]=((char *)(&sel.dlist[sel.size]))+256*i;
		snprintf(sel.dlist[i],256,"  %c    %s    %s    %s    %s",
			(sel.list[i]->user[1]&2)?'!':' ',tmp,src,type,name);
		iconv(ic,NULL,NULL,NULL,NULL);
	}

	if(!(cw=initscr()))goto err2;
	if(!(cs=initCDKScreen(cw)))goto err3;
	initCDKColor();

	if(!(sel.nw=newwin(0,0,0,0)))goto err4;
	if(!(sel.scr=initCDKScreen(sel.nw)))goto err5;
	wrefresh(sel.nw);

	if(!(sel.s=newCDKSelection(sel.scr,LEFT,TOP,RIGHT,-3,0,
		"Show  New    Program    Source    Type    Program Name",
		sel.dlist,sel.size,choices,2,A_NORMAL,1,0)))goto err6;

	for(i=0;i<sel.size;i++)if(sel.list[i]->user[1]&1)
		setCDKSelectionChoice(sel.s,i,1);

	if(!(b=newCDKButton(sel.scr,CENTER,BOTTOM,"  OK  ",okbutton,1,0)))
		goto err7;

	bindCDKObject(vSELECTION,sel.s,'\n',chpnum,&sel);
	bindCDKObject(vSELECTION,sel.s,'d',rmidx,&sel);
	bindCDKObject(vSELECTION,sel.s,'D',rmidx,&sel);

	refreshCDKScreen(sel.scr);
	traverseCDKScreen(sel.scr);

	for(i=0;i<sel.size;i++)if(getCDKSelectionChoice(sel.s,i))
		sel.list[i]->user[1]|=1;
	else sel.list[i]->user[1]&=~1;

	*list=sel.list;
	*size=sel.size;

	destroyCDKButton(b);
err7:	destroyCDKSelection(sel.s);
err6:	destroyCDKScreen(sel.scr);
err5:	eraseCursesWindow(sel.nw);
err4:	destroyCDKScreen(cs);
err3:	endCDK();
err2:	iconv_close(ic);
err1:	free(sel.dlist);
err0:	return;
}

#endif

static int readlist(char *fn,SATIP_UTIL_PROGRAM ***list,int *size)
{
	int fd;
	int l;
	int s;
	SATIP_UTIL_PROGRAM *e;
	SATIP_UTIL_PROGRAM *p;

	if((fd=open(fn,O_RDONLY|O_CLOEXEC))==-1)goto err1;

	satip_util_program_list_create(list,size);

	while(1)
	{
		if(!(l=read(fd,&s,sizeof(s))))break;
		else if(l!=sizeof(s))goto err2;
		if(!(e=malloc(s)))goto err2;
		if(read(fd,e,s)!=s)goto err3;
		e->progname=(char *)e+sizeof(*e)+
			e->streams*sizeof(e->stream[0]);
		e->provider=e->progname+strlen(e->progname)+1;

		if(satip_util_program_create(&p,e->streams,e->progname,
			e->provider))goto err3;

		for(l=0;l<4;l++)p->bits[l]=e->bits[l];
		for(l=0;l<SATIP_SDTCA_MAX;l++)p->caid[l]=e->caid[l];
		for(l=0;l<4;l++)p->user[l]=e->user[l];
		p->tune=e->tune;
		for(l=0;l<e->streams;l++)p->stream[l]=e->stream[l];

		if(satip_util_program_append(list,size,p))goto err4;
		free(e);
	}

	close(fd);
	return 0;

err4:	satip_util_program_free(p);
err3:	free(e);
err2:	satip_util_program_list_free(*list,*size);
err1:	return -1;
}

static int writelist(char *fn,SATIP_UTIL_PROGRAM **list,int size)
{
	int fd;
	int l;
	int r;
	struct iovec iov[4];

	if((fd=open(fn,O_WRONLY|O_TRUNC|O_CREAT|O_CLOEXEC,0666))==-1)goto err1;

	for(l=0;l<size;l++)
	{
		iov[0].iov_base=&r;
		iov[0].iov_len=sizeof(r);
		iov[1].iov_base=list[l];
		iov[1].iov_len=sizeof(list[0][0])+list[l]->streams*
			sizeof(list[0]->stream[0]);
		iov[2].iov_base=list[l]->progname;
		iov[2].iov_len=strlen(list[l]->progname)+1;
		iov[3].iov_base=list[l]->provider;
		iov[3].iov_len=strlen(list[l]->provider)+1;
		r=iov[1].iov_len+iov[2].iov_len+iov[3].iov_len;
		if(writev(fd,iov,4)!=r+iov[0].iov_len)goto err2;
	}

	close(fd);
	return 0;

err2:	close(fd);
err1:	return -1;
}

static int progsort(const void *d1,const void *d2)
{
	SATIP_UTIL_PROGRAM *p1=*((SATIP_UTIL_PROGRAM **)d1);
	SATIP_UTIL_PROGRAM *p2=*((SATIP_UTIL_PROGRAM **)d2);

	if(p1->user[0]<p2->user[0])return -1;
	if(p1->user[0]>p2->user[0])return 1;
	return 0;
}
static int deleter(SATIP_UTIL_PROGRAM *p1,SATIP_UTIL_PROGRAM *p2)
{
	if(p1->user[1]&2)return -1;
	if(p2->user[1]&2)return 1;
	return 0;
}

static int uninext(SATIP_UTIL_PROGRAM **p,int *user_prognum,void *priv)
{
	UNI *u=(UNI *)priv;

	if(u->idx>=u->size)return 0;
	*p=u->list[u->idx++];
	*user_prognum=(*p)->user[0];
	return 1;
}

static int multinext(SATIP_UTIL_PROGRAM **p,int *user_prognum,char *addr,
	int size,void *priv)
{
	MULTI *m=(MULTI *)priv;

	if(m->idx>=m->size)return 0;
	*p=m->list[m->idx++];
	if(m->first)m->first=0;
	else
	{
		if(satip_util_addrinc(m->addr,sizeof(m->addr)))return 0;
		if(satip_util_list_match_addr(m->mclist,m->addr))return 0;
	}
	if(strlen(m->addr)>=size)return 0;
	strcpy(addr,m->addr);
	*user_prognum=(*p)->user[0];
	return 1;
}

static void usage(void)
{
	fprintf(stderr,"Usage:\n"
"satipprog -b infile [-m mergefile] "HLP1"[-h host] [-p port] [-d] [-D] [-x]\n"
"          [-l locale] [-c charset] [-R rtspfile] [-H httpfile] [-M rtpfile]\n"
"          [-e] [-S setupfile] [-V vdrfile] [-1 satid] ... [-9 satid]\n"
"-b infile        binary input (and output) file\n"
"-m mergefile     binary data file to be merged into input file\n"
HLP2
"-h host          host name or IP or multicast ip\n"
"-p port          host port or multicast port\n"
"-d               add pmt pid to output (e.g. for Digital Devices)\n"
"-D               add 'x_pmt=<pmt>' to output (e.g. for Digital Devices)\n"
"-x               output x_sid for libsatip based servers (pid tracking)\n"
"-e               include EIT PID in M3U when EIT is available\n"
"-I               enforce strict SAT>IP query syntax compliance\n"
"-l locale        locale to be used, default de_DE\n"
"-c charset       charset to be used, default ISO-8859-15\n"
"-R rtspfile      RTSP unicast M3U output file\n"
"-H httpfile      HTTP unicast M3U output file\n"
"-M rtpfile       RTP multicast M3U output file\n"
"-S setupfile     static multicast setup URLs for libsatip based servers\n"
"-V vdrfile       vdr format output file\n"
"-1...-9 satid    SAT>IP source to vdr sources.conf identifier mapping\n"
);
	exit(1);
}

int main(int argc,char *argv[])
{
	int c;
	int val;
	int port=0;
	int flags=0;
	int interactive=0;
	int eit=0;
	int size=0;
	int msize=0;
	int rsize=0;
	char *infile=NULL;
	char *mergefile=NULL;
	char *vdrfile=NULL;
	char *rtspfile=NULL;
	char *httpfile=NULL;
	char *rtpfile=NULL;
	char *setupfile=NULL;
	char *host=NULL;
	char *locale=NULL;
	char *charset=NULL;
	FILE *fp;
	char *src;
	char *satid[9];
	SATIP_UTIL_PROGRAM **list=NULL;
	SATIP_UTIL_PROGRAM **mlist=NULL;
	SATIP_UTIL_PROGRAM **rlist=NULL;
	SATIP_UTIL_PROGRAM *e;
	MULTI m;
	UNI u;
	char bfr[1024];

	memset(satid,0,sizeof(satid));

	while((c=getopt(argc,argv,"b:m:1:2:3:4:5:6:7:8:9:H:R:H:dDh:p:M:S:xV:eI"
		CDKOPT))!=-1)switch(c)
	{
	case 'b':
		if(infile)usage();
		infile=optarg;
		break;

	case 'm':
		if(mergefile)usage();
		mergefile=optarg;
		break;

#ifndef NOCDK
	case 'i':
		if(interactive)usage();
		interactive=1;
		break;
#endif

	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		if(satid[c-'1'])usage();
		satid[c-'1']=optarg;
		break;

	case 'V':
		if(vdrfile)usage();
		vdrfile=optarg;
		break;

	case 'R':
		if(rtspfile)usage();
		rtspfile=optarg;
		break;

	case 'H':
		if(httpfile)usage();
		httpfile=optarg;
		break;

	case 'd':
		if(flags&SATIP_ADDPMT)usage();
		flags|=SATIP_ADDPMT;
		break;

	case 'D':
		if(flags&SATIP_ADDXPMT)usage();
		flags|=SATIP_ADDXPMT;
		break;

	case 'l':
		if(locale)usage();
		locale=optarg;
		break;

	case 'c':
		if(charset)usage();
		charset=optarg;
		break;

	case 'h':
		if(host)usage();
		host=optarg;
		break;

	case 'p':
		if(port)usage();
		if((port=atoi(optarg))<1||port>65535)usage();
		break;

	case 'M':
		if(rtpfile)usage();
		rtpfile=optarg;
		break;

	case 'S':
		if(setupfile)usage();
		setupfile=optarg;
		break;

	case 'x':
		if(flags&SATIP_ADDSID)usage();
		flags|=SATIP_ADDSID;
		break;

	case 'e':
		if(eit)usage();
		eit=1;
		break;

	case 'I':
		if(flags&SATIP_STRICTQRY)usage();
		flags|=SATIP_STRICTQRY;
		break;

	default:usage();
	}

	if(!locale)locale="de_DE";
	if(!charset)charset="ISO-8859-15";

	if((rtspfile||httpfile||rtpfile)&&(!host||!port))usage();
	if((rtspfile||httpfile)&&(rtpfile||setupfile))usage();
	if((rtpfile||setupfile)&&(port&1))usage();
	if(!infile)usage();

	if(readlist(infile,&list,&size))
	{
		fprintf(stderr,"can't read %s\n",infile);
		return 1;
	}

	if(mergefile)
	{
		if(readlist(mergefile,&mlist,&msize))
		{
			fprintf(stderr,"can't read %s\n",infile);
			satip_util_program_list_free(list,size);
			return 1;
		}

		for(c=0;c<msize;c++)mlist[c]->user[1]|=2;

		if(satip_util_program_merge(locale,list,size,mlist,msize,
			&rlist,&rsize,deleter))
		{
			fprintf(stderr,"satip_util_program_merge failed\n");
			satip_util_program_list_free(mlist,msize);
			satip_util_program_list_free(list,size);
			return 1;
		}

		list=rlist;
		size=rsize;
	}

#ifndef NOCDK
	if(interactive)cdklist(charset,&list,&size);
#endif

	for(c=0;c<size;c++)list[c]->user[1]&=~2;

	if(interactive||mergefile)if(writelist(infile,list,size))
	{
		fprintf(stderr,"can't write %s\n",infile);
		satip_util_program_list_free(list,size);
		return 1;
	}

	if(interactive)
	{
		satip_util_program_list_create(&rlist,&rsize);
		for(c=0;c<size;)if(!(list[c]->user[1]&1))c++;
		else if(!list[c]->user[0])c++;
		else
		{
			satip_util_program_extract(&list,&size,c,&e);
			satip_util_program_append(&rlist,&rsize,e);
		}
		qsort(rlist,rsize,sizeof(SATIP_UTIL_PROGRAM *),progsort);
		if(rsize)val=rlist[rsize-1]->user[0]+1;
		else val=1;
		for(c=0;c<size;)if(!(list[c]->user[1]&1))c++;
		else
		{
			satip_util_program_extract(&list,&size,c,&e);
			e->user[0]=val++;
			satip_util_program_append(&rlist,&rsize,e);
		}
		satip_util_program_list_free(list,size);
		list=rlist;
		size=rsize;
	}
	else for(val=1,c=0;c<size;c++)list[c]->user[0]=val++;

	if(!eit)for(c=0;c<size;c++)list[c]->m3u_eit_exclude=1;

	if(vdrfile)
	{
		if(!(fp=fopen(vdrfile,"we")))
		{
			fprintf(stderr,"can't access %s\n",vdrfile);
			satip_util_program_list_free(list,size);
			return 1;
		}

		for(c=0;c<size;c++)
		{
			if(list[c]->tune.msys==SATIP_DVBT||
				list[c]->tune.msys==SATIP_DVBT2)src="T";
			else if(list[c]->tune.msys==SATIP_DVBC||
				list[c]->tune.msys==SATIP_DVBC2)src="C";
			else if(list[c]->tune.src>=1&&list[c]->tune.src<=9)
				src=satid[list[c]->tune.src-1];
			if(!src)src="S360E";
			vdr(fp,list[c],src,charset);
		}

		fclose(fp);
	}

	if(rtspfile)
	{
		u.list=list;
		u.size=size;
		u.idx=0;

		if(satip_util_create_unicast_m3u(host,port,SATIP_TYPE_RTSP,
			flags,0,charset,uninext,&u,&src,&c))
		{
			fprintf(stderr,
				"satip_util_create_unicast_m3u failure\n");
			satip_util_program_list_free(list,size);
			return 1;
		}

		if(!(fp=fopen(rtspfile,"we")))
		{
			fprintf(stderr,"can't access %s\n",rtspfile);
			satip_util_program_list_free(list,size);
			free(src);
			return 1;
		}

		fwrite(src,c,1,fp);
		fclose(fp);
		free(src);
	}

	if(httpfile)
	{
		u.list=list;
		u.size=size;
		u.idx=0;

		if(satip_util_create_unicast_m3u(host,port,SATIP_TYPE_HTTP,
			flags,0,charset,uninext,&u,&src,&c))
		{
			fprintf(stderr,
				"satip_util_create_unicast_m3u failure\n");
			satip_util_program_list_free(list,size);
			return 1;
		}

		if(!(fp=fopen(httpfile,"we")))
		{
			fprintf(stderr,"can't access %s\n",httpfile);
			satip_util_program_list_free(list,size);
			free(src);
			return 1;
		}

		fwrite(src,c,1,fp);
		fclose(fp);
		free(src);
	}

	if(rtpfile||setupfile)
	{
		if(satip_util_list_init(&m.mclist))
		{
			fprintf(stderr,"satip_util_list_init failure\n");
			satip_util_program_list_free(list,size);
			return 1;
		}
		if(satip_util_list_add(m.mclist,"224.0.0.0/4")||
			satip_util_list_add(m.mclist,"ff02::/16")||
			satip_util_list_add(m.mclist,"ff05::/16"))
		{
			fprintf(stderr,"satip_util_list_add failure\n");
			satip_util_program_list_free(list,size);
			satip_util_list_free(m.mclist);
			return 1;
		}

		snprintf(m.addr,sizeof(m.addr),"%s",host);

		if(satip_util_list_match_addr(m.mclist,m.addr))
		{
			fprintf(stderr,"%s is not a multicast address\n",
				m.addr);
			satip_util_program_list_free(list,size);
			satip_util_list_free(m.mclist);
		}

		m.first=1;
	}

	if(rtpfile)
	{
		m.list=list;
		m.size=size;
		m.idx=0;

		if(satip_util_create_multicast_m3u(port,0,charset,multinext,
			&m,&src,&c))
		{
			fprintf(stderr,
				"satip_util_create_unicast_m3u failure\n");
			satip_util_program_list_free(list,size);
			satip_util_list_free(m.mclist);
			return 1;
		}

		if(!(fp=fopen(rtpfile,"we")))
		{
			fprintf(stderr,"can't access %s\n",rtpfile);
			satip_util_program_list_free(list,size);
			satip_util_list_free(m.mclist);
			free(src);
			return 1;
		}

		fwrite(src,c,1,fp);
		fclose(fp);
		free(src);
	}

	if(setupfile)
	{
		snprintf(m.addr,sizeof(m.addr),"%s",host);
		m.first=1;

		if(!(fp=fopen(setupfile,"we")))
		{
			fprintf(stderr,"can't access %s\n",setupfile);
			satip_util_program_list_free(list,size);
			satip_util_list_free(m.mclist);
			return 1;
		}

		for(c=0;c<size;c++)
		{
			if(m.first)m.first=0;
			else if(satip_util_addrinc(m.addr,sizeof(m.addr)))
			{
				fprintf(stderr,"satip_util_addrinc failure\n");
				satip_util_program_list_free(list,size);
				satip_util_list_free(m.mclist);
				fclose(fp);
				return 1;
			}
			else if(satip_util_list_match_addr(m.mclist,m.addr))
			{
				fprintf(stderr,"satip_util_list_match_addr "
					"failure\n");
				satip_util_program_list_free(list,size);
				satip_util_list_free(m.mclist);
				fclose(fp);
				return 1;
			}

			if(satip_util_create_rtp_setup(m.addr,port,flags,
				list[c],bfr,sizeof(bfr)))
			{
				fprintf(stderr,"satip_util_create_rtp_setup "
					"failure\n");
				satip_util_program_list_free(list,size);
				satip_util_list_free(m.mclist);
				fclose(fp);
				return 1;
			}

			fprintf(fp,"%s\n",bfr);
		}

		fclose(fp);
	}

	if(rtpfile||setupfile)satip_util_list_free(m.mclist);

	satip_util_program_list_free(list,size);

	return 0;
}
