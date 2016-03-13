/*
 * example SAT>IP client piping stream to stdout
 *
 * Copyright (c) 2016 Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 */

#include <pthread.h>
#include <sys/signalfd.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
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
#define pthread_spin_lock	pthread_mutex_lock
#define pthread_spin_unlock	pthread_mutex_unlock
#define pthread_spinlock_t	pthread_mutex_t
#else
#endif
#endif

typedef struct
{
	int showsig;
	int showtune;
	int showinfo;
	int ccnt;
	int head;
	int tail;
	int fill;
	int efd;
	unsigned long long marker;
	unsigned long long lost;
	pthread_spinlock_t mtx;
	char bfr[188*7*1000];
} CBDATA;

#ifndef PROFILE

static void cb(SATIP_DATA *data,void *priv) __attribute__ ((hot));
int main(int argc,char *argv[]) __attribute__ ((hot));
static void usage(void) __attribute__ ((cold)) __attribute__ ((noreturn));

#else

static void usage(void) __attribute__ ((noreturn));

#endif

static void cb(SATIP_DATA *data,void *priv)
{
	int len=data->intval&0xffff;
	int l;
	uint64_t dummy=1;
	char *ptr=data->ptrval;
	CBDATA *dta=(CBDATA *)priv;
	SATIP_STATUS *status;

	switch(data->intval&(SATIP_RTP|SATIP_RTCP))
	{
	case SATIP_RTP:
		if(UNLIKELY(data->intval&SATIP_LOSTPKT))dta->lost++;
		if(UNLIKELY(data->intval&SATIP_MARKER))dta->marker++;

	case 0:	if(UNLIKELY(!len))break;

		pthread_spin_lock(&dta->mtx);

		if(LIKELY(!dta->fill))
		{
			pthread_spin_unlock(&dta->mtx);
			if(LIKELY((l=write(1,ptr,len))==len))break;
			ptr+=l;
			len-=l;
			pthread_spin_lock(&dta->mtx);
		}

		if(UNLIKELY(len>sizeof(dta->bfr)-dta->fill))
		{
			if(len)dta->lost++;
			pthread_spin_unlock(&dta->mtx);
			break;
		}

		if(!dta->fill)dta->head=dta->tail=0;

		pthread_spin_unlock(&dta->mtx);

		if(UNLIKELY(dta->head+len>sizeof(dta->bfr)))
		{
			l=sizeof(dta->bfr)-dta->head;
			memcpy(dta->bfr+dta->head,ptr,l);
			memcpy(dta->bfr,ptr+l,len-l);
			dta->head=len-l;
		}
		else memcpy(dta->bfr+dta->head,ptr,len);

		if(UNLIKELY((dta->head+=len)==sizeof(dta->bfr)))dta->head=0;

		pthread_spin_lock(&dta->mtx);
		dta->fill+=len;
		pthread_spin_unlock(&dta->mtx);

		dummy=write(dta->efd,&dummy,sizeof(dummy));
		break;

	case SATIP_RTCP:
		if(LIKELY(++(dta->ccnt)<5))break;
		dta->ccnt=0;

		status=(SATIP_STATUS *)data->ptrval;

		if(dta->showsig)
			fprintf(stderr,"lock=%d level=%d quality=%d\n",
				status->lock,status->level,status->quality);

		if(dta->showtune)
		{
			fprintf(stderr,"freq=%llu fe=%d src=%d bw=%08x "
				"pol=%08x msys=%08x\n",status->tune.freq,
				status->tune.fe,status->tune.src,
				status->tune.bw,status->tune.pol,
				status->tune.msys);
			fprintf(stderr,"tmode=%08x mtype=%08x plts=%08x "
				"ro=%08x sr=%d\n",status->tune.tmode,
				status->tune.mtype,status->tune.plts,
				status->tune.ro,status->tune.sr);
			fprintf(stderr,"gi=%08x fec=%08x c2tft=%08x "
				"ds=%08x plp=%08x\n",status->tune.gi,
				status->tune.fec,status->tune.c2tft,
				status->tune.ds,status->tune.plp);
			fprintf(stderr,"t2id=%08x sm=%08x specinv=%08x\n",
				status->tune.t2id,status->tune.sm,
				status->tune.specinv);
		}

		if(dta->showinfo)
			fprintf(stderr,"marker=%llu lost=%llu\n",
				dta->marker,dta->lost);
		break;
	}
}

static void usage(void)
{
fprintf(stderr,
"Usage:\n"
"\n"
"satippipe [-m m3u-file] [-i interface] [-f fe] [-l level] [-n number]\n"
"          [-M ttl] [-s] [-t] [-S] [SAT>IP-URL]\n"
"\n"
"-m m3u-file    SAT>IP M3U file to be used, default /etc/satip.m3u\n"
"-i interface   network interface to be used, default eth0\n"
"-f fe          tuner to use, default any tuner\n"
"-l level       0=IPv4,1=0+IPv6 Link Layer,2=1+ IPv6 ULA addresses, default 0\n"
"-n number      program number to be streamed, default 1\n"
"-s             show signal info\n"
"-t             show tuning info\n"
"-S             show statistics info\n"
"-F             network receive without small delay if 1 (costs cpu)\n"
"-I             enforce strict SAT>IP query syntax compliance\n"
"-h             this help text\n"
);
	exit(1);
}

int main(int argc,char *argv[])
{
	int i;
	int r=1;
	int num=0;
	int type;
	int port;
	int stream;
	int feid=0;
	uint64_t dummy;
	void *h;
	void *h2=NULL;
	void *ch;
	char *p;
	FILE *fp;
	char *m3ufile=NULL;
	SATIP_TUNE tune;
	SATIP_PIDS set;
	SATIP_PIDS add;
	SATIP_PIDS del;
	SATIP_EXTRA xtra;
	SATIP_CLN_CONFIG conf;
	CBDATA *dta;
	sigset_t sset;
	struct pollfd pp[3];
	struct signalfd_siginfo sig;
	char ref[128];
	char bfr[8192];

	if(!(dta=malloc(sizeof(CBDATA))))goto err1;

	memset(dta,0,sizeof(CBDATA));
	memset(&conf,0,sizeof(conf));

	while((i=getopt(argc,argv,"m:i:l:n:stShf:IF"))!=-1)switch(i)
	{
	case 'm':
		if(m3ufile)usage();
		m3ufile=optarg;
		break;

	case 'i':
		if(*conf.dev||!*optarg)usage();
		strncpy(conf.dev,optarg,sizeof(conf.dev)-1);
		break;

	case 'l':
		if(conf.level)usage();
		if((conf.level=atoi(optarg))<0||conf.level>2)usage();
		conf.level++;
		break;

	case 'n':
		if((num=atoi(optarg))<1)usage();
		break;

	case 's':
		if(dta->showsig)usage();
		dta->showsig=1;
		break;

	case 't':
		if(dta->showtune)usage();
		dta->showtune=1;
		break;

	case 'S':
		if(dta->showinfo)usage();
		dta->showinfo=1;
		break;

	case 'f':
		if((feid=atoi(optarg))<1||feid>65535)usage();
		break;

	case 'I':
		if(conf.strict)usage();
		conf.strict=1;
		break;

	case 'F':
		if(conf.fast)usage();
		conf.fast=1;
		break;

	case 'h':
	default:usage();
	}

	if(!num)num=1;
	if(!m3ufile)m3ufile="/etc/satip.m3u";
	if(conf.level)conf.level--;
	if(!*conf.dev)strcpy(conf.dev,"eth0");
	conf.mttl=SATIP_MCST_TTL;
	conf.interval=SATIP_UPNP_OFF;

	if(optind<argc)
	{
		snprintf(bfr,sizeof(bfr),"%s",argv[optind]);
		p=bfr;
		goto work;
	}

	sprintf(ref,"EXTINF:0,%d. ",num);

	if(!(fp=fopen(m3ufile,"re")))
	{
		fprintf(stderr,"Can't open %s\n",m3ufile);
		goto err2;
	}

	i=0;
	while(fgets(bfr,sizeof(bfr),fp))
	{
		if(!(p=strtok(bfr,"\r\n")))continue;
		if(*p=='#')
		{
			i=0;
			p++;
			while(*p==' '||*p=='\t')p++;
			if(!strncmp(p,ref,strlen(ref)))i=1;
			continue;
		}
		while(*p==' '||*p=='\t')p++;
		if(!i||!*p)continue;
		fclose(fp);
		goto work;
	}

	fclose(fp);
	fprintf(stderr,"Cant' find program number %d\n",num);
	goto err2;

work:	satip_util_init_tune(&tune);
	satip_util_init_pids(&set);
	satip_util_init_pids(&add);
	satip_util_init_pids(&del);

	if(satip_util_parse(SATIP_PARSE_URL,0,SATIP_IGNCAPS,p,&type,ref,
			sizeof(ref),&port,&stream,&tune,&set,&add,&del,
			&xtra)||add.numpids||del.numpids||stream!=SATIP_UNDEF||
			(set.numpids<1&&type!=SATIP_RTP))
	{
		fprintf(stderr,"Bad URL\n");
		goto err2;
	}

	if(feid)tune.fe=feid;

	sigfillset(&sset);
	if((pp[0].fd=signalfd(-1,&sset,SFD_NONBLOCK|SFD_CLOEXEC))==-1)
	{
		perror("signalfd");
		goto err2;
	}
	sigprocmask(SIG_BLOCK,&sset,NULL);
	pp[0].events=POLLIN;

	if((pp[1].fd=dta->efd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)
	{
		perror("eventfd");
		goto err3;
	}
	pp[1].events=POLLIN;

	pp[2].fd=1;
	pp[2].events=POLLOUT;

	if(!(ch=satip_cln_init(&conf)))
	{
		fprintf(stderr,"satip_cln_init failure\n");
		goto err4;
	}

	if(pthread_spin_init(&dta->mtx,PTHREAD_PROCESS_PRIVATE))
	{
		perror("pthread_spin_init");
		goto err5;
	}

	switch(type)
	{
	case SATIP_TYPE_HTTP:
		if(satip_cln_stream_http(ch,ref,port,&tune,&set,&xtra,cb,
			dta,&h))
		{
			fprintf(stderr,"satip_cln_stream_http failure\n");
			goto err6;
		}
		if(dta->showsig||dta->showtune||dta->showinfo)
		{
			set.prognum=0;
			set.numpids=SATIP_SIGNAL;
			if(satip_cln_stream_http(ch,ref,port,&tune,&set,NULL,
				cb,&dta,&h2))
			{
				fprintf(stderr,
					"satip_cln_stream_http (2) failure\n");
				goto err7;
			}
		}
		break;

	case SATIP_TYPE_RTSP:
		if(satip_cln_stream_unicast(ch,ref,port,1,&tune,&set,&xtra,cb,
			dta,&h))
		{
			fprintf(stderr,"satip_cln_stream_unicast failure\n");
			goto err6;
		}
		break;

	case SATIP_TYPE_RTP:
		if(satip_cln_stream_multicast(ch,ref,port,cb,dta,&h))
		{
			fprintf(stderr,"satip_cln_stream_multicast  failure\n");
			goto err6;
		}
		break;

	default:fprintf(stderr,"unknown type error\n");
		goto err6;
	}

	ioctl(1,FIONBIO,&r);

	r=0;

	while(1)
	{
		i=2;
		pp[2].revents=0;

		pthread_spin_lock(&dta->mtx);
		if(dta->fill)i=3;
		pthread_spin_unlock(&dta->mtx);

		if(UNLIKELY(poll(pp,i,-1)<=0))continue;
		if(UNLIKELY(pp[0].revents&POLLIN))
		{
			if(read(pp[0].fd,&sig,sizeof(sig))!=sizeof(sig))
				continue;
			i=0;
			switch(sig.ssi_signo)
			{
			case SIGINT:
			case SIGHUP:
			case SIGTERM:
				i=1;
				break;
			}
			if(i)break;
		}

		if(pp[1].revents&POLLIN)
			dummy=read(pp[1].fd,&dummy,sizeof(dummy));

		if(UNLIKELY(pp[2].revents&(POLLHUP|POLLERR)))break;

		if(UNLIKELY(!(pp[2].revents&POLLOUT)))continue;

		pthread_spin_lock(&dta->mtx);
		while(1)
		{
			if(UNLIKELY(!dta->fill))
			{
				pthread_spin_unlock(&dta->mtx);
				break;
			}
			if(dta->tail+dta->fill>sizeof(dta->bfr))
				i=sizeof(dta->bfr)-dta->tail;
			else i=dta->fill;
			pthread_spin_unlock(&dta->mtx);
			if(UNLIKELY((i=write(1,dta->bfr+dta->tail,i))<=0))break;
			if(UNLIKELY((dta->tail+=i)==sizeof(dta->bfr)))
				dta->tail=0;
			pthread_spin_lock(&dta->mtx);
			dta->fill-=i;
		}
	}

	if(h2)satip_cln_stream_stop(ch,h2);
err7:	satip_cln_stream_stop(ch,h);
err6:	satip_cln_fini(ch);
err5:	pthread_spin_destroy(&dta->mtx);
err4:	close(dta->efd);
err3:	close(pp[0].fd);
err2:	free(dta);
err1:	return r;
}
