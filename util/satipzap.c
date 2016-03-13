/*
 * example SAT>IP tuning client for SAT>IP loop device
 *
 * Copyright (c) 2016 Andreas Steinmetz (ast@domdv.de)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, version 2.
 *
 */

#include <linux/dvb/frontend.h>
#include <linux/dvb/dmx.h>
#include <linux/dvb/ca.h>
#include <sys/signalfd.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <poll.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <stdio.h>
#include "satip.h"

typedef struct
{
	struct dtv_properties props;
	struct dtv_property prop[16];
	struct dvb_diseqc_master_cmd dcmd;
	int dburst;
	int dtone;
	int dsrc;
} HWTUNE;

typedef struct _tuner
{
	SATIP_TUNE tune;
	HWTUNE hwtune;
	int lnb;
	int lna;
	int diseqc;
} TUNER;

static int fe_dvbs2_tune(int fd,TUNER *tuner)
{
	int idx;
	int hl;
	int hv;
	int val;
	unsigned long long freq;
	struct dtv_properties props;
	struct dtv_property prop;

	if(tuner->tune.msys!=SATIP_DVBS&&tuner->tune.msys!=SATIP_DVBS2)
		return -1;

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

	if(freq%1000)return -1;

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

	tuner->hwtune.dburst=-1;

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
	tuner->hwtune.prop[5].u.data=INVERSION_AUTO;

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
		val=FEC_AUTO;
		break;
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

	tuner->hwtune.prop[tuner->hwtune.props.num].cmd=DTV_TUNE;
	tuner->hwtune.prop[tuner->hwtune.props.num].u.data=0;
	tuner->hwtune.props.num++;

	props.num=1;
	props.props=&prop;
	prop.cmd=DTV_TONE;
	prop.u.data=SEC_TONE_OFF;

	if(ioctl(fd,FE_SET_PROPERTY,&props))return -1;

	usleep(15000);

	switch(tuner->diseqc)
	{
	case SATIP_DSC_NONE:
		break;

	default:if(ioctl(fd,FE_DISEQC_SEND_MASTER_CMD,&tuner->hwtune.dcmd))
			return -1;
	usleep(15000);
	case SATIP_DSC_TONE:
		if(tuner->hwtune.dburst==-1)break;
		if(ioctl(fd,FE_DISEQC_SEND_BURST,tuner->hwtune.dburst))
			return -1;
	}

	usleep(15000);

	props.num=1;
	props.props=&prop;
	prop.cmd=DTV_TONE;
	prop.u.data=tuner->hwtune.dtone;

	if(ioctl(fd,FE_SET_PROPERTY,&props))return -1;

	if(ioctl(fd,FE_SET_PROPERTY,&tuner->hwtune.props))return -1;

	usleep(15000);

	return 0;
}

static int fe_dvbt2_tune(int fd,TUNER *tuner)
{
	unsigned long long freq;
	int val;

	if(tuner->tune.msys!=SATIP_DVBT&&tuner->tune.msys!=SATIP_DVBT2)
		return -1;

	freq=tuner->tune.freq;

	tuner->hwtune.props.props=tuner->hwtune.prop;

	tuner->hwtune.prop[0].cmd=DTV_CLEAR;
	tuner->hwtune.prop[0].u.data=0;
	tuner->hwtune.prop[1].cmd=DTV_FREQUENCY;
	tuner->hwtune.prop[1].u.data=freq;
	tuner->hwtune.prop[2].cmd=DTV_HIERARCHY;
	tuner->hwtune.prop[2].u.data=HIERARCHY_AUTO;
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
		val=0;
		break;
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
		val=FEC_AUTO;
		break;
	default:return -1;
	}
	tuner->hwtune.prop[6].cmd=DTV_CODE_RATE_HP;
	tuner->hwtune.prop[6].u.data=val;

	tuner->hwtune.prop[7].cmd=DTV_CODE_RATE_LP;
	tuner->hwtune.prop[7].u.data=FEC_AUTO;

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
		val=QAM_AUTO;
		break;
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
		val=TRANSMISSION_MODE_AUTO;
		break;
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
		val=GUARD_INTERVAL_AUTO;
		break;
	default:return -1;
	}
	tuner->hwtune.prop[10].cmd=DTV_GUARD_INTERVAL;
	tuner->hwtune.prop[10].u.data=val;

	if(tuner->tune.msys==SATIP_DVBT2)
	{
		tuner->hwtune.prop[11].cmd=DTV_STREAM_ID;
		tuner->hwtune.prop[11].u.data=(tuner->tune.plp!=SATIP_UNDEF?
			tuner->tune.plp:NO_STREAM_ID_FILTER);
		tuner->hwtune.props.num=12;
	}
	else tuner->hwtune.props.num=11;

	tuner->hwtune.prop[tuner->hwtune.props.num].cmd=DTV_TUNE;
	tuner->hwtune.prop[tuner->hwtune.props.num].u.data=0;
	tuner->hwtune.props.num++;

	if(ioctl(fd,FE_SET_PROPERTY,&tuner->hwtune.props))return -1;

	return 0;
}

static int fe_dvbc_tune(int fd,TUNER *tuner)
{
	unsigned long long freq;
	int val;

	if(tuner->tune.msys!=SATIP_DVBC)return -1;

	freq=tuner->tune.freq;

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
		val=QAM_AUTO;
		break;
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
		val=INVERSION_AUTO;
		break;
	default:return -1;
	}
	tuner->hwtune.prop[7].cmd=DTV_INVERSION;
	tuner->hwtune.prop[7].u.data=val;

	tuner->hwtune.props.num=8;

	tuner->hwtune.prop[tuner->hwtune.props.num].cmd=DTV_TUNE;
	tuner->hwtune.prop[tuner->hwtune.props.num].u.data=0;
	tuner->hwtune.props.num++;

	if(ioctl(fd,FE_SET_PROPERTY,&tuner->hwtune.props))return -1;

	return 0;
}

int main(int argc,char *argv[])
{
	int c;
	int x=0;
	int r=0;
	int adapter=0;
	int num=1;
	int type;
	int port;
	int fe;
	int dmx[SATIP_MAX_PIDS];
	char *m3u=NULL;
	char *p=NULL;
	FILE *fp;
	fe_status_t status;
	uint16_t snr;
	uint16_t signal;
	uint32_t ber;
	uint32_t unc;
	TUNER tuner;
	SATIP_PIDS set;
	SATIP_PIDS add;
	SATIP_PIDS del;
	sigset_t sset;
	struct pollfd pp;
	struct signalfd_siginfo sig;
	struct dmx_pes_filter_params filter;
	char ref[128];
	char bfr[8192];

	memset(&tuner,0,sizeof(TUNER));
	tuner.diseqc=2;

	while((c=getopt(argc,argv,"l:L:d:a:m:n:xrh"))!=-1)switch(c)
	{
	case 'l':
		tuner.lnb=atoi(optarg);
		break;
	case 'L':
		tuner.lna=atoi(optarg);
		break;
	case 'd':
		tuner.diseqc=atoi(optarg);
		break;
	case 'a':
		adapter=atoi(optarg);
		break;
	case 'm':
		m3u=optarg;
		break;
	case 'n':
		num=atoi(optarg);
		break;
	case 'x':
		x=1;
		break;
	case 'r':
		r=1;
		break;
	case 'h':
	default:fprintf(stderr,
"Usage: satipzap [-l lnbtype] [-L lna-mode] [-d diseqc] [-a adapter]\n"
"                [-x] [-r] [-h] [-m m3u-file] [-n program] [SAT>IP-URL]\n"
"-l lnbtype     lnb type in use, 0=universal (see satip.h header)\n"
"-L lna-mode    0=disable, 1=enable, only for DVB-T/C\n"
"-d diseqc      DisecqC version, 0=off, 2=1.0 (see satip.h header)\n"
"-a adapter     DVB device adapter number\n"
"-m m3u-file    M3U file for SAT>IP, default /etc/satip.m3u\n"
"-n program     program number within M3U file, default 1\n"
"-x             exit after tuning\n"
"-r             stream via /dev/dvb/adapterX/dvr0\n"
"-h             this help text\n");
		return 1;
	}


	if(!num)num=1;	
	if(!m3u)m3u="/etc/satip.m3u";

	if(num<1||adapter<0||tuner.lnb<0||tuner.lnb>4||tuner.diseqc<0||
		tuner.diseqc>9)
	{
		fprintf(stderr,"illegal parameter\n");
		return 1;
	}

	if(optind<argc)
	{
		snprintf(bfr,sizeof(bfr),"%s",argv[optind]);
		p=bfr;
		goto work;
	}

	sprintf(ref,"EXTINF:0,%d. ",num);

	if(!(fp=fopen(m3u,"re")))
	{
		fprintf(stderr,"Can't open %s\n",m3u);
		return 1;
	}

	c=0;
	while(fgets(bfr,sizeof(bfr),fp))
	{
		if(!(p=strtok(bfr,"\r\n")))continue;
		if(*p=='#')
		{
			c=0;
			p++;
			while(*p==' '||*p=='\t')p++;
			if(!strncmp(p,ref,strlen(ref)))c=1;
			continue;
		}
		while(*p==' '||*p=='\t')p++;
		if(!c||!*p)continue;
		fclose(fp);
		goto work;
	}

	fclose(fp);
	fprintf(stderr,"Cant' find program number %d\n",num);
	return 1;

work:	satip_util_init_tune(&tuner.tune);
	satip_util_init_pids(&set);
	satip_util_init_pids(&add);
	satip_util_init_pids(&del);

	if(satip_util_parse(SATIP_PARSE_URL,0,SATIP_IGNCAPS,p,&type,ref,
		sizeof(ref),&port,&c,&tuner.tune,&set,&add,&del,NULL)||
		add.numpids||del.numpids||set.numpids<1||c!=SATIP_UNDEF||
		type==SATIP_RTP)
	{
		fprintf(stderr,"Bad URL\n");
		return 1;
	}

	sigfillset(&sset);
	if((pp.fd=signalfd(-1,&sset,SFD_NONBLOCK|SFD_CLOEXEC))==-1)
	{       
		perror("signalfd");
		return 1;
	}
	sigprocmask(SIG_BLOCK,&sset,NULL);
	pp.events=POLLIN;

	sprintf(bfr,"/dev/dvb/adapter%d/frontend0",adapter);

	if((fe=open(bfr,O_RDWR|O_NONBLOCK|O_CLOEXEC))==-1)
	{
		perror(bfr);
		return 1;
	}

	sprintf(bfr,"/dev/dvb/adapter%d/demux0",adapter);

	switch(tuner.tune.msys)
	{
	case SATIP_DVBS:
	case SATIP_DVBS2:
		c=fe_dvbs2_tune(fe,&tuner);
		break;
	case SATIP_DVBT:
	case SATIP_DVBT2:
		c=fe_dvbt2_tune(fe,&tuner);
		break;
	case SATIP_DVBC:
	case SATIP_DVBC2:
		c=fe_dvbc_tune(fe,&tuner);
		break;
	default:c=-1;
		break;
	}

	if(c)
	{
		fprintf(stderr,"tuning failure\n");
		close(fe);
		return 1;
	}

	if(x)goto out;

	if(r)for(c=0;c<set.numpids;c++)
	{
		if((dmx[c]=open(bfr,O_RDWR|O_NONBLOCK|O_CLOEXEC))==-1)
		{
			perror(bfr);
			while(--c>=0)close(dmx[c]);
			close(fe);
			return 1;
		}

		memset(&filter,0,sizeof(filter));
		filter.pid=set.pids[c];
		filter.input=DMX_IN_FRONTEND;
		filter.output=DMX_OUT_TS_TAP;
		filter.pes_type=DMX_PES_OTHER;
		filter.flags=DMX_IMMEDIATE_START;

		if(ioctl(dmx[c],DMX_SET_PES_FILTER,&filter))
		{
			perror("DMX_SET_PES_FILTER");
			close(dmx[c]);
			while(--c>=0)close(dmx[c]);
			close(fe);
			return 1;
		}
	}

	while(1)
	{
		if(poll(&pp,1,500)==1)if(pp.revents&POLLIN)
		{
			if(read(pp.fd,&sig,sizeof(sig))!=sizeof(sig))
				continue;
			x=0;
			switch(sig.ssi_signo)
			{
			case SIGINT:
			case SIGHUP:
			case SIGTERM:
				x=1;
				break;
			}
			if(x)break;
		}

		if(ioctl(fe,FE_READ_STATUS,&status)==-1)
		{
			perror("FE_READ_STATUS");
			continue;
		}
		if(ioctl(fe,FE_READ_SIGNAL_STRENGTH,&signal)==-1)signal=-1;
		if(ioctl(fe,FE_READ_SNR,&snr)==-1)snr=-1;
		if(ioctl(fe,FE_READ_BER,&ber)==-1)ber=-1;
		if(ioctl(fe,FE_READ_UNCORRECTED_BLOCKS,&unc)==-1)unc=-1;

		printf("status %02x | signal %04x | snr %04x | ber %08x | "
			"unc %08x | %s\n",status,signal,snr,ber,unc,
			(status&FE_HAS_LOCK)?"FE_HAS_LOCK":"");
	}

	while(--c>=0)close(dmx[c]);
out:	close(fe);

	return 0;
}
