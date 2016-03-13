#include <linux/dvb/dmx.h>
#include <sys/ioctl.h>
#include <sys/signalfd.h>
#include <pthread.h>
#include <signal.h>
#include <limits.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static int fd;
static int bufsize=65424;
static int mode=0;
static int min=-1;
static int max=-1;
static unsigned char *bfr;
static unsigned long long zero=0;
static unsigned long long pkts=0;
static unsigned long long pidpkts[8192];
static unsigned long long pidenc[8192];
static unsigned long long xrun[8192];
static unsigned char track[8192];

#ifndef __GNUC__
#define __attribute__(x)
#endif

static void *sink(void *unused)  __attribute__ ((hot));
static void usage(void) __attribute__ ((noreturn));

static void *sink(void *unused)
{
	int i;
	int l;
	int pid;
	int pf;
	int cc;
	struct pollfd p;
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_BLOCK,&set,NULL);

	memset(pidpkts,0,sizeof(pidpkts));
	memset(pidenc,0,sizeof(pidenc));
	memset(xrun,0,sizeof(xrun));
	memset(track,0xff,sizeof(track));

	p.fd=fd;
	p.events=POLLIN;

	while(1)
	{
		if(mode)switch(poll(&p,1,-1))
		{
		case -1:perror("poll");
			goto out;
		case 0:	continue;
		default:if(p.revents&POLLERR)
			{
				fprintf(stderr,"poll signalled error\n");
				goto out;
			}
			if(!(p.revents&POLLIN))continue;
		}

		switch((l=read(fd,bfr,bufsize)))
		{
		case -1:perror("read");
			goto out;
		case 0:	zero++;
			continue;
		}

		pthread_setcancelstate(PTHREAD_CANCEL_DISABLE,NULL);

		if(min==-1)min=max=l;
		else if(l<min)min=l;
		else if(l>max)max=l;

		for(i=0;i<l;i+=188)
		{
			pkts++;
			pid=bfr[i+1]&0x1f;
			pid<<=8;
			pid|=bfr[i+2];
			pf=(bfr[i+3]&0x10)?1:0;
			cc=bfr[i+3]&0x0f;
			pidpkts[pid]++;
			if(bfr[i+3]&0xc0)pidenc[pid]++;
			if(track[pid]==0xff)printf("%zd new pid %d\n",
				time(NULL),pid);
			else if(pid!=8191&&((track[pid]+pf)&0xf)!=cc)
			{
				xrun[pid]++;
				printf("%zd pid %d cc error (%x!=%x)\n",
					time(NULL),pid,(track[pid]+pf)&0xf,cc);
			}
			track[pid]=cc;
		}

		pthread_setcancelstate(PTHREAD_CANCEL_ENABLE,NULL);

		printf("\r %llu        \r",pkts);
		fflush(stdout);
	}

out:	pthread_exit(NULL);
}

static void usage(void)
{
	fprintf(stderr,"Usage:\n"
	"satipsink -d [-a num] [-b size] [-B size] [-n]\n"
	"satipsink -p pid[,...] [-a num] [-b size] [-B size] [-n]\n"
	"satipsink -h\n"
	"-d               read from dvr device instead of dmx device\n"
	"-p pid[,...]     comma separated pid list\n"
	"-a num           dvb adapter number (0-255, default 0)\n"
	"-b size          local read buffer size (>=188, default 65424)\n"
	"-B size          device buffer size (>=8192, default not set)\n"
	"-n               read in non-blocking mode\n"
	"-h               this help text\n");
	exit(1);
}

int main(int argc,char *argv[])
{
	unsigned short val;
	int c;
	int i;
	int sfd;
	int devbuf=0;
	int adapter=0;
	int dvr=0;
	int total=0;
	int pid[64];
	struct pollfd p;
	pthread_t th;
	sigset_t set;
	struct signalfd_siginfo info;
	time_t start;
	time_t end;
	struct dmx_pes_filter_params filter;
	char fn[PATH_MAX];
	pthread_attr_t attr;

	while((c=getopt(argc,argv,"b:B:a:dnp:h"))!=-1)switch(c)
	{
	case 'b':
		if(!(bufsize=((atoi(optarg)+187)/188)*188))usage();
		break;

	case 'B':
		if((devbuf=atoi(optarg))<8192)usage();
		break;

	case 'a':
		if((adapter=atoi(optarg))<0||adapter>255)usage();
		break;

	case 'd':
		dvr=1;
		break;

	case 'n':
		mode=1;
		break;

	case 'p':
		for(total=0,optarg=strtok(optarg,",");optarg;
			total++,optarg=strtok(NULL,","))
		{
			if(!*optarg||total>=64)usage();
			pid[total]=atoi(optarg);
			if(pid[total]<0||pid[total]>8191)usage();
		}
		if(!total)usage();
		break;

	case 'h':
	default:usage();
	}

	if(!(bfr=malloc(bufsize)))
	{
		perror("malloc");
		return 1;
	}

	sigfillset(&set);
	if((sfd=signalfd(-1,&set,SFD_CLOEXEC|SFD_NONBLOCK))==-1)
	{
		perror("signalfd");
		return 1;
	}
	sigprocmask(SIG_BLOCK,&set,NULL);
	p.fd=sfd;
	p.events=POLLIN;

	if(dvr)
	{
		sprintf(fn,"/dev/dvb/adapter%d/dvr0",adapter);
		if((fd=open(fn,O_RDONLY|O_CLOEXEC|(mode?O_NONBLOCK:0)))==-1)
		{
			perror("open");
			return 1;
		}

		if(devbuf)if(ioctl(fd,DMX_SET_BUFFER_SIZE,devbuf))
			perror("DMX_SET_BUFFER_SIZE");
	}
	else
	{
		if(!total)
		{
			fprintf(stderr,"no pids specified\n");
			return 1;
		}

		sprintf(fn,"/dev/dvb/adapter%d/demux0",adapter);

		if((fd=open(fn,O_RDONLY|O_CLOEXEC|(mode?O_NONBLOCK:0)))==-1)
		{
			perror("open");
			return 1;
		}

		if(devbuf)if(ioctl(fd,DMX_SET_BUFFER_SIZE,devbuf))
			perror("DMX_SET_BUFFER_SIZE");

		memset(&filter,0,sizeof(filter));
		filter.pid=pid[0];
		filter.input=DMX_IN_FRONTEND;
		filter.output=DMX_OUT_TSDEMUX_TAP;
		filter.pes_type=DMX_PES_OTHER;

		if(ioctl(fd,DMX_SET_PES_FILTER,&filter))
		{
			perror("DMX_SET_PES_FILTER");
			return -1;
		}

		for(i=1;i<total;i++)
		{
			val=pid[i];
			if(ioctl(fd,DMX_ADD_PID,&val))
			{
				perror("DMX_ADD_PID");
				return 1;
			}
		}

		if(ioctl(fd,DMX_START))
		{
			perror("DMX_START");
			return 1;
		}
	}

	if(pthread_attr_init(&attr))
	{
		perror("pthread_attr_init");
		return 1;
	}

	if(pthread_attr_setstacksize(&attr,524288))
	{
		perror("pthread_attr_setstacksize");
		return 1;
	}

	if(pthread_create(&th,NULL,sink,NULL))
	{
		perror("pthread_create");
		return 1;
	}

	pthread_attr_destroy(&attr);

	start=time(NULL);

	while(1)
	{
		if(poll(&p,1,-1)<=0||!(p.revents&POLLIN))continue;
		while(read(sfd,&info,sizeof(info))==sizeof(info))
			switch(info.ssi_signo)
		{
		case SIGINT:
		case SIGHUP:
		case SIGTERM:
			goto out;
		}
	}

out:	if(!dvr)
	{
		if(ioctl(fd,DMX_STOP))
		{
			perror("DMX_STOP");
			return 1;
		}

		for(i=0;i<total;i++)
		{
			val=pid[i];
			if(ioctl(fd,DMX_REMOVE_PID,&val))
			{
				perror("DMX_REMOVE_PID");
				return 1;
			}
		}
	}

	end=time(NULL);
	pthread_cancel(th);
	pthread_join(th,NULL);
	close(fd);

	printf("\n");
	if(min==-1)return 0;

	printf("reads: zero=%llu, min=%d, max=%d\n",zero,min,max);
	printf("total of %llu packets in %zd seconds\n",pkts,end-start);
	printf("\n");
	for(i=0;i<8192;i++)if(pidpkts[i])
		printf("pid %d: %llu/%llu (%llu)\n",i,pidpkts[i]-pidenc[i],
			pidenc[i],xrun[i]);

	return 0;
}
