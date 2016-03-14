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
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/times.h>
#include <sys/timerfd.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <signal.h>
#include <limits.h>
#include <iconv.h>
#include <errno.h>
#include <poll.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include "satip.h"
#include "common.h"

#define MAX_EPOLL	32
#define TCPBUFF		32712
#define SECTIONBUFF	65537
#define MAXSPLIT	20
#define MAXUPNP		16

#define RTSP_PERSIST	0x00000001
#define RTSP_DEAD	0x00000002
#define RTSP_SEQ	0x00000004
#define RTSP_RTPONLY	0x00000008

typedef struct
{
	int total;
	char *hdr;
	char *name[MAXSPLIT];
	char *value[MAXSPLIT];
} MSGDATA;

typedef struct _upnpdevice
{
	struct _upnpdevice *next;
	ADDR addr;
	int level;
	int noanswer;
	int age;
	int use;
	int bid;
	int cid;
	char uuid[SATIP_UUID_LEN+1];
	char location[0];
} UPNPDEVICE;

typedef struct
{
	pthread_mutex_t stx;
	pthread_mutex_t etx;
	pthread_mutex_t utx;
	pthread_spinlock_t ttx;
	struct _conn *clist;
	struct _conn *hlist;
	int idleflush;
	int upnpticker;
	int upnpinterval;
	int upnpanswer;
	int portmin;
	int portmax;
	int rtpbuffer;
	int devidx;
	int loidx;
	int level;
	int mttl;
	int strict;
	int fast;
	int efd;
	int xfd;
	int tfd;
	int ifd;
	int dfd[3];
	int udp[3];
	int mdp[3];
	char cset[SATIP_CSET_LEN+1];
	struct pollfd ep[2];
	ADDR devaddr[3];
	UPNPDEVICE *nlist[3];
	pthread_attr_t attr;
	pthread_t eh;
	pthread_t th;
} INSTANCE;

typedef struct
{
	void (*func)(INSTANCE *inst,int level);
	int level;
} FUNC;

typedef struct
{
	struct mmsghdr vec[SATIP_MAX_BURST];
	struct iovec io[SATIP_MAX_BURST];
	SOCK a[SATIP_MAX_BURST];
	char cmsg[SATIP_MAX_BURST][256];
	union
	{
#pragma pack(push,1)
		unsigned char msg1[2048];
		unsigned short msg2[1024];
		unsigned int msg4[512];
#pragma pack(pop)
	} u[SATIP_MAX_BURST];
} QUEUE;

typedef struct _conn
{
#pragma pack(push,1)
	struct _conn *next;
	char epid[8];
#pragma pack(pop)
	INSTANCE *instance;
	void (*cb)(SATIP_DATA *data,void *priv);
	void *priv;
	unsigned short port;
	ADDR addr;
	union
	{
		struct
		{
			union
			{
				struct
				{
					unsigned short rtp;
					unsigned short rtcp;
					unsigned short srtp;
					unsigned short srtcp;
				};
				struct
				{
					ADDR maddr;
					unsigned short mport;
					unsigned short mttl;
				};
			};
			unsigned short seq;
			ADDR sadr;
			int rtpfd;
			int rtcpfd;
			int rtspfd;
			int tmrfd;
			int rtfail;
			int rcfail;
			int flags;
			int tmo;
			int idle;
			int tmrcnt;
			int netwake;
			int sid;
			unsigned int ssrc;
			unsigned int cseq;
			int runner;
			pthread_mutex_t mtx;
			pthread_t th;
			SATIP_TUNE tune;
			char sess[128];
			QUEUE queue[0];
		};
		struct
		{
			int httpfd;
			int htfail;
			int fill;
			unsigned char bfr[0];
		};
	};
} CONN;

typedef struct
{
	int mode;
	ADDR sadr;
	ADDR addr;
	int ttl;
	unsigned int ssrc;
	unsigned short srtp;
	unsigned short srtcp;
	unsigned short crtp;
	unsigned short crtcp;
	unsigned short mrtp;
	unsigned short mrtcp;
} TRANSPORT;

#ifndef PROFILE

static void *streamer(void *data) __attribute__ ((hot));
static void rtp(INSTANCE *inst,CONN *c) __attribute__ ((hot));
static void http(INSTANCE *inst,CONN *c) __attribute__ ((hot));

void *satip_cln_init(SATIP_CLN_CONFIG *config) __attribute__ ((cold));
void satip_cln_fini(void *data) __attribute__ ((cold));

#endif

static int resolve(char *server,ADDR *addr,int level)
{
	int curr=SATIP_V4_ONLY;
	struct addrinfo *res;
	struct addrinfo *ap;
	struct addrinfo ai;

	memset(&ai,0,sizeof(ai));
	ai.ai_family=AF_UNSPEC;

	if(getaddrinfo(server,NULL,&ai,&res))return -1;
	addr->family=0;
	for(ap=res;ap;ap=ap->ai_next)switch(ap->ai_addr->sa_family)
	{
	case AF_INET:
		if(addr->family)break;
		addr->family=AF_INET;
		addr->a4.s_addr=
			((struct sockaddr_in *)(ap->ai_addr))->sin_addr.s_addr;
		break;
	case AF_INET6:
		if(level==SATIP_V4_ONLY)break;
		if(curr==SATIP_V6_SITE)break;
		if(a6linklocal(&((struct sockaddr_in6 *)(ap->ai_addr))
			->sin6_addr))
		{
			if(curr==SATIP_V6_LINK)break;
			curr=SATIP_V6_LINK;
			addr->family=AF_INET6;
			memcpy(addr->a6.s6_addr,((struct sockaddr_in6 *)
				(ap->ai_addr))->sin6_addr.s6_addr,16);
			break;
		}
		if(level!=SATIP_V6_SITE)break;
		if(!a6sitewide(&((struct sockaddr_in6 *)(ap->ai_addr))
			->sin6_addr))break;
		curr=SATIP_V6_SITE;
		addr->family=AF_INET6;
		memcpy(addr->a6.s6_addr,((struct sockaddr_in6 *)
			(ap->ai_addr))->sin6_addr.s6_addr,16);
		break;
	}
	if(res)freeaddrinfo(res);
	return addr->family?0:-1;
}

static int addr4if(int devidx,int level,ADDR *addr)
{
	int fd;
	int len;
	int rtl;
	int site=0;
	struct nlmsghdr *nlh;
	struct ifaddrmsg *ifa;
	struct rtattr *rth;
	struct in_addr *v4;
	struct in6_addr *v6;
	struct
	{
		struct nlmsghdr n;
		struct ifaddrmsg r;
		char data[16];
	} req;
	struct rtattr *rta;
	char *bfr;

	addr->family=0;

	if(!(bfr=malloc(65536)))goto err1;

	if((fd=socket(PF_NETLINK,SOCK_RAW|SOCK_CLOEXEC,NETLINK_ROUTE))==-1)
		goto err2;

	memset(&req,0,sizeof(req));
	req.n.nlmsg_len=NLMSG_LENGTH(sizeof(struct ifaddrmsg));
	req.n.nlmsg_flags=NLM_F_REQUEST|NLM_F_ROOT;
	req.n.nlmsg_type=RTM_GETADDR;
	req.n.nlmsg_seq=0xdeadbeef;
	req.r.ifa_family=(level==SATIP_V4_ONLY)?AF_INET:AF_INET6;
	rta=(struct rtattr *)(((char *)&req)+NLMSG_ALIGN(req.n.nlmsg_len));
	rta->rta_len=RTA_LENGTH(4);

	if(send(fd,&req,req.n.nlmsg_len,0)<0)goto err3;

	if((len=recv(fd,bfr,65536,0))<=0)goto err3;
	close(fd);

	nlh=(struct nlmsghdr *)bfr;

	if(nlh->nlmsg_seq!=0xdeadbeef)goto err2;

	for(;NLMSG_OK(nlh,len);nlh=NLMSG_NEXT(nlh,len))
		if(nlh->nlmsg_type==RTM_NEWADDR)
	{
		ifa=(struct ifaddrmsg *)NLMSG_DATA(nlh);
		rth=IFA_RTA(ifa);
		rtl=IFA_PAYLOAD(nlh);
		for(;rtl&&RTA_OK(rth,rtl);rth=RTA_NEXT(rth,rtl))
			switch(rth->rta_type)
		{
		case IFA_LOCAL:
			if(ifa->ifa_family!=AF_INET)break;
			if(ifa->ifa_flags&IFA_F_SECONDARY)break;
			if(ifa->ifa_index!=devidx)break;
			if(level!=SATIP_V4_ONLY)break;
			v4=(struct in_addr *)RTA_DATA(rth);
			addr->family=AF_INET;
			addr->a4.s_addr=v4->s_addr;
			break;

		case IFA_ADDRESS:
			if(ifa->ifa_family!=AF_INET6)break;
			if(ifa->ifa_index!=devidx)break;
			if(level==SATIP_V4_ONLY)break;
			v6=(struct in6_addr *)RTA_DATA(rth);
			if(a6linklocal(v6)&&level==SATIP_V6_LINK)
			{
				addr->family=AF_INET6;
				memcpy(addr->a6.s6_addr,v6->s6_addr,16);
				break;
			}
			else if(level!=SATIP_V6_SITE)break;
			switch(a6sitewide(v6))
			{
			case 1:	if(!site||site==2)
				{
					addr->family=AF_INET6;
					memcpy(addr->a6.s6_addr,v6->s6_addr,16);
					site=1;
					break;
				}
				goto a6sel;
			case 2:	if(!site)
				{
					addr->family=AF_INET6;
					memcpy(addr->a6.s6_addr,v6->s6_addr,16);
					site=2;
					break;
				}
				if(site==1)break;
a6sel:				if(memcmp(v6->s6_addr,addr->a6.s6_addr,16)<0)
					memcpy(addr->a6.s6_addr,v6->s6_addr,16);
				break;
			}
			break;
		}
	}

	free(bfr);
	return 0;

err3:	close(fd);
err2:	free(bfr);
err1:	return -1;
}

static int udp(ADDR *addr,short port,int devidx,int bufsize)
{
	int s;
	int v=1;
	SOCK saddr;

	if((s=socket(addr->family,SOCK_DGRAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0))<0)
		goto err1;

	if(bufsize)if(setsockopt(s,SOL_SOCKET,SO_RCVBUF,&bufsize,sizeof(int)))
		goto err2;

	addr2sock(&saddr,addr,port,devidx);

	if(addr->family==AF_INET6)
	{
		if(setsockopt(s,IPPROTO_IPV6,IPV6_V6ONLY,&v,sizeof(v)))
			goto err2;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_RECVPKTINFO,&v,sizeof(v)))
			goto err2;
	}
	else if(setsockopt(s,IPPROTO_IP,IP_PKTINFO,&v,sizeof(v)))goto err2;

	if((bind(s,(struct sockaddr *)&saddr,sizeof(SOCK)))<0)goto err2;

	return s;

err2:	close(s);
err1:	return -1;
}

static int tcp(ADDR *addr,short port,int devidx,int bufsize)
{
	int s;
	int v;
	int x;
	SOCK saddr;
	struct pollfd p;

	if((s=socket(addr->family,SOCK_STREAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0))<0)
		return -1;

	if(bufsize)if(setsockopt(s,SOL_SOCKET,SO_RCVBUF,&bufsize,sizeof(int)))
	{
		close(s);
		return -1;
	}

	addr2sock(&saddr,addr,port,devidx);

	if((connect(s,(struct sockaddr *)&saddr,sizeof(SOCK)))<0)
	{
		if(errno==EINPROGRESS)
		{
			p.fd=s;
			p.events=POLLOUT;
			if(poll(&p,1,1000)==1)
			{
				v=sizeof(x);
				if(!getsockopt(s,SOL_SOCKET,SO_ERROR,&x,&v))
					if(!x)goto conn;
			}
		}

		close(s);
		return -1;
	}

conn:	v=1;
	if(setsockopt(s,IPPROTO_TCP,TCP_NODELAY,&v,sizeof(v)))
	{
		close(s);
		return -1;
	}

	return s;
}

static int mcast(INSTANCE *inst,int ttl,ADDR *addr,short port,int which,
	int bufsize)
{
	int s;
	int v=1;
	SOCK saddr;
	struct ip_mreqn m4;
	struct ipv6_mreq m6;

	if(addr&&!port)goto err1;

	if((s=socket(addr?addr->family:mcaddr[which].family,
		SOCK_DGRAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0))<0)goto err1;

	if(bufsize)if(setsockopt(s,SOL_SOCKET,SO_RCVBUF,&bufsize,sizeof(int)))
		goto err2;

	if(addr)
	{
		if(linklocal(addr))ttl=1;
	}
	else if(which==SATIP_V6_LINK)ttl=1;

	if(port)
	{
		addr2sock(&saddr,addr?addr:&mcaddr[which],htons(port),
			inst->devidx);
		if(setsockopt(s,SOL_SOCKET,SO_REUSEADDR,&v,sizeof(v)))goto err2;
	}
	else addr2sock(&saddr,&inst->devaddr[which],0,inst->devidx);

	if((addr?addr->family:mcaddr[which].family)==AF_INET6)
	{
		if(setsockopt(s,IPPROTO_IPV6,IPV6_V6ONLY,&v,sizeof(v)))
			goto err2;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_RECVPKTINFO,&v,sizeof(v)))
			goto err2;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_UNICAST_HOPS,&ttl,
			sizeof(ttl)))goto err2;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_MULTICAST_HOPS,&ttl,
			sizeof(ttl)))goto err2;
	}
	else
	{
		if(setsockopt(s,IPPROTO_IP,IP_PKTINFO,&v,sizeof(v)))goto err2;
		if(setsockopt(s,IPPROTO_IP,IP_TTL,&ttl,sizeof(ttl)))goto err2;
		if(setsockopt(s,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)))
			goto err2;
	}

	if((bind(s,(struct sockaddr *)&saddr,sizeof(saddr)))<0)goto err2;

	if(port)switch(addr?addr->family:mcaddr[which].family)
	{
	case AF_INET:
		memset(&m4,0,sizeof(m4));
		m4.imr_multiaddr.s_addr=
			addr?addr->a4.s_addr:mcaddr[which].a4.s_addr;
		m4.imr_ifindex=inst->devidx;

		if(setsockopt(s,IPPROTO_IP,IP_ADD_MEMBERSHIP,
			(const void *)&m4,sizeof(m4)))goto err2;
		break;

	case AF_INET6:
		memset(&m6,0,sizeof(m6));
		memcpy(m6.ipv6mr_multiaddr.s6_addr,
			addr?addr->a6.s6_addr:mcaddr[which].a6.s6_addr,16);
		m6.ipv6mr_interface=inst->devidx;

		if(setsockopt(s,IPPROTO_IPV6,IPV6_ADD_MEMBERSHIP,
			(const void *)&m6,sizeof(m6)))goto err2;

		if(which!=SATIP_V6_SITE)break;

		memset(&m6,0,sizeof(m6));
		memcpy(m6.ipv6mr_multiaddr.s6_addr,addr?addr->a6.s6_addr:
			mcaddr[SATIP_V6_LINK].a6.s6_addr,16);
		m6.ipv6mr_interface=inst->devidx;

		if(setsockopt(s,IPPROTO_IPV6,IPV6_ADD_MEMBERSHIP,
			(const void *)&m6,sizeof(m6)))goto err2;
		break;

	default:goto err2;
	}
	else switch(mcaddr[which].family)
	{
	case AF_INET:
		memset(&m4,0,sizeof(m4));
		m4.imr_ifindex=inst->devidx;
		if(setsockopt(s,IPPROTO_IP,IP_MULTICAST_LOOP,&v,sizeof(v)))
			goto err2;
		if(setsockopt(s,IPPROTO_IP,IP_MULTICAST_IF,&m4,sizeof(m4)))
			goto err2;
		break;

	case AF_INET6:
		if(setsockopt(s,IPPROTO_IPV6,IPV6_MULTICAST_LOOP,&v,sizeof(v)))
			goto err2;
		v=inst->devidx;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_MULTICAST_IF,&v,sizeof(v)))
			goto err2;
		break;

	default:goto err2;
	}

	return s;

err2:	close(s);
err1:	return -1;
}

static int str2utf8(char *src,char *cset,char *dst,int size)
{
	int len;
	size_t ilen;
	size_t olen;
	char *in;
	char *out;
	iconv_t ic;
	char bfr[8192];

	ilen=len=strlen(src);

	if(!strcasecmp(cset,"UTF-8")||!strcasecmp(cset,"UTF8"))goto cpy;

	if((ic=iconv_open("UTF-8//TRANSLIT",cset))==(iconv_t)(-1))
	{
cpy:		if(dst)
		{
			if(size<=len)return -1;
			strcpy(dst,src);
		}
		return len;
	}

	if(dst)
	{
		if(size<=0)return -1;
		in=src;
		out=dst;
		olen=size-1;
		if(iconv(ic,&in,&ilen,&out,&olen)==-1)
		{
			iconv_close(ic);
			if(size<=len)return -1;
			strcpy(dst,src);
			return len;
		}
		iconv_close(ic);
		len=size-1-olen;
		dst[len]=0;
		return len;
	}

	for(len=0,in=src;ilen;)
	{
		out=bfr;
		olen=sizeof(bfr);
		if(iconv(ic,&in,&ilen,&out,&olen)==-1)
			if(errno!=E2BIG)
		{
			len=strlen(src);
			break;
		}
		len+=sizeof(bfr)-olen;
	}
	iconv_close(ic);
	return len;
}

static char *parsehttp(INSTANCE *inst,char *url,char *hp,ADDR *addr,int *port)
{
	int pos;
	long val;
	char *ptr;
	char *host;
	char *hport="80";
	ADDR haddr;
	char bfr[128];

	strncpy(bfr,url,127);
	bfr[127]=0;

	if(strncmp(bfr,"http://",7))return NULL;

	ptr=bfr+7;
	if(*ptr=='[')
	{
		for(host=ptr+1;*ptr&&*ptr!='/'&&*ptr!=']';ptr++);
		if(*ptr!=']')return NULL;
		*ptr++=0;
		if(*ptr!='/'&&*ptr!=':')return NULL;
	}
	else for(host=ptr;*ptr&&*ptr!=':'&&*ptr!='/';ptr++);
	if(*ptr==':')
	{
		*ptr++=0;
		hport=ptr;
	}
	for(;*ptr&&*ptr!='/';ptr++);
	if(*ptr)*ptr=0;
	pos=ptr-bfr;

	val=strtol(hport,&ptr,10);
	if(hport==ptr||*ptr||val<1||val>65535)return NULL;

	if(resolve(host,&haddr,inst->level))return NULL;

	*addr=haddr;
	*port=(int)val;
	memcpy(hp,url+7,pos-7);
	hp[pos-7]=0;

	return url+pos;
}

static char *fetchurl(INSTANCE *inst,char *url,int *size,char *encoding,
	int esize)
{
	int i;
	int len;
	int port;
	int total;
	long val;
	char *ptr;
	char *mem;
	ADDR addr;
	struct pollfd p;
	struct iovec io[5];
	char cset[SATIP_CSET_LEN+1];
	char bfr[1024];

	if(!(io[1].iov_base=parsehttp(inst,url,bfr,&addr,&port))||
		!((char *)io[1].iov_base)[0])goto err1;

	io[0].iov_base="GET ";
	io[0].iov_len=4;
	io[1].iov_len=strlen(io[1].iov_base);
	io[2].iov_base=" HTTP/1.1\r\nHost: ";
	io[2].iov_len=17;
	io[3].iov_base=bfr;
	io[3].iov_len=strlen(bfr);
	io[4].iov_base="\r\nConnection: close\r\nAccept: */*\r\n\r\n";
	io[4].iov_len=36;

	len=io[1].iov_len+io[3].iov_len+57;

	if((p.fd=tcp(&addr,htons(port),inst->devidx,0))==-1)goto err1;

	if(writev(p.fd,io,5)!=len)goto err2;

	for(len=0,p.events=POLLIN;;)
	{
		switch(poll(&p,1,1000))
		{
		case -1:if(errno==EINTR)continue;
		case 0:	goto err2;
		default:if(!(p.revents&POLLIN))goto err2;
			if(len==sizeof(bfr))goto err2;
			if((val=read(p.fd,bfr+len,sizeof(bfr)-len))<=0)
				goto err2;
			len+=val;
			break;
		}

		for(val=0,i=0;i<len;i++)switch(bfr[i])
		{
		case '\n':
			if(++val==2)goto cont1;
		case '\r':
			break;
		default:val=0;
			break;
		}
	}

cont1:	bfr[i++]=0;
	cset[0]=0;
	ptr=strtok_r(bfr,"\r\n",&mem);
	if(!ptr||strcmp(ptr,"HTTP/1.1 200 OK"))goto err2;
	for(val=-1,ptr=strtok_r(NULL,"\r\n",&mem);ptr;
		ptr=strtok_r(NULL,"\r\n",&mem))
			if(!strncasecmp(ptr,"Content-Length:",15))
	{
		for(ptr+=15;*ptr==' '||*ptr=='\t';ptr++);
		val=strtol(ptr,&mem,10);
		if(ptr==mem||*mem||val<=0)goto err2;
		break;
	}
	else if(!strncasecmp(ptr,"Content-Type:",13))
	{
		for(ptr+=13;*ptr==' '||*ptr=='\t';ptr++);
		if(!strncmp(ptr,"text/",5)||!strncmp(ptr,"audio/mpegurl",13)||
			!strncmp(ptr,"audio/x-mpegurl",15))
				strcpy(cset,inst->cset);
		if((ptr=strstr(ptr+13,"charset=")))
		{
			ptr+=8;
			for(val=0;ptr[val];val++)
			    if((ptr[val]>='a'&&ptr[val]<='z')||
				(ptr[val]>='A'&&ptr[val]<='Z')||ptr[val]=='-'||
				(ptr[val]>='0'&&ptr[val]<='9')||ptr[val]=='_'||
				ptr[val]==':'||ptr[val]=='.')continue;
			else break;
			ptr[val]=0;
			strncpy(cset,ptr,sizeof(cset)-1);
			cset[sizeof(cset)-1]=0;
			for(ptr=cset;ptr;ptr++)if(*ptr>='a'&&*ptr<='z')
				*ptr&=0xdf;
		}
	}
	if(val!=-1&&len-i>val)goto err2;

	total=8192;
	if(!(ptr=malloc(8192)))goto err2;
	if(i<len)memcpy(ptr,bfr+i,len-i);
	len-=i;

	while(len<=1024*1024)
	{
		if(val!=-1)if(len==val)goto cont2;

		if(len==total)
		{
			total+=8192;
			if(!(mem=realloc(ptr,total)))goto err3;
			ptr=mem;
		}

		i=total-len;
		if(val!=-1)if(i>val-len)i=val-len;

		switch(poll(&p,1,1000))
		{
		case -1:if(errno==EINTR)continue;
		case 0:	goto err3;
		default:if(!(p.revents&POLLIN))goto err3;
			if((i=read(p.fd,ptr+len,i))<0)goto err3;
			if(!i)
			{
				if(val!=-1)goto err3;
				goto cont2;
			}
			len+=i;
			break;
		}

	}

cont2:	close(p.fd);

	if(len==total)
	{
		if(!(mem=realloc(ptr,total+1)))goto err3;
		ptr=mem;
	}
	ptr[len]=0;

	*size=len;

	if(encoding)
	{
		strncpy(encoding,cset,esize-1);
		encoding[esize-1]=0;
	}

	return ptr;

err3:	free(ptr);
err2:	close(p.fd);
err1:	return NULL;
}

static int msgsplit(char *data,MSGDATA *msg)
{
	char *mem;
	char *ptr;
	int cnt;

	memset(msg,0,sizeof(MSGDATA));

	for(cnt=0,ptr=strtok_r(data,"\r\n",&mem);ptr&&*ptr&&cnt<=MAXSPLIT;
		ptr=strtok_r(NULL,"\r\n",&mem),cnt++)
	{
		if(!cnt)
		{
			msg->hdr=ptr;
			continue;
		}

		for(data=ptr;*data&&*data!=':';data++);
		if(!*data)goto err;
		*data++=0;
		for(;*data==' '||*data=='\t';data++);

		msg->name[cnt-1]=ptr;
		msg->value[cnt-1]=data;

		for(ptr=msg->name[cnt-1];*ptr;ptr++)if(*ptr>='A'&&*ptr<='Z')
			*ptr|=0x20;
	}

	if(!cnt)goto err;

	msg->total=cnt-1;
	return 0;

err:	return -1;
}

static int rtsp_reply(CONN *c,SATIP_DATA *data,TRANSPORT *t)
{
	int r;
	int n=0;
	int i=0;
	int len=0;
	int result;
	long l;
	char *ptr;
	char *line;
	char *mem;
	char *end;
	char *body=NULL;
	struct pollfd p;
	char *hdr[10];
	char bfr[8192];
	char addr[INET6_ADDRSTRLEN+2];

	p.fd=c->rtspfd;
	p.events=POLLIN;

	if(t)memset(t,0,sizeof(TRANSPORT));

	while(1)
	{
		if(len==sizeof(bfr))goto err;

		switch(poll(&p,1,1000))
		{
		case -1:if(errno==EINTR)continue;
		case 0:	goto err;
		default:if(!(p.revents&POLLIN))goto err;
		}

		if((r=read(c->rtspfd,bfr+len,sizeof(bfr)-len))<=0)goto err;
		len+=r;

		for(;i<len;i++)switch(bfr[i])
		{
		default:if(bfr[i]<0x20)goto err;
		case '\t':
			n=0;
		case '\r':
			break;
		case '\n':
			if(++n==2)goto hdr;
			break;
		}
	}

hdr:	bfr[i++]=0;

	if(!(line=strtok_r(bfr,"\r\n",&mem))||!*line)goto err;
	if(!(ptr=strtok_r(line," \t",&line))||strcmp(ptr,"RTSP/1.0"))goto err;
	if(!(ptr=strtok_r(NULL," \t",&line)))goto err;
	l=strtol(ptr,&end,10);
	if(l<0||l>INT_MAX||ptr==end||*end)goto err;
	result=l;

	memset(hdr,0,sizeof(hdr));
	for(line=strtok_r(NULL,"\r\n",&mem);line;
		line=strtok_r(NULL,"\r\n",&mem))if(*line)
	{
		if(!(ptr=strtok_r(line,": \t",&end)))goto err;

		if(!strcasecmp(ptr,"Connection"))n=0;
		else if(!strcasecmp(ptr,"CSeq"))n=1;
		else if(!strcasecmp(ptr,"Session"))n=2;
		else if(!strcasecmp(ptr,"Transport"))n=3;
		else if(!strcasecmp(ptr,"com.ses.streamID"))n=4;
		else if(!strcasecmp(ptr,"RTP-Info"))n=5;
		else if(!strcasecmp(ptr,"Public"))n=6;
		else if(!strcasecmp(ptr,"Content-Type"))n=7;
		else if(!strcasecmp(ptr,"Content-Base"))n=8;
		else if(!strcasecmp(ptr,"Content-Length"))n=9;
		else continue;

		if(hdr[n])goto err;
		if(!(ptr=strtok_r(NULL,"\r\n",&end)))goto err;
		while(*ptr==' '||*ptr=='\t')ptr++;
		hdr[n]=ptr;
	}

	l=0;
	if(hdr[9])
	{
		l=strtol(hdr[9],&end,10);
		if(l<0||hdr[9]==end||*end||l>INT_MAX||l<len-i)goto err;
	}
	if(l)
	{
		if(!(body=malloc(l+1)))goto err;

		if(len-i)memcpy(body,bfr+i,len-i);
		for(i=len-i;i<l;i+=r)
		{
			switch(poll(&p,1,1000))
			{
			case -1:if(errno==EINTR)continue;
			case 0:	goto err;
			default:if(!(p.revents&POLLIN))goto err;
			}

			if((r=read(c->rtspfd,body+i,l-i))<=0)goto err;
		}
		body[i]=0;
	}

	if(hdr[0]&&strcmp(hdr[0],"close"))goto err;

	if(!hdr[1])goto err;
	else
	{
		l=strtol(hdr[1],&end,10);
		if(l<0||hdr[1]==end||*end||l!=c->cseq)goto err;
	}

	if(hdr[2])
	{
		ptr=strtok_r(hdr[2],"; \t",&mem);
		if(!c->sess[0])
		{
			if(strlen(ptr)>=sizeof(c->sess))goto err;
			strcpy(c->sess,ptr);
			if((ptr=strtok_r(NULL,"\r\n",&mem)))
			{
				while(*ptr==' '||*ptr=='\t')ptr++;
				if(strncmp(ptr,"timeout=",8))goto err;
				l=strtol(ptr+8,&end,10);
				if(l<0||ptr+8==end||l>INT_MAX)goto err;
				if(!l)c->tmo=-1;
				else if(l<10)goto err;
				else c->tmo=l<<2;
			}
		}
		else if(strcmp(ptr,c->sess))goto err;
	}
	else if(c->sess[0])goto err;

	if(hdr[3])
	{
		if(!t)goto err;

		n=0;
		for(ptr=strtok_r(hdr[3],"; \t",&mem);ptr;
			ptr=strtok_r(NULL,"; \t",&mem))
		{
			if(!strcmp(ptr,"RTP/AVP"))n=1;
			else if(!strcmp(ptr,"RTP/AVP/UDP"))n=1;
			else if(!strcmp(ptr,"unicast"))
			{
				if(t->mode)goto err;
				t->mode=SATIP_TYPE_RTSP;
			}
			else if(!strcmp(ptr,"multicast"))
			{
				if(t->mode)goto err;
				t->mode=SATIP_TYPE_RTP;
			}
			else if(!strncmp(ptr,"source",6))
			{
				if(ptr[6]=='=')
				{
					if(str2addr(ptr+7,&t->sadr))goto err;
					if(anyaddr(&t->sadr))goto err;
					if(invalid_mcast(&t->sadr)!=-1)goto err;
				}
				else if(ptr[6])goto err;
			}
			else if(!strncmp(ptr,"destination",11))
			{
				if(ptr[11]=='=')
				{
					if(str2addr(ptr+12,&t->addr))goto err;
					if(anyaddr(&t->addr))goto err;
					switch(t->mode)
					{
					case SATIP_TYPE_RTSP:
						if(invalid_mcast(&t->addr)!=-1)
							goto err;
						break;
					case SATIP_TYPE_RTP:
						if(invalid_mcast(&t->addr)==-1)
							goto err;
						break;
					}
				}
				else if(ptr[11])goto err;
			}
			else if(!strncmp(ptr,"interleaved=",12));
			else if(!strcmp(ptr,"append"));
			else if(!strncmp(ptr,"ttl=",4))
			{
				if(t->ttl)goto err;
				l=strtol(ptr+4,&end,10);
				if(l<1||l>255||ptr+4==end||*end)goto err;
				t->ttl=l;
			}
			else if(!strncmp(ptr,"layers=",7));
			else if(!strncmp(ptr,"port=",5))
			{
				if(t->mrtp||t->mrtcp)goto err;
				l=strtol(ptr+5,&end,10);
				if(l<1||l>65535||ptr+5==end||*end!='-')goto err;
				t->mrtp=l;
				ptr=end+1;
				l=strtol(ptr,&end,10);
				if(l<1||l>65535||ptr==end||*end)goto err;
				t->mrtcp=l;
				if((t->mrtp&1)||t->mrtcp!=t->mrtp+1)goto err;
			}
			else if(!strncmp(ptr,"client_port=",12))
			{
				if(t->crtp||t->crtcp)goto err;
				l=strtol(ptr+12,&end,10);
				if(l<1||l>65535||ptr+12==end||*end!='-')
					goto err;
				t->crtp=l;
				ptr=end+1;
				l=strtol(ptr,&end,10);
				if(l<1||l>65535||ptr==end||*end)goto err;
				t->crtcp=l;
				if((t->crtp&1)||t->crtcp!=t->crtp+1)goto err;
			}
			else if(!strncmp(ptr,"server_port=",12))
			{
				if(t->srtp||t->srtcp)goto err;
				l=strtol(ptr+12,&end,10);
				if(l<1||l>65535||ptr+12==end||*end!='-')
					goto err;
				t->srtp=l;
				ptr=end+1;
				l=strtol(ptr,&end,10);
				if(l<1||l>65535||ptr==end||*end)goto err;
				t->srtcp=l;
				if((t->srtp&1)||t->srtcp!=t->srtp+1)goto err;
			}
			else if(!strncmp(ptr,"ssrc=",5))
			{
				l=strtol(ptr+5,&end,16);
				if(l<0||l>UINT_MAX||ptr+5==end||*end)goto err;
				t->ssrc=l;
			}
			else if(!strncmp(ptr,"mode=",5));
			else goto err;
		}
		if(!n)goto err;

		switch(t->mode)
		{
		case SATIP_TYPE_RTSP:
			if(!t->crtp||t->mrtp||t->addr.family||t->ttl)goto err;
			if(c->rtp!=t->crtp||c->rtcp!=t->crtcp)goto err;
			break;
		case SATIP_TYPE_RTP:
			if(!t->mrtp||t->crtp||!t->addr.family||!t->ttl)goto err;
			if(c->mport!=t->mrtp||c->mport+1!=t->mrtcp)goto err;
			break;
		default:goto err;
		}
	}

	if(hdr[4])
	{
		l=strtol(hdr[4],&end,10);
		if(l<0||l>65535||hdr[1]==end||*end)goto err;
		c->sid=l;
	}

	if(hdr[5])
		for(ptr=strtok_r(hdr[5],"; \t",&mem);ptr;
			ptr=strtok_r(NULL,"; \t",&mem))
				if(!strncmp(ptr,"seq=",4))
	{
		l=strtol(ptr+4,&end,10);
		if(l<0||l>65535||ptr+4==end||*end)goto err;
		c->seq=(unsigned short)l;
		c->flags|=RTSP_SEQ;
	}

	if(hdr[6])
		for(ptr=strtok_r(hdr[5],", \t",&mem);ptr;
			ptr=strtok_r(NULL,", \t",&mem))
	{
		if(!strcmp(ptr,"OPTIONS"));
		else if(!strcmp(ptr,"DESCRIBE"));
		else if(!strcmp(ptr,"SETUP"));
		else if(!strcmp(ptr,"PLAY"));
		else if(!strcmp(ptr,"TEARDOWN"));
		else if(!strcmp(ptr,"ANNOUNCE"));
		else if(!strcmp(ptr,"PAUSE"));
		else if(!strcmp(ptr,"GET_PARAMETER"));
		else if(!strcmp(ptr,"SET_PARAMETER"));
		else if(!strcmp(ptr,"REDIRECT"));
		else if(!strcmp(ptr,"RECORD"));
		else goto err;
	}

	if(hdr[7])switch(result)
	{
	case 200:
		if(strcmp(hdr[7],"application/sdp"))goto err;
		break;
	default:if(strcmp(hdr[7],"text/parameters"))goto err;
		break;
	}
	else if(body)goto err;

	if(hdr[8])
	{
		addr2bstr(&c->addr,addr,sizeof(addr));
		l=strlen(addr);
		if(strncmp(hdr[8],"rtsp://",7))goto err;
		if(strncmp(hdr[8]+7,addr,l))goto err;
		if(strncmp(hdr[8]+7+l,"/",1))goto err;
	}

	if(!(c->flags&RTSP_PERSIST)||hdr[0])
	{
		close(c->rtspfd);
		c->rtspfd=-1;
	}

	if(data)
	{
		data->intval=i;
		data->ptrval=body;
	}
	else free(body);

	return result==200?0:result;

err:	if(body)free(body);
	return -1;
}

static int rtsp_describe(CONN *c,SATIP_DATA *d)
{
	int r;
	int len;
	char addr[INET6_ADDRSTRLEN+2];
	char bfr[1024];

	if(c->sess[0])goto err;

	addr2bstr(&c->addr,addr,sizeof(addr));
	len=snprintf(bfr,sizeof(bfr),"DESCRIBE rtsp://%s/ RTSP/1.0\r\nCSeq: 1"
		"\r\nAccept: application/sdp\r\nConnection: close\r\n\r\n",
		addr);

	if((c->rtspfd=tcp(&c->addr,htons(c->port),c->instance->devidx,0))==-1)
		goto err;

	if(write(c->rtspfd,bfr,len)!=len)goto err;

	if((r=rtsp_reply(c,d,NULL))==-1)goto err;

	if(c->rtspfd!=-1)close(c->rtspfd);

	return r;

err:	close(c->rtspfd);
	return -1;
}

static int rtsp_options(CONN *c)
{
	int r;
	int len;
	char addr[INET6_ADDRSTRLEN+2];
	char bfr[1024];

	if(!c->sess[0])goto err;

	addr2bstr(&c->addr,addr,sizeof(addr));
	len=snprintf(bfr,sizeof(bfr),"OPTIONS rtsp://%s/ RTSP/1.0\r\n"
		"CSeq: %u\r\nSession: %s\r\n%s\r\n",addr,++c->cseq,c->sess,
		(c->flags&RTSP_PERSIST)?"":"Connection: close\r\n");

	if(c->rtspfd==-1)
		if((c->rtspfd=tcp(&c->addr,htons(c->port),c->instance->devidx,
			0))==-1)goto err;

	if(write(c->rtspfd,bfr,len)!=len)goto err;

	if((r=rtsp_reply(c,NULL,NULL))==-1)goto err;

	if(c->rtspfd!=-1)if(!(c->flags&RTSP_PERSIST))
	{
		close(c->rtspfd);
		c->rtspfd=-1;
	}

	return r;

err:	if(c->rtspfd!=-1)close(c->rtspfd);
	c->rtspfd=-1;
	return -1;
}

static int rtsp_play(CONN *c,SATIP_PIDS *set,SATIP_PIDS *add,SATIP_PIDS *del,
	SATIP_EXTRA *extra)
{
	int r;
	int len;
	char addr[INET6_ADDRSTRLEN+2];
	char bfr[1024];

	if(!c->sess[0]||c->sid==-1)goto dfterr;

	addr2bstr(&c->addr,addr,sizeof(addr));
	if(set||add||del)
	{
		if(!c->tune.msys)goto dfterr;
		len=snprintf(bfr,sizeof(bfr),"PLAY rtsp://%s/stream=%d",addr,
			c->sid);
		if((r=satip_util_create(SATIP_TYPE_QRY,
			c->instance->strict?SATIP_STRICTQRY:0,
			NULL,0,0,&c->tune,set,add,del,extra,bfr+len,
			sizeof(bfr)-len))==SATIP_ERR_SYNTX)goto err;
		len+=r;
		len+=snprintf(bfr+len,sizeof(bfr)-len," RTSP/1.0\r\nCSeq: %u"
			"\r\nSession: %s\r\n%s\r\n",++c->cseq,c->sess,
			(c->flags&RTSP_PERSIST)?"":"Connection: close\r\n");

	}
	else len=snprintf(bfr,sizeof(bfr),"PLAY rtsp://%s/stream=%d RTSP/1.0"
		"\r\nCSeq: %u\r\nSession: %s\r\n%s\r\n",addr,c->sid,++c->cseq,
		c->sess,(c->flags&RTSP_PERSIST)?"":"Connection: close\r\n");

	if(c->rtspfd==-1)
		if((c->rtspfd=tcp(&c->addr,htons(c->port),c->instance->devidx,
			0))==-1)goto dfterr;

	if(write(c->rtspfd,bfr,len)!=len)goto dfterr;

	c->flags&=~RTSP_SEQ;
	if((r=rtsp_reply(c,NULL,NULL))==-1)goto dfterr;

	if(c->rtspfd!=-1)if(!(c->flags&RTSP_PERSIST))
	{
		close(c->rtspfd);
		c->rtspfd=-1;
	}

	return r;

dfterr:	r=SATIP_SYSFAIL;
err:	if(c->rtspfd!=-1)close(c->rtspfd);
	c->rtspfd=-1;
	return r;
}

static int rtsp_teardown(CONN *c)
{
	int r;
	int len;
	char addr[INET6_ADDRSTRLEN+2];
	char bfr[1024];

	if(!c->sess[0])goto err;

	addr2bstr(&c->addr,addr,sizeof(addr));
	len=snprintf(bfr,sizeof(bfr),"TEARDOWN rtsp://%s/stream=%d RTSP/1.0\r\n"
		"CSeq: %u\r\nSession: %s\r\n%s\r\n",addr,c->sid,++c->cseq,
		c->sess,(c->flags&RTSP_PERSIST)?"":"Connection: close\r\n");

	if(c->rtspfd==-1)
		if((c->rtspfd=tcp(&c->addr,htons(c->port),c->instance->devidx,0
			))==-1)goto err;

	if(write(c->rtspfd,bfr,len)!=len)goto err;

	if((r=rtsp_reply(c,NULL,NULL))==-1)goto err;

	if(c->rtspfd!=-1)if(!(c->flags&RTSP_PERSIST))
	{
		close(c->rtspfd);
		c->rtspfd=-1;
	}

	return r;

err:	if(c->rtspfd!=-1)close(c->rtspfd);
	c->rtspfd=-1;
	return SATIP_SYSFAIL;
}

static int rtsp_setup_unicast(CONN *c,SATIP_TUNE *tune,SATIP_PIDS *set,
	SATIP_EXTRA *extra)
{
	int r;
	int len;
	socklen_t l;
	SOCK a;
	char addr[INET6_ADDRSTRLEN+2];
	char bfr[1024];
	TRANSPORT t;

	if(c->sess[0])goto dfterr;

	addr2bstr(&c->addr,addr,sizeof(addr));
	len=snprintf(bfr,sizeof(bfr),"SETUP rtsp://%s/",addr);
	if((r=satip_util_create(SATIP_TYPE_QRY,
		c->instance->strict?SATIP_STRICTQRY:0,
		NULL,0,0,tune,set,NULL,NULL,extra,bfr+len,sizeof(bfr)-len))
		==SATIP_ERR_SYNTX)goto err;
	len+=r;
	len+=snprintf(bfr+len,sizeof(bfr)-len," RTSP/1.0\r\nCSeq: %u\r\n"
		"Transport: RTP/AVP;unicast;client_port=%d-%d\r\n%s\r\n",
		++c->cseq,c->rtp,c->rtcp,
		(c->flags&RTSP_PERSIST)?"":"Connection: close\r\n");

	if(c->rtspfd==-1)
		if((c->rtspfd=tcp(&c->addr,htons(c->port),c->instance->devidx,
			0))==-1)goto dfterr;

	l=sizeof(a);
	if(getsockname(c->rtspfd,(struct sockaddr *)&a,&l))goto dfterr;
	if(invalid_sock(&a,l))goto dfterr;

	if(write(c->rtspfd,bfr,len)!=len)goto dfterr;

	if((r=rtsp_reply(c,NULL,&t))==-1)goto dfterr;

	if(r)c->sess[0]=0;
	else if(t.mode!=SATIP_TYPE_RTSP||!c->sess[0]||c->sid==-1)goto dfterr;

	if(t.crtp&&t.crtp!=c->rtp)goto dfterr;
	if(t.crtcp&&t.crtcp!=c->rtcp)goto dfterr;
	if(t.addr.family&&ascmp(&t.addr,&a))goto dfterr;

	c->srtp=t.srtp;
	c->srtcp=t.srtcp;
	c->sadr=t.sadr;
	c->ssrc=t.ssrc;

	if(c->rtspfd!=-1)if(!(c->flags&RTSP_PERSIST))
	{
		close(c->rtspfd);
		c->rtspfd=-1;
	}

	return r;

dfterr:	r=SATIP_SYSFAIL;
err:	if(c->rtspfd!=-1)close(c->rtspfd);
	c->rtspfd=-1;
	return r;
}

static int rtsp_setup_multicast(CONN *c,SATIP_TUNE *tune,SATIP_PIDS *set,
	SATIP_EXTRA *extra)
{
	int r;
	int len;
	char addr[INET6_ADDRSTRLEN+2];
	char bfr[1024];
	TRANSPORT t;

	if(c->sess[0])goto dfterr;

	addr2bstr(&c->addr,addr,sizeof(addr));
	len=snprintf(bfr,sizeof(bfr),"SETUP rtsp://%s/",addr);
	if((r=satip_util_create(SATIP_TYPE_QRY,
		c->instance->strict?SATIP_STRICTQRY:0,
		NULL,0,0,tune,set,NULL,NULL,extra,bfr+len,sizeof(bfr)-len))
		==SATIP_ERR_SYNTX)goto err;
	len+=r;
	addr2str(&c->maddr,addr,sizeof(addr));
	len+=snprintf(bfr+len,sizeof(bfr)-len," RTSP/1.0\r\nCSeq: %u\r\n"
		"Transport: RTP/AVP;multicast;destination=%s;port=%u-%u;ttl=%u"
		"\r\n%s\r\n",++c->cseq,addr,c->mport,c->mport+1,c->mttl,
		(c->flags&RTSP_PERSIST)?"":"Connection: close\r\n");

	if(c->rtspfd==-1)
		if((c->rtspfd=tcp(&c->addr,htons(c->port),c->instance->devidx,
			0))==-1)goto dfterr;

	if(write(c->rtspfd,bfr,len)!=len)goto dfterr;

	if((r=rtsp_reply(c,NULL,&t))==-1)goto dfterr;

	if(r)c->sess[0]=0;
	else if(t.mode!=SATIP_TYPE_RTP||!c->sess[0]||c->sid==-1)goto dfterr;

	if(t.mrtp&&t.mrtp!=c->mport)goto dfterr;
	if(t.mrtcp&&t.mrtcp!=c->mport+1)goto dfterr;
	if(t.addr.family&&aacmp(&t.addr,&c->maddr))goto dfterr;
	if(t.ttl&&t.ttl!=c->mttl)goto dfterr;

	if(c->rtspfd!=-1)if(!(c->flags&RTSP_PERSIST))
	{
		close(c->rtspfd);
		c->rtspfd=-1;
	}

	return r;

dfterr:	r=SATIP_SYSFAIL;
err:	if(c->rtspfd!=-1)close(c->rtspfd);
	c->rtspfd=-1;
	return r;
}

static int http_setup(CONN *c,SATIP_TUNE *tune,SATIP_PIDS *set,
	SATIP_EXTRA *extra)
{
	int r;
	int pos;
	int len;
	long ll;
	char *line;
	char *ptr;
	char *mem;
	char *mem2;
	char *end;
	struct pollfd p;
	char addr[INET6_ADDRSTRLEN];

	switch(set->numpids)
	{
	case SATIP_SECTION:
		ptr="application/octet-stream";
		pos=0;
		break;
	case SATIP_SIGNAL:
		ptr="text/plain";
		pos=0;
		break;
	default:ptr="video/MP2T";
		pos=c->instance->rtpbuffer;
		break;
	}

	addr2str(&c->addr,addr,sizeof(addr));
	memcpy(c->bfr,"GET /",5);
	if((r=satip_util_create(SATIP_TYPE_QRY,
		c->instance->strict?SATIP_STRICTQRY:0,
		NULL,0,0,tune,set,NULL,NULL,extra,c->bfr+5,TCPBUFF-5))
		==SATIP_ERR_SYNTX)goto err;
	len=r+5;
	len+=snprintf(c->bfr+len,TCPBUFF-len," HTTP/1.1\r\nHost: %s\r\n"
		"Accept: %s\r\nConnection: close\r\n\r\n",addr,ptr);

	if((c->httpfd=tcp(&c->addr,htons(c->port),c->instance->devidx,pos))==-1)
		goto dfterr;

	if(write(c->httpfd,c->bfr,len)!=len)goto dfterr;

	p.fd=c->httpfd;
	p.events=POLLIN;

	for(pos=0,c->fill=0,r=0;r<2;pos++)
	{
		if(pos==TCPBUFF)goto dfterr;

		if(pos==c->fill)switch((len=poll(&p,1,1000)))
		{
		case -1:if(errno==EINTR)continue;
		case 0:	goto dfterr;
		default:if(!(p.revents&POLLIN))goto dfterr;
			if((len=read(c->httpfd,c->bfr+c->fill,
				TCPBUFF-c->fill>188?188:TCPBUFF-c->fill))<=0)
					goto dfterr;
			c->fill+=len;
			break;
		}

		switch(c->bfr[pos])
		{
		case '\n':
			r++;
		case '\r':
			break;
		default:r=0;
			break;
		}
	}

	c->bfr[pos]=0;

	if(!(line=strtok_r(c->bfr,"\r\n",&mem)))goto dfterr;
	if(!(ptr=strtok_r(line," \t",&mem2)))goto dfterr;
	if(strncmp(ptr,"HTTP/",5))goto dfterr;
	if(!(ptr=strtok_r(NULL," \t",&mem2)))goto dfterr;
	ll=strtol(ptr,&end,10);
	if(*end||end==ptr||ll<100||ll>999)goto dfterr;
	if(ll!=200)
	{
		r=ll;
		goto err;
	}

	for(line=strtok_r(NULL,"\r\n",&mem),r=0;line;
		line=strtok_r(NULL,"\r\n",&mem))
			if(!strncasecmp(line,"Content-Type:",13))
	{
		if(set->numpids!=SATIP_SECTION&&set->numpids!=SATIP_SIGNAL&&
			strstr(line+13,"video/MP2T"))
		{
			r=1;
			break;
		}
		if(set->numpids==SATIP_SIGNAL&&
			strstr(line+13,"text/plain"))
		{
			r=1;
			break;
		}
		if(set->numpids==SATIP_SECTION&&
			strstr(line+13,"application/octet-stream"))
		{
			r=1;
			break;
		}
	}
	if(!r)goto dfterr;

	if(pos!=c->fill)memmove(c->bfr,c->bfr+pos,c->fill-pos);
	c->fill-=pos;

	return 0;

dfterr:	r=SATIP_SYSFAIL;
err:	if(c->httpfd!=-1)close(c->httpfd);
	c->httpfd=-1;
	return r;
}

static int parse_tuner(char *data,SATIP_STATUS *s)
{
	long l;
	double d;
	char *ptr;
	char *end;
	char *mem;
	char *prm;

	ptr=data;
	if(!(mem=strchr(data,',')))goto err;
	*mem++=0;
	l=strtol(ptr,&end,10);
	if(ptr==end||*end||l<0||l>65535)goto err;
	s->tune.fe=l;

	ptr=mem;
	if(!(mem=strchr(mem,',')))goto err;
	*mem++=0;
	l=strtol(ptr,&end,10);
	if(ptr==end||*end||l<0||l>255)goto err;
	s->level=l;

	ptr=mem;
	if(!(mem=strchr(mem,',')))goto err;
	*mem++=0;
	l=strtol(ptr,&end,10);
	if(ptr==end||*end||l<0||l>1)goto err;
	s->lock=l;

	ptr=mem;
	if(!(mem=strchr(mem,',')))goto err;
	*mem++=0;
	l=strtol(ptr,&end,10);
	if(ptr==end||*end||l<0||l>15)goto err;
	s->quality=l;

	ptr=mem;
	if(!(mem=strchr(mem,',')))goto err;
	*mem++=0;
	d=strtod(ptr,&end);
	if(d<1||d>15000||*end||end==ptr)goto err;
	s->tune.freq=(unsigned long long)(d*1000000.0);

	prm=mem;
	if(!(mem=strchr(mem,',')))goto err;
	*mem++=0;

	ptr=mem;
	if(!(mem=strchr(mem,',')))goto err;
	*mem++=0;
	if(!strcmp(ptr,"dvbs"))s->tune.msys=SATIP_DVBS;
	else if(!strcmp(ptr,"dvbs2"))s->tune.msys=SATIP_DVBS2;
	else if(!strcmp(ptr,"dvbt"))s->tune.msys=SATIP_DVBT;
	else if(!strcmp(ptr,"dvbt2"))s->tune.msys=SATIP_DVBT2;
	else if(!strcmp(ptr,"dvbc"))s->tune.msys=SATIP_DVBC;
	else if(!strcmp(ptr,"dvbc2"))s->tune.msys=SATIP_DVBC2;
	else goto err;

	switch(s->tune.msys)
	{
	case SATIP_DVBS:
	case SATIP_DVBS2:
		if(prm[1])goto err;
		switch(*prm)
		{
		case 'h':
			s->tune.pol=SATIP_POL_H;
			break;
		case 'v':
			s->tune.pol=SATIP_POL_V;
			break;
		case 'l':
			s->tune.pol=SATIP_POL_L;
			break;
		case 'r':
			s->tune.pol=SATIP_POL_R;
			break;
		default:goto err;
		}

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			if(!strcmp(ptr,"qpsk"))s->tune.mtype=SATIP_QPSK;
			else if(!strcmp(ptr,"8psk"))s->tune.mtype=SATIP_8PSK;
			else if(!strcmp(ptr,"8psk"))s->tune.mtype=SATIP_8PSK;
			else if(!strcmp(ptr,"x_16apsk"))
				s->tune.mtype=SATIP_16APSK;
			else if(!strcmp(ptr,"x_32apsk"))
				s->tune.mtype=SATIP_32APSK;
			else goto err;
		}
		else s->tune.mtype=SATIP_AUTOQ;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			if(!strcmp(ptr,"off"))s->tune.plts=SATIP_PLTS_OFF;
			else if(!strcmp(ptr,"on"))s->tune.plts=SATIP_PLTS_ON;
			else goto err;
		}
		else s->tune.plts=SATIP_PLTS_AUTO;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			if(!strcmp(ptr,"0.35"))s->tune.ro=SATIP_ROFF_035;
			else if(!strcmp(ptr,"0.25"))s->tune.ro=SATIP_ROFF_025;
			else if(!strcmp(ptr,"0.20"))s->tune.ro=SATIP_ROFF_020;
			else goto err;
		}
		else s->tune.ro=SATIP_ROFF_AUTO;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		l=strtol(ptr,&end,10);
		if(ptr==end||*end||l<1||l>INT_MAX/1000)goto err;
		s->tune.sr=l*1000;

		ptr=mem;
		if(*ptr)
		{
			if(!strcmp(ptr,"12"))s->tune.fec=SATIP_FEC_12;
			else if(!strcmp(ptr,"23"))s->tune.fec=SATIP_FEC_23;
			else if(!strcmp(ptr,"34"))s->tune.fec=SATIP_FEC_34;
			else if(!strcmp(ptr,"35"))s->tune.fec=SATIP_FEC_35;
			else if(!strcmp(ptr,"45"))s->tune.fec=SATIP_FEC_45;
			else if(!strcmp(ptr,"56"))s->tune.fec=SATIP_FEC_56;
			else if(!strcmp(ptr,"78"))s->tune.fec=SATIP_FEC_78;
			else if(!strcmp(ptr,"89"))s->tune.fec=SATIP_FEC_89;
			else if(!strcmp(ptr,"910"))s->tune.fec=SATIP_FEC_910;
			else goto err;
		}
		else s->tune.fec=SATIP_FEC_AUTO;
		break;

	case SATIP_DVBT:
	case SATIP_DVBT2:
		if(!strcmp(prm,"1.712"))s->tune.bw=SATIP_BW_1712;
		if(!strcmp(prm,"10"))s->tune.bw=SATIP_BW_10;
		else if(!prm[1])switch(prm[0])
		{
		case '5':
			s->tune.bw=SATIP_BW_5;
			break;
		case '6':
			s->tune.bw=SATIP_BW_6;
			break;
		case '7':
			s->tune.bw=SATIP_BW_7;
			break;
		case '8':
			s->tune.bw=SATIP_BW_8;
			break;
		default:goto err;
		}
		else if(!*prm)s->tune.bw=SATIP_BW_AUTO;
		else goto err;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			if(!strcmp(ptr,"1k"))s->tune.tmode=SATIP_TMOD_1K;
			else if(!strcmp(ptr,"2k"))s->tune.tmode=SATIP_TMOD_2K;
			else if(!strcmp(ptr,"4k"))s->tune.tmode=SATIP_TMOD_4K;
			else if(!strcmp(ptr,"8k"))s->tune.tmode=SATIP_TMOD_8K;
			else if(!strcmp(ptr,"16k"))s->tune.tmode=SATIP_TMOD_16K;
			else if(!strcmp(ptr,"32k"))s->tune.tmode=SATIP_TMOD_32K;
			else goto err;
		}
		else s->tune.tmode=SATIP_TMOD_AUTO;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			if(!strcmp(ptr,"qpsk"))s->tune.mtype=SATIP_QPSK;
			else if(!strcmp(ptr,"16qam"))s->tune.mtype=SATIP_16Q;
			else if(!strcmp(ptr,"64qam"))s->tune.mtype=SATIP_64Q;
			else if(!strcmp(ptr,"256qam"))s->tune.mtype=SATIP_256Q;
			else goto err;
		}
		else s->tune.mtype=SATIP_AUTOQ;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			if(!strcmp(ptr,"14"))s->tune.gi=SATIP_GI_14;
			else if(!strcmp(ptr,"18"))s->tune.gi=SATIP_GI_18;
			else if(!strcmp(ptr,"116"))s->tune.gi=SATIP_GI_116;
			else if(!strcmp(ptr,"132"))s->tune.gi=SATIP_GI_132;
			else if(!strcmp(ptr,"1128"))s->tune.gi=SATIP_GI_1128;
			else if(!strcmp(ptr,"19128"))s->tune.gi=SATIP_GI_19128;
			else if(!strcmp(ptr,"19256"))s->tune.gi=SATIP_GI_19256;
			else goto err;
		}
		else s->tune.gi=SATIP_GI_AUTO;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			if(!strcmp(ptr,"12"))s->tune.fec=SATIP_FEC_12;
			else if(!strcmp(ptr,"23"))s->tune.fec=SATIP_FEC_23;
			else if(!strcmp(ptr,"34"))s->tune.fec=SATIP_FEC_34;
			else if(!strcmp(ptr,"35"))s->tune.fec=SATIP_FEC_35;
			else if(!strcmp(ptr,"45"))s->tune.fec=SATIP_FEC_45;
			else if(!strcmp(ptr,"56"))s->tune.fec=SATIP_FEC_56;
			else if(!strcmp(ptr,"78"))s->tune.fec=SATIP_FEC_78;
			else goto err;
		}
		else s->tune.fec=SATIP_FEC_AUTO;

		s->tune.feclp=SATIP_FEC_AUTO;
		s->tune.hier=SATIP_HIER_AUTO;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			l=strtol(ptr,&end,10);
			if(ptr==end||*end||l<0||l>255)goto err;
			s->tune.plp=l;
		}

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			l=strtol(ptr,&end,10);
			if(ptr==end||*end||l<0||l>65535)goto err;
			s->tune.t2id=l;
		}

		ptr=mem;
		if(*ptr)
		{
			if(!strcmp(ptr,"0"))s->tune.sm=SATIP_SM_SISO;
			else if(!strcmp(ptr,"1"))s->tune.sm=SATIP_SM_MISO;
			else goto err;
		}
		else s->tune.sm=SATIP_SM_AUTO;
		break;

	case SATIP_DVBC:
	case SATIP_DVBC2:
		if(!prm[1])switch(prm[0])
		{
		case '6':
			s->tune.bw=SATIP_BW_6;
			break;
		case '8':
			s->tune.bw=SATIP_BW_8;
			break;
		default:goto err;
		}
		else if(!*prm)s->tune.bw=SATIP_BW_AUTO;
		else goto err;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			if(!strcmp(ptr,"16qam"))s->tune.mtype=SATIP_16Q;
			else if(!strcmp(ptr,"32qam"))s->tune.mtype=SATIP_32Q;
			else if(!strcmp(ptr,"64qam"))s->tune.mtype=SATIP_64Q;
			else if(!strcmp(ptr,"128qam"))s->tune.mtype=SATIP_128Q;
			else if(!strcmp(ptr,"256qam"))s->tune.mtype=SATIP_256Q;
			else goto err;
		}
		else s->tune.mtype=SATIP_AUTOQ;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		l=strtol(ptr,&end,10);
		if(ptr==end||*end||l<1||l>INT_MAX/1000)goto err;
		s->tune.sr=l*1000;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			if(!strcmp(ptr,"0"))s->tune.c2tft=SATIP_TFT_DS;
			else if(!strcmp(ptr,"1"))s->tune.c2tft=SATIP_TFT_C2;
			else if(!strcmp(ptr,"2"))s->tune.c2tft=SATIP_TFT_IT;
			else goto err;
		}
		else s->tune.c2tft=SATIP_TFT_AUTO;

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			l=strtol(ptr,&end,10);
			if(ptr==end||*end||l<0||l>255)goto err;
			s->tune.ds=l;
		}

		ptr=mem;
		if(!(mem=strchr(mem,',')))goto err;
		*mem++=0;
		if(*ptr)
		{
			l=strtol(ptr,&end,10);
			if(ptr==end||*end||l<0||l>255)goto err;
			s->tune.plp=l;
		}

		ptr=mem;
		if(*ptr)
		{
			if(!strcmp(ptr,"0"))s->tune.specinv=SATIP_SPI_OFF;
			else if(!strcmp(ptr,"1"))s->tune.specinv=SATIP_SPI_ON;
			else goto err;
		}
		else s->tune.specinv=SATIP_SPI_AUTO;
		break;
	}

	return 0;

err:	return -1;
}

static SATIP_CLN_STREAMINFO *parse_streaminfo(char *data)
{
	int media=0;
	int bits=0;
	long l;
	char *port=NULL;
	char *host=NULL;
	char *ttl=NULL;
	char *stream=NULL;
	char *src=NULL;
	char *tuner=NULL;
	char *pids=NULL;
	char *line;
	char *ptr;
	char *mem;
	char *mem2;
	ADDR a;
	SATIP_CLN_STREAMINFO *info=NULL;
	SATIP_CLN_STREAMINFO *e=NULL;
	SATIP_CLN_STREAMINFO **s=&info;

	for(line=strtok_r(data,"\r\n",&mem);line;
		line=strtok_r(NULL,"\r\n",&mem))
	{
		if(!strncmp(line,"m=",2))
		{
			if(strncmp(line,"m=video",7))goto err;
			if(bits)goto err;
			bits|=0x0001;
			media=1;
			if(!(port=strtok_r(line+7," \t",&mem2)))goto err;
			if(!(ptr=strtok_r(NULL," \t",&mem2)))goto err;
			if(strcmp(ptr,"RTP/AVP"))goto err;
			if(!(ptr=strtok_r(NULL," \t",&mem2)))goto err;
			if(strcmp(ptr,"33"))goto err;
		}
		else if(!strncmp(line,"c=",2))
		{
			if(strncmp(line,"c=IN",4))goto err;
			if(!bits||(bits&0x0002))goto err;
			bits|=0x0002;
			if(!(ptr=strtok_r(line+4," \t",&mem2)))goto err;
			if(strcmp(ptr,"IP4"))goto err;
			if(!(host=strtok_r(NULL," \t",&mem2)))goto err;
			if((ttl=strchr(host,'/')))*ttl++=0;
		}
		else if(!strncmp(line,"a=",2))
		{
			if(!bits)goto err;
			if(!strncmp(line,"a=control:",10))
			{
				if(bits&0x0004)goto err;
				bits|=0x0004;
				if(!(stream=strtok_r(line+10," \t",&mem2)))
					goto err;
				if(strncmp(stream,"stream=",7))goto err;
				stream+=7;
			}
			else if(!strncmp(line,"a=fmtp:",7))
			{
				if(bits&0x0008)goto err;
				bits|=0x0008;
				for(line+=7;*line==' '||*line=='\t';line++);
				if(*line++!='3')goto err;
				if(*line++!='3')goto err;
				if(*line!=' '&&*line!='\t')goto err;
				while(*line==' '||*line=='\t')line++;
				if(strncmp(line,"ver=",4))goto err;
				for(line+=4;*line&&*line!=';';line++);
				if(!*line++)goto err;
				if(!(ptr=strtok_r(line,"; \t",&mem2)))goto err;
				if(strncmp(ptr,"src=",4))src=NULL;
				else
				{
					src=ptr+4;
					if(!(ptr=strtok_r(NULL,"; \t",&mem2)))
						goto err;
				}
				if(strncmp(ptr,"tuner=",6))goto err;
				tuner=ptr+6;
				if(!(pids=strtok_r(NULL,"; \t",&mem2)))goto err;
				if(strncmp(pids,"pids=",5))goto err;
				pids+=5;
			}
			else if(!strcmp(line,"a=inactive"))
			{
				if(bits&0x0010)goto err;
				bits|=0x0030;
			}
			else if(!strcmp(line,"a=sendonly"))
			{
				if(bits&0x0010)goto err;
				bits|=0x0010;
			}
		}
		else if(media)goto err;

		if((bits&0x001f)==0x001f)
		{
			if(!(e=malloc(sizeof(SATIP_CLN_STREAMINFO))))goto err;
			memset(e,0,sizeof(SATIP_CLN_STREAMINFO));
			e->info.tune.plp=SATIP_UNDEF;
			e->info.tune.t2id=SATIP_UNDEF;
			e->info.tune.ds=SATIP_UNDEF;
			e->stream=SATIP_UNDEF;

			l=strtol(port,&ptr,10);
			if(l<0||l>65535||port==ptr||*ptr)goto err;
			e->port=l;

			if(str2addr(host,&a))goto err;
			if(!l&&!anyaddr(&a))goto err;
			if(l&&invalid_mcast(&a)==-1)goto err;
			strcpy(e->addr,host);

			if(l)e->type=SATIP_TYPE_RTP;
			else e->type=SATIP_TYPE_RTSP;

			e->inactive=(bits&0x20)?1:0;

			if(!anyaddr(&a))
			{
				if(!ttl)goto err;
				l=strtol(ttl,&ptr,10);
				if(l<1||l>255||ttl==ptr||*ptr)goto err;
				e->ttl=l;
			}

			l=strtol(stream,&ptr,10);
			if(l<0||l>65535||stream==ptr||*ptr)goto err;
			e->stream=l;

			if(src)
			{
				l=strtol(src,&ptr,10);
				if(l<1||l>255||src==ptr||*ptr)goto err;
				e->info.tune.src=l;
			}

			if(parse_tuner(tuner,&e->info))goto err;

			if(satip_util_parse(SATIP_PARSE_PID,0,0,pids,NULL,NULL,
				0,NULL,NULL,NULL,&e->info.set,NULL,NULL,NULL))
					goto err;

			*s=e;
			s=&e->next;
			e=NULL;
			bits=0;
		}
	}

	if(bits)goto err;

	return info;

err:	if(e)free(e);
	while(info)
	{
		e=info;
		info=e->next;
		free(e);
	}
	return NULL;
}

static void sigwrk(INSTANCE *inst,CONN *c)
{
	int base;
	int len;
	long ll;
	char *line;
	char *ptr;
	char *src;
	char *tuner;
	char *pids;
	char *mem;
	SATIP_DATA d;
	SATIP_STATUS s;

	if((len=read(c->httpfd,c->bfr+c->fill,TCPBUFF-c->fill))<=0)
	{
		if(len==-1&&errno==EINTR)return;
		if(__sync_bool_compare_and_swap(&c->htfail,0,1))
			epoll_ctl(inst->efd,EPOLL_CTL_DEL,c->httpfd,NULL);
		return;
	}
	c->fill+=len;

	base=0;
	while(1)
	{
		for(len=base;len<c->fill;len++)
			if(c->bfr[len]=='\r'||c->bfr[len]=='\n')break;
		if(len==c->fill)break;
		c->bfr[len]=0;

		line=c->bfr+base;

		if(strncmp(line,"ver=",4))goto cont;
		for(line+=4;*line&&*line!=';';line++);
		if(!*line++)goto cont;
		if(!(ptr=strtok_r(line,"; \t",&mem)))goto cont;
		if(strncmp(ptr,"src=",4))src=NULL;
		else
		{
			src=ptr+4;
			if(!(ptr=strtok_r(NULL,"; \t",&mem)))goto cont;
		}
		if(strncmp(ptr,"tuner=",6))goto cont;
		tuner=ptr+6;
		if(!(pids=strtok_r(NULL,"; \t",&mem)))goto cont;
		if(strncmp(pids,"pids=",5))goto cont;
		pids+=5;

		memset(&s.tune,0,sizeof(SATIP_TUNE));
		s.tune.plp=SATIP_UNDEF;
		s.tune.t2id=SATIP_UNDEF;
		s.tune.ds=SATIP_UNDEF;

		if(src)
		{
			ll=strtol(src,&ptr,10);
			if(ll<1||ll>255||src==ptr||*ptr)goto cont;
			s.tune.src=ll;
		}

		if(parse_tuner(tuner,&s))goto cont;

		if(satip_util_parse(SATIP_PARSE_PID,0,0,pids,NULL,NULL,0,NULL,
			NULL,NULL,&s.set,NULL,NULL,NULL))goto cont;

		d.intval=SATIP_RTCP;
		d.ptrval=&s;

		c->cb(&d,c->priv);

cont:		for(len++;len<c->fill;len++)
			if(c->bfr[len]!='\r'&&c->bfr[len]!='\n')break;
		base=len;
	}
	if(base==c->fill)c->fill=0;
	else
	{
		c->fill-=base;
		memmove(c->bfr,c->bfr+base,c->fill);
	}
}

static void section(INSTANCE *inst,CONN *c)
{
	int len;
	int msglen;
	SATIP_DATA d;

	if((len=read(c->httpfd,c->bfr+c->fill,SECTIONBUFF-c->fill))<=0)
	{
		if(len==-1&&errno==EINTR)return;
		if(__sync_bool_compare_and_swap(&c->htfail,0,1))
			epoll_ctl(inst->efd,EPOLL_CTL_DEL,c->httpfd,NULL);
		return;
	}
	c->fill+=len;

	if(c->fill>=2)
	{
		msglen=c->bfr[0];
		msglen<<=8;
		msglen|=c->bfr[1];

		if(c->fill>=msglen+2)
		{
			d.intval=msglen;
			d.ptrval=c->bfr+2;
			c->cb(&d,c->priv);
			if(c->fill>msglen+2)memmove(c->bfr,c->bfr+msglen+2,
				c->fill-msglen-2);
			c->fill-=msglen+2;
		}
	}
}

static void http(INSTANCE *inst,CONN *c)
{
	int len;
	SATIP_DATA d;
	
	if(UNLIKELY((len=read(c->httpfd,c->bfr+c->fill,TCPBUFF-c->fill))<=0))
	{
		if(len==-1&&errno==EINTR)return;
		if(__sync_bool_compare_and_swap(&c->htfail,0,1))
			epoll_ctl(inst->efd,EPOLL_CTL_DEL,c->httpfd,NULL);
		return;
	}
	c->fill+=len;

	switch(c->fill)
	{
	case 188:
	case 376:
	case 564:
	case 752:
	case 940:
	case 1128:
	case 1316:
	case 1504:
	case 1692:
	case 1880:
	case 2068:
	case 2256:
	case 2444:
	case 2632:
	case 2820:
	case 3008:
	case 3196:
	case 3384:
	case 3572:
	case 3760:
	case 3948:
	case 4136:
	case 4324:
	case 4512:
	case 4700:
	case 4888:
	case 5076:
	case 5264:
	case 5452:
	case 5640:
	case 5828:
	case 6016:
	case 6204:
	case 6392:
	case 6580:
	case 6768:
	case 6956:
	case 7144:
	case 7332:
	case 7520:
	case 7708:
	case 7896:
	case 8084:
		d.intval=c->fill;
		d.ptrval=c->bfr;
		c->cb(&d,c->priv);
		c->fill=0;
		break;

	default:len=c->fill/188;
		if(!len)break;
		len*=188;
		d.intval=len;
		d.ptrval=c->bfr;
		c->cb(&d,c->priv);
		if(UNLIKELY(len!=c->fill))
			memmove(c->bfr,c->bfr+len,c->fill-len);
		c->fill-=len;
		break;
	}
}

static void rtp(INSTANCE *inst,CONN *c)
{
	unsigned short port;
	int i;
	int n;
	int hlen;
	int blen;
	int idx;
	struct cmsghdr *cm;
	struct in_pktinfo *p4;
	struct in6_pktinfo *p6;
	QUEUE *q=c->queue;
	struct epoll_event e;
	SATIP_DATA dta[2];

	if(UNLIKELY((n=recvmmsg(c->rtpfd,q->vec,SATIP_MAX_BURST,0,NULL))<0))
	{
		if(errno!=EINTR)if(__sync_bool_compare_and_swap(&c->rtfail,0,1))
			epoll_ctl(inst->efd,EPOLL_CTL_DEL,c->rtpfd,NULL);
		return;
	}

	if(inst->fast)goto fast;

	if(n==SATIP_MAX_BURST)
	{
		if(!ioctl(c->rtpfd,FIONREAD,&i)&&i)
		{
fast:			e.events=EPOLLIN|EPOLLONESHOT;
			e.data.ptr=(char *)c+sizeof(void *)+1;
			if(epoll_ctl(inst->efd,EPOLL_CTL_MOD,c->rtpfd,&e))
				c->netwake=1;
		}
		else c->netwake=1;
	}
	else c->netwake=1;

	for(idx=-1,i=0;i<n;idx=-1,i++)
	{
		if(UNLIKELY(q->vec[i].msg_len<12))continue;
		if(UNLIKELY(invalid_sock(&q->a[i],
			q->vec[i].msg_hdr.msg_namelen)))continue;

		for(cm=CMSG_FIRSTHDR(&q->vec[i].msg_hdr);cm;
			cm=CMSG_NXTHDR(&q->vec[i].msg_hdr,cm))
				switch(cm->cmsg_level)
		{
		case IPPROTO_IP:
			if(UNLIKELY(sockfam(&q->a[i])!=AF_INET||
				cm->cmsg_type!=IP_PKTINFO))break;
			p4=(struct in_pktinfo *)CMSG_DATA(cm);
			idx=p4->ipi_ifindex;
			break;

		case IPPROTO_IPV6:
			if(UNLIKELY(sockfam(&q->a[i])!=AF_INET6||
				cm->cmsg_type!=IPV6_PKTINFO))break;
			p6=(struct in6_pktinfo *)CMSG_DATA(cm);
			idx=p6->ipi6_ifindex;
			break;
		}

		if(UNLIKELY(idx!=inst->devidx&&idx!=inst->loidx))continue;

		if(!(c->flags&RTSP_RTPONLY))
		{
			if(c->sadr.family)
			{
				if(UNLIKELY(ascmp(&c->sadr,&q->a[i])))continue;
			}
			else if(UNLIKELY(ascmp(&c->addr,&q->a[i])))continue;
		}

		port=ntohs(psget(&q->a[i]));
		if(UNLIKELY(port&1))continue;
		if(UNLIKELY(c->srtp&&c->srtp!=port))continue;
		if(UNLIKELY(q->vec[i].msg_len<12||(q->u[i].msg1[0]&0xc0)!=0x80||
			(q->u[i].msg1[1]&0x7f)!=33))continue;
		hlen=12+((q->u[i].msg1[0]&0xf)<<2);
		if(q->vec[i].msg_len<hlen)continue;
		if(c->ssrc&&ntohl(q->u[i].msg4[2])!=c->ssrc)continue;
		blen=q->vec[i].msg_len-hlen;
		if(UNLIKELY(blen&&blen!=1316))blen-=blen%188;
		dta[0].ptrval=q->u[i].msg1+hlen;
		dta[0].intval=blen|SATIP_RTP|((q->u[i].msg1[1]&0x80)?
			SATIP_MARKER:0);
		dta[1].intval=ntohl(q->u[i].msg4[1]);

		if(UNLIKELY(ntohs(q->u[i].msg2[1])!=c->seq++))
		{
			if(c->flags&RTSP_SEQ)dta[0].intval|=SATIP_LOSTPKT;
			c->seq=ntohs(q->u[i].msg2[1])+1;
		}
		c->flags|=RTSP_SEQ;

		c->cb(dta,c->priv);

		c->tmrcnt=0;
	}
}

static void rtcp(INSTANCE *inst,CONN *c)
{
	unsigned short port;
	int len;
	int base;
	int size;
	int idx=0;
	long ll;
	char *line;
	char *ptr;
	char *src;
	char *tuner;
	char *pids;
	union
	{
		char *mem;
		unsigned char *msg1;
		unsigned short *msg2;
		unsigned int *msg4;
	} u;
	struct cmsghdr *cm;
	struct in_pktinfo *p4;
	struct in6_pktinfo *p6;
	struct msghdr mh;
	struct iovec io;
	SOCK a;
	SATIP_DATA dta;
	SATIP_STATUS s;
	char cmsg[256];
	unsigned char msg[2048];

	mh.msg_name=&a;
	mh.msg_namelen=sizeof(a);
	mh.msg_iov=&io;
	mh.msg_iovlen=1;
	mh.msg_control=cmsg;
	mh.msg_controllen=sizeof(cmsg);
	mh.msg_flags=0;
	io.iov_base=msg;
	io.iov_len=sizeof(msg);

	if((len=recvmsg(c->rtcpfd,&mh,0))<12)
	{
		if(len==-1&&errno!=EINTR)
			if(__sync_bool_compare_and_swap(&c->rcfail,0,1))
				epoll_ctl(inst->efd,EPOLL_CTL_DEL,c->rtcpfd,
					NULL);
		return;
	}
	if(invalid_sock(&a,mh.msg_namelen))return;

	for(cm=CMSG_FIRSTHDR(&mh);cm;cm=CMSG_NXTHDR(&mh,cm))
		switch(cm->cmsg_level)
	{
	case IPPROTO_IP:
		if(sockfam(&a)!=AF_INET||cm->cmsg_type!=IP_PKTINFO)break;
		p4=(struct in_pktinfo *)CMSG_DATA(cm);
		idx=p4->ipi_ifindex;
		break;

	case IPPROTO_IPV6:
		if(sockfam(&a)!=AF_INET6||cm->cmsg_type!=IPV6_PKTINFO)break;
		p6=(struct in6_pktinfo *)CMSG_DATA(cm);
		idx=p6->ipi6_ifindex;
		break;
	}

	if(idx!=inst->devidx&&idx!=inst->loidx)return;

	if(!(c->flags&RTSP_RTPONLY))
	{
		if(c->sadr.family)
		{
			if(ascmp(&c->sadr,&a))return;
		}
		else if(ascmp(&c->addr,&a))return;
	}
	port=ntohs(psget(&a));
	if(!(port&1))return;
	if(c->srtcp&&c->srtcp!=port)return;

	for(base=0;;base+=size)
	{
		if(base+4>len)return;
		u.msg1=msg+base;
		if((u.msg1[0]&0xc0)!=0x80)return;
		size=(ntohs(u.msg2[1])+1)<<2;
		if(base+size>len)return;
		if(!base)
		{
			if(u.msg1[1]!=200)return;
			if(size<28)return;
			if(c->ssrc&&ntohl(u.msg4[1])!=c->ssrc)return;
		}
		else if(u.msg1[1]==204)break;
	}

	if(base+size==sizeof(msg))return;
	if(ntohl(u.msg4[2])!=0x53455331)return;
	idx=ntohs(u.msg2[7]);
	if(idx<=0||idx+16>size)return;

	line=u.msg1+16;
	line[idx]=0;

	if(strncmp(line,"ver=",4))return;
	for(line+=4;*line&&*line!=';';line++);
	if(!*line++)return;
	if(!(ptr=strtok_r(line,"; \t",&u.mem)))return;
	if(strncmp(ptr,"src=",4))src=NULL;
	else
	{
		src=ptr+4;
		if(!(ptr=strtok_r(NULL,"; \t",&u.mem)))return;
	}
	if(strncmp(ptr,"tuner=",6))return;
	tuner=ptr+6;
	if(!(pids=strtok_r(NULL,"; \t",&u.mem)))return;
	if(strncmp(pids,"pids=",5))return;
	pids+=5;

	memset(&s.tune,0,sizeof(SATIP_TUNE));
	s.tune.plp=SATIP_UNDEF;
	s.tune.t2id=SATIP_UNDEF;
	s.tune.ds=SATIP_UNDEF;

	if(src)
	{
		ll=strtol(src,&ptr,10);
		if(ll<1||ll>255||src==ptr||*ptr)return;
		s.tune.src=ll;
	}

	if(parse_tuner(tuner,&s))return;

	if(satip_util_parse(SATIP_PARSE_PID,0,0,pids,NULL,NULL,0,NULL,NULL,NULL,
		&s.set,NULL,NULL,NULL))return;

	dta.intval=SATIP_RTCP;
	dta.ptrval=&s;

	c->cb(&dta,c->priv);
}

static void ticker(INSTANCE *inst,CONN *c)
{
	struct epoll_event e;
	uint64_t dummy;

	dummy=read(c->tmrfd,&dummy,sizeof(dummy));

	if(c->netwake)
	{
		e.events=EPOLLIN|EPOLLONESHOT;
		e.data.ptr=(char *)c+sizeof(void *)+1;
		if(!epoll_ctl(inst->efd,EPOLL_CTL_MOD,c->rtpfd,&e))c->netwake=0;
	}

	if(inst->idleflush)switch(++(c->tmrcnt))
	{
	case 6:	c->cb(NULL,c->priv);
		break;
	case 30:c->tmrcnt=0;
		break;
	}
}

static void *streamer(void *data)
{
	INSTANCE *inst=(INSTANCE *)data;
	int i;
	int n;
	int notify=0;
	uint64_t dummy;
	struct epoll_event e[MAX_EPOLL];
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_BLOCK,&set,NULL);

	pthread_setname_np(pthread_self(),"client streamer");

	while(1)
	{
		if((n=epoll_wait(inst->efd,e,MAX_EPOLL,-1))<=0)continue;

		for(i=0;i<n;i++)if(e[i].events&EPOLLIN)
			switch(*((char *)e[i].data.ptr))
		{
		case 1: rtp(inst,(CONN *)
				((char *)e[i].data.ptr-sizeof(void *)-1));
			break;

		case 2: rtcp(inst,(CONN *)
				((char *)e[i].data.ptr-sizeof(void *)-2));
			break;

		case 3: http(inst,(CONN *)
				((char *)e[i].data.ptr-sizeof(void *)-3));
			break;

		case 4:	section(inst,(CONN *)
				((char *)e[i].data.ptr-sizeof(void *)-4));
			break;

		case 5:	sigwrk(inst,(CONN *)
				((char *)e[i].data.ptr-sizeof(void *)-5));
			break;

		case 6:	ticker(inst,(CONN *)
				((char *)e[i].data.ptr-sizeof(void *)-6));
			break;

		case 7:	dummy=read(inst->dfd[0],&dummy,sizeof(dummy));
			notify=1;
			if(inst->dfd[2]==-1)break;
			epoll_ctl(inst->efd,EPOLL_CTL_DEL,inst->dfd[2],NULL);
			break;

		case 0:	goto done;
		}

		if(notify)
		{
			notify=0;
			dummy=1;
			dummy=write(inst->dfd[1],&dummy,sizeof(dummy));
		}
	}

done:	pthread_exit(NULL);
}

static void *owork(void *data)
{
	int r;
	CONN *c=(CONN *)data;

	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS,NULL);
	pthread_mutex_lock(&c->mtx);
	r=rtsp_options(c);
	pthread_mutex_lock(&c->instance->stx);
	if(r)c->flags|=RTSP_DEAD;
	else c->idle=0;
	pthread_mutex_unlock(&c->instance->stx);
	pthread_spin_lock(&c->instance->ttx);
	c->runner=0;
	pthread_mutex_unlock(&c->mtx);
	pthread_spin_unlock(&c->instance->ttx);
	pthread_exit(NULL);
}

static void twork(INSTANCE *inst,int unused)
{
	int i;
	int l;
	CONN *c;
	UPNPDEVICE **n;
	UPNPDEVICE *e;
	uint64_t dummy;
	SOCK a;
	char bfr[256];
	char mcast[INET6_ADDRSTRLEN+2];

	struct cmsghdr *cm;
	struct in_pktinfo *i4;
	struct in6_pktinfo *i6;
	struct msghdr mh;
	struct iovec io;
	char cmsg[64];

	dummy=read(inst->ifd,&dummy,sizeof(dummy));

	pthread_mutex_lock(&inst->utx);
	for(i=0;i<=inst->level;i++)for(n=&inst->nlist[i];*n;)if(!--(*n)->age)
	{
		e=*n;
		*n=e->next;
		if(!--e->use)free(e);
	}
	else n=&(*n)->next;
	pthread_mutex_unlock(&inst->utx);

	switch(--inst->upnpticker)
	{
	case 19:inst->upnpanswer=1;
	case 18:
	case 17:for(i=0;i<=inst->level;i++)
		{
			addr2sock(&a,&mcaddr[i],htons(MPORT),inst->devidx);
			addr2bstr(&mcaddr[i],mcast,sizeof(mcast));
			l=snprintf(bfr,sizeof(bfr),"M-SEARCH * HTTP/1.1\r\n"
				"HOST %s:1900\r\nMAN: \"ssdp:discover\"\r\n"
				"MX: 2\r\nST: urn:ses-com:device:SatIPServer:1"
				"\r\nUSER-AGENT: Linux/1.0 UPnP/1.1 " IDENT
				"\r\n\r\n",mcast);

			mh.msg_name=&a;
			mh.msg_namelen=sizeof(SOCK);
			mh.msg_iov=&io;
			mh.msg_iovlen=1;
			mh.msg_control=cmsg;
			mh.msg_flags=0;
			io.iov_base=bfr;
			io.iov_len=l;

			if(i==SATIP_V4_ONLY)
			{
				mh.msg_controllen=
					CMSG_SPACE(sizeof(struct in_pktinfo));
				cm=CMSG_FIRSTHDR(&mh);
				cm->cmsg_level=IPPROTO_IP;
				cm->cmsg_type=IP_PKTINFO;
				cm->cmsg_len=
					CMSG_LEN(sizeof(struct in_pktinfo));
				i4=(struct in_pktinfo *)CMSG_DATA(cm);
				memset(i4,0,sizeof(struct in_pktinfo));
				i4->ipi_spec_dst=inst->devaddr[i].a4;
			}
			else
			{
				mh.msg_controllen=
					CMSG_SPACE(sizeof(struct in6_pktinfo));
				cm=CMSG_FIRSTHDR(&mh);
				cm->cmsg_level=IPPROTO_IPV6;
				cm->cmsg_type=IPV6_PKTINFO;
				cm->cmsg_len=
					CMSG_LEN(sizeof(struct in6_pktinfo));
				i6=(struct in6_pktinfo *)CMSG_DATA(cm);
				memset(i6,0,sizeof(struct in6_pktinfo));
				i6->ipi6_addr=inst->devaddr[i].a6;
			}

			sendmsg(inst->udp[i],&mh,0);
		}
		break;

	case 1:	inst->upnpanswer=0;
		pthread_mutex_lock(&inst->utx);
		for(i=0;i<=inst->level;i++)for(n=&inst->nlist[i];*n;)
			if(!--(*n)->noanswer)
		{
			e=*n;
			*n=e->next;
			if(!--e->use)free(e);
		}
		else n=&(*n)->next;
		pthread_mutex_unlock(&inst->utx);
		break;

	case 0:
	case -2:inst->upnpticker=inst->upnpinterval;
		break;
	}

	pthread_mutex_lock(&inst->stx);

	for(c=inst->clist;c;c=c->next)
	{
		c->idle++;
		if(!pthread_mutex_trylock(&c->mtx))
		{
			if((c->flags&RTSP_DEAD)||c->tmo==-1)c->idle=0;
			else if(c->idle>=c->tmo)
			{
				pthread_spin_lock(&inst->ttx);
				if(!c->runner)
					if(!pthread_create(&c->th,&inst->attr,
						owork,c))
				{
					pthread_detach(c->th);
					c->runner=1;
				}
				pthread_spin_unlock(&inst->ttx);
			}
			pthread_mutex_unlock(&c->mtx);
		}
	}

	pthread_mutex_unlock(&inst->stx);
}

static void upnpmgr(INSTANCE *inst,char *uuid,SOCK *a,char *loc,int mode,
	int age,int bid,int cid)
{
	int level;
	ADDR addr;
	UPNPDEVICE **n;
	UPNPDEVICE *e=NULL;

	sock2addr(&addr,a);
	level=addr2level(&addr);
	if(level>inst->level||invalid_mcast(&addr)!=-1)return;

	pthread_mutex_lock(&inst->utx);
	for(n=&inst->nlist[level];*n;n=&(*n)->next)if(!strcmp((*n)->uuid,uuid))
	{
		e=*n;
		if(!mode)
		{
			*n=e->next;
			if(!--e->use)free(e);
			goto out;
		}
		if(e->use>1||strlen(loc)>strlen((*n)->location))
		{
			*n=e->next;
			if(!--e->use)free(e);
			e=NULL;
		}
		break;
	}
	if(!mode)goto out;
	if(!e)
	{
		if(!(e=malloc(sizeof(UPNPDEVICE)+strlen(loc)+1)))return;
		strcpy(e->uuid,uuid);
		e->use=1;
		e->next=inst->nlist[level];
		inst->nlist[level]=e;
	}
	e->addr=addr;
	e->level=level;
	e->age=age<<3;
	e->bid=bid;
	e->cid=cid;
	strcpy(e->location,loc);
	e->noanswer=2;
out:	pthread_mutex_unlock(&inst->utx);
}

static void uwork(INSTANCE *inst,int level)
{
	int l;
	int age;
	int bid;
	int cid;
	int idx=0;
	char *ptr;
	char *uuid;
	char *loc;
	struct cmsghdr *cm;
	struct in_pktinfo *p4;
	struct in6_pktinfo *p6;
	struct msghdr mh;
	struct iovec io;
	SOCK a;
	MSGDATA m;
	char cmsg[256];
	char bfr[2048];

	mh.msg_name=&a;
	mh.msg_namelen=sizeof(a);
	mh.msg_iov=&io;
	mh.msg_iovlen=1;
	mh.msg_control=cmsg;
	mh.msg_controllen=sizeof(cmsg);
	mh.msg_flags=0;
	io.iov_base=bfr;
	io.iov_len=sizeof(bfr)-1;

	if((l=recvmsg(inst->udp[level],&mh,0))==-1)return;
	if(invalid_sock(&a,mh.msg_namelen))return;

	if(!inst->upnpanswer)return;

	for(cm=CMSG_FIRSTHDR(&mh);cm;cm=CMSG_NXTHDR(&mh,cm))
		switch(cm->cmsg_level)
	{
	case IPPROTO_IP:
		if(sockfam(&a)!=AF_INET||cm->cmsg_type!=IP_PKTINFO)break;
		p4=(struct in_pktinfo *)CMSG_DATA(cm);
		idx=p4->ipi_ifindex;
		break;

	case IPPROTO_IPV6:
		if(sockfam(&a)!=AF_INET6||cm->cmsg_type!=IPV6_PKTINFO)break;
		p6=(struct in6_pktinfo *)CMSG_DATA(cm);
		idx=p6->ipi6_ifindex;
		break;
	}

	if(idx!=inst->devidx&&idx!=inst->loidx)return;

	bfr[l]=0;
	if(msgsplit(bfr,&m))return;

	if(strcmp(m.hdr,"HTTP/1.1 200 OK"))return;

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"st"))break;
	if(l==m.total||!strstr(m.value[l],"device:SatIPServer:1"))return;

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"usn"))break;
	if(l==m.total||!strstr(m.value[l],"device:SatIPServer:1"))return;
	if((uuid=strstr(m.value[l],"uuid:")))uuid+=5;
	else return;
	for(ptr=uuid;*ptr&&*ptr!=':';ptr++);
	*ptr=0;
	if(strlen(uuid)!=SATIP_UUID_LEN)return;

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"location"))break;
	if(l==m.total||strncmp(m.value[l],"http://",7))return;
	loc=m.value[l];

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"cache-control"))break;
	if(l==m.total||!(ptr=strstr(m.value[l],"max-age=")))return;
	if(!(age=atoi(ptr+8)))return;
	if(age<=0)return;
	if(age>86400)age=86400;

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"bootid.upnp.org"))break;
	if(l==m.total)return;
	bid=atoi(m.value[l]);

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"configid.upnp.org"))break;
	if(l==m.total)return;
	cid=atoi(m.value[l]);

	upnpmgr(inst,uuid,&a,loc,1,age,bid,cid);
}

static void mwork(INSTANCE *inst,int level)
{
	int l;
	int idx=0;
	int mode;
	int age;
	int bid;
	int cid;
	char *ptr;
	char *uuid;
	char *loc;
	struct cmsghdr *cm;
	struct in_pktinfo *p4;
	struct in6_pktinfo *p6;
	struct msghdr mh;
	struct iovec io;
	SOCK a;
	MSGDATA m;
	char cmsg[256];
	char bfr[2048];

	mh.msg_name=&a;
	mh.msg_namelen=sizeof(a);
	mh.msg_iov=&io;
	mh.msg_iovlen=1;
	mh.msg_control=cmsg;
	mh.msg_controllen=sizeof(cmsg);
	mh.msg_flags=0;
	io.iov_base=bfr;
	io.iov_len=sizeof(bfr)-1;

	if((l=recvmsg(inst->mdp[level],&mh,0))==-1)return;
	if(invalid_sock(&a,mh.msg_namelen))return;

	for(cm=CMSG_FIRSTHDR(&mh);cm;cm=CMSG_NXTHDR(&mh,cm))
		switch(cm->cmsg_level)
	{
	case IPPROTO_IP:
		if(sockfam(&a)!=AF_INET||cm->cmsg_type!=IP_PKTINFO)break;
		p4=(struct in_pktinfo *)CMSG_DATA(cm);
		idx=p4->ipi_ifindex;
		break;

	case IPPROTO_IPV6:
		if(sockfam(&a)!=AF_INET6||cm->cmsg_type!=IPV6_PKTINFO)break;
		p6=(struct in6_pktinfo *)CMSG_DATA(cm);
		idx=p6->ipi6_ifindex;
		break;
	}

	if(idx!=inst->devidx&&idx!=inst->loidx)return;

	bfr[l]=0;
	if(msgsplit(bfr,&m))return;

	if(strcmp(m.hdr,"NOTIFY * HTTP/1.1"))return;

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"usn"))break;
	if(l==m.total)return;
	if((uuid=strstr(m.value[l],"uuid:")))uuid+=5;
	else return;
	for(ptr=uuid;*ptr&&*ptr!=':';ptr++);
	*ptr=0;
	if(strlen(uuid)!=SATIP_UUID_LEN)return;

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"nts"))break;
	if(l==m.total)return;
	if(!strcmp(m.value[l],"ssdp:byebye"))mode=0;
	else if(!strcmp(m.value[l],"ssdp:alive"))mode=1;
	else return;

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"bootid.upnp.org"))break;
	if(l==m.total)return;
	bid=atoi(m.value[l]);

	for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"configid.upnp.org"))break;
	if(l==m.total)return;
	cid=atoi(m.value[l]);

	if(mode)
	{
		for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"deviceid.ses.com"))
			break;
		if(l==m.total)return;

		for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"cache-control"))
			break;
		if(l==m.total||!(ptr=strstr(m.value[l],"max-age=")))return;
		if(!(age=atoi(ptr+8)))return;
		if(age<=0)return;
		if(age>86400)age=86400;

		for(l=0;l<m.total;l++)if(!strcmp(m.name[l],"location"))break;
		if(l==m.total||strncmp(m.value[l],"http://",7))return;
		loc=m.value[l];
	}
	else
	{
		age=0;
		loc=NULL;
	}

	upnpmgr(inst,uuid,&a,loc,mode,age,bid,cid);
}

static void *manager(void *data)
{
	int i;
	int n;
	INSTANCE *inst=(INSTANCE *)data;
	struct epoll_event e[MAX_EPOLL];
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_BLOCK,&set,NULL);

	pthread_setname_np(pthread_self(),"client manager");

	while(1)
	{
		if((n=epoll_wait(inst->xfd,e,MAX_EPOLL,-1))<=0)continue;

		for(i=0;i<n;i++)if(!(e[i].events&EPOLLIN))continue;
		else if(!e[i].data.ptr)goto done;
		else ((FUNC *)e[i].data.ptr)->func(inst,
			((FUNC *)e[i].data.ptr)->level);
	}

done:	pthread_exit(NULL);
}

static void epdel(INSTANCE *inst,int fd)
{
	uint64_t dummy=1;

	pthread_mutex_lock(&inst->etx);

	inst->dfd[2]=fd;

	dummy=write(inst->dfd[0],&dummy,sizeof(dummy));

	while(poll(inst->ep,2,-1)<1);

	if(!(inst->ep[0].revents&POLLIN))
		dummy=read(inst->dfd[1],&dummy,sizeof(dummy));

	pthread_mutex_unlock(&inst->etx);
}

int satip_cln_upnplist(void *handle,SATIP_CLN_UPNPLIST **list)
{
	int i;
	int j;
	int total[3];
	INSTANCE *inst=(INSTANCE *)handle;
	UPNPDEVICE *e;
	UPNPDEVICE *dev[3][MAXUPNP];
	SATIP_CLN_UPNPLIST *n;
	SATIP_CLN_UPNPLIST *x;
	SATIP_CLN_UPNPLIST *a=NULL;

	if(!handle||!list)return SATIP_SYSFAIL;

	memset(total,0,sizeof(total));

	pthread_mutex_lock(&inst->utx);
	for(i=0;i<=inst->level;i++)
		for(e=inst->nlist[i];e&&total[i]<MAXUPNP;total[i]++,e=e->next)
	{
		e->use++;
		dev[i][total[i]]=e;
	}
	pthread_mutex_unlock(&inst->utx);

	for(i=0;i<=inst->level;i++)for(j=0;j<total[i];j++)
	{
		if(!(n=malloc(sizeof(SATIP_CLN_UPNPLIST)+
			strlen(dev[i][j]->location)+1)))goto err;
		n->next=NULL;
		n->same=NULL;
		n->level=dev[i][j]->level;
		n->bootcount=dev[i][j]->bid;
		n->configid=dev[i][j]->cid;
		strcpy(n->uuid,dev[i][j]->uuid);
		addr2str(&dev[i][j]->addr,n->addr,sizeof(n->addr));
		strcpy(n->location,dev[i][j]->location);
		if(!i)x=NULL;
		else for(x=a;x;x=x->next)if(!strcmp(x->uuid,n->uuid))
		{
			while(x->same)x=x->same;
			n->same=x->same;
			x->same=n;
			break;
		}
		if(!x)
		{
			n->next=a;
			a=n;
		}
	}

	pthread_mutex_lock(&inst->utx);
	for(i=0;i<=inst->level;i++)for(j=0;j<total[i];j++)if(!--dev[i][j]->use)
		free(dev[i][j]);
	pthread_mutex_unlock(&inst->utx);

	*list=a;

	return 0;

err:	while(a)
	{
		while(a->same)
		{
			n=a->same;
			a->same=n->same;
			free(n);
		}
		n=a;
		a=n->next;
		free(n);
	}

	return SATIP_SYSFAIL;
}

int satip_cln_freelist(void *handle,SATIP_CLN_UPNPLIST *list)
{
	SATIP_CLN_UPNPLIST *n;

	if(!handle||!list)return SATIP_SYSFAIL;

	while(list)
	{
		while(list->same)
		{
			n=list->same;
			list->same=n->same;
			free(n);
		}
		n=list;
		list=n->next;
		free(n);
	}

	return 0;
}

int satip_cln_upnpinfo(void *handle,SATIP_CLN_UPNPLIST *entry,
	SATIP_CLN_UPNPINFO **info)
{
	int i;
	int len;
	int tot;
	int l1;
	int l2;
	int icon=0;
	int w=0;
	int h=0;
	int devices[SATIP_TOTALS]={0,0,0,0,0};
	INSTANCE *inst=(INSTANCE *)handle;
	char *data;
	char *ptr;
	char *mem;
	char *hptr=NULL;
	char *mime=NULL;
	char *name=NULL;
	char *pres=NULL;
	char *devs=NULL;
	char *list=NULL;
	char *imgs[4]={NULL,NULL,NULL,NULL};
	char *manuname=NULL;
	char *manuurl=NULL;
	char *mname=NULL;
	char *mnum=NULL;
	char *murl=NULL;
	char *desc=NULL;
	char *serial=NULL;
	char *upc=NULL;
	SATIP_CLN_UPNPINFO *e;
	char cset[SATIP_CSET_LEN+1];

	if(!handle||!entry||!info)goto err1;

	strcpy(cset,inst->cset);

	if(!(data=fetchurl(inst,entry->location,&len,cset,sizeof(cset))))
		goto err1;

	for(ptr=strtok_r(data,"\r\n",&mem);ptr;ptr=strtok_r(NULL,"\r\n",&mem))
	{
		while(*ptr==' '||*ptr=='\t')ptr++;
		if(!strncasecmp(ptr,"<?xml version=",14))
		{
			if((hptr=strstr(ptr,"encoding=")))
			{
				hptr+=9;
				if(*hptr=='"')hptr++;
				for(ptr=hptr;*ptr;ptr++)
				    if((*ptr>='a'&&*ptr<='z')||
					(*ptr>='A'&&*ptr<='Z')||*ptr=='-'||
					(*ptr>='0'&&*ptr<='9')||*ptr=='_'||
					*ptr==':'||*ptr=='.')continue;
				else break;
				*ptr=0;
				for(ptr=hptr;*ptr;ptr++)if(*ptr>='a'&&*ptr<='z')
					*ptr&=0xdf;
				strncpy(cset,ptr,sizeof(cset)-1);
				cset[sizeof(cset)-1]=0;
			}
		}
		else if(!strncasecmp(ptr,"<root",5))
		{
			if(!(hptr=strstr(ptr,"configId=")))goto err2;
			hptr+=9;
			while(*hptr=='"'||*hptr=='\'')hptr++;
			if(atoi(hptr)!=entry->configid)goto err2;
		}
		else if(!strncasecmp(ptr,"<deviceType>",12))
		{
			if(!strstr(ptr+12,"device:SatIPServer:1"))goto err2;
		}
		else if(!strncasecmp(ptr,"<UDN>uuid:",10))
		{
			hptr=ptr+10;
			for(ptr=hptr;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(strlen(hptr)!=SATIP_UUID_LEN)goto err2;
			if(memcmp(entry->uuid,hptr,SATIP_UUID_LEN))goto err2;
		}
		else if(!strncasecmp(ptr,"<manufacturer>",14))
		{
			manuname=ptr+14;
			for(ptr=manuname;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*manuname)manuname=NULL;
		}
		else if(!strncasecmp(ptr,"<manufacturerURL>",17))
		{
			manuurl=ptr+17;
			for(ptr=manuurl;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*manuurl)manuurl=NULL;
		}
		else if(!strncasecmp(ptr,"<modelName>",11))
		{
			mname=ptr+11;
			for(ptr=mname;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*mname)mname=NULL;
		}
		else if(!strncasecmp(ptr,"<modelNumber>",13))
		{
			mnum=ptr+13;
			for(ptr=mnum;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*mnum)mnum=NULL;
		}
		else if(!strncasecmp(ptr,"<modelURL>",10))
		{
			murl=ptr+10;
			for(ptr=murl;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*murl)murl=NULL;
		}
		else if(!strncasecmp(ptr,"<modelDescription>",18))
		{
			desc=ptr+18;
			for(ptr=desc;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*desc)desc=NULL;
		}
		else if(!strncasecmp(ptr,"<serialNumber>",14))
		{
			serial=ptr+14;
			for(ptr=serial;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*serial)serial=NULL;
		}
		else if(!strncasecmp(ptr,"<UPC>",5))
		{
			upc=ptr+5;
			for(ptr=upc;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*upc)upc=NULL;
		}
		else if(!strncasecmp(ptr,"<friendlyName>",14))
		{
			name=ptr+14;
			for(ptr=name;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*name)name=NULL;
		}
		else if(!strncasecmp(ptr,"<presentationURL>",17))
		{
			pres=ptr+17;
			for(ptr=pres;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*pres)pres=NULL;
		}
		else if(!strncasecmp(ptr,"<satip:X_SATIPCAP xmlns:satip=\""
			"urn:ses-com:satip\">",50))
		{
			devs=ptr+50;
			for(ptr=devs;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*devs)devs=NULL;
		}
		else if(!strncasecmp(ptr,"<satip:X_SATIPM3U xmlns:satip=\""
			"urn:ses-com:satip\">",50))
		{
			list=ptr+50;
			for(ptr=list;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*list)list=NULL;
		}
		else if(!strcasecmp(ptr,"<icon>"))
		{
			if(icon)goto err2;
			icon=1;
			w=0;
			h=0;
			mime=NULL;
			hptr=NULL;
		}
		else if(!strcasecmp(ptr,"</icon>"))
		{
			if(!icon)goto err2;
			icon=0;
			if(w&&h&&mime&&hptr)
			{
				if(!strcasecmp(mime,"image/png"))
				{
					if(w==120&&h==120)imgs[1]=hptr;
					else if(w==48&&h==48)imgs[0]=hptr;
				}
				else if(!strcasecmp(mime,"image/jpeg"))
				{
					if(w==120&&h==120)imgs[3]=hptr;
					else if(w==48&&h==48)imgs[2]=hptr;
				}
			}
		}
		else if(!strncasecmp(ptr,"<width>",7))
		{
			if(!icon)goto err2;
			w=atoi(ptr+7);
		}
		else if(!strncasecmp(ptr,"<height>",8))
		{
			if(!icon)goto err2;
			h=atoi(ptr+8);
		}
		else if(!strncasecmp(ptr,"<mimetype>",10))
		{
			if(!icon)goto err2;
			mime=ptr+10;
			for(ptr=mime;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*mime)mime=NULL;
		}
		else if(!strncasecmp(ptr,"<url>",5))
		{
			if(!icon)goto err2;
			hptr=ptr+5;
			for(ptr=hptr;*ptr&&*ptr!='<';ptr++);
			if(!*ptr)goto err2;
			*ptr=0;
			if(!*hptr)hptr=NULL;
		}
	}

	if(devs)for(ptr=strtok_r(devs,", \t",&mem);ptr;
		ptr=strtok_r(NULL,", \t",&mem))
	{
		if(!strncasecmp(ptr,"DVBS2-",6))
		{
			if(ptr[6]<'0'||ptr[6]>'9'||ptr[7])goto err2;
			devices[SATIP_TOT_DVBS2]=ptr[6]-'0';
		}
		else if(!strncasecmp(ptr,"DVBT-",5))
		{
			if(ptr[5]<'0'||ptr[5]>'9'||ptr[6])goto err2;
			devices[SATIP_TOT_DVBT]=ptr[5]-'0';
		}
		else if(!strncasecmp(ptr,"DVBT2-",6))
		{
			if(ptr[6]<'0'||ptr[6]>'9'||ptr[7])goto err2;
			devices[SATIP_TOT_DVBT2]=ptr[6]-'0';
		}
		else if(!strncasecmp(ptr,"DVBC-",5))
		{
			if(ptr[5]<'0'||ptr[5]>'9'||ptr[6])goto err2;
			devices[SATIP_TOT_DVBC]=ptr[5]-'0';
		}
		else if(!strncasecmp(ptr,"DVBC2-",6))
		{
			if(ptr[6]<'0'||ptr[6]>'9'||ptr[7])goto err2;
			devices[SATIP_TOT_DVBC2]=ptr[6]-'0';
		}
	}

	if(!(ptr=strchr(entry->location+7,'/')))goto err2;
	l1=ptr-entry->location;
	if(!(ptr=strrchr(entry->location+7,'/')))goto err2;
	l2=&ptr[1]-entry->location;

	if(!name)name="";
	if(!pres)pres="";
	if(!list)list="";
	if(!imgs[0])imgs[0]="";
	if(!imgs[1])imgs[1]="";
	if(!imgs[2])imgs[2]="";
	if(!imgs[3])imgs[3]="";
	if(!manuname)manuname="";
	if(!manuurl)manuurl="";
	if(!mname)mname="";
	if(!mnum)mnum="";
	if(!murl)murl="";
	if(!desc)desc="";
	if(!serial)serial="";
	if(!upc)upc="";

	len=str2utf8(name,cset,NULL,0)+1;
	len+=str2utf8(pres,cset,NULL,0)+1;
	if(*pres=='/')len+=l1;
	else if(*pres&&strncmp(pres,"http://",7))len+=l2;
	len+=str2utf8(list,cset,NULL,0)+1;
	if(*list=='/')len+=l1;
	else if(*list&&strncmp(list,"http://",7))len+=l2;
	for(i=0;i<4;i++)
	{
		len+=str2utf8(imgs[i],cset,NULL,0)+1;
		if(*imgs[i]=='/')len+=l1;
		else if(*imgs[i]&&strncmp(imgs[i],"http://",7))len+=l2;
	}
	len+=str2utf8(manuname,cset,NULL,0)+1;
	len+=str2utf8(manuurl,cset,NULL,0)+1;
	if(*manuurl=='/')len+=l1;
	else if(*manuurl&&strncmp(manuurl,"http://",7)&&
		strncmp(manuurl,"https://",8))len+=l2;
	len+=str2utf8(mname,cset,NULL,0)+1;
	len+=str2utf8(mnum,cset,NULL,0)+1;
	len+=str2utf8(murl,cset,NULL,0)+1;
	if(*murl=='/')len+=l1;
	else if(*murl&&strncmp(murl,"http://",7)&&
		strncmp(murl,"https://",8))len+=l2;
	len+=str2utf8(desc,cset,NULL,0)+1;
	len+=str2utf8(serial,cset,NULL,0)+1;
	len+=str2utf8(upc,cset,NULL,0)+1;

	if(!(e=malloc(sizeof(SATIP_CLN_UPNPINFO)+len)))goto err2;

	for(tot=len,i=0;i<SATIP_TOTALS;i++)e->totals[i]=devices[i];

	e->friendlyname=e->data;
	len=str2utf8(name,cset,e->data,tot)+1;

	e->presentationurl=e->data+len;
	if(*pres=='/')
	{
		memcpy(e->data+len,entry->location,l1);
		len+=l1;
	}
	else if(*pres&&strncmp(pres,"http://",7))
	{
		memcpy(e->data+len,entry->location,l2);
		len+=l2;
	}
	len+=str2utf8(pres,cset,e->data+len,tot-len)+1;

	e->playlisturl=e->data+len;
	if(*list=='/')
	{
		memcpy(e->data+len,entry->location,l1);
		len+=l1;
	}
	else if(*list&&strncmp(list,"http://",7))
	{
		memcpy(e->data+len,entry->location,l2);
		len+=l2;
	}
	len+=str2utf8(list,cset,e->data+len,tot-len)+1;

	for(i=0;i<4;i++)
	{
		switch(i)
		{
		case 0:	e->png48url=e->data+len;
			break;
		case 1:	e->png120url=e->data+len;
			break;
		case 2:	e->jpg48url=e->data+len;
			break;
		case 3:	e->jpg120url=e->data+len;
			break;
		}

		if(*imgs[i]=='/')
		{
			memcpy(e->data+len,entry->location,l1);
			len+=l1;
		}
		else if(*imgs[i]&&strncmp(imgs[i],"http://",7))
		{
			memcpy(e->data+len,entry->location,l2);
			len+=l2;
		}
		len+=str2utf8(imgs[i],cset,e->data+len,tot-len)+1;
	}

	e->manufacturername=e->data+len;
	len+=str2utf8(manuname,cset,e->data+len,tot-len)+1;

	e->manufacturerurl=e->data+len;
	if(*manuurl=='/')
	{
		memcpy(e->data+len,entry->location,l1);
		len+=l1;
	}
	else if(*manuurl&&strncmp(manuurl,"http://",7)&&
		strncmp(manuurl,"https://",8))
	{
		memcpy(e->data+len,entry->location,l2);
		len+=l2;
	}
	len+=str2utf8(manuurl,cset,e->data+len,tot-len)+1;

	e->modelname=e->data+len;
	len+=str2utf8(mname,cset,e->data+len,tot-len)+1;
	e->modelnumber=e->data+len;
	len+=str2utf8(mnum,cset,e->data+len,tot-len)+1;

	e->modelurl=e->data+len;
	if(*murl=='/')
	{
		memcpy(e->data+len,entry->location,l1);
		len+=l1;
	}
	else if(*murl&&strncmp(murl,"http://",7)&&strncmp(murl,"https://",8))
	{
		memcpy(e->data+len,entry->location,l2);
		len+=l2;
	}
	len+=str2utf8(murl,cset,e->data+len,tot-len)+1;

	e->modeldescription=e->data+len;
	len+=str2utf8(desc,cset,e->data+len,tot-len)+1;
	e->serial=e->data+len;
	len+=str2utf8(serial,cset,e->data+len,tot-len)+1;
	e->upc=e->data+len;
	len+=str2utf8(upc,cset,e->data+len,tot-len)+1;

	*info=e;
	free(data);
	return 0;

err2:	free(data);
err1:	return SATIP_SYSFAIL;
}

int satip_cln_freeinfo(void *handle,SATIP_CLN_UPNPINFO *info)
{
	if(!handle||!info)return SATIP_SYSFAIL;

	free(info);
	return 0;
}

int satip_cln_upnpitem(void *handle,char *url,void **data,int *size)
{
	int s;
	int l;
	void *d;
	void *n;
	INSTANCE *inst=(INSTANCE *)handle;
	char cset[SATIP_CSET_LEN+1];

	if(!handle||!url||!data||!size)return SATIP_SYSFAIL;

	if(!(d=fetchurl(inst,url,&s,cset,sizeof(cset))))return SATIP_SYSFAIL;

	if(*cset&&strlen(d)==s)
	{
		l=str2utf8(d,cset,NULL,0);
		if((n=malloc(l+1)))
		{
			if(str2utf8(d,cset,n,l+1)==-1)free(n);
			else
			{
				free(d);
				*data=n;
				*size=l;
				return 0;
			}
		}
	}

	*data=d;
	*size=s;
	return 0;
}

int satip_cln_freeitem(void *handle,void *data)
{
	if(!handle||!data)return SATIP_SYSFAIL;

	free(data);
	return 0;
}

int satip_cln_stream_http(void *handle,char *server,int port,SATIP_TUNE *tune,
	SATIP_PIDS *set,SATIP_EXTRA *extra,
	void (*cb)(SATIP_DATA *data,void *priv),void *priv,void **stream)
{
	int i;
	int l;
	int r=SATIP_SYSFAIL;
	INSTANCE *inst=(INSTANCE *)handle;
	CONN *c;
	CONN **d;
	struct epoll_event e;
	ADDR addr;

	if(!handle||!server||!tune||!set||!cb||!stream||port<1||port>65535)
		goto err1;

	if(resolve(server,&addr,inst->level))goto err1;

	l=((void *)c->bfr-(void *)c)+
		(set->numpids==SATIP_SECTION?SECTIONBUFF:TCPBUFF);
	if(!(c=malloc(l)))goto err1;
	memset(c,0,l);
	memcpy(c->epid,"\x00\x01\x02\x03\x04\x05\x06\x07",8);

	c->addr=addr;
	c->port=port;
	c->cb=cb;
	c->priv=priv;
	c->httpfd=-1;
	c->instance=inst;

	if(invalid_mcast(&c->addr)!=-1)goto err1;

	pthread_mutex_lock(&inst->stx);
	c->next=inst->hlist;
	inst->hlist=c;
	pthread_mutex_unlock(&inst->stx);

	if((r=http_setup(c,tune,set,extra)))goto err2;

	r=SATIP_SYSFAIL;

	e.events=EPOLLIN;
	switch(set->numpids)
	{
	case SATIP_SIGNAL:
		i=5;
		break;
	case SATIP_SECTION:
		i=4;
		break;
	default:i=3;
		break;
	}
	e.data.ptr=(char *)c+sizeof(void *)+i;
	if(epoll_ctl(inst->efd,EPOLL_CTL_ADD,c->httpfd,&e))goto err3;

	*stream=c;

	return 0;

err3:	if(c->httpfd!=-1)close(c->httpfd);
err2:	pthread_mutex_lock(&inst->stx);
	for(d=&inst->hlist;*d;d=&(*d)->next)if(*d==c)
	{
		*d=c->next;
		break;
	}
	pthread_mutex_unlock(&inst->stx);
	free(c);
err1:	return r;
}

int satip_cln_stream_multicast(void *handle,char *address,int port,
	void (*cb)(SATIP_DATA *data,void *priv),void *priv,void **stream)
{
	int i;
	INSTANCE *inst=(INSTANCE *)handle;
	CONN *c;
	CONN **d;
	struct epoll_event e;
	ADDR addr;
	struct itimerspec it;

	if(!handle||!address||!cb||!stream||port<1||port>65535||(port&1))
		goto err1;

	if(resolve(address,&addr,inst->level))goto err1;

	if(!(c=malloc(sizeof(CONN)+sizeof(QUEUE))))goto err1;
	memset(c,0,sizeof(CONN)+sizeof(QUEUE));
	memcpy(c->epid,"\x00\x01\x02\x03\x04\x05\x06\x07",8);

	c->addr=addr;
	c->rtp=port;
	c->rtcp=port+1;
	c->tmo=-1;
	c->cb=cb;
	c->priv=priv;
	c->rtspfd=-1;
	c->flags=RTSP_RTPONLY;
	c->instance=inst;

	for(i=0;i<SATIP_MAX_BURST;i++)
	{
		c->queue[0].vec[i].msg_hdr.msg_name=&c->queue[0].a[i];
		c->queue[0].vec[i].msg_hdr.msg_namelen=sizeof(SOCK);
		c->queue[0].vec[i].msg_hdr.msg_iov=&c->queue[0].io[i];
		c->queue[0].vec[i].msg_hdr.msg_iovlen=1;
		c->queue[0].vec[i].msg_hdr.msg_control=c->queue[0].cmsg[i];
		c->queue[0].vec[i].msg_hdr.msg_controllen=
			sizeof(c->queue[0].cmsg[0]);
		c->queue[0].vec[i].msg_hdr.msg_flags=0;
		c->queue[0].io[i].iov_base=c->queue[0].u[i].msg1;
		c->queue[0].io[i].iov_len=sizeof(c->queue[0].u[0].msg1);
	}

	if(pthread_mutex_init(&c->mtx,NULL))goto err2;

	if(invalid_mcast(&c->addr)==-1)goto err3;

	if((c->rtpfd=mcast(inst,inst->mttl,&c->addr,c->rtp,0,inst->rtpbuffer))
		==-1)goto err3;

	if((c->rtcpfd=mcast(inst,inst->mttl,&c->addr,c->rtcp,0,0))==-1)
	{
		close(c->rtpfd);
		goto err3;
	}

	if((c->tmrfd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))
		==-1)
	{
		close(c->rtpfd);
		close(c->rtcpfd);
		goto err3;
	}

	pthread_mutex_lock(&c->mtx);

	pthread_mutex_lock(&inst->stx);
	c->next=inst->clist;
	inst->clist=c;
	pthread_mutex_unlock(&inst->stx);

	e.events=EPOLLIN|EPOLLONESHOT;
	e.data.ptr=(char *)c+sizeof(void *)+1;
	if(epoll_ctl(inst->efd,EPOLL_CTL_ADD,c->rtpfd,&e))goto err4;

	e.events=EPOLLIN;
	e.data.ptr=(char *)c+sizeof(void *)+2;
	if(epoll_ctl(inst->efd,EPOLL_CTL_ADD,c->rtcpfd,&e))goto err5;

	e.events=EPOLLIN;
	e.data.ptr=(char *)c+sizeof(void *)+6;
	if(epoll_ctl(inst->efd,EPOLL_CTL_ADD,c->tmrfd,&e))goto err6;

	it.it_interval.tv_sec=0;
	it.it_interval.tv_nsec=3333333;
	it.it_value.tv_sec=0;
	it.it_value.tv_nsec=3333333;

	if(timerfd_settime(c->tmrfd,0,&it,NULL))goto err7;

	*stream=c;

	pthread_mutex_unlock(&c->mtx);

	return 0;

err7:	epdel(inst,c->tmrfd);
err6:	if(__sync_bool_compare_and_swap(&c->rcfail,0,1))epdel(inst,c->rtcpfd);
err5:	if(__sync_bool_compare_and_swap(&c->rtfail,0,1))epdel(inst,c->rtpfd);
	else epdel(inst,-1);
err4:	close(c->tmrfd);
	pthread_mutex_lock(&inst->stx);
	for(d=&inst->clist;*d;d=&(*d)->next)if(*d==c)
	{
		*d=c->next;
		break;
	}
	pthread_mutex_unlock(&inst->stx);
	pthread_spin_lock(&inst->ttx);
	if(c->runner)pthread_cancel(c->th);
	pthread_spin_unlock(&inst->ttx);
	close(c->rtpfd);
	close(c->rtcpfd);
	pthread_mutex_unlock(&c->mtx);
err3:	pthread_mutex_destroy(&c->mtx);
err2:	free(c);
err1:	return SATIP_SYSFAIL;
}

int satip_cln_stream_unicast(void *handle,char *server,int port,int persist,
	SATIP_TUNE *tune,SATIP_PIDS *set,SATIP_EXTRA *extra,
	void (*cb)(SATIP_DATA *data,void *priv),void *priv,void **stream)
{
	int i;
	int r=SATIP_SYSFAIL;
	unsigned short base;
	INSTANCE *inst=(INSTANCE *)handle;
	CONN *c;
	CONN **d;
	struct epoll_event e;
	ADDR addr;
	ADDR a;
	struct itimerspec it;

	if(!handle||!server||!tune||!set||!cb||!stream||port<1||port>65535||
		set->numpids==SATIP_SECTION||set->numpids==SATIP_SIGNAL)
			goto err1;

	if(resolve(server,&addr,inst->level))goto err1;

	if(!(c=malloc(sizeof(CONN)+sizeof(QUEUE))))goto err1;
	memset(c,0,sizeof(CONN)+sizeof(QUEUE));
	memcpy(c->epid,"\x00\x01\x02\x03\x04\x05\x06\x07",8);

	c->addr=addr;
	c->port=port;
	c->flags=persist?RTSP_PERSIST:0;
	c->tmo=240;
	c->sid=-1;
	c->cb=cb;
	c->priv=priv;
	c->rtspfd=-1;
	c->instance=inst;
	c->tune=*tune;

	for(i=0;i<SATIP_MAX_BURST;i++)
	{
		c->queue[0].vec[i].msg_hdr.msg_name=&c->queue[0].a[i];
		c->queue[0].vec[i].msg_hdr.msg_namelen=sizeof(SOCK);
		c->queue[0].vec[i].msg_hdr.msg_iov=&c->queue[0].io[i];
		c->queue[0].vec[i].msg_hdr.msg_iovlen=1;
		c->queue[0].vec[i].msg_hdr.msg_control=c->queue[0].cmsg[i];
		c->queue[0].vec[i].msg_hdr.msg_controllen=
			sizeof(c->queue[0].cmsg[0]);
		c->queue[0].vec[i].msg_hdr.msg_flags=0;
		c->queue[0].io[i].iov_base=c->queue[0].u[i].msg1;
		c->queue[0].io[i].iov_len=sizeof(c->queue[0].u[0].msg1);
	}

	if(pthread_mutex_init(&c->mtx,NULL))goto err2;

	if(invalid_mcast(&c->addr)!=-1)goto err3;

	memset(&a,0,sizeof(a));
	a.family=addr.family;

	for(i=0;i<128;i++)
	{
		if(satip_util_random((unsigned char *)&base,sizeof(base)))
			continue;
		if(inst->portmin)
		{
			base%=inst->portmax-inst->portmin;
			base&=0xfffe;
			base+=inst->portmin;
		}
		else
		{
			base%=0xfc00;
			base&=0xfffe;
			base+=0x0400;
		}
		c->rtp=base;
		c->rtcp=base+1;
		if((c->rtpfd=udp(&a,htons(c->rtp),inst->devidx,inst->rtpbuffer))
			==-1)continue;
		if((c->rtcpfd=udp(&a,htons(c->rtcp),inst->devidx,0))!=-1)break;
		close(c->rtpfd);
	}
	if(i==128)goto err3;

	if((r=rtsp_setup_unicast(c,tune,set,extra)))goto err4;

	if((c->tmrfd=timerfd_create(CLOCK_MONOTONIC,TFD_NONBLOCK|TFD_CLOEXEC))
		==-1)goto err4;

	pthread_mutex_lock(&c->mtx);

	pthread_mutex_lock(&inst->stx);
	c->next=inst->clist;
	inst->clist=c;
	pthread_mutex_unlock(&inst->stx);

	r=SATIP_SYSFAIL;

	e.events=EPOLLIN|EPOLLONESHOT;
	e.data.ptr=(char *)c+sizeof(void *)+1;
	if(epoll_ctl(inst->efd,EPOLL_CTL_ADD,c->rtpfd,&e))goto err5;

	e.events=EPOLLIN;
	e.data.ptr=(char *)c+sizeof(void *)+2;
	if(epoll_ctl(inst->efd,EPOLL_CTL_ADD,c->rtcpfd,&e))goto err6;

	e.events=EPOLLIN;
	e.data.ptr=(char *)c+sizeof(void *)+6;
	if(epoll_ctl(inst->efd,EPOLL_CTL_ADD,c->tmrfd,&e))goto err7;

	it.it_interval.tv_sec=0;
	it.it_interval.tv_nsec=3333333;
	it.it_value.tv_sec=0;
	it.it_value.tv_nsec=3333333;

	if(timerfd_settime(c->tmrfd,0,&it,NULL))goto err8;

	if((r=rtsp_play(c,NULL,NULL,NULL,NULL)))goto err8;

	*stream=c;

	pthread_mutex_unlock(&c->mtx);

	return 0;

err8:	epdel(inst,c->tmrfd);
err7:	if(__sync_bool_compare_and_swap(&c->rcfail,0,1))epdel(inst,c->rtcpfd);
err6:	if(__sync_bool_compare_and_swap(&c->rtfail,0,1))epdel(inst,c->rtpfd);
	else epdel(inst,-1);
err5:	close(c->tmrfd);
	pthread_mutex_lock(&inst->stx);
	for(d=&inst->clist;*d;d=&(*d)->next)if(*d==c)
	{
		*d=c->next;
		break;
	}
	pthread_mutex_unlock(&inst->stx);
	pthread_spin_lock(&inst->ttx);
	if(c->runner)pthread_cancel(c->th);
	pthread_spin_unlock(&inst->ttx);
	rtsp_teardown(c);
	pthread_mutex_unlock(&c->mtx);
err4:	if(c->rtspfd!=-1)close(c->rtspfd);
	close(c->rtpfd);
	close(c->rtcpfd);
err3:	pthread_mutex_destroy(&c->mtx);
err2:	free(c);
err1:	return r;
}

int satip_cln_change_unicast(void *handle,void *stream,SATIP_PIDS *set,
	SATIP_PIDS *add,SATIP_PIDS *del,SATIP_EXTRA *extra)
{
	int r=SATIP_SYSFAIL;
	INSTANCE *inst=(INSTANCE *)handle;
	CONN *c=(CONN *)stream;
	CONN *d;

	if(!handle||!stream)return r;

	pthread_mutex_lock(&inst->stx);

	for(d=inst->clist;d;d=d->next)if(d==c)
	{
		r=0;
		break;
	}
	pthread_mutex_unlock(&inst->stx);

	pthread_mutex_lock(&c->mtx);

	if(!r)r=rtsp_play(c,set,add,del,extra);

	pthread_mutex_unlock(&c->mtx);

	return r;
}

int satip_cln_stream_stop(void *handle,void *stream)
{
	int r=SATIP_SYSFAIL;
	INSTANCE *inst=(INSTANCE *)handle;
	CONN *c=(CONN *)stream;
	CONN **d;

	if(!handle||!stream)return r;

	pthread_mutex_lock(&inst->stx);

	for(d=&inst->clist;*d;d=&(*d)->next)if(*d==c)
	{
		pthread_mutex_lock(&c->mtx);
		*d=c->next;
		r=0;
		break;
	}
	if(r)goto http;

	pthread_mutex_unlock(&inst->stx);

	epdel(inst,c->tmrfd);
	if(__sync_bool_compare_and_swap(&c->rtfail,0,1))epdel(inst,c->rtpfd);
	if(__sync_bool_compare_and_swap(&c->rcfail,0,1))epdel(inst,c->rtcpfd);
	else epdel(inst,-1);

	pthread_spin_lock(&inst->ttx);
	if(c->runner)pthread_cancel(c->th);
	pthread_spin_unlock(&inst->ttx);

	if(!(c->flags&RTSP_RTPONLY))
	{
		if(!(c->flags&RTSP_DEAD))r=rtsp_teardown(c);
		else r=404;
	}

	close(c->tmrfd);
	close(c->rtpfd);
	close(c->rtcpfd);
	if(c->rtspfd!=-1)close(c->rtspfd);
	pthread_mutex_unlock(&c->mtx);
	pthread_mutex_destroy(&c->mtx);
	free(c);

	return r;

http:	for(d=&inst->hlist;*d;d=&(*d)->next)if(*d==c)
	{
		*d=c->next;
		r=0;
		break;
	}
	if(r)
	{
		pthread_mutex_unlock(&inst->stx);
		return r;
	}

	pthread_mutex_unlock(&inst->stx);

	if(__sync_bool_compare_and_swap(&c->htfail,0,1))epdel(inst,c->httpfd);
	else
	{
		epdel(inst,-1);
		r=404;
	}

	close(c->httpfd);
	free(c);

	return r;
}

int satip_cln_streaminfo(void *handle,char *server,int port,
	SATIP_CLN_STREAMINFO **info)
{
	int r=SATIP_SYSFAIL;
	INSTANCE *inst=(INSTANCE *)handle;
	SATIP_DATA data;
	CONN c;

	if(!handle||!server||!info||port<1||port>65535)goto err1;

	memset(&c,0,sizeof(CONN));
	c.cseq=1;
	c.port=port;
	c.instance=inst;

	if(resolve(server,&c.addr,inst->level))goto err1;

	switch((r=rtsp_describe(&c,&data)))
	{
	case -1:r=SATIP_SYSFAIL;
		goto err1;
	case 0:	if((*info=parse_streaminfo(data.ptrval)))break;
		r=SATIP_SYSFAIL;
	default:goto err2;
	case 404:*info=NULL;
	}

	free(data.ptrval);

	return 0;

err2:	free(data.ptrval);
err1:	return r;
}

int satip_cln_streaminfo_free(SATIP_CLN_STREAMINFO *info)
{
	SATIP_CLN_STREAMINFO *e;

	if(!info)return SATIP_SYSFAIL;

	while(info)
	{
		e=info;
		info=e->next;
		free(e);
	}

	return 0;
}
 
int satip_cln_setup_multicast(void *handle,char *server,int port,char *maddr,
	int mport,int mttl,int play,char *sessionid,int size,int *streamid,
	SATIP_TUNE *tune,SATIP_PIDS *set,SATIP_EXTRA *extra)
{
	int r=SATIP_SYSFAIL;
	INSTANCE *inst=(INSTANCE *)handle;
	CONN c;

	if(!handle||!server||!maddr||!tune||!set||!sessionid||!streamid||
		size<1||port<1||port>65535||mport<1||mport>65535||(mport&1)||
		mttl<1||mttl>255||set->numpids==SATIP_SECTION||
		set->numpids==SATIP_SIGNAL)goto err1;

	memset(&c,0,sizeof(CONN));
	c.rtspfd=-1;
	c.flags=play?RTSP_PERSIST:0;
	c.sid=-1;
	c.instance=inst;
	c.port=port;
	c.mport=mport;
	c.mttl=mttl;

	if(resolve(server,&c.addr,inst->level))goto err1;

	if(invalid_mcast(&c.addr)!=-1)goto err1;

	if(resolve(maddr,&c.maddr,inst->level))goto err1;

	if(invalid_mcast(&c.maddr))goto err1;

	if((r=rtsp_setup_multicast(&c,tune,set,extra)))goto err2;

	if(c.tmo!=-1)
	{
		r=SATIP_SYSFAIL;
		goto err3;
	}

	if(play)
	{
		c.flags=0;
		if((r=rtsp_play(&c,NULL,NULL,NULL,NULL)))goto err3;
	}

	if(c.rtspfd!=-1)close(c.rtspfd);

	strncpy(sessionid,c.sess,size);
	sessionid[size-1]=0;
	*streamid=c.sid;

	return 0;

err3:	c.flags=0;
	rtsp_teardown(&c);
err2:	if(c.rtspfd!=-1)close(c.rtspfd);
err1:	return r;
}

int satip_cln_end_multicast(void *handle,char *server,int port,char *sessionid,
	int streamid)
{
	int r=SATIP_SYSFAIL;
	INSTANCE *inst=(INSTANCE *)handle;
	CONN c;

	if(!handle||!server||port<1||port>65535||!sessionid||streamid<0||
		streamid>65535)goto err1;

	memset(&c,0,sizeof(CONN));
	c.rtspfd=-1;
	c.sid=streamid;
	strncpy(c.sess,sessionid,sizeof(c.sess));
	c.sess[sizeof(c.sess)-1]=0;
	c.instance=inst;
	c.port=port;

	if(resolve(server,&c.addr,inst->level))goto err1;

	if(invalid_mcast(&c.addr)!=-1)goto err1;

	r=rtsp_teardown(&c);

	if(c.rtspfd!=-1)close(c.rtspfd);

err1:	return r;
}

void *satip_cln_init(SATIP_CLN_CONFIG *config)
{
	int i;
	INSTANCE *instance;
	uint64_t dummy=1;
	struct epoll_event e;
	struct itimerspec ts;
	static char tfdmarker=0;
	static char dfdmarker=7;
	static FUNC ttme={twork,0};
	static FUNC ucst[3]={{uwork,SATIP_V4_ONLY},{uwork,SATIP_V6_LINK},
			{uwork,SATIP_V6_SITE}};
	static FUNC mcst[3]={{mwork,SATIP_V4_ONLY},{mwork,SATIP_V6_LINK},
			{mwork,SATIP_V6_SITE}};

	if(!config||!*config->dev)goto err1;
	if(config->mttl<1||config->mttl>255)goto err1;
	if(config->interval<SATIP_UPNP_OFF||config->interval>SATIP_UPNP_MAX)
		goto err1;

	switch(config->level)
	{
	case SATIP_V4_ONLY:
	case SATIP_V6_LINK:
	case SATIP_V6_SITE:
		break;
	default:goto err1;
	}

	if(!(instance=malloc(sizeof(INSTANCE))))goto err1;
	memset(instance,0,sizeof(INSTANCE));

	if(config->charset[0])strcpy(instance->cset,config->charset);
	else strcpy(instance->cset,"ISO8859-1");

	instance->level=config->level;
	instance->mttl=config->mttl;
	instance->strict=config->strict;
	instance->fast=config->fast;
	instance->idleflush=config->idleflush;
	instance->upnpinterval=
		(config->interval==SATIP_UPNP_OFF)?-1:(config->interval*60)<<3;
	if(!instance->upnpinterval)instance->upnpinterval=60<<3;
	instance->upnpticker=(config->interval==SATIP_UPNP_OFF)?-1:20;

	if(!config->portmin&&config->portmax)goto err2;
	else if(config->portmin)
	{
		if(config->portmin<1024||config->portmin>65534)goto err2;
		if(config->portmin>=config->portmax)goto err2;
		if((config->portmin&1)||!(config->portmax&1))goto err2;
	}
	if(config->rtpbuffer)
		if(config->rtpbuffer<32768||config->rtpbuffer>1048576)goto err2;
	instance->portmin=config->portmin;
	instance->portmax=config->portmax;
	instance->rtpbuffer=config->rtpbuffer;

	if(!(instance->devidx=if_nametoindex(config->dev)))goto err2;
	if(!(instance->loidx=if_nametoindex(LOOPDEV)))goto err2;
	if(instance->devidx==instance->loidx)goto err2;

	for(i=0;i<=instance->level;i++)
		if(addr4if(instance->devidx,i,&instance->devaddr[i]))goto err2;

	if(instance->upnpinterval!=-1)
	{
		for(i=0;i<=instance->level;i++)if((instance->udp[i]=
			mcast(instance,instance->mttl,NULL,0,i,0))==-1)
		{
			while(--i>=0)close(instance->udp[i]);
			goto err2;
		}

		for(i=0;i<=instance->level;i++)
			if(i==SATIP_V6_LINK&&instance->level==SATIP_V6_SITE)
				continue;
		else if((instance->mdp[i]=
			mcast(instance,instance->mttl,NULL,MPORT,i,0))==-1)
		{
			while(--i>=0)if(i==SATIP_V6_LINK&&
				instance->level==SATIP_V6_SITE)continue;
			else close(instance->mdp[i]);
			goto err3;
		}
	}

	if(pthread_mutex_init(&instance->stx,NULL))goto err4;
	if(pthread_mutex_init(&instance->etx,NULL))goto err5;
	if(pthread_mutex_init(&instance->utx,NULL))goto err6;
	if(pthread_spin_init(&instance->ttx,PTHREAD_PROCESS_PRIVATE))goto err7;

	if((instance->efd=epoll_create1(EPOLL_CLOEXEC))==-1)goto err8;

	if((instance->tfd=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)goto err9;
	e.events=EPOLLIN;
	e.data.ptr=&tfdmarker;
	if(epoll_ctl(instance->efd,EPOLL_CTL_ADD,instance->tfd,&e))goto err10;

	if((instance->dfd[0]=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)
		goto err11;
	e.events=EPOLLIN;
	e.data.ptr=&dfdmarker;
	if(epoll_ctl(instance->efd,EPOLL_CTL_ADD,instance->dfd[0],&e))
		goto err12;

	if((instance->dfd[1]=eventfd(0,EFD_CLOEXEC|EFD_NONBLOCK))==-1)
		goto err13;

	if((instance->ifd=timerfd_create(CLOCK_MONOTONIC,TFD_CLOEXEC))==-1)
		goto err14;

	if((instance->xfd=epoll_create1(EPOLL_CLOEXEC))==-1)goto err15;

	e.events=EPOLLIN;
	e.data.ptr=NULL;
	if(epoll_ctl(instance->xfd,EPOLL_CTL_ADD,instance->tfd,&e))goto err16;

	e.events=EPOLLIN;
	e.data.ptr=&ttme;
	if(epoll_ctl(instance->xfd,EPOLL_CTL_ADD,instance->ifd,&e))goto err17;

	if(instance->upnpinterval!=-1)
	{
		for(i=0;i<=instance->level;i++)
		{
			e.events=EPOLLIN;
			e.data.ptr=&ucst[i];
			if(epoll_ctl(instance->xfd,EPOLL_CTL_ADD,
				instance->udp[i],&e))
			{
				while(--i>=0)epoll_ctl(instance->xfd,
					EPOLL_CTL_DEL,instance->udp[i],NULL);
				goto err18;
			}
		}

		for(i=0;i<=instance->level;i++)
		{
			if(i==SATIP_V6_LINK&&instance->level==SATIP_V6_SITE)
				continue;
			e.events=EPOLLIN;
			e.data.ptr=&mcst[i];
			if(epoll_ctl(instance->xfd,EPOLL_CTL_ADD,
				instance->mdp[i],&e))
			{
				while(--i>=0)if(i==SATIP_V6_LINK&&
					instance->level==SATIP_V6_SITE)continue;
				else epoll_ctl(instance->xfd,EPOLL_CTL_DEL,
					instance->mdp[i],NULL);
				goto err19;
			}
		}
	}

	memset(&ts,0,sizeof(ts));
	ts.it_value.tv_nsec=125000000;
	ts.it_interval.tv_nsec=125000000;
	if(timerfd_settime(instance->ifd,0,&ts,NULL))goto err20;

	if(pthread_attr_init(&instance->attr))goto err21;
	if(pthread_attr_setdetachstate(&instance->attr,PTHREAD_CREATE_JOINABLE))
		goto err22;
	if(config->stack)
		if(pthread_attr_setstacksize(&instance->attr,config->stack))
			goto err22;
	if(pthread_create(&instance->th,&instance->attr,manager,instance))
		goto err22;
	if(pthread_create(&instance->eh,&instance->attr,streamer,instance))
		goto err23;

	instance->ep[0].events=POLLIN;
	instance->ep[0].fd=instance->tfd;
	instance->ep[1].events=POLLIN;
	instance->ep[1].fd=instance->dfd[1];

	return instance;

err23:	dummy=write(instance->tfd,&dummy,sizeof(dummy));
	pthread_join(instance->th,NULL);
err22:	pthread_attr_destroy(&instance->attr);
err21:	memset(&ts,0,sizeof(ts));
	timerfd_settime(instance->ifd,0,&ts,NULL);
err20:	if(instance->upnpinterval!=-1)for(i=0;i<=instance->level;i++)
	{
		if(i==SATIP_V6_LINK&&instance->level==SATIP_V6_SITE)continue;
		else epoll_ctl(instance->xfd,EPOLL_CTL_DEL,instance->mdp[i],
			NULL);
	}
err19:	if(instance->upnpinterval!=-1)for(i=0;i<=instance->level;i++)
		epoll_ctl(instance->xfd,EPOLL_CTL_DEL,instance->udp[i],NULL);
err18:	epoll_ctl(instance->xfd,EPOLL_CTL_DEL,instance->ifd,NULL);
err17:	epoll_ctl(instance->xfd,EPOLL_CTL_DEL,instance->tfd,NULL);
err16:	close(instance->xfd);
err15:	close(instance->ifd);
err14:	close(instance->dfd[1]);
err13:	epoll_ctl(instance->efd,EPOLL_CTL_DEL,instance->dfd[0],NULL);
err12:	close(instance->dfd[0]);
err11:	epoll_ctl(instance->efd,EPOLL_CTL_DEL,instance->tfd,NULL);
err10:	close(instance->tfd);
err9:	close(instance->efd);
err8:	pthread_spin_destroy(&instance->ttx);
err7:	pthread_mutex_destroy(&instance->utx);
err6:	pthread_mutex_destroy(&instance->etx);
err5:	pthread_mutex_destroy(&instance->stx);
err4:	if(instance->upnpinterval!=-1)for(i=0;i<=instance->level;i++)
	{
		if(i==SATIP_V6_LINK&&instance->level==SATIP_V6_SITE)continue;
		else close(instance->mdp[i]);
	}
err3:	if(instance->upnpinterval!=-1)for(i=0;i<=instance->level;i++)
		close(instance->udp[i]);
err2:	free(instance);
err1:	return NULL;
}

void satip_cln_fini(void *data)
{
	int i;
	INSTANCE *instance=(INSTANCE *)data;
	CONN *c;
	UPNPDEVICE *e;
	struct itimerspec ts;
	uint64_t dummy=1;

	if(!data)return;

	dummy=write(instance->tfd,&dummy,sizeof(dummy));
	pthread_join(instance->eh,NULL);
	pthread_join(instance->th,NULL);

	pthread_attr_destroy(&instance->attr);

	memset(&ts,0,sizeof(ts));
	timerfd_settime(instance->ifd,0,&ts,NULL);

	for(i=0;i<=instance->level;i++)while(instance->nlist[i])
	{
		e=instance->nlist[i];
		instance->nlist[i]=e->next;
		free(e);
	}

	while(instance->clist)
	{
		c=instance->clist;
		instance->clist=c->next;
		if(!c->rtfail)epoll_ctl(instance->efd,EPOLL_CTL_DEL,c->rtpfd,
			NULL);
		if(!c->rcfail)epoll_ctl(instance->efd,EPOLL_CTL_DEL,c->rtcpfd,
			NULL);
		i=pthread_mutex_trylock(&c->mtx);
		pthread_spin_lock(&instance->ttx);
		if(c->runner)pthread_cancel(c->th);
		pthread_spin_unlock(&instance->ttx);
		close(c->rtpfd);
		close(c->rtcpfd);
		if(c->rtspfd!=-1)close(c->rtspfd);
		if(!i)pthread_mutex_unlock(&c->mtx);
		pthread_mutex_destroy(&c->mtx);
		free(c);
	}

	while(instance->hlist)
	{
		c=instance->hlist;
		instance->clist=c->next;
		if(!c->htfail)epoll_ctl(instance->efd,EPOLL_CTL_DEL,c->httpfd,
			NULL);
		if(c->httpfd!=-1)close(c->httpfd);
		free(c);
	}

	epoll_ctl(instance->efd,EPOLL_CTL_DEL,instance->dfd[0],NULL);
	epoll_ctl(instance->efd,EPOLL_CTL_DEL,instance->tfd,NULL);
	if(instance->upnpinterval!=-1)for(i=0;i<=instance->level;i++)
	{
		if(i==SATIP_V6_LINK&&instance->level==SATIP_V6_SITE)continue;
		else epoll_ctl(instance->xfd,EPOLL_CTL_DEL,instance->mdp[i],
			NULL);
	}
	if(instance->upnpinterval!=-1)for(i=0;i<=instance->level;i++)
		epoll_ctl(instance->xfd,EPOLL_CTL_DEL,instance->udp[i],NULL);
	epoll_ctl(instance->xfd,EPOLL_CTL_DEL,instance->ifd,NULL);
	epoll_ctl(instance->xfd,EPOLL_CTL_DEL,instance->tfd,NULL);
	close(instance->xfd);
	close(instance->efd);
	close(instance->tfd);
	close(instance->dfd[0]);
	close(instance->dfd[1]);
	close(instance->ifd);
	pthread_mutex_destroy(&instance->utx);
	pthread_mutex_destroy(&instance->etx);
	pthread_mutex_destroy(&instance->stx);
	pthread_spin_destroy(&instance->ttx);
	if(instance->upnpinterval!=-1)for(i=0;i<=instance->level;i++)
	{
		if(i==SATIP_V6_LINK&&instance->level==SATIP_V6_SITE)continue;
		else close(instance->mdp[i]);
	}
	if(instance->upnpinterval!=-1)for(i=0;i<=instance->level;i++)
		close(instance->udp[i]);

	free(instance);
}
