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
#include <net/if.h>
#include <sys/times.h>
#include <sys/time.h>
#include <sys/capability.h>
#include <sys/timerfd.h>
#include <sys/eventfd.h>
#include <sys/epoll.h>
#include <sys/uio.h>
#include <netdb.h>
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <poll.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include "satip.h"
#include "common.h"

#define QLEN		32
#define MAXBUFSIZE	65536
#define MAX_EPOLL	64
#define STD_RTSP_IDLE	60
#define MAX_TX_TIME	2
#define UNKNOWN_FE	65535
#define V3TIME		125
#define V3ROBUST	2
#define V3WAIT		10
#define V3FINAL		1
#define V3IDLE		((V3ROBUST*V3TIME)+V3WAIT)
#define V3NOQRY		((V3ROBUST*V3TIME)+(V3WAIT/2))

#define IFWAIT		0
#define SETUP		1
#define ANNOUNCE	2
#define IDWAIT1		3
#define IDWAIT2		4
#define IDWAIT3		5
#define IDWAIT4		6
#define PRERUN		7
#define RUNNING		8
#define RESTART		256
#define TERMINATE	257

#define KILL_ALL	0
#define KILL_RTSP	1
#define KILL_HTTP	2
#define KILL_SID	3
#define KILL_TUNER	4
#define LOCK_ALL	5
#define	LOCK_RTSP	6
#define LOCK_HTTP	7
#define ADD_MULTICAST	8
#define DEL_MULTICAST	9
#define FLUSH_MULTICAST	10
#define GET_STATS	11
#define LIST_RTSP	12
#define LIST_HTTP	13

#define HDR_PUBLIC	0x00000001
#define HDR_TYPE	0x00000002
#define HDR_BASE	0x00000004
#define HDR_LEN		0x00000008
#define HDR_TMO0	0x00000010
#define HDR_TMOXX	0x00000020
#define HDR_SID		0x00000040
#define HDR_INFO	0x00000080
#define HDR_QUERY	0x00000100
#define HDR_STRICT	0x00000200

#define PLAY_PLAYING	0x00000001
#define PLAY_PERSIST	0x00000002
#define PLAY_NOTIFIED	0x00000004
#define PLAY_UPDATED	0x00000008
#define PLAY_AUTOFE	0x00000010

#define PROT		"1.2"

typedef struct _ifaddr
{
	struct _ifaddr *next;
	struct in6_addr a6;
	int scope;
} IFADDR;

typedef struct
{
	int total;
	char *hdr;
	char *name[20];
	char *value[20];
} MSGDATA;

typedef struct _coll
{
	struct _coll *next;
	ADDR addr;
	int cnt;
	int which;
	unsigned short port;
} COLL;

typedef struct _chunk
{
	struct _chunk *next;
	int len;
	int sent;
	char msg[0];
} CHUNK;

typedef struct
{
#pragma pack(push,1)
	void *which;
	void (*func)();
#pragma pack(pop)
} FUNC;

typedef struct _rtsp
{
#pragma pack(push,1)
	struct _rtsp *next;
	void (*func)();
#pragma pack(pop)
	CHUNK *tx;
	int client;
	int idle;
	int close;
	int fill;
	int which;
	ADDR addr;
	ADDR devaddr;
	char bfr[2048];
} RTSP;

typedef struct
{
	int total;
	unsigned char a6[QLEN-1][16];
} MLDQ;

typedef struct _member
{
	struct _member *next;
	struct _group *group;
	void *h;
	int mode;
	int notified;
} MEMBER;

typedef struct _group
{
	struct _group *next;
	MEMBER *list;
	void *h;
	pthread_spinlock_t dmtx;
	pthread_spinlock_t smtx;
	SATIP_TUNE rtune;
	SATIP_TUNE tune;
	SATIP_PIDS set;
} GROUP;

typedef struct
{
	struct msghdr mh;
	struct iovec io;
	char bfr[64];
} MSGH;

typedef struct _session
{
	struct _session *next;
	struct _session *master;
	struct _session *slave;
	struct _session *mcast;
	MEMBER *handle;
	unsigned long long id;
	int sid;
	int idle;
	int play;
	int mode;
	int ttl;
	int rtpfd;
	int rtcpfd;
	int ilen;
	int v3idle;
	int v3mode;
	int rrrecv;
	int burst;
	int fill;
	int strict;
	unsigned int ssrc;
	unsigned int pkts;
	unsigned int dtot;
	unsigned short seq;
	unsigned short port;
	SOCK rtp;
	SOCK rtcp;
	MSGH msgh1;
	MSGH msgh2;
	struct mmsghdr msgqh[SATIP_MAX_BURST];
	struct iovec io[SATIP_MAX_BURST];
	char cmsg[64];
	pthread_spinlock_t mtx;
	SATIP_STATUS state;
	char info[512];
	SATIP_STREAM queue[0];
} SESSION;

typedef struct _stream
{
	struct _stream *next;
	void *handle;
	int fd;
	int mode;
	int strict;
	SOCK peer;
	pthread_spinlock_t mtx;
	SATIP_STATUS state;
} STREAM;

typedef struct
{
	unsigned short port;
	unsigned short rtsp;
	unsigned short http;
	unsigned short mdft;
	unsigned short cfghttp;
	unsigned short cfgrtsp;
	unsigned char tx64valid;
	unsigned char tx32valid;
	int sat;
	int terr;
	int cable;
	int devid;
	int oldid;
	int age;
	int state;
	int flag;
	unsigned int bootcnt;
	int reqfd;
	int ansfd;
	int efd;
	int tfd;
	int sfd;
	int nfd;
	int ffd;
	int udp[3];
	int mdp[3];
	int tcp[3];
	int rcp[3];
	int qry[2];
	int cmd;
	int ans;
	int running;
	int rtsplock;
	int httplock;
	int locked;
	int rem;
	int ms125;
	int igmpv3;
	int igmpwait;
	int igmpnext;
	int igmpquerier;
	int mldv2;
	int mldwait;
	int mldnext;
	int mldquerier;
	int nlstate;
	int level;
	int ifmap;
	int devidx;
	int webflags;
	int xmllen;
	int mttl;
	int sessions;
	int allsessions;
	int playsessions;
	int playhttp;
	int rtsplimit;
	int httplimit;
	int httpnostream;
	int strict;
	int timeout;
	int burst;
	int portmin;
	int portmax;
	unsigned int caps;
	unsigned int seq;
	unsigned int tx32[2];
	unsigned long long tx64[2];
	unsigned long long bps;
	unsigned long long bpslimit;
	void *data;
	COLL *coll;
	RTSP *rlist;
	SESSION *slist;
	STREAM *hlist;
	GROUP *glist;
	IFADDR *ifaddr;
	char *xml;
	void *priv;
	int (*cb)(int code,void *data,void *priv);
	pthread_t srv;
	pthread_mutex_t htx;
	pthread_mutex_t stx;
	pthread_mutex_t ctx;
	pthread_attr_t attr;
	ADDR devaddr[3];
	char uuid[SATIP_UUID_LEN+1];
	char dev[IFNAMSIZ+1];
	unsigned long sidmap[2033];
	unsigned int igmpq[8][QLEN];
	MLDQ mldq[8];
	SESSION *mcast[256];
	unsigned char v3hint[32];
	char xmlcharset[64];
} UPNP;

typedef struct
{
	int client;
	int state;
	int which;
	SOCK addr;
	UPNP *upnp;
} CLNDTA;

typedef struct
{
	SATIP_TUNE tune;
	SATIP_PIDS set;
	SATIP_PIDS add;
	SATIP_PIDS del;
} REQUEST;

typedef struct
{
	int mode;
	int rtp;
	int rtcp;
	int ttl;
	ADDR addr;
} TRANSPORT;

typedef struct
{
	int play;
	TRANSPORT *t;
	SATIP_TUNE *tune;
	SATIP_PIDS *set;
} MCAST;

#ifndef PROFILE

static void rtp_cb(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void http_cb(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
static void timetick(UPNP *upnp,void *data,int events) __attribute__ ((hot));
static void rtcp_cb(void *id,SATIP_STATUS *status) __attribute__ ((hot));
static void htcp_cb(void *id,SATIP_STATUS *status) __attribute__ ((hot));
static int rtcp_receiver(SESSION *s) __attribute__ ((hot));
static void *server(void *data) __attribute__ ((hot));
void satip_srv_stream(void *id,SATIP_STREAM *stream) __attribute__ ((hot));
void satip_srv_status(void *id,SATIP_STATUS *status) __attribute__ ((hot));

void *satip_srv_init(SATIP_SRV_CONFIG *config,SATIP_CFGINFO *hwinfo)
	__attribute__ ((cold));
void satip_srv_fini(void *handle) __attribute__ ((cold));

#endif

static ADDR mcauto[3]=
{
	{
		.family=AF_INET,
#if __BYTE_ORDER == __LITTLE_ENDIAN
		.a4.s_addr=0x000000ef,
#elif __BYTE_ORDER == __BIG_ENDIAN
		.a4.s_addr=0xef000000,
#else
#error Need endian!
#endif
	},
	{
		.family=AF_INET6,
		.a6.s6_addr={0xff,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
			0x73,0x61,0x74,0x69,0x70,0x00,0x00,0x00},
	},
	{
		.family=AF_INET6,
		.a6.s6_addr={0xff,0x05,0x00,0x00,0x00,0x00,0x00,0x00,
			0x73,0x61,0x74,0x69,0x70,0x00,0x00,0x00},
	},
};

static ADDR mchosts[2]=
{
	{
		.family=AF_INET,
#if __BYTE_ORDER == __LITTLE_ENDIAN
		.a4.s_addr=0x010000e0,
#elif __BYTE_ORDER == __BIG_ENDIAN
		.a4.s_addr=0xe0000001,
#else
#error Need endian!
#endif
	},
	{
		.family=AF_INET6,
		.a6.s6_addr={0xff,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01},
	},
};

static ADDR mcreports[2]=
{
	{
		.family=AF_INET,
#if __BYTE_ORDER == __LITTLE_ENDIAN
		.a4.s_addr=0x160000e0,
#elif __BYTE_ORDER == __BIG_ENDIAN
		.a4.s_addr=0xe0000016,
#else
#error Need endian!
#endif
	},
	{
		.family=AF_INET6,
		.a6.s6_addr={0xff,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x16},
	},
};

static unsigned short chksum(unsigned short *data,int len)
{
	unsigned int chk=0;

	while(len--)chk+=ntohs(*data++);
	while(chk&0xffff0000)chk=(chk&0xffff)+(chk>>16);
	return htons((unsigned short)(~chk));
}

static unsigned short mkseq(int sid)
{
	struct tms dta;
	clock_t now;

	now=times(&dta);
	return (dta.tms_utime^dta.tms_stime^(now<<4)^(sid<<8))&0xffff;
}

static void autoaddr(int which,int sid,int devid,ADDR *addr)
{
	switch(which)
	{
	case SATIP_V4_ONLY:
		*addr=mcauto[which];
		addr->a4.s_addr=htonl(ntohl(addr->a4.s_addr)|
			(devid<<16)|((sid/255)<<8)|(sid%255));
		break;

	case SATIP_V6_LINK:
	case SATIP_V6_SITE:
		*addr=mcauto[which];
		addr->a6.s6_addr[13]=(unsigned char)devid;
		addr->a6.s6_addr[14]=(unsigned char)(sid/255);
		addr->a6.s6_addr[15]=(unsigned char)(sid%255);
		break;
	}
}

static int igmpv3(UPNP *upnp)
{
	int s;
	int v=1;
	struct ip_mreqn mr;
	static unsigned char opts[4]={0x94,0x04,0x00,0x00};

	memset(&mr,0,sizeof(mr));
	mr.imr_ifindex=upnp->devidx;

	if((s=socket(AF_INET,SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,IPPROTO_IGMP))
		==-1)goto err1;

	if(setsockopt(s,SOL_SOCKET,SO_REUSEADDR,&v,sizeof(v)))goto err2;
	if(setsockopt(s,IPPROTO_IP,IP_TTL,&v,sizeof(v)))goto err2;
	if(setsockopt(s,IPPROTO_IP,IP_PKTINFO,&v,sizeof(v)))goto err2;
	if(setsockopt(s,IPPROTO_IP,IP_OPTIONS,opts,sizeof(opts)))goto err2;

	mr.imr_multiaddr.s_addr=mchosts[SATIP_V4_ONLY].a4.s_addr;
	if(setsockopt(s,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mr,sizeof(mr)))goto err2;

	mr.imr_multiaddr.s_addr=mcreports[SATIP_V4_ONLY].a4.s_addr;
	if(setsockopt(s,IPPROTO_IP,IP_ADD_MEMBERSHIP,&mr,sizeof(mr)))goto err2;

	return s;

err2:	close(s);
err1:	return -1;
}

static int mldv2(UPNP *upnp)
{
	int s;
	int v=1;
	struct ipv6_mreq mr;
	static unsigned char opts[8]={0x00,0x00,0x05,0x02,0x00,0x00,0x01,0x00};

	memset(&mr,0,sizeof(mr));
	mr.ipv6mr_interface=upnp->devidx;

	if((s=socket(AF_INET6,
		SOCK_RAW|SOCK_NONBLOCK|SOCK_CLOEXEC,IPPROTO_ICMPV6))==-1)
			goto err1;

	v=1;
	if(setsockopt(s,SOL_SOCKET,SO_REUSEADDR,&v,sizeof(v)))goto err2;
	if(setsockopt(s,IPPROTO_IPV6,IPV6_UNICAST_HOPS,&v,sizeof(v)))goto err2;
	if(setsockopt(s,IPPROTO_IPV6,IPV6_MULTICAST_HOPS,&v,sizeof(v)))
		goto err2;
	if(setsockopt(s,IPPROTO_IPV6,IPV6_RECVPKTINFO,&v,sizeof(v)))goto err2;
	if(setsockopt(s,IPPROTO_IPV6,IPV6_HOPOPTS,opts,sizeof(opts)))goto err2;

	memcpy(mr.ipv6mr_multiaddr.s6_addr,
		mchosts[SATIP_V6_LINK].a6.s6_addr,16);
	if(setsockopt(s,IPPROTO_IPV6,IPV6_ADD_MEMBERSHIP,(const void *)&mr,
		sizeof(mr)))goto err2;

	memcpy(mr.ipv6mr_multiaddr.s6_addr,
		mcreports[SATIP_V6_LINK].a6.s6_addr,16);
	if(setsockopt(s,IPPROTO_IPV6,IPV6_ADD_MEMBERSHIP,(const void *)&mr,
		sizeof(mr)))goto err2;

	return s;

err2:	close(s);
err1:	return -1;
}

static int udp(UPNP *upnp,ADDR *addr,int port,int which)
{
	int s;
	int v=1;
	SOCK a;

	if((s=socket(addr->family,SOCK_DGRAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0))<0)
		goto err1;

	if(anyaddr(addr))
	{
		addr2sock(&a,&upnp->devaddr[which],htons(port<0?-port:port),
			upnp->devidx);
		if(port>0)goto reuse;
	}
	else
	{
		addr2sock(&a,addr,htons(port<0?-port:port),upnp->devidx);
reuse:		if(setsockopt(s,SOL_SOCKET,SO_REUSEADDR,&v,sizeof(v)))goto err2;
	}

	if(addr->family==AF_INET6)
	{
		if(setsockopt(s,IPPROTO_IPV6,IPV6_V6ONLY,&v,sizeof(v)))
			goto err2;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_RECVPKTINFO,&v,sizeof(v)))
			goto err2;
	}
	else if(setsockopt(s,IPPROTO_IP,IP_PKTINFO,&v,sizeof(v)))goto err2;

	if((bind(s,(struct sockaddr *)&a,sizeof(a)))<0)goto err2;

	return s;

err2:	close(s);
err1:	return -1;
}

static int tcp(UPNP *upnp,ADDR *addr,unsigned short *port)
{
	int s;
	int v;
	socklen_t l;
	SOCK a;

	if((s=socket(addr->family,SOCK_STREAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0))<0)
		goto err1;

	addr2sock(&a,addr,htons(*port),upnp->devidx);

	if(!anyaddr(addr)||port)
	{
		v=1;
		if(setsockopt(s,SOL_SOCKET,SO_REUSEADDR,&v,sizeof(v)))goto err2;
	}

	if(addr->family==AF_INET6)
	{
		v=1;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_V6ONLY,&v,sizeof(v)))
			goto err2;
	}

	if((bind(s,(struct sockaddr *)&a,sizeof(a)))<0)goto err2;

	if(!*port)
	{
		l=sizeof(a);
		if(getsockname(s,(struct sockaddr *)&a,&l))goto err2;
		if(invalid_sock(&a,l))goto err2;
		*port=ntohs(psget(&a));
	}

	if(listen(s,255))goto err2;

	return s;

err2:	close(s);
err1:	return -1;
}

static int mcast(UPNP *upnp,ADDR *addr,int port,int ttl,int which)
{
	int s;
	int v=1;
	SOCK a;
	struct ip_mreqn m4;
	struct ipv6_mreq m6;

	if((s=socket(addr->family,SOCK_DGRAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0))<0)
		goto err1;

	addr2sock(&a,port>=0?addr:&upnp->devaddr[which],
		htons(port<0?-port:port),upnp->devidx);

	if(!anyaddr(addr)||port>0)
		if(setsockopt(s,SOL_SOCKET,SO_REUSEADDR,&v,sizeof(v)))goto err2;

	if(addr->family==AF_INET6)
	{
		if(setsockopt(s,IPPROTO_IPV6,IPV6_V6ONLY,&v,sizeof(v)))
			goto err2;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_RECVPKTINFO,&v,sizeof(v)))
			goto err2;
	}
	else if(setsockopt(s,IPPROTO_IP,IP_PKTINFO,&v,sizeof(v)))goto err2;

	if((bind(s,(struct sockaddr *)&a,sizeof(a)))<0)goto err2;

	if(!anyaddr(addr)||port>0)switch(addr->family)
	{
	case AF_INET:
		memset(&m4,0,sizeof(m4));
		m4.imr_multiaddr.s_addr=addr->a4.s_addr;
		m4.imr_ifindex=upnp->devidx;

		if(setsockopt(s,IPPROTO_IP,IP_ADD_MEMBERSHIP,
			(const void *)&m4,sizeof(m4)))goto err2;
		break;
	case AF_INET6:
		memset(&m6,0,sizeof(m6));
		memcpy(m6.ipv6mr_multiaddr.s6_addr,addr->a6.s6_addr,16);
		m6.ipv6mr_interface=upnp->devidx;

		if(setsockopt(s,IPPROTO_IPV6,IPV6_ADD_MEMBERSHIP,
			(const void *)&m6,sizeof(m6)))goto err2;
		break;
	default:goto err2;
	}
	else switch(addr->family)
	{
	case AF_INET:
		memset(&m4,0,sizeof(m4));
		m4.imr_ifindex=upnp->devidx;

		if(setsockopt(s,IPPROTO_IP,IP_MULTICAST_IF,&m4,sizeof(m4)))
			goto err2;
		if(setsockopt(s,IPPROTO_IP,IP_MULTICAST_TTL,&ttl,sizeof(ttl)))
			goto err2;
		if(setsockopt(s,IPPROTO_IP,IP_MULTICAST_LOOP,&v,sizeof(v)))
			goto err2;
		break;
	case AF_INET6:
		v=upnp->devidx;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_MULTICAST_IF,&v,sizeof(v)))
			goto err2;
		v=linklocal(addr)?1:ttl;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_UNICAST_HOPS,&v,sizeof(v)))
			goto err2;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_MULTICAST_HOPS,&v,sizeof(v)))
			goto err2;
		v=1;
		if(setsockopt(s,IPPROTO_IPV6,IPV6_MULTICAST_LOOP,&v,sizeof(v)))
			goto err2;
		break;
	default:goto err2;
	}

	return s;

err2:	close(s);
err1:	return -1;
}

static int udppair(UPNP *upnp,ADDR *addr,int *rtp,int *rtcp,
	unsigned short *port,int which)
{
	unsigned short even;
	unsigned short odd;
	int i;
	SATIP_DATA data;

	for(i=0;i<128;i++)
	{
		data.intval=sizeof(even);
		data.ptrval=&even;
		if(upnp->cb(SATIP_GETRANDOM,&data,upnp->priv))continue;
		if(upnp->portmin)
		{
			even%=upnp->portmax-upnp->portmin;
			even&=0xfffe;
			even+=upnp->portmin;
		}
		else
		{
			even%=0xfc00;
			even&=0xfffe;
			even+=0x0400;
		}
		odd=even+1;
		if((*rtp=udp(upnp,addr,-((int)even),which))==-1)continue;
		if((*rtcp=udp(upnp,addr,-((int)odd),which))!=-1)break;
		close(*rtp);
	}
	if(i<128)
	{
		*port=(unsigned short)even;
		return 0;
	}
	for(i=32768;i<49152;i+=2)
	{
		even=i;
		odd=even+1;
		if((*rtp=udp(upnp,addr,-((int)even),which))==-1)continue;
		if((*rtcp=udp(upnp,addr,-((int)odd),which))!=-1)break;
		close(*rtp);
	}
	if(i<49152)
	{
		*port=(unsigned short)even;
		return 0;
	}
	*rtp=-1;
	*rtcp=-1;
	*port=0;
	return -1;
}

static int mcastpair(UPNP *upnp,ADDR *a,int ttl,int *rtp,int *rtcp,
	unsigned short *port,int which)
{
	unsigned short even;
	unsigned short odd;
	int i;
	ADDR addr;
	SATIP_DATA data;

	memset(&addr,0,sizeof(addr));
	addr.family=a->family;

	for(i=0;i<128;i++)
	{
		data.intval=sizeof(even);
		data.ptrval=&even;
		if(upnp->cb(SATIP_GETRANDOM,&data,upnp->priv))continue;
		if(upnp->portmin)
		{
			even%=upnp->portmax-upnp->portmin;
			even&=0xfffe;
			even+=upnp->portmin;
		}
		else
		{
			even%=0xfc00;
			even&=0xfffe;
			even+=0x0400;
		}
		odd=even+1;
		if((*rtp=mcast(upnp,&addr,-((int)even),ttl,which))==-1)continue;
		if((*rtcp=mcast(upnp,&addr,-((int)odd),ttl,which))!=-1)break;
		close(*rtp);
	}
	if(i<128)
	{
		*port=(unsigned short)even;
		return 0;
	}
	for(i=32768;i<49152;i+=2)
	{
		even=i;
		odd=even+1;
		if((*rtp=mcast(upnp,&addr,-((int)even),ttl,which))==-1)continue;
		if((*rtcp=mcast(upnp,&addr,-((int)odd),ttl,which))!=-1)break;
		close(*rtp);
	}
	if(i<49152)
	{
		*port=(unsigned short)even;
		return 0;
	}
	*rtp=-1;
	*rtcp=-1;
	return -1;
}

static int tcpconnect(ADDR *addr,short port,int devidx)
{
	int s;
	int v;
	int x;
	SOCK saddr;
	struct pollfd p;

	if((s=socket(addr->family,SOCK_STREAM|SOCK_CLOEXEC|SOCK_NONBLOCK,0))<0)
		return -1;

	addr2sock(&saddr,addr,port,devidx);

	if((connect(s,(struct sockaddr *)&saddr,sizeof(SOCK)))<0)
	{
		if(errno==EINPROGRESS)
		{
			p.fd=s;
			p.events=POLLOUT;
			if(poll(&p,1,500)==1)
			{
				v=sizeof(x);
				if(!getsockopt(s,SOL_SOCKET,SO_ERROR,&x,&v))
					if(!x)goto conn;
			}
		}

		close(s);
		return -1;
	}

conn:   v=1;
	if(setsockopt(s,IPPROTO_TCP,TCP_NODELAY,&v,sizeof(v)))
	{
		close(s);
		return -1;
	}

	return s;
}

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
		if(a6linklocal(&((struct sockaddr_in6 *)
			(ap->ai_addr))->sin6_addr))
		{
			if(curr==SATIP_V6_LINK)break;
			curr=SATIP_V6_LINK;
			addr->family=AF_INET6;
			memcpy(addr->a6.s6_addr,((struct sockaddr_in6 *)
				(ap->ai_addr))->sin6_addr.s6_addr,16);
			break;
		}
		if(level!=SATIP_V6_SITE)break;
		if(!a6sitewide(&((struct sockaddr_in6 *)
			(ap->ai_addr))->sin6_addr))break;
		curr=SATIP_V6_SITE;
		addr->family=AF_INET6;
		memcpy(addr->a6.s6_addr,((struct sockaddr_in6 *)
			(ap->ai_addr))->sin6_addr.s6_addr,16);
		break;
	}
	if(res)freeaddrinfo(res);
	return addr->family?0:-1;
}

static void mkmsgh(UPNP *upnp,MSGH *msgh,int which)
{
	struct cmsghdr *cm;
	struct in_pktinfo *i4;
	struct in6_pktinfo *i6;

	msgh->mh.msg_namelen=sizeof(SOCK);
	msgh->mh.msg_iov=&msgh->io;
	msgh->mh.msg_iovlen=1;
	msgh->mh.msg_control=msgh->bfr;
	msgh->mh.msg_flags=0;

	if(which==SATIP_V4_ONLY)
	{
		msgh->mh.msg_controllen=CMSG_SPACE(sizeof(struct in_pktinfo));
		cm=CMSG_FIRSTHDR(&msgh->mh);
		cm->cmsg_level=IPPROTO_IP;
		cm->cmsg_type=IP_PKTINFO;
		cm->cmsg_len=CMSG_LEN(sizeof(struct in_pktinfo));
		i4=(struct in_pktinfo *)CMSG_DATA(cm);
		memset(i4,0,sizeof(struct in_pktinfo));
		i4->ipi_spec_dst=upnp->devaddr[which].a4;
	}
	else
	{
		msgh->mh.msg_controllen=CMSG_SPACE(sizeof(struct in6_pktinfo));
		cm=CMSG_FIRSTHDR(&msgh->mh);
		cm->cmsg_level=IPPROTO_IPV6;
		cm->cmsg_type=IPV6_PKTINFO;
		cm->cmsg_len=CMSG_LEN(sizeof(struct in6_pktinfo));
		i6=(struct in6_pktinfo *)CMSG_DATA(cm);
		memset(i6,0,sizeof(struct in6_pktinfo));
		i6->ipi6_addr=upnp->devaddr[which].a6;
	}
}

static int sendv3(UPNP *upnp,int s,int dst)
{
	struct sockaddr_in a4;
	union
	{
#pragma pack(push,1)
		unsigned char q1[12];
		unsigned short q2[6];
		unsigned int q4[3];
#pragma pack(pop)
	} u;
	MSGH msgh;

	memset(&a4,0,sizeof(a4));
	a4.sin_family=AF_INET;
	a4.sin_addr.s_addr=dst?dst:mchosts[SATIP_V4_ONLY].a4.s_addr;
	a4.sin_port=IPPROTO_IGMP;

	mkmsgh(upnp,&msgh,SATIP_V4_ONLY);
	msgh.mh.msg_name=&a4;
	msgh.io.iov_base=&u;
	msgh.io.iov_len=sizeof(u);

	memset(&u,0,sizeof(u));
	u.q1[0]=0x11;
	u.q1[1]=dst?V3FINAL*10:V3WAIT*10;
	if(dst)u.q4[1]=dst;
	u.q1[8]=V3ROBUST;
	u.q1[9]=V3TIME;
	u.q2[1]=chksum(u.q2,6);

	return sendmsg(s,&msgh.mh,0)==sizeof(u)?0:-1;
}

static int sendm2(UPNP *upnp,int s,void *dst)
{
	int t;
	struct sockaddr_in6 a6;
	unsigned char bfr[68];
	MSGH msgh;

	memset(bfr,0,sizeof(bfr));
	memset(&a6,0,sizeof(a6));

	a6.sin6_family=AF_INET6;

	if(!memcmp(dst,in6addr_any.s6_addr,16))
	{
		t=V3WAIT*1000;
		memcpy(a6.sin6_addr.s6_addr,mchosts[SATIP_V6_LINK].a6.s6_addr,
			16);
	}
	else
	{
		t=V3FINAL*1000;
		memcpy(a6.sin6_addr.s6_addr,dst,16);
		memcpy(bfr+48,dst,16);
	}

	mkmsgh(upnp,&msgh,SATIP_V6_LINK);
	msgh.mh.msg_name=&a6;
	msgh.io.iov_base=bfr+40;
	msgh.io.iov_len=sizeof(bfr)-40;

	memcpy(bfr,upnp->devaddr[SATIP_V6_LINK].a6.s6_addr,16);
	memcpy(bfr+16,mchosts[SATIP_V6_LINK].a6.s6_addr,16);
	bfr[35]=sizeof(bfr)-40;
	bfr[39]=IPPROTO_ICMPV6;
	bfr[40]=130;
	bfr[44]=t>>8;
	bfr[45]=t&0xff;
	bfr[64]=V3ROBUST;
	bfr[65]=V3TIME;

	*((unsigned short *)(bfr+42))=
		chksum((unsigned short *)bfr,sizeof(bfr)>>1);

	return sendmsg(s,&msgh.mh,0)==sizeof(bfr)-40?0:-1;
}

static MSGDATA *msgsplit(char *data)
{
	char *mem;
	char *ptr;
	int cnt;
	MSGDATA *msg;

	if(!(msg=malloc(sizeof(MSGDATA))))goto err1;

	for(cnt=0,ptr=strtok_r(data,"\r\n",&mem);ptr&&*ptr&&cnt<=20;
		ptr=strtok_r(NULL,"\r\n",&mem),cnt++)
	{
		if(!cnt)
		{
			if(!(msg->hdr=strdup(ptr)))goto err2;
			continue;
		}

		for(data=ptr;*data&&*data!=':';data++);
		if(!*data)goto err3;
		*data++=0;
		for(;*data==' ';data++);

		if(!(msg->name[cnt-1]=strdup(ptr)))goto err3;
		if(!(msg->value[cnt-1]=strdup(data)))goto err3;

		for(ptr=msg->name[cnt-1];*ptr;ptr++)if(*ptr>='A'&&*ptr<='Z')
			*ptr|=0x20;
	}

	if(!cnt)goto err2;

	msg->total=cnt-1;
	return msg;

err3:	while(cnt--)
	{
		if(msg->name[cnt])free(msg->name[cnt]);
		if(msg->value[cnt])free(msg->value[cnt]);
	}
	free(msg->hdr);
err2:	free(msg);
err1:	return NULL;
}

static int announce(UPNP *upnp,int which)
{
	int l;
	char bfr[512];
	char portinfo[64];
	char port[8];
	char addr[INET6_ADDRSTRLEN+2];
	char mcast[INET6_ADDRSTRLEN+2];
	SOCK a;
	MSGH msgh;

	addr2sock(&a,&mcaddr[which],htons(MPORT),upnp->devidx);
	mkmsgh(upnp,&msgh,which);
	msgh.mh.msg_name=&a;
	msgh.io.iov_base=bfr;

	addr2bstr(&upnp->devaddr[which],addr,sizeof(addr));
	addr2bstr(&mcaddr[which],mcast,sizeof(mcast));


	if(upnp->port==MPORT)*portinfo=0;
	else sprintf(portinfo,"SEARCHPORT.UPNP.ORG: %d\r\n",upnp->port);

	if(upnp->http==SATIP_HTTP_PORT)*port=0;
	else sprintf(port,":%u",upnp->http);

	l=sprintf(bfr,"NOTIFY * HTTP/1.1\r\n"
		"HOST %s:1900\r\n"
		"CACHE-CONTROL: max-age=%d\r\n"
		"LOCATION: http://%s%s/desc.xml\r\n"
		"NT: upnp:rootdevice\r\n"
		"NTS: ssdp:alive\r\n"
		"SERVER: Linux/1.0 UPnP/1.1 " IDENT "\r\n"
		"USN: uuid:%s::upnp:rootdevice\r\n"
		"BOOTID.UPNP.ORG: %u\r\n"
		"CONFIGID.UPNP.ORG: 0\r\n"
		"%s"
		"DEVICEID.SES.COM: %d\r\n"
		"\r\n",
		mcast,upnp->age,addr,port,upnp->uuid,
		upnp->bootcnt,portinfo,upnp->devid);

	msgh.io.iov_len=l;
	if(sendmsg(upnp->mdp[which],&msgh.mh,0)!=l)return -1;

	l=sprintf(bfr,"NOTIFY * HTTP/1.1\r\n"
		"HOST %s:1900\r\n"
		"CACHE-CONTROL: max-age=%d\r\n"
		"LOCATION: http://%s%s/desc.xml\r\n"
		"NT: uuid:%s\r\n"
		"NTS: ssdp:alive\r\n"
		"SERVER: Linux/1.0 UPnP/1.1 " IDENT "\r\n"
		"USN: uuid:%s\r\n"
		"BOOTID.UPNP.ORG: %u\r\n"
		"CONFIGID.UPNP.ORG: 0\r\n"
		"%s"
		"DEVICEID.SES.COM: %d\r\n"
		"\r\n",
		mcast,upnp->age,addr,port,upnp->uuid,
		upnp->uuid,upnp->bootcnt,portinfo,upnp->devid);

	msgh.io.iov_len=l;
	if(sendmsg(upnp->mdp[which],&msgh.mh,0)!=l)return -1;

	l=sprintf(bfr,"NOTIFY * HTTP/1.1\r\n"
		"HOST %s:1900\r\n"
		"CACHE-CONTROL: max-age=%d\r\n"
		"LOCATION: http://%s%s/desc.xml\r\n"
		"NT: urn:ses-com:device:SatIPServer:1\r\n"
		"NTS: ssdp:alive\r\n"
		"SERVER: Linux/1.0 UPnP/1.1 " IDENT "\r\n"
		"USN: uuid:%s::urn:ses-com:device:SatIPServer:1\r\n"
		"BOOTID.UPNP.ORG: %u\r\n"
		"CONFIGID.UPNP.ORG: 0\r\n"
		"%s"
		"DEVICEID.SES.COM: %d\r\n"
		"\r\n",
		mcast,upnp->age,addr,port,upnp->uuid,
		upnp->bootcnt,portinfo,upnp->devid);

	msgh.io.iov_len=l;
	if(sendmsg(upnp->mdp[which],&msgh.mh,0)!=l)return -1;

	return 0;
}

static void release(UPNP *upnp,int which)
{
	int l;
	char bfr[512];
	char mcast[INET6_ADDRSTRLEN+2];
	SOCK a;
	MSGH msgh;

	addr2sock(&a,&mcaddr[which],htons(MPORT),upnp->devidx);
	mkmsgh(upnp,&msgh,which);
	msgh.mh.msg_name=&a;
	msgh.io.iov_base=bfr;

	addr2bstr(&mcaddr[which],mcast,sizeof(mcast));

	l=sprintf(bfr,"NOTIFY * HTTP/1.1\r\n"
		"HOST %s:1900\r\n"
		"NT: upnp:rootdevice\r\n"
		"NTS: ssdp:byebye\r\n"
		"USN: uuid:%s::upnp:rootdevice\r\n"
		"BOOTID.UPNP.ORG: %u\r\n"
		"CONFIGID.UPNP.ORG: 0\r\n"
		"\r\n",
		mcast,upnp->uuid,upnp->bootcnt);

	msgh.io.iov_len=l;
	sendmsg(upnp->mdp[which],&msgh.mh,0);

	l=sprintf(bfr,"NOTIFY * HTTP/1.1\r\n"
		"HOST %s:1900\r\n"
		"NT: uuid:%s\r\n"
		"NTS: ssdp:byebye\r\n"
		"USN: uuid:%s\r\n"
		"BOOTID.UPNP.ORG: %u\r\n"
		"CONFIGID.UPNP.ORG: 0\r\n"
		"\r\n",
		mcast,upnp->uuid,upnp->uuid,upnp->bootcnt);

	msgh.io.iov_len=l;
	sendmsg(upnp->mdp[which],&msgh.mh,0);

	l=sprintf(bfr,"NOTIFY * HTTP/1.1\r\n"
		"HOST %s:1900\r\n"
		"NT: urn:ses-com:device:SatIPServer:1\r\n"
		"NTS: ssdp:byebye\r\n"
		"USN: uuid:%s::urn:ses-com:device:SatIPServer:1\r\n"
		"BOOTID.UPNP.ORG: %u\r\n"
		"CONFIGID.UPNP.ORG: 0\r\n"
		"\r\n",
		mcast,upnp->uuid,upnp->bootcnt);

	msgh.io.iov_len=l;
	sendmsg(upnp->mdp[which],&msgh.mh,0);
}

static int sendcoll(UPNP *upnp,COLL *c)
{
	int l;
	char bfr[512];
	char addr[INET6_ADDRSTRLEN+2];
	SOCK a;
	MSGH msgh;

	addr2sock(&a,&c->addr,c->port,upnp->devidx);
	mkmsgh(upnp,&msgh,c->which);
	msgh.mh.msg_name=&a;
	msgh.io.iov_base=bfr;

	addr2bstr(&upnp->devaddr[c->which],addr,sizeof(addr));

	l=sprintf(bfr,"M-SEARCH * HTTP/1.1\r\n"
		"HOST %s:%u\r\n"
		"MAN: \"ssdp:discover\"\r\n"
		"ST: urn:ses-com:device:SatIPServer:1\r\n"
		"USER-AGENT: Linux/1.0 UPnP/1.1 " IDENT "\r\n"
		"DEVICEID.SES.COM: %d\r\n"
		"\r\n",
		addr,upnp->port,upnp->devid);

	c->cnt--;

	msgh.io.iov_len=l;
	return sendmsg(upnp->udp[c->which],&msgh.mh,0)==l?0:-1;
}

static int sendok(UPNP *upnp,ADDR *a,short port,int which)
{
	int l;
	time_t t;
	struct tm tm;
	char bfr[512];
	char portinfo[64];
	char hport[8];
	char datim[64];
	char addr[INET6_ADDRSTRLEN+2];
	SOCK as;
	MSGH msgh;

	addr2sock(&as,a,port,upnp->devidx);
	mkmsgh(upnp,&msgh,which);
	msgh.mh.msg_name=&as;
	msgh.io.iov_base=bfr;

	addr2bstr(&upnp->devaddr[which],addr,sizeof(addr));

	if(upnp->http==SATIP_HTTP_PORT)*hport=0;
	else sprintf(hport,":%u",upnp->http);

	if(upnp->port==MPORT)*portinfo=0;
	else sprintf(portinfo,"SEARCHPORT.UPNP.ORG: %d\r\n",upnp->port);

	t=time(NULL);
	gmtime_r(&t,&tm);
	strftime(datim,sizeof(datim),"%a %b %e %H:%M:%S %Y",&tm);

	l=sprintf(bfr,"HTTP/1.1 200 OK\r\n"
		"CACHE-CONTROL: max-age=%d\r\n"
		"DATE: %s\r\n"
		"EXT:\r\n"
		"LOCATION: http://%s%s/desc.xml\r\n"
		"SERVER: Linux/1.0 UPnP/1.1 " IDENT "\r\n"
		"ST: urn:ses-com:device:SatIPServer:1\r\n"
		"USN: uuid:%s::urn:ses-com:device:SatIPServer:1\r\n"
		"BOOTID.UPNP.ORG: %u\r\n"
		"CONFIGID.UPNP.ORG: 0\r\n"
		"%s"
		"DEVICEID.SES.COM: %d\r\n"
		"\r\n",
		upnp->age,datim,addr,hport,upnp->uuid,
		upnp->bootcnt,portinfo,upnp->devid);

	msgh.io.iov_len=l;
	return sendmsg(upnp->udp[which],&msgh.mh,0)==l?0:-1;
}

static int streamhdr(int client,int mode)
{
	int l;
	time_t t;
	char *p;
	struct tm tm;
	char bfr[512];
	char datim[64];

	switch(mode)
	{
	case 1:	p="application/octet-stream";
		break;
	case 2:	p="text/plain; charset=utf-8";
		break;
	default:p="video/MP2T";
		break;
	}

	t=time(NULL);
	gmtime_r(&t,&tm);
	strftime(datim,sizeof(datim),"%a, %d %b %Y %H:%M:%S GMT",&tm);
	l=sprintf(bfr,"HTTP/1.1 200 OK\r\n"
		"Date: %s\r\n"
		"Connection: close\r\n"
		"Server: " SERVER "\r\n"
		"Content-Type: %s\r\n"
		"\r\n",datim,p);
	if(write(client,bfr,l)!=l)return -1;
	return 0;
}

static void httperr(int client,int code)
{
	time_t t;
	char *reason;
	struct tm tm;
	struct iovec io[2];
	char bfr[512];
	char datim[64];

	switch(code)
	{
	case 400:
		reason="400 Bad Request";
		io[1].iov_base="<html><body>"
			"The request could not be understood by the server "
			"due to malformed syntax."
			"</body></html>";
		break;
	case 403:
		reason="403 Forbidden";
		io[1].iov_base="<html><body>"
			"The server understood the request, but is refusing to "
			"fulfill it."
			"</body></html>";
		break;
	case 404:
		reason="404 Not Found";
		io[1].iov_base="<html><body>"
			"The server has not found anything matching the "
			"Request-URI.</body></html>";
		break;
	case 405:
		reason="405 Method Not Allowed";
		io[1].iov_base="<html><body>"
			"The method specified in the Request-Line is not "
			"allowed for the resource identified by the "
			"Request-URI.</body></html>";
		break;
	case 408:
		reason="408 Request Timeout";
		io[1].iov_base="<html><body>"
			"The client did not produce a request within the time "
			"that the server was prepared to wait."
			"</body></html>";
		break;
	case 503:
		reason="503 Service Unavailable";
		io[1].iov_base="<html><body>"
			"The server is currently unable to handle the request."
			"</body></html>";
		break;
	default:reason="500 Internal Server Error";
		io[1].iov_base="<html><body>"
			"The server encountered an unexpected condition which "
			"prevented it from fulfilling the request."
			"</body></html>";

		break;
	}

	t=time(NULL);
	gmtime_r(&t,&tm);
	strftime(datim,sizeof(datim),"%a, %d %b %Y %H:%M:%S GMT",&tm);
	io[1].iov_len=strlen(io[1].iov_base);
	io[0].iov_base=bfr;
	io[0].iov_len=sprintf(bfr,"HTTP/1.1 %s\r\n"
		"Date: %s\r\n"
		"Connection: close\r\n"
		"Server: " SERVER "\r\n"
		"Content-Length: %zd\r\n"
		"Content-Type: text/html; charset=utf-8\r\n"
		"\r\n",reason,datim,io[1].iov_len);
	t=writev(client,io,2);
}

static void sendimg(CLNDTA *w,int type,char *mime)
{
	SATIP_DATA d;
	time_t t;
	struct tm tm;
	struct iovec io[2];
	char bfr[512];
	char datim[64];

	d.intval=0;
	d.ptrval=NULL;
	if(w->upnp->cb(type,&d,w->upnp->priv))
	{
		httperr(w->client,404);
		return;
	}

	t=time(NULL);
	gmtime_r(&t,&tm);
	strftime(datim,sizeof(datim),"%a, %d %b %Y %H:%M:%S GMT",&tm);
	io[0].iov_base=bfr;
	io[0].iov_len=sprintf(bfr,"HTTP/1.1 200 OK\r\n"
		"Date: %s\r\n"
		"Connection: close\r\n"
		"Server: " SERVER "\r\n"
		"Content-Length: %d\r\n"
		"Content-Type: image/%s\r\n"
		"\r\n",datim,d.intval,mime);
	io[1].iov_base=d.ptrval;
	io[1].iov_len=d.intval;
	t=writev(w->client,io,2);
}

static void sendxml(CLNDTA *w)
{
	time_t t;
	struct tm tm;
	struct iovec io[2];
	char bfr[512];
	char datim[64];

	io[1].iov_base=w->upnp->xml;
	io[1].iov_len=w->upnp->xmllen;
	t=time(NULL);
	gmtime_r(&t,&tm);
	strftime(datim,sizeof(datim),"%a, %d %b %Y %H:%M:%S GMT",&tm);
	io[0].iov_base=bfr;
	io[0].iov_len=sprintf(bfr,"HTTP/1.1 200 OK\r\n"
		"Date: %s\r\n"
		"Connection: close\r\n"
		"Server: " SERVER "\r\n"
		"Content-Length: %zd\r\n"
		"Content-Type: text/xml; charset=%s\r\n"
		"\r\n",datim,io[1].iov_len,
		w->upnp->xmlcharset[0]?w->upnp->xmlcharset:"utf-8");
	t=writev(w->client,io,2);
}

static void sendlst(CLNDTA *w)
{
	int l;
	time_t t;
	struct tm tm;
	SATIP_DATA m3u[2];
	struct pollfd p;
	struct iovec io[2];
	char bfr[512];
	char datim[64];

	m3u[0].intval=w->which;
	m3u[0].ptrval=NULL;
	m3u[1].intval=0;
	m3u[1].ptrval=NULL;
	if(w->upnp->cb(SATIP_COPYM3U,&m3u,w->upnp->priv))
	{
		httperr(w->client,404);
		return;
	}

	io[1].iov_len=m3u[0].intval;
	io[1].iov_base=m3u[0].ptrval;
	t=time(NULL);
	gmtime_r(&t,&tm);
	strftime(datim,sizeof(datim),"%a, %d %b %Y %H:%M:%S GMT",&tm);
	io[0].iov_base=bfr;
	io[0].iov_len=snprintf(bfr,sizeof(bfr),"HTTP/1.1 200 OK\r\n"
		"Date: %s\r\n"
		"Connection: close\r\n"
		"Server: " SERVER "\r\n"
		"Content-Length: %zd\r\n"
		"Content-Type: audio/mpegurl; charset=%s\r\n"
		"\r\n",datim,io[1].iov_len,
			m3u[1].ptrval?(char *)m3u[1].ptrval:"utf-8");
	if(m3u[1].ptrval)free(m3u[1].ptrval);

	if((l=writev(w->client,io,2))==io[0].iov_len+io[1].iov_len)goto out;
	else if(l<io[0].iov_len)goto out;

	l-=io[0].iov_len;
	io[1].iov_len-=l;
	io[1].iov_base+=l;
	p.fd=w->client;
	p.events=POLLOUT;

	while(io[1].iov_len)
	{
		if(poll(&p,1,1000)!=1)goto out;
		if(!(p.revents&POLLOUT))goto out;
		if((l=write(w->client,io[1].iov_base,io[1].iov_len))<512)
			goto out;
		io[1].iov_len-=l;
		io[1].iov_base+=l;
	}

out:	free(m3u[0].ptrval);
}

static int rtsperr(int fd,int code,char *cseq,char *param)
{
	int len;
	char *msg;
	char *extra=NULL;
	char *body=NULL;
	char bmsg[512];
	char data[1024];

	switch(code)
	{
	case 400:
		msg="Bad Request";
		if(param)body="Check-Syntax";
		break;
	case 403:
		msg="Forbidden";
		if(param)body="Out-of-Range";
		break;
	case 404:
		msg="Not Found";
		break;
	case 405:
		msg="Method Not Allowed";
		if(param)extra="Allow";
		break;
	case 406:
		msg="Not Acceptable";
		break;
	case 408:
		msg="Request Timeout";
		break;
	case 414:
		msg="Request-URI Too Long";
		break;
	case 453:
		msg="Not Enough Bandwidth";
		break;
	case 454:
		msg="Session Not Found";
		break;
	case 455:
		msg="Method Not Valid in This State";
		break;
	case 461:
		msg="Unsupported Transport";
		break;
	case 501:
		msg="Not Implemented";
		extra="Public";
		param="OPTIONS, DESCRIBE, SETUP, PLAY, TEARDOWN";
		break;
	case 503:
		msg="Service Unavailable";
		if(param)body="No-More";
		break;
	case 505:
		msg="RTSP Version Not Supported";
		break;
	case 551:
		msg="Option Not Supported";
		if(param)
		{
			extra="Unsupported";
			break;
		}
	default:code=500;
		msg="Internal Server Error";
		break;
	}

	if(body)
	{
		snprintf(bmsg,sizeof(bmsg),"%s: %s",body,param);
		len=snprintf(data,sizeof(data),"RTSP/1.0 %d %s\r\nCseq: %s\r\n"
			"Content-Type: text/parameters\r\nContent-Length: %zd"
			"\r\n\r\n%s",code,msg,cseq,strlen(bmsg),bmsg);
	}
	else
	{
		if(extra)snprintf(bmsg,sizeof(bmsg),"%s: %s\r\n",extra,param);
		else *bmsg=0;
		len=snprintf(data,sizeof(data),"RTSP/1.0 %d %s\r\nCseq: %s\r\n"
			"%s\r\n",code,msg,cseq,bmsg);
	}

	if(write(fd,data,len)!=len)return -1;
	return 0;
}

static int rtspok(int fd,int flags,REQUEST *qry,ADDR *addr,int port,int val,
	int timeout,char *cseq,char *sess,void *ptr)
{
	int l;
	unsigned long tme;
	char *trans;
	unsigned short *seq;
	struct timespec ts;
	char smsg[128];
	char bmsg[512];
	char lmsg[128];
	char tmsg[128];
	char imsg[128];
	char pmsg[128];
	char data[1024];

	if(flags&HDR_BASE)
	{
		if(port!=SATIP_RTSP_PORT)sprintf(lmsg,":%d",port);
		else *lmsg=0;
		addr2bstr(addr,smsg,sizeof(smsg));
		if(qry&&(flags&HDR_QUERY))
		{
			if(satip_util_create(SATIP_TYPE_QRY,
				(flags&HDR_STRICT)?SATIP_STRICTQRY:0,NULL,0,0,
				&qry->tune,&qry->set,NULL,NULL,NULL,data,
				sizeof(data))<0)*data=0;
			snprintf(bmsg,sizeof(bmsg),"Content-Base: rtsp://%s%s/"
				"%s\r\n",smsg,lmsg,data);
		}
		else snprintf(bmsg,sizeof(bmsg),"Content-Base: rtsp://%s%s/"
			"\r\n",smsg,lmsg);
	}
	else *bmsg=0;

	if(flags&HDR_INFO)
	{
		trans=NULL;
		seq=ptr;
		if(port!=SATIP_RTSP_PORT)sprintf(data,":%d",port);
		else *data=0;
		addr2bstr(addr,smsg,sizeof(smsg));
		if(seq)
		{
			if(clock_gettime(CLOCK_MONOTONIC_RAW,&ts))snprintf(pmsg,
				sizeof(pmsg),"RTP-Info: url=rtsp://%s%s/stream="
				"%d;seq=%u\r\n",smsg,data,val,*seq);
			else
			{
        			tme=((unsigned long)ts.tv_sec)*10000UL+
					(unsigned long)(ts.tv_nsec/100000);
				snprintf(pmsg,sizeof(pmsg),"RTP-Info: "
					"url=rtsp://%s%s/stream=%d;seq=%u;"
					"rtptime=%u\r\n",smsg,data,val,*seq,
					(unsigned int)(tme*9UL)+(unsigned int)
					(ts.tv_nsec%100000/11112));
			}
		}
		else snprintf(pmsg,sizeof(pmsg),
			"RTP-Info: url=rtsp://%s%s/stream=%d\r\n",
				smsg,data,val);
	}
	else
	{
		*pmsg=0;
		trans=ptr;
	}

	if(flags&HDR_LEN)snprintf(lmsg,sizeof(lmsg),"Content-Length: %d\r\n",
		val);
	else *lmsg=0;

	if(flags&HDR_SID)snprintf(imsg,sizeof(imsg),"com.ses.streamID: %d\r\n",
		val);
	else *imsg=0;

	if(sess)
	{
		if(flags&HDR_TMOXX)snprintf(smsg,sizeof(smsg),
			"Session: %s;timeout=%d\r\n",sess,timeout);
		else if(flags&HDR_TMO0)snprintf(smsg,sizeof(smsg),
			"Session: %s;timeout=0\r\n",sess);
		else snprintf(smsg,sizeof(smsg),"Session: %s\r\n",sess);
	}
	else *smsg=0;

	if(trans)snprintf(tmsg,sizeof(tmsg),"Transport: %s\r\n",trans);
	else *tmsg=0;

	l=snprintf(data,sizeof(data),
	    "RTSP/1.0 200 OK\r\nCseq: %s\r\n%s%s%s%s%s%s%s%s\r\n",cseq,smsg,
	    (flags&HDR_PUBLIC)?"Public: OPTIONS, DESCRIBE, SETUP, PLAY, "
		"TEARDOWN\r\n":"",
	    (flags&HDR_TYPE)?"Content-Type: application/sdp\r\n":"",
	    bmsg,lmsg,tmsg,imsg,pmsg);

	if(write(fd,data,l)!=l)return -1;
	return 0;
}

static int rtsp_transport(char *s,TRANSPORT *t,ADDR *peer,int level,int strict)
{
	int rtpavp=0;
	long val;
	char *item;
	char *mem;
	char *end;

	memset(t,0,sizeof(TRANSPORT));

	for(item=strtok_r(s,";",&mem);item;item=strtok_r(NULL,";",&mem))
	{
		if(!strict&&!strcmp(item,"RTP/AVP/UDP"))rtpavp=1;
		else if(!strcmp(item,"RTP/AVP"))rtpavp=1;
		else if(!strcmp(item,"unicast"))
		{
			if(!rtpavp||t->mode)return -1;
			t->mode=SATIP_TYPE_RTSP;
		}
		else if(!strcmp(item,"multicast"))
		{
			if(!rtpavp||t->mode)return -1;
			t->mode=SATIP_TYPE_RTP;
		}
		else if(!strncmp(item,"client_port=",12))
		{
			if(t->mode!=SATIP_TYPE_RTSP||t->rtp)return -1;
			item+=12;
			val=strtol(item,&end,10);
			if(end==item||*end!='-'||val<1||val>65535||(val&1))
				return 1;
			t->rtp=val;
			val=strtol(end+1,&end,10);
			if(end==item||*end||val<1||val>65535||val!=t->rtp+1)
				return 1;
			t->rtcp=val;
		}
		else if(!strncmp(item,"port=",5))
		{
			if(t->mode!=SATIP_TYPE_RTP||t->rtp)return -1;
			item+=5;
			val=strtol(item,&end,10);
			if(end==item||*end!='-'||val<1||val>65535||(val&1))
				return 1;
			t->rtp=val;
			val=strtol(end+1,&end,10);
			if(end==item||*end||val<1||val>65535||val!=t->rtp+1)
				return 1;
			t->rtcp=val;
		}
		else if(!strncmp(item,"ttl=",4))
		{
			if(t->mode!=SATIP_TYPE_RTP||t->ttl)return -1;
			item+=4;
			val=strtol(item,&end,10);
			if(end==item||*end||val<1||val>255)return 1;
			t->ttl=val;
		}
		else if(!strncmp(item,"destination=",12))
		{
			if(t->mode!=SATIP_TYPE_RTP||t->addr.family)return -1;
			item+=12;
			if(str2addr(item,&t->addr))return -1;
			if(invalid_mcast(&t->addr)==-1)return -1;
		}
		else return -1;
	}

	if(!t->mode)return -1;
	if(t->rtp&&!t->rtcp)return -1;
	if(t->mode==SATIP_TYPE_RTSP&&(!t->rtp||!t->rtcp))return -1;
	if(t->mode==SATIP_TYPE_RTP&&t->addr.family)
		switch((val=addr2level(&t->addr)))
	{
	case SATIP_V6_LINK:
		if(t->ttl>1)return -1;
	default:if(val<=level)break;
	case -1:return -1;
	}
	if(!t->addr.family&&t->mode==SATIP_TYPE_RTSP)t->addr=*peer;

	return 0;
}

static SESSION *lookup_sid(UPNP *upnp,int sid)
{
	SESSION *s;

	if(sid<=0||sid>65025)return NULL;
	if(!(upnp->sidmap[sid>>5]&(1<<(sid&0x1f))))return NULL;
	for(s=upnp->slist;s;s=s->next)if(s->master->sid==sid)return s->master;
	return NULL;
}

static int alloc_sid(UPNP *upnp)
{
	int i;
	int j;

	for(i=0;i<2033;i++)if(upnp->sidmap[i]!=0xffffffff)
	{
		for(j=0;j<31;j++)if(!(upnp->sidmap[i]&(1<<j)))break;
		upnp->sidmap[i]|=1<<j;
		return (i<<5)|j;
	}
	return -1;
}

static void free_sid(UPNP *upnp,int sid)
{
	if(sid<=0||sid>65025)return;
	upnp->sidmap[sid>>5]&=~(1<<(sid&0x1f));
}

static SESSION *lookup_mcast(UPNP *upnp,ADDR *addr)
{
	SESSION *s;

	for(s=upnp->mcast[addr2idx(addr)];s;s=s->next)if(!ascmp(addr,&s->rtp))
		return s;
	return NULL;
}

static void add_mcast(UPNP *upnp,SESSION *s)
{
	int idx=sock2idx(&s->rtp);

	s->mcast=upnp->mcast[idx];
	upnp->mcast[idx]=s;
}

static void del_mcast(UPNP *upnp,SESSION *s)
{
	int idx=sock2idx(&s->rtp);
	SESSION **d;

	for(d=&upnp->mcast[idx];*d;d=&(*d)->mcast)if(*d==s)
	{
		*d=s->mcast;
		break;
	}
}

static int pidcmp(SATIP_PIDS *s1,SATIP_PIDS *s2)
{
	int i;

	if(s1->numpids!=s2->numpids)return -1;
	else if(s1->numpids==SATIP_SIGNAL)return 0;
	else if(s1->numpids==SATIP_SECTION)
	{
		if(s1->pid!=s2->pid||s1->table!=s2->table)return -1;
	}
	else
	{
		for(i=0;i<s1->numpids;i++)if(s1->pids[i]!=s2->pids[i])return -1;
		if(s1->prognum!=s2->prognum)return -1;
	}
	return 0;
}

static int tunecmp(SATIP_TUNE *t1,SATIP_TUNE *t2)
{
	if(t1->msys!=t2->msys)return -1;

	switch(t1->msys)
	{
	case SATIP_DVBS2:
		if(t1->ro!=t2->ro&&t1->ro!=SATIP_ROFF_AUTO)return -1;
		if(t1->mtype!=t2->mtype&&t1->mtype!=SATIP_AUTOQ)return -1;
		if(t1->plts!=t2->plts&&t1->plts!=SATIP_PLTS_AUTO)return -1;
	case SATIP_DVBS:
		if(t1->fe!=t2->fe&&t1->fe)return -1;
		if(t1->src!=t2->src)return -1;
		if(t1->freq!=t2->freq)return -1;
		if(t1->pol!=t2->pol)return -1;
		if(t1->sr!=t2->sr)return -1;
		if(t1->fec!=t2->fec&&t1->fec!=SATIP_FEC_AUTO)return -1;
		break;

	case SATIP_DVBT2:
		if(t1->plp!=t2->plp&&t1->plp!=SATIP_UNDEF)return -1;
		if(t1->t2id!=t2->t2id&&t1->t2id!=SATIP_UNDEF)return -1;
		if(t1->sm!=t2->sm&&t1->sm!=SATIP_SM_AUTO)return -1;
	case SATIP_DVBT:
		if(t1->freq!=t2->freq)return -1;
		if(t1->bw!=t2->bw&&t1->bw!=SATIP_BW_AUTO)return -1;
		if(t1->tmode!=t2->tmode&&t1->tmode!=SATIP_TMOD_AUTO)return -1;
		if(t1->mtype!=t2->mtype&&t1->mtype!=SATIP_AUTOQ)return -1;
		if(t1->gi!=t2->gi&&t1->gi!=SATIP_GI_AUTO)return -1;
		if(t1->fec!=t2->fec&&t1->fec!=SATIP_FEC_AUTO)return -1;
		break;

	case SATIP_DVBC2:
		if(t1->freq!=t2->freq)return -1;
		if(t1->c2tft!=t2->c2tft&&t1->c2tft!=SATIP_TFT_AUTO)return -1;
		if(t1->bw!=t2->bw&&t1->bw!=SATIP_BW_AUTO)return -1;
		if(t1->ds!=t2->ds&&t1->ds!=SATIP_UNDEF)return -1;
		if(t1->plp!=t2->plp&&t1->plp!=SATIP_UNDEF)return -1;
		break;

	case SATIP_DVBC:
		if(t1->freq!=t2->freq)return -1;
		if(t1->mtype!=t2->mtype&&t1->mtype!=SATIP_AUTOQ)return -1;
		if(t1->sr!=t2->sr)return -1;
		if(t1->specinv!=t2->specinv&&t1->specinv!=SATIP_SPI_AUTO)
			return -1;
		break;

	default:return -1;
	}

	return 0;
}

static GROUP *lookup_stream(UPNP *upnp,SATIP_TUNE *tune,SATIP_PIDS *set)
{
	GROUP *g;

	for(g=upnp->glist;g;g=g->next)
	{
		if(pidcmp(set,&g->set))continue;
		if(tunecmp(tune,&g->tune))continue;
		return g;
	}
	return NULL;
}

static MEMBER *play_stream(UPNP *upnp,SATIP_TUNE *tune,SATIP_PIDS *set,void *h,
	int mode)
{
	GROUP *g;
	MEMBER *m;
	SATIP_STRDATA dta;

	if(!(m=malloc(sizeof(MEMBER))))goto err1;

	pthread_mutex_lock(&upnp->stx);

	if(!(g=lookup_stream(upnp,tune,set)))
	{
		if(!(g=malloc(sizeof(GROUP))))goto err2;
		if(pthread_spin_init(&g->dmtx,PTHREAD_PROCESS_PRIVATE))
			goto err3;
		if(pthread_spin_init(&g->smtx,PTHREAD_PROCESS_PRIVATE))
			goto err4;
		g->list=NULL;
		g->tune=*tune;
		g->rtune=*tune;
		g->set=*set;

		dta.tune=&g->rtune;
		dta.set=set;
		dta.handle=g;
		dta.terminate=upnp->sfd;
		if(upnp->cb(SATIP_STRPLAY,&dta,upnp->priv))goto err5;
		g->h=dta.handle;

		g->next=upnp->glist;
		upnp->glist=g;
	}

	m->group=g;
	m->h=h;
	m->mode=mode;
	m->notified=(!g->list)?0:1;

	pthread_spin_lock(&g->smtx);
	pthread_spin_lock(&g->dmtx);
	m->next=g->list;
	g->list=m;
	pthread_spin_unlock(&g->dmtx);
	pthread_spin_unlock(&g->smtx);

	*tune=g->rtune;

	pthread_mutex_unlock(&upnp->stx);

	return m;

err5:	pthread_spin_destroy(&g->smtx);
err4:	pthread_spin_destroy(&g->dmtx);
err3:	free(g);
err2:	pthread_mutex_unlock(&upnp->stx);
	free(m);
err1:	return NULL;
}

static int end_stream(UPNP *upnp,MEMBER *m)
{
	int r=0;
	MEMBER **dm;
	MEMBER *x=NULL;
	GROUP *g=m->group;
	GROUP **dg;
	SATIP_STRDATA dta;

	pthread_mutex_lock(&upnp->stx);

	pthread_spin_lock(&g->smtx);
	pthread_spin_lock(&g->dmtx);

	for(dm=&g->list;*dm;dm=&(*dm)->next)if(*dm==m)
	{
		*dm=m->next;
		x=m;
		break;
	}

	pthread_spin_unlock(&g->dmtx);
	pthread_spin_unlock(&g->smtx);

	if(x)free(x);
	else
	{
		r=-1;
		goto out;
	}

	if(!g->list)
	{
		dta.tune=NULL;
		dta.set=NULL;
		dta.terminate=upnp->sfd;
		dta.handle=g->h;
		r=upnp->cb(SATIP_STREND,&dta,upnp->priv);

		pthread_spin_destroy(&g->dmtx);
		pthread_spin_destroy(&g->smtx);

		for(dg=&upnp->glist;*dg;dg=&(*dg)->next)if(*dg==g)
		{
			*dg=g->next;
			free(g);
			break;
		}
	}

out:	pthread_mutex_unlock(&upnp->stx);

	return r;
}

static int change_stream(UPNP *upnp,SATIP_TUNE *tune,SATIP_PIDS *set,MEMBER *m,
	SATIP_STATUS *state)
{
	int r=-1;
	GROUP *g;
	GROUP **dg;
	MEMBER **dm;
	SATIP_STRDATA dta;

	pthread_mutex_lock(&upnp->stx);

	if(!tunecmp(tune,&m->group->tune))
	{
		if(!pidcmp(set,&m->group->set))r=0;
		else if(m->group->list==m&&!m->group->list->next)
		{
			dta.tune=NULL;
			dta.set=set;
			dta.handle=m->group->h;
			dta.terminate=upnp->sfd;
			if(!(r=upnp->cb(SATIP_STRPIDS,&dta,upnp->priv)))
				state->set=*set;
		}
		goto out;
	}

	if((g=lookup_stream(upnp,tune,set)))
	{
		pthread_spin_lock(&m->group->smtx);
		pthread_spin_lock(&m->group->dmtx);

		for(dm=&m->group->list;*dm;dm=&(*dm)->next)if(*dm==m)
		{
			*dm=m->next;
			r=0;
			break;
		}

		pthread_spin_unlock(&m->group->dmtx);
		pthread_spin_unlock(&m->group->smtx);

		if(r)goto out;

		if(!m->group->list)
		{
			dta.tune=NULL;
			dta.set=NULL;
			dta.terminate=upnp->sfd;
			dta.handle=m->group->h;
			if((r=upnp->cb(SATIP_STREND,&dta,upnp->priv)))
			{
				pthread_spin_lock(&m->group->smtx);
				pthread_spin_lock(&m->group->dmtx);

				m->next=m->group->list;
				m->group->list=m;

				pthread_spin_unlock(&m->group->dmtx);
				pthread_spin_unlock(&m->group->smtx);

				goto out;
			}

			pthread_spin_destroy(&m->group->dmtx);
			pthread_spin_destroy(&m->group->smtx);

			for(dg=&upnp->glist;*dg;dg=&(*dg)->next)
				if(*dg==m->group)
			{
				*dg=g->next;
				free(g);
				break;
			}
		}

		state->tune=g->rtune;
		state->set=*set;
		state->level=0;
		state->lock=0;
		state->quality=0;

		m->group=g;
		m->notified=0;

		pthread_spin_lock(&g->smtx);
		pthread_spin_lock(&g->dmtx);

		m->next=g->list;
		g->list=m;

		pthread_spin_unlock(&g->dmtx);
		pthread_spin_unlock(&g->smtx);
	}

out:	pthread_mutex_unlock(&upnp->stx);
	return r;
}

static SESSION *lookup_sess(UPNP *upnp,char *sess)
{
	unsigned long long id;
	char *end;
	SESSION *s;

	id=strtoull(sess,&end,16);
	if(end==sess||*end||!id)return NULL;

	for(s=upnp->slist;s;s=s->next)if(s->id==id)break;
	return s;
}

static int alloc_sess(UPNP *upnp,TRANSPORT *t,SESSION *m,SESSION **ss,int which)
{
	int i;
	unsigned short even;
	unsigned short odd;
	SESSION *s;
	SESSION *x;
	SATIP_DATA data;

	if(upnp->rtsplimit&&upnp->sessions>=upnp->rtsplimit)return -2;

	if(!(s=malloc(sizeof(SESSION)+upnp->burst*sizeof(SATIP_STREAM))))
		return -1;
	memset(s,0,sizeof(SESSION));
	do
	{
		data.intval=sizeof(s->ssrc);
		data.ptrval=&s->ssrc;
		if(upnp->cb(SATIP_GETRANDOM,&data,upnp->priv))
		{
			free(s);
			return -1;
		}
	} while(!s->ssrc);

	if(m)s->master=m;
	else
	{
		s->master=s;
		if(pthread_spin_init(&s->mtx,PTHREAD_PROCESS_PRIVATE))
		{
			free(s);
			return -1;
		}
	}
	do
	{
		data.intval=sizeof(s->id);
		data.ptrval=&s->id;
		if(upnp->cb(SATIP_GETRANDOM,&data,upnp->priv))
		{
			if(!m)pthread_spin_destroy(&s->mtx);
			free(s);
			return -1;
		}
		if(!s->id)x=(void *)1;
		else for(x=upnp->slist;x;x=x->next)if(x->id==s->id)break;
	}
	while(x);
	if(!m)
	{
		if((s->sid=alloc_sid(upnp))==-1)
		{
			pthread_spin_destroy(&s->mtx);
			free(s);
			return -2;
		}
	}
	if(t->mode==SATIP_TYPE_RTP)
	{
		if(!t->ttl)
		{
			if(!m)
			{
				if(which==SATIP_V6_LINK)t->ttl=1;
				else t->ttl=upnp->mttl;
			}
			else t->ttl=m->ttl;
		}
		if(!t->rtp)
		{
			if(!m)
			{
				if(!upnp->mdft)
				{
					data.intval=sizeof(even);
					data.ptrval=&even;
					if(upnp->cb(SATIP_GETRANDOM,&data,
						upnp->priv))
					{
						pthread_spin_destroy(&s->mtx);
						free_sid(upnp,s->sid);
						free(s);
						return -1;
					}
					if(upnp->portmin)
					{
						even%=
						    upnp->portmax-upnp->portmin;
						even&=0xfffe;
						even+=upnp->portmin;
					}
					else
					{
						even%=0xfc00;
						even&=0xfffe;
						even+=0x0400;
					}
					odd=even+1;
					t->rtp=even;
					t->rtcp=odd;
				}
				else
				{
					t->rtp=upnp->mdft;
					t->rtcp=upnp->mdft+1;
				}
			}
			else
			{
				t->rtp=ntohs(psget(&m->rtp));
				t->rtcp=ntohs(psget(&m->rtcp));
			}
		}
		if(!t->addr.family)
		{
			if(!m)autoaddr(which,s->sid,upnp->devid,&t->addr);
			else sock2addr(&t->addr,&m->rtp);
		}
		if(!m)
		{
			if(t->mode==SATIP_TYPE_RTP)
				if(lookup_mcast(upnp,&t->addr)||
					invalid_mcast(&t->addr))
			{
				pthread_spin_destroy(&s->mtx);
				free_sid(upnp,s->sid);
				free(s);
				return -3;
			}
			if(mcastpair(upnp,&t->addr,t->ttl,&s->rtpfd,&s->rtcpfd,
				&s->port,which))
			{
				pthread_spin_destroy(&s->mtx);
				free_sid(upnp,s->sid);
				free(s);
				return -1;
			}
			s->ttl=t->ttl;
			s->idle=-1;
		}
		else if(m->mode==SATIP_TYPE_RTP)
		{
			if(t->ttl!=m->ttl||ascmp(&t->addr,&m->rtp)||
				t->rtp!=ntohs(psget(&m->rtp))||
				t->rtcp!=ntohs(psget(&m->rtcp)))
			{
				free(s);
				return -3;
			}
			s->rtpfd=-1;
			s->rtcpfd=-1;
		}
		else
		{
			free(s);
			return -3;
		}
	}
	else if(udppair(upnp,&upnp->devaddr[which],&s->rtpfd,&s->rtcpfd,
		&s->port,which))
	{
		if(!m)
		{
			pthread_spin_destroy(&s->mtx);
			free_sid(upnp,s->sid);
		}
		free(s);
		return -1;
	}

	mkmsgh(upnp,&s->msgh1,which);
	s->msgh1.mh.msg_name=&s->rtp;

	mkmsgh(upnp,&s->msgh2,which);
	s->msgh2.mh.msg_name=&s->rtcp;

	memcpy(s->cmsg,s->msgh1.bfr,64);
	for(s->burst=upnp->burst,i=0;i<upnp->burst;i++)
	{
		s->msgqh[i].msg_hdr=s->msgh1.mh;
		s->msgqh[i].msg_hdr.msg_iov=&s->io[i];
		s->msgqh[i].msg_hdr.msg_control=s->cmsg;
		s->io[i].iov_base=s->queue[i].msg;
	}

	s->strict=upnp->strict;

	s->mode=t->mode;
	addr2sock(&s->rtp,&t->addr,htons(t->rtp),upnp->devidx);
	addr2sock(&s->rtcp,&t->addr,htons(t->rtcp),upnp->devidx);

	s->next=upnp->slist;
	upnp->slist=s;

	if(m)
	{
		pthread_spin_lock(&s->mtx);
		s->slave=m->slave;
		m->slave=s;
		pthread_spin_unlock(&s->mtx);
	}
	else if(t->mode==SATIP_TYPE_RTP)add_mcast(upnp,s);

	upnp->sessions++;
	upnp->allsessions++;
	*ss=s;

	return 0;
}

static int change_sess(UPNP *upnp,TRANSPORT *t,SESSION *s,int which)
{
	int rtpfd;
	int rtcpfd;
	unsigned short port;
	unsigned short even;
	unsigned short odd;
	SATIP_DATA data;

	if(t->mode==SATIP_TYPE_RTP)
	{
		if(!t->ttl)
		{
			if(s->master==s)
			{
				if(which==SATIP_V6_LINK)t->ttl=1;
				else t->ttl=upnp->mttl;
			}
			else t->ttl=s->master->ttl;
		}
		if(!t->rtp)
		{
			if(s->master==s)
			{
				if(!upnp->mdft)
				{
					data.intval=sizeof(even);
					data.ptrval=&even;
					if(upnp->cb(SATIP_GETRANDOM,&data,
						upnp->priv))return -1;
					if(upnp->portmin)
					{
						even%=
						    upnp->portmax-upnp->portmin;
						even&=0xfffe;
						even+=upnp->portmin;
					}
					else
					{
						even%=0xfc00;
						even&=0xfffe;
						even+=0x0400;
					}
					odd=even+1;
					t->rtp=even;
					t->rtcp=odd;
				}
				else
				{
					t->rtp=upnp->mdft;
					t->rtcp=upnp->mdft+1;
				}
			}
			else
			{
				t->rtp=ntohs(psget(&s->master->rtp));
				t->rtcp=ntohs(psget(&s->master->rtcp));
			}
		}
		if(!t->addr.family)
		{
			if(s->master==s)autoaddr(which,s->sid,upnp->devid,
				&t->addr);
			else sock2addr(&t->addr,&s->master->rtp);
		}
		switch(s->mode)
		{
		case SATIP_TYPE_RTP:
			if(!ascmp(&t->addr,&s->rtp))break;
		case SATIP_TYPE_RTSP:
			if(lookup_mcast(upnp,&t->addr))return -2;
			if(invalid_mcast(&t->addr))return -2;
		}
		if(s->master==s)
		{
			if(s->ttl!=t->ttl||s->mode==SATIP_TYPE_RTSP)
			{
				if(mcastpair(upnp,&t->addr,t->ttl,&rtpfd,
					&rtcpfd,&port,which))return -1;
				if(s->mode==SATIP_TYPE_RTP)del_mcast(upnp,s);
				close(s->rtpfd);
				close(s->rtcpfd);
				s->port=port;
				s->rtpfd=rtpfd;
				s->rtcpfd=rtcpfd;
				s->ttl=t->ttl;
			}
			else if(s->mode==SATIP_TYPE_RTP)del_mcast(upnp,s);
			s->idle=-1;
		}
		else if(s->master->mode==SATIP_TYPE_RTP)
		{
			if(t->ttl!=s->master->ttl||
				ascmp(&t->addr,&s->master->rtp)||
				t->rtp!=ntohs(psget(&s->master->rtp))||
				t->rtcp!=ntohs(psget(&s->master->rtcp)))
					return -2;
			if(s->rtpfd!=-1)
			{
				close(s->rtpfd);
				s->rtpfd=-1;
			}
			if(s->rtcpfd!=-1)
			{
				close(s->rtcpfd);
				s->rtcpfd=-1;
			}
			s->ttl=0;
			s->idle=0;
		}
		else return -2;
	}
	else
	{
		if(s->mode==SATIP_TYPE_RTP)
		{
			if(udppair(upnp,&upnp->devaddr[which],&rtpfd,&rtcpfd,
				&port,which))return -1;
			if(s->master==s)del_mcast(upnp,s);
			if(s->rtpfd!=-1)close(s->rtpfd);
			if(s->rtcpfd!=-1)close(s->rtcpfd);
			s->port=port;
			s->rtpfd=rtpfd;
			s->rtcpfd=rtcpfd;
			s->ttl=0;
		}
		s->idle=0;
	}

	s->mode=t->mode;
	addr2sock(&s->rtp,&t->addr,htons(t->rtp),upnp->devidx);
	addr2sock(&s->rtcp,&t->addr,htons(t->rtcp),upnp->devidx);

	if(s->mode==SATIP_TYPE_RTP&&s->master==s)add_mcast(upnp,s);

	return 0;
}

static int move_sess(UPNP *upnp,SESSION *nmst,SESSION *omst,SESSION *s)
{
	SESSION **d;

	pthread_spin_lock(&s->master->mtx);

	if(nmst==omst)goto out;

	if(s->play)
	{
		if(!nmst->play||!omst->play)goto err;
		s->play=0;
		upnp->playsessions--;
	}

	for(d=&omst->slave;*d;d=&(*d)->slave)if(*d==s)
	{
		*d=s->next;
		break;
	}

	s->master=nmst;
	s->slave=nmst;
	nmst->slave=s;

out:	pthread_spin_unlock(&s->master->mtx);
	return 0;

err:	pthread_spin_unlock(&s->master->mtx);
	return -1;
}

static void free_sess(UPNP *upnp,SESSION *s)
{
	SESSION **d;

	if(s!=s->master)
	{
		pthread_spin_lock(&s->master->mtx);

		for(d=&s->master->slave;*d;d=&(*d)->slave)if(*d==s)
		{
			*d=s->next;
			break;
		}

		pthread_spin_unlock(&s->master->mtx);

		for(d=&upnp->slist;*d;d=&(*d)->next)if(*d==s)
		{
			*d=s->next;
			break;
		}
	}
	else
	{
		if(s->mode==SATIP_TYPE_RTP)del_mcast(upnp,s);

		while(s->slave)free_sess(upnp,s->slave);

		for(d=&upnp->slist;*d;d=&(*d)->next)if(*d==s)
		{
			*d=s->next;
			break;
		}
	}

	if(s->rtpfd!=-1)close(s->rtpfd);
	if(s->rtcpfd!=-1)close(s->rtcpfd);
	if(s==s->master)
	{
		pthread_spin_destroy(&s->mtx);
		free_sid(upnp,s->sid);
	}

	if(s->play&PLAY_PLAYING)upnp->playsessions--;
	if(s->id)upnp->sessions--;
	upnp->allsessions--;

	free(s);
}

static void flush_sess(UPNP *upnp)
{
	SESSION *s;

	for(s=upnp->slist;s;s=s->next)if(s==s->master&&s->handle)
		end_stream(upnp,s->handle);
	while(upnp->slist)free_sess(upnp,upnp->slist);
}

static void flush_http(UPNP *upnp)
{
	STREAM *hs;
	int fd;

	pthread_mutex_lock(&upnp->htx);
	while(upnp->hlist)
	{
		hs=upnp->hlist;
		pthread_spin_lock(&hs->mtx);
		if(hs->fd!=-1)
		{
			fd=hs->fd;
			hs->fd=-1;
			pthread_spin_unlock(&hs->mtx);
			close(fd);
		}
		else pthread_spin_unlock(&hs->mtx);
		if(hs->handle)
		{
			end_stream(upnp,hs->handle);
			upnp->playhttp--;
		}
		upnp->hlist=hs->next;
		pthread_spin_destroy(&hs->mtx);
		free(hs);
	}
	pthread_mutex_unlock(&upnp->htx);
}

static void flush_rtsp(UPNP *upnp)
{
	RTSP *rtspdta;

	while(upnp->rlist)
	{
		rtspdta=upnp->rlist;
		upnp->rlist=rtspdta->next;
		epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,rtspdta->client,NULL);
		close(rtspdta->client);
		free(rtspdta);
	}
}

static int pidmerge(SATIP_PIDS *s,SATIP_PIDS *a,SATIP_PIDS *d,SATIP_PIDS *r)
{
	int i;
	int j;
	int k;
	SATIP_PIDS tmp;

	if(s->numpids==SATIP_ALLPIDS||s->numpids==SATIP_SECTION||
		s->numpids==SATIP_SIGNAL)return -1;

	if(s->numpids==SATIP_NOPIDS)
	{
		if(d->numpids)return -1;
		else tmp.numpids=0;
	}
	else
	{
		for(i=0,j=0,k=0;i<s->numpids&&j<d->numpids;)
		{
			if(s->pids[i]<d->pids[j])
			{
				tmp.pids[k++]=s->pids[i++];
				continue;
			}
			if(s->pids[i]>d->pids[j])return -1;
			i++;
			j++;
		}
		if(j<d->numpids)return -1;
		while(i<s->numpids)tmp.pids[k++]=s->pids[i++];
		tmp.numpids=k;
	}

	for(i=0,j=0,k=0;i<tmp.numpids&&j<a->numpids;)
	{
		if(k==SATIP_MAX_PIDS)return -1;
		if(tmp.pids[i]<a->pids[j])
		{
			r->pids[k++]=tmp.pids[i++];
			continue;
		}
		if(tmp.pids[i]>a->pids[j])
		{
			r->pids[k++]=a->pids[j++];
			continue;
		}
		return -1;
	}
	while(i<tmp.numpids)
	{
		if(k==SATIP_MAX_PIDS)return -1;
		r->pids[k++]=tmp.pids[i++];
	}
	while(j<a->numpids)
	{
		if(k==SATIP_MAX_PIDS)return -1;
		r->pids[k++]=a->pids[j++];
	}
	if(!k)r->numpids=SATIP_NOPIDS;
	else r->numpids=k;

	r->prognum=s->prognum;

	return 0;
}

static int tuneinfo(SATIP_TUNE *tune,SATIP_PIDS *set,char *bfr,int size,
	int strict)
{
	int i;
	int len=0;
	unsigned int fh;
	unsigned int fl;

	fh=tune->freq/1000000;
	fl=(tune->freq%1000000)/1000;

	switch(tune->msys)
	{
	case SATIP_DVBS2:
	case SATIP_DVBS:
		len+=snprintf(bfr+len,size-len,"%u.%03u,",fh,fl);
		switch(tune->pol)
		{
		case SATIP_POL_H:
			len+=snprintf(bfr+len,size-len,"h,");
			break;
		case SATIP_POL_V:
			len+=snprintf(bfr+len,size-len,"v,");
			break;
		case SATIP_POL_L:
			len+=snprintf(bfr+len,size-len,"l,");
			break;
		case SATIP_POL_R:
			len+=snprintf(bfr+len,size-len,"r,");
			break;
		default:len+=snprintf(bfr+len,size-len,",");
			break;
		}
		len+=snprintf(bfr+len,size-len,"%s,",
			tune->msys==SATIP_DVBS2?"dvbs2":"dvbs");
		switch(tune->mtype)
		{
		case SATIP_32APSK:
			len+=snprintf(bfr+len,size-len,"x_32apsk,");
			break;
		case SATIP_16APSK:
			len+=snprintf(bfr+len,size-len,"x_16apsk,");
			break;
		case SATIP_8PSK:
			len+=snprintf(bfr+len,size-len,"8psk,");
			break;
		case SATIP_QPSK:
			len+=snprintf(bfr+len,size-len,"qpsk,");
			break;
		default:if(tune->msys==SATIP_DVBS&&strict)
				len+=snprintf(bfr+len,size-len,"qpsk,");
			else len+=snprintf(bfr+len,size-len,",");
			break;
		}
		switch(tune->plts)
		{
		case SATIP_PLTS_OFF:
			len+=snprintf(bfr+len,size-len,"off,");
			break;
		case SATIP_PLTS_ON:
			len+=snprintf(bfr+len,size-len,"on,");
			break;
		default:if(tune->msys==SATIP_DVBS&&strict)
				len+=snprintf(bfr+len,size-len,"off,");
			else len+=snprintf(bfr+len,size-len,",");
			break;
		}
		switch(tune->ro)
		{
		case SATIP_ROFF_035:
			len+=snprintf(bfr+len,size-len,"0.35,");
			break;
		case SATIP_ROFF_025:
			len+=snprintf(bfr+len,size-len,"0.25,");
			break;
		case SATIP_ROFF_020:
			len+=snprintf(bfr+len,size-len,"0.20,");
			break;
		default:if(tune->msys==SATIP_DVBS&&strict)
				len+=snprintf(bfr+len,size-len,"0.35,");
			else len+=snprintf(bfr+len,size-len,",");
			break;
		}
		len+=snprintf(bfr+len,size-len,"%d,",tune->sr/1000);
		switch(tune->fec)
		{
		case SATIP_FEC_12:
			len+=snprintf(bfr+len,size-len,"12");
			break;
		case SATIP_FEC_23:
			len+=snprintf(bfr+len,size-len,"23");
			break;
		case SATIP_FEC_34:
			len+=snprintf(bfr+len,size-len,"34");
			break;
		case SATIP_FEC_35:
			len+=snprintf(bfr+len,size-len,"35");
			break;
		case SATIP_FEC_45:
			len+=snprintf(bfr+len,size-len,"45");
			break;
		case SATIP_FEC_56:
			len+=snprintf(bfr+len,size-len,"56");
			break;
		case SATIP_FEC_78:
			len+=snprintf(bfr+len,size-len,"78");
			break;
		case SATIP_FEC_89:
			len+=snprintf(bfr+len,size-len,"89");
			break;
		case SATIP_FEC_910:
			len+=snprintf(bfr+len,size-len,"910");
			break;
		}
		break;

	case SATIP_DVBT2:
	case SATIP_DVBT:
		len+=snprintf(bfr+len,size-len,"%u.%03u,",fh,fl);
		switch(tune->bw)
		{
		case SATIP_BW_1712:
			len+=snprintf(bfr+len,size-len,"1.712,");
			break;
		case SATIP_BW_5:
			len+=snprintf(bfr+len,size-len,"5,");
			break;
		case SATIP_BW_6:
			len+=snprintf(bfr+len,size-len,"6,");
			break;
		case SATIP_BW_7:
			len+=snprintf(bfr+len,size-len,"7,");
			break;
		case SATIP_BW_8:
			len+=snprintf(bfr+len,size-len,"8,");
			break;
		case SATIP_BW_10:
			len+=snprintf(bfr+len,size-len,"10,");
			break;
		default:len+=snprintf(bfr+len,size-len,",");
			break;
		}
		len+=snprintf(bfr+len,size-len,"%s,",
			tune->msys==SATIP_DVBT2?"dvbt2":"dvbt");
		switch(tune->tmode)
		{
		case SATIP_TMOD_1K:
			len+=snprintf(bfr+len,size-len,"1k,");
			break;
		case SATIP_TMOD_2K:
			len+=snprintf(bfr+len,size-len,"2k,");
			break;
		case SATIP_TMOD_4K:
			len+=snprintf(bfr+len,size-len,"4k,");
			break;
		case SATIP_TMOD_8K:
			len+=snprintf(bfr+len,size-len,"8k,");
			break;
		case SATIP_TMOD_16K:
			len+=snprintf(bfr+len,size-len,"16k,");
			break;
		case SATIP_TMOD_32K:
			len+=snprintf(bfr+len,size-len,"32k,");
			break;
		default:len+=snprintf(bfr+len,size-len,",");
			break;
		}
		switch(tune->mtype)
		{
		case SATIP_QPSK:
			len+=snprintf(bfr+len,size-len,"qpsk,");
			break;
		case SATIP_16Q:
			len+=snprintf(bfr+len,size-len,"16qam,");
			break;
		case SATIP_64Q:
			len+=snprintf(bfr+len,size-len,"64qam,");
			break;
		case SATIP_256Q:
			len+=snprintf(bfr+len,size-len,"256qam,");
			break;
		default:len+=snprintf(bfr+len,size-len,",");
			break;
		}
		switch(tune->gi)
		{
		case SATIP_GI_14:
			len+=snprintf(bfr+len,size-len,"14,");
			break;
		case SATIP_GI_18:
			len+=snprintf(bfr+len,size-len,"18,");
			break;
		case SATIP_GI_116:
			len+=snprintf(bfr+len,size-len,"116,");
			break;
		case SATIP_GI_132:
			len+=snprintf(bfr+len,size-len,"132,");
			break;
		case SATIP_GI_1128:
			len+=snprintf(bfr+len,size-len,"1128,");
			break;
		case SATIP_GI_19128:
			len+=snprintf(bfr+len,size-len,"19128,");
			break;
		case SATIP_GI_19256:
			len+=snprintf(bfr+len,size-len,"19256,");
			break;
		default:len+=snprintf(bfr+len,size-len,",");
			break;
		}
		switch(tune->fec)
		{
		case SATIP_FEC_12:
			len+=snprintf(bfr+len,size-len,"12,");
			break;
		case SATIP_FEC_23:
			len+=snprintf(bfr+len,size-len,"23,");
			break;
		case SATIP_FEC_34:
			len+=snprintf(bfr+len,size-len,"34,");
			break;
		case SATIP_FEC_35:
			len+=snprintf(bfr+len,size-len,"35,");
			break;
		case SATIP_FEC_45:
			len+=snprintf(bfr+len,size-len,"45,");
			break;
		case SATIP_FEC_56:
			len+=snprintf(bfr+len,size-len,"56,");
			break;
		case SATIP_FEC_78:
			len+=snprintf(bfr+len,size-len,"78,");
			break;
		default:len+=snprintf(bfr+len,size-len,",");
			break;
		}
		if(tune->plp==SATIP_UNDEF)len+=snprintf(bfr+len,size-len,",");
		else len+=snprintf(bfr+len,size-len,"%d,",tune->plp);
		if(tune->t2id==SATIP_UNDEF)len+=snprintf(bfr+len,size-len,",");
		else len+=snprintf(bfr+len,size-len,"%d,",tune->t2id);
		switch(tune->sm)
		{
		case SATIP_SM_SISO:
			len+=snprintf(bfr+len,size-len,"0");
			break;
		case SATIP_SM_MISO:
			len+=snprintf(bfr+len,size-len,"1");
			break;
		}
		break;

	case SATIP_DVBC2:
	case SATIP_DVBC:
		len+=snprintf(bfr+len,size-len,"%u.%03u,",fh,fl);
		switch(tune->bw)
		{
		case SATIP_BW_6:
			len+=snprintf(bfr+len,size-len,"6,");
			break;
		case SATIP_BW_8:
			len+=snprintf(bfr+len,size-len,"8,");
			break;
		default:len+=snprintf(bfr+len,size-len,",");
			break;
		}
		len+=snprintf(bfr+len,size-len,"%s,",
			tune->msys==SATIP_DVBC2?"dvbc2":"dvbc");
		switch(tune->mtype)
		{
		case SATIP_16Q:
			len+=snprintf(bfr+len,size-len,"16qam,");
			break;
		case SATIP_32Q:
			len+=snprintf(bfr+len,size-len,"32qam,");
			break;
		case SATIP_64Q:
			len+=snprintf(bfr+len,size-len,"64qam,");
			break;
		case SATIP_128Q:
			len+=snprintf(bfr+len,size-len,"128qam,");
			break;
		case SATIP_256Q:
			len+=snprintf(bfr+len,size-len,"256qam,");
			break;
		default:len+=snprintf(bfr+len,size-len,",");
			break;
		}
		if(tune->sr)len+=snprintf(bfr+len,size-len,"%d,",
			tune->sr/1000);
		switch(tune->c2tft)
		{
		case SATIP_TFT_DS:
			len+=snprintf(bfr+len,size-len,"0,");
			break;
		case SATIP_TFT_C2:
			len+=snprintf(bfr+len,size-len,"1,");
			break;
		case SATIP_TFT_IT:
			len+=snprintf(bfr+len,size-len,"2,");
			break;
		default:len+=snprintf(bfr+len,size-len,",");
			break;
		}
		if(tune->ds==SATIP_UNDEF)len+=snprintf(bfr+len,size-len,",");
		else len+=snprintf(bfr+len,size-len,"%d,",tune->ds);
		if(tune->plp==SATIP_UNDEF)len+=snprintf(bfr+len,size-len,",");
		else len+=snprintf(bfr+len,size-len,"%d,",tune->plp);
		switch(tune->specinv)
		{
		case SATIP_SPI_OFF:
			len+=snprintf(bfr+len,size-len,"0");
			break;
		case SATIP_SPI_ON:
			len+=snprintf(bfr+len,size-len,"1");
			break;
		}
		break;
	}

	len+=snprintf(bfr+len,size-len,";pids=");
	switch(set->numpids)
	{
	case SATIP_SECTION:
		len+=snprintf(bfr+len,size-len,"%d",set->pid);
		break;
	case SATIP_ALLPIDS:
		len+=snprintf(bfr+len,size-len,"all");
		break;
	case SATIP_SIGNAL:
	case SATIP_NOPIDS:
		len+=snprintf(bfr+len,size-len,"none");
		break;
	default:for(i=0;i<set->numpids;i++)
			len+=snprintf(bfr+len,size-len,"%s%d",i?",":"",
				set->pids[i]);
		break;
	}

	return len;
}

static CHUNK *describe_hdr(UPNP *upnp,int which)
{
	int len;
	CHUNK *c;
	SATIP_DATA data;
	unsigned int trash[2];
	char bfr[1024];
	char wrk[128];

	data.intval=sizeof(trash);
	data.ptrval=trash;
	upnp->cb(SATIP_GETRANDOM,&data,upnp->priv);

	addr2str(&upnp->devaddr[which],wrk,sizeof(wrk));

	len=snprintf(bfr,sizeof(bfr),"v=0\r\no=- %u %u IN %s %s\r\n"
		"s=SatIPServer:1 %d,%d,%d\r\nt=0 0\r\n",
		trash[0],trash[1],which==SATIP_V4_ONLY?"IP4":"IP6",
		wrk,upnp->sat,upnp->terr,upnp->cable);

	if(!(c=malloc(sizeof(CHUNK)+len)))return NULL;
	c->next=NULL;
	c->len=len;
	c->sent=0;
	memcpy(c->msg,bfr,len);

	return c;
}

static CHUNK *describe_sess(SESSION *s)
{
	int play=0;
	int len;
	SESSION *x;
	CHUNK *c;
	char bfr[1024];
	char wrk[128];

	pthread_spin_lock(&s->mtx);

	if(s->play)play=1;
	else for(x=s->slave;x;x=x->slave)if(x->play)
	{
		play=1;
		break;
	}

	pthread_spin_unlock(&s->mtx);

	if(!s->info[0])
	{
		len=tuneinfo(&s->state.tune,&s->state.set,bfr,sizeof(s->info),
			s->strict);
		pthread_spin_lock(&s->mtx);
		if(!s->info[0])
		{
			strcpy(s->info,bfr);
			s->ilen=len;
		}
		pthread_spin_unlock(&s->mtx);
	}

	if(s->mode==SATIP_TYPE_RTP)
	{
		sock2str(&s->rtp,wrk,sizeof(wrk));
		len=snprintf(bfr,sizeof(bfr),"m=video %d RTP/AVP 33\r\n"
			"c=IN %s %s/%d\r\na=control:stream=%d\r\na=fmtp:33 ",
			ntohs(psget(&s->rtp)),
			sockfam(&s->rtp)==AF_INET6?"IP6":"IP4",
			wrk,s->ttl,s->sid);
	}
	else len=snprintf(bfr,sizeof(bfr),"m=video 0 RTP/AVP 33\r\n"
		"c=IN %s\r\na=control:stream=%d\r\na=fmtp:33 ",
		sockfam(&s->rtp)==AF_INET6?"IP6 ::":"IP4 0.0.0.0",s->sid);
	switch(s->state.tune.msys)
	{
	case SATIP_DVBS2:
	case SATIP_DVBS:
		len+=snprintf(bfr+len,sizeof(bfr)-len,
			"ver="PROT";src=%d;tuner=%d,%d,%d,%d,%s",
			s->state.tune.src,
			s->state.tune.fe?s->state.tune.fe:UNKNOWN_FE,
			s->state.level,s->state.lock,s->state.quality,s->info);
		break;
	case SATIP_DVBT2:
	case SATIP_DVBT:
	case SATIP_DVBC2:
	case SATIP_DVBC:
		len+=snprintf(bfr+len,sizeof(bfr)-len,"ver="PROT";tuner=%d,%d,"
			"%d,%d,%s",s->state.tune.fe?s->state.tune.fe:UNKNOWN_FE,
			s->state.level,s->state.lock,s->state.quality,s->info);
		break;
	}
	len+=snprintf(bfr+len,sizeof(bfr)-len,"\r\na=%s\r\n",
		play?"sendonly":"inactive");

	if(!(c=malloc(sizeof(CHUNK)+len)))return NULL;
	c->next=NULL;
	c->len=len;
	c->sent=0;
	memcpy(c->msg,bfr,len);

	return c;
}

static int describe(UPNP *upnp,SESSION *s,int mode,CHUNK **c,int which)
{
	int len;

	if(!(*c=describe_hdr(upnp,which)))return 0;
	len=(*c)->len;
	c=&(*c)->next;

	do
	{
		if(s->master==s)if((*c=describe_sess(s)))
		{
			len+=(*c)->len;
			c=&(*c)->next;
		}
		s=s->next;
	}
	while(mode&&s);

	return len;
}

static int fake_describe(UPNP *upnp,CHUNK **c,int which)
{
	int len;
	int tot;
	char bfr[1024];

	if(!(*c=describe_hdr(upnp,which)))return 0;
	tot=(*c)->len;
	c=&(*c)->next;

	len=snprintf(bfr,sizeof(bfr),"m=video 0 RTP/AVP 33\r\n"
		"c=IN %s\r\na=fmtp:33 ver="PROT"\r\na=inactive\r\n",
		which==SATIP_V4_ONLY?"IP4 0.0.0.0":"IP6 ::");

	if(!(*c=malloc(sizeof(CHUNK)+len)))return tot;
	tot+=len;

	(*c)->next=NULL;
	(*c)->len=len;
	(*c)->sent=0;
	memcpy((*c)->msg,bfr,len);

	return tot;
}

static int rtsp_request(UPNP *upnp,RTSP *rtspdta)
{
	int n=0;
	int r=0;
	int s=0;
	int qp=0;
	int fake=0;
	int len;
	long val;
	void *ptr;
	char *mem;
	char *mem2;
	char *line;
	char *name;
	char *value;
	char *end;
	char *host;
	char *cmd=NULL;
	char *uri=NULL;
	char *ver=NULL;
	char *cseq=NULL;
	char *sess=NULL;
	char *trans=NULL;
	char *acc=NULL;
	char *req=NULL;
	char *conn=NULL;
	char *port=NULL;
	REQUEST *qry=NULL;
	SESSION *ss=NULL;
	SESSION *ms;
	CHUNK *c;
	CHUNK *e;
	MEMBER *m;
	ADDR addr;
	TRANSPORT t;
	SATIP_PIDS set;
	SOCK sock;
	SATIP_DATA peer;
	char smsg[128];
	char tmsg[256];
	struct epoll_event ev;

	for(line=strtok_r(rtspdta->bfr,"\r\n",&mem);line&&*line;
		line=strtok_r(NULL,"\r\n",&mem))
	{
		if(!n++)
		{
			if(!upnp->strict)
			{
				mem2=strrchr(line,' ');
				if(!mem2)mem2=strrchr(line,'\t');
				if(mem2&&!strncmp(mem2+1,"RTSP/",5))
				{
					*mem2++=0;
					ver=mem2;
					cmd=strtok_r(line,"\t ",&mem2);
					uri=strtok_r(NULL,"\r\n",&mem2);
					continue;
				}
			}
			cmd=strtok_r(line,"\t ",&mem2);
			uri=strtok_r(NULL,"\t ",&mem2);
			ver=strtok_r(NULL,"\t ",&mem2);
			continue;
		}

		name=strtok_r(line,":",&mem2);
		if(!name)return -1;

		value=strtok_r(NULL,"\n",&mem2);
		if(!value)value="";
		while(*value=='\t'||*value==' ')value++;

		if(!strcasecmp(name,"cseq"))
		{
			if(cseq)return -1;
			cseq=value;
		}
		else if(!strcasecmp(name,"session"))
		{
			if(sess)return -1;
			sess=value;
		}
		else if(!strcasecmp(name,"transport"))
		{
			if(trans)return -1;
			trans=value;
		}
		else if(!strcasecmp(name,"accept"))
		{
			if(acc)return -1;
			acc=value;
		}
		else if(!strcasecmp(name,"require"))
		{
			if(req)return -1;
			req=value;
		}
		else if(!strcasecmp(name,"connection"))
		{
			if(conn)return -1;
			conn=value;
		}
	}


	if(!cmd||!uri||!ver||!cseq||!*cseq)return -1;

	if(conn)if(!strcmp(conn,"close"))rtspdta->close=1;

	if(strcmp(ver,"RTSP/1.0"))
	{
		if(rtsperr(rtspdta->client,505,cseq,NULL))r=-1;
	}
	else if(strncmp(uri,"rtsp://",7))
	{
		if(!strcmp(uri,"*"))goto work;
		if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
	}
	else
	{
		uri+=7;
		if(*uri=='[')
		{
			for(host=uri+1;*uri&&*uri!='/'&&*uri!=']';uri++);
			if(*uri!=']')
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
			*uri++=0;
			if(*uri!='/'&&*uri!=':')
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
		}
		else for(host=uri;*uri&&*uri!=':'&&*uri!='/';uri++);
		if(*uri==':')
		{
			*uri++=0;
			port=uri;
		}
		for(;*uri&&*uri!='/';uri++);
		if(*uri)
		{
			*uri=0;
			if(str2addr(host,&addr)||aacmp(&addr,&rtspdta->devaddr))
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
			if(port)
			{
				val=strtol(port,&end,10);
				if(port==end||*end||val!=upnp->rtsp)
				{
					if(rtsperr(rtspdta->client,400,cseq,
						NULL))r=-1;
					goto out;
				}
			}
			*uri='/';
		}
		else if(!upnp->strict)
		{
			if(str2addr(host,&addr)||aacmp(&addr,&rtspdta->devaddr))
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
			if(port)
			{
				val=strtol(port,&end,10);
				if(port==end||*end||val!=upnp->rtsp)
				{
					if(rtsperr(rtspdta->client,400,cseq,
						NULL))r=-1;
					goto out;
				}
			}
		}
		else
		{
			if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
			goto out;
		}

		if(!strncmp(uri,"/stream=",8))
		{
			val=strtol(uri+8,&end,10);
			if(end==uri+8||(*end&&*end!='?'))s=-1;
			else if(val<1||val>65535)s=-2;
			else
			{
				s=val;
				if(*end=='?')
				{
					if(!(qry=malloc(sizeof(REQUEST))))
						qp=SATIP_SYSFAIL;
					else qp=satip_util_parse(
						SATIP_PARSE_QRY,upnp->caps,0,
						end+1,NULL,NULL,0,NULL,NULL,
						&qry->tune,&qry->set,&qry->add,
						&qry->del,NULL);
				}
			}
		}
		else if(!strncmp(uri,"/?",2))
		{
			if(!upnp->strict)
			{
				mem=strchr(uri+2,'/');
				if(mem&&!mem[1])*mem=0;
			}
			if(!(qry=malloc(sizeof(REQUEST))))qp=SATIP_SYSFAIL;
			else qp=satip_util_parse(SATIP_PARSE_QRY,upnp->caps,0,
				uri+2,NULL,NULL,0,NULL,NULL,&qry->tune,
				&qry->set,&qry->add,&qry->del,NULL);
		}
		else if(!*uri&&!upnp->strict);
		else if(strcmp(uri,"/"))
		{
			if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
			goto out;
		}

		if(qry&&!qp)if(qry->set.numpids==SATIP_SECTION||
			qry->add.numpids==SATIP_SECTION||
			qry->del.numpids==SATIP_SECTION||
			qry->set.numpids==SATIP_SIGNAL||
			qry->add.numpids==SATIP_SIGNAL||
			qry->del.numpids==SATIP_SIGNAL)
		{
			if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
			goto out;
		}

work:		if(!strcmp(cmd,"OPTIONS"))
		{
			if(acc||trans)
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
			switch(s)
			{
			case -1:if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case -2:if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			}
			switch(qp)
			{
			case SATIP_SYSFAIL:
				if(rtsperr(rtspdta->client,500,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_SYNTX:
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_VALUE:
				if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_OVER:
				if(rtsperr(rtspdta->client,414,cseq,NULL))r=-1;
				goto out;
			}
			if(s||qry)
			{
				if(!qp&&!upnp->strict);
				else
				{
					if(rtsperr(rtspdta->client,403,cseq,
						NULL))r=-1;
					goto out;
				}
			}
			if(req)
			{
				if(rtsperr(rtspdta->client,551,cseq,req))r=-1;
				goto out;
			}
			if(sess)
			{
				if(!(ss=lookup_sess(upnp,sess)))
				{
					if(rtsperr(rtspdta->client,454,cseq,
						NULL))r=-1;
					goto out;
				}
				if(ss->idle!=-1)ss->idle=0;
			}
			if(rtspok(rtspdta->client,HDR_PUBLIC,NULL,NULL,0,0,
				upnp->timeout,cseq,sess,NULL))r=-1;
		}
		else if(!strcmp(cmd,"DESCRIBE"))
		{
			if(!acc||trans)
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
			switch(s)
			{
			case -1:if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case -2:if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			}
			switch(qp)
			{
			case SATIP_SYSFAIL:
				if(rtsperr(rtspdta->client,500,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_SYNTX:
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_VALUE:
				if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_OVER:
				if(rtsperr(rtspdta->client,414,cseq,NULL))r=-1;
				goto out;
			}
			if(qry)
			{
				if(!qp&&!qry->add.numpids&&!qry->del.numpids&&
					!upnp->strict)fake=1;
				else
				{
					if(rtsperr(rtspdta->client,400,cseq,
						NULL))r=-1;
					goto out;
				}
			}
			if(strcmp(acc,"application/sdp"))
			{
				if(rtsperr(rtspdta->client,406,cseq,NULL))r=-1;
				goto out;
			}
			if(req)
			{
				if(rtsperr(rtspdta->client,551,cseq,req))r=-1;
				goto out;
			}
			if(sess)
			{
				if(!(ss=lookup_sess(upnp,sess)))
				{
					if(rtsperr(rtspdta->client,454,cseq,
						NULL))r=-1;
					goto out;
				}
				if(ss->idle!=-1)ss->idle=0;
			}
			if(fake);
			else if(s)
			{
				if(!(ss=lookup_sid(upnp,s)))
				{
					if(rtsperr(rtspdta->client,404,cseq,
						NULL))r=-1;
					goto out;
				}
			}
			else if(!upnp->slist)
			{
				if(rtsperr(rtspdta->client,404,cseq,NULL))r=-1;
				goto out;
			}
			else ss=upnp->slist;

			if(fake)len=fake_describe(upnp,&c,rtspdta->which);
			else if(!(len=describe(upnp,s?ss:upnp->slist,s?0:1,&c,
				rtspdta->which)))
			{
				if(rtsperr(rtspdta->client,500,cseq,NULL))r=-1;
				goto out;
			}

			if(rtspok(rtspdta->client,HDR_TYPE|HDR_BASE|HDR_LEN|
				(fake?HDR_QUERY:0)|(upnp->strict?HDR_STRICT:0),
				qry,&rtspdta->devaddr,upnp->rtsp,len,
				upnp->timeout,cseq,sess,NULL))r=-1;
			else
			{
				ev.events=EPOLLOUT;
				ev.data.ptr=rtspdta;
				if(epoll_ctl(upnp->ffd,EPOLL_CTL_MOD,
					rtspdta->client,&ev))r=-1;
				else
				{
					rtspdta->tx=c;
					c=NULL;
				}
			}

			while(c)
			{
				e=c;
				c=e->next;
				free(e);
			}
		}
		else if(!strcmp(cmd,"SETUP"))
		{
			if(acc||!trans)
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
			switch(s)
			{
			case -1:if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case -2:if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			}
			switch(qp)
			{
			case SATIP_SYSFAIL:
				if(rtsperr(rtspdta->client,500,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_SYNTX:
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_VALUE:
				if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_OVER:
				if(rtsperr(rtspdta->client,414,cseq,NULL))r=-1;
				goto out;
			}
			if(!s&&!qry)
			{
				if(rtsperr(rtspdta->client,405,cseq,NULL))r=-1;
				goto out;
			}
			if(rtsp_transport(trans,&t,&rtspdta->addr,upnp->level,
				upnp->strict))
			{
				if(rtsperr(rtspdta->client,461,cseq,NULL))r=-1;
				goto out;
			}
			if(t.mode==SATIP_TYPE_RTP)
			{
				addr2sock(&sock,&rtspdta->addr,0,upnp->devidx);
				peer.intval=sockfam(&sock);
				peer.ptrval=&sock;
				if(upnp->cb(SATIP_MCSTOK,&peer,upnp->priv))
				{
					if(rtsperr(rtspdta->client,403,cseq,
						NULL))r=-1;
					goto out;
				}
			}
			if(req)
			{
				if(rtsperr(rtspdta->client,551,cseq,req))r=-1;
				goto out;
			}
			if(sess)
			{
				if(!(ss=lookup_sess(upnp,sess)))
				{
					if(rtsperr(rtspdta->client,454,cseq,
						NULL))r=-1;
					goto out;
				}

				if(s&&s!=ss->master->sid)
				{
					if(rtsperr(rtspdta->client,404,cseq,
						NULL))r=-1;
					goto out;
				}

				if(ss->idle!=-1)ss->idle=0;

				if(ss==ss->master)
				{
					if(qry)
					{
						if(qry->set.numpids)
							set=qry->set;
						else if(pidmerge(&ss->state.set,
							&qry->add,&qry->del,
							&set))
						{
							if(rtsperr(
								rtspdta->client,
								403,cseq,NULL))
									r=-1;
							goto out;
						}
					}

					pthread_spin_lock(&ss->mtx);

					if(ss->play||ss->slave)
					{
						pthread_spin_unlock(&ss->mtx);
						if(rtsperr(rtspdta->client,455,
							cseq,NULL))r=-1;
						goto out;
					}

					pthread_spin_unlock(&ss->mtx);

					switch(change_sess(upnp,&t,ss,
						rtspdta->which))
					{
					case -1:if(rtsperr(rtspdta->client,500,
							cseq,NULL))r=-1;
						goto out;
					case -2:if(rtsperr(rtspdta->client,403,
							cseq,NULL))r=-1;
						goto out;
					}

					if(qry)
					{
						ss->state.tune=qry->tune;
						ss->state.set=set;
					}

					if(ss->mode==SATIP_TYPE_RTP)
					{
						addr2str(&t.addr,smsg,
							sizeof(smsg));
						sprintf(tmsg,"RTP/AVP;multicast"
							";destination=%s;"
							"port=%d-%d;ttl=%d",
							smsg,t.rtp,t.rtcp,
							t.ttl);
					}
					else sprintf(tmsg,"RTP/AVP;unicast;"
						"client_port=%d-%d;server_port="
						"%u-%u;ssrc=%08x",t.rtp,t.rtcp,
						ss->port,ss->port+1,ss->ssrc);

					if(rtspok(rtspdta->client,
						(ss->mode==SATIP_TYPE_RTP?
						HDR_TMO0:HDR_TMOXX)|HDR_SID,
						NULL,NULL,0,ss->sid,
						upnp->timeout,cseq,sess,tmsg))
							r=-1;
				}
				else
				{
					if(qry)
					{
						if(rtsperr(rtspdta->client,403,
							cseq,NULL))r=-1;
						goto out;
					}

					pthread_spin_lock(&ss->master->mtx);

					if(ss->play)
					{
						pthread_spin_unlock(
							&ss->master->mtx);
						if(rtsperr(rtspdta->client,455,
							cseq,NULL))r=-1;
						goto out;
					}

					pthread_spin_unlock(&ss->master->mtx);

					switch(change_sess(upnp,&t,ss,
						rtspdta->which))
					{
					case -1:if(rtsperr(rtspdta->client,500,
							cseq,NULL))r=-1;
						goto out;
					case -2:if(rtsperr(rtspdta->client,403,
							cseq,NULL))r=-1;
						goto out;
					}

					if(ss->mode==SATIP_TYPE_RTP)
					{
						addr2str(&t.addr,smsg,
						sizeof(smsg));
						sprintf(tmsg,"RTP/AVP;multicast"
							";destination=%s;"
							"port=%d-%d;ttl=%d",
							smsg,t.rtp,t.rtcp,
							t.ttl);
					}
					else sprintf(tmsg,"RTP/AVP;unicast;"
						"client_port=%d-%d;server_port="
						"%u-%u;ssrc=%08x",t.rtp,t.rtcp,
						ss->port,ss->port+1,ss->ssrc);

					if(rtspok(rtspdta->client,
						HDR_TMOXX|HDR_SID,NULL,NULL,0,
						ss->master->sid,upnp->timeout,
						cseq,sess,tmsg))r=-1;
				}
			}
			else if(s)
			{
				if(!(ms=lookup_sid(upnp,s)))
				{
					if(rtsperr(rtspdta->client,404,cseq,
						NULL))r=-1;
					goto out;
				}

				if(qry)
				{
					if(rtsperr(rtspdta->client,403,cseq,
						NULL))r=-1;
					goto out;
				}

				if(upnp->rtsplock)
				{
					if(rtsperr(rtspdta->client,503,cseq,
						"sessions"))r=-1;
					goto out;
				}

				switch(alloc_sess(upnp,&t,ms,&ss,
					rtspdta->which))
				{
				case -1:if(rtsperr(rtspdta->client,500,cseq,
						NULL))r=-1;
					goto out;
				case -2:if(rtsperr(rtspdta->client,503,cseq,
						"sessions"))r=-1;
					goto out;
				case -3:if(rtsperr(rtspdta->client,403,cseq,
						NULL))r=-1;
					goto out;
				}

				if(ss->mode==SATIP_TYPE_RTP)
				{
					addr2str(&t.addr,smsg,sizeof(smsg));
					sprintf(tmsg,"RTP/AVP;multicast;"
						"destination=%s;port=%d-%d;"
						"ttl=%d",smsg,t.rtp,t.rtcp,
						t.ttl);
				}
				else sprintf(tmsg,"RTP/AVP;unicast;client_port="
					"%d-%d;server_port=%u-%u;ssrc=%08x",
					t.rtp,t.rtcp,ss->port,ss->port+1,
					ss->ssrc);

				sprintf(smsg,"%016llx",ss->id);

				if(rtspok(rtspdta->client,HDR_TMOXX|HDR_SID,
					NULL,NULL,0,ss->master->sid,
					upnp->timeout,cseq,smsg,tmsg))r=-1;
			}
			else
			{
				if(qry->add.numpids||qry->del.numpids)
				{
					if(rtsperr(rtspdta->client,403,cseq,
						NULL))r=-1;
					goto out;
				}

				if(upnp->rtsplock)
				{
					if(rtsperr(rtspdta->client,503,cseq,
						"sessions"))r=-1;
					goto out;
				}

				switch(alloc_sess(upnp,&t,NULL,&ss,
					rtspdta->which))
				{
				case -1:if(rtsperr(rtspdta->client,500,cseq,
						NULL))r=-1;
					goto out;
				case -2:if(rtsperr(rtspdta->client,503,cseq,
						"sessions"))r=-1;
					goto out;
				case -3:if(rtsperr(rtspdta->client,403,cseq,
						NULL))r=-1;
					goto out;
				}

				ss->state.tune=qry->tune;
				ss->state.set=qry->set;

				if(ss->mode==SATIP_TYPE_RTP)
				{
					addr2str(&t.addr,smsg,sizeof(smsg));
					sprintf(tmsg,"RTP/AVP;multicast;"
						"destination=%s;port=%d-%d;"
						"ttl=%d",smsg,t.rtp,t.rtcp,
						t.ttl);
				}
				else sprintf(tmsg,"RTP/AVP;unicast;client_port="
					"%d-%d;server_port=%u-%u;ssrc=%08x",
					t.rtp,t.rtcp,ss->port,ss->port+1,
					ss->ssrc);

				sprintf(smsg,"%016llx",ss->id);

				if(rtspok(rtspdta->client,
					(ss->mode==SATIP_TYPE_RTP?
					HDR_TMO0:HDR_TMOXX)
					|HDR_SID,NULL,NULL,0,ss->sid,
					upnp->timeout,cseq,smsg,tmsg))r=-1;
			}
		}
		else if(!strcmp(cmd,"PLAY"))
		{
			if(!sess||acc||trans)
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
			switch(s)
			{
			case -1:if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case -2:if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			}
			switch(qp)
			{
			case SATIP_SYSFAIL:
				if(rtsperr(rtspdta->client,500,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_SYNTX:
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_VALUE:
				if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_OVER:
				if(rtsperr(rtspdta->client,414,cseq,NULL))r=-1;
				goto out;
			}
			if(!upnp->strict&&!s)if((ss=lookup_sess(upnp,sess)))
				s=ss->sid;
			if(!s)
			{
				if(rtsperr(rtspdta->client,405,cseq,NULL))r=-1;
				goto out;
			}
			if(req)
			{
				if(rtsperr(rtspdta->client,551,cseq,req))r=-1;
				goto out;
			}
			if(!(ss=lookup_sess(upnp,sess)))
			{
				if(rtsperr(rtspdta->client,454,cseq,NULL))r=-1;
				goto out;
			}
			if(ss->idle!=-1)ss->idle=0;
			if(qry&&ss->master!=ss)
			{
				if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			}
			if(qry)
			{
				if(qry->set.numpids)set=qry->set;
				else if(pidmerge(&ss->state.set,&qry->add,
					&qry->del,&set))
				{
					if(rtsperr(rtspdta->client,403,cseq,
						NULL))r=-1;
					goto out;
				}

			}
			if(ss->master->sid!=s)
			{
				if(ss!=ss->master)
				{
					if(!(ms=lookup_sid(upnp,s)))
					{
						if(rtsperr(rtspdta->client,404,
							cseq,NULL))r=-1;
						goto out;
					}
					if(move_sess(upnp,ms,ss->master,ss))
					{
						if(rtsperr(rtspdta->client,455,
							cseq,NULL))r=-1;
						goto out;
					}
				}
				else
				{
					if(rtsperr(rtspdta->client,404,cseq,
						NULL))r=-1;
					goto out;
				}
			}
			if(upnp->bpslimit&&upnp->bps>=upnp->bpslimit)
			{
				pthread_spin_lock(&ss->master->mtx);
				val=(ss->master->play&&ss->play);
				pthread_spin_unlock(&ss->master->mtx);
				if(!val)
				{
					if(rtsperr(rtspdta->client,453,cseq,
						NULL))r=-1;
					goto out;
				}
			}

			if(qry)
			{
				if(ss->handle)
				{
					if(change_stream(upnp,&qry->tune,&set,
						ss->handle,&ss->state))
					{
						end_stream(upnp,ss->handle);
						ss->handle=NULL;
					}
				}

				if(!ss->handle)
				{
					ss->state.set=set;
					ss->state.tune=qry->tune;
					ss->state.level=0;
					ss->state.lock=0;
					ss->state.quality=0;
				}
			}

			if(!ss->master->handle)
			{
				ss->fill=0;
				if(!(m=play_stream(upnp,&ss->master->state.tune,
					&ss->master->state.set,ss->master,0)))
				{
					if(rtsperr(rtspdta->client,503,cseq,
						NULL))r=-1;
					goto out;
				}
				else ss->master->handle=m;
			}

			if(ss->master==ss&&ss->mode==SATIP_TYPE_RTP)
			{
				ss->v3idle=0;
				ss->v3mode=0;
			}

			pthread_spin_lock(&ss->master->mtx);

			if(ss->master==ss||ss->mode==SATIP_TYPE_RTSP)
			{
				if(!ss->play)
				{
					pthread_spin_unlock(&ss->master->mtx);
					ss->seq=mkseq(ss->sid);
					ptr=&ss->seq;
					upnp->playsessions++;
				}
				else
				{
					pthread_spin_unlock(&ss->master->mtx);
					ptr=NULL;
				}
			}
			else if(!ss->master->play)
			{
				pthread_spin_unlock(&ss->master->mtx);
				ss->master->seq=mkseq(ss->master->sid);
				ptr=&ss->master->seq;
				upnp->playsessions++;
			}
			else
			{
				pthread_spin_unlock(&ss->master->mtx);
				ptr=NULL;
			}

			if(rtspok(rtspdta->client,HDR_INFO,NULL,
				&rtspdta->devaddr,upnp->rtsp,ss->master->sid,
				upnp->timeout,cseq,sess,ptr))r=-1;

			pthread_spin_lock(&ss->master->mtx);

			if(ss->master==ss||ss->mode==SATIP_TYPE_RTSP)
				ss->play=(PLAY_PLAYING|PLAY_PERSIST);
			else if(!ss->master->play)
				ss->master->play=(PLAY_PLAYING|PLAY_PERSIST);
			else ss->play=(PLAY_PLAYING|PLAY_PERSIST);

			pthread_spin_unlock(&ss->master->mtx);
		}
		else if(!strcmp(cmd,"TEARDOWN"))
		{
			if(!sess||acc||trans)
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
			switch(s)
			{
			case -1:if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case -2:if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			}
			switch(qp)
			{
			case SATIP_SYSFAIL:
				if(rtsperr(rtspdta->client,500,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_SYNTX:
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_VALUE:
				if(rtsperr(rtspdta->client,403,cseq,NULL))r=-1;
				goto out;
			case SATIP_ERR_OVER:
				if(rtsperr(rtspdta->client,414,cseq,NULL))r=-1;
				goto out;
			}
			if(!s)
			{
				if(rtsperr(rtspdta->client,405,cseq,NULL))r=-1;
				goto out;
			}
			if(qry)
			{
				if(rtsperr(rtspdta->client,400,cseq,NULL))r=-1;
				goto out;
			}
			if(req)
			{
				if(rtsperr(rtspdta->client,551,cseq,req))r=-1;
				goto out;
			}
			if(!(ss=lookup_sess(upnp,sess)))
			{
				if(rtsperr(rtspdta->client,454,cseq,NULL))r=-1;
				goto out;
			}

			if(ss!=ss->master)
			{
				pthread_spin_lock(&ss->master->mtx);

				if(ss->master->play)n=0;
				else for(n=1,ms=ss->master->slave;ms;
					ms=ms->next)if(ms!=ss&&ms->play)
				{
					n=0;
					break;
				}

				pthread_spin_unlock(&ss->master->mtx);
			}
			else n=1;

			if(n&&ss->master->handle)
				if(end_stream(upnp,ss->master->handle))
			{
				free_sess(upnp,ss);
				if(rtsperr(rtspdta->client,500,cseq,NULL))r=-1;
				goto out;
			}

			free_sess(upnp,ss);

			if(rtspok(rtspdta->client,0,NULL,NULL,0,0,upnp->timeout,
				cseq,sess,NULL))r=-1;
		}
		else if(rtsperr(rtspdta->client,501,cseq,NULL))r=-1;
	}

out:	if(qry)free(qry);
	return r;
}

static void bootinc(UPNP *upnp)
{
	SATIP_DATA data;

	upnp->bootcnt++;
	data.intval=upnp->bootcnt;
	data.ptrval=NULL;
	upnp->cb(SATIP_SAVEBCNT,&data,upnp->priv);
}

static int loaduuid(UPNP *upnp)
{
	SATIP_DATA data;
	unsigned char bfr[16];

	data.intval=SATIP_UUID_LEN;
	data.ptrval=upnp->uuid;
	if(upnp->cb(SATIP_LOADUUID,&data,upnp->priv))
	{
		data.intval=16;
		data.ptrval=bfr;
		if(upnp->cb(SATIP_GETRANDOM,&data,upnp->priv))return -1;

		sprintf(upnp->uuid,"%08x-%04x-%04x-%04x-%04x%08x",
			*((unsigned int *)(bfr)),
			*((unsigned short *)(bfr+4)),
			*((unsigned short *)(bfr+6)),
			*((unsigned short *)(bfr+8)),
			*((unsigned short *)(bfr+10)),
			*((unsigned int *)(bfr+12)));

		data.intval=SATIP_UUID_LEN;
		data.ptrval=upnp->uuid;
		upnp->cb(SATIP_SAVEUUID,&data,upnp->priv);
	}
	else upnp->uuid[SATIP_UUID_LEN]=0;

	return 0;
}

static void loadbcnt(UPNP *upnp)
{
	SATIP_DATA data;

	upnp->bootcnt=0;
	data.intval=0;
	data.ptrval=NULL;
	if(!upnp->cb(SATIP_LOADBCNT,&data,upnp->priv))
		upnp->bootcnt=data.intval;
}

static void newdevid(UPNP *upnp)
{
	char val;
	SATIP_DATA data;

	upnp->oldid=upnp->devid;

	do
	{
		data.intval=1;
		data.ptrval=&val;
		if(upnp->cb(SATIP_GETRANDOM,&data,upnp->priv))
			val=(time(NULL)&0x7f)+1;
		else val=(val&0x3f)+1;

		upnp->devid=(upnp->devid+val)&0xff;
	} while(upnp->devid==upnp->oldid);
}

static void loaddevid(UPNP *upnp)
{
	SATIP_DATA data;

	data.intval=0;
	data.ptrval=NULL;
	if(!upnp->cb(SATIP_LOADDEVID,&data,upnp->priv))
		upnp->devid=data.intval;
	upnp->oldid=upnp->devid;
}

static void savedevid(UPNP *upnp)
{
	SATIP_DATA data;

	if(upnp->oldid==upnp->devid)return;

	data.intval=upnp->devid;
	data.ptrval=NULL;
	upnp->cb(SATIP_SAVEDEVID,&data,upnp->priv);
	upnp->oldid=upnp->devid;
}

static int msghandler(UPNP *upnp,char *data,ADDR *addr,int multi,
	unsigned short port,int which)
{
	int i;
	int err=0;
	MSGDATA *msg;
	COLL *c;
	COLL **d;

	if(!(msg=msgsplit(data)))return 0;

	if(!strcasecmp(msg->hdr,"NOTIFY * HTTP/1.1"))
	{
		for(i=0;i<msg->total;i++)if(!strcmp(msg->name[i],"nts"))break;
		if(i!=msg->total&&!strcasecmp(msg->value[i],"ssdp:alive"))
		{
			for(i=0;i<msg->total;i++)
				if(!strcmp(msg->name[i],"deviceid.ses.com"))
					break;
			if(i!=msg->total&&upnp->state>ANNOUNCE)
				if(atoi(msg->value[i])==upnp->devid)
			{
				if(upnp->state<RUNNING)
				{
					upnp->flag=1;
					goto out;
				}

				for(c=upnp->coll;c;c=c->next)
					if(!aacmp(&c->addr,addr)&&c->port==port)
						goto out;

				if(!(c=malloc(sizeof(COLL))))
				{
					err=-1;
					goto out;
				}

				c->next=upnp->coll;
				c->addr=*addr;
				c->cnt=4;
				c->which=which;
				c->port=port;
				upnp->coll=c;
				goto out;
			}
		}
	}
	else if(!strcasecmp(msg->hdr,"HTTP/1.1 200 OK"))
	{
		for(i=0;i<msg->total;i++)
			if(!strcmp(msg->name[i],"deviceid.ses.com"))break;
		if(i!=msg->total&&upnp->state==RUNNING&&!multi)
			if(atoi(msg->value[i])==upnp->devid)
		{
			for(d=&upnp->coll;*d;d=&(*d)->next)
				if(!aacmp(&(*d)->addr,addr)&&(*d)->port==port)
			{
				c=*d;
				*d=c->next;
				free(c);
				goto out;
			}
		}
	}
	else if(!strcasecmp(msg->hdr,"M-SEARCH * HTTP/1.1"))
	{
		for(i=0;i<msg->total;i++)if(!strcmp(msg->name[i],"man"))break;
		if(i!=msg->total&&
			!strcasecmp(msg->value[i],"\"ssdp:discover\""))
		{
			for(i=0;i<msg->total;i++)
				if(!strcmp(msg->name[i],"deviceid.ses.com"))
					break;
			if(i!=msg->total&&upnp->state<RUNNING&&!multi)
				if(atoi(msg->value[i])==upnp->devid)
			{
				if(sendok(upnp,addr,port,which))err=-1;
				else upnp->flag=1;
				goto out;
			}

			for(i=0;i<msg->total;i++)
				if(!strcmp(msg->name[i],"st"))
					break;
			if(i!=msg->total&&upnp->state==RUNNING&&multi)
				if(!strcasecmp(msg->value[i],
					"urn:ses-com:device:SatIPServer:1"))
			{
				if(sendok(upnp,addr,port,which))err=1;
				goto out;
			}
		}
	}

out:	while(msg->total--)
	{
		free(msg->name[msg->total]);
		free(msg->value[msg->total]);
	}
	free(msg->hdr);
	free(msg);
	return err;
}

static void closer(UPNP *upnp,int *fd)
{
	if(*fd==-1)return;
	epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,*fd,NULL);
	close(*fd);
	*fd=-1;
}

static void ifaddr_add(UPNP *upnp,struct in6_addr *a6)
{
	int scope;
	IFADDR *e;

	if(!(scope=a6sitewide(a6)))return;
	else if(scope==3)return;

	for(e=upnp->ifaddr;e;e=e->next)if(!memcmp(a6->s6_addr,e->a6.s6_addr,16))
		return;

	if(!(e=malloc(sizeof(IFADDR))))return;

	e->scope=scope;
	memcpy(e->a6.s6_addr,a6->s6_addr,16);
	e->next=upnp->ifaddr;
	upnp->ifaddr=e;
}

static void ifaddr_del(UPNP *upnp,struct in6_addr *a6)
{
	IFADDR **d;
	IFADDR *e;

	for(d=&upnp->ifaddr;*d;d=&(*d)->next)if(!memcmp(a6->s6_addr,
		(*d)->a6.s6_addr,16))
	{
		e=*d;
		*d=e->next;
		free(e);
		return;
	}
}

static void ifaddr_flush(UPNP *upnp)
{
	IFADDR *e;

	while(upnp->ifaddr)
	{
		e=upnp->ifaddr;
		upnp->ifaddr=e->next;
		free(e);
	}
}

static int ifaddr_select(UPNP *upnp,struct in6_addr *a6)
{
	IFADDR *e;
	IFADDR *r;

	for(r=NULL,e=upnp->ifaddr;e;e=e->next)if(!r)r=e;
	else if(e->scope<r->scope)r=e;
	else if(e->scope==r->scope)
		if(memcmp(e->a6.s6_addr,r->a6.s6_addr,16)<0)r=e;

	if(r)
	{
		memcpy(a6->s6_addr,r->a6.s6_addr,16);
		return 0;
	}
	else
	{
		memset(a6->s6_addr,0,16);
		return -1;
	}
}

static int getrunning(UPNP *upnp)
{
	int r;

	pthread_mutex_lock(&upnp->htx);
	r=upnp->running;
	pthread_mutex_unlock(&upnp->htx);
	return r;
}

static void *webwork(void *data)
{
	int i;
	int l=0;
	int n=0;
	CLNDTA *w=(CLNDTA *)data;
	struct pollfd p[2];
	struct pollfd hp;
	char *ptr;
	char *mem;
	REQUEST *r=NULL;
	STREAM *s=NULL;
	STREAM *hs;
	STREAM **ds;
	MEMBER *m;
	SATIP_DATA param[2];
	char bfr[2048];
	char req[2048];
	sigset_t set;

	sigfillset(&set);
	pthread_sigmask(SIG_BLOCK,&set,NULL);

	pthread_setname_np(pthread_self(),"server webwork");

	if(w->state!=RUNNING)
	{
		httperr(w->client,503);
		goto out;
	}

	param[0].intval=sockfam(&w->addr);
	param[0].ptrval=&w->addr;
	if(w->upnp->cb(SATIP_PEEROK,param,w->upnp->priv))
	{
		httperr(w->client,403);
		goto out;
	}
	if(w->upnp->cb(SATIP_HTTPOK,param,w->upnp->priv))
	{
		httperr(w->client,403);
		goto out;
	}

	p[0].fd=w->client;
	p[0].events=POLLIN;
	p[1].fd=w->upnp->tfd;
	p[1].events=POLLIN;

repeat:	if(poll(p,2,1000)<1)
	{
		httperr(w->client,408);
		goto out;
	}

	if(p[1].revents&POLLIN)
	{
		httperr(w->client,503);
		goto out;
	}

	if(!(p[0].revents&POLLIN))
	{
		httperr(w->client,408);
		goto out;
	}

	if((i=read(w->client,bfr+l,sizeof(bfr)-1-l))<=0)
	{
		httperr(w->client,400);
		goto out;
	}
	l+=i;

	for(i=0;i<l;i++)if(bfr[i]=='\r'||bfr[i]=='\n')break;
	if(i==l)
	{
		if(++n<3)goto repeat;
		httperr(w->client,400);
		goto out;
	}
	bfr[l]=0;

	memcpy(req,bfr,i);
	req[i]=0;

	param[0].intval=l;
	param[0].ptrval=bfr;
	param[1].intval=w->client;
	param[1].ptrval=NULL;

	ptr=strtok_r(req," \t",&mem);
	if(!ptr||strcmp(ptr,"GET"))
	{
		if(!w->upnp->cb(SATIP_HTTPREQ,param,w->upnp->priv))goto out;
		httperr(w->client,405);
		goto out;
	}

	if(!(ptr=strtok_r(NULL," \t",&mem)))
	{
		httperr(w->client,400);
		goto out;
	}

	if(!strcmp(ptr,"/desc.xml"))sendxml(w);
	else if((w->upnp->webflags&0x02)&&!strcmp(ptr,"/icon48.png"))
	{
		if(w->upnp->httplimit&&getrunning(w->upnp)>=w->upnp->httplimit)
		{
			httperr(w->client,503);
			goto out;
		}
		sendimg(w,SATIP_PNG48,"png");
	}
	else if((w->upnp->webflags&0x08)&&!strcmp(ptr,"/icon48.jpg"))
	{
		if(w->upnp->httplimit&&getrunning(w->upnp)>=w->upnp->httplimit)
		{
			httperr(w->client,503);
			goto out;
		}
		sendimg(w,SATIP_JPG48,"jpeg");
	}
	else if((w->upnp->webflags&0x04)&&!strcmp(ptr,"/icon120.png"))
	{
		if(w->upnp->httplimit&&getrunning(w->upnp)>=w->upnp->httplimit)
		{
			httperr(w->client,503);
			goto out;
		}
		sendimg(w,SATIP_PNG120,"png");
	}
	else if((w->upnp->webflags&0x10)&&!strcmp(ptr,"/icon120.jpg"))
	{
		if(w->upnp->httplimit&&getrunning(w->upnp)>=w->upnp->httplimit)
		{
			httperr(w->client,503);
			goto out;
		}
		sendimg(w,SATIP_JPG120,"jpeg");
	}
	else if((w->upnp->webflags&0x01)&&!strcmp(ptr,"/channellist.m3u"))
	{
		if(w->upnp->httplimit&&getrunning(w->upnp)>=w->upnp->httplimit)
		{
			httperr(w->client,503);
			goto out;
		}
		sendlst(w);
	}
	else if(!strncmp(ptr,"/?",2))
	{
		if(w->upnp->bpslimit&&w->upnp->bps>=w->upnp->bpslimit)goto over;
		if(w->upnp->httplimit&&getrunning(w->upnp)>=
			w->upnp->httplimit-w->upnp->httpnostream)
		{
over:			httperr(w->client,503);
			goto out;
		}
		if(!(r=malloc(sizeof(REQUEST))))httperr(w->client,500);
		else if(satip_util_parse(SATIP_PARSE_QRY,w->upnp->caps,0,ptr+2,
			NULL,NULL,0,NULL,NULL,&r->tune,&r->set,&r->add,&r->del,
			NULL))httperr(w->client,400);
		else if(r->add.numpids||r->del.numpids)httperr(w->client,400);
		else if(!(s=malloc(sizeof(STREAM))))httperr(w->client,500);
		else
		{
			pthread_mutex_lock(&w->upnp->htx);
			if(w->upnp->httplock)
			{
				pthread_mutex_unlock(&w->upnp->htx);
				free(s);
				httperr(w->client,503);
				goto out;
			}

			for(hs=w->upnp->hlist;hs;hs=hs->next)
			{
				pthread_spin_lock(&hs->mtx);
				if(hs->fd!=-1)
				{
					hp.events=POLLIN;
					hp.fd=hs->fd;
					if(poll(&hp,1,0)==1)
					{
						hs->fd=-1;
						pthread_spin_unlock(&hs->mtx);
						close(hp.fd);
						if(hs->handle)
						{
							end_stream(w->upnp,
								hs->handle);
							hs->handle=NULL;
							w->upnp->playhttp--;
						}
					}
					else pthread_spin_unlock(&hs->mtx);
				}
				else pthread_spin_unlock(&hs->mtx);
			}

			for(ds=&w->upnp->hlist;*ds;)
			{
				pthread_spin_lock(&(*ds)->mtx);
				if((*ds)->fd==-1)
				{
					pthread_spin_unlock(&(*ds)->mtx);
					if((*ds)->handle)
					{
						end_stream(w->upnp,
							(*ds)->handle);
						w->upnp->playhttp--;
					}
					hs=*ds;
					*ds=hs->next;
					pthread_spin_destroy(&hs->mtx);
					free(hs);
				}
				else
				{
					pthread_spin_unlock(&(*ds)->mtx);
					ds=&(*ds)->next;
				}
			}

			pthread_mutex_unlock(&w->upnp->htx);

			memset(s,0,sizeof(STREAM));
			s->fd=-1;
			s->peer=w->addr;
			s->strict=w->upnp->strict;
			if(r->set.numpids==SATIP_SIGNAL)
			{
				i=2;
				s->mode=0;
				r->set.numpids=SATIP_NOPIDS;
			}
			else
			{
				i=r->set.numpids==SATIP_SECTION?1:0;
				s->mode=1;
			}

			if(pthread_spin_init(&s->mtx,PTHREAD_PROCESS_PRIVATE))
			{
				free(s);
				httperr(w->client,500);
				goto out;
			}

			s->state.tune=r->tune;
			s->state.set=r->set;

			if(!(m=play_stream(w->upnp,&s->state.tune,&s->state.set,
				s,1)))
			{
				pthread_spin_unlock(&s->mtx);
				pthread_spin_destroy(&s->mtx);
				free(s);
				httperr(w->client,500);
				goto out;
			}
			s->handle=m;

			if(streamhdr(w->client,i))
			{
				pthread_spin_unlock(&s->mtx);
				end_stream(w->upnp,m);
				pthread_spin_destroy(&s->mtx);
				free(s);
				goto out;
			}

			pthread_spin_lock(&s->mtx);
			s->fd=w->client;
			pthread_spin_unlock(&s->mtx);

			pthread_mutex_lock(&w->upnp->htx);
			s->next=w->upnp->hlist;
			w->upnp->hlist=s;
			w->upnp->playhttp++;
			goto done;
		}
	}
	else
	{
		if(w->upnp->httplimit&&getrunning(w->upnp)>=w->upnp->httplimit)
		{
			httperr(w->client,503);
			goto out;
		}

		if(w->upnp->cb(SATIP_HTTPREQ,param,w->upnp->priv))
			httperr(w->client,404);
	}

out:	close(w->client);

	pthread_mutex_lock(&w->upnp->htx);
done:	w->upnp->running--;
	pthread_mutex_unlock(&w->upnp->htx);

	if(r)free(r);
	free(w);

	pthread_exit(NULL);
}

static int statsreq(UPNP *upnp)
{       
	int len; 
	uint32_t mask=RTEXT_FILTER_VF;
	struct rtattr *rta;
	struct  
	{       
		struct nlmsghdr n;
		struct ifinfomsg i;
		char space[1024];
	} req;

	memset(&req,0,sizeof(req));
	req.n.nlmsg_len=NLMSG_LENGTH(sizeof(struct ifinfomsg));
	req.n.nlmsg_flags=NLM_F_REQUEST;
	req.n.nlmsg_type=RTM_GETLINK;
	req.n.nlmsg_seq=0xdeadbeef;
	req.i.ifi_family=AF_PACKET;

	len=strlen(upnp->dev)+1; 
	rta=(struct rtattr *)(((char *)&req)+NLMSG_ALIGN(req.n.nlmsg_len));
	rta->rta_type=IFLA_IFNAME;
	rta->rta_len=RTA_LENGTH(len);
	memcpy(RTA_DATA(rta),upnp->dev,len);
	req.n.nlmsg_len=NLMSG_ALIGN(req.n.nlmsg_len+RTA_ALIGN(rta->rta_len));

	rta=(struct rtattr *)(((char *)&req)+NLMSG_ALIGN(req.n.nlmsg_len));
	rta->rta_type=IFLA_EXT_MASK;
	rta->rta_len=RTA_LENGTH(sizeof(uint32_t));
	memcpy(RTA_DATA(rta),&mask,sizeof(uint32_t));
	req.n.nlmsg_len=NLMSG_ALIGN(req.n.nlmsg_len+RTA_ALIGN(rta->rta_len));

	if(send(upnp->nfd,&req,req.n.nlmsg_len,0)<0)return -1;
	return 0;
}

static int getaddrlist(UPNP *upnp,int mode)
{
	struct
	{
		struct nlmsghdr n;
		struct ifaddrmsg r;
		char data[16];
	} req;
	struct rtattr *rta;

	memset(&req,0,sizeof(req));
	req.n.nlmsg_len=NLMSG_LENGTH(sizeof(struct ifaddrmsg));
	req.n.nlmsg_flags=NLM_F_REQUEST|NLM_F_ROOT;
	req.n.nlmsg_type=RTM_GETADDR;
	req.n.nlmsg_seq=++upnp->seq;
	req.r.ifa_family=mode?AF_INET6:AF_INET;
	rta=(struct rtattr *)(((char *)&req)+NLMSG_ALIGN(req.n.nlmsg_len));
	rta->rta_len=RTA_LENGTH(4);
	if(send(upnp->nfd,&req,req.n.nlmsg_len,0)<0)return -1;
	return 0;
}

static void nlmsgwork(UPNP *upnp,void *data,int events)
{
	int len;
	int rtl;
	int v;
	int wrk=0;
	struct nlmsghdr *nlh;
	struct ifaddrmsg *ifa;
	struct ifinfomsg *ifi;
	struct rtattr *rth;
	struct in_addr *v4;
	struct in6_addr *v6;
	struct rtnl_link_stats64 *s64;
	struct rtnl_link_stats *s32;
	char dev[IFNAMSIZ+1];
	char *bfr;

	if(!(bfr=malloc(MAXBUFSIZE)))goto err1;

	if((len=recv(upnp->nfd,bfr,MAXBUFSIZE,0))<=0)goto err2;
	nlh=(struct nlmsghdr *)bfr;

	if(upnp->seq!=nlh->nlmsg_seq&&nlh->nlmsg_seq!=0xdeadbeef)goto err2;

	while((NLMSG_OK(nlh,len)))
	{
		switch(nlh->nlmsg_type)
		{
		case NLMSG_DONE:
			if(upnp->nlstate!=upnp->level)
			{
				upnp->nlstate=upnp->level;
				if(getaddrlist(upnp,1))upnp->nlstate=0;
			}
			else
			{
				upnp->seq=0;
				if(upnp->level==SATIP_V6_SITE)
					if(!ifaddr_select(upnp,
					    &upnp->devaddr[SATIP_V6_SITE].a6))
				{
					upnp->ifmap|=4;
					wrk=1;
				}
			}
			break;

		case RTM_NEWLINK:
			upnp->tx64valid<<=1;
			upnp->tx32valid<<=1;
			ifi=(struct ifinfomsg *)NLMSG_DATA(nlh);
			rth=IFLA_RTA(ifi);
			rtl=IFLA_PAYLOAD(nlh);
			while(rtl&&RTA_OK(rth,rtl))
			{
				switch(rth->rta_type)
				{
				case IFLA_STATS64:
					s64=(struct rtnl_link_stats64 *)
						RTA_DATA(rth);
					upnp->tx64[1]=upnp->tx64[0];
					upnp->tx64[0]=s64->tx_bytes;
					upnp->tx64valid|=1;
					break;

				case IFLA_STATS:
					s32=(struct rtnl_link_stats *)
						RTA_DATA(rth);
					upnp->tx32[1]=upnp->tx32[0];
					upnp->tx32[0]=s32->tx_bytes;
					upnp->tx32valid|=1;
					break;
				}
				rth=RTA_NEXT(rth,rtl);
			}
			if((upnp->tx64valid&3)==3)upnp->bps=
				upnp->tx64[0]-upnp->tx64[1];
			else if((upnp->tx32valid&3)==3)upnp->bps=
				(unsigned long long)
				(upnp->tx32[0]-upnp->tx32[1]);
			else upnp->bps=0;
			break;

		case RTM_NEWADDR:
			ifa=(struct ifaddrmsg *)NLMSG_DATA(nlh);
			rth=IFA_RTA(ifa);
			rtl=IFA_PAYLOAD(nlh);
			while(rtl&&RTA_OK(rth,rtl))
			{
				switch(rth->rta_type)
				{
				case IFA_LOCAL:
					if(ifa->ifa_family!=AF_INET)break;
					if(ifa->ifa_flags&IFA_F_SECONDARY)break;
					if_indextoname(ifa->ifa_index,dev);
					if(strcmp(dev,upnp->dev))break;
					v4=(struct in_addr *)RTA_DATA(rth);
					if(v4->s_addr==
						upnp->devaddr[SATIP_V4_ONLY]
						.a4.s_addr)break;
					upnp->devaddr[SATIP_V4_ONLY].a4.s_addr=
						v4->s_addr;
					upnp->ifmap|=1;
					wrk=1;
					break;

				case IFA_ADDRESS:
					if(upnp->level==SATIP_V4_ONLY)break;
					if(ifa->ifa_family!=AF_INET6)break;
					if_indextoname(ifa->ifa_index,dev);
					v6=(struct in6_addr *)RTA_DATA(rth);
					if((v=a6linklocal(v6)))
					{
						if(v==3)break;
						if(!memcmp(upnp->devaddr[
							SATIP_V6_LINK]
							.a6.s6_addr,v6->s6_addr,
							16))break;
						memcpy(upnp->devaddr[
							SATIP_V6_LINK]
							.a6.s6_addr,v6->s6_addr,
							16);
						upnp->ifmap|=2;
						wrk=1;
						break;
					}
					if(upnp->level!=SATIP_V6_SITE)break;
					ifaddr_add(upnp,v6);
					if(upnp->nlstate!=upnp->level)break;
					if(upnp->ifmap&4)break;
					if(!ifaddr_select(upnp,
						&upnp->devaddr[SATIP_V6_SITE]
						.a6))
					{
						upnp->ifmap|=4;
						wrk=1;
					}
					break;
				}
				rth=RTA_NEXT(rth,rtl);
			}
			break;

		case RTM_DELADDR:
			ifa=(struct ifaddrmsg *)NLMSG_DATA(nlh);
			rth=IFA_RTA(ifa);
			rtl=IFA_PAYLOAD(nlh);
			while(rtl&&RTA_OK(rth,rtl))
			{
				switch(rth->rta_type)
				{
				case IFA_LOCAL:
					if(ifa->ifa_family!=AF_INET)break;
					if(ifa->ifa_flags&IFA_F_SECONDARY)break;
					if_indextoname(ifa->ifa_index,dev);
					if(strcmp(dev,upnp->dev))break;
					v4=(struct in_addr *)RTA_DATA(rth);
					if(v4->s_addr!=upnp->devaddr[
						SATIP_V4_ONLY].a4.s_addr)break;
					upnp->devaddr[SATIP_V4_ONLY].a4.s_addr=
						0;
					upnp->ifmap&=~1;
					wrk=1;
					break;

				case IFA_ADDRESS:
					if(upnp->level==SATIP_V4_ONLY)break;
					if(ifa->ifa_family!=AF_INET6)break;
					if_indextoname(ifa->ifa_index,dev);
					v6=(struct in6_addr *)RTA_DATA(rth);
					if((v=a6linklocal(v6)))
					{
						if(v==3)break;
						if(memcmp(upnp->devaddr[
							SATIP_V6_LINK]
							.a6.s6_addr,v6->s6_addr,
							16))break;
						memset(upnp->devaddr[
							SATIP_V6_LINK]
							.a6.s6_addr,0,16);
						upnp->ifmap&=~2;
						wrk=1;
						break;
					}
					if(upnp->level!=SATIP_V6_SITE)break;
					ifaddr_del(upnp,v6);
					if(memcmp(upnp->devaddr[SATIP_V6_SITE]
						.a6.s6_addr,v6->s6_addr,16))
							break;
					upnp->ifmap&=~4;
					wrk=1;
					if(!ifaddr_select(upnp,
						&upnp->devaddr[SATIP_V6_SITE]
							.a6))upnp->ifmap|=4;
					break;
				}
				rth=RTA_NEXT(rth,rtl);
			}
			break;
		}
		nlh=NLMSG_NEXT(nlh,len);
	}

	if(!wrk)goto err2;

	switch(upnp->state)
	{
	case IFWAIT:
	case RESTART:
	case TERMINATE:
		break;
	case RUNNING:
		upnp->cb(SATIP_STOPPING,NULL,upnp->priv);
	default:if(upnp->state>ANNOUNCE)for(v=0;v<=upnp->level;v++)
			release(upnp,v);
		upnp->state=RESTART;
		break;
	}

err2:	free(bfr);
err1:	return;
}

static void terminator(UPNP *upnp,void *data,int events)
{
	int i;

	if(upnp->state==RUNNING)upnp->cb(SATIP_STOPPING,NULL,upnp->priv);
	if(upnp->state>ANNOUNCE)for(i=0;i<=upnp->level;i++)release(upnp,i);
	upnp->state=TERMINATE;
}

static void webconnect(UPNP *upnp,void *data,int events)
{
	int l;
	int which=(int)((long)(((FUNC *)data)->which));
	int client;
	socklen_t ll;
	CLNDTA *clndta;
	pthread_t unused;
	SOCK addr;

	ll=sizeof(addr);
	if((client=accept4(upnp->tcp[which],(struct sockaddr *)&addr,&ll,
		SOCK_CLOEXEC|SOCK_NONBLOCK))==-1)goto err1;
	if(invalid_sock(&addr,ll))goto err2;
	l=1;
	if(setsockopt(client,IPPROTO_TCP,TCP_NODELAY,&l,sizeof(l)))goto err2;

	if(!(clndta=malloc(sizeof(CLNDTA))))goto err2;
	clndta->upnp=upnp;
	clndta->addr=addr;
	clndta->client=client;
	clndta->state=upnp->state;
	clndta->which=which;

	if(pthread_create(&unused,&upnp->attr,webwork,clndta))goto err3;

	pthread_mutex_lock(&upnp->htx);
	upnp->running++;
	pthread_mutex_unlock(&upnp->htx);

	return;

err3:	free(clndta);
err2:	close(client);
err1:	return;
}

static void rtspwork(UPNP *upnp,void *data,int events)
{
	int f=0;
	int l;
	int s;
	RTSP *rtspdta;
	RTSP **d;
	CHUNK *c;
	struct epoll_event e;

	rtspdta=(RTSP *)data;

	if(rtspdta->tx)
	{
		c=rtspdta->tx;
		if((l=write(rtspdta->client,c->msg+c->sent,c->len-c->sent))<=0)
		{
			f=1;
			goto out;
		}
		c->sent+=l;
		if(c->sent==c->len)
		{
			rtspdta->tx=c->next;
			free(c);
			if(!rtspdta->tx)
			{
				rtspdta->idle=0;
				e.events=EPOLLIN|EPOLLRDHUP;
				e.data.ptr=rtspdta;
				if(epoll_ctl(upnp->ffd,EPOLL_CTL_MOD,
					rtspdta->client,&e)||rtspdta->close)
				{
					f=1;
					goto out;
				}
			}
		}
		return;
	}

	if(events&EPOLLIN)
	{
		if((l=read(rtspdta->client,rtspdta->bfr+rtspdta->fill,
			sizeof(rtspdta->bfr)-rtspdta->fill))<=0)f=1;
		else rtspdta->fill+=l;
	}
	if(events&EPOLLRDHUP)f=1;

	for(s=0,l=0;l<rtspdta->fill;l++)switch(rtspdta->bfr[l])
	{
	case '\n':
		if(++s<2)break;
		rtspdta->bfr[l]=0;
		if(rtsp_request(upnp,rtspdta)||(rtspdta->close&&!rtspdta->tx))
		{
			f=1;
			goto out;
		}
		rtspdta->idle=0;
		if((rtspdta->fill-=++l))memmove(rtspdta->bfr,
			rtspdta->bfr+l,rtspdta->fill);
		s=0;
		l=-1;
		break;

	default:if(rtspdta->bfr[l]<0x20)
		{
			f=1;
			goto out;
		}
	case '\t':
		s=0;
	case '\r':
		break;
	}

	if(rtspdta->fill==sizeof(rtspdta->bfr))f=1;

out:	if(f)for(d=&upnp->rlist;*d;d=&(*d)->next)if(*d==rtspdta)
	{
		*d=rtspdta->next;
		epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,rtspdta->client,NULL);
		close(rtspdta->client);
		while(rtspdta->tx)
		{
			c=rtspdta->tx;
			rtspdta->tx=c->next;
			free(c);
		}
		free(rtspdta);
		break;
	}
}

static void rtspconnect(UPNP *upnp,void *data,int events)
{
	int l;
	int which=(int)((long)(((FUNC *)data)->which));
	int client;
	socklen_t ll;
	RTSP *rtspdta;
	SATIP_DATA peer;
	struct epoll_event e;
	SOCK addr;

	ll=sizeof(addr);
	if((client=accept4(upnp->rcp[which],(struct sockaddr *)&addr,&ll,
		SOCK_CLOEXEC|SOCK_NONBLOCK))==-1)goto err1;
	if(invalid_sock(&addr,ll))goto err2;
	l=1;
	if(setsockopt(client,IPPROTO_TCP,TCP_NODELAY,&l,sizeof(l)))goto err2;
	peer.intval=sockfam(&addr);
	peer.ptrval=&addr;
	if(upnp->cb(SATIP_PEEROK,&peer,upnp->priv))goto err2;
	if(upnp->cb(SATIP_RTSPOK,&peer,upnp->priv))goto err2;
	if(upnp->state!=RUNNING)goto err2;

	if(!(rtspdta=malloc(sizeof(RTSP))))goto err2;
	rtspdta->client=client;
	rtspdta->func=rtspwork;
	rtspdta->tx=NULL;
	rtspdta->close=0;
	rtspdta->idle=0;
	rtspdta->fill=0;
	rtspdta->which=which;
	rtspdta->devaddr=upnp->devaddr[which];
	sock2addr(&rtspdta->addr,&addr);

	e.events=EPOLLIN|EPOLLRDHUP;
	e.data.ptr=rtspdta;
	if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,client,&e))goto err3;

	rtspdta->next=upnp->rlist;
	upnp->rlist=rtspdta;

	return;

err3:	free(rtspdta);
err2:	close(client);
err1:	return;
}

static void upnpwork(UPNP *upnp,int which,int s)
{
	int l;
	int i;
	int multi=-1;
	int idx=0;
	short port;
	struct cmsghdr *cm;
	struct in_pktinfo *p4;
	struct in6_pktinfo *p6;
	struct msghdr mh;
	struct iovec io;
	SOCK a;
	ADDR addr;
	char bfr[2048];
	char cmsg[256];

	mh.msg_name=&a;
	mh.msg_namelen=sizeof(a);
	mh.msg_iov=&io;
	mh.msg_iovlen=1;
	mh.msg_control=cmsg;
	mh.msg_controllen=sizeof(cmsg);
	mh.msg_flags=0;
	io.iov_base=bfr;
	io.iov_len=sizeof(bfr)-1;

	if((l=recvmsg(s,&mh,0))==-1)return;
	if(invalid_sock(&a,mh.msg_namelen))return;

	for(cm=CMSG_FIRSTHDR(&mh);cm;cm=CMSG_NXTHDR(&mh,cm))
		switch(cm->cmsg_level)
	{
	case IPPROTO_IP:
		if(sockfam(&a)!=AF_INET||cm->cmsg_type!=IP_PKTINFO)break;
		p4=(struct in_pktinfo *)CMSG_DATA(cm);
		idx=p4->ipi_ifindex;
		if((ntohl(p4->ipi_addr.s_addr)&0xf0000000)==0xe0000000)multi=1;
		else multi=0;
		break;

	case IPPROTO_IPV6:
		if(sockfam(&a)!=AF_INET6||cm->cmsg_type!=IPV6_PKTINFO)break;
		p6=(struct in6_pktinfo *)CMSG_DATA(cm);
		idx=p6->ipi6_ifindex;
		if(p6->ipi6_addr.s6_addr[0]==0xff)multi=1;
		else multi=0;
		break;
	}

	if(multi==-1||idx!=upnp->devidx||upnp->state==ANNOUNCE)return;

	for(port=psget(&a),i=0;i<=upnp->level;i++)
		if(!ascmp(&upnp->devaddr[i],&a))
			if(!multi||port==htons(upnp->port))return;

	bfr[l]=0;
	sock2addr(&addr,&a);

	if(multi)if(addr2level(&addr)!=which)return;

	if(msghandler(upnp,bfr,&addr,multi,port,which))
	{
		if(upnp->state>ANNOUNCE)
			for(l=0;l<=upnp->level;l++)release(upnp,l);
		if(upnp->state!=TERMINATE)
		{
			if(upnp->state==RUNNING)
				upnp->cb(SATIP_STOPPING,NULL,upnp->priv);
			upnp->state=RESTART;
		}
	}
}

static void upnpudp(UPNP *upnp,void *data,int events)
{
	int which=(int)((long)(((FUNC *)data)->which));

	upnpwork(upnp,which,upnp->udp[which]);
}

static void upnpmdp(UPNP *upnp,void *data,int events)
{
	int which=(int)((long)(((FUNC *)data)->which));

	upnpwork(upnp,which,upnp->mdp[which]);
}

static void activate_mcast(UPNP *upnp,ADDR *addr)
{
	int idx;
	SESSION *s;
	MEMBER *m;

	if(upnp->bpslimit&&upnp->bps>=upnp->bpslimit)return;

	idx=addr2idx(addr);
	for(s=upnp->mcast[idx];s;s=s->next)if(!ascmp(addr,&s->rtp))
	{
		pthread_spin_lock(&s->mtx);

		if(!(s->play&PLAY_PERSIST))
		{
			if(!(s->play&PLAY_PLAYING))
			{
				pthread_spin_unlock(&s->mtx);
				s->fill=0;
				if(!(m=play_stream(upnp,&s->state.tune,
					&s->state.set,s,0)))
						pthread_spin_lock(&s->mtx);
				else
				{
					s->handle=m;
					pthread_spin_lock(&s->mtx);
					s->play=PLAY_PLAYING;
					if(!s->state.tune.fe)
						s->play|=PLAY_AUTOFE;
					upnp->playsessions++;
				}
			}
			if(s->play&PLAY_PLAYING)
			{
				s->v3idle=V3IDLE<<3;
				s->v3mode=0;
				upnp->v3hint[idx>>3]|=1<<(idx&7);
			}
		}

		pthread_spin_unlock(&s->mtx);
	}
}

static void deadline_mcast(UPNP *upnp,ADDR *addr)
{
	int idx;
	SESSION *s;

	idx=addr2idx(addr);
	if(upnp->v3hint[idx>>3]&(1<<(idx&0x7)))
		for(s=upnp->mcast[idx];s;s=s->next)
			if(!ascmp(addr,&s->rtp))if(s->v3idle&&!s->v3mode)
	{
		s->v3idle=V3FINAL<<4;
		s->v3mode=1;
	}
}

static void igmpwork(UPNP *upnp,void *data,int events)
{
	int i;
	int l;
	int w;
	int x=0;
	unsigned short sum;
	struct cmsghdr *cm;
	struct msghdr mh;
	struct iovec io;
	union
	{       
		unsigned char *bfr1;
		unsigned short *bfr2;
		unsigned int *bfr4;
	} u;
	ADDR addr;
	unsigned char msg[2048];
	char cmsg[256];

	mh.msg_name=NULL;
	mh.msg_namelen=0;
	mh.msg_iov=&io;
	mh.msg_iovlen=1;
	mh.msg_control=cmsg;
	mh.msg_controllen=sizeof(cmsg);
	mh.msg_flags=0;
	io.iov_base=msg;
	io.iov_len=sizeof(msg);

	if((l=recvmsg(upnp->qry[SATIP_V4_ONLY],&mh,0))<1)return;

	for(cm=CMSG_FIRSTHDR(&mh);cm;cm=CMSG_NXTHDR(&mh,cm))
		if(cm->cmsg_level==IPPROTO_IP&&cm->cmsg_type==IP_PKTINFO)
			x=((struct in_pktinfo *)CMSG_DATA(cm))->ipi_ifindex;

	if(upnp->state!=RUNNING||x!=upnp->devidx)return;
	u.bfr1=msg;

	if((u.bfr1[0]>>4)!=4)return;
	w=(u.bfr1[0]&0xf)<<2;
	if(w<20||l<w)return;
	sum=u.bfr2[5];
	u.bfr2[5]=0;
	if(sum!=chksum(u.bfr2,w>>1))return;
	if(l<ntohs(u.bfr2[1]))return;
	if(u.bfr1[9]!=IPPROTO_IGMP)return;
	if(u.bfr4[4]==mchosts[SATIP_V4_ONLY].a4.s_addr)
	{
		if(l==8)goto quiet;
		else if(l==12)if(upnp->igmpv3==SATIP_IGMPQUERY)
			if(ntohl(u.bfr4[3])<
				ntohl(upnp->devaddr[SATIP_V4_ONLY].a4.s_addr))
		{
quiet:			upnp->igmpwait=V3NOQRY;
			if(upnp->igmpquerier)
			{
				upnp->igmpquerier=0;
				for(i=0;i<8;i++)upnp->igmpq[i][0]=0;
			}
		}
		return;
	}
	if(u.bfr4[4]!=mcreports[SATIP_V4_ONLY].a4.s_addr)return;
	u.bfr1+=w;
	l-=w;

	if(l<=8||(l&7)||u.bfr1[0]!=0x22)return;
	sum=u.bfr2[1];
	u.bfr2[1]=0;
	if(sum!=chksum(u.bfr2,l>>1))return;
	if(!(w=ntohs(u.bfr2[3])))return;
	u.bfr1+=8;
	l-=8;

	for(addr.family=AF_INET,i=0;i<w;i++)
	{
		if(l<8)return;
		x=8+(((int)u.bfr1[1])<<2)+(((int)u.bfr2[1])<<2);
		if(l<x)return;
		switch(u.bfr1[0])
		{
		case 0x02:
		case 0x04:
			addr.a4.s_addr=u.bfr4[1];
			if(invalid_mcast(&addr))break;
			activate_mcast(upnp,&addr);
			break;

		case 0x03:
			addr.a4.s_addr=u.bfr4[1];
			switch(invalid_mcast(&addr))
			{
			case 0:	deadline_mcast(upnp,&addr);
			case -2:if(upnp->igmpquerier)
				{
					sendv3(upnp,upnp->qry[SATIP_V4_ONLY],
						u.bfr4[1]);
					if(upnp->igmpq[upnp->ms125][0]<QLEN-1)
						upnp->igmpq[upnp->ms125][
							++upnp->igmpq[
							upnp->ms125][0]]=
								u.bfr4[1];
				}
			case -1:break;
			}
			break;
		}
		u.bfr1+=x;
		l-=x;
	}
}

static void mldwork(UPNP *upnp,void *data,int events)
{
	int i;
	int l;
	int n;
	int x=0;
	unsigned short sum;
	struct cmsghdr *cm;
	struct msghdr mh;
	struct iovec io;
	socklen_t ll;
	SOCK a;
	ADDR addr;
	unsigned char msg[2088];
	char cmsg[256];

	mh.msg_name=&a;
	mh.msg_namelen=sizeof(a);
	mh.msg_iov=&io;
	mh.msg_iovlen=1;
	mh.msg_control=cmsg;
	mh.msg_controllen=sizeof(cmsg);
	mh.msg_flags=0;
	io.iov_base=msg+40;
	io.iov_len=sizeof(msg)-40;

	if((l=recvmsg(upnp->qry[SATIP_V6_LINK],&mh,0))<1)return;
	if(invalid_sock(&a,mh.msg_namelen))return;

	for(cm=CMSG_FIRSTHDR(&mh);cm;cm=CMSG_NXTHDR(&mh,cm))
		if(cm->cmsg_level==IPPROTO_IPV6&&cm->cmsg_type==IPV6_PKTINFO)
			x=((struct in6_pktinfo *)CMSG_DATA(cm))->ipi6_ifindex;

	if(upnp->state!=RUNNING||x!=upnp->devidx)return;
	if(memcmp(a.a6.sin6_addr.s6_addr,"\xfe\x80\x00\x00\x00\x00\x00\x00",8))
		return;

	switch(msg[40])
	{
	case 0x82:
		if(l<24)break;
		memcpy(msg,a.a6.sin6_addr.s6_addr,16);
		memcpy(msg+16,mchosts[SATIP_V6_LINK].a6.s6_addr,16);
		msg[32]=0;
		msg[33]=0;
		msg[34]=l>>8;
		msg[35]=l&0xff;
		msg[36]=0;
		msg[37]=0;
		msg[38]=0;
		msg[39]=IPPROTO_ICMPV6;
		sum=*((unsigned short *)(msg+42));
		*((unsigned short *)(msg+42))=0;
		if(sum!=chksum((unsigned short *)msg,(l+40)>>1))break;
		if(memcmp(msg+48,in6addr_any.s6_addr,16))break;
		if(l==24)goto quiet;
		else if(l==28)if(upnp->mldv2==SATIP_MLDQUERY)
			if(memcmp(a.a6.sin6_addr.s6_addr,
				upnp->devaddr[SATIP_V6_LINK].a6.s6_addr,16)<0)
		{
quiet:			upnp->mldwait=V3NOQRY;
			if(upnp->mldquerier)
			{
				upnp->mldquerier=0;
				for(i=0;i<8;i++)upnp->mldq[i].total=0;
			}
		}
		break;

	case 0x8f:
		if((l&3)||l<28)break;
		memcpy(msg,a.a6.sin6_addr.s6_addr,16);
		memcpy(msg+16,mcreports[SATIP_V6_LINK].a6.s6_addr,16);
		msg[32]=0;
		msg[33]=0;
		msg[34]=l>>8;
		msg[35]=l&0xff;
		msg[36]=0;
		msg[37]=0;
		msg[38]=0;
		msg[39]=IPPROTO_ICMPV6;
		sum=*((unsigned short *)(msg+42));
		*((unsigned short *)(msg+42))=0;
		if(sum!=chksum((unsigned short *)msg,(l+40)>>1))break;
		if(!(ll=ntohs(*((unsigned short *)(msg+46)))))break;

		for(addr.family=AF_INET6,x=8,i=0;i<ll;i++)
		{
			if(l<x+20)break;
			n=*((unsigned short *)(msg+42+x));
			if(l<x+20+(n<<4))break;
			switch(msg[40+x])
			{
			case 0x02:
			case 0x04:
				memcpy(addr.a6.s6_addr,msg+44+x,16);
				if(invalid_mcast(&addr))break;
				activate_mcast(upnp,&addr);
				break;
			case 0x03:
				memcpy(addr.a6.s6_addr,msg+44+x,16);
				switch(invalid_mcast(&addr))
				{
				case 0:	deadline_mcast(upnp,&addr);
				case -2:if(!upnp->mldquerier)break;
					sendm2(upnp,upnp->qry[SATIP_V6_LINK],
						msg+44+x);
					if(upnp->mldq[upnp->ms125].total>=
						QLEN-1)break;
					memcpy(&upnp->mldq[upnp->ms125]
						.a6[upnp->mldq[
						upnp->ms125].total++][0],
						msg+44+x,16);
				case -1:break;
				}
			}
			x+=20+(n<<4);
		}
		break;
	}
}

static void timetick(UPNP *upnp,void *data,int events)
{
	int i;
	int j;
	int k;
	int l;
	uint64_t dummy;
	COLL *c;
	COLL **d;
	RTSP *rtspdta;
	RTSP **dr;
	SESSION *s;
	SESSION *ms;
	STREAM *hs;
	STREAM **ds;
	CHUNK *ch;
	SATIP_DATA dta[5];
	SOCK info[3];
	struct pollfd hp;
	struct epoll_event e;
	static FUNC mupnp[3]={{(void *)SATIP_V4_ONLY,upnpmdp},
		{(void *)SATIP_V6_LINK,upnpmdp},{(void *)SATIP_V6_SITE,
		upnpmdp}};
	static FUNC wconn[3]={{(void *)SATIP_V4_ONLY,webconnect},
		{(void *)SATIP_V6_LINK,webconnect},{(void *)SATIP_V6_SITE,
		webconnect}};
	static FUNC rconn[3]={{(void *)SATIP_V4_ONLY,rtspconnect},
		{(void *)SATIP_V6_LINK,rtspconnect},{(void *)SATIP_V6_SITE,
		rtspconnect}};
	static FUNC idata={NULL,igmpwork};
	static FUNC mdata={NULL,mldwork};
	static FUNC ucast={NULL,upnpudp};

	dummy=read(upnp->efd,&dummy,sizeof(dummy));

	if(upnp->state==TERMINATE)return;

	upnp->ms125=(upnp->ms125+1)&0x7;

	if(LIKELY(upnp->ms125))
	{
		if(UNLIKELY(upnp->state!=RUNNING))return;
	}
	else if(upnp->state>=ANNOUNCE&&upnp->bpslimit)statsreq(upnp);

	switch(upnp->state)
	{
	case IFWAIT:
		if(upnp->ifmap!=(1<<(upnp->level+1))-1)break;
		if(!(upnp->devidx=if_nametoindex(upnp->dev)))break;
		bootinc(upnp);
		upnp->state++;

	case SETUP:
		l=0;
		e.events=EPOLLIN;
		upnp->http=upnp->cfghttp;
		upnp->rtsp=upnp->cfgrtsp;

		for(i=0;i<=upnp->level;i++)
		{
			if((upnp->mdp[i]=mcast(upnp,&mcaddr[i],MPORT,
				upnp->mttl,i))==-1)goto fail;

			e.data.ptr=&mupnp[i];
			if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,upnp->mdp[i],&e))
				goto fail;

			if((upnp->udp[i]=udp(upnp,&upnp->devaddr[i],
				upnp->port,i))==-1)
			{
				l=1;
				goto fail;
			}

			e.data.ptr=&ucast;
			if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,upnp->udp[i],&e))
				goto fail;

			if((upnp->tcp[i]=tcp(upnp,&upnp->devaddr[i],
				&upnp->http))==-1)goto fail;
			e.data.ptr=&wconn[i];
			if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,upnp->tcp[i],&e))
				goto fail;

			if((upnp->rcp[i]=tcp(upnp,&upnp->devaddr[i],
				&upnp->rtsp))==-1)goto fail;
			e.data.ptr=&rconn[i];
			if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,upnp->rcp[i],&e))
				goto fail;
		}

		switch(upnp->igmpv3)
		{
		case SATIP_IGMPQUERY:
		case SATIP_IGMPFORCE:
			upnp->igmpwait=0;
			upnp->igmpnext=V3TIME/4;
			upnp->igmpquerier=1;
		case SATIP_IGMPSNOOP:
			if((upnp->qry[SATIP_V4_ONLY]=igmpv3(upnp))==-1)
				goto fail;
			e.data.ptr=&idata;
			if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,
				upnp->qry[SATIP_V4_ONLY],&e))goto fail;
			else for(i=0;i<8;i++)upnp->igmpq[i][0]=0;
			break;
		}

		if(upnp->level!=SATIP_V4_ONLY)switch(upnp->mldv2)
		{
		case SATIP_MLDQUERY:
		case SATIP_MLDFORCE:
			upnp->mldwait=0;
			upnp->mldnext=V3TIME/4;
			upnp->mldquerier=1;
		case SATIP_MLDSNOOP:
			if((upnp->qry[SATIP_V6_LINK]=mldv2(upnp))==-1)goto fail;
			e.data.ptr=&mdata;
			if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,
				upnp->qry[SATIP_V6_LINK],&e))
			{
fail:				closer(upnp,&upnp->qry[SATIP_V6_LINK]);
				closer(upnp,&upnp->qry[SATIP_V4_ONLY]);
				for(i=0;i<=upnp->level;i++)
				{
					closer(upnp,&upnp->rcp[i]);
					closer(upnp,&upnp->tcp[i]);
					closer(upnp,&upnp->mdp[i]);
					closer(upnp,&upnp->udp[i]);
				}
				if(l)
				{
					upnp->port++;
					if(upnp->port<1024)upnp->port=1024;
				}
				goto fx;
			}
			else for(i=0;i<8;i++)upnp->mldq[i].total=0;
			break;
		}

		upnp->tx64[0]=upnp->tx64[1]=0;
		upnp->tx32[0]=upnp->tx32[1]=0;
		upnp->tx64valid=upnp->tx32valid=0;
		upnp->bps=0;

		upnp->state++;
fx:		break;

	case ANNOUNCE:
		for(i=0;i<=upnp->level;i++)if(announce(upnp,i))break;
		if(i>upnp->level)
		{
			upnp->state=IDWAIT1;
			upnp->flag=0;
		}
		break;

	case IDWAIT1:
		upnp->rem=(upnp->age>>1)-20;
	case IDWAIT2:
	case IDWAIT3:
	case IDWAIT4:
		upnp->state++;
		break;

	case PRERUN:
		if(upnp->flag)
		{
			for(i=0;i<=upnp->level;i++)release(upnp,i);
			bootinc(upnp);
			newdevid(upnp);
			upnp->state=ANNOUNCE;
			break;
		}
		else
		{
			savedevid(upnp);
			while(upnp->coll)
			{
				c=upnp->coll;
				upnp->coll=c->next;
				free(c);
			}
			if(upnp->locked)upnp->rtsplock=upnp->httplock=1;
			dta[0].intval=upnp->http;
			dta[0].ptrval=NULL;
			dta[1].intval=upnp->rtsp;
			dta[1].ptrval=NULL;
			for(i=0;i<=upnp->level;i++)
			{
				addr2sock(&info[i],&upnp->devaddr[i],0,
					upnp->devidx);
				dta[i+2].intval=sockfam(&info[i]);
				dta[i+2].ptrval=&info[i];
			}
			upnp->cb(SATIP_RUNNING,dta,upnp->priv);
			upnp->state++;
		}

	case RUNNING:
		if(upnp->igmpquerier)while(upnp->igmpq[upnp->ms125][0])sendv3(
			upnp,upnp->qry[SATIP_V4_ONLY],
			upnp->igmpq[upnp->ms125]
			[upnp->igmpq[upnp->ms125][0]--]);

		if(upnp->mldquerier)while(upnp->mldq[upnp->ms125].total)sendm2(
			upnp,upnp->qry[SATIP_V6_LINK],
			&upnp->mldq[upnp->ms125].a6[--upnp->mldq[upnp->ms125]
			.total][0]);

		for(i=0;i<32;i++)if(upnp->v3hint[i])for(l=i<<3,j=0;j<8;j++,l++)
			if(upnp->v3hint[i]&(1<<j))
		{
			for(k=0,s=upnp->mcast[l];s;s=s->next)if(s->v3idle)
			{
				if(--s->v3idle)k++;
				else
				{
					s->v3mode=0;

					pthread_spin_lock(&s->mtx);

					if((s->play&(PLAY_PLAYING|PLAY_PERSIST))
						==PLAY_PLAYING)
					{
						pthread_spin_unlock(&s->mtx);

						if(s->handle)
						{
							end_stream(upnp,
								s->handle);
							s->handle=NULL;
						}

						pthread_spin_lock(&s->mtx);

						if(s->play&PLAY_AUTOFE)
							s->state.tune.fe=0;
						s->state.level=0;
						s->state.lock=0;
						s->state.quality=0;
						s->play=0;
						upnp->playsessions--;
					}

					pthread_spin_unlock(&s->mtx);
				}
			}
			if(!k)upnp->v3hint[i]&=~(1<<j);
		}

		switch(upnp->ms125)
		{
		case 0:	if(!--upnp->rem)
			{
				upnp->rem=(upnp->age>>1)-15;
				for(i=0;i<=upnp->level;i++)if(announce(upnp,i))
				{
					upnp->cb(SATIP_STOPPING,NULL,
						upnp->priv);
					for(i=0;i<=upnp->level;i++)
						release(upnp,i);
					upnp->state=RESTART;
					break;
				}
			}
			break;

		case 1:	for(dr=&upnp->rlist;*dr;)if(++(*dr)->idle>
				((*dr)->tx?MAX_TX_TIME:upnp->timeout))
			{
				rtspdta=*dr;
				*dr=rtspdta->next;
				epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,
					rtspdta->client,NULL);
				close(rtspdta->client);
				while(rtspdta->tx)
				{
					ch=rtspdta->tx;
					rtspdta->tx=ch->next;
					free(ch);
				}
				free(rtspdta);
			}
			else dr=&(*dr)->next;
			break;

		case 2:	for(s=upnp->slist;s;s=s->next)if(s->idle!=-1)
			{
				if(!upnp->strict&&__sync_fetch_and_and(
					&s->rrrecv,~1))s->idle=0;
				else s->idle++;
			}
			break;

		case 3:	for(s=upnp->slist;s;)if(s->idle>upnp->timeout)
			{
				if(s!=s->master)
				{
					pthread_spin_lock(&s->master->mtx);

					if(s->master->play)l=0;
					else for(l=1,ms=s->master->slave;ms;
						ms=ms->next)if(ms!=s&&ms->play)
					{
						l=0;
						break;
					}

					pthread_spin_unlock(&s->master->mtx);
				}
				else l=1;

				if(l&&s->master->handle)
					end_stream(upnp,s->master->handle);

				free_sess(upnp,s);
				s=upnp->slist;
			}
			else s=s->next;
			break;

		case 4:	pthread_mutex_lock(&upnp->htx);
			for(hs=upnp->hlist;hs;hs=hs->next)
			{
				pthread_spin_lock(&hs->mtx);
				if(hs->fd!=-1)
				{
					hp.events=POLLIN;
					hp.fd=hs->fd;
					if(poll(&hp,1,0)==1)
					{
						hs->fd=-1;
						pthread_spin_unlock(&hs->mtx);
						close(hp.fd);
						if(hs->handle)
						{
							end_stream(upnp,
								hs->handle);
							hs->handle=NULL;
							upnp->playhttp--;
						}
					}
					else pthread_spin_unlock(&hs->mtx);
				}
				else pthread_spin_unlock(&hs->mtx);
			}
			pthread_mutex_unlock(&upnp->htx);
			break;

		case 5:	pthread_mutex_lock(&upnp->htx);
			for(ds=&upnp->hlist;*ds;)
			{
				pthread_spin_lock(&(*ds)->mtx);
				if((*ds)->fd==-1)
				{
					pthread_spin_unlock(&(*ds)->mtx);
					if((*ds)->handle)
					{
						end_stream(upnp,(*ds)->handle);
						upnp->playhttp--;
					}
					hs=*ds;
					*ds=hs->next;
					pthread_spin_destroy(&hs->mtx);
					free(hs);
				}
				else
				{
					pthread_spin_unlock(&(*ds)->mtx);
					ds=&(*ds)->next;
				}
			}
			pthread_mutex_unlock(&upnp->htx);
			break;

		case 6:	for(d=&upnp->coll;*d;)
			{
				if(sendcoll(upnp,*d))
				{
					upnp->cb(SATIP_STOPPING,dta,upnp->priv);
					for(i=0;i<=upnp->level;i++)
						release(upnp,i);
					upnp->state=RESTART;
					break;
				}

				if(!(*d)->cnt)
				{
					c=*d;
					(*d)=c->next;
					free(c);
					continue;
				}
				d=&(*d)->next;
			}
			break;

		case 7:	switch(upnp->igmpv3)
			{
			case SATIP_IGMPQUERY:
			case SATIP_IGMPFORCE:
				if(!upnp->igmpwait--)
				{
					upnp->igmpwait=upnp->igmpnext;
					upnp->igmpnext=V3TIME;
					upnp->igmpquerier=1;
					sendv3(upnp,upnp->qry[SATIP_V4_ONLY],0);
				}
				break;
			}
			if(upnp->level!=SATIP_V4_ONLY)switch(upnp->mldv2)
			{
			case SATIP_MLDQUERY:
			case SATIP_MLDFORCE:
				if(!upnp->mldwait--)
				{
					upnp->mldwait=upnp->mldnext;
					upnp->mldnext=V3TIME;
					upnp->mldquerier=1;
					sendm2(upnp,upnp->qry[SATIP_V6_LINK],
						(void *)in6addr_any.s6_addr);
				}
				break;
			}
			break;
		}
		break;
	}
}

static int docmd(UPNP *upnp,int cmd,void *data,void **rdata)
{
	int ans=-1;
	uint64_t dummy=1;
	struct pollfd p[2];

	pthread_mutex_lock(&upnp->ctx);
	upnp->cmd=cmd;
	upnp->data=data;
	p[0].fd=upnp->sfd;
	p[0].events=POLLIN;
	p[1].fd=upnp->ansfd;
	p[1].events=POLLIN;
	if(write(upnp->reqfd,&dummy,sizeof(dummy))!=sizeof(dummy))goto out;
	while(poll(p,2,-1)<1);
	if(p[0].revents&POLLIN)goto out;
	if(!(p[1].revents&POLLIN))goto out;
	if(read(upnp->ansfd,&dummy,sizeof(dummy))!=sizeof(dummy))goto out;
	ans=upnp->ans;
	if(rdata)*rdata=upnp->data;
out:	pthread_mutex_unlock(&upnp->ctx);
	return ans;
}

static SATIP_SRV_INFO *list_http(UPNP *upnp)
{
	STREAM *s;
	SATIP_SRV_INFO *i=NULL;
	SATIP_SRV_INFO *e;

	pthread_mutex_lock(&upnp->htx);
	for(s=upnp->hlist;s;s=s->next)
	{
		if(!(e=malloc(sizeof(SATIP_SRV_INFO))))continue;
		e->streamid=0;
		e->playing=s->handle?1:0;
		*((SOCK *)(e->addr))=s->peer;
		pthread_spin_lock(&s->mtx);
		e->state=s->state;
		pthread_spin_unlock(&s->mtx);
		e->next=i;
		i=e;
	}
	pthread_mutex_unlock(&upnp->htx);

	return i;
}

static SATIP_SRV_INFO *list_rtsp(UPNP *upnp)
{
	SESSION *s;
	SATIP_SRV_INFO *i=NULL;
	SATIP_SRV_INFO *e;

	for(s=upnp->slist;s;s=s->next)
	{
		if(!(e=malloc(sizeof(SATIP_SRV_INFO))))continue;
		e->streamid=s->master->sid;
		*((SOCK *)(e->addr))=s->rtp;
		pthread_spin_lock(&s->mtx);
		e->playing=s->play?1:0;
		e->state=s->state;
		pthread_spin_unlock(&s->mtx);
		e->next=i;
		i=e;
	}

	return i;
}

static void cmdwork(UPNP *upnp,void *data,int events)
{
	int i;
	int fd;
	uint64_t dummy;
	SESSION *s;
	STREAM *hs;
	SATIP_SRV_STATS *st;
	MCAST *m;
	MEMBER *mm;

	if(read(upnp->reqfd,&dummy,sizeof(dummy))!=sizeof(dummy))return;

	upnp->ans=-1;

	switch(upnp->cmd)
	{
	case KILL_ALL:
		flush_sess(upnp);
		flush_http(upnp);
		upnp->ans=0;
		break;

	case KILL_RTSP:
		flush_sess(upnp);
		upnp->ans=0;
		break;

	case KILL_HTTP:
		flush_http(upnp);
		upnp->ans=0;
		break;

	case KILL_SID:
		i=(int)((long)upnp->data)?1:0;
		if(!(s=lookup_sid(upnp,i)))break;
		if(s->handle)end_stream(upnp,s->handle);
		free_sess(upnp,s);
		upnp->ans=0;
		break;

	case KILL_TUNER:
		for(i=(int)((long)upnp->data),s=upnp->slist;s;)
		{
			pthread_spin_lock(&s->mtx);
			if(s->master!=s||s->state.tune.fe!=i)goto cont;
			pthread_spin_unlock(&s->mtx);
			if(s->handle)end_stream(upnp,s->handle);
			free_sess(upnp,s);
			s=upnp->slist;
			continue;
cont:			pthread_spin_unlock(&s->mtx);
			s=s->next;
		}
		pthread_mutex_lock(&upnp->htx);
		for(hs=upnp->hlist;hs;hs=hs->next)
		{
			pthread_spin_lock(&hs->mtx);
			if(hs->state.tune.fe==i)
			{
				if(hs->fd!=-1)
				{
					fd=hs->fd;
					hs->fd=-1;
					pthread_spin_unlock(&hs->mtx);
					close(fd);
				}
				else pthread_spin_unlock(&hs->mtx);
				if(hs->handle)
				{
					end_stream(upnp,hs->handle);
					hs->handle=NULL;
					upnp->playhttp--;
				}
			}
			else pthread_spin_unlock(&hs->mtx);
		}
		pthread_mutex_unlock(&upnp->htx);
		upnp->ans=0;
		break;

	case LOCK_ALL:
		i=(int)((long)upnp->data)?1:0;
		upnp->rtsplock=i;
		pthread_mutex_lock(&upnp->htx);
		upnp->httplock=i;
		pthread_mutex_unlock(&upnp->htx);
		upnp->ans=0;
		break;

	case LOCK_RTSP:
		i=(int)((long)upnp->data)?1:0;
		upnp->rtsplock=i;
		upnp->ans=0;
		break;

	case LOCK_HTTP:
		i=(int)((long)upnp->data)?1:0;
		pthread_mutex_lock(&upnp->htx);
		upnp->httplock=i;
		pthread_mutex_unlock(&upnp->htx);
		upnp->ans=0;
		break;

	case ADD_MULTICAST:
		m=(MCAST *)upnp->data;
		i=addr2level(&m->t->addr);
		if(alloc_sess(upnp,m->t,NULL,&s,i))upnp->ans=-1;
		else
		{
			s->state.tune=*m->tune;
			s->state.set=*m->set;
			s->id=0;
			upnp->sessions--;
			upnp->ans=s->sid;
			if(!m->play)break;
			s->fill=0;
			if(!(mm=play_stream(upnp,&s->state.tune,&s->state.set,
				s,0)))
			{
				free_sess(upnp,s);
				upnp->ans=-1;
			}
			s->handle=mm;
			pthread_spin_lock(&s->mtx);
			s->seq=mkseq(s->sid);
			upnp->playsessions++;
			s->play=(PLAY_PLAYING|PLAY_PERSIST);
			pthread_spin_unlock(&s->mtx);
		}
		break;

	case DEL_MULTICAST:
		i=(int)((long)upnp->data)?1:0;
		if(!(s=lookup_sid(upnp,i)))break;
		if(s->id)break;
		if(s->handle)end_stream(upnp,s->handle);
		free_sess(upnp,s);
		upnp->ans=0;
		break;

	case FLUSH_MULTICAST:
		for(i=1;i;)for(i=0,s=upnp->slist;s;s=s->next)if(!s->id)
		{
			if(s->handle)end_stream(upnp,s->handle);
			free_sess(upnp,s);
			i=1;
			break;
		}
		upnp->ans=0;
		break;

	case GET_STATS:
		st=(SATIP_SRV_STATS *)upnp->data;
		pthread_mutex_lock(&upnp->htx);
		st->rtsp_all_sessions=upnp->allsessions;
		st->rtsp_client_sessions=upnp->sessions;
		st->rtsp_playing_sessions=upnp->playsessions;
		st->http_running=upnp->running;
		st->http_playing=upnp->playhttp;
		pthread_mutex_unlock(&upnp->htx);
		upnp->ans=0;
		break;

	case LIST_RTSP:
		upnp->data=list_rtsp(upnp);
		upnp->ans=0;
		break;

	case LIST_HTTP:
		upnp->data=list_http(upnp);
		upnp->ans=0;
		break;
	}

	dummy=1;
	dummy=write(upnp->ansfd,&dummy,sizeof(dummy));
}

static void *server(void *data)
{
	UPNP *upnp=(UPNP *)data;
	union
	{
		struct
		{
			int i;
			int n;
			struct epoll_event e[MAX_EPOLL];
		} s;
		int rem;
		COLL *c;
		uint64_t dummy;
		sigset_t set;
	} u;

	sigfillset(&u.set);
	pthread_sigmask(SIG_BLOCK,&u.set,NULL);

	pthread_setname_np(pthread_self(),"server server");

restart:upnp->state=IFWAIT;
	upnp->port=MPORT;

	while(LIKELY(upnp->state<RESTART))
	{
		if(UNLIKELY(upnp->state!=RUNNING))flush_rtsp(upnp);

		if(UNLIKELY((u.s.n=epoll_wait(upnp->ffd,u.s.e,MAX_EPOLL,-1))<1))
			continue;
		for(u.s.i=0;u.s.i<u.s.n;u.s.i++)((FUNC *)u.s.e[u.s.i].data.ptr)
			->func(upnp,u.s.e[u.s.i].data.ptr,u.s.e[u.s.i].events);
	}

	flush_rtsp(upnp);

	for(u.s.i=0;u.s.i<=upnp->level;u.s.i++)
	{
		closer(upnp,&upnp->mdp[u.s.i]);
		closer(upnp,&upnp->tcp[u.s.i]);
		closer(upnp,&upnp->rcp[u.s.i]);
		closer(upnp,&upnp->udp[u.s.i]);
	}
	closer(upnp,&upnp->qry[SATIP_V4_ONLY]);
	closer(upnp,&upnp->qry[SATIP_V6_LINK]);

	while(upnp->coll)
	{
		u.c=upnp->coll;
		upnp->coll=u.c->next;
		free(u.c);
	}

	u.dummy=1;
	u.dummy=write(upnp->tfd,&u.dummy,sizeof(u.dummy));

	flush_sess(upnp);

	while(1)
	{
		pthread_mutex_lock(&upnp->htx);
		u.rem=upnp->running;
		pthread_mutex_unlock(&upnp->htx);
		if(!u.rem)break;
		usleep(10000);
	}

	flush_http(upnp);

	u.dummy=read(upnp->tfd,&u.dummy,sizeof(u.dummy));

	if(upnp->state==RESTART)goto restart;

	ifaddr_flush(upnp);

	pthread_exit(NULL);
}

static void http_cb(void *id,SATIP_STREAM *stream)
{
	STREAM *s=(STREAM *)id;
	int fd;
	struct iovec iov[2];
	unsigned char len[2];

	if(LIKELY(s->mode&&stream->fill))
	{
		pthread_spin_lock(&s->mtx);
		if(LIKELY(s->fd!=-1))
		{
			if(UNLIKELY(stream->flags&SATIP_FLGSECT))
			{
				len[0]=(unsigned char)(stream->fill>>8);
				len[1]=(unsigned char)(stream->fill);
				iov[0].iov_base=len;
				iov[0].iov_len=2;
				iov[1].iov_base=stream->section;
				iov[1].iov_len=stream->fill;
				if(UNLIKELY(writev(s->fd,iov,2)!=
					stream->fill+2))goto fail;
			}
			else if(UNLIKELY(write(s->fd,stream->data,stream->fill)
				!=stream->fill))
			{
fail:				fd=s->fd;
				s->fd=-1;
				pthread_spin_unlock(&s->mtx);
				close(fd);
				return;
			}
		}
		pthread_spin_unlock(&s->mtx);
	}
}

static void htcp_cb(void *id,SATIP_STATUS *status)
{
	int len=0;
	int fd;
	STREAM *s=(STREAM *)id;
	char bfr[1024];

	if(!s->mode)
	{
		switch(status->tune.msys)
		{
		case SATIP_DVBS2:
		case SATIP_DVBS:
			len=sprintf(bfr,"ver="PROT";src=%d;tuner=%d,%d,%d,%d,",
				status->tune.src,
				status->tune.fe?status->tune.fe:UNKNOWN_FE,
				status->level,status->lock,status->quality);
			break;
		case SATIP_DVBT2:
		case SATIP_DVBT:
		case SATIP_DVBC2:
		case SATIP_DVBC:
			len=sprintf(bfr,"ver="PROT";tuner=%d,%d,%d,%d,",
				status->tune.fe?status->tune.fe:UNKNOWN_FE,
				status->level,status->lock,status->quality);
			break;
		default:return;
		}

		len+=tuneinfo(&status->tune,&status->set,bfr+len,
			sizeof(bfr)-len-1,s->strict);
		bfr[len++]='\n';
	}

	pthread_spin_lock(&s->mtx);

	if(UNLIKELY(s->fd==-1))goto out;

	s->state=*status;

	if(UNLIKELY(s->mode))goto out;

	if(UNLIKELY(write(s->fd,bfr,len)!=len))
	{
		fd=s->fd;
		s->fd=-1;
		pthread_spin_unlock(&s->mtx);
		close(fd);
		return;
	}

out:	pthread_spin_unlock(&s->mtx);
}

static void rtp_cb(void *id,SATIP_STREAM *stream)
{
	SESSION *s=(SESSION *)id;
	SESSION *ms=s->slave;

	if(UNLIKELY(!stream))
	{
		if(!s->burst)return;

		pthread_spin_lock(&s->mtx);

		if(LIKELY(s->play))
		{
			pthread_spin_unlock(&s->mtx);

			if(LIKELY(s->rtpfd!=-1))
			{
				if(s->fill)
				{
					sendmmsg(s->rtpfd,s->msgqh,s->fill,0);
					s->fill=0;
				}
			}

			pthread_spin_lock(&s->mtx);
		}

		for(;ms;ms=ms->slave)if(LIKELY(ms->play))
		{
			if(LIKELY(ms->rtpfd!=-1))
			{
				if(ms->fill)
				{
					sendmmsg(ms->rtpfd,ms->msgqh,
						ms->fill,0);
					ms->fill=0;
				}
			}
		}

		pthread_spin_unlock(&s->mtx);
		return;
	}

	stream->hdr1[0]=0x80;
	stream->hdr4[1]=htonl(stream->khz90);

	pthread_spin_lock(&s->mtx);

	if(LIKELY(s->play))
	{
		if(UNLIKELY((stream->flags&SATIP_FLGSTART)||
			!(s->play&PLAY_NOTIFIED)))
		{
			s->play|=PLAY_NOTIFIED;
			stream->hdr1[1]=0x80|33;
		}
		else stream->hdr1[1]=33;
		stream->hdr2[1]=htons(s->seq++);
		stream->hdr4[2]=htonl(s->ssrc);

		pthread_spin_unlock(&s->mtx);

		if(LIKELY(s->rtpfd!=-1))
		{
			if(!s->burst||UNLIKELY(stream->fill!=
				sizeof(stream->data)&&!s->fill))
			{
				s->msgh1.io.iov_base=stream->msg;
				s->msgh1.io.iov_len=stream->fill+12;
				sendmsg(s->rtpfd,&s->msgh1.mh,0);
			}
			else if(UNLIKELY(stream->fill!=sizeof(stream->data)))
			{
				memcpy(&s->queue[s->fill],stream,
					sizeof(SATIP_STREAM));
				s->io[s->fill++].iov_len=stream->fill+12;
				sendmmsg(s->rtpfd,s->msgqh,s->fill,0);
				s->fill=0;
			}
			else
			{
				memcpy(&s->queue[s->fill],stream,
					sizeof(SATIP_STREAM));
				s->io[s->fill++].iov_len=stream->fill+12;
				if(s->fill==s->burst)
				{
					sendmmsg(s->rtpfd,s->msgqh,s->fill,0);
					s->fill=0;
				}
			}
		}

		pthread_spin_lock(&s->mtx);

		s->pkts++;
		s->dtot+=stream->fill;
	}

	for(;ms;ms=ms->slave)if(LIKELY(ms->play))
	{
		if(UNLIKELY(stream->flags&SATIP_FLGSTART||
			!(ms->play&PLAY_NOTIFIED)))
		{
			ms->play|=PLAY_NOTIFIED;
			stream->hdr1[1]=0x80|33;
		}
		else stream->hdr1[1]=33;
		stream->hdr2[1]=htons(ms->seq++);
		stream->hdr4[2]=htonl(ms->ssrc);
		if(LIKELY(ms->rtpfd!=-1))
		{
			if(!ms->burst||UNLIKELY(stream->fill!=
				sizeof(stream->data)&&!ms->fill))
			{
				ms->msgh1.io.iov_base=stream->msg;
				ms->msgh1.io.iov_len=stream->fill+12;
				sendmsg(ms->rtpfd,&ms->msgh1.mh,0);
			}
			else if(UNLIKELY(stream->fill!=sizeof(stream->data)))
			{
				memcpy(&ms->queue[ms->fill],stream,
					sizeof(SATIP_STREAM));
				ms->io[ms->fill++].iov_len=stream->fill+12;
				sendmmsg(ms->rtpfd,ms->msgqh,ms->fill,0);
				ms->fill=0;
			}
			else
			{
				memcpy(&ms->queue[ms->fill],stream,
					sizeof(SATIP_STREAM));
				ms->io[ms->fill++].iov_len=stream->fill+12;
				if(ms->fill==ms->burst)
				{
					sendmmsg(ms->rtpfd,ms->msgqh,
						ms->fill,0);
					ms->fill=0;
				}
			}
		}
		ms->pkts++;
		ms->dtot+=stream->fill;
	}

	pthread_spin_unlock(&s->mtx);
}

static int rtcp_receiver(SESSION *s)
{
	int match=0;
	int i=4;
	int j;
	SOCK from;
	int len;
	int plen;
	int total;
	struct msghdr mh;
	struct iovec io;
	unsigned char bfr[2048];

	mh.msg_name=&from;
	mh.msg_namelen=sizeof(SOCK);
	mh.msg_iov=&io;
	mh.msg_iovlen=1;
	mh.msg_control=NULL,
	mh.msg_controllen=0;
	io.iov_base=bfr;
	io.iov_len=sizeof(bfr);

	while(i--)
	{
		if((len=recvmsg(s->rtcpfd,&mh,MSG_DONTWAIT))<0)break;
		if(UNLIKELY(len<24))continue;
		total=bfr[0]&0x1f;
		if((bfr[0]&0xc0)!=0x80||!total||bfr[1]!=201)continue;
		plen=bfr[2];
		plen<<=8;
		plen|=bfr[3];
		plen+=1;
		plen<<=2;
		if(UNLIKELY(plen>len||(total<<4)+(total<<3)+8>plen))continue;
		if(!sscmp(&s->rtcp,&from))for(j=8;total;j+=24,total--)
			if(s->ssrc==ntohl(*((unsigned int *)(bfr+j))))
		{
			match=1;
			break;
		}
	}

	return match;
}

static void rtcp_cb(void *id,SATIP_STATUS *status)
{
	int i=0;
	int len;
	int pad;
	int match;
	unsigned long tme;
	unsigned long long ntp;
	SESSION *s=(SESSION *)id;
	SESSION *ms=s->slave;
	struct timeval tv;
	struct timespec ts;
	char bfr[128];
	char txt[1024];
	union
	{
#pragma pack(push,1)
		unsigned char msg1[2048];
		unsigned short msg2[1024];
		unsigned int msg4[512];
#pragma pack(pop)
	} u;

	if(UNLIKELY(gettimeofday(&tv,NULL)))return;
	if(UNLIKELY(clock_gettime(CLOCK_MONOTONIC_RAW,&ts)))return;

	ntp=(((unsigned long long)tv.tv_sec+2208988800ULL)<<32)|
		(((unsigned long long)tv.tv_usec<<32)/1000000ULL);
        tme=((unsigned long)ts.tv_sec)*10000UL+
		(unsigned long)(ts.tv_nsec/100000);

	u.msg1[0]=0x80;
	u.msg1[1]=200;
	u.msg2[1]=htons(6);
	u.msg4[2]=htonl((int)(ntp>>32));
	u.msg4[3]=htonl((int)(ntp&0xffffffff));
        u.msg4[4]=htonl((unsigned int)(tme*9UL)+
		(unsigned int)(ts.tv_nsec%100000/11112));
	u.msg1[28]=0x80;
	u.msg1[29]=202;
	u.msg2[15]=htons(0);
	u.msg1[32]=0x80;
	u.msg1[33]=204;
	u.msg4[10]=htonl(0x53455331);
	u.msg2[22]=htons(0);

	switch(status->tune.msys)
	{
	case SATIP_DVBS2:
	case SATIP_DVBS:
		len=sprintf(bfr,"ver="PROT";src=%d;tuner=%d,%d,%d,%d,",
			status->tune.src,status->tune.fe?
			status->tune.fe:UNKNOWN_FE,status->level,
			status->lock,status->quality);
		break;
	case SATIP_DVBT2:
	case SATIP_DVBT:
	case SATIP_DVBC2:
	case SATIP_DVBC:
		len=sprintf(bfr,"ver="PROT";tuner=%d,%d,%d,%d,",
			status->tune.fe?status->tune.fe:UNKNOWN_FE,
			status->level,status->lock,status->quality);
		break;
	default:return;
	}

	pthread_spin_lock(&s->mtx);

	s->state=*status;

	if(LIKELY(s->play))
	{
		if(UNLIKELY(!(s->play&PLAY_UPDATED)))
		{
			pthread_spin_unlock(&s->mtx);

			pad=tuneinfo(&status->tune,&status->set,txt,
				sizeof(s->info),s->strict);

			pthread_spin_lock(&s->mtx);

			if(LIKELY(s->play))if(UNLIKELY(!(s->play&PLAY_UPDATED)))
			{
				s->ilen=pad;
				strcpy(s->info,txt);
				s->play|=PLAY_UPDATED;
			}
		}

		u.msg4[9]=u.msg4[1]=htonl(s->ssrc);
		u.msg4[5]=htonl(s->pkts);
		u.msg4[6]=htonl(s->dtot);
		u.msg2[17]=htons(((len+s->ilen+3)>>2)+3);
		u.msg2[23]=htons(len+s->ilen);
		pad=(4-((len+s->ilen)&3))&3;
		memcpy(u.msg1+48,bfr,len);
		memcpy(u.msg1+48+len,s->info,s->ilen);
		if(pad)memset(u.msg1+48+len+s->ilen,0,pad);
		i=48+len+s->ilen+pad;

		pthread_spin_unlock(&s->mtx);

		if(LIKELY(s->rtcpfd!=-1))
		{
			s->msgh2.io.iov_base=u.msg1;
			s->msgh2.io.iov_len=i;
			sendmsg(s->rtcpfd,&s->msgh2.mh,0);
		}
		match=rtcp_receiver(s);

		pthread_spin_lock(&s->mtx);
		if(match)__sync_or_and_fetch(&s->rrrecv,1);
	}

	for(;ms;ms=ms->slave)if(LIKELY(ms->play))
	{
		if(UNLIKELY(!(ms->play&PLAY_UPDATED)))
		{
			if(!i)s->ilen=tuneinfo(&status->tune,&status->set,
				s->info,sizeof(s->info),s->strict);
			ms->play|=PLAY_UPDATED;
		}

		u.msg4[9]=u.msg4[1]=htonl(ms->ssrc);
		u.msg4[5]=htonl(ms->pkts);
		u.msg4[6]=htonl(ms->dtot);
		if(UNLIKELY(!i))
		{
			u.msg2[17]=htons(((len+s->ilen+3)>>2)+3);
			u.msg2[23]=htons(len+s->ilen);
			pad=(4-((len+s->ilen)&3))&3;
			memcpy(u.msg1+48,bfr,len);
			memcpy(u.msg1+48+len,s->info,s->ilen);
			if(pad)memset(u.msg1+48+len+s->ilen,0,pad);
			i=48+len+s->ilen+pad;
		}
		if(LIKELY(ms->rtcpfd!=-1))
		{
			ms->msgh2.io.iov_base=u.msg1;
			ms->msgh2.io.iov_len=i;
			sendmsg(ms->rtcpfd,&ms->msgh2.mh,0);
			if(rtcp_receiver(ms))__sync_or_and_fetch(&ms->rrrecv,1);
		}

	}

	pthread_spin_unlock(&s->mtx);
}

static void addsys(char *sys,char *msys,int total)
{
	char bfr[16];

	sprintf(bfr,"%s%s-%d",*sys?",":"",msys,total);
	strcat(sys,bfr);
}

static char *genxml(SATIP_SRV_CONFIG *c,SATIP_CFGINFO *h,char *uuid,int *len)
{
	char *res;
	char bfr[8192];
	char sys[128];

	*sys=0;
	if(h->totals[SATIP_TOT_DVBS2])
		addsys(sys,"DVBS2",h->totals[SATIP_TOT_DVBS2]);
	if(h->totals[SATIP_TOT_DVBT])
		addsys(sys,"DVBT",h->totals[SATIP_TOT_DVBT]);
	if(h->totals[SATIP_TOT_DVBT2])
		addsys(sys,"DVBT2",h->totals[SATIP_TOT_DVBT2]);
	if(h->totals[SATIP_TOT_DVBC])
		addsys(sys,"DVBC",h->totals[SATIP_TOT_DVBC]);
	if(h->totals[SATIP_TOT_DVBC2])
		addsys(sys,"DVBC2",h->totals[SATIP_TOT_DVBC2]);

	*len=snprintf(bfr,sizeof(bfr),
		"<?xml version=\"1.0\" encoding=\"%s\"?>\r\n"
		"<root xmlns=\"urn:schemas-upnp-org:device-1-0\" "
		"configId=\"0\">\r\n"
		"  <specVersion>\r\n"
		"    <major>1</major>\r\n"
		"    <minor>1</minor>\r\n"
		"  </specVersion>\r\n"
		"  <device>\r\n"
		"    <deviceType>urn:ses-com:device:SatIPServer:1"
		"</deviceType>\r\n",
		c->xmlcharset[0]?c->xmlcharset:"utf-8");
	if(c->friendlyname[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <friendlyName>%s</friendlyName>\r\n",c->friendlyname);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <friendlyName />\r\n");
	if(c->manufacturer[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <manufacturer>%s</manufacturer>\r\n",c->manufacturer);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <manufacturer />\r\n");
	if(c->manufacturerurl[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <manufacturerURL>%s</manufacturerURL>\r\n",
		c->manufacturerurl);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <manufacturerURL />\r\n");
	if(c->modeldescription[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <modelDescription>%s</modelDescription>\r\n",
		c->modeldescription);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <modelDescription />\r\n");
	if(c->modelname[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <modelName>%s</modelName>\r\n",c->modelname);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,"    <modelName />\r\n");
	if(c->modelnumber[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <modelNumber>%s</modelNumber>\r\n",c->modelnumber);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <modelNumber />\r\n");
	if(c->modelurl[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <modelURL>%s</modelURL>\r\n",c->modelnumber);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,"    <modelURL />\r\n");
	if(c->serialnumber[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <serialNumber>%s</serialNumber>\r\n",c->serialnumber);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <serialNumber />\r\n");
	*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <UDN>uuid:%s</UDN>\r\n",uuid);
	if(c->upc[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <UPC>%s</UPC>\r\n",c->upc);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,"    <UPC />\r\n");
	if(!c->png48depth&&!c->png120depth&&!c->jpg48depth&&!c->jpg120depth)
		goto noimg;
	*len+=snprintf(bfr+*len,sizeof(bfr)-*len,"    <iconList>\r\n");
	if(c->png48depth)*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"      <icon>\r\n"
		"        <mimetype>image/png</mimetype>\r\n"
		"        <width>48</width>\r\n"
		"        <height>48</height>\r\n"
		"        <depth>%d</depth>\r\n"
		"        <url>%s</url>\r\n"
		"      </icon>\r\n",c->png48depth,
		c->png48url[0]?c->png48url:"/icon48.png");
	if(c->png120depth)*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"      <icon>\r\n"
		"        <mimetype>image/png</mimetype>\r\n"
		"        <width>120</width>\r\n"
		"        <height>120</height>\r\n"
		"        <depth>%d</depth>\r\n"
		"        <url>%s</url>\r\n"
		"      </icon>\r\n",c->png120depth,
		c->png120url[0]?c->png120url:"/icon120.png");
	if(c->jpg48depth)*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"      <icon>\r\n"
		"        <mimetype>image/jpeg</mimetype>\r\n"
		"        <width>48</width>\r\n"
		"        <height>48</height>\r\n"
		"        <depth>%d</depth>\r\n"
		"        <url>%s</url>\r\n"
		"      </icon>\r\n",c->jpg48depth,
		c->jpg48url[0]?c->jpg48url:"/icon48.jpg");
	if(c->jpg120depth)*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"      <icon>\r\n"
		"        <mimetype>image/jpeg</mimetype>\r\n"
		"        <width>120</width>\r\n"
		"        <height>120</height>\r\n"
		"        <depth>%d</depth>\r\n"
		"        <url>%s</url>\r\n"
		"      </icon>\r\n",c->jpg120depth,
		c->jpg120url[0]?c->jpg120url:"/icon120.jpg");
	*len+=snprintf(bfr+*len,sizeof(bfr)-*len,"    </iconList>\r\n");	
noimg:	if(c->presentationurl[0])*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <presentationURL>%s</presentationURL>\r\n",
		c->presentationurl);
	else *len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <presentationURL />\r\n");
	*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <satip:X_SATIPCAP xmlns:satip=\"urn:ses-com:satip\">"
			"%s</satip:X_SATIPCAP>\r\n",sys);
	if(c->havem3u)*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"    <satip:X_SATIPM3U xmlns:satip=\"urn:ses-com:satip\">"
			"%s</satip:X_SATIPM3U>\r\n",
			c->m3uurl[0]?c->m3uurl:"/channellist.m3u");
	*len+=snprintf(bfr+*len,sizeof(bfr)-*len,
		"  </device>\r\n"
		"</root>\r\n");

	if(c->xmlcharset[0]&&strcasecmp(c->xmlcharset,"UTF8")&&
		strcasecmp(c->xmlcharset,"UTF-8"))
	{
		if(satip_util_cvt_utf8(c->xmlcharset,bfr,*len,&res,len))
			return NULL;
		else return res;
	}
	else return strdup(bfr);
}

int satip_srv_clr_all(void *handle)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp)return SATIP_SYSFAIL;
	return docmd(upnp,KILL_ALL,NULL,NULL)?SATIP_SYSFAIL:0;
}

int satip_srv_clr_rtsp(void *handle)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp)return SATIP_SYSFAIL;
	return docmd(upnp,KILL_RTSP,NULL,NULL)?SATIP_SYSFAIL:0;
}

int satip_srv_clr_http(void *handle)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp)return SATIP_SYSFAIL;
	return docmd(upnp,KILL_HTTP,NULL,NULL)?SATIP_SYSFAIL:0;
}

int satip_srv_clr_stream(void *handle,int streamid)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp||streamid<0||streamid>65535)return SATIP_SYSFAIL;
	return docmd(upnp,KILL_SID,(void *)((long)streamid),NULL)?
		SATIP_SYSFAIL:0;
}

int satip_srv_clr_device(void *handle,int deviceid)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp||deviceid<0||deviceid>65535)return SATIP_SYSFAIL;
	return docmd(upnp,KILL_TUNER,(void *)((long)deviceid),NULL)?
		SATIP_SYSFAIL:0;
}

int satip_srv_set_all_locks(void *handle,int islocked)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp)return SATIP_SYSFAIL;
	if(islocked)islocked=1;
	return docmd(upnp,LOCK_ALL,(void *)((long)islocked),NULL)?
		SATIP_SYSFAIL:0;
}

int satip_srv_set_rtsp_lock(void *handle,int islocked)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp)return SATIP_SYSFAIL;
	if(islocked)islocked=1;
	return docmd(upnp,LOCK_RTSP,(void *)((long)islocked),NULL)?
		SATIP_SYSFAIL:0;
}

int satip_srv_set_http_lock(void *handle,int islocked)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp)return SATIP_SYSFAIL;
	if(islocked)islocked=1;
	return docmd(upnp,LOCK_HTTP,(void *)((long)islocked),NULL)?
		SATIP_SYSFAIL:0;
}

int satip_srv_add_multicast(void *handle,char *addr,int port,int ttl,int play,
	SATIP_TUNE *tune,SATIP_PIDS *set)
{
	int l;
	UPNP *upnp=(UPNP *)handle;
	TRANSPORT t;
	MCAST m;

	if(!upnp||!addr||!tune||!set)return SATIP_SYSFAIL;
	if(port<1||port>65535||(port&1))return SATIP_SYSFAIL;
	if(ttl<1||ttl>255)return SATIP_SYSFAIL;
	switch(tune->msys)
	{
	case SATIP_DVBS:
	case SATIP_DVBS2:
	case SATIP_DVBT:
	case SATIP_DVBT2:
	case SATIP_DVBC:
	case SATIP_DVBC2:
		break;
	default:return SATIP_SYSFAIL;
	}
	if(set->numpids<1)return SATIP_SYSFAIL;

	t.mode=SATIP_TYPE_RTP;
	t.rtp=port;
	t.rtcp=port+1;
	t.ttl=ttl;
	if(str2addr(addr,&t.addr))return SATIP_SYSFAIL;
	if((l=addr2level(&t.addr))>upnp->level)return SATIP_SYSFAIL;
	if(l==SATIP_V6_LINK&&ttl!=1)return SATIP_SYSFAIL;
	if(invalid_mcast(&t.addr))return SATIP_SYSFAIL;
	m.play=play;
	m.t=&t;
	m.tune=tune;
	m.set=set;
	return docmd(upnp,ADD_MULTICAST,&m,NULL);
}

int satip_srv_del_multicast(void *handle,int streamid)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp||streamid<0||streamid>65535)return SATIP_SYSFAIL;
	return docmd(upnp,DEL_MULTICAST,(void *)((long)streamid),NULL)?
		SATIP_SYSFAIL:0;
}

int satip_srv_clr_multicast(void *handle)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp)return SATIP_SYSFAIL;
	return docmd(upnp,FLUSH_MULTICAST,NULL,NULL)?SATIP_SYSFAIL:0;
}

int satip_srv_statistics(void *handle,SATIP_SRV_STATS *stats)
{
	UPNP *upnp=(UPNP *)handle;

	if(!upnp)return SATIP_SYSFAIL;
	return docmd(upnp,GET_STATS,stats,NULL)?SATIP_SYSFAIL:0;
}

SATIP_SRV_INFO *satip_srv_list_rtsp(void *handle)
{
	UPNP *upnp=(UPNP *)handle;
	SATIP_SRV_INFO *list=NULL;
	SATIP_SRV_INFO *e;
	SOCK a;

	if(!upnp)return NULL;
	if(docmd(upnp,LIST_RTSP,NULL,(void **)&list))return NULL;
	for(e=list;e;e=e->next)
	{
		a=*((SOCK *)(e->addr));
		e->port=ntohs(psget(&a));
		sock2str(&a,e->addr,sizeof(e->addr));
	}
	return list;
}

SATIP_SRV_INFO *satip_srv_list_http(void *handle)
{
	UPNP *upnp=(UPNP *)handle;
	SATIP_SRV_INFO *list=NULL;
	SATIP_SRV_INFO *e;
	SOCK a;

	if(!upnp)return NULL;
	if(docmd(upnp,LIST_HTTP,NULL,(void **)&list))return NULL;
	for(e=list;e;e=e->next)
	{
		a=*((SOCK *)(e->addr));
		e->port=ntohs(psget(&a));
		sock2str(&a,e->addr,sizeof(e->addr));
	}
	return list;
}

int satip_srv_free_list(void *handle,SATIP_SRV_INFO *list)
{
	SATIP_SRV_INFO *e;

	if(!handle)return SATIP_SYSFAIL;
	while(list)
	{
		e=list;
		list=list->next;
		free(e);
	}
	return 0;
}

void satip_srv_stream(void *id,SATIP_STREAM *stream)
{
	GROUP *g=(GROUP *)id;
	MEMBER *m;
	int flags;

	pthread_spin_lock(&g->dmtx);

	for(m=g->list;m;m=m->next)
	{
		if(UNLIKELY(!stream))
		{
			if(!m->mode)rtp_cb(m->h,NULL);
		}
		else if(UNLIKELY(!m->notified))
		{
			m->notified=1;
			flags=stream->flags;
			stream->flags|=SATIP_FLGSTART;
			switch(m->mode)
			{
			case 0:	rtp_cb(m->h,stream);
				break;
			case 1:	http_cb(m->h,stream);
				break;
			}
			stream->flags=flags;
		}
		else switch(m->mode)
		{
		case 0:	rtp_cb(m->h,stream);
			break;
		case 1:	http_cb(m->h,stream);
			break;
		}
	}

	pthread_spin_unlock(&g->dmtx);
}

void satip_srv_status(void *id,SATIP_STATUS *status)
{
	GROUP *g=(GROUP *)id;
	MEMBER *m;

	pthread_spin_lock(&g->smtx);

	for(m=g->list;m;m=m->next)switch(m->mode)
	{
	case 0:	rtcp_cb(m->h,status);
		break;
	case 1:	htcp_cb(m->h,status);
		break;
	}

	pthread_spin_unlock(&g->smtx);
}

int satip_srv_forward(void *handle,char *host,int port,SATIP_DATA *data)
{
	UPNP *upnp=(UPNP *)handle;
	ADDR addr;
	char *ptr;
	char *mem;
	char *request=data[0].ptrval;
	int len=data[0].intval;
	int lcl=data[1].intval;
	int rmt;
        int lrfill;
        int rlfill;
        int lev;
        int rev;
        int state;
        struct pollfd p[2];
        char lclrmt[16384];
        char rmtlcl[16384];
	char oob;

	if(!handle||!host||port<1||port>65535)return SATIP_SYSFAIL;

	for(rev=0,ptr=request;*ptr;ptr++)if(*ptr=='\r'||*ptr=='\n')break;
	oob=*ptr;
	*ptr=0;
	if(strstr(request,"HTTP/1.1"))rev=1;
	*ptr=oob;

	if(resolve(host,&addr,upnp->level))return SATIP_SYSFAIL;

	if((rmt=tcpconnect(&addr,htons(port),upnp->devidx))==-1)
		return SATIP_SYSFAIL;

	if(!rev)
	{
		p[0].fd=rmt;
		p[0].events=POLLOUT;

		while(len)
		{
			if(poll(p,1,500)!=1||!(p[0].revents&POLLOUT))
			{
				close(rmt);
				return SATIP_SYSFAIL;
			}

			if((lrfill=write(rmt,request,len))<=0)
			{
				close(rmt);
				return SATIP_SYSFAIL;
			}

			request+=lrfill;
			len-=lrfill;
		}
	}
	else
	{
		p[0].fd=lcl;
		p[0].events=POLLIN;

		memcpy(lclrmt,request,len);

		while(1)
		{
			for(lrfill=0,state=0;lrfill<len&&state<2;lrfill++)
				switch(lclrmt[lrfill])
			{
			case '\n':
				state++;
			case '\r':
				break;
			default:state=0;
				break;
			}

			if(state==2)break;

			if(len==sizeof(lclrmt)-1)
			{
				close(rmt);
				return SATIP_SYSFAIL;
			}

			if(poll(p,1,500)!=1||!(p[0].revents&POLLIN))
			{
				close(rmt);
				return SATIP_SYSFAIL;
			}

			if((lrfill=read(lcl,lclrmt+len,sizeof(lclrmt)-1-len))
				<=0)
			{
				close(rmt);
				return SATIP_SYSFAIL;
			}

			len+=lrfill;
		}

		lclrmt[lrfill]=0;
		request=lclrmt+lrfill;
		len=len-lrfill;

		p[0].fd=rmt;
		p[0].events=POLLOUT;

		for(ptr=lclrmt;(ptr=strtok_r(ptr,"\r\n",&mem));ptr=NULL)
		{
			if(!strncasecmp(ptr,"Connection:",11))
				if(strstr(ptr+11,"close"))rev=0;
			strcpy(rmtlcl,ptr);
			strcat(rmtlcl,"\r\n");

			rlfill=strlen(rmtlcl);
			ptr=rmtlcl;

			while(rlfill)
			{
				if(poll(p,1,500)!=1||!(p[0].revents&POLLOUT))
				{
					close(rmt);
					return SATIP_SYSFAIL;
				}

				if((lrfill=write(rmt,ptr,rlfill))<=0)
				{
					close(rmt);
					return SATIP_SYSFAIL;
				}

				ptr+=lrfill;
				rlfill-=lrfill;
			}
		}

		if(!rev)strcpy(rmtlcl,"\r\n");
		else strcpy(rmtlcl,"Connection: close\r\n\r\n");

		rlfill=strlen(rmtlcl);
		ptr=rmtlcl;

		while(rlfill)
		{
			if(poll(p,1,500)!=1||!(p[0].revents&POLLOUT))
			{
				close(rmt);
				return SATIP_SYSFAIL;
			}

			if((lrfill=write(rmt,ptr,rlfill))<=0)
			{
				close(rmt);
				return SATIP_SYSFAIL;
			}

			ptr+=lrfill;
			rlfill-=lrfill;
		}

		while(len)
		{
			if(poll(p,1,500)!=1||!(p[0].revents&POLLOUT))
			{
				close(rmt);
				return SATIP_SYSFAIL;
			}

			if((lrfill=write(rmt,request,len))<=0)
			{
				close(rmt);
				return SATIP_SYSFAIL;
			}

			request+=lrfill;
			len-=lrfill;
		}
	}

	p[0].fd=lcl;
	p[1].fd=rmt;

	state=0x0000;
	lev=POLLIN|POLLPRI|POLLRDHUP;
	rev=POLLIN|POLLPRI|POLLRDHUP;
	lrfill=0;
	rlfill=0;

	while(lev||rev)
	{
		p[0].events=lev;
		p[1].events=rev;

		switch(poll(p,2,30000))
		{
		case -1:continue;
		case 0: shutdown(lcl,SHUT_RDWR);
			shutdown(rmt,SHUT_RDWR);
			close(rmt);
			return 0;
		}

		if(p[0].revents&POLLRDHUP)state|=0x0010;

		if(p[0].revents&(POLLERR|POLLHUP|POLLNVAL))state|=0x0004;

		if(p[0].revents&POLLPRI)switch(recv(lcl,&oob,1,MSG_OOB))
		{
		case 0:
		case -1:state|=0x0001;
			break;

		default:if(send(rmt,&oob,1,MSG_OOB)!=1)state|=0x0008;
			break;
		}

		if(p[0].revents&POLLIN)
		{
			if((len=read(lcl,lclrmt+lrfill,sizeof(lclrmt)-lrfill))
				<=0)
			{
				if(!len&&(state&0x0010))state|=0x0040;
				state|=0x0001;
			}
			else lrfill+=len;

			if((state&0x0050)==0x0050)shutdown(rmt,SHUT_WR);
		}

		if(p[1].revents&POLLRDHUP)state|=0x0020;

		if(p[1].revents&(POLLERR|POLLHUP|POLLNVAL))state|=0x0008;

		if(p[1].revents&POLLPRI)switch(recv(rmt,&oob,1,MSG_OOB))
		{
		case 0:
		case -1:state|=0x0002;
			break;

		default:if(send(lcl,&oob,1,MSG_OOB)!=1)state|=0x0004;
			break;
		}

		if(p[1].revents&POLLIN)
		{
			if((len=read(rmt,rmtlcl+rlfill,sizeof(rmtlcl)-rlfill))
				<=0)
			{
				if(!len&&(state&0x0020))state|=0x0080;
				state|=0x0002;
			}
			else rlfill+=len;

			if((state&0x00a0)==0x00a0)shutdown(lcl,SHUT_WR);
		}

		if(lrfill&&!(state&0x0008))
		{
			if((len=write(rmt,lclrmt,lrfill))>=0)
			{
				lrfill-=len;
				if(len)memmove(lclrmt,lclrmt+len,lrfill);
			}
			else
			{
				if(errno!=EAGAIN&&errno!=EWOULDBLOCK)
					state|=0x0008;
				else if((p[1].events&POLLOUT)&&
					(p[1].revents&POLLOUT))state|=0x0008;
			}
		}

		if(rlfill&&!(state&0x0004))
		{
			if((len=write(lcl,rmtlcl,rlfill))>=0)
			{
				rlfill-=len;
				if(len)memmove(rmtlcl,rmtlcl+len,rlfill);
			}
			else
			{
				if(errno!=EAGAIN&&errno!=EWOULDBLOCK)
					state|=0x0004;
				else if((p[0].events&POLLOUT)&&
					(p[0].revents&POLLOUT))state|=0x0004;
			}
		}

		if((state&0x0009)||lrfill==sizeof(lclrmt))lev=0;
		else lev=POLLIN|POLLPRI|POLLRDHUP;
		if(!(state&0x0004)&&rlfill)lev|=POLLOUT;

		if((state&0x0006)||rlfill==sizeof(rmtlcl))rev=0;
		else rev=POLLIN|POLLPRI|POLLRDHUP;
		if(!(state&0x0008)&&lrfill)rev|=POLLOUT;
	}

	close(rmt);
	return 0;
}

void *satip_srv_init(SATIP_SRV_CONFIG *config,SATIP_CFGINFO *hwinfo)
{
	int i;
	int j;
	UPNP *upnp;
	struct itimerspec ts;
	struct epoll_event e;
	struct sockaddr_nl addr;
	struct __user_cap_header_struct h;
	struct __user_cap_data_struct c[2];
	static FUNC tterm={NULL,terminator};
	static FUNC ttick={NULL,timetick};
	static FUNC nlink={NULL,nlmsgwork};
	static FUNC docmd={NULL,cmdwork};
	pthread_attr_t attr;

	switch(config->level)
	{
	case SATIP_V4_ONLY:
	case SATIP_V6_LINK:
	case SATIP_V6_SITE:
		break;
	default:goto err1;
	}

	switch(config->igmpv3)
	{
	case SATIP_IGMPNONE:
	case SATIP_IGMPSNOOP:
	case SATIP_IGMPQUERY:
	case SATIP_IGMPFORCE:
		break;
	default:goto err1;
	}

	if(config->level!=SATIP_V4_ONLY)switch(config->mldv2)
	{
	case SATIP_MLDNONE:
	case SATIP_MLDSNOOP:
	case SATIP_MLDQUERY:
	case SATIP_MLDFORCE:
		break;
	default:goto err1;
	}

	if(!config->callback)goto err1;
	if(!config->dev||!config->dev[0])goto err1;
	if(strlen(config->dev)>=sizeof(upnp->dev))goto err1;
	if(!strcmp(config->dev,LOOPDEV))goto err1;
	if(config->rtsplimit<0||config->rtsplimit>65535)goto err1;
	if(config->httplimit<0||config->httplimit>65535)goto err1;
	if(config->httpnostream<0||config->httpnostream>65535)goto err1;
	if(config->httpnostream>config->httplimit)goto err1;
	if(config->strict<0||config->strict>1)goto err1;
	if(config->timeout&&(config->timeout<10||config->timeout>180))goto err1;
	if(config->rtspport<1||config->rtspport>65535)
		if(config->rtspport!=SATIP_RTSP_AUTO)goto err1;
	if(config->httpport<1||config->httpport>65535)
		if(config->httpport!=SATIP_HTTP_AUTO)goto err1;
	if(config->mdftport<1||config->mdftport>65535)
		if(config->mdftport!=SATIP_MCST_AUTO)goto err1;
	if(config->mdftport&1)goto err1;
	if(config->upnpage&&(config->upnpage<60||config->upnpage>86400))
		goto err1;
	if(config->mttl&&(config->mttl<1||config->mttl>255))goto err1;
	if(config->locked<0||config->locked>1)goto err1;
	if(config->burst<0||config->burst>SATIP_MAX_BURST)goto err1;
	if(!config->portmin&&config->portmax)goto err1;
	else if(config->portmin)
	{
		if(config->portmin<1024||config->portmin>65534)goto err1;
		if(config->portmin>=config->portmax)goto err1;
		if((config->portmin&1)||!(config->portmax&1))goto err1;
	}
	for(i=0,j=0;i<SATIP_TOTALS;i++)
		if(hwinfo->totals[i]<0||(hwinfo->totals[i]>9&&config->strict))
			goto err1;
	else j+=hwinfo->totals[i];
	if(!j)goto err1;

	if(geteuid())if((config->rtspport!=SATIP_RTSP_AUTO&&
		config->rtspport<1024)||
		(config->httpport!=SATIP_HTTP_AUTO&&config->httpport<1024)||
		(config->mdftport!=SATIP_MCST_AUTO&&config->mdftport<1024)||
		config->igmpv3!=SATIP_IGMPNONE||
		(config->level!=SATIP_V4_ONLY&&config->mldv2!=SATIP_MLDNONE))
	{
		memset(&h,0,sizeof(h));
		h.version=_LINUX_CAPABILITY_VERSION_3;
		h.pid=0;
		memset(c,0,sizeof(c));
		if(capget(&h,c))goto err1;
		if((config->rtspport!=SATIP_RTSP_AUTO&&config->rtspport<1024)||
		    (config->httpport!=SATIP_HTTP_AUTO&&config->httpport<1024)||
		    (config->mdftport!=SATIP_MCST_AUTO&&config->mdftport<1024))
			if(!(c[0].effective&(1<<CAP_NET_BIND_SERVICE)))
				goto err1;
		if(config->igmpv3!=SATIP_IGMPNONE||
		   (config->level!=SATIP_V4_ONLY&&config->mldv2!=SATIP_MLDNONE))
			if(!(c[0].effective&(1<<CAP_NET_RAW)))goto err1;
	}

	if(!(upnp=malloc(sizeof(UPNP))))goto err1;
	memset(upnp,0,sizeof(UPNP));

	upnp->age=config->upnpage?config->upnpage:SATIP_UPNP_AGE;
	upnp->devid=1;
	upnp->level=config->level;
	upnp->devaddr[SATIP_V4_ONLY].family=AF_INET;
	upnp->devaddr[SATIP_V6_LINK].family=AF_INET6;
	upnp->devaddr[SATIP_V6_SITE].family=AF_INET6;
	upnp->mdp[SATIP_V4_ONLY]=-1;
	upnp->mdp[SATIP_V6_LINK]=-1;
	upnp->mdp[SATIP_V6_SITE]=-1;
	upnp->tcp[SATIP_V4_ONLY]=-1;
	upnp->tcp[SATIP_V6_LINK]=-1;
	upnp->tcp[SATIP_V6_SITE]=-1;
	upnp->rcp[SATIP_V4_ONLY]=-1;
	upnp->rcp[SATIP_V6_LINK]=-1;
	upnp->rcp[SATIP_V6_SITE]=-1;
	upnp->qry[SATIP_V4_ONLY]=-1;
	upnp->qry[SATIP_V6_LINK]=-1;
	upnp->udp[SATIP_V4_ONLY]=-1;
	upnp->udp[SATIP_V6_LINK]=-1;
	upnp->udp[SATIP_V6_SITE]=-1;
	upnp->cb=config->callback;
	upnp->cfgrtsp=config->rtspport;
	upnp->cfghttp=config->httpport;
	upnp->mdft=config->mdftport;
	upnp->priv=config->priv;
	upnp->caps=hwinfo->caps;
	upnp->igmpv3=config->igmpv3;
	upnp->mldv2=config->level!=SATIP_V4_ONLY?config->mldv2:SATIP_MLDNONE;
	upnp->mttl=config->mttl?config->mttl:SATIP_MCST_TTL;
	upnp->rtsplimit=config->rtsplimit;
	upnp->httplimit=config->httplimit;
	upnp->httpnostream=config->httpnostream;
	upnp->strict=config->strict;
	upnp->timeout=config->timeout?config->timeout:STD_RTSP_IDLE;
	upnp->bpslimit=config->bytespersec;
	upnp->rtsplock=config->locked;
	upnp->httplock=config->locked;
	upnp->locked=config->locked;
	upnp->portmin=config->portmin;
	upnp->portmax=config->portmax;

	upnp->sat=hwinfo->totals[SATIP_TOT_DVBS2];
	upnp->terr=hwinfo->totals[SATIP_TOT_DVBT]+
		hwinfo->totals[SATIP_TOT_DVBT2];
	upnp->cable=hwinfo->totals[SATIP_TOT_DVBC]+
		hwinfo->totals[SATIP_TOT_DVBC2];

	if(!config->strict)switch(config->burst)
	{
	case 0:	upnp->burst=SATIP_MAX_BURST;
		break;
	case 1:	upnp->burst=0;
		break;
	default:upnp->burst=config->burst;
		break;
	}
	else upnp->burst=0;

	if(config->havem3u&&!config->m3uurl[0])upnp->webflags|=0x01;
	if(config->png48depth&&!config->png48url[0])upnp->webflags|=0x02;
	if(config->png120depth&&!config->png120url[0])upnp->webflags|=0x04;
	if(config->jpg48depth&&!config->jpg48url[0])upnp->webflags|=0x08;
	if(config->jpg120depth&&!config->jpg120url[0])upnp->webflags|=0x10;

	upnp->sidmap[0]=0x00000001;
	upnp->sidmap[2032]=0xfffffffc;

	upnp->seq=time(NULL);

	strncpy(upnp->dev,config->dev,sizeof(upnp->dev)-1);
	strcpy(upnp->xmlcharset,config->xmlcharset);

	if(loaduuid(upnp))goto err2;
	if(!(upnp->xml=genxml(config,hwinfo,upnp->uuid,&upnp->xmllen)))
		goto err2;

	loadbcnt(upnp);
	loaddevid(upnp);

	if(pthread_mutex_init(&upnp->htx,NULL))goto err3;
	if(pthread_mutex_init(&upnp->stx,NULL))goto err4;
	if(pthread_mutex_init(&upnp->ctx,NULL))goto err5;
	if(pthread_attr_init(&upnp->attr))goto err6;
	if(pthread_attr_setdetachstate(&upnp->attr,PTHREAD_CREATE_DETACHED))
		goto err7;
	if(config->stack)
		if(pthread_attr_setstacksize(&upnp->attr,config->stack))
			goto err7;

	if((upnp->efd=timerfd_create(CLOCK_MONOTONIC,TFD_CLOEXEC))==-1)
		goto err7;
	if((upnp->tfd=eventfd(0,EFD_CLOEXEC))==-1)goto err8;
	if((upnp->sfd=eventfd(0,EFD_CLOEXEC))==-1)goto err9;
	if((upnp->reqfd=eventfd(0,EFD_CLOEXEC))==-1)goto err10;
	if((upnp->ansfd=eventfd(0,EFD_CLOEXEC))==-1)goto err11;
	if((upnp->nfd=socket(PF_NETLINK,SOCK_RAW|SOCK_CLOEXEC,NETLINK_ROUTE))
		==-1)goto err12;

	i=MAXBUFSIZE;
	if(setsockopt(upnp->nfd,SOL_SOCKET,SO_SNDBUF,&i,sizeof(i)))goto err13;
	if(setsockopt(upnp->nfd,SOL_SOCKET,SO_RCVBUF,&i,sizeof(i)))goto err13;

	memset(&addr,0,sizeof(addr));
	addr.nl_family=AF_NETLINK;
	addr.nl_groups=RTMGRP_IPV4_IFADDR|RTMGRP_IPV6_IFADDR;
	if(bind(upnp->nfd,(struct sockaddr *)&addr,sizeof(addr))==-1)goto err13;

	if(getaddrlist(upnp,0))goto err13;

	if((upnp->ffd=epoll_create1(EPOLL_CLOEXEC))==-1)goto err13;
	e.events=POLLIN;
	e.data.ptr=&tterm;
	if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,upnp->sfd,&e))goto err14;
	e.events=POLLIN;
	e.data.ptr=&ttick;
	if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,upnp->efd,&e))goto err15;
	e.events=POLLIN;
	e.data.ptr=&nlink;
	if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,upnp->nfd,&e))goto err16;
	e.events=POLLIN;
	e.data.ptr=&docmd;
	if(epoll_ctl(upnp->ffd,EPOLL_CTL_ADD,upnp->reqfd,&e))goto err17;

	memset(&ts,0,sizeof(ts));
	ts.it_value.tv_nsec=125000000;
	ts.it_interval.tv_nsec=125000000;
	if(timerfd_settime(upnp->efd,0,&ts,NULL))goto err18;

	if(pthread_attr_init(&attr))goto err19;
	if(pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE))
		goto err20;
	if(config->stack)if(pthread_attr_setstacksize(&attr,config->stack))
		goto err20;

	if(pthread_create(&upnp->srv,&attr,server,upnp))goto err20;

	pthread_attr_destroy(&attr);

	return upnp;

err20:	pthread_attr_destroy(&attr);
err19:	memset(&ts,0,sizeof(ts));
	timerfd_settime(upnp->efd,0,&ts,NULL);
err18:	epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,upnp->reqfd,NULL);
err17:	epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,upnp->nfd,NULL);
err16:	epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,upnp->efd,NULL);
err15:	epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,upnp->sfd,NULL);
err14:	close(upnp->ffd);
err13:	close(upnp->nfd);
err12:	close(upnp->ansfd);
err11:	close(upnp->reqfd);
err10:	close(upnp->sfd);
err9:	close(upnp->tfd);
err8:	close(upnp->efd);
err7:	pthread_attr_destroy(&upnp->attr);
err6:	pthread_mutex_destroy(&upnp->ctx);
err5:	pthread_mutex_destroy(&upnp->stx);
err4:	pthread_mutex_destroy(&upnp->htx);
err3:	free(upnp->xml);
err2:	free(upnp);
err1:	return NULL;
}

void satip_srv_fini(void *handle)
{
	UPNP *upnp=(UPNP *)handle;
	uint64_t dummy=1;
	struct itimerspec ts;

	dummy=write(upnp->sfd,&dummy,sizeof(dummy));

	pthread_join(upnp->srv,NULL);

	memset(&ts,0,sizeof(ts));
	timerfd_settime(upnp->efd,0,&ts,NULL);

	epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,upnp->reqfd,NULL);
	epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,upnp->nfd,NULL);
	epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,upnp->efd,NULL);
	epoll_ctl(upnp->ffd,EPOLL_CTL_DEL,upnp->sfd,NULL);
	close(upnp->ffd);
	close(upnp->nfd);
	close(upnp->ansfd);
	close(upnp->reqfd);
	close(upnp->sfd);
	close(upnp->tfd);
	close(upnp->efd);

	pthread_attr_destroy(&upnp->attr);
	pthread_mutex_destroy(&upnp->ctx);
	pthread_mutex_destroy(&upnp->htx);
	pthread_mutex_destroy(&upnp->stx);

	free(upnp->xml);
	free(upnp);
}
