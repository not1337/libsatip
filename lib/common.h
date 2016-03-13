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

#ifndef _COMMON_H
#define _COMMON_H

#define MPORT		1900
#define LOOPDEV		"lo"

#define SERVER		"libsatip"
#define VERSION		"1.0"
#define IDENT		SERVER "/" VERSION

#define STR(s)		_STR(s)
#define _STR(s)		#s

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

static __inline__ int __sync_bool_compare_and_swap(int *ptr,int old,int new)
{
	static pthread_mutex_t mtx=PTHREAD_MUTEX_INITIALIZER;
	int r;

	pthread_mutex_lock(&mtx);
	if(*ptr==old)
	{
		*ptr=new;
		r=1;
	}
	else r=0;
	pthread_mutex_unlock(&mtx);
	return r;
}

static __inline__ int __sync_fetch_and_and(int *ptr,int val)
{
	static pthread_mutex_t mtx=PTHREAD_MUTEX_INITIALIZER;
	int r;

	pthread_mutex_lock(&mtx);
	r=*ptr;
	*ptr&=val;
	pthread_mutex_unlock(&mtx);
	return r;
}

static __inline__ int __sync_or_and_fetch(int *ptr,int val)
{
	static pthread_mutex_t mtx=PTHREAD_MUTEX_INITIALIZER;
	int r;

	pthread_mutex_lock(&mtx);
	r=(*ptr|=val);
	pthread_mutex_unlock(&mtx);
	return r;
}

#endif


#ifdef AF_INET

typedef union
{
	struct sockaddr_in a4;
	struct sockaddr_in6 a6;
} SOCK;

typedef struct
{
	int family;
	union
	{
		struct in_addr a4;
		struct in6_addr a6;
	};
} ADDR;

static __attribute__((unused)) ADDR mcaddr[3]=
{
	{
		.family=AF_INET,
#if __BYTE_ORDER == __LITTLE_ENDIAN
		.a4.s_addr=0xfaffffef,
#elif __BYTE_ORDER == __BIG_ENDIAN
		.a4.s_addr=0xeffffffa,
#else
#error Need endian!
#endif
	},
	{
		.family=AF_INET6,
		.a6.s6_addr={0xff,0x02,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0c},
	},
	{
		.family=AF_INET6,
		.a6.s6_addr={0xff,0x05,0x00,0x00,0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0c},
	},
};

static __inline__ int sockfam(SOCK *sock)
{
	if(sock->a4.sin_family==AF_INET)return AF_INET;
	if(sock->a6.sin6_family==AF_INET6)return AF_INET6;
	return AF_UNSPEC;
}

static __inline__ int anyaddr(ADDR *addr)
{
	switch(addr->family)
	{
	case AF_INET:
		return addr->a4.s_addr==INADDR_ANY?1:0;
	case AF_INET6:
		return memcmp(addr->a6.s6_addr,in6addr_any.s6_addr,16)?0:1;
	default:return 0;
	}
}

static __inline__ int a6sitewide(struct in6_addr *a)
{
	if((a->s6_addr[0]&0xe0)==0x20)return 2;
	if((a->s6_addr[0]&0xfe)==0xfc)return 1;
	if(a->s6_addr[0]==0xff&&(a->s6_addr[1]&0x0f)==0x05)return 3;
	return 0;
}

static __inline__ int a6linklocal(struct in6_addr *a)
{
	if(a->s6_addr[0]==0xfe&&(a->s6_addr[1]&0xc0)==0x80)return 1;
	if(a->s6_addr[0]==0xff&&(a->s6_addr[1]&0x0f)==0x02)return 3;
	return 0;
}

static __inline__ int linklocal(ADDR *addr)
{
	if(addr->family!=AF_INET6)return 0;
	return a6linklocal(&addr->a6)?1:0;
}

static __inline__ void addr2sock(SOCK *dst,ADDR *src,unsigned short port,int devidx)
{
	switch(src->family)
	{
	case AF_INET:
		dst->a4.sin_family=AF_INET;
		dst->a4.sin_addr.s_addr=src->a4.s_addr;
		dst->a4.sin_port=port;
		break;
	case AF_INET6:
		dst->a6.sin6_family=AF_INET6;
		memcpy(dst->a6.sin6_addr.s6_addr,src->a6.s6_addr,16);
		dst->a6.sin6_port=port;
		dst->a6.sin6_flowinfo=0;
		dst->a6.sin6_scope_id=linklocal(src)?devidx:0;
		break;
	}
}

static __inline__ void sock2addr(ADDR *dst,SOCK *src)
{
	switch(sockfam(src))
	{
	case AF_INET:
		dst->family=AF_INET;
		dst->a4.s_addr=src->a4.sin_addr.s_addr;
		break;
	case AF_INET6:
		dst->family=AF_INET6;
		memcpy(dst->a6.s6_addr,src->a6.sin6_addr.s6_addr,16);
		break;
	}
}

static __inline__ int aainc(ADDR *a)
{
	int i;

	switch(a->family)
	{
	case AF_INET:
		a->a4.s_addr=htonl(((unsigned)ntohl(a->a4.s_addr))+1);
		return 0;
	case AF_INET6:
		for(i=15;i>=0;i--)if(++a->a6.s6_addr[i])break;
		return 0;
	default:return -1;
	}
}

static __inline__ int aacmp(ADDR *a1,ADDR* a2)
{
	if(a1->family!=a2->family)return -1;
	switch(a1->family)
	{
	case AF_INET:
		return a1->a4.s_addr==a2->a4.s_addr?0:-1;
	case AF_INET6:
		return memcmp(a1->a6.s6_addr,a2->a6.s6_addr,16)?-1:0;
	default:return -1;
	}
}

static __inline__ int ascmp(ADDR *a1,SOCK *a2)
{
	switch(a1->family)
	{
	case AF_INET:
		if(a2->a4.sin_family!=AF_INET)return -1;
		return a1->a4.s_addr==a2->a4.sin_addr.s_addr?0:-1;
	case AF_INET6:
		if(a2->a6.sin6_family!=AF_INET6)return -1;
		return memcmp(a1->a6.s6_addr,a2->a6.sin6_addr.s6_addr,16)?-1:0;
	default:return -1;
	}
}

static __inline__ int sscmp(SOCK *a1,SOCK *a2)
{
	switch(a1->a4.sin_family)
	{
	case AF_INET:
		if(a2->a4.sin_family!=AF_INET)return -1;
		return a1->a4.sin_addr.s_addr==a2->a4.sin_addr.s_addr?0:-1;
	case AF_INET6:
		if(a2->a6.sin6_family!=AF_INET6)return -1;
		return memcmp(a1->a6.sin6_addr.s6_addr,a2->a6.sin6_addr.s6_addr,16)?-1:0;
	default:return -1;
	}
}

static __inline__ const char *sock2str(SOCK *addr,char *dst,int size)
{
	switch(sockfam(addr))
	{
	case AF_INET:
		return inet_ntop(AF_INET,&addr->a4.sin_addr.s_addr,dst,size);
	case AF_INET6:
		return inet_ntop(AF_INET6,addr->a6.sin6_addr.s6_addr,dst,size);
	default:if(size>0)*dst=0;
		return NULL;
	}
}

static __inline__ int str2addr(char *src,ADDR *addr)
{
	if(inet_pton(AF_INET,src,&addr->a4)==1)addr->family=AF_INET;
	else if(inet_pton(AF_INET6,src,&addr->a6)==1)addr->family=AF_INET6;
	else return -1;
	return 0;
}

static __inline__ const char *addr2str(ADDR *addr,char *dst,int size)
{
	switch(addr->family)
	{
	case AF_INET:
		return inet_ntop(AF_INET,&addr->a4.s_addr,dst,size);
	case AF_INET6:
		return inet_ntop(AF_INET6,addr->a6.s6_addr,dst,size);
	default:if(size>0)*dst=0;
		return NULL;
	}
}

static __inline__ const char *addr2bstr(ADDR *addr,char *dst,int size)
{
	int l;
	char bfr[INET6_ADDRSTRLEN];

	switch(addr->family)
	{
	case AF_INET:
		return inet_ntop(AF_INET,&addr->a4.s_addr,dst,size);
	case AF_INET6:
		if(!inet_ntop(AF_INET6,addr->a6.s6_addr,bfr,sizeof(bfr)))return NULL;
		l=strlen(bfr);
		if(l+2>=size)return NULL;
		*dst='[';
		strcpy(dst+1,bfr);
		strcpy(dst+l+1,"]");
		return dst;
	default:if(size>0)*dst=0;
		return NULL;
	}
}

static __inline__ unsigned short psget(SOCK *addr)
{
	switch(sockfam(addr))
	{
	case AF_INET:
		return addr->a4.sin_port;
	case AF_INET6:
		return addr->a6.sin6_port;
	default:return 0;
	}
}

static __inline__ int addr2idx(ADDR *addr)
{
	switch(addr->family)
	{
	case AF_INET:
		return ntohl(addr->a4.s_addr)&0xff;
	case AF_INET6:
		return (unsigned char)addr->a6.s6_addr[15];
	default:return 0;
	}
}

static __inline__ int sock2idx(SOCK *addr)
{
	switch(sockfam(addr))
	{
	case AF_INET:
		return ntohl(addr->a4.sin_addr.s_addr)&0xff;
	case AF_INET6:
		return (unsigned char)addr->a6.sin6_addr.s6_addr[15];
	default:return 0;
	}
}

static __inline__ int invalid_sock(SOCK *addr,int ll)
{
	switch(sockfam(addr))
	{
	case AF_INET:
		if(ll<sizeof(addr->a4))return -1;
		break;
	case AF_INET6:
		if(ll<sizeof(addr->a6))return -1;
		break;
	default:return -1;
	}
	return 0;
}

static __inline__ int addr2level(ADDR *addr)
{
	switch(addr->family)
	{
	case AF_INET:
		if(!addr->a4.s_addr)return -1;
		return SATIP_V4_ONLY;
	case AF_INET6:
		if(a6sitewide(&addr->a6))return SATIP_V6_SITE;
		if(a6linklocal(&addr->a6))return SATIP_V6_LINK;
	default:return -1;
	}
}

static __inline__ int invalid_mcast(ADDR *addr)
{
	int a4;
	unsigned char *a6;

	switch(addr->family)
	{
	case AF_INET:
		a4=ntohl(addr->a4.s_addr);
		if((a4&0xf0000000)!=0xe0000000)return -1;
		if((a4&0xf7000000)==0xe0000000)return -2;
		if(a4==0xeffffffa)return -2;
		return 0;

	case AF_INET6:
		a6=addr->a6.s6_addr;
		if(a6[0]!=0xff)return -1;
		switch(a6[1]&0x0f)
		{
		case 0x02:
		case 0x05:
			if(a6[2]||a6[3])return 0;
			if((a6[1]&0xf0)!=0x30)
			{
				if(memcmp(a6+4,"\x0\x0\x0\x0\x0\x0\x0",7))return 0;
				switch(a6[11])
				{
				case 0x00:
					if(a6[12]||a6[13]>=3)return 0;
					break;

				case 0x01:
				case 0x02:
					if(a6[12]!=0xff)return 0;
					break;

				default:return 0;
				}
			}
		default:return -2;
		}

	default:return -1;
	}
}

#endif

#endif
