/*
   This file is part of the satip library.

   When building or linking against the shared library:

   The satip Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   When building not for or not linking against the shared library:

   The satip Library is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation, version 2.

   (c) 2016 Andreas Steinmetz ast@domdv.de
 */

#ifndef SATIP_H_INCLUDED
#define SATIP_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#define SATIP_HTTP_PORT 80
#define SATIP_HTTP_AUTO	0

#define SATIP_RTSP_PORT 554
#define SATIP_RTSP_AUTO	0

#define SATIP_MCST_PORT	1234
#define SATIP_MCST_AUTO	0

#define SATIP_MAX_PIDS	128
#define SATIP_UUID_LEN	36
#define SATIP_UPNP_AGE	1800
#define SATIP_MCST_TTL	2
#define SATIP_ADDR_LEN	45
#define SATIP_CSET_LEN	31
#define SATIP_FILTERSZ	16
#define SATIP_SEARCHMHZ	2
#define SATIP_EXTRA_TOT	16
#define SATIP_MAX_BURST	4

#define SATIP_GETRANDOM	0

#define SATIP_SAVEBCNT	1
#define SATIP_SAVEUUID	2
#define SATIP_SAVEDEVID	3
#define SATIP_LOADBCNT	4
#define SATIP_LOADUUID	5
#define SATIP_LOADDEVID	6

#define SATIP_COPYM3U	7

#define SATIP_PEEROK	8
#define SATIP_HTTPOK	9
#define SATIP_RTSPOK	10
#define SATIP_MCSTOK	11

#define SATIP_HTTPREQ	12
#define SATIP_PNG48	13
#define SATIP_PNG120	14
#define SATIP_JPG48	15
#define SATIP_JPG120	16

#define SATIP_STRPLAY	17
#define SATIP_STRPIDS	18
#define SATIP_STREND	19

#define SATIP_RUNNING	20
#define SATIP_STOPPING	21

#define SATIP_RTP	0x00010000
#define SATIP_RTCP	0x00020000
#define SATIP_MARKER	0x00040000
#define SATIP_LOSTPKT	0x00080000

#define SATIP_V4_ONLY	0
#define SATIP_V6_LINK	1
#define SATIP_V6_SITE	2

#define SATIP_IGMPNONE	0
#define SATIP_IGMPSNOOP	1
#define SATIP_IGMPQUERY	2
#define SATIP_IGMPFORCE	3

#define SATIP_MLDNONE	0
#define SATIP_MLDSNOOP	1
#define SATIP_MLDQUERY	2
#define SATIP_MLDFORCE	3

#define SATIP_HW_REMOTE	0
#define SATIP_HW_LOCAL	1

#define SATIP_LNB_UNIV	0
#define SATIP_LNB_DBS	1
#define SATIP_LNB_CMONO	2
#define SATIP_LNB_CMULT	3
#define SATIP_LNB_AUS	4

#define SATIP_ANNEX_A	0

#define SATIP_SNR_SLOW	0
#define SATIP_SNR_MED	1
#define SATIP_SNR_FAST	2

#define SATIP_DSC_NONE	0
#define SATIP_DSC_TONE	1
#define SATIP_DSC_1_0	2
#define SATIP_DSC_2_0	3
#define SATIP_DSC_1_1	4
#define SATIP_DSC_2_1	5
#define SATIP_DSC_1_0_T	6
#define SATIP_DSC_2_0_T	7
#define SATIP_DSC_1_1_T	8
#define SATIP_DSC_2_1_T	9

#define SATIP_CAM_SLOTS	8

#define SATIP_CAM_AVAIL	1
#define SATIP_CAM_READY	2

#define SATIP_CAM_STATE	0
#define SATIP_CAM_RESET	1
#define SATIP_CAM_READ	2
#define SATIP_CAM_WRITE	3

#define SATIP_SCAN_PARA	8

#define SATIP_SCAN_STD	0x01
#define SATIP_SCAN_FAST	0x02
#define SATIP_SCAN_LIST	0x03
#define SATIP_SCAN_HW	0x10
#define SATIP_SCAN_HTTP	0x20
#define SATIP_SCAN_RTSP	0x30

#define SATIP_SCAN_CBI	0
#define SATIP_SCAN_CBU	1
#define SATIP_SCAN_CBF	2

#define SATIP_SCAN_NIT	1
#define SATIP_SCAN_NITS	2

#define SATIP_ADDSID	0x01
#define SATIP_ADDPMT	0x02
#define SATIP_ADDXPMT	0x04
#define SATIP_STRICTQRY	0x08

#define SATIP_IGNCAPS	0x01
#define SATIP_SADOK	0x02
#define SATIP_RTPQUERY	0x04
#define SATIP_IGNPLPETC	0x08

#define SATIP_SYSFAIL	-1
#define SATIP_STRLIMIT	-2
#define SATIP_DEVLIMIT	-3
#define SATIP_TUNERERR	-4
#define SATIP_ERR_SYNTX	-5
#define SATIP_ERR_VALUE	-6
#define SATIP_ERR_OVER	-7
#define SATIP_NODATA	-8
#define SATIP_PARTIAL	-9
#define SATIP_NOMATCH	-10

#define SATIP_FLGSTART	0x00000001
#define SATIP_FLGSECT	0x00000002

#define SATIP_UPNP_OFF	-1
#define SATIP_UPNP_ONCE	0
#define SATIP_UPNP_MAX	60

#define SATIP_PARSE_URL	0
#define SATIP_PARSE_QRY	1
#define SATIP_PARSE_PID	2
#define SATIP_PARSE_HST 3

#define SATIP_TYPE_HTTP	0
#define SATIP_TYPE_RTSP	1
#define SATIP_TYPE_RTP	2
#define SATIP_TYPE_QRY	3

#define SATIP_NIT_MAX	16
#define SATIP_ONT_MAX	16
#define SATIP_CATCA_MAX	16
#define SATIP_PMTCA_MAX	16
#define SATIP_SDTCA_MAX	16
#define SATIP_FRALT_MAX	32

#define SATIP_PMT_VIDEO	1
#define SATIP_PMT_AVC	2
#define SATIP_PMT_HEVC	3
#define SATIP_PMT_AUDIO	4
#define SATIP_PMT_ADTS	5
#define SATIP_PMT_AC3	6
#define SATIP_PMT_EAC3	7
#define SATIP_PMT_LATM	8
#define SATIP_PMT_TTX	9
#define SATIP_PMT_SUBS	10

#define SATIP_SDT_NORUN	1
#define SATIP_SDT_START	2
#define SATIP_SDT_PAUSE	3
#define SATIP_SDT_RUN	4
#define SATIP_SDT_OFAIR	5

#define SATIP_SDT_MP2SD	1
#define SATIP_SDT_MP2HD	2
#define SATIP_SDT_ACSD	3
#define SATIP_SDT_ACHD	4
#define SATIP_SDT_MP1AU	5
#define SATIP_SDT_ACAU	6
#define SATIP_SDT_FMAU	7
#define SATIP_SDT_CI	8
#define SATIP_SDT_TTX	9
#define SATIP_SDT_DATA	10
#define SATIP_SDT_OTHER	11

#define SATIP_TOTALS	5
#define SATIP_TOT_DVBS2	0
#define SATIP_TOT_DVBT	1
#define SATIP_TOT_DVBT2	2
#define SATIP_TOT_DVBC	3
#define SATIP_TOT_DVBC2	4

#define SATIP_UNSPEC	0
#define SATIP_UNDEF	-1

#define SATIP_DVBS	1
#define SATIP_DVBS2	2
#define SATIP_DVBT	3
#define SATIP_DVBT2	4
#define SATIP_DVBC	5
#define SATIP_DVBC2	6

#define SATIP_POL_H	1
#define SATIP_POL_V	2
#define SATIP_POL_L	3
#define SATIP_POL_R	4

#define SATIP_ALLPIDS	-1
#define SATIP_NOPIDS	-2
#define SATIP_SECTION	-3
#define SATIP_SIGNAL	-4

#define SATIP_ROFF_035	1
#define SATIP_ROFF_025	2
#define SATIP_ROFF_020	3
#define SATIP_ROFF_AUTO	0x00010000

#define SATIP_QPSK	1
#define SATIP_8PSK	2
#define SATIP_16APSK	3
#define SATIP_32APSK	4
#define SATIP_16Q	5
#define SATIP_32Q	6
#define SATIP_64Q	7
#define SATIP_128Q	8
#define SATIP_256Q	9
#define SATIP_AUTOQ	0x00020000

#define SATIP_PLTS_OFF	1
#define SATIP_PLTS_ON	2
#define SATIP_PLTS_AUTO	0x00040000

#define SATIP_FEC_12	1
#define SATIP_FEC_23	2
#define SATIP_FEC_34	3
#define SATIP_FEC_35	4
#define SATIP_FEC_45	5
#define SATIP_FEC_56	6
#define SATIP_FEC_78	7
#define SATIP_FEC_89	8
#define SATIP_FEC_910	9
#define SATIP_FEC_NONE	10
#define SATIP_FEC_RS204	11
#define SATIP_FEC_AUTO	0x00080000

#define SATIP_BW_1712	1
#define SATIP_BW_5	2
#define SATIP_BW_6	3
#define SATIP_BW_7	4
#define SATIP_BW_8	5
#define SATIP_BW_10	6
#define SATIP_BW_AUTO	0x00100000

#define SATIP_TMOD_1K	1
#define SATIP_TMOD_2K	2
#define SATIP_TMOD_4K	3
#define SATIP_TMOD_8K	4
#define SATIP_TMOD_16K	5
#define SATIP_TMOD_32K	6
#define SATIP_TMOD_AUTO	0x00200000

#define SATIP_GI_14	1
#define SATIP_GI_18	2
#define SATIP_GI_116	3
#define SATIP_GI_132	4
#define SATIP_GI_164	5
#define SATIP_GI_1128	6
#define SATIP_GI_19128	7
#define SATIP_GI_19256	8
#define SATIP_GI_AUTO	0x00400000

#define SATIP_SM_SISO	1
#define SATIP_SM_MISO	2
#define SATIP_SM_AUTO	0x00800000

#define SATIP_TFT_DS	1
#define SATIP_TFT_C2	2
#define SATIP_TFT_IT	3
#define SATIP_TFT_AUTO	0x01000000

#define SATIP_SPI_OFF	1
#define SATIP_SPI_ON	2
#define SATIP_SPI_AUTO	0x02000000

#define SATIP_HIER_NONE	1
#define SATIP_HIER_1	2
#define SATIP_HIER_2	3
#define SATIP_HIER_4	4
#define SATIP_HIER_AUTO	0x04000000

#define SATIP_LNA_OFF	1
#define SATIP_LNA_ON	2
#define SATIP_LNA_AUTO	0

#ifndef IFNAMSIZ 
#define IFNAMSIZ	16
#endif

typedef struct
{
	int intval;
	void *ptrval;
} SATIP_DATA;

typedef struct
{
	unsigned long long freq;
	int fe;
	int src;
	int bw;
	int pol;
	int msys;
	int tmode;
	int mtype;
	int plts;
	int ro;
	int sr;
	int gi;
	int fec;
	int feclp;
	int hier;
	int c2tft;
	int ds;
	int plp;
	int t2id;
	int sm;
	int specinv;
} SATIP_TUNE;

typedef struct
{
	int prognum;
	int numpids;
	union
	{
		short pids[SATIP_MAX_PIDS];
		struct
		{
			int pid;
			int table;
			union
			{
				struct
				{
					int bits;
					unsigned char chr[3*SATIP_FILTERSZ];
				} extra;
				struct
				{
					int version:5;
					int filterversion:1;
					int filtercurrent:1;
					int filterraw:1;
					int filtercrc:1;
					unsigned char filter[SATIP_FILTERSZ];
					unsigned char mask[SATIP_FILTERSZ];
					unsigned char mode[SATIP_FILTERSZ];
				};
			};
		};
	};
} SATIP_PIDS;

typedef struct
{
	int total;
	char name[SATIP_EXTRA_TOT][16];
	char value[SATIP_EXTRA_TOT][16];
} SATIP_EXTRA;

typedef struct
{
	SATIP_TUNE tune;
	SATIP_PIDS set;
	int lock;
	int level;
	int quality;
} SATIP_STATUS;

typedef struct
{
	SATIP_TUNE *tune;
	SATIP_PIDS *set;
	void *handle;
	int terminate;
} SATIP_STRDATA;

typedef struct
{
	unsigned int khz90;
	int flags;
	int fill;
#pragma pack(push,1)
	union
	{
		unsigned char *section;
		char msg[1328];
		struct
		{
			union
			{
				unsigned char hdr1[12];
				unsigned short hdr2[6];
				unsigned int hdr4[3];
			};
			char data[1316];
		};
	};
#pragma pack(pop)
} SATIP_STREAM;

typedef struct
{
	unsigned int caps;
	int totals[SATIP_TOTALS];
} SATIP_CFGINFO;

typedef struct
{
	int deviceid;
	int type;
	int slot;
	union
	{
		int tsid;
		int state;
	};
	int len;
	unsigned char *data;
} SATIP_HW_CAM_IO;

typedef struct
{
	unsigned int caps;
	int access;
	int msys;
	int open;
	int streams;
	int lock;
	int level;
	int quality;
	SATIP_TUNE tune;
	unsigned long long streambytes;
	unsigned long long byteupdates;
	int groupstreams;
} SATIP_HW_STATUS;

typedef struct
{
	int deviceid;
	int adapter;
	int frontend;
	int demux;
	int ca;
	int inversion;
	int lna;
	int snrshift;
	int snrspeed;
	int lnbtype;
	int dvbctype;
	int diseqc;
	int dscforce;
	int dscwait;
	int dscreplywait;
	int idleflush;
	int fast;
	int prefer;
	int explicit;
	int noshare;
	int srcnum;
	int src[32];
} SATIP_HW_TUNERCFG;

typedef struct
{
	void (*camfunc)(SATIP_HW_CAM_IO *msg,void *priv);
	void *priv;
	int streamlimit;
	int rtprio;
	int stack;
} SATIP_HW_CONFIG;

typedef struct _satip_srv_info
{
	struct _satip_srv_info *next;
	SATIP_STATUS state;
	int streamid;
	int playing;
	int port;
	char addr[SATIP_ADDR_LEN+1];
} SATIP_SRV_INFO;

typedef struct
{
	int rtsp_all_sessions;
	int rtsp_client_sessions;
	int rtsp_playing_sessions;
	int http_running;
	int http_playing;
} SATIP_SRV_STATS;

typedef struct
{
	char dev[IFNAMSIZ+1];
	char xmlcharset[64];
	int level;
	int rtspport;
	int httpport;
	int mdftport;
	int igmpv3;
	int mldv2;
	int upnpage;
	int mttl;
	int rtsplimit;
	int httplimit;
	int httpnostream;
	int strict;
	int timeout;
	int locked;
	int burst;
	int portmin;
	int portmax;
	int stack;
	unsigned long long bytespersec;
	int (*callback)(int code,void *data,void *priv);
	void *priv;
	char friendlyname[128];
	char manufacturer[128];
	char manufacturerurl[128];
	char modeldescription[128];
	char modelname[128];
	char modelnumber[128];
	char modelurl[128];
	char serialnumber[128];
	char upc[128];
	char presentationurl[128];
	char png48url[128];
	char png120url[128];
	char jpg48url[128];
	char jpg120url[128];
	char m3uurl[128];
	int png48depth;
	int png120depth;
	int jpg48depth;
	int jpg120depth;
	int havem3u;
} SATIP_SRV_CONFIG;

typedef struct
{
	int totals[SATIP_TOTALS];
	char *friendlyname;
	char *presentationurl;
	char *playlisturl;
	char *png48url;
	char *png120url;
	char *jpg48url;
	char *jpg120url;
	char *manufacturername;
	char *manufacturerurl;
	char *modelname;
	char *modelnumber;
	char *modelurl;
	char *modeldescription;
	char *serial;
	char *upc;
	char data[0];
} SATIP_CLN_UPNPINFO;

typedef struct _satip_cln_upnplist
{
	struct _satip_cln_upnplist *next;
	struct _satip_cln_upnplist *same;
	int level;
	int bootcount;
	int configid;
	char uuid[SATIP_UUID_LEN+1];
	char addr[SATIP_ADDR_LEN+1];
	char location[0];
} SATIP_CLN_UPNPLIST;

typedef struct _satip_cln_streaminfo
{
	struct _satip_cln_streaminfo *next;
	char addr[SATIP_ADDR_LEN+1];
	int port;
	int ttl;
	int stream;
	int type;
	int inactive;
	SATIP_STATUS info;

} SATIP_CLN_STREAMINFO;

typedef struct
{
	char dev[IFNAMSIZ+1];
	int level;
	int mttl;
	int interval;
	int idleflush;
	int portmin;
	int portmax;
	int rtpbuffer;
	int strict;
	int fast;
	int stack;
	char charset[SATIP_CSET_LEN+1];
} SATIP_CLN_CONFIG;

typedef struct _satip_util_sdt
{
	struct _satip_util_sdt *next;
	int tsid;
	int vernum;
	int cnind;
	int secnum;
	int lastsec;
	int onid;
	int total;
	struct
	{
		int prognum;
		int eitsched;
		int eitpres;
		int running;
		int fta;
		int type;
		int catotal;
		int caid[SATIP_SDTCA_MAX];
		char provname[64];
		char servname[64];
	} data[0];
} SATIP_UTIL_SDT;

typedef struct _satip_util_pmt
{
	struct _satip_util_pmt *next;
	SATIP_UTIL_SDT *sdt;
	int prognum;
	int vernum;
	int cnind;
	int secnum;
	int lastsec;
	int pcrpid;
	int sdtindex;
	int total;
	int catotal;
	unsigned short caid[SATIP_PMTCA_MAX];
	unsigned short capid[SATIP_PMTCA_MAX];
	int raw[SATIP_PMTCA_MAX];
	struct
	{
		int pid:16;
		int type:8;
		int typenum:8;
		int type1:8;
		int catotal:24;
		unsigned short caid[SATIP_PMTCA_MAX];
		unsigned short capid[SATIP_PMTCA_MAX];
		int raw[SATIP_PMTCA_MAX];
		char lang[4];
	} data[0];
} SATIP_UTIL_PMT;

typedef struct _satip_util_cat
{
	struct _satip_util_cat *next;
	int vernum;
	int cnind;
	int secnum;
	int lastsec;
	int catotal;
	unsigned short caid[SATIP_CATCA_MAX];
	unsigned short capid[SATIP_CATCA_MAX];
	int raw[SATIP_CATCA_MAX];
} SATIP_UTIL_CAT;

typedef struct _satip_util_pat
{
	struct _satip_util_pat *next;
	int tsid;
	int vernum;
	int cnind;
	int secnum;
	int lastsec;
	int total;
	struct
	{
		int prognum;
		union
		{
			int netpid;
			int pmtpid;
		};
		SATIP_UTIL_PMT *pmt;
	} data[0];
} SATIP_UTIL_PAT;

typedef struct _satip_util_nit
{
	struct _satip_util_nit *next;
	int netid;
	int vernum;
	int cnind;
	int secnum;
	int lastsec;
	int total;
	char netname[64];
	struct
	{
		SATIP_TUNE tune;
		int tsid;
		int onid;
		int priority;
		int tslice;
		int mpefec;
		int feco;
		int feci;
		int c2gi;
		int othfreq;
		int cellid;
		int alttotal;
		int mis;
		int ssi;
		char satpos[6];
		struct
		{
			int cellid;
			int istransposer;
			unsigned long long freq;
		} alt[SATIP_FRALT_MAX];
	} data[0];
} SATIP_UTIL_NIT;

typedef struct 
{
	unsigned char filter[SATIP_FILTERSZ];
	unsigned char mask[SATIP_FILTERSZ];
	unsigned char mode[SATIP_FILTERSZ];
} SATIP_UTIL_SECFILTER;

typedef struct
{
	union
	{
		struct
		{
			unsigned int prognum:16;
			unsigned int sdttype:4;
			unsigned int streams:9;
			unsigned int tv:1;
			unsigned int radio:1;
			unsigned int fta:1;
			unsigned int index:16;
			unsigned int sdtfta:1;
			unsigned int eit:1;
			unsigned int m3u_eit_exclude:1;
			unsigned int pgroup:13;
			unsigned int tsid:16;
			unsigned int nid:16;
			unsigned int pmtpid:13;
			unsigned int pcrpid:13;
			unsigned int catotal:6;
		};
		unsigned int bits[4];
	};
	unsigned short caid[SATIP_SDTCA_MAX];
	unsigned int user[4];
	char *progname;
	char *provider;
	SATIP_TUNE tune;
#pragma pack(push,1)
	struct
	{
		unsigned int pmttype:4;
		unsigned int pmttnum:8;
		unsigned int pid:13;
		unsigned int m3u_exclude:1;
		unsigned int reserved:6;
		char lang[4];
	} stream[0];
#pragma pack(pop)
} SATIP_UTIL_PROGRAM;

typedef struct
{
	int lock;
	int level;
	int quality;
	int deviceid;
	int total;
	int done;
	int info;
	int globdone;
	int globtotal;
	int tuner;
} SATIP_SCAN_STATUS;

typedef struct
{
	unsigned char present:1;
	unsigned char ts_scramble:1;
	unsigned char psi:1;
	unsigned char pes:1;
	unsigned char pes_scramble:1;
	unsigned char audio:1;
	unsigned char video:1;
	unsigned char ecmemm:1;
} SATIP_SCAN_PIDINFO;

typedef struct
{
	unsigned int duration:16;
	unsigned int partial:1;
	unsigned int fastfind:1;
	SATIP_TUNE requested;
	SATIP_TUNE actual;
	SATIP_UTIL_PAT *pat;
	SATIP_UTIL_SDT *sdt;
	SATIP_UTIL_NIT *nit[SATIP_NIT_MAX];
	SATIP_UTIL_NIT *ont[SATIP_ONT_MAX];
	SATIP_SCAN_PIDINFO *pidinfo;
} SATIP_SCAN_RESULT;

typedef struct
{
	int termfd;
	int locktimeout;
	int mintimeout;
	int basetimeout;
	int maxtimeout;
	int pmttimeout;
	int pmtparallel;
	int pidscan;
	int getnit;
	int tuneonly;
	int fe;
} SATIP_SCAN_PARAMS;

extern int satip_srv_clr_all(void *handle);
extern int satip_srv_clr_rtsp(void *handle);
extern int satip_srv_clr_http(void *handle);
extern int satip_srv_clr_stream(void *handle,int streamid);
extern int satip_srv_clr_device(void *handle,int deviceid);
extern int satip_srv_set_all_locks(void *handle,int islocked);
extern int satip_srv_set_rtsp_lock(void *handle,int islocked);
extern int satip_srv_set_http_lock(void *handle,int islocked);
extern int satip_srv_add_multicast(void *handle,char *addr,int port,int ttl,
	int play,SATIP_TUNE *tune,SATIP_PIDS *set);
extern int satip_srv_del_multicast(void *handle,int streamid);
extern int satip_srv_clr_multicast(void *handle);
extern int satip_srv_statistics(void *handle,SATIP_SRV_STATS *stats);
extern SATIP_SRV_INFO *satip_srv_list_rtsp(void *handle);
extern SATIP_SRV_INFO *satip_srv_list_http(void *handle);
extern int satip_srv_free_list(void *handle,SATIP_SRV_INFO *list);
extern void satip_srv_stream(void *id,SATIP_STREAM *stream);
extern void satip_srv_status(void *id,SATIP_STATUS *status);
extern int satip_srv_forward(void *handle,char *host,int port,SATIP_DATA *data);
extern void *satip_srv_init(SATIP_SRV_CONFIG *config,SATIP_CFGINFO *hwinfo);
extern void satip_srv_fini(void *handle);

extern int satip_hw_cam_io(void *handle,SATIP_HW_CAM_IO *msg);
extern int satip_hw_play(void *handle,SATIP_STRDATA *params,
	void (*stream)(void *id,SATIP_STREAM *stream),
        void (*status)(void *id,SATIP_STATUS *status),
	void *user,int access);
extern int satip_hw_end(void *handle,SATIP_STRDATA *params);
extern int satip_hw_setpids(void *handle,SATIP_STRDATA *params);
extern int satip_hw_access(void *handle,int deviceid,int access);
extern int satip_hw_info(void *handle,int deviceid,SATIP_HW_STATUS *s);
extern int satip_hw_add(void *handle,SATIP_HW_TUNERCFG *tuner,
	SATIP_CFGINFO *info);
extern void *satip_hw_init(SATIP_HW_CONFIG *config,SATIP_CFGINFO *info);
extern void satip_hw_fini(void *handle);

extern int satip_remap_play(void *handle,SATIP_STRDATA *params,
	int (*play)(void *handle,SATIP_STRDATA *params,
		void (*stream)(void *id,SATIP_STREAM *stream),
		void (*status)(void *id,SATIP_STATUS *status),
		void *user,int access),
	int (*end)(void *handle,SATIP_STRDATA *params),
	int (*setpids)(void *handle,SATIP_STRDATA *params),
	void (*stream)(void *id,SATIP_STREAM *stream),
        void (*status)(void *id,SATIP_STATUS *status),
	void *user,int access);
extern int satip_remap_end(void *handle,SATIP_STRDATA *params);
extern int satip_remap_setpids(void *handle,SATIP_STRDATA *params);

extern int satip_cln_upnplist(void *handle,SATIP_CLN_UPNPLIST **list);
extern int satip_cln_freelist(void *handle,SATIP_CLN_UPNPLIST *list);
extern int satip_cln_upnpinfo(void *handle,SATIP_CLN_UPNPLIST *entry,
	SATIP_CLN_UPNPINFO **info);
extern int satip_cln_freeinfo(void *handle,SATIP_CLN_UPNPINFO *info);
extern int satip_cln_upnpitem(void *handle,char *url,void **data,int *size);
extern int satip_cln_freeitem(void *handle,void *data);
extern int satip_cln_stream_http(void *handle,char *server,int port,
	SATIP_TUNE *tune,SATIP_PIDS *set,SATIP_EXTRA *extra,
	void (*cb)(SATIP_DATA *data,void *priv),void *priv,void **stream);
extern int satip_cln_stream_multicast(void *handle,char *address,int port,
	void (*cb)(SATIP_DATA *data,void *priv),void *priv,void **stream);
extern int satip_cln_stream_unicast(void *handle,char *server,int port,
	int persist,SATIP_TUNE *tune,SATIP_PIDS *set,SATIP_EXTRA *extra,
	void (*cb)(SATIP_DATA *data,void *priv),void *priv,void **stream);
extern int satip_cln_change_unicast(void *handle,void *stream,SATIP_PIDS *set,
	SATIP_PIDS *add,SATIP_PIDS *del,SATIP_EXTRA *extra);
extern int satip_cln_stream_stop(void *handle,void *stream);
extern int satip_cln_streaminfo(void *handle,char *server,int port,
	SATIP_CLN_STREAMINFO **info);
extern int satip_cln_streaminfo_free(SATIP_CLN_STREAMINFO *info);
extern int satip_cln_setup_multicast(void *handle,char *server,int port,
	char *maddr,int mport,int mttl,int play,char *sessionid,int size,
	int *streamid,SATIP_TUNE *tune,SATIP_PIDS *set,SATIP_EXTRA *extra);
extern int satip_cln_end_multicast(void *handle,char *server,int port,
	char *sessionid,int streamid);
extern void *satip_cln_init(SATIP_CLN_CONFIG *config);
extern void satip_cln_fini(void *handle);

extern int satip_util_parse(int mode,unsigned int caps,int flags,char *input,
	int *type,char *addr,int size,int *port,int *stream,SATIP_TUNE *tune,
	SATIP_PIDS *set,SATIP_PIDS *add,SATIP_PIDS *del,SATIP_EXTRA *extra);
extern int satip_util_create(int type,int flags,char *addr,int port,int stream,
	SATIP_TUNE *tune,SATIP_PIDS *set,SATIP_PIDS *add,SATIP_PIDS *del,
	SATIP_EXTRA *extra,char *output,int size);
extern int satip_util_random(unsigned char *dst,int amount);
extern SATIP_UTIL_PAT *satip_util_unpack_pat_section(unsigned char *data,
	int len);
extern SATIP_UTIL_CAT *satip_util_unpack_cat_section(unsigned char *data,
	int len);
extern SATIP_UTIL_PMT *satip_util_unpack_pmt_section(unsigned char *data,
	int len);
extern SATIP_UTIL_NIT *satip_util_unpack_nit_section(unsigned char *data,
	int len);
extern SATIP_UTIL_SDT *satip_util_unpack_sdt_section(unsigned char *data,
	int len);
extern unsigned char *satip_util_get_raw_cat(SATIP_UTIL_CAT *cat,int offset);
extern unsigned char *satip_util_get_raw_pmt(SATIP_UTIL_PMT *pmt,int offset);
extern int satip_util_get_raw_len(unsigned char *ptr);
extern int satip_util_list_init(void **list);
extern int satip_util_list_free(void *list);
extern int satip_util_list_add(void *list,char *addr);
extern int satip_util_list_del(void *list,char *addr);
extern int satip_util_list_match_data(void *list,SATIP_DATA *data);
extern int satip_util_list_match_addr(void *list,char *addr);
extern int satip_util_list_total(void *list);
extern int satip_util_data2addr(SATIP_DATA *data,char *addr,int size);
extern int satip_util_addrinc(char *addr,int size);
extern int satip_util_filter_create(void **filter);
extern int satip_util_filter_free(void *filter);
extern int satip_util_filter_packets(void *filter,void *data,int len);
extern int satip_util_filter_packets_cb(void *filter,void *data,int len,
	void (*next)(void *priv),void *priv);
extern int satip_util_filter_add_user(void *filter,void **user,
	void (*cb)(void *data,int len,void *priv),void *priv);
extern int satip_util_filter_del_user(void *filter,void *user);
extern int satip_util_user_addpid(void *user,int pid);
extern int satip_util_user_delpid(void *user,int pid);
extern int satip_util_user_clrpid(void *user);
extern int satip_util_section_create(void **section,
	SATIP_UTIL_SECFILTER *filter,int checkcrc,
	void (*cb)(void *data,int len,void *priv),void *priv);
extern int satip_util_section_free(void *section);
extern int satip_util_section_reset(void *section);
extern void satip_util_section_packet(void *data,int len,void *section);
extern void satip_util_init_tune(SATIP_TUNE *tune);
extern void satip_util_init_pids(SATIP_PIDS *set);
extern int satip_util_tunecmp(SATIP_TUNE *t1,SATIP_TUNE *t2);
extern int satip_util_program_list_create(SATIP_UTIL_PROGRAM ***list,int *size);
extern int satip_util_program_list(SATIP_SCAN_RESULT *res,int tot,
	char *user_locale,int sdtcaid,SATIP_UTIL_PROGRAM ***list,int *size);
extern int satip_util_program_list_free(SATIP_UTIL_PROGRAM **list,int size);
extern int satip_util_program_merge(char *user_locale,
	SATIP_UTIL_PROGRAM **list1,int size1,SATIP_UTIL_PROGRAM **list2,
	int size2,SATIP_UTIL_PROGRAM ***result,int *size,
	int (*deletecallback)(SATIP_UTIL_PROGRAM *p1,SATIP_UTIL_PROGRAM *p2));
extern int satip_util_program_create(SATIP_UTIL_PROGRAM **p,int streams,
	char *progname,char *provider);
extern int satip_util_program_append(SATIP_UTIL_PROGRAM ***list,int *size,
	SATIP_UTIL_PROGRAM *p);
extern int satip_util_program_remove(SATIP_UTIL_PROGRAM ***list,int *size,
	int index);
extern int satip_util_program_extract(SATIP_UTIL_PROGRAM ***list,int *size,
	int index,SATIP_UTIL_PROGRAM **p);
extern int satip_util_program_free(SATIP_UTIL_PROGRAM *p);
extern int satip_util_cvt_utf8(char *charset,char *src,int slen,char **dst,
	int *dlen);
extern int satip_util_create_rtp_setup(char *host,int port,int flags,
	SATIP_UTIL_PROGRAM *p,char *result,int size);
extern int satip_util_create_unicast_m3u(char *host,int port,int type,int flags,
	int crlf,char *charset,
	int (*nextprogram)(SATIP_UTIL_PROGRAM **p,int *user_prognum,void *priv),
	void *priv,char **result,int *size);
extern int satip_util_create_multicast_m3u(int port,int crlf,char *charset,
	int (*nextprogram)(SATIP_UTIL_PROGRAM **p,int *user_prognum,char *addr,
	int size,void *priv),void *priv,char **result,int *size);

extern int satip_scan_transponder_hw(void *h,SATIP_TUNE *tune,
	SATIP_SCAN_PARAMS *cfg,SATIP_SCAN_RESULT *tp,
	void (*status)(SATIP_SCAN_STATUS *status,void *priv),void *priv);
extern int satip_scan_transponder_http(void *h,SATIP_TUNE *tune,
	SATIP_SCAN_PARAMS *cfg,SATIP_SCAN_RESULT *tp,
	void (*status)(SATIP_SCAN_STATUS *status,void *priv),
	void *priv,char *host,int port);
extern int satip_scan_transponder_rtsp(void *h,SATIP_TUNE *tune,
	SATIP_SCAN_PARAMS *cfg,SATIP_SCAN_RESULT *tp,
	void (*status)(SATIP_SCAN_STATUS *status,void *priv),void *priv,
	char *host,int port);
extern int satip_scan_transponder_free(void *h,SATIP_SCAN_RESULT *tp);
extern int satip_scan_dvbs(void *h,SATIP_TUNE *l,int n,SATIP_SCAN_PARAMS *cfg,
	int tuners,int mode,int *felist,
	void (*status)(SATIP_TUNE *t,SATIP_SCAN_STATUS *st,void *priv),
	void *priv,SATIP_SCAN_RESULT **result,int *total,char *host,int port);
extern int satip_scan_dvbt(void *h,SATIP_TUNE *l,int n,SATIP_SCAN_PARAMS *cfg,
	int tuners,int mode,int *felist,
	void (*status)(SATIP_TUNE *t,SATIP_SCAN_STATUS *st,void *priv),
	void *priv,SATIP_SCAN_RESULT **result,int *total,char *host,int port);
extern int satip_scan_dvbc(void *h,SATIP_TUNE *l,int n,SATIP_SCAN_PARAMS *cfg,
	int tuners,int mode,int *symtab,int totsym,int *felist,
	void (*status)(SATIP_TUNE *t,SATIP_SCAN_STATUS *st,void *priv),
	void *priv,SATIP_SCAN_RESULT **result,int *total,char *host,int port);
extern int satip_scan_free(void *h,SATIP_SCAN_RESULT *result,int total);

#ifdef __cplusplus
}
#endif

#endif
