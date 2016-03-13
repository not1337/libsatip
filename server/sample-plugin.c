/*
 * plugin skeleton for example SAT>IP server daemon
 *
 * 2016 by Andreas Steinmetz (ast@domdv.de)
 *
 * This code is put in the public domain.
 *
 * All functions are optional. You have to choose between plugin_stream
 * which gets one ts packet at a time and plugin_multi which can process
 * multiple packets at once. If both functions are available only
 * plugin_multi will be called.
 * Note that in 'strict' mode plugin_multi is disabled due to SAT>IP
 * specification requirements (must send as soon as...).
 */

#include "satip.h"

void plugin_init(void *hwhandle,void **globalctx,char *config)
{
}

void plugin_fini(void *hwhandle,void *globalctx)
{
}

void plugin_stream(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_STREAM *stream)
{
}

void plugin_multi(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_STREAM **stream,int total,int totlen,int *done,int flush)
{
	if(total>=7||flush)*done=total;
}

void plugin_prepare(void *hwhandle,void *globalctx,void **streamctx,
	SATIP_TUNE *tune,SATIP_PIDS *set)
{
}

void plugin_play(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_TUNE *tune,SATIP_PIDS *set,int success)
{
}

void plugin_end(void *hwhandle,void *globalctx,void *streamctx)
{
}

void plugin_setpids(void *hwhandle,void *globalctx,void *streamctx,
	SATIP_PIDS *set)
{
}

void plugin_cam(void *hwhandle,void *globalctx,SATIP_HW_CAM_IO *msg)
{
	char dta[3];

	if(msg->type==SATIP_CAM_READ)
	{
	}
	else if(msg->type==SATIP_CAM_STATE)if(msg->state
				==(SATIP_CAM_AVAIL|SATIP_CAM_READY))
	{
		msg->type=SATIP_CAM_WRITE;
		msg->len=3;
		msg->data=dta;
		msg->tsid=msg->slot+1;
		dta[0]=0x82;
		dta[1]=1;
		dta[2]=(unsigned char)msg->tsid;
		satip_hw_cam_io(hwhandle,msg);
	}
}
