/*
 * plugin skeleton for example SAT>IP DVB loop driver
 *
 * 2016 by Andreas Steinmetz (ast@domdv.de)
 *
 * This code is put in the public domain.
 *
 * All functions are optional. You have to choose between plugin_stream
 * which gets one ts packet at a time and plugin_multi which can process
 * multiple packets at once. If both functions are available only
 * plugin_multi will be called.
 */

#include <stdlib.h>
#include "satip.h"

typedef struct
{
	void (*plugin_pids)(void *ctx,SATIP_PIDS *set,int from_notify_hook);
	void *ctx;
	void *filter;
	void *handle;
} DATA;

static __attribute__((constructor)) void plugin_global_init(void)
{
}

static __attribute__((destructor)) void plugin_global_fini(void)
{
}

void *plugin_init(char *config,void *filter_ctx,
	void (*plugin_pids)(void *ctx,SATIP_PIDS *set,int from_notify_hook),
	void *ctx)
{
	DATA *data;

	if(!(data=malloc(sizeof(DATA))))return NULL;
	data->plugin_pids=plugin_pids;
	data->ctx=ctx;
	data->filter=filter_ctx;
	data->handle=NULL;
	return data;
}

void plugin_exit(void *plugin_ctx)
{
	free(plugin_ctx);
}

void plugin_notify_pids(void *plugin_ctx,short *pids,int total)
{
}

void plugin_pre_pids(void *plugin_ctx,SATIP_EXTRA **extra)
{
}

void plugin_post_pids(void *plugin_ctx,int success)
{
}

void plugin_pre_tune(void *plugin_ctx,SATIP_TUNE *tune,SATIP_EXTRA **extra)
{
}

void plugin_post_tune(void *plugin_ctx,SATIP_TUNE *tune,int success)
{
}

void plugin_no_tune(void *plugin_ctx)
{
}

void plugin_strinit(void *plugin_ctx,void **strctx)
{
}

void plugin_strexit(void *plugin_ctx,void *strctx)
{
}

void plugin_stream(void *plugin_ctx,void *strctx,unsigned char *ts188)
{
}

void plugin_multi(void *plugin_ctx,void *strctx,unsigned char *ts188b1,int len1,
	unsigned char *ts188b2,int len2,int *rlen,int flush)
{
	if(len1+len2>=140*188||flush)*rlen=len1+len2;
}

/* 
 * Note: If you do implement CAM handling you should set the 'noshare' option
 * for tuners that do have CAMs, otherwise you will get the first matching
 * plugin context of all of the contexts using the same tuner and you will have
 * to handle this mess.
 */

void plugin_cam(void *plugin_ctx,void *handle,SATIP_HW_CAM_IO *msg)
{
	DATA *data=(DATA *)plugin_ctx;
	char dta[3];

	if(!data->handle)data->handle=handle;

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
		satip_hw_cam_io(handle,msg);
	}
}
