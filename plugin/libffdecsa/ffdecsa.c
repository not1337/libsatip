#include "ffdecsa.h"

int ffdecsa_decrypt_packets(void *ctx,void *keys,unsigned char **cluster)
	__attribute__ ((hot));

extern int ffdecsa_int_select_mode(int mode);
extern int ffdecsa_int_get_internal_parallelism(void);
extern int ffdecsa_int_get_suggested_cluster_size(void);
extern void *ffdecsa_int_get_key_struct(void);
extern void ffdecsa_int_free_key_struct(void *keys);
extern void ffdecsa_int_set_control_words(void *keys,const unsigned char *even,
	const unsigned char *odd);
extern void ffdecsa_int_set_even_control_word(void *keys,
	const unsigned char *even);
extern void ffdecsa_int_set_odd_control_word(void *keys,
	const unsigned char *odd);
extern void ffdecsa_int_get_control_words(void *keys,unsigned char *even,
	unsigned char *odd);
extern int ffdecsa_int_decrypt_packets(void *keys,unsigned char **cluster);

extern int ffdecsa_long_select_mode(int mode);
extern int ffdecsa_long_get_internal_parallelism(void);
extern int ffdecsa_long_get_suggested_cluster_size(void);
extern void *ffdecsa_long_get_key_struct(void);
extern void ffdecsa_long_free_key_struct(void *keys);
extern void ffdecsa_long_set_control_words(void *keys,const unsigned char *even,
	const unsigned char *odd);
extern void ffdecsa_long_set_even_control_word(void *keys,
	const unsigned char *even);
extern void ffdecsa_long_set_odd_control_word(void *keys,
	const unsigned char *odd);
extern void ffdecsa_long_get_control_words(void *keys,unsigned char *even,
	unsigned char *odd);
extern int ffdecsa_long_decrypt_packets(void *keys,unsigned char **cluster);

extern int ffdecsa_mmx_select_mode(int mode);
extern int ffdecsa_mmx_get_internal_parallelism(void);
extern int ffdecsa_mmx_get_suggested_cluster_size(void);
extern void *ffdecsa_mmx_get_key_struct(void);
extern void ffdecsa_mmx_free_key_struct(void *keys);
extern void ffdecsa_mmx_set_control_words(void *keys,const unsigned char *even,
	const unsigned char *odd);
extern void ffdecsa_mmx_set_even_control_word(void *keys,
	const unsigned char *even);
extern void ffdecsa_mmx_set_odd_control_word(void *keys,
	const unsigned char *odd);
extern void ffdecsa_mmx_get_control_words(void *keys,unsigned char *even,
	unsigned char *odd);
extern int ffdecsa_mmx_decrypt_packets(void *keys,unsigned char **cluster);

extern int ffdecsa_sse2_select_mode(int mode);
extern int ffdecsa_sse2_get_internal_parallelism(void);
extern int ffdecsa_sse2_get_suggested_cluster_size(void);
extern void *ffdecsa_sse2_get_key_struct(void);
extern void ffdecsa_sse2_free_key_struct(void *keys);
extern void ffdecsa_sse2_set_control_words(void *keys,const unsigned char *even,
	const unsigned char *odd);
extern void ffdecsa_sse2_set_even_control_word(void *keys,
	const unsigned char *even);
extern void ffdecsa_sse2_set_odd_control_word(void *keys,
	const unsigned char *odd);
extern void ffdecsa_sse2_get_control_words(void *keys,unsigned char *even,
	unsigned char *odd);
extern int ffdecsa_sse2_decrypt_packets(void *keys,unsigned char **cluster);

int ffdecsa_select_mode(void **ctx,int mode)
{
	switch(mode)
	{
	case FFDECSA_32_INT:
	case FFDECSA_64_LONG:
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
	case FFDECSA_128_SSE2:
#endif
		*ctx=(void *)((long)mode);
		return 0;
	default:*ctx=(void *)((long)-1);
		return -1;
	}
}

int ffdecsa_get_internal_parallelism(void *ctx)
{
	switch((long)ctx)
	{
	case FFDECSA_32_INT:
		return ffdecsa_int_get_internal_parallelism();
	case FFDECSA_64_LONG:
		return ffdecsa_long_get_internal_parallelism();
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
		return ffdecsa_mmx_get_internal_parallelism();
	case FFDECSA_128_SSE2:
		return ffdecsa_sse2_get_internal_parallelism();
#endif
	default:return 0;
	}
}
int ffdecsa_get_suggested_cluster_size(void *ctx)
{
	switch((long)ctx)
	{
	case FFDECSA_32_INT:
		return ffdecsa_int_get_suggested_cluster_size();
	case FFDECSA_64_LONG:
		return ffdecsa_long_get_suggested_cluster_size();
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
		return ffdecsa_mmx_get_suggested_cluster_size();
	case FFDECSA_128_SSE2:
		return ffdecsa_sse2_get_suggested_cluster_size();
#endif
	default:return 0;
	}
}

void *ffdecsa_get_key_struct(void *ctx)
{
	switch((long)ctx)
	{
	case FFDECSA_32_INT:
		return ffdecsa_int_get_key_struct();
	case FFDECSA_64_LONG:
		return ffdecsa_long_get_key_struct();
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
		return ffdecsa_mmx_get_key_struct();
	case FFDECSA_128_SSE2:
		return ffdecsa_sse2_get_key_struct();
#endif
	default:return 0L;
	}
}
void ffdecsa_free_key_struct(void *ctx,void *keys)
{
	switch((long)ctx)
	{
	case FFDECSA_32_INT:
		ffdecsa_int_free_key_struct(keys);
		return;
	case FFDECSA_64_LONG:
		ffdecsa_long_free_key_struct(keys);
		return;
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
		ffdecsa_mmx_free_key_struct(keys);
		return;
	case FFDECSA_128_SSE2:
		ffdecsa_sse2_free_key_struct(keys);
		return;
#endif
	}
}

void ffdecsa_set_control_words(void *ctx,void *keys,const unsigned char *even,
	const unsigned char *odd)
{
	switch((long)ctx)
	{
	case FFDECSA_32_INT:
		ffdecsa_int_set_control_words(keys,even,odd);
		return;
	case FFDECSA_64_LONG:
		ffdecsa_long_set_control_words(keys,even,odd);
		return;
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
		ffdecsa_mmx_set_control_words(keys,even,odd);
		return;
	case FFDECSA_128_SSE2:
		ffdecsa_sse2_set_control_words(keys,even,odd);
		return;
#endif
	}
}

void ffdecsa_set_even_control_word(void *ctx,void *keys,
	const unsigned char *even)
{
	switch((long)ctx)
	{
	case FFDECSA_32_INT:
		ffdecsa_int_set_even_control_word(keys,even);
		return;
	case FFDECSA_64_LONG:
		ffdecsa_long_set_even_control_word(keys,even);
		return;
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
		ffdecsa_mmx_set_even_control_word(keys,even);
		return;
	case FFDECSA_128_SSE2:
		ffdecsa_sse2_set_even_control_word(keys,even);
		return;
#endif
	}
}

void ffdecsa_set_odd_control_word(void *ctx,void *keys,const unsigned char *odd)
{
	switch((long)ctx)
	{
	case FFDECSA_32_INT:
		ffdecsa_int_set_odd_control_word(keys,odd);
		return;
	case FFDECSA_64_LONG:
		ffdecsa_long_set_odd_control_word(keys,odd);
		return;
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
		ffdecsa_mmx_set_odd_control_word(keys,odd);
		return;
	case FFDECSA_128_SSE2:
		ffdecsa_sse2_set_odd_control_word(keys,odd);
		return;
#endif
	}
}

void ffdecsa_get_control_words(void *ctx,void *keys,unsigned char *even,
	unsigned char *odd)
{
	switch((long)ctx)
	{
	case FFDECSA_32_INT:
		ffdecsa_int_get_control_words(keys,even,odd);
		return;
	case FFDECSA_64_LONG:
		ffdecsa_long_get_control_words(keys,even,odd);
		return;
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
		ffdecsa_mmx_get_control_words(keys,even,odd);
		return;
	case FFDECSA_128_SSE2:
		ffdecsa_sse2_get_control_words(keys,even,odd);
		return;
#endif
	}
}

int ffdecsa_decrypt_packets(void *ctx,void *keys,unsigned char **cluster)
{
	switch((long)ctx)
	{
	case FFDECSA_32_INT:
		return ffdecsa_int_decrypt_packets(keys,cluster);
	case FFDECSA_64_LONG:
		return ffdecsa_long_decrypt_packets(keys,cluster);
#if defined(__x86_64) || defined(__i386)
	case FFDECSA_64_MMX:
		return ffdecsa_mmx_decrypt_packets(keys,cluster);
	case FFDECSA_128_SSE2:
		return ffdecsa_sse2_decrypt_packets(keys,cluster);
#endif
	default:return 0;
	}
}
