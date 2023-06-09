/* standard includes */
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

/* midas includes */
#include <midas.h>
#include <mfe.h>

/* ZLIB include */
#include <zlib.h>

/* MD5 include */
//#include <md5global.h>
#include <md5.h>

#define BENCHMARK_DEF
#ifdef BENCHMARK_DEF
/* for time benchmarks */
#include <sys/time.h>
#endif


/* experiment specific includes */
#include "fe_compress_z_mt.h"
#include "frontend.h"

/* local variables */
#define ZTHREAD_MAX_THREADS 1
static z_stream strm[ZTHREAD_MAX_THREADS];
static int compression = 1;              /* compression level: 1 -- best speed;  
					                       9 -- best compression; */
#ifdef DEBUG
#define dbprintf(...) printf(__VA_ARGS__)
#else
#define dbprintf(...)
#endif

INT fe_compress_z_init()
{
  INT ret = FE_SUCCESS;
  return ret;
}

INT fe_compress_z_exit()
{
  INT ret = FE_SUCCESS;
  return ret;  
}

INT fe_compress_z_bor(INT run_number __attribute__((unused)), char *error __attribute__((unused)))
{
  int i;
  INT ret = FE_SUCCESS;

  /* allocate deflate state */
  for ( i=0; i<ZTHREAD_MAX_THREADS; i++) 
    {
      strm[i].zalloc = Z_NULL;
      strm[i].zfree  = Z_NULL;
      strm[i].opaque = Z_NULL;

      //if ( deflateInit(&strm, compression) != Z_OK)
      if ( deflateInit2(&strm[i], compression, Z_DEFLATED, 15, 9, Z_HUFFMAN_ONLY) != Z_OK)
	{
	  cm_msg(MERROR, "fe_compress_z_init()", "Cannot initalize ZLIB");
	  ret = FE_ERR_HW;
	}
    }

  return ret;  
}

INT fe_compress_z_eor(INT run_number __attribute__((unused)), char *error __attribute__((unused)))
{
  int i;
  INT ret = FE_SUCCESS;

  for ( i=0; i<ZTHREAD_MAX_THREADS; i++) 
    deflateEnd(&strm[i]);

  return ret;  
}

#ifdef BENCHMARK_DEF
static char *time_print(struct timeval *t1, struct timeval *t2)
{

  long dt_s  = t2->tv_sec  - t1->tv_sec;
  long dt_us = t2->tv_usec - t1->tv_usec;
  if ( dt_us < 0. ) {
    dt_s -= 1;
    dt_us += 1000000;
  }

  /*** time in ms ***/
  int t_ms = dt_s*1000 + dt_us / 1000;

  char *text = new char[64];
  sprintf(text,"%i ms",t_ms);

  return text;

}
#endif


INT fe_compress_z(char *pevent, char *inp, unsigned int size, unsigned int space, int thread)
{
  INT ret = FE_SUCCESS;
  int zlib_ret;

  /****************************************************************
   *              create bank with compressed event               *
   ****************************************************************/

  BYTE  *pdata, *pdata0; 
  char  bkName[8];

  sprintf(bkName,"FZ%02d",get_frontend_index());
  bk_create(pevent, bkName, TID_BYTE, (void**)&pdata);
  dbprintf("%s(%d): compressed bank name %s, uncompressed size %d, available space %d\n", __func__, __LINE__, bkName, size, space );  

  pdata0 = pdata; /* save pointer */

  /****************************************************************
   *                Build MD5 sum of the event                    *
   ****************************************************************/
#define MD5_DIGEST_LEN  16
  MD5_CTX mdContext;
  

  MD5_Init  ( &mdContext );
  MD5_Update( &mdContext, inp, size);
  MD5_Final ( pdata, &mdContext);

  char  md5_digest[MD5_DIGEST_LEN];
  memcpy( md5_digest, pdata, MD5_DIGEST_LEN);
  pdata += MD5_DIGEST_LEN;
  dbprintf("%s(%d): md5 digest string %s md5 digest_length: %d\n", __func__, __LINE__, md5_digest, MD5_DIGEST_LEN);  

  struct timeval t_0, t_1;
  gettimeofday(&t_1,NULL);
  gettimeofday(&t_0,NULL);
  char * p_return = time_print(&t_0,&t_1) ;
  dbprintf("%s(%d): before compression - uncompressed size %d z_compress_time %s\n", __func__, __LINE__, size, p_return);
  delete p_return;

  /****************************************************************
   *                        compress event                        *
   ****************************************************************/
 
  /*** initialize structure strm ***/

  deflateReset(&strm[thread]);

  strm[thread].avail_in  = size;
  strm[thread].next_in   = (unsigned char*) inp;
  strm[thread].avail_out = space - MD5_DIGEST_LEN;
  strm[thread].next_out  = pdata;

  /* Z_FULL_FLUSH: all output is flushed as with Z_SYNC_FLUSH, 
     and the compression state is reset so that decompression 
     can restart from this point if previous compressed data has 
     been damaged or if random access is desired */

  zlib_ret = deflate(&strm[thread], Z_FULL_FLUSH);        /* no bad return value */

  if ( zlib_ret == Z_STREAM_ERROR ) 
    {
      cm_msg(MERROR, "fe_compress_z", "deflate failed");
      ret = FE_ERR_HW;
    }
  else 
    {
      /*** make sure that there were enough space to compress the whole event! ***/
      if ( strm[thread].avail_out == 0 ) 
	{
	  cm_msg(MERROR, "fe_compress_z", "no space to compress event. Compression faild");
	  ret = FE_ERR_HW;
	}
      else
	{
	  pdata = pdata0 + space - strm[thread].avail_out;
	  bk_close(pevent,pdata);
	}
    }

  gettimeofday(&t_1,NULL);
  p_return = time_print(&t_0,&t_1) ;
  dbprintf("%s(%d): after compression - deflated size %d z_compress_time %s\n", __func__, __LINE__, space - strm[thread].avail_out, p_return );
  delete p_return;

  return ret;  
}

INT fe_compress_z2(char *pevent, char *inp, unsigned int size, unsigned int space, int thread, char* zname)
{
  INT ret = FE_SUCCESS;
  int zlib_ret;

  /****************************************************************
   *              create bank with compressed event               *
   ****************************************************************/

  BYTE  *pdata, *pdata0; 

  bk_create(pevent, zname, TID_BYTE, (void **)&pdata);
  dbprintf("%s(%d): compressed bank name %s, uncompressed size %d, available space %d\n", __func__, __LINE__, zname, size, space );  
  
  pdata0 = pdata; /* save pointer */
  
  /****************************************************************
   *                Build MD5 sum of the event                    *
   ****************************************************************/
#define MD5_DIGEST_LEN  16
  MD5_CTX mdContext;
  
  MD5_Init  ( &mdContext ); 
  MD5_Update( &mdContext, inp, size); // run over data
  MD5_Final ( pdata, &mdContext); // extract the checksum 

  char  md5_digest[MD5_DIGEST_LEN];
  memcpy( md5_digest, pdata, MD5_DIGEST_LEN);
  pdata += MD5_DIGEST_LEN;

  char md5string[33];
  INT i;
  for(i = 0; i < 16; ++i)
    sprintf(&md5string[i*2], "%02x", (unsigned int)md5_digest[i]);
  printf("%s(%d): computed MD5 string %s length (bytes) %d\n", "decompress", __LINE__, md5string, MD5_DIGEST_LEN);

  struct timeval t_0, t_1;
  gettimeofday(&t_1,NULL);
  gettimeofday(&t_0,NULL);
  char * p_return = time_print(&t_0,&t_1) ;
  dbprintf("%s(%d): before compression - uncompressed size %d z_compress_time %s\n", __func__, __LINE__, size, p_return );
  delete p_return;

  /****************************************************************
   *                        compress event                        *
   ****************************************************************/
 
  /*** initialize structure strm ***/

  deflateReset(&strm[thread]);

  strm[thread].avail_in  = size;
  strm[thread].next_in   = (unsigned char*) inp;
  strm[thread].avail_out = space - MD5_DIGEST_LEN;
  strm[thread].next_out  = pdata;

  /* Z_FULL_FLUSH: all output is flushed as with Z_SYNC_FLUSH, 
     and the compression state is reset so that decompression 
     can restart from this point if previous compressed data has 
     been damaged or if random access is desired */

  zlib_ret = deflate(&strm[thread], Z_FULL_FLUSH);        /* no bad return value */

  if ( zlib_ret == Z_STREAM_ERROR ) 
    {
      cm_msg(MERROR, "fe_compress_z2", "deflate failed");
      ret = FE_ERR_HW;
    }
  else 
    {
      /*** make sure that there were enough space to compress the whole event! ***/
      if ( strm[thread].avail_out == 0 ) 
	{
	  cm_msg(MERROR, "fe_compress_z2", "no space to compress event. Compression faild");
	  ret = FE_ERR_HW;
	}
      else
	{
	  pdata = pdata0 + space - strm[thread].avail_out;
	  bk_close(pevent,pdata);
	}
    }

  gettimeofday(&t_1,NULL);
  p_return = time_print(&t_0,&t_1) ;
  dbprintf("%s(%d): after compression - deflated size %d z_compress_time %s\n", __func__, __LINE__, space - strm[thread].avail_out, p_return );
  delete p_return;

  return ret;  
}

