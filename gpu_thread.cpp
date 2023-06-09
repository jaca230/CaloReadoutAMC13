/**
 * @file    frontends/FakeCalo/gpu_thread.cpp
 * @author  Wes Gohn, Vladimir Tishchenko, Tim Gorringe
 * @date    Fri Nov  4 10:56:30 2011
 * @date    Last-Updated: Tue Oct 16 12:22:30 2018 (-0400)
 *          By: Wes Gohn
 *          Update #: 662
 * @version $Id$
 * 
 * @copyright (c) new (g-2) collaboration
 * 
 * @brief   GPU thread
 * 
 * @details GPU thread details
 *
 * @todo Document this code
 * 
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */

//#include <iostream>

/* Code: */
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <sstream>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <midas.h>
#include <mfe.h>
#ifdef USE_GPU
#include <cuda.h>
#include <cuda_runtime_api.h>
#include "cuda_tools_g2.h"
#endif //USE_GPU
#include "frontend.h"
#include "tcp_thread.h"
#include "amc13_odb.h"
#include "gpu_thread.h"
#include "simulator.h"
#include "timetool.h"
#include <math.h>


#include "FC7.hpp"
#include "uhal/uhal.hpp"
#include "uhal/utilities/xml.hpp"
#include "uhal/log/exception.hpp"
#include "uhal/ProtocolUDP.hpp"

#define TIME_MEASURE_DEF // for GPU time measurements

//#define DEBUG

#ifdef DEBUG
#define dbprintf(...) printf(__VA_ARGS__)
#else
#define dbprintf(...)
#endif

// a hack for now, until we know if the trigger throttling scheme helps anything
int encoder_ccc_slot = 10;
uhal::HwInterface *encoder_fc7;
FC7 *fc7help = new FC7();
bool triggersThrottled = false;

float toddiff(struct timeval*, struct timeval*);
int extractRiderHeader( uint64_t *gpu_data_header_rider, uint64_t *tcp_buf_gl, int gpu_data_header_size); // extract rider headers

GPU_THREAD_INFO gpu_thread_1_info;
static void *gpu_thread_1(void *data);

//Max sizes
int gpu_data_header_rider_size_max = 0x00100000;
int gpu_data_header_amc13_size_max = 0x00100000;
int gpu_data_header_size_max = 0x00100000;
int gpu_data_tail_size_max = 0x00100000;
int gpu_data_raw_size_max = 0x08000000; // 128MB, same as the tcp max
int gpu_data_proc_size_max = 0x01000000; // 16 MB
int gpu_data_his_size_max = 0x002000000;  // 32 MB


int gpu_thread_active = 0;
int gpu_thread_read = 0;
bool BufFullAlarmTriggered = false;
bool GPUBufFullAlarmTriggered = false;
float MaxTCPBufLoad = 0.0;
float MaxGPUBufLoad = 0.0;

pthread_mutex_t mutex_GPU_buf[GPU_BUFFER_SIZE]; /**< Controls access to the global GPU ring buffer */
pthread_mutex_t mutex_GPU_general; //General mutex lock for the GPU thread

//GPU data buffer
std::vector<GPU_Data_t> GPU_Data_Buffer(GPU_BUFFER_SIZE);

unsigned long GPUfillnumber; // GPU fill counter - zeroed at start of run
unsigned long GPUmuonfillnumber; // GPU muon fill counter - zeroed at start of run
unsigned long GPUlaserfillnumber;

// Mod user field masks, etc (mod header word 2)
int ModUserBitOffset = 48;
int ModUserBitField = 0xffff;

// Chan fill-type bit masks, etc (chan header word 2) 
int ChanFTBitOffset = 24;
int ChanFTBitField = 0x7;

// Wfm fill-type bit masks, etc (wfrm header word 1)
int WfrmFTBitOffset = 23;
int WfrmFTBitField = 0x7;

bool Any_processing_on = FALSE;

//extern INT frontend_index;

/** 
 * Called when frontend starts.
 *
 * Creates simulator thread
 * 
 * @return 0 if success
 */

int gpu_thread_init()
{  
  GPUfillnumber = 0; 
  GPUmuonfillnumber = 0; 
  GPUlaserfillnumber = 0;

  dbprintf("%s(%d): GPU init\n", __func__, __LINE__);
  long int total_size = 0;
  for (unsigned int i_buf=0 ; i_buf<GPU_BUFFER_SIZE ; i_buf++)
  {
    GPU_Data_Buffer[i_buf].gpu_data_header_rider = (uint64_t*) malloc( gpu_data_header_rider_size_max );
    GPU_Data_Buffer[i_buf].gpu_data_header_amc13 = (uint64_t*) malloc( gpu_data_header_amc13_size_max );
    GPU_Data_Buffer[i_buf].gpu_data_header = (uint64_t*) malloc( gpu_data_header_size_max );
    GPU_Data_Buffer[i_buf].gpu_data_tail = (uint64_t*) malloc( gpu_data_tail_size_max );
    GPU_Data_Buffer[i_buf].gpu_data_raw = (int16_t*) malloc( gpu_data_raw_size_max );

    total_size += gpu_data_header_rider_size_max;
    total_size += gpu_data_header_amc13_size_max;
    total_size += gpu_data_header_size_max;
    total_size += gpu_data_tail_size_max;
    total_size += gpu_data_raw_size_max;

    // array of pointers to array of buffers
    GPU_Data_Buffer[i_buf].gpu_data_proc = (int16_t**) malloc(TQMETHOD_MAX*sizeof(int16_t*)); 
    GPU_Data_Buffer[i_buf].gpu_data_his = (int**) malloc(TQMETHOD_MAX*sizeof(int*)); 

    for (int i = 0; i < TQMETHOD_MAX; i++){
      if ( tq_parameters_odb[i].TQ_on || tq_parameters_odb[i].store_hist)
      {
	GPU_Data_Buffer[i_buf].gpu_data_proc[i] = (int16_t*) malloc( gpu_data_proc_size_max );  
	total_size += gpu_data_proc_size_max;
	GPU_Data_Buffer[i_buf].gpu_data_his[i] = (int*) malloc( gpu_data_his_size_max );  
	total_size += gpu_data_his_size_max;
      }
    }
  }

  printf("GPU Total buffer size: %ld\n",total_size);

  dbprintf("%s(%d): GPU thread main memory allocated\n", __func__, __LINE__);

  for (unsigned int i = 0; i < GPU_BUFFER_SIZE ; i++) {
    pthread_mutex_init( &mutex_GPU_buf[i], 0 );
  }
  pthread_mutex_init( &mutex_GPU_general, 0 );
  
  gpu_thread_active = 1;
  gpu_thread_read = 0;

  pthread_create( &gpu_thread_1_info.thread_id, NULL, gpu_thread_1, (void *)(&gpu_thread_1_info) );
  dbprintf("%s(%d): GPU thread launched\n", __func__, __LINE__);

  // initialize access to encoder FC7 via uhal -- a bit of a hack for now, since configuration changes in the
  // CCC FC7 slots will require a recompile
  std::string ccc_mch_address;
  if ( amc13_settings_odb.mch_ip_addr[8] == '6' && amc13_settings_odb.mch_ip_addr[9] != '.' ) {
    ccc_mch_address = "192.168.60.15";
  } else {
    ccc_mch_address = "192.168.0.15";
  }
  std::string encoder_fc7_ip = fc7help->getAddress(10, 2, ccc_mch_address, encoder_ccc_slot);
  try {
    std::stringstream uri; uri << "ipbusudp-2.0://" << encoder_fc7_ip << ":50001";
    std::stringstream atf; atf << "file://" << "$GM2DAQ_DIR/address_tables/FC7_CCC.xml";
    encoder_fc7 = new uhal::HwInterface( uhal::ConnectionManager::getDevice("hw_id", uri.str(), atf.str()) );
  } catch (uhal::exception::exception& e) {
    printf("%s(%d): uHAL Exception accessing encodoer FC7: %s \n", __FUNCTION__, __LINE__, e.what());
    return FE_ERR_ODB;
  }

  return 0;
}


// exit the thread function
int gpu_thread_exit()
{
  // join gpu threads
  pthread_mutex_lock( &mutex_GPU_general );
  gpu_thread_active = 0;
  pthread_mutex_unlock( &mutex_GPU_general );
  void * dummy_ret;
  pthread_join(gpu_thread_1_info.thread_id, &dummy_ret);
  
  printf("gpu thread joined...\n");

  //free memory
  for (unsigned int i_buf=0 ; i_buf<GPU_BUFFER_SIZE ; i_buf++)
  {
    free(GPU_Data_Buffer[i_buf].gpu_data_header_rider);
    free(GPU_Data_Buffer[i_buf].gpu_data_header_amc13);
    free(GPU_Data_Buffer[i_buf].gpu_data_header);
    free(GPU_Data_Buffer[i_buf].gpu_data_tail);
    free(GPU_Data_Buffer[i_buf].gpu_data_raw);

    // array of pointers to array of buffers
    for (int i = 0; i < TQMETHOD_MAX; i++){
      if ( tq_parameters_odb[i].TQ_on || tq_parameters_odb[i].store_hist  ) 
      {
	free(GPU_Data_Buffer[i_buf].gpu_data_proc[i]);
	free(GPU_Data_Buffer[i_buf].gpu_data_his[i]);
      }
    }

    free(GPU_Data_Buffer[i_buf].gpu_data_proc);
    free(GPU_Data_Buffer[i_buf].gpu_data_his);
  }

  printf("gpu buffers freed...\n");
  return 0;
}

/*-- gpu_bor(void) -------------------------------------------------*/

/**
 * gpu_bor(void)
 *
 * initialize gpu fill number
 *                                                                                                               
 * @return 0 if success
 */

int gpu_bor(void)
{
#ifdef USE_GPU
  cudaError_t dev_set = cudaSetDevice( amc13_settings_odb.gpu_dev_id );
  if ( dev_set != cudaSuccess )
    {
      printf("ERROR: (gpu_bor) acquiring CUDA device\n");
      return -1;
    }
#endif //USE_GPU

  Any_processing_on = FALSE; // is any TQ processing or histogram processing switched on?
  for (int itq = 0; itq < TQMETHOD_MAX; itq++){
    if ( tq_parameters_odb[itq].TQ_on  ||  tq_parameters_odb[itq].store_hist  ) Any_processing_on = TRUE;
  }
  
  pthread_mutex_lock( &mutex_GPU_general );
  GPUfillnumber = 0; // reset GPU fill number each run
  GPUmuonfillnumber = 0; // reset GPU muon fill number each run
  GPUlaserfillnumber= 0;
  pthread_mutex_unlock( &mutex_GPU_general );

  // reset the buffer alarm trigger flag
  BufFullAlarmTriggered = false;
  GPUBufFullAlarmTriggered = false;
  MaxTCPBufLoad = 0.0;
  MaxGPUBufLoad = 0.0;
  
  // clear the trigger throttled bit, to be on the safe side
  int frontend_index = get_frontend_index();
  fc7help->setThrottleTriggers( encoder_fc7, frontend_index, 0);
  triggersThrottled = false;


#ifdef USE_GPU

  cudaError_t cudaCopyStatus;

  //Loop over all element in the buffer
  for (unsigned int iBuffer = 0 ; iBuffer < GPU_BUFFER_SIZE ; iBuffer++)
  {
    // compute the histogram size and memory offset for data arrays for multiple TQ-methods
    int delta_his_offset = 0; 
    for (int itq = 0; itq < TQMETHOD_MAX; itq++){
      if ( tq_parameters_odb[itq].store_hist ) {  

	int n_dtctrs = tq_parameters_odb[itq].gpu_n_segments_x * tq_parameters_odb[itq].gpu_n_segments_y;
	int n_smpls =(tq_parameters_odb[itq].last_sample_in_hist - tq_parameters_odb[itq].first_sample_in_hist + 1 ); 
	int rbn0 = tq_parameters_odb[itq].time_divide_hist;
	int n_intrvls = tq_parameters_odb[itq].rebin_intervals_in_hist;
	int rbn_mltpr = tq_parameters_odb[itq].rebin_increment_in_hist;
	int fill_seq = 8; // numbers of fills in cycle, used if separately histogramming the fill histograms by sequence number

	// before variable rebinning of fill-summed Q-method histo
	// gpu_data_his_size[itq] = sizeof(int) * n_dtctrs * n_smpls / rbn0;
	// printf("before variable rebinning hist data size (bytes) %i\n", sizeof(int) * n_dtctrs * n_smpls / rbn0 );

	// after variable rebinning of fill-summed Q-method histo
	GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq] = 0;
	int isubhist = 0;
	for (isubhist = 0; isubhist < n_intrvls; isubhist++) {
	  int rbn_fctr = rbn0 * (int) pow( (double)rbn_mltpr, (double)isubhist );
	  int wsub_bins = n_smpls / n_intrvls / rbn_fctr;
	  float fsub_bins = (float)n_smpls / (float)n_intrvls / (float)rbn_fctr;
	  GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq] += sizeof(int) * n_dtctrs * wsub_bins; 

	  if ( (double)wsub_bins != fsub_bins ) {
	    cm_msg(MERROR, __FILE__, "non-integer number of bins calculated for q-method sub-histogram\n");
	    dbprintf("isubhist %i, rbn0 %i, rbn_mltpr %i, rbn_fctr %i, n_intrvls %i, n_smpls %i, gpu_data_his_size[itq] %i\n",
		isubhist, rbn0, rbn_mltpr, rbn_fctr, n_intrvls, n_smpls, GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq]);
	  }
	}
	dbprintf("after variable rebinning hist data size (bytes) %i\n", GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq]);

	// if separately histogramming the fill histograms by sequence number we need to multiply the space by fill_seq
	if ( tq_parameters_odb[itq].separate_sequence_hist ) GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq] *= fill_seq;
	dbprintf("after sequence number hist data size (bytes) %i\n", GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq]);

	GPU_Data_Buffer[iBuffer].gpu_data_his_offset[itq] = delta_his_offset;
	delta_his_offset += GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq]; // offset for CUDA memcpy, memset in units of bytes

//	printf("Histogram size %d, max size %d\n", GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq],gpu_data_his_size_max);
	if (GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq] > gpu_data_his_size_max)
	{
	  cm_msg(MERROR, __FILE__, "Requested GPU histogram size %d is higher than the maximum %d\n", GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq],gpu_data_his_size_max );
	  return -1;
	}
	// zero histogram data
	cudaCopyStatus = cudaMemset( gpu_odata+GPU_Data_Buffer[iBuffer].gpu_data_his_offset[itq], 0, GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq]); // size unuts are bytes
	if (cudaCopyStatus !=  cudaSuccess )
	{
	  printf("cudaMemset of histo data FAIL, status: %d error: %s bytes: %d\n",
	      cudaCopyStatus, cudaGetErrorString(cudaCopyStatus), GPU_Data_Buffer[iBuffer].gpu_data_his_size[itq]);
	  if ( cudaCopyStatus == cudaErrorInvalidValue  ) printf("cudaErrorInvalidValue !\n");
	  if ( cudaCopyStatus == cudaErrorInvalidDevicePointer ) printf("cudaErrorInvalidDevicePointer!\n");
	}

      }

    } // fill-summed histograms
  }

  cuda_g2_bor_kernel();

#endif //USE_GPU

  dbprintf("%s(%d): begin-of-run GPU fill number %d, GPU muon fill number %d\n", __func__, __LINE__, GPUfillnumber, GPUmuonfillnumber );

  return 0; 
}

/*-- gpu_eor(void) -------------------------------------------------*/

/**
 * gpu_eor(void)
 *
 * @return 0 if success
 */

int gpu_eor(void)
{
  dbprintf("%s(%d): end-of-run GPU fill number %d\n", __func__, __LINE__, GPUfillnumber );

  return 0; 
}

/**
 * @section  gpu_thread_1 gpu_thread_1
 *
 * @todo document this code
 *
 * @param data pointer to GPU_THREAD_1_INFO structure
 * 
 * @return loops forever, does not return
 */
void *gpu_thread_1(void *data)
{
  struct timeval tstart, tcopy, tprocess, tpoll, tbeforeextract, tafterextract; // for performance testing
#ifdef DEBUG
  uint16_t AMC13fillcounter; // AMC13 (hardware) fill counter
#endif
  int TCPbufferindex; // index to TCP ring buffer 
  int GPUbufferindex; // index to GPU ring buffer

#ifdef USE_GPU
  int cudaInitStatus;
  cudaError_t cudaCopyStatus;
#endif //USE_GPU

#ifdef USE_GPU  
  if ( (cudaInitStatus = cuda_init_g2()) != 0 )
  {
    printf("cuda initialization of device FAILED\n");
  }
#endif //USE_GPU 

  dbprintf("%s(%d): GPU1 thread created \n", __func__, __LINE__ );

  int frontend_index = get_frontend_index();
  //Monitor odb
  std::string MonitorRootKey;
  char fe_index[3];
  sprintf(fe_index, "%02i", frontend_index);
  std::stringstream ss_monitors;
  ss_monitors << "/Equipment/AMC13"<< fe_index << "/Monitors";
  MonitorRootKey = ss_monitors.str();
  
  int ThreadStatus = 1;
  std::string ThreadStatusKey = MonitorRootKey + std::string("/GPU Thread Status");
  db_set_value(hDB, 0, ThreadStatusKey.c_str(), &ThreadStatus, sizeof(ThreadStatus), 1, TID_INT);

  while ( 1 )
  {   
    //Check TCPfillnumber and makesure TCPfillnumber is greater
    unsigned long TCPfillnumber_local;
    unsigned long GPUfillnumber_local; //bor function can change the global fill number
    unsigned long Midasfillnumber_local;
    int local_thread_active = 0;
    int local_thread_read = 0;

    pthread_mutex_lock( &mutex_TCP_general );
    TCPfillnumber_local = TCPfillnumber;
    pthread_mutex_unlock( &mutex_TCP_general );

    pthread_mutex_lock( &mutex_GPU_general );
    GPUfillnumber_local = GPUfillnumber;
    local_thread_active = gpu_thread_active;
    local_thread_read = gpu_thread_read;
    pthread_mutex_unlock( &mutex_GPU_general );

    pthread_mutex_lock(&mutex_midas);
    Midasfillnumber_local = Midasfillnumber;
    pthread_mutex_unlock(&mutex_midas);

    if (!local_thread_active)
    {
      break;
    }

    if (!local_thread_read)
    {
      usleep(100);
      continue;
    }

    if (GPUfillnumber_local == TCPfillnumber_local || TCPfillnumber_local == 0)
    {
      dbprintf("%s(%d): No new events in the TCP buffer \n", __func__, __LINE__ );
      usleep(100);
      continue;
    }

    unsigned long tcp_buffer_filled = 0;
    if (TCPfillnumber_local > GPUfillnumber_local)
    {
      tcp_buffer_filled = TCPfillnumber_local - GPUfillnumber_local;
    }else{
      tcp_buffer_filled = 0xffffffffffffffff - (GPUfillnumber_local - TCPfillnumber_local) +1 ;
    }
    dbprintf("%s(%d): tcp_ring_buffer_size %d \n", __func__, __LINE__, tcp_buffer_filled );

    dbprintf("%s(%d): tcp fill %d gpu fill %d \n", __func__, __LINE__, TCPfillnumber_local , GPUfillnumber_local );

    float BufLoad = tcp_buffer_filled * 1.0 / TCP_BUF_MAX_FILLS;
    float BufLoadThreshold = 0.9;
    if (BufLoad > BufLoadThreshold && !BufFullAlarmTriggered)
    { 
      BufFullAlarmTriggered = true;
      char AlarmMsg[500];
      sprintf(AlarmMsg,"DAQ | AMC13%02d TCP Ring buffer close to full (%f\%)",frontend_index,BufLoad*100);

      int ret_code = al_trigger_alarm("Frontend TCP Buffer Error", AlarmMsg, "Warning", "Frontend TCP Buffer Error", AT_INTERNAL); 
      if (ret_code != AL_SUCCESS) {
	cm_msg(MERROR, __FILE__, "Failure Raising Alarm: Error %d, Alarm \"%s\"", ret_code, "Frontend TCP Buffer Error"     );
      }
    }
    if (BufLoad < BufLoadThreshold && BufFullAlarmTriggered)
    { 
      BufFullAlarmTriggered = false;
      char AlarmMsg[500];
      sprintf(AlarmMsg,"DAQ | AMC13%02d TCP Ring buffer returns normal (%f\%)",frontend_index,BufLoad*100);

      int ret_code = al_trigger_alarm("Frontend TCP Buffer Recovery", AlarmMsg, "Recovery", "Frontend TCP Buffer Recovery", AT_INTERNAL); 
      if (ret_code != AL_SUCCESS) {
	cm_msg(MERROR, __FILE__, "Failure Raising Alarm: Error %d, Alarm \"%s\"", ret_code, "Frontend TCP Buffer Recovery"     );
      }
    }

    unsigned long gpu_buffer_filled = 0;
    if (GPUfillnumber_local > Midasfillnumber_local)
    {
      gpu_buffer_filled = GPUfillnumber_local - Midasfillnumber_local;
    }else{
      gpu_buffer_filled = 0xffffffffffffffff - (Midasfillnumber_local - GPUfillnumber_local) +1 ;
    }
    dbprintf("%s(%d): gpu_ring_buffer_size %d \n", __func__, __LINE__, gpu_buffer_filled );

    dbprintf("%s(%d): gpu fill %d midas fill %d \n", __func__, __LINE__, GPUfillnumber_local , Midasfillnumber_local );

    float GPUBufLoad = gpu_buffer_filled * 1.0 / GPU_BUFFER_SIZE;
    float GPUBufLoadThreshold = 0.9;
    if (GPUBufLoad > GPUBufLoadThreshold && !GPUBufFullAlarmTriggered)
    { 
      GPUBufFullAlarmTriggered = true;
      char AlarmMsg[500];
      sprintf(AlarmMsg,"DAQ | AMC13%02d GPU Ring buffer close to full (%f\%)",frontend_index,GPUBufLoad*100);

      int ret_code = al_trigger_alarm("Frontend GPU Buffer Error", AlarmMsg, "Warning", "Frontend GPU Buffer Error", AT_INTERNAL); 
      if (ret_code != AL_SUCCESS) {
	cm_msg(MERROR, __FILE__, "Failure Raising Alarm: Error %d, Alarm \"%s\"", ret_code, "Frontend GPU Buffer Error"     );
      }
    }
    if (GPUBufLoad < GPUBufLoadThreshold && GPUBufFullAlarmTriggered)
    { 
      GPUBufFullAlarmTriggered = false;
      char AlarmMsg[500];
      sprintf(AlarmMsg,"DAQ | AMC13%02d GPU Ring buffer returns normal (%f\%)",frontend_index,GPUBufLoad*100);

      int ret_code = al_trigger_alarm("Frontend GPU Buffer Recovery", AlarmMsg, "Recovery", "Frontend  GPU Buffer Recovery", AT_INTERNAL); 
      if (ret_code != AL_SUCCESS) {
	cm_msg(MERROR, __FILE__, "Failure Raising Alarm: Error %d, Alarm \"%s\"", ret_code, "Frontend GPU Buffer Recovery"     );
      }
    }

    //Do not proceed if the GPU buffer is full
    if ( (gpu_buffer_filled >= GPU_BUFFER_SIZE - 1) || (tcp_buffer_filled >= TCP_BUF_MAX_FILLS - 1) )
    {
      fc7help->setThrottleTriggers( encoder_fc7, frontend_index, 1);
      triggersThrottled = true;
      continue;
    } else if ( triggersThrottled ) {
      fc7help->setThrottleTriggers( encoder_fc7, frontend_index, 0);
      triggersThrottled = false;
    }

    // calculate TCP ring buffer index from GPU fill number
    TCPbufferindex = GPUfillnumber_local%TCP_BUF_MAX_FILLS;
    dbprintf("%s(%d): start new fill %d, buffer %d\n", __func__, __LINE__, GPUfillnumber_local, TCPbufferindex );

    // calculate the GPU ring buffer index 
    GPUbufferindex = GPUfillnumber_local % GPU_BUFFER_SIZE;

    //Lock GPU buffer unit
    pthread_mutex_lock( &mutex_GPU_buf[GPUbufferindex] );
    dbprintf("%s(%d): got lock to write to GPU buffers %d, \n", 
	__func__, __LINE__, GPUbufferindex  );

    // get start time for GPU thread processing
    gettimeofday( &tstart, NULL);
    trigger_info.time_gputhread_started_s = tstart.tv_sec; 
    trigger_info.time_gputhread_started_us = tstart.tv_usec; 
    //These has to be done after the memory copy
    //TODO: Check DATA 
    //GPU_Data_Buffer[GPUbufferindex].gpu_data_header[7] = tstart.tv_sec; 
    //GPU_Data_Buffer[GPUbufferindex].gpu_data_header[8] = tstart.tv_usec; 

    // use lock to access the tcp_thread buffers - tcp_buf_gl[i], tcp_buf_header_gl[i], tcp_buf_tail_gl[i]
    pthread_mutex_lock( &mutex_TCP_buf[TCPbufferindex] );
    dbprintf("%s(%d): got lock to read from TCP output buffers, *tcp_buf_header_gl[%d] = 0x%08x\n", 
	__func__, __LINE__, TCPbufferindex, be32toh ( *tcp_buf_header_gl[TCPbufferindex] )  );

    // get AMC13 event index from data header ( ugly fix for 64-bit AMC words )
#ifdef DEBUG
    AMC13fillcounter = ( be32toh ( *tcp_buf_header_gl[TCPbufferindex] ) & 0x00FFFFFF ); 
#endif

#ifdef USE_GPU 
#ifdef TIME_MEASURE_DEF 
    cudaEvent_t start, stop;
    float elapsedTime;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start, 0);
#endif // USE_GPU
#endif // TIME_MEASURE_DEF

    dbprintf("%s(%d): got lock to write to GPU output buffers, fill %d\n", __func__, __LINE__, GPUfillnumber_local);

    // set GPU_thread data sizes from TCP_thread data sizes and ODB parameters 
    GPU_Data_Buffer[GPUbufferindex].gpu_data_header_amc13_size = TCPtotalamc13infosize[TCPbufferindex]; // AMC13 headers / trailers
    GPU_Data_Buffer[GPUbufferindex].gpu_data_header_size = TCPtotalheadersize[TCPbufferindex]; // timing / performance data
    GPU_Data_Buffer[GPUbufferindex].gpu_data_tail_size = TCPtotaltailsize[TCPbufferindex]; // CDF 64-bit trailer word
    GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size = TCPtotaldatasize[TCPbufferindex]; // raw, unpacked AMC payload

    // copy header, trailer amc13info for every fill
    memcpy( GPU_Data_Buffer[GPUbufferindex].gpu_data_header, tcp_buf_header_gl[TCPbufferindex], GPU_Data_Buffer[GPUbufferindex].gpu_data_header_size );
    //Add the GPU processing start time stamp
    GPU_Data_Buffer[GPUbufferindex].gpu_data_header[7] = tstart.tv_sec; 
    GPU_Data_Buffer[GPUbufferindex].gpu_data_header[8] = tstart.tv_usec; 

    dbprintf("%s(%d): copied header databank  [size=0x%08x], header[0] 0x%08x, readout fill number %d, GPU fill number %d\n", 
	__func__, __LINE__, GPU_Data_Buffer[GPUbufferindex].gpu_data_header_size, be32toh(GPU_Data_Buffer[GPUbufferindex].gpu_data_header[0]), AMC13fillcounter, GPUfillnumber_local );
    memcpy( GPU_Data_Buffer[GPUbufferindex].gpu_data_tail, tcp_buf_tail_gl[TCPbufferindex], TCPtotaltailsize[TCPbufferindex] );
    dbprintf("%s(%d): copied tail databank  [size=0x%08x], tail[0] 0x%08x, readout fill number %d, GPU fill number %d\n", 
	__func__, __LINE__, GPU_Data_Buffer[GPUbufferindex].gpu_data_tail_size, be32toh(GPU_Data_Buffer[GPUbufferindex].gpu_data_tail[0]), AMC13fillcounter, GPUfillnumber_local );
    memcpy( GPU_Data_Buffer[GPUbufferindex].gpu_data_header_amc13, tcp_buf_amc13_gl[TCPbufferindex], TCPtotalamc13infosize[TCPbufferindex] );
    dbprintf("%s(%d): copied amc13 databank  [size=0x%08x], amc13[0] 0x%08x, readout fill number %d, GPU fill number %d\n", 
	__func__, __LINE__, GPU_Data_Buffer[GPUbufferindex].gpu_data_header_amc13_size, be32toh(GPU_Data_Buffer[GPUbufferindex].gpu_data_header_amc13[0]), AMC13fillcounter, GPUfillnumber_local );

    //  extract / copy rider header / trailer data from raw payload to rider header / trailer array  (call arguments mirror memcpy)
    gettimeofday( &tbeforeextract, NULL);

    GPU_Data_Buffer[GPUbufferindex].gpu_data_header_rider_size = extractRiderHeader( GPU_Data_Buffer[GPUbufferindex].gpu_data_header_rider, tcp_buf_gl[TCPbufferindex], GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size ); 
    dbprintf("%s(%d): copied rider databank[%d], rider[first] 0x%16lx, rider[last] 0x%16lx, readout fill number %d, GPU fill number %d\n",
	__func__, __LINE__, GPU_Data_Buffer[GPUbufferindex].gpu_data_header_rider_size, *(GPU_Data_Buffer[GPUbufferindex].gpu_data_header_rider),
	*(GPU_Data_Buffer[GPUbufferindex].gpu_data_header_rider+(GPU_Data_Buffer[GPUbufferindex].gpu_data_header_rider_size/sizeof(uint64_t))-1), AMC13fillcounter, GPUfillnumber_local );

    gettimeofday( &tafterextract, NULL);
    dbprintf("%s(%d): duration of extract and copy of rider headers, fdt = %e us \n", 
	__func__, __LINE__, toddiff( &tafterextract, &tbeforeextract) );

    // extract the FillType etc from rider header / trailers words 
    int indexModHeaderWord2 = 1; // using module header word
    u_int64_t ModHeader2 = be64toh ( GPU_Data_Buffer[GPUbufferindex].gpu_data_header_rider[indexModHeaderWord2] );
    u_int64_t ModUserBitMask = ModUserBitField << ModUserBitOffset;
    int UserField = ( ( ModHeader2 & ModUserBitMask ) >> ModUserBitOffset  ); // from Rider User Manual, June 17 2015
    int ModFillType = UserField & 0x7;  
    dbprintf("%s(%d): 64-bit Mod header word 0x%016lx after be64toh 0x%016lx and ModFillType 0x%04x\n",
	__func__, __LINE__, GPU_Data_Buffer[GPUbufferindex].gpu_data_header_rider[indexModHeaderWord2], ModHeader2, ModFillType);

    /*
    // 8/14/2017, TG, skip the identification of the fill length from the channel headers. This won't work
    // for async WFD5s with muon/laser fills and sync WFD5s with async fills.. The calculated variables 
    // ChanFillType and WfrmFillType were only used to verify the fill type extracted from the module header

    int indexChanHeaderWord2 = 3; // using channel header word
    u_int64_t ChanHeader2 = be64toh ( gpu_data_header_rider[indexChanHeaderWord2] );
    u_int64_t ChanFTBitMask = ChanFTBitField << ChanFTBitOffset;
    int ChanFillType = ( ( ChanHeader2 & ChanFTBitMask ) >> ChanFTBitOffset  ); // from Rider User Manual, June 17 2015
    dbprintf("%s(%d): 64-bit Chan header word 0x%016lx after be64toh 0x%016lx and chan fill type 0x%04x\n",
    __func__, __LINE__, gpu_data_header_rider[indexChanHeaderWord2], ChanHeader2, ChanFillType);

    int indexWfrmHeaderWord1 = 4; // using waveform header word
    u_int64_t WfrmHeader1 = be64toh ( gpu_data_header_rider[indexWfrmHeaderWord1] );
    u_int64_t WfrmFTBitMask = WfrmFTBitField << WfrmFTBitOffset;
    int WfrmFillType = ( ( WfrmHeader1 & WfrmFTBitMask ) >> WfrmFTBitOffset  ); // from Rider User Manual, June 17 2015
    dbprintf("%s(%d): 64-bit Wfrm header word 0x%016lx after be64toh 0x%016lx and wfrm fill type 0x%04x\n",
    __func__, __LINE__, gpu_data_header_rider[indexWfrmHeaderWord1], WfrmHeader1, WfrmFillType);
     */

    bool process_laser = false;
    for(int ii=0;ii<4;ii++){
      if(tq_parameters_odb[ii].fill_type==2) process_laser=true;
    }
    // copy raw data for pre-scaled muon fills or always of laser/pededstal type fill 
    //if ( ModFillType>1 || ( amc13_settings_odb.store_raw && !((AMC13fillcounter-1)%amc13_settings_odb.prescale_raw) ) )

    //printf("ModFillType = %i, amc13_settings_odb.store_raw = %i, GPUmuonfillnumber = %i\n",ModFillType, amc13_settings_odb.store_raw, GPUmuonfillnumber);
    //printf("store_raw = %i, GPUmuonfillnumber = %i, amc13_settings_odb.prescale_raw = %i, check = %i\n",amc13_settings_odb.store_raw,GPUmuonfillnumber,amc13_settings_odb.prescale_raw,!GPUmuonfillnumber%amc13_settings_odb.prescale_raw ); 
    if ( frontend_index==0 || ModFillType>2 || (ModFillType==2 && !process_laser) || ( amc13_settings_odb.store_raw && !GPUmuonfillnumber%amc13_settings_odb.prescale_raw  ) )
    {
      memcpy( GPU_Data_Buffer[GPUbufferindex].gpu_data_raw, tcp_buf_gl[TCPbufferindex], GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size );

      dbprintf("%s(%d): copied raw databank  [size=0x%08x], raw[0] 0x%04x, raw[1] 0x%04x, raw[2] 0x%04x, raw[3] 0x%04x, readout fill number %d, GPU fill number %d, , GPU muon fill number %d\n", 
	  __func__, __LINE__, GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size, *GPU_Data_Buffer[GPUbufferindex].gpu_data_raw, *(GPU_Data_Buffer[GPUbufferindex].gpu_data_raw+1), *(GPU_Data_Buffer[GPUbufferindex].gpu_data_raw+2), *(GPU_Data_Buffer[GPUbufferindex].gpu_data_raw+3), AMC13fillcounter, GPUfillnumber_local, GPUmuonfillnumber );
    }

#ifdef USE_GPU  

    // for muon type fill and any TQ processing switched on copy data to GPU 
    if ( (ModFillType==1 || (ModFillType==2 && process_laser)) && Any_processing_on ) {

      if ( GPU_IBUF_SIZE < GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size )
      {
	printf("%s(%d): fill is too large (%d bytes) for GPU buffer (%d bytes) \n", 
	    __func__, __LINE__, GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size, GPU_IBUF_SIZE );
	GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size = 1;
      }      

      dbprintf("%s(%d): *** GPU input data[0], data[0]: %li %li total size %d\n", 
	  __func__, __LINE__, *(tcp_buf_gl[TCPbufferindex]), *(tcp_buf_gl[TCPbufferindex]), GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size);

      // copy raw AMC payload data to GPU
      cudaCopyStatus = cudaMemcpy( gpu_idata, tcp_buf_gl[TCPbufferindex], GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size,  cudaMemcpyHostToDevice);
      if ( cudaCopyStatus != cudaSuccess )
      {
	printf("cudaMemcpy of input data FAIL, status: %d error: %s bytes: %d\n", cudaCopyStatus, cudaGetErrorString(cudaCopyStatus), GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size);
	if ( cudaCopyStatus == cudaErrorInvalidValue  ) printf("cudaErrorInvalidValue !\n");
	if ( cudaCopyStatus == cudaErrorInvalidDevicePointer ) printf("cudaErrorInvalidDevicePointer!\n");
      }

#ifdef TIME_MEASURE_DEF
      cudaEventRecord(stop, 0);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      dbprintf("%s(%d): copied data from CPU (pntr %p) to GPU (pntr %p), size %d, time %f ms\n",
	  __func__, __LINE__, tcp_buf_gl[TCPbufferindex], gpu_idata, GPU_Data_Buffer[GPUbufferindex].gpu_data_raw_size, elapsedTime);
      cudaEventDestroy(start);
      cudaEventDestroy(stop);
#endif // TIME_MEASURE_DEF	
    } // end cuda copy from host to device (if Any_processing_on is true)

    // get GPU copy time for GPU thread
    gettimeofday( &tcopy, NULL);
    dbprintf("%s(%d): duration of start to copy, fdt = %e us \n", __func__, __LINE__, toddiff( &tstart, &tcopy) );
    trigger_info.time_gputhread_copytogpu_done_s = tcopy.tv_sec;
    trigger_info.time_gputhread_copytogpu_done_us = tcopy.tv_usec;     
    GPU_Data_Buffer[GPUbufferindex].gpu_data_header[9] = tcopy.tv_sec; // fill copy to GPU time info in header
    GPU_Data_Buffer[GPUbufferindex].gpu_data_header[10] = tcopy.tv_usec; // fill copy to GPU time info in header

#endif // USE_GPU 

    // unlocked the access to TCP buffer now all data is copied to GPU buffers
    pthread_mutex_unlock( &mutex_TCP_buf[TCPbufferindex]);
    dbprintf("%s(%d): unlocking ring buffer , buffer %d, fill %d\n",  __func__, __LINE__, TCPbufferindex, GPUfillnumber_local);

#ifdef USE_GPU  

    // for muon type fill and TQ processing switched on launch processing on GPU 
    if ( ModFillType==1 || ModFillType==2) {

      for (int itq = 0; itq < TQMETHOD_MAX; itq++){

	if ( tq_parameters_odb[itq].TQ_on || tq_parameters_odb[itq].store_hist ) {
	  if(tq_parameters_odb[itq].fill_type != ModFillType) continue;

	  cuda_g2_run_kernel( gpu_idata, gpu_odata, GPU_Data_Buffer[GPUbufferindex].gpu_data_proc[itq], itq , GPUbufferindex); // see kernel.cu for gpu proceesing functions

	  // note that copy from device to host of processed data gpu_data_proc and setting of data size gpu_data_proc_size is done 
	  // in  function cuda_g2_run_kernel() whereas the copying and zeroing of histogram data on pre-scaled fills is done here.
	  //if (  tq_parameters_odb[itq].store_hist && !((AMC13fillcounter-1)%tq_parameters_odb[itq].flush_hist) ) 
	  if (  tq_parameters_odb[itq].store_hist && ((GPUmuonfillnumber+1)%tq_parameters_odb[itq].flush_hist)==0 ) {

	    // copy histogram data
	    cudaCopyStatus = cudaMemcpy( GPU_Data_Buffer[GPUbufferindex].gpu_data_his[itq], gpu_odata+GPU_Data_Buffer[GPUbufferindex].gpu_data_his_offset[itq], GPU_Data_Buffer[GPUbufferindex].gpu_data_his_size[itq], cudaMemcpyDeviceToHost); 
	    if (cudaCopyStatus !=  cudaSuccess )
	    {
	      printf("cudaMemcpy of output data FAIL, status: %d error: %s bytes: %d\n",
		  cudaCopyStatus, cudaGetErrorString(cudaCopyStatus), GPU_Data_Buffer[GPUbufferindex].gpu_data_his_size[itq]);
	      if ( cudaCopyStatus == cudaErrorInvalidValue  ) printf("cudaErrorInvalidValue !\n");
	      if ( cudaCopyStatus == cudaErrorInvalidDevicePointer ) printf("cudaErrorInvalidDevicePointer!\n");
	    }

	    dbprintf("%s(%d): TQ=%i, gpu_odata %p, copying / zeroing hist databank [ size=%d, offset=%d], hist[0] 0x%08x, hist[N/8] 0x%08x, hist[N/4] 0x%08x, readout fill number %d, GPU fill number %d, GPU muon fill number %d\n",
		__func__, __LINE__, itq, (gpu_odata+GPU_Data_Buffer[GPUbufferindex].gpu_data_his_offset[itq]), 
		GPU_Data_Buffer[GPUbufferindex].gpu_data_his_size[itq], GPU_Data_Buffer[GPUbufferindex].gpu_data_his_offset[itq], *(GPU_Data_Buffer[GPUbufferindex].gpu_data_his[itq]), *(GPU_Data_Buffer[GPUbufferindex].gpu_data_his[itq]+GPU_Data_Buffer[GPUbufferindex].gpu_data_his_size[itq]/8+1), *(GPU_Data_Buffer[GPUbufferindex].gpu_data_his[itq]+GPU_Data_Buffer[GPUbufferindex].gpu_data_his_size[itq]/4+1), AMC13fillcounter, GPUfillnumber_local, GPUmuonfillnumber );

	    // zero histogram data
	    cudaCopyStatus = cudaMemset( gpu_odata+GPU_Data_Buffer[GPUbufferindex].gpu_data_his_offset[itq], 0, GPU_Data_Buffer[GPUbufferindex].gpu_data_his_size[itq]); // size unuts are bytes
	    if (cudaCopyStatus !=  cudaSuccess )
	    {
	      printf("cudaMemset of histo data FAIL, status: %d error: %s bytes: %d\n",
		  cudaCopyStatus, cudaGetErrorString(cudaCopyStatus), GPU_Data_Buffer[GPUbufferindex].gpu_data_his_size[itq]);
	      if ( cudaCopyStatus == cudaErrorInvalidValue  ) printf("cudaErrorInvalidValue !\n");
	      if ( cudaCopyStatus == cudaErrorInvalidDevicePointer ) printf("cudaErrorInvalidDevicePointer!\n");
	    }


	  } // end flush and zero of histogram data

	} // if TQ processing or histogram processing is switched on

      } // loop over index itq of TQ methods  

    } // if muon fill

    // get GPU run time for GPU thread
    gettimeofday( &tprocess, NULL);
    dbprintf("%s(%d): duration of copy to process, fdt = %e us \n", __func__, __LINE__, toddiff( &tprocess, &tcopy) );
    trigger_info.time_gputhread_finished_s = tprocess.tv_sec;
    trigger_info.time_gputhread_finished_us = tprocess.tv_usec;     
    GPU_Data_Buffer[GPUbufferindex].gpu_data_header[11] = tprocess.tv_sec;
    GPU_Data_Buffer[GPUbufferindex].gpu_data_header[12] = tprocess.tv_usec;

#endif // USE_GPU 

    // get GPU poll variable set time for GPU thread
    gettimeofday( &tpoll, NULL);
    dbprintf("%s(%d): duration of process to poll variable, fdt = %e us \n", 
	__func__, __LINE__, toddiff( &tpoll, &tprocess) );

    GPU_Data_Buffer[GPUbufferindex].gpu_data_header[20] = GPUfillnumber; 
    GPU_Data_Buffer[GPUbufferindex].gpu_data_header[21] = GPUmuonfillnumber; // added so mfe_thread knows the correct muon fill number for flushing CQ, CR banks

    pthread_mutex_unlock( &mutex_GPU_buf[GPUbufferindex] );

    int GPUFill = 0;
    pthread_mutex_lock( &mutex_GPU_general );
    GPUfillnumber++;
    GPUFill  = GPUfillnumber % (0xffffffff/2);
    if (ModFillType == 1) GPUmuonfillnumber++;
    if (ModFillType == 2) GPUlaserfillnumber++;
    pthread_mutex_unlock( &mutex_GPU_general );

    //update odb
    char fe_index[3];
    sprintf(fe_index, "%02i", frontend_index);
    std::stringstream ss_monitors;
    ss_monitors << "/Equipment/AMC13"<< fe_index << "/Monitors/GPU Fill Number";

    db_set_value(hDB, 0, ss_monitors.str().c_str(), &GPUFill, sizeof(GPUFill), 1, TID_INT);

    if (GPUBufLoad > MaxGPUBufLoad || GPUFill == 1)
    {
      MaxGPUBufLoad = GPUBufLoad;
      std::stringstream ss_bufload;
      ss_bufload << "/Equipment/AMC13"<< fe_index << "/Monitors/GPU Buffer Peak";
      db_set_value(hDB, 0, ss_bufload.str().c_str(), &MaxGPUBufLoad, sizeof(MaxGPUBufLoad), 1, TID_FLOAT);
    }

    if (BufLoad > MaxTCPBufLoad || GPUFill == 1)
    {
      MaxTCPBufLoad = BufLoad;
      std::stringstream ss_bufload;
      ss_bufload << "/Equipment/AMC13"<< fe_index << "/Monitors/TCP Buffer Peak";
      db_set_value(hDB, 0, ss_bufload.str().c_str(), &MaxTCPBufLoad, sizeof(MaxTCPBufLoad), 1, TID_FLOAT);
    }

  } // while (1)

  ThreadStatus = 0;
  db_set_value(hDB, 0, ThreadStatusKey.c_str(), &ThreadStatus, sizeof(ThreadStatus), 1, TID_INT);

  //cuda_exit_g2(); Commented out because it is not defined in this scope

  printf("gpu thread returned...\n");

  return data;
}

/*-- int extractRiderHeaders(unit64_t *riderDataPntr, int tcpBufferIndex) ------------------------*/
/* 
* is passed the pointer to gpu_data_header_rider array and index for tcp_buf_gl[TCPbufferindex] array
* @return size of array of headers and trailers
*
* This function is only relevant for single waveform data. For multiwaveform data the raw data is stored
* and extracting the rider header / trailers is irrelevant.                                                                       
*/                                                                                                                                                                                                                                   
#ifdef DEBUG
int extractRiderHeader( uint64_t *riderData, uint64_t *rawData, int rawSize) {
#else
int extractRiderHeader( uint64_t *riderData, uint64_t *rawData, int rawSize __attribute__((unused))) {
#endif

// calculate the array index of first sample of each calo segment
 int NRiderModuleMax = 12; // max rider modules in a uTCA crate
 int NRiderChannelMax = 5;  // max rider channels in a rider module
 int nrmh_words64 = (NRMH_WORDS*sizeof(uint16_t))/sizeof(uint64_t); // number of rider module header words
 int nrch_words64 = (NRCH_WORDS*sizeof(uint16_t))/sizeof(uint64_t); // number of rider channel+single waveform header words
 int nrmt_words64 = (NRMT_WORDS*sizeof(uint16_t))/sizeof(uint64_t); // number of rider module trailer words
 int nrct_words64 = (NRCT_WORDS*sizeof(uint16_t))/sizeof(uint64_t); // number of rider channel+single waveform trailer words
 int bc_to_words64 = 2; // convert burst count to 64_bit AMC13 words
 uint64_t riderSize = 0; // size of rider header / trailers extracted

 // extract Rider header / trailers for ODB enabled modules / channels
 int im = 0, ic = 0;
 uint64_t rawOffset = 0;
 for(im=0;im<NRiderModuleMax;im++){
   if (amc13_rider_odb[im].board.rider_enabled) {
     
     memcpy( riderData+riderSize, rawData+rawOffset, NRMH_WORDS*sizeof(uint16_t) ); // copy Rider module header words
     dbprintf("%s(%d): copied rider module headers rider[0] 0x%16lx, raw[0] 0x%16lx, 16-bit words copied %i, rider size %li raw offset %li, im %i, ic %i\n", 
	      __func__, __LINE__, *(riderData+riderSize), *(rawData+rawOffset), NRMH_WORDS, riderSize, rawOffset, im, ic);

     //u_int64_t ModHeader1 = be64toh( gpu_data_header_rider[0] ); // get first module header word and change endianness
     u_int64_t ModHeader1 = be64toh( *(riderData+riderSize) ); // get first module header word and change endianness
     u_int64_t ModDataLengthBitMask = 0xfffff; // set bitmask to lowest 20-bits for data size
     int ModDataLength =  ModHeader1 & ModDataLengthBitMask; // get data size using bit mask
     dbprintf("WFD5 module %i, data length (64-bit words) %i\n", im+1, ModDataLength);

     riderSize += nrmh_words64;
     rawOffset += nrmh_words64; 

     // Handling of absence of channel headers / trailers for async WFD5s with muon/laser triggers and 
     // sync WFD5s with async triggers, TG 8/9/17. If async either WFD5s with muon/laser trigger or 
     // sync WFD5s with async trigger then skip the copying of channel/wfrm headers/trailers (and data).
     // The absence of channel/wfrm headers trailers is indentified by a data length of 4 in first module header word

     if ( ModDataLength == 4 ) { // skip loop over WFD5 channels if length 4 (i.e. async WFD5s with muon/laser trig or sync WFD5s with async trig

       dbprintf("WFD5 module %i, empty digitizer, data length (64-bit words) %i\n", im+1, ModDataLength);
     } else {
       for(ic=0;ic<NRiderChannelMax;ic++){
	 if (amc13_rider_odb[im].channel[ic].enabled) {
	   
	   memcpy( riderData+riderSize, rawData+rawOffset, NRCH_WORDS*sizeof(uint16_t) ); // copy Rider channel header words
	   dbprintf("%s(%d): copied rider channel headers rider[i] 0x%16lx, raw[i] 0x%16lx, 16-bit words copied %i, rider size %li raw offset %li, im %i, ic %i\n", 
		    __func__, __LINE__, *(riderData+riderSize), *(rawData+rawOffset), NRCH_WORDS, riderSize, rawOffset, im, ic);
	   riderSize += nrch_words64;
	   rawOffset += nrch_words64; 
	   
	   // use board-level burst count for all rider channels in particular Rider module. The burst count units are 8 ADC samples and the
	   // 64-bit AMC13 words contain 4 ADC samples - therefore multiply the burst count by 8/4 =2 to convert to 64bit AMC13 word units
	   rawOffset += (amc13_rider_odb[im].board.trig1__wvfm_length / 8) * bc_to_words64;  
	   
	   memcpy( riderData+riderSize, rawData+rawOffset, NRCT_WORDS*sizeof(uint16_t) ); // copy Rider channel trailer words
	   dbprintf("%s(%d): copied rider channel trailers rider[i] 0x%16lx, raw[i] 0x%16lx, 16-bit words copied %i, rider size %li raw offset %li, im %i, ic %i\n", 
		    __func__, __LINE__, *(riderData+riderSize), *(rawData+rawOffset), NRCT_WORDS, riderSize, rawOffset, im, ic);
	   riderSize += nrct_words64;
	   rawOffset += nrct_words64; 
	 } // WFD5 enabled if
       } // WFD5 channel loop
     } // if / else for empty non-empty WFD5

     memcpy( riderData+riderSize, rawData+rawOffset, NRMT_WORDS*sizeof(uint16_t) ); // copy Rider module trailer words
     dbprintf("%s(%d): copied rider module trailers rider[i] 0x%26lx, raw[i] 0x%16lx, words copied %i, rider size %li raw offset %li, im %i, ic %i\n", 
	      __func__, __LINE__, *(riderData+riderSize), *(rawData+rawOffset), NRMT_WORDS, riderSize, rawOffset, im, ic);
     riderSize += nrmt_words64;
     rawOffset += nrmt_words64; 
     
   }
 }

 dbprintf("%s(%d): making rider databank  [rider size = %li 64bit words, raw size = %i  64bit words], rider[0] 0x%16lx, raw[0] 0x%16lx\n",
	  __func__, __LINE__, riderSize, rawSize, *riderData, *rawData );
 
 return sizeof(uint64_t)*riderSize;

}

/* gpu_thread.c ends here */
