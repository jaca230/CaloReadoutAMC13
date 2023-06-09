/**
 * @file    frontends/FakaCalo/gpu_thread.h
 * @author  Vladimir Tishchenko <tishenko@pa.uky.edu>, modified by Tim Gorringe
 * @date    Mon Oct 24 09:17:25 2011 (-0400) 
 * @date    Last-Updated: Tue Feb 21 08:21:19 2017 (-0600)
 *          By : Data Acquisition
 *          Update #: 79 
 * @version $Id$
 * 
 * @copyright (c) new (g-2) collaboration
 * 
 * @brief   header file for gpu_thread
 * 
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */

/* Code: */
#ifndef gpu_thread_h
#define gpu_thread_h

#include "tqmethods_max.h"
#include <vector>

typedef struct s_gpu_thread 
{
  pthread_t           thread_id;    /**< ID returned by pthread_create() */
  pthread_mutex_t     mutex;        /**< mutex */
} GPU_THREAD_INFO;

// limit of detector per calo for GPU processing
#define N_DETECTORS_MAX    60

// ADC type                                                                                                                    
#define ADC_TYPE       int16_t
#define ADC_MAX        2048

#define NRMH_WORDS 12  // 3 64-bit rider module header words (changed from 8 to add the 64-bit word with WFD5 FPGA version number
#define NRMT_WORDS 4  // 1 64-bit rider module trailer words
#define NRCH_WORDS 16  // 4 64-bit rider channel + waveform header words for muon fill
#define NRCT_WORDS 12  // 3 64-bit rider channel + waveform trailer words for muon fill

#define GPU_BUFFER_SIZE 64

extern GPU_THREAD_INFO gpu_thread_1_info;

extern pthread_mutex_t mutex_GPU_buf[GPU_BUFFER_SIZE]; /**< Controls access to the GPU ring buffer */
extern pthread_mutex_t mutex_GPU_general;

extern int gpu_thread_active;
extern int gpu_thread_read;

int gpu_thread_init();
int gpu_thread_exit();
int gpu_eor();
int gpu_bor();

//Data max sizes
extern int gpu_data_header_rider_size_max;
extern int gpu_data_header_amc13_size_max;
extern int gpu_data_header_size_max;
extern int gpu_data_tail_size_max;
extern int gpu_data_raw_size_max;
extern int gpu_data_proc_size_max;
extern int gpu_data_his_size_max;

//Data struct
struct GPU_Data_t{
  // for rider header / trailer data
   uint64_t *gpu_data_header_rider;
   int gpu_data_header_rider_size;

  // for amc13 header / trailer data
   uint64_t *gpu_data_header_amc13;
   int gpu_data_header_amc13_size;

  // for timing / perforance data
   uint64_t *gpu_data_header;
   int gpu_data_header_size;

  // for final 64-bit CDF trailer word
   uint64_t *gpu_data_tail;
   int gpu_data_tail_size;

  // for raw AMC payload data
   int16_t *gpu_data_raw;
   int gpu_data_raw_size;

  // for single T/Q method data 
  // int16_t *gpu_data_proc;
  // int gpu_data_proc_size;
  // for multiple T/Q method data 
   int16_t **gpu_data_proc;
   int gpu_data_proc_size[TQMETHOD_MAX];

  // for filled-summed histogram data
  // for single T/Q method data 
  // int *gpu_data_his;
  // int gpu_data_his_size;
  // for multiple T/Q method data 
   int **gpu_data_his;
   int gpu_data_his_size[TQMETHOD_MAX];
   int gpu_data_his_offset[TQMETHOD_MAX];

  // used to record the size of fill-by-fill, time-decimated, calo-summed histogram
  // the data is stored in gpu_data_proc (see above)
  // for single T/Q method data 
  // int gpu_data_q_size;
  // for multiple T/Q method data 
   int gpu_data_q_size[TQMETHOD_MAX];
};

//GPU data buffer
extern std::vector<GPU_Data_t> GPU_Data_Buffer;

// GPU fill counter
extern unsigned long GPUfillnumber;
extern unsigned long GPUmuonfillnumber;

//#undef EXTERN
#endif /* gpu_thread_h defined */
/* gpu_thread.h ends here */
