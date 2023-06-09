/**
 * file    cuda_tools_g2.cpp
 * @author  Vladimir Tishchenko, Wes Gohn
 * @date    Mon Oct 24 16:17:46 2011 (-0400)
 * @date    Last-Updated: Tue Oct 15 11:33:24 2018 (-0500)
 *          By: Wes Gohn
 *          Update #: 149
 *          
 * \copyright (c) (g-2) collaboration 
 * 
 * @version $Id$
 * 
 * @brief   CUDA tools
 * 
 * @details various auxiliary tools for GPU
 * 
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */

#include <stdlib.h>
#include <stdio.h>

// includes, CUDA
#include <cuda.h>
#include <cuda_runtime_api.h>
//#include <cutil_inline.h>

#include "cuda_tools_g2.h"

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <midas.h>
#include "gpu_thread.h"
#include <sys/types.h>
#include "frontend.h"
#include "tcp_thread.h"
#include "amc13_odb.h"

// ATF for fitter
#include "gpu_fit.hh"
pulseFinderResultCollection* device_fitresult = NULL;
pulseFinderResultCollection* host_fitresult = NULL;

//Define the global variable defined in the header
unsigned char *gpu_idata = nullptr;
unsigned char *gpu_odata = nullptr;

/** 
 * Calls CUDA functions to
 * - Initialize GPU
 * - allocate memory in GPU for input and output
 * 
 * 
 * @return 0 if success
 */
int cuda_init_g2()
{
 
  // inquire about CUDA devices
  int num_devices, device;
  cudaGetDeviceCount(&num_devices);
  if (num_devices > 1) {
    for (device = 0; device < num_devices; device++) {
      cudaDeviceProp properties;
      cudaGetDeviceProperties(&properties, device);
      printf("device %d properties.multiProcessorCount %d\n", device, properties.multiProcessorCount);
    }                        
  } 

  // select GPU device with highest Gflops/s
  cudaError_t dev_status;
  int dev_id = amc13_settings_odb.gpu_dev_id;
  dev_status = cudaSetDevice( dev_id );
  if (dev_status != cudaSuccess) 
    {
      printf("ERROR acquiring CUDA device\n");
      return -1;
    }
  printf("SUCCESS acquiring CUDA device\n");

  // get some cuda device properties
  cudaDeviceProp prop;
  cudaGetDeviceProperties(&prop, dev_id);
  printf("Device Number: %d\n", dev_id);
  printf("Device name: %s\n", prop.name);
  printf("Memory Clock Rate (KHz): %d\n", prop.memoryClockRate);
  printf("Memory Bus Width (bits): %d\n", prop.memoryBusWidth);

  cudaError_t cudaCopyStatus;

  // allocate device memory for input data
  cudaCopyStatus = cudaMalloc( (void**) &gpu_idata, GPU_IBUF_SIZE);
  if ( cudaCopyStatus != cudaSuccess ) 
    {
      printf("input buffer cudaMalloc FAIL, bytes %d", GPU_IBUF_SIZE);
      if ( cudaCopyStatus == cudaErrorMemoryAllocation ) printf(": cudaErrorMemoryAllocation!\n");
    }
  if ( gpu_idata == NULL ) {
    perror("cannot allocate device memory for input data");
    return 1;
  }

  // allocate device memory for output data
  cudaCopyStatus = cudaMalloc( (void**) &gpu_odata, GPU_OBUF_SIZE);
  if ( cudaCopyStatus != cudaSuccess ) 
   {
     printf("output buffer cudaMalloc FAIL, bytes %d", GPU_OBUF_SIZE);
      if ( cudaCopyStatus == cudaErrorMemoryAllocation ) printf(": cudaErrorMemoryAllocation!\n");
    }
  if ( gpu_odata == NULL ) {
    perror("cannot allocate device memory for output data");
    return 2;
  }

  // ATF : alloc fit result buffers
  // allocate host fit result buffer
  cudaCopyStatus = cudaMallocHost((void**)&host_fitresult, result_size);
  if ( cudaCopyStatus != cudaSuccess ) 
   {
     printf("host fit results buffer cudaMallocHost FAIL, bytes %d", GPU_OBUF_SIZE);
      if ( cudaCopyStatus == cudaErrorMemoryAllocation ) printf(": cudaErrorMemoryAllocation!\n");
    }
  if ( host_fitresult == NULL ) {
    perror("cannot allocate device memory for fit results");
    return 2;
  }

  // allocate device fit result buffer
  cudaCopyStatus = cudaMalloc((void**)&device_fitresult, result_size);
  if ( cudaCopyStatus != cudaSuccess ) 
   {
     printf("fit results buffer cudaMalloc FAIL, bytes %d", GPU_OBUF_SIZE);
      if ( cudaCopyStatus == cudaErrorMemoryAllocation ) printf(": cudaErrorMemoryAllocation!\n");
    }
  if ( device_fitresult == NULL ) {
    perror("cannot allocate device memory for fit results");
    return 2;
  }

  cudaCopyStatus = cudaDeviceSynchronize();
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaDeviceSynchronize() FAILED\n");
    } 

  printf("done allocating memory \n");
  return 0;
}


int cuda_exit_g2()
{
  cudaError_t cudaCopyStatus;
  // free device memory for input data
  cudaCopyStatus = cudaFree( gpu_idata);
  if ( cudaCopyStatus != cudaSuccess ) 
  {
    printf("input buffer cudaFree FAIL");
  }

  // free device memory for output data
  cudaCopyStatus = cudaFree( gpu_odata);
  if ( cudaCopyStatus != cudaSuccess ) 
  {
    printf("output buffer cudaFree FAIL");
  }

  cudaCopyStatus = cudaFreeHost(host_fitresult);
  if ( cudaCopyStatus != cudaSuccess ) 
  {
    printf("host fit results buffer cudaFreeHost FAIL");
  }

  // free device fit result buffer
  cudaCopyStatus = cudaFree(device_fitresult);
  if ( cudaCopyStatus != cudaSuccess ) 
  {
    printf("fit results buffer cudaFree FAIL");
  }

  return 0;
}

void __cudaCheckError( const char *file, const int line )
{
  cudaError err = cudaGetLastError();
  if ( cudaSuccess != err )
    {
      fprintf( stderr, "cudaCheckError() failed at %s:%i : %s\n",
	       file, line, cudaGetErrorString( err ) );
      cm_msg(MERROR, file, "GPU Kernel error!!! Failed at %s:%i",file,line);
      exit( -1 );
    }
  return;
}
