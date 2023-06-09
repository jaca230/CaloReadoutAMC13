/**
 * @file    cuda_tools_g2.h
 * @author  Vladimir Tishchenko, Wes Gohn
 * @date    Mon Oct 24 16:21:54 2011 (-0400)
 * @date    Last-Updated: Tue Oct 16 11:35:16 2018 (-0500)
 *          By: Wes Gohn
 *          Update #: 44
 * @version $Id$
 * 
 * @copyright (c) (g-2) collaboration
 * 
 * @brief   (g-2) CUDA tools
 * 
 * @details auxiliary CUDA functions 
 * 
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */

#ifndef cuda_tools_g2_h
#define cuda_tools_g2_h

//GPU_THREAD_COPYHOST2DEVICE_INFO gpu_thread_copyHost2Device_info;

/**
 * Pointer to GPU memory for input data (device memory)
 */
extern unsigned char *gpu_idata; 

/**
 * Pointer to GPU memory for output data (device memory)
 */
extern unsigned char *gpu_odata;

//static unsigned char *gpu_idata; 
//static unsigned char *gpu_odata;

/**
 * Memory buffer size for input data in GPU (in the device memory)
 */
// 0x08000000 = 128 MB
//#define GPU_IBUF_SIZE       0x08000000
// 0x10000000 = 256 MB
#define GPU_IBUF_SIZE       0x10000000

/**
 * Memory buffer size for output data in GPU (in the device memory)
 */
// 
//#define GPU_OBUF_SIZE       0x08000000
// 0x20000000 = 512 MB
#define GPU_OBUF_SIZE       0x40000000
// 0x20000000 = 32 MB
//#define GPU_OBUF_SIZE       0x02000000


int cuda_init_g2(); 
int cuda_exit_g2(); 
void cuda_g2_bor_kernel(); 
//void cuda_g2_run_kernel( unsigned char *gpu_idata, unsigned char *gpu_odata, int16_t *cpu_data );
void cuda_g2_run_kernel( unsigned char *gpu_idata, unsigned char *gpu_odata, int16_t *cpu_data, int itq ,int GPUbufferindex);
//unsigned int cuda_g2_fetch_waveforms(unsigned char *gpu_odata, unsigned char *buf);
//unsigned int cuda_g2_fetch_islands  (unsigned char *gpu_odata, unsigned char *buf);
void __cudaCheckError(const char *file, const int line );


#endif /* gpu_thread_h defined */
/* cuda_tools_g2.h ends here */
