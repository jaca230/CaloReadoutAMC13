/**
 * @file    kernel.cu
 * @author  Tim Gorringe, Wes Gohn, Vladimir Tishchenko
 * @date    Last-Updated: Tue Oct 16 11:08:05 2018 (-0400)
 *          By : Wes Gohn
 *          Update #: 1063
 * @version $Id$
 * @copyright (c) new (g-2) collaboration
 *
 * 
 * 
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <linux/types.h>
//#include "cuPrintf.cu"

//#define DEBUG

#ifdef DEBUG
#define dbprintf(...) printf(__VA_ARGS__)
#else
#define dbprintf(...) 
#endif

// includes, project
#include <cuda.h>
#include "cuda_tools_g2.h"
#include "gpu_thread.h"
#include "midas.h"
#include "amc13_odb.h"

// ATF for template fitting
#include <cassert>
#include <vector>
#include <fstream>
#include <string>
#include "gpu_fit.hh"
#include "math_constants.h"

// N_SAMPLES_MAX (the maximum samples in individual waveform) is used in definition of structures in GPU_HIS_DATA  
// and GPU_AUX_DATA that are mapped to regions of gpu_odata that containing the histogram data and auxiliary data
#define N_SAMPLES_MAX 800000

#define USE_RIDER_FORMAT  1 // = 1 use Rider module/channel header trailers, = 0 dont use
#define N_RIDERCHANS    5 // unused?

// ADC type
#define ADC_TYPE       int16_t
#define ADC_MAX        2048

//cuda error checking
#define CUDA_ERROR_CHECK
#define CudaCheckError()    __cudaCheckError( __FILE__, __LINE__ )

// structure for histogram data
typedef struct s_gpu_his_data {
   int32_t  wf_hist[TQMETHOD_MAX*N_SAMPLES_MAX*N_DETECTORS_MAX];  // fill-summed waveform, 32-bit signed int array of size TQMETHOD_MAX*N_SAMPLES_MAX*N_DETECTORS_MAX
}  GPU_HIS_DATA;

// structure for auxiliary data
typedef struct s_gpu_aux_data {
  double   wf_sum[N_SAMPLES_MAX];        // sum waveform , double array of size N_SAMPLES_MAX
  double   pedestal[N_DETECTORS_MAX];        // calculated pedestal average,  double array of size N_DETECTORS_MAX
  int      island_pattern[N_SAMPLES_MAX*N_DETECTORS_MAX];// auxiliary array for island build
  int      islands_size;                 // total size of the array islands[]
  struct {
    int time;
    int detector;
    int length;
    int offset;
  } island_info[N_SAMPLES_MAX]; // structure array of size N_SAMPLES_MAX
} GPU_AUX_DATA;

// structure for output data
typedef struct s_gpu_out_data {
  int island_offset;                // used to record islands
  int n_islands;                    // number of islands found
  int CTAG ;                        //number of islands>2 GeV && t>50us
  int16_t islands[1];               // array of islands
} GPU_OUT_DATA;

/*
// energy calibration coefficients - for future?
__device__ static double A_calib[N_DETECTORS_MAX] = {
  1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0, 1.0
  };
*/

// host, device parameters for GPU processing
__constant__ int DEV_N_samples[TQMETHOD_MAX]; 
int HOST_N_samples[TQMETHOD_MAX];
__constant__ int DEV_first_chop_sample[TQMETHOD_MAX]; 
int HOST_first_chop_sample[TQMETHOD_MAX];
__constant__ int DEV_last_chop_sample[TQMETHOD_MAX]; 
int HOST_last_chop_sample[TQMETHOD_MAX];
__constant__ int DEV_N_segments_x[TQMETHOD_MAX]; 
int HOST_N_segments_x[TQMETHOD_MAX];
__constant__ int DEV_N_segments_y[TQMETHOD_MAX]; 
int HOST_N_segments_y[TQMETHOD_MAX];
__constant__ int DEV_N_detectors[TQMETHOD_MAX]; 
int HOST_N_detectors[TQMETHOD_MAX];
__constant__ int DEV_rider_index[N_DETECTORS_MAX][TQMETHOD_MAX]; // detector element to electronics channel map
int HOST_rider_index[N_DETECTORS_MAX][TQMETHOD_MAX]; // detector element to electronics channel map
__constant__ int DEV_first_sample_index[N_DETECTORS_MAX][TQMETHOD_MAX]; // array offsets of first sample of each detector
int HOST_first_sample_index[N_DETECTORS_MAX][TQMETHOD_MAX]; // array offsets of first sample of each detector
__constant__ int DEV_N_presamples[TQMETHOD_MAX];
int HOST_N_presamples[TQMETHOD_MAX];
__constant__ int DEV_N_postsamples[TQMETHOD_MAX];
int HOST_N_postsamples[TQMETHOD_MAX];
__constant__ bool DEV_threshold_sign[TQMETHOD_MAX];
bool HOST_threshold_sign[TQMETHOD_MAX];
__constant__ int DEV_threshold[TQMETHOD_MAX];
int HOST_threshold[TQMETHOD_MAX];
__constant__ int DEV_pedestal_option[TQMETHOD_MAX];
int HOST_pedestal_option[TQMETHOD_MAX];
__constant__ int DEV_island_option[TQMETHOD_MAX];
int HOST_island_option[TQMETHOD_MAX];
__constant__ int DEV_global_pedestal[TQMETHOD_MAX];
int HOST_global_pedestal[TQMETHOD_MAX];
__constant__ bool DEV_hpedsubtract[TQMETHOD_MAX];
bool HOST_hpedsubtract[TQMETHOD_MAX];
__constant__ int DEV_hdecimation[TQMETHOD_MAX];
int HOST_hdecimation[TQMETHOD_MAX];
__constant__ int DEV_decimation[TQMETHOD_MAX];
int HOST_hfirstsample[TQMETHOD_MAX];
__constant__ int DEV_hfirstsample[TQMETHOD_MAX];
int HOST_hlastsample[TQMETHOD_MAX];
__constant__ int DEV_hlastsample[TQMETHOD_MAX];
int HOST_hrebinintervals[TQMETHOD_MAX];
__constant__ int DEV_hrebinintervals[TQMETHOD_MAX];
int HOST_hrebinincrement[TQMETHOD_MAX];
__constant__ int DEV_hrebinincrement[TQMETHOD_MAX];
int HOST_decimation[TQMETHOD_MAX];
__constant__ int DEV_hoffset[TQMETHOD_MAX];
int HOST_hoffset[TQMETHOD_MAX];
__constant__ bool DEV_useindividualthresholds[TQMETHOD_MAX];
bool HOST_useindividualthresholds[TQMETHOD_MAX];
__constant__ int DEV_thresholdvalues[N_DETECTORS_MAX][TQMETHOD_MAX];
int HOST_thresholdvalues[N_DETECTORS_MAX][TQMETHOD_MAX];
__constant__ bool DEV_thresholdpolarities[N_DETECTORS_MAX][TQMETHOD_MAX];
bool HOST_thresholdpolarities[N_DETECTORS_MAX][TQMETHOD_MAX];
__constant__ int DEV_nfitislands[TQMETHOD_MAX];
int HOST_nfitislands[TQMETHOD_MAX];
__constant__ int DEV_fit_threshold[TQMETHOD_MAX];
int HOST_fit_threshold[TQMETHOD_MAX];
__constant__ int DEV_minfittime[TQMETHOD_MAX];
int HOST_minfittime[TQMETHOD_MAX];
__constant__ int DEV_ctag_threshold[TQMETHOD_MAX];
int HOST_ctag_threshold[TQMETHOD_MAX];
__constant__ int DEV_ctag_time_cut[TQMETHOD_MAX];
int HOST_ctag_time_cut[TQMETHOD_MAX];

bool HOST_TMask_window[TQMETHOD_MAX];
__constant__ bool DEV_TMask_window[TQMETHOD_MAX];
int HOST_mask_min[TQMETHOD_MAX];
__constant__ int DEV_mask_min[TQMETHOD_MAX];
int HOST_mask_max[TQMETHOD_MAX];
__constant__ int DEV_mask_max[TQMETHOD_MAX];
int HOST_mask_prescale[TQMETHOD_MAX];
__constant__ int DEV_mask_prescale[TQMETHOD_MAX];

bool HOST_save_full_calo[TQMETHOD_MAX];
__constant__ bool DEV_save_full_calo[TQMETHOD_MAX];
int HOST_fill_type[TQMETHOD_MAX];
__constant__ int DEV_fill_type[TQMETHOD_MAX];

int HOST_hsize[TQMETHOD_MAX];
__constant__ int DEV_hsize[TQMETHOD_MAX];
int HOST_fill_seq[TQMETHOD_MAX];
__constant__ int DEV_fill_seq[TQMETHOD_MAX];

bool HOST_save_xtal_border[TQMETHOD_MAX];
__constant__ bool DEV_save_xtal_border[TQMETHOD_MAX];

int HOST_fit_prescale_factor[TQMETHOD_MAX];
__constant__ int DEV_fit_prescale_factor[TQMETHOD_MAX];

/*
seems nuts but last copy of variable from host to device fails so added a dummy variable / copy
*/
__constant__ int DEV_dummy[TQMETHOD_MAX];
int HOST_dummy[TQMETHOD_MAX];

int HOST_firstsampledummy[2], HOST_lastsampledummy[2];

__device__ phaseMap d_phase_maps[N_DETECTORS_MAX];
__device__ pulseTemplate d_templates[N_DETECTORS_MAX];

#if 0
/** 
 * Makes the distribution of ADC samples (fills ADC arrays
 * in GPU_AUX_DATA)
 * Histogramming is a bad task for GPU
 * Need a better solution
 * 
 * @param gpu_idata 
 * @param gpu_odata 
 */
__global__
// kernel_wf_make_ADC is no longer used
void kernel_wf_make_ADC(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
 // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  GPU_OUT_DATA *outdata = (GPU_OUT_DATA*) (auxdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;

  int sum, idet;
  for (idet=0; idet<DEV_N_detectors[itq]; idet++)
    {
      int i = tid + bid*num_threads; 
      while ( i < DEV_N_samples[itq] )
	{
	  ADC_TYPE adc = gpu_idata[idet*DEV_N_samples[itq] + i];
	  atomicAdd( &(data->ADC[idet][adc]), 1);
	  sum += adc;
	  i += blockDim.x * gridDim.x;
	}
    }

}
#endif

#if 0
__global__
void kernel_print_map(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
 // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  GPU_OUT_DATA *outdata = (GPU_OUT_DATA*) (auxdata+1);

  // access thread id
  const unsigned int tidx = threadIdx.x;
  const unsigned int tidy = threadIdx.y;

  /*
  //cuPrintf doesn't work on the Fermilab system

  int idet = tidx+N_SEGMENTS_X*tidy;
  cuPrintf("kernel_print_map: thread.x %d, thread.y %d, module %d, channel %d\n", 
  	 tidx, tidy, SegXYtoRiderModu[tidx][tidy], SegXYtoRiderChan[tidx][tidy]);
  cuPrintf("kernel_print_map: structure nrmh %d, nrmt %d, nrch %d, nrct %d\n", 
  	   RiderParams.nrmhwords, RiderParams.nrmtwords, RiderParams.nrchwords, RiderParams.nrctwords);
  cuPrintf("kernel_print_map: DEV_first_sample_index %d dev_thres %d dev_decimation %d nrmh %d, nrmt %d, nrch %d, nrct %d\n", 
  	   DEV_first_sample_index[idet], DEV_threshold[itq], DEV_decimation[itq], NRMH_WORDS, NRMT_WORDS, NRCH_WORDS, NRCT_WORDS );
  cuPrintf("kernel_print_map: xsegment %d, ysegment %d, idet %d,  pedestal %f, first sample %i, next sample %i\n", 
  	   tidx, tidy, idet, auxdata->pedestal[idet], gpu_idata[DEV_first_sample_index[idet][itq]], gpu_idata[DEV_first_sample_index[idet][itq]+1] );
  */
}
#endif

/** 
 * Make a fill-by-fill sum of waveforms in each detector / segment
 * 
 * @param gpu_idata 
 * @param gpu_odata 
 */
__global__
void kernel_wf_be64tole16(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;

  int sampletimesdetector_nr = 4 * ( tid + bid*num_threads ); 
  if ( sampletimesdetector_nr < DEV_N_samples[itq]*DEV_N_detectors[itq] ) { 
    
    int sample_nr = sampletimesdetector_nr % DEV_N_samples[itq];
    int idet = sampletimesdetector_nr / DEV_N_samples[itq];

    // re-order the bytes within 2-byte words
    unsigned int lobyte, hibyte, four2bytewords[4];
    for (int iByteReorder = 0; iByteReorder  < 4; iByteReorder++ ){
      hibyte = (gpu_idata[DEV_first_sample_index[idet][itq] + sample_nr + iByteReorder] & 0xff00) >> 8;
      lobyte = (gpu_idata[DEV_first_sample_index[idet][itq] + sample_nr + iByteReorder] & 0xff);
      
      four2bytewords[iByteReorder] = lobyte << 8 | hibyte;
    }
    // re-order the 2-byte words within 8-byte words
    for (int iByteReorder = 0; iByteReorder  < 4; iByteReorder++ ){
      gpu_idata[DEV_first_sample_index[idet][itq] + sample_nr + (3 - iByteReorder) ] = four2bytewords[iByteReorder];
    }
  }
}

/** 
 * Make a fill-by-fill sum of waveforms in each detector / segment
 * 
 * @param gpu_idata 
 * @param gpu_odata 
 */
__global__
void kernel_wf_fillsum(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;

  // original index - has problem of serial rebinning and slow function
  int sampletimesdetector_nr = tid + bid*num_threads; 

  // maximum rebinning factor
  int rb = DEV_hdecimation[itq] * pow( (double)DEV_hrebinincrement[itq], (double)(DEV_hrebinintervals[itq] -1) );
  // new index - avoid problem of serial rebinning and slow function
  //int sampletimesdetector_nr = (tid + bid*num_threads)/rb + rb*(tid%rb); 

  if ( sampletimesdetector_nr < DEV_N_samples[itq]*DEV_N_detectors[itq] ){

    int sample_nr = sampletimesdetector_nr % DEV_N_samples[itq];  // for given detector
    int idet = sampletimesdetector_nr / DEV_N_samples[itq];  // detector identifier
    
    // check sample index is between first / last samples in histogram
    if ( sample_nr >= (DEV_hfirstsample[itq] - 1) && sample_nr < DEV_hlastsample[itq] ) {
      
      // using calo map with time decimation and memory offset for multi TQ-methods
      ADC_TYPE adc = gpu_idata[DEV_first_sample_index[idet][itq] + sample_nr];

      // subtract pedestal if desired
      if (DEV_hpedsubtract[itq]) adc = adc - auxdata->pedestal[idet];
     
      // account for first sample / last sample bookending
      int modsampletimesdetector_nr = (sample_nr - DEV_hfirstsample[itq] + 1) + idet * ( DEV_hlastsample[itq] - DEV_hfirstsample[itq] + 1);

      // calculate the number of samples in each rebinning interval
      // e.g. 512 /4 = 128
      int nsamplesperinterval = (  DEV_N_detectors[itq] * ( DEV_hlastsample[itq] - DEV_hfirstsample[itq] + 1) ) / DEV_hrebinintervals[itq];

      // calculate the index of the  sub-hist for the particular sample
      // e.g. 0 / 128 = 0
      int isubhistindex =  modsampletimesdetector_nr / nsamplesperinterval;
 
      // calculate the rebinning factor  of the corresponding sub-hist for the particular sample
      // eg. 2 * 2^0 
      // note pow() only defined in cuda for floating point
      int isubhistrebin = DEV_hdecimation[itq] * pow( (double)DEV_hrebinincrement[itq], (double)isubhistindex );

      // calculate the bin number in the ith sub-hist with rebinning factor (DEV_hdecimation[itq] + i*DEV_rebinintervals[itq]) 
      int isubhistbin = ( modsampletimesdetector_nr % nsamplesperinterval ) / isubhistrebin;

      // calculate the sub-histogram offset within the full, variable bin-width histogram
      int io, isubhistoffset = 0;
      for (io = 0; io < isubhistindex; io++) isubhistoffset += nsamplesperinterval / ( DEV_hdecimation[itq] * pow( (double)DEV_hrebinincrement[itq], (double)io ) );

      // combine sub-hist bin and sub-hist offset with memory offset for multiple TQ methods
      int index = DEV_hoffset[itq] + isubhistbin  + isubhistoffset;


      /*Block below temporary test of logarithmic histograms.
	If we decide to use it, add flag in ODB to toggle on/off
	Nothing was changed except what is inside if statement, so just delete to go back to how it was.
      */
      int log_hist=0;
      if(log_hist){
	int Nbins = (DEV_hlastsample[itq]-DEV_hfirstsample[itq])/DEV_hdecimation[itq];
	float b = log10f(DEV_hlastsample[itq]/DEV_hfirstsample[itq])/(DEV_hlastsample[itq]-DEV_hfirstsample[itq]);
	float a = DEV_hlastsample[itq]/exp10f(b*DEV_hlastsample[itq]);
	float samp_unscaled=sample_nr*(DEV_hlastsample[itq]-DEV_hfirstsample[itq])/Nbins + DEV_hfirstsample[itq];
	float logbin = a*exp10f(b*samp_unscaled);
	index = DEV_hoffset[itq] + idet*logbin;
      }

      // atomicAdd( &(hisdata->wf_hist[sampletimesdetector_nr / DEV_hdecimation[itq]]), adc );
      atomicAdd( &(hisdata->wf_hist[index]), adc );  

      // debug printout
      //printf("kernel_wf_fillsum: TQ %i, sample_nr %i, detector_nr %i,  sampletimesdetector_nr %i, sub-hist rebin %i, sub-hist bin %i, sub-hist index %i, sub-hist offset %i, initial rebin %i, rebin intervals %i, rebin multiplier %i, samples / interval %i\n", 
      //	     itq, sample_nr, idet, modsampletimesdetector_nr, isubhistrebin, isubhistbin, isubhistindex, isubhistoffset, DEV_hdecimation[itq], DEV_hrebinintervals[itq], DEV_hrebinincrement[itq], nsamplesperinterval );
      
    } //if sample index is between first / last samples in histogram
    
  } // protection against exceeding sample array

}

/** 
 * Make a fill-by-fill sum of waveforms in each detector / segment (launched for each bin of rebinned histo).
 * 
 * @param gpu_idata 
 * @param gpu_odata 
 */
__global__
void kernel_wf_fillsum2(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq, int isubhist, int ifillnum)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;

  // This kernel function is launched for each bin of a particular sub-histogram.  

  // sub-histogram bin index (i.e. bin of sub-histogram)
  int isubbin = tid + bid*num_threads; 

  // calculate the number of raw samples in each rebinning interval, 
  // number of detectors * total raw sample range (first sample to last sample) / number of rebin intervals
  int nsamplesperinterval = ( DEV_N_detectors[itq] * ( DEV_hlastsample[itq] - DEV_hfirstsample[itq] + 1) ) / DEV_hrebinintervals[itq];

  // calculate the number of raw samples per individual detector in each rebinning interval
  // total raw sample range (first sample to last sample) / number of rebin intervals
  int nsamplesperintervalperdetector = ( DEV_hlastsample[itq] - DEV_hfirstsample[itq] + 1) / DEV_hrebinintervals[itq];

  // calculate the rebinning factor  of the corresponding sub-hist for the particular sample
  // initial rebin factor * (rebin mulitiplier)^(subhist index)
  int isubhistrebin = DEV_hdecimation[itq] * pow( (double)DEV_hrebinincrement[itq], (double)isubhist );

  // calculate the number of bins for sub-histogram
  // total number of raw samples per sub-histogram / rebinning factor for sub-histogram
  int nsubbins = nsamplesperinterval / isubhistrebin;

  // calculate the number of bins per detector for sub-histogram
  // number of raw samples per individual detector per sub-histogram / rebinning factor for sub-histogram
  int nsubbinsperdetector = nsubbins / DEV_N_detectors[itq];

  // calculate sample offset for ith sub-histogram
  int isubhistoffset = nsubbinsperdetector * isubhist;

  // calculate detector index
  int idet = isubbin / nsubbinsperdetector;  

  if ( isubbin < nsubbins ){

    int adc = 0, firstrawbin = 0, sample_nr;

    // loop over raw sample bins corresponding to  particular sub-histogram bin
    firstrawbin = (( isubbin%nsubbinsperdetector ) + isubhistoffset ) * isubhistrebin;
    for (sample_nr = firstrawbin; sample_nr < firstrawbin + isubhistrebin; sample_nr++){

      // using calo map with time decimation and memory offset for multi TQ-methods
      adc += gpu_idata[ DEV_first_sample_index[idet][itq] + (DEV_hfirstsample[itq]-1) + sample_nr];

      // subtract pedestal if desired
      if (DEV_hpedsubtract[itq]) adc -= auxdata->pedestal[idet];
    }

    int io, ihistoffset = 0;
    int ioff1 = 0, ioff2 = 0;

    // calculate the sub-histogram offset within the full, variable bin-width histogram, with subhist1, subhist2, ... sequence
    //for (io = 0; io < isubhist; io++) ihistoffset += nsamplesperinterval / ( DEV_hdecimation[itq] * pow( (double)DEV_hrebinincrement[itq], (double)io ) );

    // calculate the sub-histogram offset within the full, variable bin-width histogram, with det1, det2, ... sequence
    for (io = 0; io < DEV_hrebinintervals[itq]; io++) ioff1 += nsamplesperintervalperdetector / ( DEV_hdecimation[itq] * pow( (double)DEV_hrebinincrement[itq], (double)io ) );
    for (io = 0; io < isubhist; io++) ioff2 += nsamplesperintervalperdetector / ( DEV_hdecimation[itq] * pow( (double)DEV_hrebinincrement[itq], (double)io ) );

    ihistoffset = idet * ioff1 + ioff2;    

    // combine sub-hist bin and sub-hist offset with memory offset for multiple TQ methods

    int dindex = ihistoffset + isubbin%nsubbinsperdetector;
    int index = DEV_hoffset[itq] + dindex;

    // calculate the fill sequence offset and modify index for filling hisogram
    int iseqnum = ifillnum % DEV_fill_seq[itq]; 
    dindex = ( DEV_hsize[itq] * iseqnum ) / DEV_fill_seq[itq];
    index += dindex;
    //if (isubbin == 1) printf("kernel_wf_fillsum2, adc %f, index %i, dindex %i, ifillnum %i, iseqnum %i, DEV_hsize[itq] %i,  DEV_fill_seq[itq] %i\n", adc, index, dindex, ifillnum, iseqnum, DEV_hsize[itq],  DEV_fill_seq[itq]); 

    // increment histogram array
    hisdata->wf_hist[index] += adc;

    //printf("tid %i, bid %i, num_threads %i, itq %i, isubhist %i, isubbin %i, nsubbins %i, subhistoffset %i, ioff1 %i, ioff2 %i, ihistoffset %i, rebin %i, sample %i, detector %i, dindex %i, adc sum %i\n", 
    //	   tid, bid, num_threads, itq, isubhist, isubbin, nsubbins, isubhistoffset, ioff1, ioff2, ihistoffset, isubhistrebin, firstrawbin, idet, dindex, adc );
  }

}


/** 
 * Make a sum of waveforms and identifiy triggers based on calo sum
 * 
 * @param gpu_idata 
 * @param gpu_odata 
 */
__global__
void kernel_wf_sum(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq) 
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;

  int sample_nr = tid + bid*num_threads; 

  while ( sample_nr < DEV_N_samples[itq] )
    {
      double adc_sum = 0;
      unsigned int idet;
      // calc pedestal-subtracted calo sum
      for (idet=0; idet<DEV_N_detectors[itq]; idet++)
	{

	  ADC_TYPE adc = gpu_idata[DEV_first_sample_index[idet][itq] + sample_nr];
	  adc_sum += (adc - auxdata->pedestal[idet]); 
	}      
      auxdata->wf_sum[sample_nr] = adc_sum;

      // only create a trigger if sample is within the chopping time window of first_chop_sample to last_chop_sample  
      if (sample_nr < DEV_first_chop_sample[itq] || sample_nr > DEV_last_chop_sample[itq] ) return;

      // positive-going global threashold on calo sum
      if ( DEV_island_option[itq]==1 && DEV_threshold_sign[itq] && adc_sum > DEV_threshold[itq] ) 
	{
	  //printf("kernel_wf_sum trigger : %f , sample %d\n", adc_sum, sample_nr);
	  auxdata->island_pattern[sample_nr] = 1;
	}
      // negative-going global threashold on calo sum
      if ( DEV_island_option[itq]==1 && !DEV_threshold_sign[itq] && adc_sum < DEV_threshold[itq] )
	{
	  //printf("kernel_wf_sum trigger : %f , sample %d\n", adc_sum, sample_nr);
	  auxdata->island_pattern[sample_nr] = 1;
	}
      
      // periodic trigger for testing, debugging 
      //const int trigger_period = 50000;
      const int trigger_period = 30000;
      if(DEV_island_option[itq]==0 && sample_nr%trigger_period == 0 )
	{      
	  //printf("kernel_wf_sum trigger : %f , sample %d\n", adc_sum, sample_nr);
	  auxdata->island_pattern[sample_nr] = 1;
	}
      
      sample_nr += blockDim.x * gridDim.x;
    }

}

/**                                                                                                                                                               
 * find pulses in waveform                                                                                                                                 
 *                                                                        
 * @param gpu_idata                                                                                                                                
 * @param gpu_odata                                                                                                                       
 */
__global__
void kernel_wf_fittimes(ADC_TYPE *gpu_idata, pulseFinderResultCollection* resultcol, int itq)
{
  unsigned int tracelen = DEV_N_samples[itq];
  for (uint segment_num = 0; segment_num < DEV_N_detectors[itq]; ++segment_num) {
    // find index based on global id
    uint sample_num = blockIdx.x * blockDim.x + threadIdx.x;

    // don't continue if your trace_index is out of bounds for pulse fitting
    if (sample_num < DEV_minfittime[itq] ||
        sample_num >= tracelen - (SAMPLESPERFIT - PEAKINDEXINFIT - 1)) {
      return;
    }

    ADC_TYPE* trace = gpu_idata + DEV_first_sample_index[segment_num][itq];
//    ADC_TYPE polarity = DEV_thresholdpolarities[DEV_rider_index[segment_num][itq]][itq] ? 1 : -1;
    // change to use global polarity variable. Fang Han.
    ADC_TYPE polarity = DEV_threshold_sign[itq] ? 1 : -1;


    // we need the samples at this point and surrounding (left, middle, right)
    short sample_val = trace[sample_num];
    short m = polarity * sample_val;
    // if this sample is a local minimum and is over threshold, record it
    if (m > polarity * DEV_fit_threshold[itq]) {
      short l = polarity * trace[sample_num - 1];
      short r = polarity * trace[sample_num + 1];

      // must be local minimum, but since we have digital ADC values
      // we must allow for max to equal sample on left, but not on right
      // if we allow max sample to equal right, we will fit same pulse twice
      if ((m >= l) && (m > r)) {
        uint pulse_index = atomicAdd(&resultcol[segment_num].nPulses, 1);
        if (pulse_index < OUTPUTARRAYLEN) {
          // find pulse time, phase
          // first calculate pseudo time
          float numerator = l - m;
          float denominator = r - m;
          // denominator can't be zero because m > r
          float ptime = 2.0 / CUDART_PI_F * atan(numerator / denominator);

          // next interpolate time map table
          float where = ptime * (NPOINTSPHASEMAP - 1);
          int low_index = floor(where);
          float weight_heigh = where - low_index;
          float real_time;
          // check for out of bounds
          if (low_index < 0) {
            real_time = 0;
          } else if (low_index >= NPOINTSPHASEMAP - 1) {
            real_time = 1.0;
          } else {
            // do the interpolation
            real_time =
                d_phase_maps[segment_num].table[low_index] *
                    (1 - weight_heigh) +
                d_phase_maps[segment_num].table[low_index + 1] * weight_heigh;
          }

          // record time, phase, peak index, peak value
          float time_offset = d_phase_maps[segment_num].timeOffset;
          resultcol[segment_num].fit_results[pulse_index].time =
              sample_num + real_time - 0.5 - time_offset;
          resultcol[segment_num].fit_results[pulse_index].phase = 1 - real_time;
          resultcol[segment_num].fit_results[pulse_index].peak_index =
              sample_num;
          resultcol[segment_num].fit_results[pulse_index].peak_value = sample_val;
        } else {
          // beyond limit for number of pulses we're trying to fit
          atomicSub(&resultcol[segment_num].nPulses, 1);
        }
      }
    }
  }
}

/**                                                                                                                                                               
 * fit pulses found in kernel_wf_fittimes                                                                                                                                
 *                                                                        
 * @param gpu_idata                                                                                                                                
 * @param gpu_odata                                                                                                                       
 */
// according to info on nvidia.com, no need to explicitly synchronize threads
// within groups of 32
// because warps are 32 threads and instructions in warp are always synchronous
const int PULSESPERBLOCK = 4;
const int FIT_THREADSPERBLOCK = PULSESPERBLOCK * SAMPLESPERFIT;
#if 0
__global__
void kernel_wf_fitenergies(ADC_TYPE *gpu_idata, pulseFinderResultCollection* resultcol, int itq) {
  // arrays for accumulation
  __shared__ float tSum[FIT_THREADSPERBLOCK];
  __shared__ float dSum[FIT_THREADSPERBLOCK];
  __shared__ float dDotT[FIT_THREADSPERBLOCK];
  __shared__ float tDotT[FIT_THREADSPERBLOCK];

  unsigned int segment_num = blockIdx.y;
  unsigned int pulse_num =
      blockIdx.x * PULSESPERBLOCK + threadIdx.x / SAMPLESPERFIT;

  // return asap if this pulse doesn't exit
  if ((pulse_num >= OUTPUTARRAYLEN) ||
      (pulse_num >= resultcol[segment_num].nPulses)) {
    return;
  }

  // step one: read needed inputs from resultcol
  float phase = resultcol[segment_num].fit_results[pulse_num].phase;
  unsigned int start_sample = resultcol[segment_num].fit_results[pulse_num].peak_index - PEAKINDEXINFIT; 

  ADC_TYPE* fit_trace = gpu_idata + DEV_first_sample_index[segment_num][itq] + start_sample;

  // step two: read in template values for this phase and sample num
  unsigned int sample_index = threadIdx.x % SAMPLESPERFIT;
  float phase_loc = phase * POINTSPERSAMPLE;
  int phase_index = floor(phase_loc);
  float weight_high = phase_loc - phase_index;
  // make sure we're in bounds
  if (phase_index < 0) {
    phase_index = 0;
    weight_high = 0;
  } else if (phase_index >= POINTSPERSAMPLE) {
    phase_index = POINTSPERSAMPLE - 1;
    weight_high = 1;
  }
  unsigned int low_index = phase_index * SAMPLESPERFIT + sample_index;
  unsigned int high_index = low_index + SAMPLESPERFIT;
  float low_value = d_templates[segment_num].table[low_index];
  float high_value = d_templates[segment_num].table[high_index];

  // step 2.5 evaluate template
  float t_i = low_value * (1 - weight_high) + high_value * weight_high;

  // step three : read in pulse value
  float d_i = fit_trace[sample_index];

  // step four: prepare accumulation/reduction arrays
  tSum[threadIdx.x] = t_i;
  dSum[threadIdx.x] = d_i;
  dDotT[threadIdx.x] = d_i * t_i;
  tDotT[threadIdx.x] = t_i * t_i;

  // step five: accumulate, note that explicit synchronization
  // is not required because all accumulation is done within a warp
  // it seems like this stops working if the if and for are inverted, so don't
  // do that
  for (unsigned int stride = 16; stride >= 1; stride /= 2) {
    if (sample_index < 16) {
      tSum[threadIdx.x] += tSum[threadIdx.x + stride];
      dSum[threadIdx.x] += dSum[threadIdx.x + stride];
      dDotT[threadIdx.x] += dDotT[threadIdx.x + stride];
      tDotT[threadIdx.x] += tDotT[threadIdx.x + stride];
    }
  }

  // step six : calculate pedestal, energy
  // read final accumulated results
  int result_index = (threadIdx.x / SAMPLESPERFIT) * SAMPLESPERFIT;
  float tSumFinal = tSum[result_index];
  float dSumFinal = dSum[result_index];
  float dDotTFinal = dDotT[result_index];
  float tDotTFinal = tDotT[result_index];

  float denomRecip = 1.0 / (tSumFinal * tSumFinal - SAMPLESPERFIT * tDotTFinal);

  float energy =
      denomRecip * (dSumFinal * tSumFinal - SAMPLESPERFIT * dDotTFinal);
  float pedestal =
      denomRecip * (dDotTFinal * tSumFinal - dSumFinal * tDotTFinal);

  // step seven: load partial chi^2s into shared memory
  __shared__ float chi2sum[FIT_THREADSPERBLOCK];
  float residual_i = d_i - energy * t_i - pedestal;
  chi2sum[threadIdx.x] = residual_i * residual_i;

  // step eight: accumulate partial chi2s
  for (unsigned int stride = 16; stride >= 1; stride /= 2) {
    if (sample_index < 16) {
      chi2sum[threadIdx.x] += chi2sum[threadIdx.x + stride];
    }
  }
 
  // final step: record results

  // force energy positive
  if (energy < 0) {
    energy = energy * -1;
  }
  //if (sample_index == 0) {
  if(sample_index == PEAKINDEXINFIT){
    resultcol[segment_num].fit_results[pulse_num].energy = d_i;//energy;
    resultcol[segment_num].fit_results[pulse_num].pedestal = t_i;//pedestal;
    resultcol[segment_num].fit_results[pulse_num].chi2 = chi2sum[threadIdx.x];
  }
}
#endif

// A. Fienberg 2020-Dec-09
// bug that caused incorrect energy and pedestal calculations found to be
// caused by thread synchronization, possibly from updated driver/compiler.
// Added explicit thread synchronization bewtween array reduction steps (__syncthreads()).
// Pedestal and energy are correct now.
__global__
void kernel_wf_fitenergies(ADC_TYPE *gpu_idata, pulseFinderResultCollection* resultcol, int itq) {
  // arrays for accumulation
  __shared__ float tSum[FIT_THREADSPERBLOCK];
  __shared__ float dSum[FIT_THREADSPERBLOCK];
  __shared__ float dDotT[FIT_THREADSPERBLOCK];
  __shared__ float tDotT[FIT_THREADSPERBLOCK];
  unsigned int segment_num = blockIdx.y;
  unsigned int pulse_num =
      blockIdx.x * PULSESPERBLOCK + threadIdx.x / SAMPLESPERFIT;
  // step one: read needed inputs from resultcol
  float phase;
  unsigned int start_sample;
  bool valid;
  if ((pulse_num >= OUTPUTARRAYLEN) ||
      (pulse_num >= resultcol[segment_num].nPulses)) {
    phase = 0;
    start_sample = 0;
    valid = false;
  } else {
    phase = resultcol[segment_num].fit_results[pulse_num].phase;
    start_sample = resultcol[segment_num].fit_results[pulse_num].peak_index - PEAKINDEXINFIT;
    valid = true;
  }
  ADC_TYPE* fit_trace = gpu_idata + DEV_first_sample_index[segment_num][itq] + start_sample;
  // step two: read in template values for this phase and sample num
  unsigned int sample_index = threadIdx.x % SAMPLESPERFIT;
  float phase_loc = phase * POINTSPERSAMPLE;
  int phase_index = floor(phase_loc);
  float weight_high = phase_loc - phase_index;
  // make sure we're in bounds
  if (phase_index < 0) {
    phase_index = 0;
    weight_high = 0;
  } else if (phase_index >= POINTSPERSAMPLE) {
    phase_index = POINTSPERSAMPLE - 1;
    weight_high = 1;
  }
  unsigned int low_index = phase_index * SAMPLESPERFIT + sample_index;
  unsigned int high_index = low_index + SAMPLESPERFIT;
  float low_value = d_templates[segment_num].table[low_index];
  float high_value = d_templates[segment_num].table[high_index];
  // step 2.5 evaluate template
  float t_i = low_value * (1 - weight_high) + high_value * weight_high;
  // step three : read in pulse value
  float d_i = fit_trace[sample_index];
  // step four: prepare accumulation/reduction arrays
  tSum[threadIdx.x] = t_i;
  dSum[threadIdx.x] = d_i;
  dDotT[threadIdx.x] = d_i * t_i;
  tDotT[threadIdx.x] = t_i * t_i;
  // step five: accumulate
  __syncthreads();
  for (unsigned int stride = SAMPLESPERFIT/2; stride >= 1; stride /= 2) {
    if (sample_index < stride) {
      tSum[threadIdx.x] += tSum[threadIdx.x + stride];
      dSum[threadIdx.x] += dSum[threadIdx.x + stride];
      dDotT[threadIdx.x] += dDotT[threadIdx.x + stride];
      tDotT[threadIdx.x] += tDotT[threadIdx.x + stride];
    }
    __syncthreads();
  }
  // step six : calculate pedestal, energy
  // read final accumulated results
  int result_index = (threadIdx.x / SAMPLESPERFIT) * SAMPLESPERFIT;
  float tSumFinal = tSum[result_index];
  float dSumFinal = dSum[result_index];
  float dDotTFinal = dDotT[result_index];
  float tDotTFinal = tDotT[result_index];
  float denomRecip = 1.0 / (tSumFinal * tSumFinal - SAMPLESPERFIT * tDotTFinal);
  float energy =
      denomRecip * (dSumFinal * tSumFinal - SAMPLESPERFIT * dDotTFinal);
  float pedestal =
      denomRecip * (dDotTFinal * tSumFinal - dSumFinal * tDotTFinal);
  // step seven: load partial chi^2s into shared memory
  __shared__ float chi2sum[FIT_THREADSPERBLOCK];
  float residual_i = d_i - energy * t_i - pedestal;
  chi2sum[threadIdx.x] = residual_i * residual_i;
  // step eight: accumulate partial chi2s
  __syncthreads();
  for (unsigned int stride = SAMPLESPERFIT/2; stride >= 1; stride /= 2) {
    if (sample_index < stride) {
      chi2sum[threadIdx.x] += chi2sum[threadIdx.x + stride];
    }
    __syncthreads();
  }
  // final step: record results
  // force energy positive
  if (energy < 0) {
    energy = energy * -1;
  }
  if (valid && sample_index == 0) {
    resultcol[segment_num].fit_results[pulse_num].energy = energy;
    resultcol[segment_num].fit_results[pulse_num].pedestal = pedestal;
    resultcol[segment_num].fit_results[pulse_num].chi2 = chi2sum[threadIdx.x];
  }
}


/** 
 * Find the triggers in waveforms
 * 
 * @param gpu_idata 
 * @param gpu_odata 
 */
__global__
void kernel_wf_trigger(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq, bool tmask_prescale)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);

  // access thread id, block id, .. to define the sample index
  const unsigned int tid = threadIdx.x;
  const unsigned int bid = blockIdx.x;
  const unsigned int num_threads = blockDim.x;

  // used for trigger option = 4 tht records trigger island and Tcyc delayed island
  int TcycClockTicks = 120;
 
  int sample_nr = tid + bid*num_threads;

  // only create a trigger if sample is within the chopping time window of first_chop_sample to last_chop_sample  
  if (sample_nr < DEV_first_chop_sample[itq] || sample_nr > DEV_last_chop_sample[itq] ) return;

  // only create a trigger if sample is outside the T-method mask window
  if (tmask_prescale && DEV_TMask_window[itq] && sample_nr > DEV_mask_min[itq] && sample_nr < DEV_mask_max[itq] ) return; 

  while ( sample_nr < DEV_N_samples[itq] )
    {
      
      unsigned int idet;
      // leading edge threshold on individual segments
      if ( DEV_island_option[itq] == 2 || DEV_island_option[itq] == 4 ){
	double adc_cal = 0;
	for (idet=0; idet<DEV_N_detectors[itq]; idet++)
	  {
	    ADC_TYPE adc = gpu_idata[DEV_first_sample_index[idet][itq] + sample_nr];
	    // adc_cal = A_calib[idet]*(adc - auxdata->pedestal[idet]); // with calibration coefficient
	    adc_cal = (adc - auxdata->pedestal[idet]); // w/o calibration coefficient

	    if(adc_cal==0 || abs(adc_cal)>4096) continue; //check rational adc value
	   
	    if(DEV_threshold_sign[itq] && DEV_fill_type[itq]==2 && adc_cal>0) continue; //skip waveform headers for laser triggers (for positive pulses) 

            if ( DEV_useindividualthresholds[itq] ){
	      if ( DEV_thresholdpolarities[ DEV_rider_index[idet][itq] ][itq] && adc_cal > DEV_thresholdvalues[ DEV_rider_index[idet][itq] ][itq] ){
		auxdata->island_pattern[sample_nr] = 1;
                if ( DEV_island_option[itq] == 4 && sample_nr + TcycClockTicks < DEV_N_samples[itq] ) auxdata->island_pattern[ sample_nr + TcycClockTicks ] = 1;
		break;
	      }
	      if (!DEV_thresholdpolarities[ DEV_rider_index[idet][itq] ][itq] && adc_cal < DEV_thresholdvalues[ DEV_rider_index[idet][itq] ][itq] ){
		auxdata->island_pattern[sample_nr] = 1;
                if ( DEV_island_option[itq] == 4 && sample_nr + TcycClockTicks < DEV_N_samples[itq] ) auxdata->island_pattern[ sample_nr + TcycClockTicks ] = 1;
		break;
	      } // end individual thresholds
	    } else { 
	      if ( DEV_threshold_sign[itq] && adc_cal > DEV_threshold[itq] ){
		auxdata->island_pattern[sample_nr] = 1;
                if ( DEV_island_option[itq] == 4 && sample_nr + TcycClockTicks < DEV_N_samples[itq] ) auxdata->island_pattern[ sample_nr + TcycClockTicks ] = 1;
		break;
	      }
	      if ( !DEV_threshold_sign[itq] && adc_cal < DEV_threshold[itq] ){
		auxdata->island_pattern[sample_nr] = 1;
                if ( DEV_island_option[itq] == 4 && sample_nr + TcycClockTicks < DEV_N_samples[itq] ) auxdata->island_pattern[ sample_nr + TcycClockTicks ] = 1;
		break;
	      } 
	    }// end global threshols
	    
	  } // loop over detector
 
      } // island option	
      
      // pulseshape weighted threshold on individual segments
      if ( DEV_island_option[itq] == 3 ){
	
        // hard-coded pulseshape
	const int Nwgt = 7;
	double wgt[Nwgt] = {0.0625, 0.1250, 0.1875, 0.2500, 0.1875, 0.1250, 0.0625};
	int wgtlo = -3, wgthi = 3; 	  
	
	if ( (sample_nr > -wgtlo) && (sample_nr < (DEV_N_samples[itq] - wgthi)) ){  // waveform bookend
	  for (idet=0; idet<DEV_N_detectors[itq]; idet++)
	    {
	      int firstsample = DEV_first_sample_index[idet][itq];

	      //double calconst = A_calib[idet]; // with calibration coefficient
	      double calconst = 1; // w/o calibration coefficient
	      double pedval = auxdata->pedestal[idet];
	      
	      double adcwgt = 0.0;
              int iwgt = 0;
	      for (iwgt = wgtlo; iwgt <= wgthi; iwgt++){
	        ADC_TYPE adc =  gpu_idata[firstsample + sample_nr - iwgt];
		adcwgt += wgt[iwgt - wgtlo] * calconst * (adc - pedval);
	      }

	      if ( DEV_threshold_sign[itq] && adcwgt > DEV_threshold[itq] ){  // positive-going trigger
		auxdata->island_pattern[sample_nr] = 1;
		break;
	      }
	      if ( !DEV_threshold_sign[itq] && adcwgt < DEV_threshold[itq] ){ // negative-going trigger
		auxdata->island_pattern[sample_nr] = 1;
		break;
	      }

	    } // loop over detectors
	} // waveform bookend
      } // island option

      sample_nr += blockDim.x * gridDim.x;
    }
}

__global__
void kernel_extend_islands(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;
 
  int sample_nr = tid + bid*num_threads; 
  while ( sample_nr < DEV_N_samples[itq] )
    {     
      int is_BOI = 0; // beginning of an island
      int is_EOI = 0; // end of an island

      // check the BOI and EOI conditions
      if ( auxdata->island_pattern[sample_nr] > 0 )
	{
	  // check BOI condition
	  if ( sample_nr == 0 ) 
	    is_BOI=1;
	  else
	    if ( auxdata->island_pattern[sample_nr-1] == 0 )
	      is_BOI=1;
	  
	   // check EOI condition
	  if ( sample_nr == (DEV_N_samples[itq]-1) )
	    is_EOI=1;
	  else
	    if ( auxdata->island_pattern[sample_nr+1] == 0 )
	      is_EOI=1;
	}
	
      if ( is_BOI )
	{
	  // This is a beginning of an island
	  // extend the island for N_presamples

	  //printf("kernel_extend_island::: BOI, sample_nr %i, array index \n",  sample_nr, sample_nr); //debug

	  int i1 = sample_nr - DEV_N_presamples[itq];
	  if ( i1 < 0 ) i1 = 0;
	  int k;
	  for (k=i1; k<sample_nr; k++)
	    {
	      atomicAdd( &(auxdata->island_pattern[k]), 1);		  
	    }
	}

      // check the "End Of Island" condition
      if ( is_EOI )
	{
	  // This is an end of an island
	  // extend the island for N_postsamples
	  int i2 = sample_nr + DEV_N_postsamples[itq];
	  if ( i2 >= DEV_N_samples[itq] ) i2 = DEV_N_samples[itq]-1;
	  int k;
	  for (k=i2; k>sample_nr; k--)
	    {
	      atomicAdd( &(auxdata->island_pattern[k]), 1);		  
	    }
	}

      sample_nr += blockDim.x * gridDim.x;
    }
}

__global__
void kernel_find_islands(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  GPU_OUT_DATA *outdata = (GPU_OUT_DATA*) (auxdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;

  //find new islands (some of the original islands could have merged together) 
  int sample_nr = tid + bid*num_threads; 
  while ( sample_nr < DEV_N_samples[itq] )
    {

      int is_BOI = 0; // beginning of an island

      // check the BOI and EOI conditions
      if ( auxdata->island_pattern[sample_nr] > 0 )
	{
	  // check BOI condition
	  if ( sample_nr == 0 ) 
	    is_BOI=1;
	  else
	    if ( auxdata->island_pattern[sample_nr-1] == 0 )
	      is_BOI=1;
	}


      if ( is_BOI )
	{
	  // This is a beginning of an island


	  //printf("kernel_find_island::: BOI, sample_nr %i, array index \n",  sample_nr, sample_nr); //debug
	  
	  // island number
	  int island_nr = atomicAdd( &(outdata->n_islands), 1); 
	  auxdata->island_info[island_nr].time = sample_nr;
	  
	  // determine the length of the island
	  int i;
	  int island_nr_aux = island_nr + 1;
	  for (i=sample_nr; i<DEV_N_samples[itq]; i++)
	    {
	      if ( auxdata->island_pattern[i] == 0 )
		{
		  break;
		}
	      else
		{
		  auxdata->island_pattern[i] = island_nr_aux;
		}
	    }
	  int island_len = i - sample_nr;
	  // record the length into first bin
	  auxdata->island_info[island_nr].length = island_len;

          // the ugly 4 accounts for two 32bit islander header words - the island time and island length
	  int offset = atomicAdd( &(outdata->island_offset), 4 + DEV_N_detectors[itq]*island_len);
	  auxdata->island_info[island_nr].offset = offset;
	  
          memcpy( &outdata->islands[offset], &sample_nr, sizeof(int) ); 
          memcpy( &outdata->islands[offset+2], &island_len, sizeof(int) ); 	  
	}

      sample_nr += blockDim.x * gridDim.x; 
    }
}

__global__
void kernel_make_islands(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  GPU_OUT_DATA *outdata = (GPU_OUT_DATA*) (auxdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;

  //find new islands (some of the old island could have merged   
  int sample_nr = tid + bid*num_threads; 
  while ( sample_nr < DEV_N_samples[itq] )
    {
      int island_nr = auxdata->island_pattern[sample_nr];
      if ( island_nr > 0 )
	{
	  island_nr--;
	  // +4 is needed to skip 32-bit time and length words in 16-bit array
	  int island_offset    = auxdata->island_info[island_nr].offset + 4;  
	  int island_sample_nr = sample_nr - auxdata->island_info[island_nr].time;
	  int island_length    = auxdata->island_info[island_nr].length;
	  int idet;
	  for (idet=0; idet<DEV_N_detectors[itq]; idet++)
	    {
	      int i = island_offset + idet*island_length + island_sample_nr;
	      ADC_TYPE adc=gpu_idata[DEV_first_sample_index[idet][itq] + sample_nr];
	      outdata->islands[i] = adc;
	    }
	}
      sample_nr += blockDim.x * gridDim.x; 
    }
}

/** 
 * Find the triggers in waveforms
 * 
 * @param gpu_idata 
 * @param gpu_odata 
 */
__global__
void kernel_wf_xtaltrigger(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq, bool tmask_prescale)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);

  // access thread id, block id, .. to define the sample index
  const unsigned int tid = threadIdx.x;
  const unsigned int bid = blockIdx.x;
  const unsigned int num_threads = blockDim.x;

  int sample_nr = tid + bid*num_threads;

  // only create a trigger if sample is within the chopping time window of first_chop_sample to last_chop_sample  
  if (sample_nr < DEV_first_chop_sample[itq] || sample_nr > DEV_last_chop_sample[itq] ) return;

  // only create a trigger if sample is outside the T-method mask window
  if (tmask_prescale && DEV_TMask_window[itq] && sample_nr > DEV_mask_min[itq] && sample_nr < DEV_mask_max[itq] ) return; 

  while ( sample_nr < DEV_N_samples[itq] )
    {
      
      unsigned int idet;
      
      // leading edge threshold on individual segments
      if ( DEV_island_option[itq] == 2 ){
	double adc_cal = 0;
	for (idet=0; idet<DEV_N_detectors[itq]; idet++)
	  {
	    ADC_TYPE adc = gpu_idata[DEV_first_sample_index[idet][itq] + sample_nr];
	    // adc_cal = A_calib[idet]*(adc - auxdata->pedestal[idet]); // with calibration coefficient
	    adc_cal = (adc - auxdata->pedestal[idet]); // w/o calibration coefficient

            if ( DEV_useindividualthresholds[itq] ){
	      if ( DEV_thresholdpolarities[ DEV_rider_index[idet][itq] ][itq] && adc_cal > DEV_thresholdvalues[ DEV_rider_index[idet][itq] ][itq] ){
		auxdata->island_pattern[sample_nr + idet*DEV_N_samples[itq]] = 1;
		continue; // need to continue not break when filling a samples * detectors island_pattern array
	      }
	      if (!DEV_thresholdpolarities[ DEV_rider_index[idet][itq] ][itq] && adc_cal < DEV_thresholdvalues[ DEV_rider_index[idet][itq] ][itq] ){
		auxdata->island_pattern[sample_nr + idet*DEV_N_samples[itq]] = 1;
		continue; // need to continue not break when filling a samples * detectors island_pattern array
	      } // end individual thresholds
	    } else { 
	      if ( DEV_threshold_sign[itq] && adc_cal > DEV_threshold[itq] ){
		auxdata->island_pattern[sample_nr + idet*DEV_N_samples[itq]] = 1;
		//printf("kernel_wf_trigger::: sample_nr %i , detector_nr  %i, array index %i\n",  sample_nr, idet, sample_nr + idet*DEV_N_samples[itq]); //debug
		continue; // need to continue not break when filling a samples * detectors island_pattern array
	      }
	      if ( !DEV_threshold_sign[itq] && adc_cal < DEV_threshold[itq] ){
		auxdata->island_pattern[sample_nr + idet*DEV_N_samples[itq]] = 1;
		//printf("kernel_wf_trigger::: sample_nr %i , detector_nr  %i, array index \n",  sample_nr, idet, sample_nr + idet*DEV_N_samples[itq]); //debug
		continue; // need to continue not break when filling a samples * detectors island_pattern array
	      } 
	    }// end global threshols
	    
	  } // loop over detector
 
      } // island option	
      
      // pulseshape weighted threshold on individual segments
      if ( DEV_island_option[itq] == 3 ){
	
        // hard-coded pulseshape
	const int Nwgt = 7;
	double wgt[Nwgt] = {0.0625, 0.1250, 0.1875, 0.2500, 0.1875, 0.1250, 0.0625};
	int wgtlo = -3, wgthi = 3; 	  
	
	if ( (sample_nr > -wgtlo) && (sample_nr < (DEV_N_samples[itq] - wgthi)) ){  // waveform bookend
	  for (idet=0; idet<DEV_N_detectors[itq]; idet++)
	    {
	      int firstsample = DEV_first_sample_index[idet][itq];

	      //double calconst = A_calib[idet]; // with calibration coefficient
	      double calconst = 1; // w/o calibration coefficient
	      double pedval = auxdata->pedestal[idet];
	      
	      double adcwgt = 0.0;
              int iwgt = 0;
	      for (iwgt = wgtlo; iwgt <= wgthi; iwgt++){
	        ADC_TYPE adc =  gpu_idata[firstsample + sample_nr - iwgt];
		adcwgt += wgt[iwgt - wgtlo] * calconst * (adc - pedval);
	      }

	      if ( DEV_threshold_sign[itq] && adcwgt > DEV_threshold[itq] ){  // positive-going trigger
		auxdata->island_pattern[sample_nr + idet*DEV_N_samples[itq]] = 1;
		continue; // need to continue not break when filling a samples * detectors island_pattern array
	      }
	      if ( !DEV_threshold_sign[itq] && adcwgt < DEV_threshold[itq] ){ // negative-going trigger
		auxdata->island_pattern[sample_nr + idet*DEV_N_samples[itq]] = 1;
		continue; // need to continue not break when filling a samples * detectors island_pattern array
	      }

	    } // loop over detectors
	} // waveform bookend
      } // island option

      sample_nr += blockDim.x * gridDim.x;
    }
}


__global__
void kernel_extend_xtalislands(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;
 
  int sampletimesdetector_nr = tid + bid*num_threads; 
  if ( sampletimesdetector_nr < DEV_N_samples[itq]*DEV_N_detectors[itq] ){

    int sample_nr = sampletimesdetector_nr % DEV_N_samples[itq];  // for given detector
    int idet = sampletimesdetector_nr / DEV_N_samples[itq];  // detector identifier

  while ( sample_nr < DEV_N_samples[itq] )
    {     
      int is_BOI = 0; // beginning of an island
      int is_EOI = 0; // end of an island

      // check the BOI and EOI conditions
      if ( auxdata->island_pattern[sample_nr + idet*DEV_N_samples[itq]] > 0 )
	{
	  // check BOI condition
	  if ( sample_nr == 0 ) 
	    is_BOI=1;
	  else
	    if ( auxdata->island_pattern[sample_nr - 1 + idet*DEV_N_samples[itq]] == 0 )
	      is_BOI=1;
	  
	   // check EOI condition
	  if ( sample_nr == (DEV_N_samples[itq]-1) )
	    is_EOI=1;
	  else
	    if ( auxdata->island_pattern[sample_nr + 1 + idet*DEV_N_samples[itq]] == 0 )
	      is_EOI=1;
	}
	
      if ( is_BOI )
	{
	  // This is a beginning of an island
	  // extend the island for N_presamples

	  //printf("kernel_extend_xtalisland::: BOI, sample_nr %i , detector_nr  %i, array index \n",  sample_nr, idet, sample_nr + idet*DEV_N_samples[itq]); //debug

	  int i1 = sample_nr - DEV_N_presamples[itq];
	  if ( i1 < 0 ) i1 = 0;
	  int k;
	  for (k=i1; k<sample_nr; k++)
	    {
	      //atomicAdd( &(auxdata->island_pattern[k + idet*DEV_N_samples[itq]]), 1);		  
	      auxdata->island_pattern[k + idet*DEV_N_samples[itq]] = 1; // no need for atomic add with nsamples*ndetectors array		  
	    }

	}

      // check the "End Of Island" condition
      if ( is_EOI )
	{
	  // This is an end of an island
	  // extend the island for N_postsamples

	  //printf("kernel_extend_island::: EOI, sample_nr %i , detector_nr  %i, array index \n",  sample_nr, idet, sample_nr + idet*DEV_N_samples[itq]); //debug

	  int i2 = sample_nr + DEV_N_postsamples[itq];
	  if ( i2 >= DEV_N_samples[itq] ) i2 = DEV_N_samples[itq]-1;
	  int k;
	  for (k=i2; k>sample_nr; k--)
	    {
	      //atomicAdd( &(auxdata->island_pattern[k + idet*DEV_N_samples[itq]]), 1);		  
	      auxdata->island_pattern[k + idet*DEV_N_samples[itq]] = 1; // no need for atomic add with nsamples*ndetectors array		  
	    }
	}

      sample_nr += blockDim.x * gridDim.x;
    }
  }
}

/** 
 * Find the border samples around trigger samples
 * 
 * border samples are identified as = 2 in island_pattern[]
 * trigger samples are identified as = 1 in island_pattern[]
 * @param gpu_idata 
 * @param gpu_odata 
 */
__global__
void kernel_border_xtalislands(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq) {

  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;
 
  int sampletimesdetector_nr = tid + bid*num_threads; 
  if ( sampletimesdetector_nr < DEV_N_samples[itq]*DEV_N_detectors[itq] ){
  
    int sample_nr = sampletimesdetector_nr % DEV_N_samples[itq];  // for given detector
    int idet = sampletimesdetector_nr / DEV_N_samples[itq];  // detector identifier
  
  while ( sample_nr < DEV_N_samples[itq] ){ // what about first_chop, last_chop?
    
      if ( auxdata->island_pattern[sample_nr + idet*DEV_N_samples[itq]] == 1 ){ // look for trigger sample identifier (=1)
	
	int iX = idet % DEV_N_segments_x[itq]; // x-coordinate of trigger xtal
	int iY = idet / DEV_N_segments_x[itq]; // y-coordinate of trigger xtal

	//printf("kernel_border_island:  NX %i, NY %i, sample_nr %i , detector_nr  %i, iX %i, iY %i, array index %i\n", DEV_N_segments_x[itq], DEV_N_segments_y[itq], sample_nr, idet, iX, iY, sample_nr + idet*DEV_N_samples[itq]); //debug

	for (int ixborder = iX-1; ixborder <= iX+1; ixborder++) {
	  for (int iyborder = iY-1; iyborder <= iY+1; iyborder++) {

	    int jdet = ixborder + iyborder*DEV_N_segments_x[itq]; // border xtal index
	    //printf("kernel_border_xtalisland: iX %i, iY %i, ixborder %i, iyborder %i, idet %i, jdet %i, DEV_N_segments_x[itq] %i\n", iX, iY, ixborder, iyborder, idet, jdet, DEV_N_segments_x[itq]);
	    
	    if ( iX == ixborder && iY == iyborder ) continue; // skip trigger sample
	    if ( ixborder < 0 || ixborder >= DEV_N_segments_x[itq] ) continue; // out of range or detector array 
	    if ( iyborder < 0 || iyborder >= DEV_N_segments_y[itq] ) continue; // out of range or detector array 
	    
            if ( auxdata->island_pattern[sample_nr + jdet*DEV_N_samples[itq]] == 0 )
	       auxdata->island_pattern[sample_nr + jdet*DEV_N_samples[itq]] = 2; // write  border sample  identifier (=2)   

	  } // end y-border loop
	} // end x-border loop	
      } // end found trigger sample

      sample_nr += blockDim.x * gridDim.x;
    } // end while sample 
  } // if samples*detectors
}

__global__
void kernel_find_xtalislands(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  GPU_OUT_DATA *outdata = (GPU_OUT_DATA*) (auxdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;

  int sampletimesdetector_nr = tid + bid*num_threads; 
  if ( sampletimesdetector_nr < DEV_N_samples[itq]*DEV_N_detectors[itq] ){

    int sample_nr = sampletimesdetector_nr % DEV_N_samples[itq];  // for given detector
    int idet = sampletimesdetector_nr / DEV_N_samples[itq];  // detector identifier

  //find new islands (some of the original islands could have merged together) 
  while ( sample_nr < DEV_N_samples[itq] )
    {

      int is_BOI = 0; // beginning of an island

      // check the BOI and EOI conditions
      if ( auxdata->island_pattern[sample_nr  + idet*DEV_N_samples[itq]] > 0 ) // > 0 recognizes trigger and border samples
	{
	  // check BOI condition
	  if ( sample_nr == 0 ) 
	    is_BOI=1;
	  else
	    if ( auxdata->island_pattern[sample_nr - 1  + idet*DEV_N_samples[itq]] == 0 )
	      is_BOI=1;
	}


      if ( is_BOI )
	{
	  // This is a beginning of an island

	  //printf("kernel_find_xtalisland::: BOI, sample_nr %i , detector_nr  %i, array index %i, island pattern %i\n",  
	  //	 sample_nr, idet, sample_nr + idet*DEV_N_samples[itq], auxdata->island_pattern[sample_nr - 1  + idet*DEV_N_samples[itq]]); //debug
	  
	  // island number
	  int island_nr = atomicAdd( &(outdata->n_islands), 1); 
	  auxdata->island_info[island_nr].time = sample_nr;
	  auxdata->island_info[island_nr].detector = idet;
	  
	  // determine the length of the island
	  int i;
	  int island_nr_aux = island_nr + 1;
	  for (i=sample_nr; i<DEV_N_samples[itq]; i++)
	    {
	      if ( auxdata->island_pattern[i + idet*DEV_N_samples[itq]] == 0 )
		{
		  break;
		}
	      else
		{
		  auxdata->island_pattern[i + idet*DEV_N_samples[itq]] = island_nr_aux; // writes island number to island_pattern[], used by kernel_make island
		}
	    }
	  int island_len = i - sample_nr;
	  // record the length into first bin
	  auxdata->island_info[island_nr].length = island_len;

          // the ugly 4 accounts for two 32bit islander header words - the island time and island length
	  // below changed ndet*island_length to island_length when writing each xtals as individual island
          // changed ugly 4 to ugly 6 as added the writing of detector index as additional header word for each island
	  int offset = atomicAdd( &(outdata->island_offset), 6 + island_len); 
	  auxdata->island_info[island_nr].offset = offset;

          memcpy( &outdata->islands[offset], &sample_nr, sizeof(int) ); 
          memcpy( &outdata->islands[offset+2], &island_len, sizeof(int) ); 	  
          memcpy( &outdata->islands[offset+4], &idet, sizeof(int) );
	}

      sample_nr += blockDim.x * gridDim.x; 
    }
  }
}

__global__
void kernel_make_xtalislands(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  GPU_OUT_DATA *outdata = (GPU_OUT_DATA*) (auxdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;

  //find new islands (some of the old island could have merged   
  int sampletimesdetector_nr = tid + bid*num_threads; 
  if ( sampletimesdetector_nr < DEV_N_samples[itq]*DEV_N_detectors[itq] ){
    
    int sample_nr = sampletimesdetector_nr % DEV_N_samples[itq];  // for given detector
    int idet = sampletimesdetector_nr / DEV_N_samples[itq];  // detector identifier
    
    while ( sample_nr < DEV_N_samples[itq] )
      {
	int island_nr = auxdata->island_pattern[sample_nr + idet*DEV_N_samples[itq]];
	if ( island_nr > 0 )
	  {
	    island_nr--;
	    // +4 is needed to skip 32-bit time and length words in 16-bit array
	    // changed +4 to +6 as extra header word for dtector index
	    int island_offset    = auxdata->island_info[island_nr].offset + 6;  
	    int island_sample_nr = sample_nr - auxdata->island_info[island_nr].time;
	    //int island_length    = auxdata->island_info[island_nr].length;
	    int idet = auxdata->island_info[island_nr].detector;
	    
	    //printf("kernel_make_island::: island_nr %i, sample_nr %i, islandsample_nr %i, detector_nr  %i\n",  island_nr, sample_nr, island_sample_nr, idet ); //debug
	
            int i = island_offset + island_sample_nr;    
	    ADC_TYPE adc = gpu_idata[DEV_first_sample_index[idet][itq] + sample_nr];
	    outdata->islands[i] = adc;
	  }
	sample_nr += blockDim.x * gridDim.x; 
      }
  }
}

/** 
 * Evaluate pedestals
 * option 0 - global pedestal from ODB
 * option 1 - fill-by-fill calculation
 * 
 * @param gpu_idata 
 * @param gpu_odata 
 */
__global__
void kernel_make_pedestals(ADC_TYPE *gpu_idata, ADC_TYPE* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
 
  int idet = tid;  // detector number 
   
  // pedestal option 1, fill-by-fill calculation of pedestal
  if (DEV_pedestal_option[itq] == 1) {

    const int nsamples = 100; // number of samples for averaging
    double adc_mean = 0; // mean value of adc samples
    int i;
        // calculation on pedestal using first nsamples of fill
    for (i=0; i<nsamples; i++)
      {
	ADC_TYPE adc = gpu_idata[DEV_first_sample_index[idet][itq]+i];
	adc_mean += adc;
      }
    
    // sum -> average and store in pedestal array
    adc_mean /= nsamples;
    auxdata->pedestal[idet] = adc_mean;

    //printf("kernel_make_pedestals: idet %i, itq %i, DEV_first_sample_index[idet][itq] %i gpu_idata[DEV_first_sample_index[idet][itq]] %i, pedestal %f\n",
    //	   idet, itq, DEV_first_sample_index[idet][itq], gpu_idata[DEV_first_sample_index[idet][itq]], adc_mean);

  }

  // pedestal option 0, global pedestal from ODB 
  if (DEV_pedestal_option[itq] == 0) {

    auxdata->pedestal[idet] = DEV_global_pedestal[itq];

  }
  
}

__global__
void kernel_calc_ctag( void* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  GPU_OUT_DATA *outdata = (GPU_OUT_DATA*) (auxdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;
  // global index
  const unsigned int sample_nr = bid*num_threads + tid;

  // example counting of calorimeter hits
  //int threshold_high = DEV_ctag_threshold;;

  if(sample_nr>DEV_ctag_time_cut[itq] && sample_nr<DEV_N_samples[itq] && auxdata->wf_sum[sample_nr]<DEV_ctag_threshold[itq]) {
    if( auxdata->wf_sum[sample_nr]<auxdata->wf_sum[sample_nr+1] && auxdata->wf_sum[sample_nr]<auxdata->wf_sum[sample_nr-1]) { //ONLY COUNT THE PEAK
      
      atomicAdd(&(outdata->CTAG),1); // adc>2GeV && t>50us

    } 
  }
}
  
__global__
void kernel_decimate_sum( void* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  GPU_OUT_DATA *outdata = (GPU_OUT_DATA*) (auxdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;
  
  int sample_nr_1 = (tid + bid*num_threads)*DEV_decimation[itq];
  int sample_nr_2 = sample_nr_1 +  DEV_decimation[itq];

  int i;
  double adc_sum = 0;
  for (i=sample_nr_1; i<sample_nr_2; i++)
    {
      adc_sum += auxdata->wf_sum[i];
      //break; // makes the sum just first sample of DEV_decimation samples
    }
  
  // append the sum to the end of the output data
  // the ugly "6" accounts for 6x16-bit header words for data size (size 2), 
  // island number (size 1), detector number (size 1), CTAG (size 2)
  // the "ugly" +1 pads an odd number of 16-bit T-method data words to the
  // next 32-bit word boundary. 

  //old 16-bit fill-by-fill, decimated histo
  //int16_t *data = (int16_t*)(outdata) + (6 + outdata->island_offset); 
  //data[ sample_nr_1 / DEV_decimation[itq] ] = (int16_t) ( adc_sum ); 
 
  //new 32-bit fill-by-fill, decimated histo
  int32_t *data;
  if (outdata->island_offset % 2 == 0)  data = (int32_t*)(outdata) + (6 + outdata->island_offset)/2;
  else data = (int32_t*)(outdata) + (6 + outdata->island_offset + 1)/2;

  data[ sample_nr_1 / DEV_decimation[itq] ] = (int32_t) ( adc_sum );
}

__global__
void kernel_pedestal_store( void* gpu_odata, int itq)
{
  // input / aux / output data arrays
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  GPU_OUT_DATA *outdata = (GPU_OUT_DATA*) (auxdata+1);

  // access thread id
  const unsigned int tid = threadIdx.x;

  // get detector number
  int idet = tid; 

  // append the pedestal to the end of the output data via
  // pointer to outdata + island header size + island data size + Q-method data size
  // the ugly "6" accounts for 6x16-bit header words for data size (size 2), 
  // island number (size 1), detector number (size 1), CTAG (size 2)
  // the "ugly" +1 pads an odd number of 16-bit T-method data words to the
  // next 32-bit word boundary. 
  
  //old 16-bit fill-by-fill, decimated histo + pedestal data
  //int16_t *data = (int16_t*)(outdata) + (6 + outdata->island_offset) + DEV_N_samples[itq] / DEV_decimation[itq];
  //old 16-bit fill-by-fill, decimated histo + pedestal data
  float *data;
  if (outdata->island_offset % 2 == 0)  data = (float*)(outdata) + (6 + outdata->island_offset)/2 + (DEV_N_samples[itq]/DEV_decimation[itq]);
  else data = (float*)(outdata) + (6 + outdata->island_offset + 1)/2 + (DEV_N_samples[itq]/DEV_decimation[itq]);
  
  // writeout pedestal value
  data[idet] = (float) auxdata->pedestal[idet];
  //printf("kernel_pedestal_store: idet %i, itq %i, ped %f\n",  idet, itq, data[idet]);
}

// ATF, read in binary pulse template file
void read_template(const std::string& fname, pulseTemplate& templ,
                   phaseMap& pmap) {
  std::ifstream ifile(fname.c_str(), std::ifstream::binary);

  unsigned int samplesperfit;
  ifile.read((char*)&samplesperfit, sizeof(samplesperfit));
  assert(samplesperfit == SAMPLESPERFIT);

  unsigned int pointspersample;
  ifile.read((char*)&pointspersample, sizeof(pointspersample));
  assert(pointspersample == POINTSPERSAMPLE);

  ifile.read((char*)templ.table, NPOINTSTEMPLATE * sizeof(float));

  unsigned int npointsphasemap;
  ifile.read((char*)&npointsphasemap, sizeof(npointsphasemap));
  assert(npointsphasemap == NPOINTSPHASEMAP);

  ifile.read((char*)pmap.table, NPOINTSPHASEMAP * sizeof(float));

  ifile.read((char*)&pmap.timeOffset, sizeof(pmap.timeOffset));
}

/*
 * cuda_g2_bor_kernel() called at begin-of-run to copy the paramters for data processing from 
 * the host to device.
 *
 * gpu_idata input data of continuous samples from gpu_thread to GPU
 * cpu_odata output data of T-method, Q-method to gpu_thread from GPU
 *
 */

void cuda_g2_bor_kernel(){

  cudaError_t cudaCopyStatus;
  int rider_mod_max = 12, rider_chn_max = 5;

  dbprintf("cuda_g2_bor_kernel(), TQMETHOD_MAX = %d\n", TQMETHOD_MAX);

  // needed for accessing the CaloMap to make the array of first samples
  //amc13_ODB_get(); Already called in the top-level begin_of_run function
  
  // prepare processing parameters from ODB-mapped c-structure, etc
  for (int i = 0; i < TQMETHOD_MAX; i++){	       

    HOST_N_samples[i] = tq_parameters_odb[i].gpu_waveform_length;
    HOST_first_chop_sample[i] = tq_parameters_odb[i].gpu_waveform_firstsample;
    HOST_last_chop_sample[i] = tq_parameters_odb[i].gpu_waveform_lastsample;
    HOST_N_segments_x[i] = tq_parameters_odb[i].gpu_n_segments_x;
    HOST_N_segments_y[i] = tq_parameters_odb[i].gpu_n_segments_y;
    HOST_N_presamples[i] = tq_parameters_odb[i].gpu_island_presamples;
    HOST_N_postsamples[i] = tq_parameters_odb[i].gpu_island_postsamples;
    HOST_decimation[i] = tq_parameters_odb[i].calosum_decimation_factor;
    HOST_island_option[i] = tq_parameters_odb[i].island_option;
    HOST_threshold[i] = tq_parameters_odb[i].T_threshold;
    HOST_threshold_sign[i] = tq_parameters_odb[i].T_threshold_sign;
    HOST_useindividualthresholds[i] = tq_parameters_odb[i].use_channel_thresholds;
    HOST_pedestal_option[i] = tq_parameters_odb[i].pedestal_option;
    HOST_global_pedestal[i] = tq_parameters_odb[i].global_pedestal;
    HOST_hpedsubtract[i] = tq_parameters_odb[i].subtract_ped;
    HOST_hdecimation[i] = tq_parameters_odb[i].time_divide_hist;
    HOST_hfirstsample[i] = tq_parameters_odb[i].first_sample_in_hist;
    HOST_hlastsample[i] = tq_parameters_odb[i].last_sample_in_hist;
    HOST_hrebinintervals[i] = tq_parameters_odb[i].rebin_intervals_in_hist;
    HOST_hrebinincrement[i] = tq_parameters_odb[i].rebin_increment_in_hist;
    HOST_hoffset[i] = GPU_Data_Buffer[0].gpu_data_his_offset[i] / sizeof(int32_t);     // convert size from bytes to 32-bit words of histogram array
    HOST_hsize[i] = GPU_Data_Buffer[0].gpu_data_his_size[i] / sizeof(int32_t);
    HOST_nfitislands[i] = tq_parameters_odb[i].fit_islands;
    HOST_minfittime[i] = tq_parameters_odb[i].min_fit_time;
    HOST_fit_threshold[i] = tq_parameters_odb[i].fit_threshold;
    HOST_ctag_threshold[i] = tq_parameters_odb[i].CTAG_threshold;
    HOST_ctag_time_cut[i] = tq_parameters_odb[i].CTAG_time_cut;
    HOST_TMask_window[i] = tq_parameters_odb[i].TMask_window;
    HOST_mask_min[i] = tq_parameters_odb[i].mask_min;
    HOST_mask_max[i] = tq_parameters_odb[i].mask_max;
    HOST_mask_prescale[i] = tq_parameters_odb[i].mask_prescale;
    HOST_save_full_calo[i] = tq_parameters_odb[i].save_full_calo;
    HOST_save_xtal_border[i] = tq_parameters_odb[i].save_xtal_border;
    HOST_fit_prescale_factor[i] = tq_parameters_odb[i].fit_prescale_factor;
    HOST_fill_type[i] = tq_parameters_odb[i].fill_type;

    // set sequence index when filling histograms by sequence number using GPUmuonfillnumber (update to fill index from AMC13 header) 
    int fill_seq = 8;
    if  ( tq_parameters_odb[i].separate_sequence_hist ) {
      HOST_fill_seq[i] = fill_seq;
    } else {
      HOST_fill_seq[i] = 1;
    }

    for (int im = 0; im < rider_mod_max; im++){
      for (int ic = 0; ic < rider_chn_max; ic++){
	int index = ic + im*rider_chn_max;
	HOST_thresholdvalues[index][i] = rider_map_to_calo_odb[im][ic][i].value;
	HOST_thresholdpolarities[index][i] = rider_map_to_calo_odb[im][ic][i].polarity;
        dbprintf("im %i, ic %i, index %i, TQ %i, HOST_thresholdvalues %i, HOST_thresholdpolarities %i\n", 
	         im, ic, index, i, HOST_thresholdvalues[index][i], HOST_thresholdpolarities[index][i]);
      } // loop over rider channels
    } // loop over rider modules
  } // loop of TQ methods
  
  cudaError_t dev_set = cudaSetDevice( amc13_settings_odb.gpu_dev_id );
  if ( dev_set != cudaSuccess )
    {
      printf("ERROR: (gpu_bor) acquiring CUDA device\n");
    }
   
  // calculate the array index of first sample of each calo segment accounting for
  // (i) the Rider module/channel/waveform header/trailer words within AMC payload
  // (ii) the mapping between the calorimeter segments and the rider modules / channels

  int index;
  int ix, iy, im, ic, itq, idet;
 
  for (itq = 0; itq < TQMETHOD_MAX; itq++){

    dbprintf("cuda_g2_bor_kernel(), make map itq = %d\n", itq);

    index = 0; // reset first sample insex for each TQ method
    idet = 0; // enabled detector counter "enabled detector" type map

    for(im=0; im<rider_mod_max; im++){
      if (amc13_rider_odb[im].board.rider_enabled) {
      
        // account for Rider module header words
#ifdef USE_RIDER_FORMAT
        index += NRMH_WORDS;
#endif

	for(ic=0; ic<rider_chn_max; ic++){
	if (amc13_rider_odb[im].channel[ic].enabled) {
	  
      // account for Rider channel header words
#ifdef USE_RIDER_FORMAT
          index += NRCH_WORDS;
#endif

          if (tq_parameters_odb[itq].TQ_maptype == 0){
	    ix = rider_map_to_calo_odb[im][ic][itq].x_segment;
	    iy = rider_map_to_calo_odb[im][ic][itq].y_segment;
	    dbprintf("cuda_g2_bor_kernel(): im = %i, ic = %i, ix = %i, iy = %i\n",im,ic,ix,iy);
	  
	    if (ix >= 1 && ix <= HOST_N_segments_x[itq] && iy >= 1 && iy <= HOST_N_segments_y[itq] && rider_map_to_calo_odb[im][ic][itq].enabled) {
	      HOST_first_sample_index[ (ix-1) + (iy-1)*HOST_N_segments_x[itq] ][itq] = index;
	      HOST_rider_index[ (ix-1) + (iy-1)*HOST_N_segments_x[itq] ][itq] = ic + im*rider_chn_max;
	      dbprintf("calo segment x,y %i, %i  first_sample_index %i\n", ix, iy, HOST_first_sample_index[ (ix-1) + (iy-1)*HOST_N_segments_x[itq] ][itq] );
	      idet++;
	    }
	  }
          if (tq_parameters_odb[itq].TQ_maptype == 1) {
	    if ( rider_map_to_calo_odb[im][ic][itq].enabled ) {
	      HOST_first_sample_index[idet][itq] = index;
	      dbprintf("enabled detector %i  first_sample_index %i\n", idet, HOST_first_sample_index[idet][itq] );
              idet++;
	  }
	}
	
          // assume all channels have wavefrom of length gpu_waveform_length
          // calo processing is only sensible for equal length waveforms from all segments
	  // index += tq_parameters_odb[itq].gpu_waveform_length;
  
          // fix to allow different rider modules to have different fill lengths
          index += amc13_rider_odb[im].board.trig1__wvfm_length;

      // account for Rider channel trailer words
#ifdef USE_RIDER_FORMAT
	  index += NRCT_WORDS;
#endif
  	  } // if for enabled rider channels
        } // for loop on rider channels  

      // account for Rider module trailer words
#ifdef USE_RIDER_FORMAT
       index += NRMT_WORDS;
#endif
        } // if for enabled rider modules
     } // for loop on rider channels

    HOST_N_detectors[itq] = idet;
    if ( tq_parameters_odb[itq].TQ_on || tq_parameters_odb[itq].store_hist ){
      if ( HOST_N_detectors[itq] != HOST_N_segments_x[itq]*HOST_N_segments_y[itq] ){
	printf("ERROR: enabled channels and array size do not match!");
	cm_msg(MERROR, __FILE__, "enabled channels and array size do not match! TQ index %i, detector number %i, array dimensions %ix%i = %i\n", 
	       itq, HOST_N_detectors[itq], HOST_N_segments_x[itq], HOST_N_segments_y[itq], HOST_N_segments_x[itq]*HOST_N_segments_y[itq] );
      }
    }
 
 } // for loop on TQMETHODS

  // copy gpu analysis parameters to device

  // length of waveform
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_N_samples, HOST_N_samples, sizeof(HOST_N_samples), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of N_samples FAIL, bytes %d \n", sizeof(HOST_N_samples));
    }
  dbprintf("cudaMemcpyToSymbol of number of samples[0] %i, size %i, status %i \n", HOST_N_samples[0], sizeof(HOST_N_samples), (int)cudaCopyStatus );

  // first sample for chopping
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_first_chop_sample, HOST_first_chop_sample, sizeof(HOST_first_chop_sample), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of first_chop_sample FAIL, bytes %d \n", sizeof(HOST_first_chop_sample));
    }
  dbprintf("cudaMemcpyToSymbol of first chop sample[0] %i, size %i, status %i \n", HOST_first_chop_sample[0], sizeof(HOST_first_chop_sample), (int)cudaCopyStatus );

  // last sample for chopping
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_last_chop_sample, HOST_last_chop_sample, sizeof(HOST_last_chop_sample), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of last_chop_sample FAIL, bytes %d \n", sizeof(HOST_last_chop_sample));
    }
  dbprintf("cudaMemcpyToSymbol of last chop sample[0] %i, size %i, status %i \n", HOST_last_chop_sample[0], sizeof(HOST_last_chop_sample), (int)cudaCopyStatus );

  // number of detectors
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_N_detectors, HOST_N_detectors, sizeof(HOST_N_detectors), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of N_samples FAIL, bytes %d \n", sizeof(HOST_N_detectors));
    }
  dbprintf("cudaMemcpyToSymbol of number of detectors[0] %i, size %i, status %i \n", HOST_N_detectors[0], sizeof(HOST_N_detectors), (int)cudaCopyStatus );

  // number of segments_x
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_N_segments_x, HOST_N_segments_x, sizeof(HOST_N_segments_x), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of N_samples FAIL, bytes %d \n", sizeof(HOST_N_segments_x));
    }
  dbprintf("cudaMemcpyToSymbol of number of segments_x[0] %i, size %i, status %i \n", HOST_N_segments_x[0], sizeof(HOST_N_segments_x), (int)cudaCopyStatus );

  // number of segments_y
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_N_segments_y, HOST_N_segments_y, sizeof(HOST_N_segments_y), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of N_samples FAIL, bytes %d \n", sizeof(HOST_N_segments_y));
    }
  dbprintf("cudaMemcpyToSymbol of number of segments_y[0] %i, size %i, status %i \n", HOST_N_segments_y[0], sizeof(HOST_N_segments_y), (int)cudaCopyStatus );

  // N_detector array for first samples of detector segments
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_first_sample_index, HOST_first_sample_index, sizeof(HOST_first_sample_index), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of first sample indices FAIL, bytes %d \n", sizeof(HOST_first_sample_index));
    }
  dbprintf("cudaMemcpyToSymbol of first sample indices[0][0] %i, size %i, status %i \n", HOST_first_sample_index[0][0], sizeof(HOST_first_sample_index), (int)cudaCopyStatus );

  // N_detector array for rider index
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_rider_index, HOST_rider_index, sizeof(HOST_rider_index), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of rider-detector map indices FAIL, bytes %d \n", sizeof(HOST_rider_index));
    }
  dbprintf("cudaMemcpyToSymbol of rider-detector map indices[0][0] %i, size %i, status %i \n", HOST_rider_index[0][0], sizeof(HOST_rider_index), (int)cudaCopyStatus );

  // number of pre-samples and post-samples for T-method
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_N_presamples, HOST_N_presamples, sizeof(HOST_N_presamples), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of N_presamples FAIL, bytes %d \n", sizeof(HOST_N_presamples));
    }
  dbprintf("cudaMemcpyToSymbol of number of island pre-samples[0] %i, size %i, status %i \n", HOST_N_presamples[0], sizeof(HOST_N_presamples), (int)cudaCopyStatus );
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_N_postsamples, HOST_N_postsamples, sizeof(HOST_N_postsamples), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of N_postsamples FAIL, bytes %d \n", sizeof(HOST_N_postsamples));
    }
  dbprintf("cudaMemcpyToSymbol of number of island post-samples[0] %i, size %i, status %i \n", HOST_N_postsamples[0], sizeof(HOST_N_postsamples), (int)cudaCopyStatus );

  // T-method island option and threshold parameters
 cudaCopyStatus = cudaMemcpyToSymbol( DEV_island_option, HOST_island_option, sizeof(HOST_island_option), 0, cudaMemcpyHostToDevice);
   if ( cudaCopyStatus != cudaSuccess )
     {
        printf("cudaMemcpyToSymbol of island_options FAIL, bytes %d \n", sizeof(HOST_island_option));
    }	
   dbprintf("cudaMemcpyToSymbol of island_option[0] %i, size %i, status %i \n", HOST_island_option[0], sizeof(HOST_island_option), (int)cudaCopyStatus );
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_threshold, HOST_threshold, sizeof(HOST_threshold), 0, cudaMemcpyHostToDevice);
   if ( cudaCopyStatus != cudaSuccess )
     {
        printf("cudaMemcpyToSymbol of thresholds FAIL, bytes %d \n", sizeof(HOST_threshold));
    }	
   dbprintf("cudaMemcpyToSymbol of threshold[0] %i, size %i, status %i \n", HOST_threshold[0], sizeof(HOST_threshold), (int)cudaCopyStatus );
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_threshold_sign, HOST_threshold_sign, sizeof(HOST_threshold_sign), 0, cudaMemcpyHostToDevice);
   if ( cudaCopyStatus != cudaSuccess )
     {
        printf("cudaMemcpyToSymbol of bool threshold sign FAIL, bytes %d \n", sizeof(HOST_threshold_sign));
    }	
   dbprintf("cudaMemcpyToSymbol of bool threshold sign[0] %i, size %i, status %i \n", HOST_threshold_sign[0], sizeof(HOST_threshold_sign), (int)cudaCopyStatus );

  // T-method pedestal option and pedestal parameters
 cudaCopyStatus = cudaMemcpyToSymbol( DEV_pedestal_option, HOST_pedestal_option, sizeof(HOST_pedestal_option), 0, cudaMemcpyHostToDevice);
   if ( cudaCopyStatus != cudaSuccess )
     {
        printf("cudaMemcpyToSymbol of pedestal_options FAIL, bytes %d \n", sizeof(HOST_pedestal_option));
    }	
   dbprintf("cudaMemcpyToSymbol of pedestal_option[0] %i, size %i, status %i \n", HOST_pedestal_option[0], sizeof(HOST_pedestal_option), (int)cudaCopyStatus );
 cudaCopyStatus = cudaMemcpyToSymbol( DEV_global_pedestal, HOST_global_pedestal, sizeof(HOST_global_pedestal), 0, cudaMemcpyHostToDevice);
   if ( cudaCopyStatus != cudaSuccess )
     {
        printf("cudaMemcpyToSymbol of global_pedestals FAIL, bytes %d \n", sizeof(HOST_global_pedestal));
    }	
   dbprintf("cudaMemcpyToSymbol of global_pedestal[0] %i, size %i, status %i \n", HOST_global_pedestal[0], sizeof(HOST_global_pedestal), (int)cudaCopyStatus );

  // pedestal subtraction flag for fill-summed Q-method histogram
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_hpedsubtract, HOST_hpedsubtract, sizeof(HOST_hpedsubtract), 0, cudaMemcpyHostToDevice);
   if ( cudaCopyStatus != cudaSuccess )
     {
        printf("cudaMemcpyToSymbol of bool threshold sign FAIL, bytes %d \n", sizeof(HOST_hpedsubtract));
    }	
   dbprintf("cudaMemcpyToSymbol of bool fill-summed pedestal subtraction[0] %i, size %i, status %i \n", HOST_hpedsubtract[0], sizeof(HOST_hpedsubtract), (int)cudaCopyStatus );


  // first sample for fill-summed Q-method histogram
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_hfirstsample, HOST_hfirstsample, sizeof(HOST_hfirstsample), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of histo first sample FAIL, bytes %d \n", sizeof(HOST_hfirstsample));
    }
  dbprintf("cudaMemcpyToSymbol of histo first sample[0] %i, size %i, status %i \n", HOST_hfirstsample[0], sizeof(HOST_hfirstsample), (int)cudaCopyStatus );

  // last sample for fill-summed Q-method histogram
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_hlastsample, HOST_hlastsample, sizeof(HOST_hlastsample), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of histo last sample FAIL, bytes %d \n", sizeof(HOST_hlastsample));
    }
  dbprintf("cudaMemcpyToSymbol of histo last sample[0] %i, size %i, status %i \n", HOST_hlastsample[0], sizeof(HOST_hlastsample), (int)cudaCopyStatus );

  // rebin intervals for fill-summed Q-method histogram
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_hrebinintervals, HOST_hrebinintervals, sizeof(HOST_hrebinintervals), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of histo rebin intervals FAIL, bytes %d \n", sizeof(HOST_hrebinintervals));
    }
  dbprintf("cudaMemcpyToSymbol of histo rebin interval[0] %i, size %i, status %i \n", HOST_hrebinintervals[0], sizeof(HOST_hrebinintervals), (int)cudaCopyStatus );

  // rebin increment for fill-summed Q-method histogram
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_hrebinincrement, HOST_hrebinincrement, sizeof(HOST_hrebinincrement), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of histo rebin increment FAIL, bytes %d \n", sizeof(HOST_hrebinincrement));
    }
  dbprintf("cudaMemcpyToSymbol of histo rebin interval[0] %i, size %i, status %i \n", HOST_hrebinincrement[0], sizeof(HOST_hrebinincrement), (int)cudaCopyStatus );

  // time decimation factor for fill-summed Q-method histogram
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_hdecimation, HOST_hdecimation, sizeof(HOST_hdecimation), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of histo decimation FAIL, bytes %d \n", sizeof(HOST_hdecimation));
    }
  dbprintf("cudaMemcpyToSymbol of histo decimation factor[0] %i, size %i, status %i \n", HOST_hdecimation[0], sizeof(HOST_hdecimation), (int)cudaCopyStatus );

  // memory offset for multi TQ-method fill-summed Q-method histogram
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_hoffset, HOST_hoffset, sizeof(HOST_hoffset), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of histogram memory offset FAIL, bytes %d \n", sizeof(HOST_hoffset));
    }
  dbprintf("cudaMemcpyToSymbol of histogram memory offset[0,1] %i,%i,  size %i, status %i \n", HOST_hoffset[0], HOST_hoffset[1], sizeof(HOST_hoffset), (int)cudaCopyStatus );

  // time decimation factor for fill-by-fill Q-method histogram
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_decimation, HOST_decimation, sizeof(HOST_decimation), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of decimation FAIL, bytes %d \n", sizeof(HOST_decimation));
    }
  dbprintf("cudaMemcpyToSymbol of decimation factor[0,1] %i,%i,  size %i, status %i \n", HOST_decimation[0], HOST_decimation[1], sizeof(HOST_decimation), (int)cudaCopyStatus );

  // threshold values array's
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_thresholdvalues, HOST_thresholdvalues, sizeof(HOST_thresholdvalues), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of threshold values FAIL, bytes %d \n", sizeof(HOST_thresholdvalues));
    }
  dbprintf("cudaMemcpyToSymbol of threshold values[0][0],[0][1] %i, %i,  size %i, status %i \n", HOST_thresholdvalues[0][0], HOST_thresholdvalues[0][1], sizeof(HOST_thresholdvalues), (int)cudaCopyStatus );
  // threshold polarities array's
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_thresholdpolarities, HOST_thresholdpolarities, sizeof(HOST_thresholdpolarities), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of bool threshold polarities FAIL, bytes %d \n", sizeof(HOST_thresholdpolarities));
    }
  dbprintf("cudaMemcpyToSymbol of bool threshold polarities[0][0],[0][1] %i, %i, size %i, status %i \n", HOST_thresholdpolarities[0][0], HOST_thresholdpolarities[0][1], sizeof(HOST_thresholdpolarities), (int)cudaCopyStatus );

  // use individual threshold values
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_useindividualthresholds, HOST_useindividualthresholds, sizeof(HOST_useindividualthresholds), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of bool use threshold polarities FAIL, bytes %d \n", sizeof(HOST_useindividualthresholds));
    }
  dbprintf("cudaMemcpyToSymbol of bool use threshold polarities[0] %i, size %i, status %i \n", HOST_useindividualthresholds[0], sizeof(HOST_useindividualthresholds), (int)cudaCopyStatus );

  cudaCopyStatus = cudaMemcpyToSymbol( DEV_nfitislands, HOST_nfitislands, sizeof(HOST_nfitislands), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of number of fit islands FAIL, bytes %d \n", sizeof(HOST_nfitislands));
    }
  dbprintf("cudaMemcpyToSymbol of number of fit islands[0,1] %i,%i,  size %i, status %i \n", HOST_nfitislands, sizeof(HOST_nfitislands), (int)cudaCopyStatus );


  cudaCopyStatus = cudaMemcpyToSymbol( DEV_fit_threshold, HOST_fit_threshold, sizeof(HOST_fit_threshold), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of number of fit islands FAIL, bytes %d \n", sizeof(HOST_fit_threshold));
    }
  dbprintf("cudaMemcpyToSymbol of number of fit islands[0,1] %i,%i,  size %i, status %i \n", HOST_fit_threshold, sizeof(HOST_fit_threshold), (int)cudaCopyStatus );

  cudaCopyStatus = cudaMemcpyToSymbol( DEV_minfittime, HOST_minfittime, sizeof(HOST_minfittime), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of number of fit islands FAIL, bytes %d \n", sizeof(HOST_minfittime));
    }
  dbprintf("cudaMemcpyToSymbol of number of fit islands[0,1] %i,%i,  size %i, status %i \n", HOST_minfittime, sizeof(HOST_minfittime), (int)cudaCopyStatus );

  cudaCopyStatus = cudaMemcpyToSymbol( DEV_ctag_threshold, HOST_ctag_threshold, sizeof(HOST_ctag_threshold), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of CTAG threshold FAIL, bytes %d \n", sizeof(HOST_ctag_threshold));
    }
  dbprintf("cudaMemcpyToSymbol of CTAG threshold FAIL %i,%i,  size %i, status %i \n", HOST_ctag_threshold, sizeof(HOST_ctag_threshold), (int)cudaCopyStatus );

  cudaCopyStatus = cudaMemcpyToSymbol( DEV_ctag_time_cut, HOST_ctag_time_cut, sizeof(HOST_ctag_time_cut), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of CTAG time cut FAIL, bytes %d \n", sizeof(HOST_ctag_time_cut));
    }
  dbprintf("cudaMemcpyToSymbol of number of fit islands[0,1] %i,%i,  size %i, status %i \n", HOST_ctag_time_cut, sizeof(HOST_ctag_time_cut), (int)cudaCopyStatus );
  
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_TMask_window, HOST_TMask_window, sizeof(HOST_TMask_window),0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )  
    {
      printf("cudaMemcpyToSymbol of TMask window FAIL, bytes %d \n", sizeof(HOST_TMask_window));
    }
  dbprintf("cudaMemcpyToSymbol of T-method mask window %i,%i,  size %i, status %i \n", HOST_TMask_window, sizeof(HOST_TMask_window), (int)cudaCopyStatus );

  cudaCopyStatus = cudaMemcpyToSymbol( DEV_mask_min, HOST_mask_min, sizeof(HOST_mask_min), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of CTAG time cut FAIL, bytes %d \n", sizeof(HOST_mask_min));
    }
  dbprintf("cudaMemcpyToSymbol of minimum mask time %i,%i,  size %i, status %i \n", HOST_mask_min, sizeof(HOST_mask_min), (int)cudaCopyStatus );

  cudaCopyStatus = cudaMemcpyToSymbol( DEV_mask_max, HOST_mask_max, sizeof(HOST_mask_max), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of CTAG time cut FAIL, bytes %d \n", sizeof(HOST_mask_max));
    }
  dbprintf("cudaMemcpyToSymbol of maximum mask time %i,%i,  size %i, status %i \n", HOST_mask_max, sizeof(HOST_mask_max), (int)cudaCopyStatus );

  cudaCopyStatus = cudaMemcpyToSymbol( DEV_mask_prescale, HOST_mask_prescale, sizeof(HOST_mask_prescale), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of CTAG time cut FAIL, bytes %d \n", sizeof(HOST_mask_prescale));
    }
  dbprintf("cudaMemcpyToSymbol of minimum mask time %i,%i,  size %i, status %i \n", HOST_mask_prescale, sizeof(HOST_mask_prescale), (int)cudaCopyStatus );

  cudaCopyStatus = cudaMemcpyToSymbol( DEV_save_full_calo, HOST_save_full_calo, sizeof(HOST_save_full_calo), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of save full calo bool FAIL, bytes %d \n", sizeof(HOST_save_full_calo));
    }
  dbprintf("cudaMemcpyToSymbol of save_full_calo %i,%i,  size %i, status %i \n", HOST_save_full_calo, sizeof(HOST_save_full_calo), (int)cudaCopyStatus );

cudaCopyStatus = cudaMemcpyToSymbol( DEV_fill_type, HOST_fill_type, sizeof(HOST_fill_type), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of CTAG time cut FAIL, bytes %d \n", sizeof(HOST_fill_type));
    }
  dbprintf("cudaMemcpyToSymbol of minimum mask time %i,%i,  size %i, status %i \n", HOST_fill_type, sizeof(HOST_fill_type), (int)cudaCopyStatus );

  // total memory size for multi TQ-method fill-summed Q-method histogram
  cudaCopyStatus = cudaMemcpyToSymbol( DEV_hsize, HOST_hsize, sizeof(HOST_hsize), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of histogram memory size FAIL, bytes %d \n", sizeof(HOST_hsize));
    }
  dbprintf("cudaMemcpyToSymbol of hsize %i,%i,  size %i, status %i \n", HOST_hsize, sizeof(HOST_hsize), (int)cudaCopyStatus );                  

  cudaCopyStatus = cudaMemcpyToSymbol( DEV_fill_seq, HOST_fill_seq, sizeof(HOST_fill_seq), 0, cudaMemcpyHostToDevice);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of fill sequence FAIL, bytes %d \n", sizeof(HOST_fill_seq));
    }
  dbprintf("cudaMemcpyToSymbol of fill_seq %i,%i,  size %i, status %i \n", HOST_fill_seq, sizeof(HOST_fill_seq), (int)cudaCopyStatus );                  

  // read in templates
  phaseMap pmap;
  pulseTemplate templ;
  read_template("lasertempl.bin", templ, pmap);
  std::vector<phaseMap> phaseMapVec(N_DETECTORS_MAX, pmap);
  std::vector<pulseTemplate> pTempVec(N_DETECTORS_MAX, templ);

  // copy templates to gpu
  cudaCopyStatus = cudaMemcpyToSymbol(d_phase_maps, (void*)phaseMapVec.data(), phase_maps_size);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of phase maps FAIL, bytes %d \n", phase_maps_size);
    }
  dbprintf("cudaMemcpyToSymbol of phase maps size %i, status %i \n", phase_maps_size, (int)cudaCopyStatus );

  cudaCopyStatus = cudaMemcpyToSymbol(d_templates, (void*)pTempVec.data(), templates_size);
  if ( cudaCopyStatus != cudaSuccess )
    {
      printf("cudaMemcpyToSymbol of pulse templates FAIL, bytes %d \n", templates_size);
    }
  dbprintf("cudaMemcpyToSymbol of templates size %i, status %i \n", templates_size, (int)cudaCopyStatus );


  return;
}

/*
 * cuda_g2_run_kernel() calls the kernel functions that process data for each fill and copy the processed
 * data from device memory to host memory
 *
 * gpu_idata input data of continuous samples from gpu_thread to GPU
 * cpu_odata output data of T-method, Q-method to gpu_thread from GPU
 *
 */
void cuda_g2_run_kernel( unsigned char *gpu_idata, unsigned char *gpu_odata, int16_t *cpu_odata, int itq , int GPUbufferindex) {

  // cudaError_t cudaCopyStatus;

  cudaError_t dev_set = cudaSetDevice( amc13_settings_odb.gpu_dev_id );
  if ( dev_set != cudaSuccess )
    {
      printf("ERROR: (gpu_bor) acquiring CUDA device\n");
    }

  dbprintf("cuda_g2_run_kernel: TQ method index %i\n", itq);	

  // number if threads per block is optimized for GPU hardware 
  // e.g. k20 / k40 = 1024, c1060 = 512
  const int n_threads_per_block = 1024;

  // get GPU waveform length from odb parameters
  // is number of samples used for T/Q medod processing
  HOST_N_samples[itq] = tq_parameters_odb[itq].gpu_waveform_length;

  // define various block-thread grids for GPU processing functions with total
  // threads of N_samples, N_detectors, N_samples*N_detectors, etc

  // N_samples block-thread grid
  int n_blocks_nsamples = ( HOST_N_samples[itq] / n_threads_per_block ) + 1;
  if ( n_blocks_nsamples < 1 ) n_blocks_nsamples = 1;
  dim3  grid_nsamples( n_blocks_nsamples, 1, 1);
  dim3  threads_nsamples( n_threads_per_block, 1, 1);

  dbprintf("N_samples block-thread grid ::: %d samples, %d samples*detectors, %d blocks, %d threads/block, %d threads\n", 
	 HOST_N_samples[itq], HOST_N_samples[itq]*HOST_N_detectors[itq], n_blocks_nsamples, n_threads_per_block, n_blocks_nsamples*n_threads_per_block);

  // decimated N_samples block-thread grid
  int n_blocks_decimatednsamples = HOST_N_samples[itq] / n_threads_per_block / HOST_decimation[itq] + 1;
  if ( n_blocks_decimatednsamples < 1 ) n_blocks_decimatednsamples = 1;
  dim3  grid_decimatednsamples( n_blocks_decimatednsamples, 1, 1);
  dim3  threads_decimatednsamples( n_threads_per_block, 1, 1);  

  dbprintf("Decimated N_samples block-thread grid ::: %d samples, %d samples*detectors, %d blocks, %d threads/block, %d threads\n", 
	   HOST_N_samples[itq], HOST_N_samples[itq]*HOST_N_detectors[itq], n_blocks_decimatednsamples, n_threads_per_block, n_blocks_decimatednsamples*n_threads_per_block);

  // N_samples*N_detectors block-thread grid
  int n_blocks_nsamplesndetectors = HOST_N_samples[itq] * HOST_N_detectors[itq] / n_threads_per_block + 1;
  if ( n_blocks_nsamplesndetectors < 1 ) n_blocks_nsamplesndetectors = 1;
  dim3  grid_nsamplesndetectors( n_blocks_nsamplesndetectors, 1, 1);
  dim3  threads_nsamplesndetectors( n_threads_per_block, 1, 1);  

  dbprintf("N_samples*N_Detectors block-thread grid::: %d samples, %d samples*detectors, %d blocks, %d threads/block, %d threads\n", 
	   HOST_N_samples[itq], HOST_N_samples[itq]*HOST_N_detectors[itq], n_blocks_nsamplesndetectors, n_threads_per_block, n_blocks_nsamplesndetectors*n_threads_per_block);

  // N_samples*N_detectors*sizeof(int64_t)/sizeof(int16_t) block-thread grid for big-to-little endian byte re-ordering kernel function
  int n_blocks_bytereorder = ( HOST_N_samples[itq] * HOST_N_detectors[itq] / n_threads_per_block ) / ( sizeof(int64_t) / sizeof(int16_t) ) + 1;
  if ( n_blocks_bytereorder < 1 ) n_blocks_bytereorder = 1;
  dim3  grid_bytereorder( n_blocks_bytereorder, 1, 1);
  //dim3  threads_bytereorder( n_threads_per_block, 1, 1);

  dbprintf("Byte reorder block-thread grid ::: %d samples, %d samples*detectors, %d blocks, %d threads/block, %d threads\n", 
	   HOST_N_samples[itq], HOST_N_samples[itq]*HOST_N_detectors[itq], n_blocks_bytereorder, n_threads_per_block, n_blocks_bytereorder*n_threads_per_block);

  // N_detectors block-thread grid
  int n_blocks_ndetectors = 1; 
  if ( n_blocks_ndetectors < 1 ) n_blocks_ndetectors = 1;
  dim3  grid_ndetectors( n_blocks_ndetectors, 1, 1);
  dim3  threads_ndetectors(  HOST_N_detectors[itq], 1, 1);

  dbprintf("N_detectors block-thread grid ::: %d samples, %d samples*detectors, %d blocks, %d threads/block, %d threads\n", 
	   HOST_N_samples[itq], HOST_N_samples[itq]*HOST_N_detectors[itq], n_blocks_ndetectors, n_threads_per_block, n_blocks_ndetectors*n_threads_per_block);

  // GPU memory alllocation parameters
  dbprintf(" ::: start-of-kernel, size of  GPU_OBUF_SIZE 0x%08x, GPU_HIS_DATA 0x%08x, GPU_AUX_DATA 0x%08x, GPU_OUT_DATA 0x%08x \n", 
  	   GPU_OBUF_SIZE, sizeof(GPU_HIS_DATA), sizeof(GPU_AUX_DATA), sizeof(GPU_OUT_DATA) );

  // protect against cuda malloc too small
  if ( GPU_OBUF_SIZE < sizeof(GPU_HIS_DATA)+sizeof(GPU_AUX_DATA)+sizeof(GPU_OUT_DATA) ) {
    printf("GPU_OBUF_SIZE too small!!!");
    cm_msg(MERROR, __FILE__, "GPU_OBUF_SIZE too small!!!");
    return;
  }

  // measure time
  //#define TIME_MEASURE_DEF
#ifdef TIME_MEASURE_DEF
  cudaEvent_t start, stop;
  cudaEvent_t start_all, stop_all;
  float elapsedTime;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);

  cudaEventCreate(&start_all);
  cudaEventCreate(&stop_all);
  cudaEventRecord(start_all, 0);
#endif // TIME_MEASURE_DEF

  // reset the output memory
#ifdef TIME_MEASURE_DEF
  // start event
  cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF

  // avoid zeroing of fill-by-fill histogram
  GPU_HIS_DATA *hisdata = (GPU_HIS_DATA*) gpu_odata;
  GPU_AUX_DATA *auxdata = (GPU_AUX_DATA*) (hisdata+1);
  cudaMemset( auxdata, 0, ( GPU_OBUF_SIZE - sizeof(GPU_HIS_DATA) ) );

#ifdef TIME_MEASURE_DEF
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&elapsedTime, start, stop);
  printf(" ::: GPU_OBUF reset time %f ms (%i MB)\n",elapsedTime, GPU_OBUF_SIZE/1024/1024);
#endif // TIME_MEASURE_DEF

    // re-order bytes of 16-bit ADC words for big-endian 64-bit AMC13 words
    // this function is redundant since June 2016 as Rider samples payloads
    // are now little-endian rather than big-endian. 
#if 0
#ifdef TIME_MEASURE_DEF
    // start event
    cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
    
    kernel_wf_be64tole16<<< grid_bytereorder, threads_bytereorder>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );

#ifdef CUDA_ERROR_CHECK
    CudaCheckError();
#endif
    
#ifdef TIME_MEASURE_DEF
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);
    printf(" ::: kernel_wf_be64tole16 time %f ms n_blocks %i n_threads_per_block %i\n",elapsedTime, n_blocks_bytereorder, n_threads_per_block );
#endif // TIME_MEASURE_DEF
#endif
    
#if 1
    // calculate pedestals of detector elements (functionality controlled by pedestal option ODB parameter)
#ifdef TIME_MEASURE_DEF
    // start event
    cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
    
    kernel_make_pedestals<<< grid_ndetectors, threads_ndetectors>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );
    
#ifdef CUDA_ERROR_CHECK
    CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);
    printf(" ::: kernel_make_pedestals time %f ms n_blocks %i n_threads_per_block %i\n",elapsedTime, n_blocks_ndetectors, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif 
    
#if 0
    // print function of debugging device parameters such as calo map, pedestal values
    cudaPrintfInit();
    
    dim3  grid_test( 1, 1, 1);
    dim3  threads_test( HOST_N_segments_x[itq], HOST_N_segments_y[itq], 1); 
    kernel_print_map<<< grid_test, threads_test >>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );
    
    cudaPrintfDisplay(stdout, true);
    cudaPrintfEnd();
#endif

  // if TQ processing is switched on then call TQ processing functions  
  if ( tq_parameters_odb[itq].TQ_on ){ 
    
    // function to make the distribution of ADC samples - too slow, don't use
    // kernel_wf_make_ADC<<< grid_nsamples, threads_nsamples>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata );
    
#if 1
    // calculate the calorimeter sum of all detector waveforms
#ifdef TIME_MEASURE_DEF
    // start event
    cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
    
    kernel_wf_sum<<< grid_nsamples, threads_nsamples>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq);
    
#ifdef CUDA_ERROR_CHECK
    CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);
    printf(" ::: kernel_wf_sum time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamples, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
    

#if 1

    //fit pulses
    if(HOST_nfitislands[itq]>0){

      if ( GPUmuonfillnumber%HOST_fit_prescale_factor[itq] == 0 ) { // prescale the fitting based on muon fill number


#ifdef TIME_MEASURE_DEF
	//start event
	cudaEventRecord(start, 0);
#endif
      // zero results buffer
	cudaMemset(device_fitresult, 0, result_size);

#ifdef TIME_MEASURE_DEF
	cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&elapsedTime, start, stop);
	printf(" ::: cudaMeset for device_fitresults time %f ms\n",elapsedTime);
#endif // TIME_MEASURE_DEF
	
	// 128 seems to give best performance
	int fit_threadsperblock = 128;
	dim3 fit_dimblock(fit_threadsperblock);
	int fit_blockspertrace =
	  ceil(static_cast<double>( HOST_N_samples[itq]) / fit_threadsperblock);
	dim3 fit_dimgrid(fit_blockspertrace);
#ifdef TIME_MEASURE_DEF
	//start event
	cudaEventRecord(start, 0);
#endif
	kernel_wf_fittimes<<<fit_dimgrid, fit_dimblock>>>( (ADC_TYPE*)gpu_idata, device_fitresult, itq);

#ifdef CUDA_ERROR_CHECK
	CudaCheckError();
#endif
	
#ifdef TIME_MEASURE_DEF
	cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&elapsedTime, start, stop);
	printf(" ::: kernel_wf_fittimes time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, fit_blockspertrace, fit_threadsperblock);
#endif // TIME_MEASURE_DEF
	
#endif
	
#if 1
	//fit pulses
	
	int nblocksx = ceil(static_cast<double>(OUTPUTARRAYLEN) / PULSESPERBLOCK);
	fit_dimgrid = dim3(nblocksx, HOST_N_detectors[itq]);
	fit_dimblock = dim3(SAMPLESPERFIT * PULSESPERBLOCK, 1);
	
#ifdef TIME_MEASURE_DEF
	//start event
	cudaEventRecord(start, 0);
#endif
	//float avg = 0.;
	//for(uint16_t it=0; it<HOST_N_samples[0]; ++it){avg += static_cast<float>(gpu_idata + HOST_first_sample_index[0][0]+it);}
	//printf(" trace avg (idet=0,itq=0) =  %f\n", avg/HOST_N_samples[0]);
	//printf("HOST_N_samples[0],HOST_first_sample_index[0][0]: %d, %d\n",HOST_N_samples[0], HOST_first_sample_index[0][0]);
	kernel_wf_fitenergies<<< fit_dimgrid, fit_dimblock>>>( (ADC_TYPE*)gpu_idata, device_fitresult, itq);

#ifdef CUDA_ERROR_CHECK
	CudaCheckError();
#endif
	
#ifdef TIME_MEASURE_DEF
	cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&elapsedTime, start, stop);
	printf(" ::: kernel_wf_fitenergies time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, nblocksx*HOST_N_detectors[itq], SAMPLESPERFIT * PULSESPERBLOCK);
#endif // TIME_MEASURE_DEF

      } // prescale fitting

    } // if fitting on
#endif
    
    bool tmask_prescale = false;
    if(GPUmuonfillnumber%HOST_mask_prescale[itq])
      tmask_prescale = true;
    
    if ( tq_parameters_odb[itq].save_full_calo || tq_parameters_odb[itq].save_truncated_calo )  { // save full or truncated calo islands 

      printf(" ::: Process CALO islands\n");
      
#if 1
      // find samples that trigger (functionality controlled by threshold option ODB parameter)
#ifdef TIME_MEASURE_DEF
      // start event
      cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
      
      kernel_wf_trigger<<< grid_nsamples, threads_nsamples>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq, tmask_prescale);
      
#ifdef CUDA_ERROR_CHECK
      CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
      cudaEventRecord(stop, 0);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      printf(" ::: kernel_wf_trigger time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamples, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
      
#if 1
      // extend islands by a predefined number of pre-samples and post-samples
#ifdef TIME_MEASURE_DEF
      // start event
      cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
      
      kernel_extend_islands<<< grid_nsamples, threads_nsamples>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );
      
#ifdef CUDA_ERROR_CHECK
      CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
      cudaEventRecord(stop, 0);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      printf(" ::: kernel_extend_island time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamples, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
      
#if 1
      // find contiguous islands 
#ifdef TIME_MEASURE_DEF
      // start event
      cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
      
      kernel_find_islands<<< grid_nsamples, threads_nsamples>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );
      
#ifdef CUDA_ERROR_CHECK
      CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
      cudaEventRecord(stop, 0);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      printf(" ::: kernel_find_island time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamples, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
      
#if 1
      // make island data structure of island time, length and sample values
#ifdef TIME_MEASURE_DEF
      // start event
      cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
      
      kernel_make_islands<<< grid_nsamples, threads_nsamples>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );
      
#ifdef CUDA_ERROR_CHECK
      CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
      cudaEventRecord(stop, 0);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      printf(" ::: kernel_make_island time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamples, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif

    } else { // save individual xtal islands
      
      printf(" ::: Process XTAL islands\n");
#if 1
      // find samples that trigger (functionality controlled by threshold option ODB parameter)
#ifdef TIME_MEASURE_DEF
      // start event
      cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
      
      kernel_wf_xtaltrigger<<< grid_nsamples, threads_nsamples>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq, tmask_prescale);
      
#ifdef CUDA_ERROR_CHECK
      CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
      cudaEventRecord(stop, 0);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      printf(" ::: kernel_wf_xtaltrigger time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamples, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
      
#if 1
      // extend islands by a predefined number of pre-samples and post-samples
#ifdef TIME_MEASURE_DEF
      // start event
      cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
      
      kernel_extend_xtalislands<<< grid_nsamplesndetectors, threads_nsamplesndetectors>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );
      
#ifdef CUDA_ERROR_CHECK
      CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
      cudaEventRecord(stop, 0);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      printf(" ::: kernel_xtalextend_island time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamplesndetectors, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
      
      if ( HOST_save_xtal_border[itq] ) { // save full calo islands 
	
#if 1
	// border islands identfies the border samples around the trigger samples 
#ifdef TIME_MEASURE_DEF
	// start event
	cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
	
	kernel_border_xtalislands<<< grid_nsamplesndetectors, threads_nsamplesndetectors>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );

#ifdef CUDA_ERROR_CHECK
	CudaCheckError();
#endif
	
#ifdef TIME_MEASURE_DEF
	cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&elapsedTime, start, stop);
	printf(" ::: kernel_border_xtalisland time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamplesndetectors, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
	
      } // end save xtal borders
      
#if 1
      // find contiguous islands 
#ifdef TIME_MEASURE_DEF
      // start event
      cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
      
      kernel_find_xtalislands<<< grid_nsamplesndetectors, threads_nsamplesndetectors>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );
      
#ifdef CUDA_ERROR_CHECK
      CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
      cudaEventRecord(stop, 0);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      printf(" ::: kernel_find_xtalisland time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamplesndetectors, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
      
#if 1
      // make island data structure of island time, length and sample values
#ifdef TIME_MEASURE_DEF
      // start event
      cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
      
      kernel_make_xtalislands<<< grid_nsamplesndetectors, threads_nsamplesndetectors>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );
      
#ifdef CUDA_ERROR_CHECK
      CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
      cudaEventRecord(stop, 0);
      cudaEventSynchronize(stop);
      cudaEventElapsedTime(&elapsedTime, start, stop);
      printf(" ::: kernel_make_xtalisland time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamplesndetectors, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
      
    } // end save xtal islands
	
#if 1
    //calculate calorimeter paramter ctag 
#ifdef TIME_MEASURE_DEF
    // start event
    cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
    
    kernel_calc_ctag<<< grid_nsamples, threads_nsamples>>>( gpu_odata, itq );
    
#ifdef CUDA_ERROR_CHECK
    CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);
    printf(" ::: kernel_calc_ctag time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_nsamples, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
    
#if 1
    // decimate the calorimeter sum waveform for Q'-method
#ifdef TIME_MEASURE_DEF
    // start event
    cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
    
    kernel_decimate_sum<<< grid_decimatednsamples, threads_decimatednsamples>>>( gpu_odata, itq );
    
#ifdef CUDA_ERROR_CHECK
    CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);
    printf(" ::: kernel_decimate_sum time %f ms n_blocks %i n_threads_per_block %i \n",elapsedTime, n_blocks_decimatednsamples, n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif
    
#if 1
    // append the pedestal data to the end of the output data (is done after islands are obtained)
#ifdef TIME_MEASURE_DEF
    // start event
    cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF
    
    kernel_pedestal_store<<< grid_ndetectors, threads_ndetectors>>>( gpu_odata, itq );
    
#ifdef CUDA_ERROR_CHECK
    CudaCheckError();
#endif

#ifdef TIME_MEASURE_DEF
    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&elapsedTime, start, stop);
    printf(" ::: kernel_pedestal_store time %f ms n_blocks %i n_threads_per_block %i\n",elapsedTime, n_blocks_ndetectors,  n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif 
    
  } // end of TQ_on processing
  
  if ( tq_parameters_odb[itq].store_hist ) {

#if 0
  // accumulate the fill-by-fill sum of individual waveforms for all detectors (uses a time-decimation factor from ODB)
#ifdef TIME_MEASURE_DEF
  // start event
  cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF  
  
  kernel_wf_fillsum<<< grid_nsamplesndetectors, threads_nsamplesndetectors>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq );

#ifdef CUDA_ERROR_CHECK
  CudaCheckError();
#endif
  
#ifdef TIME_MEASURE_DEF
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&elapsedTime, start, stop);
  printf(" ::: kernel_wf_fillsum time %f ms n_blocks %i n_threads_per_block %i\n",elapsedTime, n_blocks_nsamplesndetectors,  n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif

#if 1
  // accumulate the fill-by-fill sum of individual waveforms for all detectors (uses a time-decimation factor from ODB)
#ifdef TIME_MEASURE_DEF
  // start event
  cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF  
  
 
  int isubhist = 0;
  for (isubhist =0; isubhist < HOST_hrebinintervals[itq]; isubhist++){

    // block-thread grid Qmethod sub-histograms
    int noriginalsamples = HOST_N_detectors[itq] * ( HOST_hlastsample[itq] - HOST_hfirstsample[itq] + 1 );
    int nsubsamples = noriginalsamples / HOST_hrebinintervals[itq];
    int subrebinfactor = HOST_hdecimation[itq] * pow( (double)HOST_hrebinincrement[itq], (double)isubhist );
    int nsubbins = nsubsamples / subrebinfactor;
    int n_blocks_subhists = nsubbins / n_threads_per_block + 1;
    dbprintf("::: kernel_wf_fillsum2 itq %i ndetectors %i, isubhist %i, noriginalsamples %i, nsubsamples %i, rebinfactor %i nsubbins %i n_blocks %i\n", 
	   itq, HOST_N_detectors[itq], isubhist, noriginalsamples, nsubsamples, subrebinfactor, nsubbins, n_blocks_subhists );
    dim3  grid_nsubhists( n_blocks_subhists, 1, 1);
    dim3  threads_nsubhists( n_threads_per_block, 1, 1);  

    kernel_wf_fillsum2<<< grid_nsubhists, threads_nsubhists>>>( (ADC_TYPE*)gpu_idata, (ADC_TYPE*)gpu_odata, itq, isubhist, GPUmuonfillnumber );
#ifdef CUDA_ERROR_CHECK
    CudaCheckError();
#endif

  }

#ifdef TIME_MEASURE_DEF
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&elapsedTime, start, stop);
  printf(" ::: kernel_wf_fillsum2 time %f ms n_blocks %i n_threads_per_block %i\n",elapsedTime, n_blocks_nsamplesndetectors,  n_threads_per_block);
#endif // TIME_MEASURE_DEF
#endif

  } // end of store_hist processing

  // size of processed data from GPU
  GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] = 0;

#if 1
  // copy data from GPU
#ifdef TIME_MEASURE_DEF
  // start event
  cudaEventRecord(start, 0);
#endif // TIME_MEASURE_DEF

  // copy the 32-bit size in units of 16bit words of island data from GPU. The size incorporates the 32bit island time (size 2),
  // 32bit island length (size 2), and the 16bit (samples each size 1) for all islands.
  cudaMemcpy( &GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq], gpu_odata + sizeof(GPU_HIS_DATA) + sizeof(GPU_AUX_DATA), sizeof(int), 
			     cudaMemcpyDeviceToHost);

  // convert the size of N islands with time stamp and island length into units of bytes and account for padding
  // if gpu_data_proc_size[itq] is odd to 32-bit boundary.
  if (GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] % 2) GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq]++;
  GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] *= sizeof(int16_t);

  // increment the size of GPU output data for header items not included in island data size
 
 // 32-bit parameters island data size
  GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] += sizeof(int);
  // 16-bit number of islands + 16-bit number of detectors
  GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] += sizeof(int);
  // 32-bit reserved / CTAG word
  GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] += sizeof(int);

  // add the size of 32-bit decimated fill-by-fill histogram
  GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] += HOST_N_samples[itq] / HOST_decimation[itq] * sizeof(int32_t); // old 16-bit words for fill-by-fill, decimated histo

  // add the size of array of float pedestal values
  GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] += HOST_N_detectors[itq] * sizeof(float);

  dbprintf(" ::: GPU output data size %i\n",GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq]);
  if ( GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] > gpu_data_proc_size_max )
    {
      printf("***ERROR! too large output gpu data! %i\n",GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq]);
      GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq] = 0;
    }

  cudaMemcpy( cpu_odata, gpu_odata + sizeof(GPU_HIS_DATA)+ sizeof(GPU_AUX_DATA), GPU_Data_Buffer[GPUbufferindex].gpu_data_proc_size[itq], cudaMemcpyDeviceToHost);

  // Device to host copy of derived data
#ifdef TIME_MEASURE_DEF
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&elapsedTime, start, stop);
  printf(" ::: copy data from GPU time %f ms\n",elapsedTime);
#endif // TIME_MEASURE_DEF
  
  // write HOST_N_detectors to derived data after copy from GPU
  cpu_odata[3] = HOST_N_detectors[itq];

#endif

// copy fit results
  if(HOST_nfitislands[itq]>0){
#ifdef TIME_MEASURE_DEF
  cudaEventRecord(start, 0);
#endif  

  cudaMemcpy(host_fitresult, device_fitresult, result_size, cudaMemcpyDeviceToHost);
  /*  
  for(uint segment_num = 0; segment_num< 10; ++segment_num)
  {
  	printf("host_fitresult[%u]: energy=%f, pedestal=%f \n", segment_num, host_fitresult[segment_num].fit_results[0].energy, host_fitresult[segment_num].fit_results[0].pedestal);
  }
  */

#ifdef TIME_MEASURE_DEF
  cudaEventRecord(stop, 0);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&elapsedTime, start, stop);
  printf(" ::: copy fit results from GPU time %f ms\n",elapsedTime);
#endif  
  }

#ifdef TIME_MEASURE_DEF

  cudaEventRecord(stop_all, 0);
  cudaEventSynchronize(stop_all);
  cudaEventElapsedTime(&elapsedTime, start_all, stop_all);
  printf(" ::: CUDA kernel total elapsed time %f ms\n",elapsedTime);

  // Clean up:
  cudaEventDestroy(start);
  cudaEventDestroy(stop);
  cudaEventDestroy(start_all);
  cudaEventDestroy(stop_all);
#endif // TIME_MEASURE_DEF

  dbprintf(" ::: end-of-kernel, size of gpu_odata 0x%08x, GPU_HIS_DATA 0x%08x, GPU_AUX_DATA 0x%08x, GPU_OUT_DATA 0x%08x \n", 
  	   GPU_OBUF_SIZE, sizeof(GPU_HIS_DATA), sizeof(GPU_AUX_DATA), sizeof(GPU_OUT_DATA) );
}

#if 0
__global__
void kernel_wf_sum_make_islands( ADC_TYPE* gpu_idata, ADC* gpu_odata, itq)
{
  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;
  // global index
  const unsigned int sample_nr = bid*num_threads + tid;

  CALORIMETER_DATA_BLOCK *cal_data = (CALORIMETER_DATA_BLOCK*) gpu_odata;

  // If sample is zero finish.
  // If sample is not zero, record 5 samples before and 10 after

  if ( sample_nr >= WAVEFORM_LENGTH_MAX ) return;
  
  if ( cal_data->wf_sum_thr.adc[sample_nr] == 0 ) return;
  
  if ( sample_nr > 0 && sample_nr < (WAVEFORM_LENGTH_MAX-1) )
    {
      if ( cal_data->wf_sum_thr.adc[sample_nr-1] != 0 &&  cal_data->wf_sum_thr.adc[sample_nr+1] != 0 )
	{
	  return;
	}
    }


  for (int i=1; i<7; i++)
    {    
      int s = sample_nr - i;
      if ( s>=0 )
	{
	  cal_data->wf_sum_thr.adc[s] = cal_data->wf_sum.adc[s];
	}
    }
  
  for (int i=1; i<24; i++)
    {    
      int s = sample_nr + i;
      if ( s < WAVEFORM_LENGTH_MAX )
	{
	  cal_data->wf_sum_thr.adc[s] = cal_data->wf_sum.adc[s];
	}
    }

}


__global__
void kernel_wf_sum_glue_islands(unsigned char* gpu_odata, itq )
{

  CALORIMETER_DATA_BLOCK *calo = (CALORIMETER_DATA_BLOCK*)gpu_odata;
  int16_t *adc = calo->wf_sum_thr.adc;

  unsigned int i;
  bool sample_active = false;
  unsigned int N_islands = 0;
  //unsigned int island_len = 0;  
  unsigned int sample0 = 0;
  unsigned int offset = 0;
  // @todo replace WAVEFORM_LENGTH_MAX with actual wf length
  for (i=0; i<WAVEFORM_LENGTH_MAX; i++)
    {
      int16_t val = adc[i];
      if ( val == 0 )
	{
	  if ( sample_active )
	    {
	      // finish sample
	      calo->i_info[N_islands].sample0 = sample0;
	      unsigned int island_len = i - sample0;  
	      calo->i_info[N_islands].length = island_len;
	      N_islands++;
	      sample_active = false;
	      offset += ALIGN8(ISLAND_HEADER_LEN + island_len*ADC_SAMPLE_LEN);
	    }
	}
      else
	{
	  if ( sample_active )
	    {
	      // add new sample to the island
	      //calo->
	      ;
	    }
	  else
	    {
	      // start new island
	      calo->i_info[N_islands].offset = offset;
	      sample0 = i;
	      sample_active = true;
	    }
	}
    }
  
  calo->N_islands = N_islands;
  
  calo->Islands_len_total = offset;

}



__global__
void kernel_make_islands(unsigned char* gpu_odata, , itq)
{
  // access thread id
  const unsigned int tid = threadIdx.x;
  // access number of threads in this block
  const unsigned int num_threads = blockDim.x;
  // access block id
  const unsigned int bid = blockIdx.x;
  // global index
  const unsigned int island_nr = bid*num_threads + tid;

  CALORIMETER_DATA_BLOCK *cal_data = (CALORIMETER_DATA_BLOCK*) gpu_odata;

  unsigned int N_islands = cal_data->N_islands;

  if ( island_nr >= N_islands ) return;
  //if ( island_nr > 1 ) return;

  unsigned int sample0   = cal_data->i_info[island_nr].sample0;
  unsigned int len       = cal_data->i_info[island_nr].length;
  unsigned int offset    = cal_data->i_info[island_nr].offset;
  unsigned int len_total = cal_data->Islands_len_total;

  unsigned int iwf;
#if 0
  for (iwf=0; iwf<WAVEFORMS_NUM; iwf++)
#endif
#if 1
  for (iwf=0; iwf<1; iwf++)
#endif
    {
      /*
      unsigned char *ptr = (unsigned char*) &(cal_data->island);
      ISLAND_HEADER *island_header = (ISLAND_HEADER*)( ptr +  
						       iwf*len_total
						       + offset);
      */
      unsigned char *ptr = (unsigned char*) &(cal_data->island);
      int16_t *island = (int16_t*)( ptr + iwf*len_total + offset );
      //unsigned int *ptr_length  = ptr_sample0+1;
      //island_nr*(sizeof(ISLAND)-sizeof(unsigned short int)) + 
      //iwf*N_islands*(sizeof(ISLAND)-sizeof(unsigned short int));

      island[0] = sample0;
      island[1] = len;
      //unsigned short int *adc_tgt = (unsigned short int*)(island_header+1);
      //ptr = (unsigned char*)island_header; 
      //unsigned short int *adc_tgt = (unsigned short int*)(ptr+sizeof(ISLAND_HEADER));
      unsigned short int *adc_tgt = (unsigned short int*)(island+2);
      unsigned short int *adc_src = cal_data->wf[iwf].adc;
      //cal_data->wf_sum.adc[sample_nr] += cal_data->wf[i].adc[sample_nr];
      unsigned int i;
      for (i=0; i<len; i++)
	{
	  adc_tgt[i] = adc_src[sample0+i]; 
	  //adc_tgt[i] = i+1;//adc_src[sample0+i]; 
	}

#if 0
      island_header->length = 10;
      island_header->sample0 = 20;
#endif

    }


}

#endif







