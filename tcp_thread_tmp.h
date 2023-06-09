/* tcp_thread.h --- 
 * 
 * Filename:          tcp_thread.h
 * Description: 
 * Author:            Tim Gorringe
 * Maintainer:        Tim Gorringe
 * Created:           Thu May 16 07:49:29 CDT 2013
 * Version:           $id$
 * Last-Updated: Fri May 23 11:52:17 2014 (-0400)
 *           By: Data Acquisition
 *     Update #: 47
 * URL: 
 * Keywords: 
 * Compatibility: 
 * 
 */

/* Commentary: 
 * 
 * 
 * 
 */

/* Change Log:
 * 
 * $Log$
 * 
 * added ring buffer for tcp->gpu communication using 2D array 
 */

/* This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 3, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth
 * Floor, Boston, MA 02110-1301, USA.
 */

/* Code: */
#ifndef tcp_thread_h
#define tcp_thread_h

#ifdef tcp_thread_c
#define EXTERN
#else
#define EXTERN extern
#endif

// max number of TCP threads
#define TCP_THREAD_NUM_MAX               10

// number of TCP threads
extern int tcp_thread_num;


/* total = TP_BLOCK_SIZE*TP_NUM_BLOCKS = 32 MB */
#define  TP_BLOCK_SIZE  32768
#define  TP_NUM_BLOCKS   1024
#define  FRAME_SIZE      2048
#define  NUM_FRAMES     16384

/**
 *  paramters and structure for decoding the 64-bit AMC slot header word 
 */

#define AMCMODULE_NUM 12

#define CDFGeneralHeaderEventIndexMask  0x00FFFFFF00000000ULL //< Event index in CDF genral header word
#define CDFGeneralHeaderEventIndexShift 32                    //< lowest bit in CDF general header word

#define AMCGeneralHeaderAMCNumMask   0x00F0000000000000ULL //< Number of AMCs in AMC payload in AMC general header word
#define AMCGeneralHeaderAMCNumShift  52                    //< lowest bit in AMC general header word

#define CDFGeneralTrailerOverallSizeMask   0x00FFFFFF00000000ULL //< Ocerall size of data in CDF general trailer word
#define CDFGeneralTrailerOverallSizeShift  32                    //< lowest bit in AMC general trailer word

#define AMCHeaderLengthMask    0x4000000000000000ULL      //< More bit, bit-61 in 64-bit AMC header word 
#define AMCHeaderMoreMask      0x2000000000000000ULL      //< More bit, bit-61 in 64-bit AMC header word 
#define AMCHeaderSegMask       0x1000000000000000ULL      //< Segment bit, bit-60 in 64-bit AMC header word
#define AMCHeaderEnabledMask   0x0800000000000000ULL      //< enabled bit, bit-58 in 64-bit AMC header word
#define AMCHeaderPresentMask   0x0400000000000000ULL      //< present bit, bit-59 in 64-bit AMC header word
#define AMCHeaderValidMask     0x0200000000000000ULL      //< valid bit, bit-59 in 64-bit AMC header word
#define AMCHeaderCRCMask       0x0100000000000000ULL      //< CRC bit, bit-59 in 64-bit AMC header word
#define AMCHeaderSlotNumMask   0x00000000000F0000ULL      //< AMC Slot 4 bits, bis 16-19 in 64-bit AMC header word
#define AMCHeaderBlockNumMask  0x000000000FF00000ULL      //< AMC Block 8 bits, bis 20-27 in 64-bit AMC header word
#define AMCHeaderEventSizeMask 0x0003FFFF00000000ULL      //< AMC Event size 16 bits, bis 32-55 in 64-bit AMC header word

#define AMCHeaderLengthShift        62 //< lowest bit in AMC header word
#define AMCHeaderMoreShift          61 //< lowest bit in AMC header word
#define AMCHeaderSegShift           60 //< lowest bit in AMC header word
#define AMCHeaderEnabledShift       59 //< lowest bit in AMC header word
#define AMCHeaderPresentShift       58 //< lowest bit in AMC header word
#define AMCHeaderValidShift         57 //< lowest bit in AMC header word
#define AMCHeaderCRCShift           56 //< lowest bit in AMC header word
#define AMCHeaderSlotNumShift       16 //< lowest bit in AMC header word
#define AMCHeaderBlockNumShift      20 //< lowest bit in AMC header word
#define AMCHeaderEventSizeShift     32 //< lowest bit in AMC header word

typedef struct s_amc_header_info
{
  uint64_t AMCheaderWord64;      //< AMC module 64-bit header word 
  bool AMCLengthBit;             //< AMC length bit (bit 63) 
  bool AMCMoreBit;               //< AMC more bit (bit 61) 
  bool AMCSegBit;                //< AMC segment bit (bit 60) 
  bool AMCPresentBit;            //< AMC present bit (bit 60) 
  bool AMCEnabledBit;            //< AMC enabled bit (bit 60) 
  bool AMCValidBit;              //< AMC valid bit (bit 60) 
  bool AMCCRCBit;                //< AMC CRC bit (bit 60) 
  uint8_t AMCSlotNum;            //< AMC slot number 
  uint8_t AMCBlockNum;           //< AMC block number 
  uint32_t AMCEventSize;         //< AMC event size (either total size for Seg bit = 0 or segment size for  Seg bit = 1)
} AMC_HEADER_INFO;

EXTERN AMC_HEADER_INFO  amc_header_info[AMCMODULE_NUM];

/**
 *  frame trailer will be appended to each network packet in the
 *  free space of FRAME. This information will be used by GPU
 *  kernels to unabiguelly identify the origin of each frame.
 */
typedef struct s_sis3350_frame_trailer
{
  unsigned int fill_in_block;    //< fill number in the block of filles (4 fills per readout?)
  unsigned int adc_packet_nr;    //< ADC packet counter 
  unsigned int block_nr_tcp;     //< copy of the block number from Ack 0x80 packet
  unsigned int adc_header;       //< copy of ADC header of SIS3350 
  unsigned int adc_data_offset;  //< position of the ADC sample in the waveform 
  unsigned int adc_data_size;    //< the length of ADC data block in this frame
} SIS3350_FRAME_TRAILER;

// size of TCP buffer
#define TCP_BUF_MAX_FILLS 13

/**
 *  This structure is used to control the execution of TCP thread
 *  at to keep information on the received network packets
 */
typedef struct
{
  pthread_t   thread_id;        /* ID returned by pthread_create() */
  int               eth;        /* network device number servicing by the thread */
  pthread_mutex_t mutex;        /* = PTHREAD_MUTEX_INITIALIZER - controls thread execution */
  //int       thread_nr;        /* thread number set by the user */
  int       sis3350_num;         /* number of SIS3350 boards attached to the network interface */ 
  //unsigned int       *block_ready; /* 1 if data block received */
  unsigned int        sis3350_num_ready; /* = sis3350_num if data block received */  
  //unsigned int        blocks_avail[VME_THREAD_MAX_BLOCKS]; 
  pthread_mutex_t     mutex_data_ready; /* cleared by "MEMCOPY_HOST_TO_DEVICE" */
  //pthread_mutex_t     mutex_gpu;  /* unlocks gpu_thread */
  // *** packet buffer ***
  unsigned char *packet_space;
  unsigned int   i_next;    //< index of the next frame to be exemined
  //unsigned int num_trailers; //< number of trailers found
  // *** testing ***
  unsigned int block_size;    //< total number of bytes received in the block
  unsigned int packets_count; //< total number of pakets in the block
  //unsigned int adc_data_size_received; //< Length of ADC data received
  // parameters from block header
  unsigned int block_nr_tcp; 
  // block counter
  unsigned int block_nr;
  // general error
  unsigned int error;
  SIS3350_FRAME_TRAILER trailer;
  // ADC header
  unsigned int adc_header;
} TCP_THREAD_INFO;

EXTERN TCP_THREAD_INFO tcp_thread_info[TCP_THREAD_NUM_MAX];

// Global TCP fill buffer array, filled by TCP_thread and emptied by GPU thread (added by TG 7/26/13
EXTERN uint32_t **tcp_buf_header_gl;
EXTERN uint16_t **tcp_buf_gl;
EXTERN uint32_t **tcp_buf_tail_gl;

EXTERN  pthread_mutex_t mutex_TCP_buf_avail[TCP_BUF_MAX_FILLS]; /**< Controls access to the global TCP ring buffer */
EXTERN  pthread_mutex_t mutex_TCP_buf_ready[TCP_BUF_MAX_FILLS]; /**< Controls access to the global TCP ring buffer */

// tcp/gpu fill counter (used by ring buffer logic to count fills between BOR/EORs 
EXTERN int TCPfillnumber;

// for passing the amc13/tcp total header, data, tail size from tcp thread to gpu thread
// (the tcp thread knows the size of header, data and tail it received and assembled)
EXTERN int TCPtotalheadersize;
EXTERN int TCPtotaldatasize;
EXTERN int TCPtotaltailsize;


EXTERN unsigned char  *buf_packet_gl; /**< Global data buffer which contains received network packets. Filled by TCP threads */
EXTERN unsigned int    buf_packet_gl_n; /**< data counter in the global packet buffer */
EXTERN pthread_mutex_t buf_packet_gl_mutex; /**< Controls access to the global data buffer */

/* make functions callable from a C++ program */
#ifdef __cplusplus
extern "C" {
#endif
  EXTERN void *tcp_thread(void *data);
  // reset at BOR (begin of run)
  //EXTERN void tcp_thread_info_reset_BOR( TCP_THREAD_INFO *info );
  // reset at EOB (end of block)
  //EXTERN void tcp_thread_info_reset_EOB( TCP_THREAD_INFO *info );
  EXTERN unsigned int tcp_server_init(void);
  EXTERN unsigned int tcp_server_exit(void);
  EXTERN unsigned int tcp_server_bor(void);
  EXTERN unsigned int tcp_server_eor(void);
  EXTERN unsigned int tcp_client_init(void);
  EXTERN unsigned int tcp_client_exit(void);
  EXTERN unsigned int tcp_client_bor(void);
  EXTERN unsigned int tcp_client_eor(void);
  EXTERN unsigned int tcp_write(void);
  EXTERN int ReadXBytes( int, unsigned int, void*);
#ifdef __cplusplus
}
#endif 


#undef EXTERN
#endif /* tcp_thread_h defined */
/* tcp_thread.h ends here */
