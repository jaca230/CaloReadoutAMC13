/* tcp_thread.h --- 
 * 
 * Filename:          tcp_thread.h
 * Description: 
 * Author:            Tim Gorringe
 * Maintainer:        Tim Gorringe
 * Created:           Thu May 16 07:49:29 CDT 2013
 * Version:           $id$
 * Last-Updated: Fri Feb 17 14:41:53 2017 (-0500)
 *           By: Data Acquisition
 *     Update #: 67
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

// max number of TCP threads
#define TCP_THREAD_NUM_MAX  1

// number of TCP threads
extern int tcp_thread_num;

// offset for timing  info in header bank
//int iheadertimeoffset; 
//int iheadertimeoffset = 0x100; // put timing data at end of BCnn
extern int iheadertimeoffset;

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

#define AMCHeaderLengthMask    0x4000000000000000ULL      //< length bit, bit-62 in 64-bit AMC header word 
#define AMCHeaderMoreMask      0x2000000000000000ULL      //< More bit, bit-61 in 64-bit AMC header word 
#define AMCHeaderSegMask       0x1000000000000000ULL      //< Segment bit, bit-60 in 64-bit AMC header word
#define AMCHeaderEnabledMask   0x0800000000000000ULL      //< enabled bit, bit-59 in 64-bit AMC header word
#define AMCHeaderPresentMask   0x0400000000000000ULL      //< present bit, bit-58 in 64-bit AMC header word
#define AMCHeaderValidMask     0x0200000000000000ULL      //< valid bit, bit-57 in 64-bit AMC header word
#define AMCHeaderCRCMask       0x0100000000000000ULL      //< CRC bit, bit-56 in 64-bit AMC header word
#define AMCHeaderSlotNumMask   0x00000000000F0000ULL      //< AMC Slot 4 bits, bits 16-19 in 64-bit AMC header word
#define AMCHeaderBlockNumMask  0x000000000FF00000ULL      //< AMC Block 8 bits, bits 20-27 in 64-bit AMC header word
#define AMCHeaderEventSizeMask 0x00FFFFFF00000000ULL      //< AMC Event size 16 bits, bits 32-55 in 64-bit AMC header word

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

#define AMC13HeaderDataOffset     0x40 //< offset for AMC13 header/trailer data in CBnn midas databank
#define RiderHeaderDataOffset     0x800 //< offset for Rider header/trailer data in CBnn midas databank

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

//extern AMC_HEADER_INFO  amc_header_info[AMCMODULE_NUM];

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
#define TCP_BUF_MAX_FILLS 32

/**
 *  This structure is used to control the execution of TCP thread
 *  at to keep information on the received network packets
 */
typedef struct
{
  pthread_t   thread_id;        /* ID returned by pthread_create() */
  int               eth;        /* network device number servicing by the thread */
  pthread_mutex_t mutex;        /* = PTHREAD_MUTEX_INITIALIZER - controls thread execution */
  //int       sis3350_num;         /* number of SIS3350 boards attached to the network interface */ 
  //unsigned int       *block_ready; /* 1 if data block received */
  //unsigned int        sis3350_num_ready; /* = sis3350_num if data block received */  
  //unsigned int        blocks_avail[VME_THREAD_MAX_BLOCKS]; 
  pthread_mutex_t     mutex_data_ready; /* cleared by "MEMCOPY_HOST_TO_DEVICE" */
  //pthread_mutex_t     mutex_gpu;  /* unlocks gpu_thread */
  // *** packet buffer ***
  //unsigned char *packet_space;
  //unsigned int   i_next;    //< index of the next frame to be exemined
  //unsigned int num_trailers; //< number of trailers found
  // *** testing ***
  //unsigned int block_size;    //< total number of bytes received in the block
  //unsigned int packets_count; //< total number of pakets in the block
  //unsigned int adc_data_size_received; //< Length of ADC data received
  // parameters from block header
  //unsigned int block_nr_tcp; 
  // block counter
  //unsigned int block_nr;
  // general error
  unsigned int error;
  //SIS3350_FRAME_TRAILER trailer;
  // ADC header
  unsigned int adc_header;
} TCP_THREAD_INFO;

//extern TCP_THREAD_INFO tcp_thread_info[TCP_THREAD_NUM_MAX];

// for passing the amc13/tcp total header, data, tail size from tcp thread to gpu thread
// (the tcp thread knows the size of header, data and tail it received and assembled)
// is array as ring buffer can store multiple fills
extern int TCPtotalheadersize[TCP_BUF_MAX_FILLS];
extern int TCPtotalamc13infosize[TCP_BUF_MAX_FILLS];
extern int TCPtotaldatasize[TCP_BUF_MAX_FILLS];
extern int TCPtotaltailsize[TCP_BUF_MAX_FILLS];


// Global TCP fill buffer array, filled by TCP_thread and emptied by GPU thread (added by TG 7/26/13
extern uint64_t **tcp_buf_header_gl;
extern uint64_t **tcp_buf_amc13_gl;
extern uint64_t **tcp_buf_gl;
extern uint64_t **tcp_buf_tail_gl;

extern pthread_mutex_t mutex_TCP_buf[TCP_BUF_MAX_FILLS]; /**< Controls access to the global TCP ring buffer */
extern pthread_mutex_t mutex_TCP_general;

// tcp/gpu fill counter (used by ring buffer logic to count fills between BOR/EORs 
extern unsigned long TCPfillnumber;

extern int tcp_thread_active;
//extern int tcp_thread_read;


// switch to turn on/off printing the ReadXBytes error message only once each run 
//extern bool ReadXBytesErrMessageSwitch;

//Not used in run3, delete later
//extern unsigned char  *buf_packet_gl; /**< Global data buffer which contains received network packets. Filled by TCP threads */
//extern unsigned int    buf_packet_gl_n; /**< data counter in the global packet buffer */
//extern pthread_mutex_t buf_packet_gl_mutex; /**< Controls access to the global data buffer */

  void *tcp_thread(void *data);
  // reset at BOR (begin of run)
  //void tcp_thread_info_reset_BOR( TCP_THREAD_INFO *info );
  // reset at EOB (end of block)
  //void tcp_thread_info_reset_EOB( TCP_THREAD_INFO *info );
  unsigned int tcp_server_init(void);
  unsigned int tcp_server_exit(void);
  unsigned int tcp_server_bor(void);
  unsigned int tcp_server_eor(void);
  unsigned int tcp_client_init(void);
  unsigned int tcp_client_exit(void);
  unsigned int tcp_client_bor(void);
  unsigned int tcp_client_eor(void);
  unsigned int tcp_write(void);
  int ReadXBytes( int, unsigned int, void*,int &);

#endif /* tcp_thread_h defined */
/* tcp_thread.h ends here */
