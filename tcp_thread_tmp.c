/**
 * @file    tcp_thread.c
 * @author  Tim Gorringe <gorringe@pa.uky.edu>
 * @date    Thu May 16 07:49:29 CDT 2013
 * @date    Last-Updated: Fri May 23 15:35:39 2014 (-0400)
 *          By: Data Acquisition
 *          Update #: 1068 
 * @version $Id$
 * 
 * @copyright (c) new (g-2) collaboration
 * 
 * @brief   TCP thread 
 * 
 * @details Provides TCP communication and data readout from calo station for midas frontend
 *
 * tcp_client_init() - initiates TCP communications to calo station and launches tcp_thread to read data
 * tcp_client_exit() - closes TCP communications to calo station
 * tcp_client_bor()  - BOR functions for TCP communications, reset fill counter, ...
 * tcp_client_eor()  - EOR functions for TCP communications,
 * tcp_thread()      - reads data from TCP socket and unpacks and copies into tcp_buf_gl[bufIndex], 
 *                     tcp_buf_header_gl[bufIndex], tcp_buf_tail_gl[bufIndex] for transfer to GPU thread
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */

// Code:

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/ethernet.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/uio.h>
#include <netdb.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <poll.h>

#include <midas.h>
#include "frontend.h"
#include "timetool.h"
#include "frontend_aux.h"
#include "tcpreadout_odb.h"
#include "../CaloSimulatorAMC13/tcpsimulator_odb.h"
#define tcp_thread_c
#include "tcp_thread.h"
#undef tcp_thread_c
#ifdef USE_CALO_SIMULATOR
#include "simulator.h"
#endif                 

#ifdef DEBUG
#define dbprintf(...) printf(__VA_ARGS__)
#else
#define dbprintf(...)
#endif

/*-- Globals ---------------------------------------------------------*/

/*-- Local variables -------------------------------------------------*/

static unsigned int buf_packet_gl_max_size; ///< Max. size of the global data buffer for network packets (128 MB)

int tcp_thread_num;                         ///< number of TCP threads

int clientsockfd;                           ///< socket file descriptors

unsigned int TCPheadersizemax = 0x00100000;       ///< max header size 128 Bytes
unsigned int TCPdatasizemax = 0x08000000;   ///< max data size 128MB
unsigned int TCPtailsizemax = 0x00100000;         ///< mc trailer size 32 Bytes
//unsigned int BODdelimiter = 0x0000babe;     ///< HEX recognizable begin-of-data delimitor for emulator
//unsigned int EODdelimiter = 0xffffbabe;     ///< HEX recognizable end-of-data delimitor for emulator

//unsigned int BODdelimiter = 0x50000000;     ///< HEX recognizable begin-of-data delimitor for emulator
//unsigned int BODdelimiter = 0x40000000;     ///< HEX recognizable begin-of-data delimitor for emulator, changed 5/21/14 to fixx 0x5 or 0x4 possibility
uint64_t BODdelimiter = 0x4000000000000000ULL;     ///< HEX recognizable begin-of-data delimitor for 64-bit AMC, changed 5/22/14 to fixx 0x5 or 0x4 possibility
//unsigned int EODdelimiter = 0xa0000000;     ///< HEX recognizable end-of-data delimitor for emulator
uint64_t EODdelimiter = 0xa000000000000000ULL;     ///< HEX recognizable end-of-data delimitor for 64-bit AMC, changed 5/22/14 to fixx 0x5 or 0x4 possibility


//unsigned int BODmask = 0xf0000000;     ///< HEX recognizable begin-of-data delimitor for emulator, 
//unsigned int BODmask = 0xe0000000;     ///< HEX recognizable begin-of-data delimitor for emulator,  changed 5/21/14 to fixx 0x5 or 0x4 possibility
uint64_t BODmask = 0xe000000000000000ULL;     ///< HEX recognizable begin-of-data delimitor for 64-bit AMC header,  changed 5/22/14 to fixx 0x5 or 0x4 possibility
//unsigned int EODmask = 0xf0000000;     ///< HEX recognizable end-of-data delimitor for emulator
uint64_t EODmask = 0xf000000000000000ULL;     ///< HEX recognizable end-of-data delimitor for 64-bit AMC header,  changed 5/22/14 to fixx 0x5 or 0x4 possibility

int getEventIndex(uint64_t CDFGeneralHeaderWord);
int getAMCNum(uint64_t AMCGeneralHeaderWord);
int decodeAMCHeader(int iAMC, uint64_t AMCHeaderWord);
uint8_t SetSocketBlocking(int fd, uint8_t blocking);
void printData(unsigned int *data, unsigned int ndata);
float toddiff(struct timeval*, struct timeval*);

/*-- tcp_client_init(void) -------------------------------------------------*/                          
                                                                                                
/**                                                                                             
 * tcp_client_init(void)                                                              
 * This routine is called to:
 * (1) allocate the tcp_buf_gl buffer array of dimensions TCP_BUF_MAX_FILLS*buf_packet_gl_max_size and 
 *     corresponding header/tail buffers  
 * (2) create TCP socket (clientsockfd) and connect to TCP server in AMC13 for data transfer
 * (3) launch tcp_thread and initialize locks mutex_TCP_buf_avail[i],  mutex_TCP_buf_ready[i] controlling GPU access 
 *                                                                                              
 * @return 0 if success                                                                   
 */                                                                                             
unsigned int tcp_client_init(void)
{
  int i, status;

  dbprintf("%s(%d): allocate TCP fill buffer, size %d  \n", 
	   __func__, __LINE__, TCP_BUF_MAX_FILLS );
  buf_packet_gl_max_size = 0x08000000;   // 128 MB
  tcp_buf_gl = (uint16_t**) malloc(TCP_BUF_MAX_FILLS*sizeof(uint16_t*)); 
  for (i = 0; i < TCP_BUF_MAX_FILLS; i++){
    tcp_buf_gl[i] = (uint16_t*) malloc( buf_packet_gl_max_size );  
    if ( ! tcp_buf_gl[i] )
      {
	return FE_ERR_HW;
      }
  }
  dbprintf("%s(%d): allocate TCP fill header buffer, size %d  \n", 
	   __func__, __LINE__, TCP_BUF_MAX_FILLS );
  tcp_buf_header_gl = (uint32_t**) malloc(TCP_BUF_MAX_FILLS*sizeof(uint32_t*)); 
  for (i = 0; i < TCP_BUF_MAX_FILLS; i++){
    tcp_buf_header_gl[i] = (uint32_t*) malloc( TCPheadersizemax );  
    if ( ! tcp_buf_header_gl[i] )
      {
	return FE_ERR_HW;
      }
  }
  dbprintf("%s(%d): allocate TCP fill trailer buffer, size %d  \n", 
	   __func__, __LINE__, TCP_BUF_MAX_FILLS );
  tcp_buf_tail_gl = (uint32_t**) malloc(TCP_BUF_MAX_FILLS*sizeof(uint32_t*)); 
  for (i = 0; i < TCP_BUF_MAX_FILLS; i++){
    tcp_buf_tail_gl[i] = (uint32_t*) malloc( TCPtailsizemax );  
    if ( ! tcp_buf_tail_gl[i] )
      {
	return FE_ERR_HW;
      }
  }
  
  /* get configured network interfaces from ODB  */
  int eth_nused[TCP_THREAD_NUM_MAX];
  for (i=0; i<TCP_THREAD_NUM_MAX; i++)
    {
      eth_nused[i] = 0;
    }
  eth_nused[0] = 1; // quick fix, one interface
  
  for (i=0; i<TCP_THREAD_NUM_MAX; i++) 
    { 
      if ( eth_nused[i] == 0 ) continue;
      
      clientsockfd = socket(AF_INET, SOCK_STREAM, 0);
      //clientsockfd = socket(PF_UNSPEC, SOCK_STREAM, 0); // fix for amc13                                             
      if (clientsockfd < 0)                                                          
	{        
	  cm_msg(MERROR, __FILE__, "Cannot obtain a socket");
	  return FE_ERR_HW;                                
	}
      dbprintf("%s(%d): obtain socket, return file descriptor %d  \n",  __func__, __LINE__, clientsockfd );
   
      //UKY teststand addresses   
      //inet_aton(  "127.0.0.1", &client_bind_addr.sin_addr); // localhost
      //inet_aton("192.168.2.1", &client_bind_addr.sin_addr); // 1GbE
      //inet_aton("192.168.3.2", &client_bind_addr.sin_addr); // 10GbE
      struct sockaddr_in client_bind_addr;
      bzero((char *) &client_bind_addr, sizeof(client_bind_addr));                                        
      client_bind_addr.sin_family = AF_INET;
      //client_bind_addr.sin_family = PF_UNSPEC; //fix for amc13
      inet_aton( tcpreadout_channel_odb[i].ip_addr, &client_bind_addr.sin_addr);
      client_bind_addr.sin_port = htons(tcpreadout_channel_odb[i].port_no);
      status = bind( clientsockfd, ( struct sockaddr *) &client_bind_addr, sizeof(client_bind_addr));               
      if (status < 0 )                                                                       
	{                                                                                    
	  cm_msg(MERROR, __FILE__, "Cannot bind to socket");                                 
	  return FE_ERR_HW;                                                                  
	}                                                                                    
      dbprintf("%s(%d): socket bind, status %d  \n", __func__, __LINE__, status );
            
      struct sockaddr_in serv_addr;                                                          
      bzero((char *) &serv_addr, sizeof(serv_addr));                                         
      serv_addr.sin_family = AF_INET;
      //serv_addr.sin_family = PF_UNSPEC; // fix for amc13 testing
      inet_aton( tcpsimulator_channel_odb[i].ip_addr, &serv_addr.sin_addr);
      serv_addr.sin_port = htons(tcpsimulator_channel_odb[i].port_no);
      status = connect( clientsockfd,(struct sockaddr *) &serv_addr, sizeof(serv_addr));
      if (status < 0 )                                                                       
	{                                                                     
	  cm_msg(MERROR, __FILE__, "Cannot connect to socket");
	  return FE_ERR_HW;                                                                  
	}                         
      dbprintf("%s(%d): socket connect, client ip address %s, port no %d, status %d  \n", 
	       __func__, __LINE__, tcpreadout_channel_odb[i].ip_addr, tcpreadout_channel_odb[i].port_no, status );

      // set socket blocking-mode
      status = SetSocketBlocking( clientsockfd, 0x1);
      if ( status != 0 ){                     
	cm_msg(MERROR, __FILE__, "Cannot set socket blocking");
	//return FE_ERR_HW;                                                                  
      }                         
      dbprintf("%s(%d): set blocking socket, client ip address %s, port no %d, status %d\n", 
	       __func__, __LINE__, tcpreadout_channel_odb[i].ip_addr, tcpreadout_channel_odb[i].port_no, status);
      
    }
  
  for (i=0; i<TCP_THREAD_NUM_MAX; i++)
    { 
      if ( eth_nused[i] == 0 ) continue;
      tcp_thread_info[tcp_thread_num].eth = i;

      pthread_create(&tcp_thread_info[tcp_thread_num].thread_id, NULL, tcp_thread, (void *)(tcp_thread_info+tcp_thread_num));
      // ring buffer mutex locks
      for (i = 0; i < TCP_BUF_MAX_FILLS; i++){
	// mutex locks controlling access to TCP ring buffer, intial status of buffers are available  for filling by TCP thread 
        // ( mutex_TCP_buf_avail[i] unlocked ) and unavailable for processing by GPU thread ( mutex_TCP_buf_ready[i] locked )
	pthread_mutex_init( &mutex_TCP_buf_avail[i], 0);
	pthread_mutex_init( &mutex_TCP_buf_ready[i], 0);
	pthread_mutex_lock( &mutex_TCP_buf_ready[i] );
      }
      
      tcp_thread_num++;
    }

  return 0;
}

/*-- tcp_client_exit(void) -------------------------------------------------*/                          
                                                                                                
/**                                                                                             
 * tcp_client_exit(void)                                                              
 * (1) close TCP socket (clientsockfd) 
 *                                                                                              
 * @return 0 if success                                                                   
 */                                                                                             
unsigned int tcp_client_exit(void)
{

  dbprintf("%s(%d): close client socket, file descriptor %d  \n", __func__, __LINE__, clientsockfd );
  //SetSocketBlocking(clientsockfd,0x1);//added by wg in attempt to solve tcp connection errors when restarting fe
  close(clientsockfd);

  return 0;
}

/*-- tcp_client_bor(void) -------------------------------------------------*/                          
                                                                                                
/**                                                                                             
 * tcp_client_bor(void)
 *
 * initialize fill nummber counters
 *
 * @return 0 if success                                                                   
 */                                                                                             
unsigned int tcp_client_bor(void)
{

  TCPfillnumber = 0;
  dbprintf("%s(%d): begin-of-run TCP fill number %d\n", __func__, __LINE__, TCPfillnumber );

  return 0;
}

/*-- tcp_client_eor(void) -------------------------------------------------*/                          
                                                                                                
/**                                                                                             
 * tcp_client_eor(void)
 *
 *
 * @return 0 if success                                                                   
 */                                                                             
unsigned int tcp_client_eor(void)
{

  dbprintf("%s(%d): end-of-run TCP fill number %d\n", __func__, __LINE__, TCPfillnumber );

  return 0;
}

/*-- tcp_thread(void*) -------------------------------------------------*/                          

/**                                                                                             
 * tcp_thread(void*)
 *
 * (1) reads header size and trailer size from TCP readout ODB 
 * (2) loop over get header, data, trailer from TCP socket read with data size in header info
 * (3) check integrity of data, if OK, then copy to tcp_buf_gl[bufIndex] allow access using
 *     locks mutex_TCP_buf_avail[bufIndex], mutex_TCP_buf_ready[bufIndex]
 *
 * @return 0 if success                                                                   
 */
void *tcp_thread(void *inform)
{
  int status, itcpbuf, bufIndex;
  struct timeval tstart, theader, tdata, tmemcpy, tunlock;

  dbprintf("%s(%d): TCP thread created \n", __func__, __LINE__ );                                                               

  unsigned int TCPheadersize = tcpreadout_channel_odb[0].header_size;
  if (TCPheadersize > TCPheadersizemax) 
    {                                                                                    
      cm_msg(MERROR, __FILE__, "TCPheadersize too large");                          
      return FE_ERR_HW;                                                                  
    }
  dbprintf("%s(%d): expected header size %d\n", __func__, __LINE__, TCPheadersize );                                                               

  unsigned int TCPtailsize = tcpreadout_channel_odb[0].tail_size;
  if (TCPtailsize > TCPtailsizemax) 
    {                                                                                    
      cm_msg(MERROR, __FILE__, "TCPheadersize too large");                          
      return FE_ERR_HW;                                                                  
    }
  dbprintf("%s(%d): expected trailer size %d\n", __func__, __LINE__, TCPtailsize );                                 
  
  //unsigned int *header;
  //header = (unsigned int*) malloc( TCPheadersizemax );
  unsigned int headerbytes = TCPheadersize;
  uint64_t *header;
  header = (uint64_t*) malloc( TCPheadersizemax );
  
  unsigned short int *data;
  unsigned int databytes; // data bytes per fill feom amc13
  data = (unsigned short int*) malloc( TCPdatasizemax );
  
  //unsigned int *tail;
  //tail = (unsigned int*) malloc( TCPtailsizemax );
  unsigned int tailbytes = TCPtailsize;
  uint64_t *tail;
  tail = (uint64_t*) malloc( TCPtailsizemax );

  unsigned int EventIndex, OverallSize;
  unsigned int AMC13headerbytes = 1*sizeof(uint64_t);
  unsigned int payload; // data words per amc
  unsigned int databytestotal, blockcount; // amc data, block counters

  while ( 1 ){    // loops over AMC13 events

    bufIndex = TCPfillnumber%TCP_BUF_MAX_FILLS;
    dbprintf("%s(%d): after while ( 1 ), read TCP packets, fill %d, buffer %d \n", 
	     __func__, __LINE__, TCPfillnumber, bufIndex );
    
    // get start time
    status = gettimeofday( &tstart, NULL);
    trigger_info.time_tcp_start_read_s = tstart.tv_sec;
    trigger_info.time_tcp_start_read_us = tstart.tv_usec;
    
    dbprintf("%s(%d): fill %d \n", __func__, __LINE__, TCPfillnumber);

    // get header
    status = ReadXBytes( clientsockfd, AMC13headerbytes, (void*)(header) );
    if (status < 0) 
      {                                                                                    
	cm_msg(MERROR, __FILE__, "Cannot read header from socket");                                 
	return FE_ERR_HW;                                                                  
      }
    dbprintf("%s(%d): read header, header size [bytes] %d, header[0] 0x%016llX, BODdelimiter 0x%016llX, BODmask 0x%016llx\n", 
	     __func__, __LINE__, headerbytes, be64toh(header[0]), BODdelimiter, BODmask );
    EventIndex = getEventIndex( be64toh(header[0]) );
    dbprintf("%s(%d): reading CDF general header word, Event Index %i\n", __func__, __LINE__, EventIndex); 
    

    // get header time
    status = gettimeofday( &theader, NULL);                                                 
    dbprintf("%s(%d): duration of read header, fdt = %e us \n", __func__, __LINE__, toddiff( &theader, &tstart) );
    trigger_info.time_tcp_finish_header_read_s = theader.tv_sec;
    trigger_info.time_tcp_finish_header_read_us = theader.tv_usec;
    header[1] = theader.tv_sec; // fill header time info in header
    header[2] = theader.tv_usec; // fill header time info in header
    
    // get data, TCPIP simulator
    // unsigned int databytes = header[0]; // data size for emulator
      
    // get data, AMC, old 0x8008 firmware, unsegmented (<32KB) blocks
    // amc13 testing, uses be64toh to convert to big-endian order and elog# 416 to conver to data payload in bytes
    //payload = 0x7ff & be64toh(header[1]) ; // data words per amc
    //databytes = 2*12*payload; // data bytes per fill feom amc13
    
    // get data, AMC, new 0x8100 firmware, unsegmented (<32KB) blocks
    // amc13 testing, uses be64toh to convert to big-endian order and elog# 416 to conver to data payload in bytes
    //payload = 0x00ffffff & be64toh(header[4]) ; // data words per amc
    //databytes = 8*12*payload; // data bytes per fill feom amc13
    
    // ugly fix before proper unpacking of <32KB blocks
    //databytes = tcpreadout_channel_odb[0].data_size -
    //  tcpreadout_channel_odb[0].header_size - tcpreadout_channel_odb[0].tail_size;

    unsigned int dataarrayoffset, datablockoffset[12], dataAMCoffset[12];
    unsigned int iAMC, nAMC;
    uint64_t tmp[12];

    // count data / blocks
    databytestotal = 0; 
    blockcount = 0;
    // data pointer offset
    dataararyoffset = 0;
    memset( datablockoffset, 0, sizeof(datablockoffset) );
    memset( dataAMCoffset, 0, sizeof(dataAMCoffset) );
    // set more bit for first data segment to get in while loop
    amc_header_info[0].AMCMoreBit = 1;

    while ( amc_header_info[0].AMCMoreBit ){  // loops over AMC data blocks 

      // read single 64-bit AMC general header word
      //dbprintf("%s(%d): reading AMC general header word, blockcount %i\n", __func__, __LINE__, blockcount); 
      status = ReadXBytes( clientsockfd, sizeof(uint64_t), (void*)(tmp) );
      if (status < 0) 
	{                                                                                    
	  cm_msg(MERROR, __FILE__, "Cannot read data from socket, fd %d", clientsockfd);                                 
	  return FE_ERR_HW;                                                                  
	}
      nAMC = getAMCNum( be64toh(tmp[iAMC]) );
      nAMC = 12; // DOESNT SEEM TO HAVE CORRECT VALUE IN HEADER WORD !!!!
      dbprintf("%s(%d): reading AMC general header word, nAMC %i\n", __func__, __LINE__, nAMC); 

      // read AMC header words
      //dbprintf("%s(%d): reading AMC header word, blockcount %i\n", __func__, __LINE__, blockcount); 
      status = ReadXBytes( clientsockfd, nAMC*sizeof(uint64_t), (void*)(tmp) );
      if (status < 0) 
	{                                                                                    
	  cm_msg(MERROR, __FILE__, "Cannot read data from socket, fd %d", clientsockfd);                                 
	  return FE_ERR_HW;                                                                  
	}
      // calculate AMC data offsets from total event sizes in S=0 word AMC header word  
      //(for either M=1,S=0 with coninumation blocks or M=0,S-0 with only one block)
      if ( !amc_header_info[0].AMCSegBit )}
      int AMCoffsetbytes = 0;      
      for (iAMC = 0; iAMC < nAMC; iAMC++){
	dataAMCoffset[iAMC] = AMCoffsetbytes;
	_    AMCoffsetbytes += sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
      }

      // decode AMC header words
      for (iAMC = 0; iAMC < nAMC; iAMC++){
	if ( decodeAMCHeader( iAMC, be64toh(tmp[iAMC]) ) != 0 ){
	  printf("decodeAMCHeader() failed!");
	}
        //dbprintf("%s(%d): AMC index %d, AMCMoreBit %d, AMCEventSize 0x%08x\n", 
	//	 __func__, __LINE__, iAMC, amc_header_info[iAMC].AMCMoreBit,  amc_header_info[iAMC].AMCEventSize );
      }

      // read AMC data blocks
      for (iAMC = 0; iAMC < nAMC; iAMC++){
        if ( amc_header_info[0].AMCMoreBit && (!amc_header_info[0].AMCSegBit) )
	  {
	    //  dbprintf("M=1,S=0 first block in segment, set size to 32kB\n");
	    databytes = 32768;
	  }
        if ( amc_header_info[0].AMCMoreBit && amc_header_info[0].AMCSegBit )
	  {
	    // dbprintf("M=1,S=1 intermediate block in segment, set size from amc header word\n");
	    databytes = sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
	  }
	if ( (!amc_header_info[0].AMCMoreBit) && amc_header_info[0].AMCSegBit )
	  {
	    //  dbprintf("M=0,S=1 last block in segment, set size from amc header word\n");
	    databytes = sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
	  }
	if ( (!amc_header_info[0].AMCMoreBit) && (!amc_header_info[0].AMCSegBit) )
	  {
	    //  dbprintf("M=0,S=0 only block in segment, set size from amc header word\n");
	    databytes = sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
	  }
	  

	//dbprintf("%s(%d): M %d, S %d, databytes %d \n", __func__, __LINE__, amc_header_info[0].AMCMoreBit, amc_header_info[0].AMCSegBit, databytes); 
	//dbprintf("%s(%d): reading AMC data block, iAMC %d, blockcount %i\n", __func__, __LINE__, iAMC, blockcount); 
        
        // no reordering of data from block structure in AMC13 event
	//status = ReadXBytes( clientsockfd, databytes, (void*)(data+dataarrayoffset) );
        // reordering of data from block structure in AMC13 event
	status = ReadXBytes( clientsockfd, databytes, (void*)(data + dataAMCoffset[iAMC] + datablockoffset[iAMC]) );
	if (status < 0) 
	  {                                                                                    
	    cm_msg(MERROR, __FILE__, "Cannot read data from socket");                                 
	    return FE_ERR_HW;                                                                  
	  }
        dataarrayoffset +=  databytes/sizeof(uint16_t);
	datablockoffset[iAMC] +=  databytes/sizeof(uint16_t);
	databytestotal +=  databytes;
      }

      // read single 64-bit AMC general trailer word
      //dbprintf("%s(%d): reading AMC general trailer word, blockcount %i\n", __func__, __LINE__, blockcount); 
      status = ReadXBytes( clientsockfd, sizeof(uint64_t), (void*)(tmp) );
      if (status < 0) 
	{                                                                                    
	  cm_msg(MERROR, __FILE__, "Cannot read data from socket, fd %d", clientsockfd);                                 
	  return FE_ERR_HW;                                                                  
	}
      dbprintf("%s(%d): done reading AMC block %i\n", __func__, __LINE__, blockcount); 
      blockcount++;
    }

    dbprintf("%s(%d): finished data read data count 0x%08x blockcount %i\n", __func__, __LINE__, databytestotal, blockcount); 

    // get data time
    status = gettimeofday( &tdata, NULL);
    dbprintf("%s(%d): duration of read, fdt = %e us \n", __func__, __LINE__, toddiff( &tdata, &theader) );
    trigger_info.time_tcp_finish_data_read_s = tdata.tv_sec;
    trigger_info.time_tcp_finish_data_read_us = tdata.tv_usec;
    header[3] = tdata.tv_sec; // fill data time info in header
    header[4] = tdata.tv_usec; // fill data time info in header
    
    // get trailer
    status = ReadXBytes( clientsockfd, tailbytes, (void*)(tail));
    if (status < 0) 
      {                                                                                    
	cm_msg(MERROR, __FILE__, "Cannot read tail from socket");                                 
	return FE_ERR_HW;                                                                  
      }
    dbprintf("%s(%d): read trailer, trailer size [bytes] %d, tail[0] 0x%016llX, EODdelimiter 0x%016llX, EODmask 0x%016llX\n", 
	     __func__, __LINE__, tailbytes, be64toh(tail[0]), EODdelimiter, EODmask);
    OverallSize = getOverallSize( be64toh(tail[0]) );
    dbprintf("%s(%d): reading CDF general trailer word, Overall Size %i\n", __func__, __LINE__, OverallSize); 

    // check data integrity, if OK, then copy TCP buffer into global buffer 
    dbprintf("%s(%d): integrity data, BOD:  0x%016llX = 0x%016llX  ? EOD:  0x%016llX = 0x%016llX  ? \n", __func__, __LINE__, 
	     ( be64toh(header[0]) & BODmask ), BODdelimiter, ( be64toh(tail[0]) & EODmask ),  EODdelimiter  );
    
    //if ( ( ( be64toh(header[0]) & BODmask ) == BODdelimiter ) && ( ( be64toh(tail[0]) & EODmask ) == EODdelimiter ) ) // check both EOD and BOD 
    if ( ( be64toh(tail[0]) & EODmask ) == EODdelimiter  ) // only check EOD 
      {
	dbprintf("%s(%d): PASS data integrity check, buffer %d, fill %d \n", 
		 __func__, __LINE__, bufIndex, TCPfillnumber );

	// get access to ring buffer
	dbprintf("%s(%d): lock ring buffer[%d] AVAIL buffer, fill %d\n", __func__, __LINE__, bufIndex, TCPfillnumber);
	pthread_mutex_lock( &mutex_TCP_buf_avail[bufIndex] );
	
	// copied data directly to tcp_buf_gl[bufIndex] to save time
	// the tcp_thread received and assembled header, data, tail sizes are passed to gpu_thread 
	dbprintf("%s(%d): fill ring buffer[%d] AVAIL buffer, fill %d\n", __func__, __LINE__, bufIndex, TCPfillnumber);
        // TCPtotalheadersize, TCPtotaldatasize, TCPtotaltailsize are used to tell the gpu_thread the data sizes 
	TCPtotalheadersize = headerbytes;
	memcpy(tcp_buf_header_gl[bufIndex], header, TCPtotalheadersize );
        TCPtotaldatasize = databytestotal;
	memcpy(tcp_buf_gl[bufIndex], data, TCPtotaldatasize );
        TCPtotaltailsize = tailbytes;
	memcpy(tcp_buf_tail_gl[bufIndex], tail, TCPtotaltailsize );
	dbprintf("%s(%d): copied %d, %d, %d, header, data, tail bytes\n", __func__, __LINE__, headerbytes , databytestotal, tailbytes );
	
	// release access to ring buffer
	pthread_mutex_unlock( &mutex_TCP_buf_ready[bufIndex] ); // exclude for testing -> no unlocking in gpu_thread
	dbprintf("%s(%d): unlock ring buffer[%d] AVAIL buffer, fill %d\n", __func__, __LINE__, bufIndex, TCPfillnumber);
	
	// get memcpy time
	status = gettimeofday( &tmemcpy, NULL);                                                 
	dbprintf("%s(%d): duration of memcpy, fdt = %e us \n", __func__, __LINE__, toddiff( &tmemcpy, &tdata) );
	
	// get unlock time
	status = gettimeofday( &tunlock, NULL);                                                 
	dbprintf("%s(%d): duration of unlock, fdt = %e us \n", __func__, __LINE__, toddiff( &tunlock, &tmemcpy) );
	
	TCPfillnumber++;

      } 
    else 
      {
	dbprintf("%s(%d): FAIL data integrity check, buffer %d, fill %d !!!\n", 
		 __func__, __LINE__, bufIndex, TCPfillnumber );
	// need to figure out the recovery scheme after integrity failure
      }
            
  } // end of while ( 1 )
  
  // release memory for both successful and unsuccessful event reconstruction
  free(header);
  dpprintf("%s(%d): done header free()\n", __func__, __LINE__ );
  dpfree(data);
  printf("%s(%d): done data free()\n", __func__, __LINE__ );
  free(tail);	
  dpprintf("%s(%d): done tail free()\n", __func__, __LINE__ );

}

/*-- ReadXBytes(int socket, unsigned int x, void* buffer) --------------------------*/                          

/**                                                                                             
 * decodeAMCHeader(uint64_t AMCHeaderWord)
 *
 * unpacks a 64-bit AMC header word into More, Segment bits, Slot, Block numbers, and event size
 * contained in the structure amc_header_info. Maybe should return a pointer to structure and 
 * leep the scope to the tcp_thread) function.
 *
 * @return 0 if success                                                                   
 */                                                                             
int decodeAMCHeader(int i, uint64_t AMCHeaderWord){

  printf("%s(%d): index %d, AMCHeaderWord 0x%016llX\n", __func__, __LINE__, i, AMCHeaderWord);
 
  amc_header_info[i].AMCLengthBit = (bool) ( ( AMCHeaderWord & AMCHeaderLengthMask ) >> AMCHeaderLengthShift );
  amc_header_info[i].AMCMoreBit = (bool) ( ( AMCHeaderWord & AMCHeaderMoreMask ) >> AMCHeaderMoreShift );
  amc_header_info[i].AMCSegBit = (bool) ( ( AMCHeaderWord & AMCHeaderSegMask ) >> AMCHeaderSegShift );
  amc_header_info[i].AMCEnabledBit = (bool) ( ( AMCHeaderWord & AMCHeaderEnabledMask ) >> AMCHeaderEnabledShift );
  amc_header_info[i].AMCPresentBit = (bool) ( ( AMCHeaderWord & AMCHeaderPresentMask ) >> AMCHeaderPresentShift );
  amc_header_info[i].AMCCRCBit = (bool) ( ( AMCHeaderWord & AMCHeaderCRCMask ) >> AMCHeaderCRCShift );
  amc_header_info[i].AMCValidBit = (bool) ( ( AMCHeaderWord & AMCHeaderValidMask ) >> AMCHeaderValidShift );
  amc_header_info[i].AMCSlotNum = (uint8_t) ( ( AMCHeaderWord & AMCHeaderSlotNumMask ) >> AMCHeaderSlotNumShift ); 
  amc_header_info[i].AMCBlockNum = (uint8_t) ( ( AMCHeaderWord & AMCHeaderBlockNumMask ) >> AMCHeaderBlockNumShift ); 
  amc_header_info[i].AMCEventSize = (uint32_t) ( ( AMCHeaderWord & AMCHeaderEventSizeMask ) >> AMCHeaderEventSizeShift ); 

  return 0;
}

/**                                                                                             
 * getEventIndex(uint64_t CDFGeneralHeaderWord)
 *
 * gets event index from CDF gneral header word
 *
 * @return 0 if success                                                                   
 */                                             
int getEventIndex(uint64_t CDFGeneralHeaderWord){

  printf("%s(%d): CDFGeneralHeaderWord 0x%016llX\n", __func__, __LINE__, CDFGeneralHeaderWord);
  uint32_t EventNum = (uint32_t) ( ( CDFGeneralHeaderWord & CDFGeneralHeaderEventIndexMask ) >> CDFGeneralHeaderEventIndexShift );

  return EventNum;
}

/**                                                                                             
 * getOverallSize(uint64_t CDFGeneralHeaderWord)
 *
 * gets event index from CDF gneral header word
 *
 * @return 0 if success                                                                   
 */                                             
int getOverallSize(uint64_t CDFGeneralTrailerWord){

  printf("%s(%d): CDFGeneralHeaderWord 0x%016llX\n", __func__, __LINE__, CDFGeneralTrailerWord);
  uint32_t OverallSize = (uint32_t) ( ( CDFGeneralTrailerWord & CDFGeneralTrailerOverallSizeMask ) >> CDFGeneralTrailerOverallSizeShift );

  return OverallSize;
}

/**                                                                                             
 * getAMCNum(uint64_t AMCGeneralHeaderWord)
 *
 * gets number of AMCs from  64-bit AMC general header word
 *
 * @return 0 if success                                                                   
 */                                                                             
int getAMCNum(uint64_t AMCGeneralHeaderWord){

  printf("%s(%d): AMCGeneralHeaderWord 0x%016llX\n", __func__, __LINE__, AMCGeneralHeaderWord);
  uint32_t AMCNum = (uint32_t) ( ( AMCGeneralHeaderWord & AMCGeneralHeaderAMCNumMask ) >> AMCGeneralHeaderAMCNumShift );

  return AMCNum;
}


/**                                                                                             
 * ReadXBytes(int socket, unsigned int x, void* buffer)
 *
 * reads x bytes on file descriptor socket and place in buffer
 *
 * @return 0 if success                                                                   
 */                                                                             
int ReadXBytes(int socket, unsigned int x, void* buffer)
{
  int status, bytesRead = 0, result, tries = 0;

  while (bytesRead < x)
    {
      result = read(socket, buffer + bytesRead, x - bytesRead);
      if (result < 1 )
        {
	  printf("ReadXBytes: warning read return code %d, errno %d\n", result, errno);
        }
      bytesRead += result;
      tries++;
    }
  //dbprintf("%s(%d): socket file descriptor %d, request %d bytes, read %d bytes, tries %d\n", __func__, __LINE__, socket, x, bytesRead, tries);

  return 0;
}
              
/*--  SetSocketBlocking(int fd, uint8_t blocking) --------------------------*/                          
/**                                                                                             
 * SetSocketBlocking(int fd, uint8_t blocking)
 *
 * A helper function to change the blocking/non-blocking status of a socket
 * first get status flags, then change status flags via  F_GETFL,  F_SETFL flags
 * @return 0 if fcntl() success                                                                   
 */
uint8_t SetSocketBlocking(int fd, uint8_t blocking) { 

  int status;              
  dbprintf("%s(%d): blocking , O_NONBLOCK  0x%08x , 0x%08x\n", __func__, __LINE__, blocking, O_NONBLOCK);

  if (fd < 0)   
    {           
      return 0x0;                                 
    }           
  int flags = fcntl(fd, F_GETFL, 0);
  dbprintf("%s(%d): fd %d, flags before setting 0x%08x\n",  __func__, __LINE__, fd, flags);

  if (flags < 0)
    {           
      return 0x0;                                 
    }           
  flags = blocking ? (flags&~O_NONBLOCK) : (flags|O_NONBLOCK);                          
  dbprintf("%s(%d): fd %d, flags after setting 0x%08x\n",  __func__, __LINE__, fd, flags);

  status = fcntl(fd, F_SETFL, flags);
  dbprintf("%s(%d): fcntl status %d, errno %d\n",  __func__, __LINE__, status, errno);

  return status;
}

/*-- printData(unsigned int *data, unsigned int ndata) --------------------------*/                          

/**                                                                                             
 * printData(unsigned int *data, unsigned int ndata)
 *
 * helper function to print data in data buffer
 *
 * @return 0 if success                                                                   
 */                                                                             
void printData(unsigned int *data, unsigned int ndata) {
  int i; 

  dbprintf("%s(%d): ndata = %d \n", __func__, __LINE__, ndata);
  for (i = 0; i < ndata; ++i)
    printf("%s(%d): data[%d] = %d \n",__func__, __LINE__, i, data[i]);

  return 0;
}

/*--  toddiff(struct timeval *tod2, struct timeval *tod1) --------------------------*/

/**                                                                                             
 * toddiff(struct timeval *tod2, struct timeval *tod1)
 *
 * help function to return time difference in microseconds between two time-of-day structures
 *
 * @return 0 if success                                                                   
 */                                                                             
float toddiff(struct timeval *tod2, struct timeval *tod1) {
  float fdt, fmudt;
  long long t1, t2, mut1, mut2, dt, mudt;

  t1 = tod1->tv_sec;
  mut1 = tod1->tv_usec;
  t2 = tod2->tv_sec;
  mut2 = tod2->tv_usec;
  dt = ( t2 - t1);
  mudt = ( mut2 - mut1 );
  fdt = (float)dt;
  fmudt = (float)mudt;
  if ( fmudt < 0 )                                                                  
    {                                                                               
      fdt -= 1.0;                                                                    
      fmudt += 1000000.;                                                             
    }                                                                   
  
  return 1.0e6*fdt + fmudt;                                                                                                          
}
