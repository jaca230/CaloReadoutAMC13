/*
 * @file    tcp_thread.c
 * @author  Tim Gorringe <gorringe@pa.uky.edu>
 * @date    Thu May 16 07:49:29 CDT 2013
 * @date    Last-Updated: Fri Feb 17 14:43:30 2017 (-0500)
 *          By: Data Acquisition
 *          Update #: 1469 
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
 * tcp_thread()      - rleads data from TCP socket and unpacks and copies into tcp_buf_gl[bufIndex], 
 *                     tcp_buf_amc13_gl[bufIndex],tcp_buf_header_gl[bufIndex], tcp_buf_tail_gl[bufIndex] for transfer to GPU thread
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
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/uio.h>
#include <netdb.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <sstream>
#include <pthread.h>
#include <poll.h>

#include <midas.h>
#include <mfe.h>
#include "frontend.h"
#include "timetool.h"
#include "frontend_aux.h"
#include "amc13_odb.h"
#include "tcp_thread.h"
#include "gpu_thread.h"
#ifdef USE_CALO_SIMULATOR
#include "simulator.h"
#endif

//#define DEBUG 1

#ifdef DEBUG
#define dbprintf(...) printf(__VA_ARGS__)
#else
#define dbprintf(...)
#endif

int tcp_thread_num;                         ///< number of TCP threads 
int clientsockfd;                           ///< socket file descriptors

unsigned int TCPheadersize, TCPamc13infosize, TCPtailsize, TCPdatasize;
unsigned int headerbytes, tailbytes;
int databytes;  //This is signed because it can be the error code of the readAndUnpack function (R. Hong)
uint64_t amc13infobytes;
uint64_t *header; ///< temporary buffer for unpacking header
uint64_t *amc13info; ///< temporary buffer for unpacking amc13info
uint64_t *tail; ///< temporary buffer for unpacking tail
int16_t *data; ///< temporary buffer for unpacking data

uint64_t *offsetamc13info; // used in readAndUnpack() for unpacking amc13 header / trailers

struct timeval tstart, theader, tdata, tmemcpy, tunlock, tbeforeavaillock, tafteravaillock, tbeginread, tfinishread; ///< timing info
//int iheadertimeoffset = 0x3fe0; // put timing data at end of BCnn
//int iheadertimeoffset = 0x100; // put timing data at end of BCnn
int iheadertimeoffset = 0x100; // put timing data at end of BCnn

unsigned int TCPheadersizemax = 0x00100000;     ///< max header size 128 Bytes
unsigned int TCPamc13infosizemax = 0x00100000;     ///< max header size 128 Bytes
unsigned int TCPtailsizemax = 0x00200000;       ///< mc trailer size 32 Bytes
unsigned int TCPdatasizemax = 0x08000000;       ///< max data size 128MB
uint64_t BODdelimiter = 0x4000000000000000ULL;  ///< 64-bit AMC13 begin-of-data delimitor
uint64_t EODdelimiter = 0xa000000000000000ULL;  ///< 64-bit AMC13 end-of-data delimitor 
uint64_t BODmask = 0xe000000000000000ULL;       ///< 64-bit AMC13 begin-of-data mask
uint64_t EODmask = 0xf000000000000000ULL;       ///< 64-bit AMC13 end-of-data mask

int NRiderModuleMax = 12; // max number of riders 
int NRiderModuleEnabled = 0; // number of enablde riders

AMC_HEADER_INFO  amc_header_info[AMCMODULE_NUM];
TCP_THREAD_INFO tcp_thread_info[TCP_THREAD_NUM_MAX];

// Global TCP fill buffer array, filled by TCP_thread and emptied by GPU thread (added by TG 7/26/13
uint64_t **tcp_buf_header_gl;
uint64_t **tcp_buf_amc13_gl;
uint64_t **tcp_buf_gl;
uint64_t **tcp_buf_tail_gl;

pthread_mutex_t mutex_TCP_buf[TCP_BUF_MAX_FILLS]; /**< Controls access to the global TCP ring buffer */
pthread_mutex_t mutex_TCP_general; //General mutex lock for the TCP thread

int tcp_thread_active = 0;
int tcp_run_active = 0;
bool read_error;

// tcp/gpu fill counter (used by ring buffer logic to count fills between BOR/EORs 
unsigned long TCPfillnumber;

// switch to turn on/off printing the ReadXBytes error message only once each run 
bool ReadXBytesErrMessageSwitch;

// for passing the amc13/tcp total header, data, tail size from tcp thread to gpu thread
// (the tcp thread knows the size of header, data and tail it received and assembled)
// is array as ring buffer can store multiple fills
int TCPtotalheadersize[TCP_BUF_MAX_FILLS];
int TCPtotalamc13infosize[TCP_BUF_MAX_FILLS];
int TCPtotaldatasize[TCP_BUF_MAX_FILLS];
int TCPtotaltailsize[TCP_BUF_MAX_FILLS];

int readAndUnpack(int bufIndex); // read and unpack the AMC13 data on TCP socket
uint8_t SetSocketBlocking(int fd, uint8_t blocking);     // set socket blocking / non-blocking
int getEventIndex(uint64_t CDFGeneralHeaderWord);        // get fill number from AMC13 CDF header word
int getAMCNum(uint64_t AMCGeneralHeaderWord);            // get # AMCs from AMC13 CDF header word
int getOverallSize(uint64_t CDFGeneralTrailerWord);      // get overall size of AMC13 event from trailer word
int decodeAMCHeader(int iAMC, uint64_t AMCHeaderWord);   // get AMC13 block structure decoding bits
void printData(unsigned int *data, unsigned int ndata);  // debugging print statment for AMC13 data
float toddiff(struct timeval*, struct timeval*);         // compute time difference between pair of gettimeofday()'s

//extern INT frontend_index;

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

  // initilize bool for printing of ReadXBytes Error message
  ReadXBytesErrMessageSwitch = false;

  long int total_size = 0;

  dbprintf("%s(%d): allocate TCP fill header buffer, buffer size %d, array size (bytes) %d\n", 
	   __func__, __LINE__, TCP_BUF_MAX_FILLS, TCPheadersizemax );
  tcp_buf_header_gl = (uint64_t**) malloc(TCP_BUF_MAX_FILLS*sizeof(uint64_t*)); 
  for (i = 0; i < TCP_BUF_MAX_FILLS; i++){
    tcp_buf_header_gl[i] = (uint64_t*) malloc( TCPheadersizemax );  

    total_size += TCPheadersizemax;
    if ( ! tcp_buf_header_gl[i] )
      {
	return -1;
      }
  }

  dbprintf("%s(%d): allocate TCP fill amc13 buffer, buffer size %d, array size (bytes) %d\n", 
	   __func__, __LINE__, TCP_BUF_MAX_FILLS, TCPamc13infosizemax );
  tcp_buf_amc13_gl = (uint64_t**) malloc(TCP_BUF_MAX_FILLS*sizeof(uint64_t*)); 
  for (i = 0; i < TCP_BUF_MAX_FILLS; i++){
    tcp_buf_amc13_gl[i] = (uint64_t*) malloc( TCPamc13infosizemax );  
    total_size += TCPamc13infosizemax;
    if ( ! tcp_buf_amc13_gl[i] )
      {
	return -1;
      }
  }

  dbprintf("%s(%d): allocate TCP fill trailer buffer, buffer size %d, array size (bytes) %d\n", 
	   __func__, __LINE__, TCP_BUF_MAX_FILLS, TCPtailsizemax );
  tcp_buf_tail_gl = (uint64_t**) malloc(TCP_BUF_MAX_FILLS*sizeof(uint64_t*)); 
  for (i = 0; i < TCP_BUF_MAX_FILLS; i++){
    tcp_buf_tail_gl[i] = (uint64_t*) malloc( TCPtailsizemax );  
    total_size += TCPtailsizemax;
    if ( ! tcp_buf_tail_gl[i] )
      {
	return -1;
      }
  }

  dbprintf("%s(%d): allocate TCP fill data buffer, buffer size %d, array size (bytes) %d\n", 
	   __func__, __LINE__, TCP_BUF_MAX_FILLS, TCPdatasizemax );
  tcp_buf_gl = (uint64_t**) malloc(TCP_BUF_MAX_FILLS*sizeof(uint64_t*)); 
  for (i = 0; i < TCP_BUF_MAX_FILLS; i++){
    tcp_buf_gl[i] = (uint64_t*) malloc( TCPdatasizemax );  
    total_size += TCPdatasizemax;
    if ( ! tcp_buf_tail_gl[i] )
    if ( ! tcp_buf_gl[i] )
      {
	return -1;
      }
  }

  printf("TCP total buffer size %ld \n",total_size);
  
  // get enabled network interfaces from ODB (to do)
  int eth_nused[TCP_THREAD_NUM_MAX];
  for (i=0; i<TCP_THREAD_NUM_MAX; i++)
    {
      eth_nused[i] = 1;
    }
  
  // configure network interfaces from ODB (to do)
  for (i=0; i<TCP_THREAD_NUM_MAX; i++) 
    { 
      if ( eth_nused[i] == 0 ) continue;
      
      clientsockfd = socket(AF_INET, SOCK_STREAM, 0);
      if (clientsockfd < 0)                                                          
	{        
	  cm_msg(MERROR, __FILE__, "Cannot obtain a socket");
	  return -1;                                
	}
      dbprintf("%s(%d): obtain client socket, return file descriptor %d \n",  __func__, __LINE__, clientsockfd);

      // Set the size in bytes of the receive buffer for large events.  Default values  
      // are set in /proc/sys/net/core/rmem_default and /proc/sys/net/core/rmem_max
      // use /sbin/sysctl -w net.ipv4.tcp_rmem='4096 87380 536870912' to modify
      
      int iSocketOption = 0x20000000, iSocketOptionGet = 0; 
      socklen_t iSocketOptionLen = sizeof(int);

      if ( setsockopt( clientsockfd, SOL_SOCKET, SO_RCVBUF, (const char *) &iSocketOption, iSocketOptionLen ) < 0 ){
         cm_msg(MERROR, __FILE__, "Cannot set socket buffer size");
         printf("Cannot set socket buffer size \n");
         return -1;
      }
      dbprintf("%s(%d): set socket buffer size %d \n",  __func__, __LINE__, iSocketOption);

      if ( getsockopt( clientsockfd, SOL_SOCKET, SO_RCVBUF, (char *)&iSocketOptionGet, &iSocketOptionLen ) < 0){
         cm_msg(MERROR, __FILE__, "Cannot get socket buffer size");
         printf("Cannot get socket buffer size \n");
         return -1;
      } 
       dbprintf("%s(%d): get socket buffer size %d \n",  __func__, __LINE__, iSocketOptionGet);

       
      // set ip address, port no., etc, from ODB of server-side (AMC13) 10 GbE link in socket address structure
      struct sockaddr_in serv_addr;                                                          
      bzero((char *) &serv_addr, sizeof(serv_addr));                              
      serv_addr.sin_family = AF_INET;
      inet_aton(amc13_link_odb[i].source_ip, &serv_addr.sin_addr);
      serv_addr.sin_port = htons(amc13_link_odb[i].source_port);

      dbprintf("%s(%d): connect client at address %#010x  port %#010x\n",  __func__, __LINE__, serv_addr.sin_addr.s_addr, serv_addr.sin_port);
      status = connect( clientsockfd,(struct sockaddr *) &serv_addr, sizeof(serv_addr));
      if (status < 0) {
         perror( __func__ );
         cm_msg(MERROR, __FILE__, "Cannot connect to socket");
         return -1;
      }
      
      struct sockaddr_in client_addr;
      socklen_t addrlen = sizeof(client_addr);
#ifdef DEBUG
      int local_port = -1;
      std::string default_ip("-1");
      char *client_ip = &default_ip[0];
#endif
      if (getsockname(clientsockfd, (struct sockaddr *)&client_addr, &addrlen) == 0 &&
	  client_addr.sin_family == AF_INET &&
	  addrlen == sizeof(client_addr)) {
#ifdef DEBUG
	local_port = ntohs(client_addr.sin_port);
	client_ip = inet_ntoa(client_addr.sin_addr);
#endif
      }

      dbprintf("%s(%d): socket connect, client ip address %s, port no %i, server ip address %s, port no %d, status %d \n", 
	       __func__, __LINE__, client_ip, local_port, amc13_link_odb[i].source_ip, amc13_link_odb[i].source_port, status);
      
      // check MTU size is 9000
      int mtu_size = -1;
      socklen_t mlen = sizeof(mtu_size);
      getsockopt(clientsockfd, 0, IP_MTU, &mtu_size, &mlen);
      dbprintf("%s(%d): client MTU %i \n", __func__, __LINE__, mtu_size);
      if (mtu_size != -1 && mtu_size < 9000) {
	cm_msg(MERROR, __FILE__, "Client MTU Conflict: Read %i, Expected 9000", mtu_size);
        return -1;
      }

      // set socket in blocking mode
      //status = SetSocketBlocking(clientsockfd, 0x1);
      // set socket in non-blocking mode
      status = SetSocketBlocking(clientsockfd, 0x0);
      if ( status != 0 )
	{                     
	  cm_msg(MERROR, __FILE__, "Cannot set socket blocking");
	  return -1;                                                                  
	}                         
      dbprintf("%s(%d): set blocking socket, client ip address %s, port no %i, status %d \n",
	       __func__, __LINE__, client_ip, local_port, status);

      /* Check the status for the keepalive option */
      int optval;
      socklen_t optlen = sizeof(optval);
      if(getsockopt( clientsockfd, SOL_SOCKET, SO_KEEPALIVE, &optval, &optlen) < 0) {
	perror("getsockopt()");
      }
      printf("SO_KEEPALIVE is %s \n", (optval ? "ON" : "OFF"));

      /* Set the option active */
      //Run4 development: try not to have the keep_alive option. The connection may not crash after >2 hours of idle time.
     /* 
      optval = 1;
      optlen = sizeof(optval);
      if(setsockopt( clientsockfd, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
	perror("setsockopt()");
      }
      printf("SO_KEEPALIVE set on socket \n");
*/
      /* Check the status again */
      if(getsockopt( clientsockfd, SOL_SOCKET, SO_KEEPALIVE, &optval, &optlen) < 0) {
	perror("getsockopt()");
      }
      printf("SO_KEEPALIVE is %s \n", (optval ? "ON" : "OFF"));

    } // TCP thread loop
  
  TCPfillnumber = 0;
  // launch tcp_threads for each 10 Gbe link (allow multiple threads for future functionality)

  tcp_thread_active = 1;
  tcp_run_active = 0;

  for (i=0; i<TCP_THREAD_NUM_MAX; i++)
    { 
      if ( eth_nused[i] == 0 ) continue;

      tcp_thread_info[tcp_thread_num].eth = i;
      pthread_create(&tcp_thread_info[tcp_thread_num].thread_id, NULL, tcp_thread, (void *)(tcp_thread_info+tcp_thread_num));

      // mutex locks control access to TCP ring bufferS, intial status of buffers are available  for filling by TCP thread 
      for (i = 0; i < TCP_BUF_MAX_FILLS; i++) {
	pthread_mutex_init( &mutex_TCP_buf[i], 0 );
      }
      tcp_thread_num++;
    }
  pthread_mutex_init( &mutex_TCP_general, 0 );

  return 0;
}

/*-- tcp_client_exit(void) -------------------------------------------------*/                          
                                                                                                
/**                                                                                             
 * tcp_client_exit(void)                                                              
 * (1) close TCP socket (clientsockfd) 
 *      bool                                                                                     
 * @return 0 if success                                                                   
 */                                                                                             
unsigned int tcp_client_exit(void)
{
  pthread_mutex_lock( &mutex_TCP_general );
  tcp_thread_active = 0;
  pthread_mutex_unlock( &mutex_TCP_general );

  void * dummy_ret;
  for (int i=0; i<tcp_thread_num; i++)
  { 
    pthread_join(tcp_thread_info[i].thread_id, &dummy_ret);
  }

  printf("tcp thread joined...\n");

  dbprintf("%s(%d): close client socket, file descriptor %d \n", __func__, __LINE__, clientsockfd);
  close(clientsockfd);

  //free memory
  for (int i = 0; i < TCP_BUF_MAX_FILLS; i++){
    free(tcp_buf_header_gl[i]);
  }

  for (int i = 0; i < TCP_BUF_MAX_FILLS; i++){
    free(tcp_buf_amc13_gl[i]);
  }

  for (int i = 0; i < TCP_BUF_MAX_FILLS; i++){
    free(tcp_buf_tail_gl[i]);
  }

  for (int i = 0; i < TCP_BUF_MAX_FILLS; i++){
    free(tcp_buf_gl[i]);
  }

  free(tcp_buf_gl);
  free(tcp_buf_amc13_gl);
  free(tcp_buf_header_gl);
  free(tcp_buf_tail_gl);

  printf("tcp memory freed...\n");
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
  // zero TCP thread fill counter for each run
  TCPfillnumber = 0;

  // set true to permit the printing of ReadXBytes Error message
  ReadXBytesErrMessageSwitch = true;

  //Refresh the connection
  /*
  close(clientsockfd);
  sleep(2);

  struct sockaddr_in serv_addr;                                                          
  bzero((char *) &serv_addr, sizeof(serv_addr));                              
  serv_addr.sin_family = AF_INET;
  inet_aton(amc13_link_odb[0].source_ip, &serv_addr.sin_addr);
  serv_addr.sin_port = htons(amc13_link_odb[0].source_port);

  int status = connect( clientsockfd,(struct sockaddr *) &serv_addr, sizeof(serv_addr));
  if (status < 0) {
    perror( __func__ );
    cm_msg(MERROR, __FILE__, "Cannot connect to socket");
    return -1;
  }
*/
  // Enable reading in the reading thread
  pthread_mutex_lock( &mutex_TCP_general );
  tcp_run_active = 1;
  pthread_mutex_unlock( &mutex_TCP_general );

  int frontend_index = get_frontend_index();

  // compute number of active Rider modules for each run
  int im;
  NRiderModuleEnabled = 0;
  for(im=0;im<NRiderModuleMax;im++){
    if (frontend_index == 0) {
      if (amc13_fc7_odb[im].common.enabled) {
	NRiderModuleEnabled++;
      }
    } else {
      if (amc13_rider_odb[im].board.rider_enabled) {
	NRiderModuleEnabled++;
      }
    }
  } 

  dbprintf("%s(%d): begin-of-run TCP fill number %d \n", __func__, __LINE__, TCPfillnumber );

  usleep(10000);

  if (read_error)
    return -1;
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
  // Disable reading in the reading thread
  pthread_mutex_lock( &mutex_TCP_general );
  tcp_run_active = 0;
  pthread_mutex_unlock( &mutex_TCP_general );

  dbprintf("%s(%d): end-of-run TCP fill number %d \n", __func__, __LINE__, TCPfillnumber ); 

  return 0;
}

/*-- tcp_thread(void*) -------------------------------------------------*/                          

/**                                                                                             
 * tcp_thread(void*)
 *
 * (1) reads header size and trailer size from TCP readout ODB 
 * (2) loop over get header, data, trailer from TCP socket read with data size in header info
 * (3) check integrity of data, if OK, then copy to tcp_buf_gl[bufIndex], lock access when filling the buffer
 * @return 0 if success                                                                   
 */
void *tcp_thread(void* inform)
{
  int status;
  read_error = false;

  if (inform != nullptr)
  {
    dbprintf("%s(%d): TCP thread created, pointer %p \n", __func__, __LINE__, inform );
  }else
  {
    dbprintf("%s(%d): TCP thread pointer is null \n", __func__, __LINE__ );
  }  

  TCPheadersize = amc13_amc13_odb.header_size;
  if (TCPheadersize > TCPheadersizemax) 
    {    
      cm_msg(MERROR, __FILE__, "TCP headersize too large");
    }
  dbprintf("%s(%d): expected header size %d \n", 
	   __func__, __LINE__, TCPheadersize );

  TCPtailsize = amc13_amc13_odb.tail_size;
  if (TCPtailsize > TCPtailsizemax) 
    {
      cm_msg(MERROR, __FILE__, "TCPtailsize too large");
    }
  dbprintf("%s(%d): expected trailer size %d \n", 
	   __func__, __LINE__, TCPtailsize );
 
  TCPamc13infosize = 0x10000; // fix me ???
 
  headerbytes = TCPheadersize;
  header = (uint64_t*) malloc( TCPheadersizemax );
  amc13infobytes = TCPamc13infosize;
  amc13info = (uint64_t*) malloc( TCPamc13infosizemax );
  tailbytes = TCPtailsize;
  tail = (uint64_t*) malloc( TCPtailsizemax );
  databytes = 0;
  data = (int16_t*) malloc( TCPdatasizemax );

  //Monitor odb
  int frontend_index = get_frontend_index();
  std::string MonitorRootKey;
  char fe_index[3];
  sprintf(fe_index, "%02i", frontend_index);
  std::stringstream ss_monitors;
  ss_monitors << "/Equipment/AMC13"<< fe_index << "/Monitors";
  MonitorRootKey = ss_monitors.str();

  int ThreadStatus = 1;
  std::string ThreadStatusKey = MonitorRootKey + std::string("/TCP Thread Status");
  db_set_value(hDB, 0, ThreadStatusKey.c_str(), &ThreadStatus, sizeof(ThreadStatus), 1, TID_INT);

  int bufIndex; // array index of ring buffer for TCP->GPU data transfer

  while ( 1 ){    // loops over AMC13 events

    unsigned long TCPfillnumber_local;
    unsigned long GPUfillnumber_local;

    int local_thread_active = 0;
    int local_run_active = 0;

    pthread_mutex_lock( &mutex_TCP_general );  
    TCPfillnumber_local = TCPfillnumber;
    local_thread_active = tcp_thread_active;
    local_run_active = tcp_run_active;
    pthread_mutex_unlock( &mutex_TCP_general );  

    pthread_mutex_lock( &mutex_GPU_general );  
    GPUfillnumber_local = GPUfillnumber;
    pthread_mutex_unlock( &mutex_GPU_general );  

    //break the loop if thread active is 0
    if (!local_thread_active)
    {
      break;
    }

    //continue the loop if run active is 0
    //this allows the master thread to refresh the tcp connection
    if (!local_run_active)
    {
      continue;
    }

    unsigned long tcp_buffer_filled = 0;
    if (TCPfillnumber_local > GPUfillnumber_local)
    {
      tcp_buffer_filled = TCPfillnumber_local - GPUfillnumber_local;
    }else{
      tcp_buffer_filled = 0xffffffffffffffff - (GPUfillnumber_local - TCPfillnumber_local) +1 ;
    }

    //Do not keep reading if the ring buffer is full
    if (tcp_buffer_filled >= TCP_BUF_MAX_FILLS)
    {
      continue;
    }

    // set index of ring buffer
    bufIndex = TCPfillnumber_local % TCP_BUF_MAX_FILLS;
    dbprintf("%s(%d): start read of new event, fill %d, buffer %d \n",  __func__, __LINE__, TCPfillnumber_local, bufIndex );

    // get time of start of read / unpack AMC13 event
    status = gettimeofday( &tstart, NULL);
    header[1] = tstart.tv_sec;  // fill header time info in header
    header[2] = tstart.tv_usec; // fill header time info in header
    trigger_info.time_tcp_start_read_s = tstart.tv_sec; 
    trigger_info.time_tcp_start_read_us = tstart.tv_usec;

    // lock access to tcp_buf_gl[bufIndex]
    pthread_mutex_lock( &mutex_TCP_buf[bufIndex] );  

    // function reads / unpacks the AMC13 block structure
    gettimeofday( &tbeginread, NULL);
    databytes = readAndUnpack( bufIndex );
    gettimeofday( &tfinishread, NULL);

    //Test print of the fill number
    //printf("AMC13 Fill number = %d ; TCP Fill number = %d \n",getEventIndex( be64toh( header[0] ) ),int(TCPfillnumber));

    //Check if there are data readout correctly
    if (databytes == 0)
    {
      //skip this iteration if there are no data available
      pthread_mutex_unlock( &mutex_TCP_buf[bufIndex] );  
      continue; 
    }

    if (databytes < 0)
    {
      //terminate the while loop if there is an read error
      read_error = true;
      pthread_mutex_unlock( &mutex_TCP_buf[bufIndex] );  
      cm_msg(MERROR, __FILE__,"tcp_thread: break the tcp thread loop becuase of a reading error %d", databytes);
      break;
    }

    if ( toddiff( &tfinishread, &tbeginread) > 100000.) {
      printf("WARNING tcpip stall, readAndUnpack > 100ms!");
      printf("%s(%d): duration of readAndUnpack, read %d bytes, time = %e us \n", 
	  __func__, __LINE__, databytes , toddiff( &tfinishread, &tbeginread) );
    }

    amc13infobytes = (uint64_t)offsetamc13info - (uint64_t)amc13info; 
    trigger_info.time_tcp_finish_header_read_s = header[3];
    trigger_info.time_tcp_finish_header_read_us = header[4];
     
    // get time done read / unpack of AMC13 event 
    status = gettimeofday( &tdata, NULL);
    header[5] = tdata.tv_sec; // fill data time info in header
    header[6] = tdata.tv_usec; // fill data time info in header
    trigger_info.time_tcp_finish_data_read_s = tdata.tv_sec;
    trigger_info.time_tcp_finish_data_read_us = tdata.tv_usec;

    dbprintf("%s(%d): duration of read and unpack after header bank, fill %d, duration = %e us \n", 
	     __func__, __LINE__, TCPfillnumber_local, toddiff( &tdata, &theader) );

    // check data integrity, if OK, then copy TCP buffer into global buffer
    if ( ( ( be64toh(header[0]) & BODmask ) == BODdelimiter ) && ( ( be64toh(tail[0]) & EODmask ) == EODdelimiter ) )
      {
	dbprintf("%s(%d): PASS data integrity check, buffer %d, fill %d AMC general header 0x%016lX \n", 
		 __func__, __LINE__, bufIndex, TCPfillnumber_local, *header);
	dbprintf("%s(%d): integrity data, BOD: 0x%016lX = 0x%016lX ? EOD:  0x%016lX = 0x%016lX ? \n", 
		 __func__, __LINE__, ( be64toh(header[0]) & BODmask ), BODdelimiter, ( be64toh(tail[0]) & EODmask ),  EODdelimiter  );
      }
    else 
      {
	// even if data integrity checks fails we send data to gpu as failing events have bad header / trailer info,
	dbprintf("%s(%d): FAIL data integrity check, buffer %d, fill %d, AMC general header 0x%016lX! \n", 
	       __func__, __LINE__, bufIndex, TCPfillnumber_local, *header);
	dbprintf("%s(%d): integrity data, BOD: 0x%016lX = 0x%016lX ? EOD:  0x%016lX = 0x%016lX ? \n", 
		 __func__, __LINE__, ( be64toh(header[0]) & BODmask ), BODdelimiter, ( be64toh(tail[0]) & EODmask ),  EODdelimiter  );
      }

    //Record fill number in header
    header[19] = TCPfillnumber;

    // save header, data, trailer sizes -  TCPtotalheadersize, TCPtotaldatasize, TCPtotaltailsize - 
    TCPtotalheadersize[bufIndex] = headerbytes;
    TCPtotalamc13infosize[bufIndex] = amc13infobytes;
    TCPtotaltailsize[bufIndex] = tailbytes;
    TCPtotaldatasize[bufIndex] = databytes;
 
    // copy header, trailer buffers from temporary header, tail buffers to tcp_buf_header_gl[], tcp_buf_amc13_gl[], tcp_buf_tail_gl[].
    memcpy( tcp_buf_header_gl[bufIndex], header, TCPtotalheadersize[bufIndex] );
    memcpy( tcp_buf_amc13_gl[bufIndex], amc13info, TCPtotalamc13infosize[bufIndex] );
    memcpy( tcp_buf_tail_gl[bufIndex], tail, TCPtotaltailsize[bufIndex] );
    dbprintf("%s(%d): copied %d, %li, %d, header, data, tail bytes \n",
	     __func__, __LINE__, headerbytes, amc13infobytes, tailbytes );

    // get header / traier memcpy time
    status = gettimeofday( &tmemcpy, NULL);                                                 
    dbprintf("%s(%d): duration of header / trailer memcpy, fill %d, duration = %e us \n", 
	     __func__, __LINE__, TCPfillnumber_local, toddiff( &tmemcpy, &tdata) );

    // release access to ring buffer
    pthread_mutex_unlock( &mutex_TCP_buf[bufIndex] ); 
    
    // get unlock time
    status = gettimeofday( &tunlock, NULL);
    dbprintf("%s(%d): duration of unlock ring buffer READY, bufIndex %d, fill %d, fdt = %e us \n", 
	     __func__, __LINE__, bufIndex, TCPfillnumber_local, toddiff( &tunlock, &tmemcpy) );
 
    int TCPFill = 0;
    pthread_mutex_lock( &mutex_TCP_general );  
    TCPfillnumber++;
    TCPFill  = TCPfillnumber % (0xffffffff/2);
    pthread_mutex_unlock( &mutex_TCP_general );  

    //update odb
    std::string TCPFillNumberKey = MonitorRootKey + std::string("/TCP Fill Number");
    db_set_value(hDB, 0, TCPFillNumberKey.c_str(), &TCPFill, sizeof(TCPFill), 1, TID_INT);
  
  } // end of while ( 1 )
  //Print status if it is not zero
  if (status !=0 )
  {
    dbprintf("%s(%d): status = %d\n", __func__, __LINE__,status );
  }

  if (read_error)
  {
    ThreadStatus = -1;
  }else{
    ThreadStatus = 0;
  }
  db_set_value(hDB, 0, ThreadStatusKey.c_str(), &ThreadStatus, sizeof(ThreadStatus), 1, TID_INT);
  
  if (read_error)
  {
    //Set the status color to grey if the tcp_thread quits with an read error
    std::stringstream ss_color;
    ss_color << "/Equipment/AMC13" << fe_index << "/Common/Status color";
    std::string color_str = ss_color.str();
    db_set_value(hDB, 0, color_str.c_str(), "#808080", 32, 1, TID_STRING);

    //Raise alarm
    char AlarmMsg[500];
    sprintf(AlarmMsg,"DAQ | AMC13%02d Read Error. Stop run!",frontend_index);

    int ret_code = al_trigger_alarm("AMC13 Hardware Error", AlarmMsg, "Warning", "AMC13 Hardware Error", AT_INTERNAL);
    if (ret_code != AL_SUCCESS) {
      cm_msg(MERROR, __FILE__, "Failure Raising Alarm: Error %d, Alarm \"%s\"", ret_code, "AMC13 Hardware Error");
    }
  }

  // release memory for both successful and unsuccessful event reconstruction
  printf("%s(%d): Start freeing memory\n", __func__, __LINE__ );
  free(header);
  printf("%s(%d): done header free()\n", __func__, __LINE__ );
  free(amc13info);
  printf("%s(%d): done amc13info free()\n", __func__, __LINE__ );
  free(data);
  printf("%s(%d): done data free()\n", __func__, __LINE__ );
  free(tail);	
  printf("%s(%d): done tail free()\n", __func__, __LINE__ );

  return inform;
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

  dbprintf("%s(%d): index %d, AMCHeaderWord 0x%016lX\n", __func__, __LINE__, i, AMCHeaderWord);
 
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

  uint32_t EventNum = (uint32_t) ( ( CDFGeneralHeaderWord & CDFGeneralHeaderEventIndexMask ) >> CDFGeneralHeaderEventIndexShift );
  dbprintf("%s(%d): CDFGeneralHeaderWord 0x%016lX\n", 
	   __func__, __LINE__, CDFGeneralHeaderWord);

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

  uint32_t OverallSize = (uint32_t) ( ( CDFGeneralTrailerWord & CDFGeneralTrailerOverallSizeMask ) >> CDFGeneralTrailerOverallSizeShift );
  dbprintf("%s(%d): CDFGeneralHeaderWord 0x%016lX\n", 
	   __func__, __LINE__, CDFGeneralTrailerWord);

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

  uint32_t AMCNum = (uint32_t) ( ( AMCGeneralHeaderWord & AMCGeneralHeaderAMCNumMask ) >> AMCGeneralHeaderAMCNumShift );
  dbprintf("%s(%d): AMCGeneralHeaderWord 0x%016lX\n",  __func__, __LINE__, AMCGeneralHeaderWord);

  return AMCNum;
}


/**                                                                                             
 * ReadXBytes(int socket, unsigned int x, void* buffer,int & block_status)
 *
 * reads x bytes on file descriptor socket and place in buffer
 *
 * @return 0 if success                                                                   
 */                                                                             
int ReadXBytes(int socket, unsigned int x, void* buffer, int & block_status)
{
  int result, tries = 0; 
  unsigned int bytesRead = 0;
  block_status = 0;

  int block_count = 0;
  int block_limit = 1000;
  
  dbprintf("%s(%d): ReadXBytes :: x = %d\n", __func__, __LINE__, x );
  while (bytesRead < x)
  {

//    gettimeofday( &tbeginread, NULL);
    result = read(socket, (char *)buffer + bytesRead, x - bytesRead);
//    gettimeofday( &tfinishread, NULL);

    if (result < 0 )
    {
      if (errno == EWOULDBLOCK)
      {
//	printf("ReadXBytes: %d, errno %d, error message: %s\n", result,errno ,strerror(errno));
	block_count++;
	usleep(20);
      }else{
	if (ReadXBytesErrMessageSwitch) {
	  printf("ReadXBytes: warning read return code %d, errno %d\n", result, errno);
	  cm_msg(MERROR, __FILE__,"ReadXBytes: warning read return code %d, errno %d", result, errno);
	  ReadXBytesErrMessageSwitch = false; // only print one error message per run
	}

	return -1;
      }

      if ( block_count > block_limit )
      {
	block_status = 1;
	break;
      }

    }else{
      bytesRead += result;
    }
    tries++; 
/*
    if ( toddiff( &tfinishread, &tbeginread) > 100000.) {
      printf("WARNING tcpip stall, read > 100ms!");
      printf("%s(%d): duration of readXbytes, request %d bytes, read %d bytes, tries %d, time = %e us \n", 
	  __func__, __LINE__, x-bytesRead, bytesRead, tries, toddiff( &tfinishread, &tbeginread) );
    }*/
  }

  dbprintf("%s(%d): socket file descriptor %d, request %d bytes, read %d bytes, tries %d\n", 
	   __func__, __LINE__, socket, x, bytesRead, tries);

  //printf("ReadXBytes: %d bytes read\n", bytesRead);
  //printf("ReadXBytes: %d bytes read, %d tries, %d blocks\n", bytesRead,tries,block_count);
  return bytesRead;
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
  dbprintf("%s(%d): blocking, O_NONBLOCK 0x%08x, 0x%08x \n", __func__, __LINE__, blocking, O_NONBLOCK);

  if (fd < 0)   
    {           
      return 0x0;                                 
    }           
  int flags = fcntl(fd, F_GETFL, 0);
  dbprintf("%s(%d): fd %d, flags before setting 0x%08x \n",  __func__, __LINE__, fd, flags);

  if (flags < 0)
    {           
      return 0x0;                                 
    }           
  flags = blocking ? (flags&~O_NONBLOCK) : (flags|O_NONBLOCK);                          
  dbprintf("%s(%d): fd %d, flags after setting 0x%08x \n",  __func__, __LINE__, fd, flags);

  status = fcntl(fd, F_SETFL, flags);
  dbprintf("%s(%d): fcntl status %d, errno %d \n",  __func__, __LINE__, status, errno);

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
  unsigned int i; 

  printf("%s(%d): ndata = %d \n", __func__, __LINE__, ndata);
  for (i = 0; i < ndata; ++i)
  printf("%s(%d): data[%d] = %d \n",__func__, __LINE__, i, data[i]);

  return;
}

/*-- readAndUnpack(int bufIndex) --------------------------*/                          

/**                                                                                             
 * read and unpack the data from AMC13 via TCP socket
 *
 *
 *
 * @return number of bytes read if success                                                                   
 * @return 0 if no data are available
 */                                                                             
int readAndUnpack(int bufIndex){

//#ifdef DEBUG
  unsigned int EventIndex;  // AMC13 reported event number
  unsigned int OverallSize; // event size in AMC13 header
//#endif
  int iAMC, nAMC;  // AMC13 reported number of AMC modules 

//#ifdef DEBUG
  int local_headerbytes = TCPheadersize;
//#endif

  int block_status = 0;

  int retval = ReadXBytes( clientsockfd,  sizeof(uint64_t), (void*)( header ),block_status );
//  printf("Read Header: %d vs %d",retval, sizeof(uint64_t));

  // get overall CDF header word
  if (retval < int(sizeof(uint64_t)))
  {
    if ( retval < 0 ) 
    {                                                                                    
      cm_msg(MERROR, __FILE__, "Cannot read header from socket");                                 
      return -1;                                                                  
    }else if (retval == 0)
    {
      if (block_status == 1 )
      {
	return 0;
      }else{
	cm_msg(MERROR, __FILE__, "Cannot read header from socket");                                 
	return -1;                                                                  
      }
    }else{
	cm_msg(MERROR, __FILE__, "Cannot read header from socket");                                 
      return -1;
    }
  }

  // get event number from header bank
//#ifdef DEBUG
  EventIndex = getEventIndex( be64toh( *header ) );
//#endif

  // pointer location to AMC13 unpacking info in amc13info data array
  offsetamc13info = amc13info;
  // write CDF header word in the amc13info array
  *offsetamc13info = *header;
  dbprintf("%s(%d): read header, header size [bytes] %d, header[0] 0x%016lX, BODdelimiter 0x%016lX, BODmask 0x%016lx Event number %i\n", 
	     __func__, __LINE__, local_headerbytes, *offsetamc13info, BODdelimiter, BODmask, EventIndex );
  offsetamc13info++;
  
  // record time got header word
  gettimeofday( &theader, NULL);
  header[3] = theader.tv_sec; // fill header time info in header
  header[4] = theader.tv_usec; // fill header time info in header
  dbprintf("%s(%d): duration from AVAIL lock to fill header bank, buffer[%d], fill %d, duration %e us \n", 
	     __func__, __LINE__, bufIndex, TCPfillnumber, toddiff( &theader, &tstart) );

  // byte / block counters for AMC modules x AMC blocks readoout structure
  int blockdatabytes = 0; // individual AMC module bytes per AMC13 block
  int totaldatabytes = 0; // running total of all AMC modules data bytes 
  int blockcount = 0;     // AMC13  block counters

  // data offsets for unpacking data buffer structure of AMCs x blocks
  unsigned int dataoffset = 0, datablockoffset[12], dataAMCoffset[12];
  memset( datablockoffset, 0, sizeof(datablockoffset) ); // block offset of particular AMC modules data
  memset( dataAMCoffset, 0, sizeof(dataAMCoffset) ); // overall offset of particular AMC modules data

  bool moredata = 1; // more data is true of more blocks are available
  while ( moredata ){  // loops over AMC data blocks 
    
    // read single 64-bit AMC13 block header word
    //Try reading 1 times before giving up
    int read_fail = 0;
    while (read_fail<1)
    {
      retval  = ReadXBytes( clientsockfd, sizeof(uint64_t), (void*)( offsetamc13info ) ,block_status);
      if (retval>0)
      {
	break;
      }
      usleep(100000);
      read_fail++;
    }
    if (read_fail>=1)
    {
      cm_msg(MERROR, __FILE__, "Error when reading from socket, fd %d. Read %d bytes vs %d, for %d times", clientsockfd,retval,sizeof(uint64_t),read_fail);  
      cm_msg(MERROR,__FILE__, "read header, header size [bytes] %d, header[0] 0x%016lX, BODdelimiter 0x%016lX, BODmask 0x%016lx Event number %i",local_headerbytes, *offsetamc13info, BODdelimiter, BODmask, EventIndex);
    }
    if ( retval < int(sizeof(uint64_t)) ) 
    {                                                                                    
      cm_msg(MERROR, __FILE__, "Error when reading from socket, fd %d. Read %d bytes vs %d", clientsockfd,retval,sizeof(uint64_t));  
      cm_msg(MERROR,__FILE__, "read header, header size [bytes] %d, header[0] 0x%016lX, BODdelimiter 0x%016lX, BODmask 0x%016lx Event number %i",local_headerbytes, *offsetamc13info, BODdelimiter, BODmask, EventIndex);
      return -1;                                                                  
    }
    
    // get the number of enabled AMCs
    nAMC = getAMCNum( be64toh( *offsetamc13info ) );
    offsetamc13info++;
    dbprintf("%s(%d): reading AMC general header word 0x%016lX, nAMC decoded %i\n", 
	     __func__, __LINE__, *offsetamc13info, getAMCNum( be64toh( *offsetamc13info ) ) );
 
    // WARN if mismatch between ODB and AMC13 headers / trailers for number of active modules for first block
    if ( ( blockcount == 0 ) && ( nAMC != NRiderModuleEnabled ) ) {
      cm_msg(MERROR, __FILE__, "WARNING! mismatch between ODB (%i) and AMC13 headers (%i) for number of AMC modules", NRiderModuleEnabled, nAMC);
      dbprintf("%s(%d): WARNING! mis-match between ODB (%i) and AMC13 headers (%i) for number of AMC modules\n", __func__, __LINE__,  NRiderModuleEnabled, nAMC);
    }
    
    // read 64-bit AMC module header words - one per AMC
    retval  = ReadXBytes( clientsockfd, nAMC*sizeof(uint64_t), (void*)( offsetamc13info) ,block_status);
    if ( retval < int(nAMC*sizeof(uint64_t)) ) 
      {                                                                                    
	cm_msg(MERROR, __FILE__, "Error when reading from socket, fd %d. Read %d bytes vs %d", clientsockfd,retval,nAMC*sizeof(uint64_t));  
	return -1;                                                                  
      }

    // WARN if mismatch between ODB and AMC13 headers / trailers for AMC slot number
    for (iAMC = 0; iAMC < nAMC; iAMC++){
      if ( !amc13_rider_odb[amc_header_info[iAMC].AMCSlotNum-1].board.rider_enabled ) {
	//cm_msg(MERROR, __FILE__, "WARNING! AMC slot %i not enabled in ODB", amc_header_info[iAMC].AMCSlotNum);
	dbprintf("%s(%d): WARNING! amc_header_info[iAMC].AMCSlot %i\n", __func__, __LINE__, amc_header_info[iAMC].AMCSlotNum);
      }
    }

    // decode AMC header words - get continuation bits, event / block size, AMC slot number
    // set moredata = 1 if more blocks are following this block
    moredata = 0;
    for (iAMC = 0; iAMC < nAMC; iAMC++){
      if ( decodeAMCHeader( iAMC, be64toh( *( offsetamc13info ) ) ) != 0 )
	{
	  printf("decodeAMCHeader() failed!");
	}
      offsetamc13info++;
      if (amc_header_info[iAMC].AMCMoreBit) moredata = 1;
 
     dbprintf("%s(%d): AMC index %d, AMC Slot number %d, AMCMoreBit %d, more data %d, AMCEventSize 0x%08x\n", 
	      __func__, __LINE__, iAMC, amc_header_info[iAMC].AMCSlotNum, amc_header_info[iAMC].AMCMoreBit,  moredata, amc_header_info[iAMC].AMCEventSize );
    }
 
    // calculate AMC data offsets dataAMCoffset[amc_header_info[iAMC].AMCSlotNum-1] from total event sizes in S=0 word AMC header word  
    // (i.e. for either M=1,S=0 with continuation blocks or M=0,S=0 with only one block)
    // This calculation is performed once per fill / event and hanfles different total data sizes, 
    // i.e. amc_header_info[iAMC].AMCEventSize, from different amcmodules
    if ( !amc_header_info[0].AMCSegBit ) {
      int AMCoffsetbytes = 0;      
      for (iAMC = 0; iAMC < nAMC; iAMC++){
	dataAMCoffset[amc_header_info[iAMC].AMCSlotNum-1] = AMCoffsetbytes / sizeof(uint64_t);
	dbprintf("%s(%d): blockcount %d, AMC index %d, calculated AMC total data offset 0x%08x\n", 
		 __func__, __LINE__, blockcount, iAMC, dataAMCoffset[amc_header_info[iAMC].AMCSlotNum-1]); 
	AMCoffsetbytes += sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
      }
    }

    // read AMC data block
    for (iAMC = 0; iAMC < nAMC; iAMC++){
      
      // calculate the data bytes - blockdatabytes - to read for each AMC module with index iAMC 
      // bits determine if first block, intermediate block, last block or single block
      if ( amc_header_info[iAMC].AMCMoreBit && (!amc_header_info[iAMC].AMCSegBit) )
	{
	  blockdatabytes = 32768;
	  dbprintf("M=1,S=0 first block in segment, set size to 0x%08x bytes (odb 0x%08x)\n", 
		   blockdatabytes, amc13_amc13_odb.amc_block_size);
	}
      if ( amc_header_info[iAMC].AMCMoreBit && amc_header_info[iAMC].AMCSegBit )
	{
	  dbprintf("M=1,S=1 intermediate block in segment, set size from amc header word\n");
	  blockdatabytes = sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
	}
      if ( (!amc_header_info[iAMC].AMCMoreBit) && amc_header_info[iAMC].AMCSegBit )
	{
	  dbprintf("M=0,S=1 last block in segment, set size from amc header word\n");
	  blockdatabytes = sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;	  
	}
      if ( (!amc_header_info[iAMC].AMCMoreBit) && (!amc_header_info[iAMC].AMCSegBit) )
	{
	  dbprintf("M=0,S=0 only block in segment, set size from amc header word\n");
	  blockdatabytes = sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
	}
      
      // calculated the location to put the data from block structure in AMC13 event
      dataoffset = dataAMCoffset[amc_header_info[iAMC].AMCSlotNum-1] + datablockoffset[amc_header_info[iAMC].AMCSlotNum-1];
      dbprintf("%s(%d): blockcount %d, iAMC %d, calculated AMC+Block data offset 0x%08x block data bytes 0x%08x data bytes total 0x%08x\n", 
	       __func__, __LINE__, blockcount, iAMC, dataoffset, blockdatabytes, totaldatabytes); 
      
      // read the data block for each AMC module in array tcp_buf_gl[bufIndex]
      retval  = ReadXBytes( clientsockfd, blockdatabytes, (void*)( tcp_buf_gl[bufIndex] + dataoffset ) ,block_status);
	if ( retval < blockdatabytes) 
	{                                                                                    
	  cm_msg(MERROR, __FILE__, "Error when reading from socket, fd %d. Read %d bytes vs %d", clientsockfd,retval,blockdatabytes);  
	  return -1;                                                                  
	}
      dbprintf("%s(%d): done reading AMC block %i bytes %i, dataoffset %d, (tcp_buf_gl[bufIndex] + dataoffset ) %p, data[0] 0x%16lx data[1] 0x%16lx\n", 
	       __func__, __LINE__, blockcount, blockdatabytes, dataoffset, ( tcp_buf_gl[bufIndex] + dataoffset ), 
	       *( tcp_buf_gl[bufIndex] + dataoffset ), *( tcp_buf_gl[bufIndex] + dataoffset + 1 ) ); 

      //dataoffset += blockdatabytes/sizeof(uint64_t); // redundant so removed?
      datablockoffset[amc_header_info[iAMC].AMCSlotNum-1] += blockdatabytes/sizeof(uint64_t); // datablockoffset[i] is individual payload readout from ith AMC module
      totaldatabytes += blockdatabytes; // totaldatabytes is total payload readout from all AMC modules
      dbprintf("%s(%d): end of read loop for amc %i\n",__func__, __LINE__,iAMC);
    }
    
    // read single 64-bit AMC13 block trailer word
    retval = ReadXBytes( clientsockfd, sizeof(uint64_t), (void*)( offsetamc13info ) ,block_status);
    if ( retval < int(sizeof(uint64_t))) 
      {                                                                                    
	cm_msg(MERROR, __FILE__, "Error when reading from socket, fd %d. Read %d bytes vs %d", clientsockfd,retval,sizeof(uint64_t));  
	return -1;                                                                  
      }
    dbprintf("%s(%d): done reading AMC block %i, trailer word *tmp 0x%08lx\n", 
	     __func__, __LINE__, blockcount, *offsetamc13info); 
    
    offsetamc13info++;
    blockcount++;
  }
  dbprintf("%s(%d): finished data read / unpack, databytes total 0x%08x block count %i\n", 
	   __func__, __LINE__, totaldatabytes, blockcount); 
  
  // get CDF trailer word
  retval = ReadXBytes( clientsockfd, tailbytes, (void*)(tail) ,block_status);
  if ( retval < int(tailbytes)) 
    {                                                                                    
      cm_msg(MERROR, __FILE__, "Error when reading from socket, fd %d. Read %d bytes vs %d", clientsockfd,retval,tailbytes);  
      return -1;                                                                  
    }

#ifdef DEBUG
  OverallSize = getOverallSize( be64toh(tail[0]) );
#endif

  dbprintf("%s(%d): read trailer, trailer size [bytes] %d, tail[0] 0x%016lX, EODdelimiter 0x%016lX, EODmask 0x%016lX, Overall Size %i\n", 
	   __func__, __LINE__, tailbytes, be64toh(tail[0]), EODdelimiter, EODmask, OverallSize);

#if 0 // turn on/off CPU-based byte-reordering in 8-byte AMC13 words 

  // re-order data from network / big-endian to little-endian
  struct timeval tbeforeReorderBytes, tafterReorderBytes;
  gettimeofday( &tbeforeReorderBytes, NULL);

  int iReorderBytes, nReorderBytes = totaldatabytes / sizeof(uint64_t);
  for (iReorderBytes = 0; iReorderBytes < nReorderBytes; iReorderBytes++){
    tcp_buf_gl[bufIndex][iReorderBytes] = be64toh( tcp_buf_gl[bufIndex][iReorderBytes] );
  }

  gettimeofday( &tafterReorderBytes, NULL);
  dbprintf("%s(%d): duration of byte re-ordering, buffer[%d], fill %d, duration %e us \n", 
	 __func__, __LINE__, bufIndex, TCPfillnumber, toddiff( &tafterReorderBytes, &tbeforeReorderBytes) );
#endif 

  return totaldatabytes;
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
