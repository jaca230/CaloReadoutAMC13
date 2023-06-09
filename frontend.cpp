/*
 * @file    frontends/CaloReadoutAMC13/frontend.cpp
 * @author  Wes Gohn, Vladimir Tishchenko, Tim Gorringe
 * @date    Thu May 24 10:08:59 2012 (-0500)
 * @date    Last-Updated: Tue Oct 13 16:47:31 2018 (-0400)
 *          Update #: 954
 * @version $Id$
 *
 * @copyright (c) (g-2) collaboration 
 *
 * @brief   10GbE AMC13 readout frontend for (g-2)
 *
 * @ingroup page_frontends
 * 
 * @details This frontend processes calorimeter and other data from an AMC13 over 10 Gbe
 *          
 *
 * 
 * @section Changelog
 * @verbatim 
 * $Log$ 
 * @endverbatim
 * 
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <midas.h>
#include <mfe.h>
#include <msystem.h>

#ifdef USE_PARALLEL_PORT // parallel port access
#include <sys/io.h> 
#include <string.h>
#include <string>
int pp_addr = 0x37f;
#endif

#ifdef USE_GPU
#include <cuda.h>
#include <cuda_runtime_api.h>
#include "cuda_tools_g2.h"
#endif

#include "gpu_fit.hh" // for fit results
#include "experiment.h"
#include "frontend.h"
//#include "frontend_rpc.h"
#include "tcp_thread.h"
#include "gpu_thread.h"
#include "fe_compress_z_mt.h"
#include "amc13_odb.h"

#include "AMC13.hpp"
#include "FC7.hpp"
#include "WFD5.hpp"
#include "laser_config_handler.h"

#include "uhal/uhal.hpp"
#include "uhal/utilities/xml.hpp"
#include "uhal/log/exception.hpp"
#include "uhal/ProtocolUDP.hpp"
#include "boost/filesystem/operations.hpp"
#include "boost/algorithm/string.hpp"

#ifdef DEBUG
#define dbprintf(...) printf(__VA_ARGS__)
#else
#define dbprintf(...)
#endif

// counters for total trigger and got triggers in each run
int total_triggers_in_run = -1;
int got_triggers_in_run = 0;
BOOL got_triggers_sent = TRUE;

float dt_READY = 0.0;
int n_dt_READYs = 0;

unsigned long Midasfillnumber;

std::string MonitorODBRootKey;
/*
float toddiff(struct timeval *tod2, struct timeval *tod1) {
  float fdt, fmudt;
  long long t1, t2, mut1, mut2;
  long long dt, mudt;
  t1 = tod1->tv_sec;
  mut1 = tod1->tv_usec;
  t2 = tod2->tv_sec;
  mut2 = tod2->tv_usec;
  dt = (t2 - t1);
  mudt = (mut2 - mut1);
  fdt = (float)dt;
  fmudt = (float)mudt;
  return 1.0e6*fdt + fmudt;
} */             
                                                                            
/*
 * Trigger information 
*/
/*
  DWORD trigger_nr;                       // trigger number (via RPC message from master)
  DWORD trigger_mask;                     // trigger mask (via RPC message from master)
  DWORD time_master_got_eof_s;            // master EOF trigger time (via RPC message from master), seconds
  DWORD time_master_got_eof_us;           // master EOF trigger time (via RPC message from master), microseconds
  DWORD time_slave_got_eof_s;             // slave EOF trigger time called from master via RPC message, seconds
  DWORD time_slave_got_eof_us;            // slave EOF trigger time called from master via RPC message, microseconds
  DWORD time_slave_got_data_s;            // slave got data fron tcp_thread and unloacked tcp_thread, seconds
  DWORD time_slave_got_data_us;           // slave got data fron tcp_thread and unloacked tcp_thread, microseconds
  DWORD time_slave_lock_dataready_s;      // slave locking mutex_data_ready in read_trigger_event                   
  DWORD time_slave_lock_dataready_us;     // slave locking mutex_data_ready in read_trigger_event      
  DWORD time_tcp_start_read_s;            // start time of tcp read in tcp_thread, seconds
  DWORD time_tcp_start_read_us;           // start time of tcp read in tcp_thread, microseconds
  DWORD time_tcp_finish_header_read_s;    // finish time of tcp read in tcp_thread, seconds
  DWORD time_tcp_finish_header_read_us;   // finish time of tcp read in tcp_thread, microseconds
  DWORD time_tcp_finish_data_read_s;      // finish time of tcp read in tcp_thread, seconds
  DWORD time_tcp_finish_data_read_us;     // finish time of tcp read in tcp_thread, microseconds
  DWORD time_gputhread_started_s;         // woke-up gpu_thread for processing, seconds
  DWORD time_gputhread_started_us;        // woke-up gpu_thread for processing, microseconds 
  DWORD time_gputhread_copytogpu_done_s;  // copy to gpu done, seconds
  DWORD time_gputhread_copytogpu_done_us; // copy to gpu done, microseconds
  DWORD time_gputhread_finished_s;        // gpu_thread finished processing, seconds
  DWORD time_gputhread_finished_us;       // gpu_thread finished processing, microseconds
*/
S_TRIGGER_INFO trigger_info;

/* make frontend functions callable from the C framework */
//#ifdef __cplusplus
//extern "C" {
//#endif
  
  /*-- Globals -------------------------------------------------------*/

  // map from detector index in CT bank to rider module,channel to find thresholds for truncated calo bank processing
  int map_from_caloxy_to_ridermodchan[60][TQMETHOD_MAX];

  // declare WFD5 and AMC13 access objects
  AMC13 *amc13lib = new AMC13();
  FC7 *fc7lib = new FC7();
  WFD5 *wfdlib = new WFD5();

  uhal::HwInterface *amc13[2];
  uhal::HwInterface *fc7[12];
  uhal::HwInterface *wfd[12];

  // Class to handle laser config.
  g2laser::LaserConfigHandler *laser_config_handler = NULL;

  // The frontend name (client name) as seen by other MIDAS clients
  const char *frontend_name = "AMC13";

  // The frontend file name, don't change it
  const char *frontend_file_name = __FILE__;
  
  // ODB database handle
  extern HNDLE hDB;

  // frontend_loop is called periodically if this variable is TRUE
  BOOL frontend_call_loop = FALSE; //Set by Ran Hong
  
  // frontend status page is displayed with this frequency in ms
  INT display_period = 0;
  
  // maximum event size produced by this frontend
  INT max_event_size = DEFAULT_MAX_EVENT_SIZE / 2;
  
  // maximum event size for fragmented events (EQ_FRAGMENTED)
  INT max_event_size_frag = 0;
  
  // buffer size to hold events
  INT event_buffer_size = DEFAULT_MAX_EVENT_SIZE; 

  extern INT run_state;      // run state (STATE_RUNNING, ...)
  //extern INT frontend_index; // frontend index from command line argument -i
  
  INT frontend_index_offset = CALO_READOUT_FE_INDEX_OFFSET; // for CaloReadoutAMC13
  pthread_mutex_t mutex_midas = PTHREAD_MUTEX_INITIALIZER;
  
  /*-- Local variables -----------------------------------------------*/
  static int block_nr;      // acquisition block number
  int MIN_ISLAND_LENGTH = 25;

  int encoder_fc7_slot = -1;
  int trigger_fc7_slot = -1;
   
  // for call to compress_z_eor in read_trigger_event()
  INT run_number_compress_z_eor;
  char* error_compress_z_eor;

  /*-- Function declarations -----------------------------------------*/
//  extern "C" {
    INT amc13_odb_check();
    INT fc7_odb_check();
    INT wfd_odb_check();
    INT frontend_init_fc7();
    INT frontend_init_wfd();
    INT frontend_init();
    INT frontend_exit();
    INT begin_of_run_wfd();
    INT begin_of_run(INT run_number, char *error);
    INT end_of_run(INT run_number, char *error);
    INT pause_run(INT run_number, char *error);
    INT resume_run(INT run_number, char *error);
    INT frontend_loop();
    
    INT interrupt_configure(INT cmd, INT source, POINTER_T adr);
    INT poll_event(INT source, INT count, BOOL test);
    INT read_trigger_event(char *pevent, INT off);
    
//    INT rpc_g2_end_of_fill(INT index, void *prpc_param[]);
//    INT rpc_g2_arm_sampling_logic(INT index, void *prpc_param[]);
//    INT rpc_g2_recv_trigger_number(INT index, void *prpc_param[]);
    
    //extern int send_event(INT idx, BOOL manual_trig);
//  };
  
  /*-- Equipment list ------------------------------------------------*/
  
  /**
   *  @brief AMC13
   *  @details MIDAS equipment for uTCA with AMC13, FC7, and WFD5 boards
   *  @ingroup group_equipment
   */
  EQUIPMENT equipment[] = { 
    {
      "AMC13%02d",              /* equipment name */
      {1, 0xffff,          /* event ID, trigger mask */
       "BUF%02d",               /* event buffer */
       EQ_POLLED | EQ_EB,       /* equipment type */
       LAM_SOURCE(0, 0xFFFFFF), /* event source crate 0, all stations */
       "MIDAS",                 /* format */
       TRUE,                    /* enabled */
       RO_RUNNING ,              /* read only when running and end of run */
       10,                      /* poll for 1ms */
       0,                       /* stop run after this event limit */
       0,                       /* number of sub events */
       0,                       /* don't log history */
       "", "", "",              /* frontend host, frontend name, frontend file name */
       "", "", FALSE},          /* status, status color, hidden */
      read_trigger_event,       /* readout routine */
    },
    {""}
  };
//#ifdef __cplusplus
//}
//#endif


/*-- Frontend Init -------------------------------------------------*/

/**
 * Frontend Init
 * This routing is called when the frontend program is started. 
 *
 * @return SUCCESS if success
 */

// Verify AMC13 ODB parameter values
INT amc13_odb_check()
{
  std::vector<const char *> messages;

  // T1 Firmware Version Required
  if (amc13_amc13_odb.t1_fw_version > 65535) {
    messages.push_back("/AMC13/: Invalid \"T1 Firmware Version Required\"");
  }

  // T2 Firmware Version Required
  if (amc13_amc13_odb.t2_fw_version > 65535) {
    messages.push_back("/AMC13/: Invalid \"T2 Firmware Version Required\"");
  }

  /*
  // Note: The uhal::HwInterface initialization is now responsible for catching
  //       invalid address table locations. This is because environment variables
  //       are now being used to specify the file locations. The needed logic to
  //       expand the environmet variable is possible but like not needed.

  // T1 Address Table Location
  if (!boost::filesystem::exists(amc13_amc13_odb.addrTab1)) {
    messages.push_back("/AMC13/: Invalid \"T1 Address Table Location\"");
  }

  // T2 Address Table Location
  if (!boost::filesystem::exists(amc13_amc13_odb.addrTab2)) {
    messages.push_back("/AMC13/: Invalid \"T2 Address Table Location\"");
  }
  */

  // Enabled
  if (amc13_link_odb[0].enabled) {
    // AMC13 SFP Port Number
    if (amc13_link_odb[0].source_port > 65535) {
      messages.push_back("/Link01/: Invalid \"AMC13 SFP Port Number\"");
    }

    // AMC13 SFP IP Address
    struct in_addr amc13_sfp_addr;
    if (inet_aton(amc13_link_odb[0].source_ip, &amc13_sfp_addr) == 0) {
      messages.push_back("/Link01/: Invalid \"AMC13 SFP IP Address\"");
    }
  } // if link enabled

  if (messages.size() > 0) {
    for (unsigned int i = 0; i < messages.size(); ++i) {
      cm_msg(MERROR, __FUNCTION__, messages.at(i));
    }
    return FE_ERR_ODB;
  }

  return SUCCESS;
} // amc13_odb_check

// Verify FC7 ODB parameter values
INT fc7_odb_check()
{
  std::vector<const char *> messages;
  std::vector<int> slots;

  for (int i = 0; i < 12; ++i) {
    // Enabled
    // Frontend Configuration Enabled
    if (amc13_fc7_odb[i].common.enabled &&
        amc13_fc7_odb[i].common.fe_config_enabled) {
      /*
      // Note: The uhal::HwInterface initialization is now responsible for catching
      //       invalid address table locations. This is because environment variables
      //       are now being used to specify the file locations. The needed logic to
      //       expand the environmet variable is possible but like not needed.

      // Address Table Location
      if (!boost::filesystem::exists(amc13_fc7_odb[i].common.addr_table_file)) {
        messages.push_back("/FC7-%02i/Common/: Invalid \"Address Table Location\"");
      }
      */

      // Board Type
      boost::algorithm::to_lower( amc13_fc7_odb[i].common.board_type );
      if (strcmp(amc13_fc7_odb[i].common.board_type, "encoder") != 0 &&
          strcmp(amc13_fc7_odb[i].common.board_type, "fanout") != 0 &&
	  strcmp(amc13_fc7_odb[i].common.board_type, "trigger") != 0)  {
        messages.push_back("/FC7-%02i/Common/: Invalid \"Board Type\"");
      }

      // FPGA Firmware Version Required
      struct in_addr fw_ver;
      if (inet_aton(amc13_fc7_odb[i].common.firmware_version, &fw_ver) == 0) {
        messages.push_back("/FC7-%02i/Common/: Invalid \"FPGA Firmware Version Required\"");
      }

      // Threshold: Client Overflow
      if (amc13_fc7_odb[i].encoder.thres_overflow > 16777215) {
        messages.push_back("/FC7-%02i/Encoder/: Invalid \"Threshold: Client Overflow\"");
      }

      // Threshold: Cycle Start Gap
      if (amc13_fc7_odb[i].encoder.thres_cycle_start_gap % 25 != 0) {
        messages.push_back("/FC7-%02i/Encoder/: Invalid \"Threshold: Cycle Start Gap\"");
      }

      // TTS Lock: Total Length
      if (amc13_fc7_odb[i].common.thres_tts_lock % 25 != 0) {
        messages.push_back("/FC7-%02i/Common/: Invalid \"TTS Lock: Total Length\"");
      }

      // TTS Lock: Valid Length
      if (amc13_fc7_odb[i].common.thres_tts_valid % 25 != 0 ||
	  amc13_fc7_odb[i].common.thres_tts_valid > amc13_fc7_odb[i].common.thres_tts_lock) {
        messages.push_back("/FC7-%02i/Common/: Invalid \"TTS Lock: Valid Length\"");
      }

      // Input Trigger Select
      boost::algorithm::to_lower( amc13_fc7_odb[i].encoder.input_trig_sel );
      if (strcmp(amc13_fc7_odb[i].encoder.input_trig_sel, "left") != 0 &&
          strcmp(amc13_fc7_odb[i].encoder.input_trig_sel, "right") != 0)  {
        messages.push_back("/FC7-%02i/Encoder/: Invalid \"Input Trigger Select\"");
      }

      // Post-Reset Delay: Trigger Count
      if (amc13_fc7_odb[i].encoder.post_reset_delay_count % 25 != 0) {
        messages.push_back("/FC7-%02i/Encoder/: Invalid \"Post-Reset Delay: Trigger Count\"");
      }

      // Post-Reset Delay: Timestamp
      if (amc13_fc7_odb[i].encoder.post_reset_delay_timestamp % 25 != 0) {
        messages.push_back("/FC7-%02i/Encoder/: Invalid \"Post-Reset Delay: Timestamp\"");
      }

      // Left Trigger Output: Short Pulse Width
      if (amc13_fc7_odb[i].lotrig.short_pulse_width < 25 ||
          amc13_fc7_odb[i].lotrig.short_pulse_width > 6375 ||
          amc13_fc7_odb[i].lotrig.short_pulse_width % 25 != 0) {
        messages.push_back("/FC7-%02i/Left Trigger Output/: Invalid \"Short Pulse Width\"");
      }

      // Left Trigger Output: Long Pulse Width
      if (amc13_fc7_odb[i].lotrig.long_pulse_width < 25 ||
          amc13_fc7_odb[i].lotrig.long_pulse_width > 6375 ||
          amc13_fc7_odb[i].lotrig.long_pulse_width % 25 != 0) {
        messages.push_back("/FC7-%02i/Left Trigger Output/: Invalid \"Long Pulse Width\"");
      }

      // Left Trigger Output: Trigger Type XX: Width
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_00 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_01 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_02 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_03 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_04 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_05 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_06 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_07 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_08 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_09 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_10 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_11 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_12 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_13 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_14 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_15 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_16 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_17 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_18 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_19 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_20 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_21 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_22 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_23 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_24 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_25 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_26 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_27 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_28 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_29 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_30 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].lotrig.width_31 );

      std::vector<const char *> left_output_width_xx;

      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_00 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_01 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_02 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_03 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_04 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_05 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_06 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_07 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_08 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_09 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_10 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_11 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_12 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_13 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_14 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_15 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_16 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_17 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_18 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_19 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_20 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_21 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_22 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_23 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_24 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_25 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_26 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_27 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_28 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_29 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_30 );
      left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_31 );

      for (int j = 0; j < 32; ++j) {
        if (strcmp(left_output_width_xx.at(j), "disabled") != 0 &&
            strcmp(left_output_width_xx.at(j), "short") != 0 &&
            strcmp(left_output_width_xx.at(j), "long") != 0) {
          messages.push_back("/FC7-%02i/Left Trigger Output/: Invalid \"Trigger Type XX: Width\"");
          break; // send only one message
        }
      }

      // Left Trigger Output: Trigger Type XX: Delay
      std::vector<DWORD> left_output_delay_xx;

      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_00 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_01 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_02 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_03 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_04 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_05 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_06 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_07 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_08 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_09 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_10 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_11 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_12 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_13 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_14 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_15 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_16 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_17 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_18 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_19 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_20 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_21 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_22 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_23 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_24 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_25 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_26 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_27 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_28 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_29 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_30 );
      left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_31 );

      for (int j = 0; j < 32; ++j) {
        if (left_output_delay_xx.at(j) % 25 != 0) {
          messages.push_back("/FC7-%02i/Left Trigger Output/: Invalid \"Trigger Type XX: Delay (ns)\"");
          break; // send only one message
        }
      }

      // Right Trigger Output: Short Pulse Width
      if (amc13_fc7_odb[i].rotrig.short_pulse_width < 25 ||
          amc13_fc7_odb[i].rotrig.short_pulse_width > 6375 ||
          amc13_fc7_odb[i].rotrig.short_pulse_width % 25 != 0) {
        messages.push_back("/FC7-%02i/Right Trigger Output/: Invalid \"Short Pulse Width\"");
      }

      // Right Trigger Output: Long Pulse Width
      if (amc13_fc7_odb[i].rotrig.long_pulse_width < 25 ||
          amc13_fc7_odb[i].rotrig.long_pulse_width > 6375 ||
          amc13_fc7_odb[i].rotrig.long_pulse_width % 25 != 0) {
        messages.push_back("/FC7-%02i/Right Trigger Output/: Invalid \"Long Pulse Width\"");
      }

      // Right Trigger Output: Trigger Type XX: Width
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_00 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_01 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_02 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_03 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_04 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_05 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_06 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_07 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_08 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_09 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_10 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_11 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_12 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_13 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_14 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_15 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_16 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_17 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_18 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_19 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_20 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_21 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_22 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_23 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_24 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_25 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_26 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_27 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_28 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_29 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_30 );
      boost::algorithm::to_lower( amc13_fc7_odb[i].rotrig.width_31 );

      std::vector<const char *> right_output_width_xx;

      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_00 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_01 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_02 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_03 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_04 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_05 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_06 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_07 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_08 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_09 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_10 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_11 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_12 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_13 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_14 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_15 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_16 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_17 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_18 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_19 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_20 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_21 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_22 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_23 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_24 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_25 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_26 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_27 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_28 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_29 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_30 );
      right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_31 );

      for (int j = 0; j < 32; ++j) {
        if (strcmp(left_output_width_xx.at(j), "disabled") != 0 &&
            strcmp(left_output_width_xx.at(j), "short") != 0 &&
            strcmp(left_output_width_xx.at(j), "long") != 0) {
          messages.push_back("/FC7-%02i/Right Trigger Output/: Invalid \"Trigger Type XX: Width\"");
          break; // send only one message
        }
      }

      // Right Trigger Output: Trigger Type XX: Delay
      std::vector<DWORD> right_output_delay_xx;

      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_00 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_01 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_02 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_03 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_04 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_05 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_06 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_07 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_08 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_09 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_10 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_11 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_12 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_13 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_14 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_15 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_16 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_17 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_18 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_19 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_20 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_21 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_22 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_23 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_24 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_25 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_26 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_27 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_28 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_29 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_30 );
      right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_31 );

      for (int j = 0; j < 32; ++j) {
        if (left_output_delay_xx.at(j) % 25 != 0) {
          messages.push_back("/FC7-%02i/Right Trigger Output/: Invalid \"Trigger Type XX: Delay (ns)\"");
          break; // send only one message
        }
      }

      for (unsigned int j = 0; j < messages.size(); ++j) {
        slots.push_back(i+1);
      }
    } // if board enabled
  } // for slot

  if (messages.size() > 0) {
    for (unsigned int i = 0; i < messages.size(); ++i) {
      cm_msg(MERROR, __FUNCTION__, messages.at(i), slots.at(i));
    }
    return FE_ERR_ODB;
  }

  return SUCCESS;
} // fc7_odb_check

// Verify WFD5 ODB parameter values
INT wfd_odb_check()
{
  std::vector<const char *> messages;
  std::vector<int> slots, channels;

  for (int i = 0; i < 12; ++i) {
    // Enabled
    // Frontend Configuration Enabled
    if (amc13_rider_odb[i].board.rider_enabled &&
        amc13_rider_odb[i].board.fe_config_enabled) {
      /*
      // Note: The uhal::HwInterface initialization is now responsible for catching
      //       invalid address table locations. This is because environment variables
      //       are now being used to specify the file locations. The needed logic to
      //       expand the environmet variable is possible but like not needed.

      // Address Table Location
      if (!boost::filesystem::exists(amc13_rider_odb[i].board.addr_table_file)) {
        messages.push_back("/WFD5-%02i/Board/: Invalid \"Address Table Location\"");
      }
      */

      // FPGA Firmware Version Required
      struct in_addr mfw_ver;
      if (inet_aton(amc13_rider_odb[i].board.firmware_version, &mfw_ver) == 0) {
        messages.push_back("/WFD5-%02i/Board/: Invalid \"FPGA Firmware Version Required\"");
      }

      // Digitization Frequency
      if (strcmp(amc13_rider_odb[i].board.digitizer_freq, "800") != 0 &&
          strcmp(amc13_rider_odb[i].board.digitizer_freq, "600") != 0 &&
          strcmp(amc13_rider_odb[i].board.digitizer_freq, "400") != 0 &&
          strcmp(amc13_rider_odb[i].board.digitizer_freq, "300") != 0 &&
          strcmp(amc13_rider_odb[i].board.digitizer_freq, "200") != 0 &&
          strcmp(amc13_rider_odb[i].board.digitizer_freq, "100") != 0 &&
          strcmp(amc13_rider_odb[i].board.digitizer_freq, "80") != 0 &&
          strcmp(amc13_rider_odb[i].board.digitizer_freq, "60") != 0 &&
          strcmp(amc13_rider_odb[i].board.digitizer_freq, "40") != 0)  {
        messages.push_back("/WFD5-%02i/Board/: Invalid \"Digitization Frequency (MHz)\"");
      }

      // Front Panel Clock Enabled
      if (amc13_rider_odb[i].board.fp_clock_enabled) {
        // Front Panel Clock Frequency
        boost::algorithm::to_lower( amc13_rider_odb[i].board.fp_clock_freq );
        if (strcmp(amc13_rider_odb[i].board.fp_clock_freq, "ttc") != 0 &&
            strcmp(amc13_rider_odb[i].board.fp_clock_freq, "digitization") != 0) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Front Panel Clock Frequency\"");
        }
      }

      // ADC Data Endianness
      boost::algorithm::to_lower( amc13_rider_odb[i].board.ADC_endianness );
      if (strcmp(amc13_rider_odb[i].board.ADC_endianness, "big") != 0 &&
          strcmp(amc13_rider_odb[i].board.ADC_endianness, "little") != 0) {
        messages.push_back("/WFD5-%02i/Board/: Invalid \"ADC Data Endianness\"");
      }

      // Trigger Delay
      if (amc13_rider_odb[i].board.trig_delay % 25 != 0) {
        messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Delay (ns)\"");
      }

      // Async Mode: Enabled
      if (amc13_rider_odb[i].board.asyncmode_enabled) {
        // Async Mode: Waveform Length
        if (amc13_rider_odb[i].board.asyncmode_wvfm_length > 131064 ||
            amc13_rider_odb[i].board.asyncmode_wvfm_length % 8 != 0) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Async Mode: Waveform Length\"");
        }

        // Async Mode: Waveform Presamples
        if (amc13_rider_odb[i].board.asyncmode_wvfm_presamples > 131064 ||
            amc13_rider_odb[i].board.asyncmode_wvfm_length % 2 != 0) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Async Mode: Waveform Presamples\"");
        }
      }

      // Trigger Type 1: Enabled
      if (amc13_rider_odb[i].board.trig1_enabled) {
        // Trigger Type 1: Waveform Count
        if (amc13_rider_odb[i].board.trig1__wvfm_count > 4095) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Type 1: Waveform Count\"");
        }

        // Trigger Type 1: Waveform Length
        if (amc13_rider_odb[i].board.trig1__wvfm_length > 66944960 ||
            amc13_rider_odb[i].board.trig1__wvfm_length % 8 != 0) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Type 1: Waveform Length\"");
        }

        // Trigger Type 1: Waveform Gap
        if (amc13_rider_odb[i].board.trig1__wvfm_gap < 16 ||
            amc13_rider_odb[i].board.trig1__wvfm_gap > 8388606 ||
            amc13_rider_odb[i].board.trig1__wvfm_gap % 2 != 0) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Type 1: Waveform Gap\"");
        }
      }

      // Trigger Type 2: Enabled
      if (amc13_rider_odb[i].board.trig2_enabled) {
        // Trigger Type 2: Waveform Count
        if (amc13_rider_odb[i].board.trig2__wvfm_count > 4095) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Type 2: Waveform Count\"");
        }

        // Trigger Type 2: Waveform Length
        if (amc13_rider_odb[i].board.trig2__wvfm_length > 66944960 ||
            amc13_rider_odb[i].board.trig2__wvfm_length % 8 != 0) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Type 2: Waveform Length\"");
        }

        // Trigger Type 2: Waveform Gap
        if (amc13_rider_odb[i].board.trig2__wvfm_gap < 16 ||
            amc13_rider_odb[i].board.trig2__wvfm_gap > 8388606 ||
            amc13_rider_odb[i].board.trig2__wvfm_gap % 2 != 0) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Type 2: Waveform Gap\"");
        }
      }

      // Trigger Type 3: Enabled
      if (amc13_rider_odb[i].board.trig3_enabled) {
        // Trigger Type 3: Waveform Count
        if (amc13_rider_odb[i].board.trig3__wvfm_count > 4095) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Type 3: Waveform Count\"");
        }

        // Trigger Type 3: Waveform Length
        if (amc13_rider_odb[i].board.trig3__wvfm_length > 66944960 ||
            amc13_rider_odb[i].board.trig3__wvfm_length % 8 != 0) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Type 3: Waveform Length\"");
        }

        // Trigger Type 3: Waveform Gap
        if (amc13_rider_odb[i].board.trig3__wvfm_gap < 16 ||
            amc13_rider_odb[i].board.trig3__wvfm_gap > 8388606 ||
            amc13_rider_odb[i].board.trig3__wvfm_gap % 2 != 0) {
          messages.push_back("/WFD5-%02i/Board/: Invalid \"Trigger Type 3: Waveform Gap\"");
        }
      }

      // Error Threshold: DDR3 Overflow
      if (amc13_rider_odb[i].board.error_thres_DDR3ovflw > 8388608) {
        messages.push_back("/WFD5-%02i/Board/: Invalid \"Error Threshold: DDR3 Overflow\"");
      }

      for (unsigned int j = 0; j < messages.size(); ++j) {
        slots.push_back(i+1);
        channels.push_back(-1); // not used
      }

      for (int j = 0; j < 5; ++j) {
        // Enabled
        // Frontend Configuration Enabled
        if (amc13_rider_odb[i].channel[j].enabled &&
            amc13_rider_odb[i].channel[j].fe_config_enabled) {
          // FPGA Firmware Version Required
          struct in_addr cfw_ver;
          if (inet_aton(amc13_rider_odb[i].channel[j].FPGA_firmware_version, &cfw_ver) == 0) {
            messages.push_back("/WFD5-%02i/Channel%02i/: Invalid \"FPGA Firmware Version Required\"");
            slots.push_back(i+1);
            channels.push_back(j);
          }

          // Input Signal Offset
          if (amc13_rider_odb[i].channel[j].input_signal_offset < 0 ||
              amc13_rider_odb[i].channel[j].input_signal_offset > 65535) {
            messages.push_back("/WFD5-%02i/Channel%02i/: Invalid \"Input Signal Offset\"");
            slots.push_back(i+1);
            channels.push_back(j);
          }
        } // if channel enabled
      } // for channel
    } // if board enabled
  } // for slot

  if (messages.size() > 0) {
    for (unsigned int i = 0; i < messages.size(); ++i) {
      cm_msg(MERROR, __FUNCTION__, messages.at(i), slots.at(i), channels.at(i));
    }
    return FE_ERR_ODB;
  }

  return SUCCESS;
} // wfd_odb_check

INT frontend_init_fc7()
{
  /* Level 1 initialization */
  for (int i = 0; i < 12; ++i) {
    if (amc13_fc7_odb[i].common.enabled) {

      // read device address
      std::string fc7_ip = fc7lib->getAddress(10, 2, amc13_settings_odb.mch_ip_addr, (i+1));
      printf("%s(%d): Slot %02i: Read FC7 IP Address: %s \n", __FUNCTION__, __LINE__, (i+1), fc7_ip.c_str());

      if (fc7_ip == "") {
        cm_msg(MERROR, __FUNCTION__, "FC7-%02i: IP Address Read Failed", (i+1));
        return FE_ERR_HW;
      }

      // initialize device object
      try {
        std::stringstream uri; uri << "ipbusudp-2.0://" << fc7_ip << ":50001";
        std::stringstream atf; atf << "file://" << amc13_fc7_odb[i].common.addr_table_file;
        fc7[i] = new uhal::HwInterface( uhal::ConnectionManager::getDevice("hw_id", uri.str(), atf.str()) );
      } catch (uhal::exception::exception& e) {
        cm_msg(MERROR, __FUNCTION__, "/FC7-%02i/Common/: Invalid \"Address Table Location\"", (i+1));
        printf("%s(%d): uHAL Exception: %s \n", __FUNCTION__, __LINE__, e.what());
        return FE_ERR_ODB;
      }

    } // if slot enabled
  } // for slot

  for (int i = 0; i < 12; ++i) {
    if (amc13_fc7_odb[i].common.enabled) {
      
      // Note: Do not issue a hard reset here because it would disable all
      //       of the SFP ports that were enabled by MasterGM2 init.

      // firmware soft reset
      sleep(1); // inter-command delay
      printf("%s(%d): Slot %02i: FC7 Firmware Soft Reset \n", __FUNCTION__, __LINE__, (i+1));
      if (fc7lib->resetFirmware(3, 1, fc7[i], 0) == -1) {
        cm_msg(MERROR, __FUNCTION__, "FC7-%02i: FPGA Communication Failed", (i+1));
        return FE_ERR_HW;
      }

    } // if slot enabled
  } // for slot

  // post-reset delay
  printf("%s(%d): Waiting 5 s ... \n", __FUNCTION__, __LINE__);
  sleep(5);

  /* Level 1 configuration */
  double inter_cmd_delay = 0.1;

  for (int i = 0; i < 12; ++i) {
    if (amc13_fc7_odb[i].common.enabled) {

      printf("%s(%d): Slot %02i: FC7 Configuration Initiated \n", __FUNCTION__, __LINE__, (i+1));

      // Frontend Configuration Enabled
      if (amc13_fc7_odb[i].common.fe_config_enabled) {
        // FPGA Firmware Version
        char addr_hex[9];
        sprintf(addr_hex, "%08x", inet_addr(amc13_fc7_odb[i].common.firmware_version));
        std::stringstream ss_fw_ver;
        ss_fw_ver << addr_hex[6] << addr_hex[7];
        ss_fw_ver << addr_hex[4] << addr_hex[5];
        ss_fw_ver << addr_hex[0] << addr_hex[1];
        std::string fw_ver_str = ss_fw_ver.str();
        int fw_ver = strtol(fw_ver_str.c_str(), NULL, 16);

        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Read: FPGA Firmware Version \n", __FUNCTION__, __LINE__, (i+1));
        int val_read = fc7lib->getFirmwareVersion(3, 1, fc7[i]);
        if (val_read == -1) {
          cm_msg(MERROR, __FUNCTION__, "FC7-%02i: FPGA Communication Failed", (i+1));
          return FE_ERR_HW;
        } else if (val_read != fw_ver) {
          cm_msg(MERROR, __FUNCTION__, "/FC7-%02i/Common/: Conflict: \"FPGA Firmware Version\"", (i+1));
          return FE_ERR_HW;
        }

        // Board Type
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Read: Board Type \n", __FUNCTION__, __LINE__, (i+1));
        int brd_type = 1; // encoder
        if (strcmp(amc13_fc7_odb[i].common.board_type, "fanout") == 0) {
          brd_type = 2;
        } else if (strcmp(amc13_fc7_odb[i].common.board_type, "trigger") == 0) {
          brd_type = 3;
        }
        val_read = fc7lib->getBoardType(2, 1, fc7[i]);
        printf("i = %d, val_read = %d, brd_type = %d\n",i,val_read,brd_type);

        if (val_read == -1) {
          cm_msg(MERROR, __FUNCTION__, "FC7-%02i: FPGA Communication Failed", (i+1));
          return FE_ERR_HW;
        } else if (val_read != brd_type) {
          cm_msg(MERROR, __FUNCTION__, "/FC7-%02i/Common/: Conflict: \"Board Type\"", (i+1));
          return FE_ERR_HW;
        }

        // cache the encoder and trigger fc7 slot numbers
        if ( brd_type == 1) {
           encoder_fc7_slot = i;
        } else if ( brd_type == 3 ) {
           trigger_fc7_slot = i;
        }

        // FMC Startup Status
        if (fc7lib->getStartupStatus(2, 1, fc7[i]) != 1) {
          // Note: The recovery procedure from this error would be to issue a
          //       hard reset, but such a reset should not be issued because it
          //       would disable all of the SFP ports.

          cm_msg(MERROR, __FUNCTION__, "FC7-%02i: FMC Startup Failed", (i+1));
          return FE_ERR_HW;
        }

        int code = 0;
        bool print_code = false;

        // Threshold: TTC Single Bit Error
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: Threshold: TTC Single-Bit Error \n", __FUNCTION__, __LINE__, (i+1));
        code += fc7lib->setThreshold(2, 1, fc7[i], 1, amc13_fc7_odb[i].common.thres_ttc_sbit_error);
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

        // Threshold: TTC Multi Bit Error
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: Threshold: TTC Multi-Bit Error \n", __FUNCTION__, __LINE__, (i+1));
        code += fc7lib->setThreshold(2, 1, fc7[i], 2, amc13_fc7_odb[i].common.thres_ttc_mbit_error);
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

	// TTS Lock: Total Length
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: TTS Lock: Total Length \n", __FUNCTION__, __LINE__, (i+1));
        code += fc7lib->setThreshold(2, 1, fc7[i], 0, amc13_fc7_odb[i].common.thres_tts_lock);
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

	// TTS Lock: Valid Length
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: TTS Lock: Valid Length \n", __FUNCTION__, __LINE__, (i+1));
        code += fc7lib->setThreshold(2, 1, fc7[i], 5, (amc13_fc7_odb[i].common.thres_tts_lock - amc13_fc7_odb[i].common.thres_tts_valid));
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);


        if (strcmp(amc13_fc7_odb[i].common.board_type, "encoder") == 0) {
          // Threshold: Client Overflow
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Threshold: Client Overflow \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setThreshold(2, 1, fc7[i], 3, amc13_fc7_odb[i].encoder.thres_overflow);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          // Threshold: Cycle Start Gap
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Threshold: Cycle Start Gap \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setThreshold(2, 1, fc7[i], 4, amc13_fc7_odb[i].encoder.thres_cycle_start_gap);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          // WFD5 Async Mode Enabled
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: WFD5 Async Mode Enabled \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setAyncModeEnable(2, 1, fc7[i], amc13_fc7_odb[i].encoder.wfd_async_enabled);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          // Input Trigger Select
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Input Trigger Select \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setInputTriggerSelect(2, 1, fc7[i], amc13_fc7_odb[i].encoder.input_trig_sel);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

	       // Post-Reset Delay: Trigger Count
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Post-Reset Delay: Trigger Count \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setPostResetDelay(2, 1, fc7[i], 1, amc13_fc7_odb[i].encoder.post_reset_delay_count);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

	       // Post-Reset Delay: Timestamp
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Post-Reset Delay: Timestamp \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setPostResetDelay(2, 1, fc7[i], 2, amc13_fc7_odb[i].encoder.post_reset_delay_timestamp);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          // --------- These settings are now nonsense.  They predate the trigger fc7 and were meant to provide a couple of
          // analog pulse outputs, in particular for the laser system.  We should clean up these functions and the ODB
          /* ---
          // Left Trigger Output: Short Pulse Width
          // Left Trigger Output: Long Pulse Width
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Left Trigger Output: Short/Long Pulse Width \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setOutputWidthSettings(2, 1, fc7[i], "A", amc13_fc7_odb[i].lotrig.short_pulse_width, amc13_fc7_odb[i].lotrig.long_pulse_width);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          // Left Trigger Output: Trigger Type XX: Width
          std::vector<const char *> left_output_width_xx;

          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_00 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_01 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_02 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_03 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_04 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_05 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_06 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_07 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_08 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_09 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_10 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_11 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_12 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_13 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_14 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_15 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_16 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_17 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_18 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_19 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_20 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_21 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_22 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_23 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_24 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_25 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_26 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_27 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_28 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_29 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_30 );
          left_output_width_xx.push_back( amc13_fc7_odb[i].lotrig.width_31 );

          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Left Trigger Output: Trigger Type XX: Width \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setOutputWidth(2, 1, fc7[i], "A", left_output_width_xx);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          // Left Trigger Output: Trigger Typer XX: Delay
          std::vector<int> left_output_delay_xx;

          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_00 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_01 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_02 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_03 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_04 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_05 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_06 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_07 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_08 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_09 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_10 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_11 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_12 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_13 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_14 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_15 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_16 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_17 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_18 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_19 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_20 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_21 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_22 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_23 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_24 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_25 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_26 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_27 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_28 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_29 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_30 );
          left_output_delay_xx.push_back( amc13_fc7_odb[i].lotrig.delay_31 );

          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Left Trigger Output: Trigger Type XX: Delay \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setOutputDelay(2, 1, fc7[i], "A", left_output_delay_xx);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          // Right Trigger Output: Short Pulse Width
          // Right Trigger Output: Long Pulse Width
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Right Trigger Output: Short/Long Pulse Width \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setOutputWidthSettings(2, 1, fc7[i], "B", amc13_fc7_odb[i].rotrig.short_pulse_width, amc13_fc7_odb[i].rotrig.long_pulse_width);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          // Right Trigger Output: Trigger Type XX: Width
          std::vector<const char *> right_output_width_xx;

          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_00 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_01 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_02 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_03 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_04 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_05 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_06 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_07 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_08 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_09 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_10 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_11 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_12 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_13 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_14 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_15 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_16 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_17 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_18 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_19 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_20 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_21 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_22 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_23 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_24 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_25 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_26 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_27 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_28 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_29 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_30 );
          right_output_width_xx.push_back( amc13_fc7_odb[i].rotrig.width_31 );

          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Right Trigger Output: Trigger Type XX: Width \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setOutputWidth(2, 1, fc7[i], "B", right_output_width_xx);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          // Right Trigger Output: Trigger Typer XX: Delay
          std::vector<int> right_output_delay_xx;

          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_00 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_01 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_02 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_03 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_04 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_05 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_06 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_07 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_08 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_09 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_10 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_11 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_12 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_13 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_14 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_15 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_16 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_17 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_18 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_19 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_20 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_21 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_22 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_23 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_24 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_25 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_26 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_27 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_28 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_29 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_30 );
          right_output_delay_xx.push_back( amc13_fc7_odb[i].rotrig.delay_31 );

          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Right Trigger Output: Trigger Type XX: Delay \n", __FUNCTION__, __LINE__, (i+1));
          code += fc7lib->setOutputDelay(2, 1, fc7[i], "B", right_output_delay_xx);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);
          --- end of obsolete code that should be updated along with ODB */
        } // if encoder

        if (code != 0) {
          cm_msg(MERROR, __FUNCTION__, "FC7-%02i: Configuration Failed", (i+1));
          return FE_ERR_HW;
        }

        // TTS RX Reset
        if (strcmp(amc13_fc7_odb[i].common.board_type, "fanout") == 0) {
          sleep(inter_cmd_delay); // inter-command delay
          if (fc7lib->resetReceiver(2, 1, fc7[i], false) == -1) {
            cm_msg(MERROR, __FUNCTION__, "FC7-%02i: FPGA Communication Failed", (i+1));
            return FE_ERR_HW;
          }
        } // if fanout

        sleep(inter_cmd_delay); // inter-command delay
        if (fc7lib->resetReceiver(2, 1, fc7[i], true) == -1) {
          cm_msg(MERROR, __FUNCTION__, "FC7-%02i: FPGA Communication Failed", (i+1));
          return FE_ERR_HW;
        }

      } // if config enabled

    } // if slot enabled
  } // for slot

  return SUCCESS;
}

INT frontend_init_wfd()
{
  /* Level 1 initialization */
  int en_cnt = 0;
  int up_cnt = 0;
  std::vector<int> v_slot;

  printf("%s(%d): WFD5 Board Presence Check \n", __FUNCTION__, __LINE__);
  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      // board presence check
      int absent = wfdlib->getBoardAbsence(2, 10, amc13_settings_odb.mch_ip_addr, (i+1)); // typical boot time: 56 s

      switch (absent) {
        case  1: printf("%s(%d): WFD5-%02i: Unable to Detect Board \n", __FUNCTION__, __LINE__, (i+1));
                 v_slot.push_back(i+1);
                 break;
        case -1: printf("%s(%d): WFD5-%02i: IPMI Communication Failed \n", __FUNCTION__, __LINE__, (i+1));
                 v_slot.push_back(i+1);
                 break;
        default: up_cnt++;
	               break;
      }

      en_cnt++;

    } // if slot enabled
  } // for slot

  printf("%s(%d): WFD5 IPMI Communication Check: %i/%i \n", __FUNCTION__, __LINE__, up_cnt, en_cnt);

  if (0 < v_slot.size() && int(v_slot.size()) < en_cnt) {

    for (unsigned int i = 0; i < v_slot.size(); ++i) {
      // turn off backplane power to slot
      if (wfdlib->setBackplanePower(1, 0, amc13_settings_odb.mch_ip_addr, v_slot.at(i), "down") == -1) {
        cm_msg(MERROR, __FUNCTION__, "Slot %02i: Unable to Disable Backplane Power", v_slot.at(i));
        return FE_ERR_HW;
      }

      printf("%s(%d): Slot %02i: Disabled Backplane Power \n", __FUNCTION__, __LINE__, v_slot.at(i));
    }

    printf("%s(%d): Waiting 5 s ... \n", __FUNCTION__, __LINE__);
    sleep(5);

    for (unsigned int i = 0; i < v_slot.size(); ++i) {
      // turn on backplane power to slot
      if (wfdlib->setBackplanePower(1, 0, amc13_settings_odb.mch_ip_addr, v_slot.at(i), "up") == -1) {
        cm_msg(MERROR, __FUNCTION__, "Slot %02i: Unable to Enable Backplane Power", v_slot.at(i));
        return FE_ERR_HW;
      }

      printf("%s(%d): Slot %02i: Enabled Backplane Power \n", __FUNCTION__, __LINE__, v_slot.at(i));
    }

    printf("%s(%d): Waiting 60 s ... \n", __FUNCTION__, __LINE__);
    sleep(60);

    printf("%s(%d): WFD5 Board Presence Check \n", __FUNCTION__, __LINE__);
    for (unsigned int i = 0; i < v_slot.size(); ++i) {
      // board presence check
      int absent = wfdlib->getBoardAbsence(2, 10, amc13_settings_odb.mch_ip_addr, v_slot.at(i));

      switch (absent) {
        case  1: cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: Board Absent", v_slot.at(i));
                 return FE_ERR_HW;
        case -1: cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: IPMI Communication Failed", v_slot.at(i));
                 return FE_ERR_HW;
      }
    }

  } // if failed slots

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {
      // read device address
      std::string wfd_ip = wfdlib->getAddress(2, 1, amc13_settings_odb.mch_ip_addr, (i+1));

      if (wfd_ip == "") {
	      // MMC reset
        wfdlib->resetMMC(amc13_settings_odb.mch_ip_addr, (i+1));
	
      	sleep(2); // post-reset delay, required
      	wfd_ip = wfdlib->getAddress(5, 1, amc13_settings_odb.mch_ip_addr, (i+1));

      	if (wfd_ip == "") {
      	  cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: IP Address Read Failed", (i+1));
      	  return FE_ERR_HW;
      	}
      }

      printf("%s(%d): Slot %02i: Read WFD5 IP Address: %s \n", __FUNCTION__, __LINE__, (i+1), wfd_ip.c_str());

      // initialize device object
      try {
        std::stringstream uri; uri << "ipbusudp-2.0://" << wfd_ip << ":50001";
        std::stringstream atf; atf << "file://" << amc13_rider_odb[i].board.addr_table_file;
        wfd[i] = new uhal::HwInterface( uhal::ConnectionManager::getDevice("hw_id", uri.str(), atf.str()) );
      } catch (uhal::exception::exception& e) {
        cm_msg(MERROR, __FUNCTION__, "/WFD5-%02i/Board/: Invalid \"Address Table Location\"", (i+1));
        printf("%s(%d): uHAL Exception: %s \n", __FUNCTION__, __LINE__, e.what());
        return FE_ERR_ODB;
      }

    } // if slot enabled
  } // for slot

  /* Level 2 initialization */
  up_cnt = 0;

  // Note: This quickly scans the AMC modules to see if
  //       any one is responsive to Ethernet.

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {
      
      // lower timeout duration
      wfd[i]->setTimeoutPeriod(100);

      // Ethernet communication check
      if (wfdlib->getFirmwareImage(2, 1, wfd[i]) != -1) {
        up_cnt++;
      }

      // restore default timeout duration
      wfd[i]->setTimeoutPeriod(1000);

    } // if slot enabled
  } // for slot

  printf("%s(%d): WFD5 Ethernet Communication Check: %i/%i \n", __FUNCTION__, __LINE__, up_cnt, en_cnt);

  if (up_cnt == 0 &&
      en_cnt != 0) {
    printf("%s(%d): Waiting 30 s ... \n", __FUNCTION__, __LINE__);
    sleep(30);
  } else if (up_cnt > 0 &&
             up_cnt < en_cnt) {
    printf("%s(%d): Waiting 10 s ... \n", __FUNCTION__, __LINE__);
    sleep(10);
  }

  up_cnt = 0;
  int images[12];

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      // Ethernet communication check
      int image = wfdlib->getFirmwareImage(2, 1, wfd[i]);

      if (image == -1) {
        printf("%s(%d): Slot %02i: WFD5 FPGA Reboot Issued \n", __FUNCTION__, __LINE__, (i+1));

        // MMC reset
        wfdlib->resetMMC(amc13_settings_odb.mch_ip_addr, (i+1));

        // FPGA reboot
        wfdlib->programFirmware(amc13_settings_odb.mch_ip_addr, (i+1));
      } else {
        up_cnt++;
      }

      images[i] = image;

    } // if slot enabled
  } // for slot

  if (up_cnt != en_cnt) {
    printf("%s(%d): Waiting 45 s ... \n", __FUNCTION__, __LINE__);
    sleep(45); // typical boot time: 36 s
  }

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      if (images[i] == -1) {
        // Ethernet communication check
        int image = wfdlib->getFirmwareImage(3, 5, wfd[i]);

        if (image == -1) {
          cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
          return FE_ERR_HW;
        }

        images[i] = image;
      }

    } // if slot enabled
  } // for slot

  /* Level 3 initialization */
  int mfw_cnt = 0;

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      if (images[i] == 1) {
        // program master firmware
        printf("%s(%d): Slot %02i: WFD5 Master Firmware Load Request \n", __FUNCTION__, __LINE__, (i+1));
        if (wfdlib->programFirmware(2, 1, wfd[i], 1) == -1) {
          cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
          return FE_ERR_HW;
        }
      } else {
        mfw_cnt++;
      }

    } // if slot enabled
  } // for slot

  if (mfw_cnt != en_cnt) {
    printf("%s(%d): Waiting 45 s ... \n", __FUNCTION__, __LINE__);
    sleep(45); // typical boot time: 36 s
  }

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      if (images[i] == 1) {
        // firmware image check
        int image = wfdlib->getFirmwareImage(2, 1, wfd[i]);
        switch (image) {
          case  1: cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: Master Firmware Corrupted", (i+1));
                   return FE_ERR_HW;
          case -1: cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
                   return FE_ERR_HW;
        }
      }

    } // if slot enabled
  } // for slot

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      // firmware hard reset
      sleep(0.1); // inter-command delay
      printf("%s(%d): Slot %02i: WFD5 Firmware Hard Reset \n", __FUNCTION__, __LINE__, (i+1));
      if (wfdlib->resetFirmware(3, 1, wfd[i]) == -1) {
        cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
        return FE_ERR_HW;
      }

    } // if slot enabled
  } // for slot

  if (en_cnt > 0) {
    // post-reset delay
    printf("%s(%d): Waiting 5 s ... \n", __FUNCTION__, __LINE__);
    sleep(5);
  }

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {
      
      // test connection post-reset quickly
      sleep(0.1); // inter-command delay
      if (wfdlib->getFirmwareVersion(5, 0.1, wfd[i]) == -1) {
      	cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
      	return FE_ERR_HW;
      }

      // program channel firmware
      sleep(0.1); // inter-command delay
      printf("%s(%d): Slot %02i: WFD5 Channel Firmware Load Request \n", __FUNCTION__, __LINE__, (i+1));
      if (wfdlib->programFirmware(2, 1, wfd[i], 2) == -1) {
        cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
        return FE_ERR_HW;
      }

    } // if slot enabled
  } // for slot

  if (en_cnt > 0) {
    // post-program delay
    printf("%s(%d): Waiting 1 s ... \n", __FUNCTION__, __LINE__);
    sleep(1);
  }

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      // channel firmware program check
      int done = wfdlib->getChannelFirmwareState(2, 1, wfd[i]);
      switch (done) {
        case  0: cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: Channel Firmware Corrupted", (i+1));
                 return FE_ERR_HW;
        case -1: cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
                 return FE_ERR_HW;
      }

    } // if slot enabled
  } // for slot

  /* Level 1 configuration */
  double inter_cmd_delay = 0.1;

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      // Frontend Configuration Enabled
      if (amc13_rider_odb[i].board.fe_config_enabled) {
        printf("%s(%d): Slot %02i: WFD5 Configuration 1/3 Initiated \n", __FUNCTION__, __LINE__, (i+1));
        int code = 0;

        // Master FPGA Firmware Version
        char addr_hex[9];
        sprintf(addr_hex, "%08x", inet_addr(amc13_rider_odb[i].board.firmware_version));
        std::stringstream ss_fw_ver;
        ss_fw_ver << addr_hex[6] << addr_hex[7];
        ss_fw_ver << addr_hex[4] << addr_hex[5];
        ss_fw_ver << addr_hex[0] << addr_hex[1];
        std::string fw_ver_str = ss_fw_ver.str();
        int fw_ver = strtol(fw_ver_str.c_str(), NULL, 16);

        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Read: Master FPGA Firmware Version \n", __FUNCTION__, __LINE__, (i+1));
        int ver_read = wfdlib->getFirmwareVersion(5, 1, wfd[i]);
        if (ver_read == -1) {
          cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
          return FE_ERR_HW;
        } else if (ver_read != fw_ver) {
          cm_msg(MERROR, __FUNCTION__, "/WFD5-%02i/Board/: Conflict: \"FPGA Firmware Version\"", (i+1));
          return FE_ERR_HW;
        }

        // Digitization Frequency
        // Front Panel Clock Enabled
        // Front Panel Clock Frequency
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: Clock Synthesizer Configuration \n", __FUNCTION__, __LINE__, (i+1));
        code += wfdlib->configureClock(2, 1, wfd[i], amc13_rider_odb[i].board.digitizer_freq, amc13_rider_odb[i].board.fp_clock_enabled, amc13_rider_odb[i].board.fp_clock_freq);

        // Input Signal Offset
        int offsets[5] = {-1, -1, -1, -1, -1};
        for (int j = 0; j < 5; ++j) {
          // Frontend Configuration Enabled
          if (amc13_rider_odb[i].channel[j].fe_config_enabled) {

            offsets[j] = (amc13_rider_odb[i].channel[j].enabled) ? amc13_rider_odb[i].channel[j].input_signal_offset : 0;

          } // if channel enabled
        } // for channel

        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: Input Signal Offset \n", __FUNCTION__, __LINE__, (i+1));
        code += wfdlib->setInputSignalOffset(2, 1, wfd[i], offsets[0], offsets[1], offsets[2], offsets[3], offsets[4]);

        if (code != 0) {
          cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: Configuration Failed", (i+1));
          return FE_ERR_HW;
        }
      } // if config enabled

    } // if slot enabled
  } // for slot

  if (en_cnt > 0) {
    // post-configuration delay
    printf("%s(%d): Waiting 1 s ... \n", __FUNCTION__, __LINE__);
    sleep(1);
  }

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      // clear interface control bits
      if (wfdlib->clearControlBit(2, 1, wfd[i], 0) == -1) {
        cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
        return FE_ERR_HW;
      }

      if (wfdlib->clearControlBit(2, 1, wfd[i], 1) == -1) {
        cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
        return FE_ERR_HW;
      }

      // firmware hard reset
      sleep(inter_cmd_delay); // inter-command delay
      printf("%s(%d): Slot %02i: WFD5 Firmware Hard Reset \n", __FUNCTION__, __LINE__, (i+1));
      if (wfdlib->resetFirmware(3, 1, wfd[i]) == -1) {
        cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
        return FE_ERR_HW;
      }

    } // if slot enabled
  } // for slot

  if (en_cnt > 0) {
    // post-reset delay
    printf("%s(%d): Waiting 5 s ... \n", __FUNCTION__, __LINE__);
    sleep(5);
  }

  /* Level 2 configuration */
  int mode_cnt = 0;

  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      // Frontend Configuration Enabled
      if (amc13_rider_odb[i].board.fe_config_enabled) {
        printf("%s(%d): Slot %02i: WFD5 Configuration 2/3 Initiated \n", __FUNCTION__, __LINE__, (i+1));
        int code = 0;
        bool print_code = false;

        // test connection post-reset quickly
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Read: Master FPGA Firmware Version \n", __FUNCTION__, __LINE__, (i+1));
        if (wfdlib->getFirmwareVersion(5, 0.1, wfd[i]) == -1) {
          cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
          return FE_ERR_HW;
        }

        for (int j = 0; j < 5; ++j) {
          // Enabled
          // Frontend Configuration Enabled
          if (amc13_rider_odb[i].channel[j].enabled &&
              amc13_rider_odb[i].channel[j].fe_config_enabled) {
            // FPGA Firmware Version
	          char addr_hex[9];
            sprintf(addr_hex, "%08x", inet_addr(amc13_rider_odb[i].channel[j].FPGA_firmware_version));
	          std::stringstream ss_fw_ver;
            ss_fw_ver << addr_hex[6] << addr_hex[7];
            ss_fw_ver << addr_hex[4] << addr_hex[5];
            ss_fw_ver << addr_hex[0] << addr_hex[1];
	          std::string fw_ver_str = ss_fw_ver.str();
            int fw_ver = strtol(fw_ver_str.c_str(), NULL, 16);

            sleep(inter_cmd_delay); // inter-command delay
            printf("%s(%d): Slot %02i: Read: Channel FPGA %i Firmware Version \n", __FUNCTION__, __LINE__, (i+1), j);
            int ver_read = wfdlib->getFirmwareVersion(5, 1, wfd[i], j);
            printf("      found version %x\n",ver_read);
            if (ver_read == -1) {
              cm_msg(MERROR, __FUNCTION__, "WFD5-%02i, Channel-%02i: FPGA Communication Failed", (i+1), j);
              return FE_ERR_HW;
            } else if (ver_read != fw_ver) {
              cm_msg(MERROR, __FUNCTION__, "/WFD5-%02i/Channel%02i/: Conflict: \"FPGA Firmware Version\"", (i+1), j);
              return FE_ERR_HW;
            }
          } // if channel enabled
        } // for channel

	sleep(inter_cmd_delay); // inter-command delay
	printf("%s(%d): Slot %02i: Read: Asynchronous Mode \n", __FUNCTION__, __LINE__, (i+1));
	int mode_get = wfdlib->getAsynchronousMode(2, 1, wfd[i]);
	int mode_set = (amc13_rider_odb[i].board.asyncmode_enabled) ? 1 : 0;

        // Async Mode: Enabled
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: Asynchronous Mode Enable \n", __FUNCTION__, __LINE__, (i+1));
        code += wfdlib->setAsynchronousMode(2, 1, wfd[i], amc13_rider_odb[i].board.asyncmode_enabled);
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

	if (mode_get != mode_set) {
	  // post-configuration delay
	  printf("%s(%d): Waiting 2 s ... \n", __FUNCTION__, __LINE__);
	  sleep(2);

	  // firmware hard reset
	  sleep(inter_cmd_delay); // inter-command delay
	  printf("%s(%d): Slot %02i: WFD5 Firmware Hard Reset \n", __FUNCTION__, __LINE__, (i+1));
	  if (wfdlib->resetFirmware(3, 1, wfd[i]) == -1) {
	    cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
	    return FE_ERR_HW;
	  }

	  mode_cnt++;
	}
      } // if config enabled

    } // if slot enabled
  } // for slot

  if (mode_cnt > 0) {
    // post-reset delay
    printf("%s(%d): Waiting 5 s ... \n", __FUNCTION__, __LINE__);
    sleep(5);
  }

  /* Level 3 configuration */
  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      // Frontend Configuration Enabled
      if (amc13_rider_odb[i].board.fe_config_enabled) {
	printf("%s(%d): Slot %02i: WFD5 Configuration 3/3 Initiated \n", __FUNCTION__, __LINE__, (i+1));
        int code = 0;
        bool print_code = false;

	//	if (mode_cnt > 0) {
	  // test connection post-reset quickly
	  sleep(inter_cmd_delay); // inter-command delay
	  printf("%s(%d): Slot %02i: Read: Master FPGA Firmware Version \n", __FUNCTION__, __LINE__, (i+1));
	  if (wfdlib->getFirmwareVersion(5, 0.1, wfd[i]) == -1) {
	    cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: FPGA Communication Failed", (i+1));
	    return FE_ERR_HW;
	  }
	  //	}

        if (amc13_rider_odb[i].board.asyncmode_enabled) {
          // Async Mode: Waveform Length
          // Async Mode: Waveform Presamples
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Asynchronous Mode: Acquisition Pattern Configuration \n", __FUNCTION__, __LINE__, (i+1));
          code += wfdlib->setAcquisitionPattern(2, 1, wfd[i], amc13_rider_odb[i].board.asyncmode_wvfm_length, amc13_rider_odb[i].board.asyncmode_wvfm_presamples);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);
        } else {
          // Trigger Type 1: Enabled
          // Trigger Type 2: Enabled
          // Trigger Type 3: Enabled
          sleep(inter_cmd_delay); // inter-command delay
          printf("%s(%d): Slot %02i: Write: Trigger Type Enables \n", __FUNCTION__, __LINE__, (i+1));
          code += wfdlib->setTriggerTypeEnables(2, 1, wfd[i], amc13_rider_odb[i].board.trig1_enabled, amc13_rider_odb[i].board.trig2_enabled, amc13_rider_odb[i].board.trig3_enabled);
          if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          if (amc13_rider_odb[i].board.trig3_enabled) {
            // Trigger Type 3: Waveform Count
            // Trigger Type 3: Waveform Length
            // Trigger Type 3: Waveform Gap
            sleep(inter_cmd_delay); // inter-command delay
            printf("%s(%d): Slot %02i: Write: Trigger Type 3: Acquisition Pattern Configuration \n", __FUNCTION__, __LINE__, (i+1));
            code += wfdlib->setAcquisitionPattern(2, 1, wfd[i], 3, amc13_rider_odb[i].board.trig3__wvfm_count, amc13_rider_odb[i].board.trig3__wvfm_length, amc13_rider_odb[i].board.trig3__wvfm_gap);
            if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);
          }

          if (amc13_rider_odb[i].board.trig2_enabled) {
            // Trigger Type 2: Waveform Count
            // Trigger Type 2: Waveform Length
            // Trigger Type 2: Waveform Gap
            sleep(inter_cmd_delay); // inter-command delay
            printf("%s(%d): Slot %02i: Write: Trigger Type 2: Acquisition Pattern Configuration \n", __FUNCTION__, __LINE__, (i+1));
            code += wfdlib->setAcquisitionPattern(2, 1, wfd[i], 2, amc13_rider_odb[i].board.trig2__wvfm_count, amc13_rider_odb[i].board.trig2__wvfm_length, amc13_rider_odb[i].board.trig2__wvfm_gap);
            if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);
          }

          if (amc13_rider_odb[i].board.trig1_enabled) {
            // Trigger Type 1: Waveform Count
            // Trigger Type 1: Waveform Length
            // Trigger Type 1: Waveform Gap
            sleep(inter_cmd_delay); // inter-command delay
            printf("%s(%d): Slot %02i: Write: Trigger Type 1: Acquisition Pattern Configuration \n", __FUNCTION__, __LINE__, (i+1));
            code += wfdlib->setAcquisitionPattern(2, 1, wfd[i], 1, amc13_rider_odb[i].board.trig1__wvfm_count, amc13_rider_odb[i].board.trig1__wvfm_length, amc13_rider_odb[i].board.trig1__wvfm_gap);
            if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);
          }
        }

        // ADC Data Endianness
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: ADC Data Endianness \n", __FUNCTION__, __LINE__, (i+1));
        code += wfdlib->setEndianness(2, 1, wfd[i], strcmp(amc13_rider_odb[i].board.ADC_endianness, "big"));
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

        // Trigger Delay
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: Trigger Delay \n", __FUNCTION__, __LINE__, (i+1));
        code += wfdlib->setTriggerDelay(2, 1, wfd[i], (int)(amc13_rider_odb[i].board.trig_delay / 25));
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

        // Error Threshold: Corrupt Data
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: Error Threshold: Corrupt Data \n", __FUNCTION__, __LINE__, (i+1));
        code += wfdlib->setErrorThreshold(2, 1, wfd[i], 0, amc13_rider_odb[i].board.error_thres_corruption);
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

        // Error Threshold: Unknown TTC
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: Error Threshold: Unknown TTC \n", __FUNCTION__, __LINE__, (i+1));
        code += wfdlib->setErrorThreshold(2, 1, wfd[i], 1, amc13_rider_odb[i].board.error_thres_unkwnTTC);
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

        // Error Threshold: DDR3 Overflow
        sleep(inter_cmd_delay); // inter-command delay
        printf("%s(%d): Slot %02i: Write: Error Threshold: DDR3 Overflow \n", __FUNCTION__, __LINE__, (i+1));
        code += wfdlib->setErrorThreshold(2, 1, wfd[i], 2, amc13_rider_odb[i].board.error_thres_DDR3ovflw);
        if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

        for (int j = 0; j < 5; ++j) {
          // Frontend Configuration Enabled
          if (amc13_rider_odb[i].channel[j].fe_config_enabled) {

            // Enabled
            sleep(inter_cmd_delay); // inter-command delay
            printf("%s(%d): Slot %02i: Write: Channel %i Enable \n", __FUNCTION__, __LINE__, (i+1), j);
            code += wfdlib->setChannelEnable(2, 1, wfd[i], j, amc13_rider_odb[i].channel[j].enabled);
            if (print_code) printf("%s(%d): Return Code: %i \n", __FUNCTION__, __LINE__, code);

          } // if channel enabled
        } // for channel
        
        if (code != 0) {
          cm_msg(MERROR, __FUNCTION__, "WFD5-%02i: Configuration Failed", (i+1));
          return FE_ERR_HW;
        }
      } // if config enabled

    } // if slot enabled
  } // for slot

  return SUCCESS;
} // frontend_init_wfd

INT frontend_init()
{
  // hardware initialization
  int status;
  int frontend_index = get_frontend_index();

  //printf("Frontend Index = %d\n", frontend_index);

  // stop readout last to ensure triggers have stopped
  cm_set_transition_sequence(TR_STOP, 501);
  cm_set_transition_sequence(TR_START, 499);

  // increase watchdog timeout to account for electronics, if enabled
  BOOL call_watchdog;
  cm_get_watchdog_params(&call_watchdog, NULL);
  cm_set_watchdog_params(call_watchdog, 300000); // 5 min
  
  // disable uHAL terminal output
  // uhal::disableLogging();
  // -- temporarily enable error messaging from uHAL
  uhal::setLogLevelTo(uhal::Error());

  // initalize frontend settings from ODB
  if (amc13_ODB_init() != SUCCESS) {
    cm_msg(MERROR, __FUNCTION__, "Frontend ODB Initialization Failed");
    return FE_ERR_ODB;
  }

  //Initialize ODB strucutre (?)
  //amc13_ODB_set();
 
  // initialize ODB structures for this frontend
  //status = cm_get_experiment_database(&hDB, NULL);

  // change frontend status color purple
  char fe_index[3];
  sprintf(fe_index, "%02i", frontend_index);
  std::stringstream ss_color;
  ss_color << "/Equipment/AMC13" << fe_index << "/Common/Status color";
  std::string color_str = ss_color.str();
  db_set_value(hDB, 0, color_str.c_str(), "#8A2BE2", 32, 1, TID_STRING);

  // initialize the frontend monitors in ODB
  std::stringstream ss_monitors;
  ss_monitors << "/Equipment/AMC13"<< fe_index << "/Monitors";
  MonitorODBRootKey = ss_monitors.str();
  status = db_check_record(hDB, 0, MonitorODBRootKey.c_str(), AMC13_MONITORS_ODB_STR , TRUE);

  // create timestamp
  struct timeval t_init_start;
  gettimeofday(&t_init_start, NULL);

  // check TQ Bank prefixes
  for (int itq = 0; itq < TQMETHOD_MAX; itq++) {
    if (tq_parameters_odb[itq].store_hist || tq_parameters_odb[itq].TQ_on) {

      for (int jtq = itq+1; jtq < TQMETHOD_MAX; jtq++) {
        if (tq_parameters_odb[jtq].store_hist || tq_parameters_odb[jtq].TQ_on) {

          if (tq_parameters_odb[itq].TQ_bankprefix == tq_parameters_odb[jtq].TQ_bankprefix) {
            cm_msg(MERROR, __FILE__, "Error: Multiple instances of TQ Prefix %s", tq_parameters_odb[itq].TQ_bankprefix);
            return FE_ERR_ODB;
          }

        }
      }

    }
  }

  /*
  // inquire about CUDA devices, for debug purposes (REQUIRES CUDA)
  int num_devices, device;
  cudaGetDeviceCount(&num_devices);
  if (num_devices > 1) {
    for (device = 0; device < num_devices; device++) {
      cudaDeviceProp properties;
      cudaGetDeviceProperties(&properties, device);
      printf("GPU Device ID %d: %s \n", device, properties.name);
    }
  }
  printf("\n");

  // GPU Device ID
  // GPU Device Name Prefix
  cudaDeviceProp properties;
  cudaGetDeviceProperties(&properties, amc13_settings_odb.gpu_dev_id);

  boost::algorithm::to_lower( properties.name );
  std::stringstream ss_gpu_name;
  ss_gpu_name << properties.name;
  std::string gpu_name = ss_gpu_name.str();

  boost::algorithm::to_lower( amc13_settings_odb.gpu_dev_name_prefix );
  std::stringstream ss_prefix;
  ss_prefix << amc13_settings_odb.gpu_dev_name_prefix;
  std::string prefix = ss_prefix.str();

  if (gpu_name.find(prefix) != 0) {
    cm_msg(MERROR, __FUNCTION__, "/AMC13%02i/Settings/Globals/: Invalid \"GPU Device ID\"", frontend_index);
    cm_msg(MINFO, __FUNCTION__, "GPU Device Name Conflict: Read %s, Expected Prefix %s", properties.name, amc13_settings_odb.gpu_dev_name_prefix);
    return -1;
  }
  */

  // MCH IP Address
  struct in_addr mch_addr;
  if (inet_aton(amc13_settings_odb.mch_ip_addr, &mch_addr) == 0) {
    cm_msg(MERROR, __FUNCTION__, "/AMC13%02i/Settings/Globals/: Invalid \"MCH IP Address\"", frontend_index);
    return FE_ERR_ODB;
  }

  // verify frontend settings from ODB
  printf("%s(%d): AMC13 ODB Verifcation \n", __FUNCTION__, __LINE__);
  if (amc13_odb_check() != SUCCESS) {
    cm_msg(MERROR, __FUNCTION__, "AMC13 ODB Verifcation Failed");
    return FE_ERR_ODB;
  }

  if (!amc13_amc13_odb.enableSoftwareTrigger && 
      frontend_index == 0) {
    // verify frontend settings from ODB
    printf("%s(%d): FC7 ODB Verifcation \n", __FUNCTION__, __LINE__);
    if (fc7_odb_check() != SUCCESS) {
      cm_msg(MERROR, __FUNCTION__, "FC7 ODB Verifcation Failed");
      return FE_ERR_ODB;
    }
  } // if fc7 crate

  if (!amc13_amc13_odb.enableSoftwareTrigger && 
      frontend_index != 0) {
    // verify frontend settings from ODB
    printf("%s(%d): WFD5 ODB Verifcation \n", __FUNCTION__, __LINE__);
    if (wfd_odb_check() != SUCCESS) {
      cm_msg(MERROR, __FUNCTION__, "WFD5 ODB Verifcation Failed");
      return FE_ERR_ODB;
    }
  } // if wfd crate

  if (!amc13_settings_odb.simulate_data) {

  // MCH IPMI communication check
  printf("%s(%d): MCH IPMI Communication Check \n", __FUNCTION__, __LINE__);
  int n_attempts = 0;
  bool try_again = false;
  std::stringstream ss_cmd;
  ss_cmd << "ipmitool -I lan -H " << amc13_settings_odb.mch_ip_addr << " -U shelf -P shelf mc info &>/dev/null";
  std::string cmd = ss_cmd.str();

  do {
    int rc = system( cmd.c_str() );
    if (rc < 0 || !WIFEXITED(rc) || WEXITSTATUS(rc) != 0) {
      n_attempts++;
      try_again = (n_attempts < 2);
      if (try_again) sleep(60); // typical boot time: 56 s
    } else {
      try_again = false;
    }
  } while (try_again);

  if (n_attempts >= 2) {
    cm_msg(MERROR, __FUNCTION__, "MCH: IPMI Communication Failed");
    return FE_ERR_HW;
  }
  
    // read device address
    std::string t1_ip = amc13lib->getAddress(10, 2, amc13_settings_odb.mch_ip_addr, 1);
    printf("%s(%d): Read AMC13 T1 IP Address: %s \n", __FUNCTION__, __LINE__, t1_ip.c_str());

    if (t1_ip == "") {
      cm_msg(MERROR, __FUNCTION__, "AMC13: T1 IP Address Read Failed");
      return FE_ERR_HW;
    }

    std::string t2_ip = amc13lib->getAddress(10, 2, amc13_settings_odb.mch_ip_addr, 0);
    printf("%s(%d): Read AMC13 T2 IP Address: %s \n", __FUNCTION__, __LINE__, t2_ip.c_str());

    if (t2_ip == "") {
      cm_msg(MERROR, __FUNCTION__, "AMC13: T2 IP Address Read Failed");
      return FE_ERR_HW;
    }

    // initialize device object
    try {
      std::stringstream uri; uri << "ipbusudp-2.0://" << t1_ip << ":50001";
      std::stringstream atf; atf << "file://" << amc13_amc13_odb.addrTab1;
      amc13[0] = new uhal::HwInterface( uhal::ConnectionManager::getDevice("hw_id", uri.str(), atf.str()) );
    } catch (uhal::exception::exception& e) {
      cm_msg(MERROR, __FUNCTION__, "/AMC13/: Invalid \"T1 Address Table Location\"");
      printf("%s(%d): uHAL Exception: %s \n", __FUNCTION__, __LINE__, e.what());
      return FE_ERR_ODB;
    }

    try {
      std::stringstream uri; uri << "ipbusudp-2.0://" << t2_ip << ":50001";
      std::stringstream atf; atf << "file://" << amc13_amc13_odb.addrTab2;
      amc13[1] = new uhal::HwInterface( uhal::ConnectionManager::getDevice("hw_id", uri.str(), atf.str()) );
    } catch (uhal::exception::exception& e) {
      cm_msg(MERROR, __FUNCTION__, "/AMC13/: Invalid \"T2 Address Table Location\"");
      printf("%s(%d): uHAL Exception: %s \n", __FUNCTION__, __LINE__, e.what());
      return FE_ERR_ODB;
    }

    // AMC13 T1 Firmware Version Required
    printf("%s(%d): AMC13 T1 FPGA Firmware Version Check \n", __FUNCTION__, __LINE__);
    int ver_read = amc13lib->getFirmwareVersion(2, 30, amc13[0]); // typical boot time: 21 s
    if (ver_read == -1) {
      cm_msg(MERROR, __FUNCTION__, "AMC13: FPGA Communication Failed");
      return FE_ERR_HW;
    } else if (ver_read != int(amc13_amc13_odb.t1_fw_version)) {
      cm_msg(MERROR, __FUNCTION__, "/AMC13/: Conflict: \"T1 Firmware Version Required\"");
      return FE_ERR_HW;
    }
    
    // AMC13 T2 Firmware Version Required
    printf("%s(%d): AMC13 T2 FPGA Firmware Version Check \n", __FUNCTION__, __LINE__);
    ver_read = amc13lib->getFirmwareVersion(2, 1, amc13[1]);
    if (ver_read == -1) {
      cm_msg(MERROR, __FUNCTION__, "AMC13: FPGA Communication Failed");
      return FE_ERR_HW;
    } else if (ver_read != int(amc13_amc13_odb.t2_fw_version)) {
      cm_msg(MERROR, __FUNCTION__, "/AMC13/: Conflict: \"T2 Firmware Version Required\"");
      return FE_ERR_HW;
    }

    // AMC13 TTC RX LOS
    printf("%s(%d): AMC13 TTC Signal Presence Check \n", __FUNCTION__, __LINE__);
    int los = amc13lib->getClockAbsence(2, 1, amc13[0]);

    switch (los) {
      case  1: cm_msg(MERROR, __FUNCTION__, "AMC13: TTC Signal Absent");
	       return FE_ERR_HW;
      case -1: cm_msg(MERROR, __FUNCTION__, "AMC13: FPGA Communication Failed");
               return FE_ERR_HW;
    }

    // calculate enabled slots mask
    int enable_mask = -1;
    if (frontend_index != 0) {
      enable_mask = 0;
      for (int i = 0; i < 12; ++i) {
        if (amc13_rider_odb[i].board.rider_enabled) {
          enable_mask += (1 << i);
        }
      }
    }

    // AMC13 readout initialization
    printf("%s(%d): AMC13 Readout Initialization \n", __FUNCTION__, __LINE__);
    if (amc13lib->initialize(2, 5, amc13[0], amc13_amc13_odb.enableSoftwareTrigger, 0, enable_mask, amc13_link_odb[0].source_ip) == -1) {
      cm_msg(MERROR, __FUNCTION__, "AMC13: Readout Initialization Failed");
      return FE_ERR_HW;
    }

    // FC7 initialization, if needed
    if (!amc13_amc13_odb.enableSoftwareTrigger && 
        frontend_index == 0) {

      printf("%s(%d): FC7 Initialization \n", __FUNCTION__, __LINE__);
      if (frontend_init_fc7() != SUCCESS) {
        cm_msg(MERROR, __FUNCTION__, "FC7 Initialization Failed");
        return FE_ERR_HW;
      }

    } // if fc7 crate

    // WFD5 initialization, if needed
    if (!amc13_amc13_odb.enableSoftwareTrigger && 
        frontend_index != 0) {
      
      printf("%s(%d): WFD5 Initialization \n", __FUNCTION__, __LINE__);
      if (frontend_init_wfd() != SUCCESS) {
        cm_msg(MERROR, __FUNCTION__, "WFD5 Initialization Failed");
        return FE_ERR_HW;
      }

    } // if wfd crate

    int max_attempts = 3;
    n_attempts = 0;
    try_again = false;

    do {
      // AMC13 general reset
      sleep(1); // inter-command delay
      printf("%s(%d): AMC13 General Reset \n", __FUNCTION__, __LINE__);
      if (amc13lib->generalReset(3, 1, amc13[0]) == -1) {
        cm_msg(MERROR, __FUNCTION__, "AMC13: General Reset Failed");
        return FE_ERR_HW;
      }
      
      // AMC13 counter reset
      sleep(1); // inter-command delay
      printf("%s(%d): AMC13 Counter Reset \n", __FUNCTION__, __LINE__);
      if (amc13lib->counterReset(3, 1, amc13[0]) == -1) {
        cm_msg(MERROR, __FUNCTION__, "AMC13: Counter Reset Failed");
        return FE_ERR_HW;
      }

      sleep(1); // inter-command delay, required
      printf("%s(%d): AMC13 Final Status Check \n", __FUNCTION__, __LINE__);
      int loc_tts = amc13lib->getFirmwareStatus(3, 1, amc13[0], 0);
      int enc_tts = amc13lib->getFirmwareStatus(3, 1, amc13[0], 1);

      if (loc_tts != 8 ||
          enc_tts != 0) {
        n_attempts++;
        try_again = (n_attempts < max_attempts);
        printf("%s(%d): AMC13 Not Ready %i/%i \n", __FUNCTION__, __LINE__, n_attempts, max_attempts);
        if (try_again) sleep(1); // inter-command delay
      } else {
	try_again = false;
      }
    } while (try_again);

    if (n_attempts >= max_attempts) {
      cm_msg(MERROR, __FUNCTION__, "AMC13 Initialization Failed");
      return FE_ERR_HW;
    }

  } // if not simulated data

  // create timestamp
  struct timeval t_init_stop;
  gettimeofday(&t_init_stop, NULL);

  printf("MicroTCA initialization duration: %f s \n", (t_init_stop.tv_sec - t_init_start.tv_sec) + ((t_init_stop.tv_usec - t_init_start.tv_usec)/1000000.0));

  // create and initialize TCP TCP client 
  status = tcp_client_init();
  if (status != 0) {
    cm_msg(MERROR, __FILE__, "TCP initialization failed, err = %i", status);
    return FE_ERR_HW;
  }
  printf("%s(%d): TCP initialization done \n", __func__, __LINE__);

  // create and initialize GPU thread
  if (gpu_thread_init() != 0) {
    cm_msg(MERROR, __FILE__, "Cannot start gpu thread");
    return FE_ERR_HW;
  }
  printf("%s(%d): GPU initialization done \n", __func__, __LINE__);
  
  // create and initialize parallel port communications
  #ifdef USE_PARALLEL_PORT
  printf("initialize parallel port address 0x%08x\n", pp_addr);
  if (ioperm(pp_addr, 1, 1) != 0) {
    cm_msg(MERROR, __FILE__, "Cannot connect to parallel port");
    return FE_ERR_HW;
  }
  #endif
   
  // create and initialize zlib compression
  if (!fe_compress_z_init()) return FE_ERR_HW;

  // Laser configuration initialization.
  if (frontend_index == 25) {
    laser_config_handler = new g2laser::LaserConfigHandler();
    laser_config_handler->SetOdbPath("/Equipment/AMC1325/Laser");
    laser_config_handler->SetLaserPath("/home/daq/exp.gm5/laser/");
    laser_config_handler->WriteFiles();
  }
  
  Midasfillnumber = 0;

  // change frontend status color green
  db_set_value(hDB, 0, color_str.c_str(), "greenLight", 32, 1, TID_STRING);

  return SUCCESS;
} // frontend_init

/*-- Frontend Exit -------------------------------------------------*/

/** 
 * This routine is called when the frontend program is shut down. 
 * Can be used to releas any locked resources like memory, 
 * communications ports etc.
 * 
 * 
 * @return SUCCESS on success. Note that mfe.c ignores the return value
 */

INT frontend_exit()
{
  int status;
  int frontend_index = get_frontend_index();

  // exit zlib compression
  fe_compress_z_exit();

  // join gpu threads
  gpu_thread_exit();

  // disable TCP client
  status = tcp_client_exit();
  if (status != 0) {
    cm_msg(MERROR, __FILE__, "TCP exit failed, err = %i", status);
    return FE_ERR_HW;
  }

  // Clean up the laser configuration.
  if ((frontend_index == 25) && (laser_config_handler != NULL)) {
    delete laser_config_handler;
  }
  
  return SUCCESS;
} // frontend_exit

/*-- Begin of Run --------------------------------------------------*/

/** 
 * This routine is called when a new run is started. 
 * 
 * @param run_number new run number
 * @param error 
 * 
 * @return CM_SUCCESS on success 
 */

INT begin_of_run_wfd()
{
  // ODB waveform length check
  double waveform_length = 0;
  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {
      int nChannels = 0;
      for (int j = 0; j < 5; ++j) {
        if (amc13_rider_odb[i].channel[j].enabled) {
          nChannels++;
        }
      }
  
      waveform_length += amc13_rider_odb[i].board.trig1__wvfm_length * nChannels;
    }
  }

  for (int i = 0; i < TQMETHOD_MAX; ++i) {
    if (tq_parameters_odb[i].TQ_on ||
        tq_parameters_odb[i].store_hist) {
      if (waveform_length < tq_parameters_odb[i].gpu_waveform_length) {
        cm_msg(MERROR, __FUNCTION__, "TQ-%i: Waveform length is too large for GPU ODB settings", i);
        return FE_ERR_ODB;
      }
    }
  }

  // loop over slots 
  for (int i = 0; i < 12; ++i) {
    if (amc13_rider_odb[i].board.rider_enabled) {

      // loop over channels
      for (int j = 0; j < 5; ++j) {
        if (amc13_rider_odb[i].channel[j].enabled) {

          // channel buffer size check
          dbprintf("%s(%d): Slot %02i: Read: Channel FPGA %i Buffer Size \n", __FUNCTION__, __LINE__, (i+1), j);
          int size = wfdlib->getBufferSize(2, 1, wfd[i], j);

          if (size == -1) {
            cm_msg(MERROR, __FUNCTION__, "WFD5-%02i, Channel-%02i: FPGA Communication Failed", (i+1), j);
            return FE_ERR_HW;
          } else if (size > 0) {
            cm_msg(MERROR, __FUNCTION__, "WFD5-%02i, Channel-%02i: Buffer Not Empty", (i+1), j);
            return FE_ERR_HW;
          }

          // master-channel link connection check
          sleep(0.1); // inter-command delay
          dbprintf("%s(%d): Slot %02i: Read: Channel FPGA %i Firmware Version \n", __FUNCTION__, __LINE__, (i+1), j);
          if (wfdlib->getFirmwareVersion(2, 1, wfd[i], j) == -1) {
            cm_msg(MERROR, __FUNCTION__, "WFD5-%02i, Channel-%02i: FPGA Communication Failed", (i+1), j);
            return FE_ERR_HW;
          }

        } // if channel enabled
      } // for channel

    } // if slot enabled
  } // for slot

  return SUCCESS;
} // begin_of_run_wfd

INT begin_of_run(INT run_number, char *error)
{
  int status;
  int frontend_index = get_frontend_index();

  total_triggers_in_run = -1;
  got_triggers_in_run = 0;
  got_triggers_sent = FALSE;

  block_nr = 0;
  dt_READY = 0.0;
  n_dt_READYs = 0;

  // change frontend status color blue
  char fe_index[3];
  sprintf(fe_index, "%02i", frontend_index);
  std::stringstream ss_color;
  ss_color << "/Equipment/AMC13" << fe_index << "/Common/Status color";
  std::string color_str = ss_color.str();
  db_set_value(hDB, 0, color_str.c_str(), "#0033A0", 32, 1, TID_STRING);

  //Update local amc13_ODB C-structs if the hotlink is not in use
  amc13_ODB_get();

  // ODB channel enable checks 
  if (frontend_index != 0) {
    for (int i = 0; i < 12; ++i) {
      for (int j = 0; j < 5; ++j) {
        for (int k = 0; k < TQMETHOD_MAX; ++k) {
      	  if (tq_parameters_odb[k].TQ_on ||
      	      tq_parameters_odb[k].store_hist) {

      	    if (rider_map_to_calo_odb[i][j][k].enabled == 1 &&
		            amc13_rider_odb[i].channel[j].enabled == 0) {
      	      cm_msg(MERROR, __FUNCTION__, "TQ-%i, WFD5-%02i, Channel-%02i: Inconsistent Channel Enables in ODB", k, (i+1), j);
      	      return FE_ERR_ODB;
      	    }
            
      	  } // if enabled method
        } // for each method
      } // for each channel
    } // for each slot
  } // if non-zero frontend index

  int rider_mod_max =12, rider_chn_max = 5;
  for (int itq = 0; itq < TQMETHOD_MAX; itq++){
    
    dbprintf("make map between detector array and module, channelsitq = %d\n", itq);
    int idet = 0; // enabled detector counter
    
    for (int im=0; im<rider_mod_max; im++){  // loop over Rider modules
      if (amc13_rider_odb[im].board.rider_enabled) {
	
        for (int ic=0; ic<rider_chn_max; ic++){ // loop over Rider channels
	  if (amc13_rider_odb[im].channel[ic].enabled) {
	    
	    if (tq_parameters_odb[itq].TQ_maptype == 0){
	      
	      int ix = rider_map_to_calo_odb[im][ic][itq].x_segment;
	      int iy = rider_map_to_calo_odb[im][ic][itq].y_segment;
	      dbprintf("im = %i, ic = %i, ix = %i, iy = %i\n", im, ic, ix, iy);
	      
	      if (ix >= 1 && ix <= tq_parameters_odb[itq].gpu_n_segments_x && iy >= 1 && iy <= tq_parameters_odb[itq].gpu_n_segments_y && rider_map_to_calo_odb[im][ic][itq].enabled) {
		map_from_caloxy_to_ridermodchan[ (ix-1) + (iy-1)*tq_parameters_odb[itq].gpu_n_segments_x ][itq] = ic + im*rider_chn_max;
		dbprintf("calo segment x,y %i, %i  map_from_caloxy_to_ridermodchan %i\n", ix, iy, map_from_caloxy_to_ridermodchan[ (ix-1) + (iy-1)*tq_parameters_odb[itq].gpu_n_segments_x ][itq] );
		idet++;
	      }
	    }
	    
	    if (tq_parameters_odb[itq].TQ_maptype == 1) {
	      if ( rider_map_to_calo_odb[im][ic][itq].enabled ) {
		map_from_caloxy_to_ridermodchan[idet][itq] = idet;
		dbprintf("enabled detector %i  map_from_caloxy_to_ridermodchan %i\n", idet, map_from_caloxy_to_ridermodchan[idet][itq] );
		idet++;
	      }
	    }
	    
	  } // if for enabled rider channels                                                                       
	} // for loop on rider channels                                                                          
	
      } // if forr enabled rider modules                                                                                 
    } // for loop on rider channels                                                                                       
  
  } // for TQ methods
  
  if (!amc13_settings_odb.simulate_data) {

    // WFD5 begin-of-run functions
    if (!amc13_amc13_odb.enableSoftwareTrigger) {
      if (frontend_index != 0) {

        status = begin_of_run_wfd();
        if (status != SUCCESS) {
          cm_msg(MERROR, __FUNCTION__, "WFD5 Begin-of-Run Failed");
          return status;
        }

      } else {
         // update the laser prescaling factor, since we are in the AMC1300 front end.
         int odb_prescale = amc13_fc7_odb[trigger_fc7_slot].trigger.laser_prescale;
         int current_prescale = fc7lib->getLaserInfillPrescale(2, 1, fc7[trigger_fc7_slot] );
         if ( current_prescale == -1 ) {
            cm_msg(MERROR, __FUNCTION__, "FC7-%02i: FPGA Communication Failed", (trigger_fc7_slot+1));

         } else if ( odb_prescale != current_prescale) {
            current_prescale = fc7lib->setLaserInfillPrescale(2, 1, fc7[trigger_fc7_slot], odb_prescale );
            if ( current_prescale  == -1 ) {
               cm_msg(MERROR, __FUNCTION__, "FC7-%02i: Laser infill prescale set attempt failed", (trigger_fc7_slot+1));
            }
         }

         // now update the lenth of time to wait before falling over to the internal trigger when A6 is missing
         int internal_trig_fallover = amc13_fc7_odb[trigger_fc7_slot].trigger.fallover_threshold;
         if ( fc7lib->getFalloverThresh(2, 1, fc7[trigger_fc7_slot] ) != internal_trig_fallover ) {
            status = fc7lib->setFalloverThresh( 2, 1, fc7[trigger_fc7_slot], internal_trig_fallover );
            if ( status  == -1 ) {
               cm_msg(MERROR, __FUNCTION__, "FC7-%02i: Set attempt for internal trigger fallover threshold failed", (trigger_fc7_slot+1));
            }
         }
         
         // check the time after charging when a safety discharge should be sent if no other discharge has been sent
         std::string safety_node( "SAFETY_DISCHG_TIM" );
         int safety_timing_current = fc7lib->getRegister(2, 1, fc7[trigger_fc7_slot], safety_node );
         int safety_time_requested = amc13_fc7_odb[trigger_fc7_slot].trigger.safety_discharge_time * 40000; // convert from msec to 25 ns clock ticks
         if ( safety_timing_current != safety_time_requested ) {
            status = fc7lib->setRegister(2, 1, fc7[trigger_fc7_slot], safety_node, safety_time_requested, true);
            if ( status  == -1 ) {
               cm_msg(MERROR, __FUNCTION__, "FC7-%02i: Set attempt for kicker safety discharge timing failed", (trigger_fc7_slot+1));
            }
         }

         // check the enforced deadtime after a kicker charging trigger
         std::string deadtime_node( "CHARGE_DEADTIME" );
         int charge_deadtime_current = fc7lib->getRegister(2, 1, fc7[trigger_fc7_slot], deadtime_node );
         int charge_deadtime_requested = amc13_fc7_odb[trigger_fc7_slot].trigger.kicker_charge_trigger_deadtime * 40000; // convert from msec to 25 ns clock ticks
         if ( charge_deadtime_current != charge_deadtime_requested ) {
            status = fc7lib->setRegister(2, 1, fc7[trigger_fc7_slot], deadtime_node, charge_deadtime_requested, true);
            if ( status  == -1 ) {
               cm_msg(MERROR, __FUNCTION__, "FC7-%02i: Set attempt for kicker charge trigger deadtime requested failed", (trigger_fc7_slot+1));
            }
         }

         // update some of the new quad trigger settings
         unsigned int quad_t9_delay_current = fc7lib->getQuadT9Delay( 2, 1, fc7[trigger_fc7_slot] );
         unsigned int quad_t9_delay_request = amc13_fc7_odb[trigger_fc7_slot].trigger.quad_t9_delay;
         if ( quad_t9_delay_current != quad_t9_delay_request ) {
            status = fc7lib->setQuadT9Delay( 2, 1, fc7[trigger_fc7_slot], quad_t9_delay_request );
            if ( status  == -1 ) {
               cm_msg(MERROR, __FUNCTION__, "FC7-%02i: Set attempt for quad T9 delay failed", (trigger_fc7_slot+1));
            }
         }
         
         unsigned int quad_t9_width_current = fc7lib->getQuadT9Width( 2, 1, fc7[trigger_fc7_slot] );
         unsigned int quad_t9_width_request = amc13_fc7_odb[trigger_fc7_slot].trigger.quad_t9_width;
         if ( quad_t9_width_current != quad_t9_width_request ) {
            status = fc7lib->setQuadT9Width( 2, 1, fc7[trigger_fc7_slot], quad_t9_width_request );
            if ( status  == -1 ) {
               cm_msg(MERROR, __FUNCTION__, "FC7-%02i: Set attempt for quad T9 width failed", (trigger_fc7_slot+1));
            }
         }
         
         unsigned int quad_t9_enable_current = fc7lib->getQuadT9Enable( 2, 1, fc7[trigger_fc7_slot] );
         unsigned int quad_t9_enable_request = amc13_fc7_odb[trigger_fc7_slot].trigger.quad_t9_enabled;
         if ( quad_t9_enable_current != quad_t9_enable_request ) {
            status = fc7lib->setQuadT9Enable( 2, 1, fc7[trigger_fc7_slot], quad_t9_enable_request );
            if ( status  == -1 ) {
               cm_msg(MERROR, __FUNCTION__, "FC7-%02i: Set attempt for quad T9 enable failed", (trigger_fc7_slot+1));
            }
         }
         
         unsigned int quad_a6_enable_current = fc7lib->getQuadA6Enable( 2, 1, fc7[trigger_fc7_slot] );
         unsigned int quad_a6_enable_request = amc13_fc7_odb[trigger_fc7_slot].trigger.quad_a6_enabled;
         if ( quad_a6_enable_current != quad_a6_enable_request ) {
            status = fc7lib->setQuadA6Enable( 2, 1, fc7[trigger_fc7_slot], quad_a6_enable_request );
            if ( status  == -1 ) {
               cm_msg(MERROR, __FUNCTION__, "FC7-%02i: Set attempt for quad A6 enable failed", (trigger_fc7_slot+1));
            }
         }
         
      } // zero or non-zero frontend index
       
    } // if not fake data

    // AMC13 begin-of-run functions
    int loc_tts = amc13lib->getFirmwareStatus(3, 1, amc13[0], 0);
    int enc_tts = amc13lib->getFirmwareStatus(3, 1, amc13[0], 1);
    int daq_link = amc13lib->getLinkStatus(3, 1, amc13[0]);

    if (loc_tts != 8 || 
        enc_tts != 0 ||
        daq_link != 0) {
      cm_msg(MERROR, __FUNCTION__, "AMC13 Begin-of-Run Failed");
      return FE_ERR_HW;
    }

  } // if not simulated data

  //clear monitors first

  int MidasFill = 0;
  int GPUFill = 0;
  int TCPFill = 0;

  //Update fill monitor
  db_set_value(hDB, 0, (MonitorODBRootKey + std::string("/Midas Fill Number")).c_str(), &MidasFill, sizeof(MidasFill), 1, TID_INT);
  db_set_value(hDB, 0, (MonitorODBRootKey + std::string("/TCP Fill Number")).c_str(), &TCPFill, sizeof(TCPFill), 1, TID_INT);
  db_set_value(hDB, 0, (MonitorODBRootKey + std::string("/GPU Fill Number")).c_str(), &GPUFill, sizeof(GPUFill), 1, TID_INT);

  // begin-of-run functions for TCP calo readout (TCP client)
  status = tcp_client_bor();
  if (status != 0) {
    cm_msg(MERROR, __FILE__, "TCP begin_of_run failed, err = %i", status);
    //exit(1);
    return FE_ERR_HW;
  }
  
  // begin-of-run functions for GPU processing
  status = gpu_bor();
  if (status != 0) {
    cm_msg(MERROR, __FILE__, "GPU begin_of_run failed, err = %i", status);
    //exit(1);
    return FE_ERR_HW;
  }

  // Laser configuration and initialization.
  if (frontend_index == 25) {
    laser_config_handler->BeginOfRun();
  }

  // begin-of-run functions for zlib compression
  fe_compress_z_bor(run_number, error);
  
  Midasfillnumber = 0;

  //Enables readings in gpu thread
  pthread_mutex_lock( &mutex_GPU_general );
  gpu_thread_read = 1;
  pthread_mutex_unlock( &mutex_GPU_general );

  //Update operation state
  int OperationState = 1;
  db_set_value(hDB, 0, (MonitorODBRootKey + std::string("/Operation State")).c_str(), &OperationState, sizeof(OperationState), 1, TID_INT);

  // change frontend status color green
  db_set_value(hDB, 0, color_str.c_str(), "greenLight", 32, 1, TID_STRING);

  return CM_SUCCESS;
} // begin_of_run

/*-- End of Run ----------------------------------------------------*/

/** 
 * This routine is called on a request to stop a run. 
 * Can send end-of-run event and close run gates.
 * 
 * @return CM_SUCCESS on success
 */

INT end_of_run(INT run_number, char *error)
{
  INT status;
  int frontend_index = get_frontend_index();

  //Wait until no new events from the tcp thread.
  unsigned int TCPWaitLimit = 10;
  unsigned int TCPStaticCheck = 0;
  unsigned int TCPStaticCondition = 3;
  bool tcp_finished = false;
  unsigned long last_tcp_fill;
  unsigned long current_tcp_fill;

  pthread_mutex_lock( &mutex_TCP_general );
  current_tcp_fill = TCPfillnumber;
  pthread_mutex_unlock( &mutex_TCP_general );
  last_tcp_fill = current_tcp_fill;

  for (unsigned int i=0;i<TCPWaitLimit;i++)
  {
    pthread_mutex_lock( &mutex_TCP_general );
    current_tcp_fill = TCPfillnumber;
    pthread_mutex_unlock( &mutex_TCP_general );
    if (current_tcp_fill != last_tcp_fill)
    {
      TCPStaticCheck = 0;
      last_tcp_fill = current_tcp_fill;
    }else{
      TCPStaticCheck++;
    }
    
    if (TCPStaticCheck > TCPStaticCondition)
    {
      tcp_finished = true;
      break;
    }
    sleep(1);
  }

  if (!tcp_finished){
    cm_msg(MERROR, __FILE__, "TCP events still come after the end_of_run command issued.");
  }

  //Wait until all events are written to midas data stream
  unsigned int DataWritingWaitLimit = 15;
  unsigned long gpu_fill;
  bool fill_number_match = false;
  for (unsigned int i=0;i<DataWritingWaitLimit;i++)
  {
    pthread_mutex_lock( &mutex_GPU_general );
    gpu_fill = GPUfillnumber;
    pthread_mutex_unlock( &mutex_GPU_general );
    if (gpu_fill == current_tcp_fill && Midasfillnumber == gpu_fill){
      fill_number_match  = true;
      break;
    }
    sleep(1);
  }

  if (!fill_number_match){
    cm_msg(MERROR, __FILE__, "TCP/GPU/Midas fill numbers do not match at the end of the run.");
    char msg[500];
    sprintf(msg,"DAQ | End of Run Fill Number Mismatch from AMC13%02d",frontend_index);

    int ret_code = al_trigger_alarm("End of Run AMC13", msg, "Warning", "End of Run Failure", AT_INTERNAL);
    if (ret_code != AL_SUCCESS) {
      cm_msg(MERROR, __FILE__, "Failure Raising Alarm: Error %d, Alarm \"%s\"", ret_code, "End of Run Failure");
    }
  }

  // end-of-run functions for TCP client
  status = tcp_client_eor();
  if (status != 0) {
    cm_msg(MERROR, __FILE__, "TCP end_of_run failed, err = %i", status);
    return FE_ERR_HW;
  }

  // end-of-run functions for GPU processing
  status = gpu_eor();
  if (status != 0) {
    cm_msg(MERROR, __FILE__, "GPU end_of_run failed, err = %i", status);
    return FE_ERR_HW;
  }

  // Laser specific code.
  if (frontend_index == 25) {
    laser_config_handler->EndOfRun();
  }

  // end-of-run functions for zlib compression
  //fe_compress_z_eor(run_number, error);
  run_number_compress_z_eor = run_number;
  error_compress_z_eor = error;

  //Disable readings in gpu thread
  pthread_mutex_lock( &mutex_GPU_general );
  gpu_thread_read = 0;
  pthread_mutex_unlock( &mutex_GPU_general );

  //Update operation state
  int OperationState = 2;
  db_set_value(hDB, 0, (MonitorODBRootKey + std::string("/Operation State")).c_str(), &OperationState, sizeof(OperationState), 1, TID_INT);

  //Check the status color. If it is not greeLight, exit the program
  /*
  char fe_index[3];
  sprintf(fe_index, "%02i", frontend_index);
  std::stringstream ss_color;
  ss_color << "/Equipment/AMC13" << fe_index << "/Common/Status color";
  std::string color_str = ss_color.str();
  char StatusColorBuf[256];
  int StatusColorStrSize = 32;
  db_get_value(hDB, 0, color_str.c_str(), &StatusColorBuf, &StatusColorStrSize, TID_STRING,FALSE);
  std::string StatusColorString = StatusColorBuf;
  if (StatusColorString.compare("greenLight")!=0)
  {
    exit(1);
  }
  */

  return SUCCESS;
} // end_of_run

/*-- Pause Run -----------------------------------------------------*/

/** 
 *  This routine is called when a run is paused. 
 *  Should disable trigger events.
 *  Pause/resume mechanism is not implemented in our DAQ.
 * 
 * @param run_number run number
 * @param error error
 * 
 * @return always returns CM_INVALID_TRANSITION 
 */

INT pause_run(INT run_number __attribute__((unused)), char *error __attribute__((unused)))
{
  cm_msg(MERROR, __FUNCTION__, "Function Not Implemented");
  return CM_INVALID_TRANSITION;
} // pause_run

/*-- Resume Run ----------------------------------------------------*/

/** 
 * This routine is called  when a run is resumed. 
 * Should enable trigger events.
 * Pause/resume mechanism is not implemented in our DAQ.
 * 
 * @param run_number 
 * @param error 
 * 
 * @return always returns CM_INVALID_TRANSITION 
 */

INT resume_run(INT run_number __attribute__((unused)), char *error __attribute__((unused)))
{
  cm_msg(MERROR, __FUNCTION__, "Function Not Implemented");
  return CM_INVALID_TRANSITION;
} // resume_run

/*-- Frontend Loop -------------------------------------------------*/

/** 
 * If frontend_call_loop is true, this routine gets called when
 * the frontend is idle or once between every event
 * 
 * @return SUCCESS. 
 */

INT frontend_loop()
{
  return SUCCESS;
} // frontend_loop

/*------------------------------------------------------------------*/

/********************************************************************\

  Readout routines for different events

\********************************************************************/

/*-- Trigger event routines ----------------------------------------*/

/** 
 * Polling routine for events.
 * Returns TRUE if event is available.
 * If test equals TRUE, don't return. The test flag is used to time the polling
 * 
 * @param source 
 * @param count 
 * @param test 
 * 
 * @return TRUE if event is available.
 */

INT poll_event(INT source __attribute__((unused)), INT count, BOOL test)
{
  // fake calibration
  if (test) {
    for (int i = 0; i < count; i++) {
      usleep(1);
    }
    return 0;
  }

  INT retval = 0; 
  BOOL data_avail = FALSE;          // true if data is available for readout

  // Check GPU buffer
  pthread_mutex_lock( &mutex_GPU_general );
  if (GPUfillnumber > Midasfillnumber)
  {
    data_avail = TRUE;
  }
  if (GPUfillnumber < Midasfillnumber && GPUfillnumber!=0) // this is for wrapping over the largest unsigned long, which is not very probable if the run is short
  {
    unsigned long buffer_filled = 0xffffffffffffffff - (Midasfillnumber - GPUfillnumber) +1 ;
    if (buffer_filled < 0xffffffffffffffff / 2)
    {
      data_avail = TRUE;
    }
  }
  pthread_mutex_unlock( &mutex_GPU_general );

//  if (run_state == STATE_RUNNING) {
    if (data_avail) {
      retval = 1;
    }
//  }
  return retval; 
} // poll_event

/*-- Interrupt configuration ---------------------------------------*/

/** 
 * Establish interrupt handler.
 * 
 * @param cmd Possible values are:
 *            - CMD_INTERRUPT_ENABLE
 *            - CMD_INTERRUPT_DISABLE
 *            - CMD_INTERRUPT_ATTACH
 *            - CMD_INTERRUPT_DETACH
 * @param source 
 * @param adr pointer to interrupt ruotine: void interrupt_routine(void)
 * 
 * @return SUCCESS on success
 */

INT interrupt_configure(INT cmd, INT source __attribute__((unused)), POINTER_T adr __attribute__((unused)))
{
  switch (cmd) {
    case CMD_INTERRUPT_ENABLE:
      break;
    case CMD_INTERRUPT_DISABLE:
      break;
    case CMD_INTERRUPT_ATTACH:
      break;
    case CMD_INTERRUPT_DETACH:
      break;
  }

  return SUCCESS; 
} // interrupt_configure

/*-- Event readout -------------------------------------------------*/

/** 
 * Event readout routine.
 * 
 * @param pevent 
 * @param off offset (for subevents)
 * 
 * @return 
 */

INT read_trigger_event(char *pevent, INT off __attribute__((unused)))
{

  int status __attribute__((unused));
  float *fdata;
  BYTE *bdata;
  short *pdata;
  DWORD *hdata;
  char bk_name[8];
  int frontend_index = get_frontend_index();

  // temporary array for performance data to allowing unlocking gpu thread before data compression
  int perf_data_size = 0;
  uint64_t *perf_data;
  perf_data = (uint64_t*) malloc( gpu_data_header_size_max );

  dbprintf("Begin read_trigger_event!\n");

  //Obtain the address of the data struct in the GPU buffer
  int GPUbufferindex = Midasfillnumber % GPU_BUFFER_SIZE;
  GPU_Data_t* GPUDATA = &(GPU_Data_Buffer[GPUbufferindex]);

  //Lock the buffer access
  pthread_mutex_lock( &mutex_GPU_buf[GPUbufferindex] );
  // get AMC13 fill number
  unsigned int AMC13fillcounter = ( be32toh ( GPUDATA->gpu_data_header[0] ) & 0x00FFFFFF ); 

  // get GPU muon fill number that's stored by gpu_thread (used for flushing the CQ, CR banks)
  unsigned int GPUmuonfillcounter = GPUDATA->gpu_data_header[21];
  dbprintf("GPUmuonfillcounter %i\n", GPUmuonfillcounter);

  // get data ready time
  struct timeval t_lock_data, t_got_data;

  status = gettimeofday( &t_lock_data, NULL);
  trigger_info.time_slave_lock_dataready_s  = t_lock_data.tv_sec;
  trigger_info.time_slave_lock_dataready_us = t_lock_data.tv_usec;

  // store timing information and current TCPfillnumber, GPUfillnumber in header databank
  GPUDATA->gpu_data_header[13] = t_lock_data.tv_sec;
  GPUDATA->gpu_data_header[14] = t_lock_data.tv_usec;

  // get fill type
  unsigned int Fill_type = 0;
  if ( frontend_index != 0 ) Fill_type = ( ( be64toh(GPUDATA->gpu_data_header_rider[1]) >> 48 ) & 0x7 );
  dbprintf("%s(%d): GPUDATA->gpu_data_header_rider[1] %16lx, Event type: %i\n", __func__, __LINE__, GPUDATA->gpu_data_header_rider[1], Fill_type);

  /* init bank structure */
  bk_init32(pevent);

  // make processed databanks 
#ifdef USE_GPU

  for (int itq = 0; itq < TQMETHOD_MAX; itq++){
    if ( tq_parameters_odb[itq].TQ_on && Fill_type==1 ) {

      int nsegments =  *(GPUDATA->gpu_data_proc[itq]+3);

      // make pedestal databank
      dbprintf("%s(%d): fill CP data bank\n",__func__, __LINE__);
      sprintf(bk_name,"%sP%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
      bk_create(pevent, bk_name, TID_FLOAT, (void**)&fdata);
      *fdata++ = nsegments; // first word is number of segments
      int gpu_data_p_size = ( nsegments * sizeof(fdata[0]) ); // size of pedestal data in bytes
      int p_offset = (GPUDATA->gpu_data_proc_size[itq] - gpu_data_p_size) / sizeof(*GPUDATA->gpu_data_proc[0]);
      dbprintf("%s(%d): pedestal data [0], [1]: , 0x%08x 0x%08x proc data size (bytes) : %d, ped data size (bytes) : %d, ped data pointer %p, ped offset %i\n",
	  __func__,__LINE__, *(GPUDATA->gpu_data_proc[itq]+p_offset), *(GPUDATA->gpu_data_proc[itq]+p_offset+2), GPUDATA->gpu_data_proc_size[itq], gpu_data_p_size, (GPUDATA->gpu_data_proc[itq]+p_offset), p_offset);
      memcpy(fdata, GPUDATA->gpu_data_proc[itq]+p_offset, gpu_data_p_size); 
      dbprintf("%s(%d): sizeof(fdata): %lu\n", __func__, __LINE__, sizeof(fdata[0]));
      if(sizeof(fdata) != 0) fdata += gpu_data_p_size / sizeof(fdata[0]);
      bk_close(pevent, fdata);
      dbprintf("%s(%d): made pedestal databank %s size (bytes) %d\n", 
	  __func__, __LINE__, bk_name, gpu_data_p_size);

      // make temporary store of pedestal values for deriving truncated calo databank, array size is full-crate max
      float pedestalstore[60]; 
      memcpy(pedestalstore, GPUDATA->gpu_data_proc[itq]+p_offset, gpu_data_p_size);

      // test for bank-by-bank lossless compression using new function fe_compress_z2
      /*
      // create TEST bank
      sprintf(bk_name,"TEST");
      bk_create(pevent, bk_name, TID_BYTE, (void**)&bdata);
       *bdata++ = 1; // first word test data
       *bdata++ = 2; // 2nd word test data
       *bdata++ = 3; // 3rd word test data
       *bdata++ = 4; // 4th word test data
       bk_close(pevent, bdata);
       */

      // bank-by-bank compression
      if ( amc13_settings_odb.bankbybank_lossless_compression ) { 
	BANK_HEADER *bankheader = (BANK_HEADER*) pevent;
	int banksize = sizeof(BYTE)*bk_locate( pevent, bk_name,  (void**)&bdata); // bdata is byte pointer to location of data
	bk_name[0] = tolower(bk_name[0]); // name for compressed bank
	if ( fe_compress_z2(pevent, // char pointer to location of event
	      (char*)(bdata - sizeof(BANK32)), // char pointer to location of input
	      banksize + sizeof(BANK32), // data size + header size
	      max_event_size-(bankheader->data_size+sizeof(BANK_HEADER)+sizeof(EVENT_HEADER)), // available space
	      0, bk_name) != FE_SUCCESS ) // bank name
	{
	  printf("%s(%d): fill compressed databank - compression failed\n",__func__, __LINE__);
	} else {
	  printf("%s(%d): fill compressed databank - compression succeeded\n",__func__, __LINE__);
	  bk_name[0] = toupper(bk_name[0]); // name for compressed bank
	  bk_delete(pevent,bk_name); // if compression successful then delete uncompressed bank
	}
      } //bank-by-bank lossless compression

      // make fill-by-fill calo sum databank
      dbprintf("%s(%d): fill CS data bank\n",__func__, __LINE__);
      sprintf(bk_name,"%sS%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
      bk_create(pevent, bk_name, TID_DWORD, (void**)&hdata);  
      GPUDATA->gpu_data_q_size[itq] = (tq_parameters_odb[itq].gpu_waveform_length / tq_parameters_odb[itq].calosum_decimation_factor) * sizeof(hdata[0]);
      *hdata++ = GPUDATA->gpu_data_q_size[itq] / sizeof(hdata[0]); // first word is number of elements of fill-by-fill Q method histogram
      int q_offset = (GPUDATA->gpu_data_proc_size[itq] - gpu_data_p_size - GPUDATA->gpu_data_q_size[itq]) / sizeof(*GPUDATA->gpu_data_proc[0]);
      dbprintf("%s(%d): Calo sum data [0] [1]: 0x%08x 0x%08x, proc data size (bytes) : %d, Q data size  (bytes) : %d, Qmeth data pointer %p QMeth offset %i\n",__func__,__LINE__,
	  *(GPUDATA->gpu_data_proc[itq]+q_offset), *(GPUDATA->gpu_data_proc[itq]+q_offset+2), GPUDATA->gpu_data_proc_size[itq], GPUDATA->gpu_data_q_size[itq], (GPUDATA->gpu_data_proc[itq]+q_offset), q_offset);
      memcpy(hdata, GPUDATA->gpu_data_proc[itq]+q_offset, GPUDATA->gpu_data_q_size[itq]);
      dbprintf("%s(%d): sizeof(hdata): %li\n",__func__,__LINE__,sizeof(hdata[0]));
      if(sizeof(hdata) != 0) hdata += GPUDATA->gpu_data_q_size[itq] / sizeof(hdata[0]);
      bk_close(pevent, hdata);
      dbprintf("%s(%d): made fill-by-fill Q-method databank %s size (bytes) %d\n", 
	  __func__, __LINE__, bk_name, GPUDATA->gpu_data_q_size[itq]);

      // bank-by-bank compression
      if ( amc13_settings_odb.bankbybank_lossless_compression ) { 
	BANK_HEADER *bankheader = (BANK_HEADER*) pevent;
	int banksize = sizeof(BYTE)*bk_locate( pevent, bk_name,  (void**)&bdata); // bdata is byte pointer to location of data
	bk_name[0] = tolower(bk_name[0]); // name for compressed bank
	if ( fe_compress_z2(pevent, // char pointer to location of event
	      (char*)(bdata - sizeof(BANK32)), // char pointer to location of input
	      banksize + sizeof(BANK32), // data size + header size
	      max_event_size-(bankheader->data_size+sizeof(BANK_HEADER)+sizeof(EVENT_HEADER)), // available space
	      0, bk_name) != FE_SUCCESS ) // bank name
	{
	  printf("%s(%d): fill compressed databank - compression failed\n",__func__, __LINE__);
	} else {
	  printf("%s(%d): fill compressed databank - compression succeeded\n",__func__, __LINE__);
	  bk_name[0] = toupper(bk_name[0]); // name for compressed bank
	  bk_delete(pevent,bk_name); // if compression successful then delete uncompressed bank
	}
      } // bank-by-bank lossless compression

      // make island / T-method databank
      if(GPUmuonfillcounter%tq_parameters_odb[itq].T_prescale==0){
	dbprintf("%s(%d): fill CT data bank\n",__func__, __LINE__);
	sprintf(bk_name,"%sT%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
	bk_create(pevent, bk_name, TID_SHORT, (void**)&pdata);
	int t_offset = 0;
	int t_size = 0;
	dbprintf("%s(%d): Tmethod data [0] [1]: 0x%04x 0x%04x, proc data size (bytes) : %d, T data size (bytes): %d, TMeth data pointer %p TMeth offset %i\n", __func__, __LINE__, 
	    *(GPUDATA->gpu_data_proc[itq]+t_offset), *(GPUDATA->gpu_data_proc[itq]+t_offset+1), GPUDATA->gpu_data_proc_size[itq], GPUDATA->gpu_data_proc_size[itq]-GPUDATA->gpu_data_q_size[itq]-gpu_data_p_size, (GPUDATA->gpu_data_proc[itq]+t_offset), t_offset);        
	memcpy( &t_size, GPUDATA->gpu_data_proc[itq], sizeof(int) );
	t_size += 6; // t_size has units of 16-bit words, account for 6 extra 16-bit words containing data size, nislands, ndetectors, CTAG/TBA

	// development work for truncating CT bank to trigger crystals
	if ( tq_parameters_odb[itq].save_truncated_calo ) {
	  struct timeval t_start_truncate;
	  gettimeofday(&t_start_truncate, NULL);  // record time for start of truncation

	  int32_t *ctpntr32bit; // 32-bit pointer to full calo databank (times, lengths, ..., are 32-bit)
	  ctpntr32bit = (int32_t*)GPUDATA->gpu_data_proc[itq]+t_offset; // set 32-bit pointer to start of bank
	  int16_t *ctpntr16bit; // 16-bit pointer to full calo databank (ADC samples are 16-bit)
	  ctpntr16bit = GPUDATA->gpu_data_proc[itq]+t_offset; // set 16-bit pointer to start of bank

	  int32_t islandtime, islandtimeoffset = 3, islandlengthoffset = 4, islandsampleoffset = 10; // set inital values for pointer offsets of island time, length
	  int16_t nislands, nxtals,  adcsample; // set inital values for pointer offsets of island samples
	  int32_t islandlength;
    #ifdef DEBUG
        int32_t ctsize
          ctsize = *ctpntr32bit; // total size of full calo databanks
    #endif
	  nxtals = *(ctpntr16bit+3); // number of xtals in array - hard-coded 3 for location of variable in 16-bit word units
	  nislands = *(ctpntr16bit+2); // number of islands in full calo databank - hard-coded 2 for location of variable  in 16-bit word units
	  dbprintf("%s(%d): CT bank words data size %i, nislands %i, nxtals %i\n", __func__, __LINE__, ctsize, nislands, nxtals); // debug


	  // CT header = 32-bit data word total, 16-bit number of islands, 16-bit number of detectors, 32bit ctag
	  *pdata++ = 0xdead; // mark as truncated bank	
	  *pdata++ = 0xbabe; // mark as truncated bank
	  memcpy(pdata, GPUDATA->gpu_data_proc[itq]+t_offset, 6*sizeof(pdata[0]) ); // copy CT - bank header words from gpu databank to midas databank
	  pdata += 6; // skip 3 32-bit CT header words

	  int ixtals;
	  for (int iisland = 1; iisland <= nislands; iisland++){ // loop over islands in full calo bank
	    islandtime = *(ctpntr32bit + islandtimeoffset); // get island time
	    islandlength = *(ctpntr32bit + islandlengthoffset); // get island length
	    dbprintf("%s(%d): CT bank words island time %i, island length %i\n", __func__, __LINE__, islandtime, islandlength);

	    int16_t triggerstore[60] = {0}, borderstore[60] = {0}; // arrays for storing where each xtal is a trigger xtal or a border xtal

	    //if(islandlength<MIN_ISLAND_LENGTH) cm_msg(MERROR, __FUNCTION__,"Island length of %i is less than minimum island length of %i for island %i of %i at island time %i",islandlength, MIN_ISLAND_LENGTH,iisland,nislands,islandtime);

	    // find trigger samples
	    int  isample;
	    float ped;
	    int rider_chn_max = 5;
	    for (ixtals = 0; ixtals < nxtals; ixtals++){ // loop over xtals on island
	      for (isample = 0; isample < islandlength; isample++){ // loop over samples for xtal

		adcsample = *(ctpntr16bit + islandsampleoffset + ixtals*islandlength + isample); // get adc value
		ped = pedestalstore[ ixtals ]; // array temporary storing gpu pedestals for each TQ method	    

		if ( tq_parameters_odb[itq].island_option == 2 ) { // if leading edge threshold

		  if ( tq_parameters_odb[itq].use_channel_thresholds ){ // if using individual thresholds

		    int iriderchn = map_from_caloxy_to_ridermodchan[ ixtals ][ itq ] % rider_chn_max;
		    int iridermod = map_from_caloxy_to_ridermodchan[ ixtals ][ itq ] / rider_chn_max;
		    printf("ixtal = %i, iriderchn %i, iridermod %i, threshold sign %i, threshold value %i\n", 
			ixtals, iriderchn, iridermod, rider_map_to_calo_odb[iridermod][iriderchn][itq].polarity, rider_map_to_calo_odb[iridermod][iriderchn][itq].value);

		    if ( rider_map_to_calo_odb[iridermod][iriderchn][itq].polarity && adcsample - ped > rider_map_to_calo_odb[iridermod][iriderchn][itq].value ) { // needs fixing
		      triggerstore[ixtals] = 1; // store index of trigger sample
		      break;
		    }
		    if ( !rider_map_to_calo_odb[iridermod][iriderchn][itq].polarity && adcsample - ped < rider_map_to_calo_odb[iridermod][iriderchn][itq].value ) { // needs fixing
		      triggerstore[ixtals] = 1; // store index of trigger sample
		      break;
		    }

		  } else { // if global thresholds

		    if ( tq_parameters_odb[itq].T_threshold_sign && adcsample - ped > tq_parameters_odb[itq].T_threshold ) { // needs fixing
		      triggerstore[ixtals] = 1; // store index of trigger sample
		      break;
		    }
		    if ( !tq_parameters_odb[itq].T_threshold_sign && adcsample - ped < tq_parameters_odb[itq].T_threshold ) { // needs fixing
		      triggerstore[ixtals] = 1; // store index of trigger sample
		      break;
		    }

		  } 

		}  else  { // end leading edge trigger type

		  printf("WARNING - current truncated calo bank only support global type = 2 thresholds!");

		}

	      } // end sample loop
	      if ( triggerstore[ixtals] ) {
		dbprintf("set trigger: 16bit ADC pntr %p, sample offset %i, island length %i, island xtal %i, sample %i trigger adc value %i pedestal value %f\n", *ctpntr16bit, islandsampleoffset, islandlength, ixtals, isample, adcsample, ped);
	      }

	    } // end xtal loop

	    // find border samples
	    if ( tq_parameters_odb[itq].save_xtal_border ) {
	      for (ixtals = 0; ixtals < nxtals; ixtals++){ // loop over xtals
		if ( !triggerstore[ixtals] ) continue; // only look for border xtals for triggered xtals

		int iX = ixtals % tq_parameters_odb[itq].gpu_n_segments_x; // x-coordinate of trigger xtal                                        
		int iY = ixtals / tq_parameters_odb[itq].gpu_n_segments_x; // y-coordinate of trigger xtal

		for (int ixborder = iX-1; ixborder <= iX+1; ixborder++) { // loop over x border xtals
		  for (int iyborder = iY-1; iyborder <= iY+1; iyborder++) { // loop over y border xtals

		    if ( iX == ixborder && iY == iyborder ) continue; // skip trigger sample
		    if ( ixborder < 0 || ixborder >= tq_parameters_odb[itq].gpu_n_segments_x ) continue; // out of range or detector array
		    if ( iyborder < 0 || iyborder >= tq_parameters_odb[itq].gpu_n_segments_y ) continue; // out of range or detector array

		    int iborder =  ixborder + iyborder*tq_parameters_odb[itq].gpu_n_segments_x; // calc border xtal index
		    borderstore[iborder] = 1; // store border xtal
		    if ( borderstore[ixtals] ) {
		      dbprintf("set border: trigger xtal %i, border xtal %i\n", ixtals, iborder ); // debug
		    }
		  } // loop over x border xtals      
		} // loop over y border xtals
	      } // loop over trigger xtals
	    } // save border xtals

	    // count number of border. trigger xtals
	    int ntorb = 0;
	    for (ixtals = 0; ixtals < nxtals; ixtals++) {
	      if ( triggerstore[ixtals] || borderstore[ixtals] ) ntorb++; // increment for both trigger xtals and border xtals
	    }

	    // build island header = 32-bit time, 32-bit length, 32-bit number of trigger+border xtals (NEW)
	    dbprintf("build island header  time, length, number of xtals %i, %i, %i\n", islandtime, islandlength, ntorb); // debug
	    memcpy(pdata, &islandtime, 2*sizeof(pdata[0]));
	    pdata += 2; // skip to next 32-bit word (pdata is 16-bit words)
	    memcpy(pdata, &islandlength, 2*sizeof(pdata[0]));
	    pdata += 2; // skip to next 32-bit word (pdata is 16-bit words)
	    memcpy(pdata, &ntorb, 2*sizeof(pdata[0]));
	    pdata += 2; // skip to next 32-bit word (pdata is 16-bit words)

	    // samples = number of xtals * ( 32bit xtal identifier (NEW), 16bit samples )
	    for (ixtals = 0; ixtals < nxtals; ixtals++){

	      if ( triggerstore[ixtals] || borderstore[ixtals] ) { // if trigger / border xtal then store indetifier and samples
		dbprintf("do memcpy of samples for xtal %i, length %i\n", ixtals, islandlength); // debug
		memcpy(pdata, &ixtals, 2*sizeof(pdata[0])); // store xtal index
		pdata += 2; // skip to next 32-bit word (pdata is 16-bit words)
		memcpy( pdata, (ctpntr16bit + islandsampleoffset + ixtals*islandlength), islandlength*sizeof(pdata[0])); // store samples
		pdata += islandlength; // skip islandlength*16-bit samples
	      }
	    }

	    // problem with divide by two? make all pointer 16-bit with appropriate pointer arithmetic for pointer incrementing
	    islandtimeoffset += 2 + nxtals*islandlength/2; // 32-bit pointer offset for location of island times in untruncated calo bank
	    islandlengthoffset += 2 + nxtals*islandlength/2; // 32-bit pointer offset for location of island lengths in untruncated calo bank
	    islandsampleoffset += 4 + nxtals*islandlength; // 16-bit pointer offset for location of ADC samples in untruncated calo bank
	  }

	  struct timeval t_end_truncate;
	  gettimeofday(&t_end_truncate, NULL); // record time for end of truncation
	  dbprintf("%s(%d): duration of truncation of CT bank %e us \n", __func__, __LINE__, toddiff( &t_end_truncate, &t_start_truncate));
	  // end development work for truncating CT bank to trigger crystals
	} else {

	  memcpy(pdata, GPUDATA->gpu_data_proc[itq]+t_offset, t_size*sizeof(pdata[0]) );   // memcpy of full calo data
	  if(sizeof(pdata) != 0) pdata += t_size;
	}

	dbprintf("%s(%d): sizeof(pdata): %lu\n", __func__, __LINE__, sizeof(pdata[0]));
	bk_close(pevent, pdata);
	dbprintf("%s(%d): made T-method databank %s size (bytes) %d\n", __func__, __LINE__, bk_name, t_size*sizeof(pdata[0]) );

	// bank-by-bank compression
	if ( amc13_settings_odb.bankbybank_lossless_compression ) { 
	  BANK_HEADER *bankheader = (BANK_HEADER*) pevent;
	  int banksize = sizeof(BYTE)*bk_locate( pevent, bk_name,  (void**)&bdata); // bdata is byte pointer to location of data
	  bk_name[0] = tolower(bk_name[0]); // name for compressed bank
	  if ( fe_compress_z2(pevent, // char pointer to location of event
		(char*)(bdata - sizeof(BANK32)), // char pointer to location of input
		banksize + sizeof(BANK32), // data size + header size
		max_event_size-(bankheader->data_size+sizeof(BANK_HEADER)+sizeof(EVENT_HEADER)), // available space
		0, bk_name) != FE_SUCCESS ) // bank name
	  {
	    printf("%s(%d): fill compressed databank - compression failed\n",__func__, __LINE__);
	  } else {
	    printf("%s(%d): fill compressed databank - compression succeeded\n",__func__, __LINE__);
	    bk_name[0] = toupper(bk_name[0]); // name for compressed bank
	    bk_delete(pevent,bk_name); // if compression successful then delete uncompressed bank
	  }
	} // bank-by-bank lossless compression
      }//if write T-method databank

      // create bank with GPU template fit results
      if(tq_parameters_odb[itq].fit_islands>0){

	if ( GPUmuonfillcounter%tq_parameters_odb[itq].fit_prescale_factor == 0 ) { // prescaling of fitting 

	  dbprintf("%s(%d): fill CF data bank\n",__func__, __LINE__);
	  sprintf(bk_name,"%sF%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
	  bk_create(pevent, bk_name, TID_DWORD, (void**)&hdata);

	  // fill CF bank
	  unsigned int nsegments =  *(GPUDATA->gpu_data_proc[itq]+3);

	  *hdata++ = nsegments;

	  for (unsigned int seg_num = 0; seg_num < nsegments; ++seg_num) {

	    unsigned int nPulses = host_fitresult[seg_num].nPulses;
	    *hdata++ = nPulses;
	    hdata = (unsigned int *)std::copy(
		host_fitresult[seg_num].fit_results,
		host_fitresult[seg_num].fit_results + nPulses,
		(pulseFinderResult *)hdata);
	    //printf("Fit result : seg %d, pulse number %d, time %f , energy %f \n", seg_num, nPulses, host_fitresult[seg_num].fit_results[0].time,host_fitresult[seg_num].fit_results[0].energy);
	  }
	  bk_close(pevent, hdata);

	  // bank-by-bank compression
	  if ( amc13_settings_odb.bankbybank_lossless_compression ) { 
	    BANK_HEADER *bankheader = (BANK_HEADER*) pevent;
	    int banksize = sizeof(BYTE)*bk_locate( pevent, bk_name,  (void**)&bdata); // bdata is byte pointer to location of data
	    bk_name[0] = tolower(bk_name[0]); // name for compressed bank
	    if ( fe_compress_z2(pevent, // char pointer to location of event
		  (char*)(bdata - sizeof(BANK32)), // char pointer to location of input
		  banksize + sizeof(BANK32), // data size + header size
		  max_event_size-(bankheader->data_size+sizeof(BANK_HEADER)+sizeof(EVENT_HEADER)), // available space
		  0, bk_name) != FE_SUCCESS ) // bank name
	    {
	      printf("%s(%d): fill compressed databank - compression failed\n",__func__, __LINE__);
	    } else {
	      printf("%s(%d): fill compressed databank - compression succeeded\n",__func__, __LINE__);
	      bk_name[0] = toupper(bk_name[0]); // name for compressed bank
	      bk_delete(pevent,bk_name); // if compression successful then delete uncompressed bank
	    }
	  }
	} // fit prescaling
      } // if fit_islands >0
    } // end TQ processing ON

    // make fill-summed histogram databank
    if ( tq_parameters_odb[itq].store_hist && Fill_type==1 && ( (GPUmuonfillcounter+1) % tq_parameters_odb[itq].flush_hist ) == 0 ) {

      int nsegments =  *(GPUDATA->gpu_data_proc[itq]+3);

      dbprintf("%s(%d): fill CQ data bank\n",__func__, __LINE__);
      sprintf(bk_name,"%sQ%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
      dbprintf("Fill bank %s, fill number %ld\n",bk_name,Midasfillnumber);
      bk_create(pevent, bk_name, TID_DWORD, (void**)&hdata);
      *hdata++ = GPUDATA->gpu_data_his_size[itq] / sizeof(hdata[0]); // first word is number of elements of fill-summed Q method histogram
      *hdata++ = tq_parameters_odb[itq].first_sample_in_hist; // second word is first sample of histogram
      *hdata++ = tq_parameters_odb[itq].last_sample_in_hist; // third word is last sample of histogram
      *hdata++ = tq_parameters_odb[itq].time_divide_hist;; // fourth word is number rebinning factor of first sub-histogram
      *hdata++ = tq_parameters_odb[itq].rebin_intervals_in_hist; // 5th word is number of rebin intervals (sub-histograns with different rebinning)
      *hdata++ = tq_parameters_odb[itq].rebin_increment_in_hist; // 6th word is number of rebin intervals (sub-histograns with different rebinning)
      *hdata++ = nsegments; // 7th word is number of segments / detectors in histogram
      memcpy( hdata, GPUDATA->gpu_data_his[itq], GPUDATA->gpu_data_his_size[itq]); // data array of sixe given by first word of the data bank
      hdata += GPUDATA->gpu_data_his_size[itq] / sizeof(hdata[0]);
      bk_close(pevent, hdata);

      dbprintf("%s(%d): made fill-summed Q-method databank %s size 0x%08x, data[0] 0x%08x, readout electronics fill number %lu, GPUmuonfillcounter %i\n", 
	  __func__, __LINE__, bk_name, GPUDATA->gpu_data_his_size[itq], *GPUDATA->gpu_data_his[itq], GPUDATA->gpu_data_header[1], GPUmuonfillcounter);

      // bank-by-bank compression
      if ( amc13_settings_odb.bankbybank_lossless_compression ) { 
	BANK_HEADER *bankheader = (BANK_HEADER*) pevent;
	int banksize = sizeof(BYTE)*bk_locate( pevent, bk_name,  (void**)&bdata); // bdata is byte pointer to location of data
	bk_name[0] = tolower(bk_name[0]); // name for compressed bank
	if ( fe_compress_z2(pevent, // char pointer to location of event
	      (char*)(bdata - sizeof(BANK32)), // char pointer to location of input
	      banksize + sizeof(BANK32), // data size + header size
	      max_event_size-(bankheader->data_size+sizeof(BANK_HEADER)+sizeof(EVENT_HEADER)), // available space
	      0, bk_name) != FE_SUCCESS ) // bank name
	{
	  printf("%s(%d): fill compressed databank - compression failed\n",__func__, __LINE__);
	} else {
	  printf("%s(%d): fill compressed databank - compression succeeded\n",__func__, __LINE__);
	  bk_name[0] = toupper(bk_name[0]); // name for compressed bank
	  bk_delete(pevent,bk_name); // if compression successful then delete uncompressed bank
	}
      } // bank-by-bank lossless compression

    } // end fill-summed histogram ON

  } //end of loop over TQ methods

  //make clock / control data banks
  if ( frontend_index == 0 ) {

    sprintf(bk_name,"TTCA");

    bk_create(pevent, bk_name, TID_DWORD, (void**)&hdata); // TID_DWORD unsigned int of FOUR bytes
    memcpy( hdata, GPUDATA->gpu_data_header_amc13, GPUDATA->gpu_data_header_amc13_size); 
    hdata += GPUDATA->gpu_data_header_amc13_size / sizeof(hdata[0]);
    bk_close(pevent, hdata);
    dbprintf("%s(%d): made ccc AMC13 header / trailer databank %s size (bytes) 0x%08x, amc13[0] 0x%16lx, readout electronics fill number %lu\n", 
	  __func__, __LINE__, bk_name, GPUDATA->gpu_data_header_amc13_size, *GPUDATA->gpu_data_header_amc13, GPUDATA->gpu_data_header[1]);

    sprintf(bk_name,"TTCB");

    bk_create(pevent, bk_name, TID_SHORT, (void**)&pdata); // TID_SHORT signed int of two bytes                         
    dbprintf("created raw bank %s, now do memcpy\n",bk_name);
    memcpy( pdata, GPUDATA->gpu_data_raw, GPUDATA->gpu_data_raw_size); 
    dbprintf("raw data memcpy complete\n");
    pdata += GPUDATA->gpu_data_raw_size / sizeof(pdata[0]);
    bk_close(pevent, pdata);                                               
    dbprintf("%s(%d): made ccc raw databank %s size %d, *data %p, data[0] 0x%04x, readout electronics fill number %lu\n", 
	  __func__, __LINE__, bk_name, GPUDATA->gpu_data_raw_size,  GPUDATA->gpu_data_raw, *GPUDATA->gpu_data_raw, GPUDATA->gpu_data_header[1]);

    sprintf(bk_name,"TTCZ");

    bk_create(pevent, bk_name, TID_DWORD, (void**)&hdata); // TID_DWORD unsigned int of four bytes                        
    memcpy( hdata, GPUDATA->gpu_data_tail, GPUDATA->gpu_data_tail_size); 
    hdata += GPUDATA->gpu_data_tail_size / sizeof(hdata[0]); 
    bk_close(pevent, hdata);
    dbprintf("%s(%d): made ccc trailer databank %s size 0x%08x, tail[0] 0x%16lx, readout electronics fill number %lu\n", 
	  __func__, __LINE__, bk_name, GPUDATA->gpu_data_tail_size, *GPUDATA->gpu_data_tail, GPUDATA->gpu_data_header[1]);

  }     

  // make rider raw databank - name depends on fill type , CAnn, LAnn, PAnn
  if ( frontend_index != 0 ) {
    if ( Fill_type > 1 || ( amc13_settings_odb.store_raw && !( ( GPUmuonfillcounter - amc13_settings_odb.prescale_offset_raw ) % amc13_settings_odb.prescale_raw ) && AMC13fillcounter>=amc13_settings_odb.prescale_offset_raw)) {

      switch(Fill_type){
        case 0x1 : 
          sprintf(bk_name,"CR%02i",frontend_index); // muon fill type
          break;
        case 0x2 :
          sprintf(bk_name,"LR%02i",frontend_index); // laser fill type 
          break;
        case 0x3 :
          sprintf(bk_name,"PR%02i",frontend_index); // pedestal fill type  
          break;
        case 0x4 :
          sprintf(bk_name,"AR%02i",frontend_index);// Asynchronous fill type 
          break;
        default:
          cm_msg(MERROR, __FILE__, "Warning! Invalid Fill type (R) %i",Fill_type);
          break;
      }

      bk_create(pevent, bk_name, TID_SHORT, (void**)&pdata); // TID_SHORT signed int of two bytes                         
      dbprintf("created raw bank %s, now do memcpy\n",bk_name);
      memcpy( pdata, GPUDATA->gpu_data_raw, GPUDATA->gpu_data_raw_size); 
      dbprintf("raw data memcpy complete\n");
      pdata += GPUDATA->gpu_data_raw_size / sizeof(pdata[0]);
      bk_close(pevent, pdata);                                               
      dbprintf("%s(%d): made raw databank %s size %d, *data %p, data[0] 0x%04x, readout electronics fill number %lu\n", 
	    __func__, __LINE__, bk_name, GPUDATA->gpu_data_raw_size,  GPUDATA->gpu_data_raw, *GPUDATA->gpu_data_raw, GPUDATA->gpu_data_header[1]);

      /*
      if ( amc13_settings_odb.store_raw && !( ( GPUmuonfillcounter - amc13_settings_odb.prescale_offset_raw ) % amc13_settings_odb.prescale_raw ) && AMC13fillcounter>=amc13_settings_odb.prescale_offset_raw)
      {
      printf ("Output raw data %d, %d", *GPUDATA->gpu_data_raw, GPUDATA->gpu_data_header[1],GPUDATA->gpu_data_raw_size);
      }
       */

      // bank-by-bank compression
      if ( amc13_settings_odb.bankbybank_lossless_compression ) { 
        BANK_HEADER *bankheader = (BANK_HEADER*) pevent;
        int banksize = sizeof(BYTE)*bk_locate( pevent, bk_name,  (void**)&bdata); // bdata is byte pointer to location of data
        bk_name[0] = tolower(bk_name[0]); // name for compressed bank
        if ( fe_compress_z2(pevent, // char pointer to location of event
              (char*)(bdata - sizeof(BANK32)), // char pointer to location of input
              banksize + sizeof(BANK32), // data size + header size
              max_event_size-(bankheader->data_size+sizeof(BANK_HEADER)+sizeof(EVENT_HEADER)), // available space
              0, bk_name) != FE_SUCCESS ) // bank name
        {
          printf("%s(%d): fill compressed databank - compression failed\n",__func__, __LINE__);
        } else {
          printf("%s(%d): fill compressed databank - compression succeeded\n",__func__, __LINE__);
          bk_name[0] = toupper(bk_name[0]); // name for compressed bank
          bk_delete(pevent,bk_name); // if compression successful then delete uncompressed bank
        }
      } // bank-by-bank lossless compression

    }

#endif // USE_GPU

    // make AMC13 header / trailer databank - name depends on fill type , CAnn, LAnn, PAnn
    switch(Fill_type){
      case 0x1 :
        dbprintf("CA bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
        sprintf(bk_name,"CA%02i",frontend_index); // muon fill type                                                                                         
        break;
      case 0x2 :
        dbprintf("LA bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
        sprintf(bk_name,"LA%02i",frontend_index); // laser fill type                                                                                        
        break;
      case 0x3 :
          dbprintf("PA bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
          sprintf(bk_name,"PA%02i",frontend_index); // pedestal fill type                                                                                     
          break;
      case 0x4 :
	      dbprintf("AA bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
	      sprintf(bk_name,"AA%02i",frontend_index);// Asynchronous fill type                                                                                  
	      break;
      default:
	      cm_msg(MERROR, __FILE__, "Warning! Invalid Fill type (A) %i",Fill_type);
	      break;
    }

    bk_create(pevent, bk_name, TID_DWORD, (void**)&hdata); // TID_DWORD unsigned int of FOUR bytes
    memcpy( hdata, GPUDATA->gpu_data_header_amc13, GPUDATA->gpu_data_header_amc13_size); 
    hdata += GPUDATA->gpu_data_header_amc13_size / sizeof(hdata[0]);
    bk_close(pevent, hdata);
    dbprintf("%s(%d): made AMC13 header / trailer databank %s size (bytes) 0x%08x, amc13[0] 0x%16lx, readout electronics fill number %lu\n", 
	  __func__, __LINE__, bk_name, GPUDATA->gpu_data_header_amc13_size, *GPUDATA->gpu_data_header_amc13, GPUDATA->gpu_data_header[1]);

    //Only write rider header banks for single-waveform fills
    if(Fill_type==0x1){
      sprintf(bk_name,"CB%02i",frontend_index); // muon fill type 
      bk_create(pevent, bk_name, TID_DWORD, (void**)&hdata); // TID_DWORD unsigned int of FOUR bytes
      memcpy( hdata, GPUDATA->gpu_data_header_rider, GPUDATA->gpu_data_header_rider_size); 
      hdata += GPUDATA->gpu_data_header_rider_size / sizeof(hdata[0]);
      bk_close(pevent, hdata);
      dbprintf("%s(%d): made Rider header / trailer databank %s size (bytes) 0x%08x, rider[0] 0x%16lx, readout electronics fill number %lu\n", 
	  __func__, __LINE__, bk_name, GPUDATA->gpu_data_header_rider_size, *GPUDATA->gpu_data_header_rider, GPUDATA->gpu_data_header[1]);
    }
    // make trailer databank 
    switch(Fill_type){
      case 0x1 :
	      dbprintf("CZ bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
	      sprintf(bk_name,"CZ%02i",frontend_index); // muon fill type
	      break;
      case 0x2 :
	      dbprintf("LZ bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
	      sprintf(bk_name,"LZ%02i",frontend_index); // laser fill type
	      break;
      case 0x3 :
	      dbprintf("PZ bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
	      sprintf(bk_name,"PZ%02i",frontend_index); // pedestal fill type
	      break;
      case 0x4 :
	      dbprintf("AZ bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
	      sprintf(bk_name,"AZ%02i",frontend_index);// Asynchronous fill type
	      printf("Fill type should be 7, %i\n",Fill_type);
	      break;
      default:
	      cm_msg(MERROR, __FILE__, "Warning! Invalid Fill type (Z) %i",Fill_type);
	      break;
    }

    bk_create(pevent, bk_name, TID_DWORD, (void**)&hdata); // TID_DWORD unsigned int of four bytes                        
    memcpy( hdata, GPUDATA->gpu_data_tail, GPUDATA->gpu_data_tail_size); 
    hdata += GPUDATA->gpu_data_tail_size / sizeof(hdata[0]); 
    bk_close(pevent, hdata);
    dbprintf("%s(%d): made trailer databank %s size 0x%08x, tail[0] 0x%16lx, readout electronics fill number %lu\n", 
	  __func__, __LINE__, bk_name, GPUDATA->gpu_data_tail_size, *GPUDATA->gpu_data_tail, GPUDATA->gpu_data_header[1]);

    // get timing / performance data and store timing /performance data before unlock gpu thread
    // note the CC header bank is wrote at end in order to complete the timing data  
    status = gettimeofday( &t_got_data, NULL);
    trigger_info.time_slave_got_data_s  = t_got_data.tv_sec;
    trigger_info.time_slave_got_data_us = t_got_data.tv_usec;

    // make more header / timing data
    // array elements 17, 18 reserced for compression timing data
    GPUDATA->gpu_data_header[15] = t_got_data.tv_sec;
    GPUDATA->gpu_data_header[16] = t_got_data.tv_usec;
    //This is for run3 and before
    /*
       GPUDATA->gpu_data_header[19] = TCPfillnumber;
       GPUDATA->gpu_data_header[20] = GPUfillnumber;
       GPUDATA->gpu_data_header[21] = GPUmuonfillcounter; // the muon fill counter as set for fill in gpu_thread
     */
    //In Run 4 nothing has to be done here
    //TODO Check Data!

    // fix size of header / timing data
    perf_data_size = 22*sizeof(GPUDATA->gpu_data_header[0]);
    // perf_data, perf_data_size are copies of GPUDATA->gpu_data_header, GPUDATA->gpu_data_header_size in order to release gpu lock before data compression
    memcpy( perf_data, GPUDATA->gpu_data_header, perf_data_size); 

  

  // unlocking gpu thread access to GPU output buffer (commented out because causing problems)
  pthread_mutex_unlock( &mutex_GPU_buf[GPUbufferindex] );

  // for rider's make losslessly-compressed processed databank 

  
  if ( frontend_index != 0 ) {

    dbprintf("%s(%d): lossless data compression %i\n", __func__, __LINE__, amc13_settings_odb.lossless_compression);
    if ( amc13_settings_odb.lossless_compression ){
      BANK_HEADER *bank_header = (BANK_HEADER *) pevent;
      dbprintf("%s(%d): fill FZ data bank, data size %lu\n",__func__, __LINE__, bank_header->data_size+sizeof(BANK_HEADER));
      if ( fe_compress_z(pevent, // char pointer to location of output
	    (char*)bank_header, // char pointer to location of input
	    bank_header->data_size+sizeof(BANK_HEADER), // data size + header size
	    max_event_size-(bank_header->data_size+sizeof(BANK_HEADER)+sizeof(EVENT_HEADER)), // available space
	    0) != FE_SUCCESS ){
        // compression failed. store raw dats
        printf("%s(%d): fill FZ data bank - compression failed\n",__func__, __LINE__);
      }

      // if losslessly compressing the midas banks then delete the uncompressed banks

      #ifdef USE_GPU	
            for (int itq = 0; itq < TQMETHOD_MAX; itq++){

        if ( tq_parameters_odb[itq].TQ_on && Fill_type==1 ) {

          sprintf(bk_name,"%sS%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
          bk_delete(pevent,bk_name);
          dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);
          sprintf(bk_name,"%sP%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
          bk_delete(pevent,bk_name);
          dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);
          sprintf(bk_name,"%sT%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
          bk_delete(pevent,bk_name);
          dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);

        } // end delete CQ, CP, CT banks

        //if ( tq_parameters_odb[itq].store_hist && Fill_type==1 && !( (AMC13fillcounter-1-tq_parameters_odb[itq].flush_offset_hist) % tq_parameters_odb[itq].flush_hist ) ) 
        // flush offset is disabled for run4, and making sure that fill0 is not flushed.
        if ( tq_parameters_odb[itq].store_hist && Fill_type==1 && ( (GPUmuonfillcounter+1) % tq_parameters_odb[itq].flush_hist ) == 0) {

          sprintf(bk_name,"%sQ%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
          bk_delete(pevent,bk_name);
          dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);

        } // end delete CH bank

        if ( tq_parameters_odb[itq].fit_islands>0 && Fill_type==1 ){

          sprintf(bk_name,"%sF%02i", tq_parameters_odb[itq].TQ_bankprefix, frontend_index);
          bk_delete(pevent,bk_name);
          dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);

        } // end delete CF bank

            } // end loop over TQ methods

            if ( frontend_index == 0 ) {

        sprintf(bk_name,"CA00");
        bk_delete(pevent,bk_name);
        dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);
        sprintf(bk_name,"CR00");
        bk_delete(pevent,bk_name);
        dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);
        sprintf(bk_name,"CZ00");
        bk_delete(pevent,bk_name);
        dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);

            } // end delete of clock-control databanks

      #endif // USE_GPU


      if (Fill_type>1 || ( amc13_settings_odb.store_raw && !( ( GPUmuonfillcounter - amc13_settings_odb.prescale_offset_raw ) % amc13_settings_odb.prescale_raw ) && AMC13fillcounter>=amc13_settings_odb.prescale_offset_raw)) {

        if (Fill_type == 0x1) {
          sprintf(bk_name,"CR%02i",frontend_index); // muon fill type
        } 
        else if(Fill_type == 0x2) {
          sprintf(bk_name,"LR%02i",frontend_index); // laser fill type
        } 
        else if(Fill_type == 0x3) {
          sprintf(bk_name,"PR%02i",frontend_index); // pedestal fill type
        }
        else if(Fill_type == 0x4) {
          sprintf(bk_name,"AR%02i",frontend_index); // async fill type
        }
        bk_delete(pevent,bk_name);
        dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);

      } // end delete CR bank, etc

      // delete CA / LA / PA banks
      if (Fill_type == 0x1) {
	      sprintf(bk_name,"CA%02i",frontend_index); // muon fill type
      } 
      else if(Fill_type == 0x2) {
	      sprintf(bk_name,"LA%02i",frontend_index); // laser fill type
      } 
      else if(Fill_type == 0x3) {
	      sprintf(bk_name,"PA%02i",frontend_index); // pedestal fill type
      } 
      else if(Fill_type == 0x4) {
	      sprintf(bk_name,"AA%02i",frontend_index); // async fill type
      } 
      bk_delete(pevent,bk_name);
      dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);

      // delete CB bank (there's no filling of "CB" bank equivalent for laser, ped, async fills
      if (Fill_type == 0x1) {
        sprintf(bk_name,"CB%02i",frontend_index); // muon fill type
        bk_delete(pevent,bk_name);
        dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);
      }

      // delete CC / LC / PC / AC banks
      if (Fill_type == 0x1) {
	      sprintf(bk_name,"CC%02i",frontend_index); // muon fill type
      } 
      else if(Fill_type == 0x2) {
	      sprintf(bk_name,"LC%02i",frontend_index); // laser fill type
      } 
      else if(Fill_type == 0x3) {
	      sprintf(bk_name,"PC%02i",frontend_index); // pedestal fill type
      } 
      else if(Fill_type == 0x4) {
	      sprintf(bk_name,"AC%02i",frontend_index); // async fill type
      } 
      bk_delete(pevent,bk_name);
      dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);

      // delete CZ / LZ / PZ / AZ banks
      if (Fill_type == 0x1) {
	      sprintf(bk_name,"CZ%02i",frontend_index); // muon fill type
      } 
      else if(Fill_type == 0x2) {
	      sprintf(bk_name,"LZ%02i",frontend_index); // laser fill type
      } 
      else if(Fill_type == 0x3) {
	      sprintf(bk_name,"PZ%02i",frontend_index); // pedestal fill type
      } 
      else if(Fill_type == 0x4) {
	      sprintf(bk_name,"AZ%02i",frontend_index); // async fill type
      } 
      bk_delete(pevent,bk_name);
      dbprintf("%s(%d): deleted bank %s\n", __func__, __LINE__, bk_name);


    } // end lossless compression
  } // end rider's only
  


  // calculate the total time for data compression and bank deletion
  struct timeval t_done_compression;
  status = gettimeofday( &t_done_compression, NULL);
  perf_data[17] = t_done_compression.tv_sec;
  perf_data[18] = t_done_compression.tv_usec;

  t_done_compression.tv_sec -= t_got_data.tv_sec;
  t_done_compression.tv_usec -= t_got_data.tv_usec;
  if ( t_done_compression.tv_usec < 0 )
  {
    t_done_compression.tv_sec -= 1;
    t_done_compression.tv_usec += 1000000;
  }    
  dbprintf("%s(%d): lossless compression and bank deletion duration: dt = %li s %li us\n", 
      __func__, __LINE__, t_done_compression.tv_sec, t_done_compression.tv_usec);

  // make rider performance databank -  CCnn for Rider data
  if ( frontend_index != 0 ) {

    // make timing / performance databank  - name depends on fill type, CCnn, LCnn, PCnn
    switch(Fill_type){
      case 0x1 :
	dbprintf("CC bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
	sprintf(bk_name,"CC%02i",frontend_index); // muon fill type                                                                                         
	break;
      case 0x2 :
	dbprintf("LC bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
	sprintf(bk_name,"LC%02i",frontend_index); // laser fill type                                                                                        
	break;
      case 0x3 :
	dbprintf("PC bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
	sprintf(bk_name,"PC%02i",frontend_index); // pedestal fill type                                                                                     
	break;
      case 0x4 :
	dbprintf("AC bank,  Fill_type %i, frontend_index %i\n", Fill_type, frontend_index );
	sprintf(bk_name,"AC%02i",frontend_index);// Asynchronous fill type                                                                                  
	break;
      default:
	cm_msg(MERROR, __FILE__, "Warning! Invalid Fill type");
	break;
    }

    bk_create(pevent, bk_name, TID_DWORD, (void**)&hdata); // TID_DWORD unsigned int of FOUR bytes
    // perf_data, perf_data_size are copies of gpu_data_header, gpu_data_header_size in order to release gpu lock before data compression
    // memcpy( hdata, gpu_data_header, gpu_data_header_size); 
    memcpy( hdata, perf_data, perf_data_size); 
    hdata += perf_data_size / sizeof(hdata[0]);
    bk_close(pevent, hdata);
    dbprintf("%s(%d): made timing databank %s size (bytes) 0x%08x, amc13[0] 0x%16lx, readout electronics fill number %lu\n", 
	__func__, __LINE__, bk_name, perf_data_size, *perf_data, perf_data[1]);

  }

  // calculate / print some useful timing data
  // dt1 time between start, finish of tcp_thread read() of data
  long int dt1_s = trigger_info.time_slave_got_data_s;
  dt1_s -= trigger_info.time_gputhread_finished_s;
  long int dt1_us =  trigger_info.time_slave_got_data_us;
  dt1_us -= trigger_info.time_gputhread_finished_us;
  if ( dt1_us < 0 )
  {
    dt1_s -= 1;
    dt1_us += 1000000;
  }

  // dt2 time between tcp_thread read completion and read_trigger_event unlocked
  long int dt2_s = trigger_info.time_slave_got_data_s;
  dt2_s -= trigger_info.time_tcp_finish_data_read_s;
  long int dt2_us =  trigger_info.time_slave_got_data_us;
  dt2_us -= trigger_info.time_tcp_finish_data_read_us;
  if ( dt2_us < 0 )
  {
    dt2_s -= 1;
    dt2_us += 1000000;
  }

  // dt3 total duration of readout through TCP, GPU, midas FE threads
  long int dt3_s = trigger_info.time_slave_got_data_s;
  dt3_s -= trigger_info.time_tcp_finish_header_read_s;
  long int dt3_us =  trigger_info.time_slave_got_data_us;
  dt3_us -= trigger_info.time_tcp_finish_header_read_us;
  if ( dt3_us < 0 )
  {
    dt3_s -= 1;
    dt3_us += 1000000;
  }

  // dt4 total duration of readout through TCP, GPU, midas FE threads
  long int dt4_s = trigger_info.time_tcp_finish_data_read_s;
  dt4_s -= trigger_info.time_tcp_finish_header_read_s;
  long int dt4_us =  trigger_info.time_tcp_finish_data_read_us;
  dt4_us -= trigger_info.time_tcp_finish_header_read_us;
  if ( dt4_us < 0 )
  {
    dt4_s -= 1;
    dt4_us += 1000000;
  }

  //dbprintf("%s(%d): EOF master-slave propogation time: dt = %li s %li us\n", __func__, __LINE__, dt0_s, dt0_us);
  dbprintf("%s(%d): tcp got header to tcp got data duration: dt = %li s %li us\n", __func__, __LINE__, dt4_s, dt4_us);
  dbprintf("%s(%d): tcp got data to MFE done duration: dt = %li s %li us\n", __func__, __LINE__, dt2_s, dt2_us);
  dbprintf("%s(%d): tcp got header to MFE done duration: dt = %li s %li us\n", __func__, __LINE__, dt3_s, dt3_us);
  dbprintf("%s(%d): gpu done to MFE done  duration: dt = %li s %li us\n", __func__, __LINE__, dt1_s, dt1_us);
  dbprintf("%s(%d): midas bank size: %i\n", __func__, __LINE__, bk_size(pevent));


  // used for legacy parallel port debugging  
  #ifdef USE_PARALLEL_PORT
    printf("read_trigger_event: write pulse to parallel port address 0x%08x\n", pp_addr);

    outb( 0xff, pp_addr);
    usleep(20);
    outb( 0x00, pp_addr);
  #endif

  got_triggers_in_run++; // increment counter for received fills

  free (perf_data);

  // MIDAS thread lock
  pthread_mutex_lock(&mutex_midas);
  Midasfillnumber++;
  pthread_mutex_unlock(&mutex_midas);

  //Before return, update fill numbers in odb
  int MidasFill = 0;

  MidasFill = Midasfillnumber % (0xffffffff/2);

//  printf("update odb\n");
  db_set_value(hDB, 0, (MonitorODBRootKey + std::string("/Midas Fill Number")).c_str(), &MidasFill, sizeof(MidasFill), 1, TID_INT);

  //Artificial delay
  //usleep(50000);
//  printf("end reading\n");

  return bk_size(pevent);
}
  


