/**
 * @file    frontend.h
 * @author  Vladimir Tishchenko <tishenko@pa.uky.edu> Tim Gorringe <gorringe@pa.uky.edu>
 * @date    Friday Aug 31 2012
 * @date    Last-Updated: Mon Apr 28 05:47:24 2014 (-0500)
 *          By : Data Acquisition
 *          Update #: 39 
 * @version $Id$
 * 
 * @copyright (c) new (g-2) collaboration
 * 
 * @brief   common header file for frontends
 * 
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */

#ifndef frontend_h
#define frontend_h

//extern void fe_error(const char *msg, const char *file, const char *line);
extern pthread_mutex_t mutex_midas;

typedef struct s_trigger_info
{
  DWORD trigger_nr;            ///< trigger number (via RPC message from master)
  DWORD trigger_mask;          ///< trigger mask (via RPC message from master)
  DWORD time_master_got_eof_s;                ///< master EOF trigger time (via RPC message from master), seconds
  DWORD time_master_got_eof_us;               ///< master EOF trigger time (via RPC message from master), microseconds
  DWORD time_slave_got_eof_s;           ///< slave EOF trigger time called from master via RPC message, seconds
  DWORD time_slave_got_eof_us;          ///< slave EOF trigger time called from master via RPC message, microseconds
  DWORD time_slave_lock_dataready_s;           ///< slave locking mutex_data_ready in read_trigger_event
  DWORD time_slave_lock_dataready_us;          ///< slave locking mutex_data_ready in read_trigger_event
  DWORD time_slave_got_data_s;           ///< slave got data fron tcp_thread and unloacked tcp_thread, seconds
  DWORD time_slave_got_data_us;          ///< slave got data fron tcp_thread and unloacked tcp_thread, microseconds
  DWORD time_tcp_start_read_s;           ///< start time of tcp read in tcp_thread, seconds
  DWORD time_tcp_start_read_us;          ///< start time of tcp read in tcp_thread, microseconds
  DWORD time_tcp_finish_header_read_s;           ///< finish time of tcp read in tcp_thread, seconds
  DWORD time_tcp_finish_header_read_us;          ///< finish time of tcp read in tcp_thread, microseconds
  DWORD time_tcp_finish_data_read_s;           ///< finish time of tcp read in tcp_thread, seconds
  DWORD time_tcp_finish_data_read_us;          ///< finish time of tcp read in tcp_thread, microseconds
  DWORD time_gputhread_started_s;           ///< woke-up gpu_thread for processing, seconds
  DWORD time_gputhread_started_us;          ///< woke-up gpu_thread for processing, microseconds
  DWORD time_gputhread_copytogpu_done_s;           ///< copying to gpu done, seconds
  DWORD time_gputhread_copytogpu_done_us;          ///< copying to gpu done, microseconds
  DWORD time_gputhread_finished_s;           ///< gpu_thread finished processing, seconds
  DWORD time_gputhread_finished_us;          ///< gpu_thread finished processing, microseconds
} S_TRIGGER_INFO;

extern S_TRIGGER_INFO trigger_info;
extern BOOL data_avail; 

extern unsigned long Midasfillnumber;

#endif // frontend_h defined
