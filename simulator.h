/**
 * @file    frontends/FakeCalo/simulator.h
 * @author  Vladimir Tishchenko <tishenko@pa.uky.edu>
 * @date    Thu May 24 17:16:09 2012
 * @date    Last-Updated: Fri Jun  1 18:27:51 2012 (-0500)
 *          By : Data Acquisition
 *          Update #: 8 
 * @version $Id$
 * 
 * @copyright (c) new (g-2) collaboration
 * 
 * @brief   Header file for calorimeter data simulator
 * 
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */
#ifndef simulator_h
#define simulator_h


/**
 * Simulator thread info
 */ 

typedef struct 
{
  pthread_t         thread_id;     /* ID returned by pthread_create() */

  pthread_mutex_t   mutex;         /* controls thread execution */
  pthread_mutex_t   mutex_data;    /* controls access to the simulator data array */

  u_int16_t         *data;         /* simulated data */
  unsigned int      data_size;     /* size of the data array */
} CALO_SIMULATOR_THREAD_INFO;

extern CALO_SIMULATOR_THREAD_INFO calo_simulator_thread_info;

extern int calo_simulator_init();

#endif // simulator_h defined

