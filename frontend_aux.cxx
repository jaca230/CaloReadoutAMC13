/**
 * @file    frontend_aux.c
 * @author  Vladimir Tishchenko <tishenko@pa.uky.edu>
 * @date    Wed Jan 18 10:35:00 2012
 * @date    Last-Updated: Fri Feb 17 11:36:28 2012 (-0500)
 *          By : g minus two
 *          Update #: 16 
 * @version $Id$
 * 
 * @copyright (c) new (g-2) collaboration
 * 
 * @brief   
 * 
 * @details 
 * 
 * @todo Document this code 
 * 
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <midas.h>

#include "frontend_aux.h"

/** 
 * Prints error message, disconnects from the experiment, terminates the frontend
 * 
 * @param msg error message to print out
 */

void fe_error(const char *msg, const char *file, const int line)
{
  //printf("%s. File %s, line %i\n",msg,file,line);
  //printf("%s. File , line\n",msg);
  //perror(msg);
  cm_msg(MERROR, msg, "File %s, line %i", file, line);
  cm_disconnect_experiment();
  exit(1);
}
