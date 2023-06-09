/* timetool.c --- 
 * 
 * Filename: timetool.c
 * Description: 
 * Author: g minus two
 * Maintainer: 
 * Created: Wed Oct 26 13:51:19 2011 (-0400)
 * Version: 
 * Last-Updated: Fri Feb 17 12:02:03 2012 (-0500)
 *           By: g minus two
 *     Update #: 13
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
 * 
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
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "timetool.h"

static struct timeval time_start;
static struct timeval time_stop;

void timer_start(void)
{
  gettimeofday(&time_start,NULL);
}

void timer_stop(void)
{
  gettimeofday(&time_stop,NULL);
}

void time_print(const char *msg)
{
  struct timeval dt;
  printf(msg);
  timersub(&time_stop, &time_start, &dt);
  printf(":::::::::::: %ld sec, %ld usec \n", dt.tv_sec, dt.tv_usec);
}

/* timetool.c ends here */
