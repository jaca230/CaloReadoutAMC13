/* timetool.h --- 
 * 
 * Filename: timetool.h
 * Description: 
 * Author: g minus two
 * Maintainer: 
 * Created: Wed Oct 26 13:49:53 2011 (-0400)
 * Version: 
 * Last-Updated: Fri Feb 17 12:01:53 2012 (-0500)
 *           By: g minus two
 *     Update #: 11
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
#ifndef timetool_h
#define timetool_h

/* make functions callable from a C++ program */
#ifdef __cplusplus
extern "C" {
#endif

  extern void timer_start(void);
  extern void timer_stop(void);
  extern void time_print(const char *msg);

#ifdef __cplusplus
}
#endif 

#endif /* timetool_h defined */
/* timetool.h ends here */
