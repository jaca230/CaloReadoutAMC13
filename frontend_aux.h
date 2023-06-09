/**
 * @file    frontend_aux.h
 * @author  Vladimir Tishchenko <tishenko@pa.uky.edu>
 * @date    Wed Jan 18 10:13:35 2012
 * @date    Last-Updated: Wed Jan 18 10:43:59 2012 (-0500)
 *          By : g minus two
 *          Update #: 13 
 * @version $Id$
 * 
 * @copyright (c) new (g-2) collaboration
 * 
 * @addtogroup inprogress InProgress 
 *  - \ref frontend.
 * 
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

#ifndef frontend_aux_h
#define frontend_aux_h

/* make functions callable from a C++ program */
#ifdef __cplusplus
extern "C" {
#endif

extern void fe_error(const char *msg, const char *file, const int line);

#ifdef __cplusplus
}
#endif 


#endif // frontend_aux_h defined
