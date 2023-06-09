#ifndef Rider_library_h                                                                                 
#define Rider_library_h                                                                                 
                                                                                                        
#include <string>                                                                                       
#include <stdint.h>                                                                                     
#include <stdlib.h>                                                                                     
#include <stdio.h>                                                                                      
#include <vector>                                                                                       
#include <cassert>                                                                                      
#include <unistd.h>                                                                                     
#include <sys/types.h>                                                                                  
#include <sys/time.h>                                                                                   
#include <termios.h>                                                                                    
#include <sstream>                                                                                      
                                                                                                        
#include "hcal/amc13/AMC13.hh"     

class Rider_library {
 public:
  Rider_library() { };
  ~Rider_library() { };

  void WR_REG(uhal::HwInterface *wfd, int reg_num, int value);                                          
  void WR_REG(uhal::HwInterface *wfd, std::string channel,int reg_num, int value);                                          
  void RD_REG(uhal::HwInterface *wfd, int reg_num); 

 private:

};

#endif
