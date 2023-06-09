#ifndef AMC13_library_h
#define AMC13_library_h

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
#include <arpa/inet.h>

#include "hcal/amc13/AMC13.hh"
//#include "hcal/amc13/Actions.hh"

#include <midas.h>

class AMC13_library {
public:
  AMC13_library()  { }; 
  ~AMC13_library() { };

  int AMC13_rg(cms::AMC13*);
  int AMC13_FD_Setup(cms::AMC13*);
  int AMC13_Rider_Setup(cms::AMC13*, int riders_enabled, char readout_ip[16]);
  int AMC13_Reset(cms::AMC13*);
  int AMC13_kill_spartan(cms::AMC13*);
  int AMC13_Write(cms::AMC13*);
  int AMC13_Read(cms::AMC13*);
  int AMC13_bor(cms::AMC13*);

  void AMC13_Dummy();
  
private:
    
};

#endif
  
