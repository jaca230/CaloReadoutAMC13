/*
channel_calc.cpp
Created by Wes Gohn 0n July 24, 2014 at 1:48 AM PST

Function to calculate hex value for uTCA channel enable command
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

int channel_calc()
{
  
  unsigned long int channels=(1<<12);
  
  int channel[12];

  for(int i=0;i<12;i++){
    channel[i] = (1<<i);
    //    std::cout << std::hex <<channel[i]<<std::endl;
    //std::cout << channel[i] << std::endl;
    if(amc13_rider_odb[i].board.rider_enabled){
      //    if(i==10 || i==5)
      channels += channel[i];
    }
    
  }
  std::cout<<std::endl <<"AMC13 Channels Enabled: 0x"<< std::hex<<channels<<std::endl;
  
  return channels;
}
