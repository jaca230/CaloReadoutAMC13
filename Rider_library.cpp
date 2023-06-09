/*
Rider Library Class
Wes Gohn & Renee Fatemi, UKY
*/

#include "Rider_library.h"
#include <iostream>
#include <cstdlib>
#include <string>

/*
void Rider_library::WR_REG(uhal::HwInterface *wfd, int reg_num, int value)
{
  std::vector<uint32_t> write_vals;
  write_vals.push_back(0x00000003);
  write_vals.push_back(reg_num);
  write_vals.push_back(value);
  std::cout<<" Rider_library before writeBlock"<<std::endl;
  wfd->getNode("axi.chan0").writeBlock(write_vals);
  std::cout<<" Rider_library before dispatch"<<std::endl;
  wfd->dispatch();
  std::cout<<" Rider_library after dispatch"<<std::endl;
  uhal::ValVector<uint32_t> read_vals = wfd->getNode("axi.chan0").readBlock(1);
  //uhal::ValWord<uint32_t> read_vals = wfd->getNode("axi.chan0").read();
  std::cout<<" Rider_library after read"<<std::endl;


  wfd->dispatch();
  std::cout << "Response code: "<<std::hex<<read_vals.value()[0]<<std::endl;

  std::cout<<" Rider_library after 2nd  dispatch"<<std::endl;
}
*/
 
void Rider_library::WR_REG(uhal::HwInterface *wfd, int reg_num, int value)
{
  std::vector<uint32_t> write_vals;
  write_vals.push_back(static_cast<uint32_t>(0x00000003));
  write_vals.push_back(static_cast<uint32_t>(reg_num));
  write_vals.push_back(static_cast<uint32_t>(value));
  std::cout<<" Rider_library before writeBlock"<<std::endl;
  wfd->getNode("axi.chan0").writeBlock(write_vals);
  //std::cout<<" Rider_library before dispatch"<<std::endl;
  // wfd->dispatch();
  //std::cout<<" Rider_library after dispatch"<<std::endl;
  uhal::ValVector<uint32_t> read_vals = wfd->getNode("axi.chan0").readBlock(1);
  std::cout<<" Rider_library after read"<<std::endl;


  wfd->dispatch();
  std::cout << "Response code: "<<std::hex<<read_vals.value()[0]<<std::endl;


  }


 void Rider_library::WR_REG(uhal::HwInterface *wfd, std::string channel, int reg_num, int value)
{
  std::vector<uint32_t> write_vals;
  write_vals.push_back(static_cast<uint32_t>(0x00000003));
  write_vals.push_back(static_cast<uint32_t>(reg_num));
  write_vals.push_back(static_cast<uint32_t>(value));
  std::cout<<" Rider_library before writeBlock"<<std::endl;
  wfd->getNode("axi."+channel).writeBlock(write_vals);
  std::cout<<" Rider_library before dispatch"<<std::endl;
  wfd->dispatch();
  std::cout<<" Rider_library after dispatch"<<std::endl;
   uhal::ValVector<uint32_t> read_vals = wfd->getNode("axi."+channel).readBlock(1);
   //uhal::ValWord<uint32_t> read_vals = wfd->getNode("axi."+channel).read();

  std::cout<<" Rider_library after read"<<std::endl;

  wfd->dispatch();
  std::cout << "Response code: "<<std::hex<<read_vals.value()[0]<<std::endl;

}


void Rider_library::RD_REG(uhal::HwInterface *wfd, int reg_num)
{
  std::vector<uint32_t> write_vals;
  write_vals.push_back(0x00000002);
  write_vals.push_back(reg_num);
  //  write_vals.push_back(value);
  wfd->getNode("axi.chan0").writeBlock(write_vals);
  wfd->dispatch();
  uhal::ValVector<uint32_t> read_vals = wfd->getNode("axi.chan0").readBlock(1);
  wfd->dispatch();
  
  uhal::ValVector<uint32_t> read_vals2 = wfd->getNode("axi.chan0").readBlock(1);
  wfd->dispatch();
}

//void Rider_library::Test(uhal:HwInterface *wfd, int something)
//{
  

//}
