#ifndef G2_DAQ_LASER_CONFIG_HANDLER
#define G2_DAQ_LASER_CONFIG_HANDLER

/*===========================================================================*\

    author: Matthias W. Smith
    email:  matthias.smith@pi.infn.it

    about:  The class handles configuration of the laser calibration 
            systems for g-2.  All dependent variables are read from the 
            experiment's MIDAS ODB.  The class handles writing the 
            relevant text files and scripts for initializing the system.  

\*===========================================================================*/

// c++ standard library includes
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <iostream> //Added due to error

// External includes
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include "midas.h"


namespace g2laser {

namespace fs = boost::filesystem;

const char *const odb_settings_record = "\
Prescale = INT : -1                             \n\
FilterWheel = INT[6] :                          \n\
[0] -1                                          \n\
[1] -1                                          \n\
[2] -1                                          \n\
[3] -1                                          \n\
[4] -1                                          \n\
[5] -1                                          \n\
ShortDpDelayTime_ps = INT : -1                  \n\
LongDpDelayTime_us = INT : -1                   \n\
LongDpBurstLength = INT : -1                    \n\
LongDpBurstPeriod_ns = INT : -1                 \n\
LongDpBurstOffset_ns = INT : -1                 \n\
";

const char *const odb_config_record = "\
LaserMode = STRING : [32] standard\n\
Prescale = INT : 1                              \n\
FilterWheel1 = INT : 3                          \n\
FilterWheel2 = INT : 3                          \n\
FilterWheel3 = INT : 3                          \n\
FilterWheel4 = INT : 3                          \n\
FilterWheel5 = INT : 3                          \n\
FilterWheel6 = INT : 3                          \n\
                                                \n\
[1-standard-mode]                               \n\
OutFillBit = BOOL : y                           \n\
Fl2Bit = BOOL : n                               \n\
Fl1Bit = BOOL : n                               \n\
FillBit = BOOL : y                              \n\
EofBit = BOOL : n                               \n\
BofBit = BOOL : y                               \n\
RunBit = BOOL : y                               \n\
DoublePulse = INT : 0x00                        \n\
RESET = BYTE : 0x13                             \n\
Time2BOF = BYTE : 0x20                          \n\
Time2T0 = BYTE : 0x1e                           \n\
Time2EOF = BYTE : 0x46                          \n\
TimeShiftFill = INT :0x32                       \n\
NShiftRep = BYTE : 0x14                         \n\
TimeInFillPeriod = BYTE : 0x64                  \n\
NPulseInFill = BYTE : 0x04                      \n\
Time2T0OutFill = BYTE : 0x00                    \n\
TimeOutFillPeriod = BYTE : 0x14                 \n\
NPulseOutFill1 = BYTE : 0x04                    \n\
NPulseOutFill2 = BYTE : 0x0f                    \n\
NPulseOutFill3 = BYTE : 0x0f                    \n\
NPulseOutFillSim = BYTE : 0x0f                  \n\
                                                \n\
[2-sync-pulse-only-mode]                        \n\
OutFillBit = BOOL : y                           \n\
Fl2Bit = BOOL : n                               \n\
Fl1Bit = BOOL : n                               \n\
FillBit = BOOL : n                              \n\
EofBit = BOOL : n                               \n\
BofBit = BOOL : y                               \n\
RunBit = BOOL : y                               \n\
DoublePulse = INT : 0x00                        \n\
RESET = BYTE : 0x13                             \n\
Time2BOF = BYTE : 0x07                          \n\
Time2T0 = BYTE : 0x0f                           \n\
Time2EOF = BYTE : 0x46                          \n\
TimeShiftFill = INT :0x32                       \n\
NShiftRep = BYTE : 0x13                         \n\
TimeInFillPeriod = BYTE : 0x64                  \n\
NPulseInFill = BYTE : 0x01                      \n\
Time2T0OutFill = BYTE : 0x00                    \n\
TimeOutFillPeriod = BYTE : 0x14                 \n\
NPulseOutFill1 = BYTE : 0x04                    \n\
NPulseOutFill2 = BYTE : 0x0f                    \n\
NPulseOutFill3 = BYTE : 0x0f                    \n\
NPulseOutFillSim = BYTE : 0x0f                  \n\
                                                \n\
[3-alternative-mode]                            \n\
OutFillBit = BOOL : y                           \n\
Fl2Bit = BOOL : y                               \n\
Fl1Bit = BOOL : y                               \n\
FillBit = BOOL : y                              \n\
EofBit = BOOL : n                               \n\
BofBit = BOOL : n                               \n\
RunBit = BOOL : n                               \n\
DoublePulse = INT : 0x00                        \n\
RESET = BYTE : 0x13                             \n\
Time2BOF = BYTE : 0x03                          \n\
Time2T0 = BYTE : 0x0a                           \n\
Time2EOF = BYTE : 0x0f                          \n\
TimeShiftFill = INT :0x7c                       \n\
NShiftRep = BYTE : 0x13                         \n\
TimeInFillPeriod = BYTE : 0x00                  \n\
NPulseInFill = BYTE : 0x01                      \n\
Time2T0OutFill = BYTE : 0xff                    \n\
TimeOutFillPeriod = BYTE : 0xff                 \n\
NPulseOutFill1 = BYTE : 0x0f                    \n\
NPulseOutFill2 = BYTE : 0x0f                    \n\
NPulseOutFill3 = BYTE : 0x0f                    \n\
NPulseOutFillSim = BYTE : 0x0f                  \n\
ConfigFilePath = STRING : [256] /home/debian/LaserCtr_ver2/\
data/OutFRAME_FIFO_Nhit96_Nevt500k_tw2_650us.txt\n\
                                                \n\
[4-short-double-pulse-mode]                     \n\
OutFillBit = BOOL : n                           \n\
Fl2Bit = BOOL : n                               \n\
Fl1Bit = BOOL : n                               \n\
FillBit = BOOL : n                              \n\
EofBit = BOOL : n                               \n\
BofBit = BOOL : n                               \n\
RunBit = BOOL : y                               \n\
DoublePulse = INT : 0x00                        \n\
RESET = BYTE : 0x13                             \n\
Time2BOF = BYTE : 0x0a                          \n\
Time2T0 = BYTE : 0x0f                           \n\
Time2EOF = BYTE : 0x20                          \n\
TimeShiftFill = INT :0x7c                       \n\
NShiftRep = BYTE : 0x13                         \n\
TimeInFillPeriod = BYTE : 0x64                  \n\
NPulseInFill = BYTE : 0x01                      \n\
Time2T0OutFill = BYTE : 0xff                    \n\
TimeOutFillPeriod = BYTE : 0xff                 \n\
NPulseOutFill1 = BYTE : 0x2f                    \n\
NPulseOutFill2 = BYTE : 0x2f                    \n\
NPulseOutFill3 = BYTE : 0x2f                    \n\
NPulseOutFillSim = BYTE : 0x4f                  \n\
FinalDelay_ps = INT : 50000                     \n\
DeltaDelay_ps = INT : 5000                      \n\
FirstFilterWheel = INT : 6                      \n\
SecondFilterWheel = INT : 4                     \n\
                                                \n\
[5-long-double-pulse-mode]                      \n\
OutFillBit = BOOL : n                           \n\
Fl2Bit = BOOL : n                               \n\
Fl1Bit = BOOL : n                               \n\
FillBit = BOOL : n                              \n\
EofBit = BOOL : n                               \n\
BofBit = BOOL : n                               \n\
RunBit = BOOL : y                               \n\
DoublePulse = INT : 0x00                        \n\
RESET = BYTE : 0x13                             \n\
Time2BOF = BYTE : 0x0a                          \n\
Time2T0 = BYTE : 0x0f                           \n\
Time2EOF = BYTE : 0x20                          \n\
TimeShiftFill = INT :0x7c                       \n\
NShiftRep = BYTE : 0x13                         \n\
TimeInFillPeriod = BYTE : 0x64                  \n\
NPulseInFill = BYTE : 0x01                      \n\
Time2T0OutFill = BYTE : 0xff                    \n\
TimeOutFillPeriod = BYTE : 0xff                 \n\
NPulseOutFill1 = BYTE : 0x2f                    \n\
NPulseOutFill2 = BYTE : 0x2f                    \n\
NPulseOutFill3 = BYTE : 0x2f                    \n\
NPulseOutFillSim = BYTE : 0x4f                  \n\
BurstLength = INT : 10                          \n\
BurstPeriod_ns = INT : 10                       \n\
BurstOffset_ns = INT : 1000                     \n\
FinalDelay_us = INT : 300                       \n\
DeltaDelay_us = INT : 20                        \n\
FirstFilterWheel = INT : 6                      \n\
SecondFilterWheel = INT : 4                     \n\
                                                \n\
[6-calibration-mode]                            \n\
OutFillBit = BOOL : n                           \n\
Fl2Bit = BOOL : n                               \n\
Fl1Bit = BOOL : n                               \n\
FillBit = BOOL : y                              \n\
EofBit = BOOL : n                               \n\
BofBit = BOOL : y                               \n\
RunBit = BOOL : y                               \n\
DoublePulse = INT : 0x00                        \n\
RESET = BYTE : 0x13                             \n\
Time2BOF = BYTE : 0x07                          \n\
Time2T0 = BYTE : 0x32                           \n\
Time2EOF = BYTE : 0x46                          \n\
TimeShiftFill = INT :0x32                       \n\
NShiftRep = BYTE : 0x00                         \n\
TimeInFillPeriod = BYTE : 0x64                  \n\
NPulseInFill = BYTE : 0x06                      \n\
Time2T0OutFill = BYTE : 0xff                    \n\
TimeOutFillPeriod = BYTE : 0xff                 \n\
NPulseOutFill1 = BYTE : 0x00                    \n\
NPulseOutFill2 = BYTE : 0x2f                    \n\
NPulseOutFill3 = BYTE : 0x2f                    \n\
NPulseOutFillSim = BYTE : 0x4f                  \n\
CalibStartWheelPosition = INT : 1               \n\
CalibEndWheelPosition   = INT : 12              \n\
                                                \n\
[7-flight-sim-mode]                             \n\
OutFillBit = BOOL : y                           \n\
Fl2Bit = BOOL : n                               \n\
Fl1Bit = BOOL : y                               \n\
FillBit = BOOL : n                              \n\
EofBit = BOOL : n                               \n\
BofBit = BOOL : y                               \n\
RunBit = BOOL : y                               \n\
DoublePulse = INT : 0x00                        \n\
RESET = BYTE : 0x19                             \n\
Time2BOF = BYTE : 0x0a                          \n\
Time2T0 = BYTE : 0x0f                           \n\
Time2EOF = BYTE : 0x46                          \n\
TimeShiftFill = INT :0x7c                       \n\
NShiftRep = BYTE : 0x13                         \n\
TimeInFillPeriod = BYTE : 0x00                  \n\
NPulseInFill = BYTE : 0x01                      \n\
Time2T0OutFill = BYTE : 0x00                    \n\
TimeOutFillPeriod = BYTE : 0x14                 \n\
NPulseOutFill1 = BYTE : 0x0f                    \n\
NPulseOutFill2 = BYTE : 0x0f                    \n\
NPulseOutFill3 = BYTE : 0x0f                    \n\
NPulseOutFill4 = BYTE : 0x0f                    \n\
NPulseOutFillSim = BYTE : 0x0f                  \n\
ConfigFilePath = STRING : [256] /home/debian/LaserCtr_ver2/\
data/OutFRAME_FIFO_Nhit96_Nevt500k_tw2_650us.txt\n\
                                                \n\
[8-manual-mode]                                 \n\
OutFillBit = BOOL : y                           \n\
Fl2Bit = BOOL : y                               \n\
Fl1Bit = BOOL : y                               \n\
FillBit = BOOL : y                              \n\
EofBit = BOOL : y                               \n\
BofBit = BOOL : y                               \n\
RunBit = BOOL : y                               \n\
DoublePulse = INT : 0x00                        \n\
RESET = BYTE : 0x13                             \n\
Time2BOF = BYTE : 0x0a                          \n\
Time2T0 = BYTE : 0x0f                           \n\
Time2EOF = BYTE : 0x20                          \n\
TimeShiftFill = INT :0x7c                       \n\
NShiftRep = BYTE : 0x13                         \n\
TimeInFillPeriod = BYTE : 0x00                  \n\
NPulseInFill = BYTE : 0x01                      \n\
Time2T0OutFill = BYTE : 0xff                    \n\
TimeOutFillPeriod = BYTE : 0xff                 \n\
NPulseOutFill1 = BYTE : 0x0f                    \n\
NPulseOutFill2 = BYTE : 0x0f                    \n\
NPulseOutFill3 = BYTE : 0x0f                    \n\
NPulseOutFill4 = BYTE : 0x0f                    \n\
NPulseOutFillSim = BYTE : 0x0f                  \n\
                                                \n\
[debugging-flags]                               \n\
WriteOnlyMode = BOOL : false                    \n\
ResetHardware = BOOL : true                     \n\
";

class LaserConfigHandler {

public:

  // ctor
  LaserConfigHandler();

  // dtor
  ~LaserConfigHandler() {};

  inline void SetOdbPath(std::string path) {
    odb_base_path_ = path;
  }

  inline void SetLaserPath(std::string path) {
    laser_base_path_ = path;
    laser_autogen_path_ = path + "auto-gen/";
  }

  inline void BeginOfRun() {
    WriteFiles();
    RunScripts();
    WriteOdbSettings();
    CreateLockFiles();
  };

  inline void EndOfRun() {
    DeleteLockFiles();
  };

  inline void WriteFiles() {
    LoadOdb();
    WriteBe1ShortDoublePulseMsl();
    WriteBe1LongDoublePulseMsl();
    WriteBe1CalibrationScan();
    WriteLcbConfigFile();
    WriteLcbInitScript();
    WriteLcbLoadScript();
    WriteLh2InitScript();
    WriteLh2LoadScript();
  };

  inline void RunScripts() {
    RunLcbInitCommands();
    RunLh2InitCommands();
  };

private:

  // ODB Strings
  std::string odb_base_path_;
  std::string laser_base_path_;
  std::string laser_autogen_path_;

  const static std::string odb_standard_mode_;
  const static std::string odb_sync_pulse_only_mode_;
  const static std::string odb_alternative_mode_;
  const static std::string odb_short_double_pulse_mode_;
  const static std::string odb_long_double_pulse_mode_;
  const static std::string odb_calibration_mode_;
  const static std::string odb_flight_sim_mode_;
  const static std::string odb_manual_mode_;

  const static std::string str_standard_mode_;
  const static std::string str_sync_pulse_only_mode_;
  const static std::string str_alternative_mode_;  
  const static std::string str_short_double_pulse_mode_;
  const static std::string str_long_double_pulse_mode_;
  const static std::string str_calibration_mode_;
  const static std::string str_flight_sim_mode_;
  const static std::string str_manual_mode_;

  const static std::string be1_dp_short_even_sequencer_file_;
  const static std::string be1_dp_short_odd_sequencer_file_;
  const static std::string be1_dp_long_even_sequencer_file_;
  const static std::string be1_dp_long_odd_sequencer_file_;
  const static std::string be1_calibration_scan_sequencer_file_; 

  const static std::string lcb_configuration_file_;
  const static std::string lcb_default_config_file_path_;
  const static std::string lcb_load_script_;
  const static std::string lcb_init_script_;
  const static std::string lcb_user_;
  const static std::string lcb_host_;
  const static std::string lcb_path_;

  const static std::string lh2_load_script_;
  const static std::string lh2_init_script_;
  const static std::string lh2_user_;
  const static std::string lh2_host_;
  const static std::string lh2_path_;

  // Global laser paremeters
  std::string LaserMode_;
  int Prescale_;
  bool WriteOnlyMode_;
  bool ResetHardware_;
  bool SequencerRunning_;
  std::string SequencerFilePath_;
  std::map<int, int> FilterWheel_;
  std::map<std::string, std::string> LaserModeMap_;

  // LCB Configuration Parameters
  uint8_t BitMask_;
  const static int BitMaskSize_ = 7;
  uint8_t DoublePulse_;
  uint8_t Reset_;
  uint8_t Time2BOF_;
  uint8_t Time2T0_;
  uint8_t Time2EOF_;
  uint8_t TimeShiftFill_;
  uint8_t NShiftRep_;
  uint8_t TimeInFillPeriod_;
  uint8_t NPulseInFill_;
  uint8_t Time2T0OutFill_;
  uint8_t TimeOutFillPeriod_;
  uint8_t NPulseOutFill1_;
  uint8_t NPulseOutFill2_;
  uint8_t NPulseOutFill3_;
  uint8_t NPulseOutFillSim_;
  std::string ConfigFilePath_;

  // Double Pulse (DP) parameters
  int ShortDpFinalDelay_ps_;
  int ShortDpDeltaDelay_ps_;
  int LongDpFinalDelay_us_;
  int LongDpDeltaDelay_us_;
  int DpFirstFilterWheel_;
  int DpSecondFilterWheel_;
  int CalibStartWheel_;
  int CalibEndWheel_;
  int LongDpBurstLength_;
  int LongDpBurstOffset_ns_;
  int LongDpBurstPeriod_ns_;

  void LoadOdb(bool log_msg=true);
  void ValidateOdb();
  void SetParameters(std::string key="");
  void WriteOdbSettings();

  boost::property_tree::ptree LoadOdbAsJson(std::string key);
  void WriteAutomationWarning(std::ofstream &out, std::string comment="//");

  void CreateLockFiles() {};
  void DeleteLockFiles() {};

  // g2be1 write functions.
  void WriteBe1ShortDoublePulseMsl();
  void WriteBe1LongDoublePulseMsl();
  void WriteBe1CalibrationScan();
  void WriteDpShort(std::string path, bool fix_even_lasers);
  void WriteDpLong(std::string path, bool fix_even_lasers);

  // Functions concerning the Laser Control Board (LCB).
  void WriteLcbConfigFile();
  void WriteLcbInitScript();
  void WriteLcbLoadScript();
  void RunLcbInitCommands();

  // Functions concerning the mini-pc/laserhut2 machine.
  void WriteLh2InitScript();
  void WriteLh2LoadScript();
  void RunLh2InitCommands();
};
}

#endif
