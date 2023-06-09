#include "laser_config_handler.h"

namespace g2laser {

const std::string
LaserConfigHandler::odb_standard_mode_ = "1-standard-mode";
const std::string
LaserConfigHandler::odb_sync_pulse_only_mode_ = "2-sync-pulse-only-mode";
const std::string
LaserConfigHandler::odb_alternative_mode_ = "3-alternative-mode";
const std::string
LaserConfigHandler::odb_short_double_pulse_mode_ = "4-short-double-pulse-mode";
const std::string
LaserConfigHandler::odb_long_double_pulse_mode_ = "5-long-double-pulse-mode";
const std::string
LaserConfigHandler::odb_calibration_mode_ = "6-calibration-mode";
const std::string
LaserConfigHandler::odb_flight_sim_mode_ = "7-flight-sim-mode";
const std::string
LaserConfigHandler::odb_manual_mode_ = "8-manual-mode";

const std::string
LaserConfigHandler::str_standard_mode_ = "standard";
const std::string
LaserConfigHandler::str_sync_pulse_only_mode_ = "sync-pulse-only";
const std::string
LaserConfigHandler::str_alternative_mode_ = "alternative";
const std::string
LaserConfigHandler::str_short_double_pulse_mode_ = "short-double-pulse";
const std::string
LaserConfigHandler::str_long_double_pulse_mode_ = "long-double-pulse";
const std::string
LaserConfigHandler::str_calibration_mode_ = "calibration";
const std::string
LaserConfigHandler::str_flight_sim_mode_ = "flight-sim";
const std::string
LaserConfigHandler::str_manual_mode_ = "manual";

const std::string
LaserConfigHandler::be1_dp_short_even_sequencer_file_ =
"AutoGen_Double_Pulse_Short_Even_Fixed.msl";
const std::string
LaserConfigHandler::be1_dp_short_odd_sequencer_file_ =
"AutoGen_Double_Pulse_Short_Odd_Fixed.msl";
const std::string
LaserConfigHandler::be1_dp_long_even_sequencer_file_ = 
"AutoGen_Double_Pulse_Long_Even_Fixed.msl";
const std::string
LaserConfigHandler::be1_dp_long_odd_sequencer_file_ = 
"AutoGen_Double_Pulse_Long_Odd_Fixed.msl";
const std::string
LaserConfigHandler::be1_calibration_scan_sequencer_file_ = 
"AutoGen_Filter_Wheel_Calibration_Scan.msl";

const std::string
LaserConfigHandler::lcb_configuration_file_ = "lcb_configuration.txt";
const std::string
LaserConfigHandler::lcb_load_script_ = "lcb_load.sh";
const std::string
LaserConfigHandler::lcb_init_script_ = "lcb_init.sh";
const std::string
LaserConfigHandler::lcb_user_ = "root";
const std::string
LaserConfigHandler::lcb_host_ = "192.168.30.71";
const std::string
LaserConfigHandler::lcb_path_ = "~/tmp-laser-config";

const std::string
LaserConfigHandler::lh2_load_script_ = "lh2_load.sh";
const std::string
LaserConfigHandler::lh2_init_script_ = "lh2_init.sh";
const std::string
LaserConfigHandler::lh2_user_ = "daq";
const std::string
LaserConfigHandler::lh2_host_ = "g2laser1-priv";
const std::string
LaserConfigHandler::lh2_path_ = "~/tmp-laser-config";


LaserConfigHandler::LaserConfigHandler()
{ 
  ConfigFilePath_ = std::string("/home/debian/LaserCtr_ver2/data/"
                    "OutFRAME_FIFO_Nhit96_Nevt500k_tw2_650us.txt");

  odb_base_path_ = std::string("/Equipment/AMC1325/Laser/");
  laser_base_path_ = std::string("/home/daq/exp.gm5/laser/");
  laser_autogen_path_ = laser_base_path_ + "auto-gen/";

  int idx = 1;
  std::string mode;
  std::stringstream ss;

  mode = std::string("standard");
  ss.str("");
  ss << idx++;
  LaserModeMap_[mode] = mode;
  LaserModeMap_[mode + std::string("-mode")] = mode;
  LaserModeMap_[ss.str()] = mode;
  ss << std::string("-") + mode + std::string("-mode");
  LaserModeMap_[ss.str()] = mode;
  
  mode = std::string("sync-pulse-only");
  ss.str("");
  ss << idx++;
  LaserModeMap_[mode] = mode;
  LaserModeMap_[mode + std::string("-mode")] = mode;
  LaserModeMap_[ss.str()] = mode;
  ss << std::string("-") + mode + std::string("-mode");
  LaserModeMap_[ss.str()] = mode;

  mode = std::string("alternative");
  ss.str("");
  ss << idx++;  
  LaserModeMap_[mode] = mode;
  LaserModeMap_[mode + std::string("-mode")] = mode;
  LaserModeMap_[ss.str()] = mode;
  ss << std::string("-") + mode + std::string("-mode");
  LaserModeMap_[ss.str()] = mode;

  mode = std::string("short-double-pulse");
  ss.str("");
  ss << idx++;
  LaserModeMap_[mode] = mode;
  LaserModeMap_[mode + std::string("-mode")] = mode;
  LaserModeMap_[ss.str()] = mode;
  ss << std::string("-") + mode + std::string("-mode");
  LaserModeMap_[ss.str()] = mode;
  
  mode = std::string("long-double-pulse");
  ss.str("");
  ss << idx++;  
  LaserModeMap_[mode] = mode;
  LaserModeMap_[mode + std::string("-mode")] = mode;
  LaserModeMap_[ss.str()] = mode;
  ss << std::string("-") + mode + std::string("-mode");
  LaserModeMap_[ss.str()] = mode;
  
  mode = std::string("calibration");
  ss.str("");
  ss << idx++;  
  LaserModeMap_[mode] = mode;
  LaserModeMap_[mode + std::string("-mode")] = mode;
  LaserModeMap_[ss.str()] = mode;
  ss << std::string("-") + mode + std::string("-mode");
  LaserModeMap_[ss.str()] = mode;
  
  mode = std::string("flight-sim");
  ss.str("");
  ss << idx++;
  LaserModeMap_[mode] = mode;
  LaserModeMap_[mode + std::string("-mode")] = mode;
  LaserModeMap_[ss.str()] = mode;
  ss << std::string("-") + mode + std::string("-mode");
  LaserModeMap_[ss.str()] = mode;
  
  mode = std::string("manual");
  ss.str("");
  ss << idx++;  
  LaserModeMap_[mode] = mode;
  LaserModeMap_[mode + std::string("-mode")] = mode;
  LaserModeMap_[ss.str()] = mode;
  ss << std::string("-") + mode + std::string("-mode");
  LaserModeMap_[ss.str()] = mode;
};


void LaserConfigHandler::LoadOdb(bool log_msg)
{
  // Make sure the ODB tree has been initialized.
  ValidateOdb();

  // Load non-laser parameters of interest.
  boost::property_tree::ptree pt = LoadOdbAsJson("/Sequencer/State");
  SequencerRunning_ = pt.get<bool>("Running", false);
  SequencerFilePath_ = pt.get<std::string>("Path", laser_autogen_path_);
  std::cout << "SequencerFilePath_: " << SequencerFilePath_ << std::endl;

  // Load the global parameters.
  SetParameters();
  SetParameters("debugging-flags");

  if (boost::iequals(LaserMode_, str_standard_mode_)) {

    SetParameters(odb_standard_mode_);

  } else if (boost::iequals(LaserMode_, str_alternative_mode_)) {

    SetParameters(odb_alternative_mode_);

  } else if (boost::iequals(LaserMode_, str_short_double_pulse_mode_)) {

    SetParameters(odb_short_double_pulse_mode_);

  } else if (boost::iequals(LaserMode_, str_sync_pulse_only_mode_)) {

    SetParameters(odb_sync_pulse_only_mode_);

  } else if (boost::iequals(LaserMode_, str_long_double_pulse_mode_)) {

    SetParameters(odb_long_double_pulse_mode_);

  } else if (boost::iequals(LaserMode_, str_calibration_mode_)) {

    SetParameters(odb_calibration_mode_);

  } else if (boost::iequals(LaserMode_, str_flight_sim_mode_)) {

    SetParameters(odb_flight_sim_mode_);

  } else if (boost::iequals(LaserMode_, str_manual_mode_)) {

    SetParameters(odb_manual_mode_);

  } else {

    cm_msg(MERROR, 
          "LaserConfigHandler", 
          "Invalid laser mode key, defaulting to \"standard\"");

    SetParameters(odb_standard_mode_);
  }

  if (log_msg) {
     cm_msg(MINFO,
           "LaserConfigHandler", 
           "Configuring Laser with %s parameters", 
           LaserMode_.c_str());
  }
}


void LaserConfigHandler::ValidateOdb()
{
  // Data part
  HNDLE hDB;
  std::string odb_key;

  // Get the experiment database handle.
  cm_get_experiment_database(&hDB, NULL);

  // Validate the record.
  odb_key = odb_base_path_ + std::string("/Configuration");
  db_check_record(hDB, 0, odb_key.c_str(), odb_config_record, true);

  odb_key = odb_base_path_ + std::string("/Settings");
  db_check_record(hDB, 0, odb_key.c_str(), odb_settings_record, true);
}


boost::property_tree::ptree 
LaserConfigHandler::LoadOdbAsJson(std::string key)
{
  // Data part
  HNDLE hDB, hkey;
  int size = 0, bytes_written = 0;
  std::stringstream ss;

  char *json_buf = new char[0x8000];
  boost::property_tree::ptree pt;

  // Get the experiment database handle.
  cm_get_experiment_database(&hDB, NULL);
  db_find_key(hDB, 0, key.c_str(), &hkey);

  if (hkey) {
    //   db_copy_json_ls(hDB, hkey, &json_buf, &size, &bytes_written, 1, 1, 1, 0);
    db_copy_json_obsolete(hDB, hkey, &json_buf, &size, &bytes_written, 1, 1, 1);
  } else {

    cm_msg(MERROR, "LaserConfigHandler", 
                   "failed to load \"%s\" from ODB", key.c_str());
    return pt;
  }

  // Store it in a property tree.
  ss << json_buf;
  boost::property_tree::read_json(ss, pt);

  return pt;
}


void LaserConfigHandler::SetParameters(std::string key)
{
  using boost::property_tree::ptree;
  std::string odb_key = odb_base_path_ + std::string("/Configuration");
  fs::path dir = fs::path(odb_key) / fs::path(key);
  ptree pt = LoadOdbAsJson(dir.string().c_str());

  for (ptree::const_iterator it = pt.begin(); it != pt.end(); ++it) {

    // First handle global laser variables
    if (boost::iequals(it->first, "LaserMode")) {

      std::string s = pt.get<std::string>(it->first);
      s.erase(std::remove_if(s.begin(), s.end(), ::isspace), s.end());
      LaserMode_ = LaserModeMap_[s];

    } else if (boost::iequals(it->first, "Prescale")) {

      Prescale_ = pt.get<int>(it->first);

      // Check range of value.
      if (Prescale_ < 1) {
  
       Prescale_ = 1;
       cm_msg(MINFO,
             "LaserConfigHandler", 
             "Prescale set below 1, defaulting to min value of 1");

     } else if (Prescale_ > 31) {

       Prescale_ = 31;
       cm_msg(MINFO,
             "LaserConfigHandler", 
             "Prescale set above 31, defaulting to max value of 31");

     }

    } else if (boost::iequals(it->first, "WriteOnlyMode")) {

      WriteOnlyMode_ = pt.get<bool>(it->first);

    } else if (boost::iequals(it->first, "ResetHardware")) {

      ResetHardware_ = pt.get<bool>(it->first);

    }  else if (boost::iequals(it->first, "FilterWheel1")) {

      FilterWheel_[1] = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "FilterWheel2")) {

      FilterWheel_[2] = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "FilterWheel3")) {

      FilterWheel_[3] = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "FilterWheel4")) {

      FilterWheel_[4] = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "FilterWheel5")) {

      FilterWheel_[5] = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "FilterWheel6")) {

      FilterWheel_[6] = pt.get<int>(it->first);

    }

    // Now handle variables common to all modes.
    if (boost::iequals(it->first, "OutFillBit")) {

      BitMask_ ^= (-pt.get<bool>(it->first) ^ BitMask_) & (1 << 0);

    } else if (boost::iequals(it->first, "Fl2Bit")) {

      BitMask_ ^= (-pt.get<bool>(it->first) ^ BitMask_) & (1 << 1);

    } else if (boost::iequals(it->first, "Fl1Bit")) {

      BitMask_ ^= (-pt.get<bool>(it->first) ^ BitMask_) & (1 << 2);

    } else if (boost::iequals(it->first, "FillBit")) {

      BitMask_ ^= (-pt.get<bool>(it->first) ^ BitMask_) & (1 << 3);

    } else if (boost::iequals(it->first, "EofBit")) {

      BitMask_ ^= (-pt.get<bool>(it->first) ^ BitMask_) & (1 << 4);

    } else if (boost::iequals(it->first, "BofBit")) {

      BitMask_ ^= (-pt.get<bool>(it->first) ^ BitMask_) & (1 << 5);

    } else if (boost::iequals(it->first, "RunBit")) {

      BitMask_ ^= (-pt.get<bool>(it->first) ^ BitMask_) & (1 << 6);

    } else if (boost::iequals(it->first, "DoublePulse")) {

      DoublePulse_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "RESET")) {

      Reset_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "Time2BOF")) {

      Time2BOF_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "Time2T0")) {

      Time2T0_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "Time2EOF")) {

      Time2EOF_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "TimeShiftFill")) {

      TimeShiftFill_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "NShiftRep")) {

      NShiftRep_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "TimeInFillPeriod")) {

      TimeInFillPeriod_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "NPulseInFill")) {

      NPulseInFill_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "NPulseInFill")) {

      NPulseInFill_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "Time2T0OutFill")) {

      Time2T0OutFill_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "TimeOutFillPeriod")) {

      TimeOutFillPeriod_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "NPulseOutFill1")) {

      NPulseOutFill1_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "NPulseOutFill2")) {

      NPulseOutFill2_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "NPulseOutFill3")) {

      NPulseOutFill3_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "NPulseOutFillSim")) {

      NPulseOutFillSim_ = pt.get<int>(it->first);

    } else if (boost::iequals(it->first, "ConfigFilePath")) {

      ConfigFilePath_ = pt.get<std::string>(it->first);

    }

    // Now handle double pulse mode variables.
    if (boost::iequals(it->first, "FinalDelay_ps")) {

      ShortDpFinalDelay_ps_ = pt.get<int>(it->first, -1);

    } else if (boost::iequals(it->first, "DeltaDelay_ps")) {

      ShortDpDeltaDelay_ps_ = pt.get<int>(it->first, -1);

    } if (boost::iequals(it->first, "FinalDelay_us")) {

      LongDpFinalDelay_us_ = pt.get<int>(it->first, -1);

    } else if (boost::iequals(it->first, "DeltaDelay_us")) {

      LongDpDeltaDelay_us_ = pt.get<int>(it->first, -1);

    } else if (boost::iequals(it->first, "BurstLength")) {

      LongDpBurstLength_ = pt.get<int>(it->first, -1);

    } else if (boost::iequals(it->first, "BurstOffset_ns")) {

      LongDpBurstOffset_ns_ = pt.get<int>(it->first, -1);

    } else if (boost::iequals(it->first, "BurstPeriod_ns")) {

      LongDpBurstPeriod_ns_ = pt.get<int>(it->first, -1);

    } else if (boost::iequals(it->first, "FirstFilterWheel")) {

      DpFirstFilterWheel_ = pt.get<int>(it->first, -1);

    } else if (boost::iequals(it->first, "SecondFilterWheel")) {

      DpSecondFilterWheel_ = pt.get<int>(it->first, -1);

    } else if (boost::iequals(it->first, "CalibStartWheelPosition")) {

      CalibStartWheel_ = pt.get<int>(it->first, -1);

    } else if (boost::iequals(it->first, "CalibEndWheelPosition")) {

      CalibEndWheel_ = pt.get<int>(it->first, -1);

    }
  }
}


void LaserConfigHandler::WriteOdbSettings()
{
  // Let the sequencer file handle setting variables if running.
  if (SequencerRunning_) {
    return;
  }

  HNDLE hDB, hkey;
  INT x;
  std::string odb_key = odb_base_path_ + std::string("/Settings");

  // Get the experiment database handle.
  cm_get_experiment_database(&hDB, NULL);
  db_find_key(hDB, 0, odb_key.c_str(), &hkey);

  // Write the prescale rate.
  db_set_value(hDB, hkey, "Prescale", &Prescale_, sizeof(Prescale_), 1, TID_INT);

  // Write the filter wheel settings.
  for (int i = 0; i < 6; ++i) {
    x = FilterWheel_[i+1];
    db_set_value_index(hDB,
                       hkey,
                       "FilterWheel",
                       &x,
                       sizeof(x),
                       i,
                       TID_INT,
                       false);
  }

  // Reset the values of the double pulse parameters.
  // note: these should only be set in MSL files.
  x = -1;  
  db_set_value(hDB, hkey, "ShortDpDelayTime_ps", &x, sizeof(x), 1, TID_INT);
  db_set_value(hDB, hkey, "LongDpDelayTime_us", &x, sizeof(x), 1, TID_INT);
  db_set_value(hDB, hkey, "LongDpBurstLength", &x, sizeof(x), 1, TID_INT);
  db_set_value(hDB, hkey, "LongDpBurstOffset_ns", &x, sizeof(x), 1, TID_INT);
  db_set_value(hDB, hkey, "LongDpBurstPeriod_ns", &x, sizeof(x), 1, TID_INT);
}


void LaserConfigHandler::WriteAutomationWarning(std::ofstream &out, 
                                                std::string comment)
{
  out << comment << " Warning: This file was auto-generated." << std::endl;
  out << comment << " Changes will be overwritten." << std::endl << std::endl;
}


void LaserConfigHandler::WriteLcbConfigFile()
{
  fs::path base = fs::path(laser_autogen_path_);
  fs::path dir =  base / fs::path(lcb_configuration_file_);
  std::ofstream out(dir.string().c_str());
  std::stringstream ss;

  ss.str("");
  for (int i = 0; i < BitMaskSize_; ++i) {
    ss << (bool)((BitMask_ >> i) & 0x1) << " ";
  }
  ss << "//7_BIT_MASK:|OutFill|Fl2|Fl1|Fill|EoF|BoF|RUN|";
  out << ss.str() << std::endl;

  // The double pulse register is used for prescale on non-DP runs.
  int val = 0;
  if (!SequencerRunning_) {

    if (Prescale_ == 1) {

      // No need to enable prescaling to run all events.
      val = 0x0;

    } else {

      // Enable prescaling
      val = 0x1 << 2;
      val |= Prescale_ << 3; 
    }

  } else {

    val = DoublePulse_;
  }

  ss.str("");
  ss << "0x00 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss <<  val << " //DoublePulse" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x01 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)Reset_ << " //RESET_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x03 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)Time2BOF_ << " //Time2_BOF_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x04 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)Time2T0_ << " //Time2_T0_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x05 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)Time2EOF_ << " //Time2_EOF_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x06 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)TimeShiftFill_ << " //TimeShiftFill_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x07 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)NShiftRep_ << " //NShiftRep_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x08 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)TimeInFillPeriod_ << " //TimeInFillPeriod_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x09 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)NPulseInFill_ << " //NPulseInFill_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x0a 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)Time2T0OutFill_ << " //Time2T0OutFill_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x0b 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)TimeOutFillPeriod_ << " //TimeOutFillPeriod_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x0c 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)NPulseOutFill1_ << " //NPulseOutFill1_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x0d 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)NPulseOutFill2_ << " //NPulseOutFill2_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x0e 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)NPulseOutFill3_ << " //NPulseOutFill3_" << std::endl;
  out << ss.str();

  ss.str("");
  ss << "0x0f 0x" << std::setfill('0') << std::setw(2) << std::hex;
  ss << (int)NPulseOutFillSim_ << " //NPulseOutFillSim_" << std::endl;
  out << ss.str();

  out << ConfigFilePath_ << std::endl;

  out.close();
}


void LaserConfigHandler::WriteLcbInitScript()
{
  fs::path dir = fs::path(laser_autogen_path_) / fs::path(lcb_init_script_);
  std::ofstream out(dir.string().c_str());
 
  out << "#!/bin/sh" << std::endl << std::endl;

  WriteAutomationWarning(out, "#");

  out << "# Make sure there is no previous instance running." << std::endl
      << "killall RunLaserControl" << std::endl << std::endl
      << "jobID=`pidof RunLaserControl`" << std::endl
      << "echo \" LaserRun PID \" $jobID" << std::endl
      << "if [  $jobID ];" << std::endl
      << "then" << std::endl
      << "\tkill -QUIT $jobID" << std::endl
      << "\techo \"QUIT_signal to proc \"$jobID" << std::endl
      << "else" << std::endl
      << "\techo \" NO RunLaserControl process is running \"" << std::endl
      << "fi" << std::endl << std::endl;

  out << "# Load the laser config file." << std::endl;
  out << "cd ~/daq/software/control/spi/LaserCtr_ver2" << std::endl;

  dir = fs::path(lcb_path_) / fs::path(lcb_configuration_file_);
  out << "cp " << dir.string() << " data/LaserCConfig.txt\n\n";
  out << "./RunLaserControl > /dev/null &" << std::endl << std::endl;

  out << "# Run laser control board init." << std::endl;
  out << "sleep 1" << std::endl;
  out << "./LaserPulseStart 1" << std::endl;

  out.close();
}


void LaserConfigHandler::WriteLcbLoadScript()
{
  fs::path dir = fs::path(laser_autogen_path_) / fs::path(lcb_load_script_);
  std::ofstream out(dir.string().c_str());
 
  out << "#!/bin/sh" << std::endl << std::endl;

  WriteAutomationWarning(out, "#");

  out << "# Copy the LCB config files." << std::endl;
  out << "echo \" Copy the LCB config files \"" << std::endl;
  dir = fs::path(laser_autogen_path_) / fs::path(lcb_configuration_file_);
  out << "scp " << dir.string() << " " << lcb_user_ << "@" << lcb_host_;
  dir = fs::path(lcb_path_) / fs::path(lcb_configuration_file_);
  out << ":" << dir.string() << std::endl << std::endl;

  out << "# Copy the LCB init script." << std::endl;
  out << "echo \" Copy the LCB init script \"" << std::endl;
  dir = fs::path(laser_autogen_path_) / fs::path(lcb_init_script_);
  out << "scp " << dir.string() << " " << lcb_user_ << "@" << lcb_host_;
  dir = fs::path(lcb_path_) / fs::path(lcb_init_script_);
  out << ":" << dir.string() << std::endl << std::endl;

  out << "# Run the LCB init script." << std::endl;
  out << "echo \" Run the LCB init script \"" << std::endl;
  out << "ssh " << " " << lcb_user_ << "@" << lcb_host_;
  dir = fs::path(lcb_path_) / fs::path(lcb_init_script_);
  out << " \"source " << dir.string() << "\"" << std::endl << std::endl;

  out << "echo \" check if RunLaserControl is running \"" << std::endl;
  out << "ssh  root@192.168.30.71 \"ps aux | grep RunLaserControl \"";
  out << std::endl;
  out.close();
}

void LaserConfigHandler::RunLcbInitCommands()
{
  if (WriteOnlyMode_ || SequencerRunning_) {

    return;

  } else {
    fs::path dir = fs::path(laser_autogen_path_);
    std::string cmd("source ");
    cmd += (dir / fs::path(lcb_load_script_)).string();
    cmd += std::string(" >& ");
    cmd += (dir / fs::path("log/lcb_init.log")).string();
    std::cout << cmd<<std::endl;
    int ret = std::system(cmd.c_str());
    std::cout <<"lcb load command return "<< ret<<std::endl;
  }
}


void LaserConfigHandler::WriteLh2InitScript()
{
  fs::path dir = fs::path(laser_autogen_path_) / fs::path(lh2_init_script_);
  std::ofstream out(dir.string().c_str());
 
  out << "#!/bin/sh" << std::endl << std::endl;

  WriteAutomationWarning(out, "#");

  if (ResetHardware_) {
    out << "# Commands to reset the DG645." << std::endl;
    out << "source ~/dg645/dg645_OFF.sh" << std::endl << std::endl;

    out << "# Commands to set the filter wheel." << std::endl;
    std::map<int, int>::const_iterator it;
    for (it = FilterWheel_.begin(); it != FilterWheel_.end(); ++it) {
      out << "source ~/filterwheel/movewheel.sh ";
      out << it->first << " " << it->second << std::endl;
    }
    out << std::endl;

  } else {    

    out << "# Debugging flag set to leave mirror state as is."
	<< std::endl << std::endl;

    // Don't expect this to be used with standard modes.
    if (LaserMode_ == LaserModeMap_["1"] ||
	LaserMode_ == LaserModeMap_["2"] ||
	LaserMode_ == LaserModeMap_["3"]) {
      cm_msg(MINFO, "LaserConfigHandler", 
	     "Running Laser in %s mode without hardware reset. "
"This should only be done by experts.  If you are not an expert,"
" reset the ODB flag %s/debugging-flags/ResetHardware to true.",
	     LaserMode_.c_str(), odb_base_path_.c_str());
    }
  }
    
  out.close();
}


void LaserConfigHandler::WriteLh2LoadScript()
{
  fs::path dir = fs::path(laser_autogen_path_) / fs::path(lh2_load_script_);
  std::ofstream out(dir.string().c_str());
 
  out << "#!/bin/sh" << std::endl << std::endl;

  WriteAutomationWarning(out, "#");

  if (ResetHardware_) {
    out << "# Set all mirrors down." << std::endl
    << "source "<<laser_base_path_<<"mov_mirror.sh a0" 
    << std::endl << std::endl;
  } else {
    out << "# Debugging flag set to leave mirror state as is."
    << std::endl << std::endl;
  }

  out << "# Copy the LH2 init script." << std::endl;
  dir = fs::path(laser_autogen_path_) / fs::path(lh2_init_script_);
  out << "scp " << dir.string() << " " << lh2_user_ << "@" << lh2_host_;
  dir = fs::path(lh2_path_) / fs::path(lh2_init_script_);
  out << ":" << dir.string() << std::endl << std::endl;

  out << "# Run the LH2 init script." << std::endl;
  out << "ssh " << lh2_user_ << "@" << lh2_host_;
  dir = fs::path(lh2_path_) / fs::path(lh2_init_script_);
  out << " \"source " << dir.string() << "\"" << std::endl << std::endl;

  out.close();
}


void LaserConfigHandler::RunLh2InitCommands()
{
  if (WriteOnlyMode_ || SequencerRunning_) {

    return;

  } else {
    fs::path dir = fs::path(laser_autogen_path_);
    std::string cmd("source ");
    cmd += (dir / fs::path(lh2_load_script_)).string();
    cmd += std::string(" >& ");
    cmd += (dir / fs::path("log/lh2_init.log")).string();
    std::cout << cmd<<std::endl;
    int ret = std::system(cmd.c_str());
    std::cout <<"lh2 load command return "<< ret<<std::endl;
  }
}


void LaserConfigHandler::WriteBe1ShortDoublePulseMsl()
{
  // Only write if we are in short-double-pulse mode.
  if (!boost::iequals(LaserMode_, str_short_double_pulse_mode_)) {
    return;
  }

  // Load the short double-pulse params.
  SetParameters(odb_short_double_pulse_mode_);

  // Set the filepath and open the file stream.
  fs::path base = fs::path(SequencerFilePath_);
  fs::path dir1 = base / fs::path(be1_dp_short_even_sequencer_file_);
  WriteDpShort(dir1.string(), true);
 
  fs::path dir2 = base / fs::path(be1_dp_short_odd_sequencer_file_);
  WriteDpShort(dir2.string(), false);

  // Reset to the current mode.
  LoadOdb(false);
}


void LaserConfigHandler::WriteDpShort(std::string path, bool even_laser_fixed)
{
  std::ofstream out(path.c_str());

  WriteAutomationWarning(out, "#");

  out << "COMMENT " << "\"Run a seqeuence of laser runs\"" << std::endl;
  out << "RUNDESCRIPTION " << "\"Double Pulse Short Time Constant\"";
  out << std::endl << std::endl;

  out << "# Turn off run-start auto-elog." << std::endl;
  out << "ODBSET \"Logger/Run Start Elog Entry\" n";
  out << std::endl << std::endl;

  out << "# Set filter wheels" << std::endl;

  if (even_laser_fixed) {

    out << "# EVEN (first pulse) position ";
    out << DpFirstFilterWheel_ << std::endl;

    out << "# ODD (second pulse) position ";
    out << DpSecondFilterWheel_ << std::endl;

    out << "LOOP wheel, 2, 4, 6" << std::endl;
    out << "     SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, $wheel, ";
    out << DpFirstFilterWheel_ << std::endl;    
    out << "ENDLOOP" << std::endl;

    out << "LOOP wheelminusone, 1, 3, 5" << std::endl;
    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/FilterWheel[$wheelminusone]\" ";
    out << DpFirstFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl;

    out << "LOOP wheel, 1, 3, 5" << std::endl;
    out << "     SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, $wheel, ";
    out << DpSecondFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl;

    out << "LOOP wheelminusone, 0, 2, 4" << std::endl;
    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/FilterWheel[$wheelminusone]\" ";
    out << DpSecondFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl << std::endl;;

    out << "# Set the mirrors for EVEN sequence." << std:: endl;
    out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh o1" << std::endl;
    out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh e0" << std::endl << std::endl;

  } else {

    out << "# ODD (first pulse) position ";
    out << DpFirstFilterWheel_ << std::endl;

    out << "# EVEN (second pulse) position ";
    out << DpSecondFilterWheel_ << std::endl;

    out << "LOOP wheel, 1, 3, 5" << std::endl;
    out << "     SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, $wheel, ";
    out << DpFirstFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl;

    out << "LOOP wheelminusone, 0, 2, 4" << std::endl;
    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/FilterWheel[$wheelminusone]\" ";
    out << DpFirstFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl;

    out << "LOOP wheel, 2, 4, 6" << std::endl;
    out << "     SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, $wheel, ";
    out << DpSecondFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl;

    out << "LOOP wheelminusone, 1, 3, 5" << std::endl;
    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/FilterWheel[$wheelminusone]\" ";
    out << DpSecondFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl << std::endl;;

    out << "# Set the mirrors for ODD sequence." << std:: endl;
    out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh e1" << std::endl;
    out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh o0" << std::endl << std::endl;
  }

  out << "# Setup laser." << std::endl;
  out << "SCRIPT "<<laser_base_path_<<"LoadShortTimeConfig.sh";
  out << std::endl << std::endl;

  out << "# Start to loop on delays." << std::endl;
  out << "######################################################\n";
  out << "######################################################\n";
  out << std::endl;

  std::vector< std::vector<int> > delays;

  int idx = 0;
  int N = ShortDpFinalDelay_ps_ / ShortDpDeltaDelay_ps_ + 1;
  for (int i = 0; i < N; ++i) {
    if ((i % 10 == 0) && (i < N - 1)) {
      idx = i / 10;
      std::vector<int> v;
      delays.push_back(v);
    }
    delays[idx].push_back(ShortDpDeltaDelay_ps_ * i);
  }

  idx = 1;
  for (unsigned int i = 0; i < delays.size(); ++i) {
    out << "# Loop " << idx++ << std::endl;
    out << "# Delay expressed in ps." << std::endl;
    out << "LOOP delay, ";

    for (unsigned int j = 0; j < delays[i].size(); ++j) {
      out << delays[i][j] << ", ";
    }
    out << std::endl;

    if (even_laser_fixed) {

      out << "     CAT comment, \"Double Pulse Even=0 Odd=\", $delay";
      out << std::endl;

      out << "     ODBSET \"/Experiment/Edit on start/Comment\", ";
      out << "$comment" << std::endl;
 
      out << "     ODBSET \"/Experiment/Edit on start/Quality\", ";
      out << "\"S\"" << std::endl;
 
      out << "     SCRIPT "<<laser_base_path_<<"dg645_SetDPEven.sh, $delay" << std::endl;

    } else {

      out << "     CAT comment, \"Double Pulse Odd=0 Even=\", $delay";
      out << std::endl;

      out << "     ODBSET \"/Experiment/Edit on start/Comment\", ";
      out << "$comment" << std::endl;

      out << "     ODBSET \"/Experiment/Edit on start/Quality\", ";
      out << "\"S\"" << std::endl;

      out << "     SCRIPT "<<laser_base_path_<<"dg645_SetDPOdd.sh, $delay" << std::endl;
    }

    // Carry out run transitions.
    out << "     WAIT Seconds 10" << std::endl;
    out << "     TRANSITION START" << std::endl;
    
    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/ShortDpDelayTime_ps\" $delay" << std::endl;
    
    out << "     WAIT Seconds 60" << std::endl;
    out << "     TRANSITION STOP" << std::endl;

    out << "ENDLOOP" << std::endl << std::endl;
  }

  // Final setting lines.
  out << "# Reset all wheels to default position." << std::endl;
  for (int i = 1; i < 7; ++i) {
    out << "SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, " << i << ", ";
    out << FilterWheel_[i] << std::endl;
  }
  out << std::endl;
  
  // Reset the mirrors.
  out << "# Set the mirrors back to default position." << std:: endl;
  out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh a0" << std::endl << std::endl;

  // Reset ODB params.
  out << "ODBSET \"/Experiment/Edit on start/Comment\", ";
  out << "\"Enter comment\"" << std::endl;
  out << "ODBSET \"/Experiment/Edit on start/Quality\", ";
  out << "\"T\"" << std::endl << std::endl;

  out << "# Turn back on run-start auto-elog." << std::endl;
  out << "ODBSET \"Logger/Run Start Elog Entry\" y" << std::endl;

  out.close();
}


void LaserConfigHandler::WriteBe1LongDoublePulseMsl()
{
  // Only write if we are in long-double-pulse mode.
  if (!boost::iequals(LaserMode_, str_long_double_pulse_mode_)) {
    return;
  }

  // Load the short double-pulse params.
  SetParameters(odb_long_double_pulse_mode_);
  
  // Set the filepath and open the file stream.
  fs::path base = fs::path(SequencerFilePath_);
  fs::path dir1 = base / fs::path(be1_dp_long_even_sequencer_file_);
  WriteDpLong(dir1.string(), true);
 
  fs::path dir2 = base / fs::path(be1_dp_long_odd_sequencer_file_);
  WriteDpLong(dir2.string(), false);

  LoadOdb(false);
}


void LaserConfigHandler::WriteDpLong(std::string path, bool even_laser_fixed)
{
  std::ofstream out(path.c_str());
  WriteAutomationWarning(out, "#");

  out << "COMMENT " << "\"Run a sequence of laser runs\"" << std::endl;
  out << "RUNDESCRIPTION " << "\"Double Pulse Long Time Constant\"";
  out << std::endl << std::endl;

  out << "# Turn off run-start auto-elog." << std::endl;
  out << "ODBSET \"Logger/Run Start Elog Entry\" n";
  out << std::endl << std::endl;

  out << "# Set filter wheels" << std::endl;

  if (even_laser_fixed) {

    out << "# EVEN (first pulse) position ";
    out << DpFirstFilterWheel_ << std::endl;

    out << "# ODD (second pulse) position ";
    out << DpSecondFilterWheel_ << std::endl;

    out << "LOOP wheel, 2, 4, 6" << std::endl;
    out << "     SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, $wheel, ";
    out << DpFirstFilterWheel_ << std::endl;
    out << "ENDLOOP" <<std::endl;

    out << "LOOP wheelminusone, 1, 3, 5" <<std::endl;
    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/FilterWheel[$wheelminusone]\" ";
    out << DpFirstFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl;

    out << "LOOP wheel, 1, 3, 5" << std::endl;
    out << "     SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, $wheel, ";
    out << DpSecondFilterWheel_ << std::endl;
    out << "ENDLOOP" <<std::endl;

    out << "LOOP wheelminusone, 0, 2, 4" <<std::endl;
    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/FilterWheel[$wheelminusone]\" ";
    out << DpSecondFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl << std::endl;

    out << "# Set the mirrors for EVEN sequence." << std:: endl;
    out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh o1" << std::endl;
    out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh e0" << std::endl << std::endl;

    out << "# Setup burst mode on EVEN lasers:" << std::endl;
    out << "# First argument is number of pulses" << std::endl;
    out << "# Second argument is distance between pulses in nsec" << std::endl;
    out << "SCRIPT "<<laser_base_path_<<"dg645_SetBurstEven.sh ";
    out << LongDpBurstLength_ << " " << LongDpBurstPeriod_ns_;
    out << std::endl << std::endl;

    out << "# setup laser" << std::endl;
    out << "SCRIPT "<<laser_base_path_<<"LoadEVENConfig.sh";
    out << std::endl << std::endl;

  } else {

    out << "# ODD (first pulse) position ";
    out << DpFirstFilterWheel_ << std::endl;

    out << "# EVEN (second pulse) position ";
    out << DpSecondFilterWheel_ << std::endl;

    out << "LOOP wheel, 1, 3, 5" << std::endl;
    out << "     SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, $wheel, ";
    out <<  DpFirstFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl;

    out << "LOOP wheelminusone, 0, 2, 4" <<std::endl;
    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/FilterWheel[$wheelminusone]\" ";
    out << DpFirstFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl;

    out << "LOOP wheel, 2, 4, 6" << std::endl;
    out << "     SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, $wheel, ";
    out <<  DpSecondFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl << std::endl;

    out << "LOOP wheelminusone, 1, 3, 5" <<std::endl;
    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/FilterWheel[$wheelminusone]\" ";
    out << DpSecondFilterWheel_ << std::endl;
    out << "ENDLOOP" << std::endl << std::endl;

    out << "# Set the mirrors for ODD sequence." << std:: endl;
    out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh e1" << std::endl;
    out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh o0" << std::endl << std::endl;

    out << "# Setup burst mode on ODD lasers:" << std::endl;
    out << "# First argument is number of pulses" << std::endl;
    out << "# Second argument is distance between pulses in nsec" << std::endl;
    out << "SCRIPT "<<laser_base_path_<<"dg645_SetBurstOdd.sh ";
    out << LongDpBurstLength_ << " " << LongDpBurstPeriod_ns_;
    out << std::endl << std::endl;

    out << "# setup laser" << std::endl;
    out << "SCRIPT "<<laser_base_path_<<"LoadODDConfig.sh";
    out << std::endl << std::endl;
  }

  out << "# Start to loop on delays." << std::endl;
  out << "# Each time adds an offset for the duration of the pulse train.\n";
  out << "######################################################\n";
  out << "######################################################\n";
  out << std::endl;

  std::vector< std::vector<int> > delays;
  int idx = 0;
  int offset_us = LongDpBurstOffset_ns_ / 1000;
  offset_us += (LongDpBurstLength_ + 1) * LongDpBurstPeriod_ns_ / 1000;

  int N = LongDpFinalDelay_us_ / LongDpDeltaDelay_us_ + 1;
  for (int i = 0; i < N; ++i) {
    
    if ((i % 10 == 0) && (i < N - 1)) {
      idx = i / 10;
      std::vector<int> v;
      delays.push_back(v);
    }
    
    delays[idx].push_back(LongDpDeltaDelay_us_ * i + offset_us);
  }

  // Write the initial time offset to a variable.
  out << "SET offset, " << -offset_us << std::endl << std::endl;

  idx = 1;
  for (unsigned int i = 0; i < delays.size(); ++i) {
    out << "# Loop " << idx++ << std::endl;
    out << "# Delay expressed in us." << std::endl;
    out << "LOOP delay, ";

    for (unsigned int j = 0; j < delays[i].size(); ++j)
    {
      out << (delays[i][j]) << ", ";
    }
    out << std::endl;

    if (even_laser_fixed) {
      out << "     CAT comment, \"Double Pulse Even=0 Odd=\", $delay";
      out << std::endl;
    } else {
      out << "     CAT comment, \"Double Pulse Odd=0 Even=\", $delay";
      out << std::endl;
    }

    out << "     ODBSET \"/Experiment/Edit on start/Comment\", ";
    out << "$comment" << std::endl;

    out << "     ODBSET \"/Experiment/Edit on start/Quality\", ";
    out << "\"L\"" << std::endl;

    out << "     SCRIPT "<<laser_base_path_<<"WriteReg8.sh, $delay" << std::endl;
    out << "     WAIT Seconds 10" << std::endl;
    out << "     TRANSITION START" << std::endl;

    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/LongDpDelayTime_us\", $delay" << std::endl;

    out << "     ODBINC \"" << odb_base_path_;
    out << "/Settings/LongDpDelayTime_us\", $offset" << std::endl;

    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/LongDpBurstLength\" ";
    out << LongDpBurstLength_ << std::endl;

    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/LongDpBurstOffset_ns\" ";
    out << LongDpBurstOffset_ns_ << std::endl;

    out << "     ODBSET \"" << odb_base_path_;
    out << "/Settings/LongDpBurstPeriod_ns\" ";
    out << LongDpBurstPeriod_ns_ << std::endl;

    out << "     WAIT Seconds 60" << std::endl;
    out << "     TRANSITION STOP" << std::endl;
    out << "ENDLOOP" << std::endl << std::endl;
  }

  out << "# Shutdown delay generator." << std::endl;
  out << "SCRIPT "<<laser_base_path_<<"dg645_OFF.sh";
  out << std::endl << std::endl;

  // Final setting lines.
  out << "# Reset all wheels to default position." << std::endl;
  for (int i = 1; i < 7; ++i) {
    out << "SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, " << i << ", ";
    out << FilterWheel_[i] << std::endl;
  }
  out << std::endl;

  // Reset the mirrors.
  out << "# Set the mirrors back to default position." << std:: endl;
  out << "SCRIPT "<<laser_base_path_<<"mov_mirror.sh a0" << std::endl << std::endl;

  out << "ODBSET \"/Experiment/Edit on start/Comment\", ";
  out << "\"Enter comment\"" << std::endl;
  out << "ODBSET \"/Experiment/Edit on start/Quality\", \"T\"";
  out << std::endl << std::endl;

  out << "# Turn back on run-start auto-elog." << std::endl;
  out << "ODBSET \"Logger/Run Start Elog Entry\" y";
  out << std::endl;

  out.close();
}


void LaserConfigHandler::WriteBe1CalibrationScan()
{
  // Only write if we are in alternative gain mode.
  if (!boost::iequals(LaserMode_, str_calibration_mode_)) {
    return;
  }

  fs::path base = fs::path(SequencerFilePath_);
  fs::path path = base / fs::path(be1_calibration_scan_sequencer_file_);
  std::ofstream out(path.string().c_str());

  WriteAutomationWarning(out, "#");
  out << "COMMENT \"Run a sequence of laser runs\"" << std::endl;
  out << "RUNDESCRIPTION \"Calibration run\"" << std::endl << std::endl;

  out << "LOOP angle, ";
  for (int i=CalibStartWheel_;i<=CalibEndWheel_;i++){
    out << i << ", ";
  }
  out << "7 " ;//always ends in default position
  out << std::endl;

  out << "     CAT comment, \"Automated run, setting \", $angle" << std::endl;

  out << "     ODBSET \"/Experiment/Edit on start/Comment\", ";
  out << "$comment" << std::endl;

  out << "     ODBSET \"/Experiment/Edit on start/Quality\", \"C\"";
  out << std::endl;

  out << "     LOOP wheel, 1, 2, 3, 4, 5, 6" << std::endl;
  out << "          SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, $wheel, $angle";
  out << std::endl;
  out << "     ENDLOOP" << std::endl << std::endl;

  out << "     LOOP wheelminusone, 0, 1, 2, 3, 4, 5" <<std::endl;
  out << "          ODBSET \"" << odb_base_path_;
  out << "/Settings/FilterWheel[$wheelminusone]\" $angle" << std::endl;
  out << "     ENDLOOP" << std::endl;

  out << "     WAIT Seconds 20" << std::endl;
  out << "     TRANSITION START" << std::endl;
  out << "     WAIT Seconds 120" << std::endl;
  out << "     TRANSITION STOP" << std::endl;
  out << "ENDLOOP" << std::endl << std::endl;

  // Final setting lines.
  out << "# Reset all wheels to default position." << std::endl;
  for (int i = 1; i < 7; ++i) {
    out << "SCRIPT "<<laser_base_path_<<"movelaserwheel.sh, " << i << ", ";
    out << FilterWheel_[i] << std::endl;
  }

  out << std::endl;
  out << "ODBSET \"/Experiment/Edit on start/Comment\", ";
  out << "\"Enter comment\"" << std::endl;

  out << "ODBSET \"/Experiment/Edit on start/Quality\", \"T\"";
  out << std::endl;

  out.close();
}

}
