/* amc13_odb.h --- 
 * 
 * Filename:        amc13_odb.h
 * Description:     AMC13 reaout ODB interface
 * Author:          Tim Gorringe & Wes Gohn
 * Maintainer: 
 * Created:         Tue Jun 26 17:16:03 CDT 2014
 * Version:         $Id$
 * Last-Updated: Tue Oct  9 16:20:01 2018 (-0400)
 *           By: Wes Gohn
 *     Update #: 254
 * URL: 
 * Keywords: 
 * Compatibility: 
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

#ifndef amc13_odb_h
#define amc13_odb_h

#include "tqmethods_max.h"

extern bool UseHotlink;

/**
 *  structure map ODB 
 */

typedef struct s_amc13_settings_odb {
  BOOL  sync;                     // synchronous = 1, asynchronous = 0
  BOOL  simulate_data;            // use simulated data (do not connect to AMC13)
  INT   gpu_dev_id;               // device id of GPU
  char  gpu_dev_name_prefix[256]; // device name prefix of GPU
  BOOL  send_to_event_builder;
  BOOL  lossless_compression;
  BOOL  bankbybank_lossless_compression;
  BOOL  store_raw;                // y/n for storing raw TCP data
  DWORD prescale_raw;             // pre-scale factor for storing raw TCP data
  DWORD prescale_offset_raw;      // pre-scale factor for storing raw TCP data
  char  mch_ip_addr[16];
  int   ccc_amc_slot;
  char  ccc_fmc_loc[16];
  int   ccc_fmc_sfp;
} AMC13_SETTINGS_ODB;

#define AMC13_SETTINGS_ODB_STR "\
[.]\n\
Sync = BOOL : 0\n\
Use AMC13 Simulator = BOOL : 0\n\
GPU Device ID = INT : __gpu__\n\
GPU Device Name Prefix = STRING : [256] tesla\n\
Send to Event Builder = BOOL : 1\n\
FE Lossless Compression = BOOL : 0\n\
FEBankByBankLosslessCompression = BOOL : 0\n\
Raw Data Store = BOOL : 0\n\
Raw Data Prescale = DWORD : 1\n\
Raw Data Prescale Offset = DWORD : 1\n\
MCH IP Address = STRING : [16] 192.168.__subnet__.15\n\
CCC: FC7 Slot Number (1-12) = INT : __slot__\n\
CCC: FMC Location (top, bottom) = STRING : [16] __fmc__\n\
CCC: FMC SFP Number (1-8) = INT : __sfp__\n\
"

typedef struct s_amc13_monitors_odb {
  float tcp_buf_peak;
  float gpu_buf_peak;
  int   tcp_fill_number;          // tcp fill number
  int   gpu_fill_number;          // gput fill number
  int   midas_fill_number;        // midas fill number
  int   operation_state;          // operation state, for end of run state check
  int   tcp_thread_status;        // status of the tcp thread
  int   gpu_thread_status;	  // status of the gpu thread
} AMC13_MONITORS_ODB;

#define AMC13_MONITORS_ODB_STR "\
[.]\n\
TCP Buffer Peak = FLOAT : 0\n\
GPU Buffer Peak = FLOAT : 0\n\
TCP Fill Number = INT : 0\n\
GPU Fill Number = INT : 0\n\
Midas Fill Number = INT : 0\n\
Operation State = INT : 0\n\
TCP Thread Status = INT : 0\n\
GPU Thread Status = INT : 0\n\
"

typedef struct s_tq_parameters_odb {
  char  TQ_bankprefix[16];             ///< first character (+ string terminator \0) of TQ Midasbanks, i.e. CPnn, CQnn, CTnn, CHnn
  INT   TQ_maptype;                  ///< specify type of rider / module channel map, e.g. enabled channels, calo array
  BOOL  TQ_on;                      ///< T/Q-method processing in GPU on = 1, T/Q-method processing in GPU off = 0
  INT   gpu_waveform_length;        ///< GPU waveform length for calo processing
  INT   gpu_waveform_firstsample;        ///< GPU waveform length for calo processing
  INT   gpu_waveform_lastsample;        ///< GPU waveform length for calo processing
  INT   gpu_n_segments_x;        ///< GPU number of segments in x-direction
  INT   gpu_n_segments_y;        ///< GPU number of segments in y-direction
  INT   gpu_island_presamples;        ///< GPU island presamples for calo processing
  INT   gpu_island_postsamples;        ///< GPU island postsamples for calo processing
  INT   island_option;           // Select island setting, 0 periodic, 1 adc sum
  INT   T_threshold;             // Tmethod threshold value
  BOOL  T_threshold_sign;        // Tmethod threshold sign
  BOOL  use_channel_thresholds;  // use channel thresholds
  INT   pedestal_option;         // Select island setting, 0 global ODB pedestal, 1 calculated segment-by-segment pedestal
  INT   global_pedestal;         // global pedestal value
  INT   calosum_decimation_factor;        ///< waveform decimation factor for calo processing
  BOOL  store_hist;                 ///< y/n for storing histogrammed TCP data
  BOOL  subtract_ped;                 ///< y/n for subtracting fill-by-fill pedestal from storing histogrammed TCP data
  INT   time_divide_hist;        ///< y/n for storing histogrammed TCP data
  INT   first_sample_in_hist;     // lower edge of range of samples in histo
  INT   last_sample_in_hist;     // upper edge of range of samples in histo
  INT   rebin_intervals_in_hist;     // number of differnt rebinning intervals
  INT   rebin_increment_in_hist;     // rebinning increase per rebinning interval
  INT   flush_hist;              ///< pre-scale factor for storing histogrammed TCP data
  INT   flush_offset_hist;              ///< pre-scale factor for storing histogrammed TCP data
  BOOL  separate_sequence_hist; ///< separate histograms for each fill in fill sequence
  INT   fit_islands;             ///whether to run fitter, keeping old name for now -- ATF
  INT   fit_prescale_factor;      /// allow reducing of fitted data and fitting processing
  INT   min_fit_time;            // minimum sample to try fitting on
  INT   fit_threshold;            //threshold for fitting
  INT   CTAG_threshold;          //threshold for CTAG
  INT   CTAG_time_cut;            //time cut for CTAG
  BOOL  TMask_window;             //turn on masking window for T-method
  INT   mask_min;                 //first sample number to mask
  INT   mask_max;                  //last sample number to mask
  INT   mask_prescale;             //prescale of mask
  BOOL  save_full_calo;            //save all xtals in each island
  BOOL  save_truncated_calo;            //save all xtals in each island
  BOOL  save_xtal_border;            //save all border xtals for trigger xtal
  INT   fill_type;                 //Fill type to which this TQ method is applied
  INT   T_prescale;               //prescale factor at which to execute this TQ method
} TQ_PARAMETERS_ODB;

#define TQ_PARAMETERS_ODB_STR "\
[.]\n\
TQ Midas Bank prefix = STRING : [16] __prefix__\n\
TQ map type (0-array,1-enabled) = INT : 0\n\
GPU T,Q,P bank processing = BOOL : __gpu1__\n\
waveform length = INT : 560000\n\
GPU first sample = INT : 1\n\
GPU last sample = INT : 560000\n\
array x-segments = INT : 9\n\
array y-segments = INT : 6\n\
island presamples = INT : 8\n\
island postsamples = INT : 16\n\
thres type 0-dt,1-sm,2-eg,3-wgt = INT : 2\n\
global threshold value = INT : -300\n\
+ve(-ve) global thres xing Y(N) = BOOL : 0\n\
use individual thresholds Y(N) = BOOL : 0\n\
pedestal type 0-glbl,1-fbyf = INT : 1\n\
global pedestal value = INT : 0\n\
Calo sum time decimation = INT : 32\n\
GPU H bank processing  = BOOL : __gpu2__\n\
histo data subtract pedestal  = BOOL : 0\n\
histo data time decimation = INT : 16\n\
histo data first sample index = INT : 1\n\
histo data last sample index = INT : 560000\n\
histo data rebin intervals = INT : 1\n\
histo data rebin multiplier = INT : 1\n\
histo data flush period = INT : 100\n\
histo data flush offset = INT : 22\n\
histo data by seq num Y(N) = BOOL : 0\n\
run gpu fitter (1 yes, 0 no) = INT : 1\n\
fitting prescale factor = INT : 1\n\
minimum fit time = INT : 9\n\
fit threshold = INT : 1000\n\
CTAG threshold = INT : 1000\n\
CTAG time cut = INT : 40000\n\
T-method Mask = BOOL : 0\n\
T-method Mask first sample = INT : 0\n\
T-method Mask last sample = INT : 0\n\
T-method Mask Prescale = INT : 1\n\
Save full calo = BOOL : 0\n\
Save truncated calo = BOOL : 0\n\
Save xtal border = BOOL : 0\n\
Fill type = INT : 1\n\
T-method prescale = INT : 1\n\
"

typedef struct s_amc13_link_odb {
  BYTE  enabled;       // don't readout the module if false
  char  source_ip[16]; // host_name
  DWORD source_port;
} AMC13_LINK_ODB;

#define AMC13_LINK_ODB_STR "\
[.]\n\
Enabled = BYTE : 1\n\
AMC13 SFP IP Address = STRING : [16] 192.168.4__subnet__.2\n\
AMC13 SFP Port Number = DWORD : 4660\n\
"

typedef struct s_amc13_amc13_odb {
  DWORD header_size;
  DWORD amc_block_size;
  DWORD tail_size;
  DWORD t1_fw_version;
  DWORD t2_fw_version;
  char  addrTab1[65];
  char  addrTab2[65];
  BOOL  enableSoftwareTrigger; // do "wv 0x0 0x400" in rpc_g2_eof = 1, don't do "wv 0x0 0x400" in rpc_g2_eof = 0 
} AMC13_AMC13_ODB;
  
#define AMC13_AMC13_ODB_STR "\
[.]\n\
Header Size (bytes) = DWORD : 4096\n\
AMC Block Size (bytes) = DWORD : 32768\n\
Tail Size (bytes) = DWORD : 8\n\
T1 Firmware Version Required = DWORD : 33087\n\
T2 Firmware Version Required = DWORD : 46\n\
T1 Address Table Location = STRING : [65] $GM2DAQ_DIR/address_tables/AMC13XG_T1.xml\n\
T2 Address Table Location = STRING : [65] $GM2DAQ_DIR/address_tables/AMC13XG_T2.xml\n\
Fake Data Mode Enabled = BOOL : 0\n\
"

// RiderXX/ChannelXX settings
typedef struct s_rider_odb_channel {
  BOOL enabled;
  BOOL fe_config_enabled;
  char FPGA_firmware_version[16];
  int input_signal_offset;
  char input_type[16];
} RIDER_ODB_CHANNEL;
    
#define RIDER_ODB_CHANNEL_STR "\
[.]\n\
Enabled = BOOL : 1\n\
Frontend Configuration Enabled = BOOL : 1\n\
FPGA Firmware Version Required = STRING : [16] 2.3.1\n\
Input Signal Offset = INT : 65535\n\
Input Type = STRING : [16] XTAL\n\
"

// RiderXX/Board settings
typedef struct s_rider_odb_board {
  BOOL  rider_enabled;
  BOOL  fe_config_enabled;
  char  addr_table_file[64];
  char  firmware_version[16];
  char  digitizer_freq[8];
  BOOL  fp_clock_enabled;
  char  fp_clock_freq[8];
  char  ADC_endianness[8];
  DWORD trig_delay;
  BOOL  asyncmode_enabled;
  DWORD asyncmode_wvfm_length;
  DWORD asyncmode_wvfm_presamples;
  BOOL  trig1_enabled;
  DWORD trig1__wvfm_count;
  DWORD trig1__wvfm_length;
  DWORD trig1__wvfm_gap;
  BOOL  trig2_enabled;
  DWORD trig2__wvfm_count;
  DWORD trig2__wvfm_length;
  DWORD trig2__wvfm_gap;
  BOOL  trig3_enabled;
  DWORD trig3__wvfm_count;
  DWORD trig3__wvfm_length;
  DWORD trig3__wvfm_gap;
  DWORD error_thres_corruption;
  DWORD error_thres_unkwnTTC;
  DWORD error_thres_DDR3ovflw;
} RIDER_ODB_BOARD;

#define RIDER_ODB_BOARD_STR "\
Enabled = BOOL : 1\n\
Frontend Configuration Enabled = BOOL : 1\n\
Address Table Location = STRING : [64] $GM2DAQ_DIR/address_tables/WFD5.xml\n\
FPGA Firmware Version Required = STRING : [16] 2.3.5\n\
Digitization Frequency (MHz) = STRING : [8] 800\n\
Front Panel Clock Enabled = BOOL : 11\n\
Front Panel Clock Frequency = STRING : [8] ttc\n\
ADC Data Endianness = STRING : [8] little\n\
Trigger Delay (ns) = DWORD : 0\n\
Async Mode: Enabled = BOOL : 0\n\
Async Mode: Waveform Length = DWORD : 80\n\
Async Mode: Waveform Presamples = DWORD : 4\n\
Trigger Type 1: Enabled = BOOL : 1\n\
Trigger Type 1: Waveform Count = DWORD : 1\n\
Trigger Type 1: Waveform Length = DWORD : 560000\n\
Trigger Type 1: Waveform Gap = DWORD : 16\n\
Trigger Type 2: Enabled = BOOL : 1\n\
Trigger Type 2: Waveform Count = DWORD : 4\n\
Trigger Type 2: Waveform Length = DWORD : 800\n\
Trigger Type 2: Waveform Gap = DWORD : 400\n\
Trigger Type 3: Enabled = BOOL : 1\n\
Trigger Type 3: Waveform Count = DWORD : 1\n\
Trigger Type 3: Waveform Length = DWORD : 800\n\
Trigger Type 3: Waveform Gap = DWORD : 16\n\
Error Threshold: Corrupt Data = DWORD : 10\n\
Error Threshold: Unknown TTC = DWORD : 10\n\
Error Threshold: DDR3 Overflow = DWORD : 7549747\n\
"

// FC7-XX/Common settings
typedef struct s_fc7_odb_common {
  BOOL  enabled;
  BOOL  fe_config_enabled;
  char  addr_table_file[64];
  char  board_type[8];
  char  firmware_version[16];
  DWORD thres_ttc_sbit_error;
  DWORD thres_ttc_mbit_error;
  DWORD thres_tts_lock;
  DWORD thres_tts_valid;
} FC7_ODB_COMMON;

#define FC7_ODB_COMMON_STR "\
Enabled = BOOL : 1\n\
Frontend Configuration Enabled = BOOL : 1\n\
Address Table Location = STRING : [64] $GM2DAQ_DIR/address_tables/FC7.xml\n\
Board (encoder,fanout,trigger) = STRING : [8] fanout\n\
FPGA Firmware Version Required = STRING : [16] 3.2.1\n\
Threshold: TTC Single-Bit Error = DWORD : 0\n\
Threshold: TTC Multi-Bit Error = DWORD : 0\n\
TTS Lock: Total Length = DWORD : 1000000\n\
TTS Lock: Valid Length = DWORD : 25\n\
"

// FC7-XX/Encoder settings
typedef struct s_fc7_odb_encoder {
  BOOL  wfd_async_enabled;
  DWORD thres_overflow;
  DWORD thres_cycle_start_gap;
  char  input_trig_sel[8];
  DWORD post_reset_delay_count;
  DWORD post_reset_delay_timestamp;
} FC7_ODB_ENCODER;

#define FC7_ODB_ENCODER_STR "\
WFD5 Async Mode Enabled = BOOL : 1\n\
Threshold: Client Overflow = DWORD : 16\n\
Threshold: Cycle Start Gap = DWORD : 1000000\n\
Input Trigger Select = STRING : [8] left\n\
Post-Reset Delay: Trigger Count = DWORD : 1250\n\
Post-Reset Delay: Timestamp = DWORD : 1250\n\
"
// FC7-XX/Trigger settings
typedef struct s_fc7_odb_trigger {
   DWORD laser_prescale;
   DWORD toggle_8_16_fills;
   DWORD fallover_threshold;
   DWORD safety_discharge_time;
   DWORD kicker_charge_trigger_deadtime;
   DWORD quad_t9_delay;
   DWORD quad_t9_width;
   BOOL  quad_t9_enabled;
   BOOL  quad_a6_enabled;
} FC7_ODB_TRIGGER;

#define FC7_ODB_TRIGGER_STR "\
In-fill Laser Prescale = DWORD : 10\n\
Fills per supercyc (0->8 1->16) = DWORD : 1\n\
Internal fallover thresh (msec) = DWORD : 7000000000\n\
Safety discharge time (msec) = DWORD : 15\n\
Kicker chg trig deadtime (msec) = DWORD : 9\n\
Quad T9 trig delay (25 ns tiks) = DWORD : 1\n\
Quad t9 trig width (25 ns tiks) = DWORD : 4\n\
Quad T9 trigger enabled = BOOL : 1\n\
Quad A6 trigger enabled = BOOL : 1\n\
"


// FC7-XX/Left Trigger Output settings
typedef struct s_fc7_odb_left_otrig {
  DWORD short_pulse_width;
  DWORD long_pulse_width;
  char  width_00[16];
  DWORD delay_00;
  char  width_01[16];
  DWORD delay_01;
  char  width_02[16];
  DWORD delay_02;
  char  width_03[16];
  DWORD delay_03;
  char  width_04[16];
  DWORD delay_04;
  char  width_05[16];
  DWORD delay_05;
  char  width_06[16];
  DWORD delay_06;
  char  width_07[16];
  DWORD delay_07;
  char  width_08[16];
  DWORD delay_08;
  char  width_09[16];
  DWORD delay_09;
  char  width_10[16];
  DWORD delay_10;
  char  width_11[16];
  DWORD delay_11;
  char  width_12[16];
  DWORD delay_12;
  char  width_13[16];
  DWORD delay_13;
  char  width_14[16];
  DWORD delay_14;
  char  width_15[16];
  DWORD delay_15;
  char  width_16[16];
  DWORD delay_16;
  char  width_17[16];
  DWORD delay_17;
  char  width_18[16];
  DWORD delay_18;
  char  width_19[16];
  DWORD delay_19;
  char  width_20[16];
  DWORD delay_20;
  char  width_21[16];
  DWORD delay_21;
  char  width_22[16];
  DWORD delay_22;
  char  width_23[16];
  DWORD delay_23;
  char  width_24[16];
  DWORD delay_24;
  char  width_25[16];
  DWORD delay_25;
  char  width_26[16];
  DWORD delay_26;
  char  width_27[16];
  DWORD delay_27;
  char  width_28[16];
  DWORD delay_28;
  char  width_29[16];
  DWORD delay_29;
  char  width_30[16];
  DWORD delay_30;
  char  width_31[16];
  DWORD delay_31;
} FC7_ODB_LEFT_OTRIG;

#define FC7_ODB_LEFT_OTRIG_STR "\
Pulse Width: Short (ns) = DWORD : 50\n\
Pulse Width: Long (ns) = DWORD : 100\n\
Trigger Type 00: Width = STRING : [16] disabled\n\
Trigger Type 00: Delay (ns) = DWORD : 0\n\
Trigger Type 01: Width = STRING : [16] short\n\
Trigger Type 01: Delay (ns) = DWORD : 0\n\
Trigger Type 02: Width = STRING : [16] long\n\
Trigger Type 02: Delay (ns) = DWORD : 0\n\
Trigger Type 03: Width = STRING : [16] disabled\n\
Trigger Type 03: Delay (ns) = DWORD : 0\n\
Trigger Type 04: Width = STRING : [16] disabled\n\
Trigger Type 04: Delay (ns) = DWORD : 0\n\
Trigger Type 05: Width = STRING : [16] disabled\n\
Trigger Type 05: Delay (ns) = DWORD : 0\n\
Trigger Type 06: Width = STRING : [16] disabled\n\
Trigger Type 06: Delay (ns) = DWORD : 0\n\
Trigger Type 07: Width = STRING : [16] disabled\n\
Trigger Type 07: Delay (ns) = DWORD : 0\n\
Trigger Type 08: Width = STRING : [16] disabled\n\
Trigger Type 08: Delay (ns) = DWORD : 0\n\
Trigger Type 09: Width = STRING : [16] disabled\n\
Trigger Type 09: Delay (ns) = DWORD : 0\n\
Trigger Type 10: Width = STRING : [16] disabled\n\
Trigger Type 10: Delay (ns) = DWORD : 0\n\
Trigger Type 11: Width = STRING : [16] disabled\n\
Trigger Type 11: Delay (ns) = DWORD : 0\n\
Trigger Type 12: Width = STRING : [16] disabled\n\
Trigger Type 12: Delay (ns) = DWORD : 0\n\
Trigger Type 13: Width = STRING : [16] disabled\n\
Trigger Type 13: Delay (ns) = DWORD : 0\n\
Trigger Type 14: Width = STRING : [16] disabled\n\
Trigger Type 14: Delay (ns) = DWORD : 0\n\
Trigger Type 15: Width = STRING : [16] disabled\n\
Trigger Type 15: Delay (ns) = DWORD : 0\n\
Trigger Type 16: Width = STRING : [16] disabled\n\
Trigger Type 16: Delay (ns) = DWORD : 0\n\
Trigger Type 17: Width = STRING : [16] disabled\n\
Trigger Type 17: Delay (ns) = DWORD : 0\n\
Trigger Type 18: Width = STRING : [16] disabled\n\
Trigger Type 18: Delay (ns) = DWORD : 0\n\
Trigger Type 19: Width = STRING : [16] disabled\n\
Trigger Type 19: Delay (ns) = DWORD : 0\n\
Trigger Type 20: Width = STRING : [16] disabled\n\
Trigger Type 20: Delay (ns) = DWORD : 0\n\
Trigger Type 21: Width = STRING : [16] disabled\n\
Trigger Type 21: Delay (ns) = DWORD : 0\n\
Trigger Type 22: Width = STRING : [16] disabled\n\
Trigger Type 22: Delay (ns) = DWORD : 0\n\
Trigger Type 23: Width = STRING : [16] disabled\n\
Trigger Type 23: Delay (ns) = DWORD : 0\n\
Trigger Type 24: Width = STRING : [16] disabled\n\
Trigger Type 24: Delay (ns) = DWORD : 0\n\
Trigger Type 25: Width = STRING : [16] disabled\n\
Trigger Type 25: Delay (ns) = DWORD : 0\n\
Trigger Type 26: Width = STRING : [16] disabled\n\
Trigger Type 26: Delay (ns) = DWORD : 0\n\
Trigger Type 27: Width = STRING : [16] disabled\n\
Trigger Type 27: Delay (ns) = DWORD : 0\n\
Trigger Type 28: Width = STRING : [16] disabled\n\
Trigger Type 28: Delay (ns) = DWORD : 0\n\
Trigger Type 29: Width = STRING : [16] disabled\n\
Trigger Type 29: Delay (ns) = DWORD : 0\n\
Trigger Type 30: Width = STRING : [16] disabled\n\
Trigger Type 30: Delay (ns) = DWORD : 0\n\
Trigger Type 31: Width = STRING : [16] disabled\n\
Trigger Type 31: Delay (ns) = DWORD : 0\n\
"

// FC7-XX/Right Trigger Output settings
typedef struct s_fc7_odb_right_otrig {
  DWORD short_pulse_width;
  DWORD long_pulse_width;
  char  width_00[16];
  DWORD delay_00;
  char  width_01[16];
  DWORD delay_01;
  char  width_02[16];
  DWORD delay_02;
  char  width_03[16];
  DWORD delay_03;
  char  width_04[16];
  DWORD delay_04;
  char  width_05[16];
  DWORD delay_05;
  char  width_06[16];
  DWORD delay_06;
  char  width_07[16];
  DWORD delay_07;
  char  width_08[16];
  DWORD delay_08;
  char  width_09[16];
  DWORD delay_09;
  char  width_10[16];
  DWORD delay_10;
  char  width_11[16];
  DWORD delay_11;
  char  width_12[16];
  DWORD delay_12;
  char  width_13[16];
  DWORD delay_13;
  char  width_14[16];
  DWORD delay_14;
  char  width_15[16];
  DWORD delay_15;
  char  width_16[16];
  DWORD delay_16;
  char  width_17[16];
  DWORD delay_17;
  char  width_18[16];
  DWORD delay_18;
  char  width_19[16];
  DWORD delay_19;
  char  width_20[16];
  DWORD delay_20;
  char  width_21[16];
  DWORD delay_21;
  char  width_22[16];
  DWORD delay_22;
  char  width_23[16];
  DWORD delay_23;
  char  width_24[16];
  DWORD delay_24;
  char  width_25[16];
  DWORD delay_25;
  char  width_26[16];
  DWORD delay_26;
  char  width_27[16];
  DWORD delay_27;
  char  width_28[16];
  DWORD delay_28;
  char  width_29[16];
  DWORD delay_29;
  char  width_30[16];
  DWORD delay_30;
  char  width_31[16];
  DWORD delay_31;
} FC7_ODB_RIGHT_OTRIG;

#define FC7_ODB_RIGHT_OTRIG_STR "\
Pulse Width: Short (ns) = DWORD : 50\n\
Pulse Width: Long (ns) = DWORD : 100\n\
Trigger Type 00: Width = STRING : [16] short\n\
Trigger Type 00: Delay (ns) = DWORD : 0\n\
Trigger Type 01: Width = STRING : [16] short\n\
Trigger Type 01: Delay (ns) = DWORD : 0\n\
Trigger Type 02: Width = STRING : [16] short\n\
Trigger Type 02: Delay (ns) = DWORD : 0\n\
Trigger Type 03: Width = STRING : [16] short\n\
Trigger Type 03: Delay (ns) = DWORD : 0\n\
Trigger Type 04: Width = STRING : [16] short\n\
Trigger Type 04: Delay (ns) = DWORD : 0\n\
Trigger Type 05: Width = STRING : [16] short\n\
Trigger Type 05: Delay (ns) = DWORD : 0\n\
Trigger Type 06: Width = STRING : [16] short\n\
Trigger Type 06: Delay (ns) = DWORD : 0\n\
Trigger Type 07: Width = STRING : [16] short\n\
Trigger Type 07: Delay (ns) = DWORD : 0\n\
Trigger Type 08: Width = STRING : [16] short\n\
Trigger Type 08: Delay (ns) = DWORD : 0\n\
Trigger Type 09: Width = STRING : [16] short\n\
Trigger Type 09: Delay (ns) = DWORD : 0\n\
Trigger Type 10: Width = STRING : [16] short\n\
Trigger Type 10: Delay (ns) = DWORD : 0\n\
Trigger Type 11: Width = STRING : [16] short\n\
Trigger Type 11: Delay (ns) = DWORD : 0\n\
Trigger Type 12: Width = STRING : [16] short\n\
Trigger Type 12: Delay (ns) = DWORD : 0\n\
Trigger Type 13: Width = STRING : [16] short\n\
Trigger Type 13: Delay (ns) = DWORD : 0\n\
Trigger Type 14: Width = STRING : [16] short\n\
Trigger Type 14: Delay (ns) = DWORD : 0\n\
Trigger Type 15: Width = STRING : [16] short\n\
Trigger Type 15: Delay (ns) = DWORD : 0\n\
Trigger Type 16: Width = STRING : [16] short\n\
Trigger Type 16: Delay (ns) = DWORD : 0\n\
Trigger Type 17: Width = STRING : [16] short\n\
Trigger Type 17: Delay (ns) = DWORD : 0\n\
Trigger Type 18: Width = STRING : [16] short\n\
Trigger Type 18: Delay (ns) = DWORD : 0\n\
Trigger Type 19: Width = STRING : [16] short\n\
Trigger Type 19: Delay (ns) = DWORD : 0\n\
Trigger Type 20: Width = STRING : [16] short\n\
Trigger Type 20: Delay (ns) = DWORD : 0\n\
Trigger Type 21: Width = STRING : [16] short\n\
Trigger Type 21: Delay (ns) = DWORD : 0\n\
Trigger Type 22: Width = STRING : [16] short\n\
Trigger Type 22: Delay (ns) = DWORD : 0\n\
Trigger Type 23: Width = STRING : [16] short\n\
Trigger Type 23: Delay (ns) = DWORD : 0\n\
Trigger Type 24: Width = STRING : [16] short\n\
Trigger Type 24: Delay (ns) = DWORD : 0\n\
Trigger Type 25: Width = STRING : [16] short\n\
Trigger Type 25: Delay (ns) = DWORD : 0\n\
Trigger Type 26: Width = STRING : [16] short\n\
Trigger Type 26: Delay (ns) = DWORD : 0\n\
Trigger Type 27: Width = STRING : [16] short\n\
Trigger Type 27: Delay (ns) = DWORD : 0\n\
Trigger Type 28: Width = STRING : [16] short\n\
Trigger Type 28: Delay (ns) = DWORD : 0\n\
Trigger Type 29: Width = STRING : [16] short\n\
Trigger Type 29: Delay (ns) = DWORD : 0\n\
Trigger Type 30: Width = STRING : [16] short\n\
Trigger Type 30: Delay (ns) = DWORD : 0\n\
Trigger Type 31: Width = STRING : [16] short\n\
Trigger Type 31: Delay (ns) = DWORD : 0\n\
"

//electronics map (array type)
typedef struct s_rider_map_to_calo_odb {
  BOOL  enabled;
  int   x_segment;
  int   y_segment;
  int   value;
  BOOL  polarity;
} RIDER_MAP_TO_CALO_ODB;

#define DETECTOR_ODB_MAP_STR "\
[.]\n\
Chan used (for enabled map) = BOOL : __used__\n\
Det x-segmt (for array map) = INT : __x__\n\
Det y-segmt (for array map) = INT : __y__\n\
threshold value  = INT : -200\n\
positive crossing Y(N)  = BOOL : 0\n\
"

/* total number of 10 Gb links per AMC13 */
#define AMC13_LINK_NUM   1
#define AMC13_RIDER_NUM 12
#define RIDER_CHAN_NUM   5
#define NSEGMENT_X       9
#define NSEGMENT_Y       6

// WFD5 board-channel structure
typedef struct s_amc13_rider_odb {
  struct s_rider_odb_board board;
  struct s_rider_odb_channel channel[RIDER_CHAN_NUM];
} AMC13_RIDER_ODB;

// FC7 common-encoder structure
typedef struct s_amc13_fc7_odb {
  struct s_fc7_odb_common common;
  struct s_fc7_odb_encoder encoder;
  struct s_fc7_odb_trigger trigger;
  struct s_fc7_odb_left_otrig lotrig;
  struct s_fc7_odb_right_otrig rotrig;
} AMC13_FC7_ODB;

extern AMC13_SETTINGS_ODB amc13_settings_odb;
extern TQ_PARAMETERS_ODB tq_parameters_odb[TQMETHOD_MAX];
extern AMC13_LINK_ODB amc13_link_odb[AMC13_LINK_NUM];
extern RIDER_MAP_TO_CALO_ODB rider_map_to_calo_odb[AMC13_RIDER_NUM][RIDER_CHAN_NUM][TQMETHOD_MAX];
extern AMC13_AMC13_ODB amc13_amc13_odb;
extern AMC13_RIDER_ODB amc13_rider_odb[AMC13_RIDER_NUM];
extern AMC13_FC7_ODB amc13_fc7_odb[AMC13_RIDER_NUM];

  
void amc13_ODB_update(HNDLE, INT, void*);
INT amc13_ODB_init(void);
INT amc13_ODB_set(void);
INT amc13_ODB_get(void);
  

#endif // amc13_odb_h defined
/* amc13_odb.h ends here */
