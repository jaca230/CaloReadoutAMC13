/* amc13_odb.c --- 
 * 
 * Filename:         amc13_odb.c
 * Description:      CALO AMC13 readout configuration through ODB
 * Author:           Tim Gorringe & Wes Gohn
 * Maintainer: 
 * Created:          Tue Jun 26 17:06:04 CDT 2014
 * Version: 
 * Last-Updated: Mon April 27 10:39:05 2017 (-0500)
 *           By: Wes Gohn
 *     Update #: 240
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

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <unordered_set>

#include <midas.h>
#include <mfe.h>
#include "amc13_odb.h"
#include <pugixml.hpp>
#include <odbxx.h>


#ifdef DEBUG                                                                                               
#define dbprintf(...) printf(__VA_ARGS__)                                                                  
#else                                                                                                      
#define dbprintf(...)                                                                                      
#endif

extern HNDLE hDB;
//extern INT frontend_index;  
extern EQUIPMENT equipment[];

AMC13_SETTINGS_ODB amc13_settings_odb;
TQ_PARAMETERS_ODB tq_parameters_odb[TQMETHOD_MAX];
AMC13_LINK_ODB amc13_link_odb[AMC13_LINK_NUM];
RIDER_MAP_TO_CALO_ODB rider_map_to_calo_odb[AMC13_RIDER_NUM][RIDER_CHAN_NUM][TQMETHOD_MAX];
AMC13_AMC13_ODB amc13_amc13_odb;
AMC13_RIDER_ODB amc13_rider_odb[AMC13_RIDER_NUM];
AMC13_FC7_ODB amc13_fc7_odb[AMC13_RIDER_NUM];

//CONSTANTS
const char* const FRONTEND_ODB_CONFIGURATION_XML = "frontends_odb_configuration.xml"; //configuration file for frontend odb settings

std::vector<std::string> generateStringListFromXml(const std::string& xmlFilePath, int frontendIndex, int maxIterations, int& errorCode) {
    std::vector<std::string> stringList;
    errorCode = 0;

    try {
        pugi::xml_document doc;
        pugi::xml_parse_result result = doc.load_file(xmlFilePath.c_str());

        if (!result) {
            std::cerr << "Error parsing XML file: " << result.description() << std::endl;
            errorCode = 1; // Set error code for XML parsing failure
            return stringList;
        }

        pugi::xml_node frontend = doc.child("frontend");
        int frontendIterations = 0;

        while (frontend && frontendIterations < maxIterations) {
            int id = frontend.attribute("id").as_int();

            if (id == frontendIndex) {
                break;
            }

            frontend = frontend.next_sibling("frontend");
            frontendIterations++;
        }

        if (!frontend) {
            std::cout << "Frontend with the specified index not found." << std::endl;
            errorCode = 2; // Set error code for frontend not found
            return stringList;
        }

        pugi::xml_node slot = frontend.child("slot");
        int slotIterations = 0;

        while (slot && slotIterations < maxIterations) {
            int slotId = slot.attribute("id").as_int();
            std::string slotType = slot.attribute("type").as_string();
            std::string slotString;

            if (slotType == "FC7") {
                slotString = "FC7-";
                if (slotId < 10) {
                    slotString += "0";
                }
                slotString += std::to_string(slotId);
            }
            else if (slotType == "Rider" || slotType == "WFD") {
                slotString = "Rider";
                if (slotId < 10) {
                    slotString += "0";
                }
                slotString += std::to_string(slotId);
            }

            stringList.push_back(slotString);

            slot = slot.next_sibling("slot");
            slotIterations++;
        }

        // Add additional entries
        stringList.push_back("Globals");
        stringList.push_back("Link01");
        stringList.push_back("AMC13");
        for (int i = 1; i <= TQMETHOD_MAX; i++) {
            std::string tqString = "TQ0" + std::to_string(i);
            stringList.push_back(tqString);
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        errorCode = 4; // Set error code for XML exception
    }

    return stringList;
}
std::vector<std::string> generateSubkeyListFromOdb(const std::string& filepath, int& errorCode) {
    std::vector<std::string> subkeyList;
    errorCode = 0;

    try {
        // Grab a bit of the ODB
        midas::odb exp(filepath);

        for (midas::odb& subkey : exp) {
            subkeyList.push_back(subkey.get_name());
        }
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        errorCode = 3; // Set error code for ODB exception
    }

    return subkeyList;
}
bool areExactSameEntries(const std::vector<std::string>& list1, const std::vector<std::string>& list2) {
    if (list1.size() != list2.size()) {
        return false;
    }

    std::unordered_set<std::string> set1(list1.begin(), list1.end());
    std::unordered_set<std::string> set2(list2.begin(), list2.end());

    return set1 == set2;
}
bool isSubset(const std::vector<std::string>& list1, const std::vector<std::string>& list2) {
    std::unordered_set<std::string> set1(list1.begin(), list1.end());
    std::unordered_set<std::string> set2(list2.begin(), list2.end());

    for (const std::string& entry : set1) {
        if (set2.find(entry) == set2.end()) {
            return false;
        }
    }

    return true;
}
std::vector<std::string> getMissingEntries(const std::vector<std::string>& list1, const std::vector<std::string>& list2) {
    std::unordered_set<std::string> set1(list1.begin(), list1.end());
    std::unordered_set<std::string> set2(list2.begin(), list2.end());

    std::vector<std::string> missingEntries;

    for (const std::string& entry : set2) {
        if (set1.find(entry) == set1.end()) {
            missingEntries.push_back(entry);
        }
    }

    return missingEntries;
}
std::vector<std::string> filterRiders(const std::vector<std::string>& xmlSettings) {
    std::vector<std::string> ridersList;

    for (const std::string& entry : xmlSettings) {
        if (entry.find("Rider") != std::string::npos) {
            ridersList.push_back(entry);
        }
    }

    return ridersList;
}


char *replace_str(const char *str,const char *orig,const char *rep) {
  static char buffer[4096];

  const char *p;

  if (!(p = strstr(str, orig))) {
    strcpy(buffer, str);
    return buffer;
  }

  strncpy(buffer, str, p-str);
  buffer[p-str] = '\0';

  sprintf(buffer+(p-str), "%s%s", rep, p+strlen(orig));

  return buffer;
}

void amc13_ODB_update(HNDLE hDB __attribute__((unused)), INT hKey __attribute__((unused)), void* INFO __attribute__((unused))) {
  char str[MAX_ODB_PATH];
  int i;

  amc13_ODB_get();
  int frontend_index = get_frontend_index();

  sprintf(str, "/Equipment/AMC13%02d/Settings/Globals/", frontend_index);

  dbprintf("%s(%d): %s sync %d\n", __func__, __LINE__, str, amc13_settings_odb.sync); 
  dbprintf("%s(%d): %s simulate data %d\n", __func__, __LINE__, str, amc13_settings_odb.simulate_data); 
  dbprintf("%s(%d): %s GPU Device ID %d\n", __func__, __LINE__, str, amc13_settings_odb.gpu_dev_id); 
  dbprintf("%s(%d): %s Send to event builder %d",__func__,__LINE__, str,amc13_settings_odb.send_to_event_builder);
  dbprintf("%s(%d): %s FE lossless compression %d",__func__,__LINE__, str,amc13_settings_odb.lossless_compression);
  dbprintf("%s(%d): %s FE bank-by-bank lossless compression %d",__func__,__LINE__, str,amc13_settings_odb.bankbybank_lossless_compression);
  dbprintf("%s(%d): %s raw data store %d\n", __func__, __LINE__, str, amc13_settings_odb.store_raw); 
  dbprintf("%s(%d): %s raw data prescale %d\n", __func__, __LINE__, str, amc13_settings_odb.prescale_raw); 
  dbprintf("%s(%d): %s raw data prescale offset %d\n", __func__, __LINE__, str, amc13_settings_odb.prescale_offset_raw); 

  if (frontend_index != 0) {
    for (i = 0; i< TQMETHOD_MAX; i++) {
      sprintf(str,"/Equipment/AMC13%02d/Settings/TQ%02d/GlobalParameters", frontend_index, i+1);
      int ir, ic;
      for (ir = 0; ir < AMC13_RIDER_NUM; ir++) {
        for (ic = 0; ic < RIDER_CHAN_NUM; ic++) {
          sprintf(str, "/Equipment/AMC13%02d/Settings/TQ%02d/Rider%02d/Channel%02d", frontend_index, i+1, ir+1, ic);

          dbprintf("%s(%d): %s Enabled %d\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][i].enabled);
          dbprintf("%s(%d): %s detector x-segment %d\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][i].x_segment);
          dbprintf("%s(%d): %s detector y-segment %d\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][i].y_segment);
          dbprintf("%s(%d): %s threshold setting %d\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][i].value);
          dbprintf("%s(%d): %s threshold x-ing %i\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][i].polarity);
        }
      }
    }
  } // if non-zero frontend index

  for (i = 0; i < AMC13_LINK_NUM; i++) {
    sprintf(str,"/Equipment/AMC13%02d/Settings/Link%02d/", frontend_index, i+1);

    dbprintf("%s(%d): %s enabled %d\n", __func__, __LINE__, str, amc13_link_odb[i].enabled);
    dbprintf("%s(%d): %s source port no %d\n", __func__, __LINE__, str, amc13_link_odb[i].source_port);
    dbprintf("%s(%d): %s source ip %s\n", __func__, __LINE__, str, amc13_link_odb[i].source_ip);
  } 

  sprintf(str,"/Equipment/AMC13%02d/Settings/AMC13/", frontend_index);

  dbprintf("%s(%d): %s hdr size %d\n", __func__, __LINE__, str, amc13_amc13_odb.header_size);
  dbprintf("%s(%d): %s block size %d\n", __func__, __LINE__, str, amc13_amc13_odb.amc_block_size);
  dbprintf("%s(%d): %s tlr size %d\n", __func__, __LINE__, str, amc13_amc13_odb.tail_size);
  dbprintf("%s(%d): %s addr Tab1 %s\n", __func__, __LINE__, str, amc13_amc13_odb.addrTab1);
  dbprintf("%s(%d): %s addr Tab2 %s\n", __func__, __LINE__, str, amc13_amc13_odb.addrTab2);  
  dbprintf("%s(%d): %s enableSoftwareTrigger %d\n", __func__, __LINE__, str, amc13_amc13_odb.enableSoftwareTrigger); 

  return;
}

/** 
 * Initialize ODB for Calo readout
 * 
 * 
 * @return SUCCESS on success
 */

INT amc13_ODB_init(void)
{
  INT status, ret = SUCCESS;
  char str[MAX_ODB_PATH];
  HNDLE hKey;
  
  int frontend_index = get_frontend_index();
  sprintf(str,"/Equipment/AMC13%02d/Settings/Globals",frontend_index);

  amc13_ODB_get();

  return ret; 
}

/** 
 * Write information for Calo readout to ODB
 * 
 * 
 * @return SUCCESS on success
 */

INT amc13_ODB_set()
{
  INT   status;
  HNDLE hKey;
  char  str[MAX_ODB_PATH];
  int   i;

  int frontend_index = get_frontend_index();
  sprintf(str,"/Equipment/AMC13%02d/Settings/Globals",frontend_index);

  // create ODB structure /Equipment/%s/Settings/Globals if doesn't exist
  char gpu_rep[8]; char subnet_rep[8]; char slot_rep[8]; char fmc_rep[8]; char sfp_rep[8];
  if (frontend_index == 26) {
    sprintf(gpu_rep, "%i", 1);
  } else {
    sprintf(gpu_rep, "%i", frontend_index % 2);
  }
  sprintf(subnet_rep, "%i", frontend_index);
  if (frontend_index == 0) {
    sprintf(slot_rep, "%s", "11");
    sprintf(fmc_rep,  "%s", "top");
    sprintf(sfp_rep,  "%s", "1");
  } else if ( 1 <= frontend_index && frontend_index <=  8) {
    sprintf(slot_rep, "%s", "5");
    sprintf(fmc_rep,  "%s", "bottom");
    sprintf(sfp_rep,  "%i", ((frontend_index - 1) % 8) + 1);
  } else if ( 9 <= frontend_index && frontend_index <= 16) {
    sprintf(slot_rep, "%s", "5");
    sprintf(fmc_rep,  "%s", "top");
    sprintf(sfp_rep,  "%i", ((frontend_index - 1) % 8) + 1);
  } else if (17 <= frontend_index && frontend_index <= 24) {
    sprintf(slot_rep, "%s", "8");
    sprintf(fmc_rep,  "%s", "bottom");
    sprintf(sfp_rep,  "%i", ((frontend_index - 1) % 8) + 1);
  } else if (25 <= frontend_index && frontend_index <= 32) {
    sprintf(slot_rep, "%s", "8");
    sprintf(fmc_rep,  "%s", "top");
    sprintf(sfp_rep,  "%i", ((frontend_index - 1) % 8) + 1);
  } else {
    sprintf(slot_rep, "%s", "");
    sprintf(fmc_rep,  "%s", "");
    sprintf(sfp_rep,  "%s", "");
  }
  status = db_check_record(hDB, 0, str, replace_str(replace_str(replace_str(replace_str(replace_str(AMC13_SETTINGS_ODB_STR, "__gpu__", gpu_rep), "__subnet__", subnet_rep), "__slot__", slot_rep), "__fmc__", fmc_rep), "__sfp__", sfp_rep), TRUE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
    ss_sleep(3000);
  }
  
  // returns key handle "hDB" to ODB name "str" for fast access 
  status = db_find_key(hDB, 0, str, &hKey);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
    return FE_ERR_ODB;
  }
  
  // Copy a set of keys from the ODB to the structure
  db_set_record(hDB, hKey, &amc13_settings_odb, sizeof(AMC13_SETTINGS_ODB), 0);
  
  dbprintf("%s(%d): %s sync %d\n", __func__, __LINE__, str, amc13_settings_odb.sync); 

  int maxIterations = 1000;
  int errorCode = 0;
  std::vector<std::string> xml_settings = generateStringListFromXml(FRONTEND_ODB_CONFIGURATION_XML, frontend_index, maxIterations,errorCode);
  if (errorCode != 0 ) {
    return FE_ERR_ODB;
  }
  sprintf(str,"/Equipment/AMC13%02d/Settings",frontend_index);
  std::vector<std::string> odb_settings = generateSubkeyListFromOdb(str,errorCode);
  if (errorCode != 0 ) {
    return FE_ERR_ODB;
  }

  if (xml_settings.empty()) {
      std::cout << "No slots found for the specified frontend index." << std::endl;
  }

  //There is nothing left to do if the settings are the same as we want them to be.
  //This assumes the settings are populated correctly. But even if they aren't we can just change
  //The .xml file to make them popualte correctly (this is a hack, but so is this whole project)
  if (areExactSameEntries(xml_settings,odb_settings)) {
    return SUCCESS;
  }
  
  //We only update the settings if odb_settings does not contain everything in xml_settings
  //NOTE: For some reason, odb_settings always contains 12 riders and 12 FC7s, it might have something to do with amc13_ODB_init()
  //      Which is called before amc13_ODB_set() in the frontend. It doesn't seem to really matter for this code's purpose though.
  if (!isSubset(xml_settings,odb_settings)) {
    int itq;
    for (itq = 0; itq < TQMETHOD_MAX; itq++) {
      sprintf(str,"/Equipment/AMC13%02d/Settings/TQ%02d/GlobalParameters",frontend_index,itq+1);

      // create ODB structure /Equipment/%s/Settings/TQParameters if doesn't exist
      char prefix_rep[8];
      if (itq == 0) {
        sprintf(prefix_rep, "%s", "C");
      } else if (itq == 1) {
        sprintf(prefix_rep, "%s", "X");
      } else if (itq == 2) {
        sprintf(prefix_rep, "%s", "Y");
      } else if (itq == 3) {
        sprintf(prefix_rep, "%s", "Z");
      } else {
        sprintf(prefix_rep, "%s", "");
      }
      char gpux_rep[8];
      if (itq == 0) {
        sprintf(gpux_rep, "%s", "1");
      } else {
        sprintf(gpux_rep, "%s", "0");
      }
      status = db_check_record(hDB, 0, str, replace_str(replace_str(replace_str(TQ_PARAMETERS_ODB_STR, "__gpu1__", gpux_rep), "__gpu2__", gpux_rep), "__prefix__", prefix_rep), TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        ss_sleep(3000);
      }
        
      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }
        
      // copy a set of keys from the ODB to the structure
      db_set_record(hDB, hKey, &tq_parameters_odb[itq], sizeof(TQ_PARAMETERS_ODB), 0);
      
      dbprintf("%s(%d): %s TQ enabled %d\n", __func__, __LINE__, str, tq_parameters_odb[itq].TQ_on); 

      int ir, ic;
        for(ir = 0; ir < AMC13_RIDER_NUM; ir++){
          for(ic = 0; ic < RIDER_CHAN_NUM; ic++){
            sprintf(str,"/Equipment/AMC13%02d/Settings/TQ%02d/Rider%02d/Channel%02d", frontend_index, itq+1, ir+1,ic);
      
            // create ODB structure /Equipment/%s/Settings/TQArrayMap if doesn't exist
            char used_rep[8]; char x_rep[8]; char y_rep[8];
            if (ir <= 8) {
              sprintf(used_rep, "%s", "y");
              sprintf(x_rep, "%i", ir + 9 - 2*ir);
              sprintf(y_rep, "%i", ic + 1);
            } else if (ir == 9) {
              sprintf(used_rep, "%s", "y");
              sprintf(x_rep, "%i", ic + 5);
              sprintf(y_rep, "%i", 6);
            } else if (ir == 10) {
              if (ic <= 3) {
                sprintf(used_rep, "%s", "y");
                sprintf(x_rep, "%i", ic + 1);
                sprintf(y_rep, "%i", 6);
              } else {
                sprintf(used_rep, "%s", "n");
                sprintf(x_rep, "%i", 0);
                sprintf(y_rep, "%i", 0);
              }
            } else {
              sprintf(used_rep, "%s", "n");
              sprintf(x_rep, "%i", 0);
              sprintf(y_rep, "%i", 0);
            }
            status = db_check_record(hDB, 0, str, replace_str(replace_str(replace_str(DETECTOR_ODB_MAP_STR, "__used__", used_rep), "__x__", x_rep), "__y__", y_rep), TRUE);
            if (status != DB_SUCCESS) {
              cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
              ss_sleep(3000);
            }
            
            // returns key handle "hDB" to ODB name "str" for fast access 
            status = db_find_key(hDB, 0, str, &hKey);
            if (status != DB_SUCCESS) {
              cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
              return FE_ERR_ODB;
            }

            // copy a set of keys from the ODB to the structure
            db_set_record(hDB, hKey, &rider_map_to_calo_odb[ir][ic][itq], sizeof(RIDER_MAP_TO_CALO_ODB), 0);
          }
        }

      } // loop over TQ methods
          
    for (i = 0; i < AMC13_LINK_NUM; i++) {
      sprintf(str,"/Equipment/AMC13%02d/Settings/Link%02d",frontend_index,i+1);
      
      // create ODB structure /Equipment/%s/Settings/Link%02d if doesn't exist
      char subnet_rep[8];
      if (frontend_index == 26) {
        sprintf(subnet_rep, "%i", 2);
      } else {
        sprintf(subnet_rep, "%i", (frontend_index % 2) + 1);
      }
      status = db_check_record(hDB, 0, str, replace_str(AMC13_LINK_ODB_STR, "__subnet__", subnet_rep), TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        ss_sleep(3000);
      }
        
      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }

      // Copy a C-structure to a ODB sub-tree
      db_set_record(hDB, hKey, &amc13_link_odb[i], sizeof(AMC13_LINK_ODB), 0);

      dbprintf("%s(%d): %s enabled %d\n", __func__, __LINE__, str, amc13_link_odb[i].enabled);
    }

    sprintf(str,"/Equipment/AMC13%02d/Settings/AMC13",frontend_index);

    // create ODB structure /Equipment/%s/Settings/AMC13 if doesn't exist
    status = db_check_record(hDB, 0, str, AMC13_AMC13_ODB_STR, TRUE);
    if (status != DB_SUCCESS) {
      cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
      ss_sleep(3000);
    }
    
    // returns key handle "hDB" to ODB name "str" for fast access 
    status = db_find_key(hDB, 0, str, &hKey);
    if (status != DB_SUCCESS) {
      cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
      return FE_ERR_ODB;
    }
    
    // Copy a set of keys from the ODB to the structure
    db_set_record(hDB, hKey, &amc13_amc13_odb, sizeof(AMC13_AMC13_ODB), 0);

    // FC7 crate
    for (i = 0; i < 12; i++) {
      // /Settings/FC7-XX/Common
      sprintf(str, "/Equipment/AMC13%02d/Settings/FC7-%02d/Common", frontend_index, i+1);
        
      // create ODB structure /Equipment/%s/Settings/FC7-%02d if doesn't exist
      status = db_check_record(hDB, 0, str, FC7_ODB_COMMON_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        ss_sleep(3000);
      }

      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }

      // copy a C-structure to a ODB sub-tree
      db_set_record(hDB, hKey, &amc13_fc7_odb[i], sizeof(AMC13_FC7_ODB), 0);

      // /Settings/FC7-XX/Encoder
      sprintf(str, "/Equipment/AMC13%02d/Settings/FC7-%02d/Encoder", frontend_index, i+1);
      
      // create ODB structure /Equipment/%s/Settings/FC7-%02d/Encoder if doesn't exist
      status = db_check_record(hDB, 0, str, FC7_ODB_ENCODER_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        ss_sleep(3000);
      }

      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }

      // copy a C-structure to a ODB sub-tree
      db_set_record(hDB, hKey, &amc13_fc7_odb[i].encoder, sizeof(FC7_ODB_ENCODER), 0);

      // /Settings/FC7-XX/Trigger
      sprintf(str, "/Equipment/AMC13%02d/Settings/FC7-%02d/Trigger", frontend_index, i+1);
        
      // create ODB structure /Equipment/%s/Settings/FC7-%02d/Trigger if doesn't exist
      status = db_check_record(hDB, 0, str, FC7_ODB_TRIGGER_STR, TRUE);
      if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
          ss_sleep(3000);
      }
        
      // returns key handle "hDB" to ODB name "str" for fast access
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);
          return FE_ERR_ODB;
      }
        
        // copy a C-structure to a ODB sub-tree
        db_set_record(hDB, hKey, &amc13_fc7_odb[i].trigger, sizeof(FC7_ODB_TRIGGER), 0);
        

        // /Settings/FC7-XX/Left Trigger Output
      sprintf(str, "/Equipment/AMC13%02d/Settings/FC7-%02d/Left Trigger Output", frontend_index, i+1);
      
      // create ODB structure /Equipment/%s/Settings/FC7-%02d/Left Trigger Output if doesn't exist
      status = db_check_record(hDB, 0, str, FC7_ODB_LEFT_OTRIG_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        ss_sleep(3000);
      }

      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }

      // copy a C-structure to a ODB sub-tree
      db_set_record(hDB, hKey, &amc13_fc7_odb[i].lotrig, sizeof(FC7_ODB_LEFT_OTRIG), 0);

      // /Settings/FC7-XX/Right Trigger Output
      sprintf(str, "/Equipment/AMC13%02d/Settings/FC7-%02d/Right Trigger Output", frontend_index, i+1);
      
      // create ODB structure /Equipment/%s/Settings/FC7-%02d/Right Trigger Output if doesn't exist
      status = db_check_record(hDB, 0, str, FC7_ODB_RIGHT_OTRIG_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        ss_sleep(3000);
      }

      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }

      // copy a C-structure to a ODB sub-tree
      db_set_record(hDB, hKey, &amc13_fc7_odb[i].rotrig, sizeof(FC7_ODB_RIGHT_OTRIG), 0);
    }

    for (i = 0; i < 12; i++) {
      sprintf(str, "/Equipment/AMC13%02d/Settings/Rider%02d/Board", frontend_index, i+1);
        
      // create ODB structure /Equipment/%s/Settings/Rider%02d if doesn't exist
      status = db_check_record(hDB, 0, str, RIDER_ODB_BOARD_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        ss_sleep(3000);
      }

      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }

      // copy a C-structure to a ODB sub-tree
      db_set_record(hDB, hKey, &amc13_rider_odb[i], sizeof(AMC13_RIDER_ODB), 0);

      int j;
      for (j = 0; j < 5; j++) {
        sprintf(str, "/Equipment/AMC13%02d/Settings/Rider%02d/Channel%02d", frontend_index, i+1, j);
        
        // create ODB structure /Equipment/%s/Settings/Rider%02d/Channel%02d if doesn't exist
        status = db_check_record(hDB, 0, str, RIDER_ODB_CHANNEL_STR, TRUE);
        if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
          ss_sleep(3000);
        }

        // returns key handle "hDB" to ODB name "str" for fast access 
        status = db_find_key(hDB, 0, str, &hKey);
        if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
          return FE_ERR_ODB;
        }
    
        // copy a C-structure to a ODB sub-tree
        db_set_record(hDB, hKey, &amc13_rider_odb[i].channel[j], sizeof(RIDER_ODB_CHANNEL), 0);
      }
    }
  }
  
  
  // Now, we remove ODB settings that don't belong
  sprintf(str,"/Equipment/AMC13%02d/Settings",frontend_index);
  odb_settings = generateSubkeyListFromOdb(str,errorCode);
  if (errorCode != 0 ) {
    return FE_ERR_ODB;
  }
  std::vector<std::string> extraEntries = getMissingEntries(xml_settings,odb_settings);
  
  // Loop over the extraEntries vector and delete them
  for (const std::string& entry : extraEntries) {
      std::string entryPath = std::string(str) + "/" + std::string(entry);

      // Delete the key in the ODB
      midas::odb existing_bit(entryPath);
      existing_bit.delete_key();

      //std::cout << "Deleted key: " << entryPath << std::endl;
  }
  // Do same as above for Riders in TQ0y *y iterated over)
  std::vector<std::string> xmlSettingsRidersOnly = filterRiders(xml_settings);
  int itq;
  for (itq = 0; itq < TQMETHOD_MAX; itq++) {
    sprintf(str,"/Equipment/AMC13%02d/Settings/TQ%02d",frontend_index,itq+1);
    std::vector<std::string> odbTqSettingsRidersOnly = filterRiders(generateSubkeyListFromOdb(str,errorCode));
    if (errorCode != 0 ) {
      return FE_ERR_ODB;
    }
    std::vector<std::string> extraRiders = getMissingEntries(xmlSettingsRidersOnly,odbTqSettingsRidersOnly);
    for (const std::string& rider : extraRiders) {
      std::string entryPath = std::string(str) + "/" + std::string(rider);

      // Delete the key in the ODB
      midas::odb existing_bit(entryPath);
      existing_bit.delete_key();

      //std::cout << "Deleted key: " << entryPath << std::endl;
    }
  }


  
  
  /*
  // WFD5 crate
  // This else statement was acting on if (frontend_index == 0)
  else {

    for (i = 0; i < 12; i++) {
      sprintf(str, "/Equipment/AMC13%02d/Settings/Rider%02d/Board", frontend_index, i+1);
        
      // create ODB structure /Equipment/%s/Settings/Rider%02d if doesn't exist
      status = db_check_record(hDB, 0, str, RIDER_ODB_BOARD_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        ss_sleep(3000);
      }

      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }

      // copy a C-structure to a ODB sub-tree
      db_set_record(hDB, hKey, &amc13_rider_odb[i], sizeof(AMC13_RIDER_ODB), 0);

      int j;
      for (j = 0; j < 5; j++) {
        sprintf(str, "/Equipment/AMC13%02d/Settings/Rider%02d/Channel%02d", frontend_index, i+1, j);
        
        // create ODB structure /Equipment/%s/Settings/Rider%02d/Channel%02d if doesn't exist
        status = db_check_record(hDB, 0, str, RIDER_ODB_CHANNEL_STR, TRUE);
        if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
          ss_sleep(3000);
        }

        // returns key handle "hDB" to ODB name "str" for fast access 
        status = db_find_key(hDB, 0, str, &hKey);
        if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
          return FE_ERR_ODB;
        }
    
        // copy a C-structure to a ODB sub-tree
        db_set_record(hDB, hKey, &amc13_rider_odb[i].channel[j], sizeof(RIDER_ODB_CHANNEL), 0);
      }
    }

  } // end AMC module select
  */
  return SUCCESS;
}

INT amc13_ODB_get()
{
  INT   status;
  char  str[MAX_ODB_PATH];
  HNDLE hKey;
  int   size, i;

  int frontend_index = get_frontend_index();

  sprintf(str, "/Equipment/AMC13%02d/Settings/Globals", frontend_index);

  // create ODB structure /Equipment/%s/Settings/Globals if doesn't exist
  char gpu_rep[8]; char subnet_rep[8]; char slot_rep[8]; char fmc_rep[8]; char sfp_rep[8];
  if (frontend_index == 26) {
    sprintf(gpu_rep, "%i", 1);
  } else {
    sprintf(gpu_rep, "%i", frontend_index % 2);
  }
  sprintf(subnet_rep, "%i", frontend_index);
  if (frontend_index == 0) {
    sprintf(slot_rep, "%s", "11");
    sprintf(fmc_rep,  "%s", "top");
    sprintf(sfp_rep,  "%s", "1");
  } else if ( 1 <= frontend_index && frontend_index <=  8) {
    sprintf(slot_rep, "%s", "5");
    sprintf(fmc_rep,  "%s", "bottom");
    sprintf(sfp_rep,  "%i", ((frontend_index - 1) % 8) + 1);
  } else if ( 9 <= frontend_index && frontend_index <= 16) {
    sprintf(slot_rep, "%s", "5");
    sprintf(fmc_rep,  "%s", "top");
    sprintf(sfp_rep,  "%i", ((frontend_index - 1) % 8) + 1);
  } else if (17 <= frontend_index && frontend_index <= 24) {
    sprintf(slot_rep, "%s", "8");
    sprintf(fmc_rep,  "%s", "bottom");
    sprintf(sfp_rep,  "%i", ((frontend_index - 1) % 8) + 1);
  } else if (25 <= frontend_index && frontend_index <= 32) {
    sprintf(slot_rep, "%s", "8");
    sprintf(fmc_rep,  "%s", "top");
    sprintf(sfp_rep,  "%i", ((frontend_index - 1) % 8) + 1);
  } else {
    sprintf(slot_rep, "%s", "");
    sprintf(fmc_rep,  "%s", "");
    sprintf(sfp_rep,  "%s", "");
  }
  status = db_check_record(hDB, 0, str, replace_str(replace_str(replace_str(replace_str(replace_str(AMC13_SETTINGS_ODB_STR, "__gpu__", gpu_rep), "__subnet__", subnet_rep), "__slot__", slot_rep), "__fmc__", fmc_rep), "__sfp__", sfp_rep), TRUE);
  if (status != DB_SUCCESS) {

    //    cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);

    ss_sleep(3000);
  }
  
  // returns key handle "hDB" to ODB name "str" for fast access
  status = db_find_key(hDB, 0, str, &hKey);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
    return FE_ERR_ODB;
  }
  
  // Copy an ODB sub-tree to C-structure
  size = sizeof(AMC13_SETTINGS_ODB);
  status = db_get_record(hDB, hKey, &amc13_settings_odb, &size, 0);
  if (status != DB_SUCCESS) {
    if (status == DB_NO_MEMORY) printf("DB_NO_MEMORY \n");
    if (status == DB_STRUCT_SIZE_MISMATCH) printf("DB_STRUCT_SIZE_MISMATCH \n");
    cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
    return FE_ERR_ODB;
  }
  
  dbprintf("%s(%d): %s sync %d\n", __func__, __LINE__, str, amc13_settings_odb.sync);

  if (frontend_index != 0) {
    int itq;
    for (itq = 0; itq < TQMETHOD_MAX; itq++) {
      sprintf(str, "/Equipment/AMC13%02d/Settings/TQ%02d/GlobalParameters", frontend_index, itq+1);

      // create ODB structure /Equipment/%s/Settings/TQParameters if doesn't exist
      char prefix_rep[8];
      if (itq == 0) {
        sprintf(prefix_rep, "%s", "C");
      } else if (itq == 1) {
        sprintf(prefix_rep, "%s", "X");
      } else if (itq == 2) {
        sprintf(prefix_rep, "%s", "Y");
      } else if (itq == 3) {
        sprintf(prefix_rep, "%s", "Z");
      } else {
        sprintf(prefix_rep, "%s", "");
      }
      char gpux_rep[8];
      if (itq == 0) {
	      sprintf(gpux_rep, "%s", "1");
      } else {
	      sprintf(gpux_rep, "%s", "0");
      }
      status = db_check_record(hDB, 0, str, replace_str(replace_str(replace_str(TQ_PARAMETERS_ODB_STR, "__gpu1__", gpux_rep), "__gpu2__", gpux_rep), "__prefix__", prefix_rep), TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        //ss_sleep(3000);
        return FE_ERR_ODB;
      }
        
      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }
        
      // Copy an ODB sub-tree to C-structure
      size = sizeof(TQ_PARAMETERS_ODB);
      status = db_get_record(hDB, hKey, &tq_parameters_odb[itq], &size, 0);
      if (status != DB_SUCCESS) {
	if (status == DB_NO_MEMORY) printf("DB_NO_MEMORY \n");
	if (status == DB_STRUCT_SIZE_MISMATCH) printf("DB_STRUCT_SIZE_MISMATCH \n");
	cm_msg(MERROR, __FILE__, "Cannot get [%s] settings in ODB, error code = %d", str,status);
	return FE_ERR_ODB;
      }
      
      int ir, ic;
      for (ir = 0; ir < AMC13_RIDER_NUM; ir++) {
        for (ic = 0; ic < RIDER_CHAN_NUM; ic++) {
          sprintf(str,"/Equipment/AMC13%02d/Settings/TQ%02d/Rider%02d/Channel%02d", frontend_index, itq+1, ir+1,ic);
      
          // create ODB structure /Equipment/%s/Settings/TQArrayMap if doesn't exist
          char used_rep[8]; char x_rep[8]; char y_rep[8];
          if (ir <= 8) {
            sprintf(used_rep, "%s", "y");
            sprintf(x_rep, "%i", ir + 9 - 2*ir);
            sprintf(y_rep, "%i", ic + 1);
          } else if (ir == 9) {
            sprintf(used_rep, "%s", "y");
            sprintf(x_rep, "%i", ic + 5);
            sprintf(y_rep, "%i", 6);
          } else if (ir == 10) {
            if (ic <= 3) {
              sprintf(used_rep, "%s", "y");
              sprintf(x_rep, "%i", ic + 1);
              sprintf(y_rep, "%i", 6);
            } else {
              sprintf(used_rep, "%s", "n");
              sprintf(x_rep, "%i", 0);
              sprintf(y_rep, "%i", 0);
            }
          } else {
            sprintf(used_rep, "%s", "n");
            sprintf(x_rep, "%i", 0);
            sprintf(y_rep, "%i", 0);
          }
          status = db_check_record(hDB, 0, str, replace_str(replace_str(replace_str(DETECTOR_ODB_MAP_STR, "__used__", used_rep), "__x__", x_rep), "__y__", y_rep), TRUE);
          if (status != DB_SUCCESS) {
            cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
            return FE_ERR_ODB;
          }
      
          // returns key handle "hDB" to ODB name "str" for fast access 
          status = db_find_key(hDB, 0, str, &hKey);
          if (status != DB_SUCCESS) {
            cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
            return FE_ERR_ODB;
          }
      
          // Copy an ODB sub-tree to C-structure
          size = sizeof(RIDER_MAP_TO_CALO_ODB);
          status = db_get_record(hDB, hKey, &rider_map_to_calo_odb[ir][ic][itq], &size, 0);
	  if (status != DB_SUCCESS) {
	    if (status == DB_NO_MEMORY) printf("DB_NO_MEMORY \n");
	    if (status == DB_STRUCT_SIZE_MISMATCH) printf("DB_STRUCT_SIZE_MISMATCH \n");
	    cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
	    return FE_ERR_ODB;
	  }
          
          dbprintf("%s(%d): %s enabled %d\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][itq].enabled);
          dbprintf("%s(%d): %s detector x-segment %d\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][itq].x_segment);
          dbprintf("%s(%d): %s detector y-segment %d\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][itq].y_segment);
          dbprintf("%s(%d): %s threshold setting %d\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][i].value);
          dbprintf("%s(%d): %s threshold x-ing %i\n", __func__, __LINE__, str, rider_map_to_calo_odb[ir][ic][i].polarity);
        }
      }
    }
  } // if non-zero frontend index

  for (i = 0; i < AMC13_LINK_NUM; i++) {
    sprintf(str, "/Equipment/AMC13%02d/Settings/Link%02d", frontend_index, i+1);
      
    // create ODB structure /Equipment/%s/Settings/Link%02d if doesn't exist
    char subnet_rep[8];
    if (frontend_index == 26) {
      sprintf(subnet_rep, "%i", 2);
    } else {
      sprintf(subnet_rep, "%i", (frontend_index % 2) + 1);
    }
    status = db_check_record(hDB, 0, str, replace_str(AMC13_LINK_ODB_STR, "__subnet__", subnet_rep), TRUE);
    if (status != DB_SUCCESS) {
      cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
      //ss_sleep(3000);
      return FE_ERR_ODB;
    }
      
    // returns key handle "hDB" to ODB name "str" for fast access 
    status = db_find_key(hDB, 0, str, &hKey);
    if (status != DB_SUCCESS) {
      cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
      return FE_ERR_ODB;
    }
      
    // Copy an ODB sub-tree to C-structure
    size = sizeof(AMC13_LINK_ODB);
    status = db_get_record(hDB, hKey, &amc13_link_odb[i], &size, 0);
    if (status != DB_SUCCESS) {
      if (status == DB_NO_MEMORY) printf("DB_NO_MEMORY \n");
      if (status == DB_STRUCT_SIZE_MISMATCH) printf("DB_STRUCT_SIZE_MISMATCH \n");
      cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
      return FE_ERR_ODB;
    }
    
    dbprintf("%s(%d): %s enabled %d\n", __func__, __LINE__, str, amc13_link_odb[i].enabled);
  }
  
  sprintf(str, "/Equipment/AMC13%02d/Settings/AMC13", frontend_index);

  // create ODB structure /Equipment/%s/Settings/amc13 if doesn't exist
  status = db_check_record(hDB, 0, str, AMC13_AMC13_ODB_STR, TRUE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
    //ss_sleep(3000);
    return FE_ERR_ODB;
  }
  
  // returns key handle "hDB" to ODB name "str" for fast access 
  status = db_find_key(hDB, 0, str, &hKey);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
    return FE_ERR_ODB;
  }
  
  dbprintf("%s(%d): %s header size %d\n", __func__, __LINE__, str, amc13_amc13_odb.header_size);
  dbprintf("%s(%d): %s amc block size %d\n", __func__, __LINE__, str, amc13_amc13_odb.amc_block_size);
  dbprintf("%s(%d): %s tail size %d\n", __func__, __LINE__, str, amc13_amc13_odb.tail_size);

  // copy an ODB sub-tree to C-structure
  size = sizeof(AMC13_AMC13_ODB);
  status = db_get_record(hDB, hKey, &amc13_amc13_odb, &size, 0);
  if (status != DB_SUCCESS) {
    if (status == DB_NO_MEMORY) printf("DB_NO_MEMORY \n");
    if (status == DB_STRUCT_SIZE_MISMATCH) printf("DB_STRUCT_SIZE_MISMATCH \n");
    cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
    return FE_ERR_ODB;
  }
  
  // FC7 crate
  if (frontend_index == 0) {

    for (i = 0; i < 12; i++) {
      // /Settings/FC7-XX/Common
      sprintf(str,"/Equipment/AMC13%02d/Settings/FC7-%02d/Common", frontend_index, i+1);
        
      // create ODB structure /Equipment/%s/Settings/FC7-%02d if doesn't exist
      status = db_check_record(hDB, 0, str, FC7_ODB_COMMON_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        //ss_sleep(3000);
        return FE_ERR_ODB;
      }
          
      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }
          
      // copy an ODB sub-tree to C-structure
      size = sizeof(FC7_ODB_COMMON);
      status = db_get_record(hDB, hKey, &amc13_fc7_odb[i].common, &size, 0);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
        return FE_ERR_ODB;
      }
          
      // /Settings/FC7-XX/Encoder
      sprintf(str, "/Equipment/AMC13%02d/Settings/FC7-%02d/Encoder", frontend_index, i+1);
        
      // create ODB structure /Equipment/%s/Settings/FC7-%02d/Encoder if doesn't exist
      status = db_check_record(hDB, 0, str, FC7_ODB_ENCODER_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        //ss_sleep(3000);
        return FE_ERR_ODB;
      }
       
      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }
      
      // copy an ODB sub-tree to C-structure
      size = sizeof(FC7_ODB_ENCODER);
      status = db_get_record(hDB, hKey, &amc13_fc7_odb[i].encoder, &size, 0);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
        return FE_ERR_ODB;
      }

       // /Settings/FC7-XX/Trigger
       sprintf(str, "/Equipment/AMC13%02d/Settings/FC7-%02d/Trigger", frontend_index, i+1);
       
       // create ODB structure /Equipment/%s/Settings/FC7-%02d/Encoder if doesn't exist
       status = db_check_record(hDB, 0, str, FC7_ODB_TRIGGER_STR, TRUE);
       if (status != DB_SUCCESS) {
         cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
         //ss_sleep(3000);
         return FE_ERR_ODB;
       }
       
       // returns key handle "hDB" to ODB name "str" for fast access
       status = db_find_key(hDB, 0, str, &hKey);
       if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);
          return FE_ERR_ODB;
       }
       
       // copy an ODB sub-tree to C-structure
       size = sizeof(FC7_ODB_TRIGGER);
       status = db_get_record(hDB, hKey, &amc13_fc7_odb[i].trigger, &size, 0);
       if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);
          return FE_ERR_ODB;
       }
       

       // /Settings/FC7-XX/Left Trigger Output
      sprintf(str, "/Equipment/AMC13%02d/Settings/FC7-%02d/Left Trigger Output", frontend_index, i+1);
        
      // create ODB structure /Equipment/%s/Settings/FC7-%02d/Left Trigger Output if doesn't exist
      status = db_check_record(hDB, 0, str, FC7_ODB_LEFT_OTRIG_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        //ss_sleep(3000);
        return FE_ERR_ODB;
      }
       
      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }
      
      // copy an ODB sub-tree to C-structure
      size = sizeof(FC7_ODB_LEFT_OTRIG);
      status = db_get_record(hDB, hKey, &amc13_fc7_odb[i].lotrig, &size, 0);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
        return FE_ERR_ODB;
      }

      // /Settings/FC7-XX/Right Trigger Output
      sprintf(str, "/Equipment/AMC13%02d/Settings/FC7-%02d/Right Trigger Output", frontend_index, i+1);
        
      // create ODB structure /Equipment/%s/Settings/FC7-%02d/Right Trigger Output if doesn't exist
      status = db_check_record(hDB, 0, str, FC7_ODB_RIGHT_OTRIG_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        //ss_sleep(3000);
        return FE_ERR_ODB;
      }
       
      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }
      
      // copy an ODB sub-tree to C-structure
      size = sizeof(FC7_ODB_RIGHT_OTRIG);
      status = db_get_record(hDB, hKey, &amc13_fc7_odb[i].rotrig, &size, 0);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
        return FE_ERR_ODB;
      }
    }

  }
  // WFD5 crate
  else {

    for (i = 0; i < 12; i++) {
      sprintf(str,"/Equipment/AMC13%02d/Settings/Rider%02d/Board", frontend_index, i+1);
        
      // create ODB structure /Equipment/%s/Settings/Rider%02d if doesn't exist
      status = db_check_record(hDB, 0, str, RIDER_ODB_BOARD_STR, TRUE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
        //ss_sleep(3000);
        return FE_ERR_ODB;
      }
          
      // returns key handle "hDB" to ODB name "str" for fast access 
      status = db_find_key(hDB, 0, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
        return FE_ERR_ODB;
      }
          
      // copy an ODB sub-tree to C-structure
      size = sizeof(RIDER_ODB_BOARD);
      status = db_get_record(hDB, hKey, &amc13_rider_odb[i].board, &size, 0);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
        return FE_ERR_ODB;
      }
      
      int j;
      for (j = 0; j < 5; j++) {
        sprintf(str, "/Equipment/AMC13%02d/Settings/Rider%02d/Channel%02d", frontend_index, i+1, j);
          
        // create ODB structure /Equipment/%s/Settings/Rider%02d/Channel%02d if doesn't exist
        status = db_check_record(hDB, 0, str, RIDER_ODB_CHANNEL_STR, TRUE);
        if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot create [%s] entry in ODB, err = %i", str, status);
          //ss_sleep(3000);
	  return FE_ERR_ODB;
        }
         
        // returns key handle "hDB" to ODB name "str" for fast access 
        status = db_find_key(hDB, 0, str, &hKey);
        if (status != DB_SUCCESS) {
          cm_msg(MERROR, __FILE__, "Cannot find [%s] key in ODB, err = %i", str, status);     
          return FE_ERR_ODB;
        }
         
        // copy an ODB sub-tree to C-structure
        size = sizeof(RIDER_ODB_CHANNEL);
        status = db_get_record(hDB, hKey, &amc13_rider_odb[i].channel[j], &size, 0);      
	if (status != DB_SUCCESS) {
	  if (status == DB_NO_MEMORY) printf("DB_NO_MEMORY \n");
	  if (status == DB_STRUCT_SIZE_MISMATCH) printf("DB_STRUCT_SIZE_MISMATCH \n");
	  cm_msg(MERROR, __FILE__, "Cannot copy [%s] to C-structure, err = %i", str, status);     
	  return FE_ERR_ODB;
	}
      }
    }

  } // end AMC module select
  
  return SUCCESS;
}

/* amc13_odb.c ends here */
