
readandunpack( void* header, void *tail, void* data){

  unsigned int EventIndex, OverallSize;
  unsigned int iAMC, nAMC;
  uint64_t offsetheader = header;
  
  // get overall header word
  unsigned int AMC13headerbytes = sizeof(uint64_t);
  if ( ReadXBytes( clientsockfd, AMC13headerbytes, (void*)( offsetheader ) ) < 0 ) 
    {                                                                                    
      cm_msg(MERROR, __FILE__, "Cannot read header from socket");                                 
      return FE_ERR_HW;                                                                  
    }
  
  // record time got header word
  status = gettimeofday( &theader, NULL);
  trigger_info.time_tcp_finish_header_read_s = theader.tv_sec;
  trigger_info.time_tcp_finish_header_read_us = theader.tv_usec; 
  header[3] = theader.tv_sec; // fill header time info in header
  header[4] = theader.tv_usec; // fill header time info in header
  dbprintf("%s(%d): duration from AVAIL lock to fill header bank, buffer[%d], fill %d, duration %e us \n", 
	     __func__, __LINE__, bufIndex, TCPfillnumber, toddiff( &theader, &tstart) );

  // get event number from header bank
  EventIndex = getEventIndex( be64toh( offset ) );
  dbprintf("%s(%d): read header, header size [bytes] %d, header[0] 0x%016llX, BODdelimiter 0x%016llX, BODmask 0x%016llx Event number %i\n", 
	     __func__, __LINE__, headerbytes, be64toh(header[0]), BODdelimiter, BODmask, EventIndex );
  headeroffset++;

  // counters for AMC data bytes and AMC data blocks
  unsigned int databytestotal, blockcount; // amc data, block counters
  databytestotal = 0; 
  blockcount = 0;

  // data offsets for unpacking data buffer structure of AMCs x blocks
  unsigned int dataarrayoffset, datablockoffset[12], dataAMCoffset[12];
  dataarrayoffset = 0;
  memset( datablockoffset, 0, sizeof(datablockoffset) );
  memset( dataAMCoffset, 0, sizeof(dataAMCoffset) );
 
  // set more bit for first data segment to get in while loop
  amc_header_info[0].AMCMoreBit = 1;
  while ( amc_header_info[0].AMCMoreBit ){  // loops over AMC data blocks 
    
    // read single 64-bit AMC block header word
    if (ReadXBytes( clientsockfd, sizeof(uint64_t), (void*)( offsetheader ) ) < 0) 
      {                                                                                    
	cm_msg(MERROR, __FILE__, "Cannot read data from socket, fd %d", clientsockfd);                                 
	return FE_ERR_HW;                                                                  
      }
    
    // get the number if enabled AMCs
    nAMC = getAMCNum( be64toh( *offsetheader  );
    dbprintf("%s(%d): reading AMC general header word 0x%016llX, nAMC decoded %i\n", 
	     __func__, __LINE__, *offsetheader, getAMCNum( be64toh( *offsetheader ) ) ); 
    //nAMC = 12; // override due to corruption of the AMCGeneralHeader word at high rate      
    offsetheader++;
    
    // read nAMC module header words
    if (ReadXBytes( clientsockfd, nAMC*sizeof(uint64_t), (void*)( offsetheader) ); < 0) 
      {                                                                                    
	cm_msg(MERROR, __FILE__, "Cannot read data from socket, fd %d", clientsockfd);                                 
	return FE_ERR_HW;                                                                  
      }
    
    // decode AMC header words
    for (iAMC = 0; iAMC < nAMC; iAMC++){
      if ( decodeAMCHeader( iAMC, be64toh( *( offsetheader ) ) ) != 0 )
	{
	  printf("decodeAMCHeader() failed!");
	}
      offsetheader++;
      dbprintf("%s(%d): AMC index %d, AMCMoreBit %d, AMCEventSize 0x%08x\n", 
	       __func__, __LINE__, iAMC, amc_header_info[iAMC].AMCMoreBit,  amc_header_info[iAMC].AMCEventSize );
    }
    
    // calculate AMC data offsets from total event sizes in S=0 word AMC header word  
    // (i.e. for either M=1,S=0 with continuation blocks or M=0,S=0 with only one block)
    if ( !amc_header_info[0].AMCSegBit ) {

      int AMCoffsetbytes = 0;      
      for (iAMC = 0; iAMC < nAMC; iAMC++){
	dataAMCoffset[iAMC] = AMCoffsetbytes / sizeof(uint16_t);
	dbprintf("%s(%d): blockcount %d, iAMC %d, calculated AMC data offset  0x%08x\n", 
		 __func__, __LINE__, blockcount, iAMC, dataAMCoffset[iAMC]); 
	AMCoffsetbytes += sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
      }
    }
    
    // read AMC data block
    for (iAMC = 0; iAMC < nAMC; iAMC++){
      
      // calculate the data bytes to read for each AMC module
      if ( amc_header_info[0].AMCMoreBit && (!amc_header_info[0].AMCSegBit) )
	{
	  //  dbprintf("M=1,S=0 first block in segment, set size to 32kB\n");
	  databytes = 32768;
	}
      if ( amc_header_info[0].AMCMoreBit && amc_header_info[0].AMCSegBit )
	{
	  // dbprintf("M=1,S=1 intermediate block in segment, set size from amc header word\n");
	  databytes = sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
	}
      if ( (!amc_header_info[0].AMCMoreBit) && amc_header_info[0].AMCSegBit )
	{
	  //  dbprintf("M=0,S=1 last block in segment, set size from amc header word\n");
	  databytes = sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
	  
	  // TG 9/19/14 temp fix of AMC13 header problem spotted for 0x3fff0 payload - won't work for other payloads
	  if (databytes != 32664) printf("data format ERRROR! 0x3fff0 last block: databytes %d |= 32664\n",databytes);
	  databytes = 32664;
	}
      if ( (!amc_header_info[0].AMCMoreBit) && (!amc_header_info[0].AMCSegBit) )
	{
	  //  dbprintf("M=0,S=0 only block in segment, set size from amc header word\n");
	  databytes = sizeof(uint64_t)*amc_header_info[iAMC].AMCEventSize;
	}
      
      // calculated the location to put the data from block structure in AMC13 event
      dataarrayoffset = dataAMCoffset[iAMC] + datablockoffset[iAMC];
      
      // read the data blocks for each AMC module
      if ( ReadXBytes( clientsockfd, databytes, (void*)(data + dataarrayoffset) ) < 0) 
	{                                                                                    
	  cm_msg(MERROR, __FILE__, "Cannot read data from socket");                                 
	  return FE_ERR_HW;                                                                  
	}
      dbprintf("%s(%d): blockcount %d, iAMC %d, calculated AMC+Block data offset  0x%08x data bytes total 0x%08x 64-bit data 0x%16llX\n", 
	       __func__, __LINE__, blockcount, iAMC, dataarrayoffset, databytestotal, *(data + dataarrayoffset)); 
      
    dataarrayoffset +=  databytes/sizeof(uint16_t);
    datablockoffset[iAMC] +=  databytes/sizeof(uint16_t);
    databytestotal +=  databytes;
    }
    
    // read single 64-bit AMC13 general trailer word
    if ( ReadXBytes( clientsockfd, sizeof(uint64_t), (void*)( offsetheader ) ) < 0) 
      {                                                                                    
	cm_msg(MERROR, __FILE__, "Cannot read data from socket, fd %d", clientsockfd);                                 
	return FE_ERR_HW;                                                                  
      }
    
    dbprintf("%s(%d): done reading AMC block %i, trailer word *tmp 0x%08x\n", 
	     __func__, __LINE__, blockcount, *tmp); 
    
    offsetheader++;
    blockcount++;
  }
  dbprintf("%s(%d): finished data read / unpack,  databytes total  0x%08x block count %i CDF trailer 0x%08x\n", 
	   __func__, __LINE__, databytestotal, blockcount); 
  
  // get CDF trailer word
  if ( ReadXBytes( clientsockfd, tailbytes, (void*)(tail) ) < 0) 
    {                                                                                    
      cm_msg(MERROR, __FILE__, "Cannot read tail from socket");                                 
      return FE_ERR_HW;                                                                  
    }

   
  dbprintf("%s(%d): return   __func__, __LINE__ ); 
  return 0;
}

