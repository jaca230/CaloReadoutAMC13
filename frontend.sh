#!/bin/bash

source /etc/profile.d/MIDAS.sh

echo "huh"

#/home/daq/DAQ/frontends/SIS3350MT/frontend -e MTest -h be 
/home/daq/DAQ/frontends/CaloReadoutAMC13/frontend $*


