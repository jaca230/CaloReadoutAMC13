#!/bin/sh

# usage:
# start-fe.sh [experiment name] [host name] [frontend index]  [locale (optional)]

if [ $# -eq 3  ]; then
   echo "default script is already sourced in .bashrc"
#source /home/daq/gm2daq/setup.sh
else
   if [ "$4" = "CU" ]; then
      source /mnt/gm26221data/fnal_teststand_replica/gm2daq/setupCU.sh
   else
      exit 1
   fi
fi

cd $GM2DAQ_DIR/frontends/CaloReadoutAMC13
./frontend -e $1 -h $2 -i $3
