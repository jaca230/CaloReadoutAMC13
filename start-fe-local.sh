#!/bin/sh

# usage:
# start-fe.sh [experiment name] [host name] [frontend index]  [locale (optional)]
# ./start-fe-local.sh TEST gm2_daq@gm26221.classe.cornell.edu 1 CU

if [ $# -eq 2  ]; then
   source /home/daq/gm2daq/setup.sh
else
   if [ "$3" = "CU" ]; then
      source /mnt/gm26221data/teststand_sl7/gm2daq/setupCU.sh
   else
      exit 1
   fi
fi

cd $GM2DAQ_DIR/frontends/CaloReadoutAMC13
./frontend -e $1 -i $2
