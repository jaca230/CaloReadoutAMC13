#!/bin/sh

# usage:
# start-fe-testStand.sh [frontend index] [locale (optional)]

MHOST=g2calo-test.fnal.gov
if [ $# == 1  ]; then
   source /home/daq/gm2daq/setup.sh
else
   if [ "$2" = "CU" ]; then
      echo "using CU setup"
      source /mnt/gm26221data/teststand_sl7/gm2daq/setupCU.sh
      MHOST=gm26221.classe.cornell.edu
   else
      exit 1
   fi
fi
cd $GM2DAQ_DIR/frontends/CaloReadoutAMC13
./frontend -e TEST -h $MHOST -i $1
