#!/bin/sh
cd /home/daq
source ~/.bash_profile
cd /home/daq/gm2daq/frontends/CaloReadoutAMC13
./frontend -e CR -h g2be1-priv -i $1



