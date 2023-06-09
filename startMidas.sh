#!/bin/bash
export LD_LIBRARY_PATH=/opt/cactus/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/opt/boost/lib:$LD_LIBRARY_PATH
echo "/opt/cacuts/lib and /opt/boost/lib added to LD_LIBRARY_PATH"
export MIDAS_EXPT_NAME=test4
echo "MIDAS_EXPT_NAME environment variable set to '"$MIDAS_EXPT_NAME"'"
export MIDASSYS=$HOME/packages/midas
echo "MIDASSYS environment variable set to '"$MIDASSYS"'"
export MIDAS_EXPTAB=$HOME/online/exptab
echo "MIDAS_EXPTAB environment variable set to '"$MIDAS_EXPTAB"'"
export PATH=$PATH:$MIDASSYS/bin
echo "PATH environment variable set to '"$PATH"'"
export WDBSYS=/root/Desktop/wavedaq_sw
echo "WDBSYS environment variable set to /root/Desktop/wavedaq_sw"
screen -d -m -S mserver mserver -e $MIDAS_EXPT_NAME
echo "mserver running in background..."
screen -d -m -S mhttpd mhttpd -e $MIDAS_EXPT_NAME
echo "mhttpd running in background..."
screen -d -m -S mlogger mlogger -e $MIDAS_EXPT_NAME
echo "mlogger running in background..."
screen -ls
