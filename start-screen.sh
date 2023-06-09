#!/bin/sh

# usage:
# start-screen.sh [experiment name] [screen name] [screen host] [host name] [frontend index]

screen -ls | grep $2 && screen -S $2 -X quit
screen -d -m -S $2 ssh -t $3 "$GM2DAQ_DIR/frontends/CaloReadoutAMC13/start-fe.sh $1 $4 $5; exec bash"
