#
# Makefile for the CaloReadoutAMC13 
# Modified from Midas/example/experiment by Ran Hong
# 3/12/2020
#

#Frontend configuration

USE_CALO_SIMULATOR = 0
USE_PARALLEL_PORT  = 0
USE_GPU            = 0
DEBUG              = 0

# The MIDASSYS should be defined prior the use of this Makefile
ifndef MIDASSYS
missmidas::
	@echo "...";
	@echo "Missing definition of environment variable 'MIDASSYS' !";
	@echo "...";
endif

# get OS type from shell
OSTYPE = $(shell uname)

# Linux settings
ifeq ($(OSTYPE),Linux)
OSTYPE = linux
else
unsupportedos::
	@echo "This frontend works only on Linux!";
endif

ifeq ($(OSTYPE),linux)
LIBS = -lm -lz -lutil -lnsl -lpthread -lrt
endif

#-----------------------------------------
# NO ROOT
# Root related lines are deleted
#

#
#--------------------------------------------------------------------
# The GM2DAQ_DIR should be defined prior the use of this Makefile
ifndef GM2DAQ_DIR
missdaq::
	@echo "...";
	@echo "Missing definition of environment variable 'GM2DAQ_DIR' !";
	@echo "...";
endif

#
#--------------------------------------------------------------------
# The CACTUS_ROOT should be defined prior the use of this Makefile
ifndef CACTUS_ROOT
misscactus::
	@echo "...";
	@echo "Missing definition of environment variable 'CACTUS_ROOT' !";
	@echo "...";
endif

# Note: Do not delete VPATH variable
VPATH = ../common

# location of MIDAS things
INC_DIR = $(MIDASSYS)/include
LIB_DIR = $(MIDASSYS)/lib

# midas library
#LIB = $(LIB_DIR)/libmidas-shared.so
LIB = $(LIB_DIR)/libmidas.a
#LIB += /lib64/libreadline.so.6

# CXX FLAGS
CXXFLAGS = -g -std=c++14 -pthread -D_REENTRANT

#Added by Jack Carlton 06/01/2023 to "make things work"
BOOST_ROOT=/opt/boost
CXXFLAGS += -I$(MIDASSYS)/include

#CXXFLAGS += -O2 -Wextra -Wno-missing-field-initializers -Wno-unused-local-typedefs
CXXFLAGS += -O2 -Wextra -Wno-missing-field-initializers -Wno-unused-local-typedefs -Wno-deprecated-declarations -Wno-parentheses
# CUDA Flags
CXXFLAGS += -I$(CUDASYS)/include 
# Include directories
CXXFLAGS += -I../common
CXXFLAGS += -I${CACTUS_ROOT}/include
#CXXFLAGS += -I/usr/include/
#CXXFLAGS += -I/diskless/SL60/x86_64/root/usr/include
#BOOST_ROOT=/opt/boost
#BOOST_ROOT=/home/daq/Packages/boost
CXXFLAGS += -I$(BOOST_ROOT)/include
CXXFLAGS += -I$(CACTUS_ROOT)/include 
#CXXFLAGS += -I/diskless/SL60/x86_64/root/usr/include 
CXXFLAGS += -I$(GM2DAQ_DIR)/frontends/common 
CXXFLAGS += -I/usr/include/openssl
CXXFLAGS += -I/usr/include/pugixml
#CXXFLAGS +=-I$(GM2DAQ_DIR)/md5


#Link Flags
LDFLAGS  = -L$(CUDASYS)/lib64 -L$(BOOST_ROOT)/lib -lboost_filesystem -lboost_system
LDFLAGS += -L/opt/boost/lib -L/opt/cactus/lib
LDFLAGS += -lboost_filesystem -lboost_regex -lboost_system -lboost_thread -lcactus_uhal_log -lcactus_uhal_grammars -lcactus_uhal_uhal
LDFLAGS += -L/usr/lib64 -lpugixml -ldl

# activate debug print statements
ifeq ($(DEBUG),1)
CXXFLAGS += -DDEBUG
endif

SRC_DIR = $(MIDASSYS)/src

# CACTUS - Taken from: https://svnweb.cern.ch/trac/cactus/export/554/trunk/doc/ExampleMakefile
CACTUS_LIBS = -L${CACTUS_ROOT}/lib
CACTUS_LIBS += \
               -lboost_filesystem \
               -lboost_regex \
               -lboost_system \
               -lboost_thread \
               \
               -lcactus_uhal_log \
               -lcactus_uhal_grammars \
               -lcactus_uhal_uhal
#               -lcactus_extern_pugixml
LDFLAGS += $(CACTUS_LIBS)

#
# RING_BUFFER
#
ifeq ($(USE_RING_BUFFER),1)
CXXFLAGS += -DUSE_RING_BUFFER
endif 

#
# PARALLEL_PORT
#
ifeq ($(USE_PARALLEL_PORT),1)
CXXFLAGS += -DUSE_PARALLEL_PORT
endif 

#-------------------------------------------------------------------
# List of modules
#
# gpu_thread.o

MODULES = frontend_aux.o \
	timetool.o \
	amc13_odb.o \
	gpu_thread.o \
	tcp_thread.o \
	fe_compress_z_mt.o \
	AMC13.o \
	FC7.o \
	WFD5.o \
	laser_config_handler.o 
#       frontend_rpc.o \
#	../../md5/md5c.o
#	../AMC13Simulator/amc13simulator_odb.o 

#
# GPU hardware use
#
ifeq ($(USE_GPU),1)
CXXFLAGS += -DUSE_GPU
MODULES  += cuda_tools_g2.o \
        kernel.cu.o
include cuda.mk
endif

# compiler settings
#CFLAGS += -O2 -g -Wall -Wformat=2 -Wno-format-nonliteral -Wno-strict-aliasing -Wuninitialized -Wno-unused-function
#CFLAGS += -I$(INC_DIR)
#LDFLAGS +=
CC = gcc
CXX = g++
LDFLAGS += -lcrypto


# list of programs to build
PROGS:=
PROGS+= frontend

all: $(PROGS)

blank:
	touch blank

include blank $(wildcard *.d)

$(PROGS): $(LIB) $(LIB_DIR)/libmfe.a $(MODULES) $(PROGS).cpp Makefile
	$(CXX) $(CXXFLAGS) -o $(PROGS) \
	$(PROGS).cpp $(MODULES) $(LIB_DIR)/libmfe.a $(LIB) \
	$(LDFLAGS) $(LIBS)

%.o: %.cxx
	$(CXX) $(USERFLAGS) $(CXXFLAGS) -o $@ -c $<
	$(CXX) -MM $(CXXFLAGS) $< > $(notdir $(patsubst %.cxx,%.d,$<))

%.o: %.c 
	$(CXX) $(USERFLAGS) $(CXXFLAGS) -o $@ -c $<
	$(CXX) -MM $(CXXFLAGS) $< > $(notdir $(patsubst %.c,%.d,$<))


##############

clean::
	rm -f *.o *.d *~ ../CaloSimulatorAMC13/tcpsimulator_odb.o \ ../CaloSimulatorAMC13/tcpsimulator_odb.d

#end file
