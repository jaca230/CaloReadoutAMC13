#!/bin/bash

# Step 1: Declare GM2DAQ_DIR environment variable
export GM2DAQ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Step 2: Declare CACTUS_ROOT environment variable
export CACTUS_ROOT="/opt/cactus"

# Step 3: Execute the enable scripts
source /opt/rh/devtoolset-8/enable
source /opt/rh/rh-python36/enable
