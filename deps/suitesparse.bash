#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Download repo
if [ ! -d SuiteSparse ]; then
  git clone https://github.com/DrTimothyAldenDavis/SuiteSparse
fi

# Install Deps
sudo apt-get install libgmp3-dev -y -qq
sudo apt-get install libmpfr-dev libmpfr-doc -y -qq

# Build
cd SuiteSparse
make library
make install INSTALL=$INSTALL_PREFIX
