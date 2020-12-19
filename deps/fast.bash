#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Download repo
if [ ! -d fast ]; then
  git clone https://github.com/edrosten/fast-C-src
fi
