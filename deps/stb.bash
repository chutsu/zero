#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Download repo
if [ ! -d stb ]; then
  git clone https://github.com/nothings/stb
fi

# Copy to include
cp -R stb/*.h include
