#!/bin/bash
set -e
BUILD_TYPE=Release
INSTALL_PREFIX=${PWD}

# Download repo
if [ ! -d OpenBLAS ]; then
	git clone https://github.com/xianyi/OpenBLAS
fi

# Build
cd OpenBLAS
make
