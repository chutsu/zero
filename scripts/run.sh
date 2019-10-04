#!/bin/sh
set -e

# bash scripts/format_code.bash

# make format_code
make clean
make

# ./bin/test_core
# ./bin/test_data

cd firmware
make clean
make firmware.bin
# make flash
