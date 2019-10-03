#!/bin/sh
set -e

# bash scripts/format_code.bash

# make clean
# make format_code
# make

# ./bin/test_core
# ./bin/test_data


cd firmware
make clean
# make blink.o
make blink.elf
make flash
