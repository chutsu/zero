include config.mk

default: all

gcc_arm:
	@echo "[Installing gcc-arm]"
	@sudo apt-get install gcc-arm-none-eabi -y -qqq

libopencm3:
	@echo "[Installing libopencm3]"
	@cd deps/libopencm3 && make > /dev/null

stlink:
	@echo "[Installing stlink]"
	@cd deps/stlink && make > /dev/null

all: dirs
	@make -s -C src
	@make -s -C tests

dirs:
	@mkdir -p dep
	@mkdir -p build
	@mkdir -p bin

format_code:
	@bash scripts/format_code.bash

clean:
	@rm -rf dep
	@rm -rf build
	@rm -rf bin
