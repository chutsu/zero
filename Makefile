include config.mk

default: all

all: dirs
	@make -s -C src
	@make -s -C tests

dirs:
	@mkdir -p build
	@mkdir -p bin
	@mkdir -p bin/fw

format_code:
	@bash scripts/format_code.bash

clean:
	@rm -rf build
	@rm -rf bin
