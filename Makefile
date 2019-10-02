include config.mk

default: all

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
