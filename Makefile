include config.mk

all: dirs
	@make -s -C src

dirs:
	@mkdir -p build

format_code:
	@bash scripts/format_code.bash

clean:
	@rm -rf build
