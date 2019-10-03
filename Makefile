include config.mk

default: all

all: dirs
	@make -s -C src
	@make -s -C tests

dirs:
	@mkdir -p $(BLD_DIR)
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(BIN_DIR)/fw

format_code:
	@bash scripts/format_code.bash

clean:
	@rm -rf $(BLD_DIR)
