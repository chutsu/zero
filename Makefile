include config.mk

default: all

all: dirs
	@make -s -C zero
	@make -s -C zero/tests

clean:
	@rm -rf $(BLD_DIR)

dirs:
	@mkdir -p $(BLD_DIR)
	@mkdir -p $(BIN_DIR)

format_code:
	@bash scripts/format_code.bash
