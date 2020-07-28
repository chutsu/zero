include config.mk

.PHONY: format_code zero tests
default: $(BLD_DIR) $(BIN_DIR) zero tests

clean:
	@rm -rf $(BLD_DIR)

$(BLD_DIR):
	@mkdir -p $(BLD_DIR)

$(BIN_DIR):
	@mkdir -p $(BIN_DIR)

format_code:
	@bash scripts/format_code.bash

zero:
	@make -s -C zero

tests:
	@make -s -C zero/tests
