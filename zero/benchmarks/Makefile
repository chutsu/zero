include ../../config.mk

SRCS = $(wildcard *.c)
BENCHMARKS = $(addprefix $(BIN_DIR)/, $(SRCS:.c=))

all: $(BENCHMARKS)

$(BLD_DIR)/%.o: %.c $(BLD_DIR)/libzero.a
	$(COMPILE_TEST_OBJ)

$(BIN_DIR)/bench_%: $(BLD_DIR)/bench_%.o
	$(MAKE_TEST)
