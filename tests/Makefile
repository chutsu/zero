include ../config.mk

SRCS = $(wildcard *.c) $(wildcard **/*.c)
TESTS = $(addprefix $(BIN_DIR)/, $(SRCS:.c=))

all: $(TESTS)

$(BLD_DIR)/%.o : %.c $(BLD_DIR)/libzero.a
	$(COMPILE_TEST_OBJ)

$(BIN_DIR)/test_% : $(BLD_DIR)/test_%.o
	$(MAKE_TEST)
