include ../config.mk

SRCS = $(wildcard *.c)
OBJS = $(addprefix $(BLD_DIR)/, $(SRCS:.c=.o))
LIB = $(BLD_DIR)/libzero.a

.PHONY: all tests dirs

default: dirs $(LIB)

tests:
	@make -C tests

dirs:
	@mkdir -p $(BLD_DIR)

$(BLD_DIR)/%.o: %.c %.h
	$(COMPILE_OBJ)

$(BLD_DIR)/libzero.a: $(OBJS)
	$(MAKE_STATIC_LIB)
