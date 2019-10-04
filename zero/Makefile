include ../config.mk

SRCS = $(wildcard *.c)
OBJS = $(addprefix $(BLD_DIR)/, $(SRCS:.c=.o))

.PHONY: all tests dirs

all: dirs $(BLD_DIR)/libzero.a

tests:
	@make -C tests

dirs:
	@mkdir -p $(BLD_DIR)

$(BLD_DIR)/%.o : %.c
	$(COMPILE_OBJ)

$(BLD_DIR)/libzero.a: $(OBJS)
	$(MAKE_STATIC_LIB)
