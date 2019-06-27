include ../config.mk

SRCS = $(wildcard *.c) $(wildcard **/*.c)
OBJS = $(addprefix $(BLD_DIR)/, $(SRCS:.c=.o))

all: dirs $(BLD_DIR)/libzero.a

dirs:
	@mkdir -p $(BLD_DIR)/cv
	@mkdir -p $(BLD_DIR)/driver

$(BLD_DIR)/%.o : %.c
	$(COMPILE_OBJ)

$(BLD_DIR)/libzero.a: $(OBJS)
	$(MAKE_STATIC_LIB)
