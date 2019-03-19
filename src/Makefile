include ../config.mk

SRCS = $(wildcard *.c)
OBJS = $(addprefix $(BLD_DIR)/, $(SRCS:.c=.o))

all: $(BLD_DIR)/libzero.a

$(BLD_DIR)/libzero.a: $(OBJS)
	$(MAKE_STATIC_LIB)

$(BLD_DIR)/%.o : %.c
	$(COMPILE_OBJ)
