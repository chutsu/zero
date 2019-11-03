BLD_DIR=$(PWD)/build
BIN_DIR=$(PWD)/build/bin
INC_DIR=$(PWD)
DEP_DIR=$(PWD)/dep
TESTS_DIR=$(PWD)/tests

# COMPILER SETTINGS
CC=gcc -Wall -g -std=c11
CFLAGS=-I$(INC_DIR)
LIBS=-L$(BLD_DIR) \
		 -lzero \
		 -lm

# ARCHIVER SETTTINGS
AR = ar
ARFLAGS = rvs

# COMPILER COMMANDS
COMPILE_OBJ = \
	@echo "CC [$^]"; \
	$(CC) $(CFLAGS) -c $^ -o $@

MAKE_STATIC_LIB = \
	$(AR) $(ARFLAGS) $@ $^

COMPILE_TEST_OBJ = \
	$(CC) $(CFLAGS) -c $< -o $@

MAKE_TEST = \
	echo "TEST [$@]"; \
	$(CC) $(CFLAGS) \
		$(subst $(BIN_DIR), $(BLD_DIR), $@.o) \
		-o $@ \
		$(LIBS)
