BIN_DIR=$(PWD)/bin
BLD_DIR=$(PWD)/build
INC_DIR=$(PWD)/include
DEP_DIR=$(PWD)/dep
TESTS_DIR=$(PWD)/tests

# COMPILER SETTINGS
CC=gcc -Wall -g
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

# MAKE_EXE = \
# 	@echo "EXE [$@]"; \
# 	$(CC) $(CFLAGS) $@.o \
# 		-o $(addprefix $(BIN_DIR)/, $@) \
# 		-L. -lzp3 $(LIBS)
