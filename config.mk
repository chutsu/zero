BLD_DIR=$(PWD)/build
BIN_DIR=$(PWD)/build/bin
INC_DIR=$(PWD)
DEP_DIR=$(PWD)/dep
TESTS_DIR=$(PWD)/tests

# COMPILER SETTINGS
CC=gcc \
	-Wall \
	-O3 \
	-DNDEBUG \
	-march=native \
	-fopenmp \
	-D_DEFAULT_SOURCE \
	-D_POSIX_C_SOURCE=199309L
CFLAGS=-I$(INC_DIR)
LIBS=-L./deps/OpenBLAS/build/lib -L$(BLD_DIR) \
	-lzero \
	-lopenblas \
	-llapacke \
	-lpthread \
	-lm

# ARCHIVER SETTTINGS
AR = ar
ARFLAGS = rvs

# COMPILER COMMANDS
COMPILE_OBJ = \
	@echo "CC [$<]"; \
	$(CC) $(CFLAGS) -c $< -o $@

MAKE_STATIC_LIB = \
	@echo "AR [libzero.a]"; \
	$(AR) $(ARFLAGS) $@ $^

COMPILE_TEST_OBJ = \
	$(CC) $(CFLAGS) -c $< -o $@

MAKE_TEST = \
	echo "TEST [$(shell basename $@)]"; \
	$(CC) $(CFLAGS) \
		$(subst $(BIN_DIR), $(BLD_DIR), $@.o) \
		-o $@ \
		$(LIBS)
