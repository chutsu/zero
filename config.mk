BLD_DIR=$(PWD)/build
BIN_DIR=$(PWD)/build/bin
INC_DIR=$(PWD)
DEP_DIR=$(PWD)/dep
TESTS_DIR=$(PWD)/tests

# COMPILER SETTINGS
CC=gcc -Wall -O2 -g -std=c11 \
	-march=native \
	-D_DEFAULT_SOURCE \
	-D_POSIX_C_SOURCE=199309L
# CC=gcc -O3 -s -g -Wall -DNDEBUG \
# 	-std=gnu99 \
# 	-march=native \
# 	-fopenmp \
# 	-D_DEFAULT_SOURCE \
# 	-D_POSIX_C_SOURCE=199309L
CFLAGS=-I$(INC_DIR) -I/usr/include/x86_64-linux-gnu
LIBS=-L$(BLD_DIR) \
		 -lzero \
		 -L./deps/OpenBLAS/lapack/ -llapack \
		 -L./deps/OpenBLAS/lapack/ -llapacke \
		 -L./deps/OpenBLAS/ -lopenblas \
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
