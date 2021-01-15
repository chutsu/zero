BLD_DIR=$(PWD)/build
BIN_DIR=$(PWD)/build/bin
INC_DIR=$(PWD)
DEP_DIR=$(PWD)/dep
TESTS_DIR=$(PWD)/tests

# COMPILER SETTINGS
CC=tcc
# CC=gcc
CFLAGS=-g -O3 -Wall -I$(INC_DIR) -pedantic

LIBS=-L$(BLD_DIR) \
	-lzero \
	-lblas \
	-llapack \
	-lpthread \
	-lm \
	-L/usr/X11R6/lib -lX11 \
	-lX11 -lXi -lXmu -lglut -lGL -lGLU -lGLEW

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
