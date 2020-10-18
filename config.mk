BLD_DIR=$(PWD)/build
BIN_DIR=$(PWD)/build/bin
INC_DIR=$(PWD)
DEP_DIR=$(PWD)/dep
TESTS_DIR=$(PWD)/tests

# COMPILER SETTINGS
CC=tcc

CFLAGS= \
	-Wall \
	-O3 \
	-D_DEFAULT_SOURCE \
	-D_POSIX_C_SOURCE=199309L \
	-I$(INC_DIR)

LIBS=-L$(BLD_DIR) \
	-lzero \
	-lpthread \
	-lm \
	-llapack \
	-L/usr/X11R6/lib -lX11 \
	-lGL -lGLU

	# -lblas \
	# -llapack \
	# -lgfortran \
	# -lgsl \

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
