BIN_DIR=$(PWD)/bin
BLD_DIR=$(PWD)/build
INC_DIR=$(PWD)/include

# COMPILER SETTINGS
CC=gcc -Wall -g
CFLAGS=-I$(INC_DIR)
LIBS=-lm

# ARCHIVER SETTTINGS
AR = ar
ARFLAGS = rvs

# COMPILER COMMANDS
COMPILE_OBJ = \
	@echo "CC [$^]"; \
	$(CC) $(CFLAGS) -c $^ -o $@

MAKE_STATIC_LIB = \
	$(AR) $(ARFLAGS) $@ $^

# MAKE_TEST = \
# 	echo "TEST [${@:.o=}]"; \
# 	$(CC) $(CFLAGS) -c ${@:.o=.cpp} -o $@; \
# 	$(CC) $(CFLAGS) $@ \
# 		-o $(addprefix $(BIN_DIR)/, ${@:.o=}) \
# 		-L. $(LIBS)

# MAKE_EXE = \
# 	@echo "EXE [$@]"; \
# 	$(CC) $(CFLAGS) $@.o \
# 		-o $(addprefix $(BIN_DIR)/, $@) \
# 		-L. -lzp3 $(LIBS)
