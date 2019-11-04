include ../../config.mk

CC := gcc
CXX := g++
CFLAGS:= -Wall -O3 -march=native -fopenmp
INCS := -I$(INC_DIR) -I/usr/include/eigen3
LIBS := -L$(BLD_DIR) -lzero -lopenblas -lm

.PHONY: all
all: bench_matmul_zero bench_matmul_eigen

bench_matmul_zero: bench_matmul_zero.c
	@echo "CC [$<]"; \
		$(CC) $(CFLAGS) $< -o $(BIN_DIR)/$@ $(INCS) $(LIBS)

bench_matmul_eigen: bench_matmul_eigen.cpp
	@echo "CXX [$<]"; \
		$(CXX) $(CFLAGS) $< -o $(BIN_DIR)/$@ $(INCS) $(LIBS)
