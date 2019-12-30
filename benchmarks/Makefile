CC := gcc
CXX := g++
CFLAGS:= -Wall -O3 -march=native -fopenmp
INCS := -I/usr/include/eigen3
LIBS := -lopenblas -lm

.PHONY: all
all: bench_matmul

bench_matmul: bench_matmul-eigen.cpp
	@echo "CXX [$<]"; $(CXX) $(CFLAGS) $< -o $(BIN_DIR)/$@ $(INCS) $(LIBS)
