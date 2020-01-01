.PHONY: all gcc_arm

all: openblas

openblas:
	@echo "[Installing openblas]"
	@git clone https://github.com/xianyi/OpenBLAS \
		&& cd OpenBLAS \
		&& make > /dev/null

clean:
	@rm -rf openblas
