.PHONY: all gcc_arm

all: gcc_arm libopencm3 stlink

gcc_arm:
	@echo "[Installing gcc-arm]"
	@sudo apt-get install gcc-arm-none-eabi -y -qqq

libopencm3:
	@echo "[Installing libopencm3]"
	@git clone https://github.com/libopencm3/libopencm3 \
		&& cd libopencm3 \
		&& make > /dev/null

stlink:
	@echo "[Installing stlink]"
	@git clone https://github.com/texane/stlink \
		&& cd stlink \
		&& make > /dev/null \
		&& cd build/Release && sudo make install

clean:
	@sudo apt-get purge gcc-arm-none-eabi
	@rm -rf libopencm3
	@rm -rf stlink
