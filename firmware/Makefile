CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

CFLAGS  = -std=gnu99 -g -O2 -Wall
CFLAGS += --specs=nosys.specs
CFLAGS += -O
CFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m0
CFLAGS += -fsingle-precision-constant -Wdouble-promotion

blink.elf: blink.c
	$(CC) \
    -Tstm32f1.ld \
		$(CFLAGS) \
		-I../deps/libopencm3/include \
		-L../deps/libopencm3/lib \
		$^ -o $@
	@$(OBJCOPY) -Obinary blink.elf blink.bin

erase:
		@sudo ../deps/stlink/build/Release/st-flash erase

flash: blink.bin
		@sudo ../deps/stlink/build/Release/st-flash write blink.bin 0x8000000

clean:
	@rm -f *.elf
	@rm -f *.o
	@rm -f *.bin
