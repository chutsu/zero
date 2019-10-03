CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
CPUFLAGS = -mcpu=cortex-m3 -mthumb
CFLAGS = -Wall -Wextra -g3 -O0 -MD $(CPUFLAGS) -DSTM32F1 -I../deps/libopencm3/include
LDFLAGS = $(CPUFLAGS) -nostartfiles -L../deps/libopencm3/lib -Wl,-T,stm32f1.ld
LDLIBS = -lopencm3_stm32f1 -lc -lnosys

CSRC = test.c
OBJ = $(CSRC:.c=.o)

# .PHONY: clean flash
all: flash

test.elf: $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $(OBJ) $(LDLIBS)

clean:
	@rm -f *.o *.bin *.elf *.map *.d

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@

flash: test.bin
	st-flash write test.bin 0x8000000
