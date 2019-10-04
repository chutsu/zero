CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
CPUFLAGS = -mcpu=cortex-m3 -mthumb
CFLAGS = -Wall -Wextra -g3 -O0 -MD $(CPUFLAGS) -DSTM32F1 -I../deps/libopencm3/include
LDFLAGS = $(CPUFLAGS) -nostartfiles -L../deps/libopencm3/lib -Wl,-T,stm32f1.ld
LDLIBS = -lopencm3_stm32f1 -lc -lnosys

.PHONY: clean flash

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

default: $(OBJS) firmware.bin

clean:
	@rm -f *.o *.bin *.elf *.map *.d

%.o: %.c
	@echo "CC [$@]"
	@$(CC) $(CFLAGS) -c $^ -o $@

firmware.elf: $(OBJS)
	@echo "ELF [$@]"
	@$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

firmware.bin: firmware.elf
	@echo "BIN [$@]"
	@$(OBJCOPY) -O binary $< $@

flash: firmware.bin
	@echo "FLASHING [$^]"
	@st-flash write firmware.bin 0x8000000
