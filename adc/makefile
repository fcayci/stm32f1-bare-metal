TARGET = adc
SRCS = adc.c

OBJS =  $(addsuffix .o, $(basename $(SRCS)))
INCLUDES = -I.

LINKER_SCRIPT = stm32.ld

CFLAGS += -mcpu=cortex-m3 -mthumb # Processor setup
CFLAGS += -O0  # Optimization is off
#CFLAGS += -g3  # Generate debug information
CFLAGS += -fno-common -Wall
CFLAGS += -ffunction-sections -fdata-sections -Wl,--gc-sections

LDFLAGS += -march=armv7-m
LDFLAGS += -nostartfiles
LDFLAGS += --specs=nosys.specs
LDFLAGS += -T$(LINKER_SCRIPT)

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJDUMP = $(CROSS_COMPILE)objdump
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size

all: clean $(SRCS) build size
	@echo "Successfully finished..."

build: $(TARGET).elf $(TARGET).hex $(TARGET).bin $(TARGET).lst

$(TARGET).elf: $(OBJS)
	@$(CC) $(LDFLAGS) $(OBJS) -o $@

%.o: %.c
	@echo "Building" $<
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

%.o: %.s
	@echo "Building" $<
	@$(CC) $(CFLAGS) -c $< -o $@

%.hex: %.elf
	@$(OBJCOPY) -O ihex $< $@

%.bin: %.elf
	@$(OBJCOPY) -O binary $< $@

%.lst: %.elf
	@$(OBJDUMP) -x -S $(TARGET).elf > $@

size: $(TARGET).elf
	@$(SIZE) $(TARGET).elf

burn:
	@st-flash write $(TARGET).bin 0x8000000

clean:
	@echo "Cleaning..."
	@rm -f $(TARGET).elf
	@rm -f $(TARGET).bin
	@rm -f $(TARGET).map
	@rm -f $(TARGET).hex
	@rm -f $(TARGET).lst
	@rm -f $(TARGET).o

.PHONY: all build size clean burn
