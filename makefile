GFXLIB = uGFX

GFXSINGLEMAKE = yes
GFXSRC += $(GFXLIB)/src/gfx.c
include $(GFXLIB)/src/gdriver/gdriver.mk
include $(GFXLIB)/src/gos/gos.mk
include $(GFXLIB)/src/gdisp/gdisp.mk
include $(GFXLIB)/src/gfile/gfile.mk
include $(GFXLIB)/drivers/gdisp/ILI9341/driver.mk

TARGET = tft
SRCS +=  tft.c $(GFXSRC)

OBJS =  $(addsuffix .o, $(basename $(SRCS)))
SOBJS =  $(addsuffix .o, $(notdir $(basename $(SRCS))))

INCDIRS += .
GFXINC  += $(GFXLIB)
INCDIRS += $(GFXINC)
INCLUDES =$(foreach d, $(INCDIRS), -I$d)

LINKER_SCRIPT = stm32.ld

CFLAGS += -mcpu=cortex-m3 -mthumb # Processor setup
CFLAGS += -O0 # Optimization is off
CFLAGS += -g2 # Generate debug info
CFLAGS += -fno-common
CFLAGS += -Wall # Turn on warnings
#CFLAGS += -pedantic # more warnings
CFLAGS += -Wsign-compare
CFLAGS += -mfloat-abi=soft -fsingle-precision-constant
CFLAGS += -Wcast-align
#CFLAGS += -Wconversion # neg int const implicitly converted to uint
CFLAGS += -fomit-frame-pointer # Do not use fp if not needed
CFLAGS += -ffunction-sections -fdata-sections

LDFLAGS += -march=armv7-m
LDFLAGS += -nostartfiles
LDFLAGS += --specs=nosys.specs
LDFLAGS += -lnosys
LDFLAGS += -Wl,--gc-sections # Linker garbage collector
LDFLAGS += -T$(LINKER_SCRIPT)

CROSS_COMPILE = arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJDUMP = $(CROSS_COMPILE)objdump
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
DBG = $(CROSS_COMPILE)gdb

all: clean $(SRCS) build size
	@echo "Successfully finished..."

build: $(TARGET).elf $(TARGET).hex $(TARGET).bin $(TARGET).lst

$(TARGET).elf: $(OBJS)
	@$(CC) $(SOBJS) $(LDFLAGS) -o $@

%.o: %.c
	@echo "Building" $<
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $(notdir $@)

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

debug:
	@$(DBG) -tui --eval-command="target extended-remote :4242" \
	--eval-command="layout asm" \
	--eval-command="layout regs" \
	 $(TARGET).elf

clean:
	@echo $(INCLUDES)
	@echo "Cleaning..."
	@rm -f $(TARGET).elf
	@rm -f $(TARGET).bin
	@rm -f $(TARGET).map
	@rm -f $(TARGET).hex
	@rm -f $(TARGET).lst
	@rm -f $(TARGET).o
	@rm -f *.o

.PHONY: all build size clean burn debug
