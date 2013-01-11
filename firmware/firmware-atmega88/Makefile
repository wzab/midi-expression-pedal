###############################################################################
# Makefile for the project V-USB-MIDI
###############################################################################

## General Flags
PROJECT = midimega
MCU = atmega88
TARGET = $(PROJECT).elf
#DEBUG =  -DDEBUG_LEVEL=2
CC = avr-gcc
AVRDUDE = avrdude -c usbasp -B 3 -p$(MCU)
#AVRDUDE = avrdude -c blaster -P /dev/parport0 -p$(MCU)
#AVRDUDE = avrdude -c xil -P /dev/parport1 -p$(MCU)

FUSEE = 0xf9
FUSEH = 0xdd
FUSEL = 0xff
# Fuse exteded byte:
# Fuse high byte:
# 0xdd = 1 1 0 1   1 1 0 1 <-- BODLEVEL0
#        ^ ^ ^ ^   ^ ^ ^------ BODLEVEL1
#        | | | |   | +-------- BODLEVEL2
#        | | | |   + --------- EESAVE (preserve EEPROM over chip erase)
#        | | | +-------------- WDTON (watchdog timer always on)
#        | | +---------------- SPIEN (allow serial programming)
#        | +------------------ DWEN (debugWIRE Enable)
#        +-------------------- RSTDISBL (reset pin is enabled)
# Fuse low byte:
# 0xff = 1 1 1 1   1 1 1 1
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (external >8M crystal)
#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
#        | +------------------ CKOUT (Clock output)
#        +-------------------- CKDIV8 (Divide clock by 8, disabled)

## Options common to compile, link and assembly rules
COMMON = -g -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -DF_CPU=12000000UL -Os -fsigned-char  $(DEBUG)

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += -x assembler-with-cpp -Wa,

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=$(PROJECT).map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom


## Include Directories
INCLUDES = -I"." -I"../usbdrv" -I"../." 

## Objects that must be built in order to link
OBJECTS = usbdrv.o usbdrvasm.o oddebug.o main.o

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) $(PROJECT).hex $(PROJECT).lss size

$(OBJECTS): usbconfig.h Makefile

## Compile
usbdrv.o: ../usbdrv/usbdrv.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

usbdrvasm.o: ../usbdrv/usbdrvasm.S
	$(CC) $(INCLUDES) $(ASMFLAGS) -c  $<

oddebug.o: ../usbdrv/oddebug.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

main.o: main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@./checksize ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) $(PROJECT).* *~


.PHONY: flash
flash:	all
	$(AVRDUDE) -U flash:w:$(PROJECT).hex


.PHONY: fuse
fuse:
	$(AVRDUDE) -U efuse:w:$(FUSEE):m -U hfuse:w:$(FUSEH):m -U lfuse:w:$(FUSEL):m

