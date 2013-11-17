# The name of your project (used to name the compiled .hex file)
TARGET = $(notdir $(CURDIR))

# configurable options
OPTIONS = -DF_CPU=48000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH

# options needed by many Arduino libraries to configure for Teensy 3.0
OPTIONS += -D__MK20DX128__ -DARDUIO=104


#************************************************************************
# Location of Teensyduino utilities, Toolchain, and Arduino Libraries.
# To use this makefile without Arduino, copy the resources from these
# locations and edit the pathnames.  The rest of Arduino is not needed.
#************************************************************************

# path location for Teensy Loader, teensy_post_compile and teensy_reboot
TOOLSPATH = teensy3/tools

# path location for the arm-none-eabi compiler
COMPILERPATH = $(TOOLSPATH)/arm-none-eabi/bin

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -g -Os -mcpu=cortex-m4 -mthumb -nostdlib -MMD $(OPTIONS) -I. -Iteensy3

# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS =

LDSCRIPT = teensy3/mk20dx128.ld

# linker options
LDFLAGS = -Os -Wl,--gc-sections -mcpu=cortex-m4 -mthumb -T$(LDSCRIPT)

# additional libraries to link
LIBS = -lm

# names for the compiler programs
CC = $(abspath $(COMPILERPATH))/arm-none-eabi-gcc
CXX = $(abspath $(COMPILERPATH))/arm-none-eabi-g++
OBJCOPY = $(abspath $(COMPILERPATH))/arm-none-eabi-objcopy
SIZE = $(abspath $(COMPILERPATH))/arm-none-eabi-size

# automatically create lists of the sources and objects
TC_FILES := $(wildcard teensy3/*.c)
TCPP_FILES := $(wildcard teensy3/*.cpp)
C_FILES := $(wildcard *.c)
CPP_FILES := $(wildcard *.cpp)
OBJS := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) $(TC_FILES:.c=.o) $(TCPP_FILES:.cpp=.o)

all: hex

build: $(TARGET).elf

hex: $(TARGET).hex

post_compile: $(TARGET).hex
	@$(abspath $(TOOLSPATH))/teensy_post_compile -file=$(basename $<) -path=$(shell pwd) -tools=$(abspath $(TOOLSPATH))

reboot:
	@-$(abspath $(TOOLSPATH))/teensy_reboot

upload: post_compile reboot

%.o: %.c
	@echo "[CC]\t$<"
	@$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ -c $<

%.o: %.cpp
	@echo "[CXX]\t$<"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o $@ -c $<

$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	@echo "[LD]\t$@"
	@$(CC) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

%.hex: %.elf
	@echo "[HEX]\t$@"
	@$(SIZE) $<
	@$(OBJCOPY) -O ihex -R .eeprom $< $@


# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	@echo Cleaning...
	@rm -f teensy3/*.o teensy3/*.d *.o *.d $(TARGET).elf $(TARGET).hex
