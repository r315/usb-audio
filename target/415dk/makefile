######################################
# target
######################################
TARGET =usb-audio

#######################################
# Features
#######################################

FEATURE_TFT_DISPLAY :=yes

#######################################
# paths
#######################################

BUILD_DIR 		:=build
TARGET_PATH 	=$(CURDIR)

LIB_PATH        =../../lib
DRIVERS_PATH 	=../../Driver
CMSIS_PATH 		=$(DRIVERS_PATH)/CMSIS
DRIVERS_PER_PATH =$(DRIVERS_PATH)/AT32F4xx

#######################################
# Includes
#######################################
C_INCLUDES = \
$(TARGET_PATH)/inc \
$(DRIVERS_PER_PATH)/inc \
$(CMSIS_PATH)/Core/Include \
$(CMSIS_PATH)/Device/ARTERY/AT32F4xx/inc \
$(LIB_PATH)/usb-at/usb_drivers/inc \
$(LIB_PATH)/usb-at/usbd_class/audio_hid \


AS_INCLUDES =\

######################################
# Sources
######################################
DRIVER_SOURCES = \
$(CMSIS_PATH)/Device/ARTERY/AT32F4xx/src/startup_at32f415cbt7.c\
$(CMSIS_PATH)/Device/ARTERY/AT32F4xx/src/system_at32f415.c\
$(DRIVERS_PER_PATH)/src/at32f415_crm.c\
$(DRIVERS_PER_PATH)/src/at32f415_i2c.c\
$(DRIVERS_PER_PATH)/src/at32f415_spi.c\
$(DRIVERS_PER_PATH)/src/at32f415_gpio.c\
$(DRIVERS_PER_PATH)/src/at32f415_dma.c\
$(DRIVERS_PER_PATH)/src/at32f415_usb.c\
$(DRIVERS_PER_PATH)/src/at32f415_tmr.c\
$(DRIVERS_PER_PATH)/src/at32f415_misc.c\
$(LIB_PATH)/usb-at/usb_drivers/src/usb_core.c \
$(LIB_PATH)/usb-at/usb_drivers/src/usbd_core.c \
$(LIB_PATH)/usb-at/usbd_class/audio_hid/audio_hid_desc.c \
$(LIB_PATH)/usb-at/usbd_class/audio_hid/audio_hid_class.c \

C_SOURCES = \
$(DRIVER_SOURCES) \
$(TARGET_PATH)/src/board_415dk.c \
$(TARGET_PATH)/src/at32f415_clock.c \
$(TARGET_PATH)/src/audio_codec.c \
$(TARGET_PATH)/src/i2c_application.c \
$(TARGET_PATH)/src/main.c \

CPP_SOURCES = \

LDSCRIPT :=$(TARGET_PATH)/AT32F415xB_FLASH.ld
#######################################
# Misc
#######################################

OCD_CONFIG =$(TARGET_PATH)/at32f415.cfg

# C defines
C_DEFS +=\
$(DEVICE) \
BOARD_415DK \
_USE_STDPERIPH_DRIVER \

ifeq ($(FEATURE_BENCHMAK), yes)
C_DEFS +=ENABLE_BENCHMARK
endif

ifeq ($(FEATURE_BENCHMAK_DHYSTONE),yes)
C_DEFS +=ENABLE_BENCHMAK_DHYSTONE
endif

ifeq ($(FEATURE_TFT_DISPLAY),yes)
C_DEFS +=ENABLE_TFT_DISPLAY
endif

#######################################
# CFLAGS
#######################################

DEVICE =AT32F415CBT7
CPU =-mcpu=cortex-m4 -mthumb

# fpu
#at43f415 does not have FPU
#FPU =-mfloat-abi=hard -mfpu=fpv4-sp-d16 
#FPU =-mfloat-abi=soft

# float-abi
FLOAT_ABI =#-u_printf_float

# mcu
MCU = $(CPU) $(FPU) $(FLOAT_ABI)

ifdef RELEASE
DEBUG =0
OPT   =-O2
else
DEBUG =1
OPT   =-O0 -g -gdwarf-2 -Wall #-Wextra
endif

ifndef V
VERBOSE =@
else
VERBOSE =
endif

# Generate dependency information
#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst))

ASFLAGS  =$(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS   =$(MCU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -std=gnu11 -fdata-sections -ffunction-sections #-fstack-usage
CPPFLAGS =$(CPU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -fdata-sections -ffunction-sections -fno-unwind-tables -fno-exceptions -fno-rtti 
LDFLAGS  =$(MCU) $(SPECS) -Wl,--gc-sections,-Map=$(BUILD_DIR)/$(TARGET).map,--cref

#######################################
# Libraries
#######################################

LIBDIR =
ifdef SEMIHOSTING
SPECS =--specs=rdimon.specs
LDLIBS =-nostartfiles -lc -lrdimon
else
SPECS=--specs=nosys.specs --specs=nano.specs
#-nostdlib -lstd++ -lnosys -lm 
LIBS =-lstdc++
endif

#######################################
# Objects
#######################################

OBJECTS = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/, $(notdir $(CPP_SOURCES:.cpp=.obj)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
#OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
#vpath %.s $(sort $(dir $(ASM_SOURCES)))

#VPATH +=$(sort $(dir $(C_SOURCES)))

#######################################
# Tool binaries
#######################################
PREFIX = arm-none-eabi-
CC  =$(PREFIX)gcc
CPP =$(PREFIX)g++
AS  =$(PREFIX)gcc -x assembler-with-cpp
LD  =$(PREFIX)gcc
AR  =$(PREFIX)ar
SZ  =$(PREFIX)size
OBJCOPY =$(PREFIX)objcopy
OBJDUMP =$(PREFIX)objdump
HEX = $(OBJCOPY) -O ihex
BIN = $(OBJCOPY) -O binary -S

ifeq ($(shell uname -s), Linux)
JLK ="/opt/SEGGER/JLink/JLinkExe"
else
JLK :=$(JLINK)
endif

#######################################
# Rules
#######################################
all: $(BUILD_DIR)/$(TARGET).elf size
#@echo $(CMSIS_PATH)

bin: $(BUILD_DIR)/$(TARGET).bin

program: $(BUILD_DIR)/$(TARGET).jlink bin
#$(VERBOSE)$(JLK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $<
	openocd -f at32f415.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

$(BUILD_DIR)/$(TARGET).jlink: $(BUILD_DIR)/$(TARGET).bin
	@echo "Creating Jlink configuration file"
	@echo "loadfile $< 0x00000000" > $@
	@echo "r" >> $@
	@echo "q" >> $@
#######################################
# build rules
#######################################
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "[CC]  $<"
	$(VERBOSE)$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.obj: %.cpp Makefile | $(BUILD_DIR)
	@echo "[CP]  $<"
	$(VERBOSE)$(CPP) -c $(CPPFLAGS)  $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "[AS]  $<"
	$(VERBOSE)$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) | Makefile
	@echo "[LD]  $@"
	$(VERBOSE)$(LD) $(LIBDIR) -T$(LDSCRIPT) $(LDFLAGS) $^ $(LIBS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(VERBOSE)$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(VERBOSE)$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir -p $@		

size: $(BUILD_DIR)/$(TARGET).elf
	@echo "--- Size ---"
	$(VERBOSE)$(SZ) -A -x $<
	$(VERBOSE)$(SZ) -B $<

#######################################
# clean up
#######################################
clean:
	$(VERBOSE)-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
#-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
ifeq ($(GCC_COLORS), )
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'
#unexport GCC_COLORS
endif

# *** EOF ***
