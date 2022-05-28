
#########################################################
# project files
#########################################################
TARGET 		 =$(BUILD_DIR)/usb_audio
BUILD_DIR 	:=build
PRJ_DIR 	:=$(PWD)

C_SOURCES =\
$(PRJ_DIR)/src/adcuser.c \
$(PRJ_DIR)/src/system_LPC17xx.c \
$(PRJ_DIR)/src/usbdmain.c \
$(PRJ_DIR)/src/usbcore.c \
$(PRJ_DIR)/src/usbdesc.c \
$(PRJ_DIR)/src/usbuser.c \
$(PRJ_DIR)/src/usbhw.c \
$(PRJ_DIR)/src/logger.c \
$(PRJ_DIR)/src/uart_lpc17xx.c \

C_SOURCES   +=$(PRJ_DIR)/src/startup_LPC17xx.c

CPP_SOURCES = \

#ASM_SOURCES   =startup_LPC17xx.s

INCSPATH =\
$(PRJ_DIR)/inc \

GCSYMBOLS =-D__NEWLIB__ -DBOARD_BLUEBOARD -DENABLE_DEBUG

LIBSPATH +=

LDSCRIPT =$(PRJ_DIR)/LPC17xx.ld
#########################################################
#Startup files and libraries
#########################################################
CPU =-mcpu=cortex-m3 -mthumb
CFLAGS =$(CPU) -Wall -Og -g -fdata-sections -ffunction-sections -fno-unwind-tables
CPPFLAGS=$(CFLAGS) -fno-exceptions -fno-rtti

# -specs=nosys.specs -specs=nano.specs -lc -lgcc -lstdc++ -Wl,--gc-sections -nodefaultlibs -nostartfiles  -nostdlib
LDFLAGS =$(CPU) -g -nostdlib -Wl,--gc-sections -lgcc

DEVICE =LPC1768

#########################################################
GCC_EXEC_PREFIX = arm-none-eabi
GCC = $(GCC_EXEC_PREFIX)-gcc
GPP = $(GCC_EXEC_PREFIX)-g++
AS = $(GCC_EXEC_PREFIX)-as
LD = $(GCC_EXEC_PREFIX)-ld
SIZE = $(GCC_EXEC_PREFIX)-size
OBJCOPY = $(GCC_EXEC_PREFIX)-objcopy
OBJDUMP = $(GCC_EXEC_PREFIX)-objdump
DBG = $(GCC_EXEC_PREFIX)-insight
REMOVE = rm -fR


ifeq ($(shell uname -s), Linux)
JLINK ="/opt/SEGGER/JLink/JLinkExe"
JLINK_SERVER ="/opt/SEGGER/JLink/JLinkGDBServer"
else
JLINK ="C:\Tools\JLink_V500\jlink"
endif

CHECKSUM =$(LIBEMB_PATH)/tools/checksum

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.obj)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

##########################################################
# RULES
##########################################################
all: $(TARGET).elf stats

silent: $(TARGET).elf

$(TARGET).elf:  $(OBJECTS)
	@echo "Linking" $@
	@$(GCC) -T$(LDSCRIPT) $(addprefix -L, $(LIBSPATH)) $(OBJECTS) $(LDFLAGS) -o $(TARGET).elf

$(TARGET).hex: $(TARGET).elf
	@$(OBJCOPY) -O ihex -j .startup -j .text -j .data -j .ram_code -j .rodata $< $@

$(TARGET).bin: $(TARGET).elf
	@echo "Generating bin file" $@
	$(OBJCOPY) -O binary $(TARGET).elf $@
	$(CHECKSUM) $@

stats: $(TARGET).elf
	@echo "---- Sections Summary ---"
	@$(SIZE) -A -x $<
	@$(SIZE) -B $<

clean:
	$(REMOVE) $(BUILD_DIR)

$(TARGET).jlink: $(TARGET).hex
	@echo "Creating Jlink configuration file"
	@echo "loadfile $<" >> $@
	@echo "r" >> $@
	@echo "q" >> $@
#@echo "erase" > $@
	
$(TARGET)_jlink.cfg:
	@echo "Creating openocd configuration file"
	@echo "adapter driver jlink" >> $@
	@echo "transport select swd" >> $@
	@echo "source [find target/lpc17xx.cfg]" >> $@
	@echo "adapter speed 4000" >> $@

$(TARGET)_stlink.cfg:
	@echo source [find interface/stlink-v2.cfg] >> $@
	@echo source [find target/lpc17xx.cfg]  >> $@
	@echo adapter speed 4000  >> $@

program-openocd: $(TARGET)_stlink.cfg $(TARGET).bin
	openocd -f $< -c "init" -c "reset halt" -c "program $(TARGET).bin 0 verify reset exit"

program-jlink: $(TARGET).jlink
	$(VERBOSE)$(JLINK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $<

$(BUILD_DIR):
	mkdir -p $@

########################################################
# 
########################################################
$(BUILD_DIR)/%.o : %.c | $(BUILD_DIR)
	@echo "Compile" $<
	@$(GCC) $(CFLAGS) $(addprefix -I, $(INCSPATH)) $(GCSYMBOLS) -c $< -o $@

$(BUILD_DIR)/%.obj : %.cpp | $(BUILD_DIR)
	@echo "Compile" $<
	@$(GPP) $(CPPFLAGS) $(addprefix -I, $(INCSPATH)) $(GCSYMBOLS) -c $< -o $@
	
$(BUILD_DIR)/%.o : %.S | $(BUILD_DIR)
	@echo "Assemble" $<
	@$(AS) $(ASFLAGS) $< -o $@

ifeq ($(GCC_COLORS),)
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:locus=01:quote=01'
unexport GCC_COLORS
endif
