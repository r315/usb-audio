
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
GCFLAGS =-mcpu=cortex-m3 -mthumb -Wall -Og -g -fdata-sections -ffunction-sections -fno-unwind-tables
GPPFLAGS=$(GCFLAGS) -fno-exceptions -fno-rtti
LDFLAGS =-mcpu=cortex-m3 -mthumb -nostdlib -lgcc -Wl,--gc-sections -nodefaultlibs #-nostartfiles #-lstdc++ 

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

$(TARGET).elf:  $(OBJECTS)
	@echo "Linking" $@
	@$(GCC) -T$(LDSCRIPT) $(addprefix -L, $(LIBSPATH)) $(OBJECTS) $(LDFLAGS) -o $(TARGET).elf

$(TARGET).hex: $(TARGET).elf
	@$(OBJCOPY) -O ihex -j .startup -j .text -j .data -j .ram_code -j .rodata $< $@

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
	
$(TARGET).cfg:
	@echo "Creating opencod configuration file"
	@echo "adapter driver jlink" > $(TARGET).cfg
	@echo "transport select swd" >> $(TARGET).cfg
	@echo "source [find target/lpc17xx.cfg]" >> $(TARGET).cfg
	@echo "adapter speed 4000" >> $(TARGET).cfg

program: $(TARGET).jlink
#openocd -f $(notdir $(TARGET).cfg) -c "init" -c "reset halt" -c "program $(TARGET).elf verify reset exit"
	$(VERBOSE)$(JLINK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $<

$(BUILD_DIR):
	mkdir -p $@

########################################################
# 
########################################################
$(BUILD_DIR)/%.o : %.c | $(BUILD_DIR)
	@echo "Compile" $<
	@$(GCC) $(GCFLAGS) $(addprefix -I, $(INCSPATH)) $(GCSYMBOLS) -c $< -o $@

$(BUILD_DIR)/%.obj : %.cpp | $(BUILD_DIR)
	@echo "Compile" $<
	@$(GPP) $(GPPFLAGS) $(addprefix -I, $(INCSPATH)) $(GCSYMBOLS) -c $< -o $@
	
$(BUILD_DIR)/%.o : %.S | $(BUILD_DIR)
	@echo "Assemble" $<
	@$(AS) $(ASFLAGS) $< -o $@

ifeq ($(GCC_COLORS),)
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:locus=01:quote=01'
unexport GCC_COLORS
endif
