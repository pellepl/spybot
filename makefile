

# primary build target configuration
ifneq ($(SEC),1)
CONFIG_SPYBOT_ROVER = 0
CONFIG_SPYBOT_CONTROLLER = 1
CONFIG_SPYBOT_TEST = 0
# secondary build target configuration
else
CONFIG_SPYBOT_ROVER = 1
CONFIG_SPYBOT_CONTROLLER = 0
CONFIG_SPYBOT_TEST = 0
endif


ifeq ($(SEC),1)
BINARY = spybot_sec
GDB_PORT = 4445
DBG_SCRIPT = debug_sec.gdb
FLAGS += -DSECONDARY
else
BINARY = spybot
DBG_SCRIPT = debug.gdb
GDB_PORT = 4444
PRI = 1
FLAGS += -DPRIMARY
endif


ifeq ($(CONFIG_SPYBOT_ROVER),1)
CONFIG_MAKE = config_rover.mk
endif
ifeq ($(CONFIG_SPYBOT_CONTROLLER),1)
CONFIG_MAKE = config_controller.mk
endif
ifeq ($(CONFIG_SPYBOT_TEST),1)
CONFIG_MAKE = config_test.mk
endif


############
#
# STM Device settings
#
############

FLAGS += -DSYSCLK_FREQ_72MHz=72000000
FLAGS += -DSTM32F10X_MD
STARTUP = startup_stm32f10x_md.s

############
#
# Paths
#
############

sourcedir = src
ifeq ($(SEC),1)
builddir = build_sec
else
builddir = build
endif

basetoolsdir = /home/petera/toolchain/arm-elf-tools-4.8.2
#basetoolsdir = /home/petera/toolchain/gcc/arm-elf-tools-4.8.1
#basetoolsdir = /home/petera/toolchain/gcc/arm-elf-tools-4.7.1
#basetoolsdir = /usr/local/gcc/arm-elf-tools-4.8.2
#codir = ${basetoolsdir}/lib/gcc/arm-none-eabi/4.8.1/

hfile = ${sourcedir}/config_header.h

stmlibdir = STM32F10x_StdPeriph_Lib_V3.5.0/Libraries
stmdriverdir = ${stmlibdir}/STM32F10x_StdPeriph_Driver
stmcmsisdir = ${stmlibdir}/CMSIS/CM3/DeviceSupport/ST/STM32F10x
stmcmsisdircore = ${stmlibdir}/CMSIS/CM3/CoreSupport

tools = ${basetoolsdir}/bin

gensysdir = ../generic/system
comm = ../generic/comm
#gensysdir = ../generic/generic_embedded
#comm = ../comm


CPATH =
SPATH =
INC =
SFILES =
CFILES =
RFILES =

#############
#
# Build tools
#
#############

CROSS_COMPILE=${tools}/arm-none-eabi-
#CROSS_COMPILE=${tools}/arm-elf-
CC = $(CROSS_COMPILE)gcc $(COMPILEROPTIONS)
AS = $(CROSS_COMPILE)gcc $(ASSEMBLEROPTIONS)
LD = $(CROSS_COMPILE)ld
GDB = $(CROSS_COMPILE)gdb
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size
MKDIR = mkdir -p

###############
#
# Build configs
#
###############

LD_SCRIPT = arm.ld
INCLUDE_DIRECTIVES =
COMPILEROPTIONS = $(INCLUDE_DIRECTIVES) $(FLAGS) -mcpu=cortex-m3 -mno-thumb-interwork -mthumb -Wall -gdwarf-2 -Wno-packed-bitfield-compat
#-ffunction-sections -fdata-sections
COMPILEROPTIONS += -Os
# -nostartfiles -nostdlib 
ASSEMBLEROPTION = $(COMPILEROPTIONS)
LINKERSCRIPT = $(LD_SCRIPT)
LINKEROPTIONS = --gc-sections -cref
OBJCOPYOPTIONS_HEX = -O ihex ${builddir}/$(BINARY).elf
OBJCOPYOPTIONS_BIN = -O binary ${builddir}/$(BINARY).elf

BUILD_NUMBER_FILE=build-number.txt

###############
#
# Files and libs
#
###############

# app files

CPATH 		+= ${sourcedir}
SPATH 		+= ${sourcedir}
INC			+= -I./${sourcedir}



# generic system configuration
include ${CONFIG_MAKE}

# extra cflags

ifeq ($(CONFIG_SPYBOT_APP_MASTER),1)
FLAGS += -DCONFIG_SPYBOT_APP_MASTER
endif
ifeq ($(CONFIG_SPYBOT_APP_CLIENT),1)
FLAGS += -DCONFIG_SPYBOT_APP_CLIENT
endif
ifeq ($(CONFIG_SPYBOT_LSM),1)
FLAGS += -DCONFIG_SPYBOT_LSM
endif
ifeq ($(CONFIG_SPYBOT_HCSR),1)
FLAGS += -DCONFIG_SPYBOT_HCSR
endif
ifeq ($(CONFIG_SPYBOT_SERVO),1)
FLAGS += -DCONFIG_SPYBOT_SERVO
endif
ifeq ($(CONFIG_SPYBOT_MOTOR),1)
FLAGS += -DCONFIG_SPYBOT_MOTOR
endif
ifeq ($(CONFIG_SPYBOT_VIDEO),1)
FLAGS += -DCONFIG_SPYBOT_VIDEO
endif
ifeq ($(CONFIG_SPYBOT_JOYSTICK),1)
CONFIG_SPYBOT_ADC = 1
CONFIG_SPYBOT_INPUT = 1
FLAGS += -DCONFIG_SPYBOT_JOYSTICK
endif
ifeq ($(CONFIG_SPYBOT_XBUTTONS),1)
CONFIG_SPYBOT_INPUT = 1
FLAGS += -DCONFIG_SPYBOT_XBUTTONS
endif
ifeq ($(CONFIG_SPYBOT_XLEDS),1)
FLAGS += -DCONFIG_SPYBOT_XLEDS
endif

ifeq ($(CONFIG_SPYBOT_CONTROLLER),1)
FLAGS += -DCONFIG_SPYBOT_CONTROLLER
endif
ifeq ($(CONFIG_SPYBOT_ROVER),1)
FLAGS += -DCONFIG_SPYBOT_ROVER
endif
ifeq ($(CONFIG_SPYBOT_TEST),1)
FLAGS += -DCONFIG_SPYBOT_TEST
endif

# common spybot files

SFILES 		+= stm32f10x_it_h.s

CFILES 		+= main.c
CFILES 		+= processor.c
CFILES 		+= cli.c
CFILES 		+= timer.c
CFILES		+= nrf905_driver.c
CFILES		+= nrf905_impl.c
CFILES		+= comm_radio.c
CFILES		+= led.c

CFILES		+= app.c

# spybot rover files
ifeq ($(CONFIG_SPYBOT_APP_CLIENT),1)
CFILES		+= app_rover.c
endif
ifeq ($(CONFIG_SPYBOT_HCSR),1)
CFILES 		+= range_sens_hcsr04_driver.c
endif
ifeq ($(CONFIG_SPYBOT_MOTOR),1)
CFILES 		+= motor.c
endif
ifeq ($(CONFIG_M24M01), 1)
CFILES		+= configuration_ee.c
endif
ifeq ($(CONFIG_SPYBOT_SERVO), 1)
CFILES		+= servo.c
endif

# spybot controller files
ifeq ($(CONFIG_SPYBOT_APP_MASTER),1)
CFILES		+= app_controller.c
endif
ifeq ($(CONFIG_SPYBOT_VIDEO),1)
CFILES		+= cvideo.c
CFILES		+= gfx_bitmap.c
CFILES		+= gfx_3d.c
CFILES		+= gfx_img_modesty.c
CFILES		+= font_spybot.c
CFILES		+= hud.c
CFILES		+= hud_main.c
CFILES		+= hud_conf.c
CFILES		+= hud_dbg.c
CFILES		+= rover_3d.c
endif
ifeq ($(CONFIG_SPYBOT_INPUT),1)
CFILES		+= input.c
endif
ifeq ($(CONFIG_SPYBOT_ADC),1)
CFILES		+= adc.c
endif

# comm files
include ${comm}/files.mk

# stm32 lib files
SPATH	+= ${stmdriverdir}/src ${stmcmsisdir} ${stmcmsisdir}/startup/gcc_ride7
SFILES 	+= $(STARTUP)

CPATH	+= ${stmdriverdir}/src ${stmcmsisdir} ${stmcmsisdircore}
INC		+= -I./${stmdriverdir}/inc
INC		+= -I./${stmcmsisdir}
INC		+= -I./${stmcmsisdircore}

CFILES	+= misc.c
CFILES	+= stm32f10x_adc.c
CFILES	+= stm32f10x_bkp.c
CFILES	+= stm32f10x_can.c
CFILES	+= stm32f10x_cec.c
CFILES	+= stm32f10x_crc.c
CFILES	+= stm32f10x_dac.c
CFILES	+= stm32f10x_dbgmcu.c
CFILES	+= stm32f10x_dma.c
CFILES	+= stm32f10x_exti.c
CFILES	+= stm32f10x_flash.c
CFILES	+= stm32f10x_fsmc.c
CFILES	+= stm32f10x_gpio.c
CFILES	+= stm32f10x_i2c.c
CFILES	+= stm32f10x_iwdg.c
CFILES	+= stm32f10x_pwr.c
CFILES	+= stm32f10x_rcc.c
CFILES	+= stm32f10x_rtc.c
CFILES	+= stm32f10x_sdio.c
CFILES	+= stm32f10x_spi.c
CFILES	+= stm32f10x_tim.c
CFILES	+= stm32f10x_usart.c
CFILES	+= stm32f10x_wwdg.c
		
# cmsis files
CFILES	+= system_stm32f10x.c
CFILES	+= core_cm3.c

# stm32 system
CFILES 	+= stm32f10x_it.c

LIBS = 

BINARYEXT = .hex


############
#
# Tasks
#
############

vpath %.c $(CPATH)
vpath %.s $(SPATH)
INCLUDE_DIRECTIVES += $(INC)

SOBJFILES = $(SFILES:%.s=${builddir}/%.o)
OBJFILES = $(CFILES:%.c=${builddir}/%.o)
ROBJFILES = $(RFILES:%.c=${builddir}/%.o)

DEPFILES = $(CFILES:%.c=${builddir}/%.d)
DEPFILES += $(RFILES:%.c=${builddir}/%.d)

ALLOBJFILES  = $(SOBJFILES)
ALLOBJFILES += $(OBJFILES)
ALLOBJFILES += $(ROBJFILES)

DEPENDENCIES = $(DEPFILES) 

# link object files, create binary for flashing
$(BINARY): ${hfile} $(ALLOBJFILES)
	@echo "... build info"
	@if ! test -f $(BUILD_NUMBER_FILE); then echo 0 > $(BUILD_NUMBER_FILE); fi
	@echo $$(($$(cat $(BUILD_NUMBER_FILE)) + 1)) > $(BUILD_NUMBER_FILE)	
	@echo "... linking"
	@${LD} $(LINKEROPTIONS) $(BUILD_NUMBER_LDFLAGS) -T $(LINKERSCRIPT) -Map ${builddir}/$(BINARY).map -o ${builddir}/$(BINARY).elf $(ALLOBJFILES) $(LIBS)
	@echo "... objcopy"
	@${OBJCOPY} $(OBJCOPYOPTIONS_BIN) ${builddir}/$(BINARY).out
	@${OBJCOPY} $(OBJCOPYOPTIONS_HEX) ${builddir}/$(BINARY)$(BINARYEXT) 
	@echo "... disasm"
	@${OBJDUMP} -hd -j .text -j.data -j .bss -j .bootloader_text -j .bootloader_data -d -S ${builddir}/$(BINARY).elf > ${builddir}/$(BINARY)_disasm.s
	@echo "${BINARY}.out is `du -b ${builddir}/${BINARY}.out | sed 's/\([0-9]*\).*/\1/g '` bytes on flash"

-include $(DEPENDENCIES)	   	

# compile assembly files, arm
$(SOBJFILES) : ${builddir}/%.o:%.s
		@echo "... assembly $@"
		@${AS} -c -o $@ $<
		
# compile c files
$(OBJFILES) : ${builddir}/%.o:%.c
		@echo "... compile $@"
		@${CC} -c -o $@ $<

# compile c files deisgnated for ram
$(ROBJFILES) : ${builddir}/%.o:%.c
		@echo "... ram compile $@"
		@${CC} -c -o $@ $< 

# make dependencies
$(DEPFILES) : ${builddir}/%.d:%.c
		@echo "... depend $@"; \
		rm -f $@; \
		${CC} $(COMPILEROPTIONS) -M $< > $@.$$$$; \
		sed 's,\($*\)\.o[ :]*, ${builddir}/\1.o $@ : ,g' < $@.$$$$ > $@; \
		rm -f $@.$$$$

all: info mkdirs ${hfile} $(BINARY)

info:
	@echo "* Building to ${builddir}"
	@echo "* Compiler options:  $(COMPILEROPTIONS)" 
	@echo "* Assembler options: $(ASSEMBLEROPTIONS)" 
	@echo "* Linker options:    $(LINKEROPTIONS)" 
	@echo "* Linker script:     ${LINKERSCRIPT}"
	
mkdirs:
	-@${MKDIR} ${builddir}
	
clean:
	@echo ... removing build files in ${builddir}
	@rm -f ${builddir}/*.o
	@rm -f ${builddir}/*.d
	@rm -f ${builddir}/*.out
	@rm -f ${builddir}/*.hex
	@rm -f ${builddir}/*.elf
	@rm -f ${builddir}/*.map
	@rm -f ${builddir}/*_disasm.s
	@rm -f _stm32flash.script

install: binlen = $(shell stat -c%s ${builddir}/${BINARY}.out)
install: $(BINARY)
	@echo "binary length of install is ${binlen} bytes.."
	@sed 's/BUILDFILE/${builddir}\/${BINARY}.out/;s/LENGTH/${binlen}/' stm32flash.script > _stm32flash.script
	@echo "script _stm32flash.script" | nc localhost $(GDB_PORT)
	@${RM} _stm32flash.script

debug: $(BINARY)
	@${GDB} ${builddir}/${BINARY}.elf -x $(DBG_SCRIPT)
	
${hfile}: ${CONFIG_MAKE}
	@echo "* Generating config header ${hfile}.."
	@echo "// Auto generated file, do not tamper" > ${hfile}
	@echo "#ifdef INCLUDE_CONFIG_HEADER" >> ${hfile}
	@echo "#ifndef _CONFIG_HEADER_H" >> ${hfile}
	@echo "#define _CONFIG_HEADER_H" >> ${hfile}
	@sed -nr 's/([^ \t]*)?[ \t]*=[ \t]*1/#define \1/p' ${CONFIG_MAKE} >> ${hfile}
	@echo "#endif" >> ${hfile}
	@echo "#endif" >> ${hfile}

build-info:
	@echo "*** INCLUDE PATHS"
	@echo "${INC}"
	@echo "*** SOURCE PATHS"
	@echo "${CPATH}"
	@echo "*** ASSEMBLY PATHS"
	@echo "${SPATH}"
	@echo "*** SOURCE FILES"
	@echo "${CFILES}"
	@echo "*** ASSEMBLY FILES"
	@echo "${SFILES}"
	@echo "*** FLAGS"
	@echo "${FLAGS}"
	
	
############
#
# Build info
#
############

BUILD_NUMBER_LDFLAGS  = --defsym __BUILD_DATE=$$(date +'%Y%m%d')
BUILD_NUMBER_LDFLAGS += --defsym __BUILD_NUMBER=$$(cat $(BUILD_NUMBER_FILE))
