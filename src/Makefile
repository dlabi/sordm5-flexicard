SRCS=main.c system_stm32f4xx.c util.c init.c

# Library modules
SRCS += stm32f4xx_syscfg.c misc.c stm32f4xx_gpio.c stm32f4xx_rcc.c stm32f4xx_usart.c stm32f4xx_sdio.c stm32f4xx_dma.c stm32f4xx_exti.c stm32f4xx_pwr.c 

# Add in stuff for SD access
SRCS += diskio.c  ff.c ffunicode.c stm32f4_discovery.c  stm32f4_discovery_sdio_sd.c system_stm32f4xx.c misc_handlers.c stm32f4xx_sdio.c stm32f4xx_dma.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=interrupt

#######################################################################################
# Modify STM_COMMON where your STM32F4DISCOVERY firmare package is
STM_COMMON=/mnt/d/_DATA/Sord/stm32f407vet6/STM32F4-Discovery_FW_V1.1.0

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
HEX2DFU=./hex2dfu

#CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld 
CFLAGS  = -g -O0 -Wall -Tstm32_flash.ld -ffunction-sections -fdata-sections -fomit-frame-pointer
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 #-mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.
CFLAGS += --specs=nosys.specs

#used for SWO/SWV too, otherwise not stable on printfs.
#CFLAGS += -specs=nano.specs -specs=rdimon.specs -lc -lrdimon

## Even though we specify these registers in main.c as global, still need to specify here to stop FATFS and anything else using though (though we luck out with any other libs linked in)
CFLAGS += -ffixed-s0
CFLAGS += -ffixed-s1
CFLAGS += -ffixed-s2
CFLAGS += -ffixed-s3
CFLAGS += -ffixed-s4
CFLAGS += -ffixed-s5
CFLAGS += -ffixed-s6
CFLAGS += -ffixed-s7
CFLAGS += -ffixed-s8
CFLAGS += -ffixed-s9
CFLAGS += -ffixed-s10 -ffixed-s11 -ffixed-s12 -ffixed-s13 -ffixed-s14 -ffixed-s15 -ffixed-s16 -ffixed-s17 -ffixed-s18 -ffixed-s19 -ffixed-s20
CFLAGS += -ffixed-s21 -ffixed-s22 -ffixed-s23 -ffixed-s24 -ffixed-s25 -ffixed-s26 -ffixed-s27 -ffixed-s28 -ffixed-s29 -ffixed-s30 -ffixed-s31

#this will include msx bios inside firmware, if you have vanilla SORD disable it, it save some space
CFLAGS += -DMODDED_SORD

#this will include as well bi,bg roms inside firmware, in present time it is not implemented to load Basic programs. So it can be disabled now
#CFLAGS += -DOTHER_BASICS

#debug swo, board must be in debugging mode oherwise it won't boot up if enabled
#be carefull if this is enabled it destroys r12 what breaks other non-swo debuging modes below!
#CFLAGS += -DENABLE_SWO

#CFLAGS += -DDEBUG_SWO_WRITE					
#CFLAGS += -DDEBUG_SWO_MONITOR_READ
#CFLAGS += -DDEBUG_SWO_ADDRESS
#																	debug pins PA   3210
#CFLAGS += -DDEBUG_EXTI4_START				#ISR entered 							1XXX
#CFLAGS += -DDEBUG_EXTI4_MRD				#actual reading, 						XX1X
#CFLAGS += -DDEBUG_EXTI4_RAM_READ			#if not active then ROM_READ occurs		XXX1
#CFLAGS += -DDEBUG_EXTI4_INTERNAL_RAM		#access to the internal ram 7000-7fff   XXX1
#CFLAGS += -DDEBUG_EXTI4_RAM_WRITECHECK		#write checking, if area is writable	XX1X
#CFLAGS += -DDEBUG_EXTI4_RAM_WRITE			#data taken from bus 					XX1X
#CFLAGS += -DDEBUG_EXTI4_DATA_OUT			#data on the bus  						X1XX
#CFLAGS += -DDEBUG_EXTI4_TRISTATED			#pins set as inputs 					X0XX
#CFLAGS += -DDEBUG_EXTI4_END				#ISR ended  							0000

#CFLAGS += -DDEBUG_EXTI0_START
#CFLAGS += -DDEBUG_EXTI0_END
#CFLAGS += -DDEBUG_EXTI0_MEM_MODE

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include 
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc
# add git hash 
GIT_COMMIT := $(shell git rev-parse --short HEAD 2> /dev/null || true)
BUILD	   := $(shell git describe --tags --dirty --always --long 2> /dev/null || true)
DATE	   := $(shell date '+%y%m%d%H%M')
LDFLAGS := -X version=${BUILD} -X timestamp=${DATE} ${LDFLAGS}

# add startup file to build
SRCS += startup_stm32f4xx.S 
# You need to end asm files in capital S to get them to see preprocessor directives
SRCS += interrupt.S

OBJS = $(SRCS:.c=.o)

vpath %.c $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src $(STM_COMMON)/Utilities/STM32F4-Discovery

.PHONY: proj menu

all: proj

proj: $(PROJ_NAME).elf
menu: menu.rom

$(PROJ_NAME).elf: $(SRCS)
	$(CC) -D_BUILD_=\"$(BUILD)\" $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -dS $(PROJ_NAME).elf >$(PROJ_NAME).asm
	$(HEX2DFU) -i $(PROJ_NAME).hex -o $(PROJ_NAME).dfu -d ${GIT_COMMIT}
clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin

#attaches stlink from host win to wsl
stlink:
	usbipd.exe wsl attach -d Ubuntu-20.04 -i 0483:3748

# Flash the STM32F4 via stlink dongle
flash:
	st-flash write $(PROJ_NAME).bin 0x8000000

# Flash the STM32F4 via dfu
flash-dfu:
	sudo dfu-util -v -d 0483:df11 -a 0 -s 0x08000000 -D $(PROJ_NAME).bin

# builds menu.rom using pasmo cross assembler
menu.rom:
	pasmo -v --equ ROM=1 menu/menu.asm menu/menu.rom 