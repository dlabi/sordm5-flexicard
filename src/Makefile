#SRCS=main.c system_stm32f4xx.c stm32f4xx_it.c
#SRCS=main.c system_stm32f4xx.c working-stm32f4xx_it.c

#
#
SRCS=main.c system_stm32f4xx.c 

# Library modules
#SRCS += stm32f4xx_syscfg.c misc.c stm32f4xx_gpio.c stm32f4xx_rcc.c stm32f4xx_usart.c stm32f4xx_exti.c stm32f4xx_pwr.c stm32f4xx_adc.c
SRCS += stm32f4xx_syscfg.c misc.c stm32f4xx_gpio.c stm32f4xx_rcc.c stm32f4xx_usart.c stm32f4xx_sdio.c stm32f4xx_dma.c stm32f4xx_exti.c stm32f4xx_pwr.c 

#SRCS += stm32f4xx_tim.c 
#SRCS += stm32f4_discovery.c

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=interrupt

#######################################################################################

#STM_COMMON=../../..
# You need libs somewhere!
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
CFLAGS += -specs=nano.specs -specs=rdimon.specs -lc -lrdimon
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

#CFLAGS += -DENABLE_SWO #debug swo, board must be in debugging mode oherwise it won't boot up if enabled, pozor kdyz povoleno r12 se nastavi jinak, proto nefunguji ostatni non-swo debugy!

#CFLAGS += -DDEBUG_SWO_WRITE					
#CFLAGS += -DDEBUG_SWO_MONITOR_READ
#CFLAGS += -DDEBUG_SWO_ADDRESS
#																	debug pins PA   3210
CFLAGS += -DDEBUG_EXTI4_START				#ISR entered 							1XXX
#CFLAGS += -DDEBUG_EXTI4_MRD				#actual reading, 						XX1X
#CFLAGS += -DDEBUG_EXTI4_RAM_READ			#if not active then ROM_READ occurs		XXX1
CFLAGS += -DDEBUG_EXTI4_INTERNAL_RAM		#access to the internal ram 7000-7fff   XXX1
CFLAGS += -DDEBUG_EXTI4_RAM_WRITE			#data taken from bus 					XX1X
CFLAGS += -DDEBUG_EXTI4_DATA_OUT			#data on the bus  						X1XX
CFLAGS += -DDEBUG_EXTI4_TRISTATED			#pins set as inputs 					X0XX
CFLAGS += -DDEBUG_EXTI4_END					#ISR ended  							0000

#CFLAGS += -DDEBUG_EXTI0_START
#CFLAGS += -DDEBUG_EXTI0_6C
#CFLAGS += -DDEBUG_EXTI0_END
#CFLAGS += -DDEBUG_EXTI0_MEM_MODE

# Include files from STM libraries
CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/Include 
CFLAGS += -I$(STM_COMMON)/Libraries/CMSIS/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/inc
# add git hash 
GIT_COMMIT := $(shell git rev-parse --short HEAD 2> /dev/null || true)
LDFLAGS := -X version=${GIT_COMMIT} ${LDFLAGS}

# add startup file to build
SRCS += startup_stm32f4xx.S 
# You need to end asm files in capital S to get them to see preprocessor directives
SRCS += interrupt.S

OBJS = $(SRCS:.c=.o)

vpath %.c $(STM_COMMON)/Libraries/STM32F4xx_StdPeriph_Driver/src $(STM_COMMON)/Utilities/STM32F4-Discovery

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ 
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -dS $(PROJ_NAME).elf >$(PROJ_NAME).asm
	$(HEX2DFU) -i $(PROJ_NAME).hex -o $(PROJ_NAME).dfu -d ${GIT_COMMIT}
clean:
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin

stlink:
#attaches stlink from host win to wsl
	usbipd.exe wsl attach -d Ubuntu-20.04 -i 0483:3748
# Flash the STM32F4
burn:
	st-flash write $(PROJ_NAME).bin 0x8000000
