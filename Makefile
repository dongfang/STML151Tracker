# put your *.o targets here, make should handle the rest!
SRCS = system_stm32l1xx.c main.c Trace.c trace_impl.c _sbrk.c CDCE913.c Systick.c WSPR.c ADC.c StabilizedOscillator.c WSPRTransmitter.c GPS.c SelfCalibration.c RTC.c DAC.c nhash.c Callsigns.c APRS.c APRSWorldMap.c AX25.c APRSTransmitter.c Power.c RecordStorage.c
SRCS += ADCPhysics.c DummyPhysics.c UBloxGPS.c DummyGPS.c ExclusionZones.c BitBangingI2C.c
SRCS += RH_RF24.cpp RF24Wrapper.cpp SPI.cpp
SRCS += Device/startup_stm32l1xx_md.s 

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJ_NAME=trk

# Location of the Libraries folder from the STM32F0xx Standard Peripheral Library
STD_PERIPH_LIB=Libraries

# Location of the linker scripts
LDSCRIPT_INC=Device/ldscripts

# location of OpenOCD Board .cfg files (only used with 'make program')
OPENOCD_BOARD_DIR=/usr/share/openocd/scripts/board

# Configuration (cfg) file containing programming directives for OpenOCD
OPENOCD_PROC_FILE=extra/stm32f0-openocd.cfg

# that's it, no need to change anything below this line!

###################################################

CC=arm-none-eabi-gcc
CPPC=arm-none-eabi-g++
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SIZE=arm-none-eabi-size

CFLAGS  = -Wall -g -std=c99 -Os --specs=rdimon.specs
#CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m0 -march=armv6s-m
CFLAGS += -mlittle-endian -mcpu=cortex-m0  -march=armv6-m -mthumb
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections -Wl,-Map=$(PROJ_NAME).map

###################################################

vpath %.c src
vpath %.cpp src
vpath %.a $(STD_PERIPH_LIB)

ROOT=$(shell pwd)

CFLAGS += -I inc -I $(STD_PERIPH_LIB) -I $(STD_PERIPH_LIB)/CMSIS/Device/ST/STM32L1xx/Include
CFLAGS += -I $(STD_PERIPH_LIB)/CMSIS/Include -I $(STD_PERIPH_LIB)/STM32L1xx_StdPeriph_Driver/inc
CFLAGS += -include $(STD_PERIPH_LIB)/stm32l1xx_conf.h -D__ARM_ARCH_7M__
#CFLAGS += -DTRACE 
# add startup file to build
# need if you want to build with -DUSE_CMSIS 
#SRCS += stm32f0_discovery.c
#SRCS += stm32f0_discovery.c stm32f0xx_it.c

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C $(STD_PERIPH_LIB)

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -L$(STD_PERIPH_LIB) -lstm32l1 -L$(LDSCRIPT_INC) -Tstm32l1xx.ld
#	$(CPPC) $(CPPFLAGS) $^ -o $@ -L$(STD_PERIPH_LIB) -lstm32l1 -L$(LDSCRIPT_INC) -Tstm32l1xx.ld
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	$(SIZE) $(PROJ_NAME).elf

listfile: $(SRCS)
	$(CC) $(CFLAGS) $^ -c -g -Wa,-a,-ad -lstm32l1 -L$(LDSCRIPT_INC) -Tstm32l1xx.ld > $(PROJ_NAME).s

program: $(PROJ_NAME).bin
	openocd -f $(OPENOCD_BOARD_DIR)/stm32ldiscovery.cfg -f $(OPENOCD_PROC_FILE) -c "stm_flash `pwd`/$(PROJ_NAME).bin" -c shutdown

clean:
	find ./ -name '*~' | xargs rm -f	
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
	rm -f $(PROJ_NAME).lst

reallyclean: clean
	$(MAKE) -C $(STD_PERIPH_LIB) clean
