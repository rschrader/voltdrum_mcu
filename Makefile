######################################
# Makefile by CubeMX2Makefile.py
######################################

######################################
# target
######################################
TARGET = Voltdrum_Firmware

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0

#######################################
# pathes
#######################################
# source path
# Build path
BUILD_DIR = build

######################################
# source
######################################
C_SOURCES = \
  Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/system_stm32f3xx.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_adc_ex.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_cortex.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_dma.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_flash_ex.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_gpio.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pcd.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pcd_ex.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_pwr_ex.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_rcc_ex.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_spi.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_tim_ex.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart.c \
  Drivers/STM32F3xx_HAL_Driver/Src/stm32f3xx_hal_uart_ex.c \
  Middlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Src/usbd_audio.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
  Src/adc.c \
  Src/gpio.c \
  Src/main.c \
  Src/spi.c \
  Src/stm32f3xx_hal_msp.c \
  Src/stm32f3xx_it.c \
  Src/tim.c \
  Src/usart.c \
  Src/usb_device.c \
  Src/usbd_audio_if.c \
  Src/usbd_conf.c \
  Src/usbd_desc.c \
  Src/mcp4251.c \
  Src/dma.c \
  Src/triggerchannel.c  \
  Src/hihatchannel.c  \
  Src/performance.c  \
  Src/uartmessagebuffer.c  \
  Src/midi.c 
ASM_SOURCES = \
  Drivers/CMSIS/Device/ST/STM32F3xx/Source/Templates/gcc/startup_stm32f303xc.s

#######################################
# binaries
#######################################
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc -x assembler-with-cpp
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = arm-none-eabi-size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# macros for gcc
AS_DEFS =
C_DEFS = -DUSE_HAL_DRIVER -DSTM32F303xC

# includes for gcc
AS_INCLUDES =
C_INCLUDES = -IInc
C_INCLUDES += -IDrivers/STM32F3xx_HAL_Driver/Inc
C_INCLUDES += -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc
C_INCLUDES += -IMiddlewares/ST/STM32_USB_Device_Library/Class/AUDIO/Inc
C_INCLUDES += -IDrivers/CMSIS/Include
C_INCLUDES += -IDrivers/CMSIS/Device/ST/STM32F3xx/Include

# compile gcc flags
# added -mfpu=fpv4-sp-d16 -mfloat-abi=softfp to as and c flags
ASFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MD -MP -MF .dep/$(@F).d

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = arm-gcc-link.ld
# libraries
LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@
	
$(BUILD_DIR):
	mkdir -p $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(BUILD_DIR)
	
#######################################
# flash to device
#######################################
install: $(BUILD_DIR)/$(TARGET).bin
	- st-flash write $(BUILD_DIR)/$(TARGET).bin 0x8000000

	
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
