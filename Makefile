##########################################################################################################################
# File automatically-generated by tool: [projectgenerator] version: [4.1.0] date: [Wed Nov 15 21:44:48 CET 2023] 
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = X-CUBE-DSPDEMO


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Os


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Src/main.c \
Core/Src/touch.c \
Core/Src/interpolation.c \
Core/Src/stm32f4xx_it.c \
Core/Src/stm32f4xx_hal_msp.c \
Core/Src/system_stm32f4xx.c \
Core/Src/GUIDRV_stm32f407_blackboard.c \
Core/Src/GUIConf.c \
Core/Src/calibration.c \
Core/Src/display.c \
Core/Src/fft_processing.c \
Core/Src/fir_processing.c \
Drivers/BSP/Components/ili9341/ili9341.c \
Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c \
Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.c \
Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_f32.c \
Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_q31.c \
Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_q15.c \
Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_f32.c \
Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_q15.c \
Drivers/CMSIS/DSP/Source/FilteringFunctions/arm_fir_init_q31.c \
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.c \
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.c \
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.c \
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.c \
Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_q15.c \
Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_q31.c \
Drivers/CMSIS/DSP/Source/StatisticsFunctions/arm_max_f32.c \
Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_q31.c \
Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_q15.c \
Drivers/CMSIS/DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_f32.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_q15.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_q31.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_q31.c \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_cfft_radix4_q15.c \
Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sqrt_q31.c \
Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sqrt_q15.c \
Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rng.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rtc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_crc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c

# ASM sources
ASM_SOURCES =  \
startup_stm32f407xx.s \
Drivers/CMSIS/DSP/Source/TransformFunctions/arm_bitreversal2.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx \
-DARM_MATH_CM4


# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-IMiddleware/ST/STemWin/inc \
-IMiddleware/ST/STemWin/Config \
-ICore/Inc \
-IDrivers/BSP/Components/Common \
-IDrivers/BSP/Components/ili9341 \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/DSP/Include \
-IDrivers/CMSIS/Include


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -include main.h -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
ASFLAGS += -g
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
ASFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F407VETx_FLASH.ld

# libraries
LIBS = -l:STemWin528_CM4_GCC.a  -lc -lm -lnosys
LIBDIR = -LMiddleware/ST/STemWin/Lib
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


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
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
flash: all
	st-flash --serial $(SERIAL) --reset write build/$(TARGET).bin 0x8000000
erase:
	st-flash --reset erase

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
