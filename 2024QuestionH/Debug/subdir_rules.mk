################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-326554401: ../2024QuestionH.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/sysconfig_1.26.2/sysconfig_cli.bat" --script "C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/2024QuestionH.syscfg" -o "." -s "C:/ti/mspm0_sdk_2_10_00_04/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-326554401 ../2024QuestionH.syscfg
device.opt: build-326554401
device.cmd.genlibs: build-326554401
ti_msp_dl_config.c: build-326554401
ti_msp_dl_config.h: build-326554401
Event.dot: build-326554401

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2031/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/Debug" -I"C:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_10_00_04/source" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/inc" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/APP/inc" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/Board" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/APP/src" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/src" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: C:/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2031/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/Debug" -I"C:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_10_00_04/source" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/inc" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/APP/inc" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/Board" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/APP/src" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/src" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs2031/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/Debug" -I"C:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"C:/ti/mspm0_sdk_2_10_00_04/source" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/inc" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/APP/inc" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/Board" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/APP/src" -I"C:/Users/chonqin/Desktop/Ti projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/src" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


