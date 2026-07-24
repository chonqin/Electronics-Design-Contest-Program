################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1362586930: ../2024QuestionH.syscfg
	@echo 'SysConfig - building file: "$<"'
	"D:/TI/sysconfig_1.26.2/sysconfig_cli.bat" -s "D:/TI/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/2024QuestionH.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1362586930 ../2024QuestionH.syscfg
device.opt: build-1362586930
device.cmd.genlibs: build-1362586930
ti_msp_dl_config.c: build-1362586930
ti_msp_dl_config.h: build-1362586930
Event.dot: build-1362586930

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/Debug" -I"D:/TI/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/TI/mspm0_sdk_2_10_00_04/source" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/inc" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/APP/inc" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/Board" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/APP/src" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/src" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/TI/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/Debug" -I"D:/TI/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/TI/mspm0_sdk_2_10_00_04/source" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/inc" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/APP/inc" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/Board" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/APP/src" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/src" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/Debug" -I"D:/TI/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/TI/mspm0_sdk_2_10_00_04/source" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/inc" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/APP/inc" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/Board" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/APP/src" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/src" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


