################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
Board/%.o: ../Board/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"D:/TI/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/Debug" -I"D:/TI/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"D:/TI/mspm0_sdk_2_10_00_04/source" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/inc" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/APP/inc" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/Board" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/APP/src" -I"D:/desktop/TI Projects/Electronics-Design-Contest-Program/2024QuestionH/BSP/src" -gdwarf-3 -Wall -MMD -MP -MF"Board/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


