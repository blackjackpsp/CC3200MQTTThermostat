################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS

GEN_OPTS += C:/Users/Enzo/workspace_v7/tirtos_builds_CC3220SF_LAUNCHXL_release_ccs/Debug/configPkg/compiler.opt
GEN_OPTS__FLAG := --cmd_file="C:/Users/Enzo/workspace_v7/tirtos_builds_CC3220SF_LAUNCHXL_release_ccs/Debug/configPkg/compiler.opt"
GEN_CMDS += C:/Users/Enzo/workspace_v7/tirtos_builds_CC3220SF_LAUNCHXL_release_ccs/Debug/configPkg/linker.cmd
GEN_CMDS__FLAG := -l"C:/Users/Enzo/workspace_v7/tirtos_builds_CC3220SF_LAUNCHXL_release_ccs/Debug/configPkg/linker.cmd"

ORDERED_OBJS += \
"./CC3220SF_LAUNCHXL.obj" \
"./client_cbs.obj" \
"./main_tirtos.obj" \
"./mqtt_client_app.obj" \
"./network_if.obj" \
"./uart_term.obj" \
"../CC3220SF_LAUNCHXL_TIRTOS.cmd" \
$(GEN_CMDS__FLAG) \
-l"C:/ti/simplelink_cc32xx_sdk_1_50_00_06/source/ti/net/mqtt/ccs/client_TIRTOS/mqtt_client.a" \
-l"C:/ti/simplelink_cc32xx_sdk_1_50_00_06/source/ti/display/lib/display.aem4" \
-l"C:/ti/simplelink_cc32xx_sdk_1_50_00_06/source/ti/drivers/lib/drivers_cc32xx.aem4" \
-l"C:/ti/simplelink_cc32xx_sdk_1_50_00_06/source/third_party/fatfs/lib/fatfs.aem4" \
-l"C:/ti/simplelink_cc32xx_sdk_1_50_00_06/source/ti/drivers/net/wifi/ccs/rtos/simplelink.a" \
-l"C:/ti/simplelink_cc32xx_sdk_1_50_00_06/kernel/tirtos/packages/ti/dpl/lib/dpl_cc32xx.aem4" \
-l"C:/ti/simplelink_cc32xx_sdk_1_50_00_06/source/ti/devices/cc32xx/driverlib/ccs/Release/driverlib.a" \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.out \

EXE_OUTPUTS__QUOTED += \
"mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.out" \

BIN_OUTPUTS += \
mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.bin \

BIN_OUTPUTS__QUOTED += \
"mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.bin" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.out"

dependents:
	-cd C:/Users/Enzo/workspace_v7/tirtos_builds_CC3220SF_LAUNCHXL_release_ccs/Debug && $(MAKE) all

# Tool invocations
mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS) C:/Users/Enzo/workspace_v7/tirtos_builds_CC3220SF_LAUNCHXL_release_ccs/Debug/tirtos_builds_CC3220SF_LAUNCHXL_release_ccs.lib
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=vfplib -me -g --diag_warning=225 --diag_warning=255 --diag_wrap=off --display_error_number --gen_func_subsections=on -z -m"mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.map" -i"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/lib" -i"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs_linkInfo.xml" --rom_model -o "mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.bin: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armhex"  -o "mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.bin" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "CC3220SF_LAUNCHXL.obj" "client_cbs.obj" "main_tirtos.obj" "mqtt_client_app.obj" "network_if.obj" "uart_term.obj" 
	-$(RM) "CC3220SF_LAUNCHXL.d" "client_cbs.d" "main_tirtos.d" "mqtt_client_app.d" "network_if.d" "uart_term.d" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-C:/ti/ccsv7/utils/tiobj2bin/tiobj2bin mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.out mqtt_client_CC3220SF_LAUNCHXL_tirtos_ccs.bin C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armofd C:/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.4.LTS/bin/armhex C:/ti/ccsv7/utils/tiobj2bin/mkhex4bin
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:
C:/Users/Enzo/workspace_v7/tirtos_builds_CC3220SF_LAUNCHXL_release_ccs/Debug/tirtos_builds_CC3220SF_LAUNCHXL_release_ccs.lib:

-include ../makefile.targets

