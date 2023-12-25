#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-Release.mk)" "nbproject/Makefile-local-Release.mk"
include nbproject/Makefile-local-Release.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=Release
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../examples/src/pwm_basic_example.c ../src/bod.c ../src/clkctrl.c ../src/cpuint.c ../src/driver_init.c ../src/protected_io.S ../src/pwm_basic.c ../src/slpctrl.c ../src/tcb.c ../atmel_start.c ../driver_isr.c ../main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o ${OBJECTDIR}/_ext/1360937237/bod.o ${OBJECTDIR}/_ext/1360937237/clkctrl.o ${OBJECTDIR}/_ext/1360937237/cpuint.o ${OBJECTDIR}/_ext/1360937237/driver_init.o ${OBJECTDIR}/_ext/1360937237/protected_io.o ${OBJECTDIR}/_ext/1360937237/pwm_basic.o ${OBJECTDIR}/_ext/1360937237/slpctrl.o ${OBJECTDIR}/_ext/1360937237/tcb.o ${OBJECTDIR}/_ext/1472/atmel_start.o ${OBJECTDIR}/_ext/1472/driver_isr.o ${OBJECTDIR}/_ext/1472/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o.d ${OBJECTDIR}/_ext/1360937237/bod.o.d ${OBJECTDIR}/_ext/1360937237/clkctrl.o.d ${OBJECTDIR}/_ext/1360937237/cpuint.o.d ${OBJECTDIR}/_ext/1360937237/driver_init.o.d ${OBJECTDIR}/_ext/1360937237/protected_io.o.d ${OBJECTDIR}/_ext/1360937237/pwm_basic.o.d ${OBJECTDIR}/_ext/1360937237/slpctrl.o.d ${OBJECTDIR}/_ext/1360937237/tcb.o.d ${OBJECTDIR}/_ext/1472/atmel_start.o.d ${OBJECTDIR}/_ext/1472/driver_isr.o.d ${OBJECTDIR}/_ext/1472/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o ${OBJECTDIR}/_ext/1360937237/bod.o ${OBJECTDIR}/_ext/1360937237/clkctrl.o ${OBJECTDIR}/_ext/1360937237/cpuint.o ${OBJECTDIR}/_ext/1360937237/driver_init.o ${OBJECTDIR}/_ext/1360937237/protected_io.o ${OBJECTDIR}/_ext/1360937237/pwm_basic.o ${OBJECTDIR}/_ext/1360937237/slpctrl.o ${OBJECTDIR}/_ext/1360937237/tcb.o ${OBJECTDIR}/_ext/1472/atmel_start.o ${OBJECTDIR}/_ext/1472/driver_isr.o ${OBJECTDIR}/_ext/1472/main.o

# Source Files
SOURCEFILES=../examples/src/pwm_basic_example.c ../src/bod.c ../src/clkctrl.c ../src/cpuint.c ../src/driver_init.c ../src/protected_io.S ../src/pwm_basic.c ../src/slpctrl.c ../src/tcb.c ../atmel_start.c ../driver_isr.c ../main.c

# Pack Options 
PACK_COMPILER_OPTIONS=-I "${DFP_DIR}/include"
PACK_COMMON_OPTIONS=-B "${DFP_DIR}/gcc/dev/attiny402"



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-Release.mk dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=ATtiny402
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/protected_io.o: ../src/protected_io.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/protected_io.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/protected_io.o 
	@${RM} ${OBJECTDIR}/_ext/1360937237/protected_io.o.ok ${OBJECTDIR}/_ext/1360937237/protected_io.o.err 
	 ${MP_CC} $(MP_EXTRA_AS_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -DDEBUG  -x assembler-with-cpp -c -D__$(MP_PROCESSOR_OPTION)__   -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/protected_io.o.d" -MT "${OBJECTDIR}/_ext/1360937237/protected_io.o.d" -MT ${OBJECTDIR}/_ext/1360937237/protected_io.o -o ${OBJECTDIR}/_ext/1360937237/protected_io.o ../src/protected_io.S  -DXPRJ_Release=$(CND_CONF)  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1360937237/protected_io.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1
	
else
${OBJECTDIR}/_ext/1360937237/protected_io.o: ../src/protected_io.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/protected_io.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/protected_io.o 
	@${RM} ${OBJECTDIR}/_ext/1360937237/protected_io.o.ok ${OBJECTDIR}/_ext/1360937237/protected_io.o.err 
	 ${MP_CC} $(MP_EXTRA_AS_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x assembler-with-cpp -c -D__$(MP_PROCESSOR_OPTION)__   -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/protected_io.o.d" -MT "${OBJECTDIR}/_ext/1360937237/protected_io.o.d" -MT ${OBJECTDIR}/_ext/1360937237/protected_io.o -o ${OBJECTDIR}/_ext/1360937237/protected_io.o ../src/protected_io.S  -DXPRJ_Release=$(CND_CONF)  -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1360937237/protected_io.o.asm.d"
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o: ../examples/src/pwm_basic_example.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2130678003" 
	@${RM} ${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o.d 
	@${RM} ${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o.d" -MT "${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o.d" -MT ${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o  -o ${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o ../examples/src/pwm_basic_example.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/bod.o: ../src/bod.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/bod.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/bod.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/bod.o.d" -MT "${OBJECTDIR}/_ext/1360937237/bod.o.d" -MT ${OBJECTDIR}/_ext/1360937237/bod.o  -o ${OBJECTDIR}/_ext/1360937237/bod.o ../src/bod.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/clkctrl.o: ../src/clkctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/clkctrl.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/clkctrl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/clkctrl.o.d" -MT "${OBJECTDIR}/_ext/1360937237/clkctrl.o.d" -MT ${OBJECTDIR}/_ext/1360937237/clkctrl.o  -o ${OBJECTDIR}/_ext/1360937237/clkctrl.o ../src/clkctrl.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/cpuint.o: ../src/cpuint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/cpuint.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/cpuint.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/cpuint.o.d" -MT "${OBJECTDIR}/_ext/1360937237/cpuint.o.d" -MT ${OBJECTDIR}/_ext/1360937237/cpuint.o  -o ${OBJECTDIR}/_ext/1360937237/cpuint.o ../src/cpuint.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/driver_init.o: ../src/driver_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/driver_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/driver_init.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/driver_init.o.d" -MT "${OBJECTDIR}/_ext/1360937237/driver_init.o.d" -MT ${OBJECTDIR}/_ext/1360937237/driver_init.o  -o ${OBJECTDIR}/_ext/1360937237/driver_init.o ../src/driver_init.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/pwm_basic.o: ../src/pwm_basic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/pwm_basic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/pwm_basic.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/pwm_basic.o.d" -MT "${OBJECTDIR}/_ext/1360937237/pwm_basic.o.d" -MT ${OBJECTDIR}/_ext/1360937237/pwm_basic.o  -o ${OBJECTDIR}/_ext/1360937237/pwm_basic.o ../src/pwm_basic.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/slpctrl.o: ../src/slpctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/slpctrl.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/slpctrl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/slpctrl.o.d" -MT "${OBJECTDIR}/_ext/1360937237/slpctrl.o.d" -MT ${OBJECTDIR}/_ext/1360937237/slpctrl.o  -o ${OBJECTDIR}/_ext/1360937237/slpctrl.o ../src/slpctrl.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/tcb.o: ../src/tcb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/tcb.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/tcb.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/tcb.o.d" -MT "${OBJECTDIR}/_ext/1360937237/tcb.o.d" -MT ${OBJECTDIR}/_ext/1360937237/tcb.o  -o ${OBJECTDIR}/_ext/1360937237/tcb.o ../src/tcb.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/atmel_start.o: ../atmel_start.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/atmel_start.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/atmel_start.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1472/atmel_start.o.d" -MT "${OBJECTDIR}/_ext/1472/atmel_start.o.d" -MT ${OBJECTDIR}/_ext/1472/atmel_start.o  -o ${OBJECTDIR}/_ext/1472/atmel_start.o ../atmel_start.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/driver_isr.o: ../driver_isr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/driver_isr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/driver_isr.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1472/driver_isr.o.d" -MT "${OBJECTDIR}/_ext/1472/driver_isr.o.d" -MT ${OBJECTDIR}/_ext/1472/driver_isr.o  -o ${OBJECTDIR}/_ext/1472/driver_isr.o ../driver_isr.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402" -g -DDEBUG  -gdwarf-2  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1472/main.o.d" -MT "${OBJECTDIR}/_ext/1472/main.o.d" -MT ${OBJECTDIR}/_ext/1472/main.o  -o ${OBJECTDIR}/_ext/1472/main.o ../main.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
else
${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o: ../examples/src/pwm_basic_example.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/2130678003" 
	@${RM} ${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o.d 
	@${RM} ${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o.d" -MT "${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o.d" -MT ${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o  -o ${OBJECTDIR}/_ext/2130678003/pwm_basic_example.o ../examples/src/pwm_basic_example.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/bod.o: ../src/bod.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/bod.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/bod.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/bod.o.d" -MT "${OBJECTDIR}/_ext/1360937237/bod.o.d" -MT ${OBJECTDIR}/_ext/1360937237/bod.o  -o ${OBJECTDIR}/_ext/1360937237/bod.o ../src/bod.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/clkctrl.o: ../src/clkctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/clkctrl.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/clkctrl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/clkctrl.o.d" -MT "${OBJECTDIR}/_ext/1360937237/clkctrl.o.d" -MT ${OBJECTDIR}/_ext/1360937237/clkctrl.o  -o ${OBJECTDIR}/_ext/1360937237/clkctrl.o ../src/clkctrl.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/cpuint.o: ../src/cpuint.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/cpuint.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/cpuint.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/cpuint.o.d" -MT "${OBJECTDIR}/_ext/1360937237/cpuint.o.d" -MT ${OBJECTDIR}/_ext/1360937237/cpuint.o  -o ${OBJECTDIR}/_ext/1360937237/cpuint.o ../src/cpuint.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/driver_init.o: ../src/driver_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/driver_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/driver_init.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/driver_init.o.d" -MT "${OBJECTDIR}/_ext/1360937237/driver_init.o.d" -MT ${OBJECTDIR}/_ext/1360937237/driver_init.o  -o ${OBJECTDIR}/_ext/1360937237/driver_init.o ../src/driver_init.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/pwm_basic.o: ../src/pwm_basic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/pwm_basic.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/pwm_basic.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/pwm_basic.o.d" -MT "${OBJECTDIR}/_ext/1360937237/pwm_basic.o.d" -MT ${OBJECTDIR}/_ext/1360937237/pwm_basic.o  -o ${OBJECTDIR}/_ext/1360937237/pwm_basic.o ../src/pwm_basic.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/slpctrl.o: ../src/slpctrl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/slpctrl.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/slpctrl.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/slpctrl.o.d" -MT "${OBJECTDIR}/_ext/1360937237/slpctrl.o.d" -MT ${OBJECTDIR}/_ext/1360937237/slpctrl.o  -o ${OBJECTDIR}/_ext/1360937237/slpctrl.o ../src/slpctrl.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1360937237/tcb.o: ../src/tcb.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/tcb.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/tcb.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1360937237/tcb.o.d" -MT "${OBJECTDIR}/_ext/1360937237/tcb.o.d" -MT ${OBJECTDIR}/_ext/1360937237/tcb.o  -o ${OBJECTDIR}/_ext/1360937237/tcb.o ../src/tcb.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/atmel_start.o: ../atmel_start.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/atmel_start.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/atmel_start.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1472/atmel_start.o.d" -MT "${OBJECTDIR}/_ext/1472/atmel_start.o.d" -MT ${OBJECTDIR}/_ext/1472/atmel_start.o  -o ${OBJECTDIR}/_ext/1472/atmel_start.o ../atmel_start.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/driver_isr.o: ../driver_isr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/driver_isr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/driver_isr.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1472/driver_isr.o.d" -MT "${OBJECTDIR}/_ext/1472/driver_isr.o.d" -MT ${OBJECTDIR}/_ext/1472/driver_isr.o  -o ${OBJECTDIR}/_ext/1472/driver_isr.o ../driver_isr.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	 ${MP_CC}  $(MP_EXTRA_CC_PRE) -mmcu=attiny402  -I "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/include" -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -x c -c -D__$(MP_PROCESSOR_OPTION)__  -funsigned-char -funsigned-bitfields -Os -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -DNDEBUG  -I "../Config" -I "../examples/include" -I "../include" -I "../utils" -I "../utils/assembler" -I "../" -Wall -MD -MP -MF "${OBJECTDIR}/_ext/1472/main.o.d" -MT "${OBJECTDIR}/_ext/1472/main.o.d" -MT ${OBJECTDIR}/_ext/1472/main.o  -o ${OBJECTDIR}/_ext/1472/main.o ../main.c  -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD) 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mmcu=attiny402 -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"   -gdwarf-2 -D__$(MP_PROCESSOR_OPTION)__  -Wl,-Map="dist\${CND_CONF}\${IMAGE_TYPE}\aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.map"    -o dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}      -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1 -Wl,--gc-sections -Wl,--start-group  -Wl,-lm -Wl,-lm -Wl,--end-group 
	
	${MP_CC_DIR}\\avr-objcopy -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.eep" || exit 0
	${MP_CC_DIR}\\avr-objdump -h -S "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" > "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.lss"
	${MP_CC_DIR}\\avr-objcopy -O srec -R .eeprom -R .fuse -R .lock -R .signature "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.srec"
	
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mmcu=attiny402 -B "C:/Program Files (x86)/Atmel/Studio/7.0/Packs/Atmel/ATtiny_DFP/1.4.310/gcc/dev/attiny402"  -D__$(MP_PROCESSOR_OPTION)__  -Wl,-Map="dist\${CND_CONF}\${IMAGE_TYPE}\aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.map"    -o dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}      -DXPRJ_Release=$(CND_CONF)  $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION) -Wl,--gc-sections -Wl,--start-group  -Wl,-lm -Wl,-lm -Wl,--end-group 
	${MP_CC_DIR}\\avr-objcopy -O ihex "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.hex"
	${MP_CC_DIR}\\avr-objcopy -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.eep" || exit 0
	${MP_CC_DIR}\\avr-objdump -h -S "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" > "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.lss"
	${MP_CC_DIR}\\avr-objcopy -O srec -R .eeprom -R .fuse -R .lock -R .signature "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}" "dist/${CND_CONF}/${IMAGE_TYPE}/aquarov_led_pwm_firmware.X.${IMAGE_TYPE}.srec"
	
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Release
	${RM} -r dist/Release

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
