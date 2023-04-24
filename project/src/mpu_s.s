
   .def setPSP ; has to be a tab apparently?
   .def setASP ; has
   .def getPSP
   .def getMSP
   .def setUnprivileged
   .def setPrivileged
   .def pushCore
   .def popCore
   .def setupUnrun
   .def extractR0
   .def triggerSvCall

.thumb

.const
PSR_VALUE 			.field 		0x61000000
LR_THREAD_NF_PSP_VALUE .field	0xFFFFFFFD

.text

; void setPSP(uint32_t address)
setPSP:
			MSR PSP, R0
			BX LR

setASP:
			MRS R0, CONTROL
			ORR R0, #2
			MSR CONTROL, R0
			ISB
			BX LR

getPSP:
			MRS R0, PSP
			BX LR

getMSP:
			MRS R0, MSP
			BX LR

setUnprivileged:
			MRS R0, CONTROL
			ORR R0, #1
			MSR CONTROL, R0
			BX LR


; just load 2 into the control register
; clears the 1 bit
setPrivileged:
			MRS R0, CONTROL
			ANDD R0, #2 ; 010b
			MSR CONTROL, R0
			BX LR

; void pushCore()
pushCore:
			MRS R0, PSP
			SUB R0, #4 ; get to the new stack area
			STR R4, [R0]
			SUB R0, #4
			STR R5, [R0]
			SUB R0, #4
			STR R6, [R0]
			SUB R0, #4
			STR R7, [R0]
			SUB R0, #4
			STR R8, [R0]
			SUB R0, #4
			STR R9, [R0]
			SUB R0, #4
			STR R10, [R0]
			SUB R0, #4
			STR R11, [R0]
			MSR PSP, R0
			BX LR

popCore:
			MRS R0, PSP
			LDR R11, [R0]
			ADD R0, #4
			LDR R10, [R0]
			ADD R0, #4
			LDR R9, [R0]
			ADD R0, #4
			LDR R8, [R0]
			ADD R0, #4
			LDR R7, [R0]
			ADD R0, #4
			LDR R6, [R0]
			ADD R0, #4
			LDR R5, [R0]
			ADD R0, #4
			LDR R4, [R0]
			ADD R0, #4
			MSR PSP, R0
			BX LR


; void setupUnrun(uint32_t function_pointer)
setupUnrun:
			MRS R2, PSP
			SUB R2, #4

			LDR R1, PSR_VALUE
			STR R1, [R2] ; xPSR
			SUB R2, #4

			STR R0, [R2] ; PC, take the previously stored parameter
			SUB R2, #4

			LDR R1, LR_THREAD_NF_PSP_VALUE
			STR R1, [R2] ; LR
			SUB R2, #4

			MOV R1, #0
			STR R1, [R2] ; R12
			SUB R2, #4
			STR R1, [R2] ; R3
			SUB R2, #4
			STR R1, [R2] ; R2
			SUB R2, #4
			STR R1, [R2] ; R1
			SUB R2, #4
			STR R1, [R2] ; R0

			MSR PSP, R2
			BX LR

; uint32_t extractR0(void)
extractR0:
			BX LR

triggerSvCall:

.endm
