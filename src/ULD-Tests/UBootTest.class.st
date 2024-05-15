Class {
	#name : #UBootTest,
	#superclass : #TestCase,
	#category : #'ULD-Tests'
}

{ #category : #tests }
UBootTest >> assertRunningThenStop [
	"Start from newly-powered-up Arty."
	self assert: MicrowattJTAG instance core_status isEmpty. "0 flags = RUNNING"
	MicrowattJTAG instance core_stop.
	self assert: (MicrowattJTAG instance core_status includes: #STOPPED)
]

{ #category : #tests }
UBootTest >> pathToELF [
	^ '/home/boris/work/FPGA/genesys-qflash/microwatt/litedram/build/genesys2/software/obj/sdram_init.elf'
	"^ '/home/boris/work/FPGA/genesys-qflash/u-boot/build/u-boot' "
]

{ #category : #tests }
UBootTest >> test1 [ 
	| baby |
	Z3Context createGlobalContext.
	baby := BabyCProgram elf: self pathToELF.
	"boot the board and wait until execution hangs; now we'll stop"
	baby gdb stop.
	baby yourself
]

{ #category : #tests }
UBootTest >> test2 [
	| baby |
	Z3Context createGlobalContext.
	baby := BabyCProgram elf: self pathToELF.
	"boot the board and wait until execution hangs; now we'll stop"
	self assertRunningThenStop .
	
	baby gdb jtag memAt: 0  put32: 16r3d40ff00. "lis r10, -256 which gives FFFFFFFF_FF000000"
	baby gdb jtag memAt: 4  put32: 16r7d4803a6. "mtlr r10"
	baby gdb jtag memAt: 8  put32: 16r4e800020. "blr"
	self halt.
	baby yourself
]
