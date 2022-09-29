Class {
	#name : #UBootTest,
	#superclass : #TestCase,
	#category : #'ULD-Tests'
}

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
