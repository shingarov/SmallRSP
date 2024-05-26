Class {
	#name : #PowerOCDTest,
	#superclass : #TestCase,
	#instVars : [
		'ocd'
	],
	#pools : [
		'Dbcr0Fields'
	],
	#category : #'PowerOCD-Tests'
}

{ #category : #'connection defaults' }
PowerOCDTest >> driver [
	^'ARM-USB-OCD-H'
]

{ #category : #'connection defaults' }
PowerOCDTest >> frequency [
	^10000
]

{ #category : #'connection defaults' }
PowerOCDTest >> partName [
	^'PowerPC 440EPx'
]

{ #category : #running }
PowerOCDTest >> setUp [
	| jtag |
	jtag := 	JTAGBDM new
		connectToDriver: self driver
		at: self frequency
		partName: self partName.
	ocd := PowerOCD on: jtag.
	ocd enterExternalDebug
]

{ #category : #tests }
PowerOCDTest >> test3plus4 [
	ocd exec: '
		lis 1, 0
		li 1, 3
		lis 2, 0
		li 2, 4
		add 0, 1, 2
	'.
	self assert: (ocd getGPR: 0) equals: 7
]

{ #category : #tests }
PowerOCDTest >> testAdd [
	ocd setGPR: 1 to: 42.
	ocd setGPR: 2 to: 1.
	ocd exec: 'add 0, 1, 2'.
	self assert: (ocd getGPR: 0) equals: 43
]

{ #category : #tests }
PowerOCDTest >> testBootAndStop [
	"After #setUp had put the target in External Debug,
	 let it run like in #testGo; after Linux boots,
	 put it in EDM ('STOPEED BY JTAG') and look at
	 things like MSR and PC, they should make sense
	 for when executing Linux."
	| pc pcAgain |
	"1. GO"
	self assert: (ocd jtag ppcMode: 0) equals: 1.
	self assert: ocd jtag getInstructionState equals: 16r100000.
	
	"2. Programmer watch Linux boot"
	self halt.
	
	"3. Shift in EDM"
	ocd jtag ppcMode: EDM | IRPT.
	self assert: ocd jtag getInstructionState equals: 1. "STOPPED BY JTAG"

	ocd do: [
		self assert: ocd getMSR equals: 16r29000.
		pc := ocd getPC.
		self assert: pc & 2r11 equals: 0. "aligned instruction address"
		self assert: pc & 16rFFFF0000 equals: 16rC0000000. "somewhere in the linux kernel"
		"when we did the bl 0x1000, we did not actually jump anywhere"
		pcAgain := ocd getPC.
		self assert: pcAgain equals: pc
	] saving: {0. 1}.
]

{ #category : #tests }
PowerOCDTest >> testGo [
	"Suppose Olimex ARM-USB-OCD-H is connected to Sequoia's
	 14-pin JTAG/COP port.  The probe is powered, then the target
	 is powered.  The low SRST# pin, connected to the PPC440's HOLD#
	 pin, keeps the target in hold.
	 When this tests starts, #setUp places the target in External
	 Debug and deasserts HOLD#."
	| jtag |
	jtag := ocd jtag.
	self assert: jtag getPPCMode equals: 1.
	self assert: jtag getInstructionState equals: 1.
	
	"Clear the EDM flag by shifting the new PPCMODE in."
	self assert: (jtag ppcMode: 0) equals: 1.
	
	"However, at this point we are still not running.
	 Touching either getInstructionState or getPPCMode will trigger
	 the change to actually happen."
	self assert: jtag getInstructionState equals: 16r100000.
	"now we are running (see the system booting).
	 the status keeps the same..."
	self assert: jtag getInstructionState equals: 16r100000.
	self assert: jtag getInstructionState equals: 16r100000.
	self assert: jtag getInstructionState equals: 16r100000.
]

{ #category : #tests }
PowerOCDTest >> testPVR [
	| pvr |
	pvr := ocd do: [ ocd getPVR ] saving: {0}.
	pvr = 16r216218d0 ifTrue: [ ^'Secure feature (Kasumi)' ]. "cf. linux/arch/powerpc/include/asm/processor.h"
	pvr = 16r216218d4 ifTrue: [ ^'No secure feature (Kasumi)' ].
	self assert: false
]

{ #category : #tests }
PowerOCDTest >> testReset [
	"Take the target out of EDM; after Linux boots,
	 trigger system reset..."
	
	"1. GO"
	self assert: (ocd jtag ppcMode: 0) equals: 1.
	ocd jtag getInstructionState.
	
	"2. Programmer watch Linux boot"
	self halt.
	
	"3. Shift in RESET"
	ocd jtag ppcMode: RST_system.
	ocd jtag getInstructionState. "starts rebooting again"
]

{ #category : #tests }
PowerOCDTest >> testSingleStep [
	| out pc insn nextPC1 |
	"1. GO"
	self assert: (ocd jtag ppcMode: 0) equals: 1.
	self assert: ocd jtag getInstructionState equals: 16r100000.
	
	"2. Programmer watch Linux boot"
	self halt.
	
	"3. Shift in EDM"
	ocd jtag ppcMode: EDM | IRPT.
	self assert: ocd jtag getInstructionState equals: 1. "STOPPED BY JTAG"

	"4. Put IAC breakpoint"
	ocd do: [
		self assert: ocd getMSR equals: 16r29000. "just to verify sanity"
		pc := ocd getPC.
		self assert: pc & 2r11 equals: 0. "aligned instruction address"
		self assert: pc & 16rFFFF0000 equals: 16rC0000000. "somewhere in the linux kernel"
		ocd jtag ppcInstruction: 16r80010000. "lwz r0, 0(r1)"
		ocd jtag getInstructionState .
		insn := ocd getGPR: 0. "TODO: look at insn, what if it branches?"
		nextPC1 := pc + 4.
		ocd setSPR: 16r138 "IAC1"  to: nextPC1.
		ocd setSPR: 16r134 "DBCR0" to: 16r80c00001.
		ocd setSPR: 16r130 "DBSR"  to: 16rFFFFFFFF.
		ocd exec: 'sync 0'.
		ocd exec: 'isync'.
	] saving: {0. 1}.
	
	"5. GO"
	self assert: (ocd jtag ppcMode: 0) equals: 1.
	ocd jtag getInstructionState. "we stop immediately so even here we are in EAC1, but just"
	self assert: ocd jtag getInstructionState equals: 16r1000001. "EAC1"
	
	"6. Put IAC breakpoint"
	ocd do: [
		self assert: ocd getPC equals: nextPC1
	] saving: {0}
]
