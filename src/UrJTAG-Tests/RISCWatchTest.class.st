Class {
	#name : #RISCWatchTest,
	#superclass : #TestCase,
	#instVars : [
		'chain',
		'part'
	],
	#pools : [
		'Dbcr0Fields'
	],
	#category : #'UrJTAG-Tests'
}

{ #category : #'as yet unclassified' }
RISCWatchTest >> detectChainLength [
	| chlen |
	part setInstruction: 'BYPASS'.
	chain shiftInstructions.
	chain captureDR.
	chlen := chain detectRegisterSize: 0.
]

{ #category : #settings }
RISCWatchTest >> driver [
	^'ARM-USB-OCD-H'
]

{ #category : #settings }
RISCWatchTest >> frequency [
	^10000
]

{ #category : #tests }
RISCWatchTest >> getRegister: r [
	((AcProcessorDescriptions powerpc instructionAt: 'mtspr')
		inEnvironment: (Dictionary
			with: 'sprf' -> 16r3f3 reverseSprf
			with: 'rs' -> r )) execInBdmPB: self.
	self ppcInstructionZero.
	^self readDBGR
]

{ #category : #tests }
RISCWatchTest >> jtagInstruction: instrName in33: anInteger prefix: zeroOrOne [
	| all allString |
	all := zeroOrOne << 32 | anInteger.
	allString := all printStringBase: 2 length: 33 padded: true.
	^self jtagInstruction: instrName stringIn: allString
]

{ #category : #tests }
RISCWatchTest >> jtagInstruction: instrName stringIn: binString [
	| instr data_register |
	part setInstruction: instrName.
	chain shiftInstructions.
	instr := part active_instruction.
	data_register := instr data_register.
	data_register in setString: binString.
	chain shiftDataRegisters: 1.
	^data_register out getValue
	
]

{ #category : #tests }
RISCWatchTest >> powerUpSequence [
	"Go through the power-up sequence.
	After this, the target should sit in 'Stopped by JTAG request' state"
	chain assertHALT.
	self assert: (self ppcMode: EDM | IRPT) equals: 1.
	chain deassertHALT.
	(Delay forMilliseconds: 10) wait.
	self assert: (self ppcMode: EDM | IRPT) equals: 1.
]

{ #category : #tests }
RISCWatchTest >> ppcInstruction: encoding [
	^self "should always return 1?"
		jtagInstruction: 'PPCINST'
		in33: encoding
		prefix: 1
]

{ #category : #tests }
RISCWatchTest >> ppcInstructionZero [
	^self
		jtagInstruction: 'PPCINST'
		in33: 0
		prefix: 0
]

{ #category : #tests }
RISCWatchTest >> ppcMode: flags [
	^self
		jtagInstruction: 'PPCMODE'
		in33: flags
		prefix: 1

]

{ #category : #tests }
RISCWatchTest >> ppcModeZero [
	^self
		jtagInstruction: 'PPCMODE'
		in33: 0
		prefix: 0

]

{ #category : #tests }
RISCWatchTest >> readDBGR [
	^self
		jtagInstruction: 'PPCDBGR'
		in33: 0
		prefix: 0

]

{ #category : #tests }
RISCWatchTest >> setRegister: r to: v [
	self writeDBGR: v.
	((AcProcessorDescriptions powerpc instructionAt: 'mfspr')
		inEnvironment: (Dictionary
			with: 'sprf' -> 16r3f3 reverseSprf
			with: 'rt' -> r )) execInBdmPB: self.
	self ppcInstructionZero
]

{ #category : #running }
RISCWatchTest >> setUp [
	| parts |
	chain := UrJtagChain new .
	chain connectTo: self driver.
	chain struct frequency: self frequency.
	self assert: chain struct frequency equals: self frequency.
	parts := chain detect.
	part := parts at: 1
]

{ #category : #tests }
RISCWatchTest >> test1 [
	"Go through the power-up sequence as captured.
	After this, the target should sit in 'Stopped by JTAG request' state"
	self halt.	
	chain assertHALT.
	self assert: (self ppcMode: RST_core | ICMP) equals: 1.
	(Delay forMilliseconds: 120) wait.
	self assert: (self jtagInstruction: 'IDCODE' stringIn: '00000000000000000000000000000000') equals: 16rFFFFFFFF "BDI does BYPASS here".
	self assert: (self ppcMode: EDM | IRPT) equals: 1.
	
	chain deassertHALT.
	(Delay forMilliseconds: 10) wait.
	self assert: self ppcInstructionZero equals: 1.
	self assert: self ppcInstructionZero equals: 1.
	self assert: (self ppcMode: EDM | IRPT) equals: 1.
	self assert: self ppcInstructionZero equals: 1.
	self assert: self ppcInstructionZero equals: 1.
	self assert: self ppcInstructionZero equals: 1.

]

{ #category : #tests }
RISCWatchTest >> test3plus4 [
	Z3Context createGlobalContext.
	self powerUpSequence.
	
	'
	lis 1, 0
	li 1, 3
	lis 2, 0
	li 2, 4
	add 0, 1, 2
	' execInBdmPB: self.

	self assert: (self getRegister: 0) equals: 7
]

{ #category : #tests }
RISCWatchTest >> test4 [

	self halt.

	self ppcInstruction: 16r7c13fba6
]

{ #category : #tests }
RISCWatchTest >> testAdd [
	Z3Context createGlobalContext.
	self powerUpSequence.
	
	self setRegister: 1 to: 42.
	self setRegister: 2 to: 1.
	'add 0, 1, 2' execInBdmPB: self.
	self assert: (self getRegister: 0) equals: 43
]

{ #category : #tests }
RISCWatchTest >> testDetectPPC440 [
	self assert: part getPartName equals: 'PowerPC 440EPx'.
	
	
	
]

{ #category : #tests }
RISCWatchTest >> testGo [
	self powerUpSequence. "Now we are stopped by JTAG request"
	self assert: (self ppcMode: 0) equals: 1. "i.e. DBCR0 = 0x00000000"
	"
	At this point, still not running.
	The ppcInstructionZero below actually resumes booting.
	"	
	self assert: (self ppcInstructionZero) equals: 16r100000. "or ppcModeZero, also works (and also returns 100000)"
	self assert: (self ppcInstructionZero) equals: 16r100000.

]

{ #category : #tests }
RISCWatchTest >> testPVR [
	| pvr |
	self powerUpSequence.
	
	self ppcInstruction: 16r7c1f42a6. "mfpvr   r0"
	self ppcInstructionZero.
	self ppcInstruction: 16r7c13fba6. "mtspr 0x3f3, r0"
	self ppcInstructionZero.
	
	pvr := self readDBGR.
	pvr = 16r216218d0 ifTrue: [ ^'Secure feature' ].
	pvr = 16r216218d4 ifTrue: [ ^'No secure feature' ].
	self assert: false
]

{ #category : #tests }
RISCWatchTest >> testReset [
self halt.
	chain assertHALT.
	self ppcMode: EDM | RST_system | ICMP.
	self ppcMode: EDM.
	
	chain deassertHALT.
	
	self ppcMode: EDM.
	self ppcModeZero .
	self ppcModeSync .
	
	self assert: self ppcModeZero equals: 16r100000
]

{ #category : #tests }
RISCWatchTest >> writeDBGR: v [
	^self
		jtagInstruction: 'PPCDBGR'
		in33: v
		prefix: 1

]
