Class {
	#name : #PowerOCD,
	#superclass : #Object,
	#instVars : [
		'jtag'
	],
	#pools : [
		'Dbcr0Fields',
		'PPC440SPRs',
		'PowerSPRs'
	],
	#category : #PowerOCD
}

{ #category : #'instance creation' }
PowerOCD class >> on: aJTAGBDM [
	^self basicNew
		jtag: aJTAGBDM;
		yourself
]

{ #category : #bracketing }
PowerOCD >> do: aBlock saving: registers [
	| savedRegisterValues result |
	savedRegisterValues := registers collect: [ :eachReg |
		eachReg -> (self getGPR: eachReg) ].
	result := OCDScratchRegisters answer: registers do: aBlock.
	savedRegisterValues collectAssociations: [ :r :v | self setGPR: r to: v ].
	^result
]

{ #category : #'power-up' }
PowerOCD >> enterExternalDebug [
	"Place the target in External Debug Mode."
	jtag assertHALT.
	(jtag ppcMode: EDM | IRPT) = 1 ifFalse: [ self error ].
	jtag deassertHALT.
	(Delay forMilliseconds: 10) wait.
	(jtag ppcMode: EDM | IRPT) = 1 ifFalse: [ self error ].

]

{ #category : #'instruction stuffing' }
PowerOCD >> exec: code [
	^code execInBdmPB: jtag
]

{ #category : #registers }
PowerOCD >> getGPR: r [
	((AcProcessorDescriptions powerpc instructionAt: 'mtspr')
		inEnvironment: (Dictionary
			with: 'sprf' -> DBDR reverseSprf
			with: 'rs' -> r )) execInBdmPB: jtag.
	jtag getInstructionState.
	^jtag readDBGR
]

{ #category : #'specific registers' }
PowerOCD >> getLR [
	^self getSPR: LR
]

{ #category : #registers }
PowerOCD >> getMSR [
	| tmpR |
	tmpR := OCDScratchRegisters query first.
	((AcProcessorDescriptions powerpc instructionAt: 'mfmsr')
		inEnvironment: (Dictionary with: 'rt' -> tmpR )) execInBdmPB: jtag.
	jtag getInstructionState.
	^self getGPR: tmpR
]

{ #category : #registers }
PowerOCD >> getPC [
	| oldLR pc rt sprf |
	oldLR := self getLR.
	jtag ppcInstruction: 16r48000001. "bl 0x1000"

	rt := OCDScratchRegisters query first.
	sprf := LR reverseSprf.
	(AcProcessorDescriptions powerpc instructionAt: 'mfspr')
		inThisEnvironment
			execInBdmPB: jtag.
	pc := (self getGPR: rt) - 4.
	self setLR: oldLR.
	^pc
]

{ #category : #'specific registers' }
PowerOCD >> getPVR [
	^self getSPR: PVR
]

{ #category : #registers }
PowerOCD >> getSPR: spr [
	| rt sprf |
	sprf := spr reverseSprf.
	rt := OCDScratchRegisters query first.
	(AcProcessorDescriptions powerpc instructionAt: 'mfspr')
		inThisEnvironment
			execInBdmPB: jtag.
	jtag getInstructionState.
	^self getGPR: rt
]

{ #category : #'specific registers' }
PowerOCD >> getXER [
	^self getSPR: XER
]

{ #category : #accessing }
PowerOCD >> jtag [
	^ jtag
]

{ #category : #accessing }
PowerOCD >> jtag: anObject [
	jtag := anObject
]

{ #category : #registers }
PowerOCD >> setGPR: r to: v [
	jtag writeDBGR: v.
	((AcProcessorDescriptions powerpc instructionAt: 'mfspr')
		inEnvironment: (Dictionary
			with: 'sprf' -> DBDR reverseSprf
			with: 'rt' -> r )) execInBdmPB: jtag.
	jtag getInstructionState
]

{ #category : #'specific registers' }
PowerOCD >> setLR: v [
	self setGPR: LR to: v
]

{ #category : #registers }
PowerOCD >> setSPR: sprf to: v [
	| rs |
	rs := OCDScratchRegisters query first.
	self setGPR: rs to: v.
	((AcProcessorDescriptions powerpc instructionAt: 'mtspr')
		inEnvironment: (Dictionary
			with: 'sprf' -> sprf reverseSprf
			with: 'rs' -> rs )) execInBdmPB: jtag.
	jtag getInstructionState
]
