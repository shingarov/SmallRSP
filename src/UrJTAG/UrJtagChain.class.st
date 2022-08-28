Class {
	#name : #UrJtagChain,
	#superclass : #Object,
	#instVars : [
		'chain'
	],
	#category : #UrJTAG
}

{ #category : #'instance creation' }
UrJtagChain class >> new [
	| ptr |
	ptr := LibUrJTAG uniqueInstance urj_tap_chain_alloc.
	^self basicNew
		chain: ptr;
		yourself
]

{ #category : #private }
UrJtagChain >> chain [
	^ chain
]

{ #category : #private }
UrJtagChain >> chain: anObject [
	chain := anObject
]

{ #category : #API }
UrJtagChain >> connectTo: driverName [
	| struct params |
	struct := self struct.
	struct main_part: 0.
	params := FFIExternalArray externalNewType: 'void*' size: 1.
	params at: 1 put: ExternalAddress null.
	(LibUrJTAG uniqueInstance urj_tap_chain_connect: chain _: driverName _: params) = 0
		ifFalse: [ URJError signalCurrent  ].
	self preparePart0.
	^self
]

{ #category : #API }
UrJtagChain >> getPart [
	| p |
	p := LibUrJTAG uniqueInstance urj_tap_chain_active_part: chain.
	p isNull ifTrue: [ self error: 'Failed to get active JTAG part' ].
	^URJ_PART fromHandle: p getHandle
]

{ #category : #private }
UrJtagChain >> preparePart0 [
	"Hard wire part 0 (that might need to change).
	 Answer the just-prepared UrjPart."
	| rc partObject |
	rc := LibUrJTAG uniqueInstance urj_tap_manual_add: chain _: 6.
	rc < 0 ifTrue: [ self error: 'JTAG failed to add part' ].
	self struct parts len = 0 ifTrue: [ self error: 'JTAG Something''s wrong after adding part' ].
	
	LibUrJTAG uniqueInstance urj_part_parts_set_instruction: self struct parts _: 'BYPASS'.
	self struct active_part: 0.
	partObject := self getPart.
	^partObject prepare.
]

{ #category : #API }
UrJtagChain >> readIDCODE [
	| instr data_register out |
	self getPart setInstruction: 'IDCODE'.
	self shiftInstructions.
	self shiftDataRegisters: 1.
	instr := self getPart active_instruction.
	data_register := instr data_register.
	out := data_register out.
	^(LibUrJTAG uniqueInstance urj_tap_register_get_value: out) asInteger.

]

{ #category : #API }
UrJtagChain >> setUSER2 [
	self getPart setInstruction: 'USER2'.
	self shiftInstructions
]

{ #category : #private }
UrJtagChain >> shiftDataRegisters: captureOutput [
	| rc |
	rc := LibUrJTAG uniqueInstance urj_tap_chain_shift_data_registers: chain _: captureOutput.
	rc = 0 ifFalse: [ URJError signalCurrent ]
]

{ #category : #private }
UrJtagChain >> shiftInstructions [
	| rc |
	rc := LibUrJTAG uniqueInstance urj_tap_chain_shift_instructions: chain.
	rc = 0 ifFalse: [ URJError signalCurrent ].
]

{ #category : #private }
UrJtagChain >> struct [
	^URJ_CHAIN fromHandle: chain getHandle
]
