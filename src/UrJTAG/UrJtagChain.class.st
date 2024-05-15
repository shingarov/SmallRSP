Class {
	#name : #UrJtagChain,
	#superclass : #Object,
	#instVars : [
		'chain',
		'parts'
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

{ #category : #API }
UrJtagChain >> assertHALT [
	self set_pod_signal: 16 to: 16
]

{ #category : #'as yet unclassified' }
UrJtagChain >> captureDR [
	LibUrJTAG uniqueInstance urj_tap_capture_dr: chain
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
UrJtagChain >> deassertHALT [
	self set_pod_signal: 16 to: 0
]

{ #category : #API }
UrJtagChain >> detect [
	| rc urjPartsPtr ptrs len |
	rc := LibUrJTAG uniqueInstance urj_tap_detect: chain _: 10.
	rc = 0 ifFalse: [ self error ].
	urjPartsPtr := self struct parts.
	len := urjPartsPtr len.
	ptrs := FFIExternalArray fromHandle: urjPartsPtr parts getHandle type: 'void*' size: len.
	parts := (1 to: len) collect: [ :j |
		URJ_PART fromHandle: (ptrs at: j) ].
	^parts "cache the Smalltalk objects"
]

{ #category : #'as yet unclassified' }
UrJtagChain >> detectRegisterSize: maxlen [ 
	^LibUrJTAG uniqueInstance urj_tap_detect_register_size: chain _: maxlen
]

{ #category : #API }
UrJtagChain >> free [
	LibUrJTAG uniqueInstance urj_tap_chain_free: chain
]

{ #category : #API }
UrJtagChain >> getPart [
	| p |
	p := LibUrJTAG uniqueInstance urj_tap_chain_active_part: chain.
	p isNull ifTrue: [ self error: 'Failed to get active JTAG part' ].
	^URJ_PART fromHandle: p getHandle
]

{ #category : #API }
UrJtagChain >> get_pod_signal: sig [
	"?????"
	^LibUrJTAG uniqueInstance urj_tap_chain_get_pod_signal: chain _: sig
]

{ #category : #GT }
UrJtagChain >> gtPartsIn: composite [
	<gtInspectorPresentationOrder: 50>
	composite list
		title: 'Parts';
		display: [ parts ];
		format: #getPartName
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
	instr := self getPart active_instruction.
	self shiftDataRegisters: 1.
	data_register := instr data_register.
	out := data_register out. "an URJ_TAP_REGISTER"
	^out getValue
]

{ #category : #API }
UrJtagChain >> setUSER2 [
	self getPart setInstruction: 'USER2'.
	self shiftInstructions
]

{ #category : #API }
UrJtagChain >> set_pod_signal: mask to: val [
	^LibUrJTAG uniqueInstance urj_tap_chain_set_pod_signal: chain _: mask _: val
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
