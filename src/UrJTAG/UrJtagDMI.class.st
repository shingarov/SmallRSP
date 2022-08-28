Class {
	#name : #UrJtagDMI,
	#superclass : #Object,
	#instVars : [
		'jtagChain'
	],
	#category : #UrJTAG
}

{ #category : #'instance creation' }
UrJtagDMI class >> new [
	| jc |
	jc := UrJtagChain new connectTo: 'DigilentNexysVideo'.
	jc readIDCODE = 16r13631093 ifFalse: [ self error: 'Wrong IDCODE' ].
	jc setUSER2.
	^self basicNew
		jtagChain: jc;
		yourself
]

{ #category : #accessing }
UrJtagDMI >> jtagChain [
	^ jtagChain
]

{ #category : #accessing }
UrJtagDMI >> jtagChain: anObject [
	jtagChain := anObject
]

{ #category : #API }
UrJtagDMI >> jtagCommand: op addr: addr data: data [
	| p insn dr d rc |
	d := data isNil
		ifTrue: [ ExternalAddress null ]
		ifFalse: [ data first ].
	p := jtagChain getPart.
	insn := p active_instruction.
	insn isNull ifTrue: [ self error ].
	dr := insn data_register.
	dr isNull ifTrue: [ self error ].
	rc := LibUrJTAG uniqueInstance urj_tap_register_set_value_bit_range: dr in _: (ExternalAddress fromAddress: op) _: 1 _: 0.
	rc = 0 ifFalse: [ URJError signalCurrent ].
	rc := LibUrJTAG uniqueInstance urj_tap_register_set_value_bit_range: dr in _: d _: 65 _: 2.
	rc = 0 ifFalse: [ URJError signalCurrent ].
	rc := LibUrJTAG uniqueInstance urj_tap_register_set_value_bit_range: dr in _: (ExternalAddress fromAddress: addr) _: 73 _: 66.
	rc = 0 ifFalse: [ URJError signalCurrent ].
	jtagChain shiftDataRegisters: 1.
	rc = 0 ifFalse: [ URJError signalCurrent ].
	rc := LibUrJTAG uniqueInstance urj_tap_register_get_value_bit_range: dr out _: 1 _: 0.
	data isNil ifFalse: [ 
		data at: 1 put: (LibUrJTAG uniqueInstance urj_tap_register_get_value_bit_range: dr out _: 65 _: 2) getHandle ].
	^rc asInteger
]

{ #category : #API }
UrJtagDMI >> read: addr [
	| data |
	data := FFIExternalArray newType: 'void*' size: 1. "uint64 does not work"
	self read: addr into: data.
	^data first asInteger
]

{ #category : #private }
UrJtagDMI >> read: addr into: data [
	self jtagCommand: 1 addr: addr data: data.
	[ true ] whileTrue: [
		| rc |
		rc := self jtagCommand: 0 addr: addr data: data.
		rc = 0 ifTrue: [ ^self ].
		rc = 3 ifFalse: [ self error: 'Unknown status code' ]
  ]
]

{ #category : #shit }
UrJtagDMI >> readNiaXXX [
	^self read: 18
]

{ #category : #shit }
UrJtagDMI >> readStatusXXX [
	^self read: 17
]

{ #category : #shit }
UrJtagDMI >> stopXXX [
	^self write: 16 data: 1
]

{ #category : #API }
UrJtagDMI >> write: addr data: anInteger [
	| data |
	data := FFIExternalArray newType: 'void*' size: 1. "uint64 does not work"
	data at: 1 put: (ExternalAddress fromAddress: anInteger).
	self jtagCommand: 2 addr: addr data: data.
	[ true ] whileTrue: [
		| rc |
		rc := self jtagCommand: 0 addr: addr data: nil.
		rc = 0 ifTrue: [ ^self ].
		rc = 3 ifFalse: [ self error: 'Unknown status code' ]
  ]
]
