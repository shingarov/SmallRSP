Class {
	#name : #JTAGBDM,
	#superclass : #Object,
	#instVars : [
		'chain',
		'part'
	],
	#category : #PowerOCD
}

{ #category : #'HALT pin' }
JTAGBDM >> assertHALT [
	chain assertHALT
]

{ #category : #'connection open/close' }
JTAGBDM >> connectToDriver: driverName at: frequency partId: id [
	| parts |
	chain := UrJtagChain new.
	chain connectTo: driverName.
	chain struct frequency: frequency.
	parts := chain detect.
	part := parts detect: [ :eachPart | eachPart getIDCode idReg = id ]
]

{ #category : #'connection open/close' }
JTAGBDM >> connectToDriver: driverName at: frequency partName: p [
	| parts |
	chain := UrJtagChain new.
	chain connectTo: driverName.
	chain struct frequency: frequency.
	parts := chain detect.
	part := parts detect: [ :eachPart | eachPart getPartName = p ]
]

{ #category : #'HALT pin' }
JTAGBDM >> deassertHALT [
	chain deassertHALT
]

{ #category : #instructions }
JTAGBDM >> getInstructionState [
	^self
		jtagInstruction: 'PPCINST'
		in33: 0
		prefix: 0
]

{ #category : #instructions }
JTAGBDM >> getPPCMode [
	^self
		jtagInstruction: 'PPCMODE'
		in33: 0
		prefix: 0
]

{ #category : #instructions }
JTAGBDM >> jtagInstruction: instrName in33: anInteger prefix: zeroOrOne [
	| all allString |
	all := zeroOrOne << 32 | anInteger.
	allString := all printStringBase: 2 length: 33 padded: true.
	^self jtagInstruction: instrName stringIn: allString
]

{ #category : #instructions }
JTAGBDM >> jtagInstruction: instrName stringIn: binString [
	| instr data_register |
	part setInstruction: instrName.
	chain shiftInstructions.
	instr := part active_instruction.
	data_register := instr data_register.
	data_register in setString: binString.
	chain shiftDataRegisters: 1.
	^data_register out getValue
]

{ #category : #instructions }
JTAGBDM >> ppcInstruction: encoding [
	^self
		jtagInstruction: 'PPCINST'
		in33: encoding
		prefix: 1
]

{ #category : #instructions }
JTAGBDM >> ppcMode: flags [
	^self
		jtagInstruction: 'PPCMODE'
		in33: flags
		prefix: 1
]

{ #category : #instructions }
JTAGBDM >> readDBGR [
	^self
		jtagInstruction: 'PPCDBGR'
		in33: 0
		prefix: 0
]

{ #category : #instructions }
JTAGBDM >> writeDBGR: v [ 
	^self
		jtagInstruction: 'PPCDBGR'
		in33: v
		prefix: 1
]
