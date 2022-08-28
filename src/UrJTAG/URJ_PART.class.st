Class {
	#name : #'URJ_PART',
	#superclass : #FFIStructure,
	#classVars : [
		'OFFSET_ACTIVE_INSTRUCTION',
		'OFFSET_ALIAS',
		'OFFSET_BOUNDARY_LENGTH',
		'OFFSET_BSBITS',
		'OFFSET_DATA_REGISTERS',
		'OFFSET_ID',
		'OFFSET_INSTRUCTIONS',
		'OFFSET_INSTRUCTION_LENGTH',
		'OFFSET_MANUFACTURER_NAME',
		'OFFSET_PARAMS',
		'OFFSET_PART_NAME',
		'OFFSET_SALIASES',
		'OFFSET_SIGNALS',
		'OFFSET_STEPPING'
	],
	#pools : [
		'URJ_ARRAYTYPES'
	],
	#category : #UrJTAG
}

{ #category : #'field definition' }
URJ_PART class >> fieldsDesc [
	^ #(
void* id;
char* alias;
URJ_PART_MANUFACTURER manufacturer_name;
URJ_PART_PART part_name;
URJ_PART_STEPPING  stepping;
void* signals;
void* saliases;
int instruction_length;
void* instructions;
URJ_PART_INSTRUCTION* active_instruction;
void* data_registers;
int boundary_length;
void* bsbits;
void* params;
	)
	"
	URJ_PART rebuildFieldAccessors
	"
]

{ #category : #'accessing structure variables' }
URJ_PART >> active_instruction [
	"This method was automatically generated"
	^URJ_PART_INSTRUCTION fromHandle: (handle pointerAt: OFFSET_ACTIVE_INSTRUCTION)
]

{ #category : #'accessing structure variables' }
URJ_PART >> active_instruction: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_ACTIVE_INSTRUCTION put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART >> alias [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_ALIAS) type: ExternalType char asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PART >> alias: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_ALIAS put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART >> boundary_length [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_BOUNDARY_LENGTH
]

{ #category : #'accessing structure variables' }
URJ_PART >> boundary_length: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_BOUNDARY_LENGTH put: anObject
]

{ #category : #'accessing structure variables' }
URJ_PART >> bsbits [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_BSBITS) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PART >> bsbits: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_BSBITS put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART >> data_registers [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_DATA_REGISTERS) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PART >> data_registers: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_DATA_REGISTERS put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART >> id [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_ID) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PART >> id: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_ID put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART >> instruction_length [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_INSTRUCTION_LENGTH
]

{ #category : #'accessing structure variables' }
URJ_PART >> instruction_length: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_INSTRUCTION_LENGTH put: anObject
]

{ #category : #'accessing structure variables' }
URJ_PART >> instructions [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_INSTRUCTIONS) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PART >> instructions: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_INSTRUCTIONS put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART >> manufacturer_name [
	"This method was automatically generated"
	^(FFITypeArray ofType: #FFICharacterType size: 26) fromHandle: (handle copyFrom: OFFSET_MANUFACTURER_NAME to: OFFSET_MANUFACTURER_NAME + 25)
]

{ #category : #'accessing structure variables' }
URJ_PART >> manufacturer_name: anObject [
	"This method was automatically generated"
	handle replaceFrom: OFFSET_MANUFACTURER_NAME to: OFFSET_MANUFACTURER_NAME + 25 with: anObject getHandle startingAt: 1
]

{ #category : #'accessing structure variables' }
URJ_PART >> params [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_PARAMS) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PART >> params: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_PARAMS put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART >> part_name [
	"This method was automatically generated"
	^(FFITypeArray ofType: #FFICharacterType size: 21) fromHandle: (handle copyFrom: OFFSET_PART_NAME to: OFFSET_PART_NAME + 20)
]

{ #category : #'accessing structure variables' }
URJ_PART >> part_name: anObject [
	"This method was automatically generated"
	handle replaceFrom: OFFSET_PART_NAME to: OFFSET_PART_NAME + 20 with: anObject getHandle startingAt: 1
]

{ #category : #private }
URJ_PART >> prepare [
	| rc |
	rc := LibUrJTAG uniqueInstance urj_part_data_register_define: self _: 'IDCODE_REG' _: 32.
	rc = 0 ifFalse: [ self error: 'JTAG failed to add IDCODE_REG register' ].
	
	(LibUrJTAG uniqueInstance urj_part_instruction_define: self _: 'IDCODE' _: '001001' _: 'IDCODE_REG') isNull
		ifTrue: [ self error: 'JTAG failed to add IDCODE instruction' ].

	rc := LibUrJTAG uniqueInstance urj_part_data_register_define: self _: 'USER2_REG' _: 74.
	rc = 0 ifFalse: [ self error: 'JTAG failed to add USER2_REG register' ].

	(LibUrJTAG uniqueInstance urj_part_instruction_define: self _: 'USER2' _: '000011' _: 'USER2_REG') isNull
		ifTrue: [ self error: 'JTAG failed to add IDCODE instruction' ].
		
	^self
]

{ #category : #'accessing structure variables' }
URJ_PART >> saliases [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_SALIASES) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PART >> saliases: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_SALIASES put: anObject getHandle.
]

{ #category : #API }
URJ_PART >> setInstruction: iname [
	LibUrJTAG uniqueInstance urj_part_set_instruction: self _: iname
]

{ #category : #'accessing structure variables' }
URJ_PART >> signals [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_SIGNALS) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PART >> signals: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_SIGNALS put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART >> stepping [
	"This method was automatically generated"
	^(FFITypeArray ofType: #FFICharacterType size: 9) fromHandle: (handle copyFrom: OFFSET_STEPPING to: OFFSET_STEPPING + 8)
]

{ #category : #'accessing structure variables' }
URJ_PART >> stepping: anObject [
	"This method was automatically generated"
	handle replaceFrom: OFFSET_STEPPING to: OFFSET_STEPPING + 8 with: anObject getHandle startingAt: 1
]
