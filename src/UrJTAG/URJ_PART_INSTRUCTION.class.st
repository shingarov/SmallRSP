Class {
	#name : #'URJ_PART_INSTRUCTION',
	#superclass : #FFIStructure,
	#classVars : [
		'OFFSET_DATA_REGISTER',
		'OFFSET_NAME',
		'OFFSET_NEXT',
		'OFFSET_OUT',
		'OFFSET_VALUE'
	],
	#pools : [
		'URJ_ARRAYTYPES'
	],
	#category : #UrJTAG
}

{ #category : #'field definition' }
URJ_PART_INSTRUCTION class >> fieldsDesc [
	^ #(
		URJ_PART_INSTRUCTION_INSTRUCTION name;
		URJ_TAP_REGISTER* value;
    URJ_TAP_REGISTER* out;
    URJ_DATA_REGISTER* data_register;
    void* next;
	)
	"
	URJ_PART_INSTRUCTION rebuildFieldAccessors
	"
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> data_register [
	"This method was automatically generated"
	^URJ_DATA_REGISTER fromHandle: (handle pointerAt: OFFSET_DATA_REGISTER)
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> data_register: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_DATA_REGISTER put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> name [
	"This method was automatically generated"
	^(FFITypeArray ofType: #FFICharacterType size: 21) fromHandle: (handle copyFrom: OFFSET_NAME to: OFFSET_NAME + 20)
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> name: anObject [
	"This method was automatically generated"
	handle replaceFrom: OFFSET_NAME to: OFFSET_NAME + 20 with: anObject getHandle startingAt: 1
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> next [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_NEXT) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> next: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_NEXT put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> out [
	"This method was automatically generated"
	^URJ_TAP_REGISTER fromHandle: (handle pointerAt: OFFSET_OUT)
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> out: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_OUT put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> value [
	"This method was automatically generated"
	^URJ_TAP_REGISTER fromHandle: (handle pointerAt: OFFSET_VALUE)
]

{ #category : #'accessing structure variables' }
URJ_PART_INSTRUCTION >> value: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_VALUE put: anObject getHandle.
]
