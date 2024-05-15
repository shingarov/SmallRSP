Class {
	#name : #'URJ_DATA_REGISTER',
	#superclass : #FFIStructure,
	#classVars : [
		'OFFSET_IN',
		'OFFSET_NAME',
		'OFFSET_NEXT',
		'OFFSET_OUT'
	],
	#pools : [
		'URJ_ARRAYTYPES'
	],
	#category : #UrJTAG
}

{ #category : #'as yet unclassified' }
URJ_DATA_REGISTER class >> allFrom: ptr [ 
	| dr |
	ptr isNull ifTrue: [ ^#() ].
	dr := self fromHandle: ptr getHandle.
	^{dr}, (self allFrom: dr next)
]

{ #category : #'field definition' }
URJ_DATA_REGISTER class >> fieldsDesc [
	^ #(
URJ_DATA_REGISTER_NAME name;
URJ_TAP_REGISTER* in;
URJ_TAP_REGISTER* out;
void* next;
	)
	"
	URJ_DATA_REGISTER rebuildFieldAccessors
	"
]

{ #category : #API }
URJ_DATA_REGISTER >> getName [
	^String fromFFIExternalArray: self name
]

{ #category : #'accessing structure variables' }
URJ_DATA_REGISTER >> in [
	"This method was automatically generated"
	^URJ_TAP_REGISTER fromHandle: (handle pointerAt: OFFSET_IN)
]

{ #category : #'accessing structure variables' }
URJ_DATA_REGISTER >> in: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_IN put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_DATA_REGISTER >> name [
	"This method was automatically generated"
	^(FFITypeArray ofType: #FFIUInt8 size: 33) fromHandle: (handle copyFrom: OFFSET_NAME to: OFFSET_NAME + 32)
]

{ #category : #'accessing structure variables' }
URJ_DATA_REGISTER >> name: anObject [
	"This method was automatically generated"
	handle replaceFrom: OFFSET_NAME to: OFFSET_NAME + 32 with: anObject getHandle startingAt: 1
]

{ #category : #'accessing structure variables' }
URJ_DATA_REGISTER >> next [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_NEXT) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_DATA_REGISTER >> next: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_NEXT put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_DATA_REGISTER >> out [
	"This method was automatically generated"
	^URJ_TAP_REGISTER fromHandle: (handle pointerAt: OFFSET_OUT)
]

{ #category : #'accessing structure variables' }
URJ_DATA_REGISTER >> out: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_OUT put: anObject getHandle.
]

{ #category : #printing }
URJ_DATA_REGISTER >> printOn: aStream [
	aStream nextPutAll: self getName
]
