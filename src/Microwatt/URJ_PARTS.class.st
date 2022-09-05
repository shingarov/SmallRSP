Class {
	#name : #'URJ_PARTS',
	#superclass : #FFIStructure,
	#classVars : [
		'OFFSET_LEN',
		'OFFSET_PARTS'
	],
	#category : #'Microwatt-JTAG'
}

{ #category : #'field definition' }
URJ_PARTS class >> fieldsDesc [
	^ #(
		int len;
		void* parts;
	)
	"
	URJ_PARTS rebuildFieldAccessors
	"
]

{ #category : #'accessing structure variables' }
URJ_PARTS >> len [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_LEN
]

{ #category : #'accessing structure variables' }
URJ_PARTS >> len: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_LEN put: anObject
]

{ #category : #'accessing structure variables' }
URJ_PARTS >> parts [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_PARTS) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_PARTS >> parts: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_PARTS put: anObject getHandle.
]
