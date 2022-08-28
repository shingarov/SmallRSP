Class {
	#name : #'URJ_TAP_REGISTER',
	#superclass : #FFIStructure,
	#classVars : [
		'OFFSET_DATA',
		'OFFSET_LEN',
		'OFFSET_STRING'
	],
	#pools : [
		'URJ_ARRAYTYPES'
	],
	#category : #UrJTAG
}

{ #category : #'field definition' }
URJ_TAP_REGISTER class >> fieldsDesc [
	^ #(
		char* data;
		int len;
		char* string;
	)
	"
	URJ_TAP_REGISTER rebuildFieldAccessors
	"
]

{ #category : #'accessing structure variables' }
URJ_TAP_REGISTER >> data [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_DATA) type: ExternalType char asPointerType
]

{ #category : #'accessing structure variables' }
URJ_TAP_REGISTER >> data: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_DATA put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_TAP_REGISTER >> len [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_LEN
]

{ #category : #'accessing structure variables' }
URJ_TAP_REGISTER >> len: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_LEN put: anObject
]

{ #category : #'accessing structure variables' }
URJ_TAP_REGISTER >> string [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_STRING) type: ExternalType char asPointerType
]

{ #category : #'accessing structure variables' }
URJ_TAP_REGISTER >> string: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_STRING put: anObject getHandle.
]
