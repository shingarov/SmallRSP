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

{ #category : #API }
URJ_TAP_REGISTER >> getString [
	^LibUrJTAG uniqueInstance urj_tap_register_get_string: self
]

{ #category : #API }
URJ_TAP_REGISTER >> getValue [
	^(LibUrJTAG uniqueInstance urj_tap_register_get_value: self) value
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

{ #category : #API }
URJ_TAP_REGISTER >> setString: s [
	| rc |
	rc := LibUrJTAG uniqueInstance urj_tap_register_set_string: self _: s.
	rc = 0 ifFalse: [ self error ].
]

{ #category : #API }
URJ_TAP_REGISTER >> setValue: v [
	| rc |
	rc := LibUrJTAG uniqueInstance urj_tap_register_set_value: self _: v.
	rc = 0 ifFalse: [ self error ].
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
