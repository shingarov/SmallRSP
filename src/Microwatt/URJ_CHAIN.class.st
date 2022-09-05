Class {
	#name : #'URJ_CHAIN',
	#superclass : #FFIStructure,
	#classVars : [
		'OFFSET_ACTIVE_PART',
		'OFFSET_BSDL__DEBUG',
		'OFFSET_BSDL__PATH_LIST',
		'OFFSET_CABLE',
		'OFFSET_MAIN_PART',
		'OFFSET_PARTS',
		'OFFSET_STATE',
		'OFFSET_TOTAL_INSTR_LEN'
	],
	#category : #'Microwatt-JTAG'
}

{ #category : #'field definition' }
URJ_CHAIN class >> fieldsDesc [
	^ #(
		int state;
		URJ_PARTS* parts;
		int total_instr_len;
		int active_part;
		void* cable;
		char** bsdl__path_list;
		int bsdl__debug;
		int main_part;
	)
	"
	URJ_CHAIN rebuildFieldAccessors
	"
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> active_part [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_ACTIVE_PART
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> active_part: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_ACTIVE_PART put: anObject
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> bsdl__debug [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_BSDL__DEBUG
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> bsdl__debug: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_BSDL__DEBUG put: anObject
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> bsdl__path_list [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_BSDL__PATH_LIST) type: ExternalType char asPointerType
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> bsdl__path_list: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_BSDL__PATH_LIST put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> cable [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_CABLE) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> cable: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_CABLE put: anObject getHandle.
]

{ #category : #API }
URJ_CHAIN >> connectTo: driverName [
	| params |
	self main_part: 0.
	params := FFIExternalArray externalNewType: 'void*' size: 1.
	params at: 1 put: ExternalAddress null.
	(LibUrJTAG uniqueInstance urj_tap_chain_connect: self _: driverName _: params) = 0
		ifFalse: [ URJError signalCurrent  ].
	^self
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> main_part [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_MAIN_PART
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> main_part: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_MAIN_PART put: anObject
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> parts [
	"This method was automatically generated"
	^URJ_PARTS fromHandle: (handle pointerAt: OFFSET_PARTS)
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> parts: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_PARTS put: anObject getHandle.
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> state [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_STATE
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> state: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_STATE put: anObject
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> total_instr_len [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_TOTAL_INSTR_LEN
]

{ #category : #'accessing structure variables' }
URJ_CHAIN >> total_instr_len: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_TOTAL_INSTR_LEN put: anObject
]
