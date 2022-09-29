Class {
	#name : #'ULDc_LDIResult',
	#superclass : #FFIStructure,
	#classVars : [
		'OFFSET_ACLASS',
		'OFFSET_ADDRESS',
		'OFFSET_ENDPC',
		'OFFSET_FILENAMEPTR',
		'OFFSET_FILENAME_FOR_IDPTR',
		'OFFSET_LINE',
		'OFFSET_STARTPC',
		'OFFSET_SYMBOLNAMEPTR'
	],
	#category : #ULDc
}

{ #category : #'field definition' }
ULDc_LDIResult class >> fieldsDesc [
	^ #(
		void* startPC;
		void* endPC;
		int line;
		String filenamePtr;
		String filename_for_idPtr;
		String symbolNamePtr;
		int aclass;
		void* address;
	)
	"
	ULDc_LDIResult rebuildFieldAccessors
	"
]

{ #category : #comparing }
ULDc_LDIResult >> = anotherLDI [
	anotherLDI class = self class ifFalse: [ ^false ].
	self filename = anotherLDI filename ifFalse: [ ^false ].
	self line = anotherLDI line ifFalse: [ ^false ].
	^true
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> aclass [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_ACLASS
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> aclass: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_ACLASS put: anObject
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> address [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_ADDRESS) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> address: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_ADDRESS put: anObject getHandle.
]

{ #category : #'as yet unclassified' }
ULDc_LDIResult >> decoratedText [
	| str txt interval |
	str := (FileSystem / self filename_for_id) contents.
	"horrible hack, to update the inspector"
	str := str, (String with: (Character value: 64 + 20 atRandom)).
	interval := str lineIntervals at: self line.
	txt := str asText.
	txt addAttribute: TextBackgroundColor cyan from: interval first to: interval last.
	^txt

]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> endPC [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_ENDPC) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> endPC: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_ENDPC put: anObject getHandle.
]

{ #category : #'string access' }
ULDc_LDIResult >> filename [
	^self filenamePtr fromCString
]

{ #category : #'as yet unclassified' }
ULDc_LDIResult >> filenameForTab [
	^(FileSystem / self filename) basename
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> filenamePtr [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_FILENAMEPTR) type: ExternalType char asPointerType
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> filenamePtr: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_FILENAMEPTR put: anObject getHandle.
]

{ #category : #'string access' }
ULDc_LDIResult >> filename_for_id [
	^self filename_for_idPtr fromCString
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> filename_for_idPtr [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_FILENAME_FOR_IDPTR) type: ExternalType char asPointerType
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> filename_for_idPtr: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_FILENAME_FOR_IDPTR put: anObject getHandle.
]

{ #category : #comparing }
ULDc_LDIResult >> hash [
	^self line hash
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> line [
	"This method was automatically generated"
	^handle signedLongAt: OFFSET_LINE
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> line: anObject [
	"This method was automatically generated"
	handle signedLongAt: OFFSET_LINE put: anObject
]

{ #category : #printing }
ULDc_LDIResult >> printOn: aStream [
	"aStream
		nextPutAll: self symbolName;
		nextPutAll: ' in';
		cr;
		nextPutAll: self filename;
		nextPutAll: ':'.
	self line printOn: aStream.
	aStream cr; nextPutAll: 'pc: 0x'.
	self startPC getHandle asInteger printOn: aStream base: 16.
	aStream nextPutAll: '..'.
	self endPC getHandle asInteger printOn: aStream base: 16."
aStream nextPutAll: 'temporary, no printing LDIResult'
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> startPC [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_STARTPC) type: ExternalType void asPointerType
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> startPC: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_STARTPC put: anObject getHandle.
]

{ #category : #'string access' }
ULDc_LDIResult >> symbolName [
	^self symbolNamePtr fromCString
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> symbolNamePtr [
	"This method was automatically generated"
	^ExternalData fromHandle: (handle pointerAt: OFFSET_SYMBOLNAMEPTR) type: ExternalType char asPointerType
]

{ #category : #'accessing structure variables' }
ULDc_LDIResult >> symbolNamePtr: anObject [
	"This method was automatically generated"
	handle pointerAt: OFFSET_SYMBOLNAMEPTR put: anObject getHandle.
]
