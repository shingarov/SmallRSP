Class {
	#name : #RSPOneRegisterTransfer,
	#superclass : #Object,
	#instVars : [
		'regName',
		'width',
		'isLittleEndian',
		'regNum'
	],
	#category : #'GDB-RSP'
}

{ #category : #writing }
RSPOneRegisterTransfer >> encodeEnv: aDictionary endian: endian [
	| value bytes |
	value := aDictionary at: self regName.
	^self encodeValue: value endian: endian 
]

{ #category : #writing }
RSPOneRegisterTransfer >> encodeValue: value endian: endian [
	| v |
	v := value.
	width = 32 ifTrue: [ endian = #little ifTrue: [ v := v byteSwap32 ].
		^v printStringBase: 16 length: self width // 4 padded: true ].
	width = 64 ifTrue: [ endian = #little ifTrue: [ v := v byteSwap64 ].
		^v printStringBase: 16 length: self width // 4 padded: true ].
	^v
]

{ #category : #reading }
RSPOneRegisterTransfer >> readFrom: aStream endian: endian [
	| text int |
	text := aStream next: width//4.
	int := Integer readFrom: text base: 16.
	width = 32 ifTrue: [ ^endian = #little ifTrue: [ int byteSwap32 ] ifFalse: [ int ] ].
	width = 64 ifTrue: [ ^endian = #little ifTrue: [ int byteSwap64 ] ifFalse: [ int ] ].
	^text "TODO - what do we do with these i396 floats?"
]

{ #category : #accessing }
RSPOneRegisterTransfer >> regName [
	^ regName
]

{ #category : #accessing }
RSPOneRegisterTransfer >> regName: anObject [
	regName := anObject
]

{ #category : #accessing }
RSPOneRegisterTransfer >> regNum [
	^ regNum
]

{ #category : #accessing }
RSPOneRegisterTransfer >> regNum: anObject [
	regNum := anObject
]

{ #category : #accessing }
RSPOneRegisterTransfer >> width [
	^ width
]

{ #category : #accessing }
RSPOneRegisterTransfer >> width: anObject [
	width := anObject
]

{ #category : #writing }
RSPOneRegisterTransfer >> write: aDictionary to: aStream endian: endian [
	aStream nextPutAll: (self encodeEnv: aDictionary endian: endian)
]
