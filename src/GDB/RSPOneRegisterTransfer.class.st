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

{ #category : #accessing }
RSPOneRegisterTransfer >> isLittleEndian [
	^ isLittleEndian
]

{ #category : #accessing }
RSPOneRegisterTransfer >> isLittleEndian: anObject [
	isLittleEndian := anObject
]

{ #category : #reading }
RSPOneRegisterTransfer >> readFrom: aStream [
	| text int |
	text := aStream next: width//4.
	int := Integer readFrom: text base: 16.
	isLittleEndian ifTrue: [ int := int byteSwap32 ].
	"TODO - Different types (e.g. IEEE-754)"
	^int
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
RSPOneRegisterTransfer >> write: aDictionary to: aStream [
	| value bytes |
	value := aDictionary at: self regName.
	isLittleEndian ifTrue: [ value := value byteSwap32 ].
	bytes := value printStringBase: 16 length: self width // 4 padded: true.
	aStream nextPutAll: bytes
]
