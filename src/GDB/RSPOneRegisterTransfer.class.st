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
	| bytes |
	bytes := aStream next: width//4.
	isLittleEndian ifTrue: [ bytes := bytes reverseInt32Endianness ].
	"TODO - Different types (e.g. IEEE-754)"
	[^Integer readFrom: bytes base: 16] on: Error do: [ self halt ]
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
	bytes := value printStringBase: 16 length: self width // 4 padded: true.
	isLittleEndian ifTrue: [ bytes := bytes reverseInt32Endianness ].
	aStream nextPutAll: bytes
]
