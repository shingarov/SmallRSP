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

{ #category : #reading }
RSPOneRegisterTransfer >> readFrom: aStream endian: endian [
	| text int |
	text := aStream next: width//4.
	int := Integer readFrom: text base: 16.
	width = 32 ifTrue: [ endian = #little ifTrue: [ ^int byteSwap32 ] ].
	width = 64 ifTrue: [ endian = #little ifTrue: [ ^int byteSwap64 ] ].
	self error: 'Register of size neither 32 nor 64'
	"TODO - Different types (e.g. IEEE-754)"

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
	| value bytes |
	value := aDictionary at: self regName.
	width = 32 ifTrue: [ endian = #little ifTrue: [ value := value byteSwap32 ] ].
	width = 64 ifTrue: [ endian = #little ifTrue: [ value := value byteSwap64 ] ].
	bytes := value printStringBase: 16 length: self width // 4 padded: true.
	aStream nextPutAll: bytes
]
