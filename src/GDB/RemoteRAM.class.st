Class {
	#name : #RemoteRAM,
	#superclass : #SimulationAddressSpace,
	#instVars : [
		'gdb'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
RemoteRAM class >> gdb: aRemoteGDB [
	^self basicNew 
		gdb: aRemoteGDB;
		yourself 
]

{ #category : #'target synchronization' }
RemoteRAM >> flush [
	"Do nothing because I write directly to the target's memory"
]

{ #category : #'debugger access' }
RemoteRAM >> gdb [
	^ gdb
]

{ #category : #'debugger access' }
RemoteRAM >> gdb: anObject [
	gdb := anObject
]

{ #category : #accessing }
RemoteRAM >> read32At: addr [
	| answer |
	answer := gdb q: 'm',
		addr printStringHex,
		',4'.
	gdb processorDescription endian == #little ifTrue: [ answer := answer reverseInt32Endianness ].
	^Integer readFrom: answer radix: 16
]

{ #category : #accessing }
RemoteRAM >> readAt: addr nBytes: n [
	| answer stream |
	answer := self q: 'm',
		addr printStringHex,
		',',
		n printStringHex.
	stream := ReadStream on: answer
		from: 1
		to: answer size.
	answer := ByteArray new: n.
	1 to: n do: [ :idx |
		| b |
		b := stream next: 2.
		answer at: idx put: (Integer readFrom: b base: 16) ].
	^answer
]

{ #category : #accessing }
RemoteRAM >> readByteAt: addr [
	^(self readAt: addr nBytes: 1) first
]

{ #category : #accessing }
RemoteRAM >> writeByte: int toAddr: addr [
	| textualAddr answer data |
	data := int printStringBase: 16 length: 2 padded: true.
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	answer := self
		q: 'M', textualAddr,  ',1:', data.
	answer = 'OK' ifFalse: [ self error: answer ]
]

{ #category : #accessing }
RemoteRAM >> writeBytes: aByteArray toAddr: addr [
	| buffer textualAddr answer textualSize |
	buffer := WriteStream on: ''.
	aByteArray do: [ :aByte |
		| data |
		data := aByte printStringBase: 16 length: 2 padded: true.
		buffer nextPutAll: data
	].
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	textualSize := aByteArray size  printStringBase: 16.
	answer := self
		q: 'M', textualAddr,  ',', textualSize, ':', buffer contents.
	answer = 'OK' ifFalse: [ self error: 'RSP protocol failure' ].


]

{ #category : #accessing }
RemoteRAM >> writeInt32: int toAddr: addr [
	| textualAddr answer data |
	data := int printStringBase: 16 length: 8 padded: true.
	gdb processorDescription endian == #little ifTrue: [ data := data reverseInt32Endianness ].
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	answer := self gdb
		q: 'M', textualAddr,  ',4:', data.
	answer = 'OK' ifFalse: [ self error: answer ]
]

{ #category : #accessing }
RemoteRAM >> writeInt32s: arrayOfInt32s toAddr: addr [
	| buffer textualAddr answer textualSize |
	buffer := WriteStream on: ''.
	arrayOfInt32s do: [ :anInt32 |
		| data |
		data := anInt32 printStringBase: 16 length: 8 padded: true.
		gdb processorDescription endian == #little ifTrue: [ data := data reverseInt32Endianness ].
		buffer nextPutAll: data
	].
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	textualSize := arrayOfInt32s size * 4 printStringBase: 16.
	answer := self
		q: 'M', textualAddr,  ',', textualSize, ':', buffer contents.
	answer = 'OK' ifFalse: [ self error: 'RSP protocol failure' ].


]
