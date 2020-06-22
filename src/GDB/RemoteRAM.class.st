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

{ #category : #reading }
RemoteRAM >> byteAt: index [
	"Compatibe with the ByteArray.
	Index is measured in bytes, and is 1-based.
	The return value is always unsigned."
	^self byteAtAddr: index - 1
]

{ #category : #writing }
RemoteRAM >> byteAt: index put: byte [
	self byteAtAddr: index - 1 put: byte
]

{ #category : #reading }
RemoteRAM >> byteAtAddr: addr [
	^self unsignedByteAtAddr: addr
]

{ #category : #writing }
RemoteRAM >> byteAtAddr: byteAddress put: byte [
	| textualAddr data |
	data := byte printStringBase: 16 length: 2 padded: true.
	textualAddr := byteAddress printStringBase: 16 length: 8 padded: true.
	^ self writeBytesHex: data toAddrHex: textualAddr hexSize: '1'
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

{ #category : #reading }
RemoteRAM >> long64At: byteIndex [ 
	^self signedLong64At: byteIndex
]

{ #category : #writing }
RemoteRAM >> long64At: byteIndex put: aValue [
	| lowBits mask wordIndex |
	(lowBits := byteIndex - 1 \\ 4) = 0 ifTrue:
		[self "N.B. Do the access that can fail first, before altering the receiver"
			longAt: byteIndex + 4 put: (aValue bitShift: -32);
			unsignedLongAt: byteIndex put: (aValue bitAnd: 16rffffffff).
		^aValue].
	"There will always be three accesses; two partial words and a full word in the middle"
	wordIndex := byteIndex - 1 // 4 + 1.
	aValue < 0
		ifTrue:
			[(aValue bitShift: -32) < -2147483648 ifTrue:
				[^self errorImproperStore]]
		ifFalse:
			[16r7FFFFFFF < (aValue bitShift: -32) ifTrue:
				[^self errorImproperStore]].
	mask := 16rFFFFFFFF bitShift: 4 - lowBits * -8.
	self at: wordIndex put: (((self at: wordIndex) bitAnd: mask) bitXor: ((aValue bitShift: lowBits * 8) bitAnd: mask bitInvert32)).
	self at: wordIndex + 1 put: ((aValue bitShift: 4 - lowBits * -8) bitAnd: 16rFFFFFFFF).
	self at: wordIndex + 2 put: (((self at: wordIndex + 2) bitAnd: mask bitInvert32) bitXor: ((aValue bitShift: 4 - lowBits + 4 * -8) bitAnd: mask)).
	^aValue
]

{ #category : #reading }
RemoteRAM >> longAt: byteIndex [
	^self unsignedLongAtAddr: byteIndex - 1 bigEndian: false
]

{ #category : #reading }
RemoteRAM >> longAt: anInteger bigEndian: aBoolean [ 
	^self unsignedLongAt: anInteger bigEndian: aBoolean 
]

{ #category : #writing }
RemoteRAM >> longAt: byteIndex put: int [
	^self unsignedLongAt: byteIndex put: int 

]

{ #category : #writing }
RemoteRAM >> longAt: byteIndex put: aValue bigEndian: bigEndian [
	"Compatibility with the ByteArray method of the same name."
	self longAtAddr: byteIndex  - 1 put: aValue bigEndian: bigEndian
]

{ #category : #writing }
RemoteRAM >> longAtAddr: addr put: aValue bigEndian: bigEndian [
	"Compatibility with the ByteArray method of the same name."
	| textualAddr data textualData |
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	data := bigEndian
		ifTrue: [ aValue ]
		ifFalse: [ aValue byteSwap32 ].
	textualData := data printStringBase: 16 length: 8 padded: true.
	self writeBytesHex: textualData
		toAddrHex: textualAddr
		hexSize: '4'
]

{ #category : #'RSP protocol' }
RemoteRAM >> read: n bytesAsHexAt: addr [
	"Answer the hex string the gdbserver will return to represent the n bytes read from address addr.
	All read operations are implemented on top of this primitive."
	^self gdb q: 'm',
		addr printStringHex,
		',',
		n printStringHex

]

{ #category : #reading }
RemoteRAM >> read: n bytesAt: index [
	^self read: n bytesAtAddr: index - 1
]

{ #category : #reading }
RemoteRAM >> read: n bytesAtAddr: addr [
	| answer stream |
	answer := self read: n bytesAsHexAt: addr.
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

{ #category : #'remote endian' }
RemoteRAM >> readInt32fromAddr: addr [
	"Read, using the REMOTE TARGET endianness."
	^self unsignedLongAtAddr: addr bigEndian: self isBigEndian
]

{ #category : #reading }
RemoteRAM >> signedLong64At: byteIndex [
	^(self unsignedLong64At: byteIndex) signedIntFromLong64

]

{ #category : #reading }
RemoteRAM >> unsignedByteAt: index [
	^self unsignedByteAtAddr: index - 1
]

{ #category : #writing }
RemoteRAM >> unsignedByteAt: index put: byte [ 
	^self byteAt: index put: byte
]

{ #category : #reading }
RemoteRAM >> unsignedByteAtAddr: addr [
	^(self read: 1 bytesAtAddr: addr) first
]

{ #category : #reading }
RemoteRAM >> unsignedLong64At: byteIndex [
	^self unsignedLong64AtAddr: byteIndex - 1
]

{ #category : #reading }
RemoteRAM >> unsignedLong64AtAddr: addr [
	| hiWord loWord |
	addr \\ 8 ~= 0 ifTrue: [self unalignedAccessError].
	loWord := self unsignedLongAtAddr: addr bigEndian: false.
	hiWord := self unsignedLongAtAddr: addr + 4 bigEndian: false.
	^hiWord = 0
		ifTrue: [loWord]
		ifFalse: [(hiWord bitShift: 32) + loWord]
]

{ #category : #reading }
RemoteRAM >> unsignedLongAt: byteIndex [ 
	^self unsignedLongAt: byteIndex bigEndian: self isBigEndian
]

{ #category : #reading }
RemoteRAM >> unsignedLongAt: byteIndex bigEndian: bigEndian [
	^self unsignedLongAtAddr: byteIndex - 1 bigEndian: bigEndian
]

{ #category : #writing }
RemoteRAM >> unsignedLongAt: byteIndex put: int [ 
	self longAtAddr: byteIndex - 1 put: int bigEndian: self isBigEndian
]

{ #category : #reading }
RemoteRAM >> unsignedLongAtAddr: addr bigEndian: bigEndian [
	| string int |
	string := self read: 4 bytesAsHexAt: addr.
	int := Integer readFrom: string radix: 16.
	bigEndian ifFalse: [ int := int byteSwap32 ].
	^int
]

{ #category : #reading }
RemoteRAM >> unsignedShortAt: byteIndex [ 
	^self unsignedShortAtAddr: byteIndex - 1
]

{ #category : #writing }
RemoteRAM >> unsignedShortAt: byteIndex put: aValue [
	"Compatiblity with the ByteArray & Alien methods of the same name."
	^self unsignedShortAtAddr: byteIndex - 1 put: aValue
]

{ #category : #writing }
RemoteRAM >> unsignedShortAtAddr: addr [ 
	^self unsignedShortAtAddr: addr bigEndian: self isBigEndian
]

{ #category : #reading }
RemoteRAM >> unsignedShortAtAddr: addr bigEndian: bigEndian [
	| string int |
	string := self read: 2 bytesAsHexAt: addr.
	int := Integer readFrom: string radix: 16.
	bigEndian ifFalse: [ int := int byteSwap16 ].
	^int
]

{ #category : #writing }
RemoteRAM >> unsignedShortAtAddr: addr put: aValue [
	"Compatiblity with the ByteArray & Alien methods."
	self isBigEndian ifTrue: [self error].
	(aValue < 0 or: [aValue > 16rFFFF])  ifTrue: [^self errorImproperStore].
	(addr bitAnd: 1) = 0 ifFalse: [ self error: 'Unaligned short store not yet implemented' ].
	self byteAtAddr: addr put: (aValue bitAnd: 16rFF).
	self byteAtAddr: addr+1 put: (aValue bitAnd: 16rFF00)>>8.
	^aValue
]

{ #category : #writing }
RemoteRAM >> writeBytes: aByteArray toAddr: addr [
	| buffer textualAddr textualSize |
	buffer := WriteStream on: (String new: aByteArray size * 2).
	aByteArray do: [ :aByte |
		| data |
		data := aByte printStringBase: 16 length: 2 padded: true.
		buffer nextPutAll: data
	].
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	textualSize := aByteArray size  printStringBase: 16.
	self writeBytesHex: buffer contents
		toAddrHex: textualAddr
		hexSize: textualSize
]

{ #category : #'RSP protocol' }
RemoteRAM >> writeBytesHex: aString toAddrHex: addr hexSize: s [
	| answer |
	answer := gdb
		q: 'M', addr,  ',', s, ':', aString.
	answer = 'OK' ifFalse: [ self error: 'RSP protocol failure' ].


]

{ #category : #writing }
RemoteRAM >> writeInt32: int toAddr: addr [
	"Use the REMOTE TARGET's endianness."
	self longAtAddr: addr put: int bigEndian: self isBigEndian
]

{ #category : #writing }
RemoteRAM >> writeInt32s: arrayOfInt32s toAddr: addr [
	"Using the remote endianness."
	self writeInt32s: arrayOfInt32s toAddr: addr bigEndian: self isBigEndian

]

{ #category : #writing }
RemoteRAM >> writeInt32s: arrayOfInt32s toAddr: addr bigEndian: bigEndian [
	"Using the remote endianness."
	| buffer textualAddr textualSize |
	buffer := WriteStream on: ''.
	arrayOfInt32s do: [ :anInt32 |
		| data textualData |
		data := bigEndian
			ifTrue: [ anInt32 ]
			ifFalse: [ anInt32 byteSwap32 ].
		textualData := data printStringBase: 16 length: 8 padded: true.
		buffer nextPutAll: textualData
	].
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	textualSize := arrayOfInt32s size * 4 printStringBase: 16.
	self writeBytesHex: buffer contents
		toAddrHex: textualAddr
		hexSize: textualSize
]
