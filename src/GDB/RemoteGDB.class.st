Class {
	#name : #RemoteGDB,
	#superclass : #GDB,
	#instVars : [
		'rsp',
		'exe'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
RemoteGDB class >> host: h port: p [
	"Connect to the remote server and perform initial handshake."
	^self host: h port: p rspClientClass: RSPClient
]

{ #category : #'instance creation' }
RemoteGDB class >> host: h port: p rspClientClass: rspc [
	"Connect to the remote server and perform initial handshake."
	^self basicNew
		rsp: (rspc host: h port: p);
		prepareSession;
		yourself
]

{ #category : #execution }
RemoteGDB >> cont [
	^rsp cont
]

{ #category : #registers }
RemoteGDB >> decodeGPacket: aStream [
	| regs |
	regs := Dictionary new.
	processorDescription regsInGPacket do: [ :rt |
		| nextValue |
		aStream atEnd ifTrue: [ self error: 'g-packet provided by stub is too short' ].
		nextValue := rt readFrom: aStream endian: processorDescription endian.
		regs at: rt regName put: nextValue ].
	aStream atEnd ifFalse: [ self error: 'g-packet provided by stub is too long' ].
	^regs


]

{ #category : #execution }
RemoteGDB >> disconnect [
	^rsp disconnect
]

{ #category : #'BOGUS - MT exe' }
RemoteGDB >> exe [
	^exe
]

{ #category : #'BOGUS - MT exe' }
RemoteGDB >> exe: aTAJExecutable [ 
	exe := aTAJExecutable
]

{ #category : #registers }
RemoteGDB >> findOutCPU [
	"Figure out which CPU we are on, and set processorDescription, regsInGPacket etc accordingly."
	rsp askFeatures 
		ifNil: [ NoFeatures signalFor: self ]
		ifNotNil: [ :pdl | processorDescription := pdl ]

]

{ #category : #GT }
RemoteGDB >> gtInspectorRegistersIn: composite [
	<gtInspectorPresentationOrder: 20>
	^composite fastTable
		title: 'Registers';
		display: [ self getRegisters associations ];
		column: 'Register' evaluated: [ :each | each key ];
		column: 'Hex' evaluated: [ :each |
			| x |
			x := each value.
			x isNil
				ifTrue: [ '?' ]
				ifFalse: [ 
					x isString 
						ifTrue: [ x ] ifFalse: [ x printStringHex ]]]
]

{ #category : #breakpoints }
RemoteGDB >> insertHWBreakpointAt: addr [
	rsp insertBreakpoint: 1 at: addr kind: self zKind
]

{ #category : #breakpoints }
RemoteGDB >> insertSWBreakpointAt: addr [
	rsp insertBreakpoint: 0 at: addr kind: self zKind
]

{ #category : #execution }
RemoteGDB >> kill [
	^rsp kill
]

{ #category : #'memory-compatibility' }
RemoteGDB >> memoryAt: addr put: aByteArray [
	self writeBytes: aByteArray toAddr: addr
]

{ #category : #GT }
RemoteGDB >> nameForInspector [
	^'GDB'
]

{ #category : #registers }
RemoteGDB >> prepareSession [
	self findOutCPU

]

{ #category : #memory }
RemoteGDB >> read32At: addr [
	| answer |
	answer := rsp q: 'm',
		addr printStringHex,
		',4'.
	answer := Integer readFrom: answer radix: 16.
	processorDescription endian ifLittle: [ answer := answer byteSwap32 ].
	^answer
]

{ #category : #memory }
RemoteGDB >> readAt: addr nBytes: n [
	| answer stream |
	answer := rsp q: 'm',
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

{ #category : #registers }
RemoteGDB >> readRegistersFromTarget [
	| answer stream |
	answer := rsp q: 'g'.
	stream := ReadStream on: answer
		from: 1
		to: answer size.

	^self decodeGPacket: stream
]

{ #category : #breakpoints }
RemoteGDB >> removeHWBreakpointAt: addr [
	rsp removeBreakpoint: 1 at: addr kind: self zKind
]

{ #category : #breakpoints }
RemoteGDB >> removeSWBreakpointAt: addr [
	rsp removeBreakpoint: 0 at: addr kind: self zKind
]

{ #category : #accessing }
RemoteGDB >> rsp [
	^ rsp
]

{ #category : #accessing }
RemoteGDB >> rsp: anObject [
	rsp := anObject
]

{ #category : #execution }
RemoteGDB >> runUntil: aBlock [
	[ self c ] doWhileFalse: aBlock
]

{ #category : #magick }
RemoteGDB >> runWithMagickCallback: magicCallbackBlock [
	[ true ] whileTrue: [ self runUntilMagick. magicCallbackBlock value ] 
]

{ #category : #registers }
RemoteGDB >> sendRegistersToTarget [
	rsp
		sendRegisters: registerCache
		transfers: processorDescription regsInGPacket
		endian: processorDescription endian
]

{ #category : #execution }
RemoteGDB >> stepUntil: aBlock [
	[ self s ] doWhileFalse: aBlock
]

{ #category : #magick }
RemoteGDB >> stepUntilMagick [
	self stepUntil: [ self currentInstructionEncoding = self exe objectMemory targetSetup magicInstruction ]
]

{ #category : #execution }
RemoteGDB >> stepi [
	^rsp stepi
]

{ #category : #memory }
RemoteGDB >> writeByte: int toAddr: addr [
	| textualAddr answer data |
	data := int printStringBase: 16 length: 2 padded: true.
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	answer := rsp
		q: 'M', textualAddr,  ',1:', data.
	answer = 'OK' ifFalse: [ self error: answer ].


]

{ #category : #memory }
RemoteGDB >> writeBytes: aByteArray toAddr: addr [
	| buffer textualAddr answer textualSize |
	buffer := WriteStream on: ''.
	aByteArray do: [ :aByte |
		| data |
		data := aByte printStringBase: 16 length: 2 padded: true.
		buffer nextPutAll: data
	].
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	textualSize := aByteArray size  printStringBase: 16.
	answer := rsp
		q: 'M', textualAddr,  ',', textualSize, ':', buffer contents.
	answer = 'OK' ifFalse: [ self error ].


]

{ #category : #memory }
RemoteGDB >> writeInt32: int toAddr: addr [
	self writeInt32s: { int } toAddr: addr
]

{ #category : #memory }
RemoteGDB >> writeInt32s: arrayOfInt32s toAddr: addr [
	| adjusted buffer textualAddr answer textualSize |
	adjusted := processorDescription endian 
							ifBig: [ arrayOfInt32s ] 
							ifLittle: [ arrayOfInt32s collect: #byteSwap32 ].
							
	buffer := WriteStream on: ''.
	adjusted do: [ :anInt32 |
		| data |
		data := anInt32 printStringBase: 16 length: 8 padded: true.
		buffer nextPutAll: data
	].
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	textualSize := arrayOfInt32s size * 4 printStringBase: 16.
	answer := rsp
		q: 'M', textualAddr,  ',', textualSize, ':', buffer contents.
	answer = 'OK' ifFalse: [ self error ].


]

{ #category : #breakpoints }
RemoteGDB >> zKind [
	^4 "FIXME. For now, just assume this"
]
