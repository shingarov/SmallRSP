Class {
	#name : #RemoteGDBSession,
	#superclass : #RemoteGDB,
	#instVars : [
		'qXfer',
		'vFile'
	],
	#category : #'GDB-RSP'
}

{ #category : #magick }
RemoteGDBSession >> advancePastTrap [
	| regs |
	regs := self getRegisters.
	regs at: 'pc' put: (regs at: 'pc') + 4.
	self setRegisters: regs.
]

{ #category : #'RSP commands' }
RemoteGDBSession >> askFeatures [
	" Ask the stub for the target feature descriptor, and return it.
	  If the stub doesn't provide it, return nil. "
	self supportsFeatures
		ifTrue: [ ^self qXfer features: 'target.xml' ]
		ifFalse: [ ^nil ]
		
]

{ #category : #accessing }
RemoteGDBSession >> getRegister: r [
	| regs |
	regs := self getRegisters.
	^regs at: r
]

{ #category : #'RSP commands' }
RemoteGDBSession >> pc [
	^self getRegister: self pcRegisterName 
]

{ #category : #'RSP commands' }
RemoteGDBSession >> pc: newPC [
	^self setRegister: self processorDescription pcRegisterName to: newPC
]

{ #category : #'RSP commands' }
RemoteGDBSession >> pcRegisterName [
	self processorDescription architectureName = 'IA32' ifTrue: [ ^'eip' ].
	self processorDescription architectureName = 'AMD64' ifTrue: [ ^'rip' ].
	^'pc'  "BOGUS: how do we know it's pc everywhere else??"
]

{ #category : #'RSP commands' }
RemoteGDBSession >> prepareSession [
	self qSupported.
	self vMustReplyEmpty.
	self packetSize.
	self setThread: 'g' t: 0.
	self qStatus = '' ifFalse: [ self error: 'Bad status' ].
	self vCont.

	self askFeatures 
		ifNil: [ self fillDefaultFeatures ]
		ifNotNil: [ :regs | processorDescription regsInGPacket: regs ]

]

{ #category : #printing }
RemoteGDBSession >> printRegistersOn: aStream [
	| allRegisters |
	allRegisters := self getRegisters.
	allRegisters keysAndValuesDo: [ :regName :regValue |
		(#('ds' 'ss' 'cs' 'es' 'fs' 'gs') includes: regName) ifFalse: [
			aStream nextPutAll: regName; nextPutAll: ': '.
			regValue printOn: aStream base: 16 length: 8 padded: true.
			aStream cr	
	]]
]

{ #category : #'RSP Helpers' }
RemoteGDBSession >> qXfer [
	qXfer isNil ifTrue: [ qXfer := RemoteGdbXFER gdb: self ].
	^ qXfer
]

{ #category : #'RSP commands' }
RemoteGDBSession >> read32At: addr [
	| answer |
	answer := self q: 'm',
		addr printStringHex,
		',4'.
	processorDescription endian == #little ifTrue: [ answer := answer reverseInt32Endianness ].
	^Integer readFrom: answer radix: 16
]

{ #category : #'RSP commands' }
RemoteGDBSession >> readAt: addr nBytes: n [
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

{ #category : #'RSP commands' }
RemoteGDBSession >> readByteAt: addr [
	^(self readAt: addr nBytes: 1) first
]

{ #category : #magick }
RemoteGDBSession >> runStepsWithMagickCallback: magicCallbackBlock [
	[ true ] whileTrue: [ self stepUntilMagick. magicCallbackBlock value ] 
]

{ #category : #'RSP commands' }
RemoteGDBSession >> runUntil: aBlock [
	[ self c ] doWhileFalse: aBlock
]

{ #category : #magick }
RemoteGDBSession >> runUntilMagick [
	self runUntil: [ self currentInstructionEncoding = (Integer readFrom: self exe objectMemory targetSetup magicInstruction radix: 2) ]
]

{ #category : #magick }
RemoteGDBSession >> runWithMagickCallback: magicCallbackBlock [
	[ true ] whileTrue: [ self runUntilMagick. magicCallbackBlock value ] 
]

{ #category : #accessing }
RemoteGDBSession >> setRegister: r to: x [
	self getRegisters at: r put: x

]

{ #category : #'RSP commands' }
RemoteGDBSession >> stepUntil: aBlock [
	[ self s.
	Transcript show: ((self getRegisters at: 'r19') printString); cr.
	] doWhileFalse: aBlock
]

{ #category : #magick }
RemoteGDBSession >> stepUntilMagick [
	self stepUntil: [ self currentInstructionEncoding = self exe objectMemory targetSetup magicInstruction ]
]

{ #category : #'RSP commands' }
RemoteGDBSession >> supportsFeatures [
	" Answer whether the stub supports target feature descriptors. "
	| features |
	features := supported detect: [ :s | s beginsWith: 'qXfer:features:' ] ifNone: [ ^false ].
	features = 'qXfer:features:read+' ifFalse: [ self error: 'Havent encountered such a CPU yet; investigate' ].
	^true
]

{ #category : #'RSP Helpers' }
RemoteGDBSession >> vFile [
	vFile isNil ifTrue: [ vFile := RemoteGdbVFILE gdb: self ].
	^ vFile
]

{ #category : #'RSP commands' }
RemoteGDBSession >> writeByte: int toAddr: addr [
	| textualAddr answer data |
	data := int printStringBase: 16 length: 2 padded: true.
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	answer := self
		q: 'M', textualAddr,  ',1:', data.
	answer = 'OK' ifFalse: [ self error: answer ].


]

{ #category : #'RSP commands' }
RemoteGDBSession >> writeBytes: aByteArray toAddr: addr [
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
	answer = 'OK' ifFalse: [ self halt ].


]

{ #category : #'RSP commands' }
RemoteGDBSession >> writeInt32: int toAddr: addr [
	| textualAddr answer data |
	data := int printStringBase: 16 length: 8 padded: true.
	processorDescription endian == #little ifTrue: [ data := data reverseInt32Endianness ].
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	answer := self
		q: 'M', textualAddr,  ',4:', data.
	answer = 'OK' ifFalse: [ self error: answer ].


]

{ #category : #'RSP commands' }
RemoteGDBSession >> writeInt32s: arrayOfInt32s toAddr: addr [
	| buffer textualAddr answer textualSize |
	buffer := WriteStream on: ''.
	arrayOfInt32s do: [ :anInt32 |
		| data |
		data := anInt32 printStringBase: 16 length: 8 padded: true.
		processorDescription endian == #little ifTrue: [ data := data reverseInt32Endianness ].
		buffer nextPutAll: data
	].
	textualAddr := addr printStringBase: 16 length: 8 padded: true.
	textualSize := arrayOfInt32s size * 4 printStringBase: 16.
	answer := self
		q: 'M', textualAddr,  ',', textualSize, ':', buffer contents.
	answer = 'OK' ifFalse: [ self halt ].


]
