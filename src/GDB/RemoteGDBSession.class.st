Class {
	#name : #RemoteGDBSession,
	#superclass : #RemoteGDB,
	#instVars : [
		'qXfer',
		'vFile',
		'memory'
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

{ #category : #'RSP commands' }
RemoteGDBSession >> currentInstruction [
	^processorDescription disassemble: self currentInstructionEncoding

]

{ #category : #'RSP commands' }
RemoteGDBSession >> currentInstructionEncoding [
	| pc |
	pc := 	self pc.
	^self memory read32At: pc
]

{ #category : #accessing }
RemoteGDBSession >> memory [
	^ memory
]

{ #category : #accessing }
RemoteGDBSession >> memory: anObject [
	memory := anObject
]

{ #category : #'RSP commands' }
RemoteGDBSession >> pc [
	^self getRegisters at: self pcRegisterName 
]

{ #category : #'RSP commands' }
RemoteGDBSession >> pcRegisterName [
	^'pc'  "BOGUS"
]

{ #category : #'RSP commands' }
RemoteGDBSession >> prepareSession [
	self qSupported.
	self packetSize.
	self setThread: 'g' t: 0.
	self qStatus = '' ifFalse: [ self error: 'Bad status' ].
	self vCont.

	self askFeatures ifNotNil: [ :xml | processorDescription regsInGPacket: (GdbXmlParser new processXML: xml) ]

]

{ #category : #'RSP Helpers' }
RemoteGDBSession >> qXfer [
	qXfer isNil ifTrue: [ qXfer := RemoteGdbXFER gdb: self ].
	^ qXfer
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
	| regs |
	regs := self getRegisters.
	regs at: r put: x.
	self setRegisters: regs
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
	"In reality the following line must be, ^true."
	^false "disabled for now, before we have xi:include in the XML parser"

]

{ #category : #'RSP Helpers' }
RemoteGDBSession >> vFile [
	vFile isNil ifTrue: [ vFile := RemoteGdbVFILE gdb: self ].
	^ vFile
]
