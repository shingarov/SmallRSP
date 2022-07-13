Class {
	#name : #RemoteGDB,
	#superclass : #RemoteGDBTransport,
	#instVars : [
		'packetSize',
		'processorDescription',
		'tStatus',
		'why',
		'supported',
		'vCont',
		'registerCache'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
RemoteGDB class >> host: h port: p [
	"Create an instance, connect to the remote server, and fully prepare the debugging session."
	^(super host: h port: p)
		prepareSession;
		yourself
]

{ #category : #tests }
RemoteGDB class >> testArm1 [
	self testInstance testArm1 
]

{ #category : #tests }
RemoteGDB class >> testGem5 [
	^(RemoteGDBSession
		host: '192.168.75.2'
		port: 7000
		processorDescription: AcProcessorDescriptions powerpc) testGemAllRegs 
]

{ #category : #tests }
RemoteGDB class >> testInstance [
	^RemoteGDBSession
		host: 'parmenides'
		port: 7000
		processorDescription: AcProcessorDescriptions armv5
]

{ #category : #tests }
RemoteGDB class >> testSPARCv8 [
	^(RemoteGDBSession
		host: '192.168.75.2'
		port: 7000
		processorDescription: AcProcessorDescriptions sparcv8) testSPARCv8
]

{ #category : #tests }
RemoteGDB class >> testSimics [
	^(RemoteGDBSession
		host: '192.168.2.188'
		port: 9123
		processorDescription: AcProcessorDescriptions powerpc) testSimics1 
		
]

{ #category : #tests }
RemoteGDB class >> testWIN [
	" RemoteGDB testWIN  "
	| gdb  |
	gdb := (RemoteGDBSession
		host: 'vm-boris1'
		port: 7000
		processorDescription: AcProcessorDescriptions sparcv8).
	self halt.

]

{ #category : #'stop codes' }
RemoteGDB >> analyzeContinueAnswer: answer [
	answer isEmpty ifTrue: [ self error: 'Empty Stop-Reply packet' ].
	answer first = $S ifTrue: [ ^self analyzeContinueAnswerS: answer allButFirst ].
	answer first = $T ifTrue: [ ^self analyzeContinueAnswerT: answer allButFirst ].
	answer first = $W ifTrue: [ ^self inferiorExited: answer ].
	"Something unknown / as-yet-unimplemented"
	self error: answer
]

{ #category : #'stop codes' }
RemoteGDB >> analyzeContinueAnswerS: answer [
	"The program received signal number AA (a two-digit hexadecimal number).
	This is equivalent to a T response with no n:r pairs."
	^DebugStopped onSignalNum: (self signalNumberFrom: answer)

]

{ #category : #'stop codes' }
RemoteGDB >> analyzeContinueAnswerT: answer [
	| signal textPairs importantRegs thread core stopReason stopArgument |
	signal := self signalNumberFrom: answer.
	textPairs := answer copyFrom: 3 to: answer size.
	textPairs := textPairs findTokens: ';'.
	importantRegs := Dictionary new.
	core := nil.
	thread := nil.
	stopReason := nil.
	stopArgument := nil.
	textPairs do: [ :textPair |
		| pair k v |
		pair := textPair findTokens: ':'.
		k := pair first.
		v := pair size=2 ifTrue: [pair second] ifFalse: [nil].
		(self recognizedStopCodes includes: k)
			ifTrue: [ stopReason := k. stopArgument := v ] ifFalse: [
		k = 'thread' ifTrue: [ thread := v ] ifFalse: [
		k = 'core' ifTrue: [ core := v ] ifFalse: [
		importantRegs at: k put: (Integer readFrom: k base: 16)
		"TODO: Missing the reserved case;
		this will be handled by catching exception in #readFrom: and discarding it"
	]]]].
	"TODO: wrap all these things in the DebugStopped"
	^DebugStopped onSignalNum: signal
]

{ #category : #'RSP commands' }
RemoteGDB >> c [
	" Continue. "
	| answer |
	self resetRegisterCache.
	answer := self q: 'c'.  "NB: on some platforms, we have wanted 'vCont;c'.  I forgot why."
	^self analyzeContinueAnswer: answer
]

{ #category : #private }
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

{ #category : #registers }
RemoteGDB >> getRegisters [
	registerCache isNil ifTrue: [ registerCache := self getRegistersFromRSP ].
	^registerCache
]

{ #category : #'RSP commands' }
RemoteGDB >> getRegistersFromRSP [
	| answer stream |
	answer := self q: 'g'.
	stream := ReadStream on: answer
		from: 1
		to: answer size.

	^self decodeGPacket: stream
]

{ #category : #GT }
RemoteGDB >> gtInspectorItemsIn: composite [
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

{ #category : #'stop codes' }
RemoteGDB >> inferiorExited: fullAnswer [ 
	| exitCode |
	exitCode := Integer readFrom: fullAnswer copyWithoutFirst base: 16.
	InferiorExited signalWithExitCode: exitCode 
]

{ #category : #'general query commands' }
RemoteGDB >> insertHWBreakpointAt: addr [
	| answer |
	(InterestingBreakpointAddresses dontBreakAtAnyOfThese includes: addr) ifTrue: [ ^self ].
	answer := self q: 'Z1,',
		addr printStringHex,
		',',
		self z1kind printStringHex.
	answer = 'OK' ifFalse: [ self error ]
]

{ #category : #'general query commands' }
RemoteGDB >> insertSWBreakpointAt: addr [
	| answer |
	answer := self q: 'Z1,',
		addr printStringHex,
		',4'.
	answer = 'OK' ifFalse: [ self error ]
]

{ #category : #'RSP commands' }
RemoteGDB >> kill [
	[ | answer |
	answer := self q: 'k'.
	answer = 'OK' ifFalse: [ self error: answer ].
	^self ] on: ConnectionClosed do: [ ^self ]

]

{ #category : #private }
RemoteGDB >> nameForInspector [
	^'GDB'
]

{ #category : #'general query commands' }
RemoteGDB >> packetSize [
	packetSize isNil ifTrue: [ 
		| ps |
		ps := supported detect: [ :q | q beginsWith: 'PacketSize=' ].
		ps := ps copyFrom: 12 to: ps size.
		packetSize := SmallInteger readFrom: ps base: 16 ].
	^packetSize 
]

{ #category : #private }
RemoteGDB >> processorDescription [
	^processorDescription
]

{ #category : #private }
RemoteGDB >> processorDescription: aPDL [
	processorDescription isNil ifFalse: [ self error: 'Target architecture already set' ].
	processorDescription := aPDL
]

{ #category : #private }
RemoteGDB >> q: q [
	^self
		send: q;
		receive
]

{ #category : #'general query commands' }
RemoteGDB >> qOffsets [
	^self q: 'qOffsets'
]

{ #category : #'general query commands' }
RemoteGDB >> qStatus [
	^self q: 'qStatus'
]

{ #category : #'general query commands' }
RemoteGDB >> qSupported [
	supported := self q: self qSupportedQuery.
	supported isEmpty ifTrue: [ ^self error: 'qSupported returned empty string' ].
	supported := supported findTokens: ';'
]

{ #category : #'general query commands' }
RemoteGDB >> qSupportedQuery [
	^'qSupported:swbreak+;hwbreak+;xmlRegisters=i386'
]

{ #category : #'general query commands' }
RemoteGDB >> qTStatus [
	tStatus := self q: 'qTStatus'
]

{ #category : #'RSP commands' }
RemoteGDB >> r: start end: end [ 
	self shouldBeImplemented 
]

{ #category : #'stop codes' }
RemoteGDB >> recognizedStopCodes [
	^#(
	'watch' 'rwatch' 'awatch'
	'syscall_entry' 'syscall_return'
	'library' 'replaying'
	'swbreak'	'hwbreak'
	'fork' 'vfork' 	'vforkdone'
	'exec' 'create'
	)
]

{ #category : #'general query commands' }
RemoteGDB >> removeHWBreakpointAt: addr [
	| answer |
	answer := self q: 'z1,',
		addr printStringHex,
		',',
		self z1kind printStringHex.
	answer = 'OK' ifFalse: [ self error ]
]

{ #category : #registers }
RemoteGDB >> resetRegisterCache [
	registerCache isNil ifFalse: [self sendRegistersToRSP].
	registerCache := nil
]

{ #category : #'RSP commands' }
RemoteGDB >> s [
	"Single step.
	Return control with signal 5 (TRAP),
	or if the execution of the current instruction failed, with whatever signal happened."
	| answer |
	self resetRegisterCache.
	answer := self q: 's'.
	^self analyzeContinueAnswer: answer

]

{ #category : #'RSP commands' }
RemoteGDB >> sendRegistersToRSP [
	| answer stream registerTransfers |
	stream := WriteStream on: String new.
	registerTransfers := processorDescription regsInGPacket.
	registerTransfers do: [ :rt |
		rt write: registerCache to: stream endian: processorDescription endian ].
	
	answer := self q: 'G', stream contents.	
	answer = 'OK' ifFalse: [self error: answer]
]

{ #category : #'RSP commands' }
RemoteGDB >> setThread: c t: t [
	" Hxyyyy family of commands (e.g. Hc-1 or Hg0). "
	| answer |
	answer := self q: 'H', c, t printStringHex.
	answer = 'OK' "ifFalse: [ self error: answer ]" "I've seen this happen in normal operation of the GNU GDB and silently ignored"

]

{ #category : #'stop codes' }
RemoteGDB >> signalNumberFrom: answer [
	^Integer readFrom: (answer copyFrom: 1 to: 2) base: 16

]

{ #category : #tests }
RemoteGDB >> test [
	| x |
self halt.
	x := self getRegisters.
	self s.
	x := self read32At: 16r8150.
	x := self readAt: 16r814c nBytes: 4.
	x := self getRegisters.
	
	
	
]

{ #category : #tests }
RemoteGDB >> testArm1 [
	| x |
self halt.
	x := self getRegisters.
	self s.
	x := self read32At: 16r8150.
	x := self readAt: 16r814c nBytes: 4.
	x := self getRegisters.
	
	
	
]

{ #category : #tests }
RemoteGDB >> testGem1 [
	| regs pc newPC instr |
self halt.
	regs := self getRegisters.

	pc := regs at: 'pc'.
	instr := (processorDescription instructionAt: #ori) bind: (Dictionary new 
		at: 'rs' put: 3;
		at: 'rt' put: 3;
		at: 'imm' put: 25;
		yourself).
	instr := Array with: instr emit.
	self writeInt32s: instr toAddr: pc.
	self writeInt32s: instr toAddr: pc+4.
	self writeInt32s: instr toAddr: pc+8.
	instr := processorDescription decode: (self read32At: pc).
	self halt.
	self s.
	regs := self getRegisters.
	self s.
	regs := self getRegisters.
	newPC := regs at: 'pc'.
	newPC == (pc+4) ifFalse: [ self error ].
	instr := processorDescription decode: (self read32At: newPC).
	instr name = 'ori' ifFalse: [ self error ]
		
	
	
]

{ #category : #tests }
RemoteGDB >> testGemAllRegs [
	| regs    |
self halt.
	self prepareSession.
	regs := self getRegisters.
	self setThread: 'c' t: 0. "future step operations on any thread"

	0 to: 31 do: [ :r |
		| rName |
		rName := 'r', r printString.
		regs at: rName put: 100+r
	].
self halt.
	self setRegisters: regs.
	
	regs := self getRegisters.
	self s.
	regs := self getRegisters.
	self halt.
	
]

{ #category : #tests }
RemoteGDB >> testSPARCv8 [
	| regs pc newPC instr |
self halt.
	regs := self getRegisters.

	pc := regs at: 'pc'.
	instr := (processorDescription instructionAt: #ori) bind: (Dictionary new 
		at: 'rs' put: 3;
		at: 'rt' put: 3;
		at: 'imm' put: 25;
		yourself).
	instr := Array with: instr emit.
	self writeInt32s: instr toAddr: pc.
	self writeInt32s: instr toAddr: pc+4.
	self writeInt32s: instr toAddr: pc+8.
	instr := processorDescription decode: (self read32At: pc).
	self halt.
	self s.
	regs := self getRegisters.
	self s.
	regs := self getRegisters.
	newPC := regs at: 'pc'.
	newPC == (pc+4) ifFalse: [ self error ].
	instr := processorDescription decode: (self read32At: newPC).
	instr name = 'ori' ifFalse: [ self error ]
		
	
	
]

{ #category : #tests }
RemoteGDB >> testSimics1 [
	| regs pc newPC instr |
	regs := self getRegisters.
	pc := regs at: 'pc'.
	self s.
	regs := self getRegisters.
	newPC := regs at: 'pc'.
	newPC == (pc+4) ifFalse: [ self error ].
	instr := processorDescription decode: (self read32At: newPC).
	instr name = 'addis' ifFalse: [ self error ]
		
	
	
]

{ #category : #tests }
RemoteGDB >> testSimicsMemWrite [
	" (RemoteGDB
		host: '192.168.1.128'
		port: 9123
		processorDescription: AcProcessorDescriptions powerpc)
		testSimicsMemWrite
	"
	| regs pc instr answer |
	self halt.
	regs := self getRegisters.
	pc := regs at: 'pc'.
	
	instr := AcProcessorDescriptions powerpc  
		emit: 'transfer(gpr(2), 500000)'.


self writeInt32s: instr toAddr: pc.


	answer := self read32At: pc.
	self s.
	regs := self getRegisters.
	self s.
	self s.
	self s.
	
	
]

{ #category : #'RSP commands' }
RemoteGDB >> vCont [
	"Stepping mechanisms supported by the server"
	vCont := self q: 'vCont?'
]

{ #category : #'general query commands' }
RemoteGDB >> vMustReplyEmpty [
	| answer |
	answer := self q: 'vMustReplyEmpty'.
	answer isEmpty ifFalse: [ ^self error ]
]

{ #category : #'RSP commands' }
RemoteGDB >> why [
	why := self q: '?'
]

{ #category : #'RSP commands' }
RemoteGDB >> writeBytes: aByteArray toAddr: addr [
	| textualAddr currAddr answer |
	currAddr := addr.
	aByteArray do: [ :byte |
		| data |
		data := byte printStringBase: 16 length: 2 padded: true.
		textualAddr := currAddr printStringBase: 16 length: 8 padded: true.
		answer := self
			q: 'M', textualAddr,  ',1:', data.
		answer = 'OK' ifFalse: [ self error: answer ].
		currAddr := currAddr + 1
	]
]

{ #category : #'general query commands' }
RemoteGDB >> z1kind [
	^4 "bogus; for now"
]
