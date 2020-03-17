Class {
	#name : #RemoteGDB,
	#superclass : #RemoteGDBTransport,
	#instVars : [
		'packetSize',
		'processorDescription',
		'tStatus',
		'why',
		'supported',
		'vCont'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
RemoteGDB class >> host: h port: p processorDescription: pd [
	"Create an instance, connect to the remote server, and fully prepare the debugging session."
	^(self host: h port: p)
		processorDescription: pd;
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

{ #category : #'RSP commands' }
RemoteGDB >> analyzeContinueAnswer: answer [
	answer isEmpty ifTrue: [ self error: 'Empty answer from GDB server' ].
	answer = 'S05' ifTrue: [ ^self ].
	answer first = $T ifTrue: [ ^self ].
	answer first = $W ifTrue: [ ^self inferiorExited: answer ].
	"Something unknown / as-yet-unimplemented"
	self error: answer
]

{ #category : #'RSP commands' }
RemoteGDB >> c [
	" Continue. "
	| answer |
	answer := self q: 'c'.  "NB: on some platforms, we have wanted 'vCont;c'.  I forgot why."
	^self analyzeContinueAnswer: answer
]

{ #category : #private }
RemoteGDB >> decodeGPacket: aStream [
	| registerTransfers regs |
	registerTransfers := processorDescription regsInGPacket.
	regs := Dictionary new.
	registerTransfers do: [ :rt |
		| nextValue |
		aStream atEnd ifTrue: [ ^regs ].
		nextValue := rt readFrom: aStream.
		regs at: rt regName put: nextValue ].
	^regs


]

{ #category : #'RSP commands' }
RemoteGDB >> getRegisters [
	| answer stream |
	answer := self q: 'g'.
	stream := ReadStream on: answer
		from: 1
		to: answer size.
	^self decodeGPacket: stream
]

{ #category : #private }
RemoteGDB >> gtInspectorPreviewIn: composite [
	<gtInspectorPresentationOrder: 20>
	composite morph
		title: 'Registers';
		morph: [ GdbRegistersMorph on: self ]
]

{ #category : #'as yet unclassified' }
RemoteGDB >> inferiorExited: fullAnswer [ 
	| exitCode |
	exitCode := Integer readFrom: fullAnswer copyWithoutFirst base: 16.
	InferiorExited signalWithExitCode: exitCode 
]

{ #category : #'general query commands' }
RemoteGDB >> insertHWBreakpointAt: addr [
	| answer |
	answer := self q: 'Z1,',
		addr printStringHex,
		',4'.
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
RemoteGDB >> processorDescription: descr [
	processorDescription := descr
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
	supported := self q: 'qSupported:swbreak+'.
	supported isEmpty ifTrue: [ ^self error: 'qSupported returned empty string' ].
	supported := supported findTokens: ';'
]

{ #category : #'general query commands' }
RemoteGDB >> qTStatus [
	tStatus := self q: 'qTStatus'
]

{ #category : #'RSP commands' }
RemoteGDB >> s [
	"Single step.
	Return control with signal 5 (TRAP)."
	| answer |
	answer := self q: 's'.
	answer first = $S ifTrue: [ ^self ].
	answer first = $T ifTrue: [ ^self ].
	self error: answer

]

{ #category : #'RSP commands' }
RemoteGDB >> setRegisters: aDict [
	| answer stream registerTransfers |
	stream := WriteStream on: String new.
	registerTransfers := processorDescription regsInGPacket.
	registerTransfers do: [ :rt |
		rt write: aDict to: stream ].
	
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
	instr := processorDescription disassemble: (self read32At: pc).
	self halt.
	self s.
	regs := self getRegisters.
	self s.
	regs := self getRegisters.
	newPC := regs at: 'pc'.
	newPC == (pc+4) ifFalse: [ self error ].
	instr := processorDescription disassemble: (self read32At: newPC).
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
	instr := processorDescription disassemble: (self read32At: pc).
	self halt.
	self s.
	regs := self getRegisters.
	self s.
	regs := self getRegisters.
	newPC := regs at: 'pc'.
	newPC == (pc+4) ifFalse: [ self error ].
	instr := processorDescription disassemble: (self read32At: newPC).
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
	instr := processorDescription disassemble: (self read32At: newPC).
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
