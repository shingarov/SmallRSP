"
I am the next level up from RSPStream.
I know about RSP packets and their most basic meaning, so RemoteGDB can be implemented in terms of them.
"
Class {
	#name : #RSPClient,
	#superclass : #Object,
	#instVars : [
		'transport',
		'packetSize',
		'tStatus',
		'why',
		'supported',
		'vCont'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
RSPClient class >> host: h port: p [
	"Connect to the remote server and perform initial handshake."
	^self basicNew
		transport: (RSPStream host: h port: p);
		prepareSession;
		yourself
]

{ #category : #registers }
RSPClient >> G: hex [
	| answer |
	answer := self q: 'G', hex.	
	answer = 'OK' ifFalse: [self error: answer]
]

{ #category : #execution }
RSPClient >> analyzeContinueAnswer: answer [
	answer isEmpty ifTrue: [ self error: 'Empty Stop-Reply packet' ].
	answer first = $S ifTrue: [ ^self analyzeContinueAnswerS: answer allButFirst ].
	answer first = $T ifTrue: [ ^self analyzeContinueAnswerT: answer allButFirst ].
	answer first = $W ifTrue: [ ^self inferiorExited: answer ].
	"Something unknown / as-yet-unimplemented"
	self error: answer
]

{ #category : #execution }
RSPClient >> analyzeContinueAnswerS: answer [
	"The program received signal number AA (a two-digit hexadecimal number).
	This is equivalent to a T response with no n:r pairs."
	^DebugStopped onSignalNum: (self signalNumberFrom: answer)

]

{ #category : #execution }
RSPClient >> analyzeContinueAnswerT: answer [
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

{ #category : #commands }
RSPClient >> askFeatures [
	" Ask the stub for the target feature descriptor, and return it.
	  If the stub doesn't provide it, return nil. "
	self supportsFeatures
		ifTrue: [ ^self qXfer features: 'target.xml' ]
		ifFalse: [ ^nil ]
		
]

{ #category : #execution }
RSPClient >> cont [
	" Continue.  Note: the c packet is deprecated for multi-threading support."
	| answer |
	answer := self q: 'c'.  "We need to implement vCont;c as some platforms don't have c."
	^self analyzeContinueAnswer: answer
]

{ #category : #commands }
RSPClient >> currentThreadID [
	| answer |
	answer := self q: 'qC'.
	(answer beginsWith: 'QC') ifFalse: [ self shouldBeImplemented ].
	^answer allButFirst allButFirst
]

{ #category : #commands }
RSPClient >> disconnect [
	transport disconnect.
	transport := nil
]

{ #category : #'stop codes' }
RSPClient >> inferiorExited: fullAnswer [ 
	| exitCode |
	exitCode := Integer readFrom: fullAnswer copyWithoutFirst base: 16.
	InferiorExited signalWithExitCode: exitCode 
]

{ #category : #breakpoints }
RSPClient >> insertBreakpoint: type at: addr kind: kind [
	| answer |
	answer := self q: 'Z',
		type printString,     ',',
		addr printStringHex,  ',',
		kind printStringHex.
	answer = 'OK' ifFalse: [ self error ]
]

{ #category : #execution }
RSPClient >> kill [
	[ | answer |
	answer := self q: 'k'.
	answer = 'OK' ifFalse: [ self error: answer ].
	^self ] on: InferiorExited do: [ ^self ]

]

{ #category : #'general query commands' }
RSPClient >> packetSize [
	packetSize isNil ifTrue: [ 
		| ps |
		ps := supported detect: [ :q | q beginsWith: 'PacketSize=' ].
		ps := ps copyFrom: 12 to: ps size.
		packetSize := SmallInteger readFrom: ps base: 16 ].
	^packetSize 
]

{ #category : #private }
RSPClient >> prepareSession [
	self qSupported.
	self vMustReplyEmpty.
	self packetSize.
	self setThread: 'g' t: 0.
	self qStatus = '' ifFalse: [ self error: 'Bad status' ].
	self vCont.
]

{ #category : #private }
RSPClient >> q: q [
	^transport
		send: q;
		receive
]

{ #category : #commands }
RSPClient >> qRcmd: command [
	| hexEncoded answer |
	hexEncoded := WriteStream on: String new.
	command do: [ :char | char codePoint printOn: hexEncoded base: 16 length: 2 padded: true ].
	answer := self q: 'qRcmd,', hexEncoded contents.
	(answer beginsWith: 'E') ifTrue: [ self error: answer ].
	^transport receiveHexLinesUntilOkStartingWith: answer
]

{ #category : #'general query commands' }
RSPClient >> qStatus [
	^self q: 'qStatus'
]

{ #category : #'general query commands' }
RSPClient >> qSupported [
	supported := self q: self qSupportedQuery.
	supported isEmpty ifTrue: [ ^self error: 'qSupported returned empty string' ].
	supported := supported findTokens: ';'
]

{ #category : #'general query commands' }
RSPClient >> qSupportedQuery [
	^'qSupported:swbreak+;hwbreak+;xmlRegisters=i386'
]

{ #category : #'RSP Helpers' }
RSPClient >> qXfer [
	^RemoteGdbXFER rsp: self
]

{ #category : #'stop codes' }
RSPClient >> recognizedStopCodes [
	^#(
	'watch' 'rwatch' 'awatch'
	'syscall_entry' 'syscall_return'
	'library' 'replaying'
	'swbreak'	'hwbreak'
	'fork' 'vfork' 	'vforkdone'
	'exec' 'create'
	)
]

{ #category : #breakpoints }
RSPClient >> removeBreakpoint: type at: addr kind: kind [
	| answer |
	answer := self q: 'z',
		type printString,     ',',
		addr printStringHex,  ',',
		kind printStringHex.
	answer = 'OK' ifFalse: [ self error ]
]

{ #category : #registers }
RSPClient >> sendRegisters: regDictionary transfers: transfers endian: endian [ 
	| stream |
	stream := WriteStream on: String new.
	transfers do: [ :rt |
		rt write: regDictionary to: stream endian: endian ].
	self G: stream contents
]

{ #category : #commands }
RSPClient >> setThread: c t: t [
	" Hxyyyy family of commands (e.g. Hc-1 or Hg0). "
	| answer |
	answer := self q: 'H', c, t printStringHex.
	answer = 'OK' "ifFalse: [ self error: answer ]" "I've seen this happen in normal operation of the GNU GDB and silently ignored"

]

{ #category : #'stop codes' }
RSPClient >> signalNumberFrom: answer [
	^Integer readFrom: (answer copyFrom: 1 to: 2) base: 16

]

{ #category : #execution }
RSPClient >> stepi [
	| answer |
	"(self vContSupports: 's')" false
		ifTrue: [ answer := self q: 'vCont;s:p{ self currentThreadID }.-1' ]
		ifFalse: [ answer := self q: 's' ].
	^self analyzeContinueAnswer: answer

]

{ #category : #commands }
RSPClient >> supportsFeatures [
	" Answer whether the stub supports target feature descriptors. "
	| features |
	features := supported detect: [ :s | s beginsWith: 'qXfer:features:' ] ifNone: [ ^false ].
	features = 'qXfer:features:read+' ifFalse: [ self error: 'Havent encountered such a CPU yet; investigate' ].
	^true
]

{ #category : #private }
RSPClient >> transport: anRSPStream [
	transport := anRSPStream
]

{ #category : #commands }
RSPClient >> vCont [
	"Stepping mechanisms supported by the server."
	| answer |
	answer := self q: 'vCont?'.
	answer isEmpty ifTrue: [
		(supported includes: 'vContSupported+') ifTrue: [ self error: 'Inconsisted support reported by stub?' ].
		^vCont := nil
	].
	(answer beginsWith: 'vCont;') ifFalse: [ self error ].
	vCont := (answer substrings: ';') allButFirst 
]

{ #category : #'RSP Helpers' }
RSPClient >> vContSupports: action [
	vCont isNil ifTrue: [ ^false ].
	^vCont includes: action
]

{ #category : #'general query commands' }
RSPClient >> vMustReplyEmpty [
	| answer |
	answer := self q: 'vMustReplyEmpty'.
	answer isEmpty ifFalse: [ ^self error ]
]
