Class {
	#name : #TargetAware,
	#superclass : #Object,
	#instVars : [
		'gdb'
	],
	#classVars : [
		'Current'
	],
	#category : #'GDB-Cog'
}

{ #category : #'instance creation' }
TargetAware class >> current [
	^Current 
]

{ #category : #'instance creation' }
TargetAware class >> new [
	"
	TargetAwareX86 new
	"
	Current := super new connectGdb.
	^Current 
]

{ #category : #'instance creation' }
TargetAware class >> nukeCurrent [
	Current := nil 
]

{ #category : #'class initialization' }
TargetAware class >> printTempNames [
	^false
]

{ #category : #'class initialization' }
TargetAware class >> printTempNames: x [
]

{ #category : #'machine description' }
TargetAware class >> remoteMemoryClass [
	^self isHardware
		ifTrue: [SimpleSharedRAM]
		ifFalse: [Gem5SharedRAM]
]

{ #category : #'class initialization' }
TargetAware class >> setReceiverResultReg:  x [
]

{ #category : #'machine description' }
TargetAware class >> zeroROChunkIn: aMemory [
	| first last |
	first := Cogit guardPageSize.
	last := first + self nZoneSize - 1.
	first to: last do: [ :addr | aMemory byteAtAddr: addr put: 0 ]

]

{ #category : #'accessing-abstract' }
TargetAware >> accessorIsFramePointerSetter: accessor [
	"Answer if accessor is an accessor for the frame pointer.  Subclasses that don't call
	 it fp must override to check for both names."
	^#fp: == accessor
]

{ #category : #'entering execution' }
TargetAware >> bogusRetPC [
	^16rBADF00D5
]

{ #category : #'target connection' }
TargetAware >> connectGdb [
	gdb := self debuggerClass
		host: self hostIP
		port: self tcpPort.
	self runThinshellPrologue.
	^self "not gdb; #new needs the instance"
]

{ #category : #'tests support' }
TargetAware >> convertIntegerToInternal: anInteger [
	"Default conversion for 32-bit processors.  64-bit processors override."
	^anInteger signedIntToLong
]

{ #category : #'tests support' }
TargetAware >> convertInternalToInteger: unsigned [
	"Default conversion for 32-bit processors.  64-bit processors override."
	^unsigned signedIntFromLong
]

{ #category : #'target connection' }
TargetAware >> debuggerClass [
	^RemoteGDBSession
]

{ #category : #disassembly }
TargetAware >> decorateDisassembly: anInstructionString for: aSymbolManager fromAddress: address [
	| s |
	s := WriteStream on: String new.
	address printOn: s base: 16 length: 8 padded: true.
	s nextPutAll: ': '; nextPutAll: anInstructionString.
	^s contents
]

{ #category : #'target connection' }
TargetAware >> gdb [
	^gdb
]

{ #category : #'error handling' }
TargetAware >> handleExecutionPrimitiveFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Handle an execution primitive failure for an unhandled opcode."
	^self reportPrimitiveFailure
]

{ #category : #execution }
TargetAware >> handleExecutionPrimitiveFailureIn: memory minimumAddress: minimumAddress [
	"Execution stopped due to SIGSEGV.
	Convert out-of-range call and absolute memory read-into-register instructions
	into ProcessorSimulationTrap signals."

	"The SEGV could be caused by Fetching from an unmapped address,
	or a data operation with an unmapped address.
	While a simulator such as gem5 could tell us what caused the fault,
	real hardware such as silicon i386 doesn't provide an easy way;
	therefore, we have to look at circumstancial evidence."
	(self isWithinMappedRange: self pc)
		ifTrue: [ ^self handleDataFailureIn: memory ]
		ifFalse: [ ^self handlePcOutOfRangeIn: memory ]
]

{ #category : #'error handling' }
TargetAware >> handlePcOutOfRangeIn: memoryArray [
	| pc |
	pc := self pc.
	pc = self bogusRetPC ifTrue: [ ^self cResultRegister ].
	^(ProcessorSimulationTrap
			pc: nil
			nextpc: nil
			address: pc
			type: #controlTransfer)
		signal
]

{ #category : #'target connection' }
TargetAware >> hostIP [
	^'192.168.75.2'
]

{ #category : #execution }
TargetAware >> isWithinMappedRange: anAddress [
	| minimumAddress maximumAddress |
	minimumAddress := 4096.
	maximumAddress := 120*1024*1024 - 1.
	^anAddress >= minimumAddress and: [ anAddress <= maximumAddress ]
]

{ #category : #'target connection' }
TargetAware >> pdl [
	^gdb processorDescription 
]

{ #category : #primitives }
TargetAware >> primitiveDisassembleAt: address inMemory: memory [
	| encoding disasm |
	encoding := memory unsignedLongAtAddr: address bigEndian: false.
	disasm := self pdl disassemble: encoding.
	true ifTrue: [ 
		| hex s |
		s := WriteStream on: String new.
		hex := encoding printOn: s base: 16 length: 8 padded: true.
		s space.
		disasm := s contents, disasm ].
	^{ 4. disasm. }
]

{ #category : #execution }
TargetAware >> runInMemory: aMemory minimumAddress: mmm [
	| stopReason |
	stopReason := gdb c.
	stopReason signal = #SIGSEGV ifTrue: [
		^self
			handleExecutionPrimitiveFailureIn: aMemory
			minimumAddress: #zzzz ].
	stopReason signal = #SIGQUIT ifTrue: [ ^self halt ].

	"There can be a number of other reasons to stop.
	For example, a debug breakpoint."
	self shouldBeImplemented
]

{ #category : #execution }
TargetAware >> runInMemory: aMemory minimumAddress: minimumAddress readOnlyBelow: minimumWritableAddress [
	^self runInMemory: aMemory minimumAddress: #zzzz
]

{ #category : #'target connection' }
TargetAware >> runThinshellPrologue [
	"Run the thinshell's _start to do whatever initialization it needs,
	until it stops (usually indicates 'I am done' by segfaulting or trapping)"
	gdb c
]

{ #category : #'accessing-abstract' }
TargetAware >> smashRegistersWithValuesFrom: base by: step [
	self smashRegisterAccessors
	   withIndexDo:
		[:accessor :index|
		self perform: accessor with: index - 1 * step + base]
]

{ #category : #'target connection' }
TargetAware >> tcpPort [
	^7000
]
