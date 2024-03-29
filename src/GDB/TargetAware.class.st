Class {
	#name : #TargetAware,
	#superclass : #Object,
	#instVars : [
		'gdb',
		'pcMap',
		'breakpoints'
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

{ #category : #GT }
TargetAware >> addBreakpoint [
	| s pc |
	s := UIManager default request: 'Break pc (hex)'.
	s := s withBlanksTrimmed.
	s isEmpty ifTrue: [^self].
	pc := Integer readFrom: s readStream base: 16.
	self insertBreakpointAt: pc
]

{ #category : #'entering execution' }
TargetAware >> bogusRetPC [
	^16rBADF00D5
]

{ #category : #GT }
TargetAware >> breakpoints [
	breakpoints isNil ifTrue: [ breakpoints := OrderedCollection new ].
	^ breakpoints
]

{ #category : #'target connection' }
TargetAware >> connectGdb [
	gdb := self debuggerClass
		host: self hostIP
		port: self tcpPort.
	self runThinshellPrologue.
	self insertPredefinedBreakpoints.
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
	^RemoteGDB
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

{ #category : #GT }
TargetAware >> gtInspectorAccessWatchpointsIn: composite [
	<gtInspectorPresentationOrder: 33>
	composite fastList
		title: 'Access watchpoints';
		display: [ #(1 2 3) ];
		act: [ self addBreakpoint ] icon: GLMUIThemeExtraIcons glamorousAdd entitled: 'add';
		act: [ self removeBreakpoint ] icon: GLMUIThemeExtraIcons glamorousAdd entitled: 'remove'
]

{ #category : #GT }
TargetAware >> gtInspectorBreakpointsIn: composite [
	<gtInspectorPresentationOrder: 30>
	| fl |
	fl := composite fastList.
	fl
		title: 'Breakpoints';
		display: [ self breakpoints ];
		format: [ :a | '0x', a printStringHex ];
		act: [ self addBreakpoint ] icon: GLMUIThemeExtraIcons glamorousAdd entitled: 'add';
		act: [ self removeBreakpoint: [ fl selection ] ] icon: GLMUIThemeExtraIcons glamorousRemove entitled: 'remove'
]

{ #category : #GT }
TargetAware >> gtInspectorReadWatchpointsIn: composite [
	<gtInspectorPresentationOrder: 31>
	composite fastList
		title: 'Read watchpoints';
		display: [ #(1 2 3) ];
		act: [ self addBreakpoint ] icon: GLMUIThemeExtraIcons glamorousAdd entitled: 'add';
		act: [ self removeBreakpoint ] icon: GLMUIThemeExtraIcons glamorousAdd entitled: 'remove'
]

{ #category : #GT }
TargetAware >> gtInspectorWriteWatchpointsIn: composite [
	<gtInspectorPresentationOrder: 32>
	composite fastList
		title: 'Write watchpoints';
		display: [ #(1 2 3) ];
		act: [ self addBreakpoint ] icon: GLMUIThemeExtraIcons glamorousAdd entitled: 'add';
		act: [ self removeBreakpoint ] icon: GLMUIThemeExtraIcons glamorousAdd entitled: 'remove'
]

{ #category : #'error handling' }
TargetAware >> handleExecutionPrimitiveFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Handle an execution primitive failure for an unhandled opcode."
	^self reportPrimitiveFailure
]

{ #category : #execution }
TargetAware >> handleExecutionPrimitiveFailureIn: memory [
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
TargetAware >> handleJITCodeTrap [
	^JITCodeTrap signalFor: self
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

{ #category : #'error handling' }
TargetAware >> handleTrapIn: aMemory [
	| pc rtl |
	pc := gdb pc.
	"Is this in JITed code that we know of?"
	rtl := pcMap at: pc ifAbsent: [^self handleUnknownCodeTrap].
	rtl isNil ifTrue: [^self error: 'Strange SIGTRAP at ', pc hex]. "can't happen at all"
	^self handleJITCodeTrap

]

{ #category : #'error handling' }
TargetAware >> handleUnknownCodeTrap [
	self error: 'SIGTRAP in unknown code at ', gdb pc hex

]

{ #category : #'target connection' }
TargetAware >> hostIP [
	^'192.168.75.2'
]

{ #category : #'pc map' }
TargetAware >> initEmptyPcMap [
	pcMap := Dictionary new
]

{ #category : #'as yet unclassified' }
TargetAware >> insertBreakpointAt: addr [
	(self breakpoints includes: addr)
		ifFalse: [ gdb insertHWBreakpointAt: addr ].
	self breakpoints add: addr.
]

{ #category : #'target connection' }
TargetAware >> insertPredefinedBreakpoints [
	self predefinedBreakpoints do: [ :bkptAddress | self insertBreakpointAt: bkptAddress ]
]

{ #category : #execution }
TargetAware >> isWithinMappedRange: anAddress [
	| minimumAddress maximumAddress |
	minimumAddress := 4096.
	maximumAddress := 120*1024*1024 - 1.
	^anAddress >= minimumAddress and: [ anAddress <= maximumAddress ]
]

{ #category : #'pc map' }
TargetAware >> noteCogInstruction: aCogInstruction emittedAt: a [
	pcMap at: a put: aCogInstruction generatorChain. "can't put in the RTL itself, these mutate"
	aCogInstruction maybeInsertHWBreakpoint: a
	
]

{ #category : #'pc map' }
TargetAware >> pcMap [
	pcMap isNil ifTrue: [ self initEmptyPcMap ].
	^pcMap
]

{ #category : #'target connection' }
TargetAware >> pdl [
	^gdb processorDescription 
]

{ #category : #'target connection' }
TargetAware >> predefinedBreakpoints [
	^InterestingBreakpointAddresses predefinedBreakpoints
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

{ #category : #GT }
TargetAware >> removeBreakpoint [
	self shouldBeImplemented 
]

{ #category : #'as yet unclassified' }
TargetAware >> removeBreakpoint: bkptEvaluator [
	| addr |
	addr := bkptEvaluator value.
	gdb removeHWBreakpointAt: addr.
	self breakpoints remove: addr
	
]

{ #category : #execution }
TargetAware >> runInMemory: aMemory [
	| stopReason |
	stopReason := gdb c.
	stopReason signal = #SIGSEGV ifTrue: [^self handleExecutionPrimitiveFailureIn: aMemory].
	stopReason signal = #SIGQUIT ifTrue: [ ^InferiorExited signal ].

	"Single-step, or breakpoint"
	stopReason signal = #SIGTRAP ifTrue: [ ^self handleTrapIn: aMemory ].
	
	"There can be a number of other reasons to stop."
	self shouldBeImplemented
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

{ #category : #'as yet unclassified' }
TargetAware >> unknownFakeAddress: aProcessorSimulationTrap [ 
	^self handleJITCodeTrap
]
