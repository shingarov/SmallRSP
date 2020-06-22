Class {
	#name : #TargetAwareX86,
	#superclass : #TargetAware,
	#classVars : [
		'ExtendedOpcodeExceptionMap',
		'OpcodeExceptionMap',
		'PostBuildStackDelta'
	],
	#category : #'GDB-Cog'
}

{ #category : #'class initialization' }
TargetAwareX86 class >> initialize [
	"TargetAwareX86 initialize"
	| it |
	it := self basicNew.
	PostBuildStackDelta := 0.
	OpcodeExceptionMap := Array new: 256 withAll: #handleExecutionPrimitiveFailureAt:in:.
	OpcodeExceptionMap
		at: 1 + it callOpcode			put: #handleCallFailureAt:in:;
		at: 1 + it jmpOpcode			put: #handleJmpFailureAt:in:;
		at: 1 + it retOpcode			put: #handleRetFailureAt:in:;
		at: 1 + it movALObOpcode	put: #handleMovALObFailureAt:in:;
		at: 1 + it movAXOvOpcode	put: #handleMovAXOvFailureAt:in:;
		at: 1 + it movObALOpcode	put: #handleMovObALFailureAt:in:;
		at: 1 + it movOvAXOpcode	put: #handleMovOvAXFailureAt:in:;
		at: 1 + it movGvEvOpcode	put: #handleMovGvEvFailureAt:in:;
		at: 1 + it movEvGvOpcode	put: #handleMovEvGvFailureAt:in:;
		at: 1 + it movGbEbOpcode	put: #handleMovGbEbFailureAt:in:;
		at: 1 + it movEbGbOpcode	put: #handleMovEbGbFailureAt:in:.
	ExtendedOpcodeExceptionMap := Array new: 256 withAll: #handleExecutionPrimitiveFailureAt:in:.
	ExtendedOpcodeExceptionMap
		at: 1 + it movGvEbOpcode put: #handleMovGvEbFailureAt:in:
]

{ #category : #'machine description' }
TargetAwareX86 class >> isHardware [
	"Answer true if we are running on an actual hardware target.
	Browse senders to see all places where gem5 differs from silicon."

	^false
]

{ #category : #'machine description' }
TargetAwareX86 class >> nZoneSize [
	^16r100000
]

{ #category : #'machine description' }
TargetAwareX86 class >> wordSize [
	^4
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> accessorIsFramePointerSetter: accessor [
	"Answer if accessor is an accessor for the frame pointer.  Subclasses that don't call
	 it fp must override to check for both names."
	^#fp: == accessor or: [#ebp: == accessor]
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> cResultRegister [
	^self eax
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> cResultRegister: aValue [
	^self eax: aValue
]

{ #category : #opcodes }
TargetAwareX86 >> callOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2, pA8"
	^16rE8
]

{ #category : #'intel registers' }
TargetAwareX86 >> cl [
	^self ecx bitAnd: 16rFF
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> controlRegisterGetters [
	^#(eip eflags)
]

{ #category : #disassembly }
TargetAwareX86 >> decorateDisassembly: anInstructionString for: aSymbolManager "<Cogit>" fromAddress: address [
	^BochsIA32Alien new
		decorateDisassembly: anInstructionString for: aSymbolManager "<Cogit>" fromAddress: address
]

{ #category : #disassembly }
TargetAwareX86 >> disassembleCurrentInstructionIn: memory [
	^(self primitiveDisassembleAt: self pc inMemory: memory) last
]

{ #category : #disassembly }
TargetAwareX86 >> disassembleFrom: startAddress to: endAddress in: memory for: aSymbolManager "<Cogit>" labels: labelDictionary on: aStream [
	| prefix theseBytes |
	prefix := ByteArray new: startAddress.
	theseBytes := (startAddress to: endAddress) collect: [ :addr | memory byteAtAddr: addr ].
	BochsIA32Alien new
		disassembleFrom: startAddress
		to: endAddress
		in: prefix, theseBytes
		for: aSymbolManager "<Cogit>"
		labels: labelDictionary on: aStream
	
]

{ #category : #disassembly }
TargetAwareX86 >> disassembleInstructionAt: pc In: memory [
	^(self primitiveDisassembleAt: pc inMemory: memory) last
]

{ #category : #'intel registers' }
TargetAwareX86 >> eax [
	^gdb getRegister: 'eax'
]

{ #category : #'intel registers' }
TargetAwareX86 >> eax: anUnsignedInteger [
	gdb setRegister: 'eax' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #'intel registers' }
TargetAwareX86 >> ebp [
	^gdb getRegister: 'ebp'
]

{ #category : #'intel registers' }
TargetAwareX86 >> ebp: anUnsignedInteger [
	gdb setRegister: 'ebp' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #'intel registers' }
TargetAwareX86 >> ebx [
	^gdb getRegister: 'ebx'
]

{ #category : #'intel registers' }
TargetAwareX86 >> ebx: anUnsignedInteger [
	gdb setRegister: 'ebx' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #'intel registers' }
TargetAwareX86 >> ecx [
	^gdb getRegister: 'ecx'
]

{ #category : #'intel registers' }
TargetAwareX86 >> ecx: anUnsignedInteger [
	gdb setRegister: 'ecx' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #'intel registers' }
TargetAwareX86 >> edi [
	^gdb getRegister: 'edi'
]

{ #category : #'intel registers' }
TargetAwareX86 >> edi: anUnsignedInteger [
	gdb setRegister: 'edi' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #'intel registers' }
TargetAwareX86 >> edx [
	^gdb getRegister: 'edx'
]

{ #category : #'intel registers' }
TargetAwareX86 >> edx: anUnsignedInteger [
	gdb setRegister: 'edx' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #'intel registers' }
TargetAwareX86 >> eflags [
	^gdb getRegister: 'eflags'
]

{ #category : #'intel registers' }
TargetAwareX86 >> eip [
	^gdb getRegister: 'eip'
]

{ #category : #'intel registers' }
TargetAwareX86 >> eip: anUnsignedInteger [
	gdb setRegister: 'eip' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #'intel registers' }
TargetAwareX86 >> esi [
	^gdb getRegister: 'esi'
]

{ #category : #'intel registers' }
TargetAwareX86 >> esi: anUnsignedInteger [
	gdb setRegister: 'esi' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #'intel registers' }
TargetAwareX86 >> esp [
	^gdb getRegister: 'esp'
]

{ #category : #'intel registers' }
TargetAwareX86 >> esp: anUnsignedInteger [
	gdb setRegister: 'esp' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> floatingPointRegisterStateGetters [
	^#()
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> fp [
	^self ebp
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> fp: newFP [
	^self ebp: newFP
]

{ #category : #'error handling' }
TargetAwareX86 >> handleCallFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a call into a ProcessorSimulationTrap signal."
	|  relativeJump |
	relativeJump := memoryArray longAt: pc + 2 bigEndian: false.
	"NB: CRAP! CRAP! CRAP!  The real CPU already pushed the return address!"
	self esp: self esp + 4.
	
	^(ProcessorSimulationTrap
			pc: pc
			nextpc: pc + 5
			address: (pc + 5 + relativeJump) signedIntToLong
			type: #call)
		signal
]

{ #category : #'error handling' }
TargetAwareX86 >> handleDataFailureIn: memoryArray [
	| pc opcode |
	pc := self eip.
	opcode := memoryArray byteAt: pc + 1.
	opcode ~= 16r0f ifTrue:
			[^self
				perform: (OpcodeExceptionMap at: opcode + 1)
				with: pc
				with: memoryArray].
	opcode := memoryArray byteAt: pc + 2.
	^self
				perform: (ExtendedOpcodeExceptionMap at: opcode + 1)
				with: pc
				with: memoryArray
]

{ #category : #'error handling' }
TargetAwareX86 >> handleJmpFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a jmp into a ProcessorSimulationTrap signal."
	|  relativeJump |
	relativeJump := memoryArray longAt: pc + 2 bigEndian: false.
	^(ProcessorSimulationTrap
			pc: pc
			nextpc: pc + 5
			address: (pc + 5 + relativeJump) signedIntToLong
			type: #jump)
		signal
]

{ #category : #'error handling' }
TargetAwareX86 >> handleMovALObFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a read into al into a ProcessorSimulationTrap signal."
	^(ProcessorSimulationTrap
			pc: pc
			nextpc: pc + 5
			address: (memoryArray unsignedLongAt: pc + 2 bigEndian: false)
			type: #read
			accessor: #al:)
		signal
]

{ #category : #'error handling' }
TargetAwareX86 >> handleMovAXOvFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a read into eax into a ProcessorSimulationTrap signal."
	^(ProcessorSimulationTrap
			pc: pc
			nextpc: pc + 5
			address: (memoryArray unsignedLongAt: pc + 2 bigEndian: false)
			type: #read
			accessor: #eax:)
		signal
]

{ #category : #'error handling' }
TargetAwareX86 >> handleMovEbGbFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a byte register write into a ProcessorSimulationTrap signal."
	| modrmByte address |
	modrmByte := memoryArray byteAt: pc + 2.
	(modrmByte bitAnd: 7) ~= 4 ifTrue: "MoveRMbr with r = ESP requires an SIB byte"
		[address := (modrmByte bitAnd: 16rC0)
					caseOf: {
						[0 "ModRegInd"]
						->	[memoryArray unsignedLongAt: pc + 3 bigEndian: false].
						[16r80 "ModRegRegDisp32"]
						->	[(self perform: (#(eax ecx edx ebx esp ebp esi edi) at: (modrmByte bitAnd: 7) + 1))
								+ (memoryArray unsignedLongAt: pc + 3 bigEndian: false)
								bitAnd: 16rFFFFFFFF] }
					otherwise: [^self reportPrimitiveFailure].
		^(ProcessorSimulationTrap
				pc: pc
				nextpc: pc + 6
				address: address
				type: #write
				accessor: (#(al cl dl bl ah ch dh bh) at: ((modrmByte >> 3 bitAnd: 7) + 1)))
			signal].
	^self reportPrimitiveFailure
]

{ #category : #'error handling' }
TargetAwareX86 >> handleMovEvGvFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a register write into a ProcessorSimulationTrap signal."
	| modrmByte |
	^((modrmByte := memoryArray byteAt: pc + 2) bitAnd: 16rC7) = 16r5 "ModRegInd & disp32"
		ifTrue:
			[(ProcessorSimulationTrap
					pc: pc
					nextpc: pc + 6
					address: (memoryArray unsignedLongAt: pc + 3 bigEndian: false)
					type: #write
					accessor: (#(eax ecx edx ebx esp ebp esi edi) at: ((modrmByte >> 3 bitAnd: 7) + 1)))
				signal]
		ifFalse:
			[self reportPrimitiveFailure]
]

{ #category : #'error handling' }
TargetAwareX86 >> handleMovGbEbFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a byte register load into a ProcessorSimulationTrap signal."
	| modrmByte address |
	modrmByte := memoryArray byteAt: pc + 2.
	address := (modrmByte bitAnd: 16rC0)
					caseOf: {
						[0 "ModRegInd"]
						->	[memoryArray unsignedLongAt: pc + 3 bigEndian: false].
						[16r80 "ModRegRegDisp32"]
						->	[(self perform: (#(eax ecx edx ebx esp ebp esi edi) at: (modrmByte bitAnd: 7) + 1))
								+ (memoryArray unsignedLongAt: pc + 3 bigEndian: false)
								bitAnd: 16rFFFFFFFF] }
					otherwise: [^self reportPrimitiveFailure].
	^(ProcessorSimulationTrap
			pc: pc
			nextpc: pc + 6
			address: address
			type: #read
			accessor: (#(al: cl: dl: bl: ah: ch: dh: bh:) at: ((modrmByte >> 3 bitAnd: 7) + 1)))
		signal
]

{ #category : #'error handling' }
TargetAwareX86 >> handleMovGvEbFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a register load into a ProcessorSimulationTrap signal."
	| modrmByte mode srcIsSP srcVal dst offset |
	modrmByte := memoryArray byteAt: pc + 3.
	mode := modrmByte >> 6 bitAnd: 3.
	dst := #(eax: ecx: edx: ebx: esp: ebp: esi: edi:) at: ((modrmByte >> 3 bitAnd: 7) + 1).
	mode = 0 ifTrue: "ModRegInd"
		[offset := memoryArray unsignedLongAt: pc + 4. "1-relative"
		 ^(ProcessorSimulationTrap
					pc: pc
					nextpc: pc + 7
					address: offset
					type: #read
					accessor: dst)
				signal].
	srcIsSP := (modrmByte bitAnd: 7) = 4.
	srcVal := self perform: (#(eax ecx edx ebx esp ebp esi edi) at: (modrmByte bitAnd: 7) + 1).
	mode = 1 ifTrue: "ModRegRegDisp8"
		[offset := memoryArray byteAt: pc + (srcIsSP ifTrue: [5] ifFalse: [4]). "1-relative"
		 offset > 127 ifTrue: [offset := offset - 256].
		 ^(ProcessorSimulationTrap
					pc: pc
					nextpc: pc + (srcIsSP ifTrue: [5] ifFalse: [4])
					address: (srcVal + offset bitAnd: 16rFFFFFFFF)
					type: #read
					accessor: dst)
				signal].
	mode = 2 ifTrue: "ModRegRegDisp32"
		[offset := memoryArray unsignedLongAt: pc + (srcIsSP ifTrue: [5] ifFalse: [4]). "1-relative"
		 ^(ProcessorSimulationTrap
					pc: pc
					nextpc: pc + (srcIsSP ifTrue: [8] ifFalse: [7])
					address: (srcVal + offset bitAnd: 16rFFFFFFFF)
					type: #read
					accessor: dst)
				signal].
	^self reportPrimitiveFailure
]

{ #category : #'error handling' }
TargetAwareX86 >> handleMovGvEvFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a register load into a ProcessorSimulationTrap signal."
	| modrmByte |
	^(((modrmByte := memoryArray byteAt: pc + 2) bitAnd: 16rC7) = 16r5) "ModRegInd & disp32"
		ifTrue:
			[(ProcessorSimulationTrap
					pc: pc
					nextpc: pc + 6
					address: (memoryArray unsignedLongAt: pc + 3 bigEndian: false)
					type: #read
					accessor: (#(eax: ecx: edx: ebx: esp: ebp: esi: edi:) at: ((modrmByte >> 3 bitAnd: 7) + 1)))
				signal]
		ifFalse:
			[self reportPrimitiveFailure]
]

{ #category : #'error handling' }
TargetAwareX86 >> handleMovObALFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a byte write of al into a ProcessorSimulationTrap signal."
	^(ProcessorSimulationTrap
			pc: pc
			nextpc: pc + 5
			address: (memoryArray unsignedLongAt: pc + 2 bigEndian: false)
			type: #write
			accessor: #al)
		signal
]

{ #category : #'error handling' }
TargetAwareX86 >> handleMovOvAXFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a write of eax into a ProcessorSimulationTrap signal."
	^(ProcessorSimulationTrap
			pc: pc
			nextpc: pc + 5
			address: (memoryArray unsignedLongAt: pc + 2 bigEndian: false)
			type: #write
			accessor: #eax)
		signal
]

{ #category : #'error handling' }
TargetAwareX86 >> handleRetFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Convert an execution primitive failure for a ret into a ProcessorSimulationTrap signal."
	^(ProcessorSimulationTrap
			pc: pc
			nextpc: pc + 1
			address: (memoryArray unsignedLongAt: self esp + 1)
			type: #return
			accessor: #eip:)
		signal
]

{ #category : #'processor setup' }
TargetAwareX86 >> initializeStackFor: aCogit [
"Different cpus need different stack alignment etc, so handle the details here."
	"This is for testing.  On many OS's the stack must remain aligned;
	 e.g. IA32 using SSE requires 16 byte alignment."
	| stackAlignment |
	stackAlignment := 16.
	aCogit setStackAlignment: stackAlignment expectedSPOffset: 0 expectedFPOffset: 8.
	PostBuildStackDelta := stackAlignment > 8
								ifTrue: [stackAlignment - 8]
								ifFalse: [0]
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> integerRegisterState [
	| registerState |
	registerState := gdb getRegisters.
	^{ 'eax'.  'ebx'.  'ecx'.  'edx'.  'esp'.  'ebp'.  'esi'.  'edi'. 'eip'.  'eflags'} collect: [ :aRegName |
		registerState at: aRegName ]

]

{ #category : #opcodes }
TargetAwareX86 >> jmpOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2, pA8"
	^16rE9
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> leafRetpcIn: aMemory [
	^aMemory unsignedLongAt: self esp + 1 bigEndian: false
]

{ #category : #opcodes }
TargetAwareX86 >> lockPrefix [
	^16rF0
]

{ #category : #opcodes }
TargetAwareX86 >> movALObOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2, pA7"
	^16rA0
]

{ #category : #opcodes }
TargetAwareX86 >> movAXOvOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2, pA7"
	^16rA1
]

{ #category : #opcodes }
TargetAwareX86 >> movEbGbOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2"
	^16r88
]

{ #category : #opcodes }
TargetAwareX86 >> movEvGvOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2, pA8"
	^16r89
]

{ #category : #opcodes }
TargetAwareX86 >> movGbEbOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2"
	^16r8A
]

{ #category : #opcodes }
TargetAwareX86 >> movGvEbOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A3, pA14"
	^16rB6
]

{ #category : #opcodes }
TargetAwareX86 >> movGvEvOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2, pA8"
	^16r8B
]

{ #category : #opcodes }
TargetAwareX86 >> movObALOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2, pA7"
	^16rA2
]

{ #category : #opcodes }
TargetAwareX86 >> movOvAXOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2, pA7"
	^16rA3
]

{ #category : #opcodes }
TargetAwareX86 >> nopOpcode [
	^16r90
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> pc [
	^self eip
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> pc: newPC [
	^self eip: newPC
]

{ #category : #'target connection' }
TargetAwareX86 >> pdl [
	^FakeProcessorDescriptionX86 new
]

{ #category : #execution }
TargetAwareX86 >> popWordIn: aMemory [ 
	| sp word |
	word := aMemory unsignedLongAt: (sp := self esp) + 1 bigEndian: false.
	self esp: sp + 4.
	^word
]

{ #category : #execution }
TargetAwareX86 >> postCallArgumentsNumArgs: numArgs "<Integer>" in: memory [ "<ByteArray|Bitmap>"
	"Answer an argument vector of the requested size after a vanilla
	 ABI call.  On IA32 this typically means accessing stacked arguments
	 beyond the pushed return address and saved frame pointer.
	 For compatibility with Cog/Slang we answer unsigned values."
	^(9 to: numArgs * 4 + 5 by: 4) collect:
		[:i|
		memory unsignedLongAt: self ebp + i bigEndian: false]
]

{ #category : #disassembly }
TargetAwareX86 >> primitiveDisassembleAt: address inMemory: memory [
	| prefix theseBytes |
	prefix := ByteArray new: address.
	theseBytes := (address to: address + 16) collect: [ :addr | memory byteAtAddr: addr ].
	^BochsIA32Alien new
		primitiveDisassembleAt: address inMemory: prefix, theseBytes
]

{ #category : #printing }
TargetAwareX86 >> printNameOn: aStream [ 
	super printOn: aStream
]

{ #category : #printing }
TargetAwareX86 >> printOn: aStream [
	self gdb printRegistersOn: aStream

]

{ #category : #cog }
TargetAwareX86 >> pushWord: aValue in: aMemory [
	| sp |
	sp := (self esp: self esp - 4).
	aMemory longAt: sp + 1 put: aValue bigEndian: false
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> registerStateGetters [
	^#(	eax ecx edx ebx esp ebp esi edi eip eflags )
]

{ #category : #'error handling' }
TargetAwareX86 >> reportPrimitiveFailure [
	self shouldBeImplemented
]

{ #category : #opcodes }
TargetAwareX86 >> retOpcode [
	"[1] IA-32 Intel® Architecture Software Developer's Manual Volume 2B: Instruction Set Reference, N-Z.
		table A2, pA7"
	^16rC3
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> retpcIn: aMemory [
	^aMemory unsignedLongAt: self ebp + 5 bigEndian: false
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> setFramePointer: framePointer stackPointer: stackPointer [
	"Initialize the processor's frame and stack pointers"
	self ebp: framePointer.
	self esp: stackPointer
]

{ #category : #execution }
TargetAwareX86 >> simulateBuildFrameIn: t1 for: t2 [ 
	self break.
	self pushWord: self sp in: t1.
	self fp: self sp.
	PostBuildStackDelta ~= 0
		ifTrue: [self sp: self sp - PostBuildStackDelta]
]

{ #category : #execution }
TargetAwareX86 >> simulateCallIn: aMemory [
	"The return address has already been pushed."
	self pushWord: self ebp in: aMemory.
	self ebp: self esp.
	PostBuildStackDelta ~= 0 ifTrue:
		[self esp: self esp - PostBuildStackDelta].
]

{ #category : #execution }
TargetAwareX86 >> simulateCallOf: anAddress [
	self shouldBeImplemented
]

{ #category : #execution }
TargetAwareX86 >> simulateCallOf: address nextpc: nextpc memory: aMemory [
	"Simulate a frame-building call of address.  Build a frame since
	a) this is used for calls into the run-time which are unlikely to be leaf-calls, and
	b) stack alignment needs to be realistic for assert checking for platforms such as Mac OS X"
	self pushWord: nextpc in: aMemory.
	self pushWord: self ebp in: aMemory.
	self ebp: self esp.
	PostBuildStackDelta ~= 0 ifTrue:
		[self esp: self esp - PostBuildStackDelta].
	self eip: address
]

{ #category : #execution }
TargetAwareX86 >> simulateJumpCallOf: address memory: aMemory [
	"Simulate a frame-building jump call of address (i.e. do not push the return pc
	 as this has already been done).  Build a frame since
	a) this is used for calls into the run-time which are unlikely to be leaf-calls, and
	b) stack alignment needs to be realistic for assert checking for platforms such as Mac OS X"
	self pushWord: self ebp in: aMemory.
	self ebp: self esp.
	PostBuildStackDelta ~= 0 ifTrue:
		[self esp: self esp - PostBuildStackDelta].
	self eip: address
]

{ #category : #'entering execution' }
TargetAwareX86 >> simulateLeafCallOf: someFunction for: aCogit [
	"Natively execute the CPU code for someFunction until it returns.
	Answer the contents of the C ABI return register, an Integer."
	| priorSP priorPC spOnEntry |
	priorSP := self sp.
	priorPC := self pc.
	self
		setFramePointer: aCogit coInterpreter getCFramePointer stackPointer: aCogit coInterpreter getCStackPointer;
		simulateLeafCallOf: someFunction
		nextpc: self bogusRetPC
		memory: aCogit objectMemory memory.
	spOnEntry := self sp.
	aCogit singleStep
			ifTrue: [ self shouldBeImplemented ].
	[true] whileTrue: [ 
		[ self runInMemory: aCogit objectMemory memory minimumAddress: 4096 ]
			on: SimulatedLeafReturn
			do: [ ^self cResultRegister ] ]
]

{ #category : #cog }
TargetAwareX86 >> simulateLeafCallOf: address nextpc: nextpc memory: aMemory [
"this should go back to the alien"
	self pushWord: nextpc in: aMemory.
	self eip: address
]

{ #category : #execution }
TargetAwareX86 >> simulateReturnIn: aMemory [
	PostBuildStackDelta ~= 0 ifTrue:
		[self esp: self esp + PostBuildStackDelta].
	self ebp: (self popWordIn: aMemory).
	self eip: (self popWordIn: aMemory)
]

{ #category : #execution }
TargetAwareX86 >> simulateTearFrameIn: aMemory [
	"Simulate a frame-building call of address.  Build a frame since
	a) this is used for calls into the run-time which are unlikely to be leaf-calls, and
	b) stack alignment needs to be realistic for assert checking for platforms such as Mac OS X"

	PostBuildStackDelta ~= 0 ifTrue:
		[self esp: self esp + PostBuildStackDelta].
	self esp: (self popWordIn: aMemory).
]

{ #category : #execution }
TargetAwareX86 >> singleStepIn: aMemory minimumAddress: minimumAddress readOnlyBelow: minimumWritableAddress [
	| stopReason |
	stopReason := gdb s.
	stopReason signal = #SIGTRAP ifTrue: [ ^self "no fault" ].
	^self
		handleExecutionPrimitiveFailureIn: aMemory
		minimumAddress: minimumAddress
]

{ #category : #execution }
TargetAwareX86 >> smashCallerSavedRegistersWithValuesFrom: base by: step in: aMemory [
	#(eax: ecx: edx:)
	   withIndexDo:
		[:accessor :index|
		self perform: accessor with: index - 1 * step + base]
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> smashRegisterAccessors [
	^#(eax: ebx: ecx: edx: esi: edi:)
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> sp [
	^self esp
]

{ #category : #'accessing-abstract' }
TargetAwareX86 >> sp: anAddress [
	"Set whatever the processor considers its stack pointer to anAddress."
	self esp: anAddress
]

{ #category : #printing }
TargetAwareX86 >> topOfStackIn: aMemory [
	"The 32-bit word at the stack top"
	^aMemory unsignedLongAtAddr: self esp bigEndian: false
]
