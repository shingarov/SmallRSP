"
I bridge between Cog and ULD by implementing the CogProcessorAlien API.
On ARM this means replacing ARMulator with gem5.

This implementation is temporary, and will soon be replaced by a proper target-agnostic one.
"
Class {
	#name : #TargetAwareARM,
	#superclass : #TargetAware,
	#classVars : [
		'PostBuildStackDelta'
	],
	#category : #'GDB-Cog'
}

{ #category : #'instance creation' }
TargetAwareARM class >> implementationClass [
	^TargetAwareARM
]

{ #category : #'machine description' }
TargetAwareARM class >> isHardware [
	"Answer true if we are running on an actual hardware target.
	Browse senders to see all places where gem5 differs from silicon."

	^false
]

{ #category : #'machine description' }
TargetAwareARM class >> nZoneSize [
	^16r140000
]

{ #category : #'machine description' }
TargetAwareARM class >> wordSize [
	^4
]

{ #category : #'Cog API' }
TargetAwareARM >> abstractInstructionCompilerClass [
	^CogARMCompiler
]

{ #category : #'Cog API' }
TargetAwareARM >> bitsInWord [
	^32
]

{ #category : #opcodes }
TargetAwareARM >> byteSwappedNopOpcode [
	"For the Tsts class which keeps filling BitMaps with nop, provide one swapped so it turns out correct when disassembled
	mov r0, r0 swapped -> "
	^ 16r00000A0E1
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> cResultRegister [
	^self r0
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> cResultRegister: aValue [
	self r0: aValue
]

{ #category : #'accessing-registers' }
TargetAwareARM >> cflag [
	"Bit 29 in the cpsr: Carry condition code flag."
	^self rawCPSR >> 29 bitAnd: 1.
]

{ #category : #'accessing-registers' }
TargetAwareARM >> cflag: unsignedInteger [
	^self setCPSRbit: 29 to: unsignedInteger
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> controlRegisterGetters [
	^#(pc eflags)
]

{ #category : #disassembly }
TargetAwareARM >> disassembleFrom: startAddress to: endAddress in: memory for: aSymbolManager "<Cogit>" labels: labelDictionary on: aStream [
	| address |
	address := startAddress.
	[address <= endAddress] whileTrue:
		[[:sizeArg :stringArg| | size string index offset |
		size := sizeArg.
		string := stringArg.
		(aSymbolManager labelForPC: address) ifNotNil:
			[:label| aStream nextPutAll: label; nextPut: $:; cr].
		(labelDictionary at: address ifAbsent: []) ifNotNil:
			[:label|
			self printLabel: label on: aStream at: address for: aSymbolManager.
			label isArray ifTrue:
				[string := nil.
				 size := label third]].
		string ifNotNil:
			[aStream nextPutAll: (self decorateDisassembly: string for: aSymbolManager fromAddress: address).
			 (string includesSubstring: ': ldr	') ifTrue:"i.e. colon space 'ldr' tab" 
				[(index := string indexOfSubCollection: ' [pc, #' startingAt: 1) > 0
					ifTrue:
						[offset := Integer readFrom: (ReadStream on: string from: index + 7 to: (string indexOf: $] startingAt: index + 7) - 1)]
					ifFalse:
						[(string indexOfSubCollection: ' [pc]' startingAt: 1) > 0 ifTrue:
							[offset := 0]].
				 offset ifNotNil:
					[offset := address + 8 + offset.
					 labelDictionary
						at: offset
						ifPresent:
							[:entry|
							entry isString
								ifTrue: [labelDictionary at: offset put: {#literal. offset. 4. entry}]
								ifFalse: [self assert: (entry isArray and: [entry first == #literal])]]
						ifAbsentPut: [{#literal. offset. 4}]]]].
		aStream cr; flush.
		address := address + size]
			valueWithArguments: (self
									primitiveDisassembleAt: address
									inMemory: memory)].
	(labelDictionary at: address ifAbsent: []) ifNotNil:
		[:label| self printLabel: label on: aStream at: address for: aSymbolManager]
]

{ #category : #'accessing-registers' }
TargetAwareARM >> doesNotUnderstand: msg [
	msg numArgs = 0 ifTrue: [^self doesNotUnderstandRegGetter: msg].
	msg numArgs = 1 ifTrue: [^self doesNotUnderstandRegSetter: msg].
	^super doesNotUnderstand: msg
]

{ #category : #'accessing-registers' }
TargetAwareARM >> doesNotUnderstandRegGetter: msg [
	| regName |
	regName := msg selector asString.
	^self gdb getRegisters at: regName ifAbsent: [super doesNotUnderstand: msg].

]

{ #category : #'accessing-registers' }
TargetAwareARM >> doesNotUnderstandRegSetter: msg [
	| regName x |
	regName := msg selector asString allButLast.
	(gdb getRegisters includesKey: regName) ifFalse: [^super doesNotUnderstand: msg].
	x := msg arguments first.
	gdb setRegister: regName to: x.
	^x
]

{ #category : #accessing }
TargetAwareARM >> eflags [
	
	^ (((self nflag << 5 bitOr: self zflag << 4) 
			bitOr: self cflag << 3) 
				bitOr: self vflag << 2)
					bitOr: self ifflags
]

{ #category : #accessing }
TargetAwareARM >> eflags: anUnsignedInteger [
	"set the processor flags from the integer"
	self nflag: (anUnsignedInteger >> 5 bitAnd: 1).
	self zflag: (anUnsignedInteger >> 4 bitAnd: 1).
	self cflag: (anUnsignedInteger >> 3 bitAnd: 1).
	self vflag: (anUnsignedInteger >> 2 bitAnd: 1).
	self ifflags: (anUnsignedInteger bitAnd: 3)
]

{ #category : #accessing }
TargetAwareARM >> endCondition [
"why did the simulator stop?"
	^self shouldBeImplemented
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> endianness [
	^#little
]

{ #category : #accessing }
TargetAwareARM >> errorCode [
"simulator error code"
	^self shouldBeImplemented
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> extractOffsetFromLoadStore: instr [
"work out the address offset implied by instr.
We assume it has been determined it is actually a load store before attaching any meaning to the result.
If it is post-indexed, then the offset must be 0, no matter what else is encoded.
If the instr is immediate-offset, pull the relevent bits out of instr.
If it is register-offset, pull the value from the indicated register."
	|offset shiftAmt shiftType |

	"post-indexed means no offset to the read address"
	(instr bitAnd: 1 << 24) = 0 ifTrue:[^0].
	
	(self instructionIsImmediateOffsetLoadStore: instr)
		ifTrue:["two cases apply - a 12bit immediate for 010 group instructions and an 8bit for 000 group ldrh stuff"
			(instr >> 25 bitAnd: 7) = 2
				ifTrue:[ "immed word or byte op, with 12bit offset"
					offset := instr bitAnd: 16rFFF]
				ifFalse:["halfword 8bit offset"
					offset := (instr bitAnd: 16rF00)>>4 bitOr: (instr bitAnd: 16rF)]].

	(self instructionIsRegisterOffsetLoadStore: instr)
		ifTrue:["both forms use same offset-reg encoding"
			offset := self perform:(self registerStateGetters at:(instr bitAnd: 16rF) + 1).
			(instr >> 25 bitAnd: 7) = 3
				ifTrue:[ "register offset with assorted modifiers"
					"sort out modifiers"
					shiftType := instr >> 5 bitAnd: 3.
					shiftAmt := instr  >>7 bitAnd: 16r1F.
					shiftType = 0"lsl" ifTrue:[offset := offset << shiftAmt.].
					shiftType = 1"lsr" ifTrue:[offset := offset >> shiftAmt].
					shiftType = 2"asr" ifTrue:[offset := offset  >>> shiftAmt].
					"I refuse to countenance using ROR or RRX here. Just Say No" ]
				"halfword stuff register offset uses no modifiers in the form we suport. See ARM DDI0406A p. A8-156"].

	"all forms have the bit 23 up/down flag to account for"
	(instr bitAnd: 1<<23) = 0
						ifTrue:["-ve offset" ^offset negated]
						ifFalse:["+ve offset" ^offset]	
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> floatingPointRegisterStateGetters [
	^#(d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11 d12 d13 d14 d15)
]

{ #category : #'Cog API' }
TargetAwareARM >> flushICacheFrom: anInteger to: anInteger2 [ 

]

{ #category : #accessing }
TargetAwareARM >> fp [
	^self r11
]

{ #category : #accessing }
TargetAwareARM >> fp: anUnsignedInteger [
	^self r11: anUnsignedInteger
]

{ #category : #accessing }
TargetAwareARM >> fpCPSR [
	"The VFP cpsr flags.  Return just the top 4 bits, the actual flags"
	^self rawFPCPSR >>28
]

{ #category : #'floating-point emulation' }
TargetAwareARM >> handleBasicDoubleArithmetic: instr at: pc [
	"Emulate a basic math - add/sub/mul/div -  VFP instruction."
	| rn rd rm vn vm |
	rn := instr >> 16 bitAnd: 15.
	rd := instr >> 12 bitAnd: 15.
	rm := instr bitAnd: 15.
	vn := Float fromIEEE64BitWord: (self perform: (self registerStateGetters at: rn + 18)). "Assume accesses fp regs"
	vm := Float fromIEEE64BitWord: (self perform: (self registerStateGetters at: rm + 18)). "Assume accesses fp regs"

	"simplest to match the entire instruction pattern rather than mess around shifting and masking and merging"
	(instr  bitAnd: 16rFF00FF0)
		caseOf: {
		[16rE200B00 "FMULD"]	->
			[| r |
			 r := vn * vm.
			 self perform: (self registerStateSetters at: rd + 18) with: r asIEEE64BitWord].
		[16rE300B00 "FADDD"]	->
			[| r |
			 r := vn + vm.
			 self perform: (self registerStateSetters at: rd + 18) with: r asIEEE64BitWord].
		[16rE300B40 "FSUBD"]	->
			[| r |
			 r := vn - vm.
			 self perform: (self registerStateSetters at: rd + 18) with: r asIEEE64BitWord].
		[16rE800B00"FDIVD"]	->
			[| r |
			 r := vn / vm.
			 self perform: (self registerStateSetters at: rd + 18) with: r asIEEE64BitWord].}
		otherwise: [self reportPrimitiveFailure].
	self pc: pc + 4
]

{ #category : #'error handling' }
TargetAwareARM >> handleDataFailureIn: aMemory [ 
	^self handleFailingLoadStore: aMemory currentInstructionEncoding at: self gdb pc
]

{ #category : #'error handling' }
TargetAwareARM >> handleExecutionPrimitiveFailureAt: pc "<Integer>" in: memoryArray [ "<Bitmap|ByteArray>"
	"Handle an execution primitive failure for an otherwise unhandled opcode."
	^self reportPrimitiveFailure
]

{ #category : #'floating-point emulation' }
TargetAwareARM >> handleExtendedDoubleArithmetic: instr at: pc [
	"Emulate an extended math - cmp/sqrt/sitod -  VFP instruction."
	| rn rd rm vn vm vd |
	rn := instr >> 16 bitAnd: 15.
	rd := instr >> 12 bitAnd: 15.
	rm := instr bitAnd: 15.
	vn := Float fromIEEE64BitWord: (self perform: (self registerStateGetters at: rn + 18)). "Assume accesses fp regs"
	vm := Float fromIEEE64BitWord: (self perform: (self registerStateGetters at: rm + 18)). "Assume accesses fp regs"

	"simplest to match the entire instruction pattern rather than mess around shifting and masking and merging"
	(instr  bitAnd: 16rFF00FF0)
		caseOf: {
		[16rEB80B80 "FCMPD"]	->
			["read rd, compare with rm (ignore rn) and set FPSCR NZCV flags. Sigh"
				vd := Float fromIEEE64BitWord: (self perform: (self registerStateGetters at: rd + 18)).
				self break].
		[16rEB80BC0 "FSITOD"]	->
			[| r |
			 r := vm asFloat.
			 self perform: (self registerStateSetters at: rd + 18) with: r asIEEE64BitWord].
		[16rEB10BC0 "FSQRTD"]	->
			[| r |
			 r := vm sqrt.
			 self perform: (self registerStateSetters at: rd + 18) with: r asIEEE64BitWord].
		}
		otherwise: [self reportPrimitiveFailure].
	self pc: pc + 4
]

{ #category : #'floating-point emulation' }
TargetAwareARM >> handleFPLoadStore: instr at: pc [
	"Emulate a VFP load/store instruction."
	| rn rd offset |
	rn := instr >> 16 bitAnd: 15.
	rd := instr >> 12 bitAnd: 15.
	offset := instr bitAnd: 16rFF.

	"examine the U and Lbits"
	(instr >>20  bitAnd: 16rF)
		caseOf: {
		[0"Store neg"]	->
			[| r addr|
			addr := (self register: rn) - (offset<<2).
			 r := self perform: (self registerStateGetters at: rd + 18).
			 self unsignedLongLongAt: addr put: r].
		[1"Load neg"]	->
			[| r addr|
			addr := (self register: rn) - (offset<<2).
			r := self unsignedLongLongAt: addr.
			 self perform: (self registerStateSetters at: rd + 18) with: r ].
		[8"Store pos"]	->
			[| r addr|
			addr := (self register: rn) + (offset<<2).
			 r := self perform: (self registerStateGetters at: rd + 18).
			 self unsignedLongLongAt: addr put: r].
		[9"Load pos"]	->
			[| r addr|
			addr := (self register: rn) + (offset<<2).
			r := self unsignedLongLongAt: addr.
			 self perform: (self registerStateSetters at: rd + 18) with: r ].}
		otherwise: [self reportPrimitiveFailure].
	self pc: pc + 4
]

{ #category : #'floating-point emulation' }
TargetAwareARM >> handleFPStatus: instr at: pc [
	"Emulate transferring the FP status to the ARM CPSR."
	| fpcpsr |
	fpcpsr := self fpCPSR.
	self vflag: (fpcpsr bitAnd: 1).
	self cflag: ((fpcpsr >>1) bitAnd: 1).
	self zflag: ((fpcpsr >>2) bitAnd: 1).
	self nflag: ((fpcpsr >>3) bitAnd: 1).
	self pc: pc + 4
]

{ #category : #'error handling' }
TargetAwareARM >> handleFailingFPArithmetic: instr at: pc [
	
	"basic arithmetic"
	(instr bitAnd: 16rF400FB0) = 16rE000B00 ifTrue:
		[^self handleBasicDoubleArithmetic: instr at: pc].
	
	"extension instructions sqrt/cmp/sitod"
	(instr bitAnd: 16rFF00F70) = 16rEB00B40 ifTrue:
		[^self handleExtendedDoubleArithmetic: instr at: pc].
	
	"move ARM reg to coproc reg. "
	(instr bitAnd: 16rFF00FFF) = 16rE000A10 ifTrue:
		[^self handleRegToDoubleTransfer: instr at: pc].

	"move FPSCR reg to ARM CPSR"
	(instr bitAnd: 16rFFFFFFF) = 16rEF1FA10 ifTrue:
		[^self handleFPStatus: instr at: pc].

	"load and store ops. All doubles; we only use FLDD & FSTD"
	(instr bitAnd: 16rF600F00) = 16rD000B00 ifTrue:
		[^self handleFPLoadStore: instr at: pc].
	
	
	^self reportPrimitiveFailure
]

{ #category : #'error handling' }
TargetAwareARM >> handleFailingLoadStore: instr at: pc [
	"See e.g. ARM DDI0406A pp. A8-120, 124, 128, 132, 152, 156, etc. etc"
	| baseReg regIdx destReg srcReg offset |

	"find the register used as the base of the address and the register to load into or store from"
	baseReg := self registerStateGetters at: ((instr bitShift: -16) bitAnd: 15) + 1.
	srcReg :=  self registerStateGetters at: (regIdx := ((instr bitShift: -12) bitAnd: 15) + 1).
	destReg := self registerStateSetters at: regIdx.

	"work out the relevant offset, whether an immediate or register value"
	offset := self extractOffsetFromLoadStore: instr.
	
	(self instructionIsLDR: instr) ifTrue:
		[^(ProcessorSimulationTrap
				pc: pc
				nextpc: pc + 4 
				address: (self perform: baseReg) + offset
				type: #read
				accessor: destReg)
			signal].
	(self instructionIsSTR: instr) ifTrue:
		[^(ProcessorSimulationTrap
				pc: pc
				nextpc: pc + 4 
				address: (self perform: baseReg) + offset
				type: #write
				accessor: srcReg)
			signal].
	(self instructionIsLDRB: instr) ifTrue:
		[^(ProcessorSimulationTrap
				pc: pc
				nextpc: pc + 4 
				address: (self perform: baseReg) + offset
				type: #read
				accessor: destReg)
			signal].
	(self instructionIsSTRB: instr) ifTrue:
		[^(ProcessorSimulationTrap
				pc: pc
				nextpc: pc + 4 
				address: (self perform: baseReg) + offset
				type: #write
				accessor: srcReg)
			signal].
	(self instructionIsLDRH: instr) ifTrue:
		[^(ProcessorSimulationTrap
				pc: pc
				nextpc: pc + 4 
				address: (self perform: baseReg) + offset
				type: #read
				accessor: destReg)
			signal].
	(self instructionIsSTRH: instr) ifTrue:
		[^(ProcessorSimulationTrap
				pc: pc
				nextpc: pc + 4 
				address: (self perform: baseReg) + offset
				type: #write
				accessor: srcReg)
			signal].

	self error: 'handleFailingLoadStore:at: invoked for non-load/store?'
]

{ #category : #'floating-point emulation' }
TargetAwareARM >> handleOneRegTransferDoubleArithmetic: instr at: pc [
	"Emulate a one-register transfer VFP instruction."
	| rn rd rm vn vm |
	rn := instr >> 16 bitAnd: 15.
	rd := instr >> 12 bitAnd: 15.
	rm := instr bitAnd: 15.
	vn := Float fromIEEE64BitWord: (self perform: (self registerStateGetters at: rn + 18)). "Assume accesses fp regs"
	vm := Float fromIEEE64BitWord: (self perform: (self registerStateGetters at: rm + 18)). "Assume accesses fp regs"
	(instr >> 18 bitAnd: 31)
		caseOf: {
		[8 "FMULD"]	->
			[| r |
			 r := vn * vm.
			 self perform: (self registerStateSetters at: rd + 18) with: r asIEEE64BitWord].
		[12"FADDD/FSUBD"]	->
			[self shouldBeImplemented].
		[32"FDIVD"]	->
			[self shouldBeImplemented].
		[45"FCMPD"]	->
			[self shouldBeImplemented]. }
		otherwise: [self reportPrimitiveFailure].
	self pc: pc + 4
]

{ #category : #'floating-point emulation' }
TargetAwareARM >> handleRegToDoubleTransfer: instr at: pc [
	"Emulate an ARM to VFP instruction."
	| rn rd vn |
	rn := (instr >> 16 bitAnd: 15) << 1 bitOr: (instr >>6 bitAnd: 1).
	rd := instr >> 12 bitAnd: 15.
	vn := self register: rn.

	self perform: (self registerStateSetters at: rd + 18) with: vn.
	self pc: pc + 4
]

{ #category : #accessing }
TargetAwareARM >> ifflags [
	"Bits 26:25 in the cpsr: If-Then execution state bits for the Thumb IT (If-Then) instruction."
	^self rawCPSR >> 25 bitAnd: 3.
]

{ #category : #accessing }
TargetAwareARM >> ifflags: unsignedInteger [
	| x y |
	x := unsignedInteger >> 1 bitAnd: 1.
	y := unsignedInteger bitAnd: 1.
	self setCPSRbit: 26 to: x; setCPSRbit: 25 to: y
]

{ #category : #'processor setup' }
TargetAwareARM >> initializeStackFor: aCogit [
	"Different cpus need different stack alignment etc, so handle the details here."
	aCogit setStackAlignment: 8 expectedSPOffset: 0 expectedFPOffset: 0.
	PostBuildStackDelta := 0 
]

{ #category : #testing }
TargetAwareARM >> instructionIsAddSP: instr [
	"is this an add sp, sp, #? -  instruction?"
	^(instr bitShift: -28) < 16rF "test for allowed condcode - 0xF is extension"
	  and: [(instr bitAnd: 16rFFFFF00) = 16r28DD000]
]

{ #category : #testing }
TargetAwareARM >> instructionIsAlignSP: instr [
"is this a subs sp, sp, #4 -  instruction?"
	^(instr bitShift: -28) < 16rF "test for allowed condcode - 0xF is extension"
	  and: [(instr bitAnd: 16rFFFFFFF) = 16r2DDD004]
]

{ #category : #testing }
TargetAwareARM >> instructionIsAnyFPArithmetic: instr [
	"Identify VFP instructions.
	 See C3.1 - C3.4 in the ARM ARM v5 DDI01001."
	| cp isFP |
	
	"All FP instructions are coprocessor instructions on coproc 10 or 11"
	cp := (instr bitShift: -8) bitAnd: 16rF.
	isFP := cp = 10 or:[cp = 11].
	(isFP and: [((instr bitShift: -25) bitAnd: 7) = 6]) ifTrue: [^true].
	(isFP and: [((instr bitShift: -24) bitAnd: 16rF) = 16rE]) ifTrue: [^true].
	
	"nope"
	^false
]

{ #category : #testing }
TargetAwareARM >> instructionIsAnyLoadStore: instr [
	"is this any of the LDR,STR instructions?
	We handle byte, word, and halfword versions but NOT NOT signed extend, double or privileged versions"
	
	^(self instructionIsImmediateOffsetLoadStore: instr) or:[self instructionIsRegisterOffsetLoadStore: instr]
]

{ #category : #testing }
TargetAwareARM >> instructionIsB: instr [
"is this a B <offset> instruction?"
	"first test for non-NV condition code; some important instructions use it"
	^(instr bitShift: -28) < 16rF
	"See ARM DDI0406A p. A8-44"
	 and: [((instr bitShift: -24) bitAnd: 16rF) = 16rA]
]

{ #category : #testing }
TargetAwareARM >> instructionIsBL: instr [
"is this a BL <offset> instruction?"
	"first test for non-NV condition code; some important instructions use it"
	^(instr bitShift: -28) < 16rF
	"see ARM DDI0406A p. A8-58"
	  and: [((instr bitShift: -24) bitAnd: 16rF) = 16rB]
]

{ #category : #testing }
TargetAwareARM >> instructionIsBLX: instr [
"is this a BLX <targetReg> instruction? We DO NOT support the BLX immed version"
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	"See ARM DDI0406A p A8-60"
 	^(instr bitAnd: 16r0FFFFFF0) = 16r12FFF30
]

{ #category : #testing }
TargetAwareARM >> instructionIsBX: instr [
"is this a BX <targetReg> instruction?"
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	"See ARM DDI0406A p. A8-62"
 	^(instr bitAnd: 16r0FFFFFF0) = 16r12FFF10
]

{ #category : #testing }
TargetAwareARM >> instructionIsImmediateOffsetLoadStore: instr [
	"is this any of the immediate offset LDR,STR instructions?"
	| op1 |
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	"test for 010 group of load/stores"
	op1 := (instr bitShift: -25) bitAnd: 7.
	op1 = 2 ifTrue:[^true].

	"test for the ridiculously muddled 000 group"
	op1 > 0 ifTrue:[^false].
	"bit 21 must not be 1 and bit 22 must be 1"
	((instr bitShift: -21) bitAnd: 3) = 2 ifFalse:[^false].
	"bits 4:7need to be 16rB for our purpose"
	^(instr bitAnd: 16rF0) = 16rB0
]

{ #category : #testing }
TargetAwareARM >> instructionIsLDR: instr [
"is this a LDR instruction?"
	| foo |
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	"See ARM DDI0406A p. A8-120"
	foo := (instr bitShift: -20) bitAnd: 16rE5.
	^foo = 16r41 "ldr r1, [r2, #+/-imm]"
		or:[foo = 16r61 "ldr r1, [r2, r3]"]
]

{ #category : #testing }
TargetAwareARM >> instructionIsLDRB: instr [
"is this a LDRB instruction?"
	| foo |
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	"See ARM DDI0406A p. A8-128"
	foo := (instr bitShift: -20) bitAnd: 16rE5.
	^foo = 16r45 "ldrb r1, [r2, #+/-imm]"
		or:[foo = 16r65 "ldrb r1, [r2, r3]"]
]

{ #category : #testing }
TargetAwareARM >> instructionIsLDRH: instr [
"is this a LDRH instruction?"
	| foo |
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	"See ARM DDI0406A p. A8-154"
	(instr bitAnd: 16rF0) = 16rB0 ifFalse:[^false].
	foo := (instr bitShift: -20) bitAnd: 16rE3.
	^foo = 16r3 "ldrh r1, [r2, #+/-imm]"
		or:[foo = 16r1 "ldrh r1, [r2, r3]"]
]

{ #category : #testing }
TargetAwareARM >> instructionIsLDRSP: instr [
"is this a LDR sp, [??] instruction? Special case to detect LDR sp, [] affecting stack pointer"
	^(self instructionIsLDR: instr)  and:[((instr bitShift: -12) bitAnd: 16rF) = 13]
]

{ #category : #testing }
TargetAwareARM >> instructionIsPop: instr [
"is this a pop - ldr r??, [sp], #4 -  instruction?"
	^(instr bitShift: -28) < 16rF "test for allowed condcode - 0xF is extension" and: [(instr bitAnd: (16rFFF0FFF)) = (16r49D0004)]
]

{ #category : #testing }
TargetAwareARM >> instructionIsPush: instr [
"is this a push -str r??, [sp, #-4] -  instruction?"
	^(instr bitShift: -28) < 16rF "test for allowed condcode - 0xF is extension" and: [(instr bitAnd: (16rFFF0FFF)) = (16r52D0004)]
]

{ #category : #testing }
TargetAwareARM >> instructionIsRegisterOffsetLoadStore: instr [
	"is this any of the register offset LDR,STR instructions?"
	| op1 |
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	
	op1 := (instr bitShift: -25) bitAnd: 7.
	"test for the 011 group - check bit 4 as well"
	(op1 = 3 and:[(instr bitAnd: 16r10) = 0]) ifTrue:[^true].
	"test for the ridiculously muddled 000 group"
	op1 > 0 ifTrue:[^false].
	"bit 21 & 22 must not be 1"
	((instr bitShift: -21) bitAnd: 3) = 0 ifFalse:[^false].
	"bits 4:7need to be 16rB for our purpose"
	^(instr bitAnd: 16rF0) = 16rB0
]

{ #category : #testing }
TargetAwareARM >> instructionIsReturnViaLR: instr [
"is this a MOV pc, lr instruction?"
	^instr =  16rE1A0F00E
]

{ #category : #testing }
TargetAwareARM >> instructionIsSTR: instr [
"is this a STR instruction?"
	| foo |
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	"See ARM DDI0406A p. A8-382"
 	foo := (instr bitShift: -20) bitAnd: 16rE5. 
	^foo = 16r40 "str r1, [r2, #+/-imm]" 
		or:[foo = 16r60] "str r1, [r2, #-imm]"
]

{ #category : #testing }
TargetAwareARM >> instructionIsSTRB: instr [
"is this a STRB instruction?"
	| foo |
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	"See ARM DDI0406A p. A8-388"
 	foo := (instr bitShift: -20) bitAnd: 16rE5. 
	^foo = 16r44 "strb r1, [r2, #+/-imm]" 
		or:[foo = 16r64] "strb r1, [r2, #-imm]"
]

{ #category : #testing }
TargetAwareARM >> instructionIsSTRH: instr [
"is this a STRH instruction?"
	| foo |
	"first test for non-NV condition code; some important instructions use it"
	(instr bitShift: -28) = 16rF ifTrue:[^false].
	"See ARM DDI0406A p. A8-154"
	(instr bitAnd: 16rF0) = 16rB0 ifFalse:[^false].
	foo := (instr bitShift: -20) bitAnd: 16rE3.
	^foo = 16r4 "strh r1, [r2, #+/-imm]"
		or:[foo = 16r0 "strh r1, [r2, r3]"]
]

{ #category : #testing }
TargetAwareARM >> instructionIsSTRSP: instr [
"is this a STR sp, [??] instruction? Special case to detect STR sp, [] affecting stack pointer"
	^(self instructionIsSTR: instr)  and:[((instr bitShift: -12) bitAnd: 16rF) = 13]
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> integerRegisterState [
	"Answer a WordArray of the integer registers, the pc and the flags."
	^{	self r0. self r1. self r2. self r3. self r4. self r5. self r6. self r7. self r8. 
		self r9. self sl. self fp. self r12. self sp. self lr. self pc. self rawCPSR}
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> leafRetpcIn: aMemory [
	"Answer the retpc assuming that the processor is in a simulated call established
	 by simulateLeafCallOf:nextpc:memory:"
	^self lr
]

{ #category : #accessing }
TargetAwareARM >> lr [
	^gdb getRegister: 'lr'

]

{ #category : #accessing }
TargetAwareARM >> lr: anUnsignedInteger [
	gdb setRegister: 'lr' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #accessing }
TargetAwareARM >> nflag [
	"Bit 31 in the cpsr: Negative condition code flag."
	^self rawCPSR >> 31 bitAnd: 1.
]

{ #category : #accessing }
TargetAwareARM >> nflag: unsignedInteger [
	^self setCPSRbit: 31 to: unsignedInteger
]

{ #category : #opcodes }
TargetAwareARM >> nopOpcode [
	"mov r0, r0"
	^ 16rE1A00000
]

{ #category : #accessing }
TargetAwareARM >> pc [
	^gdb getRegister: 'pc'

]

{ #category : #accessing }
TargetAwareARM >> pc: anUnsignedInteger [
	gdb setRegister: 'pc' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #opcodes }
TargetAwareARM >> popPcOpcode [
"See also CogARMCompiler>concretizePopR"
	^16rE49DF004
]

{ #category : #execution }
TargetAwareARM >> popWordIn: aMemory [ 
	| sp word |
	word := aMemory unsignedLongAt: (sp := self sp) + 1 bigEndian: false.
	self sp: sp + 4.
	^word
]

{ #category : #execution }
TargetAwareARM >> postCallArgumentsNumArgs: numArgs "<Integer>" in: memory [ "<ByteArray|Bitmap>"
	"Answer an argument vector of the requested size after a vanilla
	 ABI call. For ARM the Procedure Calling Specification can be found in IHI0042D_aapcs.pdf.
	On ARM this typically means accessing r0 through r3 and fetching additional arguments from the stack, acording to pages 20f. aapcs.
	We assume that all arguments are single word arguments, which can not be supplied on co-processor-registers.
	 For compatibility with Cog/Slang we answer unsigned values."
	^(1 to: numArgs) collect: [:i |
		i < 5 
			ifTrue: [self perform: (self registerStateGetters at: i)]
			"ARM uses a full descending stack. Directly after calling a procedure, nothing but the arguments are pushed."
			ifFalse: [memory unsignedLongAt: self sp + (i-5)*4 bigEndian: false]].
]

{ #category : #primitives }
TargetAwareARM >> primitiveErrorAndLog [
	"Answer an array of the current error code and log contents"
	^self primitiveFailed
]

{ #category : #primitives }
TargetAwareARM >> primitiveFlushICacheFrom: startAddress "<Integer>" To: endAddress [ "<Integer>"
	"Flush the icache in the requested range"
	^self primitiveFailed
]

{ #category : #primitives }
TargetAwareARM >> primitiveResetCPU [
	"Reset the receiver to registers all zero, and protected 32-bit mode."
	^self reportPrimitiveFailure
]

{ #category : #printing }
TargetAwareARM >> printFields: fields inRegisterState: registerStateVector on: aStream [
	| rsvs |
	aStream ensureCr.
	rsvs := registerStateVector readStream.
	fields withIndexDo:
		[:sym :index| | val |
		sym = #cr
			ifTrue: [aStream cr]
			ifFalse:
				[(val := rsvs next) isNil ifTrue: [^self].
				aStream nextPutAll: sym; nextPut: $:; space.
				val printOn: aStream base: 16 length: 8 padded: true.
				#eflags == sym
					ifTrue:
						[aStream space.
						 "'FIVCZN'"'--VCZN' withIndexDo:
							[:flag :bitIndex|
							flag ~= $- ifTrue:
								[aStream nextPut: flag; nextPutAll: 'F='; print: (val bitAnd: 1 << (bitIndex - 1)) >> (bitIndex - 1); space]]]
					ifFalse:
						[val > 16 ifTrue:
							[aStream space; nextPut: $(.
							 val printOn: aStream base: 10 length: 1 padded: false.
							 aStream nextPut: $)]].
				(fields at: index + 1) ~~ #cr ifTrue:
					[aStream tab]]]
]

{ #category : #printing }
TargetAwareARM >> printLabel: label on: aStream at: address for: aSymbolManager [
	"Print label on aStream.  The label is either a triple of {type, printer, size} or a simple string."
	label isArray
		ifTrue: [label first == #literal
					ifTrue:
						[label size = 4 ifTrue:
							[aStream nextPutAll: label last; nextPut: $:; cr].
						 aStream
							nextPutAll: (address printStringBase: 16 length: 8 padded: true) asLowercase;
							nextPut: $:; space;
							nextPutAll: ((aSymbolManager objectMemory longAt: address) printStringBase: 16 length: 8 padded: true) asLowercase.
						 (aSymbolManager lookupAddress: (aSymbolManager objectMemory longAt: address)) ifNotNil:
							[:name| aStream space; nextPutAll: name]]
					ifFalse:
						[aStream
							nextPutAll: label first;
							nextPut: $:; cr;
							nextPutAll: (address printStringBase: 16 length: 8 padded: true) asLowercase;
							nextPut: $:; space;
							nextPutAll: (aSymbolManager perform: label second with: address) asString]]
		ifFalse: [aStream nextPutAll: label; nextPut: $:; cr]
]

{ #category : #printing }
TargetAwareARM >> printNameOn: aStream [ 
	super printOn: aStream
]

{ #category : #printing }
TargetAwareARM >> printRegisterState: registerStateVector on: aStream [
	self printFields: #(	r0 r1 r2 r3 cr
						r4 r5 r6 r7 cr
						r8 r9 sl fp cr
						r12 sp lr pc eflags cr)
		inRegisterState: registerStateVector
		on: aStream
]

{ #category : #printing }
TargetAwareARM >> printRegisterStateExceptPC: registerStateVector on: aStream [
	self printFields: #(	r0 r1 r2 r3 cr
						r4 r5 r6 r7 cr
						r8 r9 sl fp cr
						r12 sp lr eflags cr)
		inRegisterState: registerStateVector
		on: aStream
]

{ #category : #accessing }
TargetAwareARM >> priorPc [
"where did the simulator stop just before a prefetch abort?"
	^self shouldNotImplement
]

{ #category : #accessing }
TargetAwareARM >> priorPc: val [
	^self shouldNotImplement
]

{ #category : #execution }
TargetAwareARM >> pushWord: aValue in: aMemory [
	aMemory longAt: (self sp: self sp - 4) + 1 put: aValue bigEndian: false
]

{ #category : #accessing }
TargetAwareARM >> qflag [
	"Bit 27 in the cpsr: Cumulative saturation bit."
	^self rawCPSR >> 27 bitAnd: 1.
]

{ #category : #accessing }
TargetAwareARM >> qflag: unsignedInteger [
	^self setCPSRbit: 27 to: unsignedInteger
]

{ #category : #accessing }
TargetAwareARM >> rawCPSR [
	^gdb getRegister: 'cpsr'
]

{ #category : #accessing }
TargetAwareARM >> rawCPSR: anUnsignedInteger [
	gdb setRegister: 'cpsr' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #accessing }
TargetAwareARM >> rawFPCPSR [
	"The VFP cpsr register as seen by gdb."
	^gdb getRegister: 'fpscr'
]

{ #category : #accessing }
TargetAwareARM >> rawFPCPSR: anUnsignedInteger [
	gdb setRegister: 'fpscr' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #accessing }
TargetAwareARM >> register: int0to15 [
"return the value of register"
	self assert:[int0to15 between: 0 and:15].
	^self shouldBeImplemented
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> registerState [
	^{	self r0. self r1. self r2. self r3. self r4. self r5. self r6. self r7.
		self r8. self r9. self sl. self fp. self r12. self sp. self lr. self pc. self eflags.
		self d0. self d1. self d2. self d3. self d4. self d5. self d6. self d7.
		self d8. self d9. self d10. self d11. self d12. self d13. self d14. self d15 }
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> registerStateGetters [
	^#(	r0 r1 r2 r3 r4 r5 r6 r7 r8 r9 sl fp r12 sp lr pc eflags
		d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 d10 d11 d12 d13 d14 d15)
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> registerStatePCIndex [
	"Answer the index of the PC register in the Array answered by integerRegisterState"
	^16
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> registerStateSetters [
	^#(	r0: r1: r2: r3: r4: r5: r6: r7: r8: r9: sl: fp: r12: sp: lr: pc: eflags:
		d0: d1: d2: d3: d4: d5: d6: d7: d8: d9: d10: d11: d12: d13: d14: d15:)
]

{ #category : #'processor setup' }
TargetAwareARM >> reset [
	self priorPc: 0;
		primitiveResetCPU
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> retpcIn: aMemory [
	"The return address is on the stack, having been pushed by either
	 simulateCallOf:nextpc:memory: or simulateJumpCallOf:memory:"
	^aMemory unsignedLongAt: self fp + 5 bigEndian: false
]

{ #category : #'target connection' }
TargetAwareARM >> runThinshellPrologue [
	"Right now, the thinshell prologue on ARM does not work well.
	Nor is it needed at this stage.
	So, do nothing instead of c."
]

{ #category : #accessing }
TargetAwareARM >> setCPSRbit: position to: oneOrZero [
	"Flip bit 'position' (counted from LSB=0 to MSB=31) in the CPSR to oneOrZero"
	self rawCPSR: (self rawCPSR bitAt: position+1 put: oneOrZero).
	^oneOrZero
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> setFramePointer: framePointer stackPointer: stackPointer [
	"Initialize the processor's frame and stack pointers"
	self fp: framePointer.
	self sp: stackPointer
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> setRegisterState: aRegisterStateArray [
	"N.B. keep in sync with voidRegisterState"

	self r0:  (aRegisterStateArray at: 1).
	self r1: (aRegisterStateArray at: 2).
	self r2: (aRegisterStateArray at: 3).
	self r3: (aRegisterStateArray at: 4).
	self r4: (aRegisterStateArray at: 5).
	self r5: (aRegisterStateArray at: 6).
	self r6: (aRegisterStateArray at: 7).
	self r7: (aRegisterStateArray at: 8).
	self r8: (aRegisterStateArray at: 9).
	self r9: (aRegisterStateArray at: 10).
	self r10: (aRegisterStateArray at: 11).
	self fp: (aRegisterStateArray at: 12).
	self r12: (aRegisterStateArray at: 13).
	self sp: (aRegisterStateArray at: 14).
	self lr: (aRegisterStateArray at: 15).
	self pc: (aRegisterStateArray at: 16).
	self eflags:  (aRegisterStateArray at: 17).
]

{ #category : #accessing }
TargetAwareARM >> sflag [
"The ARM cpsr flags are kept as individual fields in the Alien structure. The address here is the 1-based byte offset into the ARMul_State structure"
	^self shouldNotImplement
]

{ #category : #accessing }
TargetAwareARM >> sflag: unsignedInteger [
"The ARM cpsr flags are kept as individual fields in the Alien structure. The address here is the 1-based byte offset into the ARMul_State structure"
self shouldNotImplement
]

{ #category : #execution }
TargetAwareARM >> simulateBuildFrameIn: aMemory for: evaluable [
	
	self pushWord: self lr in: aMemory.
	self pushWord: self fp in: aMemory.
	self fp: self sp.
]

{ #category : #execution }
TargetAwareARM >> simulateCallOf: address nextpc: nextpc memory: aMemory [
	"Simulate a frame-building call of address.  Build a frame since
	a) this is used for calls into the run-time which are unlikely to be leaf-calls"
	"This method builds a stack frame as expected by the simulator, not as defined by ARM aapcs-abi.
	In ARM aapcs, every method can define for itself, wether it wants to push lr (nextpc), and wether it 
	uses a frame pointer. The standard never mentions a fp. It merely defines r4-r11 to be callee-saved."

	self pushWord: self lr in: aMemory.
	self pushWord: self fp in: aMemory.
	self fp: self sp.
	PostBuildStackDelta ~= 0 ifTrue:
		[self sp: self sp - PostBuildStackDelta]. "In order to satisfy the CStackAlignment check by cogit, which is only valid on IA32 platforms."
	self pc: address
]

{ #category : #execution }
TargetAwareARM >> simulateJumpCallOf: address memory: aMemory [
	"Simulate a frame-building jump of address.  Build a frame since
	a) this is used for calls into the run-time which are unlikely to be leaf-calls"
	"This method builds a stack frame as expected by the simulator, not as defined by ARM aapcs-abi.
	In ARM aapcs, every method can define for itself, wether it wants to push lr (nextpc), and wether it 
	uses a frame pointer. The standard never mentions a fp. It merely defines r4-r11 to be callee-saved."

	self assert: self sp \\ 8 = 0. "This check ensures, that we conform with ARM abi. Before doing anything to the stack, we ensure 2-word alignment."
	self pushWord: self lr in: aMemory.
	self pushWord: self fp in: aMemory.
	self fp: self sp.
	PostBuildStackDelta ~= 0 ifTrue:
		[self sp: self sp - PostBuildStackDelta]. "In order to satisfy the CStackAlignment check by cogit, which is only valid on IA32 platforms."
	self pc: address
]

{ #category : #execution }
TargetAwareARM >> simulateLeafCallOf: address nextpc: nextpc memory: aMemory [
	self lr: nextpc.
	self pc: address
]

{ #category : #execution }
TargetAwareARM >> simulateLeafReturnIn: aMemory [
	self pc: self lr
]

{ #category : #execution }
TargetAwareARM >> simulateReturnIn: aMemory [
	PostBuildStackDelta ~= 0 ifTrue:
		[self sp: self sp + PostBuildStackDelta].
	self fp: (self popWordIn: aMemory).
	"According to tpr, most C compilers implement return by simply
	 popping into the pc, rather than popping through the link register."
	self pc: (self popWordIn: aMemory)
]

{ #category : #accessing }
TargetAwareARM >> sl [
	^self r10
]

{ #category : #accessing }
TargetAwareARM >> sl: anUnsignedInteger [
self shouldNotImplement
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> smashABICallerSavedRegistersWithValuesFrom: base by: step [
	"limited list of registers to clear out when simulating an ABI call.
	 Smash neither R0 nor R1 since many abi calls return 2 results or a 64-bit dual-reg value.
	 LR has to be left alone becasue a leaf call doesn't push it."

	#(r2: r3: r9: r12:) withIndexDo:
		[:accessor :index|
		self perform: accessor with: index - 1 * step + base]
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> smashCallerSavedRegistersWithValuesFrom: base by: step in: aMemory [
	#(r0: r1: r2: r3: r9: r12: lr:) withIndexDo:
		[:accessor :index|
		self perform: accessor with: index - 1 * step + base]
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> smashRegisterAccessors [
	^#(r0: r1: r2: r3: r4: r5: r6: r7: r8: r9: r10: "11=FP" r12: "13=SP, 14=LR, 15=PC")
]

{ #category : #accessing }
TargetAwareARM >> sp [
	^gdb getRegister: 'sp'

]

{ #category : #accessing }
TargetAwareARM >> sp: anUnsignedInteger [
	gdb setRegister: 'sp' to: anUnsignedInteger.
	^anUnsignedInteger
]

{ #category : #accessing }
TargetAwareARM >> tflag [
"The ARM cpsr flags are kept as individual fields in the Alien structure. The address here is the 1-based byte offset into the ARMul_State structure"
"This would be the Thumb flag if we have it -which depends rather oddly on the compiletime flags used to build the ARMulator. Sigh"
	^0
]

{ #category : #accessing }
TargetAwareARM >> tflag: unsignedInteger [
"The ARM cpsr flags are kept as individual fields in the Alien structure. The address here is the 1-based byte offset into the ARMul_State structure"
"This would be the Thumb flag if we have it -which depends rather oddly on the compiletime flags used to build the ARMulator. Sigh"
self shouldNotImplement
]

{ #category : #accessing }
TargetAwareARM >> vflag [
	"Bit 28 in the cpsr: Overflow condition code flag."
	^self rawCPSR >> 28 bitAnd: 1.
]

{ #category : #accessing }
TargetAwareARM >> vflag: unsignedInteger [
	^self setCPSRbit: 28 to: unsignedInteger
]

{ #category : #'accessing-abstract' }
TargetAwareARM >> voidRegisterState [
	"N.B. keep in sync with setRegisterState:"
	self setRegisterState: (Array new: 17 withAll: 0)
]

{ #category : #accessing }
TargetAwareARM >> zflag [
	"Bit 30 in the cpsr: Zero condition code flag."
	^self rawCPSR >> 30 bitAnd: 1.
]

{ #category : #accessing }
TargetAwareARM >> zflag: unsignedInteger [
	^self setCPSRbit: 30 to: unsignedInteger
]
