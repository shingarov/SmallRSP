"
In Target-Agnostic Modtalk, when we are AoT-ing the binary image, somewhere in the address space there is the arena where we construct the object heap, and the nZone.  Instances of my concrete subclasses know where these two addresses are.
The TAM Thinshell, as described in the Kilpela-Shingarov report, and available in the shingarov/thinshell GitHub repo, is an ELF binary which doesn't link to the C runtime.  Its simple, processor-specific assembly source declares areas for heap, nZone and stack, and a _start entry point leading into a ""start chain"".  The start chain has a few instructions to load the addresses of the heap, nZone and stack into register defined by convention, a magic-point for transfering control to the outer-Smalltalk ProgramBuilder, and a jump to the beginning of the nZone.
"
Class {
	#name : #ModtalkThinshellAddressSpace,
	#superclass : #Object,
	#instVars : [
		'nZone',
		'heap',
		'stack'
	],
	#category : #'GDB-tests-Connected'
}

{ #category : #'instance creation' }
ModtalkThinshellAddressSpace class >> architectures [
	^{ 'powerpc' . 'mips' . 'riscv64' }
]

{ #category : #'instance creation' }
ModtalkThinshellAddressSpace class >> gdb: aGDB [
	| concreteClass |
	concreteClass := self withAllSubclasses detect: [ :candidate | candidate architectures includes: aGDB processorDescription architectureName ].
	^concreteClass basicNew
		executeStartChain: aGDB;
		yourself
]

{ #category : #'register conventions' }
ModtalkThinshellAddressSpace class >> registerAssignments [
	^Dictionary newFromAssociations:
	{
	(#R  -> 'r1').
	(#A  -> 'r2').
	(#A  -> 'r3').
	(#FP -> 'r4').

	(#Scratch1  -> 'r5').
	(#Scratch2  -> 'r6').
	(#Scratch3  -> 'r7').
	(#Scratch4  -> 'r8').
	(#Scratch5  -> 'r9').
	(#Scratch6  -> 'r10').
	(#Scratch7  -> 'r11').
	(#Scratch8  -> 'r12').
	(#Scratch9  -> 'r13').
	(#Scratch10 -> 'r14').
	
	(#NZone -> 'r16').
	(#HEAP  -> 'r17').
	(#SP    -> 'r18').
	(#VPC   -> 'r19').
	
	(#NativePC -> 'pc').
	}
	
]

{ #category : #initialization }
ModtalkThinshellAddressSpace >> executeStartChain: gdb [
	| regs |
	regs := gdb s; "the first nop"
		s; s; "lis/ori 16"
		s; s;
		s; s;
		s; s; "2 nops"
		getRegisters.

	nZone := regs at: (self regNameFor: #NZone pdl: gdb processorDescription).
	heap := regs at: (self regNameFor: #HEAP pdl: gdb processorDescription).
	stack := regs at: (self regNameFor: #SP pdl: gdb processorDescription).
]

{ #category : #accessing }
ModtalkThinshellAddressSpace >> heap [
	^ heap
]

{ #category : #accessing }
ModtalkThinshellAddressSpace >> heap: anObject [
	heap := anObject
]

{ #category : #accessing }
ModtalkThinshellAddressSpace >> nZone [
	^ nZone
]

{ #category : #accessing }
ModtalkThinshellAddressSpace >> nZone: anObject [
	nZone := anObject
]

{ #category : #'cooperation with gdb' }
ModtalkThinshellAddressSpace >> regNameFor: vRegName pdl: pdl [
	^self class registerAssignments at: vRegName
	"^pdl architectureName = 'riscv64'
		ifTrue: [ ((pdl maps at: 'gpr') backLookup: regNum ifAbsent: [ self error ]) name ]
		ifFalse: [ 'r', regNum printString ]"
]

{ #category : #accessing }
ModtalkThinshellAddressSpace >> stack [
	^ stack
]

{ #category : #accessing }
ModtalkThinshellAddressSpace >> stack: anObject [
	stack := anObject
]
