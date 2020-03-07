"
The TAM Thinshell, as described in the Kilpela-Shingarov report, and available in the shingarov/thinshell GitHub repo, is an ELF binary which doesn't link to the C runtime.  Its simple, processor-specific assembly source declares areas for heap, nZone and stack, and a _start entry point leading into a ""start chain"".  The start chain has a few instructions to load the addresses of the heap, nZone and stack into register defined by convention, a magic-point for transfering control to the outer-Smalltalk ProgramBuilder, and a jump to the beginning of the nZone.
"
Class {
	#name : #ThinshellAddressLayout,
	#superclass : #AddressSpaceLayout,
	#instVars : [
		'gdb',
		'nZone',
		'heap',
		'stack'
	],
	#category : #'GDB-TAJ'
}

{ #category : #'instance creation' }
ThinshellAddressLayout class >> gdb: aGDB [
	^self basicNew
		gdb: aGDB;
		initialize
]

{ #category : #'register conventions' }
ThinshellAddressLayout class >> registerAssignments [
	"This is TAM-specific and probably should not be here."
	^
	(#R  -> 1),
	(#A  -> 2),
	(#A  -> 3),
	(#FP -> 4),

	(#Scratch1  -> 5),
	(#Scratch2  -> 6),
	(#Scratch3  -> 7),
	(#Scratch4  -> 8),
	(#Scratch5  -> 9),
	(#Scratch6  -> 10),
	(#Scratch7  -> 11),
	(#Scratch8  -> 12),
	(#Scratch9  -> 13),
	(#Scratch10 -> 14),
	
	(#NZone -> 16),
	(#HEAP  -> 2),
	(#SP    -> 18),
	(#VPC   -> 19),
	
	(#NativePC -> 'pc')
	
]

{ #category : #initialization }
ThinshellAddressLayout >> executeStartChain [
	| regs |
	regs := gdb s; "the first nop"
		s; s; "lis/ori 16"
		s; s;
		s; s;
		s; s; "2 nops"
		getRegisters.

	nZone := regs at: (self regNameFor: #NZone).
	heap := regs at: (self regNameFor: #HEAP).
	stack := regs at: (self regNameFor: #SP).
]

{ #category : #accessing }
ThinshellAddressLayout >> gdb [
	^ gdb
]

{ #category : #accessing }
ThinshellAddressLayout >> gdb: anObject [
	gdb := anObject
]

{ #category : #accessing }
ThinshellAddressLayout >> heap [
	^ heap
]

{ #category : #accessing }
ThinshellAddressLayout >> heap: anObject [
	heap := anObject
]

{ #category : #initialization }
ThinshellAddressLayout >> initialize [
	super initialize.
	self executeStartChain
]

{ #category : #accessing }
ThinshellAddressLayout >> nZone [
	^ nZone
]

{ #category : #accessing }
ThinshellAddressLayout >> nZone: anObject [
	nZone := anObject
]

{ #category : #'cooperation with gdb' }
ThinshellAddressLayout >> regNameFor: vRegName [
	^'r', (ThinshellAddressLayout registerAssignments at: vRegName) printString
]

{ #category : #accessing }
ThinshellAddressLayout >> stack [
	^ stack
]

{ #category : #accessing }
ThinshellAddressLayout >> stack: anObject [
	stack := anObject
]
