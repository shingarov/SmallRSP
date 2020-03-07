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

{ #category : #initialization }
ThinshellAddressLayout >> executeStartChain [
	| regs |
	regs := gdb s; "the first nop"
		s; s; "lis/ori 16"
		s; s;
		s; s;
		s; s; "2 nops"
		getRegisters.
		
	TAJWriter initializeRegisterMap.
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
	^'r', (TAJWriter vRegister: vRegName) printString
]

{ #category : #accessing }
ThinshellAddressLayout >> stack [
	^ stack
]

{ #category : #accessing }
ThinshellAddressLayout >> stack: anObject [
	stack := anObject
]
