Class {
	#name : #TrivialMemoryLayout,
	#superclass : #MTXMemoryLayout,
	#instVars : [
		'gdb',
		'nZone',
		'heap',
		'stack'
	],
	#category : #'GDB-TAJ'
}

{ #category : #'instance creation' }
TrivialMemoryLayout class >> gdb: aGDB [
	^self basicNew
		gdb: aGDB;
		initialize
]

{ #category : #initialization }
TrivialMemoryLayout >> executeStartChain [
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
TrivialMemoryLayout >> gdb [
	^ gdb
]

{ #category : #accessing }
TrivialMemoryLayout >> gdb: anObject [
	gdb := anObject
]

{ #category : #accessing }
TrivialMemoryLayout >> heap [
	^ heap
]

{ #category : #accessing }
TrivialMemoryLayout >> heap: anObject [
	heap := anObject
]

{ #category : #initialization }
TrivialMemoryLayout >> initialize [
	super initialize.
	self executeStartChain
]

{ #category : #accessing }
TrivialMemoryLayout >> nZone [
	^ nZone
]

{ #category : #accessing }
TrivialMemoryLayout >> nZone: anObject [
	nZone := anObject
]

{ #category : #'cooperation with gdb' }
TrivialMemoryLayout >> regNameFor: vRegName [
	^'r', (TAJWriter vRegister: vRegName) printString
]

{ #category : #accessing }
TrivialMemoryLayout >> stack [
	^ stack
]

{ #category : #accessing }
TrivialMemoryLayout >> stack: anObject [
	stack := anObject
]
