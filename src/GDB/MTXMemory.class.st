Class {
	#name : #MTXMemory,
	#superclass : #Object,
	#instVars : [
		'ram',
		'layout',
		'currentHeapAllocPtr',
		'setup'
	],
	#category : #'GDB-TAJ'
}

{ #category : #'as yet unclassified' }
MTXMemory class >> jumpTableSize [
	" In entries as opposed to bytes "
	^1024

]

{ #category : #'as yet unclassified' }
MTXMemory >> alloc: nBytes [
	^self baseAlloc: nBytes + 4 "for the hash on 32-bit machines"
]

{ #category : #'as yet unclassified' }
MTXMemory >> architecture [
	^self targetSetup architecture
]

{ #category : #'as yet unclassified' }
MTXMemory >> baseAlloc: nBytes [
	| evenBytes answer |
	answer := currentHeapAllocPtr.
	nBytes \\ 16 = 0
		ifTrue: [ evenBytes := nBytes ]
		ifFalse: 	[ evenBytes := (nBytes bitOr: 16r0F) + 1 ].
	currentHeapAllocPtr := currentHeapAllocPtr + evenBytes.
	^answer
]

{ #category : #'as yet unclassified' }
MTXMemory >> currentGrade [
]

{ #category : #'as yet unclassified' }
MTXMemory >> initialize [
	self initializeGDB.
	^self
]

{ #category : #'as yet unclassified' }
MTXMemory >> initializeGDB [
	setup := TAJTargetSetup current.
	ram := setup gdbClass  
		host: setup host
		port: setup port
		processorDescription: setup architecture isa.
	layout := ThinshellAddressLayout gdb: ram.
	currentHeapAllocPtr := layout heap + (self class jumpTableSize * 4).
	^self
]

{ #category : #'as yet unclassified' }
MTXMemory >> jumpTable [
	^layout heap

]

{ #category : #'as yet unclassified' }
MTXMemory >> layout [
	^layout
]

{ #category : #'as yet unclassified' }
MTXMemory >> newChunk [
]

{ #category : #'as yet unclassified' }
MTXMemory >> ram [
	^ram
]

{ #category : #'as yet unclassified' }
MTXMemory >> startAllocatingOnTarget [
	ram setVRegister: #HEAP to: currentHeapAllocPtr
]

{ #category : #'as yet unclassified' }
MTXMemory >> targetSetup [
	^setup
]
