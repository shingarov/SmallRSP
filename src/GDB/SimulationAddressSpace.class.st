Class {
	#name : #SimulationAddressSpace,
	#superclass : #Object,
	#category : #'GDB-RSP'
}

{ #category : #'as yet unclassified' }
SimulationAddressSpace class >> bytesPerElement [
	^1
]

{ #category : #'as yet unclassified' }
SimulationAddressSpace class >> new: bytes [
	"I really hate this design.
	The only reason #new: is here is because there is no concept of conneciton
	between the processor and memory."
	| instance |
	instance := self gdb: TargetAware current gdb.
	instance shmemSize: 120*1024*1024.
	TargetAware current class zeroROChunkIn: instance.
	^instance
]

{ #category : #'as yet unclassified' }
SimulationAddressSpace >> bytesPerElement [
	^self class bytesPerElement
]

{ #category : #'as yet unclassified' }
SimulationAddressSpace >> currentInstruction [
	^self gdb processorDescription decode: self currentInstructionEncoding
]

{ #category : #'as yet unclassified' }
SimulationAddressSpace >> currentInstructionEncoding [
	^self readInt32fromAddr: self gdb pc
]

{ #category : #'as yet unclassified' }
SimulationAddressSpace >> endianness [
	"Answer the endianness OF THE REMOTE TARGET."
	^self gdb processorDescription endian
]

{ #category : #'as yet unclassified' }
SimulationAddressSpace >> isBigEndian [
	^self endianness == #big
]
