Class {
	#name : #SimpleSharedRAM,
	#superclass : #SharedRAM,
	#category : #'GDB-RSP'
}

{ #category : #'as yet unclassified' }
SimpleSharedRAM >> virt2phys: anAddress [
	^anAddress - 16r20000
]
