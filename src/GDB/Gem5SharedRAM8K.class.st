Class {
	#name : #Gem5SharedRAM8K,
	#superclass : #Gem5SharedRAM,
	#category : #'GDB-RSP'
}

{ #category : #granularity }
Gem5SharedRAM8K class >> mmuPageSize [
	"GEM5 MIPS.  Sweetman claims this should not happen."
	^8192
]
