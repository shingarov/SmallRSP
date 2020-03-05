"
MMUs on some processors, such as the MIPS32 implementation found in the JZ4780 chip on the Creator CI20 board, have 8K page granularity.
"
Class {
	#name : #NBExecutableAwareGDB8K,
	#superclass : #NBExecutableAwareGDB,
	#category : #'GDB-RSP'
}

{ #category : #granularity }
NBExecutableAwareGDB8K class >> mmuPageSize [
	"GEM5 MIPS.  Sweetman claims that this should not happen."
	^8192
]
