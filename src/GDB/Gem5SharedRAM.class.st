Class {
	#name : #Gem5SharedRAM,
	#superclass : #SharedRAM,
	#instVars : [
		'tlb'
	],
	#category : #'GDB-RSP'
}

{ #category : #'address translation' }
Gem5SharedRAM >> lowestMappedAddress [
	^16r20000
]

{ #category : #'address translation' }
Gem5SharedRAM >> tlb [
	tlb isNil ifTrue: [ tlb := self getTLB ].
	^tlb
]

{ #category : #'address translation' }
Gem5SharedRAM >> translateViaTLB: anAddress [
	" Answer the physical address for the given virtual address,
	 if it is mapped, nil otherwise. "
	| pageVirt pagePhys |
	pageVirt := anAddress bitAnd: self class pageMask.
	pagePhys := self tlb at: pageVirt ifAbsent: [ ^nil ].
	^pagePhys bitOr: (anAddress bitAnd: self class offsetMask)
]

{ #category : #'address translation' }
Gem5SharedRAM >> virt2phys: anAddress [
	^self translateViaTLB: anAddress.
	"TODO: Investigate why the mapping is now more complex
	than what we had with the 2nd-hackathon's version of gem5"
	"^anAddress - self lowestMappedAddress"

]
