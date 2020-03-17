Class {
	#name : #Gem5SharedRAM,
	#superclass : #RemoteRAM,
	#instVars : [
		'tlb'
	],
	#category : #'GDB-RSP'
}

{ #category : #granularity }
Gem5SharedRAM class >> mmuPageSize [
	^4096
]

{ #category : #granularity }
Gem5SharedRAM class >> offsetMask [
	^self mmuPageSize - 1
]

{ #category : #granularity }
Gem5SharedRAM class >> pageMask [
	^16rFFFFFFFF bitXor: self offsetMask

]

{ #category : #access }
Gem5SharedRAM >> forceRead32At: addr [
	| x |
	x := super read32At: addr.
	tlb := self getTLB.
	^x

]

{ #category : #'address translation' }
Gem5SharedRAM >> forceReadAt: addr nBytes: n [
	super readAt: addr nBytes: n.
	tlb := self getTLB
]

{ #category : #'address translation' }
Gem5SharedRAM >> forceWriteBytes: aByteArray  toAddr: addr [
	| x |
	x := super writeBytes: aByteArray  toAddr: addr.
	tlb := self getTLB.
	^x

]

{ #category : #'address translation' }
Gem5SharedRAM >> forceWriteInt32: int toAddr: addr [
	| x |
	x := super writeInt32: int toAddr: addr.
	tlb := self getTLB.
	^x

]

{ #category : #RSP }
Gem5SharedRAM >> getTLB [
	| answer |
	answer := self gdb q: '.'.
	answer isEmpty ifTrue: [ self error: 'GDB failed to return TLB' ].
	^Dictionary newFromAssociations:
		(((answer substrings: ';')
		collect: [ :s | s substrings: ':' ])
		collect: [ :pair | (Integer readFrom: pair first base: 16) -> (Integer readFrom: pair last base: 16) ])
]

{ #category : #shmem }
Gem5SharedRAM >> hostPtr [
	hostPtr isNil ifTrue: [ hostPtr := ShmemUFFI allocate: 536870912 ].
	^hostPtr 
]

{ #category : #access }
Gem5SharedRAM >> read32At: addr [
	| phys bytes |
	phys := self virt2phys: addr.
	phys isNil ifTrue: [ ^ self forceRead32At: addr ].
	bytes := self hostPtr ptr getHandle + phys unsignedLongAt: 1.
	processorDescription endian = #big ifTrue: [ bytes := bytes swapEndian].
	^bytes
]

{ #category : #'address translation' }
Gem5SharedRAM >> readAt: addr nBytes: n [
	| phys ptr |
	phys := self virt2phys: addr.
	phys isNil ifTrue: [ ^ self forceReadAt: addr nBytes: n ].
	ptr := self hostPtr ptr getHandle + phys.
	^ptr copyFrom: 1 to: n
]

{ #category : #'address translation' }
Gem5SharedRAM >> tlb [
	tlb isNil ifTrue: [ tlb := self getTLB ].
	^tlb
]

{ #category : #'address translation' }
Gem5SharedRAM >> virt2phys: anAddress [
	" Answer the physical address for the given virtual address,
	 if it is mapped, nil otherwise. "
	| pageVirt pagePhys |
	pageVirt := anAddress bitAnd: self class pageMask.
	pagePhys := self tlb at: pageVirt ifAbsent: [ ^nil ].
	^pagePhys bitOr: (anAddress bitAnd: self class offsetMask)
]

{ #category : #access }
Gem5SharedRAM >> writeBytes: aByteArray toAddr: addr [
	| phys ptr |
	phys := self virt2phys: addr.
	phys isNil ifTrue: [ ^ self forceWriteBytes: aByteArray  toAddr: addr ].
	ptr := self hostPtr ptr getHandle + phys.
	aByteArray doWithIndex: [ :x :idx | ptr unsignedByteAt: idx put: x ]

]

{ #category : #access }
Gem5SharedRAM >> writeInt32: int toAddr: addr [
	| phys |
	phys := self virt2phys: addr.
	phys isNil ifTrue: [ ^ self forceWriteInt32: int toAddr: addr ].
	processorDescription endian = #big ifTrue: [
		self hostPtr ptr getHandle + phys
			byteAt: 1 put: ((int bitAnd: 16rFF000000) >> 24);
			byteAt: 2 put: ((int bitAnd: 16r00FF0000) >> 16);
			byteAt: 3 put: ((int bitAnd: 16r0000FF00) >> 8);
			byteAt: 4 put:  (int bitAnd: 16r000000FF)
		] ifFalse: [
			self shouldBeImplemented 
		]

]
