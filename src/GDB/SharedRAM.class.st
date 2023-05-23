Class {
	#name : #SharedRAM,
	#superclass : #RemoteRAM,
	#instVars : [
		'hostPtr',
		'shmemSize'
	],
	#category : #'GDB-RSP'
}

{ #category : #FFI }
SharedRAM class >> gem5: nBytes [
	"Answer the void* pointer to the backing store of the gem5 guest memory."
	| fd addr |
	fd := self shmOpen: '/gem5'
		with: 64"O_CREAT" | 2"O_RDWR"
		with: 8r666. "mode"
	addr := self mmap: 0
		with: nBytes
		with: 1"PROT_READ" | 2"PROT_WRITE"
		with: 1 "MAP_SHARED"
		with: fd
		with: 0.
	addr value = 16rFFFFFFFFFFFFFFFF ifTrue: [ self error ].
	^addr

"
Gem5SharedRAM halt; gem5: 5000
"

]

{ #category : #FFI }
SharedRAM class >> mmap: addr with: length with: prot with: flags with: fd with: offset [
	"<cdecl: 'void*' 'mmap' (longlong longlong long long long longlong) module: '/lib/x86_64-linux-gnu/libc.so.6'>"
	^LibC uniqueInstance mmap: addr with: length with: prot with: flags with: fd with: offset
]

{ #category : #granularity }
SharedRAM class >> mmuPageSize [
	^4096
]

{ #category : #granularity }
SharedRAM class >> offsetMask [
	^self mmuPageSize - 1
]

{ #category : #granularity }
SharedRAM class >> pageMask [
	^16rFFFFFFFF bitXor: self offsetMask

]

{ #category : #FFI }
SharedRAM class >> shmOpen: fileName with: oflag with: mode [
	^LibRT uniqueInstance shmOpen: fileName with: oflag with: mode
]

{ #category : #writing }
SharedRAM >> byteAtAddr: byteAddress put: byte [
	| ptr |
	ptr := self translate: byteAddress.
	ptr unsignedByteAt: 1 put: byte
]

{ #category : #writing }
SharedRAM >> fillFromStream: aFileStream startingAt: startAddress count: count [
	| contents |
	contents := aFileStream next: count.
	self writeBytes: contents toAddr: startAddress.
	^contents size
]

{ #category : #'address translation' }
SharedRAM >> forceRead32At: addr [
	| x |
	self halt. "I don't remember why this was needed."
	x := super read32At: addr.
	tlb := self getTLB.
	^x

]

{ #category : #'address translation' }
SharedRAM >> forceReadAt: addr nBytes: n [
	super readAt: addr nBytes: n.
	tlb := self getTLB
]

{ #category : #'address translation' }
SharedRAM >> forceWriteBytes: aByteArray  toAddr: addr [
	| x |
	x := super writeBytes: aByteArray  toAddr: addr.
	tlb := self getTLB.
	^x

]

{ #category : #'address translation' }
SharedRAM >> forceWriteInt32: int toAddr: addr [
	| x |
	x := super writeInt32: int toAddr: addr.
	tlb := self getTLB.
	^x

]

{ #category : #RSP }
SharedRAM >> getTLB [
	| answer |
	answer := self gdb q: '.'.
	answer isEmpty ifTrue: [ self error: 'GDB failed to return TLB' ].
	^Dictionary newFromAssociations:
		(((answer findTokens: ';')
		collect: [ :s | s findTokens: ':' ])
		collect: [ :pair | (Integer readFrom: pair first base: 16) -> (Integer readFrom: pair last base: 16) ])
]

{ #category : #shmem }
SharedRAM >> hostPtr [
	hostPtr isNil ifTrue: [ hostPtr := Gem5SharedRAM gem5: shmemSize ].
	^hostPtr
]

{ #category : #writing }
SharedRAM >> longAtAddr: addr put: aValue bigEndian: bigEndian [
	| ptr int |
	int := bigEndian
		ifTrue: [ aValue byteSwap32 ]
		ifFalse: [ aValue ].
	ptr := self translate: addr.
	ptr unsignedLongAt: 1 put: int.
	^aValue
	
	
	

]

{ #category : #reading }
SharedRAM >> read: n bytesAtAddr: addr [
	| backingAddr |
	backingAddr := self translate: addr.
	^((1 to: n) collect: [ :idx| backingAddr byteAt: idx ]) asByteArray

]

{ #category : #shmem }
SharedRAM >> shmemSize [
	^shmemSize
]

{ #category : #shmem }
SharedRAM >> shmemSize: howBig [
	shmemSize := howBig
]

{ #category : #'address translation' }
SharedRAM >> translate: addr [
	| pageAddr |
	pageAddr := self virt2phys: addr.
	pageAddr isNil ifTrue: [ ^ self error forceRead32At: addr ].
	^self hostPtr getHandle + pageAddr.

]

{ #category : #reading }
SharedRAM >> unsignedLongAtAddr: addr bigEndian: bigEndian [
	| backingAddr int |
	backingAddr := self translate: addr.
	int := backingAddr unsignedLongAt: 1.
	bigEndian ifTrue: [ int := int byteSwap32].
	^int
]

{ #category : #'address translation' }
SharedRAM >> virt2phys: anAddress [
	self subclassResponsibility
]

{ #category : #writing }
SharedRAM >> writeBytes: aByteArray toAddr: addr [
	| ptr |
addr = 16r109014 ifTrue: [self halt].
	ptr := self translate: addr.
	aByteArray doWithIndex: [ :x :idx | ptr unsignedByteAt: idx put: x ] "is there a faster way?"

]
