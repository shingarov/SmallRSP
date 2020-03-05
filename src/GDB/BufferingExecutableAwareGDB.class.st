Class {
	#name : #BufferingExecutableAwareGDB,
	#superclass : #ExecutableAwareGDB,
	#instVars : [
		'buffer',
		'minAddr',
		'leftFinger',
		'queuedBreakpoints'
	],
	#category : #'GDB-RSP'
}

{ #category : #accessing }
BufferingExecutableAwareGDB >> buffer [
	buffer isNil ifTrue: [ buffer := ByteArray new: (4*1024*1024) + (1024*1024) + (80*1024*1024) ].
	^buffer
]

{ #category : #flushing }
BufferingExecutableAwareGDB >> chunkSize [
	"In bytes"
	^2048 "maybe? this was always 4096, I have no idea why this started breaking in Maribor"
]

{ #category : #flushing }
BufferingExecutableAwareGDB >> flushRAM [
	| firstUnallocatedAddress |
	firstUnallocatedAddress := self getVRegister: #HEAP.
	leftFinger := 1.
	'Injecting program'
		displayProgressFrom: minAddr to: firstUnallocatedAddress during:
			[ :bar | 	self transmitBelow: firstUnallocatedAddress updating: bar ]

]

{ #category : #API }
BufferingExecutableAwareGDB >> insertSWBreakpointAt: addr [
	self queuedBreakpoints add: addr
 		
 

]

{ #category : #accessing }
BufferingExecutableAwareGDB >> minAddr [
	minAddr isNil ifTrue: [ minAddr := self getVRegister: #NZone ].
	^minAddr
]

{ #category : #accessing }
BufferingExecutableAwareGDB >> queuedBreakpoints [
	queuedBreakpoints isNil ifTrue: [ queuedBreakpoints := OrderedCollection new ].
	^queuedBreakpoints
]

{ #category : #API }
BufferingExecutableAwareGDB >> read32At: addr [
	leftFinger isNil ifTrue: [
		^(((self buffer at: addr - self minAddr + 1) << 24
		bitOr: (self buffer at: addr - self minAddr + 2) << 16)
		bitOr: (self buffer at: addr - self minAddr + 3) << 8)
		bitOr: (self buffer at: addr - self minAddr + 4)
		 ].
	^super read32At: addr
]

{ #category : #flushing }
BufferingExecutableAwareGDB >> transmitBelow: firstUnallocatedAddress updating: aProgressBar [
| bytes thisChunk |
	bytes := firstUnallocatedAddress - self minAddr.
	bytes = 0 ifTrue: [ buffer := nil. ^self ].
	bytes > self chunkSize ifTrue: [ bytes := self chunkSize ].

thisChunk := buffer copyFrom: leftFinger to: leftFinger + bytes - 1.
aProgressBar value: minAddr.
self writeBytes: thisChunk toAddr: minAddr.
minAddr := minAddr + bytes.
leftFinger := leftFinger + bytes.
self transmitBelow: firstUnallocatedAddress updating: aProgressBar 
]

{ #category : #API }
BufferingExecutableAwareGDB >> writeBytes: aByteArray toAddr: addr [
	| start stop |
	leftFinger isNil ifTrue: [
		start := addr - self minAddr + 1.
		stop := start + aByteArray size - 1.
		^self buffer replaceFrom: start to: stop with: aByteArray startingAt: 1 ].
	^super writeBytes: aByteArray toAddr: addr
]

{ #category : #API }
BufferingExecutableAwareGDB >> writeInt32: int toAddr: addr [
	leftFinger isNil ifTrue: [
		^self buffer
			at: addr - self minAddr + 1 put: ((int bitAnd: 16rFF000000) >> 24);
			at: addr - self minAddr + 2 put: ((int bitAnd: 16r00FF0000) >> 16);
			at: addr - self minAddr + 3 put: ((int bitAnd: 16r0000FF00) >> 8);
			at: addr - self minAddr + 4 put: (int bitAnd: 16r000000FF) ].
	super writeInt32: int toAddr: addr

]
