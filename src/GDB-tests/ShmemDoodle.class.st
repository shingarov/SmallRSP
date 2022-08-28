Class {
	#name : #ShmemDoodle,
	#superclass : #RemoteMemoryDoodle,
	#instVars : [
		'plainRAM'
	],
	#category : #'GDB-tests'
}

{ #category : #'target connection' }
ShmemDoodle >> connectGdb [
	super connectGdb.
	memory shmemSize: 16r120000 + (120*1024*1024). "for Squeak"
	plainRAM := SimpleSharedRAM gdb: gdb.
	^gdb
]

{ #category : #'tests-agreement' }
ShmemDoodle >> doodles [

ShmemDoodle new testLowLevelWrite.

ShmemDoodle new testReadFirstInstruction.

ShmemDoodle new testManualNZone.

ShmemDoodle new testManualNZone2.


]

{ #category : #'target connection' }
ShmemDoodle >> remoteMemoryClass [
	^SimpleSharedRAM
]

{ #category : #'tests-agreement' }
ShmemDoodle >> testSharedToNonShared [
	self shouldBeImplemented
]

{ #category : #'tests-agreement' }
ShmemDoodle >> testSimpleSharedRAM [
"
ShmemDoodle new testSimpleSharedRAM.

"
	| rwAddress bytes |
	self break.
	self connectGdb.
	gdb q: 'c'.  "cant do real #c because we don't have a pdl with registers"
	rwAddress := 16r120000.
	"m shmemSize: 16r120000 + (120*1024*1024)."
	bytes := memory read: 2 bytesAtAddr: rwAddress.
	
	memory writeBytes: #[1 2 3 4] toAddr: rwAddress.
	
	self assert: (memory unsignedLongAtAddr: rwAddress bigEndian: true) equals: 16r01020304.
	self assert: (memory unsignedLongAt: rwAddress + 1 bigEndian: true) equals: 16r01020304.
	self assert: (memory read: 4 bytesAtAddr: rwAddress) equals: #[1 2 3 4].
	self assert: (memory unsignedByteAtAddr: rwAddress) equals: 1.
	self assert: (memory unsignedByteAtAddr: rwAddress+1) equals: 2.
	
	[ gdb kill ] on: InferiorExited do: [ ^self ]
	
]
