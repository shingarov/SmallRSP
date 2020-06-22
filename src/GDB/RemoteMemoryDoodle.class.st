Class {
	#name : #RemoteMemoryDoodle,
	#superclass : #PPCgem5ThinshellDoodle,
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
RemoteMemoryDoodle >> hostIP [
	^'192.168.75.2'
]

{ #category : #'target connection' }
RemoteMemoryDoodle >> pdl [
	^FakeProcessorDescriptionPPC new
]

{ #category : #'tests-reading' }
RemoteMemoryDoodle >> testLowLevelWrite [
	"
	RemoteMemoryDoodle new testLowLevelWrite
	"
	| goodAddress |
	self connectGdb.
	goodAddress := self initialPC.
	memory writeBytes: #[1 2 3 4] toAddr: goodAddress.
	
	self assert: (memory unsignedLongAtAddr: goodAddress bigEndian: true) equals: 16r01020304.
	self assert: (memory unsignedLongAt: goodAddress + 1 bigEndian: true) equals: 16r01020304.
	self assert: (memory read: 4 bytesAtAddr: goodAddress) equals: #[1 2 3 4].
	self assert: (memory unsignedByteAtAddr: goodAddress) equals: 1.
	self assert: (memory unsignedByteAtAddr: goodAddress+1) equals: 2.
	
	[ gdb kill ] on: GdbChildExited do: [ ^self ]
	
]

{ #category : #'tests-reading' }
RemoteMemoryDoodle >> testReadFirstInstruction [
	"
	RemoteMemoryDoodle new testReadFirstInstruction
	"
	| entry |
	self connectGdb.
	entry := self initialPC.
	
	self assert: (memory unsignedLongAtAddr: entry bigEndian: true) equals: 16r7c000378.
	self assert: (memory unsignedLongAtAddr: entry bigEndian: false) equals: 16r7803007c.
	self assert: (memory read: 4 bytesAtAddr: entry) equals: #[124 0 3 120].
	self assert: (memory unsignedByteAtAddr: 16r10000080) equals: 124.
	
	[ gdb kill ] on: GdbChildExited do: [ ^self ]

]
