Class {
	#name : #PPCThinshellTestCase,
	#superclass : #TestCase,
	#instVars : [
		'gdb',
		'initialPC',
		'nZone'
	],
	#category : #'GDB-Tests'
}

{ #category : #'target connection' }
PPCThinshellTestCase >> connectGdbWith: processorDescription [
	gdb := RemoteGDBSession
		host: self hostIP
		port: 7000
		processorDescription: processorDescription.
	gdb memory: (RemoteRAM gdb: gdb)
]

{ #category : #tests }
PPCThinshellTestCase >> fillNZone [
	"Fill the nZone with (programmer-assembled) machine code
	that will return the SmallInteger 42.
	The Thinshell will convert the TAM SmallInteger in #R
	(which is where the Program Initializer is expected to put it)
	into a machine integer suitable for passing to the exit() syscall."

	gdb memory writeInt32: 16r382002a0 toAddr: nZone+0. "li r1,672"
	gdb memory writeInt32: 16r4e800020 toAddr: nZone+4. "blr"

]

{ #category : #'target connection' }
PPCThinshellTestCase >> hostIP [
	^'192.168.75.2'
]

{ #category : #stepping }
PPCThinshellTestCase >> stepThroughThinshellPreamble [
	initialPC := gdb pc.
	self assert: initialPC equals: 16r10000080.
	   "...that's what the ppc binary has, but if this assert is omitted the next ones should be ok"
	self assert: gdb currentInstructionEncoding equals: 16r7C000378. "mr r0,r0"
	gdb s.
	self assert: gdb pc equals: initialPC + 4.
	self assert: gdb currentInstructionEncoding equals: 16r3E001001. "lisr16,4097"
	gdb s.
	self assert: gdb pc equals: initialPC + 8.
	gdb s; s; s; s; s. "last non-nop"
	nZone := gdb getRegisters at: 'r16'.
	self assert: nZone equals: 16r10011000.
	gdb s; s; s; s; s; s. "bunch of nops (there are more!)"
	self assert: gdb pc equals: 16r100000B4
]

{ #category : #tests }
PPCThinshellTestCase >> testManualNZone [
	self connectGdbWith: FakeProcessorDescriptionPPC new.
	self stepThroughThinshellPreamble.
	self fillNZone.
	self should: [ gdb c ] raise: ConnectionClosed
	"At this point we expect gem5 to have exited and said,
	Exit code is 42"
]
