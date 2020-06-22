Class {
	#name : #ThinshellDoodle,
	#superclass : #GDBDoodle,
	#instVars : [
		'memory'
	],
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
ThinshellDoodle >> connectGdb [
	super connectGdb.
	self createRAM
]

{ #category : #'target connection' }
ThinshellDoodle >> createRAM [
	memory := self remoteMemoryClass gdb: gdb
]

{ #category : #'stepping logic' }
ThinshellDoodle >> fillNZone [
	"Fill the nZone with (programmer-assembled) machine code
	that will return the SmallInteger 42.
	The Thinshell will convert the TAM SmallInteger in #R
	(which is where the Program Initializer is expected to put it)
	into a machine integer suitable for passing to the exit() syscall."

	memory writeInt32: 16r382002a0 toAddr: self nZone+0. "li r1,672"
	memory writeInt32: 16r4e800020 toAddr: self nZone+4. "blr"

]

{ #category : #'stepping logic' }
ThinshellDoodle >> fillNZone2 [
	"Same as fillNZone, but in one shot."

	memory writeInt32s: #(16r382002a0 16r4e800020) toAddr: self nZone+0
]

{ #category : #'facts about thinshell' }
ThinshellDoodle >> initialPC [
	^16r10000080

]

{ #category : #'facts about thinshell' }
ThinshellDoodle >> nZone [
	^16r100100E0
]

{ #category : #'target connection' }
ThinshellDoodle >> remoteMemoryClass [
	^RemoteRAM
]

{ #category : #'stepping logic' }
ThinshellDoodle >> stepThroughThinshellPreamble [
	| initialPC nZone |
	initialPC := gdb pc.
	self assert: initialPC equals: 16r10000080.
	self assert: memory currentInstructionEncoding equals: 16r7C000378. "mr r0,r0"
	gdb s.
	self assert: gdb pc equals: initialPC + 4.
	self assert: memory currentInstructionEncoding equals: 16r3E001001. "lis r16,4097"
	gdb s.
	self assert: gdb pc equals: initialPC + 8.
	gdb s; s; s; s; s. "last non-nop"
	nZone := gdb getRegisters at: 'r16'.
	self assert: nZone equals: self nZone.
	gdb s; s; s; s; s; s. "bunch of nops (there are more!)"
	self assert: gdb pc equals: 16r100000B4
]
