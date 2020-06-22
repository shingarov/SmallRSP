"
See thinshell/absolute/power/absolute.s.
This gets built into an exe loaded at absolute address 0.
Execution starts at 0.
gem5 is able to simulate an MMU that can map such address.

"
Class {
	#name : #AbsoluteZeroPPC,
	#superclass : #GDBDoodle,
	#instVars : [
		'memory'
	],
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
AbsoluteZeroPPC >> connectGdb [
	super connectGdb.
	memory := RemoteRAM gdb: gdb
]

{ #category : #'target connection' }
AbsoluteZeroPPC >> hostIP [
	^'192.168.75.2'
]

{ #category : #'stepping logic' }
AbsoluteZeroPPC >> makeAFewSteps [
	self assert: gdb pc equals: 0.
	self assert: memory currentInstructionEncoding equals: 16r7c631a78.
	gdb s.
	self assert: gdb pc equals: 4.
	self assert: memory currentInstructionEncoding equals: 16r3860002a.
	gdb s.
	self assert: gdb pc equals: 8.

]

{ #category : #'target connection' }
AbsoluteZeroPPC >> pdl [
	^FakeProcessorDescriptionPPC new
]

{ #category : #'target connection' }
AbsoluteZeroPPC >> testAbs [
	"
	AbsoluteZeroPPC  new halt ;  testAbs
	"
	self connectGdb; 	makeAFewSteps.
	[ "then, run at full speed until the exit() syscall"
	gdb c
		"At this point we expect gem5 to have exited and said,
		Exit code is 42"
	] on: GdbChildExited do: [ ^self ].
	self error
]
