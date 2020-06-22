Class {
	#name : #P1025ThinshellDoodle,
	#superclass : #PPCThinshellDoodle,
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
P1025ThinshellDoodle >> host [
	^#svost
]

{ #category : #tests }
P1025ThinshellDoodle >> testStepThoughPreamble [
	"
	P1025ThinshellDoodle new testStepThoughPreamble
	"
	self connectGdb; stepThroughThinshellPreamble.
	[ gdb kill ] on: GdbChildExited do: [ ^self ].
	"should be unreachable"
	self error
]

{ #category : #tests }
P1025ThinshellDoodle >> testSurgicalJump [
	"
	P1025ThinshellDoodle new testSurgicalJump
	"
	self connectGdb; stepThroughThinshellPreamble.

	gdb setRegister: 'r1' to: 240.
	gdb setRegister: 'pc' to: 16r100000cc.
	[ gdb c ] on: InferiorExited do: [ :ex |
		"We expect gdbserver to say, Child exited with status 15"
		self assert: ex exitCode equals: 15.
		"The RSP protocol spec doesn't say what is allowed here,
		because targets can vary.
		For example, attempting to terminate the gdbserver by sending KILL,
		will not work with the normal GNU gdbserver.
		However, simply closing the connection will suffice."
		^gdb socket close
		].
	
	"shouldn't reach here"
	self error
]
