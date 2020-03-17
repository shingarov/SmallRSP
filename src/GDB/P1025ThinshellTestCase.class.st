Class {
	#name : #P1025ThinshellTestCase,
	#superclass : #PPCThinshellTestCase,
	#category : #'GDB-Tests'
}

{ #category : #'target connection' }
P1025ThinshellTestCase >> hostIP [
	^'192.168.75.199'
]

{ #category : #tests }
P1025ThinshellTestCase >> testStepThoughPreamble [
	self connectGdbWith: FakeProcessorDescriptionP1025 new.
	self stepThroughThinshellPreamble.
	gdb kill
]

{ #category : #tests }
P1025ThinshellTestCase >> testSurgicalJump [
	self connectGdbWith: FakeProcessorDescriptionP1025 new.
	self stepThroughThinshellPreamble.

	gdb setRegister: 'r1' to: 240.
	gdb setRegister: 'pc' to: 16r100000cc.
	[ gdb c ] on: InferiorExited do: [ :ex | ^self assert: ex exitCode equals: 15 ].
	"Ee expect gdbserver to say, Child exited with status 15"
	
	"shouldn't reach here"
	self error
]
