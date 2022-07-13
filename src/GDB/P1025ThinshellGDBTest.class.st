Class {
	#name : #P1025ThinshellGDBTest,
	#superclass : #SimpleRspTest,
	#category : #'GDB-Tests'
}

{ #category : #'target IP' }
P1025ThinshellGDBTest >> hostIP [
	^'192.168.75.199'
]

{ #category : #tests }
P1025ThinshellGDBTest >> testConnect [
	self connectGdb; createRAM; halt.
	gdb kill
]

{ #category : #tests }
P1025ThinshellGDBTest >> testExecuteExit [
	self connectGdb; createRAM.
	
	"_start is at 0CA20000"
	self assert: gdb pc equals: 16r0CA20000.
	
	"(At least in thinshell 126bf5d3151ac13fa8d7360b2dbc52a9c2b1b5ba),
	 _start() consists of the only instruction TRAP.
	 Immediately past it, there is exit() which syscalls returning 42.
	 Force-advance the PC beyond the trap and continue."
	gdb pc: gdb pc+4.
	self assert: gdb pc equals: 16r0CA20004.
	[gdb c] on: InferiorExited do: [ :ex | self assert: ex exitCode equals: 42. ^gdb disconnect ].
	self error: 'Process did not exit(42)'
]
