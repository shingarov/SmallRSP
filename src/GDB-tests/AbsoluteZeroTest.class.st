"
See thinshell/absolute/power/absolute.s.
This gets built into an exe loaded at absolute address 0.
Execution starts at 0.
gem5 is able to simulate an MMU that can map such address.

"
Class {
	#name : #AbsoluteZeroTest,
	#superclass : #RemoteGDBTest,
	#category : #'GDB-tests-Connected'
}

{ #category : #running }
AbsoluteZeroTest >> disconnectGDB [
self shouldBeImplemented 
]

{ #category : #running }
AbsoluteZeroTest >> gdbClass [
	^RemoteGDB
]

{ #category : #running }
AbsoluteZeroTest >> host [
	^'localhost'
]

{ #category : #running }
AbsoluteZeroTest >> tearDown [
	"The GdbChildExited is caught in the test,
	 so no need to do it here."
]

{ #category : #tests }
AbsoluteZeroTest >> test1 [
	"To execute this test, run
		make g5-absolute
	 in thinshell/absolute/power.
	 Because of the '-e 0 -Ttext 0' passed to ld,
	 text starts from absolute address 0."
	self assert: gdb pc equals: 0.
	"do a step"
	gdb s.
	self assert: gdb pc equals: 4.
	"continue until the inferior exits to the OS with 42"
	self should: [ gdb c ] raise: InferiorExited
]
