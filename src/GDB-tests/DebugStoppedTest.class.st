"
These tests assert simple facts about DebugStopped as an isolated object;
none of these tests require any connection to a target.
"
Class {
	#name : #DebugStoppedTest,
	#superclass : #TestCase,
	#category : #'GDB-tests-Unconnected'
}

{ #category : #'tests-basic' }
DebugStoppedTest >> testCreateSEGV [
	| sig |
	sig := DebugStopped onSignalNum: 11.
	self assert: sig signal equals: #SIGSEGV
]

{ #category : #'tests-basic' }
DebugStoppedTest >> testCreateSYS [
	| sig |
	sig := DebugStopped onSignalNum: 31.
	self assert: sig signal equals: #SIGSYS
]

{ #category : #'tests-basic' }
DebugStoppedTest >> testCreateTRAP [
	| sig |
	sig := DebugStopped onSignalNum: 5.
	self assert: sig signal equals: #SIGTRAP
]
