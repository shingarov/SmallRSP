Class {
	#name : #DebugStoppedTestCase,
	#superclass : #TestCase,
	#category : #'GDB-Tests'
}

{ #category : #'tests-basic' }
DebugStoppedTestCase >> testCreateSEGV [
	| sig |
	sig := DebugStopped onSignalNum: 11.
	self assert: sig signal equals: #SIGSEGV
]

{ #category : #'tests-basic' }
DebugStoppedTestCase >> testCreateSYS [
	| sig |
	sig := DebugStopped onSignalNum: 31.
	self assert: sig signal equals: #SIGSYS
]

{ #category : #'tests-basic' }
DebugStoppedTestCase >> testCreateTRAP [
	| sig |
	sig := DebugStopped onSignalNum: 5.
	self assert: sig signal equals: #SIGTRAP
]
