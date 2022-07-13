Class {
	#name : #SimpleRspTest,
	#superclass : #TestCaseWithZ3Context,
	#instVars : [
		'gdb',
		'memory'
	],
	#category : #'GDB-Tests'
}

{ #category : #tests }
SimpleRspTest >> advancePC [
	| initialPC nextPC |
	initialPC := gdb pc.
	gdb s.
	nextPC := gdb pc.
	self assert: nextPC equals: initialPC+4
]

{ #category : #connecting }
SimpleRspTest >> connectGdb [
	gdb := self debuggerClass
		host: self hostIP
		port: self tcpPort.
	^gdb
]

{ #category : #connecting }
SimpleRspTest >> createRAM [
	memory := self remoteMemoryClass gdb: gdb
]

{ #category : #connecting }
SimpleRspTest >> debuggerClass [
	^RemoteGDBSession
]

{ #category : #'target IP' }
SimpleRspTest >> hostIP [
	self subclassResponsibility
]

{ #category : #connecting }
SimpleRspTest >> remoteMemoryClass [
	^RemoteRAM
]

{ #category : #'target IP' }
SimpleRspTest >> tcpPort [
	^7000
]
