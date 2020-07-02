Class {
	#name : #AMD64GDBTest,
	#superclass : #TestCase,
	#instVars : [
		'gdb',
		'memory'
	],
	#category : #'GDB-Tests'
}

{ #category : #'target connection' }
AMD64GDBTest >> connectGdb [
	gdb := self debuggerClass
		host: self hostIP
		port: self tcpPort.
	^gdb
]

{ #category : #'target connection' }
AMD64GDBTest >> createRAM [
	memory := self remoteMemoryClass gdb: gdb
]

{ #category : #'target connection' }
AMD64GDBTest >> debuggerClass [
	^RemoteGDBSession
]

{ #category : #'target connection' }
AMD64GDBTest >> hostIP [
	^'parmenides'
]

{ #category : #'target connection' }
AMD64GDBTest >> remoteMemoryClass [
	^RemoteRAM
]

{ #category : #'target connection' }
AMD64GDBTest >> tcpPort [
	^7000
]

{ #category : #tests }
AMD64GDBTest >> testConnect [
	| initialPC nextPC |
	self connectGdb.
	self createRAM.
	initialPC := gdb pc.
	gdb s.
	nextPC := gdb pc
]
