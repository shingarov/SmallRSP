Class {
	#name : #ARM32GDBTest,
	#superclass : #TestCase,
	#instVars : [
		'gdb',
		'memory'
	],
	#category : #'GDB-Tests'
}

{ #category : #'target connection' }
ARM32GDBTest >> connectGdb [
	gdb := self debuggerClass
		host: self hostIP
		port: self tcpPort.
	^gdb
]

{ #category : #'target connection' }
ARM32GDBTest >> createRAM [
	memory := self remoteMemoryClass gdb: gdb
]

{ #category : #'target connection' }
ARM32GDBTest >> debuggerClass [
	^RemoteGDBSession
]

{ #category : #'target connection' }
ARM32GDBTest >> hostIP [
	^'z7'
]

{ #category : #'target connection' }
ARM32GDBTest >> remoteMemoryClass [
	^RemoteRAM
]

{ #category : #'target connection' }
ARM32GDBTest >> tcpPort [
	^7000
]

{ #category : #'target connection' }
ARM32GDBTest >> testConnect [
	| initialPC firstInstruction firstInstrAsm |
	self connectGdb.
	self createRAM.
	initialPC := gdb pc.
	firstInstruction := memory currentInstruction.
	firstInstrAsm := firstInstruction disassemble
]
