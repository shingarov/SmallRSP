Class {
	#name : #ARM32GDBTest,
	#superclass : #SimpleRspTest,
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
ARM32GDBTest >> debuggerClass [
	^RemoteGDBSession
]

{ #category : #'target connection' }
ARM32GDBTest >> hostIP [
	^'z7'
]

{ #category : #tests }
ARM32GDBTest >> testConnect [
	| initialPC nextPC |
	self connectGdb; createRAM.
	initialPC := gdb pc.
	gdb s.
	nextPC := gdb pc
]
