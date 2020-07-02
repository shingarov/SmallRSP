Class {
	#name : #GDBDoodle,
	#superclass : #Doodle,
	#instVars : [
		'gdb'
	],
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
GDBDoodle >> connectGdb [
	gdb := self debuggerClass
		host: self hostIP
		port: self tcpPort.
	^gdb
]

{ #category : #'target connection' }
GDBDoodle >> debuggerClass [
	^RemoteGDBSession
]

{ #category : #'target connection' }
GDBDoodle >> hostIP [
	self shouldBeImplemented
]

{ #category : #'target connection' }
GDBDoodle >> pdl [
	self shouldBeImplemented
]

{ #category : #'target connection' }
GDBDoodle >> tcpPort [
	^7000
]
