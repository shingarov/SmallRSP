Class {
	#name : #GDBDoodle,
	#superclass : #TestCase,
	#instVars : [
		'gdb'
	],
	#category : #'GDB-tests'
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
	^RemoteGDB
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
