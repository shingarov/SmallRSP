"
I am the abstract superclass of all GDB tests requiring a connection to a target,
both those going over RSP and those not going over RSP.
Subclasses must implement #connectGDB which will create some sort of gdb during setUp.

"
Class {
	#name : #GDBTest,
	#superclass : #TestCase,
	#instVars : [
		'gdb'
	],
	#category : #'GDB-tests-Connected'
}

{ #category : #running }
GDBTest >> adjustProcessorDescription [
	"Nothing by default, as the PDL coming out of parsing is usually good."
]

{ #category : #running }
GDBTest >> checkProcessorDescription [
	"Some tests may want to assert that the PDL has been determined correctly."
]

{ #category : #running }
GDBTest >> connectGDB [
	self subclassResponsibility
]

{ #category : #running }
GDBTest >> disconnectGDB [
	self subclassResponsibility
]

{ #category : #running }
GDBTest >> setUp [
	[ self connectGDB; adjustProcessorDescription ]
		on: NoFeatures
		do: [ :ex | ex gdb processorDescription: self processorDescription. ex resume ].
	self checkProcessorDescription
]

{ #category : #running }
GDBTest >> tearDown [
	self disconnectGDB
]
