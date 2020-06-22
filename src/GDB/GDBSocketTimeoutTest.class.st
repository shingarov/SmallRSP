Class {
	#name : #GDBSocketTimeoutTest,
	#superclass : #TestCase,
	#category : #'GDB-Tests'
}

{ #category : #'as yet unclassified' }
GDBSocketTimeoutTest >> connectGdb [
	^self debuggerClass
		host: self hostIP
		port: 7000
		processorDescription: FakeProcessorDescriptionPPC new
]

{ #category : #'as yet unclassified' }
GDBSocketTimeoutTest >> hostIP [
	^'192.168.75.2'
]

{ #category : #'as yet unclassified' }
GDBSocketTimeoutTest >> testBad [
	| gdb |
	gdb := self connectGdb.
	gdb halt.
	self should: [ gdb c ] raise: ConnectionClosed

]

{ #category : #'as yet unclassified' }
GDBSocketTimeoutTest >> testGood [
	| gdb |
	gdb := self connectGdb.
	gdb inspect
]
