Class {
	#name : #MicrowattGDB,
	#superclass : #GDB,
	#instVars : [
		'jtag'
	],
	#category : #'Microwatt-GDB'
}

{ #category : #'as yet unclassified' }
MicrowattGDB >> cont [
	jtag core_start
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> getRegister: r [
	r = 'pc' ifTrue: [ ^jtag core_nia ].
	r = 'msr' ifTrue: [ ^jtag core_msr ].
	^jtag readRegister: r
]

{ #category : #initialization }
MicrowattGDB >> initialize [
	jtag := MicrowattJTAG instance 
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> isRunning [
	^jtag core_status isEmpty
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> isStopped [
	^jtag core_status includes: #STOPPED
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> readRegistersFromTarget [
	self shouldBeImplemented.
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> reset [
	jtag core_reset
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> stepi [
	jtag core_step
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> stop [
	jtag core_stop
]

{ #category : #writing }
MicrowattGDB >> writeInt32: x toAddr: addr [
	jtag memAt: addr  put32: x
]
