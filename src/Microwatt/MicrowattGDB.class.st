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
	jtag := MicrowattJTAG instance.
	processorDescription := AcProcessorDescriptions powerpc.
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> isRunning [
	^jtag core_status isEmpty
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> isStopped [
	^jtag core_status includes: #STOPPED
]

{ #category : #memory }
MicrowattGDB >> read32At: addr [
	^jtag mem32At: addr
]

{ #category : #'as yet unclassified' }
MicrowattGDB >> readRegistersFromTarget [
	^Dictionary newFromAssociations: (jtag regNames collect: [ :r | r->(jtag readRegister: r) ])
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

{ #category : #memory }
MicrowattGDB >> writeInt32: x toAddr: addr [
	jtag memAt: addr  put32: x
]
