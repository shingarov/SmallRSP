Class {
	#name : #MicrowattGDB,
	#superclass : #Object,
	#instVars : [
		'jtag'
	],
	#category : #'Microwatt-GDB'
}

{ #category : #'as yet unclassified' }
MicrowattGDB >> describeTargetTo: aULD [ 
	aULD assumeMicrowatt
]

{ #category : #initialization }
MicrowattGDB >> initialize [
	jtag := MicrowattJTAG uniqueInstance 
]
