Class {
	#name : #MicrowattGDBTest,
	#superclass : #TestCase,
	#instVars : [
		'gdb',
		'uld'
	],
	#category : #'Microwatt-Tests'
}

{ #category : #tests }
MicrowattGDBTest >> test1 [ 
	gdb := MicrowattGDB new.
	uld := ULD on: gdb.
	

]
