"
A MicrowattJTAGTest is a test class for testing the behavior of MicrowattJTAG
"
Class {
	#name : #MicrowattJTAGTest,
	#superclass : #TestCase,
	#category : #'Microwatt-Tests'
}

{ #category : #tests }
MicrowattJTAGTest >> test1 [
	MicrowattJTAG uniqueInstance core_nia.
	MicrowattJTAG uniqueInstance core_step.
	MicrowattJTAG uniqueInstance core_stop.

	MicrowattJTAG uniqueInstance core_reset.
	
	MicrowattJTAG uniqueInstance core_start.

]
