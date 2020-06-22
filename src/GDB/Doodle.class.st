Class {
	#name : #Doodle,
	#superclass : #Object,
	#category : #'GDB-Doodles'
}

{ #category : #'as yet unclassified' }
Doodle >> assert: expected equals: actual [
	^self
		assert: expected = actual

]
