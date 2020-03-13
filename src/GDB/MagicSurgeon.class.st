"
Execution of the inner (target) Smalltalk VM stopped at a surgery point.
"
Class {
	#name : #MagicSurgeon,
	#superclass : #Object,
	#instVars : [
		'gdb'
	],
	#category : #'GDB-RSP'
}

{ #category : #accessing }
MagicSurgeon >> gdb [
	^ gdb
]

{ #category : #accessing }
MagicSurgeon >> gdb: anObject [
	gdb := anObject
]

{ #category : #API }
MagicSurgeon >> process [ 
	self subclassResponsibility 
]
