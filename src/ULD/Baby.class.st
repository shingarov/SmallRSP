Class {
	#name : #Baby,
	#superclass : #Object,
	#instVars : [
		'gdb'
	],
	#category : #ULD
}

{ #category : #'as yet unclassified' }
Baby >> createMicrowattGDB [
	gdb := MicrowattGDB new
]

{ #category : #accessing }
Baby >> gdb [
	^ gdb
]

{ #category : #accessing }
Baby >> gdb: anObject [
	gdb := anObject
]
