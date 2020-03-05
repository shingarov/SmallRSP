Class {
	#name : #RemoteGdbAbstractHelper,
	#superclass : #Object,
	#instVars : [
		'gdb'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
RemoteGdbAbstractHelper class >> gdb: aRemoteGDB [
	^self new
		gdb: aRemoteGDB ;
		yourself 
]

{ #category : #accessing }
RemoteGdbAbstractHelper >> gdb [
	^ gdb
]

{ #category : #accessing }
RemoteGdbAbstractHelper >> gdb: anObject [
	gdb := anObject
]
