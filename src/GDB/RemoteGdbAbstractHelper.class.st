Class {
	#name : #RemoteGdbAbstractHelper,
	#superclass : #Object,
	#instVars : [
		'rsp'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
RemoteGdbAbstractHelper class >> gdb: aRemoteGDB [
	^self new
		gdb: aRemoteGDB ;
		yourself 
]

{ #category : #'instance creation' }
RemoteGdbAbstractHelper class >> rsp: anRSPClient [
	^self new
		rsp: anRSPClient;
		yourself 
]

{ #category : #accessing }
RemoteGdbAbstractHelper >> rsp [
	^ rsp
]

{ #category : #accessing }
RemoteGdbAbstractHelper >> rsp: anObject [
	rsp := anObject
]
