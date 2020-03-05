Class {
	#name : #RemoteGdbFeatureURIProvider,
	#superclass : #Object,
	#instVars : [
		'gdb'
	],
	#category : #'GDB-RSP'
}

{ #category : #'as yet unclassified' }
RemoteGdbFeatureURIProvider class >> gdb: aGDB [
	^self new
		gdb: aGDB;
		yourself 
]

{ #category : #accessing }
RemoteGdbFeatureURIProvider >> gdb [
	^ gdb
]

{ #category : #accessing }
RemoteGdbFeatureURIProvider >> gdb: anObject [
	gdb := anObject
]

{ #category : #API }
RemoteGdbFeatureURIProvider >> get: filename [
	^gdb qXfer features: filename
]
