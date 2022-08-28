Class {
	#name : #RemoteGdbFeatureURIProvider,
	#superclass : #Object,
	#instVars : [
		'rsp'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
RemoteGdbFeatureURIProvider class >> rsp: anRSPClient [
	^self basicNew
		rsp: anRSPClient;
		yourself 
]

{ #category : #API }
RemoteGdbFeatureURIProvider >> get: filename [
	^rsp qXfer readFeatureXML: filename
]

{ #category : #accessing }
RemoteGdbFeatureURIProvider >> rsp: anObject [
	rsp := anObject
]
