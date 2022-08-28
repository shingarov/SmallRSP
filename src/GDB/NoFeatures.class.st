"
The RSP stub provided no Features XML
(for example, we asked it for qSupported and qXfer:features isn't there).

With targets for which the RSP stub does provide the Features, we use them
to figure out which ISA it is, what are the registers in the G packet, etc.
If we caught NoFeatures exception, it means we need to resort to some other
means to determine all that.
"
Class {
	#name : #NoFeatures,
	#superclass : #Exception,
	#instVars : [
		'gdb'
	],
	#category : #'GDB-RSP'
}

{ #category : #signalling }
NoFeatures class >> signalFor: aGDB [
	| ex |
	ex := self new.
	ex gdb: aGDB.
	ex signal
]

{ #category : #accessing }
NoFeatures >> gdb [
	^ gdb
]

{ #category : #accessing }
NoFeatures >> gdb: anObject [
	gdb := anObject
]
