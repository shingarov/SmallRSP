Class {
	#name : #RemoteGdbXFER,
	#superclass : #RemoteGdbAbstractHelper,
	#category : #'GDB-RSP'
}

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> features: filename [
	| answer xml |
	answer := gdb q: 'qXfer:features:read:', filename, ':0,fff'.
	answer isEmpty ifTrue: [ ^nil ]. " stub has indicated he does not understand this request "
	answer first == $l ifFalse: [ ^self error ].
	answer := answer allButFirst.
	xml := PPXmlParser parse: answer.
	xml baseURI: (RemoteGdbFeatureURIProvider gdb: gdb).
	^xml rootElement copy
]
