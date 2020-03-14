Class {
	#name : #RemoteGdbXFER,
	#superclass : #RemoteGdbAbstractHelper,
	#category : #'GDB-RSP'
}

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> features: filename [
	| answer |
	answer := self readFeatureDocument: filename.
	answer isEmpty ifTrue: [ ^nil ]. " stub has indicated he does not understand this request "
	answer first == $l ifFalse: [ ^self error ].
	answer := answer allButFirst.
	^self parseFeatures: answer
]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> parseFeatures: aStringOfXML [
	self shouldBeImplemented 
	
	"The below code used to work,
	back when we had PPXmlParser.
	
	
	| xml gdbXmlParser |
	
	gdbXmlParser := GdbXmlParser endian: gdb processorDescription endian.
	gdbXmlParser parseString: aStringOfXML.
	self halt.
	xml baseURI: (RemoteGdbFeatureURIProvider gdb: gdb).
	^xml rootElement copy"
]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> readFeatureDocument: filename [
	^gdb q: 'qXfer:features:read:', filename, ':0,fff'
]
