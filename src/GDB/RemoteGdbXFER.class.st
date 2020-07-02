Class {
	#name : #RemoteGdbXFER,
	#superclass : #RemoteGdbAbstractHelper,
	#category : #'GDB-RSP'
}

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> features: filename [
	| root |
	root := self readFeatureXML: filename.
	root isNil ifTrue: [ ^nil ].
	^self processTargetElement: root
]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> pdlForTarget: targetElement [
	| arch pdl |
	arch := (targetElement findXPath: 'architecture') contentText.
	pdl := AcProcessorDescriptions gdbArchName: arch.
	gdb processorDescription: pdl
]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> processTargetElement: target [
	| registerArray oneRegTransfers regNum |
	self pdlForTarget: target.
	^self regTransfersFrom: target

]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> readAnnexResponse: filename from: offset [
	| offsetHex |
self halt.
	offsetHex := offset printStringHex.
	^gdb q: 'qXfer:features:read:', filename, ':', offsetHex, ',4000'
]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> readFeatureAnnex: filename from: offset [
	| answer |
	answer := self readAnnexResponse: filename from: offset.
	answer isEmpty ifTrue: [ ^nil ]. " stub has indicated he does not understand this request "
	answer first == $m ifTrue: [ | thisFragment rest |
		thisFragment := answer allButFirst.
		rest := self readFeatureAnnex: filename from: offset + thisFragment size.
		^thisFragment, rest ].
	answer first == $l ifFalse: [ ^self error ].
	^answer allButFirst.

]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> readFeatureFile: filename from: offset [
	| answer |
	answer := self readAnnexResponse: filename from: offset.
	answer isEmpty ifTrue: [ ^nil ]. " stub has indicated he does not understand this request "
	answer first == $m ifTrue: [ ^self shouldBeImplemented ].
	answer first == $l ifFalse: [ ^self error ].
	^answer allButFirst.

]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> readFeatureXML: filename [
	| gdbXmlParser xmlString |
	xmlString := self readFeatureAnnex: filename from: 0.
	gdbXmlParser := GdbXmlParser baseURI: (RemoteGdbFeatureURIProvider gdb: gdb).
	^gdbXmlParser parseString: xmlString
]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> regTransfersFrom: target [
	| registerArray oneRegTransfers regNum |
	registerArray := target findXPath: 'feature/reg'.
	registerArray isNil ifTrue: [ self error ].
	regNum := 0.
	oneRegTransfers := registerArray collect: [ :regNode |
		| rt |
		regNum := (regNode attributeAt: 'regnum' ifAbsent: [ regNum ]) asInteger.
		rt := RSPOneRegisterTransfer new
			regName: (regNode attributeAt: 'name');
			width: ((regNode attributeAt: 'bitsize') asInteger);
			regNum: regNum;
			yourself.
		regNum := regNum + 1.
		rt ].
	oneRegTransfers := oneRegTransfers sorted: [ :rA :rB | rA regNum <= rB regNum ].

	"Verify that we got all registers from 0 to the total number, without holes."
	"Actually, the e500v2 violates this: there is no regnum=70."
	"regNum := 0.
	oneRegTransfers do: [ :rt |
		rt regNum == regNum ifFalse: [ self error ].
		regNum := regNum + 1 ]."
	^oneRegTransfers
]
