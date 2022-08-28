Class {
	#name : #RemoteGdbXFER,
	#superclass : #RemoteGdbAbstractHelper,
	#category : #'GDB-RSP'
}

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> features: filename [
	"Answer the fully-initialized (i.e. regsInGPacket in place, etc) PDL."
	| root |
	root := self readFeatureXML: filename.
	root isNil ifTrue: [ ^nil ].
	^self processTargetElement: root
]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> pdlForTarget: targetElement [
	| arch |
	arch := (targetElement findXPath: 'architecture') contentText.
	^AcProcessorDescriptions gdbArchName: arch
]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> processTargetElement: target [
	| pdl |
	pdl := self pdlForTarget: target.
	pdl regsInGPacket: (self regTransfersFrom: target).
	^pdl

]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> readAnnexResponse: filename from: offset [
	| offsetHex |
	offsetHex := offset printStringHex.
	^rsp q: 'qXfer:features:read:', filename, ':', offsetHex, ',4000'
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
	gdbXmlParser := GdbXmlParser baseURI: (RemoteGdbFeatureURIProvider rsp: rsp).
	^gdbXmlParser parseString: xmlString
]

{ #category : #'as yet unclassified' }
RemoteGdbXFER >> readThreads [
	| answer threads |
	answer := gdb q: 'qXfer:threads:read::0,1000'.
	(answer beginsWith: 'l') ifFalse: [ self shouldBeImplemented "multipacket???" ].
	answer := answer allButFirst.

	
	threads := GdbXmlParser new parseString: answer.
	threads name localName = 'threads' ifFalse: [ self error ].
	
	threads := threads findXPath: 'thread'.
	threads isCollection ifFalse: [ threads := Array with: threads ].
	threads := threads collect: [ :th | GdbThread
		halt.
		Integer readFrom: (th attributeAt: 'id') base: 16
		
		 ].
	
	self halt.
	answer := '<threads>
<thread id="4a0" core="3" name="modtalk.elf" />
<thread id="4ab" core="1" name="modtalk.elf" />
</threads>'.

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
