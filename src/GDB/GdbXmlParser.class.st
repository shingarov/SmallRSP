Class {
	#name : #GdbXmlParser,
	#superclass : #Object,
	#instVars : [
		'baseURI'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
GdbXmlParser class >> baseURI: uri [
	^super new 	
		baseURI: uri;
		yourself
]

{ #category : #API }
GdbXmlParser class >> parseFile: fileName in: path assumeEndian: aSymbol [
	| f s |
	self shouldBeImplemented.
	f := FileStream fileNamed:
		path, '/', fileName.
	s := f contentsOfEntireFile.
	f close.
	^self new parseString: s
]

{ #category : #private }
GdbXmlParser >> baseURI: aURIProvider [
	baseURI := aURIProvider 
]

{ #category : #API }
GdbXmlParser >> parseString: s [
	| doc |
	doc := PPXmlParser parse: s.
	doc baseURI: baseURI.
	^doc rootElement copy
]

{ #category : #private }
GdbXmlParser >> processXML: xmlDocument [
	| registerArray oneRegTransfers regNum |
	registerArray := xmlDocument findXPath: 'feature/reg'.
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
	regNum := 0.
	oneRegTransfers do: [ :rt |
		rt regNum == regNum ifFalse: [ self error ].
		regNum := regNum + 1 ].
	^oneRegTransfers
]
