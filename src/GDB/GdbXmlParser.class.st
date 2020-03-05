Class {
	#name : #GdbXmlParser,
	#superclass : #Object,
	#instVars : [
		'gdb',
		'g',
		'reg',
		'defines',
		'gdbElement'
	],
	#category : #'GDB-RSP'
}

{ #category : #API }
GdbXmlParser class >> parseFile: fileName in: path [
	| f s |
	f := FileStream fileNamed:
		path, '/', fileName.
	s := f contentsOfEntireFile.
	f close.
	^ self new parseString: s
]

{ #category : #parsing }
GdbXmlParser >> parseString: s [
	^self processXML: (PPXmlParser parse: s) rootElement 

]

{ #category : #parsing }
GdbXmlParser >> processXML: xmlDocument [
	| registerArray oneRegTransfers regNum |
	registerArray := xmlDocument findXPath: 'feature/reg'.
	regNum := 0.
	oneRegTransfers := registerArray collect: [ :regNode |
		| rt |
		regNum := (regNode attributeAt: 'regnum' ifAbsent: [ regNum ]) asInteger.
		rt := RSPOneRegisterTransfer new
			regName: (regNode attributeAt: 'name');
			width: ((regNode attributeAt: 'bitsize') asInteger);
			isLittleEndian: false;
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
