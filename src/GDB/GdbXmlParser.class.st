Class {
	#name : #GdbXmlParser,
	#superclass : #Object,
	#instVars : [
		'isLittleEndian'
	],
	#category : #'GDB-RSP'
}

{ #category : #API }
GdbXmlParser class >> endian: aSymbol [
	| le |
	aSymbol == #big ifTrue: [ le := false ]
		ifFalse: [ aSymbol == #little ifTrue: [ le := true ] ifFalse: [ self error: 'Endian must be big or little' ]].
	^self basicNew 
		isLittleEndian: le
]

{ #category : #API }
GdbXmlParser class >> new [
	self error: 'Please use #endian:'
]

{ #category : #API }
GdbXmlParser class >> parseFile: fileName in: path assumeEndian: aSymbol [
	| f s |
	f := FileStream fileNamed:
		path, '/', fileName.
	s := f contentsOfEntireFile.
	f close.
	^ (self endian: aSymbol) parseString: s
]

{ #category : #private }
GdbXmlParser >> isLittleEndian: aBoolean [
	isLittleEndian := aBoolean.
	^self
]

{ #category : #API }
GdbXmlParser >> parseString: s [
	| parser |
	parser := XMLDOMParser on: s.
	^self processXML: parser parseDocument root.
]

{ #category : #private }
GdbXmlParser >> processXML: root [
	| oneRegTransfers regNum feature regs |
	feature := root nodesDetect: [ :aChild | aChild isElementNamed: 'feature' ].
	regs := feature nodesSelect:  [ :aChild | aChild isElementNamed: 'reg' ].
	regNum := 0.
	oneRegTransfers := regs collect: [ :regNode |
		| rt |
		rt := RSPOneRegisterTransfer new
			regName: (regNode attributeAt: 'name');
			width: ((regNode attributeAt: 'bitsize') asInteger);
			isLittleEndian: isLittleEndian;
			regNum: regNum;
			yourself.
		regNum := regNum + 1.
		rt ].
	
	oneRegTransfers := oneRegTransfers asOrderedCollection sorted: [ :rA :rB | rA regNum <= rB regNum ].
	
	"Verify that we got all registers from 0 to the total number, without holes."
	regNum := 0.
	oneRegTransfers do: [ :rt |
		rt regNum == regNum ifFalse: [ self error ].
		regNum := regNum + 1 ].
	^oneRegTransfers
]
