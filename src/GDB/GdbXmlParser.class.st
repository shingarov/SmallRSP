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
