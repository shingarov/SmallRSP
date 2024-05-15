Class {
	#name : #JedecIDCode,
	#superclass : #Object,
	#instVars : [
		'idReg'
	],
	#category : #UrJTAG
}

{ #category : #'instance creation' }
JedecIDCode class >> of: bits [
	bits odd ifFalse: [
		"Devices are not *required* to provide IDCODE."
		^nil
	].
	^self basicNew
		idReg: bits;
		yourself
		
]

{ #category : #GT }
JedecIDCode >> gtFullIn: composite [
	<gtInspectorPresentationOrder: 50>
	composite text 
		title: 'Full';
		display: [ self idReg printStringHex ]
]

{ #category : #GT }
JedecIDCode >> gtManufacturerIn: composite [
	<gtInspectorPresentationOrder: 51>
	composite text 
		title: 'Mfr';
		display: [ self manufacturer printStringHex ]
]

{ #category : #GT }
JedecIDCode >> gtPartIn: composite [
	<gtInspectorPresentationOrder: 52>
	composite text 
		title: 'Part';
		display: [ self part printStringHex ]
]

{ #category : #GT }
JedecIDCode >> gtVersionIn: composite [
	<gtInspectorPresentationOrder: 53>
	composite text 
		title: 'Version';
		display: [ self version printStringHex ]
]

{ #category : #accessing }
JedecIDCode >> idReg [
	^ idReg
]

{ #category : #accessing }
JedecIDCode >> idReg: anObject [
	idReg := anObject
]

{ #category : #accessing }
JedecIDCode >> manufacturer [
	^self idReg & 2r111111111110 >> 1
]

{ #category : #accessing }
JedecIDCode >> part [
	^self idReg & 2r1111111111111111000000000000 >> 12
]

{ #category : #accessing }
JedecIDCode >> version [
	^self idReg & 2r11110000000000000000000000000000 >> 28
]
