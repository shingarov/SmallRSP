Class {
	#name : #BDI2000PPC400,
	#superclass : #RSPClient,
	#category : #'GDB-RSP'
}

{ #category : #GT }
BDI2000PPC400 >> gtInspectorInfoIn: composite [
	<gtInspectorPresentationOrder: 30>
	^composite text
		title: 'Info';
		display: [ self qRcmd: 'info' ]
]

{ #category : #GT }
BDI2000PPC400 >> gtInspectorRDIn: composite [
	<gtInspectorPresentationOrder: 31>
	^composite text
		title: 'RD';
		display: [ self qRcmd: 'rd' ]
]

{ #category : #GT }
BDI2000PPC400 >> gtInspectorRDUMPIn: composite [
	<gtInspectorPresentationOrder: 31>
	^composite text
		title: 'RDUMP';
		display: [ self qRcmd: 'rdump' ]
]

{ #category : #registers }
BDI2000PPC400 >> sendRegisters: regDictionary transfers: transfers endian: endian [
"TODO -- G packet doesn't work.  Nothing for now"
]

{ #category : #'GDB commands' }
BDI2000PPC400 >> sendRegistersToTarget [
"TODO. Nothing for now"
]

{ #category : #'GDB commands' }
BDI2000PPC400 >> stepi [
	self qRcmd: 'ti'
]
