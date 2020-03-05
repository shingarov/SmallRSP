Class {
	#name : #MTHostCallback,
	#superclass : #Object,
	#instVars : [
		'gdb',
		'primitiveProcessor',
		'privatePrimitiveProcessor'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
MTHostCallback class >> gdb: gdb [
	^self new 
		gdb: gdb;
		yourself
]

{ #category : #surgery }
MTHostCallback >> call [
	| methodDef cmRef regs |
	methodDef := gdb currentBytecode method value.
	regs := gdb getRegisters.
	regs at: 'ctr' put: (gdb exe externalReferences at: methodDef codeRuntimeName).
	cmRef := gdb exe externalReferences at: methodDef globalName.
	regs at: (TAJWriter registerMap at: #X) put: cmRef.
	gdb setRegisters: regs.
	gdb s
]

{ #category : #accessing }
MTHostCallback >> gdb [
	^ gdb
]

{ #category : #accessing }
MTHostCallback >> gdb: anObject [
	gdb := anObject
]

{ #category : #surgery }
MTHostCallback >> getRegisters [
self halt
]

{ #category : #surgery }
MTHostCallback >> handleDNU [
	| regs receiver selector className |
	regs := gdb getRegisters.
	receiver := regs at: 'r1'.
	receiver := MTRemoteObject gdb: gdb pointer: receiver.
	className := receiver mdObject definingClass name symbol.
	selector := regs at: 'r7'.
	selector := MTRemoteSymbol gdb: gdb pointer: selector.
	selector := selector symbol.
	self halt
]

{ #category : #surgery }
MTHostCallback >> inspectionPoint [
self halt.
"	selectorOop := self getRegisters at: 'r7'.
	primSaySelector := exe externalReferences at: #Symbol_primSay.
	selectorOop = primSaySelector ifTrue: [ 
		siOOP := self getVRegister: #R.
		self pharoObjectForOop: siOOP.
		self halt.
		siMD := self mdFor: siOOP.
		superMD := (MTRemoteMethodDictionary gdb: self pointer: siMD) superMd.
		superSuperMD := superMD superMd.
		 ]."
	"anotherWay := self currentBytecode selector value."
	"gdb s"
]

{ #category : #surgery }
MTHostCallback >> messageSend [
	^self messageSendSelector: gdb currentBytecode selector value
]

{ #category : #surgery }
MTHostCallback >> primitive [
	self primitiveProcessor
		processPrimitive: gdb currentBytecode selector value
		
]

{ #category : #accessing }
MTHostCallback >> primitiveProcessor [
	primitiveProcessor isNil ifTrue: [ primitiveProcessor := PrimitiveProcessor gdb: gdb ].
	^primitiveProcessor 
]

{ #category : #accessing }
MTHostCallback >> privatePrimitive [
	self privatePrimitiveProcessor
		processPrimitive: gdb currentBytecode selector value
		
]

{ #category : #accessing }
MTHostCallback >> privatePrimitiveProcessor [
	privatePrimitiveProcessor isNil ifTrue: [ privatePrimitiveProcessor := PrivatePrimitiveProcessor gdb: gdb ].
	^privatePrimitiveProcessor 
]

{ #category : #API }
MTHostCallback >> process [
	| callNo |
	callNo := gdb getRegisters at: 'r22'. "cf. TAJWriter>>nextPutMagic:"
	callNo = 1 ifTrue: [ ^self messageSend ].
	callNo = 2 ifTrue: [ ^self primitive ].
	callNo = 3 ifTrue: [ ^self return ].
	callNo = 4 ifTrue: [ ^self privatePrimitive ].
	callNo = 5 ifTrue: [ ^self call ].
	callNo = 6 ifTrue: [ ^self handleDNU ].
	callNo = 7 ifTrue: [ ^self inspectionPoint ].
	self halt  "Unknown callback"
]

{ #category : #surgery }
MTHostCallback >> return [
	| cm numToDrop |
	self halt.
	cm := MTRemoteCompiledMethod gdb: self pointer: (self getVRegister: #X).
	numToDrop := cm numArgs - 1 max: 0.
	numToDrop = 0 ifFalse: [ self setVRegister: #SP to: (self getVRegister: #SP) - (numToDrop * 4) ]

]
