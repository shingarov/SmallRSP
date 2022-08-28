Class {
	#name : #BasePrimitiveProcessor,
	#superclass : #Object,
	#instVars : [
		'gdb',
		'regs'
	],
	#category : #'GDB-Modtalk'
}

{ #category : #'as yet unclassified' }
BasePrimitiveProcessor class >> gdb: gdb [
	^self new
		gdb: gdb;
		yourself
]

{ #category : #engine }
BasePrimitiveProcessor >> advancePastTrap [
	regs at: 'pc' put: (regs at: 'pc')+4
]

{ #category : #allocating }
BasePrimitiveProcessor >> allocIndexed: size md: md [
	|  ptr obj liveObj mtNil |
	ptr := self allocSlots: size serviceSlots: 3.
	obj := MTRemoteObject gdb: gdb pointer: ptr.
	obj header: (TAJObjectWriter
		declareObjectHeaderType: 'IndexedObjectType'
		hashFormat: 'NoHash'
		numVars: size
		hash: 0
		meta: 0).
	obj md: md.
	liveObj := MTRemoteLiveIndexedObject gdb: gdb pointer: ptr.
	mtNil := gdb exe externalReferences at: #MT_nil.
	1 to: size do: [ :i |
		liveObj mtAt: i put: mtNil ].
	self return: ptr
]

{ #category : #allocating }
BasePrimitiveProcessor >> allocOop: numVars md: md [
	|  ptr obj mtNil |
	ptr := self allocSlots: numVars serviceSlots: 3 "header, md, hash".
	obj := MTRemoteObject gdb: gdb pointer: ptr.
	obj header: (TAJObjectWriter
		declareObjectHeaderType: 'OopObjectType'
		hashFormat: 'NoHash'
		numVars: numVars hash: 0 meta: 0).
	obj md: md.
	"nil the slots:"
	mtNil := gdb exe externalReferences at: #MT_nil.
	1 to: numVars do: [ :idx | obj basicSlotAt: idx+2 put: mtNil ].
	^ptr

]

{ #category : #allocating }
BasePrimitiveProcessor >> allocSlots: numSlots serviceSlots: s [
	| nBytes p |
	false ifTrue: [ ^self allocSlotsFromMTXMemory: numSlots ].
	
	nBytes := (numSlots + s) * 4.
	nBytes := nBytes+15 bitAnd: 16rFFFFFFF0.
	p := self getVRegister: #HEAP.
	self setVRegister: #HEAP to: p + nBytes.
	^p
]

{ #category : #allocating }
BasePrimitiveProcessor >> allocSlotsFromMTXMemory: numSlots [
	| nBytes |
	nBytes := numSlots * 4.
	^self gdb exe objectMemory alloc: nBytes
]

{ #category : #allocating }
BasePrimitiveProcessor >> allocString: size md: md [
	| ptr obj |
	ptr := self allocSlots: (size + 2 + 3 // 4) serviceSlots: 3 "header, md, hash".
	obj := MTRemoteObject gdb: gdb pointer: ptr.
	obj header: (TAJObjectWriter
		declareObjectHeaderType: 'ZByteObjectType'
		hashFormat: 'StringHash'
		numVars: size
		hash: 0
		meta: 0).
	obj md: md.
	self return: ptr
]

{ #category : #engine }
BasePrimitiveProcessor >> currentFrame [
"NB -- this is problematic, this parallel implementation with GDB"
	^MTRemoteStackFrame gdb: gdb pointer: (self getVRegister: #FP)
]

{ #category : #accessing }
BasePrimitiveProcessor >> gdb [
	^ gdb
]

{ #category : #accessing }
BasePrimitiveProcessor >> gdb: anObject [
	gdb := anObject
]

{ #category : #engine }
BasePrimitiveProcessor >> getAllRegisters [
	regs := gdb getRegisters
]

{ #category : #engine }
BasePrimitiveProcessor >> getRegister: r [
	^regs at: r
]

{ #category : #engine }
BasePrimitiveProcessor >> getVRegister: regSym [
	| nRegister |
	nRegister := TAJWriter vRegister: regSym.
	^self getRegister: 'r', nRegister printString
]

{ #category : #engine }
BasePrimitiveProcessor >> processPrimitive: primitiveSelector [
	self
		getAllRegisters;
		perform: primitiveSelector;
		advancePastTrap;
		setAllRegisters
]

{ #category : #engine }
BasePrimitiveProcessor >> setAllRegisters [
	gdb setRegisters: regs
]

{ #category : #engine }
BasePrimitiveProcessor >> setRegister: r to: x [
	regs at: r put: x
]

{ #category : #engine }
BasePrimitiveProcessor >> setVRegister: r to: x [
	| nRegister |
	nRegister := TAJWriter vRegister:  r.
	nRegister := 'r', nRegister printString.
	self setRegister: nRegister to: x
]
