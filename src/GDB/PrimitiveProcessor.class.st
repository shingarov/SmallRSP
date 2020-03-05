"
When the native runtime does not have a native implementation for a primitive, a callback into the outer Smalltalk happens which I then process, surgically operating on the state of the inner VM.
"
Class {
	#name : #PrimitiveProcessor,
	#superclass : #BasePrimitiveProcessor,
	#instVars : [
		'exceptionEnvSlot'
	],
	#category : #'GDB-Primitives'
}

{ #category : #accessing }
PrimitiveProcessor >> falseObject [
	^gdb exe externalReferences at: #MT_false
]

{ #category : #accessing }
PrimitiveProcessor >> getExceptionEnvSlot [ 
	exceptionEnvSlot isNil ifTrue: [ exceptionEnvSlot := self nilObject ].
	^exceptionEnvSlot 
]

{ #category : #'control primitives' }
PrimitiveProcessor >> isTerminationContextFor [
	| currentCtx startCtx block blockMarkEnv methodMarkEnv matchesP | self halt.
	currentCtx := MTRemoteStackFrame
		gdb: gdb
		pointer: ((self getVRegister: #R) bitAnd: 2r11 bitInvert32).
	startCtx := MTRemoteStackFrame gdb: gdb pointer: ((self getVRegister: #A) bitAnd: 2r11 bitInvert32).
	block := startCtx blockClosure.
	blockMarkEnv := block env markEnv.
	methodMarkEnv := currentCtx env markEnv.
	matchesP := (currentCtx method pointer = block compiledBlock homeMethod pointer) and:
		[ blockMarkEnv pointer = methodMarkEnv pointer ].
	self return: (matchesP 
		ifTrue: [ self trueObject ]
		ifFalse: [ self falseObject ])
]

{ #category : #'control primitives' }
PrimitiveProcessor >> isValueMarked [
	| frame matchesP |
	frame := MTRemoteStackFrame gdb: gdb pointer: (self receiverOop  bitAnd: 2r11 bitInvert32).
	matchesP := frame method pointer = (gdb exe externalReferences at: #MT_valueMarked).
	self return: (matchesP 
		ifTrue: [ self trueObject ]
		ifFalse: [ self falseObject ])
]

{ #category : #engine }
PrimitiveProcessor >> markFail [
	| cr |
self halt.
	cr := regs at: 'cr'.
	cr := cr bitOr: 2r0010 << (4 * 2). "EQ bit in CR5"
	regs at: 'cr' put: cr
]

{ #category : #engine }
PrimitiveProcessor >> markSuccess [
	| cr |
	cr := regs at: 'cr'.
	cr := cr bitAnd: (2r0010 << (4 * 2)) bitInvert32. "EQ bit in CR5"
	regs at: 'cr' put: cr
]

{ #category : #accessing }
PrimitiveProcessor >> nilObject [
	^gdb exe externalReferences at: #MT_nil
]

{ #category : #'integer primitives' }
PrimitiveProcessor >> primAddSI [
	| r a sum |
	r := self getVRegister: #R.
	(r bitAnd: 2r11) = 2r0001 ifFalse:[ ^self markFail ].
	r := gdb exe architecture smallIntegerToInteger: r.
	a := self getVRegister: #A.
	(a bitAnd: 2r11) = 2r0001 ifFalse:[ ^self markFail ].
	a := gdb exe architecture smallIntegerToInteger: a.
	sum := r + a.
	sum := gdb exe architecture integerToSmallInteger: sum.
	self return: sum
]

{ #category : #'object access primitives' }
PrimitiveProcessor >> primAlignedPointerOopAt [
	| r idx oop |
	idx := (self getVRegister: #A) >> 4.
	r := (self getVRegister: #R) bitAnd: 2r0011 bitInvert32.
	oop := gdb read32At: r + (idx*4).
	self return: oop
]

{ #category : #'object access primitives' }
PrimitiveProcessor >> primAlignedPointerOopAtPut [
	| r idx arg2  |
	idx := (self getVRegister: #A) >> 4.
	r := (self getVRegister: #R) bitAnd: 2r00011 bitInvert32.
	arg2 := gdb currentFrame arg: 2.
	gdb writeInt32: arg2 toAddr: r + (idx*4).
	self markSuccess 
]

{ #category : #'object access primitives' }
PrimitiveProcessor >> primBasicAt [
	| l |
	"NB - guards"
	l := MTRemoteLiveObject gdb: gdb liveObject: (self getVRegister: #R).
	self return: (l mtBasicAt: (self getVRegister: #A) >> 4)
]

{ #category : #'object access primitives' }
PrimitiveProcessor >> primBasicAtPut [
	| l idx arg2 |
	"NB - guards"
	l := MTRemoteLiveObject gdb: gdb liveObject: (self getVRegister: #R).
	idx := (self getVRegister: #A) >> 4.
	arg2 := self currentFrame arg: 2.
	l mtBasicAt: idx put: arg2.
	self markSuccess 
]

{ #category : #primitives }
PrimitiveProcessor >> primBasicSize [
	| answer |
" NB: insert prim failure guards here, such as SmallInteger "
	answer := (MTRemoteLiveObject gdb: gdb liveObject: self receiverOop) numIndexed.
	answer := (answer bitShift: 4) bitOr: 2r0001. "SI"
	self return: answer
]

{ #category : #'object access primitives' }
PrimitiveProcessor >> primCharacterBasicAt [
	| index char |
	index := self getVRegister: #A.
	"Guard that the arg is an SI:"
	(index bitAnd: 2r00011) = 1 ifFalse: [ ^self markFail ].
	index := index >> 4.
	
	char := gdb readByteAt: (self getVRegister: #R) + 8 + index - 1.
	
	"CHAR_TAG=2"
	char := char << 4 bitOr: 2.
	
	self return: char
]

{ #category : #'object access primitives' }
PrimitiveProcessor >> primCharacterBasicAtPut [
	| index char charOop |

	index := self getVRegister: #A.
	"Guard that the index arg is an SI"
	(index bitAnd: 2r00011) = 1 ifFalse: [ ^self markFail ].
	index := index >> 4.
	
	charOop := self currentFrame arg: 2.
	"Guard that the char arg is a char"
	(charOop bitAnd: 2r00011) = 2 ifFalse: [ ^self markFail ].
	char := charOop >> 4.
	
	gdb writeByte: char toAddr: (self getVRegister: #R) + 8 + index - 1.
	
	self return: charOop
]

{ #category : #'object access primitives' }
PrimitiveProcessor >> primCharacterFromCodePoint [
	| arg |
	arg := self getVRegister: #A.
	"Remove the SI tag and attach a CHAR tag"
	arg := (arg bitAnd: 16rFFFFFFF0) bitOr: 2r00010.
	self return: arg
]

{ #category : #'object access primitives' }
PrimitiveProcessor >> primCharacterValue [
	| arg |
	arg := self getVRegister: #A.
	"Remove the CHAR tag and attach a SI tag"
	arg := (arg bitAnd: 16rFFFFFFF0) bitOr: 2r00001.
	self return: arg
]

{ #category : #primitives }
PrimitiveProcessor >> primClass [
	| rcv md |
	rcv := MTRemoteObject gdb: gdb pointer:  (self getVRegister: #R).
	md := MTRemoteMethodDictionary gdb: gdb pointer: rcv md.
	[ md pointer = self nilObject ] whileFalse: [ 
		| clazz |
		clazz := md definingClass.
		clazz isRemoteNil ifFalse: [ ^self return: clazz pointer ].
		md := md superMd ].
	^self return: self nilObject 
]

{ #category : #primitives }
PrimitiveProcessor >> primClassName [
	| rcv |
	"The argument is a class."
	rcv := MTRemoteClass gdb: gdb pointer:  (self getVRegister: #R).
	self return: rcv name
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primCurrentContext [
	| context  |
	context := self currentFrame senderFrame.
	self return: (context pointer bitOr: 3)
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primExceptionEnvironment [
	^self return: self getExceptionEnvSlot 
]

{ #category : #'feature primitives' }
PrimitiveProcessor >> primFail [
	self markFail
]

{ #category : #'object access primitives' }
PrimitiveProcessor >> primHash [
	self return: 2r0001. "SI 0"

]

{ #category : #'integer primitives' }
PrimitiveProcessor >> primIntDivSI [
	| r a d |
	r := self getVRegister: #R.
	(r bitAnd: 2r111) = 2r0001 ifFalse:[ ^self markFail ].
	r := gdb exe architecture smallIntegerToInteger: r.
	a := self getVRegister: #A.
	(a bitAnd: 2r111) = 2r0001 ifFalse:[ ^self markFail ].
	a := gdb exe architecture smallIntegerToInteger: a.
	d := r // a.
	d := gdb exe architecture integerToSmallInteger: d.
	self return: d
]

{ #category : #'integer primitives' }
PrimitiveProcessor >> primIntRemSI [
	| r a rem |
	r := self getVRegister: #R.
	(r bitAnd: 2r11) = 2r0001 ifFalse:[ ^self markFail ].
	r := gdb exe architecture smallIntegerToInteger: r.
	a := self getVRegister: #A.
	(a bitAnd: 2r11) = 2r0001 ifFalse:[ ^self markFail ].
	a := gdb exe architecture smallIntegerToInteger: a.
	rem  := r \\ a.
	rem := gdb exe architecture integerToSmallInteger: rem.
	self return: rem
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primIsBottomOfStack [
	| bottomP frame |
	frame := MTRemoteStackFrame gdb: gdb pointer: ((self getVRegister: #R) bitAnd: 2r11 bitInvert32).
	bottomP := frame isBottomFrame.
	self return: (bottomP 
		ifTrue: [ self trueObject ]
		ifFalse: [ self falseObject ])
]

{ #category : #'integer primitives' }
PrimitiveProcessor >> primMultiplySI [
	| r a p |
	r := self getVRegister: #R.
	(r bitAnd: 2r111) = 2r0001 ifFalse:[ ^self markFail ].
	r := gdb exe architecture smallIntegerToInteger: r.
	a := self getVRegister: #A.
	(a bitAnd: 2r111) = 2r0001 ifFalse:[ ^self markFail ].
	a := gdb exe architecture smallIntegerToInteger: a.
	p := r * a.
	p := gdb exe architecture integerToSmallInteger: p.
	self return: p
]

{ #category : #primitives }
PrimitiveProcessor >> primNewIndexedObject [
	| clazz size |
self halt.
	size := (self getVRegister: #A) >> 4.
size >= (65536*4) ifTrue:[self halt].
	clazz := MTRemoteClass gdb: gdb pointer: self receiverOop.
	clazz structure = 5
		ifTrue: [ self allocString: size md: clazz instanceMd ]
		ifFalse: [ self allocIndexed: size md: clazz instanceMd ]
]

{ #category : #primitives }
PrimitiveProcessor >> primNewObject [
	| clazz  |
	clazz := MTRemoteClass gdb: gdb pointer: self receiverOop.
	self return: (self
		allocOop: clazz instVarCount
		md: clazz instanceMd)
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primPerform [
	"Perform a 0-arg send.
	Receiver in R, selector in A."
	| a selector fp frame jmpTarget |

	a := self getVRegister: #A.
	fp := self getVRegister: #FP.
	frame := MTRemoteStackFrame gdb: gdb pointer: fp.
	self setVRegister: #SP to: fp-4.
	self setVRegister: #FP to: frame senderFrame pointer.
	
	selector := MTRemoteSymbol gdb: gdb pointer: a.
	jmpTarget := (HostAssistedLookup regBase: self ram: gdb) messageSendSelector: selector symbol.
	"jmpTarget := gdb messageSendSelector: selector symbol."  "correct address to jump to in CTR!"
	"but the contract with the primitive invocation code is that
	the address is in scratch1."
	self setVRegister: #Scratch1 to: jmpTarget 
	
	
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primPreviousContext [
	| context prevAddr |
	context := self receiverOop.
	context := context bitAnd: 3 bitInvert32.
	prevAddr := gdb read32At: context.
	self return: (prevAddr bitOr: 3)
]

{ #category : #'feature primitives' }
PrimitiveProcessor >> primPrintString [
	| r tag |
self halt.
	r := self receiverOop.
	tag := r bitAnd: 2r00011.
	tag = 0 ifTrue: [ 
		"For pointer oops, assume it's a String"
		| rs |
		rs := MTRemoteString gdb: gdb pointer: r.
		Transcript show: rs string.
		^self markSuccess 		
	].
	tag = 1 ifTrue: [
		Transcript show: (r>>4) printString.
		^self markSuccess 		
	].
	self halt
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primReturnValueFromContext [
	| ctx |
	self setVRegister: #R to: (self getVRegister: #A).
	ctx := gdb read32At: (self getVRegister: #FP) - 8.
	ctx := ctx bitAnd: 2r11 bitInvert32. "strip off context immediate tag"
	self setVRegister: #FP to: ctx.
	self setVRegister: #SP to: ctx + 4.
	self markSuccess 
]

{ #category : #'integer primitives' }
PrimitiveProcessor >> primSIGreaterThan [
	self return: ((self getVRegister: #R) > (self getVRegister: #A)
		ifTrue: [ self trueObject ]
		ifFalse: [ self falseObject ])
]

{ #category : #'integer primitives' }
PrimitiveProcessor >> primSIGreaterThanEqual [
	self return: ((gdb getVRegister: #R) >= (gdb getVRegister: #A)
		ifTrue: [ self trueObject ]
		ifFalse: [ self falseObject ])
]

{ #category : #'integer primitives' }
PrimitiveProcessor >> primSILessThan [
	self return: ((self getVRegister: #R) < (self getVRegister: #A)
		ifTrue: [ self trueObject ]
		ifFalse: [ self falseObject ])
]

{ #category : #'integer primitives' }
PrimitiveProcessor >> primSILessThanEqual [
	self return: ((self getVRegister: #R) <= (self getVRegister: #A)
		ifTrue: [ self trueObject ]
		ifFalse: [ self falseObject ])
]

{ #category : #'feature primitives' }
PrimitiveProcessor >> primSay [
	" Show the receiver SI on the host transcript. "
	| something tag |
self halt.
	something := self getVRegister: #R.
	tag := something bitAnd: 2r00011.
	tag = 0 ifTrue: [ something := 'oop:', something asString, ' md:', (MTRemoteObject gdb: gdb pointer: something) md printString ].
	tag = 1 ifTrue: [ something := gdb exe architecture smallIntegerToInteger: something ].
	
	Transcript show: ('Modtalk says: ', something asString); cr.
	self markSuccess.


	
]

{ #category : #'feature primitives' }
PrimitiveProcessor >> primSay2 [
	" On the class side of ProtoObject. "
self halt.
	self markSuccess.


	
]

{ #category : #'feature primitives' }
PrimitiveProcessor >> primSaySomething [
self halt.
	Transcript show: 'It WORKS!!!'; cr
]

{ #category : #'feature primitives' }
PrimitiveProcessor >> primSaySomethingElse [
	Transcript show: 'It DOES NOT WORK!!!'; cr
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primSetExceptionEnvironment [
	| ee |
	ee := self getVRegister: #A.
	self setExceptionEnvSlot: ee;
		markSuccess 
]

{ #category : #'feature primitives' }
PrimitiveProcessor >> primStringCompare [
	| r b answer |
	r := self getVRegister: #R.
	r := MTRemoteString gdb: gdb pointer: r.
	r := r string.
	
	b := self getVRegister: #A.
	b := MTRemoteString gdb: gdb pointer: b.
	b := b string.
	
	answer := r = b ifTrue: [2] ifFalse: [ r < b ifTrue: [1] ifFalse: [3] ].
	answer := gdb exe architecture integerToSmallInteger: answer.
	self return: answer
]

{ #category : #'integer primitives' }
PrimitiveProcessor >> primSubSI [
	| r a diff |
	r := self getVRegister: #R.
	(r bitAnd: 2r111) = 2r0001 ifFalse:[ ^self markFail ].
	r := gdb exe architecture smallIntegerToInteger: r.
	a := self getVRegister: #A.
	(a bitAnd: 2r111) = 2r0001 ifFalse:[ ^self markFail ].
	a := gdb exe architecture smallIntegerToInteger: a.
	diff := r - a.
	diff := gdb exe architecture integerToSmallInteger: diff.
	self return: diff
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primValue [
	| addr |
	addr := (MTRemoteBlockClosure gdb: gdb pointer: self receiverOop)
		compiledBlock codeRef address.
	self setVRegister: #Scratch1 to: addr.
	self setVRegister: #X to: self receiverOop.
	self markSuccess 
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primValueWith2Args [
	| arg2 |
	arg2 := self currentFrame arg: 2.
	gdb push: arg2.
	^self primValue
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primValueWithArgument [
	^self primValue "No need to specifically pass the arg because it is already in #A"
]

{ #category : #'control primitives' }
PrimitiveProcessor >> primValueWithArguments [
self halt
]

{ #category : #engine }
PrimitiveProcessor >> receiverOop [
	^self getVRegister: #R
]

{ #category : #engine }
PrimitiveProcessor >> return: oop [
	self setVRegister: #R to: oop.
	self markSuccess
]

{ #category : #accessing }
PrimitiveProcessor >> setExceptionEnvSlot: oop [
	exceptionEnvSlot := oop
]

{ #category : #accessing }
PrimitiveProcessor >> trueObject [
	^gdb exe externalReferences at: #MT_true
]
