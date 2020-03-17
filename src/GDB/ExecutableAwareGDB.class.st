Class {
	#name : #ExecutableAwareGDB,
	#superclass : #RemoteGDBSession,
	#instVars : [
		'exe'
	],
	#category : #'GDB-RSP'
}

{ #category : #representation }
ExecutableAwareGDB >> currentBytecode [
	| def vPC |
	def := self exe whereAmI.
	vPC := self getVRegister: #VPC.
	^def ir instructions at: vPC+1
]

{ #category : #representation }
ExecutableAwareGDB >> currentFrame [
	^MTRemoteStackFrame gdb: self pointer: (self getVRegister: #FP)
]

{ #category : #accessing }
ExecutableAwareGDB >> exe [
	^ exe
]

{ #category : #accessing }
ExecutableAwareGDB >> exe: anObject [
	exe := anObject
]

{ #category : #'debug support' }
ExecutableAwareGDB >> findPreviousCallers: maxCallersToFind from: spOrNil [
	| sp |
	maxCallersToFind < 1 ifTrue: [ ^OrderedCollection new ].
	sp := spOrNil isNil ifTrue: [ self getVRegister: #SP ] ifFalse: [ spOrNil ].
	[ self isInStack: sp ] whileTrue: [ 
		| slot |
		slot := self read32At: sp.
		sp := sp - 4.
		(self isInNZone: slot) ifTrue: [ ^(self findPreviousCallers: maxCallersToFind - 1 from: sp) addFirst: (exe whereIsPC: slot); yourself ]
	].
	^OrderedCollection new
]

{ #category : #representation }
ExecutableAwareGDB >> getVRegister: r [
	| nRegister |
	nRegister := TAJWriter vRegister: r.
	^self getRegisters at: 'r', nRegister printString
]

{ #category : #'debug support' }
ExecutableAwareGDB >> isInNZone: anAddress [
	^(anAddress < 272699392) and: [ anAddress > 268505088 ]
]

{ #category : #'debug support' }
ExecutableAwareGDB >> isInStack: anAddress [
	^(anAddress >= 272699392) and: [ anAddress < 273747968 ]
]

{ #category : #representation }
ExecutableAwareGDB >> isMTNil: anOop [
	^anOop = (self exe externalReferences at: #MT_nil)
]

{ #category : #representation }
ExecutableAwareGDB >> mdFor: oop [
	| tagBits   |
	tagBits := oop bitAnd: 3.
	tagBits == 1 ifTrue: [
		"SmallInteger"
		^exe externalReferences at: #MTSmallInteger_md ].
	tagBits == 3 ifTrue: [ 
		"Context"
		^exe externalReferences at: #MTContext_md ].
	^self read32At: oop + 4
]

{ #category : #representation }
ExecutableAwareGDB >> pharoObjectForOop: oop [
	| tag |
	tag := oop bitAnd: 2r111.
	tag == 0 ifTrue: [ ^MTRemoteObject gdb: self pointer: oop ].
	tag == 1 ifTrue: [ ^exe architecture oop >> 4 ]. "SmallInteger; this code better delegate to the Target"
	tag == 3 ifTrue: [ ^MTRemoteObject gdb: self pointer: (oop bitAnd: 16rFFFFFFF0) ].
	self halt "I don't know what tag this is"
]

{ #category : #representation }
ExecutableAwareGDB >> pharoObjectForVReg: vReg [
	^self pharoObjectForOop: (self getVRegister: vReg)
]

{ #category : #surgery }
ExecutableAwareGDB >> processCallback [
	(MTHostCallback gdb: self) process
		
]

{ #category : #representation }
ExecutableAwareGDB >> push: oop [
	| sp |
	sp := self getVRegister: #SP.
	self writeInt32: oop toAddr: sp+4.
	self setVRegister: #SP to: sp+4
]

{ #category : #representation }
ExecutableAwareGDB >> setVRegister: r to: x [
	| nRegister |
	nRegister := TAJWriter vRegister: r.
	self setRegister: nRegister to: x
]

{ #category : #surgery }
ExecutableAwareGDB >> tryRunning [
	self runWithMagickCallback: [ self processCallback ]
]

{ #category : #surgery }
ExecutableAwareGDB >> tryRunningSteps [
	self runStepsWithMagickCallback: [ self processCallback ]
]
