Class {
	#name : #GDB,
	#superclass : #Object,
	#instVars : [
		'processorDescription',
		'registerCache'
	],
	#category : #'GDB-RSP'
}

{ #category : #magick }
GDB >> advancePastTrap [
	| trapPC |
	trapPC := self getRegister: 'pc'.
	self setRegister: 'pc' to: trapPC + 4
]

{ #category : #execution }
GDB >> c [
	" Continue. "
	self resetRegisterCache.
	^self cont
]

{ #category : #registers }
GDB >> getRegister: r [
	^self getRegisters at: r ifAbsent: [ self error: 'Unknown register ', r ]
]

{ #category : #registers }
GDB >> getRegisters [
	registerCache isNil ifTrue: [ registerCache := self readRegistersFromTarget ].
	^registerCache
]

{ #category : #registers }
GDB >> pc [
	^self getRegister: self pcRegisterName 
]

{ #category : #registers }
GDB >> pc: newPC [
	^self setRegister: 'pc' to: newPC
]

{ #category : #registers }
GDB >> pcRegisterName [
	^'pc'
]

{ #category : #private }
GDB >> processorDescription [
	^processorDescription
]

{ #category : #private }
GDB >> processorDescription: aPDL [
	processorDescription isNil ifFalse: [ self error: 'Target architecture already set' ].
	processorDescription := aPDL
]

{ #category : #memory }
GDB >> readByteAt: addr [
	^(self readAt: addr nBytes: 1) first
]

{ #category : #registers }
GDB >> resetRegisterCache [
	registerCache isNil ifFalse: [self sendRegistersToTarget].
	registerCache := nil
]

{ #category : #execution }
GDB >> s [
	"Single step.
	Return control with signal 5 (TRAP),
	or if the execution of the current instruction failed, with whatever signal happened."
	self resetRegisterCache.
	^self stepi

]

{ #category : #registers }
GDB >> setRegister: r to: x [
	| dict |
	dict := self getRegisters.
	(dict includesKey: r) ifFalse: [ self error: 'No such register ', r ].
	dict at: r put: x "destructively because this is the gdb's copy"

]
