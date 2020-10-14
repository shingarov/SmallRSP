"
GDB has signaled a trap in the target VM;
looking at PC has revealed that it is in JITed code.
Signaling this error is the usual way to open the ULD debugger.
"
Class {
	#name : #JITCodeTrap,
	#superclass : #Error,
	#instVars : [
		'processor'
	],
	#category : #'GDB-Cog'
}

{ #category : #signalling }
JITCodeTrap class >> signalFor: processor [
	^self new
		processor: processor;
		signal
]

{ #category : #accessing }
JITCodeTrap >> description [
	"Return a textual description of the exception."
	| trapPC |
	trapPC := self processor gdb pc.
	^'SIGTRAP: ', trapPC printStringHex
]

{ #category : #accessing }
JITCodeTrap >> processor [
	^ processor
]

{ #category : #accessing }
JITCodeTrap >> processor: anObject [
	processor := anObject
]
