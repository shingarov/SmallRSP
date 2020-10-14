"
I decide when the ULD debugger should be active.
"
Class {
	#name : #JITTrapActivationPredicate,
	#superclass : #GTDebuggerContextPredicate,
	#category : #'GDB-Cog'
}

{ #category : #testing }
JITTrapActivationPredicate >> matchContext: aContext [
	^aContext method selector = #handleJITCodeTrapIn:
]
