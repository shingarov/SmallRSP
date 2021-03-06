"
I am debug session for JITed code running on the remote target.
I interpret low-level information such as register values and absolute addresses, as JIT data structures.
"
Class {
	#name : #JITCodeDebugSession,
	#superclass : #DebugSession,
	#category : #'GDB-Cog'
}

{ #category : #accessing }
JITCodeDebugSession class >> activationPredicate [
	^JITTrapActivationPredicate new
]

{ #category : #'as yet unclassified' }
JITCodeDebugSession >> addressesAndInstructions [ 
	^self generatorChain addressesAndInstructions 
]

{ #category : #'as yet unclassified' }
JITCodeDebugSession >> cont [
	| gdbContContext |
	gdbContContext := self interruptedContext findContextSuchThat: [ :c | c methodSelector = #runInMemory: ].
	self unwindAndRestartToContext: gdbContContext.
	self resume; clear
]

{ #category : #'as yet unclassified' }
JITCodeDebugSession >> gdb [
	^self processor gdb
]

{ #category : #'as yet unclassified' }
JITCodeDebugSession >> generatorChain [
	"NB: DO NOT CACHE"
	^self processor pcMap at: self pc
]

{ #category : #'as yet unclassified' }
JITCodeDebugSession >> pc [
	^self processor gdb pc
]

{ #category : #'as yet unclassified' }
JITCodeDebugSession >> printOopString: oop [
	^self processor
		pcMap
			anyOne "CogGeneratorChain"
				cogit
					printOopString: oop
		
]

{ #category : #'as yet unclassified' }
JITCodeDebugSession >> processor [
	^self interruptedContext receiver 
]

{ #category : #'as yet unclassified' }
JITCodeDebugSession >> registers [
	^self processor gdb getRegisters 
]
