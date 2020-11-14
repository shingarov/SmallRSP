Class {
	#name : #ShdbSession,
	#superclass : #DebugSession,
	#instVars : [
		'targetProcessor',
		'targetInterruptedContext'
	],
	#category : #'GDB-Cog'
}

{ #category : #initialization }
ShdbSession >> process: aProcess context: aContext [
	| cogit |
	targetProcessor := aContext receiver.
	cogit := targetProcessor pcMap anyOne cogit.
	targetInterruptedContext := cogit coInterpreter topFrame.
	^super process: aProcess context: aContext
]

{ #category : #accessing }
ShdbSession >> resume [
	| gdbContContext |
	gdbContContext := self interruptedContext findContextSuchThat: [ :c | c methodSelector = #runInMemory: ].
	self unwindAndRestartToContext: gdbContContext.
	super resume.
	self clear
]

{ #category : #accessing }
ShdbSession >> stack [
	^targetInterruptedContext stack
]
