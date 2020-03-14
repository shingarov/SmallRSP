Class {
	#name : #InferiorExited,
	#superclass : #Error,
	#instVars : [
		'exitCode'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
InferiorExited class >> exitCode: anInteger [
	^self new exitCode: anInteger; yourself
]

{ #category : #signaling }
InferiorExited class >> signalWithExitCode: anInteger [

	^(self exitCode: anInteger) signal
]

{ #category : #accessing }
InferiorExited >> exitCode [
	^ exitCode
]

{ #category : #accessing }
InferiorExited >> exitCode: anObject [
	exitCode := anObject
]
