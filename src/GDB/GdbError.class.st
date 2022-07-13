Class {
	#name : #GdbError,
	#superclass : #Error,
	#instVars : [
		'code'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
GdbError class >> code: anErrorCode [
	^self basicNew code: anErrorCode; yourself

]

{ #category : #signalling }
GdbError class >> signalWith: anErrorCode [
	^(self code: anErrorCode) signal
]

{ #category : #accessing }
GdbError >> code [
	^ code
]

{ #category : #accessing }
GdbError >> code: anObject [
	code := anObject
]

{ #category : #private }
GdbError >> isResumable [
	^true
]
