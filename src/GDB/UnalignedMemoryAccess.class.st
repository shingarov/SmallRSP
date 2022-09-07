Class {
	#name : #UnalignedMemoryAccess,
	#superclass : #Error,
	#instVars : [
		'address'
	],
	#category : #'GDB-RSP'
}

{ #category : #signalling }
UnalignedMemoryAccess class >> signalFor: addr [
	^ self new
		address: addr;
		signal		 
]

{ #category : #accessing }
UnalignedMemoryAccess >> address [
	^ address
]

{ #category : #accessing }
UnalignedMemoryAccess >> address: anObject [
	address := anObject
]

{ #category : #accessing }
UnalignedMemoryAccess >> messageText [
	^ messageText ifNil: [ messageText := self standardMessageText ]

]

{ #category : #accessing }
UnalignedMemoryAccess >> standardMessageText [
	^'Unaligned memory access at ', self address hex
]
