Extension { #name : #String }

{ #category : #'*GDB' }
String >> charPairs [
	" 'abcdef' charPairs ---> { 'ab' . 'cd' . 'ef' } "
	self isEmpty ifTrue: [ ^#() ].
	^{ self copyFrom: 1 to: 2 }, self allButFirst allButFirst charPairs
]
