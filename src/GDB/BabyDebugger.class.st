"
I debug the Baby-Smalltalk running on the remote target.
I will be replaced by VDB.
"
Class {
	#name : #BabyDebugger,
	#superclass : #GTGenericStackDebugger,
	#category : #'GDB-Cog'
}

{ #category : #testing }
BabyDebugger class >> availableAutomatically [
	^true
]

{ #category : #accessing }
BabyDebugger class >> defaultTitle [
	^ 'shdb'
]

{ #category : #testing }
BabyDebugger class >> handlesContext: aContext [
	"If there is a remote target to debug in ULD,
	there is probably a baby to debug in VDB.
	This is not quite quite accurate, but will do for now."
	^JITCodeDebugSession activationPredicate matches: aContext
]

{ #category : #'tools registry' }
BabyDebugger class >> register [
	" BabyDebugger register "
	self registerToolsOn: Smalltalk tools.
]

{ #category : #'tools registry' }
BabyDebugger class >> registerToolsOn: registry [
	"Add ourselves to registry. See [Smalltalk tools]" 
	registry registerDebugger: self withRank: 1.
]

{ #category : #accessing }
BabyDebugger class >> sessionClass [
	^ShdbSession
]

{ #category : #'printing/formatting' }
BabyDebugger >> printStackClassColumnForContext: aContext [
	| outer |
	outer := aContext outerMostContext.
	^ String streamContents: [ :aStream |
		aStream nextPutAll: outer receiverClassName.
		outer receiverClassName = outer methodClassName 
			ifFalse:
				[ aStream 
						nextPut: $(;
						nextPutAll: (outer methodClassName);
						nextPut: $) ] ]  
]
