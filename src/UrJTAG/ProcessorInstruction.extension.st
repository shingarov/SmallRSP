Extension { #name : #ProcessorInstruction }

{ #category : #'*UrJTAG' }
ProcessorInstruction >> execInBdmPB: aBDM [
	^self emit32 execInBdmPB: aBDM
]
