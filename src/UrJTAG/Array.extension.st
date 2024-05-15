Extension { #name : #Array }

{ #category : #'*UrJTAG' }
Array >> execInBdmPB: aBDM [ 
	^self collect: [ :each | each execInBdmPB: aBDM ]
]
