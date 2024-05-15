Extension { #name : #Integer }

{ #category : #'*UrJTAG' }
Integer >> execInBdmPB: aBDM [
	^aBDM ppcInstruction: self
]

{ #category : #'*UrJTAG' }
Integer >> reverseSprf [
	^(self & 2r0000011111 << 5) |
	 (self & 2r1111100000 >> 5)
]
