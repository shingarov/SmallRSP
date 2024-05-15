Extension { #name : #String }

{ #category : #'*UrJTAG' }
String >> execInBdmPB: aBDM [ 
	^(AcProcessorDescriptions powerpc assembleLines: self) execInBdmPB: aBDM
]

{ #category : #'*UrJTAG' }
String class >> fromFFIExternalArray: bytes [
	^(bytes asArray copyUpTo: 0) asByteArray asString
]
