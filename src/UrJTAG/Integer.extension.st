Extension { #name : #Integer }

{ #category : #'*UrJTAG' }
Integer >> execInBdmPB: aBDM [
	^aBDM ppcInstruction: self
]

{ #category : #'*UrJTAG' }
Integer >> gtInspectorPowerInstructionIn: composite [
	<gtInspectorPresentationOrder: 300>
	| asm |
	asm := [ AcProcessorDescriptions powerpc disassemble: self ]
		on: Error do: [ :ex | ex class name asString ].
	^ composite table
		title: asm;
		display: [ #() ]
]

{ #category : #'*UrJTAG' }
Integer >> reverseSprf [
	^(self & 2r0000011111 << 5) |
	 (self & 2r1111100000 >> 5)
]
