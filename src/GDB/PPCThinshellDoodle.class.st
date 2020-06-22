Class {
	#name : #PPCThinshellDoodle,
	#superclass : #ThinshellDoodle,
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
PPCThinshellDoodle >> host [
	"We experiment with PowerPC on either a physical dev board (P1025),
	or on gem5.  Concrete doodles must answer #svost or #gem5."
	^self subclassResponsibility
]

{ #category : #'target connection' }
PPCThinshellDoodle >> hostIP [
	self host = #svost ifTrue: [^'192.168.75.199'].
	self host = #gem5 ifTrue: [^'192.168.75.2'].
	self error: 'what host is this?'
]

{ #category : #'target connection' }
PPCThinshellDoodle >> pdl [
	self host = #svost ifTrue: [^FakeProcessorDescriptionP1025 new].
	self host = #gem5 ifTrue: [^FakeProcessorDescriptionPPC new].
	self error: 'what host is this?'
]
