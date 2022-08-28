Class {
	#name : #FeatureParserTest,
	#superclass : #TestCase,
	#category : #'GDB-tests-Unconnected'
}

{ #category : #tests }
FeatureParserTest >> testParse1 [
	| regs r1 lr ctr |
	regs := (GdbXmlParser endian: FakeProcessorDescriptionPPC endian)
		parseString: FakeProcessorDescriptionPPC features.
	r1 := regs at: 2.
	self assert: r1 regName        equals: 'r1'.
	self assert: r1 width          equals: 32.
	self assert: r1 isLittleEndian equals: false.
	self assert: r1 regNum         equals: 1.
	
	lr := regs at: 68.
	self assert: lr regName        equals: 'lr'.
	self assert: lr width          equals: 32.
	self assert: lr isLittleEndian equals: false.
	self assert: lr regNum         equals: 67.
	
	ctr := regs at: 69.
	self assert: ctr regName        equals: 'ctr'.
	self assert: ctr width          equals: 32.
	self assert: ctr isLittleEndian equals: false.
	self assert: ctr regNum         equals: 68.

]
