Class {
	#name : #FakeProcessorDescriptionPPC,
	#superclass : #Object,
	#instVars : [
		'regsInGPacket'
	],
	#category : #'GDB-Tests'
}

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionPPC class >> endian [ 
	^#big
]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionPPC class >> fakeFeatures [
	^#(
#('r0' 32)  #('r1' 32)  #('r2' 32)  #('r3' 32)  #('r4' 32)  #('r5' 32)  #('r6' 32)  #('r7' 32)
#('r8' 32)  #('r9' 32)  #('r10' 32) #('r11' 32) #('r12' 32) #('r13' 32) #('r14' 32) #('r15' 32)
#('r16' 32) #('r17' 32) #('r18' 32) #('r19' 32) #('r20' 32) #('r21' 32) #('r22' 32) #('r23' 32)
#('r24' 32) #('r25' 32) #('r26' 32) #('r27' 32) #('r28' 32) #('r29' 32) #('r30' 32) #('r31' 32)

#('f0' 64)  #('f1' 64)  #('f2' 64)  #('f3' 64)  #('f4' 64)  #('f5' 64)  #('f6' 64)  #('f7' 64)
#('f8' 64)  #('f9' 64)  #('f10' 64) #('f11' 64) #('f12' 64) #('f13' 64) #('f14' 64) #('f15' 64)
#('f16' 64) #('f17' 64) #('f18' 64) #('f19' 64) #('f20' 64) #('f21' 64) #('f22' 64) #('f23' 64)
#('f24' 64) #('f25' 64) #('f26' 64) #('f27' 64) #('f28' 64) #('f29' 64) #('f30' 64) #('f31' 64)

#('pc' 32)
#('msr' 32)
#('cr' 32)
#('lr' 32)
#('ctr' 32)
#('xer' 32)
)

]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionPPC >> architectureName [
	^'powerpc'
]

{ #category : #accessing }
FakeProcessorDescriptionPPC >> endian [ 
	^self class endian
]

{ #category : #accessing }
FakeProcessorDescriptionPPC >> initRegsInGPacket [
	"Do not try to use the real GdbXmlParser.
	On some Smalltalks under development, there is no XML."
	| regNum |
	regNum := 0.
	regsInGPacket := self class fakeFeatures collect: [ :reg |
		| rt |
		rt := RSPOneRegisterTransfer new
			regName: (reg first);
			width: (reg second);
			isLittleEndian: (self endian = #little);
			regNum: regNum;
			yourself.
		regNum := regNum + 1.
		
		rt ]
]

{ #category : #accessing }
FakeProcessorDescriptionPPC >> regsInGPacket [
	regsInGPacket isNil ifTrue: [ self initRegsInGPacket ].
	^regsInGPacket 
]
