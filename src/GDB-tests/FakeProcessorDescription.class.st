Class {
	#name : #FakeProcessorDescription,
	#superclass : #Object,
	#instVars : [
		'regsInGPacket'
	],
	#category : #'GDB-tests'
}

{ #category : #accessing }
FakeProcessorDescription >> endian [ 
	^self class endian
]

{ #category : #accessing }
FakeProcessorDescription >> initRegsInGPacket [
	"Do not try to use the real GdbXmlParser.
	On some Smalltalks under development, there is no XML."
	| regNum |
	regNum := 0.
	regsInGPacket := self class fakeFeatures collect: [ :reg |
		| rt |
		rt := RSPOneRegisterTransfer new
			regName: (reg first);
			width: (reg second);
			regNum: regNum;
			yourself.
		regNum := regNum + 1.
		
		rt ]
]

{ #category : #accessing }
FakeProcessorDescription >> regsInGPacket [
	regsInGPacket isNil ifTrue: [ self initRegsInGPacket ].
	^regsInGPacket 
]
