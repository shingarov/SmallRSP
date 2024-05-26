Class {
	#name : #JTAGBDMTest,
	#superclass : #TestCase,
	#category : #'PowerOCD-Tests'
}

{ #category : #defaults }
JTAGBDMTest >> driver [
	^'ARM-USB-OCD-H'
]

{ #category : #defaults }
JTAGBDMTest >> frequency [
	^10000
]

{ #category : #tests }
JTAGBDMTest >> testDetectPPC440 [
	"Assume a PPC440 target on a board such as Sequoia, or ML507, or XMF5.
	 Connect to the target and detect the part on the JTAG chain, using
	 the part name from the UrJTAG database."
	JTAGBDM new
		connectToDriver: self driver
		at: self frequency
		partName: 'PowerPC 440EPx'
]

{ #category : #tests }
JTAGBDMTest >> testDetectPPC440byId [
	"Assume a PPC440 target on a board such as Sequoia, or ML507, or XMF5.
	 Connect to the target and detect the part on the JTAG chain, without
	 relying on the UrJTAG database (i.e. just by JEDEC IDCODE)."
	JTAGBDM new
		connectToDriver: self driver
		at: self frequency
		partId: 16r0440f1e1
	
	
	
]
