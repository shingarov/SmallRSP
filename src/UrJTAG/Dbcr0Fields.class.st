Class {
	#name : #Dbcr0Fields,
	#superclass : #SharedPool,
	#classVars : [
		'BRT',
		'DAC1R',
		'DAC1W',
		'DAC2R',
		'DAC2W',
		'EDM',
		'FT',
		'IAC1',
		'IAC2',
		'IAC3',
		'IAC4',
		'ICMP',
		'IDM',
		'IRPT',
		'RET',
		'RST_chip',
		'RST_core',
		'RST_system',
		'TRAP'
	],
	#category : #UrJTAG
}

{ #category : #'class initialization' }
Dbcr0Fields class >> initialize [
	" Dbcr0Fields initialize "
	EDM := 16r80000000.
	IDM := 16r40000000.
	RST_chip   := 16r20000000.
	RST_core   := 16r10000000.
	RST_system := 16r30000000.
	ICMP := 16r8000000.
	BRT  := 16r4000000.
	IRPT := 16r2000000.
	TRAP := 16r1000000.
	IAC1 := 16r800000.
	IAC2 := 16r400000.
	IAC3 := 16r200000.
	IAC4 := 16r100000.
	DAC1R := 16r80000.
	DAC1W := 16r40000.
	DAC2R := 16r20000.
	DAC2W := 16r10000.
	RET := 16r8000.
	FT := 16r1.
]
