Class {
	#name : #PPC440SPRs,
	#superclass : #SharedPool,
	#classVars : [
		'DBCR0',
		'DBCR1',
		'DBCR2',
		'DBDR',
		'DBSR',
		'IAC1',
		'IAC2',
		'IAC3',
		'IAC4'
	],
	#category : #PowerOCD
}

{ #category : #'class initialization' }
PPC440SPRs class >> initialize [
	" PPC440SPRs initialize "
	DBSR  := 16r130.
	DBCR0 := 16r134.
	DBCR1 := 16r135.
	DBCR2 := 16r136.
	DBDR  := 16r3F3.
	IAC1  := 16r138.
	IAC2  := 16r139.
	IAC3  := 16r13A.
	IAC4  := 16r13B.
]
