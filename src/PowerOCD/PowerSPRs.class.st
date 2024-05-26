"
I specify the SPR encodings for mfspr and mtspr mandated by the Green Cloth, figure 58 and 59, pp.385–388, for all PowerPC-Architecture processors.
"
Class {
	#name : #PowerSPRs,
	#superclass : #SharedPool,
	#classVars : [
		'ASR',
		'CTR',
		'DAR',
		'DBAT0L',
		'DBAT0U',
		'DBAT1L',
		'DBAT1U',
		'DBAT2L',
		'DBAT2U',
		'DBAT3L',
		'DBAT3U',
		'DEC',
		'DSISR',
		'EAR',
		'IBAT0L',
		'IBAT0U',
		'IBAT1L',
		'IBAT1U',
		'IBAT2L',
		'IBAT2U',
		'IBAT3L',
		'IBAT3U',
		'LR',
		'PVR',
		'SDR1',
		'SPRG0',
		'SPRG1',
		'SPRG2',
		'SPRG3',
		'SRR0',
		'SRR1',
		'TBL',
		'TBU',
		'XER'
	],
	#category : #PowerOCD
}

{ #category : #'class initialization' }
PowerSPRs class >> initialize [
	" PowerSPRs initialize "
	XER      :=  1.
	LR       :=  8.
	CTR      :=  9.
	DSISR    :=  18.
	DAR      :=  19.
	DEC      :=  22.
	SDR1     :=  25.
	SRR0     :=  26.
	SRR1     :=  27.
	SPRG0    :=  272.
	SPRG1    :=  273.
	SPRG2    :=  274.
	SPRG3    :=  275.
	ASR      :=  280. "Note 2: 64-bit implementations only"
	EAR      :=  282.
	PVR      :=  287.
	TBL      :=  284.
	TBU      :=  285.
	IBAT0U   :=  528.
	IBAT0L   :=  529.
	IBAT1U   :=  530.
	IBAT1L   :=  531.
	IBAT2U   :=  532.
	IBAT2L   :=  533.
	IBAT3U   :=  534.
	IBAT3L   :=  535.
	DBAT0U   :=  536.
	DBAT0L   :=  537.
	DBAT1U   :=  538.
	DBAT1L   :=  539.
	DBAT2U   :=  540.
	DBAT2L   :=  541.
	DBAT3U   :=  542.
	DBAT3L   :=  543.
]
