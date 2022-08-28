Class {
	#name : #MicrowattJTAG,
	#superclass : #Object,
	#classVars : [
		'DBG_CORE_CTRL',
		'DBG_CORE_CTRL_ICRESET',
		'DBG_CORE_CTRL_RESET',
		'DBG_CORE_CTRL_START',
		'DBG_CORE_CTRL_STEP',
		'DBG_CORE_CTRL_STOP',
		'DBG_CORE_GSPR_DATA',
		'DBG_CORE_GSPR_INDEX',
		'DBG_CORE_MSR',
		'DBG_CORE_NIA',
		'DBG_CORE_STAT',
		'DBG_CORE_STAT_STOPPED',
		'DBG_CORE_STAT_STOPPING',
		'DBG_CORE_STAT_TERM',
		'DBG_LOG_ADDR',
		'DBG_LOG_DATA',
		'DBG_LOG_TRIGGER',
		'DBG_WB_ADDR',
		'DBG_WB_CTRL',
		'DBG_WB_DATA'
	],
	#classInstVars : [
		'instance'
	],
	#category : #'Microwatt-JTAG'
}

{ #category : #'system startup' }
MicrowattJTAG class >> ensureNotifiedAtStartup [
	SessionManager default registerToolClassNamed: self name

]

{ #category : #'class initialization' }
MicrowattJTAG class >> initialize [
"
MicrowattJTAG initialize.

"
	self initializeConstants; ensureNotifiedAtStartup
]

{ #category : #'class initialization' }
MicrowattJTAG class >> initializeConstants [
	DBG_WB_ADDR := 16r00.
	DBG_WB_DATA := 16r01.
	DBG_WB_CTRL := 16r02.

  DBG_CORE_CTRL := 16r10.
  DBG_CORE_CTRL_STOP := (1 << 0).
  DBG_CORE_CTRL_RESET := (1 << 1).
  DBG_CORE_CTRL_ICRESET := (1 << 2).
  DBG_CORE_CTRL_STEP := (1 << 3).
  DBG_CORE_CTRL_START := (1 << 4).

  DBG_CORE_STAT := 16r11.
  DBG_CORE_STAT_STOPPING := (1 << 0).
  DBG_CORE_STAT_STOPPED := (1 << 1).
  DBG_CORE_STAT_TERM := (1 << 2).

  DBG_CORE_NIA := 16r12.
  DBG_CORE_MSR := 16r13.

  DBG_CORE_GSPR_INDEX := 16r14.
  DBG_CORE_GSPR_DATA := 16r15.

  DBG_LOG_ADDR := 16r16.
  DBG_LOG_DATA := 16r17.
  DBG_LOG_TRIGGER := 16r18.

]

{ #category : #'instance creation' }
MicrowattJTAG class >> instance [
	instance isNil ifTrue: [ instance := self basicNew connect ].
	^instance
]

{ #category : #'instance creation' }
MicrowattJTAG class >> new [
	"Please use #instance."
	self shouldNotImplement
]

{ #category : #'system startup' }
MicrowattJTAG class >> startUp: isImageStarting [
	instance := nil
]

{ #category : #private }
MicrowattJTAG >> connect [
	| rc |
	rc := LibMwDMI uniqueInstance init: 'DigilentNexysVideo' freq: 0.
	rc = 0 ifFalse: [ self error ].
]

{ #category : #API }
MicrowattJTAG >> core_msr [
	^self dmi read: DBG_CORE_MSR
]

{ #category : #API }
MicrowattJTAG >> core_nia [
	^LibMwDMI uniqueInstance dmi_read: DBG_CORE_NIA
]

{ #category : #API }
MicrowattJTAG >> core_reset [
	LibMwDMI uniqueInstance dmi_write: DBG_CORE_CTRL data: DBG_CORE_CTRL_RESET
]

{ #category : #API }
MicrowattJTAG >> core_start [
	LibMwDMI uniqueInstance dmi_write: DBG_CORE_CTRL data: DBG_CORE_CTRL_START
]

{ #category : #API }
MicrowattJTAG >> core_status [
	| status bits |
	status := LibMwDMI uniqueInstance dmi_read: DBG_CORE_STAT.
	bits := Set new.
	(status bitAnd: DBG_CORE_STAT_STOPPING) = 0 ifFalse: [ bits add: #STOPPING ].
	(status bitAnd: DBG_CORE_STAT_STOPPED ) = 0 ifFalse: [ bits add: #STOPPED  ].
	(status bitAnd: DBG_CORE_STAT_TERM    ) = 0 ifFalse: [ bits add: #TERM     ].
	^bits
]

{ #category : #API }
MicrowattJTAG >> core_step [
	(self core_status includes: #STOPPED ) ifFalse: [ self error: 'Core not stopped' ].
	LibMwDMI uniqueInstance dmi_write: DBG_CORE_CTRL data: DBG_CORE_CTRL_STEP
]

{ #category : #API }
MicrowattJTAG >> core_stop [
	LibMwDMI uniqueInstance dmi_write: DBG_CORE_CTRL data: DBG_CORE_CTRL_STOP
]

{ #category : #API }
MicrowattJTAG >> gpr_read: reg [
	((0 to: 95) includes: reg) ifFalse: [ SubscriptOutOfBounds signalFor: reg ].
	LibMwDMI uniqueInstance dmi_write: DBG_CORE_GSPR_INDEX data: reg.
	^LibMwDMI uniqueInstance dmi_read: DBG_CORE_GSPR_DATA
]

{ #category : #API }
MicrowattJTAG >> icache_reset [
	self dmi write: DBG_CORE_CTRL data: DBG_CORE_CTRL_ICRESET
]

{ #category : #API }
MicrowattJTAG >> mem64At: addr [
	^self mem_read_dword: addr
]

{ #category : #API }
MicrowattJTAG >> memAt: addr put32: dw [
	| lo hi all |
	addr \\ 4 = 0 ifFalse: [ UnalignedMemoryAccess signalFor: addr ].
	all := self mem64At: addr // 8 * 8.
	addr \\ 8 = 0
		ifTrue: [ lo := dw. hi := all bitAnd: 16rFFFFFFFF00000000 ]
		ifFalse: [ lo := all bitAnd: 16rFFFFFFFF. hi := dw << 32 ].
	^self mem_write: addr dword: lo | hi
]

{ #category : #API }
MicrowattJTAG >> memAt: addr put64: dw [
	addr \\ 8 = 0 ifFalse: [ UnalignedMemoryAccess signalFor: addr ].
	^self mem_write: addr dword: dw
]

{ #category : #private }
MicrowattJTAG >> mem_read: addr count: wordCount [
	| answer |
	LibMwDMI uniqueInstance dmi_write: DBG_WB_CTRL data: 16r7ff.
	LibMwDMI uniqueInstance dmi_write: DBG_WB_ADDR data: addr.
	answer := OrderedCollection new.
	wordCount timesRepeat: [ 
		answer addLast: (LibMwDMI uniqueInstance dmi_read: DBG_WB_DATA)
	].
	^answer
]

{ #category : #private }
MicrowattJTAG >> mem_read_dword: addr [
	^(self mem_read: addr count: 1) first
]

{ #category : #private }
MicrowattJTAG >> mem_write: addr dword: data [
	LibMwDMI uniqueInstance dmi_write: DBG_WB_CTRL data: 16r7ff.
	LibMwDMI uniqueInstance dmi_write: DBG_WB_ADDR data: addr.
	LibMwDMI uniqueInstance dmi_write: DBG_WB_DATA data: data.
]

{ #category : #API }
MicrowattJTAG >> readRegister: rname [
	^self gpr_read: (self regIndexFor: rname)
]

{ #category : #API }
MicrowattJTAG >> regIndexFor: regName [
	^(self regNames indexOf: regName) - 1
]

{ #category : #API }
MicrowattJTAG >> regNames [
	^#(
	'r0'  'r1'  'r2'  'r3'  'r4'  'r5'  'r6'  'r7'
	'r8'  'r9'  'r10' 'r11' 'r12' 'r13' 'r14' 'r15'
	'r16' 'r17' 'r18' 'r19' 'r20' 'r21' 'r22' 'r23'
	'r24' 'r25' 'r26' 'r27' 'r28' 'r29' 'r30' 'r31'
	"fast spr"
	'lr' 'ctr' 'srr0' 'srr1' 'hsrr0' 'hsrr1'
	'sprg0' 'sprg1' 'sprg2' 'sprg3'
	'hsprg0' 'hsprg1' 'xer'
	"gspr"
	'gspr45' 'gspr46' 'gspr47' 'gspr48' 'gspr49' 'gspr50' 'gspr51' 'gspr52'
	'gspr53' 'gspr54' 'gspr55' 'gspr56' 'gspr57' 'gspr58' 'gspr59' 'gspr60'
	'gspr61' 'gspr62' 'gspr63'
	"fp"
	'fpr0'  'fpr1'  'fpr2'  'fpr3'  'fpr4'  'fpr5'  'fpr6'  'fpr7'
	'fpr8'  'fpr9'  'fpr10' 'fpr11' 'fpr12' 'fpr13' 'fpr14' 'fpr15'
	'fpr16' 'fpr17' 'fpr18' 'fpr19' 'fpr20' 'fpr21' 'fpr22' 'fpr23'
	'fpr24' 'fpr25' 'fpr26' 'fpr27' 'fpr28' 'fpr29' 'fpr30' 'fpr31'	
	)
	
]
