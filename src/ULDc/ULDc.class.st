Class {
	#name : #ULDc,
	#superclass : #Object,
	#instVars : [
		'addressCache'
	],
	#classInstVars : [
		'instance'
	],
	#category : #ULDc
}

{ #category : #'class initialization' }
ULDc class >> ensureNotifiedAtStartup [
	SessionManager default registerToolClassNamed: self name
]

{ #category : #'instance creation' }
ULDc class >> for: filename [
"BOGUS: to be able to switch between programs
without restarting Smalltalk,
this should do symbol_file_clear() but NOT initGDB because it will segfault."

	instance isNil ifTrue: [ instance := self basicNew initGdbFor: filename ].
	^instance
]

{ #category : #'class initialization' }
ULDc class >> initialize [
"
ULDc initialize.

"
	self ensureNotifiedAtStartup
]

{ #category : #'instance creation' }
ULDc class >> instance [
	instance isNil ifTrue: [ instance := self basicNew initGDB ].
	^instance
]

{ #category : #'instance creation' }
ULDc class >> new [
	"Please use #for:."
	self shouldNotImplement
]

{ #category : #'system startup' }
ULDc class >> startUp [ 
	instance := nil
]

{ #category : #accessing }
ULDc >> addressCache [
	addressCache isNil ifTrue: [ addressCache := Dictionary new ].
	^addressCache
]

{ #category : #API }
ULDc >> askGdbAbout: addr [
	"Ask libgdb.a what it knows about the source corresponding to address addr.
	 addr can be an Integer or an ExternalAddress."
	| ldiResult rc |
	ldiResult := ULDc_LDIResult new.
	rc := LibULDc uniqueInstance ULDc_lookupPC: addr asExternalAddress _: ldiResult.
	rc = -1 ifTrue: [ ^NotFound signalFor: addr ].
	rc = 0 ifFalse: [ self error ].
	^ldiResult
]

{ #category : #initialization }
ULDc >> initGdbFor: filename [
	LibULDc uniqueInstance ULDc_init: filename.
	^self
]

{ #category : #API }
ULDc >> lookupPC: addr [
	^self addressCache anyAt: addr ifAbsent: [
		| ldiResult pcInterval |
		ldiResult := self askGdbAbout: addr.
		pcInterval := ldiResult startPC asInteger to: ldiResult endPC asInteger.
		self addressCache at: pcInterval put: ldiResult.
		^ldiResult ]
]

{ #category : #API }
ULDc >> lookupSymbol: aString [ 
	| ldiResult rc |
	ldiResult := ULDc_LDIResult new.
	rc := LibULDc uniqueInstance ULDc_symbol: aString _: ldiResult.
	rc = 0 ifFalse: [ self error ].
	^ldiResult
]

{ #category : #API }
ULDc >> lookupSymbolNameAt: addr [ 
	^LibULDc uniqueInstance ULDc_lookup_name: addr asExternalAddress _: ExternalAddress null
]
