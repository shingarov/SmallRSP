Class {
	#name : #ULDcTest,
	#superclass : #TestCase,
	#pools : [
		'AddressClassEnum'
	],
	#category : #'ULDc-Tests'
}

{ #category : #tests }
ULDcTest >> testAddressForSymbolDebugger [
	"Look up the address of function by name"
	| uldc ldiResult |
	uldc := ULDc for: '/home/boris/work/FPGA/genesys-qflash/u-boot/build/u-boot'.
	ldiResult := uldc lookupSymbol: 'debugger'.
	self assert: ldiResult aclass equals: LOC_BLOCK.
	
	"now go back"
	ldiResult := uldc lookupPC: ldiResult address asInteger.
	self assert: ldiResult symbolName equals: 'debugger'
]

{ #category : #tests }
ULDcTest >> testAddressForSymbolLoopForever [
	"Look up the address of global var by name"
	| uldc ldiResult name |
	uldc := ULDc for: '/home/boris/work/FPGA/genesys-qflash/u-boot/build/u-boot'.
	ldiResult := uldc lookupSymbol: 'loop_forever'.
	self assert: ldiResult aclass equals: LOC_STATIC.
	
	"now go back; only minsym is available because it's not a code location"
	name := uldc lookupSymbolNameAt: ldiResult address asInteger.
	self assert: name equals: 'loop_forever'
]

{ #category : #tests }
ULDcTest >> testLookupPC [
	| uldc ldiResult |
	uldc := ULDc for: '/home/boris/work/FPGA/genesys-qflash/u-boot/build/u-boot'.
	ldiResult := uldc lookupPC: 16rc013400.
	self assert: (ldiResult filename_for_id endsWith: '.c')
]

{ #category : #tests }
ULDcTest >> testLookupPcFail [
	| uldc |
	uldc := ULDc for: '/home/boris/work/FPGA/genesys-qflash/u-boot/build/u-boot'.
	self should: [ uldc lookupPC: 1 ]
		raise: NotFound
]
