Class {
	#name : #LibUrJTAG,
	#superclass : #FFILibrary,
	#category : #'Microwatt-JTAG'
}

{ #category : #'accessing platform' }
LibUrJTAG >> unixLibraryName [
	^'/home/boris/work/FPGA/uw2022/microwatt/scripts/mw_debug/libmwdbg.so'
]

{ #category : #API }
LibUrJTAG >> urj_error_describe [
	^ self ffiCall: #( char* urj_error_describe() )
]

{ #category : #'as yet unclassified' }
LibUrJTAG >> urj_part_parts_set_instruction: ps _: iname [
	^ self ffiCall: #( int urj_part_parts_set_instruction( URJ_PARTS* ps, char* iname ) )	
]

{ #category : #'as yet unclassified' }
LibUrJTAG >> urj_tap_chain_active_part: chain [ 
	^ self ffiCall: #( void* urj_tap_chain_active_part( URJ_CHAIN* chain ) )
]

{ #category : #API }
LibUrJTAG >> urj_tap_chain_alloc [
	^ self ffiCall: #( URJ_CHAIN* urj_tap_chain_alloc() )

"

LibUrJTAG uniqueInstance
urj_tap_chain_alloc
"
]

{ #category : #API }
LibUrJTAG >> urj_tap_chain_connect: jc _: cable _: params [
	^ self ffiCall: #( int urj_tap_chain_connect( URJ_CHAIN* jc, char* cable, FFIExternalArray params ) )

"

LibUrJTAG uniqueInstance
urj_tap_chain_alloc
"
]

{ #category : #API }
LibUrJTAG >> urj_tap_manual_add: chain _: instr_len [
	^ self ffiCall: #( int urj_tap_manual_add( URJ_CHAIN* chain, int instr_len ) )
]

{ #category : #API }
LibUrJTAG >> xxxx [
	| jc rc p |
	jc := self urj_tap_chain_alloc.
	jc connectTo: 'DigilentNexysVideo'.

	rc := LibUrJTAG uniqueInstance urj_tap_manual_add: jc _: 6.
	rc < 0 ifTrue: [ self error: 'JTAG failed to add part' ].
	jc parts len = 0 ifTrue: [ self error: 'JTAG Something''s wrong after adding part' ].
	
	LibUrJTAG uniqueInstance urj_part_parts_set_instruction: jc parts _: 'BYPASS'.
	jc active_part: 0.
	
	p := LibUrJTAG uniqueInstance urj_tap_chain_active_part: jc.
	
	
	self halt.

"
LibUrJTAG uniqueInstance xxxx
"
]
