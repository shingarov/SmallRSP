Class {
	#name : #LibUrJTAG,
	#superclass : #FFILibrary,
	#category : #UrJTAG
}

{ #category : #'accessing platform' }
LibUrJTAG >> unixLibraryName [
	^'/home/boris/work/FPGA/uw2022/microwatt/scripts/mw_debug/libmwdbg.so'
]

{ #category : #API }
LibUrJTAG >> urj_error_describe [
	^ self ffiCall: #( char* urj_error_describe() )
]

{ #category : #API }
LibUrJTAG >> urj_part_data_register_define: part _: name _: len [
	^ self ffiCall: #( int urj_part_data_register_define( URJ_PART* part, char* name, int len ) )
]

{ #category : #API }
LibUrJTAG >> urj_part_instruction_define: part _: instruction _: code _: data_register [
	^ self ffiCall: #( void* urj_part_instruction_define( URJ_PART* part, char* instruction, char* code, char* data_register ) )	
]

{ #category : #API }
LibUrJTAG >> urj_part_parts_set_instruction: ps _: iname [
	^ self ffiCall: #( int urj_part_parts_set_instruction( URJ_PARTS* ps, char* iname ) )	
]

{ #category : #API }
LibUrJTAG >> urj_part_set_instruction: p _: iname [
	^ self ffiCall: #( void urj_part_set_instruction( URJ_PART* p, char* iname ) )
]

{ #category : #API }
LibUrJTAG >> urj_tap_chain_active_part: chain [ 
	^ self ffiCall: #( void* urj_tap_chain_active_part( void* chain ) )
]

{ #category : #API }
LibUrJTAG >> urj_tap_chain_alloc [
	^ self ffiCall: #( void* urj_tap_chain_alloc() )
]

{ #category : #API }
LibUrJTAG >> urj_tap_chain_connect: jc _: cable _: params [
	^ self ffiCall: #( int urj_tap_chain_connect( void* jc, char* cable, FFIExternalArray params ) )
]

{ #category : #API }
LibUrJTAG >> urj_tap_chain_shift_data_registers: chain _: capture_output [ 
	^ self ffiCall: #( int urj_tap_chain_shift_data_registers( void* chain, int capture_output ) )

]

{ #category : #API }
LibUrJTAG >> urj_tap_chain_shift_instructions: chain [ 
	^ self ffiCall: #( int urj_tap_chain_shift_instructions( void* chain ) )

]

{ #category : #API }
LibUrJTAG >> urj_tap_manual_add: chain _: instr_len [
	^ self ffiCall: #( int urj_tap_manual_add( void* chain, int instr_len ) )
]

{ #category : #API }
LibUrJTAG >> urj_tap_register_get_value: tr [ 
	^ self ffiCall: #( void* urj_tap_register_get_value( URJ_TAP_REGISTER* tr ) )
]

{ #category : #API }
LibUrJTAG >> urj_tap_register_get_value_bit_range: tr _: msb _: lsb [
	^ (self ffiCall: #( void* urj_tap_register_get_value_bit_range( URJ_TAP_REGISTER* tr, int msb, int lsb ) )) asInteger
]

{ #category : #API }
LibUrJTAG >> urj_tap_register_set_value_bit_range: tr _: val _: msb _: lsb [ 
	^ self ffiCall: #( int urj_tap_register_set_value_bit_range( URJ_TAP_REGISTER* tr, void* val, int msb, int lsb ) )	
]
