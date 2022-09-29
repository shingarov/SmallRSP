Class {
	#name : #LibULDc,
	#superclass : #FFILibrary,
	#category : #ULDc
}

{ #category : #API }
LibULDc >> ULDc_init: fileName [
	^ self ffiCall: #( int ULDc_init (char * fileName) )
]

{ #category : #API }
LibULDc >> ULDc_lookupPC: pc _: result [
	^ self ffiCall: #( int ULDc_lookupPC (void* pc, ULDc_LDIResult* result) )
]

{ #category : #API }
LibULDc >> ULDc_lookup_name: addr _: section [
	^ self ffiCall: #( char* ULDc_lookup_name (void* addr, void* section) )
]

{ #category : #API }
LibULDc >> ULDc_symbol: s _: result [
	^ self ffiCall: #( int ULDc_symbol (char* s, ULDc_LDIResult* result) )
]

{ #category : #'accessing platform' }
LibULDc >> unixLibraryName [
	^'/home/boris/binutils-gdb/gdb/libuldc.so'
]
