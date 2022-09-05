Class {
	#name : #LibBSCAN2,
	#superclass : #FFILibrary,
	#category : #'Microwatt-JTAG'
}

{ #category : #API }
LibBSCAN2 >> core_status [
	^ self ffiCall: #( void core_status () )

"

LibBSCAN2 uniqueInstance
	init: 'DigilentNexysVideo' freq: 0


"
]

{ #category : #API }
LibBSCAN2 >> dmi_read: addr [
	| data rc |
	data := FFIExternalArray newType: 'void*' size: 1. "uint64 does not work"
	rc := self dmi_read: addr data: data.
	rc < 0 ifTrue: [ self error ].
	^data first asInteger
]

{ #category : #API }
LibBSCAN2 >> dmi_read: addr data: data [
	^ self ffiCall: #( int dmi_read(uint8 addr, FFIExternalArray data) )

"

LibBSCAN2 uniqueInstance 
gpr_read: 0 count: 10


core_status
	init: 'DigilentNexysVideo' freq: 0


"
]

{ #category : #API }
LibBSCAN2 >> dmi_write: addr data: data [
	^ self ffiCall: #( int dmi_write(uint8 addr, uint64 data) )

"

LibBSCAN2 uniqueInstance 
gpr_read: 0 count: 10


core_status
	init: 'DigilentNexysVideo' freq: 0


"
]

{ #category : #API }
LibBSCAN2 >> gpr_read: reg count: count [
	^ self ffiCall: #( void gpr_read (uint64 reg, uint64 count) )
]

{ #category : #API }
LibBSCAN2 >> init: target freq: freq [
	^ self ffiCall: #( int bscane2_init (char * target, int freq) )
]

{ #category : #API }
LibBSCAN2 >> mem_read: addr count: count [
	^ self ffiCall: #( void mem_read (uint64 addr, uint64 count) )

"

LibBSCAN2 uniqueInstance 
mem_read: (MicrowattJTAG new core_nia) count: 8


core_status
	init: 'DigilentNexysVideo' freq: 0


"
]

{ #category : #API }
LibBSCAN2 >> status [
"

This is the higher-level one in C.

LibBSCAN2 uniqueInstance core_status


"
]

{ #category : #'accessing platform' }
LibBSCAN2 >> unixLibraryName [
	^'/home/boris/work/FPGA/uw2022/microwatt/scripts/mw_debug/libmwdbg.so'
]
