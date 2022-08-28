Class {
	#name : #LibMwDMI,
	#superclass : #FFILibrary,
	#category : #'Microwatt-JTAG'
}

{ #category : #API }
LibMwDMI >> dmi_read: addr [
	| data rc |
	data := FFIExternalArray newType: 'void*' size: 1. "uint64 does not work"
	rc := self dmi_read: addr data: data.
	rc < 0 ifTrue: [ self error ].
	^data first asInteger
]

{ #category : #API }
LibMwDMI >> dmi_read: addr data: data [
	^ self ffiCall: #( int dmi_read(uint8 addr, FFIExternalArray data) )

"

LibBSCAN2 uniqueInstance 
gpr_read: 0 count: 10


core_status
	init: 'DigilentNexysVideo' freq: 0


"
]

{ #category : #API }
LibMwDMI >> dmi_write: addr data: data [
	^ self ffiCall: #( int dmi_write(uint8 addr, uint64 data) )

"

LibBSCAN2 uniqueInstance 
gpr_read: 0 count: 10


core_status
	init: 'DigilentNexysVideo' freq: 0


"
]

{ #category : #API }
LibMwDMI >> init: target freq: freq [
	^ self ffiCall: #( int bscane2_init (char * target, int freq) )
]

{ #category : #'accessing platform' }
LibMwDMI >> unixLibraryName [
	^'/home/boris/PHARO-ALL/sep-mwdebug/pharo-local/iceberg/shingarov/SmallRSP/mw_dmi/libmwdmi.so'
]
