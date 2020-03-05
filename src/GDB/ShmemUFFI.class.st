"
FFI interface to Shared Memory.
"
Class {
	#name : #ShmemUFFI,
	#superclass : #Object,
	#instVars : [
		'ptr'
	],
	#category : #'GDB-RSP'
}

{ #category : #'NB interface to SHM' }
ShmemUFFI class >> allocate: nBytes [
	| ptr |
	ptr := self shmaddr: nBytes.
	^ShmemUFFI new 
		ptr: ptr;
		yourself 
]

{ #category : #'NB interface to SHM' }
ShmemUFFI class >> shmaddr: size [
"   ShmemUFFI shmaddr: 1024000   "
	
	^ self
		ffiCall: #( void* shmaddr (int size) )
		module: self soName
]

{ #category : #'NB interface to SHM' }
ShmemUFFI class >> soName [
	^'/home/boris/work/thinshell/HelloNB.so'
]

{ #category : #accessing }
ShmemUFFI >> ptr [
	^ ptr
]

{ #category : #accessing }
ShmemUFFI >> ptr: anObject [
	ptr := anObject
]
