Class {
	#name : #MicrowattGDBTest,
	#superclass : #TestCase,
	#instVars : [
		'gdb'
	],
	#category : #'Microwatt-Tests'
}

{ #category : #components }
MicrowattGDBTest >> assertRunningThenStop [
	self assert: gdb isRunning.
	gdb stop.
	self assert: gdb isStopped.
	
]

{ #category : #components }
MicrowattGDBTest >> injectTrampoline [
	gdb writeInt32: 16r3d40ff00 toAddr: 0.  "lis r10, -256 which gives FFFFFFFF_FF000000"
	gdb writeInt32: 16r7d4803a6 toAddr: 4.  "mtlr r10"
	gdb writeInt32: 16r4e800020 toAddr: 8.  "blr"
]

{ #category : #components }
MicrowattGDBTest >> reset [
	gdb reset.
	self assert: gdb pc equals: 0.
	
]

{ #category : #components }
MicrowattGDBTest >> stepTrampoline [
	gdb s.  "eaten"
	gdb s.  "step the lis r10"
	self assert: gdb pc equals: 4.
	self assert: (gdb getRegister: 'r10') equals: 16rFFFFFFFFFF000000.
	gdb s.  "step the mtlr"
	self assert: (gdb getRegister: 'lr') equals: 16rFFFFFFFFFF000000.
	gdb s.  "jump to the firmware"
	self assert: gdb pc equals: 16rFFFFFFFFFF000000.
]

{ #category : #tests }
MicrowattGDBTest >> test1 [ 
	gdb := MicrowattGDB new.
	self
		assertRunningThenStop;
		injectTrampoline;
		reset;
		stepTrampoline.
	gdb c
	
]
