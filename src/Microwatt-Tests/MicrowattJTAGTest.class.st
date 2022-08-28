"
A MicrowattJTAGTest is a test class for testing the behavior of MicrowattJTAG
"
Class {
	#name : #MicrowattJTAGTest,
	#superclass : #TestCase,
	#category : #'Microwatt-Tests'
}

{ #category : #helpers }
MicrowattJTAGTest >> assertRunningThenStop [
	"Start from newly-powered-up Arty."
	self assert: MicrowattJTAG instance core_status isEmpty. "0 flags = RUNNING"
	MicrowattJTAG instance core_stop.
	self assert: (MicrowattJTAG instance core_status includes: #STOPPED)
]

{ #category : #tests }
MicrowattJTAGTest >> testFromGround [
	self assertRunningThenStop.
	MicrowattJTAG instance memAt: 0  put32: 16r3d40ff00. "lis r10, -256 which gives FFFFFFFF_FF000000"
	MicrowattJTAG instance memAt: 4  put32: 16r7d4803a6. "mtlr r10"
	MicrowattJTAG instance memAt: 8  put32: 16r4e800020. "blr"
	
	MicrowattJTAG instance core_reset.
	self assert: MicrowattJTAG instance core_nia equals: 0.
	MicrowattJTAG instance core_step.
	MicrowattJTAG instance core_step.
	self assert: MicrowattJTAG instance core_nia equals: 4.
	self assert: (MicrowattJTAG instance gpr_read: 10) equals: 16rFFFFFFFFFF000000.
	MicrowattJTAG instance core_step.
	self assert: MicrowattJTAG instance core_nia equals: 8.
	self assert: (MicrowattJTAG instance gpr_read: 32) equals: 16rFFFFFFFFFF000000.
	MicrowattJTAG instance core_step.
	self assert: MicrowattJTAG instance core_nia equals: 16rFFFFFFFFFF000000.
	MicrowattJTAG instance core_start
]

{ #category : #'aux tests' }
MicrowattJTAGTest >> testUnaligned64 [
	self
		should: [ MicrowattJTAG instance memAt: 5 put64: 0 ]
		raise: UnalignedMemoryAccess
]
