"
I am the abstract superclass of all tests that assume the target is running a thinshell,
i.e. has an empty nZone which we fill with instructions, jump to it, and see what happens.
"
Class {
	#name : #ThinshellTest,
	#superclass : #RemoteGDBTest,
	#category : #'GDB-tests-Connected'
}

{ #category : #running }
ThinshellTest >> checkProcessorDescription [
	"The concrete subclasses each know which processor they are for.
	 For example, ThinshellTestGem5PPC knows the target POWER;
	 However, one can bring up gem5 with a MIPS payload and try to
	 connect ThinshellTestGem5PPC to it.
	 This check guards against that situation."
	self assert: gdb processorDescription architectureName
		equals: self class architectureName
]

{ #category : #'target connection' }
ThinshellTest >> disconnectGDB [
	"Taken care of in the test"
]

{ #category : #'target connection' }
ThinshellTest >> gdbClass [
	^RemoteGDB
]

{ #category : #'test data' }
ThinshellTest >> messageBytes [
	^'HELLO!' asByteArray, #(10 0)
]

{ #category : #tests }
ThinshellTest >> testHwBreakpoint [
	"For guinea-pig, use the Modtalk thinshell."
	| initialPC bkptPC debugStopped |
	initialPC := gdb pc.
	bkptPC := initialPC + 8.
	gdb insertHWBreakpointAt: bkptPC.
	debugStopped := gdb c.
	self assert: gdb pc equals: bkptPC.
	gdb kill
]

{ #category : #tests }
ThinshellTest >> testIllegalStore [
	| debugStopped |
	debugStopped := gdb c.
	self assert: debugStopped signal equals: 'SIGSEGV'
]

{ #category : #tests }
ThinshellTest >> testSyscallInNZone [
	| memLayout writeInstructions exitInstructions |
	memLayout := ModtalkThinshellAddressSpace gdb: gdb.
	writeInstructions := self writeSyscall.
	exitInstructions := self exitSyscall.
	gdb writeInt32s: writeInstructions, exitInstructions toAddr: memLayout nZone.
	gdb writeBytes: self messageBytes toAddr: memLayout heap.
	self should: [ gdb c ] raise: InferiorExited
]
