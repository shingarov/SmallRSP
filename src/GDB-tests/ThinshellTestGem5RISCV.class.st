Class {
	#name : #ThinshellTestGem5RISCV,
	#superclass : #ThinshellTestRISCV,
	#category : #'GDB-tests-Connected'
}

{ #category : #'target connection' }
ThinshellTestGem5RISCV >> host [
	^'localhost'
]

{ #category : #tests }
ThinshellTestGem5RISCV >> testSyscallInNZone [
	^super testSyscallInNZone
]
