Class {
	#name : #ThinshellTestGem5MIPS,
	#superclass : #ThinshellTestMIPS,
	#category : #'GDB-tests-Connected'
}

{ #category : #'target connection' }
ThinshellTestGem5MIPS >> host [
	^'localhost'
]

{ #category : #tests }
ThinshellTestGem5MIPS >> testSyscallInNZone [
	^super testSyscallInNZone
]
