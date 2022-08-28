Class {
	#name : #ThinshellTestGem5PPC,
	#superclass : #ThinshellTestPPC,
	#category : #'GDB-tests-Connected'
}

{ #category : #'target connection' }
ThinshellTestGem5PPC >> host [
	^'localhost'
]

{ #category : #tests }
ThinshellTestGem5PPC >> testHwBreakpoint [
	^super testHwBreakpoint
]

{ #category : #tests }
ThinshellTestGem5PPC >> testIllegalStore [
	^super testIllegalStore
]

{ #category : #tests }
ThinshellTestGem5PPC >> testSyscallInNZone [
	^super testSyscallInNZone
]
