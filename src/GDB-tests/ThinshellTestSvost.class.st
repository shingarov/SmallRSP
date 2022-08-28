Class {
	#name : #ThinshellTestSvost,
	#superclass : #ThinshellTestPPC,
	#category : #'GDB-tests-Connected'
}

{ #category : #'target connection' }
ThinshellTestSvost >> host [
	^'svost'
]

{ #category : #tests }
ThinshellTestSvost >> testIllegalStore [
	^super testIllegalStore
]

{ #category : #tests }
ThinshellTestSvost >> testSyscallInNZone [
	^super testSyscallInNZone
]
