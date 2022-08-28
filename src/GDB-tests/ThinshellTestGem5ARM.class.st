Class {
	#name : #ThinshellTestGem5ARM,
	#superclass : #ThinshellTestARM,
	#category : #'GDB-tests-Connected'
}

{ #category : #'target connection' }
ThinshellTestGem5ARM >> host [
	^'localhost'
]

{ #category : #tests }
ThinshellTestGem5ARM >> testSyscallInNZone [
	^super testSyscallInNZone
]
