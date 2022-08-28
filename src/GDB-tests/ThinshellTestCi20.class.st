Class {
	#name : #ThinshellTestCi20,
	#superclass : #ThinshellTestMIPS,
	#category : #'GDB-tests-Connected'
}

{ #category : #running }
ThinshellTestCi20 >> host [
	^'ci20'
]

{ #category : #tests }
ThinshellTestCi20 >> testSyscallInNZone [
	^super testSyscallInNZone
]
