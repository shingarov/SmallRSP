Class {
	#name : #ThinshellTestBarcelona,
	#superclass : #ThinshellTestRISCV,
	#category : #'GDB-tests-Connected'
}

{ #category : #'target connection' }
ThinshellTestBarcelona >> host [
	^'barcelona'
]

{ #category : #tests }
ThinshellTestBarcelona >> testSyscallInNZone [
	^super testSyscallInNZone
]
