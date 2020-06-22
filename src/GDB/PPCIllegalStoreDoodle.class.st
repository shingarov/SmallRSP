Class {
	#name : #PPCIllegalStoreDoodle,
	#superclass : #PPCgem5ThinshellDoodle,
	#instVars : [
		'isHardware'
	],
	#category : #'GDB-Doodles'
}

{ #category : #'instance creation' }
PPCIllegalStoreDoodle class >> onHardware [
	"Set up the test for the real devboard."
	^self basicNew isHardware: true
]

{ #category : #'instance creation' }
PPCIllegalStoreDoodle class >> onSoftware [
	"Set up the test for gem5."
	^self basicNew isHardware: false
]

{ #category : #'target connection' }
PPCIllegalStoreDoodle >> hostIP [
	^isHardware
		ifTrue: [ '192.168.75.199' ]
		ifFalse: [ '192.168.75.2' ]
]

{ #category : #'as yet unclassified' }
PPCIllegalStoreDoodle >> isHardware: whetherToUseP1025 [
	isHardware := whetherToUseP1025.
	^self
]

{ #category : #'target connection' }
PPCIllegalStoreDoodle >> pdl [
	^isHardware
		ifTrue: [ FakeProcessorDescriptionP1025 new ]
		ifFalse: [ FakeProcessorDescriptionPPC new ]
]

{ #category : #'as yet unclassified' }
PPCIllegalStoreDoodle >> stepThroughThinshellPreamble [

	self assert: memory currentInstructionEncoding equals: 16r3920ffff. "li r9,-1"
self halt.
	gdb c.
	self assert: memory currentInstructionEncoding equals: 16r90090000. "stw r0,0(r9)"
self halt.
	gdb s. "Simulation dies"

]

{ #category : #'as yet unclassified' }
PPCIllegalStoreDoodle >> testSTW [
	"
	PPCIllegalStoreDoodle onSoftware testSTW.
	
	PPCIllegalStoreDoodle onHardware testSTW.
	"
	self connectGdb;
		stepThroughThinshellPreamble
]
