Class {
	#name : #AbsoluteGem5TestCase,
	#superclass : #TestCase,
	#instVars : [
		'gdb'
	],
	#category : #'GDB-Tests'
}

{ #category : #'as yet unclassified' }
AbsoluteGem5TestCase >> connectGdbWith: processorDescription [
	gdb := RemoteGDBSession
		host: '192.168.75.2'
		port: 7000
		processorDescription: processorDescription
]

{ #category : #tests }
AbsoluteGem5TestCase >> makeAFewSteps [
	self assert: gdb pc equals: 0.
	self assert: gdb currentInstructionEncoding equals: 16r7c631a78.
	gdb s.
	self assert: gdb pc equals: 4.
	self assert: gdb currentInstructionEncoding equals: 16r3860002a.
	gdb s.
	self assert: gdb pc equals: 8.
	self should: [ gdb c ] raise: GdbChildExited 
]

{ #category : #tests }
AbsoluteGem5TestCase >> testWithFakePDL [
	self connectGdbWith: FakeProcessorDescriptionPPC new.
	self makeAFewSteps 
]

{ #category : #tests }
AbsoluteGem5TestCase >> testWithRealPDL [
	self connectGdbWith: AcProcessorDescriptions powerpc.
	self makeAFewSteps 
]
