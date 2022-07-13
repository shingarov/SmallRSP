Class {
	#name : #AMD64GDBTest,
	#superclass : #SimpleRspTest,
	#category : #'GDB-Tests'
}

{ #category : #'target connection' }
AMD64GDBTest >> hostIP [
	^'parmenides'
]

{ #category : #tests }
AMD64GDBTest >> testConnect [
	| initialPC nextPC |
	self connectGdb; createRAM.
	initialPC := gdb pc.
	gdb s.
	nextPC := gdb pc
]
