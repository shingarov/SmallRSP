Class {
	#name : #RISCV64GDBTest,
	#superclass : #SimpleRspTest,
	#category : #'GDB-Tests'
}

{ #category : #'target IP' }
RISCV64GDBTest >> barcelona [
	^'192.168.75.14'
]

{ #category : #'target IP' }
RISCV64GDBTest >> c910 [
	^'192.168.75.18'
]

{ #category : #'target IP' }
RISCV64GDBTest >> hostIP [
	^self localhost
]

{ #category : #'target IP' }
RISCV64GDBTest >> localhost [
	^'127.0.0.1'
]

{ #category : #tests }
RISCV64GDBTest >> testConnect [
	self connectGdb; createRAM; advancePC.
	gdb kill.
]
