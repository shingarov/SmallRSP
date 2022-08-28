Class {
	#name : #ThinshellTestRISCV,
	#superclass : #ThinshellTest,
	#category : #'GDB-tests-Connected'
}

{ #category : #'syscall sequences' }
ThinshellTestRISCV >> exitSyscall [
	^ gdb processorDescription assembleLines: '
addi a0, zero, 42
# syscall #93
addi a7, zero, 93

ecall
'
]

{ #category : #'syscall sequences' }
ThinshellTestRISCV >> writeSyscall [
	^ gdb processorDescription assembleLines: '
# FD stdout
addi a0, zero, 1
# rHeap -> r4
add a1, zero, x17
# |msg| -> a2
addi a2, zero, 8
addi a3, zero, 0
# syscall #64
addi a7, zero, 64

ecall
'
]
