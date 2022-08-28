Class {
	#name : #ThinshellTestMIPS,
	#superclass : #ThinshellTest,
	#category : #'GDB-tests-Connected'
}

{ #category : #running }
ThinshellTestMIPS >> adjustProcessorDescription [
	"The endianness specified in the PDL, is BE.
	 However, gem5 simulates Little-Endian MIPS."
	gdb processorDescription endian: LittleEndian uniqueInstance
]

{ #category : #'syscall sequences' }
ThinshellTestMIPS >> exitSyscall [
	^ gdb processorDescription assembleLines: '
# SYS_exit -> v0
ori $v0, $zero, 4001
# exit code
ori $a0, $zero, 42

syscall
'
]

{ #category : #'syscall sequences' }
ThinshellTestMIPS >> writeSyscall [
	^ gdb processorDescription assembleLines: '
# SYS_write -> v0
ori $v0, $zero, 4004
# FD = stdout
ori $a0,$zero, 1
# rHeap points to the string
addi $a1, $17, 0
# |msg| -> r5
ori $a2, $zero, 8

syscall
'
]
