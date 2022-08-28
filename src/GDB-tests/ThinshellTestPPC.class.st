Class {
	#name : #ThinshellTestPPC,
	#superclass : #ThinshellTest,
	#category : #'GDB-tests-Connected'
}

{ #category : #'as yet unclassified' }
ThinshellTestPPC class >> architectureName [
	^'powerpc'
]

{ #category : #'syscall sequences' }
ThinshellTestPPC >> exitSyscall [
	^ gdb processorDescription assembleLines: '
# syscall #1 -> r0
li 0,1
# exit code -> r3
li 3,42

sc
'
]

{ #category : #'syscall sequences' }
ThinshellTestPPC >> writeSyscall [
	^ gdb processorDescription assembleLines: '
# syscall #4 -> r0
li 0,4
# FD stdout -> r3
li 3,1
# rHeap -> r4
addi 4,17,0
# |msg| -> r5
li 5,8

sc
'
]
