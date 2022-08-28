Class {
	#name : #ThinshellTestARM,
	#superclass : #ThinshellTest,
	#category : #'GDB-tests-Connected'
}

{ #category : #'syscall sequences' }
ThinshellTestARM >> exitSyscall [
	^ gdb processorDescription assembleLines: '
# syscall #1
mov r7, #1
# exit code -> r0
mov r0, #42

swi 0
'
]

{ #category : #'syscall sequences' }
ThinshellTestARM >> writeSyscall [
	^ gdb processorDescription assembleLines: '
# syscall #4
mov r7, #4
# stdout = fd=1
mov r0, #1
# ptr to string in r1
# HEAP already in r1 on ARM
# length in r2
mov r2, #7

swi 0
'
]
