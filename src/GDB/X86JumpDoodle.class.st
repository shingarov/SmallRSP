Class {
	#name : #X86JumpDoodle,
	#superclass : #GDBDoodle,
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
X86JumpDoodle >> hostIP [
	^'192.168.75.2'
]

{ #category : #'target connection' }
X86JumpDoodle >> pdl [
	^FakeProcessorDescriptionX86 new
]

{ #category : #'target connection' }
X86JumpDoodle >> tcpPort [
	^7000
]

{ #category : #tests }
X86JumpDoodle >> testAtomicSimpleCPU [
	"
	X86JumpDoodle new testAtomicSimpleCPU
	"
	self halt; connectGdb.
	gdb pc: 17.
	gdb s.
	gdb pc.
	gdb getRegisters at: 'eax'.
	gdb getRegisters at: 'ebp'.
]
