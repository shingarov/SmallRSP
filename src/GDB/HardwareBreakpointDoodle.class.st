Class {
	#name : #HardwareBreakpointDoodle,
	#superclass : #ThinshellDoodle,
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
HardwareBreakpointDoodle >> hostIP [
	^'192.168.75.2'

]

{ #category : #tests }
HardwareBreakpointDoodle >> initialPC [
	^16r10000000
]

{ #category : #tests }
HardwareBreakpointDoodle >> installBrk [
	gdb insertHWBreakpointAt: self initialPC + 15 "3 loads"
]

{ #category : #'target connection' }
HardwareBreakpointDoodle >> pdl [
	^FakeProcessorDescriptionX86 new

]

{ #category : #'target connection' }
HardwareBreakpointDoodle >> tcpPort [
	^7000

]

{ #category : #tests }
HardwareBreakpointDoodle >> testHWBrk [
	"
	HardwareBreakpointDoodle new halt; testHWBrk.
	"
	| regs |
	self connectGdb;
		installBrk.
	regs := gdb getRegisters.
	self assert: (regs at: 'eip') = self initialPC.
	self assert: (regs at: 'eax') ~= 16r12345678.
	self assert: (regs at: 'ebx') ~= 16r12345678.
	self assert: (regs at: 'ecx') ~= 16r12345678.
	self assert: (regs at: 'edx') ~= 16r12345678.
	
	gdb c.
	regs := gdb getRegisters.
	self assert: (regs at: 'eip') = (self initialPC + 15).
	self assert: (regs at: 'eax') = 16r12345678.
	self assert: (regs at: 'ebx') = 16r12345678.
	self assert: (regs at: 'ecx') = 16r12345678.
	self assert: (regs at: 'edx') ~= 16r12345678.

	[ gdb kill ] on: GdbChildExited do: [ ^self ].
	self error: 'gdbserver did not disconnect'
]
