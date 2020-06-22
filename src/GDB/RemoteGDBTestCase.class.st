"
I am deprecated leftover from TA-MT.

"
Class {
	#name : #RemoteGDBTestCase,
	#superclass : #TestCase,
	#instVars : [
		'gdb',
		'pdl'
	],
	#category : #'GDB-Doodles'
}

{ #category : #'as yet unclassified' }
RemoteGDBTestCase >> connectGDB [
	pdl := AcProcessorDescriptions powerpc.
	gdb := RemoteGDBSession
		host: '192.168.75.2'
		port: 7000
		processorDescription: pdl.
]

{ #category : #'syscall sequences' }
RemoteGDBTestCase >> exitSyscall: rtnInt [
	| loadSyscallNo loadReturnValue sc |
	loadSyscallNo := (pdl instructionAt: #addi) bind: (Dictionary new
		at: 'ra' put: 0;
		at: 'rt' put: 0;
		at: 'd' put: 1;
		yourself).
	loadReturnValue := (pdl instructionAt: #addi) bind: (Dictionary new
		at: 'ra' put: 0;
		at: 'rt' put: 3;
		at: 'd' put: rtnInt;
		yourself).
	sc := (pdl instructionAt: #sc) bind: (Dictionary new
		at: 'lev' put: 0;
		yourself).
	^(Array with: loadSyscallNo with: loadReturnValue with: sc)
	collect: [ :instr | instr emit ]
]

{ #category : #'syscall sequences' }
RemoteGDBTestCase >> messageBytes [
	^'HELLO!' asByteArray, #(10 0)
]

{ #category : #'as yet unclassified' }
RemoteGDBTestCase >> testManualSyscallInNZone [
	| memLayout writeInstructions exitInstructions |
	self halt.
	self connectGDB.
	memLayout := ThinshellAddressLayout gdb: gdb.
	memLayout executeStartChain.
	writeInstructions := self writeSyscall: self messageBytes.
	exitInstructions := self exitSyscall: 1.
	gdb writeInt32s: writeInstructions, exitInstructions toAddr: memLayout nZone.
	gdb writeBytes: self messageBytes toAddr: memLayout heap.
	
	gdb
		stepUntil: [ gdb currentInstruction name = 'sc' ];
		s.   "the actual write syscall"
	
	gdb s; s; s. "exit"
	Transcript yourself
	
]

{ #category : #'as yet unclassified' }
RemoteGDBTestCase >> testSetRegisters [

	| regs1 regs2 |
	self halt.
	regs1 := gdb getRegisters.
	gdb setRegisters: regs1.
	regs2 := gdb getRegisters.
	self assert: regs1 = regs2.
]

{ #category : #'syscall sequences' }
RemoteGDBTestCase >> writeSyscall: aByteArray [
	| loadSyscallNo loadFD loadBuf loadLength sc |
	loadSyscallNo := (pdl instructionAt: #addi) bind: (Dictionary new
		at: 'ra' put: 0;
		at: 'rt' put: 0;
		at: 'd' put: 4;
		yourself).
	loadFD := (pdl instructionAt: #addi) bind: (Dictionary new
		at: 'ra' put: 0;
		at: 'rt' put: 3;
		at: 'd' put: 1;
		yourself).
	loadBuf := (pdl instructionAt: #addi) bind: (Dictionary new
		at: 'ra' put: 17;
		at: 'rt' put: 4;
		at: 'd' put: 0;
		yourself).
	loadLength := (pdl instructionAt: #addi) bind: (Dictionary new
		at: 'ra' put: 0;
		at: 'rt' put: 5;
		at: 'd' put: (aByteArray size);
		yourself).
	sc := (pdl instructionAt: #sc) bind: (Dictionary new
		at: 'lev' put: 0;
		yourself).
	^(Array with: loadSyscallNo with: loadFD with: loadBuf with: loadLength with: sc)
	collect: [ :instr | instr emit ]
]
