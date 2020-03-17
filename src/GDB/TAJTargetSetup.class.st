Class {
	#name : #TAJTargetSetup,
	#superclass : #Object,
	#instVars : [
		'gdbClass',
		'host',
		'port',
		'architecture',
		'processor',
		'magicInstruction',
		'instructionStreamClass',
		'breakpointKind'
	],
	#category : #'GDB-TAJ'
}

{ #category : #settings }
TAJTargetSetup class >> current [
	^self gem5PPC   
]

{ #category : #settings }
TAJTargetSetup class >> gem5MIPS [
	| arch |
	arch := self tajMIPSArchitecture.
	^self new
		gdbClass: Gem5SharedRAM8K;
		host: '192.168.75.2';
		port: 7000;
		architecture: arch;
		instructionStreamClass: TargetAgnosticInstructionStream;
		breakpointKind: #hard;
		magicInstruction: (((arch isa instructionAt: #sll) bind: (Dictionary new
			at: 'rs' put: 0;
			at: 'rt' put: 0;
			at: 'rd' put: 0;
			at: 'shamt' put: 0;
			yourself)) emit);
		yourself
]

{ #category : #settings }
TAJTargetSetup class >> gem5PPC [
	| arch |
	arch := self tajPOWERArchitecture.
	^self new
		gdbClass: Gem5SharedRAM;
		host: '127.0.0.1';
		port: 7000;
		architecture: arch;
		instructionStreamClass: TargetAgnosticInstructionStream;
		breakpointKind: #hard;
		magicInstruction: (((arch isa instructionAt: #ore) bind: (Dictionary new
			at: 'ra' put: 1;
			at: 'rb' put: 1;
			at: 'rs' put: 1;
			at: 'rc' put: 0;
			yourself)) emit);
		yourself
]

{ #category : #settings }
TAJTargetSetup class >> gem86 [
	| arch |
	arch := self tajIA32Architecture.
	^self new
		gdbClass: BufferingRemoteRAM;
		host: '192.168.75.2';
		port: 7000;
		architecture: arch;
		instructionStreamClass: TargetAgnosticInstructionStream;
		
		"bogus, I still don't what it should be on x86"
		magicInstruction: 16r0;
		breakpointKind: #trap;
		yourself
]

{ #category : #settings }
TAJTargetSetup class >> mpc5125 [
	| arch |
	arch := self tajPOWERArchitecture.
	^self new
		gdbClass: BufferingRemoteRAM;
		host: '192.168.75.39';
		port: 7000;
		architecture: arch;
		instructionStreamClass: TargetAgnosticInstructionStream;
		
		"twge r2,r2; see gdb/gdbserver/linux-ppc-low.c.
		ptrace will happily send SIGTRAP for any tw,
		but gdb's breakpoint_at() will be confused unless
		the memory contents are exactly what it thinks
		the SW break instruction is."
		magicInstruction: 16r7D821008;
		breakpointKind: #trap;
		yourself
]

{ #category : #settings }
TAJTargetSetup class >> p1025 [
	"Freescale TWR-P1025 PowerPC e500v2"
	| arch |
	arch := self tajPOWERArchitecture.
	^self new
		gdbClass: BufferingRemoteRAM;
		host: '192.168.75.199';
		port: 7000;
		architecture: arch;
		instructionStreamClass: TargetAgnosticInstructionStream;
		
		"twge r2,r2; see gdb/gdbserver/linux-ppc-low.c.
		ptrace will happily send SIGTRAP for any tw,
		but gdb's breakpoint_at() will be confused unless
		the memory contents are exactly what it thinks
		the SW break instruction is."
		magicInstruction: 16r7D821008;
		breakpointKind: #trap;
		yourself
]

{ #category : #TAJArchitectures }
TAJTargetSetup class >> tajIA32Architecture [
	^"TAJIA32Architecture"0 resetDefault
]

{ #category : #TAJArchitectures }
TAJTargetSetup class >> tajMIPSArchitecture [
	^"TAJMIPSArchitecture"0 resetDefault
]

{ #category : #TAJArchitectures }
TAJTargetSetup class >> tajPOWERArchitecture [
	^"TAJPowerArchitecture"0 resetDefault
]

{ #category : #accessing }
TAJTargetSetup >> architecture [
	^ architecture
]

{ #category : #accessing }
TAJTargetSetup >> architecture: anObject [
	architecture := anObject
]

{ #category : #accessing }
TAJTargetSetup >> breakpointKind [
	^ breakpointKind
]

{ #category : #accessing }
TAJTargetSetup >> breakpointKind: anObject [
	breakpointKind := anObject
]

{ #category : #accessing }
TAJTargetSetup >> gdbClass [
	^ gdbClass
]

{ #category : #accessing }
TAJTargetSetup >> gdbClass: anObject [
	gdbClass := anObject
]

{ #category : #accessing }
TAJTargetSetup >> host [
	^ host
]

{ #category : #accessing }
TAJTargetSetup >> host: anObject [
	host := anObject
]

{ #category : #accessing }
TAJTargetSetup >> instructionStreamClass [
	^ instructionStreamClass
]

{ #category : #accessing }
TAJTargetSetup >> instructionStreamClass: anObject [
	instructionStreamClass := anObject
]

{ #category : #accessing }
TAJTargetSetup >> magicInstruction [
	^ magicInstruction
]

{ #category : #accessing }
TAJTargetSetup >> magicInstruction: anObject [
	magicInstruction := anObject
]

{ #category : #accessing }
TAJTargetSetup >> port [
	^ port
]

{ #category : #accessing }
TAJTargetSetup >> port: anObject [
	port := anObject
]
