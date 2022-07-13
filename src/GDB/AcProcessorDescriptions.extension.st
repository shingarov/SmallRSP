Extension { #name : #AcProcessorDescriptions }

{ #category : #'*GDB' }
AcProcessorDescriptions class >> gdbArchName: aString [
	^self perform: (self knownGdbArchNames at: aString)
]

{ #category : #'*GDB' }
AcProcessorDescriptions class >> knownGdbArchNames [
	^Dictionary new
		at: 'powerpc' put: #powerpc;
		at: 'powerpc:e500' put: #powerpc;
		at: 'powerpc:common' put: #powerpc;
		at: 'powerpc:common64' put: #powerpc;

		at: 'riscv:rv64id' put: #riscv64;
		at: 'riscv' put: #riscv64;
		
		at: 'arm' put: #armv5;
		at: 'aarch64' put: #aarch64;

		at: 'mips' put: #mips;
		
		at: 'i386:x86-64' put: #amd64;
		
	yourself
]
