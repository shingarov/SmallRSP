Class {
	#name : #PrivatePrimitiveProcessor,
	#superclass : #BasePrimitiveProcessor,
	#category : #'GDB-Modtalk'
}

{ #category : #primitives }
PrivatePrimitiveProcessor >> primAllocEnv [
	| stackFrame count ptr env |
	count := gdb currentBytecode count.
	stackFrame := self currentFrame.
	ptr := self allocSlots: count serviceSlots: 2 "header, hash".
	env := MTRemoteIndexedPart gdb: gdb pointer: ptr.
	env header: (TAJObjectWriter
		declareObjectHeaderType: 'IndexedPartType'
		hashFormat: 'NoHash'
		numVars: count
		hash: 0
		meta: 0).
	env basicSlotAt: 1 put: stackFrame env pointer.
	stackFrame env: env.
	self setVRegister: #X to: ptr.

]

{ #category : #primitives }
PrivatePrimitiveProcessor >> primMethodClosure [
	|  stackFrame method blocks ptr md count block blockClosure numArgs blockClosureMDKey |

	count := gdb currentBytecode count.
	stackFrame := self currentFrame.
	method := stackFrame method.
	blocks := method blocks.
	block := MTRemoteCompiledBlock gdb: gdb pointer: (blocks at: count).
	numArgs := block numArgs.
	blockClosureMDKey := (Array
		with: #MTZeroArgumentBlockClosure_md
		with: #MTOneArgumentBlockClosure_md
		with: #MTTwoArgumentBlockClosure_md
	) at: numArgs + 1.
	md := gdb exe externalReferences at: blockClosureMDKey.
	ptr := self allocOop: 4 md: md.
	blockClosure := MTRemoteBlockClosure gdb: gdb pointer: ptr.
	blockClosure 
		literals: method literals;
		compiledBlock: block;
		env:(stackFrame env);
		receiver: (MTRemoteObject gdb: gdb pointer: (self getVRegister: #R)).
	self setVRegister: #R to: ptr.
]
