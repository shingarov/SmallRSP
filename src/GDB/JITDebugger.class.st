"
I am a UI front-end to GDB for the case when the target code was JITed by Cog.
"
Class {
	#name : #JITDebugger,
	#superclass : #GTGenericStackDebugger,
	#category : #'GDB-Cog'
}

{ #category : #testing }
JITDebugger class >> availableAutomatically [
	^true
]

{ #category : #accessing }
JITDebugger class >> defaultTitle [
	^ 'ULD'
]

{ #category : #testing }
JITDebugger class >> handlesContext: aContext [
	| activationPredicate |
	activationPredicate := JITCodeDebugSession activationPredicate.
	^activationPredicate matches: aContext 

	"^ (aContext stackOfSize: self lookupStackSize) 
		anySatisfy: [ :ctx | activationPredicate matches: ctx ]"
]

{ #category : #'tools registry' }
JITDebugger class >> register [
	" JITDebugger register "
	self registerToolsOn: Smalltalk tools.
]

{ #category : #'tools registry' }
JITDebugger class >> registerToolsOn: registry [

	"Add ourselves to registry. See [Smalltalk tools]" 
	registry registerDebugger: self withRank: 80.
]

{ #category : #accessing }
JITDebugger class >> sessionClass [
	^JITCodeDebugSession
]

{ #category : #building }
JITDebugger >> debuggerStructureIn: browser [
	browser
		row: #stack;
		row: [ :c | c
			column: #code; 
			column: #disassembly ] span: 2;
		row: #inspector.

]

{ #category : #building }
JITDebugger >> debuggerTransmissionsIn: browser [

	super debuggerTransmissionsIn: browser.
	browser transmit
		from: #stack port: #selection;
		to: #disassembly;
		andShow: [ :composite :aContext | self disassemblyIn: composite forContext: aContext ]

]

{ #category : #'building presentations' }
JITDebugger >> disassemblyActionsPragmas [
	#()
]

{ #category : #'building presentations' }
JITDebugger >> disassemblyIn: aComposite forContext: aContext [

	(self disassemblyWidgetIn: aComposite forContext: aContext)
		initialize: [ :disassemblyPresentation |
				disassemblyPresentation selection: 1 "(self session x: self currentContext)" ];
		with: [ :disassembly | self installActionsForRootPragmas: self disassemblyActionsPragmas in: disassembly ].
]

{ #category : #accessing }
JITDebugger >> disassemblyPane [
	^ self browser paneNamed: #disassembly
]

{ #category : #accessing }
JITDebugger >> disassemblyPresentation [
	^self disassemblyPane presentations first
]

{ #category : #'building presentations' }
JITDebugger >> disassemblyWidgetIn: aComposite forContext: aContext [
	^ aComposite fastList 
		title: 'CPU Instructions';
		display: [ self session addressesAndInstructions ];
		format: [ :ai | "instr disassemble" 
			(ai key = self session pc ifTrue: '--> ' ifFalse: ''),			
			ai value disassemble  ]
]

{ #category : #'building presentations' }
JITDebugger >> genChainIn: aComposite [
	^aComposite fastList
		title: 'Gen chain';
		display: [ self session generatorChain stack collect: #key ]
]

{ #category : #'building presentations' }
JITDebugger >> generatorIn: aComposite [
	| g |
	g := self session generatorChain whatGeneratedMe.
	^aComposite pharoMethod 
		title: [ 'Generator' ];
		display: [ (g method sourceNodeForPC: g pc) source ];
		smalltalkClass: [ (g method sourceNodeForPC: g pc) parents first compilationContext ifNotNil: [ :ctx | ctx getClass ] ]; 
		initialize: [ :presentation | presentation selectionInterval: (g method sourceNodeForPC: g pc) sourceInterval ];
		when: [ (g method sourceNodeForPC: g pc) source notNil ];
		act: [ (g method sourceNodeForPC: g pc) methodNode method compiledMethod browse ] icon: GLMUIThemeExtraIcons glamorousBrowse entitled: 'Browse'
]

{ #category : #'building presentations' }
JITDebugger >> inspectorIn: aComposite [
	(super inspectorIn: aComposite) title: 'Mother Smalltalk'.
	self
		registersIn: aComposite;
		genChainIn: aComposite;
		generatorIn: aComposite
]

{ #category : #'building presentations' }
JITDebugger >> registersIn: aComposite [
	^aComposite fastTable
		title: 'Registers';
		display: [ self session registers associations ];
		column: 'Register' evaluated: [ :each | each key ];
		column: 'Hex' evaluated: [ :each | each value printStringHex ]
]

{ #category : #accessing }
JITDebugger >> title [
	^'ULD DEBUGGER!!!'
]
