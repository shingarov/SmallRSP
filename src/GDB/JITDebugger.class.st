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

{ #category : #'building presentations' }
JITDebugger >> chainIn: aComposite forContext: aContext [

	(self chainWidgetIn: aComposite forContext: aContext)
		initialize: [ :chainPresentation |
				chainPresentation selection: self session generatorChain interestingContext ];
		with: [ :g | self "installActionsForRootPragmas: self disassemblyActionsPragmas in: disassembly" ].
]

{ #category : #'building presentations' }
JITDebugger >> chainWidgetIn: aComposite forContext: aContext [
	^aComposite fastList
		title: 'Nativizer postmortem';
		display: [ self session generatorChain stack collect: #key ]
]

{ #category : #'target actions' }
JITDebugger >> cont [ 
	self session gdb shouldBeImplemented 
]

{ #category : #building }
JITDebugger >> debuggerStructureIn: browser [
	browser
		column: [ :r | r
				row: #chain;
				row: #gen;
				row: #inspector ];
		column: [ :r | r
				row: #disassembly;
				row: #registers ]
]

{ #category : #building }
JITDebugger >> debuggerTransmissionsIn: browser [
	browser transmit
		fromOutsideEntityPort;
		to: #chain;
		andShow: [ :composite :aContext | self chainIn: composite forContext: aContext ].
	browser transmit
		fromOutsideEntityPort;
		to: #disassembly;
		andShow: [ :composite :aContext | self disassemblyIn: composite forContext: aContext ].
	browser transmit
		fromOutsideEntityPort;
		to: #registers;
		andShow: [ :composite :aContext | self registersIn: composite "forContext: aContext" ].
	browser transmit
		from: #chain port: #selection;
		to: #inspector;
		andShow: [ :composite :aContext | self inspectorIn: composite ].
	browser transmit
		from: #chain port: #selection;
		to: #gen;
		andShow: [ :composite :aContext | self genIn: composite forContext: aContext ].
		

]

{ #category : #'building presentations' }
JITDebugger >> disassemblyIn: aComposite forContext: aContext [

	(self disassemblyWidgetIn: aComposite forContext: aContext)
		initialize: [ :disassemblyPresentation |
				disassemblyPresentation selection: (self session generatorChain instructionAtPC) ]
]

{ #category : #'building presentations' }
JITDebugger >> disassemblyWidgetIn: aComposite forContext: aContext [
	^ aComposite fastList 
		title: (self session gdb processorDescription architectureName, ' instructions');
		display: [ self session generatorChain instructions ];
		act: [ self cont ] icon: GLMUIThemeExtraIcons glamorousGo entitled: 'cont';
		act: [ self stepi ] icon: GLMUIThemeExtraIcons glamorousInto entitled: 'stepi';
		act: [ self kill ] icon: GLMUIThemeExtraIcons glamorousCancel entitled: 'kill inferior'
		
		  ";
		format: [ :ai | instr disassemble
			(ai key = self session pc ifTrue: '--> ' ifFalse: ''),			
			ai value disassemble  ]"
]

{ #category : #'building presentations' }
JITDebugger >> genIn: aComposite forContext: aContext [
	(self genWidgetIn: aComposite forContext: aContext)
		initialize: [ :code |
				code selectionInterval: (self session selectedCodeRangeForContext: self selectedGen)
				"self session generatorChain instructionAtPC  <- this is for the list"
				
				 ];
		with: [ :g | self " installCodeActionsFor: code" ].
		
	

]

{ #category : #'building presentations' }
JITDebugger >> genWidgetIn: aComposite forContext: aContext [
	| g |
	g := self selectedGen.
	^aComposite pharoMethod 
		title: [ 'Generator source' ];
		display: [ (self selectedGen method sourceNodeForPC: g pc) source ];
		smalltalkClass: [ (self selectedGen method sourceNodeForPC: g pc) parents first compilationContext ifNotNil: [ :ctx | ctx getClass ] ]; 
		initialize: [ :presentation | presentation selectionInterval: (self selectedGen method sourceNodeForPC: g pc) sourceInterval ];
		when: [ (self selectedGen method sourceNodeForPC: g pc) source notNil ]
		
]

{ #category : #'target actions' }
JITDebugger >> kill [ 
	self session gdb kill
]

{ #category : #'target actions' }
JITDebugger >> printOopInRegister [
	| name2value oop |
	name2value := (self browser paneNamed: #registers) presentations first selection.
	oop := name2value value.
	UIManager default edit: (self session printOopString: oop) label: oop hex
	 
]

{ #category : #'building presentations' }
JITDebugger >> registersIn: aComposite [
	^aComposite fastTable
		title: 'Registers';
		display: [ self session registers associations ];
		column: 'Register' evaluated: [ :each | each key ];
		column: 'Hex' evaluated: [ :each | each value printStringHex ];
		act: [ self printOopInRegister ] iconName: #objects entitled: 'print oop'
]

{ #category : #'accessing context' }
JITDebugger >> selectedGen [
	^ ((self browser paneNamed: #chain) port: #selection) value
]

{ #category : #building }
JITDebugger >> setDebuggerToFirstNonFilteredContext [
	"HACK, until we figure out how not to be a subclass of StackDebugger"
		

]

{ #category : #'target actions' }
JITDebugger >> stepi [ 
	"BOGUS.  This primitive approach will serve for now,
	but it's missing the most interesting case when execution re-enters surgery.
	This is where REAL fun with Smalltalk exceptions and contexts starts."
	self session gdb s.
	self session: self session copy
]
