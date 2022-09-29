Class {
	#name : #BabyCProgram,
	#superclass : #Baby,
	#instVars : [
		'uldc'
	],
	#category : #ULD
}

{ #category : #'instance creation' }
BabyCProgram class >> elf: filePath [
	^self basicNew
		uldc: (ULDc for: filePath);
		createMicrowattGDB;
		yourself
]

{ #category : #GT }
BabyCProgram >> disassembly [
	| pc |
	pc := gdb pc.
	^(pc-8 to: pc+16 by: 4) collect: [ :addr |
		| instr dis |
		instr := gdb read32At: addr.
		dis := [gdb processorDescription disassemble: instr]
			on: IllegalInstruction 
			do: [ '?' ].
		dis := (addr printStringBase: 16 length: 8 padded: true), '   ', dis.
		dis := dis asText.
		addr = pc ifTrue: [ dis := dis allBold ].
		dis ]
		
]

{ #category : #'as yet unclassified' }
BabyCProgram >> editVariable: varName [ 
	| ldi newString |
	varName isNil ifTrue: [ ^nil ].
	varName isEmpty ifTrue: [ ^nil ].
	ldi := [ uldc lookupSymbol: varName ]
		on: Error
		do: [ ^UIManager default inform: 'No address for ', varName ].
	ldi address isNil ifTrue: [ ^UIManager default inform: 'Nil address for ', varName ].
	newString := UIManager default
		request: varName, ' = 0x...'
		initialAnswer: ((gdb read32At: ldi address asInteger) printStringBase: 16 length: 8 padded: true).
	gdb writeInt32: (Integer readFrom: newString base: 16) toAddr: ldi address asInteger
]

{ #category : #GT }
BabyCProgram >> gtInspectorCSourceIn: composite [
	<gtInspectorPresentationOrder: 6>
	| ldiResult |
	ldiResult := self lookupPC.
	ldiResult isNil ifTrue: [ ^nil ].
	^composite text
			title: ldiResult filenameForTab;
			shouldValidate: true;
			format: #asText;
			act: [ :widget | self showVariable: widget selectedText asString ]
					icon: GLMUIThemeExtraIcons glamorousInspect
					entitled: 'show selected var';
			act: [ :widget | self editVariable: widget selectedText asString ]
					icon: GLMUIThemeExtraIcons glamorousEdit
					entitled: 'set selected var';
			act: [ :widget | self nextCLine. widget update ]
					icon: GLMUIThemeExtraIcons glamorousOver
					entitled: 'step to next line';
			display: [ self lookupPC decoratedText ]
]

{ #category : #GT }
BabyCProgram >> gtInspectorDisassemblyIn: composite [
	<gtInspectorPresentationOrder: 5>
	^composite fastList
			title: gdb processorDescription architectureName, '-asm';
			shouldValidate: true;
			format: #asText;
			act: [ :widget | self stepi. widget update ]
					icon: GLMUIThemeExtraIcons glamorousInto
					entitled: 'stepi';
			display: [ self disassembly ]
]

{ #category : #'as yet unclassified' }
BabyCProgram >> lookupPC [
	[ ^uldc lookupPC: gdb pc ]
		on: Error
		do: [ ^nil ]
]

{ #category : #'as yet unclassified' }
BabyCProgram >> nextCLine [
	| originalSourcePosition |
	originalSourcePosition := self lookupPC.
	[ self lookupPC = originalSourcePosition ] whileTrue: [ gdb s ]
]

{ #category : #'as yet unclassified' }
BabyCProgram >> showVariable: varName [ 
	| ldi |
	varName isNil ifTrue: [ ^nil ].
	varName isEmpty ifTrue: [ ^nil ].
	ldi := [ uldc lookupSymbol: varName ]
		on: Error
		do: [ ^UIManager default inform: 'No address for ', varName ].
	ldi address isNil ifTrue: [ ^UIManager default inform: 'Nil address for ', varName ].
	UIManager default
		confirm: '0x', ((gdb read32At: ldi address asInteger) printStringBase: 16 length: 8 padded: true)
		label: varName
]

{ #category : #'as yet unclassified' }
BabyCProgram >> stepi [
	gdb s
]

{ #category : #accessing }
BabyCProgram >> uldc [
	^ uldc
]

{ #category : #accessing }
BabyCProgram >> uldc: anObject [
	uldc := anObject
]
