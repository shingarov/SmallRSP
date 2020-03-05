"
I add a 'Modtalk' tab to the GT Inspector.
"
Class {
	#name : #GdbMTEngineMorph,
	#superclass : #BorderedMorph,
	#instVars : [
		'gdb',
		'regs'
	],
	#category : #'GDB-UI'
}

{ #category : #'instance creation' }
GdbMTEngineMorph class >> on: aGDB [
	^self new
		gdb: aGDB;
		yourself
]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> defaultBounds [
	"Answer the default bounds for the receiver."

	^0 @ 0 corner: 500 @ 300
]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawCurrentBytecodeOn: clippedCanvas fromHeight: y [
	| bc |
	bc := [ gdb currentBytecode printString ]
		on: Error do: [ '???' ].
	clippedCanvas
		drawString: bc
		at: (self bounds topLeft + (0 @ y))
		font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 12)
		color: Color black.
	^y + 14
]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawCurrentFrameOn: clippedCanvas fromHeight: y [
	| fp  |
	fp := gdb getVRegister: #FP.
	self drawFrame: fp withAllSendersOn: clippedCanvas fromHeight: y

]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawCurrentInstructionOn: clippedCanvas fromHeight: y [
	| pc nativeInstr |
	pc := regs at: 'pc'.
	nativeInstr := gdb currentInstruction.
	clippedCanvas
		drawString: nativeInstr printString
		at: (self bounds topLeft + (0 @ y))
		font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 12)
		color: Color black.
	^y + 14
]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawFrame: fp on: clippedCanvas fromHeight: y [
	| frame cm |
	frame := MTRemoteStackFrame gdb: gdb pointer: fp.
	cm := frame method.
	clippedCanvas
		drawString: cm selector symbol asString
		at: (self bounds topLeft + (0 @ y))
		font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 12)
		color: Color black.
	^y+20
]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawFrame: fp withAllSendersOn: clippedCanvas fromHeight: y [
	| frame yy |
	yy := self drawFrame: fp on: clippedCanvas fromHeight: y.
	frame := MTRemoteStackFrame gdb: gdb pointer: fp.
	frame .
	^y+20

]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawOn: aCanvas [
	regs := gdb getRegisters.
	aCanvas
		clipBy: self bounds
		during: [:clippedCanvas |
			clippedCanvas
				fillRectangle: self bounds
				color: Color white.	
		self drawCurrentFrameOn: clippedCanvas fromHeight:
		(self drawCurrentBytecodeOn: clippedCanvas fromHeight:
		(self drawCurrentInstructionOn: clippedCanvas fromHeight:
		(self drawVregsOn: clippedCanvas) + 15)) + 15
				].
 
]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawSPR: spr on: clippedCanvas fromHeight: y [
	clippedCanvas
		drawString: spr, ' = ', (regs at: spr) printString
		at: (self bounds topLeft + (0 @ y))
		font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 12)
		color: Color black.
	^y + 14	
]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawSPRsOn: clippedCanvas fromHeight: y [
	| yy |
	yy := y.
	#('pc' 'lr' 'cr' 'ctr' 'msr' 'xer') do: [ :spr |
		self drawSPR: spr on: clippedCanvas fromHeight: yy.
		yy := yy + 14.
		].
	^yy
]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawVRegHeaderOn: clippedCanvas [
	clippedCanvas
		drawString: 'VRegs:'
		at: self bounds topLeft 
		font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 14)
		color: Color black.
	^18 "BOGUS CODE"
	
]

{ #category : #'as yet unclassified' }
GdbMTEngineMorph >> drawVregsOn: clippedCanvas [
	| y |
	y := self drawVRegHeaderOn: clippedCanvas.
	TAJWriter registerMap keysAndValuesDo: [ :vReg :physReg |
		| regName |
		regName := physReg isInteger ifTrue: ['r', physReg printString]
			ifFalse: [ physReg ].

		clippedCanvas
			drawString: vReg printString, ' = ', (regs at: regName) printString
			at: (self bounds topLeft + (0 @ y))
			font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 12)
			color: Color black.
		y := y + 14.
		 ].
	^y
	
]

{ #category : #accessing }
GdbMTEngineMorph >> gdb [
	^ gdb
]

{ #category : #accessing }
GdbMTEngineMorph >> gdb: anObject [
	gdb := anObject
]
