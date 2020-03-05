"
I add a 'Registers' tab to the GT Inspector.
"
Class {
	#name : #GdbRegistersMorph,
	#superclass : #BorderedMorph,
	#instVars : [
		'gdb',
		'regs'
	],
	#category : #'GDB-UI'
}

{ #category : #'instance creation' }
GdbRegistersMorph class >> concreteClassFor: aGDB [
	^Smalltalk classNamed: 'GdbRegistersMorph', aGDB processorDescription architectureName 
]

{ #category : #'instance creation' }
GdbRegistersMorph class >> on: aGDB [
	^(self concreteClassFor: aGDB) new
		gdb: aGDB;
		yourself
]

{ #category : #drawing }
GdbRegistersMorph >> defaultBounds [
	"Answer the default bounds for the receiver."

	^0 @ 0 corner: 500 @ 300
]

{ #category : #drawing }
GdbRegistersMorph >> drawGPRHeaderOn: clippedCanvas [
	clippedCanvas
		drawString: 'GPR:'
		at: self bounds topLeft 
		font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 14)
		color: Color black.
	^18 "BOGUS CODE"
	
]

{ #category : #drawing }
GdbRegistersMorph >> drawGPRsOn: clippedCanvas [
	| y |
	y := self drawGPRHeaderOn: clippedCanvas.
	0 to: 31 do: [ :r |
		| regName |
		regName := 'r', r printString.
		clippedCanvas
			drawString: regName, ' = ', (regs at: regName) printString
			at: (self bounds topLeft + (0 @ y))
			font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 12)
			color: Color black.
		y := y + 15.
		 ].
	^y
	
]

{ #category : #drawing }
GdbRegistersMorph >> drawOn: aCanvas [
	regs := gdb getRegisters.
	aCanvas
		clipBy: self bounds
		during: [:clippedCanvas |
			clippedCanvas
				fillRectangle: self bounds
				color: Color yellow.	
			self drawRegistersOn: clippedCanvas
		]
 
]

{ #category : #accessing }
GdbRegistersMorph >> gdb [
	^ gdb
]

{ #category : #accessing }
GdbRegistersMorph >> gdb: anObject [
	gdb := anObject
]
