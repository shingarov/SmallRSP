Class {
	#name : #GdbRegistersMorphpowerpc,
	#superclass : #GdbRegistersMorph,
	#category : #'GDB-UI'
}

{ #category : #drawing }
GdbRegistersMorphpowerpc >> drawRegistersOn: clippedCanvas [
	self drawSPRsOn: clippedCanvas fromHeight:
		(self drawGPRsOn: clippedCanvas) + 15
 
]

{ #category : #drawing }
GdbRegistersMorphpowerpc >> drawSPR: spr on: clippedCanvas fromHeight: y [
	clippedCanvas
		drawString: spr, ' = ', (regs at: spr) printString
		at: (self bounds topLeft + (0 @ y))
		font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 12)
		color: Color black.
	^y + 14	
]

{ #category : #drawing }
GdbRegistersMorphpowerpc >> drawSPRsOn: clippedCanvas fromHeight: y [
	| yy |
	yy := y.
	self sprNames  do: [ :spr |
		self drawSPR: spr on: clippedCanvas fromHeight: yy.
		yy := yy + 15.
		]
]

{ #category : #drawing }
GdbRegistersMorphpowerpc >> sprNames [
	gdb processorDescription architectureName = 'powerpc' 
		ifTrue: [ ^#('pc' 'lr' 'cr' 'ctr' 'msr' 'xer') ].
	^#('pc' 'sr' 'hi' 'lo' 'cause' 'badvaddr')
]
