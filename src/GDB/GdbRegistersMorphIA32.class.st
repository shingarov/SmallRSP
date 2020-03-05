Class {
	#name : #GdbRegistersMorphIA32,
	#superclass : #GdbRegistersMorph,
	#category : #'GDB-UI'
}

{ #category : #drawing }
GdbRegistersMorphIA32 >> drawRegistersOn: clippedCanvas [
	| y |
	y := self drawGPRHeaderOn: clippedCanvas.
	gdb processorDescription gdb do: [ :r |
		| regName |
		regName := r regName.
		clippedCanvas
			drawString: regName, ' = ', (regs at: regName) printString
			at: (self bounds topLeft + (0 @ y))
			font: (LogicalFont familyName: 'Bitmap DejaVu Sans' pointSize: 12)
			color: Color black.
		y := y + 15.
		 ].
	^y
	
]
