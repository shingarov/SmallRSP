Class {
	#name : #BabySourceFileArray,
	#superclass : #SourceFileArray,
	#category : #'GDB-Cog'
}

{ #category : #'public - file system operations' }
BabySourceFileArray >> ensureOpen [
	"Ensure that the source and changes files are opened.
	Close them before re-opening them."
	
	| aSourcesFile aChangesFile |
	self close.
	aSourcesFile := SourceFile
		on: 'SqueakV50.sources'
		potentialLocations: #('/home/boris/work/opensmalltalk-vm/image').
	aChangesFile := SourceFile
		on: 'r.changes'
		potentialLocations: #('/home/boris/work/opensmalltalk-vm/image').

	files := Array
		with: aSourcesFile
		with: aChangesFile.
			
	readOnlyQueue := SharedQueue new.
]

{ #category : #'public - file system operations' }
BabySourceFileArray >> readStreamAtFileIndex: index atPosition: position ifPresent: presentBlock ifAbsent: absentBlock [
 
   | result file |
   

	file := files at: index.
	file tryOpenReadOnly: true.
	
   file position: position.
   result := presentBlock value: file.
       
	file close.                     
    ^ result

]

{ #category : #'public - file system operations' }
BabySourceFileArray >> remoteStringAt: sourcePointer [
	^(super remoteStringAt: sourcePointer)
		sourceFileArray: (self ensureOpen);
		yourself
]
