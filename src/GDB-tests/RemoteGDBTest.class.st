"
I am the superclass of all GDB tests in which the target is connected over RSP.
I abstract the common sequence of creating/connecting the RSP client given host/port
(which must be supplied by the concrete subclass) during test setUp.
I also abstract the disconnection sequence during tearDown.
"
Class {
	#name : #RemoteGDBTest,
	#superclass : #GDBTest,
	#category : #'GDB-tests-Connected'
}

{ #category : #'target connection' }
RemoteGDBTest >> connectGDB [
	gdb := RemoteGDB
		host: self host
		port: self port
			
]

{ #category : #'target connection' }
RemoteGDBTest >> gdbClass [
	self subclassResponsibility
]

{ #category : #'target connection' }
RemoteGDBTest >> host [
	self subclassResponsibility
]

{ #category : #'target connection' }
RemoteGDBTest >> port [
	"Subclasses may override this.
	 The default is 7000, this is set in the Makefiles from which we launch the targets."
	^7000
]
