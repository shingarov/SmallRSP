Class {
	#name : #RefBoardGDBTest,
	#superclass : #RemoteGDBTest,
	#category : #'GDB-tests-Connected'
}

{ #category : #'target connection' }
RefBoardGDBTest >> connectGDB [
	gdb := RemoteGDB
	
		host: self host
		port: self port
		rspClientClass: BDI2000PPC400
]
