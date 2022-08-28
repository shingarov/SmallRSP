Class {
	#name : #ModtalkThinshellAddressSpaceARM,
	#superclass : #ModtalkThinshellAddressSpace,
	#category : #'GDB-tests-Connected'
}

{ #category : #'instance creation' }
ModtalkThinshellAddressSpaceARM class >> architectures [
	^{ 'arm' }
]

{ #category : #'register conventions' }
ModtalkThinshellAddressSpaceARM class >> registerAssignments [
	^Dictionary newFromAssociations:
	{
	(#NZone -> 'lr').
	(#HEAP  -> 'r1').
	(#SP    -> 'sp').
	(#NativePC -> 'pc').
	}
	
]
