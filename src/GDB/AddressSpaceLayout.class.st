"
In Target-Agnostic Modtalk, when we are AoT-ing the binary image, somewhere in the address space there is the arena where we construct the object heap, and the nZone.  Instances of my concrete subclasses know where these two addresses are.
"
Class {
	#name : #AddressSpaceLayout,
	#superclass : #Object,
	#category : #'GDB-TAJ'
}

{ #category : #accessing }
AddressSpaceLayout >> heap [
	self subclassResponsibility 
]

{ #category : #accessing }
AddressSpaceLayout >> nZone [
	self subclassResponsibility 
]
