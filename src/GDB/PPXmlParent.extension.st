Extension { #name : #PPXmlParent }

{ #category : #'*GDB' }
PPXmlParent >> postCopy [
	super postCopy.
	self setChildren: (self children
		collect: [ :each | each copyWithInclusion ])
]
