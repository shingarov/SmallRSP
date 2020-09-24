Extension { #name : #LibC }

{ #category : #'*GDB' }
LibC >> mmap: addr with: length with: prot with: flags with: fd with: offset [
	^self
		ffiCall: #(  ExternalAddress*  mmap #(longlong addr, longlong length, long prot, long flags, long fd, long offset)  )
]
