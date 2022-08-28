Extension { #name : #Integer }

{ #category : #'*GDB' }
Integer >> byteSwap32 [
	"It is not clear where this extension shoud be.
	 This was copied from VMMaker, but ULD is below VMMaker and needs it."
	
	^((self << 24) bitAnd: 16rFF000000) bitOr:
	  (((self << 8) bitAnd: 16rFF0000) bitOr:
	  (((self >> 8) bitAnd: 16rFF00) bitOr:
	  (((self >> 24) bitAnd: 16rFF))))
]

{ #category : #'*GDB' }
Integer >> byteSwap64 [
	"It is not clear where this extension shoud be.
	 This was copied from VMMaker, but ULD is below VMMaker and needs it."

	^(((self << 56) bitAnd: 16rFF00000000000000) bitOr:
	  (((self << 40) bitAnd: 16rFF000000000000) bitOr:
	  (((self << 24) bitAnd: 16rFF0000000000) bitOr:
	  (((self << 8) bitAnd: 16rFF00000000) bitOr:
	  (((self >> 8) bitAnd: 16rFF000000) bitOr:
	  (((self >> 24) bitAnd: 16rFF0000) bitOr:
	  (((self >> 40) bitAnd: 16rFF00) bitOr:
	  ((self >> 56) bitAnd: 16rFF))))))))
]
