Class {
	#name : #InterestingBreakpointAddresses,
	#superclass : #Object,
	#category : #'GDB-Cog'
}

{ #category : #'general query commands' }
InterestingBreakpointAddresses class >> dontBreakAtAnyOfThese [
	^self dontBreakAtAnyOfTheseOnPOWER
]

{ #category : #'general query commands' }
InterestingBreakpointAddresses class >> dontBreakAtAnyOfTheseOnARM [
^#(
16r21d84
16r22280

)


]

{ #category : #'general query commands' }
InterestingBreakpointAddresses class >> dontBreakAtAnyOfTheseOnPOWER [
^#(

)


]

{ #category : #'general query commands' }
InterestingBreakpointAddresses class >> predefinedBreakpoints [ 
	^#(
	)
]
