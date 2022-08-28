Class {
	#name : #URJError,
	#superclass : #Error,
	#category : #UrJTAG
}

{ #category : #signaling }
URJError class >> signalCurrent [
	"Signal the URJ error the UrJTAG is currently reporting."
	^self signal: LibUrJTAG uniqueInstance urj_error_describe
]
