Class {
	#name : #HostAssistedLookup,
	#superclass : #Object,
	#instVars : [
		'regBase',
		'ram'
	],
	#category : #'GDB-Modtalk'
}

{ #category : #'instance creation' }
HostAssistedLookup class >> regBase: regBase ram: ram [
	^self new 
		regBase: regBase;
		ram: ram;
		yourself
]

{ #category : #API }
HostAssistedLookup >> activateCM: cm [
	| jmpTarget |
	jmpTarget := cm codeRef address.
	regBase
		setRegister: 'ctr' to: jmpTarget; "we should do something about it"
		setVRegister: #X to: cm pointer.
	^jmpTarget
]

{ #category : #API }
HostAssistedLookup >> messageSendSelector: selectorSymbol [
	| receiver  mdRef md  cm |
	receiver := regBase getVRegister: #R.
	mdRef := ram mdFor: receiver.
	md := MTRemoteMethodDictionary gdb: ram pointer: mdRef.
	cm := md lookupSelector: selectorSymbol.
	^self activateCM: cm
]

{ #category : #accessing }
HostAssistedLookup >> ram [
	^ ram
]

{ #category : #accessing }
HostAssistedLookup >> ram: anObject [
	ram := anObject
]

{ #category : #accessing }
HostAssistedLookup >> regBase [
	^ regBase
]

{ #category : #accessing }
HostAssistedLookup >> regBase: anObject [
	regBase := anObject
]
