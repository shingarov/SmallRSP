Class {
	#name : #FakeProcessorDescriptionX86,
	#superclass : #FakeProcessorDescription,
	#category : #'GDB-Doodles'
}

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionX86 class >> endian [ 
	^#little
]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionX86 class >> fakeFeatures [
	^TargetAwareX86 isHardware
		ifTrue: [self fakeFeaturesStockSilicon]
		ifFalse: [self fakeFeaturesGem5]
]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionX86 class >> fakeFeaturesGem5 [
	^#(
#('eax' 32)  #('ecx' 32)  #('edx' 32)  #('ebx' 32)  #('esp' 32)  #('ebp' 32)  #('esi' 32)  #('edi' 32)
#('eip' 32)  #('eflags' 32)  #('cs' 32) #('ss' 32) #('ds' 32) #('es' 32) #('fs' 32) #('gs' 32)
)
]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionX86 class >> fakeFeaturesStockSilicon [
	^#(
#('eax' 32)  #('ecx' 32)  #('edx' 32)  #('ebx' 32)  #('esp' 32)  #('ebp' 32)  #('esi' 32)  #('edi' 32)
#('eip' 32)  #('eflags' 32)  #('cs' 32) #('ss' 32) #('ds' 32) #('es' 32) #('fs' 32) #('gs' 32)

#('st0' 80) #('st1' 80) #('st2' 80) #('st3' 80) #('st4' 80) #('st5' 80) #('st6' 80) #('st7' 80)
#('fctrl' 32) #('fstat' 32) #('ftag' 32) #('fiseg' 32) #('fioff' 32) #('foseg' 32) #('fooff' 32) #('fop' 32)

"sse"
#('xmm0' 128) #('xmm1' 128) #('xmm2' 128) #('xmm3' 128) #('xmm4' 128) #('xmm5' 128) #('xmm6' 128) #('xmm7' 128) 
#('mxcsr' 32)

#('orig_eax' 32)
)


]

{ #category : #'for DUI' }
FakeProcessorDescriptionX86 >> architectureName [
	^'x86'
]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionX86 >> pcRegisterName [
	^'eip'
]

{ #category : #'for HWBKPT' }
FakeProcessorDescriptionX86 >> z1kind [
	^1
]
