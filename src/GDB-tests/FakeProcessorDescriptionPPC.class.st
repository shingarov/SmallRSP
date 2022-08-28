Class {
	#name : #FakeProcessorDescriptionPPC,
	#superclass : #FakeProcessorDescription,
	#category : #'GDB-tests'
}

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionPPC class >> endian [ 
	^#big
]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionPPC class >> fakeFeatures [
	^#(
#('r0' 32)  #('r1' 32)  #('r2' 32)  #('r3' 32)  #('r4' 32)  #('r5' 32)  #('r6' 32)  #('r7' 32)
#('r8' 32)  #('r9' 32)  #('r10' 32) #('r11' 32) #('r12' 32) #('r13' 32) #('r14' 32) #('r15' 32)
#('r16' 32) #('r17' 32) #('r18' 32) #('r19' 32) #('r20' 32) #('r21' 32) #('r22' 32) #('r23' 32)
#('r24' 32) #('r25' 32) #('r26' 32) #('r27' 32) #('r28' 32) #('r29' 32) #('r30' 32) #('r31' 32)

#('f0' 64)  #('f1' 64)  #('f2' 64)  #('f3' 64)  #('f4' 64)  #('f5' 64)  #('f6' 64)  #('f7' 64)
#('f8' 64)  #('f9' 64)  #('f10' 64) #('f11' 64) #('f12' 64) #('f13' 64) #('f14' 64) #('f15' 64)
#('f16' 64) #('f17' 64) #('f18' 64) #('f19' 64) #('f20' 64) #('f21' 64) #('f22' 64) #('f23' 64)
#('f24' 64) #('f25' 64) #('f26' 64) #('f27' 64) #('f28' 64) #('f29' 64) #('f30' 64) #('f31' 64)

#('pc' 32)
#('msr' 32)
#('cr' 32)
#('lr' 32)
#('ctr' 32)
#('xer' 32)
)

]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionPPC class >> features [
	^'<?xml version="1.0"?>

<!-- This description is slightly simplified in that it does
     not take into account possible variations in features resulting
     in non-sequential numbering of registers.  -->

<target>
  <architecture>powerpc</architecture>
  <feature name="org.gnu.gdb.power">
    <reg name="r0" bitsize="32"/>
    <reg name="r1" bitsize="32"/>
    <reg name="r2" bitsize="32"/>
    <reg name="r3" bitsize="32"/>
    <reg name="r4" bitsize="32"/>
    <reg name="r5" bitsize="32"/>
    <reg name="r6" bitsize="32"/>
    <reg name="r7" bitsize="32"/>
    <reg name="r8" bitsize="32"/>
    <reg name="r9" bitsize="32"/>
    <reg name="r10" bitsize="32"/>
    <reg name="r11" bitsize="32"/>
    <reg name="r12" bitsize="32"/>
    <reg name="r13" bitsize="32"/>
    <reg name="r14" bitsize="32"/>
    <reg name="r15" bitsize="32"/>
    <reg name="r16" bitsize="32"/>
    <reg name="r17" bitsize="32"/>
    <reg name="r18" bitsize="32"/>
    <reg name="r19" bitsize="32"/>
    <reg name="r20" bitsize="32"/>
    <reg name="r21" bitsize="32"/>
    <reg name="r22" bitsize="32"/>
    <reg name="r23" bitsize="32"/>
    <reg name="r24" bitsize="32"/>
    <reg name="r25" bitsize="32"/>
    <reg name="r26" bitsize="32"/>
    <reg name="r27" bitsize="32"/>
    <reg name="r28" bitsize="32"/>
    <reg name="r29" bitsize="32"/>
    <reg name="r30" bitsize="32"/>
    <reg name="r31" bitsize="32"/>

    <reg name="f0" bitsize="64" type="ieee_double" regnum="32"/>
    <reg name="f1" bitsize="64" type="ieee_double"/>
    <reg name="f2" bitsize="64" type="ieee_double"/>
    <reg name="f3" bitsize="64" type="ieee_double"/>
    <reg name="f4" bitsize="64" type="ieee_double"/>
    <reg name="f5" bitsize="64" type="ieee_double"/>
    <reg name="f6" bitsize="64" type="ieee_double"/>
    <reg name="f7" bitsize="64" type="ieee_double"/>
    <reg name="f8" bitsize="64" type="ieee_double"/>
    <reg name="f9" bitsize="64" type="ieee_double"/>
    <reg name="f10" bitsize="64" type="ieee_double"/>
    <reg name="f11" bitsize="64" type="ieee_double"/>
    <reg name="f12" bitsize="64" type="ieee_double"/>
    <reg name="f13" bitsize="64" type="ieee_double"/>
    <reg name="f14" bitsize="64" type="ieee_double"/>
    <reg name="f15" bitsize="64" type="ieee_double"/>
    <reg name="f16" bitsize="64" type="ieee_double"/>
    <reg name="f17" bitsize="64" type="ieee_double"/>
    <reg name="f18" bitsize="64" type="ieee_double"/>
    <reg name="f19" bitsize="64" type="ieee_double"/>
    <reg name="f20" bitsize="64" type="ieee_double"/>
    <reg name="f21" bitsize="64" type="ieee_double"/>
    <reg name="f22" bitsize="64" type="ieee_double"/>
    <reg name="f23" bitsize="64" type="ieee_double"/>
    <reg name="f24" bitsize="64" type="ieee_double"/>
    <reg name="f25" bitsize="64" type="ieee_double"/>
    <reg name="f26" bitsize="64" type="ieee_double"/>
    <reg name="f27" bitsize="64" type="ieee_double"/>
    <reg name="f28" bitsize="64" type="ieee_double"/>
    <reg name="f29" bitsize="64" type="ieee_double"/>
    <reg name="f30" bitsize="64" type="ieee_double"/>
    <reg name="f31" bitsize="64" type="ieee_double"/>

    <reg name="pc"  bitsize="32" type="code_ptr" regnum="64"/>
    <reg name="msr" bitsize="32"/>
    <reg name="cr"  bitsize="32"/>
    <reg name="lr"  bitsize="32" type="code_ptr"/>
    <reg name="ctr" bitsize="32"/>
    <reg name="xer" bitsize="32"/>
  </feature>
</target>
'
]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionPPC >> architectureName [
	^'powerpc'
]

{ #category : #accessing }
FakeProcessorDescriptionPPC >> endian [ 
	^self class endian
]

{ #category : #accessing }
FakeProcessorDescriptionPPC >> initRegsInGPacket [
	regsInGPacket := (GdbXmlParser endian: self endian)
		parseString: self class features
]

{ #category : #accessing }
FakeProcessorDescriptionPPC >> regsInGPacket [
	regsInGPacket isNil ifTrue: [ self initRegsInGPacket ].
	^regsInGPacket 
]
