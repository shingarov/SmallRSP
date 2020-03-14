"
This will go away when we implement xi:include
"
Class {
	#name : #FakeProcessorDescriptionP1025,
	#superclass : #FakeProcessorDescriptionPPC,
	#category : #'GDB-Tests'
}

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionP1025 class >> features [
	^'<?xml version="1.0"?>
<target>
<architecture>powerpc</architecture>

<feature name="org.gnu.gdb.power">
  <reg name="r0" bitsize="32" type="uint32"/>
  <reg name="r1" bitsize="32" type="uint32"/>
  <reg name="r2" bitsize="32" type="uint32"/>
  <reg name="r3" bitsize="32" type="uint32"/>
  <reg name="r4" bitsize="32" type="uint32"/>
  <reg name="r5" bitsize="32" type="uint32"/>
  <reg name="r6" bitsize="32" type="uint32"/>
  <reg name="r7" bitsize="32" type="uint32"/>
  <reg name="r8" bitsize="32" type="uint32"/>
  <reg name="r9" bitsize="32" type="uint32"/>
  <reg name="r10" bitsize="32" type="uint32"/>
  <reg name="r11" bitsize="32" type="uint32"/>
  <reg name="r12" bitsize="32" type="uint32"/>
  <reg name="r13" bitsize="32" type="uint32"/>
  <reg name="r14" bitsize="32" type="uint32"/>
  <reg name="r15" bitsize="32" type="uint32"/>
  <reg name="r16" bitsize="32" type="uint32"/>
  <reg name="r17" bitsize="32" type="uint32"/>
  <reg name="r18" bitsize="32" type="uint32"/>
  <reg name="r19" bitsize="32" type="uint32"/>
  <reg name="r20" bitsize="32" type="uint32"/>
  <reg name="r21" bitsize="32" type="uint32"/>
  <reg name="r22" bitsize="32" type="uint32"/>
  <reg name="r23" bitsize="32" type="uint32"/>
  <reg name="r24" bitsize="32" type="uint32"/>
  <reg name="r25" bitsize="32" type="uint32"/>
  <reg name="r26" bitsize="32" type="uint32"/>
  <reg name="r27" bitsize="32" type="uint32"/>
  <reg name="r28" bitsize="32" type="uint32"/>
  <reg name="r29" bitsize="32" type="uint32"/>
  <reg name="r30" bitsize="32" type="uint32"/>
  <reg name="r31" bitsize="32" type="uint32"/>

  <reg name="ev0h" bitsize="32" regnum="32"/>
  <reg name="ev1h" bitsize="32"/>
  <reg name="ev2h" bitsize="32"/>
  <reg name="ev3h" bitsize="32"/>
  <reg name="ev4h" bitsize="32"/>
  <reg name="ev5h" bitsize="32"/>
  <reg name="ev6h" bitsize="32"/>
  <reg name="ev7h" bitsize="32"/>
  <reg name="ev8h" bitsize="32"/>
  <reg name="ev9h" bitsize="32"/>
  <reg name="ev10h" bitsize="32"/>
  <reg name="ev11h" bitsize="32"/>
  <reg name="ev12h" bitsize="32"/>
  <reg name="ev13h" bitsize="32"/>
  <reg name="ev14h" bitsize="32"/>
  <reg name="ev15h" bitsize="32"/>
  <reg name="ev16h" bitsize="32"/>
  <reg name="ev17h" bitsize="32"/>
  <reg name="ev18h" bitsize="32"/>
  <reg name="ev19h" bitsize="32"/>
  <reg name="ev20h" bitsize="32"/>
  <reg name="ev21h" bitsize="32"/>
  <reg name="ev22h" bitsize="32"/>
  <reg name="ev23h" bitsize="32"/>
  <reg name="ev24h" bitsize="32"/>
  <reg name="ev25h" bitsize="32"/>
  <reg name="ev26h" bitsize="32"/>
  <reg name="ev27h" bitsize="32"/>
  <reg name="ev28h" bitsize="32"/>
  <reg name="ev29h" bitsize="32"/>
  <reg name="ev30h" bitsize="32"/>
  <reg name="ev31h" bitsize="32"/>

  <reg name="pc" bitsize="32" type="code_ptr" regnum="64"/>
  <reg name="msr" bitsize="32" type="uint32"/>
  <reg name="cr" bitsize="32" type="uint32"/>
  <reg name="lr" bitsize="32" type="code_ptr"/>
  <reg name="ctr" bitsize="32" type="uint32"/>
  <reg name="xer" bitsize="32" type="uint32"/>

  <reg name="orig_r3" bitsize="32" regnum="71"/>
  <reg name="trap" bitsize="32"/>

  <reg name="acc" bitsize="64" regnum="73"/>
  <reg name="spefscr" bitsize="32"/>


</feature>
</target>
'
]

{ #category : #'as yet unclassified' }
FakeProcessorDescriptionP1025 class >> fullFeatures [
	"A tiny bit less bogus than #features,
	but still needs to be deleted in favor of getting the features
	from the RSP server -- currently dont know what to do with xi:include"
	^'<?xml version="1.0"?>
<target>
<architecture>powerpc</architecture>

<feature name="org.gnu.gdb.power.core">
  <reg name="r0" bitsize="32" type="uint32"/>
  <reg name="r1" bitsize="32" type="uint32"/>
  <reg name="r2" bitsize="32" type="uint32"/>
  <reg name="r3" bitsize="32" type="uint32"/>
  <reg name="r4" bitsize="32" type="uint32"/>
  <reg name="r5" bitsize="32" type="uint32"/>
  <reg name="r6" bitsize="32" type="uint32"/>
  <reg name="r7" bitsize="32" type="uint32"/>
  <reg name="r8" bitsize="32" type="uint32"/>
  <reg name="r9" bitsize="32" type="uint32"/>
  <reg name="r10" bitsize="32" type="uint32"/>
  <reg name="r11" bitsize="32" type="uint32"/>
  <reg name="r12" bitsize="32" type="uint32"/>
  <reg name="r13" bitsize="32" type="uint32"/>
  <reg name="r14" bitsize="32" type="uint32"/>
  <reg name="r15" bitsize="32" type="uint32"/>
  <reg name="r16" bitsize="32" type="uint32"/>
  <reg name="r17" bitsize="32" type="uint32"/>
  <reg name="r18" bitsize="32" type="uint32"/>
  <reg name="r19" bitsize="32" type="uint32"/>
  <reg name="r20" bitsize="32" type="uint32"/>
  <reg name="r21" bitsize="32" type="uint32"/>
  <reg name="r22" bitsize="32" type="uint32"/>
  <reg name="r23" bitsize="32" type="uint32"/>
  <reg name="r24" bitsize="32" type="uint32"/>
  <reg name="r25" bitsize="32" type="uint32"/>
  <reg name="r26" bitsize="32" type="uint32"/>
  <reg name="r27" bitsize="32" type="uint32"/>
  <reg name="r28" bitsize="32" type="uint32"/>
  <reg name="r29" bitsize="32" type="uint32"/>
  <reg name="r30" bitsize="32" type="uint32"/>
  <reg name="r31" bitsize="32" type="uint32"/>

  <reg name="pc" bitsize="32" type="code_ptr" regnum="64"/>
  <reg name="msr" bitsize="32" type="uint32"/>
  <reg name="cr" bitsize="32" type="uint32"/>
  <reg name="lr" bitsize="32" type="code_ptr"/>
  <reg name="ctr" bitsize="32" type="uint32"/>
  <reg name="xer" bitsize="32" type="uint32"/>
</feature>

<feature name="org.gnu.gdb.power.spe">
  <reg name="ev0h" bitsize="32" regnum="32"/>
  <reg name="ev1h" bitsize="32"/>
  <reg name="ev2h" bitsize="32"/>
  <reg name="ev3h" bitsize="32"/>
  <reg name="ev4h" bitsize="32"/>
  <reg name="ev5h" bitsize="32"/>
  <reg name="ev6h" bitsize="32"/>
  <reg name="ev7h" bitsize="32"/>
  <reg name="ev8h" bitsize="32"/>
  <reg name="ev9h" bitsize="32"/>
  <reg name="ev10h" bitsize="32"/>
  <reg name="ev11h" bitsize="32"/>
  <reg name="ev12h" bitsize="32"/>
  <reg name="ev13h" bitsize="32"/>
  <reg name="ev14h" bitsize="32"/>
  <reg name="ev15h" bitsize="32"/>
  <reg name="ev16h" bitsize="32"/>
  <reg name="ev17h" bitsize="32"/>
  <reg name="ev18h" bitsize="32"/>
  <reg name="ev19h" bitsize="32"/>
  <reg name="ev20h" bitsize="32"/>
  <reg name="ev21h" bitsize="32"/>
  <reg name="ev22h" bitsize="32"/>
  <reg name="ev23h" bitsize="32"/>
  <reg name="ev24h" bitsize="32"/>
  <reg name="ev25h" bitsize="32"/>
  <reg name="ev26h" bitsize="32"/>
  <reg name="ev27h" bitsize="32"/>
  <reg name="ev28h" bitsize="32"/>
  <reg name="ev29h" bitsize="32"/>
  <reg name="ev30h" bitsize="32"/>
  <reg name="ev31h" bitsize="32"/>

  <reg name="acc" bitsize="64" regnum="73"/>
  <reg name="spefscr" bitsize="32"/>
</feature>

<feature name="org.gnu.gdb.power.linux">
  <reg name="orig_r3" bitsize="32" regnum="71"/>
  <reg name="trap" bitsize="32"/>
</feature>
</target>
'
]
