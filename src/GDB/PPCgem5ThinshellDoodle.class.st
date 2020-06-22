Class {
	#name : #PPCgem5ThinshellDoodle,
	#superclass : #PPCThinshellDoodle,
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
PPCgem5ThinshellDoodle >> host [
	^#gem5
]

{ #category : #tests }
PPCgem5ThinshellDoodle >> testManualNZone [
	"
	PPCgem5ThinshellDoodle new halt;  testManualNZone
	"
	self connectGdb;
		stepThroughThinshellPreamble;
		fillNZone.
	[ "then, run at full speed until the exit() syscall"
	gdb c
		"At this point we expect gem5 to have exited and said,
		Exit code is 42"
	] on: GdbChildExited do: [ ^self ].
	self error

]

{ #category : #tests }
PPCgem5ThinshellDoodle >> testManualNZone2 [
	"
	PPCgem5ThinshellDoodle new testManualNZone2
	"
	self connectGdb;
		stepThroughThinshellPreamble;
		fillNZone2.
	[ "then, run at full speed until the exit() syscall"
	gdb c
		"At this point we expect gem5 to have exited and said,
		Exit code is 42"
	] on: GdbChildExited do: [ ^self ].
	self error

]
