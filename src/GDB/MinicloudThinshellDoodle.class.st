Class {
	#name : #MinicloudThinshellDoodle,
	#superclass : #PPCThinshellDoodle,
	#category : #'GDB-Doodles'
}

{ #category : #'target connection' }
MinicloudThinshellDoodle >> hostIP [
	^'127.0.0.1'
]

{ #category : #tests }
MinicloudThinshellDoodle >> testConnection [
	"
	MinicloudThinshellDoodle new testConnection
	"
	self halt; connectGdb
]
