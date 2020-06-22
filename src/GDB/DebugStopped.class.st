Class {
	#name : #DebugStopped,
	#superclass : #Object,
	#instVars : [
		'signal'
	],
	#category : #'GDB-RSP'
}

{ #category : #'instance creation' }
DebugStopped class >> onSignalNum: anInteger [
	^self basicNew
		signal: (self signalNames at: anInteger)
		yourself
]

{ #category : #'signal numbers' }
DebugStopped class >> signalNames [
	^#(
	SIGHUP
	SIGINT
	SIGQUIT
	SIGILL
	SIGTRAP
	SIGABRT
	SIGBUS
	SIGFPE
	SIGKILL
	SIGUSR1
	SIGSEGV
	SIGUSR2
	SIGPIPE
	SIGALRM
	SIGTERM
	SIGSTKFLT
	SIGCHLD
	SIGCONT
	SIGSTOP
	SIGTSTP
	SIGTTIN
	SIGTTOU
	SIGURG
	SIGXCPU
	SIGXFSZ
	SIGVTALRM
	SIGPROF
	SIGWINCH
	SIGIO
	SIGPWR
	SIGSYS
	SIGRTMIN
	)
]

{ #category : #printing }
DebugStopped >> printOn: aStream [
	aStream nextPutAll: 'Got '; nextPutAll: self signal asString
]

{ #category : #accessing }
DebugStopped >> signal [
	^signal
]

{ #category : #accessing }
DebugStopped >> signal: aSymbol [
	signal := aSymbol
]
