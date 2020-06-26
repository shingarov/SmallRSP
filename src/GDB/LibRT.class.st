Class {
	#name : #LibRT,
	#superclass : #FFILibrary,
	#category : #'GDB-RSP'
}

{ #category : #'accessing platform' }
LibRT >> shmOpen: fileName with: oflag with: mode [
	"<cdecl: ulong 'shm_open' (char* ulong ulong) module: '/lib/x86_64-linux-gnu/librt.so.1'>"
	^self ffiCall: #(ulong shm_open (String fileName, uint oflag, uint mode) )
]

{ #category : #'accessing platform' }
LibRT >> unix64LibraryName [
	^ '/lib/x86_64-linux-gnu/librt.so.1'
]
