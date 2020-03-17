Class {
	#name : #RemoteGDBTransport,
	#superclass : #Object,
	#instVars : [
		'socket'
	],
	#category : #'GDB-RSP'
}

{ #category : #'as yet unclassified' }
RemoteGDBTransport class >> host: h [
	^self host: h port: self wellKnownPort 
]

{ #category : #'as yet unclassified' }
RemoteGDBTransport class >> host: h port: p [
	| socket |
	socket := Socket newTCP 
		setOption: 'TCP_NODELAY' value: 1;
		connectTo: (NetNameResolver addressForName: h) port: p.
	^self onSocket: socket
]

{ #category : #'as yet unclassified' }
RemoteGDBTransport class >> onSocket: aSocket [
	^self new socket: aSocket 
]

{ #category : #'as yet unclassified' }
RemoteGDBTransport class >> wellKnownPort [
	^2159
]

{ #category : #private }
RemoteGDBTransport >> assemblePacket: packetDataString [
	| s cksum |
	s := '$', (self escape: packetDataString), '#'.
	cksum := packetDataString  inject: 0 into: [ :soFar :c | soFar + c asciiValue \\ 256 ].
	^s, (cksum printStringBase: 16 nDigits: 2) asLowercase 

]

{ #category : #'target connection' }
RemoteGDBTransport >> disconnect [
	self socket close
]

{ #category : #private }
RemoteGDBTransport >> escape: aString [
	^aString inject: '' into: [ :soFar :c | soFar, (self escapeChar: c) ].
	
]

{ #category : #private }
RemoteGDBTransport >> escapeChar: c [
	"Where the characters '#' or '$' appear in the packet data,
	they must be escaped.
	The escape character is ASCII 0x7d ('}'),
	and is followed by the original character XORed with 0x20.
	The character '}' itself must also be escaped."
	
	((c == $# or: [c == $$]) or: [c == $}]) ifTrue: [ 
		^String with: $} with: (Character value: (c asciiValue bitXor: 16r20))
	] ifFalse: [ ^String with: c ]
]

{ #category : #private }
RemoteGDBTransport >> readAndVerifyCksum: anInteger [
	"Read next two characters from the TCP stream and verify that they represent the same hext number as anInteger."
	| cksumFromServer |

	cksumFromServer := Integer
		readFrom: (String with: self receiveByte with: self receiveByte)
		radix: 16.
	"cksumFromServer = anInteger ifFalse: [ self error: 'Wrong checksum' ]"
]

{ #category : #API }
RemoteGDBTransport >> receive [
	| c stream cksum cc |
	"Receive and decode the server's response.
	It may come in chunks on the TCP stream; in that sense, the word Packet may be confusing."
	
	self receiveByte = $$ ifFalse: [ self error: 'Packet does not start with $' ].

	stream := WriteStream with: ''.
	cksum := 0.
	[ c := self receiveChar. c = $# ] whileFalse: [
		cksum := cksum + c asInteger \\ 256.
		c = $* ifTrue: [
			| nChar |
			nChar := self receiveChar asciiValue.
			cksum := cksum + nChar \\ 256.
			nChar - 29 timesRepeat: [ stream nextPut: cc.  ]
		"RLE"
		 ] ifFalse: [ stream nextPut: c. cc := c. ]].
"TODO: escaping"
	self readAndVerifyCksum: cksum.
	socket sendData: '+'. 

	^stream contents

]

{ #category : #API }
RemoteGDBTransport >> receiveByte [
	| buf1 |
	"Receive exactly one byte, waiting for it if neccesary.
	This is at a level lower than escaping; see #receiveChar for the escaped version."
	buf1 := ByteString new: 1.
	(socket receiveDataInto: buf1) == 1 ifFalse: [GdbChildExited new signal].
	^buf1 first
]

{ #category : #API }
RemoteGDBTransport >> receiveChar [
	"Receive the next char from the TCP stream."

	| byte |
	byte := self receiveByte.
	"Do NOT escape at this level; check for RLE, and then escaping is above."
	^byte
]

{ #category : #API }
RemoteGDBTransport >> send: aString [
	| packet ack n |
	packet := self assemblePacket: aString.
	socket sendData: packet.
	ack := '*'.
	n := socket receiveDataInto: ack.
	ack = '+' ifFalse: [ self error: 'gdb server answered ', ack ].
(Delay forMilliseconds: 50) wait.

]

{ #category : #accessing }
RemoteGDBTransport >> socket [
	^ socket
]

{ #category : #accessing }
RemoteGDBTransport >> socket: aSocket [
	socket := aSocket
]
