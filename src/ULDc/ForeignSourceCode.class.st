Class {
	#name : #ForeignSourceCode,
	#superclass : #Model,
	#instVars : [
		'file',
		'line'
	],
	#category : #ULDc
}

{ #category : #'instance creation' }
ForeignSourceCode class >> file: f line: l [
	^self basicNew
		file: f;
		line: l;
		yourself
]

{ #category : #accessing }
ForeignSourceCode >> file [
	^ file
]

{ #category : #accessing }
ForeignSourceCode >> file: anObject [
	file := anObject
]

{ #category : #GT }
ForeignSourceCode >> gtInspectorSourceIn: composite [
	<gtInspectorPresentationOrder: 5>
	| str txt interval |
	str := self contents.
	interval := str lineIntervals at: line.
	txt := str asText.
	txt addAttribute: TextBackgroundColor cyan from: interval first to: interval last.
	composite text
			title: 'Contents';
			format: #asText;
			display: [ txt ]
]

{ #category : #accessing }
ForeignSourceCode >> line [
	^ line
]

{ #category : #accessing }
ForeignSourceCode >> line: anObject [
	line := anObject
]
