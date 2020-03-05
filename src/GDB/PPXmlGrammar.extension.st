Extension { #name : #PPXmlGrammar }

{ #category : #'*GDB' }
PPXmlGrammar >> doctype [
	"[28]   	doctypedecl	   ::=   	'<!DOCTYPE' S Name (S ExternalID)? S? ('[' intSubset ']' S?)? '>'"
"bgs -- the original method was bogus. I replaced it with something equally bogus for now, so at least it does not blow up"
	^ '<!DOCTYPE' asParser , $> asParser negate star , $> asParser ==> [ :x | 'bogus' ]
]
