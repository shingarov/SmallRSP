Class {
	#name : #BaselineOfSmallRSP,
	#superclass : #BaselineOf,
	#category : #BaselineOfSmallRSP
}

{ #category : #baselines }
BaselineOfSmallRSP >> baseline: spec [
	<baseline>
	spec
		for: #pharo
		do: [ 
			spec baseline: 'PetitParser' with: [
				spec loads: 'Core'; loads: 'Analyzer'.
				spec loads: 'PetitXml'; loads: 'PetitXPath'. 
				spec repository: 'github://shingarov/PetitParser:ULD' "for XInclude"
			].
		
			spec baseline: 'ArchC' with: [
				spec repository: 'github://shingarov/Pharo-ArchC:pure-z3'
			].
		
			spec
				package: #'GDB' with:[spec requires: #('PetitParser' 'ArchC')]
		]
]
