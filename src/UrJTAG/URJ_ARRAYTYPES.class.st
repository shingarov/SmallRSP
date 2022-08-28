Class {
	#name : #'URJ_ARRAYTYPES',
	#superclass : #SharedPool,
	#classVars : [
		'URJ_DATA_REGISTER_NAME',
		'URJ_PART_INSTRUCTION_INSTRUCTION',
		'URJ_PART_MANUFACTURER',
		'URJ_PART_PART',
		'URJ_PART_STEPPING'
	],
	#category : #UrJTAG
}

{ #category : #'class initialization' }
URJ_ARRAYTYPES class >> initialize [
	URJ_PART_MANUFACTURER := FFIArray newArrayTypeOf: #char size: 25+1.
	URJ_PART_PART         := FFIArray newArrayTypeOf: #char size: 20+1.
	URJ_PART_STEPPING     := FFIArray newArrayTypeOf: #char size:  8+1.
	URJ_PART_INSTRUCTION_INSTRUCTION := FFIArray newArrayTypeOf: #char size:  20+1.
	URJ_DATA_REGISTER_NAME := FFIArray newArrayTypeOf: #char size:  32+1.
	"URJ_ARRAYTYPES initialize"
]
