Class {
	#name : #AddressClassEnum,
	#superclass : #SharedPool,
	#classVars : [
		'LOC_ARG',
		'LOC_BLOCK',
		'LOC_COMMON_BLOCK',
		'LOC_COMPUTED',
		'LOC_CONST',
		'LOC_CONST_BYTES',
		'LOC_FINAL_VALUE',
		'LOC_LABEL',
		'LOC_LOCAL',
		'LOC_OPTIMIZED_OUT',
		'LOC_REF_ARG',
		'LOC_REGISTER',
		'LOC_REGPARM_ADDR',
		'LOC_STATIC',
		'LOC_TYPEDEF',
		'LOC_UNDEF',
		'LOC_UNRESOLVED'
	],
	#category : #ULDc
}

{ #category : #'class initialization' }
AddressClassEnum class >> initialize [
	"cf. gdb/symtab.h"
	
  "Not used; catches errors. "
  LOC_UNDEF := 0.

  "Value is constant int SYMBOL_VALUE, host byteorder. "
  LOC_CONST := 1.

  "Value is at fixed address SYMBOL_VALUE_ADDRESS. "
  LOC_STATIC := 2.

  "Value is in register.  SYMBOL_VALUE is the register number
     in the original debug format.  SYMBOL_REGISTER_OPS holds a
     function that can be called to transform this into the
     actual register number this represents in a specific target
     architecture (gdbarch).
     For some symbol formats (stabs, for some compilers at least),
     the compiler generates two symbols, an argument and a register.
     In some cases we combine them to a single LOC_REGISTER in symbol
     reading, but currently not for all cases (e.g. it's passed on the
     stack and then loaded into a register). " 
  LOC_REGISTER := 3.

  "It's an argument; the value is at SYMBOL_VALUE offset in arglist. "
  LOC_ARG := 4.

  "Value address is at SYMBOL_VALUE offset in arglist. "
  LOC_REF_ARG := 5.

  "Value is in specified register.  Just like LOC_REGISTER except the
     register holds the address of the argument instead of the argument
     itself.  This is currently used for the passing of structs and unions
     on sparc and hppa.  It is also used for call by reference where the
     address is in a register, at least by mipsread.c. " 
  LOC_REGPARM_ADDR := 6.

  "Value is a local variable at SYMBOL_VALUE offset in stack frame. "
  LOC_LOCAL := 7.

  "Value not used; definition in SYMBOL_TYPE.  Symbols in the domain
     STRUCT_DOMAIN all have this class. " 
  LOC_TYPEDEF := 8.

  "Value is address SYMBOL_VALUE_ADDRESS in the code. "
  LOC_LABEL := 9.


  "  In a symbol table, value is SYMBOL_BLOCK_VALUE of a `struct block'.
     In a partial symbol table, SYMBOL_VALUE_ADDRESS is the start address
     of the block.  Function names have this class.  "

  LOC_BLOCK := 10.

  "Value is a constant byte-sequence pointed to by SYMBOL_VALUE_BYTES, in
     target byte order. " 
  LOC_CONST_BYTES := 11.

  "Value is at fixed address, but the address of the variable has
     to be determined from the minimal symbol table whenever the
     variable is referenced.
     This happens if debugging information for a global symbol is
     emitted and the corresponding minimal symbol is defined
     in another object file or runtime common storage.
     The linker might even remove the minimal symbol if the global
     symbol is never referenced, in which case the symbol remains
     unresolved.

     GDB would normally find the symbol in the minimal symbol table if it will
     not find it in the full symbol table.  But a reference to an external
     symbol in a local block shadowing other definition requires full symbol
     without possibly having its address available for LOC_STATIC.  Testcase
     is provided as `gdb.dwarf2/dw2-unresolved.exp'.

     This is also used for thread local storage (TLS) variables.  In this case,
     the address of the TLS variable must be determined when the variable is
     referenced, from the MSYMBOL_VALUE_RAW_ADDRESS, which is the offset
     of the TLS variable in the thread local storage of the shared
     library/object. " 
  LOC_UNRESOLVED := 12.

  "The variable does not actually exist in the program.
     The value is ignored. " 
  LOC_OPTIMIZED_OUT := 13.

  "The variable's address is computed by a set of location
     functions (see 'struct symbol_computed_ops' below)."
  LOC_COMPUTED := 14.

  "The variable uses general_symbol_info->value->common_block field.
     It also always uses COMMON_BLOCK_DOMAIN. "
  LOC_COMMON_BLOCK := 15.

  "Not used, just notes the boundary of the enum. "
  LOC_FINAL_VALUE := 16.
]
