/**
 LD script for flash and ram 19/02
**/ 

ENTRY(Reset_Handler)

MEMORY {
	flash (rx) : ORIGIN = 0x00000000, LENGTH = 512k
	ram0 (rwx) : ORIGIN = 0x10000000, LENGTH = 32k
	ram1 (rwx) : ORIGIN = 0x2007C000, LENGTH = 32k
}

_ram0  = ORIGIN(ram0);
_ram1  = ORIGIN(ram1);
_stack = ORIGIN(ram0) + LENGTH(ram0);	

SECTIONS {
	.text : 
	{
		KEEP(*(.startup .startup.*))
		*(.text)
		*(.text*)		
		. = ALIGN(4);
		_etext = .;
		_stext = LOADADDR(.text);
		_estartup = _stext;
	} > flash

	.rodata :
  	{
		. = ALIGN(4);
		*(.rodata)         /* .rodata sections (constants, strings, etc.) */
		*(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
		. = ALIGN(4);
	} > flash
	
	.ram_code : 			/* functions that will run on ram */
	{
		. = ALIGN(4);
		_siramcode = LOADADDR(.ram_code);
		_sramcode = .;
		*(.ram_code)
		*(.ramcode.*)
		. = ALIGN(4);
		_eramcode = .;
	} > ram0 AT > flash
	
	.data : 
	{
		. = ALIGN(4);
		_sidata = LOADADDR (.data); /* get the address where data content is stored on flash*/
		_sdata = .;			        /* get the actual data address on ram */
		*(.data)
		*(.data.*)				
		. = ALIGN(4);
		_edata = .;
	} > ram0 AT > flash  /* instructs that the content of data section is stored no flash */
    
	.bss : 
	{
		. = ALIGN(4);		
		_sbss = .;
		__bss_start__ = .;
		*(.bss)
		*(.bss.*)		
		. = ALIGN(4);
		_ebss = .;
		__bss_end__ = .;
		. = ALIGN(8);
		_end = .;        /* used for heap management */
	} > ram0

	/*section for libc exceptions and stack trace */
	.ARM.exidx ALIGN(4) : {		 /* understand this...., required for cpp*/
		__exidx_start = LOADADDR(.ARM.exidx);
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)		
		__exidx_end = .;
	} > ram0
	
/*		
	.stack : {
		. = ALIGN(4);		
		*(.stack*)
		. = ALIGN(4);	
		end = .;	
	} > ram0
*/
	
}
