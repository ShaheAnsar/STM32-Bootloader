MEMORY{
	flash(rx) : ORIGIN = 0x08000000, LENGTH = 64K
	ram(rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

EXTERN(vector_table)

ENTRY(start)

SECTIONS{
	.vector_table : {
		      . = ALIGN(4);
		      KEEP(*(.vector_table))
		      . = ALIGN(4);
	} > flash

	.text : {
	      . = ALIGN(4);
	      *(.text)
	      *(.text*)
	      *(.glue_7)
	      *(.glue_7t)
	      *(.eh_frame)
	      KEEP(*(.init))
	      KEEP(*(.fini))
	      . = ALIGN(4);
	} > flash

	.rodata : {
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
	} > flash

	.ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >flash
	.ARM : {
	     __exidx_start = .;
	     *(.ARM.exidx*)
	     __exidx_end = .;
	} >flash

	.preinit_array     :
	{
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} >flash
	.init_array :
	{
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
	} >flash
	.fini_array :
	{
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >flash
	.preinit_array     :
	{
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array*))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} >flash
	.init_array :
	{
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT(.init_array.*)))
		KEEP (*(.init_array*))
		PROVIDE_HIDDEN (__init_array_end = .);
	} >flash
	.fini_array :
	{
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT(.fini_array.*)))
		KEEP (*(.fini_array*))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >flash

	_sidata = LOADADDR(.data);
	.data : {
	      . = ALIGN(4);
	      _sdata = .;
	      *(.data)
	      *(.data*)
	      . = ALIGN(4);
	      _edata = .;
	} > ram AT> flash

	.bss : {
	     . = ALIGN(4);
	     _sbss = .;
	     __bss_start__ = .;
	     *(.bss)
	     *(.bss*)
	     *(COMMON)
	     . = ALIGN(4);
	     _ebss = .;
	     __bss_end__ = .;
	} > ram

	/DISCARD/ :
	{
	     libc.a ( * )
	     libm.a ( * )
	     libgcc.a ( * )
	}
}
