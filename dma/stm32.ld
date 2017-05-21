ENTRY(main)

MEMORY {
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
}

SECTIONS {
	.text ORIGIN(rom) :
	{
		KEEP(*(.vectors))  /* Vector table */
		*(.text*)          /* Program code */
		*(.rodata*)        /* Read only data */
		. = ALIGN(4);
		_etext = .;
	}

	_sidata = .;

	.data ORIGIN(ram) :  AT ( ADDR (.text) + SIZEOF (.text) )
	{
		_sdata = .;
		*(.data*)      /* Read-write initialized data */
		. = ALIGN(4);
		_edata = .;
	}


	.bss ADDR (.data) + SIZEOF (.data):
	{
		_sbss = .;
		. = ALIGN(4);
		*(.bss*)       /* Read-write zero initialized data */
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	}
}

__text_size = SIZEOF (.text);
__data_size = SIZEOF (.data);
__bss_size  = SIZEOF (.bss);
