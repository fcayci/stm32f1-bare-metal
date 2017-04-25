ENTRY(main)

MEMORY {
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
}

SECTIONS {
	.text :
	{
		KEEP(*(.vectors))  /* Vector table */
		*(.text*)          /* Program code */
		*(.rodata*)        /* Read only data */
		. = ALIGN(4);
	} >rom

	.data :
	{
		*(.data*)      /* Read-write initialized data */
		. = ALIGN(4);
	} >ram AT >rom

	.bss :
	{
		. = ALIGN(4);
		*(.bss*)       /* Read-write zero initialized data */
		*(COMMON)
		. = ALIGN(4);
	} >ram
}
