MEMORY {
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
}

ENTRY(main)

SECTIONS {
	. = ORIGIN(rom);     /* From 0x08000000 */

	.text :
	{
		*(.vectors)      /* Vector table */
		*(.text*)        /* Program code */
		. = ALIGN(4);
		*(.rodata*)      /* Read only data */
		. = ALIGN(4);
	} >rom

	. = ORIGIN(ram);     /* From 0x20000000 */

	.data :
	{
		*(.data*)      /* Read-write initialized data */
		. = ALIGN(4);
	} >ram

	.bss :
	{
		*(.bss*)       /* Read-write zero initialized data */
		. = ALIGN(4);
	} >ram AT > rom

	. = ALIGN(4);
	end = .;
}
