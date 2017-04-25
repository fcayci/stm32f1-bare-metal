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
		*(.text)
		*(.text*)        /* Program code */
		*(.gnu.linkonce.t.*)
		. = ALIGN(4);
		*(.rodata)
		*(.rodata*)      /* Read only data */
		*(.gnu.linkonce.r.*)
		. = ALIGN(4);
	} >rom

	. = ORIGIN(ram);     /* From 0x20000000 */

	.data :
	{
		*(.data)
		*(.data*)      /* Read-write initialized data */
		*(.gnu.linkonce.d.*)
		. = ALIGN(4);
	} >ram AT > rom

	.bss :
	{
		*(.bss)
		*(.bss*)       /* Read-write zero initialized data */
		*(.gnu.linkonce.b.*)
		. = ALIGN(4);
	} >ram AT > rom

	. = ALIGN(4);
	end = .;
}
