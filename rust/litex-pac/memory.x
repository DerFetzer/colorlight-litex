MEMORY {
	sram : ORIGIN = 0x10000000, LENGTH = 0x00002000
	main_ram : ORIGIN = 0x40000000, LENGTH = 0x00400000
	spiflash : ORIGIN = 0x20000000, LENGTH = 0x02000000
	rom : ORIGIN = 0x20100000, LENGTH = 0x01f00000
	ethmac : ORIGIN = 0x80000000, LENGTH = 0x00002000
	csr : ORIGIN = 0xf0000000, LENGTH = 0x00010000
}

REGION_ALIAS("REGION_TEXT", spiflash);
REGION_ALIAS("REGION_RODATA", spiflash);
REGION_ALIAS("REGION_DATA", sram);
REGION_ALIAS("REGION_BSS", sram);
REGION_ALIAS("REGION_HEAP", sram);
REGION_ALIAS("REGION_STACK", sram);

/* CPU reset location. */
_stext = 0x20100000;

SECTIONS
{
    .main_ram (NOLOAD) : ALIGN(4)
    {
        *(.main_ram .main_ram.*);
        . = ALIGN(4);
    } > main_ram
} INSERT AFTER .bss;
