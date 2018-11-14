/*
 * GENERATED FILE - DO NOT EDIT
 * (c) Code Red Technologies Ltd, 2008-2013
 * (c) NXP Semiconductors 2013-2018
 * Generated linker script file for MKE04Z8xxx4
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.23
 * MCUXpresso IDE v10.2.1 [Build 795] [2018-07-25] on 12-Nov-2018 9:31:32 PM
 */

MEMORY
{
  /* Define each memory region */
  PROGRAM_FLASH (rx) : ORIGIN = 0x0, LENGTH = 0x2000 /* 8K bytes (alias Flash) */  
  SRAM (rwx) : ORIGIN = 0x1fffff00, LENGTH = 0x400 /* 1K bytes (alias RAM) */  
}

  /* Define a symbol for the top of each memory region */
  __base_PROGRAM_FLASH = 0x0  ; /* PROGRAM_FLASH */  
  __base_Flash = 0x0 ; /* Flash */  
  __top_PROGRAM_FLASH = 0x0 + 0x2000 ; /* 8K bytes */  
  __top_Flash = 0x0 + 0x2000 ; /* 8K bytes */  
  __base_SRAM = 0x1fffff00  ; /* SRAM */  
  __base_RAM = 0x1fffff00 ; /* RAM */  
  __top_SRAM = 0x1fffff00 + 0x400 ; /* 1K bytes */  
  __top_RAM = 0x1fffff00 + 0x400 ; /* 1K bytes */  
