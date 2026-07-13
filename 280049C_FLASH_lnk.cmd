
MEMORY
{
PAGE 0 :
   BEGIN            : origin = 0x080000, length = 0x000002
   FLASH_BANK0      : origin = 0x080002, length = 0x00FFFE   /* 64KB - 2B */
   FLASH_BANK1      : origin = 0x090000, length = 0x010000   /* 64KB */

   RAMM0            : origin = 0x0000F5, length = 0x00030B
   RAMLS0           : origin = 0x008000, length = 0x000800
   RAMLS1           : origin = 0x008800, length = 0x000800
   RAMLS2           : origin = 0x009000, length = 0x000800
   RAMLS3           : origin = 0x009800, length = 0x000800
   RAMLS4           : origin = 0x00A000, length = 0x000800
   RAMLS5           : origin = 0x00A800, length = 0x000800
   RAMLS6           : origin = 0x00B000, length = 0x000800
   RAMLS7           : origin = 0x00B800, length = 0x000800
   RESET            : origin = 0x3FFFC0, length = 0x000002

PAGE 1 :
   BOOT_RSVD        : origin = 0x000002, length = 0x0000F3
   RAMM1            : origin = 0x000400, length = 0x000400

   RAMGS0           : origin = 0x00C000, length = 0x002000
   RAMGS1           : origin = 0x00E000, length = 0x002000
   RAMGS2           : origin = 0x010000, length = 0x002000
   RAMGS3           : origin = 0x012000, length = 0x002000
}


SECTIONS
{
   codestart        : > BEGIN,       PAGE = 0
   .TI.ramfunc      : LOAD = FLASH_BANK0,
                      RUN = RAMLS0,
                      LOAD_START(RamfuncsLoadStart),
                      LOAD_SIZE(RamfuncsLoadSize),
                      RUN_START(RamfuncsRunStart),
                      PAGE = 0
   .text            : > FLASH_BANK0 | FLASH_BANK1,   PAGE = 0
   .cinit           : > FLASH_BANK0, PAGE = 0
   .pinit           : > FLASH_BANK0, PAGE = 0
   .switch          : > FLASH_BANK0, PAGE = 0
   .init_array      : > FLASH_BANK0, PAGE = 0
   .reset           : > RESET,       PAGE = 0, TYPE = DSECT

   .stack           : > RAMM1,       PAGE = 1
   .ebss            : > RAMGS0,      PAGE = 1
   .bss             : > RAMGS0,      PAGE = 1
   .econst          : > FLASH_BANK0, PAGE = 0
   .const           : > FLASH_BANK0, PAGE = 0
   .data            : > RAMGS2,      PAGE = 1
   .sysmem          : > RAMGS2,      PAGE = 1
   .esysmem         : > RAMGS2,      PAGE = 1

   ramgs0           : > RAMGS3,      PAGE = 1
   ramgs1           : > RAMGS3,      PAGE = 1
}
