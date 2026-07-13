
MEMORY
{
PAGE 0 :
   BEGIN            : origin = 0x000000, length = 0x000002
   RAMM0            : origin = 0x0000F5, length = 0x00030B

   RAMLS0           : origin = 0x008000, length = 0x000800
   RAMLS1           : origin = 0x008800, length = 0x000800
   RAMLS2           : origin = 0x009000, length = 0x000800
   RAMLS3           : origin = 0x009800, length = 0x000800
   RAMLS4           : origin = 0x00A000, length = 0x000800
   RAMLS5           : origin = 0x00A800, length = 0x000800
   RAMLS6           : origin = 0x00B000, length = 0x000800
   RAMLS7           : origin = 0x00B800, length = 0x000800
   RAMGS3           : origin = 0x012000, length = 0x002000   // 从PAGE1移入, 扩大.text可用空间
   RESET            : origin = 0x3FFFC0, length = 0x000002

PAGE 1 :
   BOOT_RSVD        : origin = 0x000002, length = 0x0000F3
   RAMM1            : origin = 0x000400, length = 0x000400

   RAMGS0           : origin = 0x00C000, length = 0x002000
   RAMGS1           : origin = 0x00E000, length = 0x002000
   RAMGS2           : origin = 0x010000, length = 0x002000
}


SECTIONS
{
   codestart        : > BEGIN,     PAGE = 0
   .TI.ramfunc      : > RAMM0      PAGE = 0
   .text            : >>RAMM0 | RAMLS0 | RAMLS1 | RAMLS2 | RAMLS3 | RAMLS4 | RAMLS5 | RAMLS6 | RAMLS7 | RAMGS3, PAGE = 0
   .cinit           : > RAMLS7,    PAGE = 0
   .pinit           : > RAMM0,     PAGE = 0
   .switch          : > RAMM0,     PAGE = 0
   .init_array      : > RAMM0,     PAGE = 0
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT

   .stack           : > RAMM1,     PAGE = 1
   .ebss            : > RAMGS0,    PAGE = 1
   .bss             : > RAMGS0,    PAGE = 1
   .econst          : > RAMGS1,    PAGE = 1
   .const           : > RAMGS1,    PAGE = 1
   .data            : > RAMGS2,    PAGE = 1
   .sysmem          : > RAMGS2,    PAGE = 1
   .esysmem         : > RAMGS2,    PAGE = 1
}
