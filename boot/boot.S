/* Multiboot1 header + 32-bit entry point for Egorov OS.
 * Requests a linear VBE framebuffer at 1024x768x32.
 *
 * Video flag (bit 2) is required: without it GRUB leaves the multiboot
 * framebuffer fields empty and the kernel panics.  The mode can still be
 * overridden by grub.cfg if needed. */

.set MB_MAGIC,    0x1BADB002
/* bit0=page-align modules, bit1=mem info, bit2=video mode request */
.set MB_FLAGS,    0x00000007
.set MB_CHECKSUM, -(MB_MAGIC + MB_FLAGS)

.section .multiboot, "a"
.align 4
    .long MB_MAGIC
    .long MB_FLAGS
    .long MB_CHECKSUM
    /* AOUT kludge unused (5 zeros). */
    .long 0
    .long 0
    .long 0
    .long 0
    .long 0
    /* Video request: linear graphics, 1024x768x32. */
    .long 0
    .long 1024
    .long 768
    .long 32

.section .bss
.align 16
stack_bottom:
    .skip 65536
stack_top:

.section .text
.code32
.global _start
.type _start, @function
/* Inline COM1 byte write for early boot debug (destroys AL, DX only) */
.macro com1b val
    mov  $0x3F8, %dx
    mov  $\val,  %al
    out  %al, %dx
.endm

_start:
    cli
    /* Debug: reached _start */
    mov  $0x3F8, %dx
    mov  $'1', %al
    out  %al, %dx

    /* EAX = Multiboot magic (0x2BADB002), EBX = multiboot_info pointer.
     * Zero BSS before entering C: the HDD bootloader's flat binary has no
     * NOBITS section, so BSS may hold stale RAM (the 3 MB back-buffer lives
     * in BSS - junk there would corrupt the first frame).  Temporary stack
     * at 0x7BFE (stack_top itself is inside BSS); EBX survives rep stosl. */

    mov  $0x7BFE, %esp          /* temporary stack, nothing pushed yet */

    /* v0.3.1 FIX (критично): отладочная запись '2' в COM1 стояла МЕЖДУ
     * xor %eax,%eax и rep stosl — AL='2'=0x32 становился значением
     * заполнения, и ВЕСЬ BSS заполнялся байтами 0x32 (мусор во всех
     * неинициализированных переменных: uk.attached, g_io, g_present,
     * состояние mouse_hw и т.д.). Маркер перенесён ДО установки
     * регистров заполнения. */
    /* Debug: about to zero BSS */
    mov  $0x3F8, %dx
    mov  $'2', %al
    out  %al, %dx

    cld
    mov  $_bss_start, %edi
    mov  $_bss_end,   %ecx
    sub  %edi, %ecx             /* byte count */
    mov  %ecx, %edx             /* tail (0..3 bytes), kept for after stosl */
    shr  $2,  %ecx              /* bytes -> dwords */
    xor  %eax, %eax             /* fill value = 0 — маркеры уже выведены */

    rep  stosl                  /* EBX preserved; EAX=0 — заполнение НУЛЯМИ */
    and  $3,  %edx
    mov  %edx, %ecx
    rep  stosb                  /* exact tail - nothing past _bss_end */

    /* Debug: BSS zeroed */
    mov  $0x3F8, %dx
    mov  $'3', %al
    out  %al, %dx

    /* Глубокий аудит #219: CR0.EM (бит 2) и CR0.MP (бит 1).  BIOS/загрузчик
     * могли оставить EM=1 — тогда любое FPU-подобное событие даст #NM без
     * обработчика.  Ядро собрано БЕЗ FPU-кода (-mno-sse/-mno-mmx/
     * -mgeneral-regs-only): оба бита обязаны быть чистыми — сбрасываем их
     * ДО перехода на C.  Сегментные регистры не трогаем: GRUB (multiboot)
     * и HDD-загрузчик (s2_pm32) уже вошли в PM с плоскими селекторами. */
    mov  %cr0, %eax
    and  $~0x6, %eax            /* ~0x6 = ~(EM|MP) — чистим биты 1 и 2 */
    mov  %eax, %cr0

    /* Debug: about to call kernel_main (must be BEFORE restoring EAX!) */
    mov  $0x3F8, %dx
    mov  $'4', %al
    out  %al, %dx

    mov  $stack_top,    %esp    /* switch to proper kernel stack */
    mov  $0x2BADB002,   %eax   /* restore Multiboot magic */
    push %ebx                   /* arg1: multiboot_info* */
    push %eax                   /* arg0: magic            */
    call kernel_main
.hang:
    cli
    hlt
    jmp .hang

.size _start, . - _start
