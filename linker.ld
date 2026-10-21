ENTRY(_start)

SECTIONS
{
    . = 1M;

    .multiboot ALIGN(4) :
    {
        *(.multiboot)
    }

    .text ALIGN(4K) :
    {
        *(.text*)
    }

    .rodata ALIGN(4K) :
    {
        *(.rodata*)
    }

    .data ALIGN(4K) :
    {
        *(.data*)
    }

    /* Глубокий аудит #220: BSS проверен — _bss_start стоит после
     * ALIGN(4K), т.е. кратен 4096 (и тем более 4) — rep stosl в boot.S
     * стартует с выровненного адреса, хвост <4 байт добивается stosb.
     * ЖЁСТКОГО ЛИМИТА РОСТА НЕТ: секция тянется свободно (fb_backbuf
     * ~3 МБ + близнец klog 16 КБ уже здесь), ядро грузится с 1 МБ при
     * RAM от 128 МБ — запас огромен.  HDD-путь: загрузчик копирует
     * лишь 384 КБ образа, но BSS обнуляет УЖЕ boot.S ПО МАРКЕРАМ
     * _bss_start/_bss_end (rep stosl в PM) — рост BSS подхватывается
     * автоматически, за пределами скопированных 384 КБ обнуление
     * тоже доходит (лимит плоского сегмента — 4 ГБ). */
    .bss ALIGN(4K) :
    {
        _bss_start = .;
        *(COMMON)
        *(.bss*)
        /* близнец журнала — 16 КБ в САМОМ КОНЦЕ.bss (за всеми
         * данными и 3 МБ fb_backbuf).  NOBITS: в файл не попадает,
         * boot.S очищает (внутри _bss_start.._bss_end). */
        . = ALIGN(4096);
        _klog2_start = .;
        . += 0x4000;
        _bss_end = .;
    }

    /* Embedded flat binary of the kernel (written to LBA 2048 by installer).
       Filled in by boot/embed.S via .incbin "build/egorov.bin". */
    .kimage ALIGN(4K) :
    {
        _kernel_image_start = .;
        *(.kimage)
        _kernel_image_end = .;
    }

    /* Embedded HDD bootloader binary (written to sector 0 / MBR by installer).
       Filled in by boot/embed.S via .incbin "build/hdd_boot.bin". */
    .bootloader ALIGN(4K) :
    {
        _bootloader_start = .;
        *(.bootloader)
        _bootloader_end = .;
    }
}
