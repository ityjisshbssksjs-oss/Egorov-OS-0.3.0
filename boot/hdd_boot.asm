; ===========================================================================
; Egorov OS — HDD Two-Stage Bootloader
; ===========================================================================
; Compile with:  nasm -f bin -o build/hdd_boot.bin boot/hdd_boot.asm
;
; Stage 1 (bytes   0..511, sector 0 / MBR):
;   Loaded by BIOS at 0x7C00.
;   Loads stage 2 from disk sectors 1..63 to physical 0x7E00.
;   Passes boot-drive number to stage 2 in DL.
;
; Stage 2 (bytes 512.., sectors 1..N):
;   Loaded at 0x7E00.
;   1. Enables A20 and verifies it with a wrap-around test (port 0x92,
;      keyboard-controller fallback 0xD1/0x60 if needed).
;   2. Queries VBE info and sets mode 1024x768x32 with linear framebuffer.
;   3. Loads the Egorov OS kernel flat binary (768 sectors = 384 KB) from LBA 2048
;      into real-mode memory starting at segment 0x1000 (physical 0x10000).
;   4. Enters 32-bit protected mode.
;   5. Copies kernel from 0x10000 to 0x100000 (the load address in linker.ld).
;   6. Builds a minimal multiboot_info_t at 0x9000 with boot_device field set.
;   7. Sets EAX = 0x2BADB002, EBX = 0x9000, then jumps to 0x101000 (_start).
;
; Memory: VBE info 0x6000, mode info 0x6200, mbi 0x9000,
;         kernel staging 0x10000..0x6FFFF (384 KB = 768 sectors).
;         NOTE: сегмент 0x6000 (физ 0x60000) - это НЕ VBE-буфер (физ
;         0x6000): пересечения нет, оба могут использоваться.
; ===========================================================================

[BITS 16]
[ORG  0x7C00]

; ---------------------------------------------------------------------------
; Глубокий аудит #218: контрольная сумма ядра.
; Makefile (цель build/kernel_sum.h) считает по build/egorov.bin
; 32-битную сумму dwords + хвостовые байты (bin НЕ паддируется — см.
; .ksum-блок в PM-части).  Файл ищется по -I build и генерируется ДО
; сборки этого загрузчика (зависимости целей в Makefile: FLAT -> sum).
; ---------------------------------------------------------------------------
%include "kernel_sum.h"        ; KERNEL_SUM / KERNEL_DWORDS / KERNEL_TAIL

; ---------------------------------------------------------------------------
; STAGE 1  —  MBR, must be exactly 512 bytes ending with 0x55, 0xAA
; ---------------------------------------------------------------------------

s1_entry:
    cli
    xor  ax, ax
    mov  ds, ax
    mov  es, ax
    mov  ss, ax
    mov  sp, 0x7BFE
    sti

    mov  [s1_drive], dl          ; save BIOS boot-drive number

    ; Load stage 2: sectors 1..63 → 0x0000:0x7E00
    mov  ah, 0x42                ; INT 13h extended read
    mov  dl, [s1_drive]
    mov  si, s1_dap
    int  0x13
    jc   s1_error

    mov  dl, [s1_drive]          ; pass drive number to stage 2
    jmp  0x0000:s2_entry

s1_error:
    ; Print a short error message then freeze
    mov  si, s1_errmsg
.pchar:
    lodsb
    test al, al
    jz   .freeze
    mov  ah, 0x0E
    xor  bx, bx
    int  0x10
    jmp  .pchar
.freeze:
    cli
    hlt
    jmp  .freeze

s1_errmsg:  db 'Egorov OS: stage-2 load failed', 0x0D, 0x0A, 0
s1_drive:   db 0x80

; Disk Address Packet for stage 2 load
ALIGN 4
s1_dap:
    db  0x10            ; DAP size
    db  0               ; reserved
    dw  63              ; number of sectors to read
    dw  0x7E00          ; destination offset
    dw  0x0000          ; destination segment  → physical 0x7E00
    dq  1               ; starting LBA (sector 1)

; Pad to 510 bytes, then the mandatory boot signature
TIMES 510 - ($ - $$) db 0
dw 0xAA55

; ---------------------------------------------------------------------------
; STAGE 2  —  assembled at ORG 0x7C00+512 = 0x7E00
; ---------------------------------------------------------------------------
; Macro: send byte AL to COM1 (0x3F8), destroys AL and DX only
; ---------------------------------------------------------------------------
%macro com1_byte 1
    mov  dx, 0x3F8
    mov  al, (%1) & 0xFF
    out  dx, al
%endmacro

s2_entry:
    xor  ax, ax
    mov  ds, ax
    mov  es, ax
    mov  ss, ax
    mov  sp, 0x7BFE
    mov  [s2_drive], dl

    ; -----------------------------------------------------------------------
    ; Init COM1 (115200, 8N1) for debug
    ; -----------------------------------------------------------------------
    mov  dx, 0x3F8 + 3       ; LCR
    mov  al, 0x80
    out  dx, al              ; DLAB = 1
    mov  dx, 0x3F8 + 0       ; DLL
    mov  al, 1               ; divisor low  (115200)
    out  dx, al
    mov  dx, 0x3F8 + 1       ; DLM
    xor  al, al              ; divisor high
    out  dx, al
    mov  dx, 0x3F8 + 3       ; LCR
    mov  al, 0x03            ; 8N1, DLAB=0
    out  dx, al

    ; Debug: S2 alive
    com1_byte 'S'
    com1_byte '2'
    com1_byte 0x0D
    com1_byte 0x0A

    ; -----------------------------------------------------------------------
    ; Enable A20 line (fast-A20 port 0x92) + ПРОВЕРКА wrap-тестом.
    ; -----------------------------------------------------------------------
    in   al, 0x92
    or   al, 0x02
    and  al, 0xFE           ; keep bit 0 clear (don't trigger reset)
    out  0x92, al

    ; короткий settle: портам нужен момент на срабатывание
    mov  cx, 16
.a20_settle:
    in   al, 0x80           ; io-wait через POST-порт (чтение без эффекта)
    loop .a20_settle

    ; ── A20 wrap-тест: сравниваем байты по физ 0x00000004 и 0x100004 ──
    ; При ВЫКЛЮЧЕННОЙ A20 адрес 0x100004 «заворачивается» на 0x000004 и
    ; байты совпадают.  DS=0 -> [0x0004] (физ 4, вектор INT 1);
    ; ES=0xFFFF:0x0014 -> 0xFFFF0 + 0x14 = физ 0x100004 (1 MB + 4).
    mov  al, [0x0004]       ; байт из физ 0x000004
    mov  bx, 0xFFFF
    mov  es, bx
    mov  bl, [es:0x0014]    ; байт из физ 0x100004
    cmp  al, bl
    jne  .a20_done          ; байты различны -> A20 активна

    ; A20 не поднялась портом 0x92 -> альтернатива: контроллер клавиатуры
    ; (команда 0xD1 в порт 0x64, затем значение с битом 1 в порт 0x60).
    mov  dx, 0x64
.a20_kbd_wait1:
    in   al, dx
    test al, 2              ; input buffer full -> подождать
    jnz  .a20_kbd_wait1
    mov  al, 0xD1           ; «записать в выходной порт P2»
    out  0x64, al
.a20_kbd_wait2:
    in   al, dx
    test al, 2
    jnz  .a20_kbd_wait2
    mov  al, 0xDF           ; бит 1 = A20 enable, бит 0 = 1 (без reset)
    out  0x60, al

    mov  cx, 16
.a20_settle2:
    in   al, 0x80
    loop .a20_settle2

    ; повторный wrap-тест после kbd-пути
    mov  al, [0x0004]
    mov  bx, 0xFFFF
    mov  es, bx
    mov  bl, [es:0x0014]
    cmp  al, bl
    jne  .a20_done          ; теперь различны -> A20 активна

    ; оба способа не подняли A20: ядро на 0x100000 (>= 1 MB) работать не
    ; будет, но продолжаем загрузку - порт 0x92 на реальном железе
    ; практически всегда ок, а тест мог дать ложный негатив.
    com1_byte 'A'
    com1_byte '2'
    com1_byte '0'
    com1_byte ' '
    com1_byte 'F'
    com1_byte 'A'
    com1_byte 'I'
    com1_byte 'L'
    com1_byte 0x0D
    com1_byte 0x0A

.a20_done:
    xor  ax, ax
    mov  es, ax             ; ES обратно в 0: A20-тест ставил 0xFFFF, а
                             ; VBE-запрос и DAP-чтения ниже ждут ES=0
    com1_byte 'A'           ; A20 done

    ; -----------------------------------------------------------------------
    ; Probe VBE and find mode 1024x768x32 with linear framebuffer
    ; -----------------------------------------------------------------------
    mov  ax, 0x4F00
    mov  di, 0x6000
    mov  dword [0x6000], 0x32454256   ; signature 'VBE2'
    int  0x10
    cmp  ax, 0x004F
    jne  .vbe_fail

    ; VideoModePtr is a real-mode far pointer at VBEInfoBlock+14
    ;   [0x600E] = offset,  [0x6010] = segment
    movzx esi, word [0x600E]
    movzx eax, word [0x6010]
    mov  edx, esi
    or   edx, eax
    jz   .vbe_fail          ; VideoModePtr is null -> no VBE modes
    shl  eax, 4
    add  esi, eax           ; linear address of mode list

.scan_mode:
    cmp  esi, 0x100000      ; mode list must live below 1 MB
    jae  .vbe_fail
    mov  cx, [esi]          ; read mode number
    add  esi, 2
    cmp  cx, 0xFFFF         ; end-of-list sentinel
    je   .vbe_fail

    ; Query mode info → buffer at 0x6200.
    ; ES=0 before each INT 10h: some BIOSes clobber ES inside the handler.
    push esi
    push cx
    xor  ax, ax
    mov  es, ax
    mov  ax, 0x4F01
    mov  di, 0x6200         ; ES:DI = 0x0000:0x6200 = physical 0x6200
    int  0x10
    pop  cx
    pop  esi
    cmp  ax, 0x004F
    jne  .scan_mode

    ; VBE ModeInfoBlock offsets (VBE 2.0):
    ;  +0x00  ModeAttributes     — bit 7: linear framebuffer supported
    ;  +0x10  BytesPerScanLine
    ;  +0x12  XResolution
    ;  +0x14  YResolution
    ;  +0x19  BitsPerPixel
    ;  +0x1B  MemoryModel        — must be 6 (direct / packed 32-bit RGB)
    ;  +0x28  PhysBasePtr        — linear framebuffer physical address

    test word [0x6200], 0x0080   ; bit 7: linear FB available in this mode?
    jz   .scan_mode
    cmp  word [0x6212], 1024     ; XResolution
    jne  .scan_mode
    cmp  word [0x6214], 768      ; YResolution
    jne  .scan_mode
    cmp  byte [0x6219], 32       ; BitsPerPixel
    jne  .scan_mode
    cmp  byte [0x621B], 6        ; MemoryModel == 6 (direct colour / true colour)
    jne  .scan_mode              ; skip packed-pixel and planar modes

    ; Save mode number — we need it again after the mode is set.
    ; On many BIOSes PhysBasePtr in the ModeInfoBlock is only populated
    ; AFTER the mode is activated, so we must re-query mode info afterwards.
    mov  [s2_vbe_mode], cx

    ; Set the mode (bit 14 = use linear framebuffer).
    ; Clear ES and DI to keep INT 10h state clean.
    xor  ax, ax
    mov  es, ax
    mov  ax, 0x4F02
    mov  bx, cx
    or   bx, 0x4000         ; bit 14: request linear/flat framebuffer
    xor  di, di
    int  0x10
    ; Ignore return — if mode set fails PhysBasePtr will be 0 and we fall
    ; through to .vbe_fail naturally via the zero-check below.

    ; Re-query mode info now that the mode is active — PhysBasePtr is valid.
    ; Reset ES again (INT 10h above may have modified it).
    xor  ax, ax
    mov  es, ax
    mov  cx, [s2_vbe_mode]
    mov  ax, 0x4F01
    mov  di, 0x6200         ; ES:DI = physical 0x6200
    int  0x10
    ; If re-query fails, fall through with whatever PhysBasePtr was from the
    ; scan loop (may still be valid on some BIOSes).

    ; Read framebuffer address.  Guard against zero (VBE set-mode failure).
    mov  eax, [0x6228]           ; PhysBasePtr
    test eax, eax
    jz   .vbe_fail               ; zero address → mode not actually set
    mov  [s2_fb_addr], eax

    ; Read bytes-per-scanline with fallback if BIOS returns 0.
    movzx eax, word [0x6210]     ; BytesPerScanLine
    test eax, eax
    jnz  .pitch_nonzero
    mov  eax, 1024 * 4           ; fallback: width(1024) * 4 bytes/pixel
.pitch_nonzero:
    mov  [s2_fb_pitch], eax

    com1_byte 'V'           ; VBE OK, mode found
    jmp  .vbe_done

.vbe_fail:
    ; Could not set graphics mode; kernel will panic with a VGA text message.
    com1_byte 'F'           ; VBE Failed
    xor  eax, eax
    mov  [s2_fb_addr],  eax
    mov  [s2_fb_pitch], eax

.vbe_done:
    com1_byte 'D'           ; VBE done

    ; -----------------------------------------------------------------------
    ; Load kernel flat binary from LBA 2048 (768 sectors = 384 KB)
    ; into real-mode memory starting at physical 0x10000.
    ;
    ; Split into 6 × 128-sector reads, each aligned to a 64 KB segment
    ; boundary, to avoid both the DMA-boundary limitation (some BIOSes
    ; refuse transfers crossing a 64 KB boundary) and per-call limits.
    ;
    ; Read 1:  128 sectors, LBA 2048 → 0x1000:0x0000 (phys 0x10000)
    ; Read 2:  128 sectors, LBA 2176 → 0x2000:0x0000 (phys 0x20000)
    ; Read 3:  128 sectors, LBA 2304 → 0x3000:0x0000 (phys 0x30000)
    ; Read 4:  128 sectors, LBA 2432 → 0x4000:0x0000 (phys 0x40000)
    ; Read 5:  128 sectors, LBA 2560 → 0x5000:0x0000 (phys 0x50000)
    ; Read 6:  128 sectors, LBA 2688 → 0x6000:0x0000 (phys 0x60000)
    ;
    ; Итого стейджинг 0x10000..0x6FFFF = 384 КБ = 768 секторов; Makefile
    ; проверяет egorov.bin <= 393216 байт.  Сегмент 0x6000 = физ
    ; 0x60000 - это НЕ VBE-буфер (физ 0x6000), пересечения нет.
    ; -----------------------------------------------------------------------
    ; DS=0 so the DAP labels (ORG 0x7C00) stay correct after INT 10h.
    xor  ax, ax
    mov  ds, ax
    mov  es, ax

    ; ---- Read 1 ----
    mov  ah, 0x42
    mov  dl, [s2_drive]
    mov  si, s2_kern_dap1
    int  0x13
    jnc  .r1_ok
    mov  dx, 0x3F8 & 0xFFFF     ; 'E' + digit: E1 = read 1 failed
    mov  al, 'E'
    out  dx, al
    mov  al, '1'
    out  dx, al
.r1_ok:

    ; Restore DS=0 in case INT 13h clobbered it
    xor  ax, ax
    mov  ds, ax

    ; ---- Read 2 ----
    mov  ah, 0x42
    mov  dl, [s2_drive]
    mov  si, s2_kern_dap2
    int  0x13
    jnc  .r2_ok
    mov  dx, 0x3F8
    mov  al, 'E'
    out  dx, al
    mov  al, '2'
    out  dx, al
.r2_ok:

    xor  ax, ax
    mov  ds, ax

    ; ---- Read 3 ----
    mov  ah, 0x42
    mov  dl, [s2_drive]
    mov  si, s2_kern_dap3
    int  0x13
    jnc  .r3_ok
    mov  dx, 0x3F8
    mov  al, 'E'
    out  dx, al
    mov  al, '3'
    out  dx, al
.r3_ok:

    xor  ax, ax
    mov  ds, ax

    ; ---- Read 4 ----
    mov  ah, 0x42
    mov  dl, [s2_drive]
    mov  si, s2_kern_dap4
    int  0x13
    jnc  .r4_ok
    mov  dx, 0x3F8
    mov  al, 'E'
    out  dx, al
    mov  al, '4'
    out  dx, al
.r4_ok:

    xor  ax, ax
    mov  ds, ax

    ; ---- Read 5 ----
    mov  ah, 0x42
    mov  dl, [s2_drive]
    mov  si, s2_kern_dap5
    int  0x13
    jnc  .r5_ok
    mov  dx, 0x3F8
    mov  al, 'E'
    out  dx, al
    mov  al, '5'
    out  dx, al
.r5_ok:

    xor  ax, ax
    mov  ds, ax

    ; ---- Read 6 ----
    mov  ah, 0x42
    mov  dl, [s2_drive]
    mov  si, s2_kern_dap6
    int  0x13
    jnc  .r6_ok
    mov  dx, 0x3F8
    mov  al, 'E'
    out  dx, al
    mov  al, '6'
    out  dx, al
.r6_ok:

    xor  ax, ax
    mov  ds, ax

    com1_byte 'K'           ; Kernel load done

    ; -----------------------------------------------------------------------
    ; Enter 32-bit protected mode
    ; -----------------------------------------------------------------------
    cli
    lgdt [s2_gdtr]

    ; Null IDT: IDTR must not point at the BIOS IVT in protected mode.
    ; Real 256-entry IDT is installed by kernel_main() before enabling
    ; anything that can interrupt.
    lidt [s2_null_idtr]

    mov  eax, cr0
    or   eax, 1
    mov  cr0, eax

    ; Reload CS
    jmp  dword 0x08:s2_pm32

[BITS 32]
s2_pm32:
    mov  ax, 0x10           ; flat data segment
    mov  ds, ax
    mov  es, ax
    mov  ss, ax
    mov  fs, ax
    mov  gs, ax
    mov  esp, 0x7C000       ; stack safely above our code

    ; Debug: PM reached (COM1 in PM mode)
    mov  dx, 0x3F8
    mov  al, 'P'
    out  dx, al

    ; -----------------------------------------------------------------------
    ; Copy kernel from staging area 0x10000 → final address 0x100000
    ; 768 sectors × 512 B = 393216 B = 98304 dwords (384 КБ)
    ; -----------------------------------------------------------------------
    mov  esi, 0x10000
    mov  edi, 0x100000
    mov  ecx, (768 * 512) / 4
    rep  movsd

    ; -----------------------------------------------------------------------
    ; Глубокий аудит #218: проверка целостности скопированного ядра.
    ; Суммируем РОВНО KERNEL_DWORDS dwords (с 0x100000) и KERNEL_TAIL
    ; хвостовых байт — так же, как Makefile считал KERNEL_SUM по
    ; egorov.bin (без паддинга: хвост 1-3 байта суммируется ПОБАЙТНО,
    ; чтобы не зависеть от мусора стейджинга за пределом образа).
    ; Несовпадение = битое чтение диска/копирование — прыгать в битое
    ; ядро нельзя: код «KS» в COM1 и halt.  Кода ~45 байт: запас stage1
    ; (63 сектора = 32256 Б, было 1558) огромен.
    ; -----------------------------------------------------------------------
    mov  esi, 0x100000
    mov  ecx, KERNEL_DWORDS
    xor  eax, eax
    jecxz .ksum_tail
.ksum_dw:
    add  eax, [esi]
    add  esi, 4
    loop .ksum_dw
.ksum_tail:
    mov  ecx, KERNEL_TAIL
    jecxz .ksum_cmp
.ksum_tb:
    movzx ebx, byte [esi]
    add  eax, ebx
    inc  esi
    loop .ksum_tb
.ksum_cmp:
    cmp  eax, KERNEL_SUM
    je   .ksum_ok
.ksum_fail:
    com1_byte 'K'
    com1_byte 'S'
    com1_byte 0x0D
    com1_byte 0x0A
.ksum_halt:
    cli
    hlt
    jmp  .ksum_halt
.ksum_ok:

    ; Debug: Copy done, about to jump
    mov  dx, 0x3F8
    mov  al, 'J'
    out  dx, al

    ; -----------------------------------------------------------------------
    ; Build a minimal multiboot_info_t at 0x9000
    ; multiboot_info layout (from kernel.c, __attribute__((packed))):
    ;   +0x00  flags        (uint32)  — bit 1 = boot_device, bit 12 = framebuffer
    ;   +0x04  mem_lower    (uint32)
    ;   +0x08  mem_upper    (uint32)
    ;   +0x0C  boot_device  (uint32)  — bits 31-24: BIOS drive number
    ;   +0x58  framebuffer_addr  (uint64)
    ;   +0x60  framebuffer_pitch (uint32)
    ;   +0x64  framebuffer_width (uint32)
    ;   +0x68  framebuffer_height(uint32)
    ;   +0x6C  framebuffer_bpp   (uint8)
    ;   +0x6D  framebuffer_type  (uint8)  — 1 = direct RGB color
    ; -----------------------------------------------------------------------
    mov  edi, 0x9000
    mov  ecx, 256 / 4
    xor  eax, eax
    rep  stosd                   ; zero the whole structure

    ; Both flags required: without bit 1 detect_hdd_boot() always fails.
    mov  dword [0x9000], (1 << 12) | (1 << 1)   ; framebuffer + boot_device

    ; Set boot_device: bits 31-24 = BIOS drive number (0x80 = first HDD).
    ; The BIOS drive number was saved from DL at stage 1 entry.
    ; We encode it as: drive_byte << 24 | 0x00FFFFFF (partitions = 0xFF = unknown).
    movzx eax, byte [s2_drive]
    shl   eax, 24
    or    eax, 0x00FFFFFF        ; partition fields = 0xFF (unused/unknown)
    mov   [0x900C], eax          ; boot_device field at offset 0x0C

    mov  eax, [s2_fb_addr]
    mov  [0x9058], eax               ; framebuffer_addr  (low 32 bits)
    mov  dword [0x905C], 0           ; framebuffer_addr  (high 32 bits)
    mov  eax, [s2_fb_pitch]
    mov  [0x9060], eax               ; framebuffer_pitch
    mov  dword [0x9064], 1024        ; framebuffer_width
    mov  dword [0x9068], 768         ; framebuffer_height
    mov  byte  [0x906C], 32          ; framebuffer_bpp
    mov  byte  [0x906D], 1           ; framebuffer_type = 1 (RGB)

    ; -----------------------------------------------------------------------
    ; Debug: dump first 8 bytes at 0x101000 to COM1 as hex
    ; -----------------------------------------------------------------------
    mov  esi, 0x101000
    mov  ecx, 8
.dump_loop:
    push ecx
    push esi
    movzx ebx, byte [esi]   ; byte to dump in BL

    ; High nibble
    mov  eax, ebx
    shr  eax, 4
    and  eax, 0x0F
    add  eax, 0x30
    cmp  eax, 0x39
    jle  .h1
    add  eax, 7
.h1:
    mov  dx, 0x3F8
    out  dx, al

    ; Low nibble
    mov  eax, ebx
    and  eax, 0x0F
    add  eax, 0x30
    cmp  eax, 0x39
    jle  .l1
    add  eax, 7
.l1:
    mov  dx, 0x3F8
    out  dx, al

    ; Space separator
    mov  al, ' '
    out  dx, al

    pop  esi
    inc  esi
    pop  ecx
    loop .dump_loop

    ; CRLF
    mov  dx, 0x3F8
    mov  al, 0x0D
    out  dx, al
    mov  al, 0x0A
    out  dx, al

    ; -----------------------------------------------------------------------
    ; Launch kernel
    ; EAX = Multiboot1 magic (0x2BADB002)
    ; EBX = pointer to multiboot_info_t (0x9000)
    ; JMP  0x101000  (_start in Egorov OS — start of .text in linker.ld)
    ; -----------------------------------------------------------------------
    mov  eax, 0x2BADB002
    mov  ebx, 0x9000
    jmp  0x101000

[BITS 16]

; ---------------------------------------------------------------------------
; Stage 2 data
; ---------------------------------------------------------------------------

s2_drive:    db 0x80
s2_vbe_mode: dw 0
s2_fb_addr:  dd 0
s2_fb_pitch: dd 0

; Null IDT descriptor: limit=0, base=0 (BIOS IVT is not a valid PM IDT).
s2_null_idtr:
    dw  0           ; limit  = 0  (no valid descriptors)
    dd  0           ; base   = 0

; Six DAPs for kernel load (128 sectors each = 64 KB, 64KB-aligned segments)
; This avoids DMA boundary issues and per-call sector limits.
; Total: 6 × 64 КБ = 384 КБ staging (0x10000..0x6FFFF), LBA 2048..2815.
ALIGN 4
s2_kern_dap1:           ; Read 1: LBA 2048, 128 sectors → phys 0x10000
    db  0x10
    db  0
    dw  128
    dw  0x0000
    dw  0x1000          ; segment → 0x1000 × 16 = 0x10000
    dq  2048

ALIGN 4
s2_kern_dap2:           ; Read 2: LBA 2176, 128 sectors → phys 0x20000
    db  0x10
    db  0
    dw  128
    dw  0x0000
    dw  0x2000          ; segment → 0x2000 × 16 = 0x20000
    dq  2176            ; 2048 + 128

ALIGN 4
s2_kern_dap3:           ; Read 3: LBA 2304, 128 sectors → phys 0x30000
    db  0x10
    db  0
    dw  128
    dw  0x0000
    dw  0x3000          ; segment → 0x3000 × 16 = 0x30000
    dq  2304            ; 2048 + 256

ALIGN 4
s2_kern_dap4:           ; Read 4: LBA 2432, 128 sectors → phys 0x40000
    db  0x10
    db  0
    dw  128
    dw  0x0000
    dw  0x4000          ; segment → 0x4000 × 16 = 0x40000
    dq  2432            ; 2048 + 384

ALIGN 4
s2_kern_dap5:           ; Read 5: LBA 2560, 128 sectors → phys 0x50000
    db  0x10
    db  0
    dw  128
    dw  0x0000
    dw  0x5000          ; segment → 0x5000 × 16 = 0x50000
    dq  2560            ; 2048 + 512

ALIGN 4
s2_kern_dap6:           ; Read 6: LBA 2688, 128 sectors → phys 0x60000
    db  0x10
    db  0
    dw  128
    dw  0x0000
    dw  0x6000          ; segment → 0x6000 × 16 = 0x60000
    dq  2688            ; 2048 + 640

; GDT — must be 8-byte aligned
ALIGN 8
s2_gdt_base:
    dq  0                           ; [0x00] null descriptor
    ; [0x08] 32-bit code: base=0, limit=4 GB, DPL=0, exec/read
    db  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x9A, 0xCF, 0x00
    ; [0x10] 32-bit data: base=0, limit=4 GB, DPL=0, read/write
    db  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x92, 0xCF, 0x00
s2_gdt_end:

s2_gdtr:
    dw  s2_gdt_end - s2_gdt_base - 1   ; limit
    dd  s2_gdt_base                     ; base (physical = label value, DS=0)
