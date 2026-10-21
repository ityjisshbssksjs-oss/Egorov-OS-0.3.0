/*
 * Egorov OS — embedded installer payload (second link pass).
 * .incbin bakes egorov.bin and hdd_boot.bin in, so the linker-defined
 * boundary symbols (_kernel_image_start/_end, _bootloader_start/_end)
 * get the real byte lengths.  ("objcopy --add-section" would leave them
 * at size 0 and the installer would write nothing to disk.)
 */

.section .kimage, "a"
.incbin "build/egorov.bin"

.section .bootloader, "a"
.incbin "build/hdd_boot.bin"
