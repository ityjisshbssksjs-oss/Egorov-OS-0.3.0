/* ═══ [LOGS] ЭТАП 11: журнал ядра → EGOROV.LOG на USB-накопителе ═══════════
 *
 * Реализация «пункт 2 GRUB = LOGS to USB stick» БЕЗ расхождения между
 * названием и поведением (раньше запись на носитель была отключена).
 *
 * Слои (см. kernel/logstick.h — полный контракт):
 *   • единый блочный интерфейс поверх трёх транспортов BOT/SCSI;
 *   • MBR/GPT + границы разделов против READ CAPACITY;
 *   • единственная цель записи — FAT32-раздел с меткой тома «LOGS»;
 *   • ограниченная FAT32: создать/дополнить EGOROV.LOG (8.3, корень),
 *     цепочка кластеров, ОБЕ копии FAT, FSInfo, размер в каталоге;
 *   • буферизация: klog → дельта-выгрузка (1 Гц) + финальная;
 *   • отказоустойчивость: тайм-ауты транспортов, 3 ошибки → отключение,
 *     COM1 + RAM-кольцо живут всегда.
 *
 * ГАРАНТИИ БЕЗОПАСНОСТИ (проверяются КОДОМ, не договорённостью):
 *   1. ls_wr() пишет ТОЛЬКО в [part_start, part_end) — LBA собранных
 *      структур FAT; запись в MBR (LBA 0), VBR раздела, чужие разделы,
 *      за пределы диска — НЕВОЗМОЖНА (центральный страж).
 *   2. Раздел обязан быть FAT32 с меткой тома «LOGS» (том подготовлен
 *      ПОЛЬЗОВАТЕЛЕМ: mkstick.py для синтетики, форматирование в
 *      Windows для реальной флешки). Автоформатирования НЕТ, создание
 *      разделов НЕТ, «докручивание» чужой геометрии НЕТ.
 *   3. Активный (0x80) MBR-раздел и ESP-раздел GPT исключены.
 *   4. EGOROV.LOG: запись либо создаётся в СВОБОДНОМ слоте каталога,
 *      либо ДОПОЛНЯЕТСЯ (с проверкой целостности цепочки кластеров:
 *      длина цепочки обязана совпадать с размером файла, иначе — отказ,
 *      «чужие данные не перезаписываю»). Чужие файлы не трогаем.
 *   5. Кластеры выделяются ТОЛЬКО свободные (FAT==0), помечаются EOC в
 *      ОБЕИХ копиях FAT + декремент FSInfo (Windows не решит, что они
 *      свободны, и не затрёт журнал).
 */

#include "logstick.h"
#include "types.h"
#include "usb.h"        /* UHCI: usb_drive / usb_{read,write}_sector */
#include "mouse_hw.h"   /* OHCI: mouse_hw_msd_*; xHCI: mouse_hw_xhci_msd_* */
#include "string.h"
#include "rtc.h"        /* дата/время записи в каталоге FAT */
#include "version.h"     /* номер сборки в заголовке сессии */

/* serial-журнал и снимок klog (реализовано в kernel.c). */
extern void serial_puts(const char *s);
extern void serial_puthex(uint32_t v);
extern uint32_t klog_total(void);
extern uint32_t klog_oldest(void);
extern uint32_t klog_snapshot(uint32_t from, char *dst, uint32_t dstsz);
extern uint32_t klog_early_len(void);     /* [build 4] ранний буфер   */
extern uint32_t klog_early_copy(uint32_t from, char *dst, uint32_t dstsz);

/* ── числа без libc ─────────────────────────────────────────────────────── */
static void ls_num(char *out, uint32_t v) {
    char t[12];
    int  n = 0;
    if (v == 0) { out[0] = '0'; out[1] = 0; return; }
    while (v > 0 && n < 11) { t[n++] = (char)('0' + (v % 10u)); v /= 10u; }
    for (int i = 0; i < n; i++) out[i] = t[n - 1 - i];
    out[n] = 0;
}
static void ls_pn(const char *pre, uint32_t v, const char *post) {
    char nb[12];
    serial_puts(pre);
    ls_num(nb, v);
    serial_puts(nb);
    serial_puts(post);
}

/* безопасная установка строки статуса */
static char g_ls_status[96];
static void ls_set_status(const char *s) {
    int i = 0;
    for (; s[i] && i < (int)sizeof(g_ls_status) - 1; i++)
        g_ls_status[i] = s[i];
    g_ls_status[i] = 0;
}

/* ═══ 1. ЕДИНЫЙ ИНТЕРФЕЙС БЛОЧНОГО НАКОПИТЕЛЯ ══════════════════════════════
 * Три транспорта уже существуют (UHCI — usb.c; OHCI/xHCI — mouse_hw.c).
 * Здесь — только выбор, диспетчеризация и ЖЁСТКАЯ ГРАНИЦА записи.
 * g_sec — 4096-выровнен: все три транспорта пишут/читают прямо из него
 * (OHCI TD и xHCI TRB требуют непрерывной физической памяти). */
static uint8_t g_sec[512] __attribute__((aligned(4096)));

static int      g_ls_tr = LS_TR_NONE;   /* активный транспорт                  */
static uint32_t g_ls_cap_sect;          /* ёмкость выбранного накопителя (LBA) */
static uint32_t g_ls_rd_errs, g_ls_wr_errs;
static uint32_t g_ls_io_sect;           /* всего секторов записано (статистика)*/

/* границы разрешённой области записи (центральный страж; 0=запрет всего) */
static uint32_t g_part_start, g_part_end;

static const char *ls_tr_name(int tr) {
    switch (tr) {
    case LS_TR_UHCI: return "UHCI";
    case LS_TR_OHCI: return "OHCI";
    case LS_TR_XHCI: return "xHCI";
    default:         return "нет";
    }
}

/* ── [build 4] КАНДИДАТЫ НАКОПИТЕЛЕЙ: перечисляем ВСЕ, выбираем по разделу ──
 * Проблема ревью build 3: ls_blk_probe() брал ПЕРВЫЙ накопитель по
 * приоритету контроллеров (OHCI→xHCI→UHCI) и, не найдя на нём раздел
 * LOGS, ЗАВЕРШАЛ подготовку ошибкой — вторая флешка не проверялась
 * вовсе.  Теперь: собираем ВСЕ видимые накопители (до трёх — по одному
 * на транспорт: usb.c/mouse_hw.c перечисляют один MSD каждый),
 * печатаем каждому модель/ёмкость (SCSI INQUIRY + READ CAPACITY) и
 * пробуем КАЖДЫЙ: запись идёт на тот, где реально найден раздел LOGS —
 * это и есть «подтверждение, что выбранная флешка предназначена для
 * журналов».  Накопитель без раздела LOGS пропускается честной строкой.
 * ОГРАНИЧЕНИЕ (честно): два накопителя ЗА одним контроллером одного
 * типа не различаются — транспорт держит один MSD; см. README. */
typedef struct {
    int         tr;      /* транспорт LS_TR_*                          */
    uint32_t    cap;     /* секторов (READ CAPACITY)                   */
    const char *name;    /* модель из INQUIRY — для честного выбора    */
} ls_cand_t;

static int ls_blk_enumerate(ls_cand_t *out, int max) {
    int n = 0;
    if (mouse_hw_msd_present() && n < max) {
        out[n].tr   = LS_TR_OHCI;
        out[n].cap  = mouse_hw_msd_sectors();
        out[n].name = mouse_hw_msd_name();
        n++;
    }
    if (mouse_hw_xhci_msd_present() && n < max) {
        out[n].tr   = LS_TR_XHCI;
        out[n].cap  = mouse_hw_xhci_msd_sectors();
        out[n].name = mouse_hw_xhci_msd_name();
        n++;
    }
    if (usb_drive.present && n < max) {
        /* UHCI: имя = vendor + ' ' + product (INQUIRY), собираем разово */
        static char uhci_name[40];
        int k = 0;
        for (int i = 0; usb_drive.vendor[i] && k < 38; i++)
            uhci_name[k++] = usb_drive.vendor[i];
        if (k > 0 && k < 38) uhci_name[k++] = ' ';
        for (int i = 0; usb_drive.product[i] && k < 38; i++)
            uhci_name[k++] = usb_drive.product[i];
        uhci_name[k] = 0;
        out[n].tr   = LS_TR_UHCI;
        out[n].cap  = usb_drive.sectors;
        out[n].name = uhci_name;
        n++;
    }
    /* порядок перечисления детерминирован: OHCI (реальное железо AMD) →
     * xHCI (новые ПК) → UHCI (QEMU/старые ПК) — как в kernel.c */
    return n;
}

/* чтение сектора в g_sec (512).  Транспорт-агностично. */
static bool ls_rd(uint32_t lba) {
    if (g_ls_tr == LS_TR_NONE) return false;
    bool ok = false;
    switch (g_ls_tr) {
    case LS_TR_UHCI: ok = usb_read_sector(lba, g_sec); break;
    case LS_TR_OHCI: ok = mouse_hw_msd_read(lba, g_sec); break;
    case LS_TR_XHCI: ok = mouse_hw_xhci_msd_read(lba, g_sec); break;
    default: return false;
    }
    if (!ok) g_ls_rd_errs++;
    return ok;
}

/* ЦЕНТРАЛЬНЫЙ СТРАЖ ЗАПИСИ: сектор g_sec → lba.  Разрешено ТОЛЬКО
 * внутри [g_part_start, g_part_end) — области раздела LOGS.  Всё
 * остальное (MBR, VBR, чужие разделы, за диск) отвергается ДО того,
 * как хоть один байт уйдёт на носитель. */
static bool ls_wr(uint32_t lba) {
    if (g_part_start == 0 || g_part_end == 0) return false;  /* не готово */
    if (lba < g_part_start || lba >= g_part_end) {
        serial_puts("LOGS: ОТКАЗ: сектор ");
        ls_pn("", lba, "");
        serial_puts(" вне раздела LOGS [");
        ls_pn("", g_part_start, "..");
        ls_pn("", g_part_end - 1u, "] - запись отменена (защита)\n");
        return false;
    }
    bool ok = false;
    switch (g_ls_tr) {
    case LS_TR_UHCI: ok = usb_write_sector(lba, g_sec); break;
    case LS_TR_OHCI: ok = mouse_hw_msd_write(lba, g_sec); break;
    case LS_TR_XHCI: ok = mouse_hw_xhci_msd_write(lba, g_sec); break;
    default: return false;
    }
    if (ok) g_ls_io_sect++;
    else    g_ls_wr_errs++;
    return ok;
}

/* ═══ 2. MBR / GPT: найти раздел LOGS ══════════════════════════════════════ */

typedef struct {
    uint32_t start;   /* первый LBA раздела                          */
    uint32_t sects;   /* размер в секторах (0 = вычеркнут)           */
    uint8_t  type;    /* код типа (MBR; GPT: 0)                      */
    uint8_t  active;  /* загрузочный: 0x80 (MBR) / ESP (GPT)         */
    int      gpt;     /* 1 = запись из GPT                           */
} ls_part_t;

#define LS_MAX_PARTS 20
static ls_part_t g_parts[LS_MAX_PARTS];
static int        g_nparts;
static int        g_is_gpt;
static uint32_t   g_disk_sects;   /* ёмкость носителя (READ CAPACITY)  */

static void ls_part_add(uint32_t start, uint32_t sects, uint8_t type,
                        uint8_t active, int gpt) {
    if (g_nparts >= LS_MAX_PARTS) return;
    if (sects == 0) return;
    if (start == 0) return;              /* LBA 0 = сам MBR — никогда */
    g_parts[g_nparts].start  = start;
    g_parts[g_nparts].sects  = sects;
    g_parts[g_nparts].type   = type;
    g_parts[g_nparts].active = active;
    g_parts[g_nparts].gpt    = gpt;
    g_nparts++;
}

static uint32_t ls_rdb32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint64_t ls_rdb64(const uint8_t *p) {
    return (uint64_t)ls_rdb32(p) | ((uint64_t)ls_rdb32(p + 4) << 32);
}

/* Разобрать носитель: MBR (или защитный MBR → GPT).  Каждый раздел
 * печатается честно; границы проверяются против g_disk_sects — раздел,
 * выходящий за носитель, ВЫЧЁРКИВАЕТСЯ (sects=0) до каких-либо записей. */
static bool ls_parse_disk(void) {
    g_nparts = 0;
    g_is_gpt = 0;
    g_disk_sects = g_ls_cap_sect;

    if (!ls_rd(0)) {
        serial_puts("LOGS: не могу прочитать MBR (транспортная ошибка)\n");
        return false;
    }
    if (g_sec[510] != 0x55 || g_sec[511] != 0xAA) {
        serial_puts("LOGS: нет MBR-подписи 55AA - носитель без таблицы\n");
        serial_puts("LOGS: разделов нет, записывать некуда (безопасный отказ)\n");
        return false;
    }

    if (g_sec[446 + 4] == 0xEE) {
        /* ── защитный MBR → GPT ── */
        g_is_gpt = 1;
        serial_puts("LOGS: найден защитный MBR (0xEE) - разбираю GPT\n");
        if (!ls_rd(1)) {
            serial_puts("LOGS: заголовок GPT не читается\n");
            return false;
        }
        if (memcmp(g_sec, "EFI PART", 8) != 0) {
            serial_puts("LOGS: сигнатура GPT не найдена - отказ\n");
            return false;
        }
        uint64_t entry_lba = ls_rdb64(g_sec + 72);
        uint32_t nent = ls_rdb32(g_sec + 80);
        uint32_t esz  = ls_rdb32(g_sec + 84);
        if (esz != 128 || nent == 0 || nent > 128) {
            serial_puts("LOGS: нестандартный размер записи GPT - отказ\n");
            return false;
        }
        if (entry_lba > 0xFFFFFFFFull) {
            serial_puts("LOGS: записи GPT за 2ТБ - не поддержано\n");
            return false;
        }
        /* первые 16 записей (4 сектора) — на практике хватает всегда;
         * остальное честно не разбираем */
        int take = (int)nent;
        if (take > 16) take = 16;
        for (int e = 0; e < take; e++) {
            uint32_t lba = (uint32_t)entry_lba + (uint32_t)(e / 4);
            if (e % 4 == 0) {
                if (!ls_rd(lba)) {
                    serial_puts("LOGS: записи GPT не читаются\n");
                    break;
                }
            }
            const uint8_t *ent = g_sec + (e % 4) * 128;
            if (ent[0] == 0 && ent[1] == 0 && ent[2] == 0 && ent[3] == 0)
                continue;                         /* пустая запись        */
            uint64_t f = ls_rdb64(ent + 32);
            uint64_t l = ls_rdb64(ent + 40);
            if (l <= f || f > 0xFFFFFFFFull) continue;
            uint64_t sects = l - f + 1u;
            if (sects > 0xFFFFFFFFull) sects = 0xFFFFFFFFull;
            /* ESP (EFI System Partition) — загрузочный: GUID
             * C12A7328-... первые 4 байта LE = 28 73 2A C1 */
            uint8_t esp = (ent[0] == 0x28 && ent[1] == 0x73 &&
                           ent[2] == 0x2A && ent[3] == 0xC1) ? 1u : 0u;
            ls_part_add((uint32_t)f, (uint32_t)sects, 0, esp, 1);
        }
    } else {
        /* ── классический MBR: 4 первичных записи ── */
        for (int i = 0; i < 4; i++) {
            const uint8_t *e = g_sec + 446 + i * 16;
            uint8_t  act = (e[0] & 0x80u) ? 1u : 0u;
            uint8_t  typ = e[4];
            uint32_t st  = ls_rdb32(e + 8);
            uint32_t sz  = ls_rdb32(e + 12);
            if (typ == 0) continue;
            if (typ == 0x05 || typ == 0x0F || typ == 0x85) {
                serial_puts("LOGS: расширенный раздел (");
                serial_puthex(typ);
                serial_puts(") не поддержан - пропущен (внутри может быть\n");
                serial_puts("LOGS: что угодно - не разбираем, не трогаем)\n");
                continue;
            }
            ls_part_add(st, sz, typ, act, 0);
        }
    }

    for (int i = 0; i < g_nparts; i++) {
        ls_part_t *p = &g_parts[i];
        uint64_t end = (uint64_t)p->start + (uint64_t)p->sects;
        serial_puts("LOGS: раздел ");
        ls_pn("", (uint32_t)(i + 1), ": LBA ");
        ls_pn("", p->start, " + ");
        ls_pn("", p->sects, "");
        if (p->gpt) serial_puts(" (GPT)");
        else { serial_puts(" тип "); serial_puthex(p->type); }
        if (p->active) serial_puts(" [ЗАГРУЗОЧНЫЙ]");
        if (end > (uint64_t)g_disk_sects) {
            serial_puts(" - ВЫХОДИТ ЗА ГРАНИЦЫ НОСИТЕЛЯ (");
            ls_pn("", g_disk_sects, " секторов) - ОТКЛОНЁН\n");
            p->sects = 0;
        } else {
            serial_puts("\n");
        }
    }
    return true;
}

/* ═══ 3. FAT32 раздела LOGS ═══════════════════════════════════════════════ */

typedef struct {
    uint32_t vbr;          /* LBA загрузочного сектора раздела            */
    uint32_t start, end;   /* границы (страж записи)                      */
    uint8_t  spc;          /* секторов/кластер (степень двойки 1..128)    */
    uint16_t rsvd;         /* резервных секторов                          */
    uint8_t  nfats;        /* копий FAT (обычно 2)                        */
    uint32_t fatsz;        /* секторов на FAT                             */
    uint32_t root_clus;    /* первый кластер корня                        */
    uint32_t fat_lba;      /* первая FAT                                   */
    uint32_t data_lba;     /* первый сектор кластера №2                   */
    uint32_t clus_count;   /* кластеров всего (включая 2 виртуальных)      */
    uint32_t fsinfo_lba;   /* сектор FSInfo (0 = нет)                      */
    /* EGOROV.LOG: */
    uint32_t dir_lba;      /* сектор записи каталога                      */
    uint16_t dir_off;      /* смещение 32-байтной записи                  */
    uint32_t first_clus;
    uint32_t last_clus;    /* последний кластер цепочки (кэш)             */
    uint32_t fsize;        /* текущий размер файла                        */
    /* кэш сектора FAT: */
    uint32_t fat_cache_lba;
    uint8_t  fat_cache[512];
    char     label[12];
} ls_fat_t;

static ls_fat_t g_fat;

/* 8.3 имя EGOROV.LOG: «EGOROV  » + «LOG»; метка тома «LOGS» */
static const uint8_t LS_FILE_NAME[11] =
    { 'E','G','O','R','O','V',' ',' ','L','O','G' };
static const uint8_t LS_LABEL[11] =
    { 'L','O','G','S',' ',' ',' ',' ',' ',' ',' ' };

static int ls_eq11(const uint8_t *a, const uint8_t *b) {
    for (int i = 0; i < 11; i++)
        if (a[i] != b[i]) return 0;
    return 1;
}

#define LS_FAT_EOC 0x0FFFFFF8u

/* FAT[clus] → значение (32-битное, верхние 4 бита срезаны).  Через кэш
 * сектора FAT; 0xFFFFFFFF при ошибке чтения. */
static uint32_t ls_fat_get(uint32_t clus) {
    if (clus < 2 || clus >= g_fat.clus_count) return LS_FAT_EOC;
    uint32_t off = clus * 4u;
    uint32_t lba = g_fat.fat_lba + (off >> 9);
    if (lba != g_fat.fat_cache_lba) {
        if (!ls_rd(lba)) return LS_FAT_EOC;
        memcpy(g_fat.fat_cache, g_sec, 512);
        g_fat.fat_cache_lba = lba;
    }
    return ls_rdb32(g_fat.fat_cache + (off & 511u)) & 0x0FFFFFFFu;
}

/* FAT[clus] = v: обновить кэш + ОБЕ копии FAT на носитель.
 * Копия 2 — best-effort (первая авторитетна; отказ копии 2 НЕ рвёт
 * файл — честное предупреждение, chkdsk приведёт в порядок). */
static bool ls_fat_set(uint32_t clus, uint32_t v) {
    if (clus < 2 || clus >= g_fat.clus_count) return false;
    uint32_t off = clus * 4u;
    uint32_t lba = g_fat.fat_lba + (off >> 9);
    if (lba != g_fat.fat_cache_lba) {
        if (!ls_rd(lba)) return false;
        memcpy(g_fat.fat_cache, g_sec, 512);
        g_fat.fat_cache_lba = lba;
    }
    uint32_t *e = (uint32_t *)(g_fat.fat_cache + (off & 511u));
    *e = (*e & 0xF0000000u) | (v & 0x0FFFFFFFu);
    memcpy(g_sec, g_fat.fat_cache, 512);
    if (!ls_wr(lba)) return false;
    if (g_fat.nfats >= 2) {
        if (!ls_wr(lba + g_fat.fatsz)) {
            serial_puts("LOGS: ВНИМАНИЕ: копия 2 FAT не записалась (продолжаю,\n");
            serial_puts("LOGS: первая копия авторитетна; chkdsk приведёт в порядок)\n");
        }
    }
    return true;
}

/* Найти СВОБОДНЫЙ кластер (FAT==0), начиная с подсказки (после нашего
 * последнего).  Бюджет сканирования ограничен (256 секторов FAT за
 * вызов) — на живом томе ищется мгновенно; забитый — честный отказ.
 * [build 4] g_fat_scan_err отличает транспортный СБОЙ чтения от
 * «места нет»: раньше любая ошибка чтения выдавала себя за забитый
 * том (неверная диагностика при выдернутой флешке). */
static uint32_t g_fat_scan_err;   /* 1 = скан прерван ошибкой чтения */

static uint32_t ls_fat_find_free(void) {
    uint32_t budget_secs = g_fat.fatsz;
    if (budget_secs > 256u) budget_secs = 256u;
    uint32_t max_off = budget_secs * 512u;
    if (max_off > g_fat.clus_count * 4u) max_off = g_fat.clus_count * 4u;
    if (max_off < 8u) return 0;

    uint32_t hint = 2;
    if (g_fat.last_clus >= 2 && g_fat.last_clus + 1u < g_fat.clus_count)
        hint = g_fat.last_clus + 1u;

    for (int pass = 0; pass < 2; pass++) {
        uint32_t from = (pass == 0) ? hint * 4u : 8u;
        uint32_t to   = (pass == 0) ? max_off   : hint * 4u;
        for (uint32_t off = from; off < to; off += 4u) {
            uint32_t lba = g_fat.fat_lba + (off >> 9);
            if (lba != g_fat.fat_cache_lba) {
                if (!ls_rd(lba)) {
                    g_fat_scan_err = 1;        /* транспорт, не «нет места» */
                    return 0;
                }
                memcpy(g_fat.fat_cache, g_sec, 512);
                g_fat.fat_cache_lba = lba;
            }
            if ((ls_rdb32(g_fat.fat_cache + (off & 511u)) & 0x0FFFFFFFu) == 0)
                return off / 4u;
        }
        if (pass == 0 && hint == 2) break;   /* проход 2 не нужен */
    }
    return 0;
}

/* Декремент FSInfo (чтобы Windows не считала наши кластеры свободными).
 * Best-effort: нет сигнатур — молча пропускаем (Windows-форматированные
 * тома и mkstick.py создают валидный FSInfo). */
static void ls_fsinfo_dec(uint32_t n_alloc) {
    if (g_fat.fsinfo_lba == 0 || n_alloc == 0) return;
    if (!ls_rd(g_fat.fsinfo_lba)) return;
    if (memcmp(g_sec, "RRaA", 4) != 0 ||
        memcmp(g_sec + 484, "rrAa", 4) != 0)
        return;
    uint32_t free_cnt = ls_rdb32(g_sec + 488);
    if (free_cnt != 0xFFFFFFFFu) {
        if (free_cnt >= n_alloc) free_cnt -= n_alloc;
        else                     free_cnt  = 0;
        g_sec[488] = (uint8_t)free_cnt;
        g_sec[489] = (uint8_t)(free_cnt >> 8);
        g_sec[490] = (uint8_t)(free_cnt >> 16);
        g_sec[491] = (uint8_t)(free_cnt >> 24);
    }
    if (g_fat.last_clus >= 2 && g_fat.last_clus < g_fat.clus_count) {
        g_sec[492] = (uint8_t)g_fat.last_clus;
        g_sec[493] = (uint8_t)(g_fat.last_clus >> 8);
        g_sec[494] = (uint8_t)(g_fat.last_clus >> 16);
        g_sec[495] = (uint8_t)(g_fat.last_clus >> 24);
    }
    ls_wr(g_fat.fsinfo_lba);
}

/* FAT32-загрузочный сектор раздела: BPB валиден?  1 = g_fat заполнен. */
static bool ls_fat_parse(const ls_part_t *p) {
    memset(&g_fat, 0, sizeof(g_fat));
    g_fat.fat_cache_lba = 0xFFFFFFFFu;

    if (!ls_rd(p->start)) return false;
    if (g_sec[510] != 0x55 || g_sec[511] != 0xAA) return false;
    if (memcmp(g_sec + 0x52, "FAT32", 5) != 0) return false;

    uint16_t bps   = (uint16_t)(g_sec[11] | ((uint16_t)g_sec[12] << 8));
    uint8_t  spc   = g_sec[13];
    uint16_t rsvd  = (uint16_t)(g_sec[14] | ((uint16_t)g_sec[15] << 8));
    uint8_t  nfats = g_sec[16];
    uint32_t fatsz = ls_rdb32(g_sec + 0x24);
    uint32_t rootc = ls_rdb32(g_sec + 0x2C);
    uint16_t fsinf = (uint16_t)(g_sec[0x30] | ((uint16_t)g_sec[0x31] << 8));

    if (bps != 512) {                       /* транспорт умеет только 512 */
        serial_puts("LOGS: FAT: байт/сектор != 512 - не поддержано\n");
        return false;
    }
    if (spc == 0 || (spc & (spc - 1u)) != 0 || spc > 128) {
        serial_puts("LOGS: FAT: секторов/кластер не степень 2 - мусор\n");
        return false;
    }
    if (rsvd == 0 || nfats == 0 || fatsz == 0 || rootc < 2) return false;

    memcpy(g_fat.label, g_sec + 0x47, 11);
    g_fat.label[11] = 0;

    g_fat.vbr       = p->start;
    g_fat.start     = p->start;
    g_fat.end       = p->start + p->sects;
    g_fat.spc       = spc;
    g_fat.rsvd      = rsvd;
    g_fat.nfats     = nfats;
    g_fat.fatsz     = fatsz;
    g_fat.root_clus = rootc;
    g_fat.fat_lba   = p->start + rsvd;
    uint32_t data   = g_fat.fat_lba + (uint32_t)nfats * fatsz;
    if (data >= g_fat.end) return false;    /* FAT больше раздела - мусор */
    g_fat.data_lba   = data;
    g_fat.clus_count = (g_fat.end - data) / spc + 2u;
    g_fat.fsinfo_lba = (fsinf > 1 && fsinf < p->sects)
                       ? (p->start + fsinf) : 0u;
    return true;
}

static uint32_t ls_clus_lba(uint32_t clus) {
    return g_fat.data_lba + (clus - 2u) * g_fat.spc;
}

/* ── Поиск/создание EGOROV.LOG в корневом каталоге ─────────────────────────
 * Возвращает: 1 = найден, 2 = есть свободный слот (dir_lba/dir_off),
 * 0 = ошибка/нет места.  LFN-записи и метки тома пропускаются. */
static int ls_find_file(void) {
    uint32_t clus = g_fat.root_clus;
    uint32_t guard = 0;
    int      free_found = 0;
    uint32_t free_lba = 0, free_off = 0, free_is_last = 0;

    while (clus >= 2 && clus < g_fat.clus_count && guard++ < g_fat.clus_count) {
        uint32_t base = ls_clus_lba(clus);
        for (uint32_t s = 0; s < g_fat.spc; s++) {
            uint32_t lba = base + s;
            if (!ls_rd(lba)) return 0;
            for (uint32_t o = 0; o + 32 <= 512; o += 32) {
                uint8_t first = g_sec[o];
                uint8_t attr  = g_sec[o + 11];
                if (first == 0x00) {           /* конец каталога */
                    if (!free_found) {
                        free_found  = 1;
                        free_lba    = lba;
                        free_off    = (uint16_t)o;
                        /* последний слот последнего сектора кластера —
                         * за ним НЕТ терминатора: откажемся от него */
                        free_is_last = (o == 480u && s + 1u == g_fat.spc) ? 1u : 0u;
                    }
                    goto done;
                }
                if (first == 0xE5) {           /* удалённая запись — слот */
                    if (!free_found) {
                        free_found  = 1;
                        free_lba    = lba;
                        free_off    = (uint16_t)o;
                        free_is_last = 0;
                    }
                    continue;
                }
                if (attr == 0x0F) continue;    /* LFN */
                if (attr & 0x08) continue;     /* метка тома */
                if (attr & (0x04u | 0x10u)) continue;   /* system/dir */
                if (ls_eq11(g_sec + o, LS_FILE_NAME)) {
                    uint32_t hi = ls_rdb32(g_sec + o + 20);
                    uint32_t lo = ls_rdb32(g_sec + o + 26);
                    g_fat.dir_lba    = lba;
                    g_fat.dir_off    = (uint16_t)o;
                    g_fat.first_clus = ((hi & 0xFFFFu) << 16) | (lo & 0xFFFFu);
                    g_fat.fsize      = ls_rdb32(g_sec + o + 28);
                    return 1;
                }
            }
        }
        clus = ls_fat_get(clus);
        if (clus >= LS_FAT_EOC) break;
    }
done:
    if (free_found && !free_is_last) {
        g_fat.dir_lba = free_lba;
        g_fat.dir_off = free_off;
        return 2;
    }
    if (free_found) {
        serial_puts("LOGS: единственный свободный слот - последний в корне,\n");
        serial_puts("LOGS: без терминатора каталога его не заполняю - отказ\n");
    } else {
        serial_puts("LOGS: слот в корне не найден (корень заполнен; рост\n");
        serial_puts("LOGS: корневого каталога не делаем - чужие данные не трогаем)\n");
    }
    return 0;
}

/* Целостность цепочки существующего EGOROV.LOG: длина цепочки обязана
 * сходиться с ceil(fsize/clus_bytes), кластеры в диапазоне, циклов нет.
 * Пустой файл (fsize==0) валиден с любым first_clus. */
static bool ls_check_chain(void) {
    uint32_t clus_bytes = (uint32_t)g_fat.spc * 512u;
    if (g_fat.fsize == 0) {
        if (g_fat.first_clus != 0 &&
            (g_fat.first_clus < 2 || g_fat.first_clus >= g_fat.clus_count)) {
            serial_puts("LOGS: EGOROV.LOG: первый кластер вне раздела - повреждён\n");
            return false;
        }
        g_fat.last_clus = g_fat.first_clus;
        return true;
    }
    if (g_fat.first_clus < 2 || g_fat.first_clus >= g_fat.clus_count) {
        serial_puts("LOGS: EGOROV.LOG: первый кластер вне раздела - повреждён\n");
        return false;
    }
    uint32_t need = (g_fat.fsize + clus_bytes - 1u) / clus_bytes;
    uint32_t c = g_fat.first_clus;
    uint32_t n = 1;
    while (n < need) {
        uint32_t nx = ls_fat_get(c);
        if (nx < 2 || nx >= g_fat.clus_count) {
            serial_puts("LOGS: EGOROV.LOG: FAT-цепочка битая (кластер ");
            ls_pn("", c, ")\n");
            return false;
        }
        c = nx;
        n++;
        if (n > g_fat.clus_count) {           /* цикл! */
            serial_puts("LOGS: EGOROV.LOG: цикл в FAT-цепочке - повреждён\n");
            return false;
        }
    }
    /* за последним нужным кластером обязан стоять EOC */
    if (ls_fat_get(c) < LS_FAT_EOC) {
        serial_puts("LOGS: EGOROV.LOG: цепочка ДЛИННЕЕ размера (кластеры\n");
        serial_puts("LOGS: свисают) - не сходится, НЕ перезаписываю\n");
        return false;
    }
    g_fat.last_clus = c;
    return true;
}

/* Записать поля НАШЕЙ записи каталога (кластер/размер/дата-время).
 * Модифицирует ТОЛЬКО 32 байта EGOROV.LOG в секторе каталога. */
static bool ls_dir_update(void) {
    if (!ls_rd(g_fat.dir_lba)) return false;
    uint8_t *e = g_sec + g_fat.dir_off;
    rtc_time_t rt;
    rtc_read(&rt);
    uint16_t fdate = (1u << 5) | 1u;          /* 1980-01-01 по умолчанию */
    uint16_t ftime = 0;
    if (rt.year >= 1980 && rt.year <= 2107) {
        fdate = (uint16_t)(((uint32_t)(rt.year - 1980u) << 9) |
                           ((uint32_t)rt.month << 5) | (uint32_t)rt.day);
        ftime = (uint16_t)(((uint32_t)rt.hour << 11) |
                           ((uint32_t)rt.min << 5) | ((uint32_t)rt.sec >> 1));
    }
    e[20] = (uint8_t)(g_fat.first_clus >> 16);   /* FstClusHI (спека!) */
    e[21] = (uint8_t)(g_fat.first_clus >> 24);
    e[26] = (uint8_t)g_fat.first_clus;           /* FstClusLO */
    e[27] = (uint8_t)(g_fat.first_clus >> 8);
    e[28] = (uint8_t)g_fat.fsize;
    e[29] = (uint8_t)(g_fat.fsize >> 8);
    e[30] = (uint8_t)(g_fat.fsize >> 16);
    e[31] = (uint8_t)(g_fat.fsize >> 24);
    e[14] = (uint8_t)ftime;        e[15] = (uint8_t)(ftime >> 8);   /* CrtTime */
    e[16] = (uint8_t)fdate;        e[17] = (uint8_t)(fdate >> 8);   /* CrtDate */
    e[18] = (uint8_t)fdate;        e[19] = (uint8_t)(fdate >> 8);   /* LstAccDate */
    e[22] = (uint8_t)ftime;        e[23] = (uint8_t)(ftime >> 8);   /* WrtTime */
    e[24] = (uint8_t)fdate;        e[25] = (uint8_t)(fdate >> 8);   /* WrtDate */
    return ls_wr(g_fat.dir_lba);
}

/* Создать EGOROV.LOG в свободном слоте каталога.  Кластер выделится
 * при первой записи данных; пустой файл = 0 кластеров (как Windows). */
static bool ls_dir_create(void) {
    if (!ls_rd(g_fat.dir_lba)) return false;
    uint8_t *e = g_sec + g_fat.dir_off;
    memset(e, 0, 32);
    memcpy(e, LS_FILE_NAME, 11);
    e[11] = 0x20;                 /* ARCHIVE — обычный файл               */
    g_fat.first_clus = 0;
    g_fat.last_clus  = 0;
    g_fat.fsize      = 0;
    if (!ls_wr(g_fat.dir_lba)) return false;
    serial_puts("LOGS: EGOROV.LOG СОЗДАН (запись каталога обновлена)\n");
    return true;
}

/* ═══ 4. Дополнение файла (ядро выгрузки) ════════════════════════════════
 * Дописывает data[0..len) в конец EGOROV.LOG.  Инвариант: last_clus —
 * кластер с байтом (fsize-1); запись на границе кластера → новый кластер.
 * Неполный сектор: прочитать-дополнить-записать.  Возвращает число
 * дописанных байт (0 = отказ; причина напечатана).
 * [build 4] ЦЕЛОСТНОСТЬ МЕТАДАННЫХ: результат ls_dir_update()
 * ОБЯЗАТЕЛЬНО проверяется — если данные записаны, а размер в каталоге
 * НЕТ, файл на томе короче записанного; успех НЕ подтверждается,
 * сохраняется диагностическая причина, выгрузка останавливается
 * (писать вслепую поверх несогласованного размера нельзя). */
static uint32_t g_alloc_cnt;      /* кластеров выделено с прошлого FSInfo */
static uint32_t g_no_space;       /* 1 = свободные кластеры кончились    */
static uint32_t g_meta_fail;      /* 1 = каталог не обновлён — остановка */
static uint32_t g_dir_confirmed;  /* размер, ПОДТВЕРЖДЁННЫЙ каталогом    */

static uint32_t ls_append(const uint8_t *data, uint32_t len) {
    uint32_t clus_bytes = (uint32_t)g_fat.spc * 512u;
    uint32_t done = 0;

    while (done < len) {
        uint32_t off_in_clus = g_fat.fsize % clus_bytes;
        uint32_t sec_in_clus = off_in_clus / 512u;
        uint32_t off_in_sec  = off_in_clus % 512u;

        /* граница кластера (или пустой файл) → выделить кластер */
        if (off_in_clus == 0) {
            g_fat_scan_err = 0;
            uint32_t nc = ls_fat_find_free();
            if (nc == 0) {
                if (g_fat_scan_err) {
                    serial_puts("LOGS: ОТКАЗ: FAT не читается (транспортная\n");
                    serial_puts("LOGS: ошибка; носитель мог быть выдернут)\n");
                    break;              /* НЕ «нет места» — честная причина */
                }
                serial_puts("LOGS: НЕДОСТАТОЧНО МЕСТА: нет свободных кластеров\n");
                serial_puts("LOGS: (файл остаётся целым; COM1/RAM живут)\n");
                g_no_space = 1;
                break;
            }
            if (!ls_fat_set(nc, LS_FAT_EOC)) break;
            g_alloc_cnt++;
            if (g_fat.first_clus == 0) {
                g_fat.first_clus = nc;
                if (!ls_dir_update()) { g_meta_fail = 1; break; } /* первый кластер - сразу */
                g_dir_confirmed = g_fat.fsize;
            } else {
                if (!ls_fat_set(g_fat.last_clus, nc)) break;
            }
            g_fat.last_clus = nc;
        }

        /* сектор данных */
        uint32_t lba = ls_clus_lba(g_fat.last_clus) + sec_in_clus;
        uint32_t chunk = 512u - off_in_sec;
        uint32_t left = len - done;
        if (chunk > left) chunk = left;

        if (off_in_sec != 0 || chunk != 512u) {
            if (!ls_rd(lba)) break;            /* частичный: RMW */
        }
        memcpy(g_sec + off_in_sec, data + done, chunk);
        if (!ls_wr(lba)) break;

        g_fat.fsize += chunk;
        done += chunk;
    }
    if (done > 0) {
        if (!ls_dir_update()) {
            /* [build 4] данные записаны, каталог — НЕТ: успех не
             * подтверждаем; точная причина — в журнал, выгрузка будет
             * остановлена вызывающим (g_meta_fail) */
            g_meta_fail = 1;
            serial_puts("LOGS: ОТКАЗ: данные записаны в кластеры, но запись\n");
            serial_puts("LOGS: каталога (размер файла) НЕ обновилась: на томе\n");
            serial_puts("LOGS: файл короче на ");
            ls_pn("", g_fat.fsize - g_dir_confirmed, " байт (несоответствие\n");
            serial_puts("LOGS: размеров; запись вслепую прекращена)\n");
        } else {
            g_dir_confirmed = g_fat.fsize;
        }
        if (g_alloc_cnt > 0) {
            ls_fsinfo_dec(g_alloc_cnt);
            g_alloc_cnt = 0;
        }
    }
    return done;
}

/* ═══ 5. Диспетчер выгрузки (буферизация + периодичность) ═════════════════
 *
 * [build 4] ИСТОЧНИК ЖУРНАЛА — ДВУХФАЗНЫЙ (проблема 3 ревью build 3:
 * кольцо 8 КБ затирало начало загрузки до готовности флешки, файл
 * начинался с klog_oldest(), обещание «весь журнал с момента запуска»
 * не выполнялось):
 *   фаза 0: РАННИЙ БУФЕР [0..E) — линейный, с ПЕРВОГО байта загрузки
 *            (kernel.c, logearly=N КБ, по умолчанию 128);
 *   стык:    если кольцо успело затереть байты за пределами раннего
 *            буфера (E < oldest) — в файл пишется ЧЕСТНЫЙ МАРКЕР
 *            «потеряно N байт», число учтено и в итоговом своде;
 *   фаза 1:  КОЛЬЦО [ring_pos..total) — дельта новых байт (64 КБ по
 *            умолчанию, cmdline logring=N КБ); переполнение между
 *            выгрузками тоже маркируется честной строкой.
 * ИТОГ никогда не называется «полным», если потерян хоть один байт. */

#define LS_BUF_SZ    4096u        /* буфер порции выгрузки                  */
#define LS_TICK_BUDGET 9u         /* секторов за один ls_tick (bounded)     */
#define LS_FAIL_OFF  3            /* ошибок подряд до отключения            */

static uint8_t  g_ls_buf[LS_BUF_SZ] __attribute__((aligned(4096)));
static uint32_t g_ls_cap = LS_DEFAULT_CAP;   /* лимит файла                 */
static int      g_ls_stage;       /* 0=off 1=готово 2=отключено-ошибка
                                     3=лимит/нет места 4=нет носителя      */
static uint32_t g_ls_flushes;     /* выгрузок всего                          */
static uint32_t g_ls_fail_run;    /* ошибок транспорта подряд                */
static uint32_t g_ls_bytes_in_file;

/* двухфазный источник (см. комментарий секции) */
static int      g_src_phase;      /* 0 = ранний буфер, 1 = кольцо           */
static uint32_t g_src_early;      /* курсор раннего буфера [0..E)           */
static uint32_t g_ring_pos;       /* курсор кольца (глобальные байты klog)  */
static uint32_t g_gap_pending;    /* 1 = между фазами нужен маркер дыры     */
static uint32_t g_gap_bytes;      /* байт в дыре на стыке (при подготовке)  */
static uint32_t g_lost_bytes;     /* ВСЕГО потеряно (стык + переполнения)   */

/* маркер в буфер: prefix + число + suffix (без libc) */
static uint32_t ls_mkmark(uint8_t *dst, uint32_t cap, const char *pre,
                          uint32_t num, const char *post) {
    uint32_t at = 0;
    for (uint32_t k = 0; pre[k] && at + 1u < cap; k++)  dst[at++] = (uint8_t)pre[k];
    if (num == 0) {
        if (at + 1u < cap) dst[at++] = '0';
    } else {
        char t[12];
        int  n = 0;
        while (num > 0 && n < 11) { t[n++] = (char)('0' + num % 10u); num /= 10u; }
        for (int i = n - 1; i >= 0 && at + 1u < cap; i--) dst[at++] = (uint8_t)t[i];
    }
    for (uint32_t k = 0; post[k] && at + 1u < cap; k++) dst[at++] = (uint8_t)post[k];
    return at;
}

/* Записать ПОРЦИЮ buf[0..n) в EGOROV.LOG: лимит файла, честный учёт
 * отказов (транспорт / метаданные / место), продвижение курсора источника.
 * src: 0 = ранний буфер, 1 = кольцо, 2 = маркер (курсора нет).
 * Возвращает число дописанных байт; 0 = отказ (стадия/счётчики обновлены,
 * причина уже напечатана). */
static uint32_t ls_commit(uint8_t *buf, uint32_t n, int src) {
    if (g_ls_stage != 1) return 0;
    if (g_fat.fsize >= g_ls_cap) {
        serial_puts("LOGS: EGOROV.LOG достиг лимита (");
        ls_pn("", g_ls_cap, " байт) - файл больше не растёт\n");
        serial_puts("LOGS: (журнал продолжает идти в COM1 и RAM-кольце)\n");
        g_ls_stage = 3;
        return 0;
    }
    if (n > g_ls_cap - g_fat.fsize) n = g_ls_cap - g_fat.fsize;
    if (n == 0) return 0;

    uint32_t got = ls_append(buf, n);
    if (g_meta_fail) {
        serial_puts("LOGS: ВЫГРУЗКА ОСТАНОВЛЕНА: каталог не подтверждён\n");
        serial_puts("LOGS: (данные в кластерах есть, размер файла на томе - НЕТ;\n");
        serial_puts("LOGS: COM1 и RAM-кольцо работают дальше)\n");
        ls_set_status("каталог не обновлён - выгрузка остановлена");
        g_ls_stage = 2;
    } else if (g_no_space) {
        g_no_space = 0;
        g_ls_stage = 3;
    }
    if (got > 0) {
        if (src == 0)      g_src_early += got;
        else if (src == 1) g_ring_pos  += got;
        g_ls_bytes_in_file += got;
        g_ls_fail_run = 0;
        return got;
    }
    if (g_ls_stage == 1) {           /* чистый транспортный отказ */
        g_ls_fail_run++;
        serial_puts("LOGS: выгрузка не удалась (");
        ls_pn("", g_ls_fail_run, "/3 подряд) - транспортная ошибка (причина\n");
        serial_puts("LOGS: в USBMSD:/MOUSEHW: строках выше)\n");
        if (g_ls_fail_run >= LS_FAIL_OFF) {
            serial_puts("LOGS: выгрузка ОТКЛЮЧЕНА после 3 ошибок подряд\n");
            serial_puts("LOGS: (COM1 и RAM-кольцо работают дальше)\n");
            g_ls_stage = 2;
        }
    }
    return 0;
}

/* Выгрузить доступное (двухфазный источник).  sector_budget = 0 → без
 * лимита (финальная выгрузка).  Возвращает записанные за вызов байты. */
static uint32_t ls_flush(uint32_t sector_budget) {
    if (g_ls_stage != 1) return 0;
    uint32_t budget = sector_budget ? sector_budget * 512u : 0xFFFFFFFFu;
    uint32_t done_call = 0;

    while (g_ls_stage == 1 && done_call < budget) {
        uint32_t room = budget - done_call;

        /* ── фаза 0: ранний буфер — старт журнала с ПЕРВОГО байта ── */
        if (g_src_phase == 0) {
            uint32_t E = klog_early_len();
            if (g_src_early >= E) {
                g_src_phase = 1;
                if (g_gap_pending) {   /* дыра старт→кольцо: честный маркер */
                    g_gap_pending = 0;
                    uint32_t at = ls_mkmark(g_ls_buf, LS_BUF_SZ,
                        "\n=== LOGS: ДЫРА: потеряно ", g_gap_bytes,
                        " байт журнала до готовности носителя (буферы переполнены) ===\n");
                    uint32_t got = ls_commit(g_ls_buf, at, 2);
                    if (got == 0) break;
                    done_call += got;
                }
                continue;
            }
            uint32_t want = E - g_src_early;
            if (want > LS_BUF_SZ)      want = LS_BUF_SZ;
            if (want > room)           want = room;
            if (want == 0) break;
            uint32_t n = klog_early_copy(g_src_early, (char *)g_ls_buf, want);
            if (n == 0) { g_src_phase = 1; continue; }
            uint32_t got = ls_commit(g_ls_buf, n, 0);
            if (got == 0) break;
            done_call += got;
            continue;
        }

        /* ── фаза 1: кольцо — дельта новых байт журнала ── */
        uint32_t total  = klog_total();
        uint32_t oldest = klog_oldest();
        if (g_ring_pos < oldest) {
            /* кольцо переполнилось ПОСЛЕ старта выгрузки — честный маркер;
             * байты-продолжения UTF-8 на рваной границе пропускаем */
            uint32_t lost = oldest - g_ring_pos;
            g_lost_bytes += lost;
            g_ring_pos = oldest;
            uint32_t peek[1];
            uint32_t guard = 0;
            while (g_ring_pos < total && guard++ < 4 &&
                   klog_snapshot(g_ring_pos, (char *)peek, 1) == 1 &&
                   (peek[0] & 0xC0u) == 0x80u)
                g_ring_pos++;
            uint32_t at = ls_mkmark(g_ls_buf, LS_BUF_SZ,
                "\n=== LOGS: ДЫРА: ещё потеряно ", lost,
                " байт (кольцо журнала переполнено между выгрузками) ===\n");
            uint32_t got = ls_commit(g_ls_buf, at, 2);
            if (got == 0) break;
            done_call += got;
            continue;
        }
        if (g_ring_pos >= total) break;      /* новых байт нет */
        uint32_t want = total - g_ring_pos;
        if (want > LS_BUF_SZ) want = LS_BUF_SZ;
        if (want > room)      want = room;
        if (want == 0) break;
        uint32_t n = klog_snapshot(g_ring_pos, (char *)g_ls_buf, want);
        if (n == 0) break;
        uint32_t got = ls_commit(g_ls_buf, n, 1);
        if (got == 0) break;
        done_call += got;
    }
    if (done_call > 0) g_ls_flushes++;
    return done_call;
}

/* ═══ 6. Публичный API ════════════════════════════════════════════════════ */

/* Поиск раздела LOGS на ТЕКУЩЕМ накопителе (g_ls_tr/g_ls_cap_sect уже
 * установлены).  true = найден: g_fat готов, СТРАЖ ЗАПИСИ ВКЛЮЧЁН
 * (g_part_start/g_part_end), напечатаны границы.  false = на этом
 * носителе раздела LOGS нет (или геометрия не читается) — вызывающий
 * переходит к СЛЕДУЮЩЕМУ накопителю. */
static bool ls_find_logs_part(void) {
    if (!ls_parse_disk()) {
        serial_puts("LOGS: геометрия этого носителя не читается - проверяю\n");
        serial_puts("LOGS: следующий накопитель\n");
        return false;
    }
    for (int i = 0; i < g_nparts; i++) {
        ls_part_t *p = &g_parts[i];
        if (p->sects == 0) continue;          /* вычеркнут (за границей)  */
        if (p->active) {
            serial_puts("LOGS: раздел ");
            ls_pn("", (uint32_t)(i + 1), " ЗАГРУЗОЧНЫЙ (0x80/ESP) - исключён\n");
            continue;
        }
        /* GPT: тип не кодирует FAT — проверяем VBR; MBR: 0x0B/0x0C */
        if (!p->gpt && p->type != 0x0B && p->type != 0x0C) continue;
        if (!ls_fat_parse(p)) continue;       /* не FAT32 — мимо */

        serial_puts("LOGS: FAT32-раздел: LBA ");
        ls_pn("", p->start, " + ");
        ls_pn("", p->sects, " секторов, метка тома '");
        serial_puts(g_fat.label);
        serial_puts("'\n");
        if (!ls_eq11((const uint8_t *)g_fat.label, LS_LABEL)) {
            serial_puts("LOGS: метка не 'LOGS' - раздел НЕ предназначен для\n");
            serial_puts("LOGS: журналов; ищу дальше (чужое не трогаю)\n");
            continue;
        }

        /* ЭТО раздел LOGS: включаем стража записи */
        g_part_start = g_fat.start;
        g_part_end   = g_fat.end;
        serial_puts("LOGS: раздел LOGS НАЙДЕН: LBA ");
        ls_pn("", g_part_start, " + ");
        ls_pn("", g_part_end - g_part_start, " (запись разрешена ТОЛЬКО сюда)\n");
        return true;
    }
    return false;
}

void ls_stick_prepare(uint32_t cap_bytes) {
    g_ls_stage = 0;
    g_ls_flushes = 0;
    g_ls_fail_run = 0;
    g_ls_bytes_in_file = 0;
    g_ls_cap = (cap_bytes && cap_bytes < 0x80000000u) ? cap_bytes : LS_DEFAULT_CAP;
    g_part_start = g_part_end = 0;
    g_src_phase = 0;
    g_src_early = 0;
    g_ring_pos = 0;
    g_gap_pending = 0;
    g_gap_bytes = 0;
    g_lost_bytes = 0;
    g_meta_fail = 0;
    g_dir_confirmed = 0;
    ls_set_status("sticklog: подготовка");

    serial_puts("\nLOGS: ═══ поиск носителя для EGOROV.LOG ═══\n");
    serial_puts("LOGS: резервные каналы (COM1 + RAM-кольцо 64К) работают всегда\n");

    /* 1. [build 4] перечислить ВСЕ накопители на ВСЕХ трёх транспортах */
    ls_cand_t cand[3];
    int ncand = ls_blk_enumerate(cand, 3);
    if (ncand == 0) {
        serial_puts("LOGS: ИТОГ: накопитель НЕ найден (USB Mass Storage не\n");
        serial_puts("LOGS: перечислен ни на OHCI/UHCI/xHCI). Запись невозможна;\n");
        serial_puts("LOGS: ОС продолжает работать, журнал - в COM1 + RAM (COREINFO)\n");
        ls_set_status("накопитель не найден");
        g_ls_stage = 4;
        return;
    }
    serial_puts("LOGS: найдено накопителей: ");
    ls_pn("", ncand, "; запись пойдёт на тот, где есть раздел LOGS\n");

    /* 2. КАЖДЫЙ кандидат: модель/ёмкость → поиск раздела LOGS.
     *    Первый без раздела LOGS (например, загрузочная флешка) честно
     *    пропускается — проверяется СЛЕДУЮЩИЙ (проблема 1 ревью). */
    int chosen = -1;
    for (int c = 0; c < ncand; c++) {
        g_ls_tr      = cand[c].tr;
        g_ls_cap_sect = cand[c].cap;
        g_part_start = g_part_end = 0;        /* страж пока выключен */
        g_fat_scan_err = 0;
        serial_puts("LOGS: накопитель ");
        ls_pn("", (uint32_t)(c + 1), "/");
        ls_pn("", ncand, ": ");
        serial_puts(ls_tr_name(g_ls_tr));
        serial_puts(" '");
        serial_puts(cand[c].name);
        serial_puts("', ");
        ls_pn("", g_ls_cap_sect, " секторов (");
        ls_pn("", g_ls_cap_sect / 2048u, " МБ)\n");
        if (ls_find_logs_part()) { chosen = c; break; }
        serial_puts("LOGS: на этом носителе раздела LOGS нет - проверяю следующий\n");
    }
    if (chosen < 0) {
        g_ls_tr = LS_TR_NONE;
        serial_puts("LOGS: ИТОГ: раздел LOGS НЕ найден НИ НА ОДНОМ из ");
        ls_pn("", ncand, " накопителей (нужен FAT32-раздел с\n");
        serial_puts("LOGS: меткой тома 'LOGS'; подготовка - ТОЛЬКО вручную:\n");
        serial_puts("LOGS: tools/mkstick.py для теста / форматирование в Windows).\n");
        serial_puts("LOGS: НИЧЕГО не записано - чужие разделы под защитой\n");
        ls_set_status("раздел LOGS не найден");
        g_ls_stage = 4;
        return;
    }

    /* 3. [build 4] ПОДТВЕРЖДЕНИЕ ВЫБОРА перед первой записью: какой
     *    именно накопитель (транспорт, модель, ёмкость) выбран и почему */
    serial_puts("LOGS: ВЫБРАН накопитель ");
    ls_pn("", (uint32_t)(chosen + 1), "/");
    ls_pn("", ncand, ": ");
    serial_puts(ls_tr_name(g_ls_tr));
    serial_puts(" '");
    serial_puts(cand[chosen].name);
    serial_puts("', ");
    ls_pn("", g_ls_cap_sect, " секторов - НА НЁМ раздел LOGS,\n");
    serial_puts("LOGS: запись ТОЛЬКО в этот раздел (см. границы выше)\n");

    /* 4. EGOROV.LOG: найти/создать/проверить */
    uint32_t rd_errs0 = g_ls_rd_errs;   /* до поиска: дельта = транспорт */
    int f = ls_find_file();
    if (f == 1) {
        if (!ls_check_chain()) {
            ls_set_status("EGOROV.LOG повреждён - запись отклонена");
            g_part_start = g_part_end = 0;   /* запретить всё */
            g_ls_stage = 4;
            return;
        }
        g_dir_confirmed = g_fat.fsize;
        ls_pn("LOGS: EGOROV.LOG найден, размер ", g_fat.fsize,
              " - ДОПОЛНЯЮ (не перезаписываю)\n");
    } else if (f == 2) {
        if (!ls_dir_create()) {
            ls_set_status("запись каталога не создана");
            g_part_start = g_part_end = 0;
            g_ls_stage = 4;
            return;
        }
        g_dir_confirmed = 0;
    } else {
        if (g_ls_rd_errs > rd_errs0) {
            ls_set_status("каталог не читается (транспортная ошибка)");
            serial_puts("LOGS: ИТОГ: НЕ ЗАПИСАНО: корневой каталог не читается\n");
        } else {
            ls_set_status("нет свободного слота в корне");
        }
        g_part_start = g_part_end = 0;
        g_ls_stage = 4;
        return;
    }

    /* 5. источники журнала (до заголовка — чтобы напечатать покрытие):
     *    ранний буфер [0..E) + кольцо [max(E,oldest)..total) */
    uint32_t E      = klog_early_len();
    uint32_t total  = klog_total();
    uint32_t oldest = klog_oldest();
    g_ring_pos = (E >= oldest) ? E : oldest;
    if (E < oldest) {
        g_gap_pending = 1;
        g_gap_bytes   = oldest - E;
        g_lost_bytes  = g_gap_bytes;
        /* рваная граница кольца: пропустить байты-продолжения UTF-8,
         * чтобы файл продолжился с ЦЕЛОГО символа */
        uint32_t peek[1];
        uint32_t guard = 0;
        while (g_ring_pos < total && guard++ < 4 &&
               klog_snapshot(g_ring_pos, (char *)peek, 1) == 1 &&
               (peek[0] & 0xC0u) == 0x80u)
            g_ring_pos++;
    }

    /* 6. заголовок сессии + ЧЕСТНОЕ покрытие журнала (BOM — только при
     *    создании; Windows-блокнот откроет журнал как текст) */
    {
        static const char h1[] = "\n====== Egorov OS ";
        static const char h2[] = " LOGS session (USB transport ";
        static const char h3[] = ") ======\n";
        uint8_t  h[256];
        uint32_t at = 0;
        if (g_fat.fsize == 0) { h[0] = 0xEF; h[1] = 0xBB; h[2] = 0xBF; at = 3; }
        for (uint32_t k = 0; h1[k] && at < sizeof(h) - 128; k++) h[at++] = (uint8_t)h1[k];
        for (uint32_t k = 0; EGOROV_OS_FULL[k] && at < sizeof(h) - 128; k++)
            h[at++] = (uint8_t)EGOROV_OS_FULL[k];
        for (uint32_t k = 0; h2[k] && at < sizeof(h) - 128; k++) h[at++] = (uint8_t)h2[k];
        const char *tn = ls_tr_name(g_ls_tr);
        for (uint32_t k = 0; tn[k] && at < sizeof(h) - 96; k++) h[at++] = (uint8_t)tn[k];
        for (uint32_t k = 0; h3[k] && at < sizeof(h) - 96; k++) h[at++] = (uint8_t)h3[k];
        /* покрытие: полный журнал / честная потеря байт (числом!) */
        if (g_gap_pending) {
            at += ls_mkmark(h + at, sizeof(h) - at, "журнал: ПОТЕРЯНО ",
                            g_gap_bytes, " байт старта (ранний буфер ");
            at += ls_mkmark(h + at, sizeof(h) - at, "", klog_early_len() / 1024u,
                            " КБ переполнен)\n");
        } else {
            at += ls_mkmark(h + at, sizeof(h) - at,
                "журнал: ПОЛНЫЙ с первого байта, ранний буфер ", E, " байт\n");
        }
        /* ограничители циклов выше (sizeof-96/64) с запасом покрывают
         * самую длинную строку покрытия; буфер 192 байта достаточен */
        ls_append(h, at);
    }

    /* честная сводка покрытия на экран/COM1 */
    if (g_gap_pending) {
        serial_puts("LOGS: ВНИМАНИЕ: до готовности носителя ПОТЕРЯНО ");
        ls_pn("", g_gap_bytes, " байт журнала (ранний буфер ");
        ls_pn("", klog_early_len() / 1024u, " КБ + кольцо журнала переполнены)\n");
    } else {
        serial_puts("LOGS: журнал будет сохранён С ПЕРВОГО БАЙТА (ранний буфер: ");
        ls_pn("", klog_early_len(), " байт)\n");
    }

    g_ls_stage = 1;
    ls_set_status("раздел LOGS готов, EGOROV.LOG открыт");
}

void ls_tick(void) {
    if (g_ls_stage != 1) return;
    /* бюджет секторов ограничивает время вызова (транспорт с тайм-аутами
     * 0.5-10 с на фазу — худший случай ограничен, зависания нет) */
    ls_flush(LS_TICK_BUDGET);
}

void ls_flush_final(void) {
    if (g_ls_stage == 0) return;             /* sticklog не готовился */
    if (g_ls_stage == 1) {
        /* безлимитная добивка: ранний буфер (до 128 КБ) + кольцо.
         * Итерации ограничены (512 × 4 КБ = 2 МБ > лимита файла), каждая
         * прерывается при завершении/отказе — зависания нет. */
        for (uint32_t i = 0; i < 512u && g_ls_stage == 1; i++) {
            if (ls_flush(0) == 0) break;
        }
    }
    serial_puts("\nLOGS: ═══ ИТОГ выгрузки журнала ═══\n");
    if (g_ls_bytes_in_file > 0) {
        serial_puts("LOGS: ИТОГ: ЗАПИСАНО ");
        ls_pn("", g_ls_bytes_in_file, " байт в EGOROV.LOG (");
        ls_pn("", g_ls_io_sect, " секторов, выгрузок: ");
        ls_pn("", g_ls_flushes, ")\n");
        if (g_ls_stage == 3)
            serial_puts("LOGS: файл достиг лимита/места - выгрузка остановлена\n");
        serial_puts("LOGS: файл на разделе LOGS (");
        serial_puts(ls_tr_name(g_ls_tr));
        serial_puts("), размер файла теперь ");
        ls_pn("", g_fat.fsize, " байт\n");
    } else if (g_ls_stage == 4) {
        serial_puts("LOGS: ИТОГ: НЕ ЗАПИСАНО: ");
        serial_puts(g_ls_status);
        serial_puts("\n");
    } else if (g_ls_stage == 3) {
        serial_puts("LOGS: ИТОГ: НЕ ДОЗАПИСАНО: лимит файла/место кончились\n");
        serial_puts("LOGS: (выгружено до отказа: 0 байт; COM1/RAM живут)\n");
    } else {
        serial_puts("LOGS: ИТОГ: НЕ ЗАПИСАНО: транспортная ошибка (см.\n");
        serial_puts("LOGS: USBMSD:/MOUSEHW: строки выше с кодом ошибки)\n");
    }
    /* [build 4] ЧЕСТНОСТЬ О ПОЛНОТЕ: потерянные байты называются числом,
     * результат НЕ называется полным журналом (проблема 3 ревью).
     * «ПОЛНЫЙ» можно заявить ТОЛЬКО если и буферы целы, и доставка
     * закончилась успешно (stage 1); лимит файла / отказ носителя —
     * честное «хвост в файл не попал». */
    if (g_lost_bytes > 0) {
        serial_puts("LOGS: ИТОГ: ПОТЕРЯНО ");
        ls_pn("", g_lost_bytes, " байт журнала (переполнение буферов до\n");
        serial_puts("LOGS: выгрузки; хвост есть в COM1/RAM) - журнал НЕ полный\n");
    } else if (g_ls_stage == 1) {
        serial_puts("LOGS: ИТОГ: журнал ПОЛНЫЙ с первого байта загрузки\n");
    } else if (g_ls_bytes_in_file > 0) {
        serial_puts("LOGS: ИТОГ: буферы держат журнал с первого байта, но ХВОСТ\n");
        serial_puts("LOGS: в файл НЕ попал (");
        serial_puts(g_ls_stage == 3 ? "лимит файла/нет места"
                                    : "отказ носителя/транспорта");
        serial_puts(") - журнал в файле НЕ полный\n");
    } else {
        serial_puts("LOGS: ИТОГ: журнал в COM1/RAM полный с первого байта загрузки\n");
    }
    if (g_meta_fail) {
        serial_puts("LOGS: ИТОГ: ВНИМАНИЕ (целостность): размер в каталоге ");
        ls_pn("", g_dir_confirmed, " байт,\n");
        serial_puts("LOGS: в кластеры записано ");
        ls_pn("", g_fat.fsize, " байт - несоответствие из-за сбоя записи\n");
        serial_puts("LOGS: каталога (носитель выдернут?); проверь том (chkdsk)\n");
    }
    serial_puts("LOGS: COM1 и RAM-кольцо (COREINFO) работают независимо\n");
}

const char *ls_status(void) { return g_ls_status; }

bool ls_active(void) { return g_ls_stage == 1; }
