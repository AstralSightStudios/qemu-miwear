/*
 * BES 固件镜像解析器
 *
 * 解析逻辑与 best1503_vela.py（IDA loader）完全一致——所有布局信息
 * 从镜像头解析，不写死任何地址，换固件版本自动适配。
 *
 * 格式依据（实测自 vela_ap.bin v3.101.043 / vela_bl2.bin）：
 *   [0x00] magic 0xFFFFFFFF（best1502p 的 REDMI Watch 6 包为 boot struct
 *          magic 0xBE57EC1C，其余字段相同）/ [0x04] version 0x00050000 /
 *          [0x08] 保留
 *   [0x0C] boot info 指针（NC 闪存别名地址）
 *   [0x10] 入口 stub：ldr r0,[pc,#imm]; bx r0
 *   [0x64] stub 取的 mapped 入口指针（奇数 Thumb 指针，指向 stub 之后），
 *          XIP 基址 = (ptr & ~1) - 0x14
 *   [0x68..0x80] MSP / MSPLIM / data LMA / data VMA / data end / bss 范围
 *   文件尾 4 字节：NC 别名映射基址（用于定位 boot info 文件偏移）
 *   boot info 为 KEY=value 文本，提供 FLASH_BASE/FLASH_NC_BASE/OTA_CODE_OFFSET
 *
 * 签名镜像（version 0x00050001，q63 vela_tee.bin）：头后插入签名块
 * （[0x498] 长度 + RSA-3072 签名等），入口 stub 及其后各字段整体后移
 * （q63 TEE：stub @0x628、mapped 指针 @0x680）；文件末尾另有 AVB 尾
 * （最后 64 字节以 "AVBf" 开头，+0x0C 为大端 64 位原始镜像长度），NC
 * 基址在原始镜像的最后 4 字节。故 stub 位置按指令形态查找：mapped
 * 指针必须恰好指向 stub 之后、且所得 XIP 基址 4KB 对齐。
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/misc/bes_image.h"
#include "qemu/error-report.h"
#include "qemu/bswap.h"

#define BES_HDR_MAGIC    0xFFFFFFFFu
#define BES_HDR_MAGIC_BOOT_STRUCT 0xBE57EC1Cu
#define BES_HDR_VERSION  0x00050000u
#define BES_HDR_VERSION_SIGNED 0x00050001u

#define BES_OFF_BOOTINFO_PTR  0x0C
/* 以下相对 mapped 入口指针所在偏移 */
#define BES_OFF_MSP           0x04
#define BES_OFF_MSPLIM        0x08
#define BES_OFF_DATA_LMA      0x0C
#define BES_OFF_DATA_START    0x10
#define BES_OFF_DATA_END      0x14
#define BES_OFF_BSS_START     0x18
#define BES_OFF_BSS_END       0x1C

#define BES_STUB_SEARCH       0x1000    /* stub 在镜像头 4KB 内 */
#define BES_AVB_FOOTER_SIZE   64

static uint32_t bes_ldl(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* 解析 boot info 文本里的 KEY=value（NUL 结尾的 ASCII 块） */
static bool bes_parse_boot_cfg(BesImage *img, const char *text, Error **errp)
{
    const char *p = text;
    bool got_flash = false, got_flash_nc = false, got_ota = false;

    while (*p) {
        const char *eol = strchr(p, '\n');
        gsize len = eol ? (gsize)(eol - p) : strlen(p);
        g_autofree char *line = g_strndup(p, len);
        char *eq = strchr(line, '=');

        if (eq) {
            *eq = '\0';
            const char *key = g_strstrip(line);
            const char *val = g_strstrip(eq + 1);
            if (g_str_equal(key, "FLASH_BASE")) {
                img->flash_base = (uint32_t)g_ascii_strtoull(val, NULL, 0);
                got_flash = true;
            } else if (g_str_equal(key, "FLASH_NC_BASE")) {
                img->flash_nc_base = (uint32_t)g_ascii_strtoull(val, NULL, 0);
                got_flash_nc = true;
            } else if (g_str_equal(key, "OTA_CODE_OFFSET")) {
                img->ota_code_offset = (uint32_t)g_ascii_strtoull(val, NULL, 0);
                got_ota = true;
            } else if (g_str_equal(key, "FLASH_SIZE")) {
                img->flash_size = (uint32_t)g_ascii_strtoull(val, NULL, 0);
            } else if (g_str_equal(key, "CHIP")) {
                g_strlcpy(img->chip, val, sizeof(img->chip));
            }
        }
        if (!eol) {
            break;
        }
        p = eol + 1;
    }

    if (!got_flash || !got_flash_nc || !got_ota) {
        error_setg(errp, "BES image boot info missing "
                   "FLASH_BASE/FLASH_NC_BASE/OTA_CODE_OFFSET");
        return false;
    }
    return true;
}

bool bes_image_load(BesImage *img, const char *path, Error **errp)
{
    memset(img, 0, sizeof(*img));

    {
        GError *gerr = NULL;
        if (!g_file_get_contents(path, (gchar **)&img->data,
                                 (gsize *)&img->size, &gerr)) {
            error_setg(errp, "failed to read %s: %s", path, gerr->message);
            g_error_free(gerr);
            return false;
        }
    }
    if (img->size < BES_STUB_SEARCH) {
        error_setg(errp, "%s: file too small", path);
        goto fail;
    }

    uint32_t magic = bes_ldl(img->data);
    uint32_t version = bes_ldl(img->data + 0x04);
    if ((magic != BES_HDR_MAGIC && magic != BES_HDR_MAGIC_BOOT_STRUCT) ||
        (version != BES_HDR_VERSION && version != BES_HDR_VERSION_SIGNED)) {
        error_setg(errp, "%s: not a BES image (magic=0x%08x ver=0x%08x)",
                   path, magic, version);
        goto fail;
    }

    /* AVB 尾：只看原始镜像部分 */
    {
        const uint8_t *f = img->data + img->size - BES_AVB_FOOTER_SIZE;

        if (!memcmp(f, "AVBf", 4)) {
            uint64_t orig = ldq_be_p(f + 0x0C);

            if (orig < BES_STUB_SEARCH || orig > img->size) {
                error_setg(errp, "%s: bad AVB footer", path);
                goto fail;
            }
            img->size = orig;
        }
    }

    uint32_t boot_info_ptr = bes_ldl(img->data + BES_OFF_BOOTINFO_PTR);
    uint32_t mapped_off = 0, mapped_ptr = 0;
    uint32_t stub;

    for (stub = 0x10; stub < BES_STUB_SEARCH; stub += 2) {
        const uint8_t *p = img->data + stub;
        uint32_t lit;

        /* ldr r0,[pc,#imm8*4] (0x48xx); bx r0 (0x4700) */
        if (p[1] != 0x48 || p[2] != 0x00 || p[3] != 0x47) {
            continue;
        }
        lit = ((stub + 4) & ~3u) + p[0] * 4;
        if (lit + BES_OFF_BSS_END + 4 > img->size) {
            continue;
        }
        mapped_ptr = bes_ldl(img->data + lit);
        if ((mapped_ptr & 1) &&
            !(((mapped_ptr & ~1u) - (stub + 4)) & 0xFFF)) {
            mapped_off = lit;
            break;
        }
    }
    if (!mapped_off) {
        error_setg(errp, "%s: no entry stub", path);
        goto fail;
    }
    img->xip_base = (mapped_ptr & ~1u) - (stub + 4);
    img->entry = img->xip_base + stub;

    /* last 4 bytes = NC-alias mapped base, used to locate boot info */
    uint32_t mapped_nc = bes_ldl(img->data + img->size - 4);
    if (boot_info_ptr < mapped_nc) {
        error_setg(errp, "%s: bad boot info pointer", path);
        goto fail;
    }
    uint64_t boot_off = (uint64_t)boot_info_ptr - mapped_nc;
    if (boot_off >= img->size - 4) {
        error_setg(errp, "%s: boot info offset out of range", path);
        goto fail;
    }

    if (!bes_parse_boot_cfg(img, (const char *)img->data + boot_off, errp)) {
        goto fail;
    }
    img->cached_base = img->flash_base + img->ota_code_offset;
    img->nc_base = img->flash_nc_base + img->ota_code_offset;
    img->has_boot_cfg = true;

    /* consistency check (same as the IDA loader) */
    if (img->nc_base != mapped_nc) {
        error_setg(errp, "%s: NC base mismatch (parsed 0x%08x != footer 0x%08x)",
                   path, img->nc_base, mapped_nc);
        goto fail;
    }

    img->msp        = bes_ldl(img->data + mapped_off + BES_OFF_MSP);
    img->msplim     = bes_ldl(img->data + mapped_off + BES_OFF_MSPLIM);
    img->data_lma   = bes_ldl(img->data + mapped_off + BES_OFF_DATA_LMA);
    img->data_start = bes_ldl(img->data + mapped_off + BES_OFF_DATA_START);
    img->data_end   = bes_ldl(img->data + mapped_off + BES_OFF_DATA_END);
    img->bss_start  = bes_ldl(img->data + mapped_off + BES_OFF_BSS_START);
    img->bss_end    = bes_ldl(img->data + mapped_off + BES_OFF_BSS_END);

    if (!(img->msplim < img->msp &&
          img->data_start < img->data_end &&
          img->data_end == img->bss_start &&
          img->bss_start <= img->bss_end)) {   /* bss 可为空（==） */
        error_setg(errp, "%s: RAM layout validation failed", path);
        goto fail;
    }

    return true;

fail:
    bes_image_unload(img);
    return false;
}

void bes_image_unload(BesImage *img)
{
    g_free(img->data);
    img->data = NULL;
    img->size = 0;
}

unsigned bes_image_count(const BesImage *img, const char *str)
{
    size_t len = strlen(str);
    const uint8_t *p = img->data;
    const uint8_t *end = img->data + img->size;
    unsigned n = 0;

    if (!len) {
        return 1;
    }
    while (end - p >= len && (p = memchr(p, str[0], end - p - len + 1))) {
        if (!memcmp(p, str, len)) {
            n++;
            p += len;
        } else {
            p++;
        }
    }
    return n;
}

bool bes_image_contains(const BesImage *img, const char *str)
{
    return bes_image_count(img, str) != 0;
}
