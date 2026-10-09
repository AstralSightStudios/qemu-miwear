/*
 * BES 固件镜像解析器
 *
 * 解析逻辑与 best1503_vela.py（IDA loader）完全一致——所有布局信息
 * 从镜像头解析，不写死任何地址，换固件版本自动适配。
 *
 * 格式依据（实测自 vela_ap.bin v3.101.043 / vela_bl2.bin）：
 *   [0x00] magic 0xFFFFFFFF / [0x04] version 0x00050000 / [0x08] 保留
 *   [0x0C] boot info 指针（NC 闪存别名地址）
 *   [0x64] mapped 入口指针（奇数 Thumb 指针），XIP 基址 = (ptr & ~1) - 0x14
 *   [0x68..0x80] MSP / MSPLIM / data LMA / data VMA / data end / bss 范围
 *   文件尾 4 字节：NC 别名映射基址（用于定位 boot info 文件偏移）
 *   boot info 为 KEY=value 文本，提供 FLASH_BASE/FLASH_NC_BASE/OTA_CODE_OFFSET
 *
 * This code is licensed under the GPL version 2 or later.
 */

#include "qemu/osdep.h"
#include "hw/misc/bes_image.h"
#include "qemu/error-report.h"

#define BES_HDR_MAGIC    0xFFFFFFFFu
#define BES_HDR_VERSION  0x00050000u

#define BES_OFF_BOOTINFO_PTR  0x0C
#define BES_OFF_MAPPED_PTR    0x64
#define BES_OFF_MSP           0x68
#define BES_OFF_MSPLIM        0x6C
#define BES_OFF_DATA_LMA      0x70
#define BES_OFF_DATA_START    0x74
#define BES_OFF_DATA_END      0x78
#define BES_OFF_BSS_START     0x7C
#define BES_OFF_BSS_END       0x80

/* 入口 stub 相对 XIP 基址的偏移（ldr r0,[pc,#0x50]; bx r0） */
#define BES_ENTRY_OFFSET      0x10
#define BES_MAPPED_OFFSET     0x14

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
    if (img->size < BES_OFF_BSS_END + 4) {
        error_setg(errp, "%s: file too small", path);
        goto fail;
    }

    uint32_t magic = bes_ldl(img->data);
    uint32_t version = bes_ldl(img->data + 0x04);
    if (magic != BES_HDR_MAGIC || version != BES_HDR_VERSION) {
        error_setg(errp, "%s: not a BES image (magic=0x%08x ver=0x%08x)",
                   path, magic, version);
        goto fail;
    }

    uint32_t boot_info_ptr = bes_ldl(img->data + BES_OFF_BOOTINFO_PTR);
    uint32_t mapped_ptr = bes_ldl(img->data + BES_OFF_MAPPED_PTR);
    if (!(mapped_ptr & 1)) {
        error_setg(errp, "%s: mapped entry pointer is not Thumb", path);
        goto fail;
    }
    img->xip_base = (mapped_ptr & ~1u) - BES_MAPPED_OFFSET;
    img->entry = img->xip_base + BES_ENTRY_OFFSET;

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

    img->msp        = bes_ldl(img->data + BES_OFF_MSP);
    img->msplim     = bes_ldl(img->data + BES_OFF_MSPLIM);
    img->data_lma   = bes_ldl(img->data + BES_OFF_DATA_LMA);
    img->data_start = bes_ldl(img->data + BES_OFF_DATA_START);
    img->data_end   = bes_ldl(img->data + BES_OFF_DATA_END);
    img->bss_start  = bes_ldl(img->data + BES_OFF_BSS_START);
    img->bss_end    = bes_ldl(img->data + BES_OFF_BSS_END);

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
