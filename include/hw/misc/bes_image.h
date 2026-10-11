/*
 * BES 固件镜像解析器
 *
 * This code is licensed under the GPL version 2 or later.
 */

#ifndef HW_MISC_BES_IMAGE_H
#define HW_MISC_BES_IMAGE_H

#include "qapi/error.h"

typedef struct BesImage {
    uint8_t *data;          /* 镜像内容（本结构拥有） */
    size_t size;

    uint32_t xip_base;      /* 执行别名基址 */
    uint32_t entry;         /* 入口 stub 地址 */
    uint32_t cached_base;   /* cached 闪存别名基址 */
    uint32_t nc_base;       /* non-cached 闪存别名基址 */

    uint32_t msp;           /* 初始主栈指针 */
    uint32_t msplim;        /* 主栈下限 */
    uint32_t data_lma;      /* .data 加载地址（cached 别名） */
    uint32_t data_start;    /* .data VMA */
    uint32_t data_end;
    uint32_t bss_start;
    uint32_t bss_end;

    /* boot info 里的关键配置 */
    uint32_t flash_base;
    uint32_t flash_nc_base;
    uint32_t flash_size;
    uint32_t ota_code_offset;
    char chip[32];          /* CHIP=，如 "best1503"；缺省为空串 */
    bool has_boot_cfg;
} BesImage;

bool bes_image_load(BesImage *img, const char *path, Error **errp);
void bes_image_unload(BesImage *img);
/* 镜像里是否含有字符串 str（不含结尾 NUL），用于识别产品 */
/* how often str occurs in the image */
unsigned bes_image_count(const BesImage *img, const char *str);
bool bes_image_contains(const BesImage *img, const char *str);

#endif
