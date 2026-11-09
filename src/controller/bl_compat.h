/*
 * bl_compat.h -- run-time check that the bootloader agrees with what this image was
 * built for.
 *
 * The bootloader publishes a read-only block at a fixed address describing the
 * constants the two images must share. They live in separate repositories
 * (../../nxp_bms_bootloader), so nothing checks them at build time.
 *
 * The struct below must match src/bl_info.h in that repository. Its first three
 * fields are a stable prefix, so the ABI number stays readable across a layout change.
 */
#ifndef BL_COMPAT_H
#define BL_COMPAT_H

#include <stdint.h>

/* Must match BL_SHARED_BASE / BL_ENTRY_MAGIC in the bootloader, and the 16-byte
 * shortfall in m_data_2 in linker/S32K116_17_app4000.ld. */
#define BL_ENTRY_MAGIC_ADDR  0x200037F0u
#define BL_ENTRY_MAGIC       0x5570644Du

typedef enum {
    BL_COMPAT_NONE = 0,   /* no bootloader: this is a flat image, or an old one */
    BL_COMPAT_OK,
    BL_COMPAT_ABI,        /* bootloader implements a different contract version */
    BL_COMPAT_CONTRACT,   /* same ABI, but the shared constants disagree */
} bl_compat_status_t;

bl_compat_status_t bl_compat_check(void);
uint16_t           bl_compat_abi(void);
uint32_t           bl_compat_features(void);
uint32_t           bl_compat_build_id(void);
void               bl_compat_print(void);

/* Packed into the command response's 32-bit value field:
 *   bits 0-7   status      8-15  ABI      16-31  features
 */
uint32_t bl_compat_report_word(void);

#endif
