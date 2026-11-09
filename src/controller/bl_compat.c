#include "bl_compat.h"
#include "hal_uart.h"

#define BL_INFO_ADDR   0x000000C0u
#define BL_INFO_MAGIC  0x424C494Eu   /* "BLIN" */
#define BL_INFO_ABI    1u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t abi;
    uint16_t size;
    uint32_t app_base;
    uint32_t shared_addr;
    uint32_t shared_magic;
    uint32_t features;
    uint32_t build_id;
    uint32_t reserved;
} bl_info_t;

/* Where this image starts, taken from the linker rather than a constant. */
extern uint32_t __VECTOR_TABLE[];

static const bl_info_t *info(void)
{
    return (const bl_info_t *)BL_INFO_ADDR;
}

bl_compat_status_t bl_compat_check(void)
{
    const bl_info_t *i = info();

    if (i->magic != BL_INFO_MAGIC)  return BL_COMPAT_NONE;
    if (i->abi   != BL_INFO_ABI)    return BL_COMPAT_ABI;

    /* app_base against where this image is linked, so a bootloader that would jump
     * elsewhere is caught; and the handover window, since a disagreement there leaves
     * the magic where nothing reads it and the board resets back into the application. */
    if (i->app_base     != (uint32_t)__VECTOR_TABLE) return BL_COMPAT_CONTRACT;
    if (i->shared_addr  != BL_ENTRY_MAGIC_ADDR)      return BL_COMPAT_CONTRACT;
    if (i->shared_magic != BL_ENTRY_MAGIC)           return BL_COMPAT_CONTRACT;

    return BL_COMPAT_OK;
}

uint16_t bl_compat_abi(void)      { return info()->abi; }
uint32_t bl_compat_features(void) { return info()->features; }
uint32_t bl_compat_build_id(void) { return info()->build_id; }

uint32_t bl_compat_report_word(void)
{
    bl_compat_status_t st = bl_compat_check();
    uint32_t feat = (st == BL_COMPAT_NONE) ? 0u : (bl_compat_features() & 0xFFFFu);
    uint32_t abi  = (st == BL_COMPAT_NONE) ? 0u : (uint32_t)bl_compat_abi();

    return ((uint32_t)st & 0xFFu) | ((abi & 0xFFu) << 8) | (feat << 16);
}

void bl_compat_print(void)
{
    const bl_info_t *i = info();

    switch (bl_compat_check()) {
    case BL_COMPAT_NONE:
        hal_uart_printf("[BLC] no bootloader (flat image at 0x%lX)\r\n",
                        (unsigned long)(uint32_t)__VECTOR_TABLE);
        break;

    case BL_COMPAT_OK:
        hal_uart_printf("[BLC] bootloader abi=%u build=%lX features=%lX\r\n",
                        (unsigned)i->abi,
                        (unsigned long)i->build_id,
                        (unsigned long)i->features);
        break;

    case BL_COMPAT_ABI:
        hal_uart_printf("[BLC] WARNING bootloader abi=%u, this image expects %u\r\n",
                        (unsigned)i->abi, (unsigned)BL_INFO_ABI);
        break;

    case BL_COMPAT_CONTRACT:
    default:
        hal_uart_printf("[BLC] WARNING shared constants differ: "
                        "base %lX vs %lX, window %lX vs %lX\r\n",
                        (unsigned long)i->app_base,
                        (unsigned long)(uint32_t)__VECTOR_TABLE,
                        (unsigned long)i->shared_addr,
                        (unsigned long)BL_ENTRY_MAGIC_ADDR);
        break;
    }
}
