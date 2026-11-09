/*
 * syscalls.c -- Minimal syscall stubs for newlib-nano on S32K116
 *
 * Provides _sbrk for heap allocation (required by vsnprintf via nano.specs).
 */

#include <stdint.h>
#include <errno.h>

/* Defined by linker script */
extern uint32_t __end__;
extern uint32_t __HeapLimit;

void *_sbrk(int incr)
{
    static uint8_t *heap_end = 0;
    uint8_t *prev_heap_end;

    if (heap_end == 0) {
        heap_end = (uint8_t *)&__end__;
    }

    prev_heap_end = heap_end;

    if ((heap_end + incr) > (uint8_t *)&__HeapLimit) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return (void *)prev_heap_end;
}
