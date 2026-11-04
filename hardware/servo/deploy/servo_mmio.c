/* Exactly one aligned 32-bit device access per call. No memcpy/pack_into. */
#include <stdint.h>

static inline void device_barrier(void)
{
#if defined(__aarch64__)
    __asm__ volatile("dsb sy" ::: "memory");
#else
    __sync_synchronize();
#endif
}

uint32_t servo_read32(void *base, uint32_t offset)
{
    device_barrier();
    uint32_t value = *(volatile uint32_t *)((unsigned char *)base + offset);
    device_barrier();
    return value;
}

void servo_write32(void *base, uint32_t offset, uint32_t value)
{
    device_barrier();
    *(volatile uint32_t *)((unsigned char *)base + offset) = value;
    device_barrier();
}
