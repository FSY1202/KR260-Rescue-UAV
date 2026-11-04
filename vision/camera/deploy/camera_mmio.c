/* Atomic-width MMIO access. Never use struct.pack_into/memset on device memory. */
#include <stdint.h>

static void barrier(void)
{
#if defined(__aarch64__)
    __asm__ volatile("dsb sy" ::: "memory");
#else
    __sync_synchronize();
#endif
}

uint32_t camera_mmio_read32(void *base, uint32_t offset)
{
    barrier();
    uint32_t value = *(volatile uint32_t *)((unsigned char *)base + offset);
    barrier();
    return value;
}

void camera_mmio_write32(void *base, uint32_t offset, uint32_t value)
{
    barrier();
    *(volatile uint32_t *)((unsigned char *)base + offset) = value;
    barrier();
}

void camera_mmio_copy32(void *base, uint32_t offset, void *output, uint32_t words)
{
    volatile uint32_t *source = (volatile uint32_t *)((unsigned char *)base + offset);
    uint32_t *destination = (uint32_t *)output;
    barrier();
    for (uint32_t i = 0; i < words; ++i)
        destination[i] = source[i];
    barrier();
}
