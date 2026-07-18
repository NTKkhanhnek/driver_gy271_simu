#include "delay.h"

#define DEMCR       (*(volatile uint32_t *)0xE000EDFC)
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004)

#define DEMCR_TRCENA      (1UL << 24)
#define DWT_CTRL_CYCCNTENA (1UL << 0)

static uint32_t cycles_per_us = 16;

void delay_init(uint32_t core_clock_hz)
{
    cycles_per_us = core_clock_hz / 1000000UL;
    if (cycles_per_us == 0)
    {
        cycles_per_us = 1;
    }

    DEMCR |= DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT_CYCCNT;
    uint32_t cycles = us * cycles_per_us;

    while ((uint32_t)(DWT_CYCCNT - start) < cycles)
    {
    }
}

void delay_ms(uint32_t ms)
{
    while (ms-- > 0)
    {
        delay_us(1000);
    }
}
