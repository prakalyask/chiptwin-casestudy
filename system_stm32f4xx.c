/**
 * System initialization for STM32F4 - Simplified for QEMU
 */

#include <stdint.h>

// System Core Clock (HCLK) frequency in Hz
uint32_t SystemCoreClock = 168000000;

/**
 * Setup the microcontroller system
 * Initialize the FPU, Vector Table, and PLL
 */
void SystemInit(void)
{
// Set FPU settings (enable FPU if needed)
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    // SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2)); // Set CP10 and CP11 Full Access
    *((volatile uint32_t *)0xE000ED88) |= ((3UL << 10 * 2) | (3UL << 11 * 2));
#endif

    // Reset RCC clock configuration to default state
    // Simplified for QEMU - no need to configure clocks

    // Configure Vector Table location
    // SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;
    *((volatile uint32_t *)0xE000ED08) = 0x08000000;
}

/**
 * Update SystemCoreClock variable
 */
void SystemCoreClockUpdate(void)
{
    // For QEMU, just set a fixed value
    SystemCoreClock = 168000000;
}