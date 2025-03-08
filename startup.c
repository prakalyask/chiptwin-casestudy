/**
 * Startup file for STM32F4 - Simplified for QEMU
 */

#include <stdint.h>

// Forward declaration of the reset handler
void Reset_Handler(void);

// Forward declaration of main
extern int main(void);

// Stack top
extern uint32_t _estack;

// Weak definition of system init
void SystemInit(void) __attribute__((weak));

// Default handler for unimplemented interrupts
void Default_Handler(void)
{
    while (1)
    {
    }
}

// Weak aliases for all interrupts
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

// Vector table
__attribute__((section(".isr_vector"))) void (*const g_pfnVectors[])(void) = {
    (void (*)(void))&_estack, // The initial stack pointer
    Reset_Handler,            // The reset handler
    NMI_Handler,              // The NMI handler
    HardFault_Handler,        // The hard fault handler
    MemManage_Handler,        // The MPU fault handler
    BusFault_Handler,         // The bus fault handler
    UsageFault_Handler,       // The usage fault handler
    0,                        // Reserved
    0,                        // Reserved
    0,                        // Reserved
    0,                        // Reserved
    SVC_Handler,              // SVCall handler
    DebugMon_Handler,         // Debug monitor handler
    0,                        // Reserved
    PendSV_Handler,           // The PendSV handler
    SysTick_Handler,          // The SysTick handler

    // Add more interrupt handlers here if needed for peripherals
};

// Reset handler
void Reset_Handler(void)
{
    // Call system initialization
    SystemInit();

    // Call main
    main();

    // If main returns, loop forever
    while (1)
    {
    }
}

// If SystemInit is not defined elsewhere, provide a minimal implementation
void SystemInit(void)
{
    // Nothing needed for QEMU simulation
}