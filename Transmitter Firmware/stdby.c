#include "stdby.h"

void RTC_Config(void) {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable Power interface clock
    PWR->CR |= PWR_CR_DBP;             // Enable access to backup domain

    RCC->CSR |= RCC_CSR_LSION;         // Enable LSI oscillator
    while (!(RCC->CSR & RCC_CSR_LSIRDY)); // Wait for LSI to stabilize

    RCC->CSR |= RCC_CSR_RTCSEL_LSI;    // Select LSI as RTC clock source
    RCC->CSR |= RCC_CSR_RTCEN;         // Enable RTC clock

    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;                   // Disable write protection

    RTC->ISR |= RTC_ISR_INIT;          // Enter initialization mode
    while (!(RTC->ISR & RTC_ISR_INITF)); // Wait for INITF flag

    RTC->PRER = (127 << RTC_PRER_PREDIV_A_Pos) | 288; // Set prescaler (1 Hz RTC clock)

    RTC->ISR &= ~RTC_ISR_INIT;         // Exit initialization mode
    RTC->WPR = 0xFF;                   // Enable write protection
}

void RTC_WUT_Config(uint16_t seconds) {
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;                   // Disable write protection

    RTC->CR &= ~RTC_CR_WUTE;           // Disable Wakeup Timer
    while (!(RTC->ISR & RTC_ISR_WUTWF)); // Wait until WUTWF is set

    RTC->WUTR = seconds;               // Set Wakeup Timer for the desired seconds
    RTC->CR |= RTC_CR_WUTIE;           // Enable Wakeup Timer interrupt

    RTC->CR &= ~RTC_CR_WUCKSEL;        // Select RTC 1 Hz clock
    RTC->CR |= RTC_CR_WUCKSEL_2;

    RTC->CR |= RTC_CR_WUTE;            // Enable Wakeup Timer
    RTC->WPR = 0xFF;                   // Enable write protection
}

void RTC_EXTI_Setup(void) {
    EXTI->IMR |= EXTI_IMR_IM20;   // Unmask EXTI Line 20
    EXTI->RTSR |= EXTI_RTSR_TR20; // Enable rising trigger
}

void Enable_IRQ(void) {
    NVIC_EnableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 2); // Set interrupt priority
}

void RTC_IRQHandler(void) {
    if (RTC->ISR & RTC_ISR_WUTF) {  // Check Wakeup flag
        RTC->ISR &= ~RTC_ISR_WUTF;  // Clear Wakeup flag
        GPIOA->ODR ^= GPIO_ODR_OD13; // Example: Toggle an LED
    }

    EXTI->PR |= EXTI_PR_PIF20;      // Clear EXTI Line 20 pending flag
}

void STANDBY(void) {
    RTC->ISR &= ~RTC_ISR_WUTF;      			// Clear Wakeup flag
    PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF; // Clear Wakeup and Standby flags

    PWR->CR |= PWR_CR_PDDS;         		// Select Standby mode
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; 	// Set SLEEPDEEP bit

    __DSB();                        // Ensure memory operations are complete
    __WFI();                        // Enter Standby mode
}