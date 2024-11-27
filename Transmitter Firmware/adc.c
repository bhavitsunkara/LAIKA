#include "adc.h"

// Function to initialize ADC
void ADC_Init(void) {
    // Enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
    
    // Enable GPIOA clock (for analog input pin, e.g., PA0)
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    // Configure PA0 as analog mode
    GPIOA->MODER |= (0x3 << GPIO_MODER_MODE0_Pos); // Set PA0 to Analog Mode
    GPIOA->PUPDR &= ~(0x3 << GPIO_PUPDR_PUPD0_Pos); // No pull-up, no pull-down

    // Ensure the ADC is disabled before configuration
    if (ADC1->CR & ADC_CR_ADEN) {
        ADC1->CR |= ADC_CR_ADDIS; // Disable ADC
        while (ADC1->CR & ADC_CR_ADEN); // Wait until it is disabled
    }

    // Configure ADC settings
    ADC1->CFGR1 &= ~ADC_CFGR1_CONT;  // Single conversion mode
    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN; // Right alignment (default)

    // Set ADC clock (use synchronous clock mode divided by 1)
    ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; // Clear clock mode bits
    ADC1->CFGR2 |= ADC_CFGR2_CKMODE_1; // PCLK divided by 4

    // Enable the voltage regulator for ADC
    ADC1->CR |= ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 1000; i++); // Small delay for the regulator to stabilize

    // Calibrate the ADC
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL); // Wait for calibration to complete
		
		ADC1->SMPR |= 7;
		
    // Enable the ADC
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY)); // Wait until ADC is ready
}

// Function to start ADC conversion and get the result
uint16_t ADC_Read(void) {
    // Select the input channel (channel 0 for PA0)
    ADC1->CHSELR = ADC_CHSELR_CHSEL0; // Channel 0 (PA0)

    // Start the conversion
    ADC1->CR |= ADC_CR_ADSTART;

    // Wait for the conversion to complete
    while (!(ADC1->ISR & ADC_ISR_EOC));

    // Return the ADC conversion result
    return (uint16_t)ADC1->DR;
}
