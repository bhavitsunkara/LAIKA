#include "stm32l051xx.h"  // Device-specific header file
#include "lora.h"
#include "adc.h"
#include "stdby.h"

#define LED_PIN        13  // Pin PC13 (usually onboard LED)
#define LED_PORT       GPIOC
#define LED_PORT_CLK   RCC_IOPENR_GPIOCEN  // Clock for GPIOC in STM32L051

volatile uint16_t i;

void delay(volatile uint32_t delay_count) {
    while (delay_count--) {
        __NOP();  // No operation, just waste time
    }
}

int main(void) {
		
		ADC_Init();
    // Enable the clock for GPIOC
    RCC->IOPENR |= LED_PORT_CLK;
    
    // Wait for the clock to stabilize
    __NOP(); 
    
    // Configure PC13 as output (clear MODE bits, set to output mode)
    LED_PORT->MODER &= ~(0x03 << (LED_PIN * 2)); // Clear MODE[1:0] for PC13
    LED_PORT->MODER |= (0x01 << (LED_PIN * 2));  // Set MODE[1:0] to 01 (output mode)

    // Configure PC13 as push-pull (clear OTYPER bit)
    LED_PORT->OTYPER &= ~(1 << LED_PIN); // Clear the output type bit for PC13
		
		begin(FREQ_868_MHZ);
	
		// Toggle the LED (flip output state)
    LED_PORT->ODR &= ~(1 << LED_PIN);  // Toggle PC13
		
		float j  =  ADC_Read();
		j +=  ADC_Read();
		j +=	ADC_Read();
		
		j = j/3;
		
		i = (uint16_t)(j/4096.0*3.3*100);
		
		char buffer [20];
		uint8_t int_buffer [20];
			
		sprintf(buffer, "Temp Data: %d C", i);
		
		for(uint8_t j = 0; j < sizeof(buffer)-1; j++){
			int_buffer[j] = buffer[j];
		}
				
		beginPacket(0);
		write(int_buffer, sizeof(buffer)-1);
		endPacket();
				
		++i;
		
		//if(!(PWR->CSR & PWR_CSR_SBF)){
			//i = 1;
		RTC_Config();
		//}
		
		RTC_WUT_Config();
		Enable_IRQ();
		STANDBY();
		
		while(1){
		}
}