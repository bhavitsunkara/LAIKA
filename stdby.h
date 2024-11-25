#include "main.h"

#define TIMEOUT_MS		10000 // 10 seconds

void RTC_Config(void);
void RTC_WUT_Config(void);
void Enable_IRQ(void);
void RTC_EXTI_Setup(void);
void RTC_IRQHandler(void);
void STANDBY(void);
