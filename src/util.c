#include "util.h"
#include "stm32f4xx.h"

// probably have to tweak these if the processor speed changes. Just have to be rough
void delay_us(const uint16_t us)
{
   uint32_t i = us * 60;
   while (i-- > 0) {
      __asm volatile ("nop");
   }
}

void delay_ms(const uint16_t ms)
{
   //uint32_t i = ms * 27778;
   uint32_t i = ms * 30000 *2;
   while (i-- > 0) {
      __asm volatile ("nop");
   }
}

void blink_pa1(int n) {
        while(n) {
                GPIOA->BSRRH = 0x02; // LED ON
                delay_ms(50);
				GPIOA->BSRRL = 0x02; // LED OFF
                delay_ms(50);
                n--;
        }
		GPIOA->BSRRL = 0x02; // LED OFF
}

void reset_sord(int delay) {
		return;
		/*
		unfortunately didn't find way how to reset Sord programmatically
		below code isn't possible as the rst pin is connected to the inventor output what could potentially causing signal hazard.
		only way could be inject rst0 instruction when z80 asking for instruction from memory area we control
		GPIO_ResetBits(GPIOB, GPIO_Pin_10);                // ground PB10 -> reset
		GPIOB->MODER |= GPIO_MODER_MODER10_0;      // PB10 - as output
		delay_ms(delay);
		GPIOB->MODER &= ~(GPIO_MODER_MODER10);     // PB10 - as input     
		*/
}

void blink_debug_led(int delay) {
        while(1) {
                GPIOA->ODR |= GPIO_DEBUG_LED;
                delay_ms(delay);
                GPIOA->ODR &= ~(GPIO_DEBUG_LED);
                delay_ms(delay);
        }
}

void my_memcpy(unsigned char *dest, unsigned char *from, int length) {
	unsigned char *q = dest;
	unsigned char *p = from;

	while (p < from + length) {
		*q++ = *p++;
	}

}
