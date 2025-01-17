#include <stdint.h>
#include <stddef.h>

#define FCLK 72000000UL
#include "stm32f103.h"



void fucked() {
  RCC_APB2ENR |= 1 << 4;
  // Set PC13 as an output
  GPIOC_CRH |= 0b11 << 20;
  GPIOC_CRH &= ~(0b11 << 22);
  while(1) {
    GPIOC_BSRR |= 1 << 13;
    GPIOC_BSRR |= 1 << 29;
  }
}

void heartbeat() {
  RCC_APB2ENR |= (1 << 2);
  // Set PA8 to an AFIO Push Pull Output
  GPIOA_CRH |= 0b11 | (0b10 << 2);
  GPIOA_CRH &= ~(1 << 2);
  // Set PA8 to an AFIO Push Pull Output
  GPIOA_CRH |= 0b11 | (0b10 << 2);
  GPIOA_CRH &= ~(1 << 2);
  // Output PLLCLK/2 on PA8
  RCC_CFGR |= 0b111 << 24;
}


void systick_init() {
  STK_LOAD = 9000;
}

void delay_ms(unsigned int ms) {
  for (; ms >= 1; ms--) {
    // Start timer
    STK_CTRL = 0;
    STK_CTRL |= 1;
    // Check whether the counter has counted down to 0
    while (!(STK_CTRL & (1 << 16)));
      
    // Stop timer
    STK_CTRL &= ~(1);
  }
}

void on_board_blink(){
	// Enable clock to Port C
	RCC_APB2ENR |= (1 << 4);
	// Setup TIM2 for interrupt generation. Period is 500ms.
	// Reset the GPIOC config register for the pins connected to the LED
	GPIOC_CRH &= ~((0b11 << 20) | (0b11 << 22));
	// Set the PC13 as an output
	GPIOC_CRH |= (0b10 << 20);
	while(1) {
		GPIOC_BSRR |= (1 << 13);
		delay_ms(1000);
		GPIOC_BSRR |= (1 << 29);
		delay_ms(100);
	}
}


void system_init() {
  // Turn on the external crystal clock
  RCC_CR |= 1 << 16;
  // Wait till the oscillator is stable
  while(!(RCC_CR & (1 << 17)));
  // Set a PLL MUL factor of 9 (9 * 8MHz = 72MHz)
  RCC_CFGR |= 0b111 << 18;
  // Set XO as the source for the PLL
  RCC_CFGR |= 1 << 16;
  // Set PCLK1 to 36Mhz
  RCC_CFGR |= 0b100 << 8;
  // Turn the PLL on
  RCC_CR |= 1 << 24;
  // Wait till PLL is stable
  while(!(RCC_CR & (1 << 25)));
  // Turn on heartbeat
  heartbeat();
  // Set flash latency to two wait states.
  FLASH_ACR |= 0b10;
  // Switch to the PLL as the main clock
  RCC_CFGR |= 0b10;
  systick_init();
}



int main(void) {
	system_init();
	on_board_blink();
}
