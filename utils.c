#include "utils.h"
#define FCLK 72000000UL
#include "stm32f103.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>


void itoa(int a, char* buf, int n) {
	if(a == 0) {
		if(n >= 2){
			buf[0] = '0';
			buf[1] = 0; 
		}
		return;
	}
	int num_digits = 0;
	int a2 = a;
	for(;a2 != 0; a2 /= 10) {
		num_digits++;
	}
	int i = 0;
	for(; i < num_digits; i++) {
		if(a == 0)
			break;
		buf[num_digits - 1 - i] = '0' + (a % 10);
		a = a/10;
	}
	buf[i] = 0;
}


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

void on_board_blink(int delay){
	// Enable clock to Port C
	RCC_APB2ENR |= (1 << 4);
	// Setup TIM2 for interrupt generation. Period is 500ms.
	// Reset the GPIOC config register for the pins connected to the LED
	GPIOC_CRH &= ~((0b11 << 20) | (0b11 << 22));
	// Set the PC13 as an output
	GPIOC_CRH |= (0b10 << 20);
	while(1) {
		GPIOC_BSRR |= (1 << 13);
		delay_ms(delay);
		GPIOC_BSRR |= (1 << 29);
		delay_ms(delay);
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

void init_uart() {
  //Setup the GPIO to the appropriate AFIO
  // Enable AFIO clocks
  RCC_APB2ENR |= 1;
  // Clock PortA
  RCC_APB2ENR |= 1 << 2;
  // Reset the GPIO states to 0 for PA9 and PA10
  GPIOA_CRH &= ~(0xff << 4);
  //Set PA9 as an AFIO Output
  GPIOA_CRH |= (0b11 << 4) | (0b10 << 6);
  //Set PA10 as an input
  GPIOA_CRH |= (0b01 << 10);

  // Enable the uart clock
  RCC_APB2ENR |= 1 << 14;
  // Set the baud rate. The default is 9600
  USART1_BRR = BAUDRATE_FRACTION(9600) | (BAUDRATE_MANTISSA(9600) << 4);

  // Enable the UART
  USART1_CR1 = 1 << 13;

  // Reset the control resistors to a known state
  USART1_CR2 = 0;
  USART1_CR3 = 0; 
  // Enable the transmitter and receiver
  USART1_CR1 |= 0b11 << 2;
}

void send_uart(char a) {
  // Wait till data is flushed from the output buffer
  while(!( USART1_SR & (1 << 7 )));
  USART1_DR = a;
}


char receive_byte_uart() {
  // Wait till data is in the input buffer
  while(!(USART1_SR & (1 << 5)));
  return USART1_DR;
}

void print_uart(char* msg) {
  for(uint32_t i = 0; msg[i] != 0; i++) {
    send_uart(msg[i]);
  }
}

void iprint_uart(int num){
	char buf[10];
	itoa(num, buf, 10);
	print_uart(buf);
}

void read_uart(char* buf, unsigned int len) {
  for(uint32_t i = 0; i < len - 1; i++) {
    buf[i] = receive_byte_uart();
    if(( buf[i] == '\r' ) || (buf[i] == '\n')){
      buf[i + 1] = 0;
      return;
    }
  }
  buf[len - 1] = 0;
}



void debug_log(char* msg) {
#if LOG==1
	print_uart(msg);
#endif
}

void kill(char* msg) {
#if LOG==1
	print_uart(msg);
	fucked();
#endif
}
