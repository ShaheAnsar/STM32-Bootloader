#include <stdint.h>
#include <stddef.h>

#define FCLK 72000000UL

#define RCC 0x40021000
#define RCC_APB2ENR ( *( (volatile uint32_t*)(RCC + 0x18) ) )
#define RCC_CR (*(volatile uint32_t*)(RCC + 0x00))
#define RCC_CFGR (*(volatile uint32_t*)(RCC + 0x04))

#define GPIOC 0x40011000
#define GPIOC_CRL (*((volatile uint32_t*)(GPIOC + 0x00)))
#define GPIOC_CRH (*((volatile uint32_t*)(GPIOC + 0x04)))
#define GPIOC_IDR (*(volatile uint32_t*)(GPIOC + 0x08))
#define GPIOC_ODR (*(volatile uint32_t*)(GPIOC + 0x0C))
#define GPIOC_BSRR (*(volatile uint32_t*)(GPIOC + 0x10))
#define GPIOC_BRR (*(volatile uint32_t*)(GPIOC + 0x14))
#define FLASH 0x40022000
#define FLASH_ACR (*(volatile uint32_t*)(FLASH + 0x00))

#define GPIOB 0x40010C00
#define GPIOB_CRL (*((volatile uint32_t*)(GPIOB + 0x00)))
#define GPIOB_CRH (*((volatile uint32_t*)(GPIOB + 0x04)))
#define GPIOB_IDR (*(volatile uint32_t*)(GPIOB + 0x08))
#define GPIOB_ODR (*(volatile uint32_t*)(GPIOB + 0x0C))
#define GPIOB_BSRR (*(volatile uint32_t*)(GPIOB + 0x10))
#define GPIOB_BRR (*(volatile uint32_t*)(GPIOB + 0x14))

#define GPIOA 0x40010800
#define GPIOA_CRL (*((volatile uint32_t*)(GPIOA + 0x00)))
#define GPIOA_CRH (*((volatile uint32_t*)(GPIOA + 0x04)))
#define GPIOA_IDR (*(volatile uint32_t*)(GPIOA + 0x08))
#define GPIOA_ODR (*(volatile uint32_t*)(GPIOA + 0x0C))
#define GPIOA_BSRR (*(volatile uint32_t*)(GPIOA + 0x10))
#define GPIOA_BRR (*(volatile uint32_t*)(GPIOA + 0x14))

#define SYSTICK 0xE000E010
#define STK_CTRL (*((volatile uint32_t*)(SYSTICK)))
#define STK_LOAD (*((volatile uint32_t*)(SYSTICK + 0x4)))
#define STK_CALIB (*((volatile uint32_t*)(SYSTICK + 0xC)))

#define NVIC_BASE 0xE000E100
#define NVIC_ISER (*((volatile uint32_t*)(NVIC_BASE)))
#define NVIC_ICER (*((volatile uint32_t*)(NVIC_BASE + 0x80)))
#define NVIC_ISPR (*((volatile uint32_t*)(NVIC_BASE + 0x100)))
#define NVIC_ICPR (*((volatile uint32_t*)(NVIC_BASE + 0x180)))
#define NVIC_IABR (*((volatile uint32_t*)(NVIC_BASE + 0x200)))
#define NVIC_IPR (*((volatile uint32_t*)(NVIC_BASE + 0x300)))
#define NVIC_STIR (*((volatile uint32_t*)(NVIC_BASE + 0xE00)))

#define USART1 0x40013800
#define USART1_SR (*(volatile uint32_t*)(USART1))
#define USART1_DR (*(volatile uint32_t*)(USART1 + 0x04))
#define USART1_BRR (*(volatile uint32_t*)(USART1 + 0x08))
#define USART1_CR1 (*(volatile uint32_t*)(USART1 + 0x0C))
#define USART1_CR2 (*(volatile uint32_t*)(USART1 + 0x10))
#define USART1_CR3 (*(volatile uint32_t*)(USART1 + 0x14))
#define USART1_GTPR (*(volatile uint32_t*)(USART1 + 0x18))
#define BAUDRATE_REGVAL(baudrate) ( FCLK/16.0/(baudrate) )
#define BAUDRATE_FRACTION(baudrate) ( (int)((BAUDRATE_REGVAL((baudrate)) - (int)(BAUDRATE_REGVAL((baudrate))))*16) )
#define BAUDRATE_MANTISSA(baudrate) ( (int)(BAUDRATE_REGVAL((baudrate))) )

#define SPI1 0x40013000
#define SPI1_CR1 (*(volatile uint16_t*)(SPI1))
#define SPI1_CR2 (*(volatile uint16_t*)(SPI1 + 0x04))
#define SPI1_SR (*(volatile uint16_t*)(SPI1 + 0x08))
#define SPI1_DR (*(volatile uint16_t*)(SPI1 + 0x0C))
#define SPI1_CRCPR (*(volatile uint16_t*)(SPI1 + 0x10))
#define SPI1_RXCRCR (*(volatile uint16_t*)(SPI1 + 0x14))
#define SPI1_TXCRCR (*(volatile uint16_t*)(SPI1 + 0x18))

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
		delay_ms(500);
		GPIOC_BSRR |= (1 << 29);
		delay_ms(500);
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

void read_uart(char* buf, unsigned int len) {
  for(uint32_t i = 0; i < len - 1; i++) {
    buf[i] = receive_byte_uart();
    if(buf[i] == '\r'){
      buf[i + 1] = 0;
      return;
    }
  }
  buf[len - 1] = 0;
}
int main(void) {
	system_init(); // Initialize clocks, flash etc
	on_board_blink();
}

//int main(void) {
//  system_init();
//  RCC_APB2ENR |= ( 1 << 3 ) | (1 << 4); // Enable port B (3) and Port C (4) clocks
//  RCC_APB2ENR |= (1 << 12) | (1 << 2); // Enable SPI (12) and Port A (2) clocks
////  init_uart();
//  char msg[1024];
//  //fucked();
//  GPIOC_CRH |= 0b11 << 20;
//  GPIOC_CRH &= ~(0b11 << 22);
//
//  //Enable PA5 and PA7 as the SPI SCK and MOSI pins
//  GPIOA_CRL |= (0b10 << 22) | (0b11 << 20) | (0b10 << 30) | (0b11 << 28);
//  GPIOA_CRL &= ~(1 << 22);
//  GPIOA_CRL &= ~(1 << 30);
//  //Enable PA4 as a normal output
//  GPIOA_CRL |= (0b11 << 16);
//  GPIOA_CRL &= ~(0b11 << 18);
//  //Pull PA4 HIGH to keep AD9833 in idle mode
//  GPIOA_BSRR = (1 << 4);
//  // Set SPI1 to simplex transmit only mode, and set the data frame to 16 bits
//  SPI1_CR1 |= (1 << 15) | (1 << 14) | (1 << 11);
//  // Set SPI1's NSS to software mode and pull it high internally
//  SPI1_CR1 |= (1 << 9) | (1 << 8);
//  // Set SPI1 to master mode
//  SPI1_CR1 |= (1 << 2);
//  // Set the timing info (CPOL = 1, CPHA = 0)
//  SPI1_CR1 |= (1 << 1);
//  // Set the SPI1 clock rate to PCLK/256 = 281KHz, just for safety. Change once everything is confirmed
//  // to be working
//  SPI1_CR1 |= (0b111 << 3);
//  // Enable SPI1
//  SPI1_CR1 |= (1 << 6);
//    // GPIOC_BSRR |= 1 << 13;
//    // delay_ms(1);
//    // GPIOC_BSRR |= 1 << 29;
//    // delay_ms(1);
//    // Pull PA4 low to initiate comms with AD9833
//    GPIOA_BSRR = (1 << 20);
//    delay_ms(1); // Wait for 1ms to let the AD9833 initialize
//    ad9833_reset(&SPI1_DR, &SPI1_SR);
//    delay_ms(1);
//    ad9833_set_frequency(&SPI1_DR, &SPI1_SR, 0, 10000);
//    delay_ms(1);
//    // Pull PA4 high to return to idle
//    GPIOA_BSRR = (1 << 4);
//    delay_ms(1);
//    uint32_t freq = 10000;
//    uint8_t upslope = 1;
////  print_uart("Hello. This is the echo text. Whatever you enter will be echoed back\r\n> ");
//  while(1) {
////    read_uart(msg, 1024);
////    print_uart(msg);
////    print_uart("\r\n> ");
//    //if(upslope) {
//    //  GPIOA_BSRR = (1 << 20);
//    //  delay_ms(1);
//    //  ad9833_set_frequency(&SPI1_DR, &SPI1_SR, 0, freq);
//    //  delay_ms(1);
//    //  GPIOA_BSRR = (1 << 4);
//    //  freq += 100;
//    //  if(freq > 20000) {
//    //	upslope = 0;
//    //  }
//    //} else {
//    //  GPIOA_BSRR = (1 << 20);
//    //  delay_ms(1);
//    //  ad9833_set_frequency(&SPI1_DR, &SPI1_SR, 0, freq);
//    //  delay_ms(1);
//    //  GPIOA_BSRR = (1 << 4);
//    //  freq -= 100;
//    //  if(freq < 10000) {
//    //	upslope = 1;
//    //  }
//    //}
//    GPIOA_BSRR = (1 << 20);
//    delay_ms(1);
//    ad9833_change_mode(&SPI1_DR, &SPI1_SR, AD9833_MODE_SQUARE);
//    delay_ms(1);
//    GPIOA_BSRR = (1 << 4);
//    delay_ms(2000);
//    GPIOA_BSRR = (1 << 20);
//    delay_ms(1);
//    ad9833_change_mode(&SPI1_DR, &SPI1_SR, AD9833_MODE_TRI);
//    delay_ms(1);
//    GPIOA_BSRR = (1 << 4);
//    delay_ms(2000);
//    GPIOA_BSRR = (1 << 20);
//    delay_ms(1);
//    ad9833_change_mode(&SPI1_DR, &SPI1_SR, AD9833_MODE_SINE);
//    delay_ms(1);
//    GPIOA_BSRR = (1 << 4);
//    delay_ms(2000);
//  }
//  return 0;
//}
