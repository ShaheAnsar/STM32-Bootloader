#ifndef STM32F103_H
#define STM32F103_H

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
#define FLASH_KEYR (*(volatile uint32_t*)(FLASH + 0x04))
#define FLASH_OPTKEYR (*(volatile uint32_t*)(FLASH + 0x08))
#define FLASH_SR (*(volatile uint32_t*)(FLASH + 0x0C))
#define FLASH_CR (*(volatile uint32_t*)(FLASH + 0x10))
#define FLASH_AR (*(volatile uint32_t*)(FLASH + 0x14))
#define FLASH_OBR (*(volatile uint32_t*)(FLASH + 0x1C))
#define FLASH_WRPR (*(volatile uint32_t*)(FLASH + 0x20))

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

// The following are arrays. Index them to use them.
#define NVIC_BASE 0xE000E100
#define NVIC_ISER ((volatile uint32_t*)(NVIC_BASE))
#define NVIC_ICER ((volatile uint32_t*)(NVIC_BASE + 0x80))
#define NVIC_ISPR ((volatile uint32_t*)(NVIC_BASE + 0x100))
#define NVIC_ICPR ((volatile uint32_t*)(NVIC_BASE + 0x180))
#define NVIC_IABR ((volatile uint32_t*)(NVIC_BASE + 0x200))
#define NVIC_IPR ((volatile uint32_t*)(NVIC_BASE + 0x300))
#define NVIC_STIR ((volatile uint32_t*)(NVIC_BASE + 0xE00))
//----------------------------

#define SCB_BASE 0xE000E008
#define SCB_VTOR (*((volatile uint32_t*)(SCB_BASE + 0x08)))

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

extern void __set_msp(void* sp);
#endif
