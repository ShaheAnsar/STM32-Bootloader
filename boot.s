	.syntax unified
	.thumb
	.cpu cortex-m3
	.fpu softvfp

	.equ stack_bottom,0x20005000 	

	.section .text
	.global start
  .global Reset_Handler
start:	
Reset_Handler:
	/* Copy the data segment initializers from flash to SRAM */
	movs r1, #0
	b LoopCopyDataInit
	
CopyDataInit:
	ldr r3, =_sidata
	ldr r3, [r3, r1]
	str r3, [r0, r1]
	adds r1, r1, #4
	
LoopCopyDataInit:
	ldr r0, =_sdata
	ldr r3, =_edata
	adds r2, r0, r1
	cmp r2, r3
	bcc CopyDataInit
	ldr r2, =_sbss
	b LoopFillZerobss
	/* Zero fill the bss segment. */
FillZerobss:
	movs r3, #0
	str r3, [r2], #4
	
LoopFillZerobss:
	ldr r3, = _ebss
	cmp r2, r3
	bcc FillZerobss
	
	/* Call static constructors */
	/*bl __libc_init_array*/
/* Call the application's entry point.*/
	bl main
	bx lr

.global __set_msp
__set_msp:
	mov r13, r1
	bx lr
.global Default_Handler
Default_Handler:
	bl fucked
Infinite_Loop:
	b Infinite_Loop
	

	.section .vector_table,"a"
v_table:
	.word stack_bottom 	// The top of the stack
	.word Reset_Handler + 1
	.word NMI_Handler
	.word HardFault_Handler
	.word MemManage_Handler
	.word BusFault_Handler
	.word UsageFault_Handler
	.word 0
	.word 0
	.word 0
	.word 0
	.word SVC_Handler
	.word DebugMon_Handler
	.word 0
	.word PendSV_Handler
	.word SysTick_Handler
	.word WWDG_IRQHandler
	.word PVD_IRQHandler
	.word TAMPER_IRQHandler
	.word RTC_IRQHandler
	.word FLASH_IRQHandler
	.word RCC_IRQHandler
	.word EXTI0_IRQHandler
	.word EXTI1_IRQHandler
	.word EXTI2_IRQHandler
	.word EXTI3_IRQHandler
	.word EXTI4_IRQHandler
	.word DMA1_Channel1_IRQHandler
	.word DMA1_Channel2_IRQHandler
	.word DMA1_Channel3_IRQHandler
	.word DMA1_Channel4_IRQHandler
	.word DMA1_Channel5_IRQHandler
	.word DMA1_Channel6_IRQHandler
	.word DMA1_Channel7_IRQHandler
	.word ADC1_IRQHandler
	.word 0
	.word 0
	.word 0
	.word 0
	.word EXTI9_5_IRQHandler
	.word 0
	.word 0
	.word 0
	.word 0
	.word TIM2_IRQHandler
	.word TIM3_IRQHandler
	.word TIM4_IRQHandler
	.word I2C1_EV_IRQHandler
	.word I2C1_ER_IRQHandler
	.word I2C2_EV_IRQHandler
	.word I2C2_ER_IRQHandler
	.word SPI1_IRQHandler
	.word SPI2_IRQHandler
	.word USART1_IRQHandler
	.word USART2_IRQHandler
	.word USART3_IRQHandler
	.word EXTI15_10_IRQHandler
	.word RTC_Alarm_IRQHandler
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	.word 0
	
	

	.weak NMI_Handler
	.thumb_set NMI_Handler,Default_Handler
	
	.weak HardFault_Handler
	.thumb_set HardFault_Handler,Default_Handler
	
	.weak MemManage_Handler
	.thumb_set MemManage_Handler,Default_Handler
	
	.weak BusFault_Handler
	.thumb_set BusFault_Handler,Default_Handler
	
	.weak UsageFault_Handler
	.thumb_set UsageFault_Handler,Default_Handler
	
	.weak SVC_Handler
	.thumb_set SVC_Handler,Default_Handler
	
	.weak DebugMon_Handler
	.thumb_set DebugMon_Handler,Default_Handler
	
	.weak PendSV_Handler
	.thumb_set PendSV_Handler,Default_Handler
	
	.weak SysTick_Handler
	.thumb_set SysTick_Handler,Default_Handler
	
	.weak WWDG_IRQHandler
	.thumb_set WWDG_IRQHandler,Default_Handler
	
	.weak PVD_IRQHandler
	.thumb_set PVD_IRQHandler,Default_Handler
	
	.weak TAMPER_IRQHandler
	.thumb_set TAMPER_IRQHandler,Default_Handler
	
	.weak RTC_IRQHandler
	.thumb_set RTC_IRQHandler,Default_Handler
	
	.weak FLASH_IRQHandler
	.thumb_set FLASH_IRQHandler,Default_Handler
	
	.weak RCC_IRQHandler
	.thumb_set RCC_IRQHandler,Default_Handler
	
	.weak EXTI0_IRQHandler
	.thumb_set EXTI0_IRQHandler,Default_Handler
	
	.weak EXTI1_IRQHandler
	.thumb_set EXTI1_IRQHandler,Default_Handler
	
	.weak EXTI2_IRQHandler
	.thumb_set EXTI2_IRQHandler,Default_Handler
	
	.weak EXTI3_IRQHandler
	.thumb_set EXTI3_IRQHandler,Default_Handler
	
	.weak EXTI4_IRQHandler
	.thumb_set EXTI4_IRQHandler,Default_Handler
	
	.weak DMA1_Channel1_IRQHandler
	.thumb_set DMA1_Channel1_IRQHandler,Default_Handler
	
	.weak DMA1_Channel2_IRQHandler
	.thumb_set DMA1_Channel2_IRQHandler,Default_Handler
	
	.weak DMA1_Channel3_IRQHandler
	.thumb_set DMA1_Channel3_IRQHandler,Default_Handler
	
	.weak DMA1_Channel4_IRQHandler
	.thumb_set DMA1_Channel4_IRQHandler,Default_Handler
	
	.weak DMA1_Channel5_IRQHandler
	.thumb_set DMA1_Channel5_IRQHandler,Default_Handler
	
	.weak DMA1_Channel6_IRQHandler
	.thumb_set DMA1_Channel6_IRQHandler,Default_Handler
	
	.weak DMA1_Channel7_IRQHandler
	.thumb_set DMA1_Channel7_IRQHandler,Default_Handler
	
	.weak  ADC1_IRQHandler
	.thumb_set ADC1_IRQHandler,Default_Handler
	
	.weak EXTI9_5_IRQHandler
	.thumb_set EXTI9_5_IRQHandler,Default_Handler
	
	.weak TIM2_IRQHandler
	.thumb_set TIM2_IRQHandler,Default_Handler
	
	.weak TIM3_IRQHandler
	.thumb_set TIM3_IRQHandler,Default_Handler
	
	.weak TIM4_IRQHandler
	.thumb_set TIM4_IRQHandler,Default_Handler
	
	.weak I2C1_EV_IRQHandler
	.thumb_set I2C1_EV_IRQHandler,Default_Handler
	
	.weak I2C1_ER_IRQHandler
	.thumb_set I2C1_ER_IRQHandler,Default_Handler
	
	.weak I2C2_EV_IRQHandler
	.thumb_set I2C2_EV_IRQHandler,Default_Handler
	
	.weak I2C2_ER_IRQHandler
	.thumb_set I2C2_ER_IRQHandler,Default_Handler
	
	.weak SPI1_IRQHandler
	.thumb_set SPI1_IRQHandler,Default_Handler
	
	.weak SPI2_IRQHandler
	.thumb_set SPI2_IRQHandler,Default_Handler
	
	.weak USART1_IRQHandler
	.thumb_set USART1_IRQHandler,Default_Handler
	
	.weak USART2_IRQHandler
	.thumb_set USART2_IRQHandler,Default_Handler
	
	.weak USART3_IRQHandler
	.thumb_set USART3_IRQHandler,Default_Handler
	
	.weak EXTI15_10_IRQHandler
	.thumb_set EXTI15_10_IRQHandler,Default_Handler
	
	.weak RTC_Alarm_IRQHandler
	.thumb_set RTC_Alarm_IRQHandler,Default_Handler
	
	


