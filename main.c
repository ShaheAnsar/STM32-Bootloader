#include <stdint.h>
#include <stddef.h>

#define FCLK 72000000UL

#include "stm32f103.h"

#define LOG 1

// Redundant bootloader
#define BOOTLOADER_BASE 0x8000000
#define BOOTLOADER_LENGTH 0x2000
#define BOOTLOADER_STORAGE_OFFSET 0x100
#define BOOTLOADER_STORAGE ((uint16_t*)(BOOTLOADER_BASE + BOOTLOADER_LENGTH - BOOTLOADER_STORAGE_OFFSET)) //Pointer to a 256 byte area used for perma storage
#define APPLICATION_BASE (BOOTLOADER_BASE + BOOTLOADER_LENGTH)
#define APPLICATION_PTR ((uint16_t*)APPLICATION_BASE)
#define APPLICATION_MAX_LENGTH 0x7000
#define BACKUP_BASE (APPLICATION_BASE + APPLICATION_MAX_LENGTH)
#define BACKUP_PTR ((uint16_t*)BACKUP_BASE)
#define BACKUP_MAX_LENGTH 0x7000
#define APPLICATION_PAGES 28
#define BACKUP_PAGES 28

// Bootloader perma storage indices
#define APPLICATION_VALID_INDEX 0 // The value at this index is 0 if the application is invalid.
#define BACKUP_VALID_INDEX 1 // The value at this index is 0 if the backup is invalid.
#define APPLICATION_CRC_INDEX 2
#define BACKUP_CRC_INDEX 3
#define APPLICATION_DATA_LENGTH_INDEX 4
#define BACKUP_DATA_LENGTH_INDEX 5

// Pre-defined Constants (Do NOT change)
#define FPEC_KEY1 0x45670123
#define FPEC_KEY2 0xCDEF89AB
#define BYTES_PER_PAGE 1024

// Implementation constants
#define BOOTLOADER_MAGIC_START 0x27 // Wait till we receive this to start programming
#define BOOTLOADER_MAGIC_RESET 0x17 // Used to invalidate both the application and backup
#define BOOTLOADER_METADATA_PACKET_HEADER 0x12 // Used to signal the start of the metadata packet. The header is followed by a 2 byte size.
#define BOOTLOADER_COMMAND_PAUSE "CMD:PAUSE" // Wait 100ms to flush the flash. Sent by the MCU to the program on the PC.
#define TRANSFER_PAGES 1 // These many pages are read and then written to flash


void itoa(int a, char* buf, int n) {
	if(a == 0) {
		if(n >= 2){
			buf[0] = '0';
			buf[1] = 0; 
		}
		return;
	}
	int i = 0;
	for(; i < n - 1; i++) {
		if(a == 0)
			break;
		buf[n - 1 - i] = '0' + (a % 10);
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

void unlock_fpec() {
	FLASH_KEYR = FPEC_KEY1;
	FLASH_KEYR = FPEC_KEY2;
}

int wait_for_magic(int timeout_ms) {
  for (; timeout_ms >= 1; timeout_ms--) {
    // Start timer
    STK_CTRL = 0;
    STK_CTRL |= 1;
    // Check whether the counter has counted down to 0
	// If the byte received is the magic bye, stop the timer and return 1
    while (!(STK_CTRL & (1 << 16))){
		if((USART1_SR & (1 << 5))){
			if(USART1_DR == BOOTLOADER_MAGIC_START){
				STK_CTRL &= ~(1);
				return 1;
			}
		}
	}
      
    // Stop timer
    STK_CTRL &= ~(1);
  }
  // If the byte was not received within the given time, return 0
  return 0;
}


void write_program() {
}

void erase_region(char* ptr, int pages) {
	// Make sure ptr is page aligned (0x400 aligned)
	for(int i = 0; i < pages; i++) {
		while(FLASH_SR & 1); // Wait till busy flag is clear
		FLASH_AR = (uint32_t)ptr + BYTES_PER_PAGE * i;
		FLASH_CR |= (1 << 1); // Configure for page erase
		FLASH_CR |= (1 << 6); // Start the erase procedure
	}
}

void write_flash(uint16_t* base_ptr, uint16_t* buf, int nhwords) {
	// Make sure pointer is half word (16 bit) aligned
	for(int i = 0; i < nhwords; i++){
		while(FLASH_SR & 1); // Wait till busy flag is clear
		FLASH_CR |= 1; // Set the programming bit
		base_ptr[i] = buf[i];
		while(FLASH_SR & 1); // Wait till busy flag is clear
		if(buf[i] != base_ptr[i]) {
			kill("Write failed! Quitting....\n");
		}
	}
	while(FLASH_SR & 1); // Wait till busy flag is clear
	FLASH_CR &= ~(1); // Clear the programming bit
	
}

void transfer_to_backup() {
	// Erase the backup region
	debug_log("\nErasing the backup regions...");
	erase_region((char*)BACKUP_PTR, BACKUP_PAGES);
	debug_log("\nTransferring data to the backup region");
	uint16_t buf[TRANSFER_PAGES * BYTES_PER_PAGE / 2];
	for(int i = 0;;){
		int j = 0;
		// Copy data over to the buffer
		for(; ( j < TRANSFER_PAGES * BYTES_PER_PAGE / 2) &&
				(i + j < ( BOOTLOADER_STORAGE[APPLICATION_DATA_LENGTH_INDEX]/2 + ( BOOTLOADER_STORAGE[APPLICATION_DATA_LENGTH_INDEX] % 2 ))); j++){
			buf[j] = APPLICATION_PTR[i + j];
		}
		// Flush all data in the buffer
		write_flash(BACKUP_PTR + i, buf, j);
		debug_log("\nWrote halfwords: ");
		char num[10];
		itoa(j, num, 10);
		debug_log(num);
		// Re-index to account for all the written data
		i += j;
		itoa(i, num, 10);
		debug_log("\nTotal halfwords written: ");
		debug_log(num);
	}
	// Write the backup metadata
	uint16_t backup_length = BOOTLOADER_STORAGE[APPLICATION_DATA_LENGTH_INDEX];
	uint16_t backup_valid = 1;
	write_flash(BOOTLOADER_STORAGE + BACKUP_DATA_LENGTH_INDEX, &backup_length, 1);
	write_flash(BOOTLOADER_STORAGE + BACKUP_VALID_INDEX, &backup_valid, 1);
}

uint16_t decode_metadata() {
	// Wait for the metadata header
	char r = 0;
	while(r != BOOTLOADER_METADATA_PACKET_HEADER){
		r = receive_byte_uart();
	}
	// Get the two bytes of size. Little endian	
	uint16_t size = 0;
	char* win = (char*)&size;
	read_uart(win, 2);
	return size;
}

void update_firmware() {
	// If a valid firmware already exists, transfer it to backup
	if(!BOOTLOADER_STORAGE[APPLICATION_VALID_INDEX]) {
		debug_log("\nValid firmware found! Transferring to backup region...");
		transfer_to_backup();
	}
	// Invalidate the current application
	uint16_t temp = 0;
	write_flash(BOOTLOADER_STORAGE + APPLICATION_VALID_INDEX, &temp, 1);
	// Erase the application region
	debug_log("\nErasing application region....");
	erase_region((char*)APPLICATION_PTR, APPLICATION_PAGES);
	debug_log("\nWaiting for magic code...");
	debug_log("\nGot the magic code, waiting for metadata...");
	uint16_t firm_size = decode_metadata();
	char num[10];
	itoa(firm_size, num, 10);
	debug_log("\nSize of firmware: ");
	debug_log(num);
	// Read and upload in chunks
	uint16_t buf[BYTES_PER_PAGE/2];
	char* cbuf = (char*)buf;
	int received_bytes = 0;
	debug_log("\nReceiving data and flushing in pages....");
	while(1){
		if(( firm_size - received_bytes ) > BYTES_PER_PAGE) {
			debug_log("\nFlashing one page");
			read_uart(cbuf, BYTES_PER_PAGE);
			write_flash(APPLICATION_PTR, buf, BYTES_PER_PAGE/2);
		} else {
			debug_log("\nFlashing less than one page");
			read_uart(cbuf, firm_size - received_bytes);
			write_flash(APPLICATION_PTR, buf, ( (firm_size - received_bytes)/2 ) + ((firm_size - received_bytes) % 2));
		}
	}
	debug_log("\nFirmware download completed!");
	temp = 1;
	write_flash(BOOTLOADER_STORAGE + APPLICATION_VALID_INDEX, &temp, 1);
	write_flash(BOOTLOADER_STORAGE + APPLICATION_DATA_LENGTH_INDEX, &firm_size, 1);
}
void deinit_system() {
	// Disable the systick
	STK_CTRL = 0;
	STK_LOAD = 0;
	// Reset GPIO and UART
	// Reset clocks
}

void boot() {
	// If the main firmware is valid, boot from it
	if(BOOTLOADER_STORAGE[APPLICATION_VALID_INDEX]) {

	}
	// If the backup firwmare is valid, boot from it
	else if(BOOTLOADER_STORAGE[APPLICATION_VALID_INDEX]) {

	}
	// If neither are valid
	else {
		print_uart("Both firmwares are invalid!!!! Hanging indefinitely!!!");
		while(1);
	}
}

int main(void) {
	system_init(); // Initialize clocks, flash etc
	init_uart();
	while(1) {
		int program_flash = 0;
		int read_flash = 0;
		while(1) {
			char a = receive_byte_uart();
			print_uart("Received byte: ");
			send_uart(a);
			print_uart("\r\n");
			if(a == 'p'){
				program_flash = 1;
				break;
			} else if(a == 'r'){
				read_flash = 1;
				break;
			}
		}
		print_uart("Hello\r\n");
		if(program_flash){
			print_uart("Unlocking fpec...\r\n");
			unlock_fpec();
			print_uart("Unlocked fpec!\r\n");
			print_uart("Erasing region...\r\n");
			uint16_t* flash_addr = (uint16_t*)0x08007C00; //31st page in flash
			erase_region((char*)flash_addr, 1);	
			print_uart("Writing Hello World to the flash....\r\n");
			char msg[40] = "Hello World!";
			write_flash(flash_addr, (uint16_t*)msg, 7); //13 bytes total, so we write 14 bytes (7 half words)
			print_uart("Hello world written!\r\n");
		}
		if(read_flash){
			print_uart("Reading from 0x08007C00 in flash...\r\n");
			print_uart((char*)0x08007C00);
			print_uart("Successful?\r\n");
		}
	} 
	int r = wait_for_magic(3000);
	if(r == 1) {
		debug_log("Got magic! YAY");
		on_board_blink(100);
	} else {
		debug_log("Timed out...");
		on_board_blink(2000);
	}
	return 0;
}

