#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define FCLK 72000000UL
#define LOG 1

#include "stm32f103.h"
#include "utils.h"


void unlock_fpec() {
	FLASH_KEYR = FPEC_KEY1;
	FLASH_KEYR = FPEC_KEY2;
}

int wait_for_magic(int timeout_ms) {
  for (; timeout_ms >= 1; timeout_ms--) {
    // Start timer
    STK_CTRL = 0;
    STK_CTRL |= 1;
	char in_buf[6];
    // Check whether the counter has counted down to 0
	// If the byte received is the magic bye, stop the timer and return 1
    while (!(STK_CTRL & (1 << 16))){
		//if((USART1_SR & (1 << 5))){
		//	if(USART1_DR == BOOTLOADER_MAGIC_START){
		//		STK_CTRL &= ~(1);
		//		return 1;
		//	}
		//}
		read_uart(in_buf, 6);
		if(strncmp(in_buf, "start", 5) == 0) {
			//stop timer
			STK_CTRL &= ~(1);
			return 1;
		}
	}
      
    // Stop timer
    STK_CTRL &= ~(1);
  }
  // If the byte was not received within the given time, return 0
  return 0;
}

// Struct used to store the state of an ihex decoding


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




int convert_char_to_num(char c){
	switch(c){
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			return c - '0';
		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
		case 'F':
			return 10 - 'A' + c;
		case 'a':
		case 'b':
		case 'c':
		case 'd':
		case 'e':
		case 'f':
			return 10 - 'a' + c;
		default:
			kill("Invalid num conversion");
			return -1;

	}
}

int pow16(int n) {
	int ret = 1;
	for(int i = 0; i < n; i++) {
		ret *= 0x10;
	}
	return ret;
}

int atoi(char* buf, int len){ // Works only for hex
	int num = 0;
	for(int i = 0; i < len; i++) {
		num += pow16(len - 1 - i)*convert_char_to_num(buf[i]);
	}
	return num;
}

int hex__get_byte_count() {
	char buf[2]; // Buffer to hold incoming messages
	int byte_count = 0;
	// Get the byte count
	for(int i = 0; i < 2; i++) {
		buf[i] = receive_byte_uart();
	}
	byte_count = 0x10*convert_char_to_num(buf[0]) + convert_char_to_num(buf[1]);
	return byte_count;
}

int hex__get_address() {
	char buf[4];
	for(int i = 0; i < 4; i++) {
		buf[i] = receive_byte_uart();
	}
	return atoi(buf, 4);
}

typedef enum IntelHexRecordType{
	IHexRTypeData, //Supported
	IHexRTypeEOF, // Supported
	IHexRTypeESA, //Extended Segment Address - Unhandled
	IHexRTypeSSA, //Start segment address - Unhandled
	IHexRTypeELA, //Extended Linear Address - Supported
	IHexRTypeSLA, //Start Linear Address - the MCU starts here - Supported
} IntelHexRecordType;

typedef struct ihex_info_t{
	size_t size; // Byte count
	IntelHexRecordType rtype;
	char* databuffer; // Holds all the data
	size_t buffer_size;
	uint32_t address; // Holds the address to which this data must be copied
	uint32_t base_address;
	uint32_t start_address; // Entry point; Start execution here
}ihex_info_it;

void init_ihex_info(ihex_info_it* info) {
	info->size = 0;
	info->rtype = 0;
	info->databuffer = NULL;
	info->buffer_size = 0;
	info->address = 0;
	info->base_address = 0;
	info->start_address = 0;
}

IntelHexRecordType hex__get_record_type() {
	char buf[2];
	for(int i = 0; i < 2; i++) {
		buf[i] = receive_byte_uart();
	}
	int rtype = atoi(buf, 2);
	return rtype;
}

void hex__get_data(char* buf, int len) {
	char byte[2];
	for(int i = 0; i < len; i++) {
		byte[0] = receive_byte_uart();
		byte[1] = receive_byte_uart();
		buf[i] = atoi(byte, 2);
	}
}

int hex__get_checksum() {
	char buf[2];
	for(int i = 0; i < 2; i++) {
		buf[i] = receive_byte_uart();
	}
	return atoi(buf, 2);
}

int hex_decode_data(ihex_info_it* info){
	size_t byte_count = info->size;
	char* databuf = info->databuffer;
	if(info->buffer_size < byte_count) {
		return -1;
	}
	hex__get_data(databuf, byte_count);
	char checksum = hex__get_checksum();
	return 0;
}

int hex_decode_extended_linear_address(ihex_info_it* info) {
	size_t byte_count = info->size;
	char buffer[2];
	hex__get_data(buffer, 2);
	info->base_address = ((size_t)buffer[0] << 24 ) | ((size_t)buffer[1] << 16); // Set the upper 16 bits of the 32 bit address
	char checksum = hex__get_checksum();
	return 0;
}

int hex_decode_start_linear_address(ihex_info_it* info) {
	size_t byte_count = info->size;
	char buffer[4];
	hex__get_data(buffer, 4);
	info->start_address = ((size_t)buffer[0] << 24 ) | ((size_t)buffer[1] << 16) | ((size_t)buffer[2] << 8) | ((size_t)buffer[3]);
	char checksum = hex__get_checksum();
	return 0;
}

int hex_decode(ihex_info_it* info) { // Get a single ihex record
	while(1) {
		char c = 0;
		c = receive_byte_uart();
		if(c == ':'){ //Is a valid ihex record
			int byte_count = hex__get_byte_count();
			info->size = byte_count;
			uint32_t addr = hex__get_address();
			info->address = addr;
			IntelHexRecordType rtype = hex__get_record_type();
			info->rtype = rtype;
			switch(rtype) {
				case IHexRTypeData:
					return hex_decode_data(info);
				case IHexRTypeEOF:
					//Verify that that is the case by looking at the checksum
					{
						int checksum = hex__get_checksum();
						if(checksum == 0xff)
							return 0;
						else
							kill("Oh no! EOF is invalid");
					}
				case IHexRTypeESA:
				case IHexRTypeSSA:
					kill("Unimplemented!");
				case IHexRTypeELA:
					return hex_decode_extended_linear_address(info);
				case IHexRTypeSLA:
					return hex_decode_start_linear_address(info);


			}
			//char* databuf = info->databuffer;
			//if(info->buffer_size < byte_count) {
			//	return -1;
			//}
			//hex__get_data(databuf, byte_count);
			//char checksum = hex__get_checksum();
			//return 0;
		}
	}
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

#define IHEX_BUFSIZE BYTES_PER_PAGE
char temp_buffer[IHEX_BUFSIZE]; //Buffer to store info before flushing to flash
int main(void) {
	system_init(); // Initialize clocks, flash etc
	init_uart();
	ihex_info_it ihex_info;
	init_ihex_info(&ihex_info);
	while(1){
		char ihex_buffer[IHEX_BUFSIZE];
		ihex_info.databuffer = ihex_buffer;
		ihex_info.buffer_size = IHEX_BUFSIZE;
		int err = hex_decode(&ihex_info);
		if(err != 0) {
			kill("Error!!!");
			while(1);
		}
		print_uart("Received hex record!");
		print_uart("\r\nAddress: ");
		iprint_uart(ihex_info.address);
		print_uart("\r\nBase Address: ");
		iprint_uart(ihex_info.base_address);
		print_uart("\r\nEffective Address: ");
		iprint_uart(ihex_info.base_address + ihex_info.address);
		print_uart("\r\nByte Count: ");
		iprint_uart(ihex_info.size);
		print_uart("\r\nRecord Type: ");
		iprint_uart(ihex_info.rtype);
		print_uart("\r\nData:\r\n");
		for(int i = 0; (i < ihex_info.size) && ( i < ihex_info.buffer_size ); i++) {
			iprint_uart(i);
			print_uart(": ");
			iprint_uart(ihex_info.databuffer[i]);
			print_uart("\r\n");
		}
	}

	//if(1) {
	//	debug_log("Got magic! YAY");
	//	on_board_blink(100);
	//} else {
	//	debug_log("Timed out...");
	//	on_board_blink(2000);
	//}
	return 0;
}

