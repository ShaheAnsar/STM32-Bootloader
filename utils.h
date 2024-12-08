#ifndef STM32_F103_UTILS_H
#define STM32_F103_UTILS_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>



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

extern void itoa(int a, char* buf, int n); 


extern void fucked(); 

extern void heartbeat(); 


extern void systick_init(); 

extern void delay_ms(unsigned int ms); 

extern void on_board_blink(int delay);


extern void system_init(); 

extern void init_uart(); 

extern void send_uart(char a); 


extern char receive_byte_uart(); 

extern void print_uart(char* msg); 
extern void iprint_uart(int num);

extern void read_uart(char* buf, unsigned int len); 



extern void debug_log(char* msg); 

extern void kill(char* msg); 
#endif
