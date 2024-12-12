cmake_minimum_required(VERSION 3.30)
project(stm32-bootloader C ASM)
find_file(LINKER_FILE linker.ld PATHS "${PROJECT_SOURCE_DIR}" NO_CMAKE_FIND_ROOT_PATH)
message(STATUS "Linker search path: ${PROJECT_SOURCE_DIR}")
set(CMAKE_C_FLAGS "-std=c11 -Wall -Wextra -pedantic -Os -nostdlib -mcpu=cortex-m3 -pipe -T ${LINKER_FILE} -fno-strict-aliasing")
set(CMAKE_LD_FLAGS "-std=c11 -Wall -Wextra -pedantic -Os -nostdlib -mcpu=cortex-m3 -pipe -T ${LINKER_FILE} -fno-strict-aliasing")
set(CMAKE_AS_FLAGS "-std=c11 -Wall -Wextra -pedantic -Os -nostdlib -mcpu=cortex-m3 -pipe -T ${LINKER_FILE} -fno-strict-aliasing")
set(SOURCES main.c boot.s utils.c)
set(outfile_prefix bootloader)
add_compile_definitions(LOG=${LOG})
add_executable(${outfile_prefix}_exe ${SOURCES})
add_custom_command(TARGET ${outfile_prefix}_exe POST_BUILD
				   COMMAND arm-none-eabi-objcopy -Obinary ${outfile_prefix}_exe ${outfile_prefix}.bin
			   	   COMMENT "Converting to binary")
add_custom_command(TARGET ${outfile_prefix}_exe POST_BUILD
				   COMMAND arm-none-eabi-objcopy -Oihex ${outfile_prefix}_exe ${outfile_prefix}.ihex
			   	   COMMENT "Converting to ihex")
add_custom_command(TARGET ${outfile_prefix}_exe POST_BUILD
				   COMMAND arm-none-eabi-size -Ax ${outfile_prefix}_exe 
			   	   COMMENT "Printing size...")
