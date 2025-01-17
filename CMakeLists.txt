cmake_minimum_required(VERSION 3.30)
project(stm32-bootloader C ASM)
find_file(LINKER_FILE linker.ld PATHS "${PROJECT_SOURCE_DIR}" NO_CMAKE_FIND_ROOT_PATH)
find_file(APP_LINKER_FILE linker_app.ld PATHS "${PROJECT_SOURCE_DIR}" NO_CMAKE_FIND_ROOT_PATH)
message(STATUS "Linker search path: ${PROJECT_SOURCE_DIR}")
set(CMAKE_C_FLAGS "-std=c11 -Wall -Wextra -pedantic -Os -nostdlib -mcpu=cortex-m3 -pipe  -fno-strict-aliasing")
set(CMAKE_LD_FLAGS "-std=c11 -Wall -Wextra -pedantic -Os -nostdlib -mcpu=cortex-m3 -pipe -fno-strict-aliasing")
set(CMAKE_AS_FLAGS "-std=c11 -Wall -Wextra -pedantic -Os -nostdlib -mcpu=cortex-m3 -pipe -fno-strict-aliasing")
set(SOURCES main.c boot.s utils.c)
set(outfile_prefix bootloader)
add_compile_definitions(LOG=${LOG})
add_executable(${outfile_prefix}_exe ${SOURCES})
add_executable(blink_exe boot.s blink.c)
target_link_options(${outfile_prefix}_exe PRIVATE -T ${LINKER_FILE})
target_link_options(blink_exe PRIVATE -T ${APP_LINKER_FILE})
add_custom_command(TARGET ${outfile_prefix}_exe POST_BUILD
				   COMMAND arm-none-eabi-objcopy -Obinary ${outfile_prefix}_exe ${outfile_prefix}.bin
			   	   COMMENT "Converting to binary")
add_custom_command(TARGET ${outfile_prefix}_exe POST_BUILD
				   COMMAND arm-none-eabi-objcopy -Oihex ${outfile_prefix}_exe ${outfile_prefix}.ihex
			   	   COMMENT "Converting to ihex")
add_custom_command(TARGET ${outfile_prefix}_exe POST_BUILD
				   COMMAND arm-none-eabi-size -Ax ${outfile_prefix}_exe 
			   	   COMMENT "Printing size...")
add_custom_command(TARGET blink_exe POST_BUILD
				   COMMAND arm-none-eabi-objcopy -Obinary blink_exe blink.bin
				   COMMENT "Converting to binary (Blink)")
add_custom_command(TARGET blink_exe POST_BUILD
				   COMMAND arm-none-eabi-objcopy -Oihex blink_exe blink.ihex
				   COMMENT "Converting to ihex (Blink)")
add_custom_command(TARGET blink_exe POST_BUILD
				   COMMAND arm-none-eabi-size -Ax blink_exe 
				   COMMENT "Printing size... (Blink)")
