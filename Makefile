TARGET=AD9833

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SRC=$(shell fd "\\.c$$")
ASRC=$(shell fd "\\.s$$")
OBJ=$(foreach i,$(SRC),$(i:%.c=%.o)) $(foreach i,$(ASRC),$(i:%.s=%_s.o))
CFLAGS=-Wall -Wextra -pedantic -std=c11 -Os -nostdlib -mcpu=cortex-m3 -pipe -T linker.ld
LDFLAGS=


.PHONY: all test clean

test:
	echo $(SRC)
	echo $(OBJ)

clean:
	-rm $(OBJ)
	-rm $(TARGET).out $(TARGET).bin

%_s.o: %.s
	$(CC) $(CFLAGS) -c -o $@ $<

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<


$(TARGET).out: $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $(TARGET).out $(LDFLAGS)

$(TARGET).bin: $(TARGET).out
	$(OBJCOPY) -O binary $< $@

all: $(TARGET).bin
