DEVICE  = atmega328p
F_CPU = 16000000

CFLAGS  = -std=c99 -mmcu=$(DEVICE) -DF_CPU=$(F_CPU) -O2

AVRDUDEFLAGS = -c arduino -p $(DEVICE) -P/dev/tty.usbmodem00000001

SREC    = srec_cat
CC      = avr-gcc
SIZE    = avr-size
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
AVRDUDE = avrdude

hex: replicator.hex

clean:
	rm -f replicator.hex replicator.elf pic_code.h pic_conf.h

pic_code.h: pic.hex
	#TODO: error handling...
	$(SREC) $< -intel -crop 0x00000 0x10000 -o -carray pic_code -dataonly | sed 's/\[\]/\[\] PROGMEM/' > $@
pic_conf.h: pic.hex
	$(SREC) $< -intel -crop 0x1000E 0x10012 -offset -0x1000E -o $@ -carray pic_conf -dataonly

replicator.elf: replicator.c pic_code.h pic_conf.h
	$(CC) $(CFLAGS) replicator.c -o replicator.elf

replicator.hex: replicator.elf
	$(OBJCOPY) -j .text -j .data -O ihex replicator.elf replicator.hex
	$(SIZE) replicator.hex

flash: replicator.hex
	$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:$<

.PHONY: hex,clean,flash

