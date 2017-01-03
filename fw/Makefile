##############################################################################
#  ___ _____ _       _                         _ _         _                 #
# | _ \_   _| |  ___(_)_ _  ___   _ _ ___ _ __| (_)__ __ _| |_ ___ _ _       #
# |  _/ | | | |_|___| | ' \/ _ \ | '_/ -_) '_ \ | / _/ _` |  _/ _ \ '_|      #
# |_|   |_| |____|  |_|_||_\___/ |_| \___| .__/_|_\__\__,_|\__\___/_|        #
#                                      |_|                                   #
#                                                                            #
# This file is part of PTL-ino replicator                                    #
# Copyright (C) 2016, 2017  magnustron, Post Tenebras Lab                    #  
# This program is free software; you can redistribute it and/or modify       #
# it under the terms of the GNU General Public License as published by       #
# the Free Software Foundation; either version 3 of the License, or          #
# (at your option) any later version.                                        #
# This program is distributed in the hope that it will be useful,            #
# but WITHOUT ANY WARRANTY; without even the implied warranty of             #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              #
# GNU General Public License for more details.                               #
# You should have received a copy of the GNU General Public License          #
# along with this program; if not, write to the Free Software Foundation,    #
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA          #
##############################################################################

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

