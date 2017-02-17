/*****************************************************************************
*  ___ _____ _       _                         _ _         _                 *
* | _ \_   _| |  ___(_)_ _  ___   _ _ ___ _ __| (_)__ __ _| |_ ___ _ _       *
* |  _/ | | | |_|___| | ' \/ _ \ | '_/ -_) '_ \ | / _/ _` |  _/ _ \ '_|      *
* |_|   |_| |____|  |_|_||_\___/ |_| \___| .__/_|_\__\__,_|\__\___/_|        *
*                                      |_|                                   *
*                                                                            *
* Copyright (C) 2016, 2017  magnustron, Post Tenebras Lab                    *  
* This program is free software; you can redistribute it and/or modify       *
* it under the terms of the GNU General Public License as published by       *
* the Free Software Foundation; either version 3 of the License, or          *
* (at your option) any later version.                                        *
* This program is distributed in the hope that it will be useful,            *
* but WITHOUT ANY WARRANTY; without even the implied warranty of             *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
* GNU General Public License for more details.                               *
* You should have received a copy of the GNU General Public License          *
* along with this program; if not, write to the Free Software Foundation,    *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA          *
*****************************************************************************/

/*****************************************************************************
*  _                                                                         *
* /  _ ._ ._  _  __|_o _ ._  _|_ _.|_ | _                                    *
* \_(_)| || |(/_(_ |_|(_)| |  |_(_||_)|(/_                                   *
*                                                                            *
* HOST       TARGET                                                          *
* =========  ==================                                              *
* GND        GND                                                             *
* +5V        +5V                                                             *
* 18 (PC4)   /RST (AVR /RST)                                                 *
* 10 (PB2)   13 (PB5, AVR SCK)                                               *
*  9 (PB1)   12 (PB4, AVR MISO)                                              *
*  8 (PB0)   11 (PB3, MOSI)                                                  *
*  7 (PD7)   PIC MCLR                                                        *
*  6 (PD6)   PIC DAT                                                         *
*  5 (PD5)   PIC CLK                                                         *
*  _                                                                         *
* / \._  _ .__._|_o._  _  ._ _  _  _| _  _                                   *
* \_/|_)(/_|(_| |_|| |(_| | | |(_)(_|(/__>                                   *
*    |                 _|                                                    *
* A0   A1   MODE                                                             *
* ===  ===  ===================================                              *
* NC   NC   automatic (once)                                                 *
* GND  NC   manual mode (console)                                            *
* NC   GND  PIC hex file                                                     *
* GND  GND  serial pass through (9k6) RX=2,TX=3                              *
*****************************************************************************/

#include <SoftwareSerial.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include "avr_isp.h"
#include "pic_icsp.h"
#include "avr_code.h"
#include "pic_code.h"
#include "pic_conf.h"

int ledPin=13;
int mode0Pin=A0;
int mode1Pin=A1;
int rxPin=2;
int txPin=3;

uint8_t programmed_pic;
uint8_t programmed_avr;

void flash_led(int mson,int msoff,int n) {
  for (int i=0;i<n;++i) {
    digitalWrite(ledPin,HIGH);
    delay(mson);
    digitalWrite(ledPin,LOW);
    delay(msoff);
  }
}

void serial_passthrough() {
  Serial.println(F("Entering serial pass-through mode. 9k6. Pins: RX=2,TX=3."));
  SoftwareSerial softserial(rxPin,txPin);
  softserial.begin(9600);
  while (true) {
    if (softserial.available()) Serial.write(softserial.read());
    if (Serial.available()) softserial.write(Serial.read());
  }
}

void serial_monitor() {
  Serial.println(F("Entered serial monitor mode. 9k6. Pins: RX=2,TX=3 (nothing being sent though). Send anything to quit."));
  SoftwareSerial softserial(rxPin,txPin);
  softserial.begin(9600);
  while (!Serial.available()) {
    if (softserial.available()) Serial.write(softserial.read());
  }
  Serial.read();
  softserial.end();
}

void avr_manual() {
  Serial.print(F("AVR\r\nEntering AVR programming mode... "));
  if (avr_isp_enter()) {
    Serial.println(F("OK"));
  }
  else {
    Serial.println(F("FAILED. Leaving..."));
    avr_isp_leave();
    return;
  }
  while(1) {
    Serial.print(F("Options:\r\n"
                   " - F: read fuse bits\r\n"
                   " - R: read flash\r\n"
                   " - Q: quit\r\n"
                   "> "));
    int c;
    uint16_t data;
    while ((c=Serial.read())==-1);
    switch (c) {
      case 'Q': case 'q': // exit
        avr_isp_leave();
        return;
      case 'F': case 'f': // read fuse bits
        {
        uint8_t lfuse=boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS     );
        uint8_t hfuse=boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS    );
        uint8_t efuse=boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
        uint8_t lock =boot_lock_fuse_bits_get(GET_LOCK_BITS         );
        uint8_t lfuse_read=avr_isp_cmd(0x50,0x00,0x00,0x00);
        uint8_t hfuse_read=avr_isp_cmd(0x58,0x08,0x00,0x00);
        uint8_t efuse_read=avr_isp_cmd(0x50,0x08,0x00,0x00);
        uint8_t lock_read =avr_isp_cmd(0x58,0x00,0x00,0x00);
        //TODO: Serial.println(F("low:  ");send_hex(lfuse);send_msg(""));
        //TODO: Serial.println(F("high: ");send_hex(hfuse);send_msg(""));
        //TODO: Serial.println(F("ext:  ");send_hex(efuse);send_msg(""));
        //TODO: Serial.println(F("lock: ");send_hex(lock );send_msg(""));
        //TODO: Serial.println(F("target low:  ");send_hex(lfuse_read);send_msg(""));
        //TODO: Serial.println(F("target high: ");send_hex(hfuse_read);send_msg(""));
        //TODO: Serial.println(F("target ext:  ");send_hex(efuse_read);send_msg(""));
        //TODO: Serial.println(F("target lock: ");send_hex(lock_read );send_msg(""));
        }
      case 'r': case 'R':  // read
        {
          Serial.println(F("Enter address: "));
          uint16_t addr=Serial.parseInt();
          boot_rww_enable ();
          uint16_t data=pgm_read_word(addr<<1);
          uint8_t lo=avr_isp_cmd(0x20,addr>>8&0xFF,addr>>0&0xFF,0x00);
          uint8_t hi=avr_isp_cmd(0x28,addr>>8&0xFF,addr>>0&0xFF,0x00);
          Serial.print(addr,HEX);
          Serial.print(F(" : "));
          Serial.print(hi<<8|lo,HEX);
          Serial.print(F(" = "));
          Serial.println(data,HEX);
        }
    }
  }
}

void pic_manual() {
  Serial.print(F("PIC\r\nEntering PIC programming mode... "));
  if (pic_icsp_enter()) {
    Serial.println(F("OK"));
  }
  else {
    Serial.println(F("FAILED. Leaving..."));
    pic_icsp_leave();
    return;
  }
  while(1) {
    Serial.print(F("Options:\r\n"
                   " - C: load configuration (0x00)\r\n"
                   " - D: load data          (0x02)\r\n"
                   " - I: increment address  (0x06)\r\n"
                   " - B: reset address      (0x16)\r\n"
                   " - R: read               (0x04)\r\n"
                   " - P: program            (0x08)\r\n"
                   " - X: erase              (0x09)\r\n"
                   " - H: program hex file\r\n"
                   " - Q: quit\r\n"
                   "> "));
    int c;
    uint16_t data;
    int ret;
    while ((c=Serial.read())==-1);
    switch (c) {
      case 'Q': case 'q': // exit
        pic_icsp_leave();
        return;
      case 'C': case 'c':  // load configuration
        Serial.print(F("C\r\nEnter data > "));
        data=Serial.parseInt();
        Serial.print(F("\r\nWriting "));
        Serial.print(data,HEX);
        Serial.print(F(" ..."));
        pic_icsp_cmd(0x00);
        pic_icsp_write(data);
        Serial.println(F(" done."));
        break;
      case 'D': case 'd':  // load data
        Serial.print(F("C\r\nEnter data > "));
        data=Serial.parseInt();
        Serial.print(F("\r\nWriting "));
        Serial.print(data,HEX);
        Serial.print(F(" ..."));
        pic_icsp_cmd(0x02);
        pic_icsp_write(data);
        Serial.println(F(" done."));
        break;
      case 'r': case 'R':  // read
        Serial.print(F("R\r\nReading..."));
        pic_icsp_cmd(0x04);
        data=pic_icsp_read();
        Serial.print(F(" done: "));
        Serial.println(data,HEX);
        break;
      case 'I': case 'i':  // increment address
        Serial.print(F("I\r\nIncrementing..."));
        pic_icsp_cmd(0x06);
        Serial.println(F(" done."));
        break;
      case 'B': case 'b':  // reset address
        Serial.print(F("B\r\nResetting address..."));
        pic_icsp_cmd(0x16);
        Serial.println(F(" done."));
        break;
      case 'P': case 'p': // program
        pic_icsp_cmd(0x08);
        break;
      case 'X': case 'x': // erase
        Serial.print(F("X\r\nErasing device..."));
        pic_icsp_cmd(0x09);
        Serial.println(F(" done."));
        break;
      case 'H': case 'h': // hex file
        Serial.print(F("H\r\nProgramming hex file.\r\nPlease send file..."));
        ret=pic_icsp_program_hexfile();
        if (ret==0)
          Serial.println(F(" done."));
        else {
          Serial.print(F(" ERROR #"));
          Serial.println(ret,DEC);
        }
        break;
      default:
        Serial.println(F("ERROR: Invalid option!"));
        break;
    }
  }
}

void manual_mode() {
  int c;
  do {
    Serial.print(F("Options:\r\n"
                   " - P: PIC\r\n"
                   " - A: AVR\r\n"
                   " - S: serial monitor\r\n"
                   "> "
                ));
    while ((c=Serial.read())==-1);
    switch (c) {
      case 'p':case 'P': pic_manual();break;
      case 'a':case 'A': avr_manual();break;
      case 's':case 'S':Serial.println("S");serial_monitor();break;
    }
  }  while (!(c=='q' || c=='!'));
  Serial.print(F("LEAVING MANUAL MODE"));
}

void setup() {
  programmed_pic=0;
  programmed_avr=0;

  pinMode(ledPin,OUTPUT); 
  pinMode(mode0Pin,INPUT_PULLUP);
  pinMode(mode1Pin,INPUT_PULLUP);

  Serial.begin(9600);

  if (digitalRead(mode0Pin)==HIGH && digitalRead(mode1Pin)==LOW
   || digitalRead(mode0Pin)==LOW  && digitalRead(mode1Pin)==LOW) {
    // done waste time...
  }
  else {
    Serial.println(F("\r\n\r\n\r\n\r\n"
" ___ _____ _       _                         _ _         _\r\n"
"| _ \\_   _| |  ___(_)_ _  ___   _ _ ___ _ __| (_)__ __ _| |_ ___ _ _\r\n"
"|  _/ | | | |_|___| | ' \\/ _ \\ | '_/ -_) '_ \\ | / _/ _` |  _/ _ \\ '_|\r\n"
"|_|   |_| |____|  |_|_||_\\___/ |_| \\___| .__/_|_\\__\\__,_|\\__\\___/_|\r\n"
"                                     |_|\r\n"
"Copyright (C) 2016, 2017  magnustron, Post Tenebras Lab\r\n"
"Released under GNU Public Licence v3 (GPLv3)\r\n"
"More infos here:\r\n"
" - Project page at PTL: https://www.posttenebraslab.ch/wiki/projects/electronics/ptl-ino/replicator\r\n"
" - Code at github: https://github.com/PostTenebrasLab/PTL-ino-replicator\r\n"
" _\r\n"
"/  _ ._ ._  _  __|_o _ ._  _|_ _.|_ | _\r\n"
"\\_(_)| || |(/_(_ |_|(_)| |  |_(_||_)|(/_\r\n"
"\r\n"
"HOST       TARGET\r\n"
"=========  ==============\r\n"
"GND        GND\r\n"
"+5V        +5V\r\n"
"18 (PC4)    /RST (AVR /RST)\r\n"
"10 (PB2)    13 (PB5, AVR SCK)\r\n"
" 9 (PB1)    12 (PB4, AVR MISO)\r\n"
" 8 (PB0)    11 (PB3, MOSI)\r\n"
" 7 (PD7)    PIC MCLR\r\n"
" 6 (PD6)    PIC DAT\r\n"
" 5 (PD5)    PIC CLK\r\n"
" _\r\n"
"/ \._  _ .__._|_o._  _  ._ _  _  _| _  _\r\n"
"\\_/|_)(/_|(_| |_|| |(_| | | |(_)(_|(/__>\r\n"
"   |                 _|\r\n"
"A0   A1   MODE\r\n"
"===  ===  =====================\r\n"
"NC   NC   automatic (once)\r\n"
"GND  NC   manual mode (console)\r\n"
"NC   GND  PIC hex file\r\n"
"GND  GND  reserved\r\n"));
  }
}

void done() {
  Serial.println(F("ALL DONE. Press reset to continue..."));
  while(1);
}

void loop() {
  if (digitalRead(mode0Pin)==HIGH && digitalRead(mode1Pin)==LOW) {
    Serial.println(F("Please send PIC hex file..."));
    int ret=pic_icsp_program_hexfile();
    if (ret==0)
      done();
    else {
      Serial.print(F("ERROR #"));
      Serial.println(ret,DEC);
      done();
    }
  }
  else if (digitalRead(mode0Pin)==LOW  && digitalRead(mode1Pin)==HIGH || Serial.available()>0) {
    Serial.println(F("Entering manual mode..."));
    manual_mode();
    done();
  }
  else if (digitalRead(mode0Pin)==LOW  && digitalRead(mode1Pin)==LOW) {
    serial_passthrough();
  }
  else {
    // PIC PROGRAMMING
    if (!programmed_pic) {
      Serial.println(F("Trying to enter PIC programming mode..."));
      if (pic_icsp_enter()) {
        Serial.println(F("... PIC found! Programming it..."));
        flash_led(100,400,3); // signal PIC programming attempt
        if (pic_icsp_program(pic_code,PIC_CODE_LENGTH,pic_conf,PIC_CONF_LENGTH)) {
          Serial.println(F("... PIC programming SUCCESSFUL..."));
          flash_led(100,100,5); // signal success
          programmed_pic=1;
        }
        else {
          Serial.println(F("... PIC programming FAILED..."));
          flash_led(900,100,5); // signal error
        }
      }
      else {
        Serial.println(F("... no PIC found..."));
      }
      Serial.println(F("... leaving PIC programming mode\r\n"));
      pic_icsp_leave();
    }
    // AVR PROGRAMMING
    if (!programmed_avr) {
      Serial.println(F("Trying to enter AVR programming mode..."));
      if (avr_isp_enter()) {
        Serial.println(F("... AVR found! Programming it..."));
        flash_led(400,100,3); // signal AVR programming attempt
        if (avr_isp_program(avr_code,AVR_CODE_START,AVR_CODE_FINISH)) {
          Serial.println(F("... AVR programming SUCCESSFUL..."));
          flash_led(100,100,5); // signal success
          programmed_avr=1;
        }
        else {
          Serial.println(F("... AVR programming FAILED..."));
          flash_led(900,100,5); // signal error
        }
      }
      else {
        Serial.println(F("... no AVR found..."));
      }
      Serial.println(F("... leaving AVR programming mode\r\n"));
      avr_isp_leave();
    }
    if (programmed_avr && programmed_pic) done();
    delay(1000);
  }
}

