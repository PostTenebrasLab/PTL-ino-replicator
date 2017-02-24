/*****************************************************************************
* This file is part of PTL-ino replicator.                                   *
*                                                                            *
* PTL-ino replcator is free software; you can redistribute it and/or modify  *
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

#include "pic_icsp.h"
#include "portutil.h"

void pic_icsp_cmd(uint8_t cmd) {
  for (int i=0;i<6;++i) {
    PORT(PIC_ICSP_CLK_PORT)|= (1<<PIC_ICSP_CLK_PIN);
    if (cmd&0x01)
      PORT(PIC_ICSP_DAT_PORT)|= (1<<PIC_ICSP_DAT_PIN);
    else
      PORT(PIC_ICSP_DAT_PORT)&=~(1<<PIC_ICSP_DAT_PIN);
    delayMicroseconds(1); 
    PORT(PIC_ICSP_CLK_PORT)&=~(1<<PIC_ICSP_CLK_PIN);
    delayMicroseconds(1); 
    cmd>>=1;
  }
}

uint16_t pic_icsp_read() {
  uint16_t data=0;
  DDR(PIC_ICSP_DAT_PORT)&=~(1<<PIC_ICSP_DAT_PIN);
  for (int i=0;i<16;++i) {
    data>>=1;
    PORT(PIC_ICSP_CLK_PORT)|= (1<<PIC_ICSP_CLK_PIN);
    delayMicroseconds(1); 
    if (PIN(PIC_ICSP_DAT_PORT)&1<<PIC_ICSP_DAT_PIN) data|=0x8000;
    PORT(PIC_ICSP_CLK_PORT)&=~(1<<PIC_ICSP_CLK_PIN);
    delayMicroseconds(1); 
  }
  DDR(PIC_ICSP_DAT_PORT)|= (1<<PIC_ICSP_DAT_PIN);
  return data>>1&0x3FFF;
}

void pic_icsp_write(uint16_t data) {
  data=data<<1&0x7FFE;
  for (int i=0;i<16;++i) {
    PORT(PIC_ICSP_CLK_PORT)|= (1<<PIC_ICSP_CLK_PIN);
    if (data&0x01)
      PORT(PIC_ICSP_DAT_PORT)|= (1<<PIC_ICSP_DAT_PIN);
    else
      PORT(PIC_ICSP_DAT_PORT)&=~(1<<PIC_ICSP_DAT_PIN);
    delayMicroseconds(1); 
    PORT(PIC_ICSP_CLK_PORT)&=~(1<<PIC_ICSP_CLK_PIN);
    delayMicroseconds(1); 
    data>>=1;
  }
}

uint8_t pic_icsp_enter() {
  // INIT PINS
  DDR(PIC_ICSP_CLK_PORT )|= (1<<PIC_ICSP_CLK_PIN );
  DDR(PIC_ICSP_DAT_PORT )|= (1<<PIC_ICSP_DAT_PIN );
  delayMicroseconds(1); // >100ns (here: 1us)
  // ASSERT RESET
  DDR(PIC_ICSP_MCLR_PORT)|= (1<<PIC_ICSP_MCLR_PIN);
  delayMicroseconds(500); // >250us (here: 500us?

  // SEND MAGIC NUMBER
  uint32_t magic=0x4D434850;
  for (int i=0;i<33;++i) { // 33(!) cycles
    PORT(PIC_ICSP_CLK_PORT)|= (1<<PIC_ICSP_CLK_PIN);
    if (magic&0x01)
      PORT(PIC_ICSP_DAT_PORT)|= (1<<PIC_ICSP_DAT_PIN);
    else
      PORT(PIC_ICSP_DAT_PORT)&=~(1<<PIC_ICSP_DAT_PIN);
    delayMicroseconds(1); 
    PORT(PIC_ICSP_CLK_PORT)&=~(1<<PIC_ICSP_CLK_PIN);
    delayMicroseconds(1); 
    magic>>=1;
  }
  delayMicroseconds(100);
  // CHECK DEVICE ID
  pic_icsp_cmd(0x00); // load configuration (addr=0x8000)
  pic_icsp_write(0xFFFF); // dummy data
  for (int i=0;i<6;++i)
    pic_icsp_cmd(0x06); // increment address
  pic_icsp_cmd(0x04);
  uint16_t id=pic_icsp_read();
  return id==PIC16F1455_DEVICE_ID || id==PIC16F1454_DEVICE_ID;
}

void pic_icsp_leave() {
  DDR(PIC_ICSP_MCLR_PORT)&=~(1<<PIC_ICSP_MCLR_PIN);
  DDR(PIC_ICSP_CLK_PORT )&=~(1<<PIC_ICSP_CLK_PIN );
  DDR(PIC_ICSP_DAT_PORT )&=~(1<<PIC_ICSP_DAT_PIN );
}

uint8_t pic_icsp_program(const uint8_t *code,int16_t code_size,const uint8_t *conf,int16_t conf_size) {
  // ERASE CHIP
  pic_icsp_cmd(0x16); // reset address counter
  pic_icsp_cmd(0x09); // bulk erase
  delayMicroseconds(6000); // wait >5ms (6ms)

  // PROGRAM PRORGRAM MEMORY
  for (int i=0;i<code_size;i+=2) {
    if (i) pic_icsp_cmd(0x06); // increment
    pic_icsp_cmd(0x02); // load data
    pic_icsp_write(pgm_read_byte(&code[i+1])<<8|pgm_read_byte(&code[i]));
    pic_icsp_cmd(0x08); // program 
    delayMicroseconds(3000); // wait >2.5ms (3ms) 
  }

  // PROGRAM USER ID
  pic_icsp_cmd(0x00); // load configuration (addr=0x8000)
  pic_icsp_write(0xFFFF); // TODO: dummy data
  for (int i=0;i<4;++i) {
    if (i) {
      pic_icsp_cmd(0x06); // increment
//      pic_icsp_cmd(0x02); // load data
    }
//    pic_icsp_write(pic_user[i]);
//    pic_icsp_cmd(0x08); // program 
    delayMicroseconds(6000); // wait >5ms (6ms)
  }

  // PROGRAM CONFIGURATION BYTES
  for (int i=0;i<3;++i)
    pic_icsp_cmd(0x06); // increment
  for (int i=0;i<conf_size;i+=2) {
    pic_icsp_cmd(0x06); // increment
    pic_icsp_cmd(0x02); // load data
    pic_icsp_write(conf[i+1]<<8|conf[i]);
    pic_icsp_cmd(0x08); // program 
    delayMicroseconds(6000); // wait >5ms (6ms) 
  }

  return 1;
}

int32_t hex2int(const char *str,int n) {
  int i=0;
  for (int j=0;j<n;++j) {
    i*=16;
    if (0<=str[j] && str[j]<='9')
      i+=str[j]-'0';
    else if ('A'<=str[j] && str[j]<='F')
      i+=str[j]-'A'+10;
    else if ('a'<=str[j] && str[j]<='f')
      i+=str[j]-'a'+10;
    else
      return -1;
  }
  return i;
}


uint8_t pic_icsp_program_hexfile() {
  char line[SERIAL_RX_BUFFER_SIZE];
  int n;
  int32_t extaddr=0;
  uint32_t curraddr=0;
  uint32_t lastaddr=0xFFFFFFFF;
  int bytecount ;
  int address   ;
  int recordtype;
  int checksum  ;
  do {
    while ((n=Serial.readBytesUntil('\n',line,sizeof(line)))==0);
    if (n==sizeof(line)) return 10; // potential overflow
    if (line[n-1]=='\r') n-=1;
    if (n<11) return 1; // ':CCAAAARRSS'
    if (line[0]!=':') return 2;
    bytecount =hex2int(&line[  1],2);
    address   =hex2int(&line[  3],4);
    recordtype=hex2int(&line[  7],2);
    checksum  =hex2int(&line[n-2],2);
    if (bytecount <0) return 3;
    if (address   <0) return 4;
    if (recordtype<0) return 5;
    if (checksum  <0) return 6;
    if (n!=11+bytecount*2) return 7;
    //send_msg("n: ");send_hex(n);send_msg("\r\n");
    //send_msg("BC: ");send_hex(bytecount);send_msg("\r\n");
    //send_msg("AD: ");send_hex(address);send_msg("\r\n");
    //send_msg("RT: ");send_hex(recordtype);send_msg("\r\n");
    //send_msg("CS: ");send_hex(checksum);send_msg("\r\n");
    switch (recordtype) {
      case 0x00:
        if (bytecount%2!=0) return 8;
        for (int i=0;i<bytecount;i+=2) {
          uint8_t b1=hex2int(&line[9+2*i+0],2);
          uint8_t b2=hex2int(&line[9+2*i+2],2);
          uint16_t word=b2<<8|b1;
          curraddr=extaddr<<16|address+i;
          if (curraddr<lastaddr) {
            if (curraddr>=0x10000UL) {
              pic_icsp_cmd(0x00); // load configuration (addr=0x8000)
              pic_icsp_write(0xFFFF); // TODO: dummy data
              lastaddr=0x10000UL;
            }
            else {
              pic_icsp_cmd(0x16); // reset address
              lastaddr=0;
            }
          }
          if (lastaddr<0x10000UL && curraddr>=0x10000UL) {
            pic_icsp_cmd(0x00); // load configuration (addr=0x8000)
            pic_icsp_write(0xFFFF); // TODO: dummy data
            lastaddr=0x10000UL;
          }
          while (lastaddr<curraddr) {
            pic_icsp_cmd(0x06);
            lastaddr+=2;
          }
          pic_icsp_cmd(0x02); // load data
          pic_icsp_write(word);
          pic_icsp_cmd(0x08); // program 
          delayMicroseconds(3000); // wait >2.5ms (3ms) 
          //send_hex(curraddr>>16);send_hex(curraddr);send_msg("->");send_hex(word);send_msg("\r\n");
        }
        break;
      case 0x04:
        extaddr=hex2int(&line[9],4);
        if (extaddr<0) return 9;
        break;
    }
    // todo checksum validation
  } while (recordtype!=0x01);
  return 0;
}
