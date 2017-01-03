/******************************************************************************
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
 ******************************************************************************/

/******************************************************************************
 *  _                                                                         *
 * /  _ ._ ._  _  __|_o _ ._  _|_ _.|_ | _                                    *
 * \_(_)| || |(/_(_ |_|(_)| |  |_(_||_)|(/_                                   *
 *                                                                            *
 * HOST       TARGET                                                          *
 * =========  ==============                                                  *
 * GND        GND                                                             *
 * +5V        +5V                                                             *
 * 2 (PD2)    11 (PB3, MOSI)                                                  *
 * 3 (PD3)    13 (PB5, SCK)                                                   *
 * 4 (PD4)    12 (PB4, MISO)                                                  *
 * 5 (PD5)    /RST                                                            *
 * 6 (PD6)    PIC MCLR                                                        *
 * 7 (PD7)    PIC CLK                                                         *
 * 8 (PB0)    PIC DAT                                                         *
 *                                                                            *
 * LED: 13 (PB5)                                                              *
 *                                                                            *
 *                                                                            *
 * Ton (ms)   Toff (ms)  number  meaning                                      *
 * ========   =========  ======  =======                                      *
 * 100        100        1                                                    *
 *                                                                            *
 ******************************************************************************/

// AVR FUSES:
// TYPE  BLANK  ARDUINO
// ====  =====  =======
// LOW   62     FF
// HIGH  D9     DE
// EXT   FF     FD
// LOCK  FF     CF



#include <inttypes.h>
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "pic_code.h"
#include "pic_conf.h"

#define BAUD 9600

#define LED_PORT B
#define LED_PIN  5

#define PIC_DEVICE_ID 0x3021
#define AVR_SIGNATURE_0 0x1E
#define AVR_SIGNATURE_1 0x95
#define AVR_SIGNATURE_2 0x0F
#define PIC_ICSP_MCLR_PORT D
#define PIC_ICSP_MCLR_PIN  6
#define PIC_ICSP_CLK_PORT  D
#define PIC_ICSP_CLK_PIN   7
#define PIC_ICSP_DAT_PORT  B
#define PIC_ICSP_DAT_PIN   0

#define AVR_ISP_RST_PORT  D
#define AVR_ISP_RST_PIN   5
#define AVR_ISP_SCK_PORT  D
#define AVR_ISP_SCK_PIN   3
#define AVR_ISP_MOSI_PORT D
#define AVR_ISP_MOSI_PIN  2
#define AVR_ISP_MISO_PORT D
#define AVR_ISP_MISO_PIN  4

#define concat(a,b) a ## b
#define PORT(x) concat(PORT,x)
#define PIN(x)  concat(PIN,x)
#define DDR(x)  concat(DDR,x)

#define usleep(x) _delay_us(x)

//TODO: needed?
void  __attribute__ ((naked)) TIMER1_COMPA_vect(void){
  __asm("in  r0, 0x3f"  );    //Status sichern
  __asm("inc r3"       );    //tic++;
  __asm("out  0x3f, r0");    //Status wiederherstellen
  __asm("reti         ");
}


int getch() {
  if (!(UCSR0A&1<<RXC0))
    return -1;
  else
    return UDR0;
}

int getnumber() {
  int c;
  do {
    c=getch();
  } while (c==-1||c==' '||c=='\n'||c=='\r'||c=='\t');
  int n=0;
  while (1) {
    if (c<'0'||c>'9') break;
    n=n*10+(c&0xF);
    do {
      c=getch();
    } while (c==-1);
  }
  return n; 
}

void putch(uint8_t c) {
  while (!(UCSR0A&1<<UDRE0));
  UDR0=c;
}

void send_msg(const char *msg) {
  while (*msg) {
    putch(*msg);
++msg;
  }
}

void send_hex(uint16_t n) {
  for (int i=0;i<4;++i) {
    uint8_t c=(n>>(4*(3-i)))&0xF;
    if (c<10)
      c|='0';
    else
      c='A'+c-10;
    putch(c);
  }
}

void flash_led(int mson,int msoff,int n) {
  for (int i=0;i<n;++i) {
    PORT(LED_PORT)|= (1<<LED_PIN);
    usleep(1000.0*mson);
    PORT(LED_PORT)&=~(1<<LED_PIN);
    usleep(1000.0*msoff);
  }
}

void pic_icsp_cmd(uint8_t cmd) {
  for (int i=0;i<6;++i) {
    PORT(PIC_ICSP_CLK_PORT)|= (1<<PIC_ICSP_CLK_PIN);
    if (cmd&0x01)
      PORT(PIC_ICSP_DAT_PORT)|= (1<<PIC_ICSP_DAT_PIN);
    else
      PORT(PIC_ICSP_DAT_PORT)&=~(1<<PIC_ICSP_DAT_PIN);
    usleep(1); 
    PORT(PIC_ICSP_CLK_PORT)&=~(1<<PIC_ICSP_CLK_PIN);
    usleep(1); 
    cmd>>=1;
  }
}

uint16_t pic_icsp_read() {
  uint16_t data=0;
  DDR(PIC_ICSP_DAT_PORT)&=~(1<<PIC_ICSP_DAT_PIN);
  for (int i=0;i<16;++i) {
    data>>=1;
    PORT(PIC_ICSP_CLK_PORT)|= (1<<PIC_ICSP_CLK_PIN);
    usleep(1); 
    if (PIN(PIC_ICSP_DAT_PORT)&1<<PIC_ICSP_DAT_PIN) data|=0x8000;
    PORT(PIC_ICSP_CLK_PORT)&=~(1<<PIC_ICSP_CLK_PIN);
    usleep(1); 
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
    usleep(1); 
    PORT(PIC_ICSP_CLK_PORT)&=~(1<<PIC_ICSP_CLK_PIN);
    usleep(1); 
    data>>=1;
  }
}

uint8_t pic_icsp_enter() {
  // INIT PINS
  DDR(PIC_ICSP_CLK_PORT )|= (1<<PIC_ICSP_CLK_PIN );
  DDR(PIC_ICSP_DAT_PORT )|= (1<<PIC_ICSP_DAT_PIN );
  usleep(1); // >100ns (here: 1us)
  // ASSERT RESET
  DDR(PIC_ICSP_MCLR_PORT)|= (1<<PIC_ICSP_MCLR_PIN);
  usleep(500); // >250us (here: 500us?

  // SEND MAGIC NUMBER
  uint32_t magic=0x4D434850;
  for (int i=0;i<33;++i) { // 33(!) cycles
    PORT(PIC_ICSP_CLK_PORT)|= (1<<PIC_ICSP_CLK_PIN);
    if (magic&0x01)
      PORT(PIC_ICSP_DAT_PORT)|= (1<<PIC_ICSP_DAT_PIN);
    else
      PORT(PIC_ICSP_DAT_PORT)&=~(1<<PIC_ICSP_DAT_PIN);
    usleep(1); 
    PORT(PIC_ICSP_CLK_PORT)&=~(1<<PIC_ICSP_CLK_PIN);
    usleep(1); 
    magic>>=1;
  }
  usleep(100);
  // CHECK DEVICE ID
  pic_icsp_cmd(0x00); // load configuration (addr=0x8000)
  pic_icsp_write(0xFFFF); // dummy data
  for (int i=0;i<6;++i)
    pic_icsp_cmd(0x06); // increment address
  pic_icsp_cmd(0x04);
  uint16_t id=pic_icsp_read();
  return id==PIC_DEVICE_ID;
}

void pic_icsp_leave() {
  DDR(PIC_ICSP_MCLR_PORT)&=~(1<<PIC_ICSP_MCLR_PIN);
  DDR(PIC_ICSP_CLK_PORT )&=~(1<<PIC_ICSP_CLK_PIN );
  DDR(PIC_ICSP_DAT_PORT )&=~(1<<PIC_ICSP_DAT_PIN );
}

uint8_t pic_icsp_program() {
  // ERASE CHIP
  pic_icsp_cmd(0x16); // reset address counter
  pic_icsp_cmd(0x09); // bulk erase
  usleep(6000); // wait >5ms (6ms)

  // PROGRAM PRORGRAM MEMORY
  for (int i=0;i<sizeof(pic_code);i+=2) {
    if (i) pic_icsp_cmd(0x06); // increment
    pic_icsp_cmd(0x02); // load data
    pic_icsp_write(pgm_read_byte(&pic_code[i+1])<<8|pgm_read_byte(&pic_code[i]));
    pic_icsp_cmd(0x08); // program 
    usleep(3000); // wait >2.5ms (3ms) 
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
    usleep(6000); // wait >5ms (6ms)
  }

  // PROGRAM CONFIGURATION BYTES
  for (int i=0;i<3;++i)
    pic_icsp_cmd(0x06); // increment
  for (int i=0;i<sizeof(pic_conf);i+=2) {
    pic_icsp_cmd(0x06); // increment
    pic_icsp_cmd(0x02); // load data
    pic_icsp_write(pic_conf[i+1]<<8|pic_conf[i]);
    pic_icsp_cmd(0x08); // program 
    usleep(6000); // wait >5ms (6ms) 
  }

  return 1;
}

uint8_t avr_isp_spi(uint8_t c) {
  uint8_t recv=0x00;
  for (uint8_t i=0;i<8;++i) {
    if (c&0x80)
      PORT(AVR_ISP_MOSI_PORT)|= (1<<AVR_ISP_MOSI_PIN);
    else
      PORT(AVR_ISP_MOSI_PORT)&=~(1<<AVR_ISP_MOSI_PIN);
    c<<=1;
    PORT(AVR_ISP_SCK_PORT)|= (1<<AVR_ISP_SCK_PIN);
    usleep(10);
    recv<<=1;
    if (PIN(AVR_ISP_MISO_PORT)&1<<AVR_ISP_MISO_PIN)
      recv|=0x1;
    PORT(AVR_ISP_SCK_PORT)&=~(1<<AVR_ISP_SCK_PIN);
    usleep(10);
  }  
  return recv;
}

uint8_t avr_isp_cmd(uint8_t c1,uint8_t c2,uint8_t c3,uint8_t c4) {
  avr_isp_spi(c1);
  avr_isp_spi(c2);
  avr_isp_spi(c3);
  return avr_isp_spi(c4);
}

uint8_t avr_isp_enter() {
  // INIT PINS
  DDR(AVR_ISP_SCK_PORT )|= (1<<AVR_ISP_SCK_PIN );
  DDR(AVR_ISP_MOSI_PORT)|= (1<<AVR_ISP_MOSI_PIN);
  // TODO: wait?
  // ASSERT RESET
  DDR(AVR_ISP_RST_PORT )|= (1<<AVR_ISP_RST_PIN );

  usleep(100000); // wait for more than 20ms (100ms)

  // ENTER ISP
  //if (avr_isp_cmd(0xAC,0x53,0x00,0x00)!=0x53) return 0;
  avr_isp_spi(0xAC);
  avr_isp_spi(0x53);
  if (avr_isp_spi(0x00)!=0x53) return 0;
  avr_isp_spi(0);

  // CHECK DEVICE SIGNATURE
  if (avr_isp_cmd(0x30,0x00,0x00,0x00)!=AVR_SIGNATURE_0) return 0;
  if (avr_isp_cmd(0x30,0x00,0x01,0x00)!=AVR_SIGNATURE_1) return 0;
  if (avr_isp_cmd(0x30,0x00,0x02,0x00)!=AVR_SIGNATURE_2) return 0;

  return 1;
}

void avr_isp_leave() {
  DDR(AVR_ISP_RST_PORT )&=~(1<<AVR_ISP_RST_PIN );
  DDR(AVR_ISP_SCK_PORT )&=~(1<<AVR_ISP_SCK_PIN );
  DDR(AVR_ISP_MOSI_PORT)&=~(1<<AVR_ISP_MOSI_PIN);
}

uint8_t avr_isp_program() {
  // ERASE CHIP
  avr_isp_cmd(0xAC,0x80,0x00,0x00);
  usleep(10000);                    // wait for more than 9.0ms (10ms)

  // PROGRAM FLASH
  uint16_t addr=0;
  for (int ipage=0;ipage<(FLASHEND+1L)/SPM_PAGESIZE;++ipage) {
    for (int i=0;i<SPM_PAGESIZE/2;++i) {
      uint16_t data=pgm_read_word(addr<<1);
      avr_isp_cmd(0x40,0x00,i,data>>0&0xFF);
      avr_isp_cmd(0x48,0x00,i,data>>8&0xFF);
      ++addr;
    }
    uint16_t pageaddr=(addr-1)-(addr-1)%(SPM_PAGESIZE/2);
    avr_isp_cmd(0x4C,pageaddr>>8,pageaddr,0x00);
    usleep(5000);                   // wait for more than 4.5ms (5ms)
  }

  // VERIFY FLASH
  addr=0;
  for (int ipage=0;ipage<(FLASHEND+1L)/SPM_PAGESIZE;++ipage) {
    for (int i=0;i<SPM_PAGESIZE/2;++i) {
      uint16_t data=pgm_read_word(addr<<1);
      uint8_t lo=avr_isp_cmd(0x20,addr>>8&0xFF,addr>>0&0xFF,0x00);
      uint8_t hi=avr_isp_cmd(0x28,addr>>8&0xFF,addr>>0&0xFF,0x00);
      if ((hi<<8|lo)!=data) {
        send_hex(addr);
        putch(':');
        send_hex(hi<<8|lo);
        putch('=');
        send_hex(data);
        putch('\r');
        putch('\n');
      }
      if ((hi<<8|lo)!=data) return 0;
      ++addr;
    }
  }


  // PROGRAM AND VERIFY FUSES
  putch('L');
  uint8_t lfuse=boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS     );
  send_hex(lfuse);
//  avr_isp_cmd(0xAC,0xA0,0x00,lfuse);
//  usleep(5000); // wait for more than 4.5ms (5ms)
  send_hex(avr_isp_cmd(0x50,0x00,0x00,0x00));
//  if (avr_isp_cmd(0x50,0x00,0x00,0x00)!=lfuse) return 0;
//
  putch('H');
  uint8_t hfuse=boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS    );
  send_hex(hfuse);
//  avr_isp_cmd(0xAC,0xA8,0x00,hfuse);
//  usleep(5000); // wait for more than 4.5ms (5ms)
  send_hex(avr_isp_cmd(0x58,0x08,0x00,0x00));
//  if (avr_isp_cmd(0x58,0x08,0x00,0x00)!=hfuse) return 0;
//
  putch('E');
  uint8_t efuse=boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
  send_hex(efuse);
//  avr_isp_cmd(0xAC,0xA4,0x00,efuse);
//  usleep(5000); // wait for more than 4.5ms (5ms)
  send_hex(avr_isp_cmd(0x50,0x08,0x00,0x00));
//  if (avr_isp_cmd(0x50,0x08,0x00,0x00)!=efuse) return 0;
//
   putch('C');
  uint8_t lock =boot_lock_fuse_bits_get(GET_LOCK_BITS         );
  send_hex(lock);
//  avr_isp_cmd(0xAC,0xE0,0x00,lock );
//  usleep(5000); // wait for more than 4.5ms (5ms)
  send_hex(avr_isp_cmd(0x58,0x00,0x00,0x00));
//  if (avr_isp_cmd(0x58,0x00,0x00,0x00)!=lock ) return 0;

  return 1; 
}

void pic_manual() {
  pic_icsp_leave();
  usleep(1000);
  if (pic_icsp_enter())
    send_msg("PIC\n\r");
  while(1) {
    int c;
    uint16_t data;
    while ((c=getch())==-1);
    switch (c) {
      case 'C': case 'c':  // load configuration
        data=getnumber();
        pic_icsp_cmd(0x00);
        pic_icsp_write(data);
        break;
      case 'D': case 'd':  // load data
        data=getnumber();
        pic_icsp_cmd(0x02);
        pic_icsp_write(data);
        break;
      case 'r': case 'R':  // read
        pic_icsp_cmd(0x04);
        data=pic_icsp_read();
        putch('0');
        putch('x');
        send_hex(data);
        putch('\r');
        putch('\n');
        break;
      case 'P':
        pic_icsp_leave();
        usleep(1000);
        if (pic_icsp_enter())
          send_msg("PIC\n\r");
        break;
      case 'p':
        pic_icsp_leave();
        break;
      case 'I': case 'i':  // increment address
        pic_icsp_cmd(0x06);
        break;
      case 'S': case 's':  // reset address
        pic_icsp_cmd(0x16);
        break;
      case 'Q': case 'q': // program
        pic_icsp_cmd(0x08);
        break;
      case 'X': case 'x': // erase
        pic_icsp_cmd(0x09);
        break;
      case 'E': case 'e': // exit
        pic_icsp_leave();
    }
  }
}

void main() {
  UBRR0=F_CPU/16/BAUD-1; // TODO: correct rounding
  UCSR0B=1<<RXEN0|1<<TXEN0;
  UCSR0C=1<<UCSZ01|1<<UCSZ00; // 8 data, 1 stop bit, no parity
  DDR(LED_PORT)|=1<<LED_PIN;
  flash_led(100,100,10); // say hello
  flash_led(1000,1000,1); // say hello
  send_msg("PTL-INO REPLICATOR\n\rPress P or A to enter manual PIC or AVR modes\n\r");
  while (1) {
    int c;
    if ((c=getch())!=-1) {
      // ENTER MANUAL PROCEDURE
      switch (c) {
        case 'P': pic_manual();
      }
    }
    else {
      // FLASH COUNTDOWN
      flash_led(500,500,1);
      flash_led(250,250,1);
      flash_led(125,1250,1);
      // AUTOMATIC PROCEDURE
      // PIC PROGRAMMING
      if (pic_icsp_enter()) {
        flash_led(100,400,3); // signal PIC programming attempt
        if (pic_icsp_program())
          flash_led(100,100,5); // signal success
        else
          flash_led(900,100,5); // signal error
      }
      pic_icsp_leave();
      // AVR PROGRAMMING
      if (avr_isp_enter()) {
        flash_led(400,100,3); // signal AVR programming attempt
        if (avr_isp_program())
          flash_led(100,100,5); // signal success
        else
          flash_led(900,100,5); // signal error
      }
      avr_isp_leave();
    }
  }
}
