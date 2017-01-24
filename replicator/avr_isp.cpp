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

#include "avr_isp.h"
#include "portutil.h"
#include <avr/boot.h>
#include <avr/pgmspace.h>

uint8_t avr_isp_spi(uint8_t c) {
  uint8_t recv=0x00;
  for (uint8_t i=0;i<8;++i) {
    if (c&0x80)
      PORT(AVR_ISP_MOSI_PORT)|= (1<<AVR_ISP_MOSI_PIN);
    else
      PORT(AVR_ISP_MOSI_PORT)&=~(1<<AVR_ISP_MOSI_PIN);
    c<<=1;
    PORT(AVR_ISP_SCK_PORT)|= (1<<AVR_ISP_SCK_PIN);
    delayMicroseconds(10);
    recv<<=1;
    if (PIN(AVR_ISP_MISO_PORT)&1<<AVR_ISP_MISO_PIN)
      recv|=0x1;
    PORT(AVR_ISP_SCK_PORT)&=~(1<<AVR_ISP_SCK_PIN);
    delayMicroseconds(10);
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

  delayMicroseconds(100000); // wait for more than 20ms (100ms)

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

uint8_t avr_isp_program(const uint8_t *code,int16_t code_start,int16_t code_finish) {
  // ERASE CHIP
  avr_isp_cmd(0xAC,0x80,0x00,0x00);
  delayMicroseconds(10000);                    // wait for more than 9.0ms (10ms)

  // PROGRAM FLASH
  uint16_t addr=0; // in words
  for (int ipage=0;ipage<(FLASHEND+1L)/SPM_PAGESIZE;++ipage) {
    for (int i=0;i<SPM_PAGESIZE/2;++i) {
      uint16_t data;
      if (code && code_start<=(addr<<1) && (addr<<1)<code_finish)
        data=pgm_read_byte(&code[(addr<<1)-code_start  ])
            |pgm_read_byte(&code[(addr<<1)-code_start+1])<<8;
      else
        data=pgm_read_word(addr<<1);
      avr_isp_cmd(0x40,0x00,i,data>>0&0xFF);
      avr_isp_cmd(0x48,0x00,i,data>>8&0xFF);
      ++addr;
    }
    uint16_t pageaddr=(addr-1)-(addr-1)%(SPM_PAGESIZE/2);
    avr_isp_cmd(0x4C,pageaddr>>8,pageaddr,0x00);
    delayMicroseconds(5000);                   // wait for more than 4.5ms (5ms)
  }

  // VERIFY FLASH
  addr=0;
  for (int ipage=0;ipage<(FLASHEND+1L)/SPM_PAGESIZE;++ipage) {
    for (int i=0;i<SPM_PAGESIZE/2;++i) {
      uint16_t data;
      if (code &&code_start<=(addr<<1) && (addr<<1)<code_finish)
        data=pgm_read_byte(&code[(addr<<1)-code_start  ])
            |pgm_read_byte(&code[(addr<<1)-code_start+1])<<8;
      else
        data=pgm_read_word(addr<<1);
      uint8_t lo=avr_isp_cmd(0x20,addr>>8&0xFF,addr>>0&0xFF,0x00);
      uint8_t hi=avr_isp_cmd(0x28,addr>>8&0xFF,addr>>0&0xFF,0x00);
      if ((hi<<8|lo)!=data) {
  //      send_hex(addr);
  //      putch(':');
  //      send_hex(hi<<8|lo);
  //      putch('=');
  //      send_hex(data);
  //      putch('\r');
  //      putch('\n');
        return 0;
      }
      ++addr;
    }
  }


  // PROGRAM AND VERIFY FUSES
  uint8_t lfuse=boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS     );
  avr_isp_cmd(0xAC,0xA0,0x00,lfuse);
  delayMicroseconds(5000); // wait for more than 4.5ms (5ms)
  uint8_t lfuse_read=avr_isp_cmd(0x50,0x00,0x00,0x00);
  if (lfuse_read!=lfuse) return 0;

  uint8_t hfuse=boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS    );
  avr_isp_cmd(0xAC,0xA8,0x00,hfuse);
  delayMicroseconds(5000); // wait for more than 4.5ms (5ms)
  uint8_t hfuse_read=avr_isp_cmd(0x58,0x08,0x00,0x00);
  if (hfuse_read!=hfuse) return 0;

  uint8_t efuse=boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
  avr_isp_cmd(0xAC,0xA4,0x00,efuse);
  delayMicroseconds(5000); // wait for more than 4.5ms (5ms)
  uint8_t efuse_read=avr_isp_cmd(0x50,0x08,0x00,0x00);
  if (efuse_read!=efuse) return 0;

  uint8_t lock =boot_lock_fuse_bits_get(GET_LOCK_BITS         );
  avr_isp_cmd(0xAC,0xE0,0x00,lock );
  delayMicroseconds(5000); // wait for more than 4.5ms (5ms)
  uint8_t lock_read=avr_isp_cmd(0x58,0x00,0x00,0x00);
  if (lock_read!=lock ) return 0;

  return 1; 
}

