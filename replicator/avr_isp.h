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

#ifndef AVR_ISP_H
#define AVR_ISP_H

#include <Arduino.h>

#define AVR_SIGNATURE_0 0x1E
#define AVR_SIGNATURE_1 0x95
#define AVR_SIGNATURE_2 0x0F
#define AVR_ISP_RST_PORT  C
#define AVR_ISP_RST_PIN   4
#define AVR_ISP_SCK_PORT  B
#define AVR_ISP_SCK_PIN   2
#define AVR_ISP_MISO_PORT B
#define AVR_ISP_MISO_PIN  1
#define AVR_ISP_MOSI_PORT B
#define AVR_ISP_MOSI_PIN  0

uint8_t avr_isp_enter();
void    avr_isp_leave();
uint8_t avr_isp_spi(uint8_t c);
uint8_t avr_isp_cmd(uint8_t c1,uint8_t c2,uint8_t c3,uint8_t c4);
uint8_t avr_isp_program(const uint8_t *code=0,int16_t code_start=-1,int16_t code_finish=-1);

#endif
