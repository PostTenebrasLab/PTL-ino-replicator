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

#ifndef PIC_ICSP_H
#define PIC_ICSP_H

#include <Arduino.h>

#define PIC_DEVICE_ID 0x3021
#define PIC_ICSP_MCLR_PORT D
#define PIC_ICSP_MCLR_PIN  7
#define PIC_ICSP_CLK_PORT  D
#define PIC_ICSP_CLK_PIN   5
#define PIC_ICSP_DAT_PORT  D
#define PIC_ICSP_DAT_PIN   6

uint8_t  pic_icsp_enter();
void     pic_icsp_leave();
void     pic_icsp_cmd(uint8_t cmd);
uint16_t pic_icsp_read();
void     pic_icsp_write(uint16_t data);
uint8_t  pic_icsp_program(const uint8_t *code,int16_t code_size,const uint8_t *conf,int16_t conf_size);
uint8_t  pic_icsp_program_hexfile();

#endif

