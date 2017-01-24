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

#ifndef PORTUTIL_H
#define PORTUTIL_H

#define concat(a,b) a ## b
#define PORT(x) concat(PORT,x)
#define PIN(x)  concat(PIN,x)
#define DDR(x)  concat(DDR,x)

#endif

