00001 /*
00002  This file is part of the VectorNav Support Library.
00003  
00004  The VectorNav Support Library is free software: you can redistribute it and/or modify
00005  it under the terms of the Lesser GNU General Public License as published by
00006  the Free Software Foundation, either version 3 of the License, or
00007  (at your option) any later version.
00008  
00009  The VectorNav Support Library is distributed in the hope that it will be useful,
00010  but WITHOUT ANY WARRANTY; without even the implied warranty of
00011  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
00012  Lesser GNU General Public License for more details.
00013  
00014  You should have received a copy of the Lesser GNU General Public License
00015  along with the VectorNav Support Library.  If not, see <http://www.gnu.org/licenses/>.
00016  */
00017 
00018 #ifndef VNCONTROL_H
00019 #define VNCONTROL_H
00020 #include "VNDefines.h"
00021 #include "VNStructs.h"
00022 #include <unistd.h>
00023 
00046 int VNS_UART_setSleepDelay(struct DeviceControl *control, useconds_t sleepDelay);
00047 
00059 const char* VNS_UART_VNStrerror(int errornum);
00060 
00072 #ifndef MINGW
00073 int VNS_UART_drainBuffer(struct DeviceControl *control);
00074 #endif
00075 #endif
