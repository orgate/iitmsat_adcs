/*00001 /*
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
00017/ 
00027/ #ifndef VNWRITE_H
00028/ #define VNWRITE_H
00029/ #include "VNDefines.h"
00030/ #include "VNStructs.h"
00031/ #include "VNChecksum.h"
00032/ #include "VNControl.h"
00033/ 
00044/ int VNS_UART_tareDevice(struct DeviceControl *control);
00045/ 
00056/ int VNS_UART_restoreFactorySettings(struct DeviceControl *control);
00057/ 
00068/ int VNS_UART_resetDevice(struct DeviceControl *control);
00069/ 
00080/ int VNS_UART_writeSettings(struct DeviceControl *control);
00081/ 
00082/ /*
00083/  See VNWrite.c to see why setBaudRate is disabled
00084/ 
00085*/ int setBaudRate(struct DeviceControl *control, int baudRate);
00086*/ */
00087*/ 
00102*/ int VNS_UART_setAsyncOutputFrequency(struct DeviceControl *control, int frequency);
00103*/ 
 int VNS_UART_setAsyncOutputType(struct DeviceControl *control, int outputType);

 void VNS_UART_commonMessageSend(struct DeviceControl *control, int timeOut, int type, char *message, struct returnPacket *rPacket);
 
 int VNS_UART_getReturnValue(struct DeviceControl *control, char *expectedMessage, struct returnPacket *rPacket);
 
 #endif
