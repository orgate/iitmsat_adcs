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
00027 #ifndef VNREAD_H
00028 #define VNREAD_H
00029 #include "VNDefines.h"
00030 #include "VNStructs.h"
00031 #include "VNChecksum.h"
00032 #include "VNWrite.h"
00033 
00044 int VNS_UART_getHardwareRevision(struct DeviceControl *control);
00045 
00056 int VNS_UART_getFirmwareVersion(struct DeviceControl *control);
00057 
00069 int VNS_UART_getSerialNumber(struct DeviceControl *control, struct SerialNumber *serialNumber);
00070 
00082 int VNS_UART_getModelNumber(struct DeviceControl *control, struct ModelNumber *modelNumber);
00083 
00094 int VNS_UART_getAsyncOutputType(struct DeviceControl *control);
00095 
00106 int VNS_UART_getAsyncOutputFrequency(struct DeviceControl *control);
00107 
00119 int VNS_UART_getYawPitchRoll(struct DeviceControl *control, struct Attitude *ypr);
00120 
00132 int VNS_UART_getQuaternion(struct DeviceControl *control, struct Quaternion *quaternion);
00133 
00145 int VNS_UART_getQuaternionMagnetic(struct DeviceControl *control, struct QuaternionMagnetic *quaternionMagnetic);
00146 
00158 int VNS_UART_getQuaternionAcceleration(struct DeviceControl *control, struct QuaternionAcceleration *quaternionAcceleration);
00159 
00171 int VNS_UART_getQuaternionAngularRate(struct DeviceControl *control, struct QuaternionAngularRate *quaternionAngularRate);
00172 
00184 int VNS_UART_getQuaternionMagneticAcceleration(struct DeviceControl *control, struct QuaternionMagneticAcceleration *quaternionMagneticAcceleration);
00185 
00197 int VNS_UART_getQuaternionAccelerationAngularRate(struct DeviceControl *control, struct QuaternionAccelerationAngularRate *quaternionAccelerationAngularRate);
00198 
00209 int VNS_UART_getBaudrate(struct DeviceControl *control);
00210 
00228 int VNS_UART_asynchronousCapture(struct DeviceControl *control, union asyncDataList *dataType);
00229 #endif
