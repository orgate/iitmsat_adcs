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
00024 //STRUCTS
00025 
00026 #ifndef VNSTRUCTS_H
00027 #define VNSTRUCTS_H
00028 #include <unistd.h>
00029 
00033 struct DeviceControl
00034 {
00036         int fd;
00038         int type;
00040         useconds_t sleepDelay;
00042         int __n;
00044         unsigned int __bytesRead;
00046         int __deviceType;
00048         char __buffer[500];
00049 };
00050 
00054 struct ModelNumber
00055 {
00057         char modelNumber[7];
00059         unsigned int length;
00060 };
00061 
00065 struct SerialNumber
00066 {
00068         char serialNumber[25];
00070         unsigned int length;
00071 };
00072 
00076 struct Quaternion
00077 {
00079         float x;
00081         float y;
00083         float z;
00085         float w;
00086 };
00087 
00091 struct Acceleration
00092 {
00094         float x;
00096         float y;
00098         float z;
00099 };
00100 
00104 struct QuaternionAcceleration
00105 {
00107         struct Quaternion quaternion;
00109         struct Acceleration acceleration;
00111         int garbageData;
00112 };
00116 struct Magnetic
00117 {
00119         float x;
00121         float y;
00123         float z;
00124 };
00125 
00128 struct QuaternionMagnetic
00129 {
00131         struct Quaternion quaternion;
00133         struct Magnetic magnetic;
00134 };
00135 
00138 struct AngularRate
00139 {
00141         float x;
00143         float y;
00145         float z;
00146 };
00147 
00150 struct QuaternionAngularRate
00151 {
00153         struct Quaternion quaternion;
00155         struct AngularRate angularRate;
00156 };
00157 
00160 struct QuaternionMagneticAcceleration
00161 {
00163         struct Quaternion quaternion;
00165         struct Magnetic magnetic;
00167         struct Acceleration acceleration;
00168 };
00169 
00172 struct QuaternionMagneticAccelerationAngularRate
00173 {
00175         struct Quaternion quaternion;
00177         struct Magnetic magnetic;
00179         struct Acceleration acceleration;
00181         struct AngularRate angularRate;
00183         int garbageData;
00184 };
00185 
00188 struct QuaternionAccelerationAngularRate
00189 {
00191         struct Quaternion quaternion;
00193         struct Acceleration acceleration;
00195         struct AngularRate angularRate;
00196 };
00197 
00200 struct Attitude
00201 {
00203         float yaw;
00205         float pitch;
00207         float roll;
00208 };
00209 
00212 struct DirectionalCosineMatrix
00213 {
00215         float R1C1;
00217         float R1C2;
00219         float R1C3;
00221         float R2C1;
00223         float R2C2;
00225         float R2C3;
00227         float R3C1;
00229         float R3C2;
00231         float R3C3;
00232 };
00233 
00236 struct MagneticAccelerationAngularRate
00237 {
00239         struct Magnetic magnetic;
00241         struct Acceleration acceleration;
00243         struct AngularRate angularRate;
00244 };
00245 
00248 struct MagneticGravityReference
00249 {
00251         float xMagRef;
00253         float yMagRef;
00255         float zMagRef;
00257         float xGravRef;
00259         float yGravRef;
00261         float zGravRef;
00262 };
00263 
00266 struct FilterMeasurementsVarianceParameters
00267 {
00269         float vAngularWalk;
00271         float vXAxisAngularRate;
00273         float vYAxisAngularRate;
00275         float vZAxisAngularRate;
00277         float vXAxisMagnetic;
00279         float vYAxisMagnetic;
00281         float vZAxisMagnetic;
00283         float vXAxisAcceleration;
00285         float vYAxisAcceleration;
00287         float vZAxisAcceleration;
00288 };
00289 
00292 struct MagneticHardSoftIronCompensationParameters
00293 {
00295         float S11;
00297         float S12;
00299         float S13;
00301         float S21;
00303         float S22;
00305         float S23;
00307         float S31;
00309         float S32;
00311         float S33;
00313         float H1;
00315         float H2;
00317         float H3;
00318 };
00319 
00322 struct FilterActiveTuningParameters
00323 {
00325         float magneticDisturbanceGain;
00327         float accelerationDisturbanceGain;
00329         float magneticDisturbanceMemory;
00331         float accelerationDisturbanceMemory;
00332 };
00333 
00336 struct AccelerometerCompensation
00337 {
00339         float C11;
00341         float C12;
00343         float C13;
00345         float C21;
00347         float C22;
00349         float C23;
00351         float C31;
00353         float C32;
00355         float C33;
00357         float B1;
00359         float B2;
00361         float B3;
00362 };
00363 
00366 struct ReferenceFrameRotation
00367 {
00369         float C11;
00371         float C12;
00373         float C13;
00375         float C21;
00377         float C22;
00379         float C23;
00381         float C31;
00383         float C32;
00385         float C33;
00386 };
00387 
00390 struct AttitudeMagneticAccelerationAngularRate
00391 {
00393         struct Attitude ypr;
00395         struct Magnetic magnetic;
00397         struct Acceleration acceleration;
00399         struct AngularRate angularRate;
00400 };
00401 
00404 union asyncDataList
00405 {
00407         struct QuaternionAcceleration quatAcc;
00409         struct QuaternionMagneticAccelerationAngularRate qmar;
00410 };
00411 
00416 union dataPacket
00417 {
00419         int hardwareRevisionRegister;
00421         int firmwareVersion;
00423         int baudRate;
00425         int asynchronousOutput;
00427         int frequency;
00429         struct ModelNumber model;
00431         struct SerialNumber serial;
00433         struct Attitude attitude;
00435         struct Quaternion quaternion;
00437         struct QuaternionMagnetic quatMag;
00439         struct QuaternionAcceleration quatAcc;
00441         struct QuaternionAngularRate quatAR;
00443         struct QuaternionMagneticAcceleration quatMagAcc;
00445         struct QuaternionAccelerationAngularRate quatAccAR;
00447         struct QuaternionMagneticAccelerationAngularRate quatMagAccAR; 
00449         struct DirectionalCosineMatrix dcm;
00451         struct Magnetic magnetic;
00453         struct Acceleration acceleration;
00455         struct AngularRate angularRate;
00457         struct MagneticAccelerationAngularRate magAccAR;
00459         struct MagneticGravityReference magGravRef;
00461         struct FilterMeasurementsVarianceParameters fmvp;
00463         struct MagneticHardSoftIronCompensationParameters mhsicp;
00465         struct FilterActiveTuningParameters fatp;
00467         struct AccelerometerCompensation accComp;
00469         struct ReferenceFrameRotation rfr;
00471         struct AttitudeMagneticAccelerationAngularRate attMagAccAR;
00472 };
00473 
00478 struct returnPacket
00479 {
00481         int returnValue;
00483         union dataPacket packet;
00484 };
00485 
00486 #endif
