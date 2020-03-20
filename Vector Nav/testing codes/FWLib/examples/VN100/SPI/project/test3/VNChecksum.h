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
00025 #ifndef VNCHECKSUM_H
00026 #define VNCHECKSUM_H
00027 
00037 unsigned char VNS_UART_calculateChecksum(char *command, int length);
00046 unsigned char VNS_UART_parseNibble(unsigned char c);
00057 unsigned char VNS_UART_parseHex(char A, char B);
00058 #endif
