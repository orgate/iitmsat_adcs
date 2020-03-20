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
00023 #ifndef PE_H_
00024 #define PE_H_
00025 
00026 #include <errno.h>
00027 #include <stdio.h>
00028 #include <string.h> // for strerror()
00029 #include <stdlib.h> // for exit()
00030 
00031 extern int PEReturnValue; 
00032 
00033 // The PE macro is for wrapping functions to detect whether a function call
00034 //      returns -1 to indicate an error.  If -1 is returned, an error message
00035 //  including strerror() is inserted on stderr.
00036 //  Program continues to execute
00037 #define PE(function) (((PEReturnValue = (function)) == -1) ? \
00038 ( \
00039 (fprintf(stderr, "\n\n**** Function invocation: " #function \
00040                                  "\n at line: %d" \
00041                                  "\n of file: " __FILE__ \
00042                                  "\n resulted in error: %s\n", __LINE__, strerror(errno))), -1) : PEReturnValue)
00043 
00044 // The PE_NOT macro is like PE but is intended for situations where a 
00045 // specific return value is required.
00046 #define PE_NOT(function, desiredRv) (((PEReturnValue = (function)) != desiredRv) ? \
00047         ( \
00048         (fprintf(stderr, "\n\n**** Function invocation: " #function \
00049                 "\n at line: %d" \
00050                 "\n of file: " __FILE__, __LINE__ )), \
00051         ((PEReturnValue == -1) ? \
00052                 (fprintf(stderr, "\n resulted in error: %s", strerror(errno))) : \
00053                 (fprintf(stderr, "\n returned: %d" \
00054                         "\n instead of: %d", (PEReturnValue), (desiredRv)))), \
00055         (fprintf(stderr, "\n")), \
00056         0,0) : \
00057         PEReturnValue)
00058 
00059 #endif /*PE_H_*/
