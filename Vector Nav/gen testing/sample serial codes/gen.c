



#include<windows.h>
#include<stdio.h>

main()
{
HANDLE hSerial;
hSerial = CreateFile("COM22",
GENERIC_READ | GENERIC_WRITE,
0,
0,
OPEN_EXISTING,
FILE_ATTRIBUTE_NORMAL,
0);
if(hSerial==INVALID_HANDLE_VALUE){
if(GetLastError()==ERROR_FILE_NOT_FOUND){
//serial port does not exist. Inform user.
}
//some other error occurred. Inform user.
}




DCB dcbSerialParams = {0};
//dcbSerial.DCBlength=sizeof(dcbSerialParams);
dcbSerialParams.DCBlength=sizeof(dcbSerialParams);

if (!GetCommState(hSerial, &dcbSerialParams)) {
//error getting state
}
dcbSerialParams.BaudRate=CBR_115200;
dcbSerialParams.ByteSize=8;
dcbSerialParams.StopBits=ONESTOPBIT;
dcbSerialParams.Parity=NOPARITY;
if(!SetCommState(hSerial, &dcbSerialParams)){
//error setting serial port state
}



COMMTIMEOUTS timeouts={0};
timeouts.ReadIntervalTimeout=50;
timeouts.ReadTotalTimeoutConstant=50;
timeouts.ReadTotalTimeoutMultiplier=10;
timeouts.WriteTotalTimeoutConstant=50;
timeouts.WriteTotalTimeoutMultiplier=10;
if(!SetCommTimeouts(hSerial, &timeouts)){
//error occureed. Inform user
}
//int n=0;
char szBuff[9] = {'\0'};
DWORD dwBytesRead = 0;
if(!ReadFile(hSerial, szBuff, 8, &dwBytesRead, NULL)){
//error occurred. Report to user.
}

CloseHandle(hSerial);

printf("%s %s %s\n",ReadFile(hSerial, szBuff, 7, &dwBytesRead, NULL),szBuff,dwBytesRead);

char lastError[1024];
FormatMessage(
FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
NULL,
GetLastError(),
MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
lastError,
1024,
NULL);


printf("it worked");
getch();
}





















#include<stdio.h>
#include<conio.h>
#include<dos.h>

void main()
  {
    char data;
    int choice;
    clrscr();

  printf("Enter the choice to send or receive from COM1::");
  scanf("%d",&choice);

choice=2;
  if(choice==1)
    {
      printf("Enter data to send::");
      scanf("%c",data);
      inportb(0x03f8,data);
    }


  else
    {
data=outport(0x3f8);
printf("reding from  COM1::%d",data);

    }
getch();
}















int main(int argc, char *argv[])
{
   getch();
   DCB dcb;
   HANDLE hCom;
   BOOL fSuccess;
   char *pcCommPort = "COM22";

   hCom = CreateFile( pcCommPort,
                    GENERIC_READ | GENERIC_WRITE,
                    0,    // must be opened with exclusive-access
                    NULL, // no security attributes
                    OPEN_EXISTING, // must use OPEN_EXISTING
                    0,    // not overlapped I/O
                    NULL  // hTemplate must be NULL for comm devices
                    );

   getch();
   
  

   // Build on the current configuration, and skip setting the size
   // of the input and output buffers with SetupComm.

   fSuccess = GetCommState(hCom, &dcb);

   if (!fSuccess) 
   {
      // Handle the error.
      printf ("GetCommState failed with error %d.\n", GetLastError());
      getch();
      return (2);
   }

   getch();
   
   dcb.BaudRate = 115200;     // set the baud rate
   dcb.ByteSize = 8;             // data size, xmit, and rcv
   dcb.Parity = NOPARITY;        // no parity bit
   dcb.StopBits = ONESTOPBIT;    // one stop bit

   fSuccess = SetCommState(hCom, &dcb);

   if (!fSuccess) 
   {
      // Handle the error.
      printf ("SetCommState failed with error %d.\n", GetLastError());
      getch();
      return (3);
   }
   getch();
   
   printf ("Serial port %s successfully reconfigured.\n", pcCommPort);
   getch();
   return (0);
   
   
    
   if (hCom == INVALID_HANDLE_VALUE) 
   {
       // Handle the error.
       printf ("CreateFile failed with error %d.\n", GetLastError());
       getch();
       return (1);
   }
   
   getch();
   
}




















