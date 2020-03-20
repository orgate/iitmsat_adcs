#include <windows.h>
#include <stdio.h>


int main()
{
SerialPort^ serialPort = gcnew SerialPort("COM22", 115200, Parity::None, 8, StopBits::One);
serialPort->Open();
serialPort->WriteLine("<-S->");
serialPort->Close();
getch();
return 0;
}
