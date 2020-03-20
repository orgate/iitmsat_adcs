/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "SysClock.h"
#include "VN_lib.h"
#include <stdio.h>
//#include "VN_lib.c"
//#include "VN_math.c"
//#include "VN_user.c"
//#include "VN100.c"
#include "VN_user.h"
#include "VN_math.h"
#include "VN_type.h"
#include "VN_lib.h"
#include "VN100.h"
/* Local includes ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DELAY							1e3

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


//global VN100_SPI_Packet VN_SPI_LastReceivedPacket = {0, 0, 0, 0, {0}};



 int main(void)
{
    printf("The program started\n");

    float yaw, pitch, roll;

    VN100_SPI_Packet VN_SPI_LastReceivedPacket = {0, 0, 0, 0, {0}};

    VN100_SPI_GetYPR(0,&yaw,&pitch,&roll);
    printf("%f %f %f",yaw,pitch,roll);

    printf("The program ended");
    return 0;
}
