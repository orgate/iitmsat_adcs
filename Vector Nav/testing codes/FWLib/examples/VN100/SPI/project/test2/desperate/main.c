/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "SysClock.h"
#include "VN_lib.h"
#include <stdio.h>
//#include "VN_lib.c"
//#include "VN_math.c"
//#include "VN_user.c"
//#include "VN100.c"
//#include "VN_user.h"
//#include "VN_math.h"
//#include "VN_type.h"
//#include "VN100.h"
/* Local includes ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DELAY							1e3

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


//global VN100_SPI_Packet VN_SPI_LastReceivedPacket = {0, 0, 0, 0, {0}};
















/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN100.c
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file provides all of the firmware functions specific
*                    : to the VN100.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "VN100.h"
#include "VN_lib.h"

#ifdef _VN100
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Buffer used for SPI read and write responses */
/* Both the read and write register SPI routines below use this packet 
   to store the returned SPI response. None of the write register commands 
   implemented in this library check the data that is returned by the sensor 
   to ensure that it is consistent with the data that was sent.  For normal
   cases this isn't necessary however if you wish to implement your own
   checking then this is the structure that you need to check after each 
   register set command.  The structure has the following form:
   VN_SPI_LastReceivedPacket.CmdID -> This is the ID for the command that
                                   the response is for
   VN_SPI_LastReceivedPacket.RegID -> This is the ID for the register that
                                   the response is for
   VN_SPI_LastReceivedPacket.Data[] -> This is the data that was returned by
                                    the sensor as an array of unsigned 32-bit
                                    integers  */
VN100_SPI_Packet VN_SPI_LastReceivedPacket = {0, 0, 0, 0, {0}};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : VN100_SPI_ReadRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth)
* Description    : Read the register with the ID regID on a VN-100 sensor
*                  using the SPI interface.                                     
* Input          : sensorID -> The sensor to get the requested data from.
*                : regID -> The requested register ID number
*                : regWidth -> The width of the requested register in 32-bit words
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_ReadRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth){

  unsigned long i;

  /* Pull SS line low to start transaction*/
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Send request */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, regID, VN100_CmdID_ReadRegister));
  VN_SPI_SendReceive(0);

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Delay for 50us */
  VN_Delay(100);

  /* Pull SS line low to start SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Get response over SPI */
  for(i=0;i<=regWidth;i++){
    *(((unsigned long*)&VN_SPI_LastReceivedPacket) + i) = VN_SPI_SendReceive(0);
  }

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);  


  /* Return Error code */
  return &VN_SPI_LastReceivedPacket;  
}

/*******************************************************************************
* Function Name  : VN100_SPI_WriteRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth, unsigned long* ptrWriteValues)
* Description    : Write to the register with the ID regID on VN-100 sensor
*                  using the SPI interface.                                        
* Input          : sensorID -> The sensor to write the requested data to.
*                : regID -> The register ID number
*                : regWidth -> The width of the register in 32-bit words
* Output         : ptrWriteValues -> The data to write to the requested register.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_WriteRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth, unsigned long* ptrWriteValues){

  unsigned long i;

  /* Pull SS line low to start transaction*/
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Send write command */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, regID, VN100_CmdID_WriteRegister));
  for(i=0;i<regWidth;i++){
    VN_SPI_SendReceive(ptrWriteValues[i]);
  }

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Delay for 50us */
  VN_Delay(100);

  /* Pull SS line low to start SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Get response over SPI */
  for(i=0;i<4;i++){
    *(((unsigned long*)&VN_SPI_LastReceivedPacket) + i) = VN_SPI_SendReceive(0);
  }

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);  


  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetModel(unsigned char sensorID, char* model)
* Description    : Read the model number from the sensor.                                       
* Input          : sensorID -> The sensor to get the model number from.
* Output         : model -> Pointer to a character array where the requested
*                           model number is placed. This needs to be a character
*                           array that is 12 characters in size.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetModel(unsigned char sensorID, char* model){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_MODEL, 3);

  /* Get model number */
  for(i=0;i<3;i++){
    *((unsigned long*)model + i) = VN_SPI_LastReceivedPacket.Data[i].UInt;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetHWRev(unsigned char sensorID, unsigned long* revision)
* Description    : Get the hardware revision for the sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : revision -> The hardware revision requested.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetHWRev(unsigned char sensorID, unsigned long* revision){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_HWREV, 1);  
  
  /* Get hardware revision */
  *revision = VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetSerial(unsigned char sensorID, unsigned long* serialNumber)
* Description    : Get the serial number from the requested sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : serialNumber -> The serial number returned by the sensor.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetSerial(unsigned char sensorID, unsigned long* serialNumber){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_SN, 3);  
  
  /* Get model number */
  for(i=0;i<3;i++){
    *(serialNumber + i) = VN_SPI_LastReceivedPacket.Data[i].UInt;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetFWVer(unsigned char sensorID, unsigned long* firmwareVersion)
* Description    : Get the firmware version from the requested sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : firmwareVersion -> The firmware version returned.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetFWVer(unsigned char sensorID, unsigned long* firmwareVersion){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_FWVER, 1);  
  
  /* Get hardware revision */
  *firmwareVersion = VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetBaudRate(unsigned char sensorID, VN100_BaudType baudRate)
* Description    : Get the serial baud rate from the requested sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : baudRate -> The baud rate returned by the sensor.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetBaudRate(unsigned char sensorID, VN100_BaudType* baudRate){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_SBAUD, 1);  
  
  /* Get hardware revision */
  *baudRate = (VN100_BaudType)VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetBaudRate(unsigned char sensorID, VN100_BaudType baudRate)
* Description    : Set the serial baud rate for the requested sensor.                                        
* Input          : sensorID -> The sensor to set.
* Output         : baudRate -> The baud rate to set on the sensor.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetBaudRate(unsigned char sensorID, VN100_BaudType baudRate){

  unsigned long regValue = (unsigned long)baudRate;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_SBAUD, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetADOR(unsigned char sensorID, VN100_ADORType ADOR)
* Description    : Get the ADOR register value from the requested sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : ADOR -> The value returned for the ADOR register.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetADOR(unsigned char sensorID, VN100_ADORType* ADOR){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ADOR, 1);  
  
  /* Get hardware revision */
  *ADOR = (VN100_ADORType)VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetADOR(unsigned char sensorID, VN100_ADORType ADOR)
* Description    : Set the ADOR register value from the requested sensor.                                
* Input          : sensorID -> The sensor to set.
* Output         : ADOR -> The value to set the ADOR register to.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetADOR(unsigned char sensorID, VN100_ADORType ADOR){

  unsigned long regValue = (unsigned long)ADOR;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ADOR, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetADOF(unsigned char sensorID, VN100_ADOFType ADOF)
* Description    : Get the async data output frequency.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : ADOR -> The frequency returned for the ADOF register.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetADOF(unsigned char sensorID, VN100_ADOFType* ADOF){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ADOF, 1);  
  
  /* Get hardware revision */
  *ADOF = (VN100_ADOFType)VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetADOF(unsigned char sensorID, VN100_ADOFType ADOF)
* Description    : Set the async data output frequency.
* Input          : sensorID -> The sensor to set.
* Output         : ADOR -> The desired frequency of the async data output.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetADOF(unsigned char sensorID, VN100_ADOFType ADOF){

  unsigned long regValue = (unsigned long)ADOF;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ADOR, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetYPR(unsigned char sensorID, float yaw, float pitch, float roll)
* Description    : Get the measured yaw, pitch, roll orientation angles.                                        
* Input          : sensorID -> The sensor to set.
* Output         : yaw -> The yaw angle measured in degrees.
*                  pitch -> The pitch angle measured in degrees.
*                  roll -> The roll angle measured in degrees.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetYPR(unsigned char sensorID, float* yaw, float* pitch, float* roll){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_YPR, 3);
  
  /* Get Yaw, Pitch, Roll */
  *yaw   = VN_SPI_LastReceivedPacket.Data[0].Float;
  *pitch = VN_SPI_LastReceivedPacket.Data[1].Float;
  *roll  = VN_SPI_LastReceivedPacket.Data[2].Float;
  
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuat(unsigned char sensorID, float* q)
* Description    : Get the measured attitude quaternion. The quaternion is a 4x1
*                  vector unit vector with the fourth term q[3] as the scalar
*                  term.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> The address of the location to write the returned
*                       measured quaternion (4x1). 
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuat(unsigned char sensorID, float* q){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTN, 4);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatMag(unsigned char sensorID, float* q, float* mag)
* Description    : Get the measured attitude quaternion and magnetic vector. The
*                  quaternion is a 4x1 unit vector with the fourth term q[3] as
*                  the scalar term. The magnetic is a 3x1 vector.  The measured
*                  magnetic vector does not have any usable units.  The magnetic
*                  vector is calibrated at the factory to have a magnitude of
*                  one on the XY plane.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> The address of the location to write the returned
*                       measured quaternion (4x1).
*                  mag -> The magnetic measured vector (3x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatMag(unsigned char sensorID, float* q, float* mag){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTM, 7);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatAcc(unsigned char sensorID, float* q, float* acc)
* Description    : Get the measured attitude quaternion and acceleration vector.
*                  The quaternion is a 4x1 unit vector with the fourth term q[3]
*                  as the scalar term.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatAcc(unsigned char sensorID, float* q, float* Acc){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTA, 7);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatRates(unsigned char sensorID, float* q, float* rates)
* Description    : Get the measured attitude quaternion and angular rates.                                       
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatRates(unsigned char sensorID, float* q, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTR, 7);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatMagAcc(unsigned char sensorID, float* q, float* mag, float* acc)
* Description    : Get the measured attitude quaternion, magnetic and acceleration.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  mag -> The magnetic measured vector (3x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatMagAcc(unsigned char sensorID, float* q, float* mag, float* Acc){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QMA, 10);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i+7].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatAccRates(unsigned char sensorID, float* q, float* acc, float* rates)
* Description    : Get the measured attitude quaternion, acceleration, and angular rates.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatAccRates(unsigned char sensorID, float* q, float* acc, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QAR, 10);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    acc[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+7].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatMagAccRates(unsigned char sensorID, float* q, float* mag, float* acc, float* rates)
* Description    : Get the measured attitude quaternion, magnetic, acceleration, and angular rates.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  mag -> The magnetic measured vector (3x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatMagAccRates(unsigned char sensorID, float* q, float* mag, float* acc, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QMR, 13);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }  
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    acc[i] = VN_SPI_LastReceivedPacket.Data[i+7].Float;
  }
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+10].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetYPRMagAccRates(unsigned char sensorID, float* YPR, float* mag, float* acc, float* rates)
* Description    : Get the yaw, pitch, roll, magnetic, acceleration, and angular rates.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : YPR -> Euler angles (Yaw, Pitch, Roll) in deg.
*                  mag -> The magnetic measured vector (3x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetYPRMagAccRates(unsigned char sensorID, float* YPR, float* mag, float* acc, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_YMR, 12);
  
  /* Get Euler angles */
  for(i=0;i<3;i++){
    YPR[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+3].Float;
  }  
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    acc[i] = VN_SPI_LastReceivedPacket.Data[i+6].Float;
  }
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+9].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetDCM(unsigned char sensorID, float* DCM)
* Description    : Get the measured attitude as a directional cosine matrix.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : DCM -> Directional Cosine Matrix (9x1). The order of the terms
*                         in the matrix is {first row, second row, third row}.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetDCM(unsigned char sensorID, float **DCM){

  unsigned long i,j;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_DCM, 9);
  
  /* Get Directional Cosine Matrix */
  for(i=0;i<3;i++){
    for(j=0;j<3;j++){
      DCM[i][j] = VN_SPI_LastReceivedPacket.Data[i*3+j].Float;
    }
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMag(unsigned char sensorID, float* mag)
* Description    : Get the measured magnetic field. The measured magnetic field
*                  does not have any usable units.  The magnetic vector is
*                  calibrated at the factory to have a magnitude of one on the
*                  XY plane.                                                
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : mag -> The magnetic measured vector (3x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetMag(unsigned char sensorID, float* mag){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_MAG, 3);
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAcc(unsigned char sensorID, float* Acc)
* Description    : Get the measured acceleration. The measured acceleration has
*                  the units of m/s^2 and its range is dependent upon the gain
*                  set by the VN100_SPI_SetAccGain() function.                                                
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : Acc -> The measured acceleration (3x1) in m/s^2.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetAcc(unsigned char sensorID, float* Acc){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ACC, 3);
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetRates(unsigned char sensorID, float* rates)
* Description    : Get the measured angular rates. The measured angular rates
*                  have units of rad/s. This is the filtered angular rate and is
*                  compensated by the onboard Kalman filter to account for gyro
*                  bias drift.                                                
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : rates -> The measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetRates(unsigned char sensorID, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_GYR, 3);
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMagAccRates(unsigned char sensorID, float* mag, float* Acc, float* rates)
* Description    : Get the measured magnetic, acceleration, and angular rates.
*                  The measurements are taken in the body reference frame.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : mag -> Measured magnetic field (3x1) [Non-dimensional].
*                  Acc -> Measured acceleration (3x1) [m/s^2].
*                  rates -> Measured angular rates (3x1) [rad/s].
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetMagAccRates(unsigned char sensorID, float* mag, float* Acc, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_MAR, 9);
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i+3].Float;
  }    
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+6].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc)
* Description    : Get the magnetic and acceleration reference vectors. The
*                  reference vectors are the vectors measured by the magnetomter
*                  and Accerometer respectively in the inertial reference
*                  frame.  The inertial reference frame is NED (North, East, Down).                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : refMag -> The reference vector for the magnetic field.
*                  refAcc -> The reference vector for the Accerometer (gravity).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_REF, 6);
  
  /* Get magnetic reference */
  for(i=0;i<3;i++){
    refMag[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get acceleration reference */
  for(i=0;i<3;i++){
    refAcc[i] = VN_SPI_LastReceivedPacket.Data[i+3].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetMagAccReference(unsigned char sensorID, float* refMag, float* refAcc)
* Description    : Set the magnetic and acceleration reference vectors. The
*                  reference vectors are the vectors measured by the magnetometer
*                  and accelerometer respectively in the inertial reference
*                  frame.  The inertial reference frame is NED (North, East, Down).                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : refMag -> The reference vector for the magnetic field.
*                  refAcc -> The reference vector for the Accelerometer (gravity).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc){

  float ref[6];
  
  ref[0] = refMag[0];
  ref[1] = refMag[1];
  ref[2] = refMag[2];
  ref[3] = refAcc[0];
  ref[4] = refAcc[1];
  ref[5] = refAcc[2];

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_REF, 6, (unsigned long*)ref);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetFiltMeasVar(unsigned char sensorID, float* measVar)
* Description    : Get the Kalman filter measurement variance parameters. This is
*                  discussed in the User Manual in Section 6.22. The measurement
*                  variance parameters controls how much weight the Kalman filter
*                  will place on each measurement.  See application note A001 for
*                  more details on how to set these values for your specific
*                  application.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : measVar -> The variance on the measured inputs to the
*                             filter. This is a (10x1) vector.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetFiltMeasVar(unsigned char sensorID, float* measVar){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_SIG, 10);
  
  /* Get filter measurement variance */
  for(i=0;i<10;i++){
    measVar[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetFiltMeasVar(unsigned char sensorID, float* measVar)
* Description    : Set the Kalman filter measurement variance parameters. This is
*                  discussed in the User Manual in Section 6.22. The measurement
*                  variance parameters controls how much weight the Kalman filter
*                  will place on each measurement.  See application note A001 for
*                  more details on how to set these values for your specific
*                  application.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : measVar -> The variance on the measured inputs to the
*                                  filter. This is a (10x1) vector.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetFiltMeasVar(unsigned char sensorID, float* measVar){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_SIG, 10, (unsigned long*)measVar);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetHardSoftIronComp(unsigned char sensorID, float* HSI)
* Description    : Get the magnetic hard/soft iron compensation parameters. These
*                  values allow the magnetometer to compensate for distortions in
*                  the local magnetic field due to ferromagnetic materials in the
*                  vacinity of the sensor. More information on the parameters can
*                  be found in the User Manual in Section 6.23.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : HSI -> magnetic hard/soft iron paramteters (12x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetHardSoftIronComp(unsigned char sensorID, float* HSI){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_HSI, 12);
  
  /* Get magnetic hard/soft iron compensation parameters */
  for(i=0;i<12;i++){
    HSI[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetHardSoftIronComp(unsigned char sensorID, float* HSI)
* Description    : Set the magnetic hard/soft iron compensation parameters. These
*                  values allow the magnetometer to compensate for distortions in
*                  the local magnetic field due to ferromagnetic materials in the
*                  vacinity of the sensor. More information on the parameters can
*                  be found in the User Manual in Section 6.23.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : HSI -> magnetic hard/soft iron parameters (12x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetHardSoftIronComp(unsigned char sensorID, float* HSI){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_HSI, 12, (unsigned long*)HSI);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA)
* Description    : Get the filter active tuning parameters. The active tuning
*                  parameters control how the filter handles dynamic disturbances
*                  in both magnetic and acceleration.  These values are not needed
*                  for normal operation.  More on these parameters can be found in
*                  the User Manual in Section 6.24.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : gainM -> Magnetic Disturbance Gain
*                  gainA -> Acceleration Disturbance Gain
*                  memM -> Magnetic Disturbance Memory
*                  memA -> Acceleration Disturbance Gain
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetFiltActTuning(unsigned char sensorID, float* gainM, float* gainA, float* memM, float* memA){
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ATP, 6);
  
  /* Get magnetic gain */
  *gainM = VN_SPI_LastReceivedPacket.Data[0].Float;
  
  /* Get acceleration gain */
  *gainA = VN_SPI_LastReceivedPacket.Data[3].Float;
  
  /* Get magnetic memory */
  *memM = VN_SPI_LastReceivedPacket.Data[6].Float;
  
  /* Get acceleration memory */
  *memA = VN_SPI_LastReceivedPacket.Data[9].Float;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA)
* Description    : Set the filter active tuning parameters. The active tuning
*                  parameters control how the filter handles dynamic disturbances
*                  in both magnetic and acceleration.  These values are not needed
*                  for normal operation.  More on these parameters can be found in
*                  the User Manual in Section 6.24.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : gainM -> Magnetic Disturbance Gain
*                  gainA -> Acceleration Disturbance Gain
*                  memM -> Magnetic Disturbance Memory
*                  memA -> Acceleration Disturbance Gain
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA){

  float atp[4];
  
  atp[0] = gainM;
  atp[1] = gainA;
  atp[2] = memM;
  atp[3] = memA;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ATP, 6, (unsigned long*)atp);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAccComp(unsigned char sensorID, float* AccComp)
* Description    : Get the accelerometer compensation parameters. The purpose of
*                  these parameters are explained in Section 6.25 of the User
*                  Manual. These parameters are not required for normal operation.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : AccComp -> Acceleration compensation register values.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetAccComp(unsigned char sensorID, float* AccComp){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ACT, 12);
  
  /* Get accelerometer compensation parameters */
  for(i=0;i<12;i++){
    AccComp[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetAccComp(unsigned char sensorID, float* AccComp)
* Description    : Set the accelerometer compensation parameters. The purpose of
*                  these parameters is explained in Section 6.25 of the User
*                  Manual. These parameters are not required for normal operation.                                        
* Input          : sensorID -> The sensor to get the requested data from.
                   AccComp -> Acceleration compensation register values.
* Output:        : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetAccComp(unsigned char sensorID, float* AccComp){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ACT, 12, (unsigned long*)AccComp);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetRefFrameRot(unsigned char sensorID, float* refFrameRot)
* Description    : Get the reference frame rotation matrix. This matrix allows
*                  the user to transform all measured vectors from the body
*                  reference frame of the VN-100, to any other rigidly attached
*                  coordinate frame. The effect of this transformation is that
*                  the computed attitude solution and measured measurement
*                  vectors will now be measured in the chosen coordinate system
*                  of the user and not the VN-100 coordinate system.  This is
*                  further explained in Section 6.26 of the User Manual.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : refFrameRot -> Reference frame rotation matrix (9x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetRefFrameRot(unsigned char sensorID, float* refFrameRot){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_RFR, 12);
  
  /* Get reference frame rotation parameters */
  for(i=0;i<12;i++){
    refFrameRot[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetRefFrameRot(unsigned char sensorID, float* refFrameRot)
* Description    : Set the reference frame rotation matrix. This matrix allows
*                  the user to transform all measured vectors from the body
*                  reference frame of the VN-100, to any other rigidly attached
*                  coordinate frame. The effect of this transformation is that
*                  the computed attitude solution and measured measurement
*                  vectors will now be measured in the chosen coordinate system
*                  of the user and not the VN-100 coordinate system.  This is
*                  further explained in Section 6.26 of the User Manual.                                        
* Input          : sensorID -> The sensor to get the requested data from.
*                  refFrameRot -> Reference frame rotation matrix (9x1).
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetRefFrameRot(unsigned char sensorID, float* refFrameRot){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_RFR, 12, (unsigned long*)refFrameRot);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAccGain(unsigned char sensorID, VN100_AccGainType gain)
* Description    : Get the current accelerometer gain setting. The accelerometer
*                  on the VN-100 can be set to either a +/- 2g or +/- 6g gain
*                  setting.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : gain -> The current accelerometer gain setting.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetAccGain(unsigned char sensorID, VN100_AccGainType* gain){
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ACG, 1);
  
  /* Get accelerometer gain */
  *gain = (VN100_AccGainType)VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetAccGain(unsigned char sensorID, VN100_AccGainType gain)
* Description    : Set the current accelerometer gain setting. The accelerometer
*                  on the VN-100 can be set to either a +/- 2g or +/- 6g gain
*                  setting.
* Input          : sensorID -> The sensor to get the requested data from.
*                : gain -> The current accelerometer gain setting.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetAccGain(unsigned char sensorID, VN100_AccGainType gain){

  unsigned long regValue = (unsigned long)gain;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ACG, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_RestoreFactoryDefaultSettings(unsigned char sensorID)
* Description    : Restore the selected sensor to factory default state. The
*                  values for factory default state for each register can be
*                  found in Section 7 of the User Manual.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_RestoreFactorySettings(unsigned char sensorID){
  
  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_RestoreFactorySettings));
  VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  
  /* Delay for 50 uS */
  VN_Delay(50);
  
  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Get response bytes */
  *((unsigned long*)&VN_SPI_LastReceivedPacket    ) = VN_SPI_SendReceive(0);
  *((unsigned long*)&VN_SPI_LastReceivedPacket + 1) = VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_Tare(unsigned char sensorID)
* Description    : Send a tare command to the selected VN-100. The tare command
*                  will zero out the current sensor orientation.  The attitude
*                  of the sensor will be measured form this point onwards with
*                  respect to the attitude present when the tare command was
*                  issued.  It is important with v4 of the firmware to keep
*                  the device still for at least 3 seconds after performing a
*                  tare command.  The tare command will also set the reference
*                  vectors in the inertial frame to the vectors currently
*                  measured in the body frame.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_Tare(unsigned char sensorID){
  
  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_Tare));
  VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  
  /* Delay for 50 uS */
  VN_Delay(50);
  
  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Get response bytes */
  *((unsigned long*)&VN_SPI_LastReceivedPacket    ) = VN_SPI_SendReceive(0);
  *((unsigned long*)&VN_SPI_LastReceivedPacket + 1) = VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_Reset(unsigned char sensorID)
* Description    : Command the given sensor to perform a device hardware reset.
*                  This is equivalent to pulling the NRST pin low on the VN-100.
*                  Any changes to any of the registers on the VN-100 that were
*                  made since last issuing a Write Settings commands will be lost.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_Reset(unsigned char sensorID){

  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_Reset));
  VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAccInertial(unsigned char sensorID, float *AccI)
* Description    : Request the inertial acceleration from the VN-100. This
*                  function will internally request both the measured acceleration
*                  and attitude from the sensor, then compute the inertial
*                  acceleration.  If you are wanting to integrate your acceleration
*                  to find velocity or position, then this is the acceleration
*                  that you want to measure. It is measured in a fixed
*                  NED (North, East, Down) coordinate frame.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : AccI -> The inertial acceleration measured by the device.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_GetAccInertial(unsigned char sensorID, float *AccI){

  /* Create a matrix for the attitude */
#if __STDC_VERSION__ >= 199901L  
  VN_CreateMatrix(A, 3, 3, {0.0});
#else
  static float A_data[9] = {0.0};
  static float *A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
  static float **A = A_ptr;
#endif

  /* Attitude quaternion */
  float q[4];
  
  /* Body acceleration vector */
  float AccB[3];
  
  /* Get the attitude quaternion and acceleration from VN-100 */
  VN100_SPI_GetQuatAcc(sensorID, q, AccB);
  
  /* Convert the quaternion into a directional cosine matrix */
  VN_Quat2DCM(q, A);
  
  /* Multiply transpose of DCM by body acceleration to get inertial acceleration */
  VN_MatTVecMult(A, AccB, 3, 3, AccI);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMagInertial(unsigned char sensorID, float *MagI)
* Description    : Request the inertial magnetic measurement from the VN-100. This
*                  function will internally request both the measured magnetic
*                  and attitude from the sensor, then compute the inertial
*                  magnetic measurement.  It is measured in a fixed
*                  NED (North, East, Down) coordinate frame.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : MagI -> The inertial magnetic measurement measured by the
*                : device.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_GetMagInertial(unsigned char sensorID, float *MagI){

  /* Create a matrix for the attitude */
#if __STDC_VERSION__ >= 199901L  
  VN_CreateMatrix(A, 3, 3, {0.0});
#else
  static float A_data[9] = {0.0};
  static float *A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
  static float **A = A_ptr;
#endif

  /* Attitude quaternion */
  float q[4];
  
  /* Body magnetic vector */
  float MagB[3];
  
  /* Get the attitude quaternion and magnetic from VN-100 */
  VN100_SPI_GetQuatMag(sensorID, q, MagB);
  
  /* Convert the quaternion into a directional cosine matrix */
  VN_Quat2DCM(q, A);
  
  /* Multiply transpose of DCM by body magnetic to get inertial magnetic */
  VN_MatTVecMult(A, MagB, 3, 3, MagI);
}

#endif /* _VN100 */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/









/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_adc.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the ADC firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ADC DISCNUM mask */
#define CR1_DISCNUM_Reset           ((u32)0xFFFF1FFF)

/* ADC DISCEN mask */
#define CR1_DISCEN_Set              ((u32)0x00000800)
#define CR1_DISCEN_Reset            ((u32)0xFFFFF7FF)

/* ADC JAUTO mask */
#define CR1_JAUTO_Set               ((u32)0x00000400)
#define CR1_JAUTO_Reset             ((u32)0xFFFFFBFF)

/* ADC JDISCEN mask */
#define CR1_JDISCEN_Set             ((u32)0x00001000)
#define CR1_JDISCEN_Reset           ((u32)0xFFFFEFFF)

/* ADC AWDCH mask */
#define CR1_AWDCH_
             ((u32)0xFFFFFFE0)

/* ADC Analog watchdog enable mode mask */
#define CR1_AWDMode_Reset           ((u32)0xFF3FFDFF)

/* CR1 register Mask */
#define CR1_CLEAR_Mask              ((u32)0xFFF0FEFF)

/* ADC ADON mask */
#define CR2_ADON_Set                ((u32)0x00000001)
#define CR2_ADON_Reset              ((u32)0xFFFFFFFE)

/* ADC DMA mask */
#define CR2_DMA_Set                 ((u32)0x00000100)
#define CR2_DMA_Reset               ((u32)0xFFFFFEFF)

/* ADC RSTCAL mask */
#define CR2_RSTCAL_Set              ((u32)0x00000008)

/* ADC CAL mask */
#define CR2_CAL_Set                 ((u32)0x00000004)

/* ADC SWSTART mask */
#define CR2_SWSTART_Set             ((u32)0x00400000)

/* ADC EXTTRIG mask */
#define CR2_EXTTRIG_Set             ((u32)0x00100000)
#define CR2_EXTTRIG_Reset           ((u32)0xFFEFFFFF)

/* ADC Software start mask */
#define CR2_EXTTRIG_SWSTART_Set     ((u32)0x00500000)
#define CR2_EXTTRIG_SWSTART_Reset   ((u32)0xFFAFFFFF)

/* ADC JEXTSEL mask */
#define CR2_JEXTSEL_Reset           ((u32)0xFFFF8FFF)

/* ADC JEXTTRIG mask */
#define CR2_JEXTTRIG_Set            ((u32)0x00008000)
#define CR2_JEXTTRIG_Reset          ((u32)0xFFFF7FFF)

/* ADC JSWSTART mask */
#define CR2_JSWSTART_Set            ((u32)0x00200000)

/* ADC injected software start mask */
#define CR2_JEXTTRIG_JSWSTART_Set   ((u32)0x00208000)
#define CR2_JEXTTRIG_JSWSTART_Reset ((u32)0xFFDF7FFF)

/* ADC TSPD mask */
#define CR2_TSVREFE_Set             ((u32)0x00800000)
#define CR2_TSVREFE_Reset           ((u32)0xFF7FFFFF)

/* CR2 register Mask */
#define CR2_CLEAR_Mask              ((u32)0xFFF1F7FD)

/* ADC SQx mask */
#define SQR3_SQ_Set                 ((u32)0x0000001F)
#define SQR2_SQ_Set                 ((u32)0x0000001F)
#define SQR1_SQ_Set                 ((u32)0x0000001F)

/* SQR1 register Mask */
#define SQR1_CLEAR_Mask             ((u32)0xFF0FFFFF)

/* ADC JSQx mask */
#define JSQR_JSQ_Set                ((u32)0x0000001F)

/* ADC JL mask */
#define JSQR_JL_Set                 ((u32)0x00300000)
#define JSQR_JL_Reset               ((u32)0xFFCFFFFF)

/* ADC SMPx mask */
#define SMPR1_SMP_Set               ((u32)0x00000007)
#define SMPR2_SMP_Set               ((u32)0x00000007)

/* ADC JDRx registers offset */
#define JDR_Offset                  ((u8)0x28)

/* ADC1 DR register base address */
#define DR_ADDRESS                  ((u32)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : ADC_DeInit
* Description    : Deinitializes the ADCx peripheral registers to their default
*                  reset values.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_DeInit(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  switch (*(u32*)&ADCx)
  {
    case ADC1_BASE:
      /* Enable ADC1 reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
      /* Release ADC1 from reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);
      break;
    
    case ADC2_BASE:
      /* Enable ADC2 reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2, ENABLE);
      /* Release ADC2 from reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2, DISABLE);
      break;
      
    case ADC3_BASE:
      /* Enable ADC3 reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3, ENABLE);
      /* Release ADC3 from reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3, DISABLE);
      break; 

    default:
      break;
  }
}

/*******************************************************************************
* Function Name  : ADC_Init
* Description    : Initializes the ADCx peripheral according to the specified parameters
*                  in the ADC_InitStruct.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_InitStruct: pointer to an ADC_InitTypeDef structure that
*                    contains the configuration information for the specified
*                    ADC peripheral.
* Output         : None
* Return         : None
******************************************************************************/
void ADC_Init(ADC_TypeDef* ADCx, ADC_InitTypeDef* ADC_InitStruct)
{
  u32 tmpreg1 = 0;
  u8 tmpreg2 = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_MODE(ADC_InitStruct->ADC_Mode));
  assert_param(IS_FUNCTIONAL_STATE(ADC_InitStruct->ADC_ScanConvMode));
  assert_param(IS_FUNCTIONAL_STATE(ADC_InitStruct->ADC_ContinuousConvMode));  		    
  assert_param(IS_ADC_EXT_TRIG(ADC_InitStruct->ADC_ExternalTrigConv));   
  assert_param(IS_ADC_DATA_ALIGN(ADC_InitStruct->ADC_DataAlign)); 
  assert_param(IS_ADC_REGULAR_LENGTH(ADC_InitStruct->ADC_NbrOfChannel));

  /*---------------------------- ADCx CR1 Configuration -----------------*/
  /* Get the ADCx CR1 value */
  tmpreg1 = ADCx->CR1;
  /* Clear DUALMOD and SCAN bits */
  tmpreg1 &= CR1_CLEAR_Mask;
  /* Configure ADCx: Dual mode and scan conversion mode */
  /* Set DUALMOD bits according to ADC_Mode value */
  /* Set SCAN bit according to ADC_ScanConvMode value */
  tmpreg1 |= (u32)(ADC_InitStruct->ADC_Mode | ((u32)ADC_InitStruct->ADC_ScanConvMode << 8));
  /* Write to ADCx CR1 */
  ADCx->CR1 = tmpreg1;

  /*---------------------------- ADCx CR2 Configuration -----------------*/
  /* Get the ADCx CR2 value */
  tmpreg1 = ADCx->CR2;
  /* Clear CONT, ALIGN and EXTSEL bits */
  tmpreg1 &= CR2_CLEAR_Mask;
  /* Configure ADCx: external trigger event and continuous conversion mode */
  /* Set ALIGN bit according to ADC_DataAlign value */
  /* Set EXTSEL bits according to ADC_ExternalTrigConv value */
  /* Set CONT bit according to ADC_ContinuousConvMode value */
  tmpreg1 |= (u32)(ADC_InitStruct->ADC_DataAlign | ADC_InitStruct->ADC_ExternalTrigConv |
            ((u32)ADC_InitStruct->ADC_ContinuousConvMode << 1));
  /* Write to ADCx CR2 */
  ADCx->CR2 = tmpreg1;

  /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
  tmpreg1 = ADCx->SQR1;
  /* Clear L bits */
  tmpreg1 &= SQR1_CLEAR_Mask;
  /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfChannel value */
  tmpreg2 |= (ADC_InitStruct->ADC_NbrOfChannel - 1);
  tmpreg1 |= ((u32)tmpreg2 << 20);
  /* Write to ADCx SQR1 */
  ADCx->SQR1 = tmpreg1;
}

/*******************************************************************************
* Function Name  : ADC_StructInit
* Description    : Fills each ADC_InitStruct member with its default value.
* Input          : ADC_InitStruct : pointer to an ADC_InitTypeDef structure
*                  which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct)
{
  /* Reset ADC init structure parameters values */
  /* Initialize the ADC_Mode member */
  ADC_InitStruct->ADC_Mode = ADC_Mode_Independent;

  /* initialize the ADC_ScanConvMode member */
  ADC_InitStruct->ADC_ScanConvMode = DISABLE;

  /* Initialize the ADC_ContinuousConvMode member */
  ADC_InitStruct->ADC_ContinuousConvMode = DISABLE;

  /* Initialize the ADC_ExternalTrigConv member */
  ADC_InitStruct->ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;

  /* Initialize the ADC_DataAlign member */
  ADC_InitStruct->ADC_DataAlign = ADC_DataAlign_Right;

  /* Initialize the ADC_NbrOfChannel member */
  ADC_InitStruct->ADC_NbrOfChannel = 1;
}

/*******************************************************************************
* Function Name  : ADC_Cmd
* Description    : Enables or disables the specified ADC peripheral.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - NewState: new state of the ADCx peripheral. This parameter
*                    can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_Cmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the ADON bit to wake up the ADC from power down mode */
    ADCx->CR2 |= CR2_ADON_Set;
  }
  else
  {
    /* Disable the selected ADC peripheral */
    ADCx->CR2 &= CR2_ADON_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_DMACmd
* Description    : Enables or disables the specified ADC DMA request.
* Input          : - ADCx: where x can be 1 or 3 to select the ADC peripheral.
*                    Note: ADC2 hasn't a DMA capability.
*                  - NewState: new state of the selected ADC DMA transfer.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_DMACmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_DMA_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC DMA request */
    ADCx->CR2 |= CR2_DMA_Set;
  }
  else
  {
    /* Disable the selected ADC DMA request */
    ADCx->CR2 &= CR2_DMA_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_ITConfig
* Description    : Enables or disables the specified ADC interrupts.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_IT: specifies the ADC interrupt sources to be enabled
*                    or disabled. 
*                    This parameter can be any combination of the following values:
*                       - ADC_IT_EOC: End of conversion interrupt mask
*                       - ADC_IT_AWD: Analog watchdog interrupt mask
*                       - ADC_IT_JEOC: End of injected conversion interrupt mask
*                  - NewState: new state of the specified ADC interrupts.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_ITConfig(ADC_TypeDef* ADCx, u16 ADC_IT, FunctionalState NewState)
{
  u8 itmask = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_ADC_IT(ADC_IT));

  /* Get the ADC IT index */
  itmask = (u8)ADC_IT;

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC interrupts */
    ADCx->CR1 |= itmask;
  }
  else
  {
    /* Disable the selected ADC interrupts */
    ADCx->CR1 &= (~(u32)itmask);
  }
}

/*******************************************************************************
* Function Name  : ADC_ResetCalibration
* Description    : Resets the selected ADC calibration registers.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_ResetCalibration(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  /* Resets the selected ADC calibartion registers */  
  ADCx->CR2 |= CR2_RSTCAL_Set;
}

/*******************************************************************************
* Function Name  : ADC_GetResetCalibrationStatus
* Description    : Gets the selected ADC reset calibration registers status.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
* Output         : None
* Return         : The new state of ADC reset calibration registers (SET or RESET).
*******************************************************************************/
FlagStatus ADC_GetResetCalibrationStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  /* Check the status of RSTCAL bit */
  if ((ADCx->CR2 & CR2_RSTCAL_Set) != (u32)RESET)
  {
    /* RSTCAL bit is set */
    bitstatus = SET;
  }
  else
  {
    /* RSTCAL bit is reset */
    bitstatus = RESET;
  }

  /* Return the RSTCAL bit status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : ADC_StartCalibration
* Description    : Starts the selected ADC calibration process.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_StartCalibration(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  /* Enable the selected ADC calibration process */  
  ADCx->CR2 |= CR2_CAL_Set;
}

/*******************************************************************************
* Function Name  : ADC_GetCalibrationStatus
* Description    : Gets the selected ADC calibration status.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
* Output         : None
* Return         : The new state of ADC calibration (SET or RESET).
*******************************************************************************/
FlagStatus ADC_GetCalibrationStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  /* Check the status of CAL bit */
  if ((ADCx->CR2 & CR2_CAL_Set) != (u32)RESET)
  {
    /* CAL bit is set: calibration on going */
    bitstatus = SET;
  }
  else
  {
    /* CAL bit is reset: end of calibration */
    bitstatus = RESET;
  }

  /* Return the CAL bit status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : ADC_SoftwareStartConvCmd
* Description    : Enables or disables the selected ADC software start conversion .
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - NewState: new state of the selected ADC software start conversion.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_SoftwareStartConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC conversion on external event and start the selected
       ADC conversion */
    ADCx->CR2 |= CR2_EXTTRIG_SWSTART_Set;
  }
  else
  {
    /* Disable the selected ADC conversion on external event and stop the selected
       ADC conversion */
    ADCx->CR2 &= CR2_EXTTRIG_SWSTART_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_GetSoftwareStartConvStatus
* Description    : Gets the selected ADC Software start conversion Status.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
* Output         : None
* Return         : The new state of ADC software start conversion (SET or RESET).
*******************************************************************************/
FlagStatus ADC_GetSoftwareStartConvStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  /* Check the status of SWSTART bit */
  if ((ADCx->CR2 & CR2_SWSTART_Set) != (u32)RESET)
  {
    /* SWSTART bit is set */
    bitstatus = SET;
  }
  else
  {
    /* SWSTART bit is reset */
    bitstatus = RESET;
  }

  /* Return the SWSTART bit status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : ADC_DiscModeChannelCountConfig
* Description    : Configures the discontinuous mode for the selected ADC regular
*                  group channel.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - Number: specifies the discontinuous mode regular channel
*                    count value. This number must be between 1 and 8.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_DiscModeChannelCountConfig(ADC_TypeDef* ADCx, u8 Number)
{
  u32 tmpreg1 = 0;
  u32 tmpreg2 = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_REGULAR_DISC_NUMBER(Number));

  /* Get the old register value */
  tmpreg1 = ADCx->CR1;
  /* Clear the old discontinuous mode channel count */
  tmpreg1 &= CR1_DISCNUM_Reset;
  /* Set the discontinuous mode channel count */
  tmpreg2 = Number - 1;
  tmpreg1 |= tmpreg2 << 13;
  /* Store the new register value */
  ADCx->CR1 = tmpreg1;
}

/*******************************************************************************
* Function Name  : ADC_DiscModeCmd
* Description    : Enables or disables the discontinuous mode on regular group
*                  channel for the specified ADC
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - NewState: new state of the selected ADC discontinuous mode
*                    on regular group channel.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_DiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC regular discontinuous mode */
    ADCx->CR1 |= CR1_DISCEN_Set;
  }
  else
  {
    /* Disable the selected ADC regular discontinuous mode */
    ADCx->CR1 &= CR1_DISCEN_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_RegularChannelConfig
* Description    : Configures for the selected ADC regular channel its corresponding
*                  rank in the sequencer and its sample time.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_Channel: the ADC channel to configure. 
*                    This parameter can be one of the following values:
*                       - ADC_Channel_0: ADC Channel0 selected
*                       - ADC_Channel_1: ADC Channel1 selected
*                       - ADC_Channel_2: ADC Channel2 selected
*                       - ADC_Channel_3: ADC Channel3 selected
*                       - ADC_Channel_4: ADC Channel4 selected
*                       - ADC_Channel_5: ADC Channel5 selected
*                       - ADC_Channel_6: ADC Channel6 selected
*                       - ADC_Channel_7: ADC Channel7 selected
*                       - ADC_Channel_8: ADC Channel8 selected
*                       - ADC_Channel_9: ADC Channel9 selected
*                       - ADC_Channel_10: ADC Channel10 selected
*                       - ADC_Channel_11: ADC Channel11 selected
*                       - ADC_Channel_12: ADC Channel12 selected
*                       - ADC_Channel_13: ADC Channel13 selected
*                       - ADC_Channel_14: ADC Channel14 selected
*                       - ADC_Channel_15: ADC Channel15 selected
*                       - ADC_Channel_16: ADC Channel16 selected
*                       - ADC_Channel_17: ADC Channel17 selected
*                  - Rank: The rank in the regular group sequencer. This parameter
*                    must be between 1 to 16.
*                  - ADC_SampleTime: The sample time value to be set for the
*                    selected channel. 
*                    This parameter can be one of the following values:
*                       - ADC_SampleTime_1Cycles5: Sample time equal to 1.5 cycles
*                       - ADC_SampleTime_7Cycles5: Sample time equal to 7.5 cycles
*                       - ADC_SampleTime_13Cycles5: Sample time equal to 13.5 cycles
*                       - ADC_SampleTime_28Cycles5: Sample time equal to 28.5 cycles	
*                       - ADC_SampleTime_41Cycles5: Sample time equal to 41.5 cycles	
*                       - ADC_SampleTime_55Cycles5: Sample time equal to 55.5 cycles	
*                       - ADC_SampleTime_71Cycles5: Sample time equal to 71.5 cycles	
*                       - ADC_SampleTime_239Cycles5: Sample time equal to 239.5 cycles	
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_RegularChannelConfig(ADC_TypeDef* ADCx, u8 ADC_Channel, u8 Rank, u8 ADC_SampleTime)
{
  u32 tmpreg1 = 0, tmpreg2 = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_CHANNEL(ADC_Channel));
  assert_param(IS_ADC_REGULAR_RANK(Rank));
  assert_param(IS_ADC_SAMPLE_TIME(ADC_SampleTime));

  /* if ADC_Channel_10 ... ADC_Channel_17 is selected */
  if (ADC_Channel > ADC_Channel_9)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR1;
    /* Calculate the mask to clear */
    tmpreg2 = SMPR1_SMP_Set << (3 * (ADC_Channel - 10));
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (u32)ADC_SampleTime << (3 * (ADC_Channel - 10));
    /* Set the discontinuous mode channel count */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR1 = tmpreg1;
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR2;
    /* Calculate the mask to clear */
    tmpreg2 = SMPR2_SMP_Set << (3 * ADC_Channel);
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (u32)ADC_SampleTime << (3 * ADC_Channel);
    /* Set the discontinuous mode channel count */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR2 = tmpreg1;
  }
  /* For Rank 1 to 6 */
  if (Rank < 7)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SQR3;
    /* Calculate the mask to clear */
    tmpreg2 = SQR3_SQ_Set << (5 * (Rank - 1));
    /* Clear the old SQx bits for the selected rank */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (u32)ADC_Channel << (5 * (Rank - 1));
    /* Set the SQx bits for the selected rank */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SQR3 = tmpreg1;
  }
  /* For Rank 7 to 12 */
  else if (Rank < 13)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SQR2;
    /* Calculate the mask to clear */
    tmpreg2 = SQR2_SQ_Set << (5 * (Rank - 7));
    /* Clear the old SQx bits for the selected rank */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (u32)ADC_Channel << (5 * (Rank - 7));
    /* Set the SQx bits for the selected rank */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SQR2 = tmpreg1;
  }
  /* For Rank 13 to 16 */
  else
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SQR1;
    /* Calculate the mask to clear */
    tmpreg2 = SQR1_SQ_Set << (5 * (Rank - 13));
    /* Clear the old SQx bits for the selected rank */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (u32)ADC_Channel << (5 * (Rank - 13));
    /* Set the SQx bits for the selected rank */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SQR1 = tmpreg1;
  }
}

/*******************************************************************************
* Function Name  : ADC_ExternalTrigConvCmd
* Description    : Enables or disables the ADCx conversion through external trigger.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - NewState: new state of the selected ADC external trigger
*                    start of conversion.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_ExternalTrigConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC conversion on external event */
    ADCx->CR2 |= CR2_EXTTRIG_Set;
  }
  else
  {
    /* Disable the selected ADC conversion on external event */
    ADCx->CR2 &= CR2_EXTTRIG_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_GetConversionValue
* Description    : Returns the last ADCx conversion result data for regular channel.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
* Output         : None
* Return         : The Data conversion value.
*******************************************************************************/
u16 ADC_GetConversionValue(ADC_TypeDef* ADCx)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  /* Return the selected ADC conversion value */
  return (u16) ADCx->DR;
}

/*******************************************************************************
* Function Name  : ADC_GetDualModeConversionValue
* Description    : Returns the last ADC1 and ADC2 conversion result data in dual mode.
* Output         : None
* Return         : The Data conversion value.
*******************************************************************************/
u32 ADC_GetDualModeConversionValue(void)
{
  /* Return the dual mode conversion value */
  return (*(vu32 *) DR_ADDRESS);
}

/*******************************************************************************
* Function Name  : ADC_AutoInjectedConvCmd
* Description    : Enables or disables the selected ADC automatic injected group
*                  conversion after regular one.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - NewState: new state of the selected ADC auto injected
*                    conversion
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_AutoInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC automatic injected group conversion */
    ADCx->CR1 |= CR1_JAUTO_Set;
  }
  else
  {
    /* Disable the selected ADC automatic injected group conversion */
    ADCx->CR1 &= CR1_JAUTO_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_InjectedDiscModeCmd
* Description    : Enables or disables the discontinuous mode for injected group
*                  channel for the specified ADC
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - NewState: new state of the selected ADC discontinuous mode
*                    on injected group channel.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_InjectedDiscModeCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC injected discontinuous mode */
    ADCx->CR1 |= CR1_JDISCEN_Set;
  }
  else
  {
    /* Disable the selected ADC injected discontinuous mode */
    ADCx->CR1 &= CR1_JDISCEN_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_ExternalTrigInjectedConvConfig
* Description    : Configures the ADCx external trigger for injected channels conversion.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_ExternalTrigInjecConv: specifies the ADC trigger to
*                    start injected conversion. 
*                    This parameter can be one of the following values:
*                       - ADC_ExternalTrigInjecConv_T1_TRGO: Timer1 TRGO event 
*                         selected (for ADC1, ADC2 and ADC3)
*                       - ADC_ExternalTrigInjecConv_T1_CC4: Timer1 capture
*                         compare4 selected (for ADC1, ADC2 and ADC3)
*                       - ADC_ExternalTrigInjecConv_T2_TRGO: Timer2 TRGO event
*                         selected (for ADC1 and ADC2)
*                       - ADC_External TrigInjecConv_T2_CC1: Timer2 capture
*                         compare1 selected (for ADC1 and ADC2)
*                       - ADC_ExternalTrigInjecConv_T3_CC4: Timer3 capture
*                         compare4 selected (for ADC1 and ADC2)
*                       - ADC_ExternalTrigInjecConv_T4_TRGO: Timer4 TRGO event
*                         selected (for ADC1 and ADC2)
*                       - ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4: External
*                         interrupt line 15 or Timer8 capture compare4 event selected
*                         (for ADC1 and ADC2)                       
*                       - ADC_External TrigInjecConv_T4_CC3: Timer4 capture
*                         compare3 selected (for ADC3 only)
*                       - ADC_External TrigInjecConv_T8_CC2: Timer8 capture
*                         compare2 selected (for ADC3 only)                         
*                       - ADC_External TrigInjecConv_T8_CC4: Timer8 capture
*                         compare4 selected (for ADC3 only)
*                       - ADC_ExternalTrigInjecConv_T5_TRGO: Timer5 TRGO event
*                         selected (for ADC3 only)                         
*                       - ADC_External TrigInjecConv_T5_CC4: Timer5 capture
*                         compare4 selected (for ADC3 only)                        
*                       - ADC_ExternalTrigInjecConv_None: Injected conversion
*                         started by software and not by external trigger (for 
*                         ADC1, ADC2 and ADC3)
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_ExternalTrigInjectedConvConfig(ADC_TypeDef* ADCx, u32 ADC_ExternalTrigInjecConv)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_EXT_INJEC_TRIG(ADC_ExternalTrigInjecConv));

  /* Get the old register value */
  tmpreg = ADCx->CR2;
  /* Clear the old external event selection for injected group */
  tmpreg &= CR2_JEXTSEL_Reset;
  /* Set the external event selection for injected group */
  tmpreg |= ADC_ExternalTrigInjecConv;
  /* Store the new register value */
  ADCx->CR2 = tmpreg;
}

/*******************************************************************************
* Function Name  : ADC_ExternalTrigInjectedConvCmd
* Description    : Enables or disables the ADCx injected channels conversion
*                  through external trigger
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - NewState: new state of the selected ADC external trigger
*                    start of injected conversion.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_ExternalTrigInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC external event selection for injected group */
    ADCx->CR2 |= CR2_JEXTTRIG_Set;
  }
  else
  {
    /* Disable the selected ADC external event selection for injected group */
    ADCx->CR2 &= CR2_JEXTTRIG_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_SoftwareStartInjectedConvCmd
* Description    : Enables or disables the selected ADC start of the injected 
*                  channels conversion.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - NewState: new state of the selected ADC software start
*                    injected conversion.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_SoftwareStartInjectedConvCmd(ADC_TypeDef* ADCx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected ADC conversion for injected group on external event and start the selected
       ADC injected conversion */
    ADCx->CR2 |= CR2_JEXTTRIG_JSWSTART_Set;
  }
  else
  {
    /* Disable the selected ADC conversion on external event for injected group and stop the selected
       ADC injected conversion */
    ADCx->CR2 &= CR2_JEXTTRIG_JSWSTART_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_GetSoftwareStartInjectedConvCmdStatus
* Description    : Gets the selected ADC Software start injected conversion Status.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
* Output         : None
* Return         : The new state of ADC software start injected conversion (SET or RESET).
*******************************************************************************/
FlagStatus ADC_GetSoftwareStartInjectedConvCmdStatus(ADC_TypeDef* ADCx)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));

  /* Check the status of JSWSTART bit */
  if ((ADCx->CR2 & CR2_JSWSTART_Set) != (u32)RESET)
  {
    /* JSWSTART bit is set */
    bitstatus = SET;
  }
  else
  {
    /* JSWSTART bit is reset */
    bitstatus = RESET;
  }

  /* Return the JSWSTART bit status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : ADC_InjectedChannelConfig
* Description    : Configures for the selected ADC injected channel its corresponding
*                  rank in the sequencer and its sample time.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_Channel: the ADC channel to configure. 
*                    This parameter can be one of the following values:
*                       - ADC_Channel_0: ADC Channel0 selected
*                       - ADC_Channel_1: ADC Channel1 selected
*                       - ADC_Channel_2: ADC Channel2 selected
*                       - ADC_Channel_3: ADC Channel3 selected
*                       - ADC_Channel_4: ADC Channel4 selected
*                       - ADC_Channel_5: ADC Channel5 selected
*                       - ADC_Channel_6: ADC Channel6 selected
*                       - ADC_Channel_7: ADC Channel7 selected
*                       - ADC_Channel_8: ADC Channel8 selected
*                       - ADC_Channel_9: ADC Channel9 selected
*                       - ADC_Channel_10: ADC Channel10 selected
*                       - ADC_Channel_11: ADC Channel11 selected
*                       - ADC_Channel_12: ADC Channel12 selected
*                       - ADC_Channel_13: ADC Channel13 selected
*                       - ADC_Channel_14: ADC Channel14 selected
*                       - ADC_Channel_15: ADC Channel15 selected
*                       - ADC_Channel_16: ADC Channel16 selected
*                       - ADC_Channel_17: ADC Channel17 selected
*                  - Rank: The rank in the injected group sequencer. This parameter
*                    must be between 1 to 4.
*                  - ADC_SampleTime: The sample time value to be set for the
*                    selected channel. 
*                    This parameter can be one of the following values:
*                       - ADC_SampleTime_1Cycles5: Sample time equal to 1.5 cycles
*                       - ADC_SampleTime_7Cycles5: Sample time equal to 7.5 cycles
*                       - ADC_SampleTime_13Cycles5: Sample time equal to 13.5 cycles
*                       - ADC_SampleTime_28Cycles5: Sample time equal to 28.5 cycles	
*                       - ADC_SampleTime_41Cycles5: Sample time equal to 41.5 cycles	
*                       - ADC_SampleTime_55Cycles5: Sample time equal to 55.5 cycles	
*                       - ADC_SampleTime_71Cycles5: Sample time equal to 71.5 cycles	
*                       - ADC_SampleTime_239Cycles5: Sample time equal to 239.5 cycles	
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_InjectedChannelConfig(ADC_TypeDef* ADCx, u8 ADC_Channel, u8 Rank, u8 ADC_SampleTime)
{
  u32 tmpreg1 = 0, tmpreg2 = 0, tmpreg3 = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_CHANNEL(ADC_Channel));
  assert_param(IS_ADC_INJECTED_RANK(Rank));
  assert_param(IS_ADC_SAMPLE_TIME(ADC_SampleTime));

  /* if ADC_Channel_10 ... ADC_Channel_17 is selected */
  if (ADC_Channel > ADC_Channel_9)
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR1;
    /* Calculate the mask to clear */
    tmpreg2 = SMPR1_SMP_Set << (3*(ADC_Channel - 10));
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (u32)ADC_SampleTime << (3*(ADC_Channel - 10));
    /* Set the discontinuous mode channel count */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR1 = tmpreg1;
  }
  else /* ADC_Channel include in ADC_Channel_[0..9] */
  {
    /* Get the old register value */
    tmpreg1 = ADCx->SMPR2;
    /* Calculate the mask to clear */
    tmpreg2 = SMPR2_SMP_Set << (3 * ADC_Channel);
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= ~tmpreg2;
    /* Calculate the mask to set */
    tmpreg2 = (u32)ADC_SampleTime << (3 * ADC_Channel);
    /* Set the discontinuous mode channel count */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ADCx->SMPR2 = tmpreg1;
  }

  /* Rank configuration */
  /* Get the old register value */
  tmpreg1 = ADCx->JSQR;
  /* Get JL value: Number = JL+1 */
  tmpreg3 =  (tmpreg1 & JSQR_JL_Set)>> 20;
  /* Calculate the mask to clear: ((Rank-1)+(4-JL-1)) */
  tmpreg2 = JSQR_JSQ_Set << (5 * (u8)((Rank + 3) - (tmpreg3 + 1)));
  /* Clear the old JSQx bits for the selected rank */
  tmpreg1 &= ~tmpreg2;
  /* Calculate the mask to set: ((Rank-1)+(4-JL-1)) */
  tmpreg2 = (u32)ADC_Channel << (5 * (u8)((Rank + 3) - (tmpreg3 + 1)));
  /* Set the JSQx bits for the selected rank */
  tmpreg1 |= tmpreg2;
  /* Store the new register value */
  ADCx->JSQR = tmpreg1;
}

/*******************************************************************************
* Function Name  : ADC_InjectedSequencerLengthConfig
* Description    : Configures the sequencer length for injected channels
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - Length: The sequencer length. 
*                    This parameter must be a number between 1 to 4.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_InjectedSequencerLengthConfig(ADC_TypeDef* ADCx, u8 Length)
{
  u32 tmpreg1 = 0;
  u32 tmpreg2 = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_INJECTED_LENGTH(Length));
  
  /* Get the old register value */
  tmpreg1 = ADCx->JSQR;
  /* Clear the old injected sequnence lenght JL bits */
  tmpreg1 &= JSQR_JL_Reset;
  /* Set the injected sequnence lenght JL bits */
  tmpreg2 = Length - 1; 
  tmpreg1 |= tmpreg2 << 20;
  /* Store the new register value */
  ADCx->JSQR = tmpreg1;
}

/*******************************************************************************
* Function Name  : ADC_SetInjectedOffset
* Description    : Set the injected channels conversion value offset
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_InjectedChannel: the ADC injected channel to set its
*                    offset. 
*                    This parameter can be one of the following values:
*                       - ADC_InjectedChannel_1: Injected Channel1 selected
*                       - ADC_InjectedChannel_2: Injected Channel2 selected
*                       - ADC_InjectedChannel_3: Injected Channel3 selected
*                       - ADC_InjectedChannel_4: Injected Channel4 selected
*                  - Offset: the offset value for the selected ADC injected channel
*                    This parameter must be a 12bit value.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_SetInjectedOffset(ADC_TypeDef* ADCx, u8 ADC_InjectedChannel, u16 Offset)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_INJECTED_CHANNEL(ADC_InjectedChannel));
  assert_param(IS_ADC_OFFSET(Offset));  

  /* Set the selected injected channel data offset */
  *((vu32 *)((*(u32*)&ADCx) + ADC_InjectedChannel)) = (u32)Offset;
}

/*******************************************************************************
* Function Name  : ADC_GetInjectedConversionValue
* Description    : Returns the ADC injected channel conversion result
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_InjectedChannel: the converted ADC injected channel.
*                    This parameter can be one of the following values:
*                       - ADC_InjectedChannel_1: Injected Channel1 selected
*                       - ADC_InjectedChannel_2: Injected Channel2 selected
*                       - ADC_InjectedChannel_3: Injected Channel3 selected
*                       - ADC_InjectedChannel_4: Injected Channel4 selected
* Output         : None
* Return         : The Data conversion value.
*******************************************************************************/
u16 ADC_GetInjectedConversionValue(ADC_TypeDef* ADCx, u8 ADC_InjectedChannel)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_INJECTED_CHANNEL(ADC_InjectedChannel));

  /* Returns the selected injected channel conversion data value */
  return (u16) (*(vu32*) (((*(u32*)&ADCx) + ADC_InjectedChannel + JDR_Offset)));
}

/*******************************************************************************
* Function Name  : ADC_AnalogWatchdogCmd
* Description    : Enables or disables the analog watchdog on single/all regular
*                  or injected channels
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_AnalogWatchdog: the ADC analog watchdog configuration.
*                    This parameter can be one of the following values:
*                       - ADC_AnalogWatchdog_SingleRegEnable: Analog watchdog on
*                         a single regular channel
*                       - ADC_AnalogWatchdog_SingleInjecEnable: Analog watchdog on
*                         a single injected channel
*                       - ADC_AnalogWatchdog_SingleRegOrInjecEnable: Analog 
*                         watchdog on a single regular or injected channel
*                       - ADC_AnalogWatchdog_AllRegEnable: Analog watchdog on
*                         all regular channel
*                       - ADC_AnalogWatchdog_AllInjecEnable: Analog watchdog on
*                         all injected channel
*                       - ADC_AnalogWatchdog_AllRegAllInjecEnable: Analog watchdog
*                         on all regular and injected channels
*                       - ADC_AnalogWatchdog_None: No channel guarded by the
*                         analog watchdog
* Output         : None
* Return         : None	  
*******************************************************************************/
void ADC_AnalogWatchdogCmd(ADC_TypeDef* ADCx, u32 ADC_AnalogWatchdog)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_ANALOG_WATCHDOG(ADC_AnalogWatchdog));

  /* Get the old register value */
  tmpreg = ADCx->CR1;
  /* Clear AWDEN, AWDENJ and AWDSGL bits */
  tmpreg &= CR1_AWDMode_Reset;
  /* Set the analog watchdog enable mode */
  tmpreg |= ADC_AnalogWatchdog;
  /* Store the new register value */
  ADCx->CR1 = tmpreg;
}

/*******************************************************************************
* Function Name  : ADC_AnalogWatchdogThresholdsConfig
* Description    : Configures the high and low thresholds of the analog watchdog.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - HighThreshold: the ADC analog watchdog High threshold value.
*                    This parameter must be a 12bit value.
*                  - LowThreshold: the ADC analog watchdog Low threshold value.
*                    This parameter must be a 12bit value.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_AnalogWatchdogThresholdsConfig(ADC_TypeDef* ADCx, u16 HighThreshold,
                                        u16 LowThreshold)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_THRESHOLD(HighThreshold));
  assert_param(IS_ADC_THRESHOLD(LowThreshold));

  /* Set the ADCx high threshold */
  ADCx->HTR = HighThreshold;
  /* Set the ADCx low threshold */
  ADCx->LTR = LowThreshold;
}

/*******************************************************************************
* Function Name  : ADC_AnalogWatchdogSingleChannelConfig
* Description    : Configures the analog watchdog guarded single channel
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_Channel: the ADC channel to configure for the analog
*                    watchdog. 
*                    This parameter can be one of the following values:
*                       - ADC_Channel_0: ADC Channel0 selected
*                       - ADC_Channel_1: ADC Channel1 selected
*                       - ADC_Channel_2: ADC Channel2 selected
*                       - ADC_Channel_3: ADC Channel3 selected
*                       - ADC_Channel_4: ADC Channel4 selected
*                       - ADC_Channel_5: ADC Channel5 selected
*                       - ADC_Channel_6: ADC Channel6 selected
*                       - ADC_Channel_7: ADC Channel7 selected
*                       - ADC_Channel_8: ADC Channel8 selected
*                       - ADC_Channel_9: ADC Channel9 selected
*                       - ADC_Channel_10: ADC Channel10 selected
*                       - ADC_Channel_11: ADC Channel11 selected
*                       - ADC_Channel_12: ADC Channel12 selected
*                       - ADC_Channel_13: ADC Channel13 selected
*                       - ADC_Channel_14: ADC Channel14 selected
*                       - ADC_Channel_15: ADC Channel15 selected
*                       - ADC_Channel_16: ADC Channel16 selected
*                       - ADC_Channel_17: ADC Channel17 selected
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_AnalogWatchdogSingleChannelConfig(ADC_TypeDef* ADCx, u8 ADC_Channel)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_CHANNEL(ADC_Channel));

  /* Get the old register value */
  tmpreg = ADCx->CR1;
  /* Clear the Analog watchdog channel select bits */
  tmpreg &= CR1_AWDCH_Reset;
  /* Set the Analog watchdog channel */
  tmpreg |= ADC_Channel;
  /* Store the new register value */
  ADCx->CR1 = tmpreg;
}

/*******************************************************************************
* Function Name  : ADC_TempSensorVrefintCmd
* Description    : Enables or disables the temperature sensor and Vrefint channel.
* Input          : - NewState: new state of the temperature sensor.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_TempSensorVrefintCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the temperature sensor and Vrefint channel*/
    ADC1->CR2 |= CR2_TSVREFE_Set;
  }
  else
  {
    /* Disable the temperature sensor and Vrefint channel*/
    ADC1->CR2 &= CR2_TSVREFE_Reset;
  }
}

/*******************************************************************************
* Function Name  : ADC_GetFlagStatus
* Description    : Checks whether the specified ADC flag is set or not.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_FLAG: specifies the flag to check. 
*                    This parameter can be one of the following values:
*                       - ADC_FLAG_AWD: Analog watchdog flag
*                       - ADC_FLAG_EOC: End of conversion flag
*                       - ADC_FLAG_JEOC: End of injected group conversion flag
*                       - ADC_FLAG_JSTRT: Start of injected group conversion flag
*                       - ADC_FLAG_STRT: Start of regular group conversion flag
* Output         : None
* Return         : The new state of ADC_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus ADC_GetFlagStatus(ADC_TypeDef* ADCx, u8 ADC_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_GET_FLAG(ADC_FLAG));

  /* Check the status of the specified ADC flag */
  if ((ADCx->SR & ADC_FLAG) != (u8)RESET)
  {
    /* ADC_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* ADC_FLAG is reset */
    bitstatus = RESET;
  }

  /* Return the ADC_FLAG status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : ADC_ClearFlag
* Description    : Clears the ADCx's pending flags.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_FLAG: specifies the flag to clear. 
*                    This parameter can be any combination of the following values:
*                       - ADC_FLAG_AWD: Analog watchdog flag
*                       - ADC_FLAG_EOC: End of conversion flag
*                       - ADC_FLAG_JEOC: End of injected group conversion flag
*                       - ADC_FLAG_JSTRT: Start of injected group conversion flag
*                       - ADC_FLAG_STRT: Start of regular group conversion flag
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_ClearFlag(ADC_TypeDef* ADCx, u8 ADC_FLAG)
{
  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_CLEAR_FLAG(ADC_FLAG));

  /* Clear the selected ADC flags */
  ADCx->SR = ~(u32)ADC_FLAG;
}

/*******************************************************************************
* Function Name  : ADC_GetITStatus
* Description    : Checks whether the specified ADC interrupt has occurred or not.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_IT: specifies the ADC interrupt source to check. 
*                    This parameter can be one of the following values:
*                       - ADC_IT_EOC: End of conversion interrupt mask
*                       - ADC_IT_AWD: Analog watchdog interrupt mask
*                       - ADC_IT_JEOC: End of injected conversion interrupt mask
* Output         : None
* Return         : The new state of ADC_IT (SET or RESET).
*******************************************************************************/
ITStatus ADC_GetITStatus(ADC_TypeDef* ADCx, u16 ADC_IT)
{
  ITStatus bitstatus = RESET;
  u32 itmask = 0, enablestatus = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_GET_IT(ADC_IT));

  /* Get the ADC IT index */
  itmask = ADC_IT >> 8;

  /* Get the ADC_IT enable bit status */
  enablestatus = (ADCx->CR1 & (u8)ADC_IT) ;

  /* Check the status of the specified ADC interrupt */
  if (((ADCx->SR & itmask) != (u32)RESET) && enablestatus)
  {
    /* ADC_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* ADC_IT is reset */
    bitstatus = RESET;
  }

  /* Return the ADC_IT status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : ADC_ClearITPendingBit
* Description    : Clears the ADCx’s interrupt pending bits.
* Input          : - ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
*                  - ADC_IT: specifies the ADC interrupt pending bit to clear.
*                    This parameter can be any combination of the following values:
*                       - ADC_IT_EOC: End of conversion interrupt mask
*                       - ADC_IT_AWD: Analog watchdog interrupt mask
*                       - ADC_IT_JEOC: End of injected conversion interrupt mask
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_ClearITPendingBit(ADC_TypeDef* ADCx, u16 ADC_IT)
{
  u8 itmask = 0;

  /* Check the parameters */
  assert_param(IS_ADC_ALL_PERIPH(ADCx));
  assert_param(IS_ADC_IT(ADC_IT));

  /* Get the ADC IT index */
  itmask = (u8)(ADC_IT >> 8);

  /* Clear the selected ADC interrupt pending bits */
  ADCx->SR = ~(u32)itmask;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_bkp.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the BKP firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_bkp.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ------------ BKP registers bit address in the alias region ----------- */
#define BKP_OFFSET        (BKP_BASE - PERIPH_BASE)

/* --- CR Register ---*/
/* Alias word address of TPAL bit */
#define CR_OFFSET         (BKP_OFFSET + 0x30)
#define TPAL_BitNumber    0x01
#define CR_TPAL_BB        (PERIPH_BB_BASE + (CR_OFFSET * 32) + (TPAL_BitNumber * 4))

/* Alias word address of TPE bit */
#define TPE_BitNumber     0x00
#define CR_TPE_BB         (PERIPH_BB_BASE + (CR_OFFSET * 32) + (TPE_BitNumber * 4))

/* --- CSR Register ---*/
/* Alias word address of TPIE bit */
#define CSR_OFFSET        (BKP_OFFSET + 0x34)
#define TPIE_BitNumber    0x02
#define CSR_TPIE_BB       (PERIPH_BB_BASE + (CSR_OFFSET * 32) + (TPIE_BitNumber * 4))

/* Alias word address of TIF bit */
#define TIF_BitNumber     0x09
#define CSR_TIF_BB        (PERIPH_BB_BASE + (CSR_OFFSET * 32) + (TIF_BitNumber * 4))

/* Alias word address of TEF bit */
#define TEF_BitNumber     0x08
#define CSR_TEF_BB        (PERIPH_BB_BASE + (CSR_OFFSET * 32) + (TEF_BitNumber * 4))


/* ---------------------- BKP registers bit mask ------------------------ */
/* RTCCR register bit mask */
#define RTCCR_CAL_Mask    ((u16)0xFF80)
#define RTCCR_Mask        ((u16)0xFC7F)

/* CSR register bit mask */
#define CSR_CTE_Set       ((u16)0x0001)
#define CSR_CTI_Set       ((u16)0x0002)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : BKP_DeInit
* Description    : Deinitializes the BKP peripheral registers to their default
*                  reset values.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BKP_DeInit(void)
{
  RCC_BackupResetCmd(ENABLE);
  RCC_BackupResetCmd(DISABLE);
}

/*******************************************************************************
* Function Name  : BKP_TamperPinLevelConfig
* Description    : Configures the Tamper Pin active level.
* Input          : - BKP_TamperPinLevel: specifies the Tamper Pin active level.
*                    This parameter can be one of the following values:
*                       - BKP_TamperPinLevel_High: Tamper pin active on high level
*                       - BKP_TamperPinLevel_Low: Tamper pin active on low level
* Output         : None
* Return         : None
*******************************************************************************/
void BKP_TamperPinLevelConfig(u16 BKP_TamperPinLevel)
{
  /* Check the parameters */
  assert_param(IS_BKP_TAMPER_PIN_LEVEL(BKP_TamperPinLevel));

  *(vu32 *) CR_TPAL_BB = BKP_TamperPinLevel;
}

/*******************************************************************************
* Function Name  : BKP_TamperPinCmd
* Description    : Enables or disables the Tamper Pin activation.
* Input          : - NewState: new state of the Tamper Pin activation.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void BKP_TamperPinCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) CR_TPE_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : BKP_ITConfig
* Description    : Enables or disables the Tamper Pin Interrupt.
* Input          : - NewState: new state of the Tamper Pin Interrupt.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void BKP_ITConfig(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) CSR_TPIE_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : BKP_RTCOutputConfig
* Description    : Select the RTC output source to output on the Tamper pin.
* Input          : - BKP_RTCOutputSource: specifies the RTC output source.
*                    This parameter can be one of the following values:
*                       - BKP_RTCOutputSource_None: no RTC output on the Tamper pin.
*                       - BKP_RTCOutputSource_CalibClock: output the RTC clock
*                         with frequency divided by 64 on the Tamper pin.
*                       - BKP_RTCOutputSource_Alarm: output the RTC Alarm pulse 
*                         signal on the Tamper pin.
*                       - BKP_RTCOutputSource_Second: output the RTC Second pulse 
*                         signal on the Tamper pin.  
* Output         : None
* Return         : None
*******************************************************************************/
void BKP_RTCOutputConfig(u16 BKP_RTCOutputSource)
{
  u16 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_BKP_RTC_OUTPUT_SOURCE(BKP_RTCOutputSource));

  tmpreg = BKP->RTCCR;

  /* Clear CCO, ASOE and ASOS bits */
  tmpreg &= RTCCR_Mask;
  
  /* Set CCO, ASOE and ASOS bits according to BKP_RTCOutputSource value */
  tmpreg |= BKP_RTCOutputSource;

  /* Store the new value */
  BKP->RTCCR = tmpreg;
}

/*******************************************************************************
* Function Name  : BKP_SetRTCCalibrationValue
* Description    : Sets RTC Clock Calibration value.
* Input          : - CalibrationValue: specifies the RTC Clock Calibration value.
*                    This parameter must be a number between 0 and 0x7F.
* Output         : None
* Return         : None
*******************************************************************************/
void BKP_SetRTCCalibrationValue(u8 CalibrationValue)
{
  u16 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_BKP_CALIBRATION_VALUE(CalibrationValue));

  tmpreg = BKP->RTCCR;

  /* Clear CAL[6:0] bits */
  tmpreg &= RTCCR_CAL_Mask;

  /* Set CAL[6:0] bits according to CalibrationValue value */
  tmpreg |= CalibrationValue;

  /* Store the new value */
  BKP->RTCCR = tmpreg;
}

/*******************************************************************************
* Function Name  : BKP_WriteBackupRegister
* Description    : Writes user data to the specified Data Backup Register.
* Input          : - BKP_DR: specifies the Data Backup Register.
*                    This parameter can be BKP_DRx where x:[1, 42]
*                  - Data: data to write
* Output         : None
* Return         : None
*******************************************************************************/
void BKP_WriteBackupRegister(u16 BKP_DR, u16 Data)
{
  /* Check the parameters */
  assert_param(IS_BKP_DR(BKP_DR));

  *(vu16 *) (BKP_BASE + BKP_DR) = Data;
}

/*******************************************************************************
* Function Name  : BKP_ReadBackupRegister
* Description    : Reads data from the specified Data Backup Register.
* Input          : - BKP_DR: specifies the Data Backup Register.
*                    This parameter can be BKP_DRx where x:[1, 42]
* Output         : None
* Return         : The content of the specified Data Backup Register
*******************************************************************************/
u16 BKP_ReadBackupRegister(u16 BKP_DR)
{
  /* Check the parameters */
  assert_param(IS_BKP_DR(BKP_DR));

  return (*(vu16 *) (BKP_BASE + BKP_DR));
}

/*******************************************************************************
* Function Name  : BKP_GetFlagStatus
* Description    : Checks whether the Tamper Pin Event flag is set or not.
* Input          : None
* Output         : None
* Return         : The new state of the Tamper Pin Event flag (SET or RESET).
*******************************************************************************/
FlagStatus BKP_GetFlagStatus(void)
{
  return (FlagStatus)(*(vu32 *) CSR_TEF_BB);
}

/*******************************************************************************
* Function Name  : BKP_ClearFlag
* Description    : Clears Tamper Pin Event pending flag.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BKP_ClearFlag(void)
{
  /* Set CTE bit to clear Tamper Pin Event flag */
  BKP->CSR |= CSR_CTE_Set;
}

/*******************************************************************************
* Function Name  : BKP_GetITStatus
* Description    : Checks whether the Tamper Pin Interrupt has occurred or not.
* Input          : None
* Output         : None
* Return         : The new state of the Tamper Pin Interrupt (SET or RESET).
*******************************************************************************/
ITStatus BKP_GetITStatus(void)
{
  return (ITStatus)(*(vu32 *) CSR_TIF_BB);
}

/*******************************************************************************
* Function Name  : BKP_ClearITPendingBit
* Description    : Clears Tamper Pin Interrupt pending bit.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BKP_ClearITPendingBit(void)
{
  /* Set CTI bit to clear Tamper Pin Interrupt pending bit */
  BKP->CSR |= CSR_CTI_Set;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_can.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the CAN firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* CAN Master Control Register bits */
#define MCR_INRQ     ((u32)0x00000001) /* Initialization request */
#define MCR_SLEEP    ((u32)0x00000002) /* Sleep mode request */
#define MCR_TXFP     ((u32)0x00000004) /* Transmit FIFO priority */
#define MCR_RFLM     ((u32)0x00000008) /* Receive FIFO locked mode */
#define MCR_NART     ((u32)0x00000010) /* No automatic retransmission */
#define MCR_AWUM     ((u32)0x00000020) /* Automatic wake up mode */
#define MCR_ABOM     ((u32)0x00000040) /* Automatic bus-off management */
#define MCR_TTCM     ((u32)0x00000080) /* time triggered communication */

/* CAN Master Status Register bits */
#define MSR_INAK     ((u32)0x00000001)    /* Initialization acknowledge */
#define MSR_WKUI     ((u32)0x00000008)    /* Wake-up interrupt */
#define MSR_SLAKI    ((u32)0x00000010)    /* Sleep acknowledge interrupt */

/* CAN Transmit Status Register bits */
#define TSR_RQCP0    ((u32)0x00000001)    /* Request completed mailbox0 */
#define TSR_TXOK0    ((u32)0x00000002)    /* Transmission OK of mailbox0 */
#define TSR_ABRQ0    ((u32)0x00000080)    /* Abort request for mailbox0 */
#define TSR_RQCP1    ((u32)0x00000100)    /* Request completed mailbox1 */
#define TSR_TXOK1    ((u32)0x00000200)    /* Transmission OK of mailbox1 */
#define TSR_ABRQ1    ((u32)0x00008000)    /* Abort request for mailbox1 */
#define TSR_RQCP2    ((u32)0x00010000)    /* Request completed mailbox2 */
#define TSR_TXOK2    ((u32)0x00020000)    /* Transmission OK of mailbox2 */
#define TSR_ABRQ2    ((u32)0x00800000)    /* Abort request for mailbox2 */
#define TSR_TME0     ((u32)0x04000000)    /* Transmit mailbox 0 empty */
#define TSR_TME1     ((u32)0x08000000)    /* Transmit mailbox 1 empty */
#define TSR_TME2     ((u32)0x10000000)    /* Transmit mailbox 2 empty */

/* CAN Receive FIFO 0 Register bits */
#define RF0R_FULL0   ((u32)0x00000008)    /* FIFO 0 full */
#define RF0R_FOVR0   ((u32)0x00000010)    /* FIFO 0 overrun */
#define RF0R_RFOM0   ((u32)0x00000020)    /* Release FIFO 0 output mailbox */

/* CAN Receive FIFO 1 Register bits */
#define RF1R_FULL1   ((u32)0x00000008)    /* FIFO 1 full */
#define RF1R_FOVR1   ((u32)0x00000010)    /* FIFO 1 overrun */
#define RF1R_RFOM1   ((u32)0x00000020)    /* Release FIFO 1 output mailbox */

/* CAN Error Status Register bits */
#define ESR_EWGF     ((u32)0x00000001)    /* Error warning flag */
#define ESR_EPVF     ((u32)0x00000002)    /* Error passive flag */
#define ESR_BOFF     ((u32)0x00000004)    /* Bus-off flag */

/* CAN Mailbox Transmit Request */
#define TMIDxR_TXRQ  ((u32)0x00000001) /* Transmit mailbox request */

/* CAN Filter Master Register bits */
#define FMR_FINIT    ((u32)0x00000001) /* Filter init mode */


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static ITStatus CheckITStatus(u32 CAN_Reg, u32 It_Bit);

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : CAN_DeInit
* Description    : Deinitializes the CAN peripheral registers to their default
*                  reset values.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_DeInit(void)
{
  /* Enable CAN reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN, ENABLE);
  /* Release CAN from reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN, DISABLE);
}

/*******************************************************************************
* Function Name  : CAN_Init
* Description    : Initializes the CAN peripheral according to the specified
*                  parameters in the CAN_InitStruct.
* Input          : CAN_InitStruct: pointer to a CAN_InitTypeDef structure that
                   contains the configuration information for the CAN peripheral.
* Output         : None.
* Return         : Constant indicates initialization succeed which will be 
*                  CANINITFAILED or CANINITOK.
*******************************************************************************/
u8 CAN_Init(CAN_InitTypeDef* CAN_InitStruct)
{
  u8 InitStatus = 0;
  u16 WaitAck = 0;

  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_TTCM));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_ABOM));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_AWUM));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_NART));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_RFLM));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_TXFP));
  assert_param(IS_CAN_MODE(CAN_InitStruct->CAN_Mode));
  assert_param(IS_CAN_SJW(CAN_InitStruct->CAN_SJW));
  assert_param(IS_CAN_BS1(CAN_InitStruct->CAN_BS1));
  assert_param(IS_CAN_BS2(CAN_InitStruct->CAN_BS2));
  assert_param(IS_CAN_PRESCALER(CAN_InitStruct->CAN_Prescaler));

  /* Request initialisation */
  CAN->MCR = MCR_INRQ;

  /* ...and check acknowledged */
  if ((CAN->MSR & MSR_INAK) == 0)
  {
    InitStatus = CANINITFAILED;
  }
  else
  {
    /* Set the time triggered communication mode */
    if (CAN_InitStruct->CAN_TTCM == ENABLE)
    {
      CAN->MCR |= MCR_TTCM;
    }
    else
    {
      CAN->MCR &= ~MCR_TTCM;
    }

    /* Set the automatic bus-off management */
    if (CAN_InitStruct->CAN_ABOM == ENABLE)
    {
      CAN->MCR |= MCR_ABOM;
    }
    else
    {
      CAN->MCR &= ~MCR_ABOM;
    }

    /* Set the automatic wake-up mode */
    if (CAN_InitStruct->CAN_AWUM == ENABLE)
    {
      CAN->MCR |= MCR_AWUM;
    }
    else
    {
      CAN->MCR &= ~MCR_AWUM;
    }

    /* Set the no automatic retransmission */
    if (CAN_InitStruct->CAN_NART == ENABLE)
    {
      CAN->MCR |= MCR_NART;
    }
    else
    {
      CAN->MCR &= ~MCR_NART;
    }

    /* Set the receive FIFO locked mode */
    if (CAN_InitStruct->CAN_RFLM == ENABLE)
    {
      CAN->MCR |= MCR_RFLM;
    }
    else
    {
      CAN->MCR &= ~MCR_RFLM;
    }

    /* Set the transmit FIFO priority */
    if (CAN_InitStruct->CAN_TXFP == ENABLE)
    {
      CAN->MCR |= MCR_TXFP;
    }
    else
    {
      CAN->MCR &= ~MCR_TXFP;
    }

    /* Set the bit timing register */
    CAN->BTR = (u32)((u32)CAN_InitStruct->CAN_Mode << 30) | ((u32)CAN_InitStruct->CAN_SJW << 24) |
               ((u32)CAN_InitStruct->CAN_BS1 << 16) | ((u32)CAN_InitStruct->CAN_BS2 << 20) |
               ((u32)CAN_InitStruct->CAN_Prescaler - 1);

    InitStatus = CANINITOK;

    /* Request leave initialisation */
    CAN->MCR &= ~MCR_INRQ;

    /* Wait the acknowledge */
    for(WaitAck = 0x400; WaitAck > 0x0; WaitAck--)
    {
    }
    
    /* ...and check acknowledged */
    if ((CAN->MSR & MSR_INAK) == MSR_INAK)
    {
      InitStatus = CANINITFAILED;
    }
  }

  /* At this step, return the status of initialization */
  return InitStatus;
}

/*******************************************************************************
* Function Name  : CAN_FilterInit
* Description    : Initializes the CAN peripheral according to the specified
*                  parameters in the CAN_FilterInitStruct.
* Input          : CAN_FilterInitStruct: pointer to a CAN_FilterInitTypeDef
*                  structure that contains the configuration information.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct)
{
  u16 FilterNumber_BitPos = 0;

  /* Check the parameters */
  assert_param(IS_CAN_FILTER_NUMBER(CAN_FilterInitStruct->CAN_FilterNumber));
  assert_param(IS_CAN_FILTER_MODE(CAN_FilterInitStruct->CAN_FilterMode));
  assert_param(IS_CAN_FILTER_SCALE(CAN_FilterInitStruct->CAN_FilterScale));
  assert_param(IS_CAN_FILTER_FIFO(CAN_FilterInitStruct->CAN_FilterFIFOAssignment));
  assert_param(IS_FUNCTIONAL_STATE(CAN_FilterInitStruct->CAN_FilterActivation));

  FilterNumber_BitPos = 
  (u16)(((u16)0x0001) << ((u16)CAN_FilterInitStruct->CAN_FilterNumber));

  /* Initialisation mode for the filter */
  CAN->FMR |= FMR_FINIT;

  /* Filter Deactivation */
  CAN->FA1R &= ~(u32)FilterNumber_BitPos;

  /* Filter Scale */
  if (CAN_FilterInitStruct->CAN_FilterScale == CAN_FilterScale_16bit)
  {
    /* 16-bit scale for the filter */
    CAN->FS1R &= ~(u32)FilterNumber_BitPos;

    /* First 16-bit identifier and First 16-bit mask */
    /* Or First 16-bit identifier and Second 16-bit identifier */
    CAN->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR1 = 
    ((u32)((u32)0x0000FFFF & CAN_FilterInitStruct->CAN_FilterMaskIdLow) << 16) |
        ((u32)0x0000FFFF & CAN_FilterInitStruct->CAN_FilterIdLow);

    /* Second 16-bit identifier and Second 16-bit mask */
    /* Or Third 16-bit identifier and Fourth 16-bit identifier */
    CAN->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR2 = 
    ((u32)((u32)0x0000FFFF & CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
        ((u32)0x0000FFFF & CAN_FilterInitStruct->CAN_FilterIdHigh);
  }
  if (CAN_FilterInitStruct->CAN_FilterScale == CAN_FilterScale_32bit)
  {
    /* 32-bit scale for the filter */
    CAN->FS1R |= FilterNumber_BitPos;

    /* 32-bit identifier or First 32-bit identifier */
    CAN->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR1 = 
    ((u32)((u32)0x0000FFFF & CAN_FilterInitStruct->CAN_FilterIdHigh) << 16) |
        ((u32)0x0000FFFF & CAN_FilterInitStruct->CAN_FilterIdLow);

    /* 32-bit mask or Second 32-bit identifier */
    CAN->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR2 = 
    ((u32)((u32)0x0000FFFF & CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
        ((u32)0x0000FFFF & CAN_FilterInitStruct->CAN_FilterMaskIdLow);

  }

  /* Filter Mode */
  if (CAN_FilterInitStruct->CAN_FilterMode == CAN_FilterMode_IdMask)
  {
    /*Id/Mask mode for the filter*/
    CAN->FM1R &= ~(u32)FilterNumber_BitPos;
  }
  else /* CAN_FilterInitStruct->CAN_FilterMode == CAN_FilterMode_IdList */
  {
    /*Identifier list mode for the filter*/
    CAN->FM1R |= (u32)FilterNumber_BitPos;
  }

  /* Filter FIFO assignment */
  if (CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CAN_FilterFIFO0)
  {
    /* FIFO 0 assignation for the filter */
    CAN->FFA1R &= ~(u32)FilterNumber_BitPos;
  }
  if (CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CAN_FilterFIFO1)
  {
    /* FIFO 1 assignation for the filter */
    CAN->FFA1R |= (u32)FilterNumber_BitPos;
  }
  
  /* Filter activation */
  if (CAN_FilterInitStruct->CAN_FilterActivation == ENABLE)
  {
    CAN->FA1R |= FilterNumber_BitPos;
  }

  /* Leave the initialisation mode for the filter */
  CAN->FMR &= ~FMR_FINIT;
}

/*******************************************************************************
* Function Name  : CAN_StructInit
* Description    : Fills each CAN_InitStruct member with its default value.
* Input          : CAN_InitStruct: pointer to a CAN_InitTypeDef structure which
*                  will be initialized.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct)
{
  /* Reset CAN init structure parameters values */

  /* Initialize the time triggered communication mode */
  CAN_InitStruct->CAN_TTCM = DISABLE;

  /* Initialize the automatic bus-off management */
  CAN_InitStruct->CAN_ABOM = DISABLE;

  /* Initialize the automatic wake-up mode */
  CAN_InitStruct->CAN_AWUM = DISABLE;

  /* Initialize the no automatic retransmission */
  CAN_InitStruct->CAN_NART = DISABLE;

  /* Initialize the receive FIFO locked mode */
  CAN_InitStruct->CAN_RFLM = DISABLE;

  /* Initialize the transmit FIFO priority */
  CAN_InitStruct->CAN_TXFP = DISABLE;

  /* Initialize the CAN_Mode member */
  CAN_InitStruct->CAN_Mode = CAN_Mode_Normal;

  /* Initialize the CAN_SJW member */
  CAN_InitStruct->CAN_SJW = CAN_SJW_1tq;

  /* Initialize the CAN_BS1 member */
  CAN_InitStruct->CAN_BS1 = CAN_BS1_4tq;

  /* Initialize the CAN_BS2 member */
  CAN_InitStruct->CAN_BS2 = CAN_BS2_3tq;

  /* Initialize the CAN_Prescaler member */
  CAN_InitStruct->CAN_Prescaler = 1;
}

/*******************************************************************************
* Function Name  : CAN_ITConfig
* Description    : Enables or disables the specified CAN interrupts.
* Input          : - CAN_IT: specifies the CAN interrupt sources to be enabled or
*                    disabled.
*                    This parameter can be: CAN_IT_TME, CAN_IT_FMP0, CAN_IT_FF0,
*                                           CAN_IT_FOV0, CAN_IT_FMP1, CAN_IT_FF1,
*                                           CAN_IT_FOV1, CAN_IT_EWG, CAN_IT_EPV,
*                                           CAN_IT_LEC, CAN_IT_ERR, CAN_IT_WKU or
*                                           CAN_IT_SLK.
*                  - NewState: new state of the CAN interrupts.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_ITConfig(u32 CAN_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_CAN_ITConfig(CAN_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected CAN interrupt */
    CAN->IER |= CAN_IT;
  }
  else
  {
    /* Disable the selected CAN interrupt */
    CAN->IER &= ~CAN_IT;
  }
}

/*******************************************************************************
* Function Name  : CAN_Transmit
* Description    : Initiates the transmission of a message.
* Input          : TxMessage: pointer to a structure which contains CAN Id, CAN
*                  DLC and CAN datas.
* Output         : None.
* Return         : The number of the mailbox that is used for transmission
*                  or CAN_NO_MB if there is no empty mailbox.
*******************************************************************************/
u8 CAN_Transmit(CanTxMsg* TxMessage)
{
  u8 TransmitMailbox = 0;

  /* Check the parameters */
  assert_param(IS_CAN_STDID(TxMessage->StdId));
  assert_param(IS_CAN_EXTID(TxMessage->StdId));
  assert_param(IS_CAN_IDTYPE(TxMessage->IDE));
  assert_param(IS_CAN_RTR(TxMessage->RTR));
  assert_param(IS_CAN_DLC(TxMessage->DLC));

  /* Select one empty transmit mailbox */
  if ((CAN->TSR&TSR_TME0) == TSR_TME0)
  {
    TransmitMailbox = 0;
  }
  else if ((CAN->TSR&TSR_TME1) == TSR_TME1)
  {
    TransmitMailbox = 1;
  }
  else if ((CAN->TSR&TSR_TME2) == TSR_TME2)
  {
    TransmitMailbox = 2;
  }
  else
  {
    TransmitMailbox = CAN_NO_MB;
  }

  if (TransmitMailbox != CAN_NO_MB)
  {
    /* Set up the Id */
    CAN->sTxMailBox[TransmitMailbox].TIR &= TMIDxR_TXRQ;
    if (TxMessage->IDE == CAN_ID_STD)
    {
      TxMessage->StdId &= (u32)0x000007FF;
      TxMessage->StdId = TxMessage->StdId << 21;
      
      CAN->sTxMailBox[TransmitMailbox].TIR |= (TxMessage->StdId | TxMessage->IDE |
                                               TxMessage->RTR);
    }
    else
    {
      TxMessage->ExtId &= (u32)0x1FFFFFFF;
      TxMessage->ExtId <<= 3;

      CAN->sTxMailBox[TransmitMailbox].TIR |= (TxMessage->ExtId | TxMessage->IDE | 
                                               TxMessage->RTR);
    }
    
    /* Set up the DLC */
    TxMessage->DLC &= (u8)0x0000000F;
    CAN->sTxMailBox[TransmitMailbox].TDTR &= (u32)0xFFFFFFF0;
    CAN->sTxMailBox[TransmitMailbox].TDTR |= TxMessage->DLC;

    /* Set up the data field */
    CAN->sTxMailBox[TransmitMailbox].TDLR = (((u32)TxMessage->Data[3] << 24) | 
                                             ((u32)TxMessage->Data[2] << 16) |
                                             ((u32)TxMessage->Data[1] << 8) | 
                                             ((u32)TxMessage->Data[0]));
    CAN->sTxMailBox[TransmitMailbox].TDHR = (((u32)TxMessage->Data[7] << 24) | 
                                             ((u32)TxMessage->Data[6] << 16) |
                                             ((u32)TxMessage->Data[5] << 8) |
                                             ((u32)TxMessage->Data[4]));

    /* Request transmission */
    CAN->sTxMailBox[TransmitMailbox].TIR |= TMIDxR_TXRQ;
  }

  return TransmitMailbox;
}

/*******************************************************************************
* Function Name  : CAN_TransmitStatus
* Description    : Checks the transmission of a message.
* Input          : TransmitMailbox: the number of the mailbox that is used for
*                  transmission.
* Output         : None.
* Return         : CANTXOK if the CAN driver transmits the message, CANTXFAILED
*                  in an other case.
*******************************************************************************/
u8 CAN_TransmitStatus(u8 TransmitMailbox)
{
  /* RQCP, TXOK and TME bits */
  u8 State = 0;

  /* Check the parameters */
  assert_param(IS_CAN_TRANSMITMAILBOX(TransmitMailbox));

  switch (TransmitMailbox)
  {
    case (0): State |= (u8)((CAN->TSR & TSR_RQCP0) << 2);
      State |= (u8)((CAN->TSR & TSR_TXOK0) >> 0);
      State |= (u8)((CAN->TSR & TSR_TME0) >> 26);
      break;
    case (1): State |= (u8)((CAN->TSR & TSR_RQCP1) >> 6);
      State |= (u8)((CAN->TSR & TSR_TXOK1) >> 8);
      State |= (u8)((CAN->TSR & TSR_TME1) >> 27);
      break;
    case (2): State |= (u8)((CAN->TSR & TSR_RQCP2) >> 14);
      State |= (u8)((CAN->TSR & TSR_TXOK2) >> 16);
      State |= (u8)((CAN->TSR & TSR_TME2) >> 28);
      break;
    default:
      State = CANTXFAILED;
      break;
  }

  switch (State)
  {
      /* transmit pending  */
    case (0x0): State = CANTXPENDING;
      break;
      /* transmit failed  */
    case (0x5): State = CANTXFAILED;
      break;
      /* transmit succedeed  */
    case (0x7): State = CANTXOK;
      break;
    default:
      State = CANTXFAILED;
      break;
  }

  return State;
}

/*******************************************************************************
* Function Name  : CAN_CancelTransmit
* Description    : Cancels a transmit request.
* Input          : Mailbox number.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_CancelTransmit(u8 Mailbox)
{
  /* Check the parameters */
  assert_param(IS_CAN_TRANSMITMAILBOX(Mailbox));

  /* abort transmission */
  switch (Mailbox)
  {
    case (0): CAN->TSR |= TSR_ABRQ0;
      break;
    case (1): CAN->TSR |= TSR_ABRQ1;
      break;
    case (2): CAN->TSR |= TSR_ABRQ2;
      break;
    default:
      break;
  }
}

/*******************************************************************************
* Function Name  : CAN_FIFORelease
* Description    : Releases a FIFO.
* Input          : FIFONumber: FIFO to release, CAN_FIFO0 or CAN_FIFO1.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_FIFORelease(u8 FIFONumber)
{
  /* Check the parameters */
  assert_param(IS_CAN_FIFO(FIFONumber));

  /* Release FIFO0 */
  if (FIFONumber == CAN_FIFO0)
  {
    CAN->RF0R = RF0R_RFOM0;
  }
  /* Release FIFO1 */
  else /* FIFONumber == CAN_FIFO1 */
  {
    CAN->RF1R = RF1R_RFOM1;
  }
}

/*******************************************************************************
* Function Name  : CAN_MessagePending
* Description    : Returns the number of pending messages.
* Input          : FIFONumber: Receive FIFO number, CAN_FIFO0 or CAN_FIFO1.
* Output         : None.
* Return         : NbMessage which is the number of pending message.
*******************************************************************************/
u8 CAN_MessagePending(u8 FIFONumber)
{
  u8 MessagePending=0;

  /* Check the parameters */
  assert_param(IS_CAN_FIFO(FIFONumber));

  if (FIFONumber == CAN_FIFO0)
  {
    MessagePending = (u8)(CAN->RF0R&(u32)0x03);
  }
  else if (FIFONumber == CAN_FIFO1)
  {
    MessagePending = (u8)(CAN->RF1R&(u32)0x03);
  }
  else
  {
    MessagePending = 0;
  }
  return MessagePending;
}

/*******************************************************************************
* Function Name  : CAN_Receive
* Description    : Receives a message.
* Input          : FIFONumber: Receive FIFO number, CAN_FIFO0 or CAN_FIFO1.
* Output         : RxMessage: pointer to a structure which contains CAN Id,
*                  CAN DLC, CAN datas and FMI number.
* Return         : None.
*******************************************************************************/
void CAN_Receive(u8 FIFONumber, CanRxMsg* RxMessage)
{
  /* Check the parameters */
  assert_param(IS_CAN_FIFO(FIFONumber));

  /* Get the Id */
  RxMessage->IDE = (u8)0x04 & CAN->sFIFOMailBox[FIFONumber].RIR;
  if (RxMessage->IDE == CAN_ID_STD)
  {
    RxMessage->StdId = (u32)0x000007FF & (CAN->sFIFOMailBox[FIFONumber].RIR >> 21);
  }
  else
  {
    RxMessage->ExtId = (u32)0x1FFFFFFF & (CAN->sFIFOMailBox[FIFONumber].RIR >> 3);
  }
  
  RxMessage->RTR = (u8)0x02 & CAN->sFIFOMailBox[FIFONumber].RIR;

  /* Get the DLC */
  RxMessage->DLC = (u8)0x0F & CAN->sFIFOMailBox[FIFONumber].RDTR;

  /* Get the FMI */
  RxMessage->FMI = (u8)0xFF & (CAN->sFIFOMailBox[FIFONumber].RDTR >> 8);

  /* Get the data field */
  RxMessage->Data[0] = (u8)0xFF & CAN->sFIFOMailBox[FIFONumber].RDLR;
  RxMessage->Data[1] = (u8)0xFF & (CAN->sFIFOMailBox[FIFONumber].RDLR >> 8);
  RxMessage->Data[2] = (u8)0xFF & (CAN->sFIFOMailBox[FIFONumber].RDLR >> 16);
  RxMessage->Data[3] = (u8)0xFF & (CAN->sFIFOMailBox[FIFONumber].RDLR >> 24);

  RxMessage->Data[4] = (u8)0xFF & CAN->sFIFOMailBox[FIFONumber].RDHR;
  RxMessage->Data[5] = (u8)0xFF & (CAN->sFIFOMailBox[FIFONumber].RDHR >> 8);
  RxMessage->Data[6] = (u8)0xFF & (CAN->sFIFOMailBox[FIFONumber].RDHR >> 16);
  RxMessage->Data[7] = (u8)0xFF & (CAN->sFIFOMailBox[FIFONumber].RDHR >> 24);

  /* Release the FIFO */
  CAN_FIFORelease(FIFONumber);
}

/*******************************************************************************
* Function Name  : CAN_Sleep
* Description    : Enters the low power mode.
* Input          : None.
* Output         : None.
* Return         : CANSLEEPOK if sleep entered, CANSLEEPFAILED in an other case.
*******************************************************************************/
u8 CAN_Sleep(void)
{
  u8 SleepStatus = 0;

  /* Sleep mode entering request */
  CAN->MCR |= MCR_SLEEP;
  SleepStatus = CANSLEEPOK;

  /* Sleep mode status */
  if ((CAN->MCR&MCR_SLEEP) == 0)
  {
    /* Sleep mode not entered */
    SleepStatus = CANSLEEPFAILED;
  }

  /* At this step, sleep mode status */
  return SleepStatus;
}

/*******************************************************************************
* Function Name  : CAN_WakeUp
* Description    : Wakes the CAN up.
* Input          : None.
* Output         : None.
* Return         : CANWAKEUPOK if sleep mode left, CANWAKEUPFAILED in an other
*                  case.
*******************************************************************************/
u8 CAN_WakeUp(void)
{
  u8 WakeUpStatus = 0;

  /* Wake up request */
  CAN->MCR &= ~MCR_SLEEP;
  WakeUpStatus = CANWAKEUPFAILED;

  /* Sleep mode status */
  if ((CAN->MCR&MCR_SLEEP) == 0)
  {
    /* Sleep mode exited */
    WakeUpStatus = CANWAKEUPOK;
  }

  /* At this step, sleep mode status */
  return WakeUpStatus;
}

/*******************************************************************************
* Function Name  : CAN_GetFlagStatus
* Description    : Checks whether the specified CAN flag is set or not.
* Input          : CAN_FLAG: specifies the flag to check.
*                  This parameter can be: CAN_FLAG_EWG, CAN_FLAG_EPV or
*                                         CAN_FLAG_BOF.
* Output         : None.
* Return         : The new state of CAN_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus CAN_GetFlagStatus(u32 CAN_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_CAN_FLAG(CAN_FLAG));

  /* Check the status of the specified CAN flag */
  if ((CAN->ESR & CAN_FLAG) != (u32)RESET)
  {
    /* CAN_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* CAN_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the CAN_FLAG status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : CAN_ClearFlag
* Description    : Clears the CAN's pending flags.
* Input          : CAN_FLAG: specifies the flag to clear.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_ClearFlag(u32 CAN_FLAG)
{
  /* Check the parameters */
  assert_param(IS_CAN_FLAG(CAN_FLAG));

  /* Clear the selected CAN flags */
  CAN->ESR &= ~CAN_FLAG;
}

/*******************************************************************************
* Function Name  : CAN_GetITStatus
* Description    : Checks whether the specified CAN interrupt has occurred or 
*                  not.
* Input          : CAN_IT: specifies the CAN interrupt source to check.
*                  This parameter can be: CAN_IT_RQCP0, CAN_IT_RQCP1, CAN_IT_RQCP2,
*                                         CAN_IT_FF0, CAN_IT_FOV0, CAN_IT_FF1,
*                                         CAN_IT_FOV1, CAN_IT_EWG, CAN_IT_EPV, 
*                                         CAN_IT_BOF, CAN_IT_WKU or CAN_IT_SLK.
* Output         : None.
* Return         : The new state of CAN_IT (SET or RESET).
*******************************************************************************/
ITStatus CAN_GetITStatus(u32 CAN_IT)
{
  ITStatus pendingbitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_CAN_ITStatus(CAN_IT));

  switch (CAN_IT)
  {
    case CAN_IT_RQCP0:
      pendingbitstatus = CheckITStatus(CAN->TSR, TSR_RQCP0);
      break;
    case CAN_IT_RQCP1:
      pendingbitstatus = CheckITStatus(CAN->TSR, TSR_RQCP1);
      break;
    case CAN_IT_RQCP2:
      pendingbitstatus = CheckITStatus(CAN->TSR, TSR_RQCP2);
      break;
    case CAN_IT_FF0:
      pendingbitstatus = CheckITStatus(CAN->RF0R, RF0R_FULL0);
      break;
    case CAN_IT_FOV0:
      pendingbitstatus = CheckITStatus(CAN->RF0R, RF0R_FOVR0);
      break;
    case CAN_IT_FF1:
      pendingbitstatus = CheckITStatus(CAN->RF1R, RF1R_FULL1);
      break;
    case CAN_IT_FOV1:
      pendingbitstatus = CheckITStatus(CAN->RF1R, RF1R_FOVR1);
      break;
    case CAN_IT_EWG:
      pendingbitstatus = CheckITStatus(CAN->ESR, ESR_EWGF);
      break;
    case CAN_IT_EPV:
      pendingbitstatus = CheckITStatus(CAN->ESR, ESR_EPVF);
      break;
    case CAN_IT_BOF:
      pendingbitstatus = CheckITStatus(CAN->ESR, ESR_BOFF);
      break;
    case CAN_IT_SLK:
      pendingbitstatus = CheckITStatus(CAN->MSR, MSR_SLAKI);
      break;
    case CAN_IT_WKU:
      pendingbitstatus = CheckITStatus(CAN->MSR, MSR_WKUI);
      break;

    default :
      pendingbitstatus = RESET;
      break;
  }

  /* Return the CAN_IT status */
  return  pendingbitstatus;
}

/*******************************************************************************
* Function Name  : CAN_ClearITPendingBit
* Description    : Clears the CAN’s interrupt pending bits.
* Input          : CAN_IT: specifies the interrupt pending bit to clear.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CAN_ClearITPendingBit(u32 CAN_IT)
{
  /* Check the parameters */
  assert_param(IS_CAN_ITStatus(CAN_IT));

  switch (CAN_IT)
  {
    case CAN_IT_RQCP0:
      CAN->TSR = TSR_RQCP0; /* rc_w1*/
      break;
    case CAN_IT_RQCP1:
      CAN->TSR = TSR_RQCP1; /* rc_w1*/
      break;
    case CAN_IT_RQCP2:
      CAN->TSR = TSR_RQCP2; /* rc_w1*/
      break;
    case CAN_IT_FF0:
      CAN->RF0R = RF0R_FULL0; /* rc_w1*/
      break;
    case CAN_IT_FOV0:
      CAN->RF0R = RF0R_FOVR0; /* rc_w1*/
      break;
    case CAN_IT_FF1:
      CAN->RF1R = RF1R_FULL1; /* rc_w1*/
      break;
    case CAN_IT_FOV1:
      CAN->RF1R = RF1R_FOVR1; /* rc_w1*/
      break;
    case CAN_IT_EWG:
      CAN->ESR &= ~ ESR_EWGF; /* rw */
      break;
    case CAN_IT_EPV:
      CAN->ESR &= ~ ESR_EPVF; /* rw */
      break;
    case CAN_IT_BOF:
      CAN->ESR &= ~ ESR_BOFF; /* rw */
      break;
    case CAN_IT_WKU:
      CAN->MSR = MSR_WKUI;  /* rc_w1*/
      break;
    case CAN_IT_SLK:
      CAN->MSR = MSR_SLAKI;  /* rc_w1*/
      break;
    default :
      break;
  }
}

/*******************************************************************************
* Function Name  : CheckITStatus
* Description    : Checks whether the CAN interrupt has occurred or not.
* Input          : CAN_Reg: specifies the CAN interrupt register to check.
*                  It_Bit: specifies the interrupt source bit to check.
* Output         : None.
* Return         : The new state of the CAN Interrupt (SET or RESET).
*******************************************************************************/
static ITStatus CheckITStatus(u32 CAN_Reg, u32 It_Bit)
{
  ITStatus pendingbitstatus = RESET;

  if ((CAN_Reg & It_Bit) != (u32)RESET)
  {
    /* CAN_IT is set */
    pendingbitstatus = SET;
  }
  else
  {
    /* CAN_IT is reset */
    pendingbitstatus = RESET;
  }

  return pendingbitstatus;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_crc.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the CRC firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_crc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* CR register bit mask */
#define CR_RESET_Set    ((u32)0x00000001)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : CRC_ResetDR
* Description    : Resets the CRC Data register (DR).
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CRC_ResetDR(void)
{
  /* Reset CRC generator */
  CRC->CR = CR_RESET_Set;
}

/*******************************************************************************
* Function Name  : CRC_CalcCRC
* Description    : Computes the 32-bit CRC of a given data word(32-bit).
* Input          : - Data: data word(32-bit) to compute its CRC
* Output         : None
* Return         : 32-bit CRC
*******************************************************************************/
u32 CRC_CalcCRC(u32 Data)
{
  CRC->DR = Data;
  
  return (CRC->DR);
}

/*******************************************************************************
* Function Name  : CRC_CalcBlockCRC
* Description    : Computes the 32-bit CRC of a given buffer of data word(32-bit).
* Input          : - pBuffer: pointer to the buffer containing the data to be 
*                    computed
*                  - BufferLength: length of the buffer to be computed					
* Output         : None
* Return         : 32-bit CRC
*******************************************************************************/
u32 CRC_CalcBlockCRC(u32 pBuffer[], u32 BufferLength)
{
  u32 index = 0;
  
  for(index = 0; index < BufferLength; index++)
  {
    CRC->DR = pBuffer[index];
  }

  return (CRC->DR);
}

/*******************************************************************************
* Function Name  : CRC_GetCRC
* Description    : Returns the current CRC value.
* Input          : None
* Output         : None
* Return         : 32-bit CRC
*******************************************************************************/
u32 CRC_GetCRC(void)
{
  return (CRC->DR);
}

/*******************************************************************************
* Function Name  : CRC_SetIDRegister
* Description    : Stores a 8-bit data in the Independent Data(ID) register.
* Input          : - IDValue: 8-bit value to be stored in the ID register 					
* Output         : None
* Return         : None
*******************************************************************************/
void CRC_SetIDRegister(u8 IDValue)
{
  CRC->IDR = IDValue;
}

/*******************************************************************************
* Function Name  : CRC_GetIDRegister
* Description    : Returns the 8-bit data stored in the Independent Data(ID) register
* Input          : None
* Output         : None
* Return         : 8-bit value of the ID register 
*******************************************************************************/
u8 CRC_GetIDRegister(void)
{
  return (CRC->IDR);
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_dac.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the DAC firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_dac.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* DAC EN mask */
#define CR_EN_Set                  ((u32)0x00000001)

/* DAC DMAEN mask */
#define CR_DMAEN_Set               ((u32)0x00001000)

/* CR register Mask */
#define CR_CLEAR_Mask              ((u32)0x00000FFE)

/* DAC SWTRIG mask */
#define SWTRIGR_SWTRIG_Set         ((u32)0x00000001)

/* DAC Dual Channels SWTRIG masks */
#define DUAL_SWTRIG_Set            ((u32)0x00000003)
#define DUAL_SWTRIG_Reset          ((u32)0xFFFFFFFC)

/* DHR registers offsets */
#define DHR12R1_Offset             ((u32)0x00000008)
#define DHR12R2_Offset             ((u32)0x00000014)
#define DHR12RD_Offset             ((u32)0x00000020)

/* DOR register offset */
#define DOR_Offset                 ((u32)0x0000002C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : DAC_DeInit
* Description    : Deinitializes the DAC peripheral registers to their default
*                  reset values.
* Input          : None.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_DeInit(void)
{
  /* Enable DAC reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, ENABLE);
  /* Release DAC from reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, DISABLE);
}

/*******************************************************************************
* Function Name  : DAC_Init
* Description    : Initializes the DAC peripheral according to the specified 
*                  parameters in the DAC_InitStruct.
* Input          : - DAC_Channel: the selected DAC channel. 
*                    This parameter can be one of the following values:
*                       - DAC_Channel_1: DAC Channel1 selected
*                       - DAC_Channel_2: DAC Channel2 selected
*                  - DAC_InitStruct: pointer to a DAC_InitTypeDef structure that
*                    contains the configuration information for the specified
*                    DAC channel.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_Init(u32 DAC_Channel, DAC_InitTypeDef* DAC_InitStruct)
{
  u32 tmpreg1 = 0, tmpreg2 = 0;

  /* Check the DAC parameters */
  assert_param(IS_DAC_TRIGGER(DAC_InitStruct->DAC_Trigger));
  assert_param(IS_DAC_GENERATE_WAVE(DAC_InitStruct->DAC_WaveGeneration));
  assert_param(IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude));
  assert_param(IS_DAC_OUTPUT_BUFFER_STATE(DAC_InitStruct->DAC_OutputBuffer));

/*---------------------------- DAC CR Configuration --------------------------*/
  /* Get the DAC CR value */
  tmpreg1 = DAC->CR;
  /* Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits */
  tmpreg1 &= ~(CR_CLEAR_Mask << DAC_Channel);
  /* Configure for the selected DAC channel: buffer output, trigger, wave genration,
     mask/amplitude for wave genration */
  /* Set TSELx and TENx bits according to DAC_Trigger value */
  /* Set WAVEx bits according to DAC_WaveGeneration value */
  /* Set MAMPx bits according to DAC_LFSRUnmask_TriangleAmplitude value */ 
  /* Set BOFFx bit according to DAC_OutputBuffer value */   
  tmpreg2 = (DAC_InitStruct->DAC_Trigger | DAC_InitStruct->DAC_WaveGeneration |
             DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude | DAC_InitStruct->DAC_OutputBuffer);
  /* Calculate CR register value depending on DAC_Channel */
  tmpreg1 |= tmpreg2 << DAC_Channel;
  /* Write to DAC CR */
  DAC->CR = tmpreg1;
}

/*******************************************************************************
* Function Name  : DAC_StructInit
* Description    : Fills each DAC_InitStruct member with its default value.
* Input          : - DAC_InitStruct : pointer to a DAC_InitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct)
{
/*--------------- Reset DAC init structure parameters values -----------------*/
  /* Initialize the DAC_Trigger member */
  DAC_InitStruct->DAC_Trigger = DAC_Trigger_None;

  /* Initialize the DAC_WaveGeneration member */
  DAC_InitStruct->DAC_WaveGeneration = DAC_WaveGeneration_None;

  /* Initialize the DAC_LFSRUnmask_TriangleAmplitude member */
  DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;

  /* Initialize the DAC_OutputBuffer member */
  DAC_InitStruct->DAC_OutputBuffer = DAC_OutputBuffer_Enable;
}

/*******************************************************************************
* Function Name  : DAC_Cmd
* Description    : Enables or disables the specified DAC channel.
* Input            - DAC_Channel: the selected DAC channel. 
*                    This parameter can be one of the following values:
*                       - DAC_Channel_1: DAC Channel1 selected
*                       - DAC_Channel_2: DAC Channel2 selected
*                  - NewState: new state of the DAC channel. 
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_Cmd(u32 DAC_Channel, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected DAC channel */
    DAC->CR |= CR_EN_Set << DAC_Channel;
  }
  else
  {
    /* Disable the selected DAC channel */
    DAC->CR &= ~(CR_EN_Set << DAC_Channel);
  }
}

/*******************************************************************************
* Function Name  : DAC_DMACmd
* Description    : Enables or disables the specified DAC channel DMA request.
* Input            - DAC_Channel: the selected DAC channel. 
*                    This parameter can be one of the following values:
*                       - DAC_Channel_1: DAC Channel1 selected
*                       - DAC_Channel_2: DAC Channel2 selected
*                  - NewState: new state of the selected DAC channel DMA request.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_DMACmd(u32 DAC_Channel, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected DAC channel DMA request */
    DAC->CR |= CR_DMAEN_Set << DAC_Channel;
  }
  else
  {
    /* Disable the selected DAC channel DMA request */
    DAC->CR &= ~(CR_DMAEN_Set << DAC_Channel);
  }
}

/*******************************************************************************
* Function Name  : DAC_SoftwareTriggerCmd
* Description    : Enables or disables the selected DAC channel software trigger.
* Input            - DAC_Channel: the selected DAC channel. 
*                    This parameter can be one of the following values:
*                       - DAC_Channel_1: DAC Channel1 selected
*                       - DAC_Channel_2: DAC Channel2 selected
*                  - NewState: new state of the selected DAC channel software trigger.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_SoftwareTriggerCmd(u32 DAC_Channel, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable software trigger for the selected DAC channel */
    DAC->SWTRIGR |= SWTRIGR_SWTRIG_Set << (DAC_Channel >> 4);
  }
  else
  {
    /* Disable software trigger for the selected DAC channel */
    DAC->SWTRIGR &= ~(SWTRIGR_SWTRIG_Set << (DAC_Channel >> 4));
  }
}

/*******************************************************************************
* Function Name  : DAC_DualSoftwareTriggerCmd
* Description    : Enables or disables simultaneously the two DAC channels software
*                  triggers.
* Input            - NewState: new state of the DAC channels software triggers.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable software trigger for both DAC channels */
    DAC->SWTRIGR |= DUAL_SWTRIG_Set ;
  }
  else
  {
    /* Disable software trigger for both DAC channels */
    DAC->SWTRIGR &= DUAL_SWTRIG_Reset;
  }
}

/*******************************************************************************
* Function Name  : DAC_WaveGenerationCmd
* Description    : Enables or disables the selected DAC channel wave generation.
* Input            - DAC_Channel: the selected DAC channel. 
*                    This parameter can be one of the following values:
*                       - DAC_Channel_1: DAC Channel1 selected
*                       - DAC_Channel_2: DAC Channel2 selected
*                  - DAC_Wave: Specifies the wave type to enable or disable.
*                    This parameter can be one of the following values:
*                       - DAC_Wave_Noise: noise wave generation
*                       - DAC_Wave_Triangle: triangle wave generation
*                  - NewState: new state of the selected DAC channel wave generation.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_WaveGenerationCmd(u32 DAC_Channel, u32 DAC_Wave, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_WAVE(DAC_Wave)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected wave generation for the selected DAC channel */
    DAC->CR |= DAC_Wave << DAC_Channel;
  }
  else
  {
    /* Disable the selected wave generation for the selected DAC channel */
    DAC->CR &= ~(DAC_Wave << DAC_Channel);
  }
}

/*******************************************************************************
* Function Name  : DAC_SetChannel1Data
* Description    : Set the specified data holding register value for DAC channel1.
* Input          : - DAC_Align: Specifies the data alignement for DAC channel1.
*                    This parameter can be one of the following values:
*                       - DAC_Align_8b_R: 8bit right data alignement selected
*                       - DAC_Align_12b_L: 12bit left data alignement selected
*                       - DAC_Align_12b_R: 12bit right data alignement selected
*                  - Data : Data to be loaded in the selected data holding 
*                    register.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_SetChannel1Data(u32 DAC_Align, u16 Data)
{
  /* Check the parameters */
  assert_param(IS_DAC_ALIGN(DAC_Align));
  assert_param(IS_DAC_DATA(Data));

  /* Set the DAC channel1 selected data holding register */
  *((vu32 *)(DAC_BASE + DHR12R1_Offset + DAC_Align)) = (u32)Data;
}

/*******************************************************************************
* Function Name  : DAC_SetChannel2Data
* Description    : Set the specified data holding register value for DAC channel2.
* Input          : - DAC_Align: Specifies the data alignement for DAC channel2.
*                    This parameter can be one of the following values:
*                       - DAC_Align_8b_R: 8bit right data alignement selected
*                       - DAC_Align_12b_L: 12bit left data alignement selected
*                       - DAC_Align_12b_R: 12bit right data alignement selected
*                  - Data : Data to be loaded in the selected data holding 
*                    register.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_SetChannel2Data(u32 DAC_Align, u16 Data)
{
  /* Check the parameters */
  assert_param(IS_DAC_ALIGN(DAC_Align));
  assert_param(IS_DAC_DATA(Data));

  /* Set the DAC channel2 selected data holding register */
  *((vu32 *)(DAC_BASE + DHR12R2_Offset + DAC_Align)) = (u32)Data;
}

/*******************************************************************************
* Function Name  : DAC_SetDualChannelData
* Description    : Set the specified data holding register value for dual channel
*                  DAC.
* Input          : - DAC_Align: Specifies the data alignement for dual channel DAC.
*                    This parameter can be one of the following values:
*                       - DAC_Align_8b_R: 8bit right data alignement selected
*                       - DAC_Align_12b_L: 12bit left data alignement selected
*                       - DAC_Align_12b_R: 12bit right data alignement selected
*                  - Data2: Data for DAC Channel2 to be loaded in the selected data 
*                    holding register.
*                  - Data1: Data for DAC Channel1 to be loaded in the selected data 
*                    holding register.
* Output         : None
* Return         : None
*******************************************************************************/
void DAC_SetDualChannelData(u32 DAC_Align, u16 Data2, u16 Data1)
{
  u32 data = 0;

  /* Check the parameters */
  assert_param(IS_DAC_ALIGN(DAC_Align));
  assert_param(IS_DAC_DATA(Data1));
  assert_param(IS_DAC_DATA(Data2));
  
  /* Calculate and set dual DAC data holding register value */
  if (DAC_Align == DAC_Align_8b_R)
  {
    data = ((u32)Data2 << 8) | Data1; 
  }
  else
  {
    data = ((u32)Data2 << 16) | Data1;
  }

  /* Set the dual DAC selected data holding register */
  *((vu32 *)(DAC_BASE + DHR12RD_Offset + DAC_Align)) = data;
}

/*******************************************************************************
* Function Name  : DAC_GetDataOutputValue
* Description    : Returns the last data output value of the selected DAC cahnnel.
* Input            - DAC_Channel: the selected DAC channel. 
*                    This parameter can be one of the following values:
*                       - DAC_Channel_1: DAC Channel1 selected
*                       - DAC_Channel_2: DAC Channel2 selected
* Output         : None
* Return         : The selected DAC channel data output value.
*******************************************************************************/
u16 DAC_GetDataOutputValue(u32 DAC_Channel)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));

  /* Returns the DAC channel data output register value */
  return (u16) (*(vu32*)(DAC_BASE + DOR_Offset + ((u32)DAC_Channel >> 2)));
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_dbgmcu.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the DBGMCU firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_dbgmcu.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define IDCODE_DEVID_Mask    ((u32)0x00000FFF)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : DBGMCU_GetREVID
* Description    : Returns the device revision identifier.
* Input          : None
* Output         : None
* Return         : Device revision identifier
*******************************************************************************/
u32 DBGMCU_GetREVID(void)
{
   return(DBGMCU->IDCODE >> 16);
}

/*******************************************************************************
* Function Name  : DBGMCU_GetDEVID
* Description    : Returns the device identifier.
* Input          : None
* Output         : None
* Return         : Device identifier
*******************************************************************************/
u32 DBGMCU_GetDEVID(void)
{
   return(DBGMCU->IDCODE & IDCODE_DEVID_Mask);
}

/*******************************************************************************
* Function Name  : DBGMCU_Config
* Description    : Configures the specified peripheral and low power mode behavior
*                  when the MCU under Debug mode.
* Input          : - DBGMCU_Periph: specifies the peripheral and low power mode.
*                    This parameter can be any combination of the following values:
*                       - DBGMCU_SLEEP: Keep debugger connection during SLEEP mode              
*                       - DBGMCU_STOP: Keep debugger connection during STOP mode               
*                       - DBGMCU_STANDBY: Keep debugger connection during STANDBY mode            
*                       - DBGMCU_IWDG_STOP: Debug IWDG stopped when Core is halted          
*                       - DBGMCU_WWDG_STOP: Debug WWDG stopped when Core is halted          
*                       - DBGMCU_TIM1_STOP: TIM1 counter stopped when Core is halted          
*                       - DBGMCU_TIM2_STOP: TIM2 counter stopped when Core is halted          
*                       - DBGMCU_TIM3_STOP: TIM3 counter stopped when Core is halted          
*                       - DBGMCU_TIM4_STOP: TIM4 counter stopped when Core is halted          
*                       - DBGMCU_CAN_STOP: Debug CAN stopped when Core is halted           
*                       - DBGMCU_I2C1_SMBUS_TIMEOUT: I2C1 SMBUS timeout mode stopped
*                                                    when Core is halted
*                       - DBGMCU_I2C2_SMBUS_TIMEOUT: I2C2 SMBUS timeout mode stopped
*                                                    when Core is halted
*                       - DBGMCU_TIM5_STOP: TIM5 counter stopped when Core is halted          
*                       - DBGMCU_TIM6_STOP: TIM6 counter stopped when Core is halted          
*                       - DBGMCU_TIM7_STOP: TIM7 counter stopped when Core is halted          
*                       - DBGMCU_TIM8_STOP: TIM8 counter stopped when Core is halted          
*                  - NewState: new state of the specified peripheral in Debug mode.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void DBGMCU_Config(u32 DBGMCU_Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DBGMCU_PERIPH(DBGMCU_Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    DBGMCU->CR |= DBGMCU_Periph;
  }
  else
  {
    DBGMCU->CR &= ~DBGMCU_Periph;
  }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_dma.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the DMA firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* DMA ENABLE mask */
#define CCR_ENABLE_Set          ((u32)0x00000001)
#define CCR_ENABLE_Reset        ((u32)0xFFFFFFFE)

/* DMA1 Channelx interrupt pending bit masks */
#define DMA1_Channel1_IT_Mask    ((u32)0x0000000F)
#define DMA1_Channel2_IT_Mask    ((u32)0x000000F0)
#define DMA1_Channel3_IT_Mask    ((u32)0x00000F00)
#define DMA1_Channel4_IT_Mask    ((u32)0x0000F000)
#define DMA1_Channel5_IT_Mask    ((u32)0x000F0000)
#define DMA1_Channel6_IT_Mask    ((u32)0x00F00000)
#define DMA1_Channel7_IT_Mask    ((u32)0x0F000000)

/* DMA2 Channelx interrupt pending bit masks */
#define DMA2_Channel1_IT_Mask    ((u32)0x0000000F)
#define DMA2_Channel2_IT_Mask    ((u32)0x000000F0)
#define DMA2_Channel3_IT_Mask    ((u32)0x00000F00)
#define DMA2_Channel4_IT_Mask    ((u32)0x0000F000)
#define DMA2_Channel5_IT_Mask    ((u32)0x000F0000)

/* DMA2 FLAG mask */
#define FLAG_Mask                ((u32)0x10000000)

/* DMA registers Masks */
#define CCR_CLEAR_Mask           ((u32)0xFFFF800F)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : DMA_DeInit
* Description    : Deinitializes the DMAy Channelx registers to their default reset
*                  values.
* Input          : - DMAy_Channelx: where y can be 1 or 2 to select the DMA and
*                    x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the 
*                    DMA Channel.
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_DeInit(DMA_Channel_TypeDef* DMAy_Channelx)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));

  /* Disable the selected DMAy Channelx */
  DMAy_Channelx->CCR &= CCR_ENABLE_Reset;

  /* Reset DMAy Channelx control register */
  DMAy_Channelx->CCR  = 0;
  
  /* Reset DMAy Channelx remaining bytes register */
  DMAy_Channelx->CNDTR = 0;
  
  /* Reset DMAy Channelx peripheral address register */
  DMAy_Channelx->CPAR  = 0;
  
  /* Reset DMAy Channelx memory address register */
  DMAy_Channelx->CMAR = 0;

  switch (*(u32*)&DMAy_Channelx)
  {
    case DMA1_Channel1_BASE:
      /* Reset interrupt pending bits for DMA1 Channel1 */
      DMA1->IFCR |= DMA1_Channel1_IT_Mask;
      break;

    case DMA1_Channel2_BASE:
      /* Reset interrupt pending bits for DMA1 Channel2 */
      DMA1->IFCR |= DMA1_Channel2_IT_Mask;
      break;

    case DMA1_Channel3_BASE:
      /* Reset interrupt pending bits for DMA1 Channel3 */
      DMA1->IFCR |= DMA1_Channel3_IT_Mask;
      break;

    case DMA1_Channel4_BASE:
      /* Reset interrupt pending bits for DMA1 Channel4 */
      DMA1->IFCR |= DMA1_Channel4_IT_Mask;
      break;

    case DMA1_Channel5_BASE:
      /* Reset interrupt pending bits for DMA1 Channel5 */
      DMA1->IFCR |= DMA1_Channel5_IT_Mask;
      break;

    case DMA1_Channel6_BASE:
      /* Reset interrupt pending bits for DMA1 Channel6 */
      DMA1->IFCR |= DMA1_Channel6_IT_Mask;
      break;

    case DMA1_Channel7_BASE:
      /* Reset interrupt pending bits for DMA1 Channel7 */
      DMA1->IFCR |= DMA1_Channel7_IT_Mask;
      break;

    case DMA2_Channel1_BASE:
      /* Reset interrupt pending bits for DMA2 Channel1 */
      DMA2->IFCR |= DMA2_Channel1_IT_Mask;
      break;

    case DMA2_Channel2_BASE:
      /* Reset interrupt pending bits for DMA2 Channel2 */
      DMA2->IFCR |= DMA2_Channel2_IT_Mask;
      break;

    case DMA2_Channel3_BASE:
      /* Reset interrupt pending bits for DMA2 Channel3 */
      DMA2->IFCR |= DMA2_Channel3_IT_Mask;
      break;

    case DMA2_Channel4_BASE:
      /* Reset interrupt pending bits for DMA2 Channel4 */
      DMA2->IFCR |= DMA2_Channel4_IT_Mask;
      break;

    case DMA2_Channel5_BASE:
      /* Reset interrupt pending bits for DMA2 Channel5 */
      DMA2->IFCR |= DMA2_Channel5_IT_Mask;
      break;
      
    default:
      break;
  }
}

/*******************************************************************************
* Function Name  : DMA_Init
* Description    : Initializes the DMAy Channelx according to the specified
*                  parameters in the DMA_InitStruct.
* Input          : - DMAy_Channelx: where y can be 1 or 2 to select the DMA and 
*                    x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the 
*                    DMA Channel.
*                  - DMA_InitStruct: pointer to a DMA_InitTypeDef structure that
*                    contains the configuration information for the specified
*                    DMA Channel.
* Output         : None
* Return         : None
******************************************************************************/
void DMA_Init(DMA_Channel_TypeDef* DMAy_Channelx, DMA_InitTypeDef* DMA_InitStruct)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));
  assert_param(IS_DMA_DIR(DMA_InitStruct->DMA_DIR));
  assert_param(IS_DMA_BUFFER_SIZE(DMA_InitStruct->DMA_BufferSize));
  assert_param(IS_DMA_PERIPHERAL_INC_STATE(DMA_InitStruct->DMA_PeripheralInc));
  assert_param(IS_DMA_MEMORY_INC_STATE(DMA_InitStruct->DMA_MemoryInc));   
  assert_param(IS_DMA_PERIPHERAL_DATA_SIZE(DMA_InitStruct->DMA_PeripheralDataSize));
  assert_param(IS_DMA_MEMORY_DATA_SIZE(DMA_InitStruct->DMA_MemoryDataSize));
  assert_param(IS_DMA_MODE(DMA_InitStruct->DMA_Mode));
  assert_param(IS_DMA_PRIORITY(DMA_InitStruct->DMA_Priority));
  assert_param(IS_DMA_M2M_STATE(DMA_InitStruct->DMA_M2M));

/*--------------------------- DMAy Channelx CCR Configuration -----------------*/
  /* Get the DMAy_Channelx CCR value */
  tmpreg = DMAy_Channelx->CCR;
  /* Clear MEM2MEM, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
  tmpreg &= CCR_CLEAR_Mask;
  /* Configure DMAy Channelx: data transfer, data size, priority level and mode */
  /* Set DIR bit according to DMA_DIR value */
  /* Set CIRC bit according to DMA_Mode value */
  /* Set PINC bit according to DMA_PeripheralInc value */
  /* Set MINC bit according to DMA_MemoryInc value */
  /* Set PSIZE bits according to DMA_PeripheralDataSize value */
  /* Set MSIZE bits according to DMA_MemoryDataSize value */
  /* Set PL bits according to DMA_Priority value */
  /* Set the MEM2MEM bit according to DMA_M2M value */
  tmpreg |= DMA_InitStruct->DMA_DIR | DMA_InitStruct->DMA_Mode |
            DMA_InitStruct->DMA_PeripheralInc | DMA_InitStruct->DMA_MemoryInc |
            DMA_InitStruct->DMA_PeripheralDataSize | DMA_InitStruct->DMA_MemoryDataSize |
            DMA_InitStruct->DMA_Priority | DMA_InitStruct->DMA_M2M;
  /* Write to DMAy Channelx CCR */
  DMAy_Channelx->CCR = tmpreg;

/*--------------------------- DMAy Channelx CNDTR Configuration ---------------*/
  /* Write to DMAy Channelx CNDTR */
  DMAy_Channelx->CNDTR = DMA_InitStruct->DMA_BufferSize;

/*--------------------------- DMAy Channelx CPAR Configuration ----------------*/
  /* Write to DMAy Channelx CPAR */
  DMAy_Channelx->CPAR = DMA_InitStruct->DMA_PeripheralBaseAddr;

/*--------------------------- DMAy Channelx CMAR Configuration ----------------*/
  /* Write to DMAy Channelx CMAR */
  DMAy_Channelx->CMAR = DMA_InitStruct->DMA_MemoryBaseAddr;
}

/*******************************************************************************
* Function Name  : DMA_StructInit
* Description    : Fills each DMA_InitStruct member with its default value.
* Input          : - DMA_InitStruct : pointer to a DMA_InitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_StructInit(DMA_InitTypeDef* DMA_InitStruct)
{
/*-------------- Reset DMA init structure parameters values ------------------*/
  /* Initialize the DMA_PeripheralBaseAddr member */
  DMA_InitStruct->DMA_PeripheralBaseAddr = 0;

  /* Initialize the DMA_MemoryBaseAddr member */
  DMA_InitStruct->DMA_MemoryBaseAddr = 0;

  /* Initialize the DMA_DIR member */
  DMA_InitStruct->DMA_DIR = DMA_DIR_PeripheralSRC;

  /* Initialize the DMA_BufferSize member */
  DMA_InitStruct->DMA_BufferSize = 0;

  /* Initialize the DMA_PeripheralInc member */
  DMA_InitStruct->DMA_PeripheralInc = DMA_PeripheralInc_Disable;

  /* Initialize the DMA_MemoryInc member */
  DMA_InitStruct->DMA_MemoryInc = DMA_MemoryInc_Disable;

  /* Initialize the DMA_PeripheralDataSize member */
  DMA_InitStruct->DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

  /* Initialize the DMA_MemoryDataSize member */
  DMA_InitStruct->DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;

  /* Initialize the DMA_Mode member */
  DMA_InitStruct->DMA_Mode = DMA_Mode_Normal;

  /* Initialize the DMA_Priority member */
  DMA_InitStruct->DMA_Priority = DMA_Priority_Low;

  /* Initialize the DMA_M2M member */
  DMA_InitStruct->DMA_M2M = DMA_M2M_Disable;
}

/*******************************************************************************
* Function Name  : DMA_Cmd
* Description    : Enables or disables the specified DMAy Channelx.
* Input          : - DMAy_Channelx: where y can be 1 or 2 to select the DMA and 
*                    x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the 
*                    DMA Channel.
*                  - NewState: new state of the DMAy Channelx. 
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_Cmd(DMA_Channel_TypeDef* DMAy_Channelx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected DMAy Channelx */
    DMAy_Channelx->CCR |= CCR_ENABLE_Set;
  }
  else
  {
    /* Disable the selected DMAy Channelx */
    DMAy_Channelx->CCR &= CCR_ENABLE_Reset;
  }
}

/*******************************************************************************
* Function Name  : DMA_ITConfig
* Description    : Enables or disables the specified DMAy Channelx interrupts.
* Input          : - DMAy_Channelx: where y can be 1 or 2 to select the DMA and 
*                    x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the 
*                    DMA Channel.
*                  - DMA_IT: specifies the DMA interrupts sources to be enabled
*                    or disabled. 
*                    This parameter can be any combination of the following values:
*                       - DMA_IT_TC:  Transfer complete interrupt mask
*                       - DMA_IT_HT:  Half transfer interrupt mask
*                       - DMA_IT_TE:  Transfer error interrupt mask
*                  - NewState: new state of the specified DMA interrupts.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_ITConfig(DMA_Channel_TypeDef* DMAy_Channelx, u32 DMA_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));
  assert_param(IS_DMA_CONFIG_IT(DMA_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected DMA interrupts */
    DMAy_Channelx->CCR |= DMA_IT;
  }
  else
  {
    /* Disable the selected DMA interrupts */
    DMAy_Channelx->CCR &= ~DMA_IT;
  }
}

/*******************************************************************************
* Function Name  : DMA_GetCurrDataCounter
* Description    : Returns the number of remaining data units in the current
*                  DMAy Channelx transfer.
* Input          : - DMAy_Channelx: where y can be 1 or 2 to select the DMA and 
*                    x can be 1 to 7 for DMA1 and 1 to 5 for DMA2 to select the 
*                    DMA Channel.
* Output         : None
* Return         : The number of remaining data units in the current DMAy Channelx
*                  transfer.
*******************************************************************************/
u16 DMA_GetCurrDataCounter(DMA_Channel_TypeDef* DMAy_Channelx)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Channelx));

  /* Return the number of remaining data units for DMAy Channelx */
  return ((u16)(DMAy_Channelx->CNDTR));
}

/*******************************************************************************
* Function Name  : DMA_GetFlagStatus
* Description    : Checks whether the specified DMAy Channelx flag is set or not.
* Input          : - DMA_FLAG: specifies the flag to check.
*                    This parameter can be one of the following values:
*                       - DMA1_FLAG_GL1: DMA1 Channel1 global flag.
*                       - DMA1_FLAG_TC1: DMA1 Channel1 transfer complete flag.
*                       - DMA1_FLAG_HT1: DMA1 Channel1 half transfer flag.
*                       - DMA1_FLAG_TE1: DMA1 Channel1 transfer error flag.
*                       - DMA1_FLAG_GL2: DMA1 Channel2 global flag.
*                       - DMA1_FLAG_TC2: DMA1 Channel2 transfer complete flag.
*                       - DMA1_FLAG_HT2: DMA1 Channel2 half transfer flag.
*                       - DMA1_FLAG_TE2: DMA1 Channel2 transfer error flag.
*                       - DMA1_FLAG_GL3: DMA1 Channel3 global flag.
*                       - DMA1_FLAG_TC3: DMA1 Channel3 transfer complete flag.
*                       - DMA1_FLAG_HT3: DMA1 Channel3 half transfer flag.
*                       - DMA1_FLAG_TE3: DMA1 Channel3 transfer error flag.
*                       - DMA1_FLAG_GL4: DMA1 Channel4 global flag.
*                       - DMA1_FLAG_TC4: DMA1 Channel4 transfer complete flag.
*                       - DMA1_FLAG_HT4: DMA1 Channel4 half transfer flag.
*                       - DMA1_FLAG_TE4: DMA1 Channel4 transfer error flag.
*                       - DMA1_FLAG_GL5: DMA1 Channel5 global flag.
*                       - DMA1_FLAG_TC5: DMA1 Channel5 transfer complete flag.
*                       - DMA1_FLAG_HT5: DMA1 Channel5 half transfer flag.
*                       - DMA1_FLAG_TE5: DMA1 Channel5 transfer error flag.
*                       - DMA1_FLAG_GL6: DMA1 Channel6 global flag.
*                       - DMA1_FLAG_TC6: DMA1 Channel6 transfer complete flag.
*                       - DMA1_FLAG_HT6: DMA1 Channel6 half transfer flag.
*                       - DMA1_FLAG_TE6: DMA1 Channel6 transfer error flag.
*                       - DMA1_FLAG_GL7: DMA1 Channel7 global flag.
*                       - DMA1_FLAG_TC7: DMA1 Channel7 transfer complete flag.
*                       - DMA1_FLAG_HT7: DMA1 Channel7 half transfer flag.
*                       - DMA1_FLAG_TE7: DMA1 Channel7 transfer error flag.
*                       - DMA2_FLAG_GL1: DMA2 Channel1 global flag.
*                       - DMA2_FLAG_TC1: DMA2 Channel1 transfer complete flag.
*                       - DMA2_FLAG_HT1: DMA2 Channel1 half transfer flag.
*                       - DMA2_FLAG_TE1: DMA2 Channel1 transfer error flag.
*                       - DMA2_FLAG_GL2: DMA2 Channel2 global flag.
*                       - DMA2_FLAG_TC2: DMA2 Channel2 transfer complete flag.
*                       - DMA2_FLAG_HT2: DMA2 Channel2 half transfer flag.
*                       - DMA2_FLAG_TE2: DMA2 Channel2 transfer error flag.
*                       - DMA2_FLAG_GL3: DMA2 Channel3 global flag.
*                       - DMA2_FLAG_TC3: DMA2 Channel3 transfer complete flag.
*                       - DMA2_FLAG_HT3: DMA2 Channel3 half transfer flag.
*                       - DMA2_FLAG_TE3: DMA2 Channel3 transfer error flag.
*                       - DMA2_FLAG_GL4: DMA2 Channel4 global flag.
*                       - DMA2_FLAG_TC4: DMA2 Channel4 transfer complete flag.
*                       - DMA2_FLAG_HT4: DMA2 Channel4 half transfer flag.
*                       - DMA2_FLAG_TE4: DMA2 Channel4 transfer error flag.
*                       - DMA2_FLAG_GL5: DMA2 Channel5 global flag.
*                       - DMA2_FLAG_TC5: DMA2 Channel5 transfer complete flag.
*                       - DMA2_FLAG_HT5: DMA2 Channel5 half transfer flag.
*                       - DMA2_FLAG_TE5: DMA2 Channel5 transfer error flag.
* Output         : None
* Return         : The new state of DMA_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus DMA_GetFlagStatus(u32 DMA_FLAG)
{
  FlagStatus bitstatus = RESET;
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_DMA_GET_FLAG(DMA_FLAG));

  /* Calculate the used DMA */
  if ((DMA_FLAG & FLAG_Mask) != (u32)RESET)
  {
    /* Get DMA2 ISR register value */
    tmpreg = DMA2->ISR ;
  }
  else
  {
    /* Get DMA1 ISR register value */
    tmpreg = DMA1->ISR ;
  }

  /* Check the status of the specified DMA flag */
  if ((tmpreg & DMA_FLAG) != (u32)RESET)
  {
    /* DMA_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* DMA_FLAG is reset */
    bitstatus = RESET;
  }
  
  /* Return the DMA_FLAG status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : DMA_ClearFlag
* Description    : Clears the DMAy Channelx's pending flags.
* Input          : - DMA_FLAG: specifies the flag to clear.
*                    This parameter can be any combination (for the same DMA) of 
*                    the following values:
*                       - DMA1_FLAG_GL1: DMA1 Channel1 global flag.
*                       - DMA1_FLAG_TC1: DMA1 Channel1 transfer complete flag.
*                       - DMA1_FLAG_HT1: DMA1 Channel1 half transfer flag.
*                       - DMA1_FLAG_TE1: DMA1 Channel1 transfer error flag.
*                       - DMA1_FLAG_GL2: DMA1 Channel2 global flag.
*                       - DMA1_FLAG_TC2: DMA1 Channel2 transfer complete flag.
*                       - DMA1_FLAG_HT2: DMA1 Channel2 half transfer flag.
*                       - DMA1_FLAG_TE2: DMA1 Channel2 transfer error flag.
*                       - DMA1_FLAG_GL3: DMA1 Channel3 global flag.
*                       - DMA1_FLAG_TC3: DMA1 Channel3 transfer complete flag.
*                       - DMA1_FLAG_HT3: DMA1 Channel3 half transfer flag.
*                       - DMA1_FLAG_TE3: DMA1 Channel3 transfer error flag.
*                       - DMA1_FLAG_GL4: DMA1 Channel4 global flag.
*                       - DMA1_FLAG_TC4: DMA1 Channel4 transfer complete flag.
*                       - DMA1_FLAG_HT4: DMA1 Channel4 half transfer flag.
*                       - DMA1_FLAG_TE4: DMA1 Channel4 transfer error flag.
*                       - DMA1_FLAG_GL5: DMA1 Channel5 global flag.
*                       - DMA1_FLAG_TC5: DMA1 Channel5 transfer complete flag.
*                       - DMA1_FLAG_HT5: DMA1 Channel5 half transfer flag.
*                       - DMA1_FLAG_TE5: DMA1 Channel5 transfer error flag.
*                       - DMA1_FLAG_GL6: DMA1 Channel6 global flag.
*                       - DMA1_FLAG_TC6: DMA1 Channel6 transfer complete flag.
*                       - DMA1_FLAG_HT6: DMA1 Channel6 half transfer flag.
*                       - DMA1_FLAG_TE6: DMA1 Channel6 transfer error flag.
*                       - DMA1_FLAG_GL7: DMA1 Channel7 global flag.
*                       - DMA1_FLAG_TC7: DMA1 Channel7 transfer complete flag.
*                       - DMA1_FLAG_HT7: DMA1 Channel7 half transfer flag.
*                       - DMA1_FLAG_TE7: DMA1 Channel7 transfer error flag.
*                       - DMA2_FLAG_GL1: DMA2 Channel1 global flag.
*                       - DMA2_FLAG_TC1: DMA2 Channel1 transfer complete flag.
*                       - DMA2_FLAG_HT1: DMA2 Channel1 half transfer flag.
*                       - DMA2_FLAG_TE1: DMA2 Channel1 transfer error flag.
*                       - DMA2_FLAG_GL2: DMA2 Channel2 global flag.
*                       - DMA2_FLAG_TC2: DMA2 Channel2 transfer complete flag.
*                       - DMA2_FLAG_HT2: DMA2 Channel2 half transfer flag.
*                       - DMA2_FLAG_TE2: DMA2 Channel2 transfer error flag.
*                       - DMA2_FLAG_GL3: DMA2 Channel3 global flag.
*                       - DMA2_FLAG_TC3: DMA2 Channel3 transfer complete flag.
*                       - DMA2_FLAG_HT3: DMA2 Channel3 half transfer flag.
*                       - DMA2_FLAG_TE3: DMA2 Channel3 transfer error flag.
*                       - DMA2_FLAG_GL4: DMA2 Channel4 global flag.
*                       - DMA2_FLAG_TC4: DMA2 Channel4 transfer complete flag.
*                       - DMA2_FLAG_HT4: DMA2 Channel4 half transfer flag.
*                       - DMA2_FLAG_TE4: DMA2 Channel4 transfer error flag.
*                       - DMA2_FLAG_GL5: DMA2 Channel5 global flag.
*                       - DMA2_FLAG_TC5: DMA2 Channel5 transfer complete flag.
*                       - DMA2_FLAG_HT5: DMA2 Channel5 half transfer flag.
*                       - DMA2_FLAG_TE5: DMA2 Channel5 transfer error flag.
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_ClearFlag(u32 DMA_FLAG)
{
  /* Check the parameters */
  assert_param(IS_DMA_CLEAR_FLAG(DMA_FLAG));

  /* Calculate the used DMA */
  if ((DMA_FLAG & FLAG_Mask) != (u32)RESET)
  {
    /* Clear the selected DMA flags */
    DMA2->IFCR = DMA_FLAG;
  }
  else
  {
    /* Clear the selected DMA flags */
    DMA1->IFCR = DMA_FLAG;
  }
}

/*******************************************************************************
* Function Name  : DMA_GetITStatus
* Description    : Checks whether the specified DMAy Channelx interrupt has 
*                  occurred or not.
* Input          : - DMA_IT: specifies the DMA interrupt source to check. 
*                    This parameter can be one of the following values:
*                       - DMA1_IT_GL1: DMA1 Channel1 global interrupt.
*                       - DMA1_IT_TC1: DMA1 Channel1 transfer complete interrupt.
*                       - DMA1_IT_HT1: DMA1 Channel1 half transfer interrupt.
*                       - DMA1_IT_TE1: DMA1 Channel1 transfer error interrupt.
*                       - DMA1_IT_GL2: DMA1 Channel2 global interrupt.
*                       - DMA1_IT_TC2: DMA1 Channel2 transfer complete interrupt.
*                       - DMA1_IT_HT2: DMA1 Channel2 half transfer interrupt.
*                       - DMA1_IT_TE2: DMA1 Channel2 transfer error interrupt.
*                       - DMA1_IT_GL3: DMA1 Channel3 global interrupt.
*                       - DMA1_IT_TC3: DMA1 Channel3 transfer complete interrupt.
*                       - DMA1_IT_HT3: DMA1 Channel3 half transfer interrupt.
*                       - DMA1_IT_TE3: DMA1 Channel3 transfer error interrupt.
*                       - DMA1_IT_GL4: DMA1 Channel4 global interrupt.
*                       - DMA1_IT_TC4: DMA1 Channel4 transfer complete interrupt.
*                       - DMA1_IT_HT4: DMA1 Channel4 half transfer interrupt.
*                       - DMA1_IT_TE4: DMA1 Channel4 transfer error interrupt.
*                       - DMA1_IT_GL5: DMA1 Channel5 global interrupt.
*                       - DMA1_IT_TC5: DMA1 Channel5 transfer complete interrupt.
*                       - DMA1_IT_HT5: DMA1 Channel5 half transfer interrupt.
*                       - DMA1_IT_TE5: DMA1 Channel5 transfer error interrupt.
*                       - DMA1_IT_GL6: DMA1 Channel6 global interrupt.
*                       - DMA1_IT_TC6: DMA1 Channel6 transfer complete interrupt.
*                       - DMA1_IT_HT6: DMA1 Channel6 half transfer interrupt.
*                       - DMA1_IT_TE6: DMA1 Channel6 transfer error interrupt.
*                       - DMA1_IT_GL7: DMA1 Channel7 global interrupt.
*                       - DMA1_IT_TC7: DMA1 Channel7 transfer complete interrupt.
*                       - DMA1_IT_HT7: DMA1 Channel7 half transfer interrupt.
*                       - DMA1_IT_TE7: DMA1 Channel7 transfer error interrupt.
*                       - DMA2_IT_GL1: DMA2 Channel1 global interrupt.
*                       - DMA2_IT_TC1: DMA2 Channel1 transfer complete interrupt.
*                       - DMA2_IT_HT1: DMA2 Channel1 half transfer interrupt.
*                       - DMA2_IT_TE1: DMA2 Channel1 transfer error interrupt.
*                       - DMA2_IT_GL2: DMA2 Channel2 global interrupt.
*                       - DMA2_IT_TC2: DMA2 Channel2 transfer complete interrupt.
*                       - DMA2_IT_HT2: DMA2 Channel2 half transfer interrupt.
*                       - DMA2_IT_TE2: DMA2 Channel2 transfer error interrupt.
*                       - DMA2_IT_GL3: DMA2 Channel3 global interrupt.
*                       - DMA2_IT_TC3: DMA2 Channel3 transfer complete interrupt.
*                       - DMA2_IT_HT3: DMA2 Channel3 half transfer interrupt.
*                       - DMA2_IT_TE3: DMA2 Channel3 transfer error interrupt.
*                       - DMA2_IT_GL4: DMA2 Channel4 global interrupt.
*                       - DMA2_IT_TC4: DMA2 Channel4 transfer complete interrupt.
*                       - DMA2_IT_HT4: DMA2 Channel4 half transfer interrupt.
*                       - DMA2_IT_TE4: DMA2 Channel4 transfer error interrupt.
*                       - DMA2_IT_GL5: DMA2 Channel5 global interrupt.
*                       - DMA2_IT_TC5: DMA2 Channel5 transfer complete interrupt.
*                       - DMA2_IT_HT5: DMA2 Channel5 half transfer interrupt.
*                       - DMA2_IT_TE5: DMA2 Channel5 transfer error interrupt.
* Output         : None
* Return         : The new state of DMA_IT (SET or RESET).
*******************************************************************************/
ITStatus DMA_GetITStatus(u32 DMA_IT)
{
  ITStatus bitstatus = RESET;
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_DMA_GET_IT(DMA_IT));

  /* Calculate the used DMA */
  if ((DMA_IT & FLAG_Mask) != (u32)RESET)
  {
    /* Get DMA2 ISR register value */
    tmpreg = DMA2->ISR ;
  }
  else
  {
    /* Get DMA1 ISR register value */
    tmpreg = DMA1->ISR ;
  }

  /* Check the status of the specified DMA interrupt */
  if ((tmpreg & DMA_IT) != (u32)RESET)
  {
    /* DMA_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* DMA_IT is reset */
    bitstatus = RESET;
  }
  /* Return the DMA_IT status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : DMA_ClearITPendingBit
* Description    : Clears the DMAy Channelx’s interrupt pending bits.
* Input          : - DMA_IT: specifies the DMA interrupt pending bit to clear.
*                    This parameter can be any combination (for the same DMA) of
*                    the following values:
*                       - DMA1_IT_GL1: DMA1 Channel1 global interrupt.
*                       - DMA1_IT_TC1: DMA1 Channel1 transfer complete interrupt.
*                       - DMA1_IT_HT1: DMA1 Channel1 half transfer interrupt.
*                       - DMA1_IT_TE1: DMA1 Channel1 transfer error interrupt.
*                       - DMA1_IT_GL2: DMA1 Channel2 global interrupt.
*                       - DMA1_IT_TC2: DMA1 Channel2 transfer complete interrupt.
*                       - DMA1_IT_HT2: DMA1 Channel2 half transfer interrupt.
*                       - DMA1_IT_TE2: DMA1 Channel2 transfer error interrupt.
*                       - DMA1_IT_GL3: DMA1 Channel3 global interrupt.
*                       - DMA1_IT_TC3: DMA1 Channel3 transfer complete interrupt.
*                       - DMA1_IT_HT3: DMA1 Channel3 half transfer interrupt.
*                       - DMA1_IT_TE3: DMA1 Channel3 transfer error interrupt.
*                       - DMA1_IT_GL4: DMA1 Channel4 global interrupt.
*                       - DMA1_IT_TC4: DMA1 Channel4 transfer complete interrupt.
*                       - DMA1_IT_HT4: DMA1 Channel4 half transfer interrupt.
*                       - DMA1_IT_TE4: DMA1 Channel4 transfer error interrupt.
*                       - DMA1_IT_GL5: DMA1 Channel5 global interrupt.
*                       - DMA1_IT_TC5: DMA1 Channel5 transfer complete interrupt.
*                       - DMA1_IT_HT5: DMA1 Channel5 half transfer interrupt.
*                       - DMA1_IT_TE5: DMA1 Channel5 transfer error interrupt.
*                       - DMA1_IT_GL6: DMA1 Channel6 global interrupt.
*                       - DMA1_IT_TC6: DMA1 Channel6 transfer complete interrupt.
*                       - DMA1_IT_HT6: DMA1 Channel6 half transfer interrupt.
*                       - DMA1_IT_TE6: DMA1 Channel6 transfer error interrupt.
*                       - DMA1_IT_GL7: DMA1 Channel7 global interrupt.
*                       - DMA1_IT_TC7: DMA1 Channel7 transfer complete interrupt.
*                       - DMA1_IT_HT7: DMA1 Channel7 half transfer interrupt.
*                       - DMA1_IT_TE7: DMA1 Channel7 transfer error interrupt.
*                       - DMA2_IT_GL1: DMA2 Channel1 global interrupt.
*                       - DMA2_IT_TC1: DMA2 Channel1 transfer complete interrupt.
*                       - DMA2_IT_HT1: DMA2 Channel1 half transfer interrupt.
*                       - DMA2_IT_TE1: DMA2 Channel1 transfer error interrupt.
*                       - DMA2_IT_GL2: DMA2 Channel2 global interrupt.
*                       - DMA2_IT_TC2: DMA2 Channel2 transfer complete interrupt.
*                       - DMA2_IT_HT2: DMA2 Channel2 half transfer interrupt.
*                       - DMA2_IT_TE2: DMA2 Channel2 transfer error interrupt.
*                       - DMA2_IT_GL3: DMA2 Channel3 global interrupt.
*                       - DMA2_IT_TC3: DMA2 Channel3 transfer complete interrupt.
*                       - DMA2_IT_HT3: DMA2 Channel3 half transfer interrupt.
*                       - DMA2_IT_TE3: DMA2 Channel3 transfer error interrupt.
*                       - DMA2_IT_GL4: DMA2 Channel4 global interrupt.
*                       - DMA2_IT_TC4: DMA2 Channel4 transfer complete interrupt.
*                       - DMA2_IT_HT4: DMA2 Channel4 half transfer interrupt.
*                       - DMA2_IT_TE4: DMA2 Channel4 transfer error interrupt.
*                       - DMA2_IT_GL5: DMA2 Channel5 global interrupt.
*                       - DMA2_IT_TC5: DMA2 Channel5 transfer complete interrupt.
*                       - DMA2_IT_HT5: DMA2 Channel5 half transfer interrupt.
*                       - DMA2_IT_TE5: DMA2 Channel5 transfer error interrupt.
* Output         : None
* Return         : None
*******************************************************************************/
void DMA_ClearITPendingBit(u32 DMA_IT)
{
  /* Check the parameters */
  assert_param(IS_DMA_CLEAR_IT(DMA_IT));

  /* Calculate the used DMA */
  if ((DMA_IT & FLAG_Mask) != (u32)RESET)
  {
    /* Clear the selected DMA interrupt pending bits */
    DMA2->IFCR = DMA_IT;
  }
  else
  {
    /* Clear the selected DMA interrupt pending bits */
    DMA1->IFCR = DMA_IT;
  }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/














/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_exti.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the EXTI firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_exti.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define EXTI_LineNone    ((u32)0x00000)  /* No interrupt selected */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EXTI_DeInit
* Description    : Deinitializes the EXTI peripheral registers to their default 
*                  reset values.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI_DeInit(void)
{
  EXTI->IMR = 0x00000000;
  EXTI->EMR = 0x00000000;
  EXTI->RTSR = 0x00000000; 
  EXTI->FTSR = 0x00000000; 
  EXTI->PR = 0x0007FFFF;
}

/*******************************************************************************
* Function Name  : EXTI_Init
* Description    : Initializes the EXTI peripheral according to the specified
*                  parameters in the EXTI_InitStruct.
* Input          : - EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure
*                    that contains the configuration information for the EXTI
*                    peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct)
{
  /* Check the parameters */
  assert_param(IS_EXTI_MODE(EXTI_InitStruct->EXTI_Mode));
  assert_param(IS_EXTI_TRIGGER(EXTI_InitStruct->EXTI_Trigger));
  assert_param(IS_EXTI_LINE(EXTI_InitStruct->EXTI_Line));  
  assert_param(IS_FUNCTIONAL_STATE(EXTI_InitStruct->EXTI_LineCmd));
     
  if (EXTI_InitStruct->EXTI_LineCmd != DISABLE)
  {
    /* Clear EXTI line configuration */
    EXTI->IMR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->EMR &= ~EXTI_InitStruct->EXTI_Line;
    
    *(vu32 *)(EXTI_BASE + (u32)EXTI_InitStruct->EXTI_Mode)|= EXTI_InitStruct->EXTI_Line;

    /* Clear Rising Falling edge configuration */
    EXTI->RTSR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->FTSR &= ~EXTI_InitStruct->EXTI_Line;
    
    /* Select the trigger for the selected external interrupts */
    if (EXTI_InitStruct->EXTI_Trigger == EXTI_Trigger_Rising_Falling)
    {
      /* Rising Falling edge */
      EXTI->RTSR |= EXTI_InitStruct->EXTI_Line;
      EXTI->FTSR |= EXTI_InitStruct->EXTI_Line;
    }
    else
    {
      *(vu32 *)(EXTI_BASE + (u32)EXTI_InitStruct->EXTI_Trigger)|= EXTI_InitStruct->EXTI_Line;
    }
  }
  else
  {
    /* Disable the selected external lines */
    *(vu32 *)(EXTI_BASE + (u32)EXTI_InitStruct->EXTI_Mode)&= ~EXTI_InitStruct->EXTI_Line;
  }
}

/*******************************************************************************
* Function Name  : EXTI_StructInit
* Description    : Fills each EXTI_InitStruct member with its reset value.
* Input          : - EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI_StructInit(EXTI_InitTypeDef* EXTI_InitStruct)
{
  EXTI_InitStruct->EXTI_Line = EXTI_LineNone;
  EXTI_InitStruct->EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct->EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStruct->EXTI_LineCmd = DISABLE;
}

/*******************************************************************************
* Function Name  : EXTI_GenerateSWInterrupt
* Description    : Generates a Software interrupt.
* Input          : - EXTI_Line: specifies the EXTI lines to be enabled or
*                    disabled.
*                    This parameter can be any combination of EXTI_Linex where 
*                    x can be (0..18).
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI_GenerateSWInterrupt(u32 EXTI_Line)
{
  /* Check the parameters */
  assert_param(IS_EXTI_LINE(EXTI_Line));
  
  EXTI->SWIER |= EXTI_Line;
}

/*******************************************************************************
* Function Name  : EXTI_GetFlagStatus
* Description    : Checks whether the specified EXTI line flag is set or not.
* Input          : - EXTI_Line: specifies the EXTI line flag to check.
*                    This parameter can be:
*                       - EXTI_Linex: External interrupt line x where x(0..18)
* Output         : None
* Return         : The new state of EXTI_Line (SET or RESET).
*******************************************************************************/
FlagStatus EXTI_GetFlagStatus(u32 EXTI_Line)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_GET_EXTI_LINE(EXTI_Line));
  
  if ((EXTI->PR & EXTI_Line) != (u32)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : EXTI_ClearFlag
* Description    : Clears the EXTI’s line pending flags.
* Input          : - EXTI_Line: specifies the EXTI lines flags to clear.
*                    This parameter can be any combination of EXTI_Linex where 
*                    x can be (0..18).
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI_ClearFlag(u32 EXTI_Line)
{
  /* Check the parameters */
  assert_param(IS_EXTI_LINE(EXTI_Line));
  
  EXTI->PR = EXTI_Line;
}

/*******************************************************************************
* Function Name  : EXTI_GetITStatus
* Description    : Checks whether the specified EXTI line is asserted or not.
* Input          : - EXTI_Line: specifies the EXTI line to check.
*                    This parameter can be:
*                       - EXTI_Linex: External interrupt line x where x(0..18)
* Output         : None
* Return         : The new state of EXTI_Line (SET or RESET).
*******************************************************************************/
ITStatus EXTI_GetITStatus(u32 EXTI_Line)
{
  ITStatus bitstatus = RESET;
  u32 enablestatus = 0;

  /* Check the parameters */
  assert_param(IS_GET_EXTI_LINE(EXTI_Line));
  
  enablestatus =  EXTI->IMR & EXTI_Line;

  if (((EXTI->PR & EXTI_Line) != (u32)RESET) && (enablestatus != (u32)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : EXTI_ClearITPendingBit
* Description    : Clears the EXTI’s line pending bits.
* Input          : - EXTI_Line: specifies the EXTI lines to clear.
*                    This parameter can be any combination of EXTI_Linex where 
*                    x can be (0..18).
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI_ClearITPendingBit(u32 EXTI_Line)
{
  /* Check the parameters */
  assert_param(IS_EXTI_LINE(EXTI_Line));
  
  EXTI->PR = EXTI_Line;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/


















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_flash.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the FLASH firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_flash.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Flash Access Control Register bits */
#define ACR_LATENCY_Mask         ((u32)0x00000038)
#define ACR_HLFCYA_Mask          ((u32)0xFFFFFFF7)
#define ACR_PRFTBE_Mask          ((u32)0xFFFFFFEF)

#ifdef _FLASH_PROG
/* Flash Access Control Register bits */
#define ACR_PRFTBS_Mask          ((u32)0x00000020) 

/* Flash Control Register bits */
#define CR_PG_Set                ((u32)0x00000001)
#define CR_PG_Reset              ((u32)0x00001FFE) 

#define CR_PER_Set               ((u32)0x00000002)
#define CR_PER_Reset             ((u32)0x00001FFD)

#define CR_MER_Set               ((u32)0x00000004)
#define CR_MER_Reset             ((u32)0x00001FFB)

#define CR_OPTPG_Set             ((u32)0x00000010)
#define CR_OPTPG_Reset           ((u32)0x00001FEF)

#define CR_OPTER_Set             ((u32)0x00000020)
#define CR_OPTER_Reset           ((u32)0x00001FDF)

#define CR_STRT_Set              ((u32)0x00000040)
							 
#define CR_LOCK_Set              ((u32)0x00000080)

/* FLASH Mask */
#define RDPRT_Mask               ((u32)0x00000002)
#define WRP0_Mask                ((u32)0x000000FF)
#define WRP1_Mask                ((u32)0x0000FF00)
#define WRP2_Mask                ((u32)0x00FF0000)
#define WRP3_Mask                ((u32)0xFF000000)

/* FLASH Keys */
#define RDP_Key                  ((u16)0x00A5)
#define FLASH_KEY1               ((u32)0x45670123)
#define FLASH_KEY2               ((u32)0xCDEF89AB)

/* Delay definition */   
#define EraseTimeout             ((u32)0x00000FFF)
#define ProgramTimeout           ((u32)0x0000000F)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef _FLASH_PROG
static void delay(void);
#endif

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : FLASH_SetLatency
* Description    : Sets the code latency value.
* Input          : - FLASH_Latency: specifies the FLASH Latency value.
*                    This parameter can be one of the following values:
*                       - FLASH_Latency_0: FLASH Zero Latency cycle
*                       - FLASH_Latency_1: FLASH One Latency cycle
*                       - FLASH_Latency_2: FLASH Two Latency cycles
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_SetLatency(u32 FLASH_Latency)
{
  /* Check the parameters */
  assert_param(IS_FLASH_LATENCY(FLASH_Latency));
  
  /* Sets the Latency value */
  FLASH->ACR &= ACR_LATENCY_Mask;
  FLASH->ACR |= FLASH_Latency;
}

/*******************************************************************************
* Function Name  : FLASH_HalfCycleAccessCmd
* Description    : Enables or disables the Half cycle flash access.
* Input          : - FLASH_HalfCycle: specifies the FLASH Half cycle Access mode.
*                    This parameter can be one of the following values:
*                       - FLASH_HalfCycleAccess_Enable: FLASH Half Cycle Enable
*                       - FLASH_HalfCycleAccess_Disable: FLASH Half Cycle Disable
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_HalfCycleAccessCmd(u32 FLASH_HalfCycleAccess)
{
  /* Check the parameters */
  assert_param(IS_FLASH_HALFCYCLEACCESS_STATE(FLASH_HalfCycleAccess));
  
  /* Enable or disable the Half cycle access */
  FLASH->ACR &= ACR_HLFCYA_Mask;
  FLASH->ACR |= FLASH_HalfCycleAccess;
}

/*******************************************************************************
* Function Name  : FLASH_PrefetchBufferCmd
* Description    : Enables or disables the Prefetch Buffer.
* Input          : - FLASH_PrefetchBuffer: specifies the Prefetch buffer status.
*                    This parameter can be one of the following values:
*                       - FLASH_PrefetchBuffer_Enable: FLASH Prefetch Buffer Enable
*                       - FLASH_PrefetchBuffer_Disable: FLASH Prefetch Buffer Disable
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_PrefetchBufferCmd(u32 FLASH_PrefetchBuffer)
{
  /* Check the parameters */
  assert_param(IS_FLASH_PREFETCHBUFFER_STATE(FLASH_PrefetchBuffer));
  
  /* Enable or disable the Prefetch Buffer */
  FLASH->ACR &= ACR_PRFTBE_Mask;
  FLASH->ACR |= FLASH_PrefetchBuffer;
}

#ifdef _FLASH_PROG
/*******************************************************************************
* Function Name  : FLASH_Unlock
* Description    : Unlocks the FLASH Program Erase Controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_Unlock(void)
{
  /* Authorize the FPEC Access */
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
}

/*******************************************************************************
* Function Name  : FLASH_Lock
* Description    : Locks the FLASH Program Erase Controller.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_Lock(void)
{
  /* Set the Lock Bit to lock the FPEC and the FCR */
  FLASH->CR |= CR_LOCK_Set;
}

/*******************************************************************************
* Function Name  : FLASH_ErasePage
* Description    : Erases a specified FLASH page.
* Input          : - Page_Address: The page address to be erased.
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT.
*******************************************************************************/
FLASH_Status FLASH_ErasePage(u32 Page_Address)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Page_Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(EraseTimeout);
  
  if(status == FLASH_COMPLETE)
  { 
    /* if the previous operation is completed, proceed to erase the page */
    FLASH->CR|= CR_PER_Set;
    FLASH->AR = Page_Address; 
    FLASH->CR|= CR_STRT_Set;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(EraseTimeout);

    if(status != FLASH_BUSY)
    {
      /* if the erase operation is completed, disable the PER Bit */
      FLASH->CR &= CR_PER_Reset;
    }
  }
  /* Return the Erase Status */
  return status;
}

/*******************************************************************************
* Function Name  : FLASH_EraseAllPages
* Description    : Erases all FLASH pages.
* Input          : None
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT.
*******************************************************************************/
FLASH_Status FLASH_EraseAllPages(void)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(EraseTimeout);
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to erase all pages */
     FLASH->CR |= CR_MER_Set;
     FLASH->CR |= CR_STRT_Set;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(EraseTimeout);

    if(status != FLASH_BUSY)
    {
      /* if the erase operation is completed, disable the MER Bit */
      FLASH->CR &= CR_MER_Reset;
    }
  }	   
  /* Return the Erase Status */
  return status;
}

/*******************************************************************************
* Function Name  : FLASH_EraseOptionBytes
* Description    : Erases the FLASH option bytes.
* Input          : None
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT.
*******************************************************************************/
FLASH_Status FLASH_EraseOptionBytes(void)
{
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(EraseTimeout);

  if(status == FLASH_COMPLETE)
  {
    /* Authorize the small information block programming */
    FLASH->OPTKEYR = FLASH_KEY1;
    FLASH->OPTKEYR = FLASH_KEY2;
    
    /* if the previous operation is completed, proceed to erase the option bytes */
    FLASH->CR |= CR_OPTER_Set;
    FLASH->CR |= CR_STRT_Set;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(EraseTimeout);
    
    if(status == FLASH_COMPLETE)
    {
      /* if the erase operation is completed, disable the OPTER Bit */
      FLASH->CR &= CR_OPTER_Reset;
       
      /* Enable the Option Bytes Programming operation */
      FLASH->CR |= CR_OPTPG_Set;

      /* Enable the readout access */
      OB->RDP= RDP_Key; 

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(ProgramTimeout);
 
      if(status != FLASH_BUSY)
      {
        /* if the program operation is completed, disable the OPTPG Bit */
        FLASH->CR &= CR_OPTPG_Reset;
      }
    }
    else
    {
      if (status != FLASH_BUSY)
      {
        /* Disable the OPTPG Bit */
        FLASH->CR &= CR_OPTPG_Reset;
      }
    }  
  }
  /* Return the erase status */
  return status;
}

/*******************************************************************************
* Function Name  : FLASH_ProgramWord
* Description    : Programs a word at a specified address.
* Input          : - Address: specifies the address to be programmed.
*                  - Data: specifies the data to be programmed.
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT. 
*******************************************************************************/
FLASH_Status FLASH_ProgramWord(u32 Address, u32 Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(ProgramTimeout);
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new first 
    half word */
    FLASH->CR |= CR_PG_Set;
  
    *(vu16*)Address = (u16)Data;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(ProgramTimeout);
 
    if(status == FLASH_COMPLETE)
    {
      /* if the previous operation is completed, proceed to program the new second 
      half word */
      *(vu16*)(Address + 2) = Data >> 16;
    
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(ProgramTimeout);
        
      if(status != FLASH_BUSY)
      {
        /* Disable the PG Bit */
        FLASH->CR &= CR_PG_Reset;
      }
    }
    else
    {
      if (status != FLASH_BUSY)
      {
        /* Disable the PG Bit */
        FLASH->CR &= CR_PG_Reset;
      }
     }
  }
  /* Return the Program Status */
  return status;
}

/*******************************************************************************
* Function Name  : FLASH_ProgramHalfWord
* Description    : Programs a half word at a specified address.
* Input          : - Address: specifies the address to be programmed.
*                  - Data: specifies the data to be programmed.
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT. 
*******************************************************************************/
FLASH_Status FLASH_ProgramHalfWord(u32 Address, u16 Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FLASH_ADDRESS(Address));

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(ProgramTimeout);
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR |= CR_PG_Set;
  
    *(vu16*)Address = Data;
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(ProgramTimeout);

    if(status != FLASH_BUSY)
    {
      /* if the program operation is completed, disable the PG Bit */
      FLASH->CR &= CR_PG_Reset;
    }
  } 
  /* Return the Program Status */
  return status;
}

/*******************************************************************************
* Function Name  : FLASH_ProgramOptionByteData
* Description    : Programs a half word at a specified Option Byte Data address.
* Input          : - Address: specifies the address to be programmed.
*                    This parameter can be 0x1FFFF804 or 0x1FFFF806. 
*                  - Data: specifies the data to be programmed.
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT. 
*******************************************************************************/
FLASH_Status FLASH_ProgramOptionByteData(u32 Address, u8 Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_OB_DATA_ADDRESS(Address));

  status = FLASH_WaitForLastOperation(ProgramTimeout);

  if(status == FLASH_COMPLETE)
  {
    /* Authorize the small information block programming */
    FLASH->OPTKEYR = FLASH_KEY1;
    FLASH->OPTKEYR = FLASH_KEY2;

    /* Enables the Option Bytes Programming operation */
    FLASH->CR |= CR_OPTPG_Set; 
    *(vu16*)Address = Data;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(ProgramTimeout);

    if(status != FLASH_BUSY)
    {
      /* if the program operation is completed, disable the OPTPG Bit */
      FLASH->CR &= CR_OPTPG_Reset;
    }
  }    
  /* Return the Option Byte Data Program Status */
  return status;      
}

/*******************************************************************************
* Function Name  : FLASH_EnableWriteProtection
* Description    : Write protects the desired pages
* Input          : - FLASH_Pages: specifies the address of the pages to be 
*                    write protected. This parameter can be:
*                    - For STM32F10Xxx Medium-density devices (FLASH page size equal to 1 KB)
*                       - A value between FLASH_WRProt_Pages0to3 and 
*                         FLASH_WRProt_Pages124to127
*                    - For STM32F10Xxx High-density devices (FLASH page size equal to 2 KB) 
*                       - A value between FLASH_WRProt_Pages0to1 and
*                         FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to255 
*                       - FLASH_WRProt_AllPages
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT.
*******************************************************************************/
FLASH_Status FLASH_EnableWriteProtection(u32 FLASH_Pages)
{
  u16 WRP0_Data = 0xFFFF, WRP1_Data = 0xFFFF, WRP2_Data = 0xFFFF, WRP3_Data = 0xFFFF;
  
  FLASH_Status status = FLASH_COMPLETE;
  
  /* Check the parameters */
  assert_param(IS_FLASH_WRPROT_PAGE(FLASH_Pages));
  
  FLASH_Pages = (u32)(~FLASH_Pages);
  WRP0_Data = (vu16)(FLASH_Pages & WRP0_Mask);
  WRP1_Data = (vu16)((FLASH_Pages & WRP1_Mask) >> 8);
  WRP2_Data = (vu16)((FLASH_Pages & WRP2_Mask) >> 16);
  WRP3_Data = (vu16)((FLASH_Pages & WRP3_Mask) >> 24);
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(ProgramTimeout);
  
  if(status == FLASH_COMPLETE)
  {
    /* Authorizes the small information block programming */
    FLASH->OPTKEYR = FLASH_KEY1;
    FLASH->OPTKEYR = FLASH_KEY2;
    FLASH->CR |= CR_OPTPG_Set;

    if(WRP0_Data != 0xFF)
    {
      OB->WRP0 = WRP0_Data;
      
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(ProgramTimeout);
    }
    if((status == FLASH_COMPLETE) && (WRP1_Data != 0xFF))
    {
      OB->WRP1 = WRP1_Data;
      
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(ProgramTimeout);
    }

    if((status == FLASH_COMPLETE) && (WRP2_Data != 0xFF))
    {
      OB->WRP2 = WRP2_Data;
      
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(ProgramTimeout);
    }
    
    if((status == FLASH_COMPLETE)&& (WRP3_Data != 0xFF))
    {
      OB->WRP3 = WRP3_Data;
     
      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(ProgramTimeout);
    }
          
    if(status != FLASH_BUSY)
    {
      /* if the program operation is completed, disable the OPTPG Bit */
      FLASH->CR &= CR_OPTPG_Reset;
    }
  } 
  /* Return the write protection operation Status */
  return status;       
}

/*******************************************************************************
* Function Name  : FLASH_ReadOutProtection
* Description    : Enables or disables the read out protection.
*                  If the user has already programmed the other option bytes before 
*                  calling this function, he must re-program them since this 
*                  function erases all option bytes.
* Input          : - Newstate: new state of the ReadOut Protection.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT.
*******************************************************************************/
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  status = FLASH_WaitForLastOperation(EraseTimeout);

  if(status == FLASH_COMPLETE)
  {
    /* Authorizes the small information block programming */
    FLASH->OPTKEYR = FLASH_KEY1;
    FLASH->OPTKEYR = FLASH_KEY2;

    FLASH->CR |= CR_OPTER_Set;
    FLASH->CR |= CR_STRT_Set;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(EraseTimeout);

    if(status == FLASH_COMPLETE)
    {
      /* if the erase operation is completed, disable the OPTER Bit */
      FLASH->CR &= CR_OPTER_Reset;

      /* Enable the Option Bytes Programming operation */
      FLASH->CR |= CR_OPTPG_Set; 

      if(NewState != DISABLE)
      {
        OB->RDP = 0x00;
      }
      else
      {
        OB->RDP = RDP_Key;  
      }

      /* Wait for last operation to be completed */
      status = FLASH_WaitForLastOperation(EraseTimeout); 
    
      if(status != FLASH_BUSY)
      {
        /* if the program operation is completed, disable the OPTPG Bit */
        FLASH->CR &= CR_OPTPG_Reset;
      }
    }
    else 
    {
      if(status != FLASH_BUSY)
      {
        /* Disable the OPTER Bit */
        FLASH->CR &= CR_OPTER_Reset;
      }
    }
  }
  /* Return the protection operation Status */
  return status;      
}
  	
/*******************************************************************************
* Function Name  : FLASH_UserOptionByteConfig
* Description    : Programs the FLASH User Option Byte: IWDG_SW / RST_STOP /
*                  RST_STDBY.
* Input          : - OB_IWDG: Selects the IWDG mode
*                     This parameter can be one of the following values:
*                     - OB_IWDG_SW: Software IWDG selected
*                     - OB_IWDG_HW: Hardware IWDG selected
*                  - OB_STOP: Reset event when entering STOP mode.
*                     This parameter can be one of the following values:
*                     - OB_STOP_NoRST: No reset generated when entering in STOP
*                     - OB_STOP_RST: Reset generated when entering in STOP
*                  - OB_STDBY: Reset event when entering Standby mode.
*                    This parameter can be one of the following values:
*                     - OB_STDBY_NoRST: No reset generated when entering in STANDBY
*                     - OB_STDBY_RST: Reset generated when entering in STANDBY
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT.
*******************************************************************************/
FLASH_Status FLASH_UserOptionByteConfig(u16 OB_IWDG, u16 OB_STOP, u16 OB_STDBY)
{
  FLASH_Status status = FLASH_COMPLETE; 

  /* Check the parameters */
  assert_param(IS_OB_IWDG_SOURCE(OB_IWDG));
  assert_param(IS_OB_STOP_SOURCE(OB_STOP));
  assert_param(IS_OB_STDBY_SOURCE(OB_STDBY));

  /* Authorize the small information block programming */
  FLASH->OPTKEYR = FLASH_KEY1;
  FLASH->OPTKEYR = FLASH_KEY2;
  
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(ProgramTimeout);
  
  if(status == FLASH_COMPLETE)
  {  
    /* Enable the Option Bytes Programming operation */
    FLASH->CR |= CR_OPTPG_Set; 
           
    OB->USER = ( OB_IWDG | OB_STOP |OB_STDBY) | (u16)0xF8; 
  
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(ProgramTimeout);

    if(status != FLASH_BUSY)
    {
      /* if the program operation is completed, disable the OPTPG Bit */
      FLASH->CR &= CR_OPTPG_Reset;
    }
  }    
  /* Return the Option Byte program Status */
  return status;
}

/*******************************************************************************
* Function Name  : FLASH_GetUserOptionByte
* Description    : Returns the FLASH User Option Bytes values.
* Input          : None
* Output         : None
* Return         : The FLASH User Option Bytes values:IWDG_SW(Bit0), RST_STOP(Bit1)
*                  and RST_STDBY(Bit2).
*******************************************************************************/
u32 FLASH_GetUserOptionByte(void)
{
  /* Return the User Option Byte */
  return (u32)(FLASH->OBR >> 2);
}

/*******************************************************************************
* Function Name  : FLASH_GetWriteProtectionOptionByte
* Description    : Returns the FLASH Write Protection Option Bytes Register value.
* Input          : None
* Output         : None
* Return         : The FLASH Write Protection  Option Bytes Register value
*******************************************************************************/
u32 FLASH_GetWriteProtectionOptionByte(void)
{
  /* Return the Falsh write protection Register value */
  return (u32)(FLASH->WRPR);
}

/*******************************************************************************
* Function Name  : FLASH_GetReadOutProtectionStatus
* Description    : Checks whether the FLASH Read Out Protection Status is set 
*                  or not.
* Input          : None
* Output         : None
* Return         : FLASH ReadOut Protection Status(SET or RESET)
*******************************************************************************/
FlagStatus FLASH_GetReadOutProtectionStatus(void)
{
  FlagStatus readoutstatus = RESET;

  if ((FLASH->OBR & RDPRT_Mask) != (u32)RESET)
  {
    readoutstatus = SET;
  }
  else
  {
    readoutstatus = RESET;
  }
  return readoutstatus;
}

/*******************************************************************************
* Function Name  : FLASH_GetPrefetchBufferStatus
* Description    : Checks whether the FLASH Prefetch Buffer status is set or not.
* Input          : None
* Output         : None
* Return         : FLASH Prefetch Buffer Status (SET or RESET).
*******************************************************************************/
FlagStatus FLASH_GetPrefetchBufferStatus(void)
{
  FlagStatus bitstatus = RESET;
  
  if ((FLASH->ACR & ACR_PRFTBS_Mask) != (u32)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the new state of FLASH Prefetch Buffer Status (SET or RESET) */
  return bitstatus; 
}

/*******************************************************************************
* Function Name  : FLASH_ITConfig
* Description    : Enables or disables the specified FLASH interrupts.
* Input          : - FLASH_IT: specifies the FLASH interrupt sources to be 
*                    enabled or disabled.
*                    This parameter can be any combination of the following values:
*                       - FLASH_IT_ERROR: FLASH Error Interrupt
*                       - FLASH_IT_EOP: FLASH end of operation Interrupt
* Output         : None
* Return         : None 
*******************************************************************************/
void FLASH_ITConfig(u16 FLASH_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FLASH_IT(FLASH_IT)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if(NewState != DISABLE)
  {
    /* Enable the interrupt sources */
    FLASH->CR |= FLASH_IT;
  }
  else
  {
    /* Disable the interrupt sources */
    FLASH->CR &= ~(u32)FLASH_IT;
  }
}

/*******************************************************************************
* Function Name  : FLASH_GetFlagStatus
* Description    : Checks whether the specified FLASH flag is set or not.
* Input          : - FLASH_FLAG: specifies the FLASH flag to check.
*                     This parameter can be one of the following values:
*                    - FLASH_FLAG_BSY: FLASH Busy flag           
*                    - FLASH_FLAG_PGERR: FLASH Program error flag       
*                    - FLASH_FLAG_WRPRTERR: FLASH Write protected error flag      
*                    - FLASH_FLAG_EOP: FLASH End of Operation flag           
*                    - FLASH_FLAG_OPTERR:  FLASH Option Byte error flag     
* Output         : None
* Return         : The new state of FLASH_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus FLASH_GetFlagStatus(u16 FLASH_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_FLASH_GET_FLAG(FLASH_FLAG)) ;

  if(FLASH_FLAG == FLASH_FLAG_OPTERR) 
  {
    if((FLASH->OBR & FLASH_FLAG_OPTERR) != (u32)RESET)
    {
      bitstatus = SET;
    }
    else
    {
      bitstatus = RESET;
    }
  }
  else
  {
   if((FLASH->SR & FLASH_FLAG) != (u32)RESET)
    {
      bitstatus = SET;
    }
    else
    {
      bitstatus = RESET;
    }
  }
  /* Return the new state of FLASH_FLAG (SET or RESET) */
  return bitstatus;
}

/*******************************************************************************
* Function Name  : FLASH_ClearFlag
* Description    : Clears the FLASH’s pending flags.
* Input          : - FLASH_FLAG: specifies the FLASH flags to clear.
*                    This parameter can be any combination of the following values:
*                    - FLASH_FLAG_BSY: FLASH Busy flag           
*                    - FLASH_FLAG_PGERR: FLASH Program error flag       
*                    - FLASH_FLAG_WRPRTERR: FLASH Write protected error flag      
*                    - FLASH_FLAG_EOP: FLASH End of Operation flag           
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_ClearFlag(u16 FLASH_FLAG)
{
  /* Check the parameters */
  assert_param(IS_FLASH_CLEAR_FLAG(FLASH_FLAG)) ;
  
  /* Clear the flags */
  FLASH->SR = FLASH_FLAG;
}

/*******************************************************************************
* Function Name  : FLASH_GetStatus
* Description    : Returns the FLASH Status.
* Input          : None
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP or FLASH_COMPLETE
*******************************************************************************/
FLASH_Status FLASH_GetStatus(void)
{
  FLASH_Status flashstatus = FLASH_COMPLETE;
  
  if((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) 
  {
    flashstatus = FLASH_BUSY;
  }
  else 
  {  
    if(FLASH->SR & FLASH_FLAG_PGERR)
    { 
      flashstatus = FLASH_ERROR_PG;
    }
    else 
    {
      if(FLASH->SR & FLASH_FLAG_WRPRTERR)
      {
        flashstatus = FLASH_ERROR_WRP;
      }
      else
      {
        flashstatus = FLASH_COMPLETE;
      }
    }
  }
  /* Return the Flash Status */
  return flashstatus;
}

/*******************************************************************************
* Function Name  : FLASH_WaitForLastOperation
* Description    : Waits for a Flash operation to complete or a TIMEOUT to occur.
* Input          : - Timeout: FLASH progamming Timeout
* Output         : None
* Return         : FLASH Status: The returned value can be: FLASH_BUSY, 
*                  FLASH_ERROR_PG, FLASH_ERROR_WRP, FLASH_COMPLETE or 
*                  FLASH_TIMEOUT.
*******************************************************************************/
FLASH_Status FLASH_WaitForLastOperation(u32 Timeout)
{ 
  FLASH_Status status = FLASH_COMPLETE;
   
  /* Check for the Flash Status */
  status = FLASH_GetStatus();

  /* Wait for a Flash operation to complete or a TIMEOUT to occur */
  while((status == FLASH_BUSY) && (Timeout != 0x00))
  {
    delay();
    status = FLASH_GetStatus();
    Timeout--;
  }

  if(Timeout == 0x00 )
  {
    status = FLASH_TIMEOUT;
  }

  /* Return the operation status */
  return status;
}

/*******************************************************************************
* Function Name  : delay
* Description    : Inserts a time delay.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void delay(void)
{
  vu32 i = 0;

  for(i = 0xFF; i != 0; i--)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_fsmc.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the FSMC firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_fsmc.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* --------------------- FSMC registers bit mask ---------------------------- */
/* FSMC BCRx Mask */
#define BCR_MBKEN_Set                       ((u32)0x00000001)
#define BCR_MBKEN_Reset                     ((u32)0x000FFFFE)
#define BCR_FACCEN_Set                      ((u32)0x00000040)

/* FSMC PCRx Mask */
#define PCR_PBKEN_Set                       ((u32)0x00000004)
#define PCR_PBKEN_Reset                     ((u32)0x000FFFFB)
#define PCR_ECCEN_Set                       ((u32)0x00000040)
#define PCR_ECCEN_Reset                     ((u32)0x000FFFBF)
#define PCR_MemoryType_NAND                 ((u32)0x00000008)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : FSMC_NORSRAMDeInit
* Description    : Deinitializes the FSMC NOR/SRAM Banks registers to their default 
*                  reset values.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank1_NORSRAM1: FSMC Bank1 NOR/SRAM1  
*                       - FSMC_Bank1_NORSRAM2: FSMC Bank1 NOR/SRAM2 
*                       - FSMC_Bank1_NORSRAM3: FSMC Bank1 NOR/SRAM3 
*                       - FSMC_Bank1_NORSRAM4: FSMC Bank1 NOR/SRAM4                       
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NORSRAMDeInit(u32 FSMC_Bank)
{
  /* Check the parameter */
  assert_param(IS_FSMC_NORSRAM_BANK(FSMC_Bank));
  
  /* FSMC_Bank1_NORSRAM1 */
  if(FSMC_Bank == FSMC_Bank1_NORSRAM1)
  {
    FSMC_Bank1->BTCR[FSMC_Bank] = 0x000030DB;    
  }
  /* FSMC_Bank1_NORSRAM2,  FSMC_Bank1_NORSRAM3 or FSMC_Bank1_NORSRAM4 */
  else
  {   
    FSMC_Bank1->BTCR[FSMC_Bank] = 0x000030D2; 
  }

  FSMC_Bank1->BTCR[FSMC_Bank + 1] = 0x0FFFFFFF;
  FSMC_Bank1E->BWTR[FSMC_Bank] = 0x0FFFFFFF;  
}

/*******************************************************************************
* Function Name  : FSMC_NANDDeInit
* Description    : Deinitializes the FSMC NAND Banks registers to their default 
*                  reset values.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank2_NAND: FSMC Bank2 NAND 
*                       - FSMC_Bank3_NAND: FSMC Bank3 NAND                       
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NANDDeInit(u32 FSMC_Bank)
{
  /* Check the parameter */
  assert_param(IS_FSMC_NAND_BANK(FSMC_Bank));
  
  if(FSMC_Bank == FSMC_Bank2_NAND)
  {
    /* Set the FSMC_Bank2 registers to their reset values */
    FSMC_Bank2->PCR2 = 0x00000018;
    FSMC_Bank2->SR2 = 0x00000040;
    FSMC_Bank2->PMEM2 = 0xFCFCFCFC;
    FSMC_Bank2->PATT2 = 0xFCFCFCFC;  
  }
  /* FSMC_Bank3_NAND */  
  else
  {
    /* Set the FSMC_Bank3 registers to their reset values */
    FSMC_Bank3->PCR3 = 0x00000018;
    FSMC_Bank3->SR3 = 0x00000040;
    FSMC_Bank3->PMEM3 = 0xFCFCFCFC;
    FSMC_Bank3->PATT3 = 0xFCFCFCFC; 
  }  
}

/*******************************************************************************
* Function Name  : FSMC_PCCARDDeInit
* Description    : Deinitializes the FSMC PCCARD Bank registers to their default 
*                  reset values.
* Input          : None                       
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_PCCARDDeInit(void)
{
  /* Set the FSMC_Bank4 registers to their reset values */
  FSMC_Bank4->PCR4 = 0x00000018; 
  FSMC_Bank4->SR4 = 0x00000000;	
  FSMC_Bank4->PMEM4 = 0xFCFCFCFC;
  FSMC_Bank4->PATT4 = 0xFCFCFCFC;
  FSMC_Bank4->PIO4 = 0xFCFCFCFC;
}

/*******************************************************************************
* Function Name  : FSMC_NORSRAMInit
* Description    : Initializes the FSMC NOR/SRAM Banks according to the 
*                  specified parameters in the FSMC_NORSRAMInitStruct.
* Input          : - FSMC_NORSRAMInitStruct : pointer to a FSMC_NORSRAMInitTypeDef
*                  structure that contains the configuration information for 
*                  the FSMC NOR/SRAM specified Banks.                       
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NORSRAMInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct)
{ 
  /* Check the parameters */
  assert_param(IS_FSMC_NORSRAM_BANK(FSMC_NORSRAMInitStruct->FSMC_Bank));
  assert_param(IS_FSMC_MUX(FSMC_NORSRAMInitStruct->FSMC_DataAddressMux));
  assert_param(IS_FSMC_MEMORY(FSMC_NORSRAMInitStruct->FSMC_MemoryType));
  assert_param(IS_FSMC_MEMORY_WIDTH(FSMC_NORSRAMInitStruct->FSMC_MemoryDataWidth));
  assert_param(IS_FSMC_BURSTMODE(FSMC_NORSRAMInitStruct->FSMC_BurstAccessMode));
  assert_param(IS_FSMC_WAIT_POLARITY(FSMC_NORSRAMInitStruct->FSMC_WaitSignalPolarity));
  assert_param(IS_FSMC_WRAP_MODE(FSMC_NORSRAMInitStruct->FSMC_WrapMode));
  assert_param(IS_FSMC_WAIT_SIGNAL_ACTIVE(FSMC_NORSRAMInitStruct->FSMC_WaitSignalActive));
  assert_param(IS_FSMC_WRITE_OPERATION(FSMC_NORSRAMInitStruct->FSMC_WriteOperation));
  assert_param(IS_FSMC_WAITE_SIGNAL(FSMC_NORSRAMInitStruct->FSMC_WaitSignal));
  assert_param(IS_FSMC_EXTENDED_MODE(FSMC_NORSRAMInitStruct->FSMC_ExtendedMode));
  assert_param(IS_FSMC_WRITE_BURST(FSMC_NORSRAMInitStruct->FSMC_WriteBurst));  
  assert_param(IS_FSMC_ADDRESS_SETUP_TIME(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressSetupTime));
  assert_param(IS_FSMC_ADDRESS_HOLD_TIME(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressHoldTime));
  assert_param(IS_FSMC_DATASETUP_TIME(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataSetupTime));
  assert_param(IS_FSMC_TURNAROUND_TIME(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_BusTurnAroundDuration));
  assert_param(IS_FSMC_CLK_DIV(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_CLKDivision));
  assert_param(IS_FSMC_DATA_LATENCY(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataLatency));
  assert_param(IS_FSMC_ACCESS_MODE(FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AccessMode)); 
  
  /* Bank1 NOR/SRAM control register configuration */ 
  FSMC_Bank1->BTCR[FSMC_NORSRAMInitStruct->FSMC_Bank] = 
            (u32)FSMC_NORSRAMInitStruct->FSMC_DataAddressMux |
            FSMC_NORSRAMInitStruct->FSMC_MemoryType |
            FSMC_NORSRAMInitStruct->FSMC_MemoryDataWidth |
            FSMC_NORSRAMInitStruct->FSMC_BurstAccessMode |
            FSMC_NORSRAMInitStruct->FSMC_WaitSignalPolarity |
            FSMC_NORSRAMInitStruct->FSMC_WrapMode |
            FSMC_NORSRAMInitStruct->FSMC_WaitSignalActive |
            FSMC_NORSRAMInitStruct->FSMC_WriteOperation |
            FSMC_NORSRAMInitStruct->FSMC_WaitSignal |
            FSMC_NORSRAMInitStruct->FSMC_ExtendedMode |
            FSMC_NORSRAMInitStruct->FSMC_WriteBurst;

  if(FSMC_NORSRAMInitStruct->FSMC_MemoryType == FSMC_MemoryType_NOR)
  {
    FSMC_Bank1->BTCR[FSMC_NORSRAMInitStruct->FSMC_Bank] |= (u32)BCR_FACCEN_Set;
  }

  /* Bank1 NOR/SRAM timing register configuration */
  FSMC_Bank1->BTCR[FSMC_NORSRAMInitStruct->FSMC_Bank+1] = 
            (u32)FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressSetupTime |
            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressHoldTime << 4) |
            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataSetupTime << 8) |
            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_BusTurnAroundDuration << 16) |
            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_CLKDivision << 20) |
            (FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataLatency << 24) |
             FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AccessMode;
            

    
  /* Bank1 NOR/SRAM timing register for write configuration, if extended mode is used */
  if(FSMC_NORSRAMInitStruct->FSMC_ExtendedMode == FSMC_ExtendedMode_Enable)
  {
    assert_param(IS_FSMC_ADDRESS_SETUP_TIME(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressSetupTime));
    assert_param(IS_FSMC_ADDRESS_HOLD_TIME(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressHoldTime));
    assert_param(IS_FSMC_DATASETUP_TIME(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataSetupTime));
    assert_param(IS_FSMC_CLK_DIV(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_CLKDivision));
    assert_param(IS_FSMC_DATA_LATENCY(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataLatency));
    assert_param(IS_FSMC_ACCESS_MODE(FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AccessMode));

    FSMC_Bank1E->BWTR[FSMC_NORSRAMInitStruct->FSMC_Bank] = 
              (u32)FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressSetupTime |
              (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressHoldTime << 4 )|
              (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataSetupTime << 8) |
              (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_CLKDivision << 20) |
              (FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataLatency << 24) |
               FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AccessMode;
  }
  else
  {
    FSMC_Bank1E->BWTR[FSMC_NORSRAMInitStruct->FSMC_Bank] = 0x0FFFFFFF;
  }
}

/*******************************************************************************
* Function Name  : FSMC_NANDInit
* Description    : Initializes the FSMC NAND Banks according to the specified 
*                  parameters in the FSMC_NANDInitStruct.
* Input          : - FSMC_NANDInitStruct : pointer to a FSMC_NANDInitTypeDef 
*                    structure that contains the configuration information for 
*                    the FSMC NAND specified Banks.                       
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NANDInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct)
{
  u32 tmppcr = 0x00000000, tmppmem = 0x00000000, tmppatt = 0x00000000; 
    
  /* Check the parameters */
  assert_param( IS_FSMC_NAND_BANK(FSMC_NANDInitStruct->FSMC_Bank));
  assert_param( IS_FSMC_WAIT_FEATURE(FSMC_NANDInitStruct->FSMC_Waitfeature));
  assert_param( IS_FSMC_DATA_WIDTH(FSMC_NANDInitStruct->FSMC_MemoryDataWidth));
  assert_param( IS_FSMC_ECC_STATE(FSMC_NANDInitStruct->FSMC_ECC));
  assert_param( IS_FSMC_ECCPAGE_SIZE(FSMC_NANDInitStruct->FSMC_ECCPageSize));
  assert_param( IS_FSMC_ADDRESS_LOW_MAPPING(FSMC_NANDInitStruct->FSMC_AddressLowMapping));
  assert_param( IS_FSMC_TCLR_TIME(FSMC_NANDInitStruct->FSMC_TCLRSetupTime));
  assert_param( IS_FSMC_TAR_TIME(FSMC_NANDInitStruct->FSMC_TARSetupTime));

  assert_param(IS_FSMC_SETUP_TIME(FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_SetupTime));
  assert_param(IS_FSMC_WAIT_TIME(FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_WaitSetupTime));
  assert_param(IS_FSMC_HOLD_TIME(FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HoldSetupTime));
  assert_param(IS_FSMC_HIZ_TIME(FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HiZSetupTime));

  assert_param(IS_FSMC_SETUP_TIME(FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_SetupTime));
  assert_param(IS_FSMC_WAIT_TIME(FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_WaitSetupTime));
  assert_param(IS_FSMC_HOLD_TIME(FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HoldSetupTime));
  assert_param(IS_FSMC_HIZ_TIME(FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HiZSetupTime));
  
  /* Set the tmppcr value according to FSMC_NANDInitStruct parameters */
  tmppcr = (u32)FSMC_NANDInitStruct->FSMC_Waitfeature |
            PCR_MemoryType_NAND |
            FSMC_NANDInitStruct->FSMC_MemoryDataWidth |
            FSMC_NANDInitStruct->FSMC_ECC |
            FSMC_NANDInitStruct->FSMC_ECCPageSize |
            FSMC_NANDInitStruct->FSMC_AddressLowMapping |
            (FSMC_NANDInitStruct->FSMC_TCLRSetupTime << 9 )|
            (FSMC_NANDInitStruct->FSMC_TARSetupTime << 13);
            
  /* Set tmppmem value according to FSMC_CommonSpaceTimingStructure parameters */
  tmppmem = (u32)FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_SetupTime |
            (FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_WaitSetupTime << 8) |
            (FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HoldSetupTime << 16)|
            (FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HiZSetupTime << 24); 
            
  /* Set tmppatt value according to FSMC_AttributeSpaceTimingStructure parameters */
  tmppatt = (u32)FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_SetupTime |
            (FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_WaitSetupTime << 8) |
            (FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HoldSetupTime << 16)|
            (FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HiZSetupTime << 24);
  
  if(FSMC_NANDInitStruct->FSMC_Bank == FSMC_Bank2_NAND)
  {
    /* FSMC_Bank2_NAND registers configuration */
    FSMC_Bank2->PCR2 = tmppcr;
    FSMC_Bank2->PMEM2 = tmppmem;
    FSMC_Bank2->PATT2 = tmppatt;
  }
  else
  {
    /* FSMC_Bank3_NAND registers configuration */
    FSMC_Bank3->PCR3 = tmppcr;
    FSMC_Bank3->PMEM3 = tmppmem;
    FSMC_Bank3->PATT3 = tmppatt;
  }
}

/*******************************************************************************
* Function Name  : FSMC_PCCARDInit
* Description    : Initializes the FSMC PCCARD Bank according to the specified 
*                  parameters in the FSMC_PCCARDInitStruct.
* Input          : - FSMC_PCCARDInitStruct : pointer to a FSMC_PCCARDInitTypeDef
*                    structure that contains the configuration information for 
*                    the FSMC PCCARD Bank.                       
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_PCCARDInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct)
{
  /* Check the parameters */
  assert_param(IS_FSMC_WAIT_FEATURE(FSMC_PCCARDInitStruct->FSMC_Waitfeature));
  assert_param(IS_FSMC_ADDRESS_LOW_MAPPING(FSMC_PCCARDInitStruct->FSMC_AddressLowMapping));
  assert_param(IS_FSMC_TCLR_TIME(FSMC_PCCARDInitStruct->FSMC_TCLRSetupTime));
  assert_param(IS_FSMC_TAR_TIME(FSMC_PCCARDInitStruct->FSMC_TARSetupTime));

 
  assert_param(IS_FSMC_SETUP_TIME(FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_SetupTime));
  assert_param(IS_FSMC_WAIT_TIME(FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_WaitSetupTime));
  assert_param(IS_FSMC_HOLD_TIME(FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HoldSetupTime));
  assert_param(IS_FSMC_HIZ_TIME(FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HiZSetupTime));
  
  assert_param(IS_FSMC_SETUP_TIME(FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_SetupTime));
  assert_param(IS_FSMC_WAIT_TIME(FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_WaitSetupTime));
  assert_param(IS_FSMC_HOLD_TIME(FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HoldSetupTime));
  assert_param(IS_FSMC_HIZ_TIME(FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HiZSetupTime));

  assert_param(IS_FSMC_SETUP_TIME(FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_SetupTime));
  assert_param(IS_FSMC_WAIT_TIME(FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_WaitSetupTime));
  assert_param(IS_FSMC_HOLD_TIME(FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_HoldSetupTime));
  assert_param(IS_FSMC_HIZ_TIME(FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_HiZSetupTime));
  
  /* Set the PCR4 register value according to FSMC_PCCARDInitStruct parameters */
  FSMC_Bank4->PCR4 = (u32)FSMC_PCCARDInitStruct->FSMC_Waitfeature |
                     FSMC_MemoryDataWidth_16b |  
                     FSMC_PCCARDInitStruct->FSMC_AddressLowMapping |
                     (FSMC_PCCARDInitStruct->FSMC_TCLRSetupTime << 9) |
                     (FSMC_PCCARDInitStruct->FSMC_TARSetupTime << 13);
            
  /* Set PMEM4 register value according to FSMC_CommonSpaceTimingStructure parameters */
  FSMC_Bank4->PMEM4 = (u32)FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_SetupTime |
                      (FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_WaitSetupTime << 8) |
                      (FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HoldSetupTime << 16)|
                      (FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HiZSetupTime << 24); 
            
  /* Set PATT4 register value according to FSMC_AttributeSpaceTimingStructure parameters */
  FSMC_Bank4->PATT4 = (u32)FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_SetupTime |
                      (FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_WaitSetupTime << 8) |
                      (FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HoldSetupTime << 16)|
                      (FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HiZSetupTime << 24);	
            
  /* Set PIO4 register value according to FSMC_IOSpaceTimingStructure parameters */
  FSMC_Bank4->PIO4 = (u32)FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_SetupTime |
                     (FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_WaitSetupTime << 8) |
                     (FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_HoldSetupTime << 16)|
                     (FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_HiZSetupTime << 24);             
}

/*******************************************************************************
* Function Name  : FSMC_NORSRAMStructInit
* Description    : Fills each FSMC_NORSRAMInitStruct member with its default value.
* Input          : - FSMC_NORSRAMInitStruct: pointer to a FSMC_NORSRAMInitTypeDef 
*                    structure which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NORSRAMStructInit(FSMC_NORSRAMInitTypeDef* FSMC_NORSRAMInitStruct)
{  
  /* Reset NOR/SRAM Init structure parameters values */
  FSMC_NORSRAMInitStruct->FSMC_Bank = FSMC_Bank1_NORSRAM1;
  FSMC_NORSRAMInitStruct->FSMC_DataAddressMux = FSMC_DataAddressMux_Enable;
  FSMC_NORSRAMInitStruct->FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStruct->FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
  FSMC_NORSRAMInitStruct->FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStruct->FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStruct->FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStruct->FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStruct->FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStruct->FSMC_WaitSignal = FSMC_WaitSignal_Enable;
  FSMC_NORSRAMInitStruct->FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStruct->FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressSetupTime = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AddressHoldTime = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataSetupTime = 0xFF;
  FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_BusTurnAroundDuration = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_CLKDivision = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_DataLatency = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_ReadWriteTimingStruct->FSMC_AccessMode = FSMC_AccessMode_A; 
  FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressSetupTime = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AddressHoldTime = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataSetupTime = 0xFF;
  FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_BusTurnAroundDuration = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_CLKDivision = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_DataLatency = 0xF;
  FSMC_NORSRAMInitStruct->FSMC_WriteTimingStruct->FSMC_AccessMode = FSMC_AccessMode_A;
}

/*******************************************************************************
* Function Name  : FSMC_NANDStructInit
* Description    : Fills each FSMC_NANDInitStruct member with its default value.
* Input          : - FSMC_NORSRAMInitStruct: pointer to a FSMC_NANDInitTypeDef 
*                    structure which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NANDStructInit(FSMC_NANDInitTypeDef* FSMC_NANDInitStruct)
{ 
  /* Reset NAND Init structure parameters values */
  FSMC_NANDInitStruct->FSMC_Bank = FSMC_Bank2_NAND;
  FSMC_NANDInitStruct->FSMC_Waitfeature = FSMC_Waitfeature_Disable;
  FSMC_NANDInitStruct->FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_8b;
  FSMC_NANDInitStruct->FSMC_ECC = FSMC_ECC_Disable;
  FSMC_NANDInitStruct->FSMC_ECCPageSize = FSMC_ECCPageSize_256Bytes;
  FSMC_NANDInitStruct->FSMC_AddressLowMapping = FSMC_AddressLowMapping_Direct;
  FSMC_NANDInitStruct->FSMC_TCLRSetupTime = 0x0;
  FSMC_NANDInitStruct->FSMC_TARSetupTime = 0x0;
  FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_SetupTime = 0xFC;
  FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_WaitSetupTime = 0xFC;
  FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HoldSetupTime = 0xFC;
  FSMC_NANDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HiZSetupTime = 0xFC;
  FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_SetupTime = 0xFC;
  FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_WaitSetupTime = 0xFC;
  FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HoldSetupTime = 0xFC;
  FSMC_NANDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HiZSetupTime = 0xFC;	  
}

/*******************************************************************************
* Function Name  : FSMC_PCCARDStructInit
* Description    : Fills each FSMC_PCCARDInitStruct member with its default value.
* Input          : - FSMC_PCCARDInitStruct: pointer to a FSMC_PCCARDInitTypeDef 
*                    structure which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_PCCARDStructInit(FSMC_PCCARDInitTypeDef* FSMC_PCCARDInitStruct)
{
  /* Reset PCCARD Init structure parameters values */
  FSMC_PCCARDInitStruct->FSMC_Waitfeature = FSMC_Waitfeature_Disable;
  FSMC_PCCARDInitStruct->FSMC_AddressLowMapping = FSMC_AddressLowMapping_Direct;
  FSMC_PCCARDInitStruct->FSMC_TCLRSetupTime = 0x0;
  FSMC_PCCARDInitStruct->FSMC_TARSetupTime = 0x0;
  FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_SetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_WaitSetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HoldSetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_CommonSpaceTimingStruct->FSMC_HiZSetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_SetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_WaitSetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HoldSetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_AttributeSpaceTimingStruct->FSMC_HiZSetupTime = 0xFC;	
  FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_SetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_WaitSetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_HoldSetupTime = 0xFC;
  FSMC_PCCARDInitStruct->FSMC_IOSpaceTimingStruct->FSMC_HiZSetupTime = 0xFC;
}

/*******************************************************************************
* Function Name  : FSMC_NORSRAMCmd
* Description    : Enables or disables the specified NOR/SRAM Memory Bank.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank1_NORSRAM1: FSMC Bank1 NOR/SRAM1  
*                       - FSMC_Bank1_NORSRAM2: FSMC Bank1 NOR/SRAM2 
*                       - FSMC_Bank1_NORSRAM3: FSMC Bank1 NOR/SRAM3 
*                       - FSMC_Bank1_NORSRAM4: FSMC Bank1 NOR/SRAM4 
*                : - NewState: new state of the FSMC_Bank.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NORSRAMCmd(u32 FSMC_Bank, FunctionalState NewState)
{
  assert_param(IS_FSMC_NORSRAM_BANK(FSMC_Bank));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected NOR/SRAM Bank by setting the PBKEN bit in the BCRx register */
    FSMC_Bank1->BTCR[FSMC_Bank] |= BCR_MBKEN_Set;
  }
  else
  {
    /* Disable the selected NOR/SRAM Bank by clearing the PBKEN bit in the BCRx register */
    FSMC_Bank1->BTCR[FSMC_Bank] &= BCR_MBKEN_Reset;
  }
}

/*******************************************************************************
* Function Name  : FSMC_NANDCmd
* Description    : Enables or disables the specified NAND Memory Bank.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank2_NAND: FSMC Bank2 NAND 
*                       - FSMC_Bank3_NAND: FSMC Bank3 NAND
*                : - NewState: new state of the FSMC_Bank.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NANDCmd(u32 FSMC_Bank, FunctionalState NewState)
{
  assert_param(IS_FSMC_NAND_BANK(FSMC_Bank));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected NAND Bank by setting the PBKEN bit in the PCRx register */
    if(FSMC_Bank == FSMC_Bank2_NAND)
    {
      FSMC_Bank2->PCR2 |= PCR_PBKEN_Set;
    }
    else
    {
      FSMC_Bank3->PCR3 |= PCR_PBKEN_Set;
    }
  }
  else
  {
    /* Disable the selected NAND Bank by clearing the PBKEN bit in the PCRx register */
    if(FSMC_Bank == FSMC_Bank2_NAND)
    {
      FSMC_Bank2->PCR2 &= PCR_PBKEN_Reset;
    }
    else
    {
      FSMC_Bank3->PCR3 &= PCR_PBKEN_Reset;
    }
  }
}

/*******************************************************************************
* Function Name  : FSMC_PCCARDCmd
* Description    : Enables or disables the PCCARD Memory Bank.
* Input          : - NewState: new state of the PCCARD Memory Bank.  
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_PCCARDCmd(FunctionalState NewState)
{
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the PCCARD Bank by setting the PBKEN bit in the PCR4 register */
    FSMC_Bank4->PCR4 |= PCR_PBKEN_Set;
  }
  else
  {
    /* Disable the PCCARD Bank by clearing the PBKEN bit in the PCR4 register */
    FSMC_Bank4->PCR4 &= PCR_PBKEN_Reset;
  }
}

/*******************************************************************************
* Function Name  : FSMC_NANDECCCmd
* Description    : Enables or disables the FSMC NAND ECC feature.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank2_NAND: FSMC Bank2 NAND 
*                       - FSMC_Bank3_NAND: FSMC Bank3 NAND
*                : - NewState: new state of the FSMC NAND ECC feature.  
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_NANDECCCmd(u32 FSMC_Bank, FunctionalState NewState)
{
  assert_param(IS_FSMC_NAND_BANK(FSMC_Bank));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected NAND Bank ECC function by setting the ECCEN bit in the PCRx register */
    if(FSMC_Bank == FSMC_Bank2_NAND)
    {
      FSMC_Bank2->PCR2 |= PCR_ECCEN_Set;
    }
    else
    {
      FSMC_Bank3->PCR3 |= PCR_ECCEN_Set;
    }
  }
  else
  {
    /* Disable the selected NAND Bank ECC function by clearing the ECCEN bit in the PCRx register */
    if(FSMC_Bank == FSMC_Bank2_NAND)
    {
      FSMC_Bank2->PCR2 &= PCR_ECCEN_Reset;
    }
    else
    {
      FSMC_Bank3->PCR3 &= PCR_ECCEN_Reset;
    }
  }
}

/*******************************************************************************
* Function Name  : FSMC_GetECC
* Description    : Returns the error correction code register value.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank2_NAND: FSMC Bank2 NAND 
*                       - FSMC_Bank3_NAND: FSMC Bank3 NAND
* Output         : None
* Return         : The Error Correction Code (ECC) value.
*******************************************************************************/
u32 FSMC_GetECC(u32 FSMC_Bank)
{
  u32 eccval = 0x00000000;
  
  if(FSMC_Bank == FSMC_Bank2_NAND)
  {
    /* Get the ECCR2 register value */
    eccval = FSMC_Bank2->ECCR2;
  }
  else
  {
    /* Get the ECCR3 register value */
    eccval = FSMC_Bank3->ECCR3;
  }
  /* Return the error correction code value */
  return(eccval);
}

/*******************************************************************************
* Function Name  : FSMC_ITConfig
* Description    : Enables or disables the specified FSMC interrupts.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank2_NAND: FSMC Bank2 NAND 
*                       - FSMC_Bank3_NAND: FSMC Bank3 NAND
*                       - FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
*                  - FSMC_IT: specifies the FSMC interrupt sources to be
*                    enabled or disabled.
*                    This parameter can be any combination of the following values:
*                       - FSMC_IT_RisingEdge: Rising edge detection interrupt. 
*                       - FSMC_IT_Level: Level edge detection interrupt.                                  
*                       - FSMC_IT_FallingEdge: Falling edge detection interrupt.
*                  - NewState: new state of the specified FSMC interrupts.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_ITConfig(u32 FSMC_Bank, u32 FSMC_IT, FunctionalState NewState)
{
  assert_param(IS_FSMC_IT_BANK(FSMC_Bank));
  assert_param(IS_FSMC_IT(FSMC_IT));	
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected FSMC_Bank2 interrupts */
    if(FSMC_Bank == FSMC_Bank2_NAND)
    {
      FSMC_Bank2->SR2 |= FSMC_IT;
    }
    /* Enable the selected FSMC_Bank3 interrupts */
    else if (FSMC_Bank == FSMC_Bank3_NAND)
    {
      FSMC_Bank3->SR3 |= FSMC_IT;
    }
    /* Enable the selected FSMC_Bank4 interrupts */
    else
    {
      FSMC_Bank4->SR4 |= FSMC_IT;    
    }
  }
  else
  {
    /* Disable the selected FSMC_Bank2 interrupts */
    if(FSMC_Bank == FSMC_Bank2_NAND)
    {
      
      FSMC_Bank2->SR2 &= (u32)~FSMC_IT;
    }
    /* Disable the selected FSMC_Bank3 interrupts */
    else if (FSMC_Bank == FSMC_Bank3_NAND)
    {
      FSMC_Bank3->SR3 &= (u32)~FSMC_IT;
    }
    /* Disable the selected FSMC_Bank4 interrupts */
    else
    {
      FSMC_Bank4->SR4 &= (u32)~FSMC_IT;    
    }
  }
}
                  
/*******************************************************************************
* Function Name  : FSMC_GetFlagStatus
* Description    : Checks whether the specified FSMC flag is set or not.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank2_NAND: FSMC Bank2 NAND 
*                       - FSMC_Bank3_NAND: FSMC Bank3 NAND
*                       - FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
*                  - FSMC_FLAG: specifies the flag to check.
*                    This parameter can be one of the following values:
*                       - FSMC_FLAG_RisingEdge: Rising egde detection Flag.
*                       - FSMC_FLAG_Level: Level detection Flag.
*                       - FSMC_FLAG_FallingEdge: Falling egde detection Flag.
*                       - FSMC_FLAG_FEMPT: Fifo empty Flag. 
* Output         : None
* Return         : The new state of FSMC_FLAG (SET or RESET).
*******************************************************************************/                   
FlagStatus FSMC_GetFlagStatus(u32 FSMC_Bank, u32 FSMC_FLAG)
{
  FlagStatus bitstatus = RESET;
  u32 tmpsr = 0x00000000;
  
  /* Check the parameters */
  assert_param(IS_FSMC_GETFLAG_BANK(FSMC_Bank));
  assert_param(IS_FSMC_GET_FLAG(FSMC_FLAG));
  
  if(FSMC_Bank == FSMC_Bank2_NAND)
  {
    tmpsr = FSMC_Bank2->SR2;
  }  
  else if(FSMC_Bank == FSMC_Bank3_NAND)
  {
    tmpsr = FSMC_Bank3->SR3;
  }
  /* FSMC_Bank4_PCCARD*/
  else
  {
    tmpsr = FSMC_Bank4->SR4;
  } 
  
  /* Get the flag status */
  if ((tmpsr & FSMC_FLAG) != (u16)RESET )
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  /* Return the flag status */
  return bitstatus;
}

/*******************************************************************************
* Function Name  : FSMC_ClearFlag
* Description    : Clears the FSMC’s pending flags.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank2_NAND: FSMC Bank2 NAND 
*                       - FSMC_Bank3_NAND: FSMC Bank3 NAND
*                       - FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
*                  - FSMC_FLAG: specifies the flag to clear.
*                    This parameter can be any combination of the following values:
*                       - FSMC_FLAG_RisingEdge: Rising egde detection Flag.
*                       - FSMC_FLAG_Level: Level detection Flag.
*                       - FSMC_FLAG_FallingEdge: Falling egde detection Flag.
* Output         : None
* Return         : None
*******************************************************************************/                   
void FSMC_ClearFlag(u32 FSMC_Bank, u32 FSMC_FLAG)
{
 /* Check the parameters */
  assert_param(IS_FSMC_GETFLAG_BANK(FSMC_Bank));
  assert_param(IS_FSMC_CLEAR_FLAG(FSMC_FLAG)) ;
    
  if(FSMC_Bank == FSMC_Bank2_NAND)
  {
    FSMC_Bank2->SR2 &= ~FSMC_FLAG; 
  }  
  else if(FSMC_Bank == FSMC_Bank3_NAND)
  {
    FSMC_Bank3->SR3 &= ~FSMC_FLAG;
  }
  /* FSMC_Bank4_PCCARD*/
  else
  {
    FSMC_Bank4->SR4 &= ~FSMC_FLAG;
  }
}

/*******************************************************************************
* Function Name  : FSMC_GetITStatus
* Description    : Checks whether the specified FSMC interrupt has occurred or not.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank2_NAND: FSMC Bank2 NAND 
*                       - FSMC_Bank3_NAND: FSMC Bank3 NAND
*                       - FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
*                  - FSMC_IT: specifies the FSMC interrupt source to check.
*                    This parameter can be one of the following values:
*                       - FSMC_IT_RisingEdge: Rising edge detection interrupt. 
*                       - FSMC_IT_Level: Level edge detection interrupt.                                  
*                       - FSMC_IT_FallingEdge: Falling edge detection interrupt. 
* Output         : None
* Return         : The new state of FSMC_IT (SET or RESET).
*******************************************************************************/ 
ITStatus FSMC_GetITStatus(u32 FSMC_Bank, u32 FSMC_IT)
{
  ITStatus bitstatus = RESET;
  u32 tmpsr = 0x0, itstatus = 0x0, itenable = 0x0; 
  
  /* Check the parameters */
  assert_param(IS_FSMC_IT_BANK(FSMC_Bank));
  assert_param(IS_FSMC_GET_IT(FSMC_IT));
  
  if(FSMC_Bank == FSMC_Bank2_NAND)
  {
    tmpsr = FSMC_Bank2->SR2;
  }  
  else if(FSMC_Bank == FSMC_Bank3_NAND)
  {
    tmpsr = FSMC_Bank3->SR3;
  }
  /* FSMC_Bank4_PCCARD*/
  else
  {
    tmpsr = FSMC_Bank4->SR4;
  } 
  
  itstatus = tmpsr & FSMC_IT;
  
  itenable = tmpsr & (FSMC_IT >> 3);

  if ((itstatus != (u32)RESET)  && (itenable != (u32)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus; 
}

/*******************************************************************************
* Function Name  : FSMC_ClearITPendingBit
* Description    : Clears the FSMC’s interrupt pending bits.
* Input          : - FSMC_Bank: specifies the FSMC Bank to be used
*                    This parameter can be one of the following values:
*                       - FSMC_Bank2_NAND: FSMC Bank2 NAND 
*                       - FSMC_Bank3_NAND: FSMC Bank3 NAND
*                       - FSMC_Bank4_PCCARD: FSMC Bank4 PCCARD
*                  - FSMC_IT: specifies the interrupt pending bit to clear.
*                    This parameter can be any combination of the following values:
*                       - FSMC_IT_RisingEdge: Rising edge detection interrupt. 
*                       - FSMC_IT_Level: Level edge detection interrupt.                                  
*                       - FSMC_IT_FallingEdge: Falling edge detection interrupt.
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_ClearITPendingBit(u32 FSMC_Bank, u32 FSMC_IT)
{
  /* Check the parameters */
  assert_param(IS_FSMC_IT_BANK(FSMC_Bank));
  assert_param(IS_FSMC_IT(FSMC_IT));
    
  if(FSMC_Bank == FSMC_Bank2_NAND)
  {
    FSMC_Bank2->SR2 &= ~(FSMC_IT >> 3); 
  }  
  else if(FSMC_Bank == FSMC_Bank3_NAND)
  {
    FSMC_Bank3->SR3 &= ~(FSMC_IT >> 3);
  }
  /* FSMC_Bank4_PCCARD*/
  else
  {
    FSMC_Bank4->SR4 &= ~(FSMC_IT >> 3);
  }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_gpio.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the GPIO firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ------------ RCC registers bit address in the alias region ----------- */
#define AFIO_OFFSET                 (AFIO_BASE - PERIPH_BASE)

/* --- EVENTCR Register ---*/
/* Alias word address of EVOE bit */
#define EVCR_OFFSET                 (AFIO_OFFSET + 0x00)
#define EVOE_BitNumber              ((u8)0x07)
#define EVCR_EVOE_BB                (PERIPH_BB_BASE + (EVCR_OFFSET * 32) + (EVOE_BitNumber * 4))

#define EVCR_PORTPINCONFIG_MASK     ((u16)0xFF80)
#define LSB_MASK                    ((u16)0xFFFF)
#define DBGAFR_POSITION_MASK        ((u32)0x000F0000)
#define DBGAFR_SWJCFG_MASK          ((u32)0xF0FFFFFF)
#define DBGAFR_LOCATION_MASK        ((u32)0x00200000)
#define DBGAFR_NUMBITS_MASK         ((u32)0x00100000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : GPIO_DeInit
* Description    : Deinitializes the GPIOx peripheral registers to their default
*                  reset values.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_DeInit(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  switch (*(u32*)&GPIOx)
  {
    case GPIOA_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, DISABLE);
      break;

    case GPIOB_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, DISABLE);
      break;

    case GPIOC_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, DISABLE);
      break;

    case GPIOD_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, DISABLE);
      break;
      
    case GPIOE_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOE, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOE, DISABLE);
      break; 

    case GPIOF_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOF, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOF, DISABLE);
      break;

    case GPIOG_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOG, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOG, DISABLE);
      break;                       

    default:
      break;
  }
}

/*******************************************************************************
* Function Name  : GPIO_AFIODeInit
* Description    : Deinitializes the Alternate Functions (remap, event control
*                  and EXTI configuration) registers to their default reset
*                  values.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_AFIODeInit(void)
{
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, DISABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Init
* Description    : Initializes the GPIOx peripheral according to the specified
*                  parameters in the GPIO_InitStruct.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
*                  - GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that
*                    contains the configuration information for the specified GPIO
*                    peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
{
  u32 currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
  u32 tmpreg = 0x00, pinmask = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
  assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));  
  
/*---------------------------- GPIO Mode Configuration -----------------------*/
  currentmode = ((u32)GPIO_InitStruct->GPIO_Mode) & ((u32)0x0F);

  if ((((u32)GPIO_InitStruct->GPIO_Mode) & ((u32)0x10)) != 0x00)
  { 
    /* Check the parameters */
    assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));
    /* Output mode */
    currentmode |= (u32)GPIO_InitStruct->GPIO_Speed;
  }

/*---------------------------- GPIO CRL Configuration ------------------------*/
  /* Configure the eight low port pins */
  if (((u32)GPIO_InitStruct->GPIO_Pin & ((u32)0x00FF)) != 0x00)
  {
    tmpreg = GPIOx->CRL;

    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = ((u32)0x01) << pinpos;
      /* Get the port pins position */
      currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;

      if (currentpin == pos)
      {
        pos = pinpos << 2;
        /* Clear the corresponding low control register bits */
        pinmask = ((u32)0x0F) << pos;
        tmpreg &= ~pinmask;

        /* Write the mode configuration in the corresponding bits */
        tmpreg |= (currentmode << pos);

        /* Reset the corresponding ODR bit */
        if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
        {
          GPIOx->BRR = (((u32)0x01) << pinpos);
        }
        else
        {
          /* Set the corresponding ODR bit */
          if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
          {
            GPIOx->BSRR = (((u32)0x01) << pinpos);
          }
        }
      }
    }
    GPIOx->CRL = tmpreg;
  }

/*---------------------------- GPIO CRH Configuration ------------------------*/
  /* Configure the eight high port pins */
  if (GPIO_InitStruct->GPIO_Pin > 0x00FF)
  {
    tmpreg = GPIOx->CRH;
    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = (((u32)0x01) << (pinpos + 0x08));
      /* Get the port pins position */
      currentpin = ((GPIO_InitStruct->GPIO_Pin) & pos);
      if (currentpin == pos)
      {
        pos = pinpos << 2;
        /* Clear the corresponding high control register bits */
        pinmask = ((u32)0x0F) << pos;
        tmpreg &= ~pinmask;

        /* Write the mode configuration in the corresponding bits */
        tmpreg |= (currentmode << pos);

        /* Reset the corresponding ODR bit */
        if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
        {
          GPIOx->BRR = (((u32)0x01) << (pinpos + 0x08));
        }
        /* Set the corresponding ODR bit */
        if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
        {
          GPIOx->BSRR = (((u32)0x01) << (pinpos + 0x08));
        }
      }
    }
    GPIOx->CRH = tmpreg;
  }
}

/*******************************************************************************
* Function Name  : GPIO_StructInit
* Description    : Fills each GPIO_InitStruct member with its default value.
* Input          : - GPIO_InitStruct : pointer to a GPIO_InitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct)
{
  /* Reset GPIO init structure parameters values */
  GPIO_InitStruct->GPIO_Pin  = GPIO_Pin_All;
  GPIO_InitStruct->GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct->GPIO_Mode = GPIO_Mode_IN_FLOATING;
}

/*******************************************************************************
* Function Name  : GPIO_ReadInputDataBit
* Description    : Reads the specified input port pin.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
*                : - GPIO_Pin:  specifies the port bit to read.
*                    This parameter can be GPIO_Pin_x where x can be (0..15).
* Output         : None
* Return         : The input port pin value.
*******************************************************************************/
u8 GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
  u8 bitstatus = 0x00;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin)); 
  
  if ((GPIOx->IDR & GPIO_Pin) != (u32)Bit_RESET)
  {
    bitstatus = (u8)Bit_SET;
  }
  else
  {
    bitstatus = (u8)Bit_RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : GPIO_ReadInputData
* Description    : Reads the specified GPIO input data port.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
* Output         : None
* Return         : GPIO input data port value.
*******************************************************************************/
u16 GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  return ((u16)GPIOx->IDR);
}

/*******************************************************************************
* Function Name  : GPIO_ReadOutputDataBit
* Description    : Reads the specified output data port bit.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
*                : - GPIO_Pin:  specifies the port bit to read.
*                    This parameter can be GPIO_Pin_x where x can be (0..15).
* Output         : None
* Return         : The output port pin value.
*******************************************************************************/
u8 GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
  u8 bitstatus = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin)); 
  
  if ((GPIOx->ODR & GPIO_Pin) != (u32)Bit_RESET)
  {
    bitstatus = (u8)Bit_SET;
  }
  else
  {
    bitstatus = (u8)Bit_RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : GPIO_ReadOutputData
* Description    : Reads the specified GPIO output data port.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
* Output         : None
* Return         : GPIO output data port value.
*******************************************************************************/
u16 GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    
  return ((u16)GPIOx->ODR);
}

/*******************************************************************************
* Function Name  : GPIO_SetBits
* Description    : Sets the selected data port bits.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
*                  - GPIO_Pin: specifies the port bits to be written.
*                    This parameter can be any combination of GPIO_Pin_x where 
*                    x can be (0..15).
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_SetBits(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BSRR = GPIO_Pin;
}

/*******************************************************************************
* Function Name  : GPIO_ResetBits
* Description    : Clears the selected data port bits.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
*                  - GPIO_Pin: specifies the port bits to be written.
*                    This parameter can be any combination of GPIO_Pin_x where 
*                    x can be (0..15).
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BRR = GPIO_Pin;
}

/*******************************************************************************
* Function Name  : GPIO_WriteBit
* Description    : Sets or clears the selected data port bit.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
*                  - GPIO_Pin: specifies the port bit to be written.
*                    This parameter can be one of GPIO_Pin_x where x can be (0..15).
*                  - BitVal: specifies the value to be written to the selected bit.
*                    This parameter can be one of the BitAction enum values:
*                       - Bit_RESET: to clear the port pin
*                       - Bit_SET: to set the port pin
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_WriteBit(
* GPIOx, u16 GPIO_Pin, BitAction BitVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_BIT_ACTION(BitVal)); 
  
  if (BitVal != Bit_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = GPIO_Pin;
  }
}

/*******************************************************************************
* Function Name  : GPIO_Write
* Description    : Writes data to the specified GPIO data port.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
*                  - PortVal: specifies the value to be written to the port output
*                    data register.
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Write(GPIO_TypeDef* GPIOx, u16 PortVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  GPIOx->ODR = PortVal;
}

/*******************************************************************************
* Function Name  : GPIO_PinLockConfig
* Description    : Locks GPIO Pins configuration registers.
* Input          : - GPIOx: where x can be (A..G) to select the GPIO peripheral.
*                  - GPIO_Pin: specifies the port bit to be written.
*                    This parameter can be any combination of GPIO_Pin_x where 
*                    x can be (0..15).
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, u16 GPIO_Pin)
{
  u32 tmp = 0x00010000;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  tmp |= GPIO_Pin;
  /* Set LCKK bit */
  GPIOx->LCKR = tmp;
  /* Reset LCKK bit */
  GPIOx->LCKR =  GPIO_Pin;
  /* Set LCKK bit */
  GPIOx->LCKR = tmp;
  /* Read LCKK bit*/
  tmp = GPIOx->LCKR;
  /* Read LCKK bit*/
  tmp = GPIOx->LCKR;
}

/*******************************************************************************
* Function Name  : GPIO_EventOutputConfig
* Description    : Selects the GPIO pin used as Event output.
* Input          : - GPIO_PortSource: selects the GPIO port to be used as source
*                    for Event output.
*                    This parameter can be GPIO_PortSourceGPIOx where x can be
*                    (A..E).
*                  - GPIO_PinSource: specifies the pin for the Event output.
*                    This parameter can be GPIO_PinSourcex where x can be (0..15).
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_EventOutputConfig(u8 GPIO_PortSource, u8 GPIO_PinSource)
{
  u32 tmpreg = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_EVENTOUT_PORT_SOURCE(GPIO_PortSource));
  assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
    
  tmpreg = AFIO->EVCR;
  /* Clear the PORT[6:4] and PIN[3:0] bits */
  tmpreg &= EVCR_PORTPINCONFIG_MASK;
  tmpreg |= (u32)GPIO_PortSource << 0x04;
  tmpreg |= GPIO_PinSource;

  AFIO->EVCR = tmpreg;
}

/*******************************************************************************
* Function Name  : GPIO_EventOutputCmd
* Description    : Enables or disables the Event Output.
* Input          : - NewState: new state of the Event output.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_EventOutputCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) EVCR_EVOE_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : GPIO_PinRemapConfig
* Description    : Changes the mapping of the specified pin.
* Input          : - GPIO_Remap: selects the pin to remap.
*                    This parameter can be one of the following values:
*                       - GPIO_Remap_SPI1
*                       - GPIO_Remap_I2C1
*                       - GPIO_Remap_USART1
*                       - GPIO_Remap_USART2
*                       - GPIO_PartialRemap_USART3
*                       - GPIO_FullRemap_USART3
*                       - GPIO_PartialRemap_TIM1
*                       - GPIO_FullRemap_TIM1
*                       - GPIO_PartialRemap1_TIM2
*                       - GPIO_PartialRemap2_TIM2
*                       - GPIO_FullRemap_TIM2
*                       - GPIO_PartialRemap_TIM3
*                       - GPIO_FullRemap_TIM3
*                       - GPIO_Remap_TIM4
*                       - GPIO_Remap1_CAN
*                       - GPIO_Remap2_CAN
*                       - GPIO_Remap_PD01
*                       - GPIO_Remap_TIM5CH4_LSI
*                       - GPIO_Remap_ADC1_ETRGINJ
*                       - GPIO_Remap_ADC1_ETRGREG
*                       - GPIO_Remap_ADC2_ETRGINJ
*                       - GPIO_Remap_ADC2_ETRGREG
*                       - GPIO_Remap_SWJ_NoJTRST
*                       - GPIO_Remap_SWJ_JTAGDisable
*                       - GPIO_Remap_SWJ_Disable
*                  - NewState: new state of the port pin remapping.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_PinRemapConfig(u32 GPIO_Remap, FunctionalState NewState)
{
  u32 tmp = 0x00, tmp1 = 0x00, tmpreg = 0x00, tmpmask = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_REMAP(GPIO_Remap));
  assert_param(IS_FUNCTIONAL_STATE(NewState));  
  
  tmpreg = AFIO->MAPR;

  tmpmask = (GPIO_Remap & DBGAFR_POSITION_MASK) >> 0x10;
  tmp = GPIO_Remap & LSB_MASK;

  if ((GPIO_Remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK))
  {
    tmpreg &= DBGAFR_SWJCFG_MASK;
    AFIO->MAPR &= DBGAFR_SWJCFG_MASK;
  }
  else if ((GPIO_Remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK)
  {
    tmp1 = ((u32)0x03) << tmpmask;
    tmpreg &= ~tmp1;
    tmpreg |= ~DBGAFR_SWJCFG_MASK;
  }
  else
  {
    tmpreg &= ~(tmp << ((GPIO_Remap >> 0x15)*0x10));
    tmpreg |= ~DBGAFR_SWJCFG_MASK;
  }

  if (NewState != DISABLE)
  {
    tmpreg |= (tmp << ((GPIO_Remap >> 0x15)*0x10));
  }

  AFIO->MAPR = tmpreg;
}

/*******************************************************************************
* Function Name  : GPIO_EXTILineConfig
* Description    : Selects the GPIO pin used as EXTI Line.
* Input          : - GPIO_PortSource: selects the GPIO port to be used as
*                    source for EXTI lines.
*                    This parameter can be GPIO_PortSourceGPIOx where x can be
*                    (A..G).
*                  - GPIO_PinSource: specifies the EXTI line to be configured.
*                   This parameter can be GPIO_PinSourcex where x can be (0..15).
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_EXTILineConfig(u8 GPIO_PortSource, u8 GPIO_PinSource)
{
  u32 tmp = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_EXTI_PORT_SOURCE(GPIO_PortSource));
  assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
  
  tmp = ((u32)0x0F) << (0x04 * (GPIO_PinSource & (u8)0x03));

  AFIO->EXTICR[GPIO_PinSource >> 0x02] &= ~tmp;
  AFIO->EXTICR[GPIO_PinSource >> 0x02] |= (((u32)GPIO_PortSource) << (0x04 * (GPIO_PinSource & (u8)0x03)));
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_i2c.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the I2C firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
	 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* I2C SPE mask */
#define CR1_PE_Set              ((u16)0x0001)
#define CR1_PE_Reset            ((u16)0xFFFE)

/* I2C START mask */
#define CR1_START_Set           ((u16)0x0100)
#define CR1_START_Reset         ((u16)0xFEFF)

/* I2C STOP mask */
#define CR1_STOP_Set            ((u16)0x0200)
#define CR1_STOP_Reset          ((u16)0xFDFF)

/* I2C ACK mask */
#define CR1_ACK_Set             ((u16)0x0400)
#define CR1_ACK_Reset           ((u16)0xFBFF)

/* I2C ENGC mask */
#define CR1_ENGC_Set            ((u16)0x0040)
#define CR1_ENGC_Reset          ((u16)0xFFBF)

/* I2C SWRST mask */
#define CR1_SWRST_Set           ((u16)0x8000)
#define CR1_SWRST_Reset         ((u16)0x7FFF)

/* I2C PEC mask */
#define CR1_PEC_Set             ((u16)0x1000)
#define CR1_PEC_Reset           ((u16)0xEFFF)

/* I2C ENPEC mask */
#define CR1_ENPEC_Set           ((u16)0x0020)
#define CR1_ENPEC_Reset         ((u16)0xFFDF)

/* I2C ENARP mask */
#define CR1_ENARP_Set           ((u16)0x0010)
#define CR1_ENARP_Reset         ((u16)0xFFEF)

/* I2C NOSTRETCH mask */
#define CR1_NOSTRETCH_Set       ((u16)0x0080)
#define CR1_NOSTRETCH_Reset     ((u16)0xFF7F)

/* I2C registers Masks */
#define CR1_CLEAR_Mask          ((u16)0xFBF5)

/* I2C DMAEN mask */
#define CR2_DMAEN_Set           ((u16)0x0800)
#define CR2_DMAEN_Reset         ((u16)0xF7FF)

/* I2C LAST mask */
#define CR2_LAST_Set            ((u16)0x1000)
#define CR2_LAST_Reset          ((u16)0xEFFF)

/* I2C FREQ mask */
#define CR2_FREQ_Reset          ((u16)0xFFC0)

/* I2C ADD0 mask */
#define OAR1_ADD0_Set           ((u16)0x0001)
#define OAR1_ADD0_Reset         ((u16)0xFFFE)

/* I2C ENDUAL mask */
#define OAR2_ENDUAL_Set         ((u16)0x0001)
#define OAR2_ENDUAL_Reset       ((u16)0xFFFE)

/* I2C ADD2 mask */
#define OAR2_ADD2_Reset         ((u16)0xFF01)

/* I2C F/S mask */
#define CCR_FS_Set              ((u16)0x8000)

/* I2C CCR mask */
#define CCR_CCR_Set             ((u16)0x0FFF)

/* I2C FLAG mask */
#define FLAG_Mask               ((u32)0x00FFFFFF)

/* I2C Interrupt Enable mask */
#define ITEN_Mask               ((u32)0x07000000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : I2C_DeInit
* Description    : Deinitializes the I2Cx peripheral registers to their default
*                  reset values.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_DeInit(I2C_TypeDef* I2Cx)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));

  switch (*(u32*)&I2Cx)
  {
    case I2C1_BASE:
      /* Enable I2C1 reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
      /* Release I2C1 from reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
      break;

    case I2C2_BASE:
      /* Enable I2C2 reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, ENABLE);
      /* Release I2C2 from reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2, DISABLE);
      break;

    default:
      break;
  }
}

/*******************************************************************************
* Function Name  : I2C_Init
* Description    : Initializes the I2Cx peripheral according to the specified 
*                  parameters in the I2C_InitStruct.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_InitStruct: pointer to a I2C_InitTypeDef structure that
*                    contains the configuration information for the specified
*                    I2C peripheral.
* Output         : None
* Return         : None
******************************************************************************/
void I2C_Init(I2C_TypeDef* I2Cx, I2C_InitTypeDef* I2C_InitStruct)
{
  u16 tmpreg = 0, freqrange = 0;
  u16 result = 0x04;
  u32 pclk1 = 8000000;
  RCC_ClocksTypeDef  rcc_clocks;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_MODE(I2C_InitStruct->I2C_Mode));
  assert_param(IS_I2C_DUTY_CYCLE(I2C_InitStruct->I2C_DutyCycle));
  assert_param(IS_I2C_OWN_ADDRESS1(I2C_InitStruct->I2C_OwnAddress1));
  assert_param(IS_I2C_ACK_STATE(I2C_InitStruct->I2C_Ack));
  assert_param(IS_I2C_ACKNOWLEDGE_ADDRESS(I2C_InitStruct->I2C_AcknowledgedAddress));
  assert_param(IS_I2C_CLOCK_SPEED(I2C_InitStruct->I2C_ClockSpeed));

/*---------------------------- I2Cx CR2 Configuration ------------------------*/
  /* Get the I2Cx CR2 value */
  tmpreg = I2Cx->CR2;
  /* Clear frequency FREQ[5:0] bits */
  tmpreg &= CR2_FREQ_Reset;
  /* Get pclk1 frequency value */
  RCC_GetClocksFreq(&rcc_clocks);
  pclk1 = rcc_clocks.PCLK1_Frequency;
  /* Set frequency bits depending on pclk1 value */
  freqrange = (u16)(pclk1 / 1000000);
  tmpreg |= freqrange;
  /* Write to I2Cx CR2 */
  I2Cx->CR2 = tmpreg;

/*---------------------------- I2Cx CCR Configuration ------------------------*/
  /* Disable the selected I2C peripheral to configure TRISE */
  I2Cx->CR1 &= CR1_PE_Reset;

  /* Reset tmpreg value */
  /* Clear F/S, DUTY and CCR[11:0] bits */
  tmpreg = 0;

  /* Configure speed in standard mode */
  if (I2C_InitStruct->I2C_ClockSpeed <= 100000)
  {
    /* Standard mode speed calculate */
    result = (u16)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed << 1));
    /* Test if CCR value is under 0x4*/
    if (result < 0x04)
    {
      /* Set minimum allowed value */
      result = 0x04;  
    }
    /* Set speed value for standard mode */
    tmpreg |= result;	  
    /* Set Maximum Rise Time for standard mode */
    I2Cx->TRISE = freqrange + 1; 
  }
  /* Configure speed in fast mode */
  else /*(I2C_InitStruct->I2C_ClockSpeed <= 400000)*/
  {
    if (I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_2)
    {
      /* Fast mode speed calculate: Tlow/Thigh = 2 */
      result = (u16)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 3));
    }
    else /*I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_16_9*/
    {
      /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
      result = (u16)(pclk1 / (I2C_InitStruct->I2C_ClockSpeed * 25));
      /* Set DUTY bit */
      result |= I2C_DutyCycle_16_9;
    }
    /* Test if CCR value is under 0x1*/
    if ((result & CCR_CCR_Set) == 0)
    {
      /* Set minimum allowed value */
      result |= (u16)0x0001;  
    }
    /* Set speed value and set F/S bit for fast mode */
    tmpreg |= result | CCR_FS_Set;
    /* Set Maximum Rise Time for fast mode */
    I2Cx->TRISE = (u16)(((freqrange * 300) / 1000) + 1);  
  }
  /* Write to I2Cx CCR */
  I2Cx->CCR = tmpreg;

  /* Enable the selected I2C peripheral */
  I2Cx->CR1 |= CR1_PE_Set;

/*---------------------------- I2Cx CR1 Configuration ------------------------*/
  /* Get the I2Cx CR1 value */
  tmpreg = I2Cx->CR1;
  /* Clear ACK, SMBTYPE and  SMBUS bits */
  tmpreg &= CR1_CLEAR_Mask;
  /* Configure I2Cx: mode and acknowledgement */
  /* Set SMBTYPE and SMBUS bits according to I2C_Mode value */
  /* Set ACK bit according to I2C_Ack value */
  tmpreg |= (u16)((u32)I2C_InitStruct->I2C_Mode | I2C_InitStruct->I2C_Ack);
  /* Write to I2Cx CR1 */
  I2Cx->CR1 = tmpreg;

/*---------------------------- I2Cx OAR1 Configuration -----------------------*/
  /* Set I2Cx Own Address1 and acknowledged address */
  I2Cx->OAR1 = (I2C_InitStruct->I2C_AcknowledgedAddress | I2C_InitStruct->I2C_OwnAddress1);
}

/*******************************************************************************
* Function Name  : I2C_StructInit
* Description    : Fills each I2C_InitStruct member with its default value.
* Input          : - I2C_InitStruct: pointer to an I2C_InitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_StructInit(I2C_InitTypeDef* I2C_InitStruct)
{
/*---------------- Reset I2C init structure parameters values ----------------*/
  /* Initialize the I2C_Mode member */
  I2C_InitStruct->I2C_Mode = I2C_Mode_I2C;

  /* Initialize the I2C_DutyCycle member */
  I2C_InitStruct->I2C_DutyCycle = I2C_DutyCycle_2;

  /* Initialize the I2C_OwnAddress1 member */
  I2C_InitStruct->I2C_OwnAddress1 = 0;

  /* Initialize the I2C_Ack member */
  I2C_InitStruct->I2C_Ack = I2C_Ack_Disable;

  /* Initialize the I2C_AcknowledgedAddress member */
  I2C_InitStruct->I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

  /* initialize the I2C_ClockSpeed member */
  I2C_InitStruct->I2C_ClockSpeed = 5000;
}

/*******************************************************************************
* Function Name  : I2C_Cmd
* Description    : Enables or disables the specified I2C peripheral.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2Cx peripheral. This parameter
*                    can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_Cmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected I2C peripheral */
    I2Cx->CR1 |= CR1_PE_Set;
  }
  else
  {
    /* Disable the selected I2C peripheral */
    I2Cx->CR1 &= CR1_PE_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_DMACmd
* Description    : Enables or disables the specified I2C DMA requests.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2C DMA transfer.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_DMACmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected I2C DMA requests */
    I2Cx->CR2 |= CR2_DMAEN_Set;
  }
  else
  {
    /* Disable the selected I2C DMA requests */
    I2Cx->CR2 &= CR2_DMAEN_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_DMALastTransferCmd
* Description    : Specifies that the next DMA transfer is the last one.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2C DMA last transfer.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_DMALastTransferCmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Next DMA transfer is the last transfer */
    I2Cx->CR2 |= CR2_LAST_Set;
  }
  else
  {
    /* Next DMA transfer is not the last transfer */
    I2Cx->CR2 &= CR2_LAST_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_GenerateSTART
* Description    : Generates I2Cx communication START condition.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2C START condition generation.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None.
*******************************************************************************/
void I2C_GenerateSTART(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Generate a START condition */
    I2Cx->CR1 |= CR1_START_Set;
  }
  else
  {
    /* Disable the START condition generation */
    I2Cx->CR1 &= CR1_START_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_GenerateSTOP
* Description    : Generates I2Cx communication STOP condition.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2C STOP condition generation.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None.
*******************************************************************************/
void I2C_GenerateSTOP(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Generate a STOP condition */
    I2Cx->CR1 |= CR1_STOP_Set;
  }
  else
  {
    /* Disable the STOP condition generation */
    I2Cx->CR1 &= CR1_STOP_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_AcknowledgeConfig
* Description    : Enables or disables the specified I2C acknowledge feature.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2C Acknowledgement.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None.
*******************************************************************************/
void I2C_AcknowledgeConfig(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the acknowledgement */
    I2Cx->CR1 |= CR1_ACK_Set;
  }
  else
  {
    /* Disable the acknowledgement */
    I2Cx->CR1 &= CR1_ACK_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_OwnAddress2Config
* Description    : Configures the specified I2C own address2.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - Address: specifies the 7bit I2C own address2.
* Output         : None
* Return         : None.
*******************************************************************************/
void I2C_OwnAddress2Config(I2C_TypeDef* I2Cx, u8 Address)
{
  u16 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));

  /* Get the old register value */
  tmpreg = I2Cx->OAR2;
  /* Reset I2Cx Own address2 bit [7:1] */
  tmpreg &= OAR2_ADD2_Reset;
  /* Set I2Cx Own address2 */
  tmpreg |= (u16)(Address & (u16)0x00FE);
  /* Store the new register value */
  I2Cx->OAR2 = tmpreg;
}

/*******************************************************************************
* Function Name  : I2C_DualAddressCmd
* Description    : Enables or disables the specified I2C dual addressing mode.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2C dual addressing mode.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_DualAddressCmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable dual addressing mode */
    I2Cx->OAR2 |= OAR2_ENDUAL_Set;
  }
  else
  {
    /* Disable dual addressing mode */
    I2Cx->OAR2 &= OAR2_ENDUAL_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_GeneralCallCmd
* Description    : Enables or disables the specified I2C general call feature.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2C General call.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_GeneralCallCmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable generall call */
    I2Cx->CR1 |= CR1_ENGC_Set;
  }
  else
  {
    /* Disable generall call */
    I2Cx->CR1 &= CR1_ENGC_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_ITConfig
* Description    : Enables or disables the specified I2C interrupts.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_IT: specifies the I2C interrupts sources to be enabled
*                    or disabled. 
*                    This parameter can be any combination of the following values:
*                       - I2C_IT_BUF: Buffer interrupt mask
*                       - I2C_IT_EVT: Event interrupt mask
*                       - I2C_IT_ERR: Error interrupt mask
*                  - NewState: new state of the specified I2C interrupts.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_ITConfig(I2C_TypeDef* I2Cx, u16 I2C_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_I2C_CONFIG_IT(I2C_IT));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected I2C interrupts */
    I2Cx->CR2 |= I2C_IT;
  }
  else
  {
    /* Disable the selected I2C interrupts */
    I2Cx->CR2 &= (u16)~I2C_IT;
  }
}

/*******************************************************************************
* Function Name  : I2C_SendData
* Description    : Sends a data byte through the I2Cx peripheral.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - Data: Byte to be transmitted..
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_SendData(I2C_TypeDef* I2Cx, u8 Data)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));

  /* Write in the DR register the data to be sent */
  I2Cx->DR = Data;
}

/*******************************************************************************
* Function Name  : I2C_ReceiveData
* Description    : Returns the most recent received data by the I2Cx peripheral.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
* Output         : None
* Return         : The value of the received data.
*******************************************************************************/
u8 I2C_ReceiveData(I2C_TypeDef* I2Cx)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));

  /* Return the data in the DR register */
  return (u8)I2Cx->DR;
}

/*******************************************************************************
* Function Name  : I2C_Send7bitAddress
* Description    : Transmits the address byte to select the slave device.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - Address: specifies the slave address which will be transmitted
*                  - I2C_Direction: specifies whether the I2C device will be a
*                    Transmitter or a Receiver. 
*                    This parameter can be one of the following values
*                       - I2C_Direction_Transmitter: Transmitter mode
*                       - I2C_Direction_Receiver: Receiver mode
* Output         : None
* Return         : None.
*******************************************************************************/
void I2C_Send7bitAddress(I2C_TypeDef* I2Cx, u8 Address, u8 I2C_Direction)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_DIRECTION(I2C_Direction));

  /* Test on the direction to set/reset the read/write bit */
  if (I2C_Direction != I2C_Direction_Transmitter)
  {
    /* Set the address bit0 for read */
    Address |= OAR1_ADD0_Set;
  }
  else
  {
    /* Reset the address bit0 for write */
    Address &= OAR1_ADD0_Reset;
  }
  /* Send the address */
  I2Cx->DR = Address;
}

/*******************************************************************************
* Function Name  : I2C_ReadRegister
* Description    : Reads the specified I2C register and returns its value.
* Input1         : - I2C_Register: specifies the register to read.
*                    This parameter can be one of the following values:
*                       - I2C_Register_CR1:  CR1 register.
*                       - I2C_Register_CR2:   CR2 register.
*                       - I2C_Register_OAR1:  OAR1 register.
*                       - I2C_Register_OAR2:  OAR2 register.
*                       - I2C_Register_DR:    DR register.
*                       - I2C_Register_SR1:   SR1 register.
*                       - I2C_Register_SR2:   SR2 register.
*                       - I2C_Register_CCR:   CCR register.
*                       - I2C_Register_TRISE: TRISE register.
* Output         : None
* Return         : The value of the read register.
*******************************************************************************/
u16 I2C_ReadRegister(I2C_TypeDef* I2Cx, u8 I2C_Register)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_REGISTER(I2C_Register));

  /* Return the selected register value */
  return (*(vu16 *)(*((vu32 *)&I2Cx) + I2C_Register));
}

/*******************************************************************************
* Function Name  : I2C_SoftwareResetCmd
* Description    : Enables or disables the specified I2C software reset.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2C software reset.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_SoftwareResetCmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Peripheral under reset */
    I2Cx->CR1 |= CR1_SWRST_Set;
  }
  else
  {
    /* Peripheral not under reset */
    I2Cx->CR1 &= CR1_SWRST_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_SMBusAlertConfig
* Description    : Drives the SMBusAlert pin high or low for the specified I2C.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_SMBusAlert: specifies SMBAlert pin level. 
*                    This parameter can be one of the following values:
*                       - I2C_SMBusAlert_Low: SMBAlert pin driven low
*                       - I2C_SMBusAlert_High: SMBAlert pin driven high
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_SMBusAlertConfig(I2C_TypeDef* I2Cx, u16 I2C_SMBusAlert)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_SMBUS_ALERT(I2C_SMBusAlert));

  if (I2C_SMBusAlert == I2C_SMBusAlert_Low)
  {
    /* Drive the SMBusAlert pin Low */
    I2Cx->CR1 |= I2C_SMBusAlert_Low;
  }
  else
  {
    /* Drive the SMBusAlert pin High  */
    I2Cx->CR1 &= I2C_SMBusAlert_High;
  }
}

/*******************************************************************************
* Function Name  : I2C_TransmitPEC
* Description    : Enables or disables the specified I2C PEC transfer.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2C PEC transmission.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_TransmitPEC(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected I2C PEC transmission */
    I2Cx->CR1 |= CR1_PEC_Set;
  }
  else
  {
    /* Disable the selected I2C PEC transmission */
    I2Cx->CR1 &= CR1_PEC_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_PECPositionConfig
* Description    : Selects the specified I2C PEC position.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_PECPosition: specifies the PEC position. 
*                    This parameter can be one of the following values:
*                       - I2C_PECPosition_Next: indicates that the next
*                         byte is PEC
*                       - I2C_PECPosition_Current: indicates that current
*                         byte is PEC
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_PECPositionConfig(I2C_TypeDef* I2Cx, u16 I2C_PECPosition)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_PEC_POSITION(I2C_PECPosition));

  if (I2C_PECPosition == I2C_PECPosition_Next)
  {
    /* Next byte in shift register is PEC */
    I2Cx->CR1 |= I2C_PECPosition_Next;
  }
  else
  {
    /* Current byte in shift register is PEC */
    I2Cx->CR1 &= I2C_PECPosition_Current;
  }
}

/*******************************************************************************
* Function Name  : I2C_CalculatePEC
* Description    : Enables or disables the PEC value calculation of the
*                  transfered bytes.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2Cx PEC value calculation.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_CalculatePEC(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected I2C PEC calculation */
    I2Cx->CR1 |= CR1_ENPEC_Set;
  }
  else
  {
    /* Disable the selected I2C PEC calculation */
    I2Cx->CR1 &= CR1_ENPEC_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_GetPEC
* Description    : Returns the PEC value for the specified I2C.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
* Output         : None
* Return         : The PEC value.
*******************************************************************************/
u8 I2C_GetPEC(I2C_TypeDef* I2Cx)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));

  /* Return the selected I2C PEC value */
  return ((I2Cx->SR2) >> 8);
}

/*******************************************************************************
* Function Name  : I2C_ARPCmd
* Description    : Enables or disables the specified I2C ARP.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2Cx ARP. 
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_ARPCmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected I2C ARP */
    I2Cx->CR1 |= CR1_ENARP_Set;
  }
  else
  {
    /* Disable the selected I2C ARP */
    I2Cx->CR1 &= CR1_ENARP_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_StretchClockCmd
* Description    : Enables or disables the specified I2C Clock stretching.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - NewState: new state of the I2Cx Clock stretching.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_StretchClockCmd(I2C_TypeDef* I2Cx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState == DISABLE)
  {
    /* Enable the selected I2C Clock stretching */
    I2Cx->CR1 |= CR1_NOSTRETCH_Set;
  }
  else
  {
    /* Disable the selected I2C Clock stretching */
    I2Cx->CR1 &= CR1_NOSTRETCH_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2C_FastModeDutyCycleConfig
* Description    : Selects the specified I2C fast mode duty cycle.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_DutyCycle: specifies the fast mode duty cycle.
*                    This parameter can be one of the following values:
*                       - I2C_DutyCycle_2: I2C fast mode Tlow/Thigh = 2
*                       - I2C_DutyCycle_16_9: I2C fast mode Tlow/Thigh = 16/9
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_FastModeDutyCycleConfig(I2C_TypeDef* I2Cx, u16 I2C_DutyCycle)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_DUTY_CYCLE(I2C_DutyCycle));

  if (I2C_DutyCycle != I2C_DutyCycle_16_9)
  {
    /* I2C fast mode Tlow/Thigh=2 */
    I2Cx->CCR &= I2C_DutyCycle_2;
  }
  else
  {
    /* I2C fast mode Tlow/Thigh=16/9 */
    I2Cx->CCR |= I2C_DutyCycle_16_9;
  }
}

/*******************************************************************************
* Function Name  : I2C_GetLastEvent
* Description    : Returns the last I2Cx Event.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
* Output         : None
* Return         : The last event
*******************************************************************************/
u32 I2C_GetLastEvent(I2C_TypeDef* I2Cx)
{
  u32 lastevent = 0;
  u32 flag1 = 0, flag2 = 0;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));

  /* Read the I2Cx status register */
  flag1 = I2Cx->SR1;
  flag2 = I2Cx->SR2;
  flag2 = flag2 << 16;

  /* Get the last event value from I2C status register */
  lastevent = (flag1 | flag2) & FLAG_Mask;

  /* Return status */
  return lastevent;
}

/*******************************************************************************
* Function Name  : I2C_CheckEvent
* Description    : Checks whether the last I2Cx Event is equal to the one passed
*                  as parameter.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_EVENT: specifies the event to be checked. 
*                    This parameter can be one of the following values:
*                       - I2C_EVENT_SLAVE_ADDRESS_MATCHED   : EV1
*                       - I2C_EVENT_SLAVE_BYTE_RECEIVED     : EV2
*                       - I2C_EVENT_SLAVE_BYTE_TRANSMITTED  : EV3
*                       - I2C_EVENT_SLAVE_ACK_FAILURE       : EV3-2
*                       - I2C_EVENT_MASTER_MODE_SELECT      : EV5
*                       - I2C_EVENT_MASTER_MODE_SELECTED    : EV6
*                       - I2C_EVENT_MASTER_BYTE_RECEIVED    : EV7
*                       - I2C_EVENT_MASTER_BYTE_TRANSMITTED : EV8
*                       - I2C_EVENT_MASTER_MODE_ADDRESS10   : EV9
*                       - I2C_EVENT_SLAVE_STOP_DETECTED     : EV4
* Output         : None
* Return         : An ErrorStatus enumuration value:
*                       - SUCCESS: Last event is equal to the I2C_EVENT
*                       - ERROR: Last event is different from the I2C_EVENT
*******************************************************************************/
ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, u32 I2C_EVENT)
{
  u32 lastevent = 0;
  u32 flag1 = 0, flag2 = 0;
  ErrorStatus status = ERROR;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_EVENT(I2C_EVENT));

  /* Read the I2Cx status register */
  flag1 = I2Cx->SR1;
  flag2 = I2Cx->SR2;
  flag2 = flag2 << 16;

  /* Get the last event value from I2C status register */
  lastevent = (flag1 | flag2) & FLAG_Mask;

  /* Check whether the last event is equal to I2C_EVENT */
  if (lastevent == I2C_EVENT )
  {
    /* SUCCESS: last event is equal to I2C_EVENT */
    status = SUCCESS;
  }
  else
  {
    /* ERROR: last event is different from I2C_EVENT */
    status = ERROR;
  }

  /* Return status */
  return status;
}

/*******************************************************************************
* Function Name  : I2C_GetFlagStatus
* Description    : Checks whether the specified I2C flag is set or not.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_FLAG: specifies the flag to check. 
*                    This parameter can be one of the following values:
*                       - I2C_FLAG_DUALF: Dual flag (Slave mode)
*                       - I2C_FLAG_SMBHOST: SMBus host header (Slave mode)
*                       - I2C_FLAG_SMBDEFAULT: SMBus default header (Slave mode)
*                       - I2C_FLAG_GENCALL: General call header flag (Slave mode)
*                       - I2C_FLAG_TRA: Transmitter/Receiver flag
*                       - I2C_FLAG_BUSY: Bus busy flag
*                       - I2C_FLAG_MSL: Master/Slave flag
*                       - I2C_FLAG_SMBALERT: SMBus Alert flag
*                       - I2C_FLAG_TIMEOUT: Timeout or Tlow error flag
*                       - I2C_FLAG_PECERR: PEC error in reception flag
*                       - I2C_FLAG_OVR: Overrun/Underrun flag (Slave mode)
*                       - I2C_FLAG_AF: Acknowledge failure flag
*                       - I2C_FLAG_ARLO: Arbitration lost flag (Master mode)
*                       - I2C_FLAG_BERR: Bus error flag
*                       - I2C_FLAG_TXE: Data register empty flag (Transmitter)
*                       - I2C_FLAG_RXNE: Data register not empty (Receiver) flag
*                       - I2C_FLAG_STOPF: Stop detection flag (Slave mode)
*                       - I2C_FLAG_ADD10: 10-bit header sent flag (Master mode)
*                       - I2C_FLAG_BTF: Byte transfer finished flag
*                       - I2C_FLAG_ADDR: Address sent flag (Master mode) “ADSL”
*                                        Address matched flag (Slave mode)”ENDAD”
*                       - I2C_FLAG_SB: Start bit flag (Master mode)
* Output         : None
* Return         : The new state of I2C_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, u32 I2C_FLAG)
{
  FlagStatus bitstatus = RESET;
  u32 i2creg = 0, i2cxbase = 0;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_GET_FLAG(I2C_FLAG));

  /* Get the I2Cx peripheral base address */
  i2cxbase = (*(u32*)&(I2Cx));
  
  /* Read flag register index */
  i2creg = I2C_FLAG >> 28;
  
  /* Get bit[23:0] of the flag */
  I2C_FLAG &= FLAG_Mask;
  
  if(i2creg != 0)
  {
    /* Get the I2Cx SR1 register address */
    i2cxbase += 0x14;
  }
  else
  {
    /* Flag in I2Cx SR2 Register */
    I2C_FLAG = (u32)(I2C_FLAG >> 16);
    /* Get the I2Cx SR2 register address */
    i2cxbase += 0x18;
  }
  
  if(((*(vu32 *)i2cxbase) & I2C_FLAG) != (u32)RESET)
  {
    /* I2C_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* I2C_FLAG is reset */
    bitstatus = RESET;
  }
  
  /* Return the I2C_FLAG status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : I2C_ClearFlag
* Description    : Clears the I2Cx's pending flags.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_FLAG: specifies the flag to clear. 
*                    This parameter can be any combination of the following
*                    values:
*                       - I2C_FLAG_SMBALERT: SMBus Alert flag
*                       - I2C_FLAG_TIMEOUT: Timeout or Tlow error flag
*                       - I2C_FLAG_PECERR: PEC error in reception flag
*                       - I2C_FLAG_OVR: Overrun/Underrun flag (Slave mode)
*                       - I2C_FLAG_AF: Acknowledge failure flag
*                       - I2C_FLAG_ARLO: Arbitration lost flag (Master mode)
*                       - I2C_FLAG_BERR: Bus error flag
*                       
*                  Notes: 
*                        - STOPF (STOP detection) is cleared by software 
*                          sequence: a read operation to I2C_SR1 register 
*                          (I2C_GetFlagStatus()) followed by a write operation 
*                          to I2C_CR1 register (I2C_Cmd() to re-enable the 
*                          I2C peripheral). 
*                        - ADD10 (10-bit header sent) is cleared by software 
*                          sequence: a read operation to I2C_SR1 
*                          (I2C_GetFlagStatus()) followed by writing the
*                          second byte of the address in DR register.
*                        - BTF (Byte Transfer Finished) is cleared by software 
*                          sequence: a read operation to I2C_SR1 register 
*                          (I2C_GetFlagStatus()) followed by a read/write to 
*                          I2C_DR register (I2C_SendData()).
*                        - ADDR (Address sent) is cleared by software sequence: 
*                          a read operation to I2C_SR1 register 
*                          (I2C_GetFlagStatus()) followed by a read operation to 
*                          I2C_SR2 register ((void)(I2Cx->SR2)).
*                        - SB (Start Bit) is cleared software sequence: a read 
*                          operation to I2C_SR1 register (I2C_GetFlagStatus()) 
*                          followed by a write operation to I2C_DR reigister 
*                          (I2C_SendData()). 
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_ClearFlag(I2C_TypeDef* I2Cx, u32 I2C_FLAG)
{
  u32 flagpos = 0;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_CLEAR_FLAG(I2C_FLAG));

  /* Get the I2C flag position */
  flagpos = I2C_FLAG & FLAG_Mask;

  /* Clear the selected I2C flag */
  I2Cx->SR1 = (u16)~flagpos;
}

/*******************************************************************************
* Function Name  : I2C_GetITStatus
* Description    : Checks whether the specified I2C interrupt has occurred or not.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_IT: specifies the interrupt source to check. 
*                    This parameter can be one of the following values:
*                       - I2C_IT_SMBALERT: SMBus Alert flag
*                       - I2C_IT_TIMEOUT: Timeout or Tlow error flag
*                       - I2C_IT_PECERR: PEC error in reception flag
*                       - I2C_IT_OVR: Overrun/Underrun flag (Slave mode)
*                       - I2C_IT_AF: Acknowledge failure flag
*                       - I2C_IT_ARLO: Arbitration lost flag (Master mode)
*                       - I2C_IT_BERR: Bus error flag
*                       - I2C_IT_TXE: Data register empty flag (Transmitter)
*                       - I2C_IT_RXNE: Data register not empty (Receiver) flag
*                       - I2C_IT_STOPF: Stop detection flag (Slave mode)
*                       - I2C_IT_ADD10: 10-bit header sent flag (Master mode)
*                       - I2C_IT_BTF: Byte transfer finished flag
*                       - I2C_IT_ADDR: Address sent flag (Master mode) “ADSL”
*                                      Address matched flag (Slave mode)”ENDAD”
*                       - I2C_IT_SB: Start bit flag (Master mode)
* Output         : None
* Return         : The new state of I2C_IT (SET or RESET).
*******************************************************************************/
ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, u32 I2C_IT)
{
  ITStatus bitstatus = RESET;
  u32 enablestatus = 0;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_GET_IT(I2C_IT));

  /* Check if the interrupt source is enabled or not */
  enablestatus = (u32)(((I2C_IT & ITEN_Mask) >> 16) & (I2Cx->CR2)) ;  

  /* Get bit[23:0] of the flag */
  I2C_IT &= FLAG_Mask;

  /* Check the status of the specified I2C flag */
  if (((I2Cx->SR1 & I2C_IT) != (u32)RESET) && enablestatus)
  {
    /* I2C_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* I2C_IT is reset */
    bitstatus = RESET;
  }
  /* Return the I2C_IT status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : I2C_ClearITPendingBit
* Description    : Clears the I2Cx’s interrupt pending bits.
* Input          : - I2Cx: where x can be 1 or 2 to select the I2C peripheral.
*                  - I2C_IT: specifies the interrupt pending bit to clear. 
*                    This parameter can be any combination of the following 
*                    values:
*                       - I2C_IT_SMBALERT: SMBus Alert interrupt
*                       - I2C_IT_TIMEOUT: Timeout or Tlow error interrupt
*                       - I2C_IT_PECERR: PEC error in reception  interrupt
*                       - I2C_IT_OVR: Overrun/Underrun interrupt (Slave mode)
*                       - I2C_IT_AF: Acknowledge failure interrupt
*                       - I2C_IT_ARLO: Arbitration lost interrupt (Master mode)
*                       - I2C_IT_BERR: Bus error interrupt
*                       
*                  Notes:
*                        - STOPF (STOP detection) is cleared by software 
*                          sequence: a read operation to I2C_SR1 register 
*                          (I2C_GetITStatus()) followed by a write operation to 
*                          I2C_CR1 register (I2C_Cmd() to re-enable the I2C 
*                          peripheral). 
*                        - ADD10 (10-bit header sent) is cleared by software 
*                          sequence: a read operation to I2C_SR1 
*                          (I2C_GetITStatus()) followed by writing the second 
*                          byte of the address in I2C_DR register.
*                        - BTF (Byte Transfer Finished) is cleared by software 
*                          sequence: a read operation to I2C_SR1 register 
*                          (I2C_GetITStatus()) followed by a read/write to 
*                          I2C_DR register (I2C_SendData()).
*                        - ADDR (Address sent) is cleared by software sequence: 
*                          a read operation to I2C_SR1 register (I2C_GetITStatus()) 
*                          followed by a read operation to I2C_SR2 register 
*                          ((void)(I2Cx->SR2)).
*                        - SB (Start Bit) is cleared by software sequence: a 
*                          read operation to I2C_SR1 register (I2C_GetITStatus()) 
*                          followed by a write operation to I2C_DR reigister 
*                          (I2C_SendData()). 
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, u32 I2C_IT)
{
  u32 flagpos = 0;

  /* Check the parameters */
  assert_param(IS_I2C_ALL_PERIPH(I2Cx));
  assert_param(IS_I2C_CLEAR_IT(I2C_IT));

  /* Get the I2C flag position */
  flagpos = I2C_IT & FLAG_Mask;

  /* Clear the selected I2C flag */
  I2Cx->SR1 = (u16)~flagpos;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/


















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_it.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Main Interrupt Service Routines.
*                      This file provides template for all exceptions handler
*                      and peripherals interrupt service routine.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : NMIException
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMIException(void)
{}

/*******************************************************************************
* Function Name  : HardFaultException
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFaultException(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {}
}

/*******************************************************************************
* Function Name  : MemManageException
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManageException(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {}
}

/*******************************************************************************
* Function Name  : BusFaultException
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFaultException(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {}
}

/*******************************************************************************
* Function Name  : UsageFaultException
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFaultException(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {}
}

/*******************************************************************************
* Function Name  : DebugMonitor
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMonitor(void)
{}

/*******************************************************************************
* Function Name  : SVCHandler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVCHandler(void)
{}

/*******************************************************************************
* Function Name  : PendSVC
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSVC(void)
{}

/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
void SysTickHandler(void)
{}
*/
/*******************************************************************************
* Function Name  : WWDG_IRQHandler
* Description    : This function handles WWDG interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : PVD_IRQHandler
* Description    : This function handles PVD interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PVD_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TAMPER_IRQHandler
* Description    : This function handles Tamper interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TAMPER_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : RTC_IRQHandler
* Description    : This function handles RTC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : FLASH_IRQHandler
* Description    : This function handles Flash interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FLASH_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : RCC_IRQHandler
* Description    : This function handles RCC interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : EXTI0_IRQHandler
* Description    : This function handles External interrupt Line 0 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI0_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : EXTI1_IRQHandler
* Description    : This function handles External interrupt Line 1 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI1_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : EXTI2_IRQHandler
* Description    : This function handles External interrupt Line 2 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI2_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : EXTI3_IRQHandler
* Description    : This function handles External interrupt Line 3 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI3_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : EXTI4_IRQHandler
* Description    : This function handles External interrupt Line 4 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI4_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA1_Channel1_IRQHandler
* Description    : This function handles DMA1 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA1_Channel2_IRQHandler
* Description    : This function handles DMA1 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA1_Channel3_IRQHandler
* Description    : This function handles DMA1 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel3_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA1_Channel4_IRQHandler
* Description    : This function handles DMA1 Channel 4 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel4_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA1_Channel5_IRQHandler
* Description    : This function handles DMA1 Channel 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel5_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA1_Channel6_IRQHandler
* Description    : This function handles DMA1 Channel 6 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel6_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA1_Channel7_IRQHandler
* Description    : This function handles DMA1 Channel 7 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : ADC1_2_IRQHandler
* Description    : This function handles ADC1 and ADC2 global interrupts requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC1_2_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : USB_HP_CAN_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_HP_CAN_TX_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : USB_LP_CAN_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : CAN_RX1_IRQHandler
* Description    : This function handles CAN RX1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_RX1_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : CAN_SCE_IRQHandler
* Description    : This function handles CAN SCE interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CAN_SCE_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : EXTI9_5_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI9_5_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM1_BRK_IRQHandler
* Description    : This function handles TIM1 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM1_UP_IRQHandler
* Description    : This function handles TIM1 overflow and update interrupt
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_UP_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM1_TRG_COM_IRQHandler
* Description    : This function handles TIM1 Trigger and commutation interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_TRG_COM_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM1_CC_IRQHandler
* Description    : This function handles TIM1 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM1_CC_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM3_IRQHandler
* Description    : This function handles TIM3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM3_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM4_IRQHandler
* Description    : This function handles TIM4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM4_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : I2C1_EV_IRQHandler
* Description    : This function handles I2C1 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_EV_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : I2C1_ER_IRQHandler
* Description    : This function handles I2C1 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C1_ER_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : I2C2_EV_IRQHandler
* Description    : This function handles I2C2 Event interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_EV_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : I2C2_ER_IRQHandler
* Description    : This function handles I2C2 Error interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void I2C2_ER_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : SPI1_IRQHandler
* Description    : This function handles SPI1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI1_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : SPI2_IRQHandler
* Description    : This function handles SPI2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI2_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : EXTI15_10_IRQHandler
* Description    : This function handles External lines 15 to 10 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI15_10_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : RTCAlarm_IRQHandler
* Description    : This function handles RTC Alarm interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTCAlarm_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : USBWakeUp_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USBWakeUp_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM8_BRK_IRQHandler
* Description    : This function handles TIM8 Break interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_BRK_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM8_UP_IRQHandler
* Description    : This function handles TIM8 overflow and update interrupt
*                  request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_UP_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM8_TRG_COM_IRQHandler
* Description    : This function handles TIM8 Trigger and commutation interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_TRG_COM_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM8_CC_IRQHandler
* Description    : This function handles TIM8 capture compare interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM8_CC_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : ADC3_IRQHandler
* Description    : This function handles ADC3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC3_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : FSMC_IRQHandler
* Description    : This function handles FSMC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FSMC_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM5_IRQHandler
* Description    : This function handles TIM5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM5_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : SPI3_IRQHandler
* Description    : This function handles SPI3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI3_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : UART4_IRQHandler
* Description    : This function handles UART4 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART4_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : UART5_IRQHandler
* Description    : This function handles UART5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART5_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM6_IRQHandler
* Description    : This function handles TIM6 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM6_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : TIM7_IRQHandler
* Description    : This function handles TIM7 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM7_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA2_Channel1_IRQHandler
* Description    : This function handles DMA2 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel1_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA2_Channel2_IRQHandler
* Description    : This function handles DMA2 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel2_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA2_Channel3_IRQHandler
* Description    : This function handles DMA2 Channel 3 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel3_IRQHandler(void)
{}

/*******************************************************************************
* Function Name  : DMA2_Channel4_5_IRQHandler
* Description    : This function handles DMA2 Channel 4 and DMA2 Channel 5
*                  interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA2_Channel4_5_IRQHandler(void)
{}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_iwdg.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the IWDG firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_iwdg.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ---------------------- IWDG registers bit mask ------------------------ */
/* KR register bit mask */
#define KR_KEY_Reload    ((u16)0xAAAA)
#define KR_KEY_Enable    ((u16)0xCCCC)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : IWDG_WriteAccessCmd
* Description    : Enables or disables write access to IWDG_PR and IWDG_RLR
*                  registers.
* Input          : - IWDG_WriteAccess: new state of write access to IWDG_PR and
*                    IWDG_RLR registers.
*                    This parameter can be one of the following values:
*                       - IWDG_WriteAccess_Enable: Enable write access to 
*                         IWDG_PR and IWDG_RLR registers
*                       - IWDG_WriteAccess_Disable: Disable write access to
*                         IWDG_PR and IWDG_RLR registers
* Output         : None
* Return         : None
*******************************************************************************/
void IWDG_WriteAccessCmd(u16 IWDG_WriteAccess)
{
  /* Check the parameters */
  assert_param(IS_IWDG_WRITE_ACCESS(IWDG_WriteAccess));

  IWDG->KR = IWDG_WriteAccess;
}

/*******************************************************************************
* Function Name  : IWDG_SetPrescaler
* Description    : Sets IWDG Prescaler value.
* Input          : - IWDG_Prescaler: specifies the IWDG Prescaler value.
*                    This parameter can be one of the following values:
*                       - IWDG_Prescaler_4: IWDG prescaler set to 4
*                       - IWDG_Prescaler_8: IWDG prescaler set to 8
*                       - IWDG_Prescaler_16: IWDG prescaler set to 16
*                       - IWDG_Prescaler_32: IWDG prescaler set to 32
*                       - IWDG_Prescaler_64: IWDG prescaler set to 64
*                       - IWDG_Prescaler_128: IWDG prescaler set to 128
*                       - IWDG_Prescaler_256: IWDG prescaler set to 256
* Output         : None
* Return         : None
*******************************************************************************/
void IWDG_SetPrescaler(u8 IWDG_Prescaler)
{
  /* Check the parameters */
  assert_param(IS_IWDG_PRESCALER(IWDG_Prescaler));

  IWDG->PR = IWDG_Prescaler;
}

/*******************************************************************************
* Function Name  : IWDG_SetReload
* Description    : Sets IWDG Reload value.
* Input          : - Reload: specifies the IWDG Reload value.
*                    This parameter must be a number between 0 and 0x0FFF.
* Output         : None
* Return         : None
*******************************************************************************/
void IWDG_SetReload(u16 Reload)
{
  /* Check the parameters */
  assert_param(IS_IWDG_RELOAD(Reload));

  IWDG->RLR = Reload;
}

/*******************************************************************************
* Function Name  : IWDG_ReloadCounter
* Description    : Reloads IWDG counter with value defined in the reload register
*                  (write access to IWDG_PR and IWDG_RLR registers disabled).
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void IWDG_ReloadCounter(void)
{
  IWDG->KR = KR_KEY_Reload;
}

/*******************************************************************************
* Function Name  : IWDG_Enable
* Description    : Enables IWDG (write access to IWDG_PR and IWDG_RLR registers
*                  disabled).
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void IWDG_Enable(void)
{
  IWDG->KR = KR_KEY_Enable;
}

/*******************************************************************************
* Function Name  : IWDG_GetFlagStatus
* Description    : Checks whether the specified IWDG flag is set or not.
* Input          : - IWDG_FLAG: specifies the flag to check.
*                    This parameter can be one of the following values:
*                       - IWDG_FLAG_PVU: Prescaler Value Update on going
*                       - IWDG_FLAG_RVU: Reload Value Update on going
* Output         : None
* Return         : The new state of IWDG_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus IWDG_GetFlagStatus(u16 IWDG_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_IWDG_FLAG(IWDG_FLAG));

  if ((IWDG->SR & IWDG_FLAG) != (u32)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }

  /* Return the flag status */
  return bitstatus;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/




















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_lib.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all peripherals pointers initialization.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

#define EXT

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#ifdef DEBUG
/*******************************************************************************
* Function Name  : debug
* Description    : This function initialize peripherals pointers.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void debug(void)
{

/************************************* ADC ************************************/
#ifdef _ADC1
  ADC1 = (ADC_TypeDef *)  ADC1_BASE;
#endif /*_ADC1 */

#ifdef _ADC2
  ADC2 = (ADC_TypeDef *)  ADC2_BASE;
#endif /*_ADC2 */

#ifdef _ADC3
  ADC3 = (ADC_TypeDef *)  ADC3_BASE;
#endif /*_ADC3 */

/************************************* BKP ************************************/
#ifdef _BKP
  BKP = (BKP_TypeDef *)  BKP_BASE;
#endif /*_BKP */

/************************************* CAN ************************************/
#ifdef _CAN
  CAN = (CAN_TypeDef *)  CAN_BASE;
#endif /*_CAN */

/************************************* CRC ************************************/
#ifdef _CRC
  CRC = (CRC_TypeDef *)  CRC_BASE;
#endif /*_CRC */

/************************************* DAC ************************************/
#ifdef _DAC
  DAC = (DAC_TypeDef *)  DAC_BASE;
#endif /*_DAC */

/************************************* DBGMCU**********************************/
#ifdef _DBGMCU
  DBGMCU = (DBGMCU_TypeDef *)  DBGMCU_BASE;
#endif /*_DBGMCU */

/************************************* DMA ************************************/
#ifdef _DMA
  DMA1 = (DMA_TypeDef *)  DMA1_BASE;
  DMA2 = (DMA_TypeDef *)  DMA2_BASE;
#endif /*_DMA */

#ifdef _DMA1_Channel1
  DMA1_Channel1 = (DMA_Channel_TypeDef *)  DMA1_Channel1_BASE;
#endif /*_DMA1_Channel1 */

#ifdef _DMA1_Channel2
  DMA1_Channel2 = (DMA_Channel_TypeDef *)  DMA1_Channel2_BASE;
#endif /*_DMA1_Channel2 */

#ifdef _DMA1_Channel3
  DMA1_Channel3 = (DMA_Channel_TypeDef *)  DMA1_Channel3_BASE;
#endif /*_DMA1_Channel3 */

#ifdef _DMA1_Channel4
  DMA1_Channel4 = (DMA_Channel_TypeDef *)  DMA1_Channel4_BASE;
#endif /*_DMA1_Channel4 */

#ifdef _DMA1_Channel5
  DMA1_Channel5 = (DMA_Channel_TypeDef *)  DMA1_Channel5_BASE;
#endif /*_DMA1_Channel5 */

#ifdef _DMA1_Channel6
  DMA1_Channel6 = (DMA_Channel_TypeDef *)  DMA1_Channel6_BASE;
#endif /*_DMA1_Channel6 */

#ifdef _DMA1_Channel7
  DMA1_Channel7 = (DMA_Channel_TypeDef *)  DMA1_Channel7_BASE;
#endif /*_DMA1_Channel7 */

#ifdef _DMA2_Channel1
  DMA2_Channel1 = (DMA_Channel_TypeDef *)  DMA2_Channel1_BASE;
#endif /*_DMA2_Channel1 */

#ifdef _DMA2_Channel2
  DMA2_Channel2 = (DMA_Channel_TypeDef *)  DMA2_Channel2_BASE;
#endif /*_DMA2_Channel2 */

#ifdef _DMA2_Channel3
  DMA2_Channel3 = (DMA_Channel_TypeDef *)  DMA2_Channel3_BASE;
#endif /*_DMA2_Channel3 */

#ifdef _DMA2_Channel4
  DMA2_Channel4 = (DMA_Channel_TypeDef *)  DMA2_Channel4_BASE;
#endif /*_DMA2_Channel4 */

#ifdef _DMA2_Channel5
  DMA2_Channel5 = (DMA_Channel_TypeDef *)  DMA2_Channel5_BASE;
#endif /*_DMA2_Channel5 */

/************************************* EXTI ***********************************/
#ifdef _EXTI
  EXTI = (EXTI_TypeDef *)  EXTI_BASE;
#endif /*_EXTI */

/************************************* FLASH and Option Bytes *****************/
#ifdef _FLASH
  FLASH = (FLASH_TypeDef *)  FLASH_R_BASE;
  OB = (OB_TypeDef *)        OB_BASE;
#endif /*_FLASH */

/************************************* FSMC ***********************************/
#ifdef _FSMC
  FSMC_Bank1 = (FSMC_Bank1_TypeDef *)    FSMC_Bank1_R_BASE;
  FSMC_Bank1E = (FSMC_Bank1E_TypeDef *)  FSMC_Bank1E_R_BASE;  
  FSMC_Bank2 = (FSMC_Bank2_TypeDef *)    FSMC_Bank2_R_BASE; 
  FSMC_Bank3 = (FSMC_Bank3_TypeDef *)    FSMC_Bank3_R_BASE;
  FSMC_Bank4 = (FSMC_Bank4_TypeDef *)    FSMC_Bank4_R_BASE;
#endif /*_FSMC */

/************************************* GPIO ***********************************/
#ifdef _GPIOA
  GPIOA = (GPIO_TypeDef *)  GPIOA_BASE;
#endif /*_GPIOA */

#ifdef _GPIOB
  GPIOB = (GPIO_TypeDef *)  GPIOB_BASE;
#endif /*_GPIOB */

#ifdef _GPIOC
  GPIOC = (GPIO_TypeDef *)  GPIOC_BASE;
#endif /*_GPIOC */

#ifdef _GPIOD
  GPIOD = (GPIO_TypeDef *)  GPIOD_BASE;
#endif /*_GPIOD */

#ifdef _GPIOE
  GPIOE = (GPIO_TypeDef *)  GPIOE_BASE;
#endif /*_GPIOE */

#ifdef _GPIOF
  GPIOF = (GPIO_TypeDef *)  GPIOF_BASE;
#endif /*_GPIOF */

#ifdef _GPIOG
  GPIOG = (GPIO_TypeDef *)  GPIOG_BASE;
#endif /*_GPIOG */

#ifdef _AFIO
  AFIO = (AFIO_TypeDef *)  AFIO_BASE;
#endif /*_AFIO */

/************************************* I2C ************************************/
#ifdef _I2C1
  I2C1 = (I2C_TypeDef *)  I2C1_BASE;
#endif /*_I2C1 */

#ifdef _I2C2
  I2C2 = (I2C_TypeDef *)  I2C2_BASE;
#endif /*_I2C2 */

/************************************* IWDG ***********************************/
#ifdef _IWDG
  IWDG = (IWDG_TypeDef *) IWDG_BASE;
#endif /*_IWDG */

/************************************* NVIC ***********************************/
#ifdef _NVIC
  NVIC = (NVIC_TypeDef *)  NVIC_BASE;
  SCB = (SCB_TypeDef *)  SCB_BASE;
#endif /*_NVIC */

/************************************* PWR ************************************/
#ifdef _PWR
  PWR = (PWR_TypeDef *)  PWR_BASE;
#endif /*_PWR */

/************************************* RCC ************************************/
#ifdef _RCC
  RCC = (RCC_TypeDef *)  RCC_BASE;
#endif /*_RCC */

/************************************* RTC ************************************/
#ifdef _RTC
  RTC = (RTC_TypeDef *)  RTC_BASE;
#endif /*_RTC */

/************************************* SDIO ***********************************/
#ifdef _SDIO
  SDIO = (SDIO_TypeDef *)  SDIO_BASE;
#endif /*_SDIO */

/************************************* SPI ************************************/
#ifdef _SPI1
  SPI1 = (SPI_TypeDef *)  SPI1_BASE;
#endif /*_SPI1 */

#ifdef _SPI2
  SPI2 = (SPI_TypeDef *)  SPI2_BASE;
#endif /*_SPI2 */

#ifdef _SPI3
  SPI3 = (SPI_TypeDef *)  SPI3_BASE;
#endif /*_SPI3 */

/************************************* SysTick ********************************/
#ifdef _SysTick
  SysTick = (SysTick_TypeDef *)  SysTick_BASE;
#endif /*_SysTick */

/************************************* TIM ************************************/
#ifdef _TIM1
  TIM1 = (TIM_TypeDef *)  TIM1_BASE;
#endif /*_TIM1 */

#ifdef _TIM2
  TIM2 = (TIM_TypeDef *)  TIM2_BASE;
#endif /*_TIM2 */

#ifdef _TIM3
  TIM3 = (TIM_TypeDef *)  TIM3_BASE;
#endif /*_TIM3 */

#ifdef _TIM4
  TIM4 = (TIM_TypeDef *)  TIM4_BASE;
#endif /*_TIM4 */

#ifdef _TIM5
  TIM5 = (TIM_TypeDef *)  TIM5_BASE;
#endif /*_TIM5 */

#ifdef _TIM6
  TIM6 = (TIM_TypeDef *)  TIM6_BASE;
#endif /*_TIM6 */

#ifdef _TIM7
  TIM7 = (TIM_TypeDef *)  TIM7_BASE;
#endif /*_TIM7 */

#ifdef _TIM8
  TIM8 = (TIM_TypeDef *)  TIM8_BASE;
#endif /*_TIM8 */

/************************************* USART **********************************/
#ifdef _USART1
  USART1 = (USART_TypeDef *) USART1_BASE;
#endif /*_USART1 */

#ifdef _USART2
  USART2 = (USART_TypeDef *) USART2_BASE;
#endif /*_USART2 */

#ifdef _USART3
  USART3 = (USART_TypeDef *) USART3_BASE;
#endif /*_USART3 */

#ifdef _UART4
  UART4 = (USART_TypeDef *) UART4_BASE;
#endif /*_UART4 */

#ifdef _UART5
  UART5 = (USART_TypeDef *) UART5_BASE;
#endif /*_UART5 */

/************************************* WWDG ***********************************/
#ifdef _WWDG
  WWDG = (WWDG_TypeDef *)  WWDG_BASE;
#endif /*_WWDG */
}
#endif  /* DEBUG*/

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/


















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_nvic.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the NVIC firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_nvic.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define AIRCR_VECTKEY_MASK    ((u32)0x05FA0000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : NVIC_DeInit
* Description    : Deinitializes the NVIC peripheral registers to their default
*                  reset values.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_DeInit(void)
{
  u32 index = 0;
  
  NVIC->ICER[0] = 0xFFFFFFFF;
  NVIC->ICER[1] = 0x0FFFFFFF;
  NVIC->ICPR[0] = 0xFFFFFFFF;
  NVIC->ICPR[1] = 0x0FFFFFFF;
  
  for(index = 0; index < 0x0F; index++)
  {
     NVIC->IPR[index] = 0x00000000;
  } 
}

/*******************************************************************************
* Function Name  : NVIC_SCBDeInit
* Description    : Deinitializes the SCB peripheral registers to their default 
*                  reset values.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_SCBDeInit(void)
{
  u32 index = 0x00;
  
  SCB->ICSR = 0x0A000000;
  SCB->VTOR = 0x00000000;
  SCB->AIRCR = AIRCR_VECTKEY_MASK;
  SCB->SCR = 0x00000000;
  SCB->CCR = 0x00000000;
  for(index = 0; index < 0x03; index++)
  {
     SCB->SHPR[index] = 0;
  }
  SCB->SHCSR = 0x00000000;
  SCB->CFSR = 0xFFFFFFFF;
  SCB->HFSR = 0xFFFFFFFF;
  SCB->DFSR = 0xFFFFFFFF;
}

/*******************************************************************************
* Function Name  : NVIC_PriorityGroupConfig
* Description    : Configures the priority grouping: pre-emption priority
*                  and subpriority.
* Input          : - NVIC_PriorityGroup: specifies the priority grouping bits
*                    length. This parameter can be one of the following values:
*                       - NVIC_PriorityGroup_0: 0 bits for pre-emption priority
*                         4 bits for subpriority
*                       - NVIC_PriorityGroup_1: 1 bits for pre-emption priority
*                         3 bits for subpriority
*                       - NVIC_PriorityGroup_2: 2 bits for pre-emption priority
*                         2 bits for subpriority
*                       - NVIC_PriorityGroup_3: 3 bits for pre-emption priority
*                         1 bits for subpriority
*                       - NVIC_PriorityGroup_4: 4 bits for pre-emption priority
*                         0 bits for subpriority
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_PriorityGroupConfig(u32 NVIC_PriorityGroup)
{
  /* Check the parameters */
  assert_param(IS_NVIC_PRIORITY_GROUP(NVIC_PriorityGroup));
  
  /* Set the PRIGROUP[10:8] bits according to NVIC_PriorityGroup value */
  SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup;
}

/*******************************************************************************
* Function Name  : NVIC_Init
* Description    : Initializes the NVIC peripheral according to the specified
*                  parameters in the NVIC_InitStruct.
* Input          : - NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure
*                    that contains the configuration information for the
*                    specified NVIC peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct)
{
  u32 tmppriority = 0x00, tmpreg = 0x00, tmpmask = 0x00;
  u32 tmppre = 0, tmpsub = 0x0F;

  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NVIC_InitStruct->NVIC_IRQChannelCmd));
  assert_param(IS_NVIC_IRQ_CHANNEL(NVIC_InitStruct->NVIC_IRQChannel));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority));  
  assert_param(IS_NVIC_SUB_PRIORITY(NVIC_InitStruct->NVIC_IRQChannelSubPriority));
    
  if (NVIC_InitStruct->NVIC_IRQChannelCmd != DISABLE)
  {
    /* Compute the Corresponding IRQ Priority --------------------------------*/    
    tmppriority = (0x700 - (SCB->AIRCR & (u32)0x700))>> 0x08;
    tmppre = (0x4 - tmppriority);
    tmpsub = tmpsub >> tmppriority;
    
    tmppriority = (u32)NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority << tmppre;
    tmppriority |=  NVIC_InitStruct->NVIC_IRQChannelSubPriority & tmpsub;

    tmppriority = tmppriority << 0x04;
    tmppriority = ((u32)tmppriority) << ((NVIC_InitStruct->NVIC_IRQChannel & (u8)0x03) * 0x08);
    
    tmpreg = NVIC->IPR[(NVIC_InitStruct->NVIC_IRQChannel >> 0x02)];
    tmpmask = (u32)0xFF << ((NVIC_InitStruct->NVIC_IRQChannel & (u8)0x03) * 0x08);
    tmpreg &= ~tmpmask;
    tmppriority &= tmpmask;  
    tmpreg |= tmppriority;

    NVIC->IPR[(NVIC_InitStruct->NVIC_IRQChannel >> 0x02)] = tmpreg;
    
    /* Enable the Selected IRQ Channels --------------------------------------*/
    NVIC->ISER[(NVIC_InitStruct->NVIC_IRQChannel >> 0x05)] =
      (u32)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (u8)0x1F);
  }
  else
  {
    /* Disable the Selected IRQ Channels -------------------------------------*/
    NVIC->ICER[(NVIC_InitStruct->NVIC_IRQChannel >> 0x05)] =
      (u32)0x01 << (NVIC_InitStruct->NVIC_IRQChannel & (u8)0x1F);
  }
}

/*******************************************************************************
* Function Name  : NVIC_StructInit
* Description    : Fills each NVIC_InitStruct member with its default value.
* Input          : - NVIC_InitStruct: pointer to a NVIC_InitTypeDef structure which
*                    will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_StructInit(NVIC_InitTypeDef* NVIC_InitStruct)
{
  /* NVIC_InitStruct members default value */
  NVIC_InitStruct->NVIC_IRQChannel = 0x00;
  NVIC_InitStruct->NVIC_IRQChannelPreemptionPriority = 0x00;
  NVIC_InitStruct->NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStruct->NVIC_IRQChannelCmd = DISABLE;
}

/*******************************************************************************
* Function Name  : NVIC_SETPRIMASK
* Description    : Enables the PRIMASK priority: Raises the execution priority to 0.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_SETPRIMASK(void)
{
  __SETPRIMASK();
}

/*******************************************************************************
* Function Name  : NVIC_RESETPRIMASK
* Description    : Disables the PRIMASK priority.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_RESETPRIMASK(void)
{
  __RESETPRIMASK();
}

/*******************************************************************************
* Function Name  : NVIC_SETFAULTMASK
* Description    : Enables the FAULTMASK priority: Raises the execution priority to -1.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_SETFAULTMASK(void)
{
  __SETFAULTMASK();
}

/*******************************************************************************
* Function Name  : NVIC_RESETFAULTMASK
* Description    : Disables the FAULTMASK priority.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_RESETFAULTMASK(void)
{
  __RESETFAULTMASK();
}

/*******************************************************************************
* Function Name  : NVIC_BASEPRICONFIG
* Description    : The execution priority can be changed from 15 (lowest 
                   configurable priority) to 1. Writing a zero  value will disable 
*                  the mask of execution priority.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_BASEPRICONFIG(u32 NewPriority)
{
  /* Check the parameters */
  assert_param(IS_NVIC_BASE_PRI(NewPriority));
  
  __BASEPRICONFIG(NewPriority << 0x04);
}

/*******************************************************************************
* Function Name  : NVIC_GetBASEPRI
* Description    : Returns the BASEPRI mask value.
* Input          : None
* Output         : None
* Return         : BASEPRI register value
*******************************************************************************/
u32 NVIC_GetBASEPRI(void)
{
  return (__GetBASEPRI());
}

/*******************************************************************************
* Function Name  : NVIC_GetCurrentPendingIRQChannel
* Description    : Returns the current pending IRQ channel identifier.
* Input          : None
* Output         : None
* Return         : Pending IRQ Channel Identifier.
*******************************************************************************/
u16 NVIC_GetCurrentPendingIRQChannel(void)
{
  return ((u16)((SCB->ICSR & (u32)0x003FF000) >> 0x0C));
}

/*******************************************************************************
* Function Name  : NVIC_GetIRQChannelPendingBitStatus
* Description    : Checks whether the specified IRQ Channel pending bit is set
*                  or not.
* Input          : - NVIC_IRQChannel: specifies the interrupt pending bit to check.
* Output         : None
* Return         : The new state of IRQ Channel pending bit(SET or RESET).
*******************************************************************************/
ITStatus NVIC_GetIRQChannelPendingBitStatus(u8 NVIC_IRQChannel)
{
  ITStatus pendingirqstatus = RESET;
  u32 tmp = 0x00;
  
  /* Check the parameters */
  assert_param(IS_NVIC_IRQ_CHANNEL(NVIC_IRQChannel));
  
  tmp = ((u32)0x01 << (NVIC_IRQChannel & (u32)0x1F));

  if (((NVIC->ISPR[(NVIC_IRQChannel >> 0x05)]) & tmp) == tmp)
  {
    pendingirqstatus = SET;
  }
  else
  {
    pendingirqstatus = RESET;
  }
  return pendingirqstatus;
}

/*******************************************************************************
* Function Name  : NVIC_SetIRQChannelPendingBit
* Description    : Sets the NVIC’s interrupt pending bit.
* Input          : - NVIC_IRQChannel: specifies the interrupt pending bit to Set.
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_SetIRQChannelPendingBit(u8 NVIC_IRQChannel)
{
  /* Check the parameters */
  assert_param(IS_NVIC_IRQ_CHANNEL(NVIC_IRQChannel));
  
  *(vu32*) 0xE000EF00 = (u32)NVIC_IRQChannel;
}

/*******************************************************************************
* Function Name  : NVIC_ClearIRQChannelPendingBit
* Description    : Clears the NVIC’s interrupt pending bit.
* Input          : - NVIC_IRQChannel: specifies the interrupt pending bit to clear.
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_ClearIRQChannelPendingBit(u8 NVIC_IRQChannel)
{
  /* Check the parameters */
  assert_param(IS_NVIC_IRQ_CHANNEL(NVIC_IRQChannel));
  
  NVIC->ICPR[(NVIC_IRQChannel >> 0x05)] = (u32)0x01 << (NVIC_IRQChannel & (u32)0x1F);
}

/*******************************************************************************
* Function Name  : NVIC_GetCurrentActiveHandler
* Description    : Returns the current active Handler (IRQ Channel and
*                  SystemHandler) identifier.
* Input          : None
* Output         : None
* Return         : Active Handler Identifier.
*******************************************************************************/
u16 NVIC_GetCurrentActiveHandler(void)
{
  return ((u16)(SCB->ICSR & (u32)0x3FF));
}

/*******************************************************************************
* Function Name  : NVIC_GetIRQChannelActiveBitStatus
* Description    : Checks whether the specified IRQ Channel active bit is set
*                  or not.
* Input          : - NVIC_IRQChannel: specifies the interrupt active bit to check.
* Output         : None
* Return         : The new state of IRQ Channel active bit(SET or RESET).
*******************************************************************************/
ITStatus NVIC_GetIRQChannelActiveBitStatus(u8 NVIC_IRQChannel)
{
  ITStatus activeirqstatus = RESET;
  u32 tmp = 0x00;

  /* Check the parameters */
  assert_param(IS_NVIC_IRQ_CHANNEL(NVIC_IRQChannel));
  
  tmp = ((u32)0x01 << (NVIC_IRQChannel & (u32)0x1F));

  if (((NVIC->IABR[(NVIC_IRQChannel >> 0x05)]) & tmp) == tmp )
  {
    activeirqstatus = SET;
  }
  else
  {
    activeirqstatus = RESET;
  }
  return activeirqstatus;
}

/*******************************************************************************
* Function Name  : NVIC_GetCPUID
* Description    : Returns the ID number, the version number and the implementation
*                  details of the Cortex-M3 core.
* Input          : None
* Output         : None
* Return         : CPU ID.
*******************************************************************************/
u32 NVIC_GetCPUID(void)
{
  return (SCB->CPUID);
}

/*******************************************************************************
* Function Name  : NVIC_SetVectorTable
* Description    : Sets the vector table location and Offset.
* Input          : - NVIC_VectTab: specifies if the vector table is in RAM or
*                    FLASH memory.
*                    This parameter can be one of the following values:
*                       - NVIC_VectTab_RAM
*                       - NVIC_VectTab_FLASH
*                  - Offset: Vector Table base offset field. 
*                            This value must be a multiple of 0x100.
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)
{ 
  /* Check the parameters */
  assert_param(IS_NVIC_VECTTAB(NVIC_VectTab));
  assert_param(IS_NVIC_OFFSET(Offset));  
   
  SCB->VTOR = NVIC_VectTab | (Offset & (u32)0x1FFFFF80);
}

/*******************************************************************************
* Function Name  : NVIC_GenerateSystemReset
* Description    : Generates a system reset.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_GenerateSystemReset(void)
{
  SCB->AIRCR = AIRCR_VECTKEY_MASK | (u32)0x04;
}

/*******************************************************************************
* Function Name  : NVIC_GenerateCoreReset
* Description    : Generates a Core (Core + NVIC) reset.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_GenerateCoreReset(void)
{
  SCB->AIRCR = AIRCR_VECTKEY_MASK | (u32)0x01;
}

/*******************************************************************************
* Function Name  : NVIC_SystemLPConfig
* Description    : Selects the condition for the system to enter low power mode.
* Input          : - LowPowerMode: Specifies the new mode for the system to enter
*                    low power mode.
*                    This parameter can be one of the following values:
*                       - NVIC_LP_SEVONPEND
*                       - NVIC_LP_SLEEPDEEP
*                       - NVIC_LP_SLEEPONEXIT
*                  - NewState: new state of LP condition.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_SystemLPConfig(u8 LowPowerMode, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_NVIC_LP(LowPowerMode));
  assert_param(IS_FUNCTIONAL_STATE(NewState));  
  
  if (NewState != DISABLE)
  {
    SCB->SCR |= LowPowerMode;
  }
  else
  {
    SCB->SCR &= (u32)(~(u32)LowPowerMode);
  }
}

/*******************************************************************************
* Function Name  : NVIC_SystemHandlerConfig
* Description    : Enables or disables the specified System Handlers.
* Input          : - SystemHandler: specifies the system handler to be enabled
*                    or disabled.
*                    This parameter can be one of the following values:
*                       - SystemHandler_MemoryManage
*                       - SystemHandler_BusFault
*                       - SystemHandler_UsageFault
*                  - NewState: new state of  specified System Handlers.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_SystemHandlerConfig(u32 SystemHandler, FunctionalState NewState)
{
  u32 tmpreg = 0x00;

  /* Check the parameters */
  assert_param(IS_CONFIG_SYSTEM_HANDLER(SystemHandler));
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 
  
  tmpreg =  (u32)0x01 << (SystemHandler & (u32)0x1F);

  if (NewState != DISABLE)
  {
    SCB->SHCSR |= tmpreg;
  }
  else
  {
    SCB->SHCSR &= ~tmpreg;
  }
}

/*******************************************************************************
* Function Name  : NVIC_SystemHandlerPriorityConfig
* Description    : Configures the specified System Handlers priority.
* Input          : - SystemHandler: specifies the system handler to be
*                    enabled or disabled.
*                    This parameter can be one of the following values:
*                       - SystemHandler_MemoryManage
*                       - SystemHandler_BusFault
*                       - SystemHandler_UsageFault
*                       - SystemHandler_SVCall
*                       - SystemHandler_DebugMonitor
*                       - SystemHandler_PSV
*                       - SystemHandler_SysTick
*                  - SystemHandlerPreemptionPriority: new priority group of the
*                    specified system handlers.
*                  - SystemHandlerSubPriority: new sub priority of the specified
*                    system handlers.
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_SystemHandlerPriorityConfig(u32 SystemHandler, u8 SystemHandlerPreemptionPriority,
                                      u8 SystemHandlerSubPriority)
{
  u32 tmp1 = 0x00, tmp2 = 0xFF, handlermask = 0x00;
  u32 tmppriority = 0x00;

  /* Check the parameters */
  assert_param(IS_PRIORITY_SYSTEM_HANDLER(SystemHandler));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(SystemHandlerPreemptionPriority));  
  assert_param(IS_NVIC_SUB_PRIORITY(SystemHandlerSubPriority));
    
  tmppriority = (0x700 - (SCB->AIRCR & (u32)0x700))>> 0x08;
  tmp1 = (0x4 - tmppriority);
  tmp2 = tmp2 >> tmppriority;
    
  tmppriority = (u32)SystemHandlerPreemptionPriority << tmp1;
  tmppriority |=  SystemHandlerSubPriority & tmp2;

  tmppriority = tmppriority << 0x04;
  tmp1 = SystemHandler & (u32)0xC0;
  tmp1 = tmp1 >> 0x06; 
  tmp2 = (SystemHandler >> 0x08) & (u32)0x03;
  tmppriority = tmppriority << (tmp2 * 0x08);
  handlermask = (u32)0xFF << (tmp2 * 0x08);
  
  SCB->SHPR[tmp1] &= ~handlermask;
  SCB->SHPR[tmp1] |= tmppriority;
}

/*******************************************************************************
* Function Name  : NVIC_GetSystemHandlerPendingBitStatus
* Description    : Checks whether the specified System handlers pending bit is
*                  set or not.
* Input          : - SystemHandler: specifies the system handler pending bit to
*                    check.
*                    This parameter can be one of the following values:
*                       - SystemHandler_MemoryManage
*                       - SystemHandler_BusFault
*                       - SystemHandler_SVCall
* Output         : None
* Return         : The new state of System Handler pending bit(SET or RESET).
*******************************************************************************/
ITStatus NVIC_GetSystemHandlerPendingBitStatus(u32 SystemHandler)
{
  ITStatus bitstatus  = RESET;
  u32 tmp = 0x00, tmppos = 0x00;

  /* Check the parameters */
  assert_param(IS_GET_PENDING_SYSTEM_HANDLER(SystemHandler));
  
  tmppos = (SystemHandler >> 0x0A);
  tmppos &= (u32)0x0F;

  tmppos = (u32)0x01 << tmppos;

  tmp = SCB->SHCSR & tmppos;

  if (tmp == tmppos)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : NVIC_SetSystemHandlerPendingBit
* Description    : Sets System Handler pending bit.
* Input          : - SystemHandler: specifies the system handler pending bit
*                    to be set.
*                    This parameter can be one of the following values:
*                       - SystemHandler_NMI
*                       - SystemHandler_PSV
*                       - SystemHandler_SysTick
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_SetSystemHandlerPendingBit(u32 SystemHandler)
{
  u32 tmp = 0x00;

  /* Check the parameters */
  assert_param(IS_SET_PENDING_SYSTEM_HANDLER(SystemHandler));
  
  /* Get the System Handler pending bit position */
  tmp = SystemHandler & (u32)0x1F;
  /* Set the corresponding System Handler pending bit */
  SCB->ICSR |= ((u32)0x01 << tmp);
}

/*******************************************************************************
* Function Name  : NVIC_ClearSystemHandlerPendingBit
* Description    : Clears System Handler pending bit.
* Input          : - SystemHandler: specifies the system handler pending bit to
*                    be clear.
*                    This parameter can be one of the following values:
*                       - SystemHandler_PSV
*                       - SystemHandler_SysTick
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_ClearSystemHandlerPendingBit(u32 SystemHandler)
{
  u32 tmp = 0x00;

  /* Check the parameters */
  assert_param(IS_CLEAR_SYSTEM_HANDLER(SystemHandler));
  
  /* Get the System Handler pending bit position */
  tmp = SystemHandler & (u32)0x1F;
  /* Clear the corresponding System Handler pending bit */
  SCB->ICSR |= ((u32)0x01 << (tmp - 0x01));
}

/*******************************************************************************
* Function Name  : NVIC_GetSystemHandlerActiveBitStatus
* Description    : Checks whether the specified System handlers active bit is
*                  set or not.
* Input          : - SystemHandler: specifies the system handler active bit to
*                    check.
*                    This parameter can be one of the following values:
*                       - SystemHandler_MemoryManage
*                       - SystemHandler_BusFault
*                       - SystemHandler_UsageFault
*                       - SystemHandler_SVCall
*                       - SystemHandler_DebugMonitor
*                       - SystemHandler_PSV
*                       - SystemHandler_SysTick
* Output         : None
* Return         : The new state of System Handler active bit(SET or RESET).
*******************************************************************************/
ITStatus NVIC_GetSystemHandlerActiveBitStatus(u32 SystemHandler)
{
  ITStatus bitstatus  = RESET;

  u32 tmp = 0x00, tmppos = 0x00;

  /* Check the parameters */
  assert_param(IS_GET_ACTIVE_SYSTEM_HANDLER(SystemHandler));
  
  tmppos = (SystemHandler >> 0x0E) & (u32)0x0F;

  tmppos = (u32)0x01 << tmppos;

  tmp = SCB->SHCSR & tmppos;

  if (tmp == tmppos)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : NVIC_GetFaultHandlerSources
* Description    : Returns the system fault handlers sources.
* Input          : - SystemHandler: specifies the system handler to get its fault
*                    sources.
*                    This parameter can be one of the following values:
*                       - SystemHandler_HardFault
*                       - SystemHandler_MemoryManage
*                       - SystemHandler_BusFault
*                       - SystemHandler_UsageFault
*                       - SystemHandler_DebugMonitor
* Output         : None
* Return         : Source of the fault handler.
*******************************************************************************/
u32 NVIC_GetFaultHandlerSources(u32 SystemHandler)
{
  u32 faultsources = 0x00;
  u32 tmpreg = 0x00, tmppos = 0x00;

  /* Check the parameters */
  assert_param(IS_FAULT_SOURCE_SYSTEM_HANDLER(SystemHandler));
  
  tmpreg = (SystemHandler >> 0x12) & (u32)0x03;
  tmppos = (SystemHandler >> 0x14) & (u32)0x03;

  if (tmpreg == 0x00)
  {
    faultsources = SCB->HFSR;
  }
  else if (tmpreg == 0x01)
  {
    faultsources = SCB->CFSR >> (tmppos * 0x08);
    if (tmppos != 0x02)
    {
      faultsources &= (u32)0x0F;
    }
    else
    {
      faultsources &= (u32)0xFF;
    }
  }
  else
  {
    faultsources = SCB->DFSR;
  }
  return faultsources;
}

/*******************************************************************************
* Function Name  : NVIC_GetFaultAddress
* Description    : Returns the address of the location that generated a fault
*                  handler.
* Input          : - SystemHandler: specifies the system handler to get its
*                    fault address.
*                    This parameter can be one of the following values:
*                       - SystemHandler_MemoryManage
*                       - SystemHandler_BusFault
* Output         : None
* Return         : Fault address.
*******************************************************************************/
u32 NVIC_GetFaultAddress(u32 SystemHandler)
{
  u32 faultaddress = 0x00;
  u32 tmp = 0x00;

  /* Check the parameters */
  assert_param(IS_FAULT_ADDRESS_SYSTEM_HANDLER(SystemHandler));
  
  tmp = (SystemHandler >> 0x16) & (u32)0x01;

  if (tmp == 0x00)
  {
    faultaddress = SCB->MMFAR;
  }
  else
  {
    faultaddress = SCB->BFAR;
  }
  return faultaddress;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_pwr.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the PWR firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* --------- PWR registers bit address in the alias region ---------- */
#define PWR_OFFSET               (PWR_BASE - PERIPH_BASE)

/* --- CR Register ---*/
/* Alias word address of DBP bit */
#define CR_OFFSET                (PWR_OFFSET + 0x00)
#define DBP_BitNumber            0x08
#define CR_DBP_BB                (PERIPH_BB_BASE + (CR_OFFSET * 32) + (DBP_BitNumber * 4))

/* Alias word address of PVDE bit */
#define PVDE_BitNumber           0x04
#define CR_PVDE_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (PVDE_BitNumber * 4))

/* --- CSR Register ---*/
/* Alias word address of EWUP bit */
#define CSR_OFFSET               (PWR_OFFSET + 0x04)
#define EWUP_BitNumber           0x08
#define CSR_EWUP_BB              (PERIPH_BB_BASE + (CSR_OFFSET * 32) + (EWUP_BitNumber * 4))

/* ------------------ PWR registers bit mask ------------------------ */
/* CR register bit mask */
#define CR_PDDS_Set              ((u32)0x00000002)
#define CR_DS_Mask               ((u32)0xFFFFFFFC)
#define CR_CWUF_Set              ((u32)0x00000004)
#define CR_PLS_Mask              ((u32)0xFFFFFF1F)

/* --------- Cortex System Control register bit mask ---------------- */
/* Cortex System Control register address */
#define SCB_SysCtrl              ((u32)0xE000ED10)
/* SLEEPDEEP bit mask */
#define SysCtrl_SLEEPDEEP_Set    ((u32)0x00000004)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : PWR_DeInit
* Description    : Deinitializes the PWR peripheral registers to their default
*                  reset values.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PWR_DeInit(void)
{
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_PWR, DISABLE);
}

/*******************************************************************************
* Function Name  : PWR_BackupAccessCmd
* Description    : Enables or disables access to the RTC and backup registers.
* Input          : - NewState: new state of the access to the RTC and backup
*                    registers. This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void PWR_BackupAccessCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) CR_DBP_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : PWR_PVDCmd
* Description    : Enables or disables the Power Voltage Detector(PVD).
* Input          : - NewState: new state of the PVD.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void PWR_PVDCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) CR_PVDE_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : PWR_PVDLevelConfig
* Description    : Configures the voltage threshold detected by the Power Voltage
*                  Detector(PVD).
* Input          : - PWR_PVDLevel: specifies the PVD detection level
*                    This parameter can be one of the following values:
*                       - PWR_PVDLevel_2V2: PVD detection level set to 2.2V
*                       - PWR_PVDLevel_2V3: PVD detection level set to 2.3V
*                       - PWR_PVDLevel_2V4: PVD detection level set to 2.4V
*                       - PWR_PVDLevel_2V5: PVD detection level set to 2.5V
*                       - PWR_PVDLevel_2V6: PVD detection level set to 2.6V
*                       - PWR_PVDLevel_2V7: PVD detection level set to 2.7V
*                       - PWR_PVDLevel_2V8: PVD detection level set to 2.8V
*                       - PWR_PVDLevel_2V9: PVD detection level set to 2.9V
* Output         : None
* Return         : None
*******************************************************************************/
void PWR_PVDLevelConfig(u32 PWR_PVDLevel)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_PWR_PVD_LEVEL(PWR_PVDLevel));

  tmpreg = PWR->CR;

  /* Clear PLS[7:5] bits */
  tmpreg &= CR_PLS_Mask;

  /* Set PLS[7:5] bits according to PWR_PVDLevel value */
  tmpreg |= PWR_PVDLevel;

  /* Store the new value */
  PWR->CR = tmpreg;
}

/*******************************************************************************
* Function Name  : PWR_WakeUpPinCmd
* Description    : Enables or disables the WakeUp Pin functionality.
* Input          : - NewState: new state of the WakeUp Pin functionality.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void PWR_WakeUpPinCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) CSR_EWUP_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : PWR_EnterSTOPMode
* Description    : Enters STOP mode.
* Input          : - PWR_Regulator: specifies the regulator state in STOP mode.
*                    This parameter can be one of the following values:
*                       - PWR_Regulator_ON: STOP mode with regulator ON
*                       - PWR_Regulator_LowPower: STOP mode with
*                         regulator in low power mode
*                  - PWR_STOPEntry: specifies if STOP mode in entered with WFI or 
*                    WFE instruction.
*                    This parameter can be one of the following values:
*                       - PWR_STOPEntry_WFI: enter STOP mode with WFI instruction
*                       - PWR_STOPEntry_WFE: enter STOP mode with WFE instruction
* Output         : None
* Return         : None
*******************************************************************************/
void PWR_EnterSTOPMode(u32 PWR_Regulator, u8 PWR_STOPEntry)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_PWR_REGULATOR(PWR_Regulator));
  assert_param(IS_PWR_STOP_ENTRY(PWR_STOPEntry));
  
  /* Select the regulator state in STOP mode ---------------------------------*/
  tmpreg = PWR->CR;

  /* Clear PDDS and LPDS bits */
  tmpreg &= CR_DS_Mask;

  /* Set LPDS bit according to PWR_Regulator value */
  tmpreg |= PWR_Regulator;

  /* Store the new value */
  PWR->CR = tmpreg;

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  *(vu32 *) SCB_SysCtrl |= SysCtrl_SLEEPDEEP_Set;
  
  /* Select STOP mode entry --------------------------------------------------*/
  if(PWR_STOPEntry == PWR_STOPEntry_WFI)
  {   
    /* Request Wait For Interrupt */
    __WFI();
  }
  else
  {
    /* Request Wait For Event */
    __WFE();
  }
}

/*******************************************************************************
* Function Name  : PWR_EnterSTANDBYMode
* Description    : Enters STANDBY mode.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PWR_EnterSTANDBYMode(void)
{
  /* Clear Wake-up flag */
  PWR->CR |= CR_CWUF_Set;

  /* Select STANDBY mode */
  PWR->CR |= CR_PDDS_Set;

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  *(vu32 *) SCB_SysCtrl |= SysCtrl_SLEEPDEEP_Set;

  /* Request Wait For Interrupt */
  __WFI();
}

/*******************************************************************************
* Function Name  : PWR_GetFlagStatus
* Description    : Checks whether the specified PWR flag is set or not.
* Input          : - PWR_FLAG: specifies the flag to check.
*                    This parameter can be one of the following values:
*                       - PWR_FLAG_WU: Wake Up flag
*                       - PWR_FLAG_SB: StandBy flag
*                       - PWR_FLAG_PVDO: PVD Output
* Output         : None
* Return         : The new state of PWR_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus PWR_GetFlagStatus(u32 PWR_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_PWR_GET_FLAG(PWR_FLAG));
  
  if ((PWR->CSR & PWR_FLAG) != (u32)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }

  /* Return the flag status */
  return bitstatus;
}

/*******************************************************************************
* Function Name  : PWR_ClearFlag
* Description    : Clears the PWR's pending flags.
* Input          : - PWR_FLAG: specifies the flag to clear.
*                    This parameter can be one of the following values:
*                       - PWR_FLAG_WU: Wake Up flag
*                       - PWR_FLAG_SB: StandBy flag
* Output         : None
* Return         : None
*******************************************************************************/
void PWR_ClearFlag(u32 PWR_FLAG)
{
  /* Check the parameters */
  assert_param(IS_PWR_CLEAR_FLAG(PWR_FLAG));
         
  PWR->CR |=  PWR_FLAG << 2;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_rcc.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the RCC firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ------------ RCC registers bit address in the alias region ----------- */
#define RCC_OFFSET                (RCC_BASE - PERIPH_BASE)

/* --- CR Register ---*/
/* Alias word address of HSION bit */
#define CR_OFFSET                 (RCC_OFFSET + 0x00)
#define HSION_BitNumber           0x00
#define CR_HSION_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (HSION_BitNumber * 4))

/* Alias word address of PLLON bit */
#define PLLON_BitNumber           0x18
#define CR_PLLON_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (PLLON_BitNumber * 4))

/* Alias word address of CSSON bit */
#define CSSON_BitNumber           0x13
#define CR_CSSON_BB               (PERIPH_BB_BASE + (CR_OFFSET * 32) + (CSSON_BitNumber * 4))

/* --- CFGR Register ---*/
/* Alias word address of USBPRE bit */
#define CFGR_OFFSET               (RCC_OFFSET + 0x04)
#define USBPRE_BitNumber          0x16
#define CFGR_USBPRE_BB            (PERIPH_BB_BASE + (CFGR_OFFSET * 32) + (USBPRE_BitNumber * 4))

/* --- BDCR Register ---*/
/* Alias word address of RTCEN bit */
#define BDCR_OFFSET               (RCC_OFFSET + 0x20)
#define RTCEN_BitNumber           0x0F
#define BDCR_RTCEN_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (RTCEN_BitNumber * 4))

/* Alias word address of BDRST bit */
#define BDRST_BitNumber           0x10
#define BDCR_BDRST_BB             (PERIPH_BB_BASE + (BDCR_OFFSET * 32) + (BDRST_BitNumber * 4))

/* --- CSR Register ---*/
/* Alias word address of LSION bit */
#define CSR_OFFSET                (RCC_OFFSET + 0x24)
#define LSION_BitNumber           0x00
#define CSR_LSION_BB              (PERIPH_BB_BASE + (CSR_OFFSET * 32) + (LSION_BitNumber * 4))

/* ---------------------- RCC registers bit mask ------------------------ */
/* CR register bit mask */
#define CR_HSEBYP_Reset           ((u32)0xFFFBFFFF)
#define CR_HSEBYP_Set             ((u32)0x00040000)
#define CR_HSEON_Reset            ((u32)0xFFFEFFFF)
#define CR_HSEON_Set              ((u32)0x00010000)
#define CR_HSITRIM_Mask           ((u32)0xFFFFFF07)

/* CFGR register bit mask */
#define CFGR_PLL_Mask             ((u32)0xFFC0FFFF)
#define CFGR_PLLMull_Mask         ((u32)0x003C0000)
#define CFGR_PLLSRC_Mask          ((u32)0x00010000)
#define CFGR_PLLXTPRE_Mask        ((u32)0x00020000)
#define CFGR_SWS_Mask             ((u32)0x0000000C)
#define CFGR_SW_Mask              ((u32)0xFFFFFFFC)
#define CFGR_HPRE_Reset_Mask      ((u32)0xFFFFFF0F)
#define CFGR_HPRE_Set_Mask        ((u32)0x000000F0)
#define CFGR_PPRE1_Reset_Mask     ((u32)0xFFFFF8FF)
#define CFGR_PPRE1_Set_Mask       ((u32)0x00000700)
#define CFGR_PPRE2_Reset_Mask     ((u32)0xFFFFC7FF)
#define CFGR_PPRE2_Set_Mask       ((u32)0x00003800)
#define CFGR_ADCPRE_Reset_Mask    ((u32)0xFFFF3FFF)
#define CFGR_ADCPRE_Set_Mask      ((u32)0x0000C000)

/* CSR register bit mask */
#define CSR_RMVF_Set              ((u32)0x01000000)

/* RCC Flag Mask */
#define FLAG_Mask                 ((u8)0x1F)

/* Typical Value of the HSI in Hz */
#define HSI_Value                 ((u32)8000000)

/* CIR register byte 2 (Bits[15:8]) base address */
#define CIR_BYTE2_ADDRESS         ((u32)0x40021009)
/* CIR register byte 3 (Bits[23:16]) base address */
#define CIR_BYTE3_ADDRESS         ((u32)0x4002100A)

/* CFGR register byte 4 (Bits[31:24]) base address */
#define CFGR_BYTE4_ADDRESS        ((u32)0x40021007)

/* BDCR register base address */
#define BDCR_ADDRESS              (PERIPH_BASE + BDCR_OFFSET)

#ifndef HSEStartUp_TimeOut
/* Time out for HSE start up */
#define HSEStartUp_TimeOut        ((u16)0x0500)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uc8 APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};
static uc8 ADCPrescTable[4] = {2, 4, 6, 8};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : RCC_DeInit
* Description    : Resets the RCC clock configuration to the default reset state.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_DeInit(void)
{
  /* Set HSION bit */
  RCC->CR |= (u32)0x00000001;

  /* Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0] and MCO[2:0] bits */
  RCC->CFGR &= (u32)0xF8FF0000;
  
  /* Reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (u32)0xFEF6FFFF;

  /* Reset HSEBYP bit */
  RCC->CR &= (u32)0xFFFBFFFF;

  /* Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE bits */
  RCC->CFGR &= (u32)0xFF80FFFF;

  /* Disable all interrupts */
  RCC->CIR = 0x00000000;
}

/*******************************************************************************
* Function Name  : RCC_HSEConfig
* Description    : Configures the External High Speed oscillator (HSE).
*                  HSE can not be stopped if it is used directly or through the 
*                  PLL as system clock.
* Input          : - RCC_HSE: specifies the new state of the HSE.
*                    This parameter can be one of the following values:
*                       - RCC_HSE_OFF: HSE oscillator OFF
*                       - RCC_HSE_ON: HSE oscillator ON
*                       - RCC_HSE_Bypass: HSE oscillator bypassed with external
*                         clock
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_HSEConfig(u32 RCC_HSE)
{
  /* Check the parameters */
  assert_param(IS_RCC_HSE(RCC_HSE));

  /* Reset HSEON and HSEBYP bits before configuring the HSE ------------------*/
  /* Reset HSEON bit */
  RCC->CR &= CR_HSEON_Reset;

  /* Reset HSEBYP bit */
  RCC->CR &= CR_HSEBYP_Reset;

  /* Configure HSE (RCC_HSE_OFF is already covered by the code section above) */
  switch(RCC_HSE)
  {
    case RCC_HSE_ON:
      /* Set HSEON bit */
      RCC->CR |= CR_HSEON_Set;
      break;
      
    case RCC_HSE_Bypass:
      /* Set HSEBYP and HSEON bits */
      RCC->CR |= CR_HSEBYP_Set | CR_HSEON_Set;
      break;            
      
    default:
      break;      
  }
}

/*******************************************************************************
* Function Name  : RCC_WaitForHSEStartUp
* Description    : Waits for HSE start-up.
* Input          : None
* Output         : None
* Return         : An ErrorStatus enumuration value:
*                         - SUCCESS: HSE oscillator is stable and ready to use
*                         - ERROR: HSE oscillator not yet ready
*******************************************************************************/
ErrorStatus RCC_WaitForHSEStartUp(void)
{
  vu32 StartUpCounter = 0;
  ErrorStatus status = ERROR;
  FlagStatus HSEStatus = RESET;
  
  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC_GetFlagStatus(RCC_FLAG_HSERDY);
    StartUpCounter++;  
  } while((HSEStatus == RESET) && (StartUpCounter != HSEStartUp_TimeOut));


  if (RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET)
  {
    status = SUCCESS;
  }
  else
  {
    status = ERROR;
  }  

  return (status);
}

/*******************************************************************************
* Function Name  : RCC_AdjustHSICalibrationValue
* Description    : Adjusts the Internal High Speed oscillator (HSI) calibration
*                  value.
* Input          : - HSICalibrationValue: specifies the calibration trimming value.
*                    This parameter must be a number between 0 and 0x1F.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_AdjustHSICalibrationValue(u8 HSICalibrationValue)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_RCC_CALIBRATION_VALUE(HSICalibrationValue));

  tmpreg = RCC->CR;

  /* Clear HSITRIM[4:0] bits */
  tmpreg &= CR_HSITRIM_Mask;

  /* Set the HSITRIM[4:0] bits according to HSICalibrationValue value */
  tmpreg |= (u32)HSICalibrationValue << 3;

  /* Store the new value */
  RCC->CR = tmpreg;
}

/*******************************************************************************
* Function Name  : RCC_HSICmd
* Description    : Enables or disables the Internal High Speed oscillator (HSI).
*                  HSI can not be stopped if it is used directly or through the 
*                  PLL as system clock.
* Input          : - NewState: new state of the HSI.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_HSICmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) CR_HSION_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : RCC_PLLConfig
* Description    : Configures the PLL clock source and multiplication factor.
*                  This function must be used only when the PLL is disabled.
* Input          : - RCC_PLLSource: specifies the PLL entry clock source.
*                    This parameter can be one of the following values:
*                       - RCC_PLLSource_HSI_Div2: HSI oscillator clock divided
*                         by 2 selected as PLL clock entry
*                       - RCC_PLLSource_HSE_Div1: HSE oscillator clock selected
*                         as PLL clock entry
*                       - RCC_PLLSource_HSE_Div2: HSE oscillator clock divided
*                         by 2 selected as PLL clock entry
*                  - RCC_PLLMul: specifies the PLL multiplication factor.
*                    This parameter can be RCC_PLLMul_x where x:[2,16]
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_PLLConfig(u32 RCC_PLLSource, u32 RCC_PLLMul)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_RCC_PLL_SOURCE(RCC_PLLSource));
  assert_param(IS_RCC_PLL_MUL(RCC_PLLMul));

  tmpreg = RCC->CFGR;

  /* Clear PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
  tmpreg &= CFGR_PLL_Mask;

  /* Set the PLL configuration bits */
  tmpreg |= RCC_PLLSource | RCC_PLLMul;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/*******************************************************************************
* Function Name  : RCC_PLLCmd
* Description    : Enables or disables the PLL.
*                  The PLL can not be disabled if it is used as system clock.
* Input          : - NewState: new state of the PLL.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_PLLCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) CR_PLLON_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : RCC_SYSCLKConfig
* Description    : Configures the system clock (SYSCLK).
* Input          : - RCC_SYSCLKSource: specifies the clock source used as system
*                    clock. This parameter can be one of the following values:
*                       - RCC_SYSCLKSource_HSI: HSI selected as system clock
*                       - RCC_SYSCLKSource_HSE: HSE selected as system clock
*                       - RCC_SYSCLKSource_PLLCLK: PLL selected as system clock
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_SYSCLKConfig(u32 RCC_SYSCLKSource)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_RCC_SYSCLK_SOURCE(RCC_SYSCLKSource));

  tmpreg = RCC->CFGR;

  /* Clear SW[1:0] bits */
  tmpreg &= CFGR_SW_Mask;

  /* Set SW[1:0] bits according to RCC_SYSCLKSource value */
  tmpreg |= RCC_SYSCLKSource;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/*******************************************************************************
* Function Name  : RCC_GetSYSCLKSource
* Description    : Returns the clock source used as system clock.
* Input          : None
* Output         : None
* Return         : The clock source used as system clock. The returned value can
*                  be one of the following:
*                       - 0x00: HSI used as system clock
*                       - 0x04: HSE used as system clock
*                       - 0x08: PLL used as system clock
*******************************************************************************/
u8 RCC_GetSYSCLKSource(void)
{
  return ((u8)(RCC->CFGR & CFGR_SWS_Mask));
}

/*******************************************************************************
* Function Name  : RCC_HCLKConfig
* Description    : Configures the AHB clock (HCLK).
* Input          : - RCC_SYSCLK: defines the AHB clock divider. This clock is
*                    derived from the system clock (SYSCLK).
*                    This parameter can be one of the following values:
*                       - RCC_SYSCLK_Div1: AHB clock = SYSCLK
*                       - RCC_SYSCLK_Div2: AHB clock = SYSCLK/2
*                       - RCC_SYSCLK_Div4: AHB clock = SYSCLK/4
*                       - RCC_SYSCLK_Div8: AHB clock = SYSCLK/8
*                       - RCC_SYSCLK_Div16: AHB clock = SYSCLK/16
*                       - RCC_SYSCLK_Div64: AHB clock = SYSCLK/64
*                       - RCC_SYSCLK_Div128: AHB clock = SYSCLK/128
*                       - RCC_SYSCLK_Div256: AHB clock = SYSCLK/256
*                       - RCC_SYSCLK_Div512: AHB clock = SYSCLK/512
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_HCLKConfig(u32 RCC_SYSCLK)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_RCC_HCLK(RCC_SYSCLK));

  tmpreg = RCC->CFGR;

  /* Clear HPRE[3:0] bits */
  tmpreg &= CFGR_HPRE_Reset_Mask;

  /* Set HPRE[3:0] bits according to RCC_SYSCLK value */
  tmpreg |= RCC_SYSCLK;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/*******************************************************************************
* Function Name  : RCC_PCLK1Config
* Description    : Configures the Low Speed APB clock (PCLK1).
* Input          : - RCC_HCLK: defines the APB1 clock divider. This clock is
*                    derived from the AHB clock (HCLK).
*                    This parameter can be one of the following values:
*                       - RCC_HCLK_Div1: APB1 clock = HCLK
*                       - RCC_HCLK_Div2: APB1 clock = HCLK/2
*                       - RCC_HCLK_Div4: APB1 clock = HCLK/4
*                       - RCC_HCLK_Div8: APB1 clock = HCLK/8
*                       - RCC_HCLK_Div16: APB1 clock = HCLK/16
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_PCLK1Config(u32 RCC_HCLK)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_RCC_PCLK(RCC_HCLK));

  tmpreg = RCC->CFGR;

  /* Clear PPRE1[2:0] bits */
  tmpreg &= CFGR_PPRE1_Reset_Mask;

  /* Set PPRE1[2:0] bits according to RCC_HCLK value */
  tmpreg |= RCC_HCLK;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/*******************************************************************************
* Function Name  : RCC_PCLK2Config
* Description    : Configures the High Speed APB clock (PCLK2).
* Input          : - RCC_HCLK: defines the APB2 clock divider. This clock is
*                    derived from the AHB clock (HCLK).
*                    This parameter can be one of the following values:
*                       - RCC_HCLK_Div1: APB2 clock = HCLK
*                       - RCC_HCLK_Div2: APB2 clock = HCLK/2
*                       - RCC_HCLK_Div4: APB2 clock = HCLK/4
*                       - RCC_HCLK_Div8: APB2 clock = HCLK/8
*                       - RCC_HCLK_Div16: APB2 clock = HCLK/16
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_PCLK2Config(u32 RCC_HCLK)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_RCC_PCLK(RCC_HCLK));

  tmpreg = RCC->CFGR;

  /* Clear PPRE2[2:0] bits */
  tmpreg &= CFGR_PPRE2_Reset_Mask;

  /* Set PPRE2[2:0] bits according to RCC_HCLK value */
  tmpreg |= RCC_HCLK << 3;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/*******************************************************************************
* Function Name  : RCC_ITConfig
* Description    : Enables or disables the specified RCC interrupts.
* Input          : - RCC_IT: specifies the RCC interrupt sources to be enabled
*                    or disabled.
*                    This parameter can be any combination of the following values:
*                       - RCC_IT_LSIRDY: LSI ready interrupt
*                       - RCC_IT_LSERDY: LSE ready interrupt
*                       - RCC_IT_HSIRDY: HSI ready interrupt
*                       - RCC_IT_HSERDY: HSE ready interrupt
*                       - RCC_IT_PLLRDY: PLL ready interrupt
*                  - NewState: new state of the specified RCC interrupts.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_ITConfig(u8 RCC_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_IT(RCC_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Perform Byte access to RCC_CIR[12:8] bits to enable the selected interrupts */
    *(vu8 *) CIR_BYTE2_ADDRESS |= RCC_IT;
  }
  else
  {
    /* Perform Byte access to RCC_CIR[12:8] bits to disable the selected interrupts */
    *(vu8 *) CIR_BYTE2_ADDRESS &= (u8)~RCC_IT;
  }
}

/*******************************************************************************
* Function Name  : RCC_USBCLKConfig
* Description    : Configures the USB clock (USBCLK).
* Input          : - RCC_USBCLKSource: specifies the USB clock source. This clock
*                    is derived from the PLL output.
*                    This parameter can be one of the following values:
*                       - RCC_USBCLKSource_PLLCLK_1Div5: PLL clock divided by 1,5
*                         selected as USB clock source
*                       - RCC_USBCLKSource_PLLCLK_Div1: PLL clock selected as USB
*                         clock source
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_USBCLKConfig(u32 RCC_USBCLKSource)
{
  /* Check the parameters */
  assert_param(IS_RCC_USBCLK_SOURCE(RCC_USBCLKSource));

  *(vu32 *) CFGR_USBPRE_BB = RCC_USBCLKSource;
}

/*******************************************************************************
* Function Name  : RCC_ADCCLKConfig
* Description    : Configures the ADC clock (ADCCLK).
* Input          : - RCC_PCLK2: defines the ADC clock divider. This clock is
*                    derived from the APB2 clock (PCLK2).
*                    This parameter can be one of the following values:
*                       - RCC_PCLK2_Div2: ADC clock = PCLK2/2
*                       - RCC_PCLK2_Div4: ADC clock = PCLK2/4
*                       - RCC_PCLK2_Div6: ADC clock = PCLK2/6
*                       - RCC_PCLK2_Div8: ADC clock = PCLK2/8
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_ADCCLKConfig(u32 RCC_PCLK2)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_RCC_ADCCLK(RCC_PCLK2));

  tmpreg = RCC->CFGR;

  /* Clear ADCPRE[1:0] bits */
  tmpreg &= CFGR_ADCPRE_Reset_Mask;

  /* Set ADCPRE[1:0] bits according to RCC_PCLK2 value */
  tmpreg |= RCC_PCLK2;

  /* Store the new value */
  RCC->CFGR = tmpreg;
}

/*******************************************************************************
* Function Name  : RCC_LSEConfig
* Description    : Configures the External Low Speed oscillator (LSE).
* Input          : - RCC_LSE: specifies the new state of the LSE.
*                    This parameter can be one of the following values:
*                       - RCC_LSE_OFF: LSE oscillator OFF
*                       - RCC_LSE_ON: LSE oscillator ON
*                       - RCC_LSE_Bypass: LSE oscillator bypassed with external
*                         clock
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_LSEConfig(u8 RCC_LSE)
{
  /* Check the parameters */
  assert_param(IS_RCC_LSE(RCC_LSE));

  /* Reset LSEON and LSEBYP bits before configuring the LSE ------------------*/
  /* Reset LSEON bit */
  *(vu8 *) BDCR_ADDRESS = RCC_LSE_OFF;

  /* Reset LSEBYP bit */
  *(vu8 *) BDCR_ADDRESS = RCC_LSE_OFF;

  /* Configure LSE (RCC_LSE_OFF is already covered by the code section above) */
  switch(RCC_LSE)
  {
    case RCC_LSE_ON:
      /* Set LSEON bit */
      *(vu8 *) BDCR_ADDRESS = RCC_LSE_ON;
      break;
      
    case RCC_LSE_Bypass:
      /* Set LSEBYP and LSEON bits */
      *(vu8 *) BDCR_ADDRESS = RCC_LSE_Bypass | RCC_LSE_ON;
      break;            
      
    default:
      break;      
  }
}

/*******************************************************************************
* Function Name  : RCC_LSICmd
* Description    : Enables or disables the Internal Low Speed oscillator (LSI).
*                  LSI can not be disabled if the IWDG is running.
* Input          : - NewState: new state of the LSI.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_LSICmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) CSR_LSION_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : RCC_RTCCLKConfig
* Description    : Configures the RTC clock (RTCCLK).
*                  Once the RTC clock is selected it can’t be changed unless the
*                  Backup domain is reset.
* Input          : - RCC_RTCCLKSource: specifies the RTC clock source.
*                    This parameter can be one of the following values:
*                       - RCC_RTCCLKSource_LSE: LSE selected as RTC clock
*                       - RCC_RTCCLKSource_LSI: LSI selected as RTC clock
*                       - RCC_RTCCLKSource_HSE_Div128: HSE clock divided by 128
*                         selected as RTC clock
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_RTCCLKConfig(u32 RCC_RTCCLKSource)
{
  /* Check the parameters */
  assert_param(IS_RCC_RTCCLK_SOURCE(RCC_RTCCLKSource));

  /* Select the RTC clock source */
  RCC->BDCR |= RCC_RTCCLKSource;
}

/*******************************************************************************
* Function Name  : RCC_RTCCLKCmd
* Description    : Enables or disables the RTC clock.
*                  This function must be used only after the RTC clock was
*                  selected using the RCC_RTCCLKConfig function.
* Input          : - NewState: new state of the RTC clock.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_RTCCLKCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) BDCR_RTCEN_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : RCC_GetClocksFreq
* Description    : Returns the frequencies of different on chip clocks.
* Input          : - RCC_Clocks: pointer to a RCC_ClocksTypeDef structure which
*                    will hold the clocks frequencies.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks)
{
  u32 tmp = 0, pllmull = 0, pllsource = 0, presc = 0;

  /* Get SYSCLK source -------------------------------------------------------*/
  tmp = RCC->CFGR & CFGR_SWS_Mask;

  switch (tmp)
  {
    case 0x00:  /* HSI used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSI_Value;
      break;

    case 0x04:  /* HSE used as system clock */
      RCC_Clocks->SYSCLK_Frequency = HSE_Value;
      break;

    case 0x08:  /* PLL used as system clock */
      /* Get PLL clock source and multiplication factor ----------------------*/
      pllmull = RCC->CFGR & CFGR_PLLMull_Mask;
      pllmull = ( pllmull >> 18) + 2;

      pllsource = RCC->CFGR & CFGR_PLLSRC_Mask;

      if (pllsource == 0x00)
      {/* HSI oscillator clock divided by 2 selected as PLL clock entry */
        RCC_Clocks->SYSCLK_Frequency = (HSI_Value >> 1) * pllmull;
      }
      else
      {/* HSE selected as PLL clock entry */

        if ((RCC->CFGR & CFGR_PLLXTPRE_Mask) != (u32)RESET)
        {/* HSE oscillator clock divided by 2 */

          RCC_Clocks->SYSCLK_Frequency = (HSE_Value >> 1) * pllmull;
        }
        else
        {
          RCC_Clocks->SYSCLK_Frequency = HSE_Value * pllmull;
        }
      }
      break;

    default:
      RCC_Clocks->SYSCLK_Frequency = HSI_Value;
      break;
  }

  /* Compute HCLK, PCLK1, PCLK2 and ADCCLK clocks frequencies ----------------*/
  /* Get HCLK prescaler */
  tmp = RCC->CFGR & CFGR_HPRE_Set_Mask;
  tmp = tmp >> 4;
  presc = APBAHBPrescTable[tmp];

  /* HCLK clock frequency */
  RCC_Clocks->HCLK_Frequency = RCC_Clocks->SYSCLK_Frequency >> presc;

  /* Get PCLK1 prescaler */
  tmp = RCC->CFGR & CFGR_PPRE1_Set_Mask;
  tmp = tmp >> 8;
  presc = APBAHBPrescTable[tmp];

  /* PCLK1 clock frequency */
  RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency >> presc;

  /* Get PCLK2 prescaler */
  tmp = RCC->CFGR & CFGR_PPRE2_Set_Mask;
  tmp = tmp >> 11;
  presc = APBAHBPrescTable[tmp];

  /* PCLK2 clock frequency */
  RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency >> presc;

  /* Get ADCCLK prescaler */
  tmp = RCC->CFGR & CFGR_ADCPRE_Set_Mask;
  tmp = tmp >> 14;
  presc = ADCPrescTable[tmp];

  /* ADCCLK clock frequency */
  RCC_Clocks->ADCCLK_Frequency = RCC_Clocks->PCLK2_Frequency / presc;
}

/*******************************************************************************
* Function Name  : RCC_AHBPeriphClockCmd
* Description    : Enables or disables the AHB peripheral clock.
* Input          : - RCC_AHBPeriph: specifies the AHB peripheral to gates its clock.
*                    This parameter can be any combination of the following values:
*                       - RCC_AHBPeriph_DMA1
*                       - RCC_AHBPeriph_DMA2
*                       - RCC_AHBPeriph_SRAM
*                       - RCC_AHBPeriph_FLITF
*                       - RCC_AHBPeriph_CRC
*                       - RCC_AHBPeriph_FSMC
*                       - RCC_AHBPeriph_SDIO
*                    SRAM and FLITF clock can be disabled only during sleep mode.
*                  - NewState: new state of the specified peripheral clock.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_AHBPeriphClockCmd(u32 RCC_AHBPeriph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_AHB_PERIPH(RCC_AHBPeriph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->AHBENR |= RCC_AHBPeriph;
  }
  else
  {
    RCC->AHBENR &= ~RCC_AHBPeriph;
  }
}

/*******************************************************************************
* Function Name  : RCC_APB2PeriphClockCmd
* Description    : Enables or disables the High Speed APB (APB2) peripheral clock.
* Input          : - RCC_APB2Periph: specifies the APB2 peripheral to gates its
*                    clock.
*                    This parameter can be any combination of the following values:
*                       - RCC_APB2Periph_AFIO, RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB,
*                         RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE,
*                         RCC_APB2Periph_GPIOF, RCC_APB2Periph_GPIOG, RCC_APB2Periph_ADC1,
*                         RCC_APB2Periph_ADC2, RCC_APB2Periph_TIM1, RCC_APB2Periph_SPI1,
*                         RCC_APB2Periph_TIM8, RCC_APB2Periph_USART1, RCC_APB2Periph_ADC3,
*                         RCC_APB2Periph_ALL
*                  - NewState: new state of the specified peripheral clock.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_APB2PeriphClockCmd(u32 RCC_APB2Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB2ENR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2ENR &= ~RCC_APB2Periph;
  }
}

/*******************************************************************************
* Function Name  : RCC_APB1PeriphClockCmd
* Description    : Enables or disables the Low Speed APB (APB1) peripheral clock.
* Input          : - RCC_APB1Periph: specifies the APB1 peripheral to gates its
*                    clock.
*                    This parameter can be any combination of the following values:
*                       - RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4,
*                         RCC_APB1Periph_TIM5, RCC_APB1Periph_TIM6, RCC_APB1Periph_TIM7,
*                         RCC_APB1Periph_WWDG, RCC_APB1Periph_SPI2, RCC_APB1Periph_SPI3,
*                         RCC_APB1Periph_USART2, RCC_APB1Periph_USART3, RCC_APB1Periph_USART4, 
*                         RCC_APB1Periph_USART5, RCC_APB1Periph_I2C1, RCC_APB1Periph_I2C2,
*                         RCC_APB1Periph_USB, RCC_APB1Periph_CAN, RCC_APB1Periph_BKP,
*                         RCC_APB1Periph_PWR, RCC_APB1Periph_DAC, RCC_APB1Periph_ALL
*                  - NewState: new state of the specified peripheral clock.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_APB1PeriphClockCmd(u32 RCC_APB1Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB1ENR |= RCC_APB1Periph;
  }
  else
  {
    RCC->APB1ENR &= ~RCC_APB1Periph;
  }
}

/*******************************************************************************
* Function Name  : RCC_APB2PeriphResetCmd
* Description    : Forces or releases High Speed APB (APB2) peripheral reset.
* Input          : - RCC_APB2Periph: specifies the APB2 peripheral to reset.
*                    This parameter can be any combination of the following values:
*                       - RCC_APB2Periph_AFIO, RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB,
*                         RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE,
*                         RCC_APB2Periph_GPIOF, RCC_APB2Periph_GPIOG, RCC_APB2Periph_ADC1,
*                         RCC_APB2Periph_ADC2, RCC_APB2Periph_TIM1, RCC_APB2Periph_SPI1,
*                         RCC_APB2Periph_TIM8, RCC_APB2Periph_USART1, RCC_APB2Periph_ADC3,
*                         RCC_APB2Periph_ALL
*                  - NewState: new state of the specified peripheral reset.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_APB2PeriphResetCmd(u32 RCC_APB2Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB2_PERIPH(RCC_APB2Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB2RSTR |= RCC_APB2Periph;
  }
  else
  {
    RCC->APB2RSTR &= ~RCC_APB2Periph;
  }
}

/*******************************************************************************
* Function Name  : RCC_APB1PeriphResetCmd
* Description    : Forces or releases Low Speed APB (APB1) peripheral reset.
* Input          : - RCC_APB1Periph: specifies the APB1 peripheral to reset.
*                    This parameter can be any combination of the following values:
*                       - RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4,
*                         RCC_APB1Periph_TIM5, RCC_APB1Periph_TIM6, RCC_APB1Periph_TIM7,
*                         RCC_APB1Periph_WWDG, RCC_APB1Periph_SPI2, RCC_APB1Periph_SPI3,
*                         RCC_APB1Periph_USART2, RCC_APB1Periph_USART3, RCC_APB1Periph_USART4, 
*                         RCC_APB1Periph_USART5, RCC_APB1Periph_I2C1, RCC_APB1Periph_I2C2,
*                         RCC_APB1Periph_USB, RCC_APB1Periph_CAN, RCC_APB1Periph_BKP,
*                         RCC_APB1Periph_PWR, RCC_APB1Periph_DAC, RCC_APB1Periph_ALL
*                  - NewState: new state of the specified peripheral clock.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_APB1PeriphResetCmd(u32 RCC_APB1Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RCC_APB1_PERIPH(RCC_APB1Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    RCC->APB1RSTR |= RCC_APB1Periph;
  }
  else
  {
    RCC->APB1RSTR &= ~RCC_APB1Periph;
  }
}

/*******************************************************************************
* Function Name  : RCC_BackupResetCmd
* Description    : Forces or releases the Backup domain reset.
* Input          : - NewState: new state of the Backup domain reset.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_BackupResetCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) BDCR_BDRST_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : RCC_ClockSecuritySystemCmd
* Description    : Enables or disables the Clock Security System.
* Input          : - NewState: new state of the Clock Security System..
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_ClockSecuritySystemCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(vu32 *) CR_CSSON_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : RCC_MCOConfig
* Description    : Selects the clock source to output on MCO pin.
* Input          : - RCC_MCO: specifies the clock source to output.
*                    This parameter can be one of the following values:
*                       - RCC_MCO_NoClock: No clock selected
*                       - RCC_MCO_SYSCLK: System clock selected
*                       - RCC_MCO_HSI: HSI oscillator clock selected
*                       - RCC_MCO_HSE: HSE oscillator clock selected
*                       - RCC_MCO_PLLCLK_Div2: PLL clock divided by 2 selected
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_MCOConfig(u8 RCC_MCO)
{
  /* Check the parameters */
  assert_param(IS_RCC_MCO(RCC_MCO));

  /* Perform Byte access to MCO[2:0] bits to select the MCO source */
  *(vu8 *) CFGR_BYTE4_ADDRESS = RCC_MCO;
}

/*******************************************************************************
* Function Name  : RCC_GetFlagStatus
* Description    : Checks whether the specified RCC flag is set or not.
* Input          : - RCC_FLAG: specifies the flag to check.
*                    This parameter can be one of the following values:
*                       - RCC_FLAG_HSIRDY: HSI oscillator clock ready
*                       - RCC_FLAG_HSERDY: HSE oscillator clock ready
*                       - RCC_FLAG_PLLRDY: PLL clock ready
*                       - RCC_FLAG_LSERDY: LSE oscillator clock ready
*                       - RCC_FLAG_LSIRDY: LSI oscillator clock ready
*                       - RCC_FLAG_PINRST: Pin reset
*                       - RCC_FLAG_PORRST: POR/PDR reset
*                       - RCC_FLAG_SFTRST: Software reset
*                       - RCC_FLAG_IWDGRST: Independent Watchdog reset
*                       - RCC_FLAG_WWDGRST: Window Watchdog reset
*                       - RCC_FLAG_LPWRRST: Low Power reset
* Output         : None
* Return         : The new state of RCC_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus RCC_GetFlagStatus(u8 RCC_FLAG)
{
  u32 tmp = 0;
  u32 statusreg = 0;
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_RCC_FLAG(RCC_FLAG));

  /* Get the RCC register index */
  tmp = RCC_FLAG >> 5;

  if (tmp == 1)               /* The flag to check is in CR register */
  {
    statusreg = RCC->CR;
  }
  else if (tmp == 2)          /* The flag to check is in BDCR register */
  {
    statusreg = RCC->BDCR;
  }
  else                       /* The flag to check is in CSR register */
  {
    statusreg = RCC->CSR;
  }

  /* Get the flag position */
  tmp = RCC_FLAG & FLAG_Mask;

  if ((statusreg & ((u32)1 << tmp)) != (u32)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }

  /* Return the flag status */
  return bitstatus;
}

/*******************************************************************************
* Function Name  : RCC_ClearFlag
* Description    : Clears the RCC reset flags.
*                  The reset flags are: RCC_FLAG_PINRST, RCC_FLAG_PORRST,
*                  RCC_FLAG_SFTRST, RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST,
*                  RCC_FLAG_LPWRRST
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_ClearFlag(void)
{
  /* Set RMVF bit to clear the reset flags */
  RCC->CSR |= CSR_RMVF_Set;
}

/*******************************************************************************
* Function Name  : RCC_GetITStatus
* Description    : Checks whether the specified RCC interrupt has occurred or not.
* Input          : - RCC_IT: specifies the RCC interrupt source to check.
*                    This parameter can be one of the following values:
*                       - RCC_IT_LSIRDY: LSI ready interrupt
*                       - RCC_IT_LSERDY: LSE ready interrupt
*                       - RCC_IT_HSIRDY: HSI ready interrupt
*                       - RCC_IT_HSERDY: HSE ready interrupt
*                       - RCC_IT_PLLRDY: PLL ready interrupt
*                       - RCC_IT_CSS: Clock Security System interrupt
* Output         : None
* Return         : The new state of RCC_IT (SET or RESET).
*******************************************************************************/
ITStatus RCC_GetITStatus(u8 RCC_IT)
{
  ITStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_RCC_GET_IT(RCC_IT));

  /* Check the status of the specified RCC interrupt */
  if ((RCC->CIR & RCC_IT) != (u32)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }

  /* Return the RCC_IT status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : RCC_ClearITPendingBit
* Description    : Clears the RCC’s interrupt pending bits.
* Input          : - RCC_IT: specifies the interrupt pending bit to clear.
*                    This parameter can be any combination of the following values:
*                       - RCC_IT_LSIRDY: LSI ready interrupt
*                       - RCC_IT_LSERDY: LSE ready interrupt
*                       - RCC_IT_HSIRDY: HSI ready interrupt
*                       - RCC_IT_HSERDY: HSE ready interrupt
*                       - RCC_IT_PLLRDY: PLL ready interrupt
*                       - RCC_IT_CSS: Clock Security System interrupt
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_ClearITPendingBit(u8 RCC_IT)
{
  /* Check the parameters */
  assert_param(IS_RCC_CLEAR_IT(RCC_IT));

  /* Perform Byte access to RCC_CIR[23:16] bits to clear the selected interrupt
     pending bits */
  *(vu8 *) CIR_BYTE3_ADDRESS = RCC_IT;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_rtc.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the RTC firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_rtc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CRL_CNF_Set      ((u16)0x0010)      /* Configuration Flag Enable Mask */
#define CRL_CNF_Reset    ((u16)0xFFEF)      /* Configuration Flag Disable Mask */
#define RTC_LSB_Mask     ((u32)0x0000FFFF)  /* RTC LSB Mask */
#define PRLH_MSB_Mask    ((u32)0x000F0000)  /* RTC Prescaler MSB Mask */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : RTC_ITConfig
* Description    : Enables or disables the specified RTC interrupts.
* Input          : - RTC_IT: specifies the RTC interrupts sources to be enabled
*                    or disabled.
*                    This parameter can be any combination of the following values:
*                       - RTC_IT_OW: Overflow interrupt
*                       - RTC_IT_ALR: Alarm interrupt
*                       - RTC_IT_SEC: Second interrupt
*                  - NewState: new state of the specified RTC interrupts.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_ITConfig(u16 RTC_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RTC_IT(RTC_IT));  
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    RTC->CRH |= RTC_IT;
  }
  else
  {
    RTC->CRH &= (u16)~RTC_IT;
  }
}

/*******************************************************************************
* Function Name  : RTC_EnterConfigMode
* Description    : Enters the RTC configuration mode.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_EnterConfigMode(void)
{
  /* Set the CNF flag to enter in the Configuration Mode */
  RTC->CRL |= CRL_CNF_Set;
}

/*******************************************************************************
* Function Name  : RTC_ExitConfigMode
* Description    : Exits from the RTC configuration mode.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_ExitConfigMode(void)
{
  /* Reset the CNF flag to exit from the Configuration Mode */
  RTC->CRL &= CRL_CNF_Reset;
}

/*******************************************************************************
* Function Name  : RTC_GetCounter
* Description    : Gets the RTC counter value.
* Input          : None
* Output         : None
* Return         : RTC counter value.
*******************************************************************************/
u32 RTC_GetCounter(void)
{
  u16 tmp = 0;
  tmp = RTC->CNTL;

  return (((u32)RTC->CNTH << 16 ) | tmp) ;
}

/*******************************************************************************
* Function Name  : RTC_SetCounter
* Description    : Sets the RTC counter value.
* Input          : - CounterValue: RTC counter new value.
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_SetCounter(u32 CounterValue)
{ 
  RTC_EnterConfigMode();

  /* Set RTC COUNTER MSB word */
  RTC->CNTH = CounterValue >> 16;
  /* Set RTC COUNTER LSB word */
  RTC->CNTL = (CounterValue & RTC_LSB_Mask);

  RTC_ExitConfigMode();
}

/*******************************************************************************
* Function Name  : RTC_SetPrescaler
* Description    : Sets the RTC prescaler value.
* Input          : - PrescalerValue: RTC prescaler new value.
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_SetPrescaler(u32 PrescalerValue)
{
  /* Check the parameters */
  assert_param(IS_RTC_PRESCALER(PrescalerValue));
  
  RTC_EnterConfigMode();

  /* Set RTC PRESCALER MSB word */
  RTC->PRLH = (PrescalerValue & PRLH_MSB_Mask) >> 16;
  /* Set RTC PRESCALER LSB word */
  RTC->PRLL = (PrescalerValue & RTC_LSB_Mask);

  RTC_ExitConfigMode();
}

/*******************************************************************************
* Function Name  : RTC_SetAlarm
* Description    : Sets the RTC alarm value.
* Input          : - AlarmValue: RTC alarm new value.
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_SetAlarm(u32 AlarmValue)
{  
  RTC_EnterConfigMode();

  /* Set the ALARM MSB word */
  RTC->ALRH = AlarmValue >> 16;
  /* Set the ALARM LSB word */
  RTC->ALRL = (AlarmValue & RTC_LSB_Mask);

  RTC_ExitConfigMode();
}

/*******************************************************************************
* Function Name  : RTC_GetDivider
* Description    : Gets the RTC divider value.
* Input          : None
* Output         : None
* Return         : RTC Divider value.
*******************************************************************************/
u32 RTC_GetDivider(void)
{
  u32 tmp = 0x00;

  tmp = ((u32)RTC->DIVH & (u32)0x000F) << 16;
  tmp |= RTC->DIVL;

  return tmp;
}

/*******************************************************************************
* Function Name  : RTC_WaitForLastTask
* Description    : Waits until last write operation on RTC registers has finished.
*                  This function must be called before any write to RTC registers.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_WaitForLastTask(void)
{
  /* Loop until RTOFF flag is set */
  while ((RTC->CRL & RTC_FLAG_RTOFF) == (u16)RESET)
  {
  }
}

/*******************************************************************************
* Function Name  : RTC_WaitForSynchro
* Description    : Waits until the RTC registers (RTC_CNT, RTC_ALR and RTC_PRL)
*                  are synchronized with RTC APB clock.
*                  This function must be called before any read operation after
*                  an APB reset or an APB clock stop.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_WaitForSynchro(void)
{
  /* Clear RSF flag */
  RTC->CRL &= (u16)~RTC_FLAG_RSF;

  /* Loop until RSF flag is set */
  while ((RTC->CRL & RTC_FLAG_RSF) == (u16)RESET)
  {
  }
}

/*******************************************************************************
* Function Name  : RTC_GetFlagStatus
* Description    : Checks whether the specified RTC flag is set or not.
* Input          : - RTC_FLAG: specifies the flag to check.
*                    This parameter can be one the following values:
*                       - RTC_FLAG_RTOFF: RTC Operation OFF flag
*                       - RTC_FLAG_RSF: Registers Synchronized flag
*                       - RTC_FLAG_OW: Overflow flag
*                       - RTC_FLAG_ALR: Alarm flag
*                       - RTC_FLAG_SEC: Second flag
* Output         : None
* Return         : The new state of RTC_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus RTC_GetFlagStatus(u16 RTC_FLAG)
{
  FlagStatus bitstatus = RESET;
  
  /* Check the parameters */
  assert_param(IS_RTC_GET_FLAG(RTC_FLAG)); 
  
  if ((RTC->CRL & RTC_FLAG) != (u16)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : RTC_ClearFlag
* Description    : Clears the RTC’s pending flags.
* Input          : - RTC_FLAG: specifies the flag to clear.
*                    This parameter can be any combination of the following values:
*                       - RTC_FLAG_RSF: Registers Synchronized flag. This flag
*                         is cleared only after an APB reset or an APB Clock stop.
*                       - RTC_FLAG_OW: Overflow flag
*                       - RTC_FLAG_ALR: Alarm flag
*                       - RTC_FLAG_SEC: Second flag
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_ClearFlag(u16 RTC_FLAG)
{
  /* Check the parameters */
  assert_param(IS_RTC_CLEAR_FLAG(RTC_FLAG)); 
    
  /* Clear the coressponding RTC flag */
  RTC->CRL &= (u16)~RTC_FLAG;
}

/*******************************************************************************
* Function Name  : RTC_GetITStatus
* Description    : Checks whether the specified RTC interrupt has occured or not.
* Input          : - RTC_IT: specifies the RTC interrupts sources to check.
*                    This parameter can be one of the following values:
*                       - RTC_IT_OW: Overflow interrupt
*                       - RTC_IT_ALR: Alarm interrupt
*                       - RTC_IT_SEC: Second interrupt
* Output         : None
* Return         : The new state of the RTC_IT (SET or RESET).
*******************************************************************************/
ITStatus RTC_GetITStatus(u16 RTC_IT)
{
  ITStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_RTC_GET_IT(RTC_IT)); 
  
  bitstatus = (ITStatus)(RTC->CRL & RTC_IT);

  if (((RTC->CRH & RTC_IT) != (u16)RESET) && (bitstatus != (u16)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : RTC_ClearITPendingBit
* Description    : Clears the RTC’s interrupt pending bits.
* Input          : - RTC_IT: specifies the interrupt pending bit to clear.
*                    This parameter can be any combination of the following values:
*                       - RTC_IT_OW: Overflow interrupt
*                       - RTC_IT_ALR: Alarm interrupt
*                       - RTC_IT_SEC: Second interrupt
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_ClearITPendingBit(u16 RTC_IT)
{
  /* Check the parameters */
  assert_param(IS_RTC_IT(RTC_IT));  
  
  /* Clear the coressponding RTC pending bit */
  RTC->CRL &= (u16)~RTC_IT;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_sdio.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the SDIO firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_sdio.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* ------------ SDIO registers bit address in the alias region ----------- */
#define SDIO_OFFSET                (SDIO_BASE - PERIPH_BASE)

/* --- CLKCR Register ---*/
/* Alias word address of CLKEN bit */
#define CLKCR_OFFSET              (SDIO_OFFSET + 0x04)
#define CLKEN_BitNumber           0x08
#define CLKCR_CLKEN_BB            (PERIPH_BB_BASE + (CLKCR_OFFSET * 32) + (CLKEN_BitNumber * 4))

/* --- CMD Register ---*/
/* Alias word address of SDIOSUSPEND bit */
#define CMD_OFFSET                (SDIO_OFFSET + 0x0C)
#define SDIOSUSPEND_BitNumber     0x0B
#define CMD_SDIOSUSPEND_BB        (PERIPH_BB_BASE + (CMD_OFFSET * 32) + (SDIOSUSPEND_BitNumber * 4))

/* Alias word address of ENCMDCOMPL bit */
#define ENCMDCOMPL_BitNumber      0x0C
#define CMD_ENCMDCOMPL_BB         (PERIPH_BB_BASE + (CMD_OFFSET * 32) + (ENCMDCOMPL_BitNumber * 4))

/* Alias word address of NIEN bit */
#define NIEN_BitNumber            0x0D
#define CMD_NIEN_BB               (PERIPH_BB_BASE + (CMD_OFFSET * 32) + (NIEN_BitNumber * 4))

/* Alias word address of ATACMD bit */
#define ATACMD_BitNumber          0x0E
#define CMD_ATACMD_BB             (PERIPH_BB_BASE + (CMD_OFFSET * 32) + (ATACMD_BitNumber * 4))

/* --- DCTRL Register ---*/
/* Alias word address of DMAEN bit */
#define DCTRL_OFFSET              (SDIO_OFFSET + 0x2C)
#define DMAEN_BitNumber           0x03
#define DCTRL_DMAEN_BB            (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (DMAEN_BitNumber * 4))

/* Alias word address of RWSTART bit */
#define RWSTART_BitNumber         0x08
#define DCTRL_RWSTART_BB          (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (RWSTART_BitNumber * 4))

/* Alias word address of RWSTOP bit */
#define RWSTOP_BitNumber          0x09
#define DCTRL_RWSTOP_BB           (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (RWSTOP_BitNumber * 4))

/* Alias word address of RWMOD bit */
#define RWMOD_BitNumber           0x0A
#define DCTRL_RWMOD_BB            (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (RWMOD_BitNumber * 4))

/* Alias word address of SDIOEN bit */
#define SDIOEN_BitNumber          0x0B
#define DCTRL_SDIOEN_BB           (PERIPH_BB_BASE + (DCTRL_OFFSET * 32) + (SDIOEN_BitNumber * 4))


/* ---------------------- SDIO registers bit mask ------------------------ */
/* --- CLKCR Register ---*/
/* CLKCR register clear mask */
#define CLKCR_CLEAR_MASK         ((u32)0xFFFF8100) 

/* --- PWRCTRL Register ---*/
/* SDIO PWRCTRL Mask */
#define PWR_PWRCTRL_MASK         ((u32)0xFFFFFFFC)

/* --- DCTRL Register ---*/
/* SDIO DCTRL Clear Mask */
#define DCTRL_CLEAR_MASK         ((u32)0xFFFFFF08)

/* --- CMD Register ---*/
/* CMD Register clear mask */
#define CMD_CLEAR_MASK           ((u32)0xFFFFF800)

/* SDIO RESP Registers Address */
#define SDIO_RESP_ADDR           ((u32)(SDIO_BASE + 0x14))

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : SDIO_DeInit
* Description    : Deinitializes the SDIO peripheral registers to their default
*                  reset values.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_DeInit(void)
{
  SDIO->POWER = 0x00000000;
  SDIO->CLKCR = 0x00000000;
  SDIO->ARG = 0x00000000;
  SDIO->CMD = 0x00000000;
  SDIO->DTIMER = 0x00000000;
  SDIO->DLEN = 0x00000000;
  SDIO->DCTRL = 0x00000000;
  SDIO->ICR = 0x00C007FF;
  SDIO->MASK = 0x00000000;
}

/*******************************************************************************
* Function Name  : SDIO_Init
* Description    : Initializes the SDIO peripheral according to the specified 
*                  parameters in the SDIO_InitStruct.
* Input          : SDIO_InitStruct : pointer to a SDIO_InitTypeDef structure 
*                  that contains the configuration information for the SDIO 
*                  peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_Init(SDIO_InitTypeDef* SDIO_InitStruct)
{
  u32 tmpreg = 0;
    
  /* Check the parameters */
  assert_param(IS_SDIO_CLOCK_EDGE(SDIO_InitStruct->SDIO_ClockEdge));
  assert_param(IS_SDIO_CLOCK_BYPASS(SDIO_InitStruct->SDIO_ClockBypass));
  assert_param(IS_SDIO_CLOCK_POWER_SAVE(SDIO_InitStruct->SDIO_ClockPowerSave));
  assert_param(IS_SDIO_BUS_WIDE(SDIO_InitStruct->SDIO_BusWide));
  assert_param(IS_SDIO_HARDWARE_FLOW_CONTROL(SDIO_InitStruct->SDIO_HardwareFlowControl)); 
   
/*---------------------------- SDIO CLKCR Configuration ------------------------*/  
  /* Get the SDIO CLKCR value */
  tmpreg = SDIO->CLKCR;
  
  /* Clear CLKDIV, PWRSAV, BYPASS, WIDBUS, NEGEDGE, HWFC_EN bits */
  tmpreg &= CLKCR_CLEAR_MASK;
  
  /* Set CLKDIV bits according to SDIO_ClockDiv value */
  /* Set PWRSAV bit according to SDIO_ClockPowerSave value */
  /* Set BYPASS bit according to SDIO_ClockBypass value */
  /* Set WIDBUS bits according to SDIO_BusWide value */
  /* Set NEGEDGE bits according to SDIO_ClockEdge value */
  /* Set HWFC_EN bits according to SDIO_HardwareFlowControl value */
  tmpreg |= (SDIO_InitStruct->SDIO_ClockDiv  | SDIO_InitStruct->SDIO_ClockPowerSave |
             SDIO_InitStruct->SDIO_ClockBypass | SDIO_InitStruct->SDIO_BusWide |
             SDIO_InitStruct->SDIO_ClockEdge | SDIO_InitStruct->SDIO_HardwareFlowControl); 
  
  /* Write to SDIO CLKCR */
  SDIO->CLKCR = tmpreg;             
}

/*******************************************************************************
* Function Name  : SDIO_StructInit
* Description    : Fills each SDIO_InitStruct member with its default value.
* Input          : SDIO_InitStruct: pointer to an SDIO_InitTypeDef structure which 
*                  will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_StructInit(SDIO_InitTypeDef* SDIO_InitStruct)
{
  /* SDIO_InitStruct members default value */
  SDIO_InitStruct->SDIO_ClockDiv = 0x00;
  SDIO_InitStruct->SDIO_ClockEdge = SDIO_ClockEdge_Rising;
  SDIO_InitStruct->SDIO_ClockBypass = SDIO_ClockBypass_Disable;
  SDIO_InitStruct->SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
  SDIO_InitStruct->SDIO_BusWide = SDIO_BusWide_1b;
  SDIO_InitStruct->SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
}

/*******************************************************************************
* Function Name  : SDIO_ClockCmd
* Description    : Enables or disables the SDIO Clock.
* Input          : NewState: new state of the SDIO Clock.
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_ClockCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) CLKCR_CLKEN_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : SDIO_SetPowerState
* Description    : Sets the power status of the controller.
* Input          : SDIO_PowerState: new state of the Power state. 
*                  This parameter can be one of the following values:
*                   - SDIO_PowerState_OFF
*                   - SDIO_PowerState_ON
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_SetPowerState(u32 SDIO_PowerState)
{
  /* Check the parameters */
  assert_param(IS_SDIO_POWER_STATE(SDIO_PowerState));
  
  SDIO->POWER &= PWR_PWRCTRL_MASK;
  SDIO->POWER |= SDIO_PowerState;
}

/*******************************************************************************
* Function Name  : SDIO_GetPowerState
* Description    : Gets the power status of the controller.
* Input          : None
* Output         : None
* Return         : Power status of the controller. The returned value can
*                  be one of the following:
*                       - 0x00: Power OFF
*                       - 0x02: Power UP
*                       - 0x03: Power ON 
*******************************************************************************/
u32 SDIO_GetPowerState(void)
{
  return (SDIO->POWER & (~PWR_PWRCTRL_MASK));
}

/*******************************************************************************
* Function Name  : SDIO_ITConfig
* Description    : Enables or disables the SDIO interrupts.
* Input          : - SDIO_IT: specifies the SDIO interrupt sources to be 
*                    enabled or disabled.
*                    This parameter can be one or a combination of the following
*                    values:
*                      - SDIO_IT_CCRCFAIL: Command response received (CRC check
*                                          failed) interrupt    
*                      - SDIO_IT_DCRCFAIL: Data block sent/received (CRC check 
*                                          failed) interrupt    
*                      - SDIO_IT_CTIMEOUT: Command response timeout interrupt    
*                      - SDIO_IT_DTIMEOUT: Data timeout interrupt    
*                      - SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt    
*                      - SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt     
*                      - SDIO_IT_CMDREND:  Command response received (CRC check 
*                                          passed) interrupt     
*                      - SDIO_IT_CMDSENT:  Command sent (no response required) 
*                                          interrupt     
*                      - SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is 
*                                          zero) interrupt     
*                      - SDIO_IT_STBITERR: Start bit not detected on all data 
*                                          signals in wide bus mode interrupt    
*                      - SDIO_IT_DBCKEND:  Data block sent/received (CRC check 
*                                          passed) interrupt    
*                      - SDIO_IT_CMDACT:   Command transfer in progress interrupt     
*                      - SDIO_IT_TXACT:    Data transmit in progress interrupt       
*                      - SDIO_IT_RXACT:    Data receive in progress interrupt      
*                      - SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt    
*                      - SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt   
*                      - SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt     
*                      - SDIO_IT_RXFIFOF:  Receive FIFO full interrupt     
*                      - SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt      
*                      - SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt     
*                      - SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt      
*                      - SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt      
*                      - SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt      
*                      - SDIO_IT_CEATAEND: CE-ATA command completion signal 
*                                          received for CMD61 interrupt
*                  - NewState: new state of the specified SDIO interrupts.
*                  This parameter can be: ENABLE or DISABLE.  
* Output         : None
* Return         : None 
*******************************************************************************/
void SDIO_ITConfig(u32 SDIO_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_SDIO_IT(SDIO_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the SDIO interrupts */
    SDIO->MASK |= SDIO_IT;
  }
  else
  {
    /* Disable the SDIO interrupts */
    SDIO->MASK &= ~SDIO_IT;
  } 
}

/*******************************************************************************
* Function Name  : SDIO_DMACmd
* Description    : Enables or disables the SDIO DMA request.
* Input          : NewState: new state of the selected SDIO DMA request.
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_DMACmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) DCTRL_DMAEN_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : SDIO_SendCommand
* Description    : Initializes the SDIO Command according to the specified 
*                  parameters in the SDIO_CmdInitStruct and send the command.
* Input          : SDIO_CmdInitStruct : pointer to a SDIO_CmdInitTypeDef 
*                  structure that contains the configuration information 
*                  for the SDIO command.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_SendCommand(SDIO_CmdInitTypeDef *SDIO_CmdInitStruct)
{
  u32 tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_SDIO_CMD_INDEX(SDIO_CmdInitStruct->SDIO_CmdIndex));
  assert_param(IS_SDIO_RESPONSE(SDIO_CmdInitStruct->SDIO_Response));
  assert_param(IS_SDIO_WAIT(SDIO_CmdInitStruct->SDIO_Wait));
  assert_param(IS_SDIO_CPSM(SDIO_CmdInitStruct->SDIO_CPSM));
  
/*---------------------------- SDIO ARG Configuration ------------------------*/
  /* Set the SDIO Argument value */
  SDIO->ARG = SDIO_CmdInitStruct->SDIO_Argument;
  
/*---------------------------- SDIO CMD Configuration ------------------------*/  
  /* Get the SDIO CMD value */
  tmpreg = SDIO->CMD;

  /* Clear CMDINDEX, WAITRESP, WAITINT, WAITPEND, CPSMEN bits */
  tmpreg &= CMD_CLEAR_MASK;
  /* Set CMDINDEX bits according to SDIO_CmdIndex value */
  /* Set WAITRESP bits according to SDIO_Response value */
  /* Set WAITINT and WAITPEND bits according to SDIO_Wait value */
  /* Set CPSMEN bits according to SDIO_CPSM value */
  tmpreg |= (u32)SDIO_CmdInitStruct->SDIO_CmdIndex | SDIO_CmdInitStruct->SDIO_Response
           | SDIO_CmdInitStruct->SDIO_Wait | SDIO_CmdInitStruct->SDIO_CPSM;
  
  /* Write to SDIO CMD */
  SDIO->CMD = tmpreg;
}

/*******************************************************************************
* Function Name  : SDIO_CmdStructInit
* Description    : Fills each SDIO_CmdInitStruct member with its default value.
* Input          : SDIO_CmdInitStruct: pointer to an SDIO_CmdInitTypeDef 
*                  structure which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_CmdStructInit(SDIO_CmdInitTypeDef* SDIO_CmdInitStruct)
{
  /* SDIO_CmdInitStruct members default value */
  SDIO_CmdInitStruct->SDIO_Argument = 0x00;
  SDIO_CmdInitStruct->SDIO_CmdIndex = 0x00;
  SDIO_CmdInitStruct->SDIO_Response = SDIO_Response_No;
  SDIO_CmdInitStruct->SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStruct->SDIO_CPSM = SDIO_CPSM_Disable;
}

/*******************************************************************************
* Function Name  : SDIO_GetCommandResponse
* Description    : Returns command index of last command for which response 
*                  received.
* Input          : None
* Output         : None
* Return         : Returns the command index of the last command response received.
*******************************************************************************/
u8 SDIO_GetCommandResponse(void)
{
  return (u8)(SDIO->RESPCMD);
}

/*******************************************************************************
* Function Name  : SDIO_GetResponse
* Description    : Returns response received from the card for the last command.
* Input          : - SDIO_RESP: Specifies the SDIO response register. 
*                     This parameter can be one of the following values:
*                       - SDIO_RESP1: Response Register 1
*                       - SDIO_RESP2: Response Register 2
*                       - SDIO_RESP3: Response Register 3
*                       - SDIO_RESP4: Response Register 4                       
* Output         : None
* Return         : The Corresponding response register value.
*******************************************************************************/
u32 SDIO_GetResponse(u32 SDIO_RESP)
{
  /* Check the parameters */
  assert_param(IS_SDIO_RESP(SDIO_RESP));
  
  return (*(vu32 *)(SDIO_RESP_ADDR + SDIO_RESP)); 
}

/*******************************************************************************
* Function Name  : SDIO_DataConfig
* Description    : Initializes the SDIO data path according to the specified 
*                  parameters in the SDIO_DataInitStruct.
* Input          : SDIO_DataInitStruct : pointer to a SDIO_DataInitTypeDef 
*                  structure that contains the configuration information 
*                  for the SDIO command.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_DataConfig(SDIO_DataInitTypeDef* SDIO_DataInitStruct)
{
  u32 tmpreg = 0;
  
  /* Check the parameters */
  assert_param(IS_SDIO_DATA_LENGTH(SDIO_DataInitStruct->SDIO_DataLength));
  assert_param(IS_SDIO_BLOCK_SIZE(SDIO_DataInitStruct->SDIO_DataBlockSize));
  assert_param(IS_SDIO_TRANSFER_DIR(SDIO_DataInitStruct->SDIO_TransferDir));
  assert_param(IS_SDIO_TRANSFER_MODE(SDIO_DataInitStruct->SDIO_TransferMode));
  assert_param(IS_SDIO_DPSM(SDIO_DataInitStruct->SDIO_DPSM));

/*---------------------------- SDIO DTIMER Configuration ---------------------*/
  /* Set the SDIO Data TimeOut value */
  SDIO->DTIMER = SDIO_DataInitStruct->SDIO_DataTimeOut;
    
/*---------------------------- SDIO DLEN Configuration -----------------------*/
  /* Set the SDIO DataLength value */
  SDIO->DLEN = SDIO_DataInitStruct->SDIO_DataLength;
  
/*---------------------------- SDIO DCTRL Configuration ----------------------*/  
  /* Get the SDIO DCTRL value */
  tmpreg = SDIO->DCTRL;

  /* Clear DEN, DTMODE, DTDIR and DBCKSIZE bits */
  tmpreg &= DCTRL_CLEAR_MASK;
  /* Set DEN bit according to SDIO_DPSM value */
  /* Set DTMODE bit according to SDIO_TransferMode value */
  /* Set DTDIR bit according to SDIO_TransferDir value */
  /* Set DBCKSIZE bits according to SDIO_DataBlockSize value */
  tmpreg |= (u32)SDIO_DataInitStruct->SDIO_DataBlockSize | SDIO_DataInitStruct->SDIO_TransferDir
           | SDIO_DataInitStruct->SDIO_TransferMode | SDIO_DataInitStruct->SDIO_DPSM;
  
  /* Write to SDIO DCTRL */
  SDIO->DCTRL = tmpreg;
}

/*******************************************************************************
* Function Name  : SDIO_DataStructInit
* Description    : Fills each SDIO_DataInitStruct member with its default value.
* Input          : SDIO_DataInitStruct: pointer to an SDIO_DataInitTypeDef 
*                  structure which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_DataStructInit(SDIO_DataInitTypeDef* SDIO_DataInitStruct)
{
  /* SDIO_DataInitStruct members default value */
  SDIO_DataInitStruct->SDIO_DataTimeOut = 0xFFFFFFFF;
  SDIO_DataInitStruct->SDIO_DataLength = 0x00;
  SDIO_DataInitStruct->SDIO_DataBlockSize = SDIO_DataBlockSize_1b;
  SDIO_DataInitStruct->SDIO_TransferDir = SDIO_TransferDir_ToCard;
  SDIO_DataInitStruct->SDIO_TransferMode = SDIO_TransferMode_Block;  
  SDIO_DataInitStruct->SDIO_DPSM = SDIO_DPSM_Disable;
}

/*******************************************************************************
* Function Name  : SDIO_GetDataCounter
* Description    : Returns number of remaining data bytes to be transferred.
* Input          : None
* Output         : None
* Return         : Number of remaining data bytes to be transferred
*******************************************************************************/
u32 SDIO_GetDataCounter(void)
{ 
  return SDIO->DCOUNT;
}

/*******************************************************************************
* Function Name  : SDIO_ReadData
* Description    : Read one data word from Rx FIFO.
* Input          : None
* Output         : None
* Return         : Data received
*******************************************************************************/
u32 SDIO_ReadData(void)
{ 
  return SDIO->FIFO;
}

/*******************************************************************************
* Function Name  : SDIO_WriteData
* Description    : Write one data word to Tx FIFO.
* Input          : Data: 32-bit data word to write.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_WriteData(u32 Data)
{ 
  SDIO->FIFO = Data;
}

/*******************************************************************************
* Function Name  : SDIO_GetFIFOCount
* Description    : Returns the number of words left to be written to or read
*                  from FIFO.	
* Input          : None
* Output         : None
* Return         : Remaining number of words.
*******************************************************************************/
u32 SDIO_GetFIFOCount(void)
{ 
  return SDIO->FIFOCNT;
}

/*******************************************************************************
* Function Name  : SDIO_StartSDIOReadWait
* Description    : Starts the SD I/O Read Wait operation.	
* Input          : NewState: new state of the Start SDIO Read Wait operation. 
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_StartSDIOReadWait(FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) DCTRL_RWSTART_BB = (u32) NewState;
}

/*******************************************************************************
* Function Name  : SDIO_StopSDIOReadWait
* Description    : Stops the SD I/O Read Wait operation.	
* Input          : NewState: new state of the Stop SDIO Read Wait operation. 
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_StopSDIOReadWait(FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) DCTRL_RWSTOP_BB = (u32) NewState;
}

/*******************************************************************************
* Function Name  : SDIO_SetSDIOReadWaitMode
* Description    : Sets one of the two options of inserting read wait interval.	
* Input          : SDIOReadWaitMode: SD I/O Read Wait operation mode.
*                  This parametre can be:
*                    - SDIO_ReadWaitMode_CLK: Read Wait control by stopping SDIOCLK
*                    - SDIO_ReadWaitMode_DATA2: Read Wait control using SDIO_DATA2
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_SetSDIOReadWaitMode(u32 SDIO_ReadWaitMode)
{
  /* Check the parameters */
  assert_param(IS_SDIO_READWAIT_MODE(SDIO_ReadWaitMode));
  
  *(vu32 *) DCTRL_RWMOD_BB = SDIO_ReadWaitMode;
}

/*******************************************************************************
* Function Name  : SDIO_SetSDIOOperation
* Description    : Enables or disables the SD I/O Mode Operation.	
* Input          : NewState: new state of SDIO specific operation. 
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_SetSDIOOperation(FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) DCTRL_SDIOEN_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : SDIO_SendSDIOSuspendCmd
* Description    : Enables or disables the SD I/O Mode suspend command sending.
* Input          : NewState: new state of the SD I/O Mode suspend command.
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_SendSDIOSuspendCmd(FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) CMD_SDIOSUSPEND_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : SDIO_CommandCompletionCmd
* Description    : Enables or disables the command completion signal.
* Input          : NewState: new state of command completion signal. 
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_CommandCompletionCmd(FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) CMD_ENCMDCOMPL_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : SDIO_CEATAITCmd
* Description    : Enables or disables the CE-ATA interrupt.
* Input          : NewState: new state of CE-ATA interrupt. 
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_CEATAITCmd(FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) CMD_NIEN_BB = (u32)((~((u32)NewState)) & ((u32)0x1));
}

/*******************************************************************************
* Function Name  : SDIO_SendCEATACmd
* Description    : Sends CE-ATA command (CMD61).
* Input          : NewState: new state of CE-ATA command. 
*                  This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_SendCEATACmd(FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(vu32 *) CMD_ATACMD_BB = (u32)NewState;
}

/*******************************************************************************
* Function Name  : SDIO_GetFlagStatus
* Description    : Checks whether the specified SDIO flag is set or not.	
* Input          : SDIO_FLAG: specifies the flag to check. 
*                  This parameter can be one of the following values:
*                     - SDIO_FLAG_CCRCFAIL: Command response received (CRC check
*                                           failed)    
*                     - SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check 
*                                           failed)    
*                     - SDIO_FLAG_CTIMEOUT: Command response timeout    
*                     - SDIO_FLAG_DTIMEOUT: Data timeou   
*                     - SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error   
*                     - SDIO_FLAG_RXOVERR:  Received FIFO overrun error    
*                     - SDIO_FLAG_CMDREND:  Command response received (CRC check 
*                                           passed)    
*                     - SDIO_FLAG_CMDSENT:  Command sent (no response required)    
*                     - SDIO_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is
*                                           zero)    
*                     - SDIO_FLAG_STBITERR: Start bit not detected on all data 
*                                           signals in wide bus mode   
*                     - SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check 
*                                           passed)    
*                     - SDIO_FLAG_CMDACT:   Command transfer in progress     
*                     - SDIO_FLAG_TXACT:    Data transmit in progress      
*                     - SDIO_FLAG_RXACT:    Data receive in progress      
*                     - SDIO_FLAG_TXFIFOHE: Transmit FIFO Half Empty   
*                     - SDIO_FLAG_RXFIFOHF: Receive FIFO Half Full   
*                     - SDIO_FLAG_TXFIFOF:  Transmit FIFO full    
*                     - SDIO_FLAG_RXFIFOF:  Receive FIFO full     
*                     - SDIO_FLAG_TXFIFOE:  Transmit FIFO empty    
*                     - SDIO_FLAG_RXFIFOE:  Receive FIFO empty    
*                     - SDIO_FLAG_TXDAVL:   Data available in transmit FIFO     
*                     - SDIO_FLAG_RXDAVL:   Data available in receive FIFO     
*                     - SDIO_FLAG_SDIOIT:   SD I/O interrupt received     
*                     - SDIO_FLAG_CEATAEND: CE-ATA command completion signal 
*                                           received for CMD61    
* Output         : None
* Return         : The new state of SDIO_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus SDIO_GetFlagStatus(u32 SDIO_FLAG)
{ 
  FlagStatus bitstatus = RESET;
  
  /* Check the parameters */
  assert_param(IS_SDIO_FLAG(SDIO_FLAG));
  
  if ((SDIO->STA & SDIO_FLAG) != (u32)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : SDIO_ClearFlag
* Description    : Clears the SDIO's pending flags.	
* Input          : SDIO_FLAG: specifies the flag to clear.  
*                  This parameter can be one or a combination of the following
*                  values:
*                     - SDIO_FLAG_CCRCFAIL: Command response received (CRC check
*                                           failed)    
*                     - SDIO_FLAG_DCRCFAIL: Data block sent/received (CRC check 
*                                           failed)    
*                     - SDIO_FLAG_CTIMEOUT: Command response timeout    
*                     - SDIO_FLAG_DTIMEOUT: Data timeou   
*                     - SDIO_FLAG_TXUNDERR: Transmit FIFO underrun error   
*                     - SDIO_FLAG_RXOVERR:  Received FIFO overrun error    
*                     - SDIO_FLAG_CMDREND:  Command response received (CRC check 
*                                           passed)    
*                     - SDIO_FLAG_CMDSENT:  Command sent (no response required)    
*                     - SDIO_FLAG_DATAEND:  Data end (data counter, SDIDCOUNT, is
*                                           zero)    
*                     - SDIO_FLAG_STBITERR: Start bit not detected on all data 
*                                           signals in wide bus mode   
*                     - SDIO_FLAG_DBCKEND:  Data block sent/received (CRC check 
*                                           passed)         
*                     - SDIO_FLAG_SDIOIT:   SD I/O interrupt received     
*                     - SDIO_FLAG_CEATAEND: CE-ATA command completion signal 
*                                           received for CMD61    
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_ClearFlag(u32 SDIO_FLAG)
{ 
  /* Check the parameters */
  assert_param(IS_SDIO_CLEAR_FLAG(SDIO_FLAG));
   
  SDIO->ICR = SDIO_FLAG;
}

/*******************************************************************************
* Function Name  : SDIO_GetITStatus
* Description    : Checks whether the specified SDIO interrupt has occurred or not.	
* Input          : SDIO_IT: specifies the SDIO interrupt source to check. 
*                  This parameter can be one of the following values:
*                      - SDIO_IT_CCRCFAIL: Command response received (CRC check
*                                          failed) interrupt    
*                      - SDIO_IT_DCRCFAIL: Data block sent/received (CRC check 
*                                          failed) interrupt    
*                      - SDIO_IT_CTIMEOUT: Command response timeout interrupt    
*                      - SDIO_IT_DTIMEOUT: Data timeout interrupt    
*                      - SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt    
*                      - SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt     
*                      - SDIO_IT_CMDREND:  Command response received (CRC check 
*                                          passed) interrupt     
*                      - SDIO_IT_CMDSENT:  Command sent (no response required) 
*                                          interrupt     
*                      - SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is 
*                                          zero) interrupt     
*                      - SDIO_IT_STBITERR: Start bit not detected on all data 
*                                          signals in wide bus mode interrupt    
*                      - SDIO_IT_DBCKEND:  Data block sent/received (CRC check 
*                                          passed) interrupt    
*                      - SDIO_IT_CMDACT:   Command transfer in progress interrupt     
*                      - SDIO_IT_TXACT:    Data transmit in progress interrupt       
*                      - SDIO_IT_RXACT:    Data receive in progress interrupt      
*                      - SDIO_IT_TXFIFOHE: Transmit FIFO Half Empty interrupt    
*                      - SDIO_IT_RXFIFOHF: Receive FIFO Half Full interrupt   
*                      - SDIO_IT_TXFIFOF:  Transmit FIFO full interrupt     
*                      - SDIO_IT_RXFIFOF:  Receive FIFO full interrupt     
*                      - SDIO_IT_TXFIFOE:  Transmit FIFO empty interrupt      
*                      - SDIO_IT_RXFIFOE:  Receive FIFO empty interrupt     
*                      - SDIO_IT_TXDAVL:   Data available in transmit FIFO interrupt      
*                      - SDIO_IT_RXDAVL:   Data available in receive FIFO interrupt      
*                      - SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt      
*                      - SDIO_IT_CEATAEND: CE-ATA command completion signal 
*                                          received for CMD61 interrupt
* Output         : None
* Return         : The new state of SDIO_IT (SET or RESET).
*******************************************************************************/
ITStatus SDIO_GetITStatus(u32 SDIO_IT)
{ 
  ITStatus bitstatus = RESET;
  
  /* Check the parameters */
  assert_param(IS_SDIO_GET_IT(SDIO_IT));

  if ((SDIO->STA & SDIO_IT) != (u32)RESET)  
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : SDIO_ClearITPendingBit
* Description    : Clears the SDIO’s interrupt pending bits.	
* Input          : SDIO_IT: specifies the interrupt pending bit to clear. 
*                   This parameter can be one or a combination of the following
*                   values:
*                      - SDIO_IT_CCRCFAIL: Command response received (CRC check
*                                          failed) interrupt    
*                      - SDIO_IT_DCRCFAIL: Data block sent/received (CRC check 
*                                          failed) interrupt    
*                      - SDIO_IT_CTIMEOUT: Command response timeout interrupt    
*                      - SDIO_IT_DTIMEOUT: Data timeout interrupt    
*                      - SDIO_IT_TXUNDERR: Transmit FIFO underrun error interrupt    
*                      - SDIO_IT_RXOVERR:  Received FIFO overrun error interrupt     
*                      - SDIO_IT_CMDREND:  Command response received (CRC check 
*                                          passed) interrupt     
*                      - SDIO_IT_CMDSENT:  Command sent (no response required) 
*                                          interrupt     
*                      - SDIO_IT_DATAEND:  Data end (data counter, SDIDCOUNT, is 
*                                          zero) interrupt     
*                      - SDIO_IT_STBITERR: Start bit not detected on all data 
*                                          signals in wide bus mode interrupt          
*                      - SDIO_IT_SDIOIT:   SD I/O interrupt received interrupt      
*                      - SDIO_IT_CEATAEND: CE-ATA command completion signal 
*                                          received for CMD61 
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_ClearITPendingBit(u32 SDIO_IT)
{ 
  /* Check the parameters */
  assert_param(IS_SDIO_CLEAR_IT(SDIO_IT));
   
  SDIO->ICR = SDIO_IT;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_spi.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the SPI firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* SPI SPE mask */
#define CR1_SPE_Set          ((u16)0x0040)
#define CR1_SPE_Reset        ((u16)0xFFBF)

/* I2S I2SE mask */
#define I2SCFGR_I2SE_Set     ((u16)0x0400)
#define I2SCFGR_I2SE_Reset   ((u16)0xFBFF)

/* SPI CRCNext mask */
#define CR1_CRCNext_Set      ((u16)0x1000)

/* SPI CRCEN mask */
#define CR1_CRCEN_Set        ((u16)0x2000)
#define CR1_CRCEN_Reset      ((u16)0xDFFF)

/* SPI SSOE mask */
#define CR2_SSOE_Set         ((u16)0x0004)
#define CR2_SSOE_Reset       ((u16)0xFFFB)

/* SPI registers Masks */
#define CR1_CLEAR_Mask       ((u16)0x3040)
#define I2SCFGR_CLEAR_Mask   ((u16)0xF040)

/* SPI or I2S mode selection masks */
#define SPI_Mode_Select      ((u16)0xF7FF)
#define I2S_Mode_Select      ((u16)0x0800) 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : SPI_I2S_DeInit
* Description    : Deinitializes the SPIx peripheral registers to their default
*                  reset values (Affects also the I2Ss).
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_I2S_DeInit(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  
  switch (*(u32*)&SPIx)
  {
    case SPI1_BASE:
      /* Enable SPI1 reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);
      /* Release SPI1 from reset state */
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE);
      break;

    case SPI2_BASE:
      /* Enable SPI2 reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);
      /* Release SPI2 from reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, DISABLE);
      break;

    case SPI3_BASE:
      /* Enable SPI3 reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, ENABLE);
      /* Release SPI3 from reset state */
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3, DISABLE);
      break;

    default:
      break;
  }
}

/*******************************************************************************
* Function Name  : SPI_Init
* Description    : Initializes the SPIx peripheral according to the specified 
*                  parameters in the SPI_InitStruct.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
*                  - SPI_InitStruct: pointer to a SPI_InitTypeDef structure that
*                    contains the configuration information for the specified
*                    SPI peripheral.
* Output         : None
* Return         : None
******************************************************************************/
void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct)
{
  u16 tmpreg = 0;
  
  /* check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));   
  
  /* Check the SPI parameters */
  assert_param(IS_SPI_DIRECTION_MODE(SPI_InitStruct->SPI_Direction));
  assert_param(IS_SPI_MODE(SPI_InitStruct->SPI_Mode));
  assert_param(IS_SPI_DATASIZE(SPI_InitStruct->SPI_DataSize));
  assert_param(IS_SPI_CPOL(SPI_InitStruct->SPI_CPOL));
  assert_param(IS_SPI_CPHA(SPI_InitStruct->SPI_CPHA));
  assert_param(IS_SPI_NSS(SPI_InitStruct->SPI_NSS));
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_InitStruct->SPI_BaudRatePrescaler));
  assert_param(IS_SPI_FIRST_BIT(SPI_InitStruct->SPI_FirstBit));
  assert_param(IS_SPI_CRC_POLYNOMIAL(SPI_InitStruct->SPI_CRCPolynomial));

/*---------------------------- SPIx CR1 Configuration ------------------------*/
  /* Get the SPIx CR1 value */
  tmpreg = SPIx->CR1;
  /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, MSTR, CPOL and CPHA bits */
  tmpreg &= CR1_CLEAR_Mask;
  /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
     master/salve mode, CPOL and CPHA */
  /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
  /* Set SSM, SSI and MSTR bits according to SPI_Mode and SPI_NSS values */
  /* Set LSBFirst bit according to SPI_FirstBit value */
  /* Set BR bits according to SPI_BaudRatePrescaler value */
  /* Set CPOL bit according to SPI_CPOL value */
  /* Set CPHA bit according to SPI_CPHA value */
  tmpreg |= (u16)((u32)SPI_InitStruct->SPI_Direction | SPI_InitStruct->SPI_Mode |
                  SPI_InitStruct->SPI_DataSize | SPI_InitStruct->SPI_CPOL |  
                  SPI_InitStruct->SPI_CPHA | SPI_InitStruct->SPI_NSS |  
                  SPI_InitStruct->SPI_BaudRatePrescaler | SPI_InitStruct->SPI_FirstBit);
  /* Write to SPIx CR1 */
  SPIx->CR1 = tmpreg;
  
  /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
  SPIx->I2SCFGR &= SPI_Mode_Select;		

/*---------------------------- SPIx CRCPOLY Configuration --------------------*/
  /* Write to SPIx CRCPOLY */
  SPIx->CRCPR = SPI_InitStruct->SPI_CRCPolynomial;
}

/*******************************************************************************
* Function Name  : I2S_Init
* Description    : Initializes the SPIx peripheral according to the specified 
*                  parameters in the I2S_InitStruct.
* Input          : - SPIx: where x can be  2 or 3 to select the SPI peripheral
*                     (configured in I2S mode).
*                  - I2S_InitStruct: pointer to an I2S_InitTypeDef structure that
*                    contains the configuration information for the specified
*                    SPI peripheral configured in I2S mode.
* Output         : None
* Return         : None
******************************************************************************/
void I2S_Init(SPI_TypeDef* SPIx, I2S_InitTypeDef* I2S_InitStruct)
{
  u16 tmpreg = 0, i2sdiv = 2, i2sodd = 0, packetlength = 1;
  u32 tmp = 0;
  RCC_ClocksTypeDef RCC_Clocks;
   
  /* Check the I2S parameters */
  assert_param(IS_SPI_23_PERIPH(SPIx));
  assert_param(IS_I2S_MODE(I2S_InitStruct->I2S_Mode));
  assert_param(IS_I2S_STANDARD(I2S_InitStruct->I2S_Standard));
  assert_param(IS_I2S_DATA_FORMAT(I2S_InitStruct->I2S_DataFormat));
  assert_param(IS_I2S_MCLK_OUTPUT(I2S_InitStruct->I2S_MCLKOutput));
  assert_param(IS_I2S_AUDIO_FREQ(I2S_InitStruct->I2S_AudioFreq));
  assert_param(IS_I2S_CPOL(I2S_InitStruct->I2S_CPOL));  

/*----------------------- SPIx I2SCFGR & I2SPR Configuration -----------------*/

  /* Clear I2SMOD, I2SE, I2SCFG, PCMSYNC, I2SSTD, CKPOL, DATLEN and CHLEN bits */
  SPIx->I2SCFGR &= I2SCFGR_CLEAR_Mask; 
  SPIx->I2SPR = 0x0002;
  
  /* Get the I2SCFGR register value */
  tmpreg = SPIx->I2SCFGR;
  
  /* If the default value has to be written, reinitialize i2sdiv and i2sodd*/
  if(I2S_InitStruct->I2S_AudioFreq == I2S_AudioFreq_Default)
  {
    i2sodd = (u16)0;
    i2sdiv = (u16)2;   
  }
  /* If the requested audio frequency is not the default, compute the prescaler */
  else
  {
    /* Check the frame length (For the Prescaler computing) */
    if(I2S_InitStruct->I2S_DataFormat == I2S_DataFormat_16b)
    {
      /* Packet length is 16 bits */
      packetlength = 1;
    }
    else
    {
      /* Packet length is 32 bits */
      packetlength = 2;
    }
    /* Get System Clock frequency */
    RCC_GetClocksFreq(&RCC_Clocks);
    
    /* Compute the Real divider depending on the MCLK output state with a flaoting point */
    if(I2S_InitStruct->I2S_MCLKOutput == I2S_MCLKOutput_Enable)
    {
      /* MCLK output is enabled */
      tmp = (u16)(((10 * RCC_Clocks.SYSCLK_Frequency) / (256 * I2S_InitStruct->I2S_AudioFreq)) + 5);
    }
    else
    {
      /* MCLK output is disabled */
      tmp = (u16)(((10 * RCC_Clocks.SYSCLK_Frequency) / (32 * packetlength * I2S_InitStruct->I2S_AudioFreq)) + 5);
    }
    
    /* Remove the flaoting point */
    tmp = tmp/10;  
      
    /* Check the parity of the divider */
    i2sodd = (u16)(tmp & (u16)0x0001);
   
    /* Compute the i2sdiv prescaler */
    i2sdiv = (u16)((tmp - i2sodd) / 2);
   
    /* Get the Mask for the Odd bit (SPI_I2SPR[8]) register */
    i2sodd = (u16) (i2sodd << 8);
  }
  
  /* Test if the divider is 1 or 0 */
  if ((i2sdiv < 2) || (i2sdiv > 0xFF))
  {
    /* Set the default values */
    i2sdiv = 2;
    i2sodd = 0;
  }

  /* Write to SPIx I2SPR register the computed value */
  SPIx->I2SPR = (u16)(i2sdiv | i2sodd | I2S_InitStruct->I2S_MCLKOutput);  
 
  /* Configure the I2S with the SPI_InitStruct values */
  tmpreg |= (u16)(I2S_Mode_Select | I2S_InitStruct->I2S_Mode | \
                  I2S_InitStruct->I2S_Standard | I2S_InitStruct->I2S_DataFormat | \
                  I2S_InitStruct->I2S_CPOL);
 
  /* Write to SPIx I2SCFGR */  
  SPIx->I2SCFGR = tmpreg;                                    
}

/*******************************************************************************
* Function Name  : SPI_StructInit
* Description    : Fills each SPI_InitStruct member with its default value.
* Input          : - SPI_InitStruct : pointer to a SPI_InitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct)
{
/*--------------- Reset SPI init structure parameters values -----------------*/
  /* Initialize the SPI_Direction member */
  SPI_InitStruct->SPI_Direction = SPI_Direction_2Lines_FullDuplex;

  /* initialize the SPI_Mode member */
  SPI_InitStruct->SPI_Mode = SPI_Mode_Slave;

  /* initialize the SPI_DataSize member */
  SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;

  /* Initialize the SPI_CPOL member */
  SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;

  /* Initialize the SPI_CPHA member */
  SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;

  /* Initialize the SPI_NSS member */
  SPI_InitStruct->SPI_NSS = SPI_NSS_Hard;

  /* Initialize the SPI_BaudRatePrescaler member */
  SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;

  /* Initialize the SPI_FirstBit member */
  SPI_InitStruct->SPI_FirstBit = SPI_FirstBit_MSB;

  /* Initialize the SPI_CRCPolynomial member */
  SPI_InitStruct->SPI_CRCPolynomial = 7;
}

/*******************************************************************************
* Function Name  : I2S_StructInit
* Description    : Fills each I2S_InitStruct member with its default value.
* Input          : - I2S_InitStruct : pointer to a I2S_InitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void I2S_StructInit(I2S_InitTypeDef* I2S_InitStruct)
{
/*--------------- Reset I2S init structure parameters values -----------------*/
  /* Initialize the I2S_Mode member */
  I2S_InitStruct->I2S_Mode = I2S_Mode_SlaveTx;
  
  /* Initialize the I2S_Standard member */
  I2S_InitStruct->I2S_Standard = I2S_Standard_Phillips;
  
  /* Initialize the I2S_DataFormat member */
  I2S_InitStruct->I2S_DataFormat = I2S_DataFormat_16b;
  
  /* Initialize the I2S_MCLKOutput member */
  I2S_InitStruct->I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  
  /* Initialize the I2S_AudioFreq member */
  I2S_InitStruct->I2S_AudioFreq = I2S_AudioFreq_Default;
  
  /* Initialize the I2S_CPOL member */
  I2S_InitStruct->I2S_CPOL = I2S_CPOL_Low;
}

/*******************************************************************************
* Function Name  : SPI_Cmd
* Description    : Enables or disables the specified SPI peripheral.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
*                  - NewState: new state of the SPIx peripheral. 
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected SPI peripheral */
    SPIx->CR1 |= CR1_SPE_Set;
  }
  else
  {
    /* Disable the selected SPI peripheral */
    SPIx->CR1 &= CR1_SPE_Reset;
  }
}

/*******************************************************************************
* Function Name  : I2S_Cmd
* Description    : Enables or disables the specified SPI peripheral (in I2S mode).
* Input          : - SPIx: where x can be 2 or 3 to select the SPI peripheral.
*                  - NewState: new state of the SPIx peripheral. 
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void I2S_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_SPI_23_PERIPH(SPIx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected SPI peripheral (in I2S mode) */
    SPIx->I2SCFGR |= I2SCFGR_I2SE_Set;
  }
  else
  {
    /* Disable the selected SPI peripheral (in I2S mode) */
    SPIx->I2SCFGR &= I2SCFGR_I2SE_Reset;
  }
}

/*******************************************************************************
* Function Name  : SPI_I2S_ITConfig
* Description    : Enables or disables the specified SPI/I2S interrupts.
* Input          : - SPIx: where x can be :
*                         - 1, 2 or 3 in SPI mode 
*                         - 2 or 3 in I2S mode
*                  - SPI_I2S_IT: specifies the SPI/I2S interrupt source to be 
*                    enabled or disabled. 
*                    This parameter can be one of the following values:
*                       - SPI_I2S_IT_TXE: Tx buffer empty interrupt mask
*                       - SPI_I2S_IT_RXNE: Rx buffer not empty interrupt mask
*                       - SPI_I2S_IT_ERR: Error interrupt mask
*                  - NewState: new state of the specified SPI/I2S interrupt.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_I2S_ITConfig(SPI_TypeDef* SPIx, u8 SPI_I2S_IT, FunctionalState NewState)
{
  u16 itpos = 0, itmask = 0 ;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_SPI_I2S_CONFIG_IT(SPI_I2S_IT));

  /* Get the SPI/I2S IT index */
  itpos = SPI_I2S_IT >> 4;
  /* Set the IT mask */
  itmask = (u16)((u16)1 << itpos);

  if (NewState != DISABLE)
  {
    /* Enable the selected SPI/I2S interrupt */
    SPIx->CR2 |= itmask;
  }
  else
  {
    /* Disable the selected SPI/I2S interrupt */
    SPIx->CR2 &= (u16)~itmask;
  }
}

/*******************************************************************************
* Function Name  : SPI_I2S_DMACmd
* Description    : Enables or disables the SPIx/I2Sx DMA interface.
* Input          : - SPIx: where x can be :
*                         - 1, 2 or 3 in SPI mode 
*                         - 2 or 3 in I2S mode
*                  - SPI_I2S_DMAReq: specifies the SPI/I2S DMA transfer request 
*                    to be enabled or disabled. 
*                    This parameter can be any combination of the following values:
*                       - SPI_I2S_DMAReq_Tx: Tx buffer DMA transfer request
*                       - SPI_I2S_DMAReq_Rx: Rx buffer DMA transfer request
*                  - NewState: new state of the selected SPI/I2S DMA transfer 
*                    request.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_I2S_DMACmd(SPI_TypeDef* SPIx, u16 SPI_I2S_DMAReq, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_SPI_I2S_DMAREQ(SPI_I2S_DMAReq));

  if (NewState != DISABLE)
  {
    /* Enable the selected SPI/I2S DMA requests */
    SPIx->CR2 |= SPI_I2S_DMAReq;
  }
  else
  {
    /* Disable the selected SPI/I2S DMA requests */
    SPIx->CR2 &= (u16)~SPI_I2S_DMAReq;
  }
}

/*******************************************************************************
* Function Name  : SPI_I2S_SendData
* Description    : Transmits a Data through the SPIx/I2Sx peripheral.
* Input          : - SPIx: where x can be :
*                         - 1, 2 or 3 in SPI mode 
*                         - 2 or 3 in I2S mode
*                  - Data : Data to be transmitted..
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_I2S_SendData(SPI_TypeDef* SPIx, u16 Data)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  
  /* Write in the DR register the data to be sent */
  SPIx->DR = Data;
}

/*******************************************************************************
* Function Name  : SPI_I2S_ReceiveData
* Description    : Returns the most recent received data by the SPIx/I2Sx peripheral. 
* Input          : - SPIx: where x can be :
*                         - 1, 2 or 3 in SPI mode 
*                         - 2 or 3 in I2S mode
* Output         : None
* Return         : The value of the received data.
*******************************************************************************/
u16 SPI_I2S_ReceiveData(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  
  /* Return the data in the DR register */
  return SPIx->DR;
}

/*******************************************************************************
* Function Name  : SPI_NSSInternalSoftwareConfig
* Description    : Configures internally by software the NSS pin for the selected 
*                  SPI.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
*                  - SPI_NSSInternalSoft: specifies the SPI NSS internal state.
*                    This parameter can be one of the following values:
*                       - SPI_NSSInternalSoft_Set: Set NSS pin internally
*                       - SPI_NSSInternalSoft_Reset: Reset NSS pin internally
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_NSSInternalSoftwareConfig(SPI_TypeDef* SPIx, u16 SPI_NSSInternalSoft)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_NSS_INTERNAL(SPI_NSSInternalSoft));

  if (SPI_NSSInternalSoft != SPI_NSSInternalSoft_Reset)
  {
    /* Set NSS pin internally by software */
    SPIx->CR1 |= SPI_NSSInternalSoft_Set;
  }
  else
  {
    /* Reset NSS pin internally by software */
    SPIx->CR1 &= SPI_NSSInternalSoft_Reset;
  }
}

/*******************************************************************************
* Function Name  : SPI_SSOutputCmd
* Description    : Enables or disables the SS output for the selected SPI.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
*                  - NewState: new state of the SPIx SS output. 
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_SSOutputCmd(SPI_TypeDef* SPIx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected SPI SS output */
    SPIx->CR2 |= CR2_SSOE_Set;
  }
  else
  {
    /* Disable the selected SPI SS output */
    SPIx->CR2 &= CR2_SSOE_Reset;
  }
}

/*******************************************************************************
* Function Name  : SPI_DataSizeConfig
* Description    : Configures the data size for the selected SPI.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
*                  - SPI_DataSize: specifies the SPI data size.
*                    This parameter can be one of the following values:
*                       - SPI_DataSize_16b: Set data frame format to 16bit
*                       - SPI_DataSize_8b: Set data frame format to 8bit
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_DataSizeConfig(SPI_TypeDef* SPIx, u16 SPI_DataSize)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_DATASIZE(SPI_DataSize));

  /* Clear DFF bit */
  SPIx->CR1 &= (u16)~SPI_DataSize_16b;
  /* Set new DFF bit value */
  SPIx->CR1 |= SPI_DataSize;
}

/*******************************************************************************
* Function Name  : SPI_TransmitCRC
* Description    : Transmit the SPIx CRC value.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_TransmitCRC(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  
  /* Enable the selected SPI CRC transmission */
  SPIx->CR1 |= CR1_CRCNext_Set;
}

/*******************************************************************************
* Function Name  : SPI_CalculateCRC
* Description    : Enables or disables the CRC value calculation of the
*                  transfered bytes.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
*                  - NewState: new state of the SPIx CRC value calculation.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_CalculateCRC(SPI_TypeDef* SPIx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected SPI CRC calculation */
    SPIx->CR1 |= CR1_CRCEN_Set;
  }
  else
  {
    /* Disable the selected SPI CRC calculation */
    SPIx->CR1 &= CR1_CRCEN_Reset;
  }
}

/*******************************************************************************
* Function Name  : SPI_GetCRC
* Description    : Returns the transmit or the receive CRC register value for
*                  the specified SPI.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
*                  - SPI_CRC: specifies the CRC register to be read.
*                    This parameter can be one of the following values:
*                       - SPI_CRC_Tx: Selects Tx CRC register
*                       - SPI_CRC_Rx: Selects Rx CRC register
* Output         : None
* Return         : The selected CRC register value..
*******************************************************************************/
u16 SPI_GetCRC(SPI_TypeDef* SPIx, u8 SPI_CRC)
{
  u16 crcreg = 0;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_CRC(SPI_CRC));

  if (SPI_CRC != SPI_CRC_Rx)
  {
    /* Get the Tx CRC register */
    crcreg = SPIx->TXCRCR;
  }
  else
  {
    /* Get the Rx CRC register */
    crcreg = SPIx->RXCRCR;
  }

  /* Return the selected CRC register */
  return crcreg;
}

/*******************************************************************************
* Function Name  : SPI_GetCRCPolynomial
* Description    : Returns the CRC Polynomial register value for the specified SPI.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
* Output         : None
* Return         : The CRC Polynomial register value.
*******************************************************************************/
u16 SPI_GetCRCPolynomial(SPI_TypeDef* SPIx)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  
  /* Return the CRC polynomial register */
  return SPIx->CRCPR;
}

/*******************************************************************************
* Function Name  : SPI_BiDirectionalLineConfig
* Description    : Selects the data transfer direction in bi-directional mode
*                  for the specified SPI.
* Input          : - SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
*                  - SPI_Direction: specifies the data transfer direction in
*                    bi-directional mode. 
*                    This parameter can be one of the following values:
*                       - SPI_Direction_Tx: Selects Tx transmission direction
*                       - SPI_Direction_Rx: Selects Rx receive direction
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, u16 SPI_Direction)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_DIRECTION(SPI_Direction));

  if (SPI_Direction == SPI_Direction_Tx)
  {
    /* Set the Tx only mode */
    SPIx->CR1 |= SPI_Direction_Tx;
  }
  else
  {
    /* Set the Rx only mode */
    SPIx->CR1 &= SPI_Direction_Rx;
  }
}

/*******************************************************************************
* Function Name  : SPI_I2S_GetFlagStatus
* Description    : Checks whether the specified SPI/I2S flag is set or not.
* Input          : - SPIx: where x can be :
*                         - 1, 2 or 3 in SPI mode 
*                         - 2 or 3 in I2S mode
*                  - SPI_I2S_FLAG: specifies the SPI/I2S flag to check. 
*                    This parameter can be one of the following values:
*                       - SPI_I2S_FLAG_TXE: Transmit buffer empty flag.
*                       - SPI_I2S_FLAG_RXNE: Receive buffer not empty flag.
*                       - SPI_I2S_FLAG_BSY: Busy flag.
*                       - SPI_I2S_FLAG_OVR: Overrun flag.
*                       - SPI_FLAG_MODF: Mode Fault flag.
*                       - SPI_FLAG_CRCERR: CRC Error flag.
*                       - I2S_FLAG_UDR: Underrun Error flag.
*                       - I2S_FLAG_CHSIDE: Channel Side flag.
* Output         : None
* Return         : The new state of SPI_I2S_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus SPI_I2S_GetFlagStatus(SPI_TypeDef* SPIx, u16 SPI_I2S_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_I2S_GET_FLAG(SPI_I2S_FLAG));

  /* Check the status of the specified SPI/I2S flag */
  if ((SPIx->SR & SPI_I2S_FLAG) != (u16)RESET)
  {
    /* SPI_I2S_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* SPI_I2S_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the SPI_I2S_FLAG status */
  return  bitstatus;
}

/*******************************************************************************
* Function Name  : SPI_I2S_ClearFlag
* Description    : Clears the SPIx CRC Error (CRCERR) flag.
* Input          : - SPIx: where x can be :
*                         - 1, 2 or 3 in SPI mode 
*                  - SPI_I2S_FLAG: specifies the SPI flag to clear. 
*                    This function clears only CRCERR flag.                                           
*                  Notes:
*                       - OVR (OverRun error) flag is cleared by software 
*                         sequence: a read operation to SPI_DR register 
*                         (SPI_I2S_ReceiveData()) followed by a read operation 
*                         to SPI_SR register (SPI_I2S_GetFlagStatus()).                           
*                       - UDR (UnderRun error) flag is cleared by a read 
*                         operation to SPI_SR register (SPI_I2S_GetFlagStatus()).                             
*                       - MODF (Mode Fault) flag is cleared by software sequence: 
*                         a read/write operation to SPI_SR register 
*                         (SPI_I2S_GetFlagStatus()) followed by a write 
*                         operation to SPI_CR1 register (SPI_Cmd() to enable 
*                         the SPI).   
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_I2S_ClearFlag(SPI_TypeDef* SPIx, u16 SPI_I2S_FLAG)
{
  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_I2S_CLEAR_FLAG(SPI_I2S_FLAG));
    
    /* Clear the selected SPI CRC Error (CRCERR) flag */
    SPIx->SR = (u16)~SPI_I2S_FLAG;
}

/*******************************************************************************
* Function Name  : SPI_I2S_GetITStatus
* Description    : Checks whether the specified SPI/I2S interrupt has occurred or not.
* Input          : - SPIx: where x can be :
*                         - 1, 2 or 3 in SPI mode 
*                         - 2 or 3 in I2S mode
*                  - SPI_I2S_IT: specifies the SPI/I2S interrupt source to check. 
*                    This parameter can be one of the following values:
*                       - SPI_I2S_IT_TXE: Transmit buffer empty interrupt.
*                       - SPI_I2S_IT_RXNE: Receive buffer not empty interrupt.
*                       - SPI_I2S_IT_OVR: Overrun interrupt.
*                       - SPI_IT_MODF: Mode Fault interrupt.
*                       - SPI_IT_CRCERR: CRC Error interrupt.
*                       - I2S_IT_UDR: Underrun Error interrupt.
* Output         : None
* Return         : The new state of SPI_I2S_IT (SET or RESET).
*******************************************************************************/
ITStatus SPI_I2S_GetITStatus(SPI_TypeDef* SPIx, u8 SPI_I2S_IT)
{
  ITStatus bitstatus = RESET;
  u16 itpos = 0, itmask = 0, enablestatus = 0;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_I2S_GET_IT(SPI_I2S_IT));

  /* Get the SPI/I2S IT index */
  itpos = (u16)((u16)0x01 << (SPI_I2S_IT & (u8)0x0F));

  /* Get the SPI/I2S IT mask */
  itmask = SPI_I2S_IT >> 4;
  /* Set the IT mask */
  itmask = (u16)((u16)0x01 << itmask);
  /* Get the SPI_I2S_IT enable bit status */
  enablestatus = (SPIx->CR2 & itmask) ;

  /* Check the status of the specified SPI/I2S interrupt */
  if (((SPIx->SR & itpos) != (u16)RESET) && enablestatus)
  {
    /* SPI_I2S_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* SPI_I2S_IT is reset */
    bitstatus = RESET;
  }
  /* Return the SPI_I2S_IT status */
  return bitstatus;
}

/*******************************************************************************
* Function Name  : SPI_I2S_ClearITPendingBit
* Description    : Clears the SPIx CRC Error (CRCERR) interrupt pending bit.
* Input          : - SPIx: where x can be :
*                         - 1, 2 or 3 in SPI mode 
*                  - SPI_I2S_IT: specifies the SPI interrupt pending bit to clear.
*                    This function clears only CRCERR intetrrupt pending bit.   
*                  Notes:
*                       - OVR (OverRun Error) interrupt pending bit is cleared 
*                         by software sequence: a read operation to SPI_DR 
*                         register (SPI_I2S_ReceiveData()) followed by a read 
*                         operation to SPI_SR register (SPI_I2S_GetITStatus()).
*                       - UDR (UnderRun Error) interrupt pending bit is cleared 
*                         by a read operation to SPI_SR register 
*                         (SPI_I2S_GetITStatus()).                           
*                       - MODF (Mode Fault) interrupt pending bit is cleared by 
*                         software sequence: a read/write operation to SPI_SR 
*                         register (SPI_I2S_GetITStatus()) followed by a write 
*                         operation to SPI_CR1 register (SPI_Cmd() to enable the 
*                         SPI).   
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_I2S_ClearITPendingBit(SPI_TypeDef* SPIx, u8 SPI_I2S_IT)
{
  u16 itpos = 0;

  /* Check the parameters */
  assert_param(IS_SPI_ALL_PERIPH(SPIx));
  assert_param(IS_SPI_I2S_CLEAR_IT(SPI_I2S_IT));

  /* Get the SPI IT index */
  itpos = (u16)((u16)0x01 << (SPI_I2S_IT & (u8)0x0F));
  /* Clear the selected SPI CRC Error (CRCERR) interrupt pending bit */
  SPIx->SR = (u16)~itpos;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_systick.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the SysTick firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_systick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ---------------------- SysTick registers bit mask -------------------- */
/* CTRL TICKINT Mask */
#define CTRL_TICKINT_Set      ((u32)0x00000002)
#define CTRL_TICKINT_Reset    ((u32)0xFFFFFFFD)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : SysTick_CLKSourceConfig
* Description    : Configures the SysTick clock source.
* Input          : - SysTick_CLKSource: specifies the SysTick clock source.
*                    This parameter can be one of the following values:
*                       - SysTick_CLKSource_HCLK_Div8: AHB clock divided by 8
*                         selected as SysTick clock source.
*                       - SysTick_CLKSource_HCLK: AHB clock selected as
*                         SysTick clock source.
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_CLKSourceConfig(u32 SysTick_CLKSource)
{
  /* Check the parameters */
  assert_param(IS_SYSTICK_CLK_SOURCE(SysTick_CLKSource));

  if (SysTick_CLKSource == SysTick_CLKSource_HCLK)
  {
    SysTick->CTRL |= SysTick_CLKSource_HCLK;
  }
  else
  {
    SysTick->CTRL &= SysTick_CLKSource_HCLK_Div8;
  }
}

/*******************************************************************************
* Function Name  : SysTick_SetReload
* Description    : Sets SysTick Reload value.
* Input          : - Reload: SysTick Reload new value.
*                    This parameter must be a number between 1 and 0xFFFFFF.
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_SetReload(u32 Reload)
{
  /* Check the parameters */
  assert_param(IS_SYSTICK_RELOAD(Reload));

  SysTick->LOAD = Reload;
}

/*******************************************************************************
* Function Name  : SysTick_CounterCmd
* Description    : Enables or disables the SysTick counter.
* Input          : - SysTick_Counter: new state of the SysTick counter.
*                    This parameter can be one of the following values:
*                       - SysTick_Counter_Disable: Disable counter
*                       - SysTick_Counter_Enable: Enable counter
*                       - SysTick_Counter_Clear: Clear counter value to 0
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_CounterCmd(u32 SysTick_Counter)
{
  /* Check the parameters */
  assert_param(IS_SYSTICK_COUNTER(SysTick_Counter));

  if (SysTick_Counter == SysTick_Counter_Enable)
  {
    SysTick->CTRL |= SysTick_Counter_Enable;
  }
  else if (SysTick_Counter == SysTick_Counter_Disable) 
  {
    SysTick->CTRL &= SysTick_Counter_Disable;
  }
  else /* SysTick_Counter == SysTick_Counter_Clear */
  {
    SysTick->VAL = SysTick_Counter_Clear;
  }    
}

/*******************************************************************************
* Function Name  : SysTick_ITConfig
* Description    : Enables or disables the SysTick Interrupt.
* Input          : - NewState: new state of the SysTick Interrupt.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_ITConfig(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    SysTick->CTRL |= CTRL_TICKINT_Set;
  }
  else
  {
    SysTick->CTRL &= CTRL_TICKINT_Reset;
  }
}

/*******************************************************************************
* Function Name  : SysTick_GetCounter
* Description    : Gets SysTick counter value.
* Input          : None
* Output         : None
* Return         : SysTick current value
*******************************************************************************/
u32 SysTick_GetCounter(void)
{
  return(SysTick->VAL);
}

/*******************************************************************************
* Function Name  : SysTick_GetFlagStatus
* Description    : Checks whether the specified SysTick flag is set or not.
* Input          : - SysTick_FLAG: specifies the flag to check.
*                    This parameter can be one of the following values:
*                       - SysTick_FLAG_COUNT
*                       - SysTick_FLAG_SKEW
*                       - SysTick_FLAG_NOREF
* Output         : None
* Return         : None
*******************************************************************************/
FlagStatus SysTick_GetFlagStatus(u8 SysTick_FLAG)
{
  u32 statusreg = 0, tmp = 0 ;
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_SYSTICK_FLAG(SysTick_FLAG));

  /* Get the SysTick register index */
  tmp = SysTick_FLAG >> 3;

  if (tmp == 2) /* The flag to check is in CTRL register */
  {
    statusreg = SysTick->CTRL;
  }
  else          /* The flag to check is in CALIB register */
  {
    statusreg = SysTick->CALIB;
  }

  if ((statusreg & ((u32)1 << SysTick_FLAG)) != (u32)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_tim.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the TIM firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ---------------------- TIM registers bit mask ------------------------ */
#define CR1_CEN_Set                 ((u16)0x0001)
#define CR1_CEN_Reset               ((u16)0x03FE)
#define CR1_UDIS_Set                ((u16)0x0002)
#define CR1_UDIS_Reset              ((u16)0x03FD)
#define CR1_URS_Set                 ((u16)0x0004)
#define CR1_URS_Reset               ((u16)0x03FB)
#define CR1_OPM_Reset               ((u16)0x03F7)
#define CR1_CounterMode_Mask        ((u16)0x038F)
#define CR1_ARPE_Set                ((u16)0x0080)
#define CR1_ARPE_Reset              ((u16)0x037F)
#define CR1_CKD_Mask                ((u16)0x00FF)

#define CR2_CCPC_Set                ((u16)0x0001)
#define CR2_CCPC_Reset              ((u16)0xFFFE)
#define CR2_CCUS_Set                ((u16)0x0004)
#define CR2_CCUS_Reset              ((u16)0xFFFB)
#define CR2_CCDS_Set                ((u16)0x0008)
#define CR2_CCDS_Reset              ((u16)0xFFF7)
#define CR2_MMS_Mask                ((u16)0xFF8F)
#define CR2_TI1S_Set                ((u16)0x0080)
#define CR2_TI1S_Reset              ((u16)0xFF7F)
#define CR2_OIS1_Reset              ((u16)0x7EFF)
#define CR2_OIS1N_Reset             ((u16)0x7DFF)
#define CR2_OIS2_Reset              ((u16)0x7BFF)
#define CR2_OIS2N_Reset             ((u16)0x77FF)
#define CR2_OIS3_Reset              ((u16)0x6FFF)
#define CR2_OIS3N_Reset             ((u16)0x5FFF)
#define CR2_OIS4_Reset              ((u16)0x3FFF)

#define SMCR_SMS_Mask               ((u16)0xFFF8)
#define SMCR_ETR_Mask               ((u16)0x00FF)
#define SMCR_TS_Mask                ((u16)0xFF8F)
#define SMCR_MSM_Reset              ((u16)0xFF7F)
#define SMCR_ECE_Set                ((u16)0x4000)

#define CCMR_CC13S_Mask             ((u16)0xFFFC)
#define CCMR_CC24S_Mask             ((u16)0xFCFF)
#define CCMR_TI13Direct_Set         ((u16)0x0001)
#define CCMR_TI24Direct_Set         ((u16)0x0100)
#define CCMR_OC13FE_Reset           ((u16)0xFFFB)
#define CCMR_OC24FE_Reset           ((u16)0xFBFF)
#define CCMR_OC13PE_Reset           ((u16)0xFFF7)
#define CCMR_OC24PE_Reset           ((u16)0xF7FF)
#define CCMR_OC13M_Mask             ((u16)0xFF8F)
#define CCMR_OC24M_Mask             ((u16)0x8FFF) 

#define CCMR_OC13CE_Reset           ((u16)0xFF7F)
#define CCMR_OC24CE_Reset           ((u16)0x7FFF)

#define CCMR_IC13PSC_Mask           ((u16)0xFFF3)
#define CCMR_IC24PSC_Mask           ((u16)0xF3FF)
#define CCMR_IC13F_Mask             ((u16)0xFF0F)
#define CCMR_IC24F_Mask             ((u16)0x0FFF)

#define CCMR_Offset                 ((u16)0x0018)
#define CCER_CCE_Set                ((u16)0x0001)
#define	CCER_CCNE_Set               ((u16)0x0004)

#define CCER_CC1P_Reset             ((u16)0xFFFD)
#define CCER_CC2P_Reset             ((u16)0xFFDF)
#define CCER_CC3P_Reset             ((u16)0xFDFF)
#define CCER_CC4P_Reset             ((u16)0xDFFF)

#define CCER_CC1NP_Reset            ((u16)0xFFF7)
#define CCER_CC2NP_Reset            ((u16)0xFF7F)
#define CCER_CC3NP_Reset            ((u16)0xF7FF)

#define CCER_CC1E_Set               ((u16)0x0001)
#define CCER_CC1E_Reset             ((u16)0xFFFE)

#define CCER_CC1NE_Reset            ((u16)0xFFFB)

#define CCER_CC2E_Set               ((u16)0x0010)
#define CCER_CC2E_Reset             ((u16)0xFFEF)

#define CCER_CC2NE_Reset            ((u16)0xFFBF)

#define CCER_CC3E_Set               ((u16)0x0100)
#define CCER_CC3E_Reset             ((u16)0xFEFF)

#define CCER_CC3NE_Reset            ((u16)0xFBFF)

#define CCER_CC4E_Set               ((u16)0x1000)
#define CCER_CC4E_Reset             ((u16)0xEFFF)

#define BDTR_MOE_Set                ((u16)0x8000)
#define BDTR_MOE_Reset              ((u16)0x7FFF)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void TI1_Config(TIM_TypeDef* TIMx, u16 TIM_ICPolarity, u16 TIM_ICSelection,
                       u16 TIM_ICFilter);
static void TI2_Config(TIM_TypeDef* TIMx, u16 TIM_ICPolarity, u16 TIM_ICSelection,
                       u16 TIM_ICFilter);
static void TI3_Config(TIM_TypeDef* TIMx, u16 TIM_ICPolarity, u16 TIM_ICSelection,
                       u16 TIM_ICFilter);
static void TI4_Config(TIM_TypeDef* TIMx, u16 TIM_ICPolarity, u16 TIM_ICSelection,
                       u16 TIM_ICFilter);
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : TIM_DeInit
* Description    : Deinitializes the TIMx peripheral registers to their default
*                  reset values.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_DeInit(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx)); 
 
  switch (*(u32*)&TIMx)
  {
    case TIM1_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, DISABLE);  
      break; 
      
    case TIM2_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM2, DISABLE);
      break;
 
    case TIM3_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);
      break;
 
    case TIM4_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM4, DISABLE);
      break;
      
    case TIM5_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM5, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM5, DISABLE);
      break;
      
    case TIM6_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, DISABLE);
      break;
      
    case TIM7_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM7, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM7, DISABLE);
      break;
      
    case TIM8_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM8, DISABLE);  
      break; 
      
    default:
      break;
  }
}

/*******************************************************************************
* Function Name  : TIM_TimeBaseInit
* Description    : Initializes the TIMx Time Base Unit peripheral according to 
*                  the specified parameters in the TIM_TimeBaseInitStruct.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_TimeBaseInitStruct: pointer to a TIM_TimeBaseInitTypeDef
*                   structure that contains the configuration information for
*                   the specified TIM peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx)); 
  assert_param(IS_TIM_COUNTER_MODE(TIM_TimeBaseInitStruct->TIM_CounterMode));
  assert_param(IS_TIM_CKD_DIV(TIM_TimeBaseInitStruct->TIM_ClockDivision));

  /* Select the Counter Mode and set the clock division */
  TIMx->CR1 &= CR1_CKD_Mask & CR1_CounterMode_Mask;
  TIMx->CR1 |= (u32)TIM_TimeBaseInitStruct->TIM_ClockDivision |
                TIM_TimeBaseInitStruct->TIM_CounterMode;
  /* Set the Autoreload value */
  TIMx->ARR = TIM_TimeBaseInitStruct->TIM_Period ;

  /* Set the Prescaler value */
  TIMx->PSC = TIM_TimeBaseInitStruct->TIM_Prescaler;

  /* Generate an update event to reload the Prescaler value immediatly */
  TIMx->EGR = TIM_PSCReloadMode_Immediate;
    
  if (((*(u32*)&TIMx) == TIM1_BASE) || ((*(u32*)&TIMx) == TIM8_BASE))  
  {
    /* Set the Repetition Counter value */
    TIMx->RCR = TIM_TimeBaseInitStruct->TIM_RepetitionCounter;
  }        
}

/*******************************************************************************
* Function Name  : TIM_OC1Init
* Description    : Initializes the TIMx Channel1 according to the specified
*                  parameters in the TIM_OCInitStruct.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure
*                    that contains the configuration information for the specified
*                    TIM peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  u16 tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
   
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx)); 
  assert_param(IS_TIM_OC_MODE(TIM_OCInitStruct->TIM_OCMode));
  assert_param(IS_TIM_OUTPUT_STATE(TIM_OCInitStruct->TIM_OutputState));
  assert_param(IS_TIM_OC_POLARITY(TIM_OCInitStruct->TIM_OCPolarity));   

  /* Disable the Channel 1: Reset the CC1E Bit */
  TIMx->CCER &= CCER_CC1E_Reset;
  
  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;

  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;
  
  /* Get the TIMx CCMR1 register value */
  tmpccmrx = TIMx->CCMR1;
    
  /* Reset the Output Compare Mode Bits */
  tmpccmrx &= CCMR_OC13M_Mask;
  
  /* Select the Output Compare Mode */
  tmpccmrx |= TIM_OCInitStruct->TIM_OCMode;
  
  /* Reset the Output Polarity level */
  tmpccer &= CCER_CC1P_Reset;

  /* Set the Output Compare Polarity */
  tmpccer |= TIM_OCInitStruct->TIM_OCPolarity;
  
  /* Set the Output State */
  tmpccer |= TIM_OCInitStruct->TIM_OutputState;
  
  /* Set the Capture Compare Register value */
  TIMx->CCR1 = TIM_OCInitStruct->TIM_Pulse;
  
  if((*(u32*)&TIMx == TIM1_BASE) || (*(u32*)&TIMx == TIM8_BASE))
  {
    assert_param(IS_TIM_OUTPUTN_STATE(TIM_OCInitStruct->TIM_OutputNState));
    assert_param(IS_TIM_OCN_POLARITY(TIM_OCInitStruct->TIM_OCNPolarity));
    assert_param(IS_TIM_OCNIDLE_STATE(TIM_OCInitStruct->TIM_OCNIdleState));
    assert_param(IS_TIM_OCIDLE_STATE(TIM_OCInitStruct->TIM_OCIdleState));
    
    /* Reset the Output N Polarity level */
    tmpccer &= CCER_CC1NP_Reset;

    /* Set the Output N Polarity */
    tmpccer |= TIM_OCInitStruct->TIM_OCNPolarity;

    /* Reset the Output N State */
    tmpccer &= CCER_CC1NE_Reset;
    
    /* Set the Output N State */
    tmpccer |= TIM_OCInitStruct->TIM_OutputNState;

    /* Reset the Ouput Compare and Output Compare N IDLE State */
    tmpcr2 &= CR2_OIS1_Reset;
    tmpcr2 &= CR2_OIS1N_Reset;

    /* Set the Output Idle state */
    tmpcr2 |= TIM_OCInitStruct->TIM_OCIdleState;

    /* Set the Output N Idle state */
    tmpcr2 |= TIM_OCInitStruct->TIM_OCNIdleState;
  }
  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;
  
  /* Write to TIMx CCMR1 */
  TIMx->CCMR1 = tmpccmrx;
  
  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_OC2Init
* Description    : Initializes the TIMx Channel2 according to the specified
*                  parameters in the TIM_OCInitStruct.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure
*                    that contains the configuration information for the specified
*                    TIM peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC2Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  u16 tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
   
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx)); 
  assert_param(IS_TIM_OC_MODE(TIM_OCInitStruct->TIM_OCMode));
  assert_param(IS_TIM_OUTPUT_STATE(TIM_OCInitStruct->TIM_OutputState));
  assert_param(IS_TIM_OC_POLARITY(TIM_OCInitStruct->TIM_OCPolarity));   

  /* Disable the Channel 2: Reset the CC2E Bit */
  TIMx->CCER &= CCER_CC2E_Reset;
  
  /* Get the TIMx CCER register value */  
  tmpccer = TIMx->CCER;

  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;
  
  /* Get the TIMx CCMR1 register value */
  tmpccmrx = TIMx->CCMR1;
    
  /* Reset the Output Compare Mode Bits */
  tmpccmrx &= CCMR_OC24M_Mask;
  
  /* Select the Output Compare Mode */
  tmpccmrx |= (u16)(TIM_OCInitStruct->TIM_OCMode << 8);
  
  /* Reset the Output Polarity level */
  tmpccer &= CCER_CC2P_Reset;

  /* Set the Output Compare Polarity */
  tmpccer |= (u16)(TIM_OCInitStruct->TIM_OCPolarity << 4);
  
  /* Set the Output State */
  tmpccer |= (u16)(TIM_OCInitStruct->TIM_OutputState << 4);
  
  /* Set the Capture Compare Register value */
  TIMx->CCR2 = TIM_OCInitStruct->TIM_Pulse;
  
  if((*(u32*)&TIMx == TIM1_BASE) || (*(u32*)&TIMx == TIM8_BASE))
  {
    assert_param(IS_TIM_OUTPUTN_STATE(TIM_OCInitStruct->TIM_OutputNState));
    assert_param(IS_TIM_OCN_POLARITY(TIM_OCInitStruct->TIM_OCNPolarity));
    assert_param(IS_TIM_OCNIDLE_STATE(TIM_OCInitStruct->TIM_OCNIdleState));
    assert_param(IS_TIM_OCIDLE_STATE(TIM_OCInitStruct->TIM_OCIdleState));
    
    /* Reset the Output N Polarity level */
    tmpccer &= CCER_CC2NP_Reset;

    /* Set the Output N Polarity */
    tmpccer |= (u16)(TIM_OCInitStruct->TIM_OCNPolarity << 4);

    /* Reset the Output N State */
    tmpccer &= CCER_CC2NE_Reset;
    
    /* Set the Output N State */
    tmpccer |= (u16)(TIM_OCInitStruct->TIM_OutputNState << 4);

    /* Reset the Ouput Compare and Output Compare N IDLE State */
    tmpcr2 &= CR2_OIS2_Reset;
    tmpcr2 &= CR2_OIS2N_Reset;

    /* Set the Output Idle state */
    tmpcr2 |= (u16)(TIM_OCInitStruct->TIM_OCIdleState << 2);

    /* Set the Output N Idle state */
    tmpcr2 |= (u16)(TIM_OCInitStruct->TIM_OCNIdleState << 2);
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;
  
  /* Write to TIMx CCMR1 */
  TIMx->CCMR1 = tmpccmrx;
  
  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_OC3Init
* Description    : Initializes the TIMx Channel3 according to the specified
*                  parameters in the TIM_OCInitStruct.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure
*                    that contains the configuration information for the specified
*                    TIM peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC3Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  u16 tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
   
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx)); 
  assert_param(IS_TIM_OC_MODE(TIM_OCInitStruct->TIM_OCMode));
  assert_param(IS_TIM_OUTPUT_STATE(TIM_OCInitStruct->TIM_OutputState));
  assert_param(IS_TIM_OC_POLARITY(TIM_OCInitStruct->TIM_OCPolarity));   

  /* Disable the Channel 2: Reset the CC2E Bit */
  TIMx->CCER &= CCER_CC3E_Reset;
  
  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;

  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;
  
  /* Get the TIMx CCMR2 register value */
  tmpccmrx = TIMx->CCMR2;
    
  /* Reset the Output Compare Mode Bits */
  tmpccmrx &= CCMR_OC13M_Mask;
  
  /* Select the Output Compare Mode */
  tmpccmrx |= TIM_OCInitStruct->TIM_OCMode;
  
  /* Reset the Output Polarity level */
  tmpccer &= CCER_CC3P_Reset;

  /* Set the Output Compare Polarity */
  tmpccer |= (u16)(TIM_OCInitStruct->TIM_OCPolarity << 8);
  
  /* Set the Output State */
  tmpccer |= (u16)(TIM_OCInitStruct->TIM_OutputState << 8);
  
  /* Set the Capture Compare Register value */
  TIMx->CCR3 = TIM_OCInitStruct->TIM_Pulse;
  
  if((*(u32*)&TIMx == TIM1_BASE) || (*(u32*)&TIMx == TIM8_BASE))
  {
    assert_param(IS_TIM_OUTPUTN_STATE(TIM_OCInitStruct->TIM_OutputNState));
    assert_param(IS_TIM_OCN_POLARITY(TIM_OCInitStruct->TIM_OCNPolarity));
    assert_param(IS_TIM_OCNIDLE_STATE(TIM_OCInitStruct->TIM_OCNIdleState));
    assert_param(IS_TIM_OCIDLE_STATE(TIM_OCInitStruct->TIM_OCIdleState));
    
    /* Reset the Output N Polarity level */
    tmpccer &= CCER_CC3NP_Reset;

    /* Set the Output N Polarity */
    tmpccer |= (u16)(TIM_OCInitStruct->TIM_OCNPolarity << 8);

    /* Reset the Output N State */
    tmpccer &= CCER_CC3NE_Reset;
    
    /* Set the Output N State */
    tmpccer |= (u16)(TIM_OCInitStruct->TIM_OutputNState << 8);

    /* Reset the Ouput Compare and Output Compare N IDLE State */
    tmpcr2 &= CR2_OIS3_Reset;
    tmpcr2 &= CR2_OIS3N_Reset;

    /* Set the Output Idle state */
    tmpcr2 |= (u16)(TIM_OCInitStruct->TIM_OCIdleState << 4);

    /* Set the Output N Idle state */
    tmpcr2 |= (u16)(TIM_OCInitStruct->TIM_OCNIdleState << 4);
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;
  
  /* Write to TIMx CCMR2 */
  TIMx->CCMR2 = tmpccmrx;
  
  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_OC4Init
* Description    : Initializes the TIMx Channel4 according to the specified
*                  parameters in the TIM_OCInitStruct.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure
*                    that contains the configuration information for the specified
*                    TIM peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC4Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  u16 tmpccmrx = 0, tmpccer = 0, tmpcr2 = 0;
   
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx)); 
  assert_param(IS_TIM_OC_MODE(TIM_OCInitStruct->TIM_OCMode));
  assert_param(IS_TIM_OUTPUT_STATE(TIM_OCInitStruct->TIM_OutputState));
  assert_param(IS_TIM_OC_POLARITY(TIM_OCInitStruct->TIM_OCPolarity));   

  /* Disable the Channel 2: Reset the CC4E Bit */
  TIMx->CCER &= CCER_CC4E_Reset;
  
  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;

  /* Get the TIMx CR2 register value */
  tmpcr2 =  TIMx->CR2;
  
  /* Get the TIMx CCMR2 register value */
  tmpccmrx = TIMx->CCMR2;
    
  /* Reset the Output Compare Mode Bits */
  tmpccmrx &= CCMR_OC24M_Mask;
  
  /* Select the Output Compare Mode */
  tmpccmrx |= (u16)(TIM_OCInitStruct->TIM_OCMode << 8);
  
  /* Reset the Output Polarity level */
  tmpccer &= CCER_CC4P_Reset;

  /* Set the Output Compare Polarity */
  tmpccer |= (u16)(TIM_OCInitStruct->TIM_OCPolarity << 12);
  
  /* Set the Output State */
  tmpccer |= (u16)(TIM_OCInitStruct->TIM_OutputState << 12);
  
  /* Set the Capture Compare Register value */
  TIMx->CCR4 = TIM_OCInitStruct->TIM_Pulse;
  
  if((*(u32*)&TIMx == TIM1_BASE) || (*(u32*)&TIMx == TIM8_BASE))
  {
    assert_param(IS_TIM_OCIDLE_STATE(TIM_OCInitStruct->TIM_OCIdleState));

    /* Reset the Ouput Compare IDLE State */
    tmpcr2 &= CR2_OIS4_Reset;

    /* Set the Output Idle state */
    tmpcr2 |= (u16)(TIM_OCInitStruct->TIM_OCIdleState << 6);
  }

  /* Write to TIMx CR2 */
  TIMx->CR2 = tmpcr2;
  
  /* Write to TIMx CCMR2 */  
  TIMx->CCMR2 = tmpccmrx;
  
  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_ICInit
* Description    : Initializes the TIM peripheral according to the specified
*                  parameters in the TIM_ICInitStruct.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICInitStruct: pointer to a TIM_ICInitTypeDef structure
*                    that contains the configuration information for the specified
*                    TIM peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_CHANNEL(TIM_ICInitStruct->TIM_Channel));
  assert_param(IS_TIM_IC_POLARITY(TIM_ICInitStruct->TIM_ICPolarity));
  assert_param(IS_TIM_IC_SELECTION(TIM_ICInitStruct->TIM_ICSelection));
  assert_param(IS_TIM_IC_PRESCALER(TIM_ICInitStruct->TIM_ICPrescaler));
  assert_param(IS_TIM_IC_FILTER(TIM_ICInitStruct->TIM_ICFilter));
  
  if (TIM_ICInitStruct->TIM_Channel == TIM_Channel_1)
  {
    /* TI1 Configuration */
    TI1_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity,
               TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);

    /* Set the Input Capture Prescaler value */
    TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
  else if (TIM_ICInitStruct->TIM_Channel == TIM_Channel_2)
  {
    /* TI2 Configuration */
    TI2_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity,
               TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);

    /* Set the Input Capture Prescaler value */
    TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
  else if (TIM_ICInitStruct->TIM_Channel == TIM_Channel_3)
  {
    /* TI3 Configuration */
    TI3_Config(TIMx,  TIM_ICInitStruct->TIM_ICPolarity,
               TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);

    /* Set the Input Capture Prescaler value */
    TIM_SetIC3Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
  else
  {
    /* TI4 Configuration */
    TI4_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity,
               TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);

    /* Set the Input Capture Prescaler value */
    TIM_SetIC4Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
}

/*******************************************************************************
* Function Name  : TIM_PWMIConfig
* Description    : Configures the TIM peripheral according to the specified
*                  parameters in the TIM_ICInitStruct to measure an external PWM
*                  signal.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICInitStruct: pointer to a TIM_ICInitTypeDef structure
*                    that contains the configuration information for the specified
*                    TIM peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct)
{
  u16 icoppositepolarity = TIM_ICPolarity_Rising;
  u16 icoppositeselection = TIM_ICSelection_DirectTI;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));

  /* Select the Opposite Input Polarity */
  if (TIM_ICInitStruct->TIM_ICPolarity == TIM_ICPolarity_Rising)
  {
    icoppositepolarity = TIM_ICPolarity_Falling;
  }
  else
  {
    icoppositepolarity = TIM_ICPolarity_Rising;
  }

  /* Select the Opposite Input */
  if (TIM_ICInitStruct->TIM_ICSelection == TIM_ICSelection_DirectTI)
  {
    icoppositeselection = TIM_ICSelection_IndirectTI;
  }
  else
  {
    icoppositeselection = TIM_ICSelection_DirectTI;
  }

  if (TIM_ICInitStruct->TIM_Channel == TIM_Channel_1)
  {
    /* TI1 Configuration */
    TI1_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity, TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);

    /* Set the Input Capture Prescaler value */
    TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);

    /* TI2 Configuration */
    TI2_Config(TIMx, icoppositepolarity, icoppositeselection, TIM_ICInitStruct->TIM_ICFilter);

    /* Set the Input Capture Prescaler value */
    TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
  else
  { 
    /* TI2 Configuration */
    TI2_Config(TIMx, TIM_ICInitStruct->TIM_ICPolarity, TIM_ICInitStruct->TIM_ICSelection,
               TIM_ICInitStruct->TIM_ICFilter);

    /* Set the Input Capture Prescaler value */
    TIM_SetIC2Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);

    /* TI1 Configuration */
    TI1_Config(TIMx, icoppositepolarity, icoppositeselection, TIM_ICInitStruct->TIM_ICFilter);

    /* Set the Input Capture Prescaler value */
    TIM_SetIC1Prescaler(TIMx, TIM_ICInitStruct->TIM_ICPrescaler);
  }
}

/*******************************************************************************
* Function Name  : TIM_BDTRConfig
* Description    : Configures the: Break feature, dead time, Lock level, the OSSI,
*                  the OSSR State and the AOE(automatic output enable).
* Input          :- TIMx: where x can be  1 or 8 to select the TIM 
*                 - TIM_BDTRInitStruct: pointer to a TIM_BDTRInitTypeDef
*                    structure that contains the BDTR Register configuration
*                    information for the TIM peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_BDTRConfig(TIM_TypeDef* TIMx, TIM_BDTRInitTypeDef *TIM_BDTRInitStruct)
{
  /* Check the parameters */
  assert_param(IS_TIM_18_PERIPH(TIMx));
  assert_param(IS_TIM_OSSR_STATE(TIM_BDTRInitStruct->TIM_OSSRState));
  assert_param(IS_TIM_OSSI_STATE(TIM_BDTRInitStruct->TIM_OSSIState));
  assert_param(IS_TIM_LOCK_LEVEL(TIM_BDTRInitStruct->TIM_LOCKLevel));
  assert_param(IS_TIM_BREAK_STATE(TIM_BDTRInitStruct->TIM_Break));
  assert_param(IS_TIM_BREAK_POLARITY(TIM_BDTRInitStruct->TIM_BreakPolarity));
  assert_param(IS_TIM_AUTOMATIC_OUTPUT_STATE(TIM_BDTRInitStruct->TIM_AutomaticOutput));

  /* Set the Lock level, the Break enable Bit and the Ploarity, the OSSR State,
     the OSSI State, the dead time value and the Automatic Output Enable Bit */

  TIMx->BDTR = (u32)TIM_BDTRInitStruct->TIM_OSSRState | TIM_BDTRInitStruct->TIM_OSSIState |
             TIM_BDTRInitStruct->TIM_LOCKLevel | TIM_BDTRInitStruct->TIM_DeadTime |
             TIM_BDTRInitStruct->TIM_Break | TIM_BDTRInitStruct->TIM_BreakPolarity |
             TIM_BDTRInitStruct->TIM_AutomaticOutput;

}

/*******************************************************************************
* Function Name  : TIM_TimeBaseStructInit
* Description    : Fills each TIM_TimeBaseInitStruct member with its default value.
* Input          : - TIM_TimeBaseInitStruct : pointer to a TIM_TimeBaseInitTypeDef
*                    structure which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_TimeBaseStructInit(TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct)
{
  /* Set the default configuration */
  TIM_TimeBaseInitStruct->TIM_Period = 0xFFFF;
  TIM_TimeBaseInitStruct->TIM_Prescaler = 0x0000;
  TIM_TimeBaseInitStruct->TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct->TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct->TIM_RepetitionCounter = 0x0000;
}

/*******************************************************************************
* Function Name  : TIM_OCStructInit
* Description    : Fills each TIM_OCInitStruct member with its default value.
* Input          : - TIM_OCInitStruct : pointer to a TIM_OCInitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct)
{
  /* Set the default configuration */
  TIM_OCInitStruct->TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStruct->TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStruct->TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStruct->TIM_Pulse = 0x0000;
  TIM_OCInitStruct->TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStruct->TIM_OCNPolarity = TIM_OCPolarity_High;
  TIM_OCInitStruct->TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStruct->TIM_OCNIdleState = TIM_OCNIdleState_Reset;
}

/*******************************************************************************
* Function Name  : TIM_ICStructInit
* Description    : Fills each TIM_ICInitStruct member with its default value.
* Input          : - TIM_ICInitStruct : pointer to a TIM_ICInitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct)
{
  /* Set the default configuration */
  TIM_ICInitStruct->TIM_Channel = TIM_Channel_1;
  TIM_ICInitStruct->TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStruct->TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStruct->TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStruct->TIM_ICFilter = 0x00;
}

/*******************************************************************************
* Function Name  : TIM_BDTRStructInit
* Description    : Fills each TIM_BDTRInitStruct member with its default value.
* Input          : - TIM_BDTRInitStruct : pointer to a TIM_BDTRInitTypeDef
*                    structure which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_BDTRStructInit(TIM_BDTRInitTypeDef* TIM_BDTRInitStruct)
{
  /* Set the default configuration */
  TIM_BDTRInitStruct->TIM_OSSRState = TIM_OSSRState_Disable;
  TIM_BDTRInitStruct->TIM_OSSIState = TIM_OSSIState_Disable;
  TIM_BDTRInitStruct->TIM_LOCKLevel = TIM_LOCKLevel_OFF;
  TIM_BDTRInitStruct->TIM_DeadTime = 0x00;
  TIM_BDTRInitStruct->TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStruct->TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM_BDTRInitStruct->TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
}

/*******************************************************************************
* Function Name  : TIM_Cmd
* Description    : Enables or disables the specified TIM peripheral.
* Input          : - TIMx: where x can be 1 to 8 to select the TIMx peripheral.
*                  - NewState: new state of the TIMx peripheral.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the TIM Counter */
    TIMx->CR1 |= CR1_CEN_Set;
  }
  else
  {
    /* Disable the TIM Counter */
    TIMx->CR1 &= CR1_CEN_Reset;
  }
}

/*******************************************************************************
* Function Name  : TIM_CtrlPWMOutputs
* Description    : Enables or disables the TIM peripheral Main Outputs.
* Input          :- TIMx: where x can be 1 or 8 to select the TIMx peripheral.
*                 - NewState: new state of the TIM peripheral Main Outputs.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_CtrlPWMOutputs(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_TIM_18_PERIPH(TIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the TIM Main Output */
    TIMx->BDTR |= BDTR_MOE_Set;
  }
  else
  {
    /* Disable the TIM Main Output */
    TIMx->BDTR &= BDTR_MOE_Reset;
  }  
}

/*******************************************************************************
* Function Name  : TIM_ITConfig
* Description    : Enables or disables the specified TIM interrupts.
* Input          : - TIMx: where x can be 1 to 8 to select the TIMx peripheral.
*                  - TIM_IT: specifies the TIM interrupts sources to be enabled
*                    or disabled.
*                    This parameter can be any combination of the following values:
*                       - TIM_IT_Update: TIM update Interrupt source
*                       - TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
*                       - TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
*                       - TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
*                       - TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
*                       - TIM_IT_COM: TIM Commutation Interrupt source
*                       - TIM_IT_Trigger: TIM Trigger Interrupt source
*                       - TIM_IT_Break: TIM Break Interrupt source
*                  - NewState: new state of the TIM interrupts.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ITConfig(TIM_TypeDef* TIMx, u16 TIM_IT, FunctionalState NewState)
{  
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_IT(TIM_IT));
  assert_param(IS_TIM_PERIPH_IT((TIMx), (TIM_IT)));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the Interrupt sources */
    TIMx->DIER |= TIM_IT;
  }
  else
  {
    /* Disable the Interrupt sources */
    TIMx->DIER &= (u16)~TIM_IT;
  }
}

/*******************************************************************************
* Function Name  : TIM_GenerateEvent
* Description    : Configures the TIMx event to be generate by software.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - TIM_EventSource: specifies the event source.
*                    This parameter can be one or more of the following values:	   
*                       - TIM_EventSource_Update: Timer update Event source
*                       - TIM_EventSource_CC1: Timer Capture Compare 1 Event source
*                       - TIM_EventSource_CC2: Timer Capture Compare 2 Event source
*                       - TIM_EventSource_CC3: Timer Capture Compare 3 Event source
*                       - TIM_EventSource_CC4: Timer Capture Compare 4 Event source
*                       - TIM_EventSource_Trigger: Timer Trigger Event source
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_GenerateEvent(TIM_TypeDef* TIMx, u16 TIM_EventSource)
{ 
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_EVENT_SOURCE(TIM_EventSource));
  assert_param(IS_TIM_PERIPH_EVENT((TIMx), (TIM_EventSource)));

  /* Set the event sources */
  TIMx->EGR = TIM_EventSource;
}

/*******************************************************************************
* Function Name  : TIM_DMAConfig
* Description    : Configures the TIMx’s DMA interface.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_DMABase: DMA Base address.
*                    This parameter can be one of the following values:
*                       - TIM_DMABase_CR, TIM_DMABase_CR2, TIM_DMABase_SMCR,
*                         TIM_DMABase_DIER, TIM1_DMABase_SR, TIM_DMABase_EGR,
*                         TIM_DMABase_CCMR1, TIM_DMABase_CCMR2, TIM_DMABase_CCER,
*                         TIM_DMABase_CNT, TIM_DMABase_PSC, TIM_DMABase_ARR,
*                         TIM_DMABase_RCR, TIM_DMABase_CCR1, TIM_DMABase_CCR2,
*                         TIM_DMABase_CCR3, TIM_DMABase_CCR4, TIM_DMABase_BDTR,
*                         TIM_DMABase_DCR.
*                   - TIM_DMABurstLength: DMA Burst length.
*                     This parameter can be one value between:
*                     TIM_DMABurstLength_1Byte and TIM_DMABurstLength_18Bytes.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_DMAConfig(TIM_TypeDef* TIMx, u16 TIM_DMABase, u16 TIM_DMABurstLength)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_DMA_BASE(TIM_DMABase));
  assert_param(IS_TIM_DMA_LENGTH(TIM_DMABurstLength));

  /* Set the DMA Base and the DMA Burst Length */
  TIMx->DCR = TIM_DMABase | TIM_DMABurstLength;
}

/*******************************************************************************
* Function Name  : TIM_DMACmd
* Description    : Enables or disables the TIMx’s DMA Requests.
* Input          : - TIMx: where x can be  1 to 8 to select the TIM peripheral. 
*                  - TIM_DMASources: specifies the DMA Request sources.
*                    This parameter can be any combination of the following values:
*                       - TIM_DMA_Update: TIM update Interrupt source
*                       - TIM_DMA_CC1: TIM Capture Compare 1 DMA source
*                       - TIM_DMA_CC2: TIM Capture Compare 2 DMA source
*                       - TIM_DMA_CC3: TIM Capture Compare 3 DMA source
*                       - TIM_DMA_CC4: TIM Capture Compare 4 DMA source
*                       - TIM_DMA_COM: TIM Commutation DMA source
*                       - TIM_DMA_Trigger: TIM Trigger DMA source
*                  - NewState: new state of the DMA Request sources.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_DMACmd(TIM_TypeDef* TIMx, u16 TIM_DMASource, FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_DMA_SOURCE(TIM_DMASource));
  assert_param(IS_TIM_PERIPH_DMA(TIMx, TIM_DMASource));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the DMA sources */
    TIMx->DIER |= TIM_DMASource; 
  }
  else
  {
    /* Disable the DMA sources */
    TIMx->DIER &= (u16)~TIM_DMASource;
  }
}

/*******************************************************************************
* Function Name  : TIM_InternalClockConfig
* Description    : Configures the TIMx interrnal Clock
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_InternalClockConfig(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));

  /* Disable slave mode to clock the prescaler directly with the internal clock */
  TIMx->SMCR &=  SMCR_SMS_Mask;
}
/*******************************************************************************
* Function Name  : TIM_ITRxExternalClockConfig
* Description    : Configures the TIMx Internal Trigger as External Clock
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ITRSource: Trigger source.
*                    This parameter can be one of the following values:
*                       - TIM_TS_ITR0: Internal Trigger 0
*                       - TIM_TS_ITR1: Internal Trigger 1
*                       - TIM_TS_ITR2: Internal Trigger 2
*                       - TIM_TS_ITR3: Internal Trigger 3
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ITRxExternalClockConfig(TIM_TypeDef* TIMx, u16 TIM_InputTriggerSource)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_INTERNAL_TRIGGER_SELECTION(TIM_InputTriggerSource));

  /* Select the Internal Trigger */
  TIM_SelectInputTrigger(TIMx, TIM_InputTriggerSource);

  /* Select the External clock mode1 */
  TIMx->SMCR |= TIM_SlaveMode_External1;
}
/*******************************************************************************
* Function Name  : TIM_TIxExternalClockConfig
* Description    : Configures the TIMx Trigger as External Clock
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_TIxExternalCLKSource: Trigger source.
*                    This parameter can be one of the following values:
*                       - TIM_TIxExternalCLK1Source_TI1ED: TI1 Edge Detector
*                       - TIM_TIxExternalCLK1Source_TI1: Filtered Timer Input 1
*                       - TIM_TIxExternalCLK1Source_TI2: Filtered Timer Input 2
*                  - TIM_ICPolarity: specifies the TIx Polarity.
*                    This parameter can be:
*                       - TIM_ICPolarity_Rising
*                       - TIM_ICPolarity_Falling
*                   - ICFilter : specifies the filter value.
*                     This parameter must be a value between 0x0 and 0xF.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_TIxExternalClockConfig(TIM_TypeDef* TIMx, u16 TIM_TIxExternalCLKSource,
                                u16 TIM_ICPolarity, u16 ICFilter)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_TIXCLK_SOURCE(TIM_TIxExternalCLKSource));
  assert_param(IS_TIM_IC_POLARITY(TIM_ICPolarity));
  assert_param(IS_TIM_IC_FILTER(ICFilter));

  /* Configure the Timer Input Clock Source */
  if (TIM_TIxExternalCLKSource == TIM_TIxExternalCLK1Source_TI2)
  {
    TI2_Config(TIMx, TIM_ICPolarity, TIM_ICSelection_DirectTI, ICFilter);
  }
  else
  {
    TI1_Config(TIMx, TIM_ICPolarity, TIM_ICSelection_DirectTI, ICFilter);
  }

  /* Select the Trigger source */
  TIM_SelectInputTrigger(TIMx, TIM_TIxExternalCLKSource);

  /* Select the External clock mode1 */
  TIMx->SMCR |= TIM_SlaveMode_External1;
}

/*******************************************************************************
* Function Name  : TIM_ETRClockMode1Config
* Description    : Configures the External clock Mode1
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ExtTRGPrescaler: The external Trigger Prescaler.
*                    It can be one of the following values:
*                       - TIM_ExtTRGPSC_OFF
*                       - TIM_ExtTRGPSC_DIV2
*                       - TIM_ExtTRGPSC_DIV4
*                       - TIM_ExtTRGPSC_DIV8.
*                  - TIM_ExtTRGPolarity: The external Trigger Polarity.
*                    It can be one of the following values:
*                       - TIM_ExtTRGPolarity_Inverted
*                       - TIM_ExtTRGPolarity_NonInverted
*                  - ExtTRGFilter: External Trigger Filter.
*                    This parameter must be a value between 0x00 and 0x0F
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ETRClockMode1Config(TIM_TypeDef* TIMx, u16 TIM_ExtTRGPrescaler, u16 TIM_ExtTRGPolarity,
                             u16 ExtTRGFilter)
{
  u16 tmpsmcr = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_EXT_PRESCALER(TIM_ExtTRGPrescaler));
  assert_param(IS_TIM_EXT_POLARITY(TIM_ExtTRGPolarity));
  assert_param(IS_TIM_EXT_FILTER(ExtTRGFilter));

  /* Configure the ETR Clock source */
  TIM_ETRConfig(TIMx, TIM_ExtTRGPrescaler, TIM_ExtTRGPolarity, ExtTRGFilter);
  
  /* Get the TIMx SMCR register value */
  tmpsmcr = TIMx->SMCR;

  /* Reset the SMS Bits */
  tmpsmcr &= SMCR_SMS_Mask;
  /* Select the External clock mode1 */
  tmpsmcr |= TIM_SlaveMode_External1;

  /* Select the Trigger selection : ETRF */
  tmpsmcr &= SMCR_TS_Mask;
  tmpsmcr |= TIM_TS_ETRF;

  /* Write to TIMx SMCR */
  TIMx->SMCR = tmpsmcr;
}

/*******************************************************************************
* Function Name  : TIM_ETRClockMode2Config
* Description    : Configures the External clock Mode2
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ExtTRGPrescaler: The external Trigger Prescaler.
*                    It can be one of the following values:
*                       - TIM_ExtTRGPSC_OFF
*                       - TIM_ExtTRGPSC_DIV2
*                       - TIM_ExtTRGPSC_DIV4
*                       - TIM_ExtTRGPSC_DIV8
*                  - TIM_ExtTRGPolarity: The external Trigger Polarity.
*                    It can be one of the following values:
*                       - TIM_ExtTRGPolarity_Inverted
*                       - TIM_ExtTRGPolarity_NonInverted
*                  - ExtTRGFilter: External Trigger Filter.
*                    This parameter must be a value between 0x00 and 0x0F
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, u16 TIM_ExtTRGPrescaler, 
                             u16 TIM_ExtTRGPolarity, u16 ExtTRGFilter)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_EXT_PRESCALER(TIM_ExtTRGPrescaler));
  assert_param(IS_TIM_EXT_POLARITY(TIM_ExtTRGPolarity));
  assert_param(IS_TIM_EXT_FILTER(ExtTRGFilter));

  /* Configure the ETR Clock source */
  TIM_ETRConfig(TIMx, TIM_ExtTRGPrescaler, TIM_ExtTRGPolarity, ExtTRGFilter);

  /* Enable the External clock mode2 */
  TIMx->SMCR |= SMCR_ECE_Set;
}

/*******************************************************************************
* Function Name  : TIM_ETRConfig
* Description    : Configures the TIMx External Trigger (ETR).
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ExtTRGPrescaler: The external Trigger Prescaler.
*                    This parameter can be one of the following values:
*                       - TIM_ExtTRGPSC_OFF
*                       - TIM_ExtTRGPSC_DIV2
*                       - TIM_ExtTRGPSC_DIV4
*                       - TIM_ExtTRGPSC_DIV8
*                  - TIM_ExtTRGPolarity: The external Trigger Polarity.
*                    This parameter can be one of the following values:
*                       - TIM_ExtTRGPolarity_Inverted
*                       - TIM_ExtTRGPolarity_NonInverted
*                  - ExtTRGFilter: External Trigger Filter.
*                    This parameter must be a value between 0x00 and 0x0F.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ETRConfig(TIM_TypeDef* TIMx, u16 TIM_ExtTRGPrescaler, u16 TIM_ExtTRGPolarity,
                   u16 ExtTRGFilter)
{
  u16 tmpsmcr = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_EXT_PRESCALER(TIM_ExtTRGPrescaler));
  assert_param(IS_TIM_EXT_POLARITY(TIM_ExtTRGPolarity));
  assert_param(IS_TIM_EXT_FILTER(ExtTRGFilter));

  tmpsmcr = TIMx->SMCR;

  /* Reset the ETR Bits */
  tmpsmcr &= SMCR_ETR_Mask;

  /* Set the Prescaler, the Filter value and the Polarity */
  tmpsmcr |= TIM_ExtTRGPrescaler | TIM_ExtTRGPolarity | (u16)(ExtTRGFilter << 8);

  /* Write to TIMx SMCR */
  TIMx->SMCR = tmpsmcr;
}

/*******************************************************************************
* Function Name  : TIM_PrescalerConfig
* Description    : Configures the TIMx Prescaler.
* Input          : - TIMx: where x can be  1 to 8 to select the TIM peripheral.
*                  - Prescaler: specifies the Prescaler Register value
*                  - TIM_PSCReloadMode: specifies the TIM Prescaler Reload mode
*                    This parameter can be one of the following values:
*                       - TIM_PSCReloadMode_Update: The Prescaler is loaded at
*                         the update event.
*                       - TIM_PSCReloadMode_Immediate: The Prescaler is loaded
*                         immediatly.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_PrescalerConfig(TIM_TypeDef* TIMx, u16 Prescaler, u16 TIM_PSCReloadMode)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_PRESCALER_RELOAD(TIM_PSCReloadMode));

  /* Set the Prescaler value */
  TIMx->PSC = Prescaler;

  /* Set or reset the UG Bit */
  TIMx->EGR = TIM_PSCReloadMode;
}

/*******************************************************************************
* Function Name  : TIM_CounterModeConfig
* Description    : Specifies the TIMx Counter Mode to be used.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_CounterMode: specifies the Counter Mode to be used
*                    This parameter can be one of the following values:
*                       - TIM_CounterMode_Up: TIM Up Counting Mode
*                       - TIM_CounterMode_Down: TIM Down Counting Mode
*                       - TIM_CounterMode_CenterAligned1: TIM Center Aligned Mode1
*                       - TIM_CounterMode_CenterAligned2: TIM Center Aligned Mode2
*                       - TIM_CounterMode_CenterAligned3: TIM Center Aligned Mode3
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_CounterModeConfig(TIM_TypeDef* TIMx, u16 TIM_CounterMode)
{
  u16 tmpcr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_COUNTER_MODE(TIM_CounterMode));

  tmpcr1 = TIMx->CR1;

  /* Reset the CMS and DIR Bits */
  tmpcr1 &= CR1_CounterMode_Mask;

  /* Set the Counter Mode */
  tmpcr1 |= TIM_CounterMode;

  /* Write to TIMx CR1 register */
  TIMx->CR1 = tmpcr1;
}

/*******************************************************************************
* Function Name  : TIM_SelectInputTrigger
* Description    : Selects the Input Trigger source
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_InputTriggerSource: The Input Trigger source.
*                    This parameter can be one of the following values:
*                       - TIM_TS_ITR0: Internal Trigger 0
*                       - TIM_TS_ITR1: Internal Trigger 1
*                       - TIM_TS_ITR2: Internal Trigger 2
*                       - TIM_TS_ITR3: Internal Trigger 3
*                       - TIM_TS_TI1F_ED: TI1 Edge Detector
*                       - TIM_TS_TI1FP1: Filtered Timer Input 1
*                       - TIM_TS_TI2FP2: Filtered Timer Input 2
*                       - TIM_TS_ETRF: External Trigger input
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, u16 TIM_InputTriggerSource)
{
  u16 tmpsmcr = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_TRIGGER_SELECTION(TIM_InputTriggerSource));

  /* Get the TIMx SMCR register value */
  tmpsmcr = TIMx->SMCR;

  /* Reset the TS Bits */
  tmpsmcr &= SMCR_TS_Mask;

  /* Set the Input Trigger source */
  tmpsmcr |= TIM_InputTriggerSource;

  /* Write to TIMx SMCR */
  TIMx->SMCR = tmpsmcr;
}

/*******************************************************************************
* Function Name  : TIM_EncoderInterfaceConfig
* Description    : Configures the TIMx Encoder Interface.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_EncoderMode: specifies the TIMx Encoder Mode.
*                    This parameter can be one of the following values:
*                       - TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge
*                         depending on TI2FP2 level.
*                       - TIM_EncoderMode_TI2: Counter counts on TI2FP2 edge
*                         depending on TI1FP1 level.
*                       - TIM_EncoderMode_TI12: Counter counts on both TI1FP1 and
*                         TI2FP2 edges depending on the level of the other input.
*                  - TIM_IC1Polarity: specifies the IC1 Polarity
*                    This parmeter can be one of the following values:
*                        - TIM_ICPolarity_Falling: IC Falling edge.
*                        - TIM_ICPolarity_Rising: IC Rising edge.
*                  - TIM_IC2Polarity: specifies the IC2 Polarity
*                    This parmeter can be one of the following values:
*                        - TIM_ICPolarity_Falling: IC Falling edge.
*                        - TIM_ICPolarity_Rising: IC Rising edge.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, u16 TIM_EncoderMode,
                                u16 TIM_IC1Polarity, u16 TIM_IC2Polarity)
{
  u16 tmpsmcr = 0;
  u16 tmpccmr1 = 0;
  u16 tmpccer = 0;
    
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_ENCODER_MODE(TIM_EncoderMode));
  assert_param(IS_TIM_IC_POLARITY(TIM_IC1Polarity));
  assert_param(IS_TIM_IC_POLARITY(TIM_IC2Polarity));

  /* Get the TIMx SMCR register value */
  tmpsmcr = TIMx->SMCR;

  /* Get the TIMx CCMR1 register value */
  tmpccmr1 = TIMx->CCMR1;

  /* Get the TIMx CCER register value */
  tmpccer = TIMx->CCER;

  /* Set the encoder Mode */
  tmpsmcr &= SMCR_SMS_Mask;
  tmpsmcr |= TIM_EncoderMode;

  /* Select the Capture Compare 1 and the Capture Compare 2 as input */
  tmpccmr1 &= CCMR_CC13S_Mask & CCMR_CC24S_Mask;
  tmpccmr1 |= CCMR_TI13Direct_Set | CCMR_TI24Direct_Set;

  /* Set the TI1 and the TI2 Polarities */
  tmpccer &= CCER_CC1P_Reset & CCER_CC2P_Reset;
  tmpccer |= (TIM_IC1Polarity | (u16)(TIM_IC2Polarity << 4));

  /* Write to TIMx SMCR */
  TIMx->SMCR = tmpsmcr;

  /* Write to TIMx CCMR1 */
  TIMx->CCMR1 = tmpccmr1;

  /* Write to TIMx CCER */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_ForcedOC1Config
* Description    : Forces the TIMx output 1 waveform to active or inactive level.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ForcedAction: specifies the forced Action to be set to
*                    the output waveform.
*                    This parameter can be one of the following values:
*                       - TIM_ForcedAction_Active: Force active level on OC1REF
*                       - TIM_ForcedAction_InActive: Force inactive level on
*                         OC1REF.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ForcedOC1Config(TIM_TypeDef* TIMx, u16 TIM_ForcedAction)
{
  u16 tmpccmr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_FORCED_ACTION(TIM_ForcedAction));

  tmpccmr1 = TIMx->CCMR1;

  /* Reset the OC1M Bits */
  tmpccmr1 &= CCMR_OC13M_Mask;

  /* Configure The Forced output Mode */
  tmpccmr1 |= TIM_ForcedAction;

  /* Write to TIMx CCMR1 register */
  TIMx->CCMR1 = tmpccmr1;
}

/*******************************************************************************
* Function Name  : TIM_ForcedOC2Config
* Description    : Forces the TIMx output 2 waveform to active or inactive level.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ForcedAction: specifies the forced Action to be set to
*                    the output waveform.
*                    This parameter can be one of the following values:
*                       - TIM_ForcedAction_Active: Force active level on OC2REF
*                       - TIM_ForcedAction_InActive: Force inactive level on
*                         OC2REF.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ForcedOC2Config(TIM_TypeDef* TIMx, u16 TIM_ForcedAction)
{
  u16 tmpccmr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_FORCED_ACTION(TIM_ForcedAction));

  tmpccmr1 = TIMx->CCMR1;

  /* Reset the OC2M Bits */
  tmpccmr1 &= CCMR_OC24M_Mask;

  /* Configure The Forced output Mode */
  tmpccmr1 |= (u16)(TIM_ForcedAction << 8);

  /* Write to TIMx CCMR1 register */
  TIMx->CCMR1 = tmpccmr1;
}

/*******************************************************************************
* Function Name  : TIM_ForcedOC3Config
* Description    : Forces the TIMx output 3 waveform to active or inactive level.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ForcedAction: specifies the forced Action to be set to
*                    the output waveform.
*                    This parameter can be one of the following values:
*                       - TIM_ForcedAction_Active: Force active level on OC3REF
*                       - TIM_ForcedAction_InActive: Force inactive level on
*                         OC3REF.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ForcedOC3Config(TIM_TypeDef* TIMx, u16 TIM_ForcedAction)
{
  u16 tmpccmr2 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_FORCED_ACTION(TIM_ForcedAction));

  tmpccmr2 = TIMx->CCMR2;

  /* Reset the OC1M Bits */
  tmpccmr2 &= CCMR_OC13M_Mask;

  /* Configure The Forced output Mode */
  tmpccmr2 |= TIM_ForcedAction;

  /* Write to TIMx CCMR2 register */
  TIMx->CCMR2 = tmpccmr2;
}

/*******************************************************************************
* Function Name  : TIM_ForcedOC4Config
* Description    : Forces the TIMx output 4 waveform to active or inactive level.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ForcedAction: specifies the forced Action to be set to
*                    the output waveform.
*                    This parameter can be one of the following values:
*                       - TIM_ForcedAction_Active: Force active level on OC4REF
*                       - TIM_ForcedAction_InActive: Force inactive level on
*                         OC4REF.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ForcedOC4Config(TIM_TypeDef* TIMx, u16 TIM_ForcedAction)
{
  u16 tmpccmr2 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_FORCED_ACTION(TIM_ForcedAction));
  tmpccmr2 = TIMx->CCMR2;

  /* Reset the OC2M Bits */
  tmpccmr2 &= CCMR_OC24M_Mask;

  /* Configure The Forced output Mode */
  tmpccmr2 |= (u16)(TIM_ForcedAction << 8);

  /* Write to TIMx CCMR2 register */
  TIMx->CCMR2 = tmpccmr2;
}

/*******************************************************************************
* Function Name  : TIM_ARRPreloadConfig
* Description    : Enables or disables TIMx peripheral Preload register on ARR.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - NewState: new state of the TIMx peripheral Preload register
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ARRPreloadConfig(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the ARR Preload Bit */
    TIMx->CR1 |= CR1_ARPE_Set;
  }
  else
  {
    /* Reset the ARR Preload Bit */
    TIMx->CR1 &= CR1_ARPE_Reset;
  }
}

/*******************************************************************************
* Function Name  : TIM_SelectCOM
* Description    : Selects the TIM peripheral Commutation event.
* Input          :- TIMx: where x can be  1 or 8 to select the TIMx peripheral
*                 - NewState: new state of the Commutation event.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SelectCOM(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_TIM_18_PERIPH(TIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the COM Bit */
    TIMx->CR2 |= CR2_CCUS_Set;
  }
  else
  {
    /* Reset the COM Bit */
    TIMx->CR2 &= CR2_CCUS_Reset;
  }
}

/*******************************************************************************
* Function Name  : TIM_SelectCCDMA
* Description    : Selects the TIMx peripheral Capture Compare DMA source.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - NewState: new state of the Capture Compare DMA source
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SelectCCDMA(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the CCDS Bit */
    TIMx->CR2 |= CR2_CCDS_Set;
  }
  else
  {
    /* Reset the CCDS Bit */
    TIMx->CR2 &= CR2_CCDS_Reset;
  }
}

/*******************************************************************************
* Function Name  : TIM_CCPreloadControl
* Description    : Sets or Resets the TIM peripheral Capture Compare Preload 
*                  Control bit.
* Input          :- TIMx: where x can be  1 or 8 to select the TIMx peripheral
*                 - NewState: new state of the Capture Compare Preload Control bit
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_CCPreloadControl(TIM_TypeDef* TIMx, FunctionalState NewState)
{ 
  /* Check the parameters */
  assert_param(IS_TIM_18_PERIPH(TIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the CCPC Bit */
    TIMx->CR2 |= CR2_CCPC_Set;
  }
  else
  {
    /* Reset the CCPC Bit */
    TIMx->CR2 &= CR2_CCPC_Reset;
  }
}

/*******************************************************************************
* Function Name  : TIM_OC1PreloadConfig
* Description    : Enables or disables the TIMx peripheral Preload register on CCR1.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCPreload: new state of the TIMx peripheral Preload
*                    register
*                    This parameter can be one of the following values:
*                       - TIM_OCPreload_Enable
*                       - TIM_OCPreload_Disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC1PreloadConfig(TIM_TypeDef* TIMx, u16 TIM_OCPreload)
{
  u16 tmpccmr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCPRELOAD_STATE(TIM_OCPreload));

  tmpccmr1 = TIMx->CCMR1;

  /* Reset the OC1PE Bit */
  tmpccmr1 &= CCMR_OC13PE_Reset;

  /* Enable or Disable the Output Compare Preload feature */
  tmpccmr1 |= TIM_OCPreload;

  /* Write to TIMx CCMR1 register */
  TIMx->CCMR1 = tmpccmr1;
}

/*******************************************************************************
* Function Name  : TIM_OC2PreloadConfig
* Description    : Enables or disables the TIMx peripheral Preload register on CCR2.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCPreload: new state of the TIMx peripheral Preload
*                    register
*                    This parameter can be one of the following values:
*                       - TIM_OCPreload_Enable
*                       - TIM_OCPreload_Disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC2PreloadConfig(TIM_TypeDef* TIMx, u16 TIM_OCPreload)
{
  u16 tmpccmr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCPRELOAD_STATE(TIM_OCPreload));

  tmpccmr1 = TIMx->CCMR1;

  /* Reset the OC2PE Bit */
  tmpccmr1 &= CCMR_OC24PE_Reset;

  /* Enable or Disable the Output Compare Preload feature */
  tmpccmr1 |= (u16)(TIM_OCPreload << 8);

  /* Write to TIMx CCMR1 register */
  TIMx->CCMR1 = tmpccmr1;
}

/*******************************************************************************
* Function Name  : TIM_OC3PreloadConfig
* Description    : Enables or disables the TIMx peripheral Preload register on CCR3.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCPreload: new state of the TIMx peripheral Preload
*                    register
*                    This parameter can be one of the following values:
*                       - TIM_OCPreload_Enable
*                       - TIM_OCPreload_Disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC3PreloadConfig(TIM_TypeDef* TIMx, u16 TIM_OCPreload)
{
  u16 tmpccmr2 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCPRELOAD_STATE(TIM_OCPreload));

  tmpccmr2 = TIMx->CCMR2;

  /* Reset the OC3PE Bit */
  tmpccmr2 &= CCMR_OC13PE_Reset;

  /* Enable or Disable the Output Compare Preload feature */
  tmpccmr2 |= TIM_OCPreload;

  /* Write to TIMx CCMR2 register */
  TIMx->CCMR2 = tmpccmr2;
}

/*******************************************************************************
* Function Name  : TIM_OC4PreloadConfig
* Description    : Enables or disables the TIMx peripheral Preload register on CCR4.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCPreload: new state of the TIMx peripheral Preload
*                    register
*                    This parameter can be one of the following values:
*                       - TIM_OCPreload_Enable
*                       - TIM_OCPreload_Disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC4PreloadConfig(TIM_TypeDef* TIMx, u16 TIM_OCPreload)
{
  u16 tmpccmr2 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCPRELOAD_STATE(TIM_OCPreload));

  tmpccmr2 = TIMx->CCMR2;

  /* Reset the OC4PE Bit */
  tmpccmr2 &= CCMR_OC24PE_Reset;

  /* Enable or Disable the Output Compare Preload feature */
  tmpccmr2 |= (u16)(TIM_OCPreload << 8);

  /* Write to TIMx CCMR2 register */
  TIMx->CCMR2 = tmpccmr2;
}

/*******************************************************************************
* Function Name  : TIM_OC1FastConfig
* Description    : Configures the TIMx Output Compare 1 Fast feature.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCFast: new state of the Output Compare Fast Enable Bit.
*                    This parameter can be one of the following values:
*                       - TIM_OCFast_Enable: TIM output compare fast enable
*                       - TIM_OCFast_Disable: TIM output compare fast disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC1FastConfig(TIM_TypeDef* TIMx, u16 TIM_OCFast)
{
  u16 tmpccmr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCFAST_STATE(TIM_OCFast));

  /* Get the TIMx CCMR1 register value */
  tmpccmr1 = TIMx->CCMR1;

  /* Reset the OC1FE Bit */
  tmpccmr1 &= CCMR_OC13FE_Reset;

  /* Enable or Disable the Output Compare Fast Bit */
  tmpccmr1 |= TIM_OCFast;

  /* Write to TIMx CCMR1 */
  TIMx->CCMR1 = tmpccmr1;
}

/*******************************************************************************
* Function Name  : TIM_OC2FastConfig
* Description    : Configures the TIMx Output Compare 2 Fast feature.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCFast: new state of the Output Compare Fast Enable Bit.
*                    This parameter can be one of the following values:
*                       - TIM_OCFast_Enable: TIM output compare fast enable
*                       - TIM_OCFast_Disable: TIM output compare fast disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC2FastConfig(TIM_TypeDef* TIMx, u16 TIM_OCFast)
{
  u16 tmpccmr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCFAST_STATE(TIM_OCFast));

  /* Get the TIMx CCMR1 register value */
  tmpccmr1 = TIMx->CCMR1;

  /* Reset the OC2FE Bit */
  tmpccmr1 &= CCMR_OC24FE_Reset;

  /* Enable or Disable the Output Compare Fast Bit */
  tmpccmr1 |= (u16)(TIM_OCFast << 8);

  /* Write to TIMx CCMR1 */
  TIMx->CCMR1 = tmpccmr1;
}

/*******************************************************************************
* Function Name  : TIM_OC3FastConfig
* Description    : Configures the TIMx Output Compare 3 Fast feature.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCFast: new state of the Output Compare Fast Enable Bit.
*                    This parameter can be one of the following values:
*                       - TIM_OCFast_Enable: TIM output compare fast enable
*                       - TIM_OCFast_Disable: TIM output compare fast disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC3FastConfig(TIM_TypeDef* TIMx, u16 TIM_OCFast)
{
  u16 tmpccmr2 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCFAST_STATE(TIM_OCFast));

  /* Get the TIMx CCMR2 register value */
  tmpccmr2 = TIMx->CCMR2;

  /* Reset the OC3FE Bit */
  tmpccmr2 &= CCMR_OC13FE_Reset;

  /* Enable or Disable the Output Compare Fast Bit */
  tmpccmr2 |= TIM_OCFast;

  /* Write to TIMx CCMR2 */
  TIMx->CCMR2 = tmpccmr2;
}

/*******************************************************************************
* Function Name  : TIM_OC4FastConfig
* Description    : Configures the TIMx Output Compare 4 Fast feature.
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCFast: new state of the Output Compare Fast Enable Bit.
*                    This parameter can be one of the following values:
*                       - TIM_OCFast_Enable: TIM output compare fast enable
*                       - TIM_OCFast_Disable: TIM output compare fast disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC4FastConfig(TIM_TypeDef* TIMx, u16 TIM_OCFast)
{
  u16 tmpccmr2 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCFAST_STATE(TIM_OCFast));

  /* Get the TIMx CCMR2 register value */
  tmpccmr2 = TIMx->CCMR2;

  /* Reset the OC4FE Bit */
  tmpccmr2 &= CCMR_OC24FE_Reset;

  /* Enable or Disable the Output Compare Fast Bit */
  tmpccmr2 |= (u16)(TIM_OCFast << 8);

  /* Write to TIMx CCMR2 */
  TIMx->CCMR2 = tmpccmr2;
}

/*******************************************************************************
* Function Name  : TIM_ClearOC1Ref
* Description    : Clears or safeguards the OCREF1 signal on an external event
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCClear: new state of the Output Compare Clear Enable Bit.
*                    This parameter can be one of the following values:
*                       - TIM_OCClear_Enable: TIM Output clear enable
*                       - TIM_OCClear_Disable: TIM Output clear disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ClearOC1Ref(TIM_TypeDef* TIMx, u16 TIM_OCClear)
{
  u16 tmpccmr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCCLEAR_STATE(TIM_OCClear));

  tmpccmr1 = TIMx->CCMR1;

  /* Reset the OC1CE Bit */
  tmpccmr1 &= CCMR_OC13CE_Reset;

  /* Enable or Disable the Output Compare Clear Bit */
  tmpccmr1 |= TIM_OCClear;

  /* Write to TIMx CCMR1 register */
  TIMx->CCMR1 = tmpccmr1;
}

/*******************************************************************************
* Function Name  : TIM_ClearOC2Ref
* Description    : Clears or safeguards the OCREF2 signal on an external event
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCClear: new state of the Output Compare Clear Enable Bit.
*                    This parameter can be one of the following values:
*                       - TIM_OCClear_Enable: TIM Output clear enable
*                       - TIM_OCClear_Disable: TIM Output clear disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ClearOC2Ref(TIM_TypeDef* TIMx, u16 TIM_OCClear)
{
  u16 tmpccmr1 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCCLEAR_STATE(TIM_OCClear));

  tmpccmr1 = TIMx->CCMR1;

  /* Reset the OC2CE Bit */
  tmpccmr1 &= CCMR_OC24CE_Reset;

  /* Enable or Disable the Output Compare Clear Bit */
  tmpccmr1 |= (u16)(TIM_OCClear << 8);

  /* Write to TIMx CCMR1 register */
  TIMx->CCMR1 = tmpccmr1;
}

/*******************************************************************************
* Function Name  : TIM_ClearOC3Ref
* Description    : Clears or safeguards the OCREF3 signal on an external event
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCClear: new state of the Output Compare Clear Enable Bit.
*                    This parameter can be one of the following values:
*                       - TIM_OCClear_Enable: TIM Output clear enable
*                       - TIM_OCClear_Disable: TIM Output clear disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ClearOC3Ref(TIM_TypeDef* TIMx, u16 TIM_OCClear)
{
  u16 tmpccmr2 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCCLEAR_STATE(TIM_OCClear));

  tmpccmr2 = TIMx->CCMR2;

  /* Reset the OC3CE Bit */
  tmpccmr2 &= CCMR_OC13CE_Reset;

  /* Enable or Disable the Output Compare Clear Bit */
  tmpccmr2 |= TIM_OCClear;

  /* Write to TIMx CCMR2 register */
  TIMx->CCMR2 = tmpccmr2;
}

/*******************************************************************************
* Function Name  : TIM_ClearOC4Ref
* Description    : Clears or safeguards the OCREF4 signal on an external event
* Input          : - TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCClear: new state of the Output Compare Clear Enable Bit.
*                    This parameter can be one of the following values:
*                       - TIM_OCClear_Enable: TIM Output clear enable
*                       - TIM_OCClear_Disable: TIM Output clear disable
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ClearOC4Ref(TIM_TypeDef* TIMx, u16 TIM_OCClear)
{
  u16 tmpccmr2 = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OCCLEAR_STATE(TIM_OCClear));

  tmpccmr2 = TIMx->CCMR2;

  /* Reset the OC4CE Bit */
  tmpccmr2 &= CCMR_OC24CE_Reset;

  /* Enable or Disable the Output Compare Clear Bit */
  tmpccmr2 |= (u16)(TIM_OCClear << 8);

  /* Write to TIMx CCMR2 register */
  TIMx->CCMR2 = tmpccmr2;
}

/*******************************************************************************
* Function Name  : TIM_OC1PolarityConfig
* Description    : Configures the TIMx channel 1 polarity.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCPolarity: specifies the OC1 Polarity
*                    This parmeter can be one of the following values:
*                       - TIM_OCPolarity_High: Output Compare active high
*                       - TIM_OCPolarity_Low: Output Compare active low
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC1PolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCPolarity)
{
  u16 tmpccer = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OC_POLARITY(TIM_OCPolarity));

  tmpccer = TIMx->CCER;

  /* Set or Reset the CC1P Bit */
  tmpccer &= CCER_CC1P_Reset;
  tmpccer |= TIM_OCPolarity;

  /* Write to TIMx CCER register */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_OC1NPolarityConfig
* Description    : Configures the TIMx Channel 1N polarity.
* Input          : - TIMx: where x can be 1 or 8 to select the TIM peripheral.
*                  - TIM_OCNPolarity: specifies the OC1N Polarity
*                    This parmeter can be one of the following values:
*                       - TIM_OCNPolarity_High: Output Compare active high
*                       - TIM_OCNPolarity_Low: Output Compare active low
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC1NPolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCNPolarity)
{
  u16 tmpccer = 0;

  /* Check the parameters */
  assert_param(IS_TIM_18_PERIPH(TIMx));
  assert_param(IS_TIM_OCN_POLARITY(TIM_OCNPolarity));
   
  tmpccer = TIMx->CCER;

  /* Set or Reset the CC1NP Bit */
  tmpccer &= CCER_CC1NP_Reset;
  tmpccer |= TIM_OCNPolarity;

  /* Write to TIMx CCER register */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_OC2PolarityConfig
* Description    : Configures the TIMx channel 2 polarity.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCPolarity: specifies the OC2 Polarity
*                    This parmeter can be one of the following values:
*                       - TIM_OCPolarity_High: Output Compare active high
*                       - TIM_OCPolarity_Low: Output Compare active low
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC2PolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCPolarity)
{
  u16 tmpccer = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OC_POLARITY(TIM_OCPolarity));

  tmpccer = TIMx->CCER;

  /* Set or Reset the CC2P Bit */
  tmpccer &= CCER_CC2P_Reset;
  tmpccer |= (u16)(TIM_OCPolarity << 4);

  /* Write to TIMx CCER register */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_OC2NPolarityConfig
* Description    : Configures the TIMx Channel 2N polarity.
* Input          : - TIMx: where x can be 1 or 8 to select the TIM peripheral.
*                  - TIM_OCNPolarity: specifies the OC2N Polarity
*                    This parmeter can be one of the following values:
*                       - TIM_OCNPolarity_High: Output Compare active high
*                       - TIM_OCNPolarity_Low: Output Compare active low
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC2NPolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCNPolarity)
{
  u16 tmpccer = 0;

  /* Check the parameters */
  assert_param(IS_TIM_18_PERIPH(TIMx));
  assert_param(IS_TIM_OCN_POLARITY(TIM_OCNPolarity));
  
  tmpccer = TIMx->CCER;

  /* Set or Reset the CC2NP Bit */
  tmpccer &= CCER_CC2NP_Reset;
  tmpccer |= (u16)(TIM_OCNPolarity << 4);

  /* Write to TIMx CCER register */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_OC3PolarityConfig
* Description    : Configures the TIMx channel 3 polarity.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCPolarity: specifies the OC3 Polarity
*                    This parmeter can be one of the following values:
*                       - TIM_OCPolarity_High: Output Compare active high
*                       - TIM_OCPolarity_Low: Output Compare active low
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC3PolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCPolarity)
{
  u16 tmpccer = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OC_POLARITY(TIM_OCPolarity));

  tmpccer = TIMx->CCER;

  /* Set or Reset the CC3P Bit */
  tmpccer &= CCER_CC3P_Reset;
  tmpccer |= (u16)(TIM_OCPolarity << 8);

  /* Write to TIMx CCER register */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_OC3NPolarityConfig
* Description    : Configures the TIMx Channel 3N polarity.
* Input          : - TIMx: where x can be 1 or 8 to select the TIM peripheral.
*                  - TIM_OCNPolarity: specifies the OC3N Polarity
*                    This parmeter can be one of the following values:
*                       - TIM_OCNPolarity_High: Output Compare active high
*                       - TIM_OCNPolarity_Low: Output Compare active low
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC3NPolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCNPolarity)
{
  u16 tmpccer = 0;
 
  /* Check the parameters */
  assert_param(IS_TIM_18_PERIPH(TIMx));
  assert_param(IS_TIM_OCN_POLARITY(TIM_OCNPolarity));
    
  tmpccer = TIMx->CCER;

  /* Set or Reset the CC3NP Bit */
  tmpccer &= CCER_CC3NP_Reset;
  tmpccer |= (u16)(TIM_OCNPolarity << 8);

  /* Write to TIMx CCER register */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_OC4PolarityConfig
* Description    : Configures the TIMx channel 4 polarity.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_OCPolarity: specifies the OC4 Polarity
*                    This parmeter can be one of the following values:
*                       - TIM_OCPolarity_High: Output Compare active high
*                       - TIM_OCPolarity_Low: Output Compare active low
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_OC4PolarityConfig(TIM_TypeDef* TIMx, u16 TIM_OCPolarity)
{
  u16 tmpccer = 0;

  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_OC_POLARITY(TIM_OCPolarity));

  tmpccer = TIMx->CCER;

  /* Set or Reset the CC4P Bit */
  tmpccer &= CCER_CC4P_Reset;
  tmpccer |= (u16)(TIM_OCPolarity << 12);

  /* Write to TIMx CCER register */
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TIM_CCxCmd
* Description    : Enables or disables the TIM Capture Compare Channel x.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM
*                    peripheral.
*                  - TIM_Channel: specifies the TIM Channel
*                    This parmeter can be one of the following values:
*                       - TIM_Channel_1: TIM Channel 1
*                       - TIM_Channel_2: TIM Channel 2
*                       - TIM_Channel_3: TIM Channel 3
*                       - TIM_Channel_4: TIM Channel 4
*                 - TIM_CCx: specifies the TIM Channel CCxE bit new state.
*                   This parameter can be: TIM_CCx_Enable or TIM_CCx_Disable. 
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_CCxCmd(TIM_TypeDef* TIMx, u16 TIM_Channel, u16 TIM_CCx)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_CHANNEL(TIM_Channel));
  assert_param(IS_TIM_CCX(TIM_CCx));

  /* Reset the CCxE Bit */
  TIMx->CCER &= (u16)(~((u16)(CCER_CCE_Set << TIM_Channel)));

  /* Set or reset the CCxE Bit */ 
  TIMx->CCER |=  (u16)(TIM_CCx << TIM_Channel);
}

/*******************************************************************************
* Function Name  : TIM_CCxNCmd
* Description    : Enables or disables the TIM Capture Compare Channel xN.
* Input          :- TIMx: where x can be 1 or 8 to select the TIM peripheral.
*                 - TIM_Channel: specifies the TIM Channel
*                    This parmeter can be one of the following values:
*                       - TIM_Channel_1: TIM Channel 1
*                       - TIM_Channel_2: TIM Channel 2
*                       - TIM_Channel_3: TIM Channel 3
*                 - TIM_CCx: specifies the TIM Channel CCxNE bit new state.
*                   This parameter can be: TIM_CCxN_Enable or TIM_CCxN_Disable. 
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_CCxNCmd(TIM_TypeDef* TIMx, u16 TIM_Channel, u16 TIM_CCxN)
{
  /* Check the parameters */
  assert_param(IS_TIM_18_PERIPH(TIMx));
  assert_param(IS_TIM_COMPLEMENTARY_CHANNEL(TIM_Channel));
  assert_param(IS_TIM_CCXN(TIM_CCxN));

  /* Reset the CCxNE Bit */
  TIMx->CCER &= (u16)(~((u16)(CCER_CCNE_Set << TIM_Channel)));

  /* Set or reset the CCxNE Bit */ 
  TIMx->CCER |=  (u16)(TIM_CCxN << TIM_Channel);
}

/*******************************************************************************
* Function Name  : TIM_SelectOCxM
* Description    : Selects the TIM Ouput Compare Mode.
*                  This function disables the selected channel before changing 
*                  the Ouput Compare Mode. User has to enable this channel using
*                  TIM_CCxCmd and TIM_CCxNCmd functions.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM
*                    peripheral.
*                  - TIM_Channel: specifies the TIM Channel
*                    This parmeter can be one of the following values:
*                       - TIM_Channel_1: TIM Channel 1
*                       - TIM_Channel_2: TIM Channel 2
*                       - TIM_Channel_3: TIM Channel 3
*                       - TIM_Channel_4: TIM Channel 4
*                  - TIM_OCMode: specifies the TIM Output Compare Mode.
*                    This paramter can be one of the following values:
*                       - TIM_OCMode_Timing
*                       - TIM_OCMode_Active
*                       - TIM_OCMode_Toggle
*                       - TIM_OCMode_PWM1
*                       - TIM_OCMode_PWM2
*                       - TIM_ForcedAction_Active
*                       - TIM_ForcedAction_InActive
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SelectOCxM(TIM_TypeDef* TIMx, u16 TIM_Channel, u16 TIM_OCMode)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_CHANNEL(TIM_Channel));
  assert_param(IS_TIM_OCM(TIM_OCMode));
  
  /* Disable the Channel: Reset the CCxE Bit */
  TIMx->CCER &= (u16)(~((u16)(CCER_CCE_Set << TIM_Channel)));

  if((TIM_Channel == TIM_Channel_1) ||(TIM_Channel == TIM_Channel_3))
  {
    /* Reset the OCxM bits in the CCMRx register */
    *((vu32 *)((*(u32*)&TIMx) + CCMR_Offset + (TIM_Channel>>1))) &= CCMR_OC13M_Mask;
   
    /* Configure the OCxM bits in the CCMRx register */
    *((vu32 *)((*(u32*)&TIMx) + CCMR_Offset + (TIM_Channel>>1))) |= TIM_OCMode;

  }
  else
  {
    /* Reset the OCxM bits in the CCMRx register */
    *((vu32 *)((*(u32*)&TIMx) + CCMR_Offset + ((u16)(TIM_Channel - 4)>> 1))) &= CCMR_OC24M_Mask;
    
    /* Configure the OCxM bits in the CCMRx register */
    *((vu32 *)((*(u32*)&TIMx) + CCMR_Offset + ((u16)(TIM_Channel - 4)>> 1))) |= (u16)(TIM_OCMode << 8);
  }
}

/*******************************************************************************
* Function Name  : TIM_UpdateDisableConfig
* Description    : Enables or Disables the TIMx Update event.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - NewState: new state of the TIMx UDIS bit
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_UpdateDisableConfig(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the Update Disable Bit */
    TIMx->CR1 |= CR1_UDIS_Set;
  }
  else
  {
    /* Reset the Update Disable Bit */
    TIMx->CR1 &= CR1_UDIS_Reset;
  }
}

/*******************************************************************************
* Function Name  : TIM_UpdateRequestConfig
* Description    : Configures the TIMx Update Request Interrupt source.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - TIM_UpdateSource: specifies the Update source.
*                    This parameter can be one of the following values:
*                       - TIM_UpdateSource_Regular
*                       - TIM_UpdateSource_Global
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_UpdateRequestConfig(TIM_TypeDef* TIMx, u16 TIM_UpdateSource)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_UPDATE_SOURCE(TIM_UpdateSource));

  if (TIM_UpdateSource != TIM_UpdateSource_Global)
  {
    /* Set the URS Bit */
    TIMx->CR1 |= CR1_URS_Set;
  }
  else
  {
    /* Reset the URS Bit */
    TIMx->CR1 &= CR1_URS_Reset;
  }
}

/*******************************************************************************
* Function Name  : TIM_SelectHallSensor
* Description    : Enables or disables the TIMx’s Hall sensor interface.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
*                  - NewState: new state of the TIMx Hall sensor interface.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SelectHallSensor(TIM_TypeDef* TIMx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Set the TI1S Bit */
    TIMx->CR2 |= CR2_TI1S_Set;
  }
  else
  {
    /* Reset the TI1S Bit */
    TIMx->CR2 &= CR2_TI1S_Reset;
  }
}

/*******************************************************************************
* Function Name  : TIM_SelectOnePulseMode
* Description    : Selects the TIMx’s One Pulse Mode.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - TIM_OPMode: specifies the OPM Mode to be used.
*                    This parameter can be one of the following values:
*                       - TIM_OPMode_Single
*                       - TIM_OPMode_Repetitive
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SelectOnePulseMode(TIM_TypeDef* TIMx, u16 TIM_OPMode)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_OPM_MODE(TIM_OPMode));

  /* Reset the OPM Bit */
  TIMx->CR1 &= CR1_OPM_Reset;

  /* Configure the OPM Mode */
  TIMx->CR1 |= TIM_OPMode;
}

/*******************************************************************************
* Function Name  : TIM_SelectOutputTrigger
* Description    : Selects the TIMx Trigger Output Mode.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - TIM_TRGOSource: specifies the Trigger Output source.
*                    This paramter can be as follow:
*                      1/ For TIM1 to TIM8:
*                       - TIM_TRGOSource_Reset 
*                       - TIM_TRGOSource_Enable
*                       - TIM_TRGOSource_Update
*                      2/ These parameters are available for all TIMx except 
*                         TIM6 and TIM7:
*                       - TIM_TRGOSource_OC1
*                       - TIM_TRGOSource_OC1Ref
*                       - TIM_TRGOSource_OC2Ref
*                       - TIM_TRGOSource_OC3Ref
*                       - TIM_TRGOSource_OC4Ref
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, u16 TIM_TRGOSource)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_TRGO_SOURCE(TIM_TRGOSource));
  assert_param(IS_TIM_PERIPH_TRGO(TIMx, TIM_TRGOSource));

  /* Reset the MMS Bits */
  TIMx->CR2 &= CR2_MMS_Mask;

  /* Select the TRGO source */
  TIMx->CR2 |=  TIM_TRGOSource;
}

/*******************************************************************************
* Function Name  : TIM_SelectSlaveMode
* Description    : Selects the TIMx Slave Mode.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_SlaveMode: specifies the Timer Slave Mode.
*                    This paramter can be one of the following values:
*                       - TIM_SlaveMode_Reset
*                       - TIM_SlaveMode_Gated
*                       - TIM_SlaveMode_Trigger
*                       - TIM_SlaveMode_External1
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, u16 TIM_SlaveMode)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_SLAVE_MODE(TIM_SlaveMode));

  /* Reset the SMS Bits */
  TIMx->SMCR &= SMCR_SMS_Mask;

  /* Select the Slave Mode */
  TIMx->SMCR |= TIM_SlaveMode;
}

/*******************************************************************************
* Function Name  : TIM_SelectMasterSlaveMode
* Description    : Sets or Resets the TIMx Master/Slave Mode.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_MasterSlaveMode: specifies the Timer Master Slave Mode.
*                    This paramter can be one of the following values:
*                       - TIM_MasterSlaveMode_Enable: synchronization between the
*                         current timer and its slaves (through TRGO).
*                       - TIM_MasterSlaveMode_Disable: No action
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, u16 TIM_MasterSlaveMode)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_MSM_STATE(TIM_MasterSlaveMode));

  /* Reset the MSM Bit */
  TIMx->SMCR &= SMCR_MSM_Reset;
  
  /* Set or Reset the MSM Bit */
  TIMx->SMCR |= TIM_MasterSlaveMode;
}

/*******************************************************************************
* Function Name  : TIM_SetCounter
* Description    : Sets the TIMx Counter Register value
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - Counter: specifies the Counter register new value.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetCounter(TIM_TypeDef* TIMx, u16 Counter)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));

  /* Set the Counter Register value */
  TIMx->CNT = Counter;
}

/*******************************************************************************
* Function Name  : TIM_SetAutoreload
* Description    : Sets the TIMx Autoreload Register value
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - Autoreload: specifies the Autoreload register new value.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetAutoreload(TIM_TypeDef* TIMx, u16 Autoreload)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));

  /* Set the Autoreload Register value */
  TIMx->ARR = Autoreload;
}

/*******************************************************************************
* Function Name  : TIM_SetCompare1
* Description    : Sets the TIMx Capture Compare1 Register value
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - Compare1: specifies the Capture Compare1 register new value.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetCompare1(TIM_TypeDef* TIMx, u16 Compare1)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));

  /* Set the Capture Compare1 Register value */
  TIMx->CCR1 = Compare1;
}

/*******************************************************************************
* Function Name  : TIM_SetCompare2
* Description    : Sets the TIMx Capture Compare2 Register value
* Input          :  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                   peripheral.
*                  - Compare2: specifies the Capture Compare2 register new value.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetCompare2(TIM_TypeDef* TIMx, u16 Compare2)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));

  /* Set the Capture Compare2 Register value */
  TIMx->CCR2 = Compare2;
}

/*******************************************************************************
* Function Name  : TIM_SetCompare3
* Description    : Sets the TIMx Capture Compare3 Register value
* Input          :  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                   peripheral.
*                  - Compare3: specifies the Capture Compare3 register new value.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetCompare3(TIM_TypeDef* TIMx, u16 Compare3)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));

  /* Set the Capture Compare3 Register value */
  TIMx->CCR3 = Compare3;
}

/*******************************************************************************
* Function Name  : TIM_SetCompare4
* Description    : Sets the TIMx Capture Compare4 Register value
* Input          :  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                   peripheral.
*                  - Compare4: specifies the Capture Compare4 register new value.
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetCompare4(TIM_TypeDef* TIMx, u16 Compare4)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));

  /* Set the Capture Compare4 Register value */
  TIMx->CCR4 = Compare4;
}

/*******************************************************************************
* Function Name  : TIM_SetIC1Prescaler
* Description    : Sets the TIMx Input Capture 1 prescaler.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICPSC: specifies the Input Capture1 prescaler
*                    new value.
*                    This parameter can be one of the following values:
*                       - TIM_ICPSC_DIV1: no prescaler
*                       - TIM_ICPSC_DIV2: capture is done once every 2 events
*                       - TIM_ICPSC_DIV4: capture is done once every 4 events
*                       - TIM_ICPSC_DIV8: capture is done once every 8 events
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetIC1Prescaler(TIM_TypeDef* TIMx, u16 TIM_ICPSC)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_IC_PRESCALER(TIM_ICPSC));

  /* Reset the IC1PSC Bits */
  TIMx->CCMR1 &= CCMR_IC13PSC_Mask;

  /* Set the IC1PSC value */
  TIMx->CCMR1 |= TIM_ICPSC;
}

/*******************************************************************************
* Function Name  : TIM_SetIC2Prescaler
* Description    : Sets the TIMx Input Capture 2 prescaler.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICPSC: specifies the Input Capture2 prescaler
*                    new value.
*                    This parameter can be one of the following values:
*                       - TIM_ICPSC_DIV1: no prescaler
*                       - TIM_ICPSC_DIV2: capture is done once every 2 events
*                       - TIM_ICPSC_DIV4: capture is done once every 4 events
*                       - TIM_ICPSC_DIV8: capture is done once every 8 events
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetIC2Prescaler(TIM_TypeDef* TIMx, u16 TIM_ICPSC)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_IC_PRESCALER(TIM_ICPSC));

  /* Reset the IC2PSC Bits */
  TIMx->CCMR1 &= CCMR_IC24PSC_Mask;

  /* Set the IC2PSC value */
  TIMx->CCMR1 |= (u16)(TIM_ICPSC << 8);
}

/*******************************************************************************
* Function Name  : TIM_SetIC3Prescaler
* Description    : Sets the TIMx Input Capture 3 prescaler.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICPSC: specifies the Input Capture3 prescaler
*                    new value.
*                    This parameter can be one of the following values:
*                       - TIM_ICPSC_DIV1: no prescaler
*                       - TIM_ICPSC_DIV2: capture is done once every 2 events
*                       - TIM_ICPSC_DIV4: capture is done once every 4 events
*                       - TIM_ICPSC_DIV8: capture is done once every 8 events
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetIC3Prescaler(TIM_TypeDef* TIMx, u16 TIM_ICPSC)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_IC_PRESCALER(TIM_ICPSC));

  /* Reset the IC3PSC Bits */
  TIMx->CCMR2 &= CCMR_IC13PSC_Mask;

  /* Set the IC3PSC value */
  TIMx->CCMR2 |= TIM_ICPSC;
}

/*******************************************************************************
* Function Name  : TIM_SetIC4Prescaler
* Description    : Sets the TIMx Input Capture 4 prescaler.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICPSC: specifies the Input Capture4 prescaler
*                    new value.
*                    This parameter can be one of the following values:
*                      - TIM_ICPSC_DIV1: no prescaler
*                      - TIM_ICPSC_DIV2: capture is done once every 2 events
*                      - TIM_ICPSC_DIV4: capture is done once every 4 events
*                      - TIM_ICPSC_DIV8: capture is done once every 8 events
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetIC4Prescaler(TIM_TypeDef* TIMx, u16 TIM_ICPSC)
{  
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_IC_PRESCALER(TIM_ICPSC));

  /* Reset the IC4PSC Bits */
  TIMx->CCMR2 &= CCMR_IC24PSC_Mask;

  /* Set the IC4PSC value */
  TIMx->CCMR2 |= (u16)(TIM_ICPSC << 8);
}

/*******************************************************************************
* Function Name  : TIM_SetClockDivision
* Description    : Sets the TIMx Clock Division value.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_CKD: specifies the clock division value.
*                    This parameter can be one of the following value:
*                       - TIM_CKD_DIV1: TDTS = Tck_tim
*                       - TIM_CKD_DIV2: TDTS = 2*Tck_tim
*                       - TIM_CKD_DIV4: TDTS = 4*Tck_tim
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_SetClockDivision(TIM_TypeDef* TIMx, u16 TIM_CKD)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));
  assert_param(IS_TIM_CKD_DIV(TIM_CKD));

  /* Reset the CKD Bits */
  TIMx->CR1 &= CR1_CKD_Mask;

  /* Set the CKD value */
  TIMx->CR1 |= TIM_CKD;
}
/*******************************************************************************
* Function Name  : TIM_GetCapture1
* Description    : Gets the TIMx Input Capture 1 value.
* Input          :  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                   peripheral.
* Output         : None
* Return         : Capture Compare 1 Register value.
*******************************************************************************/
u16 TIM_GetCapture1(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));

  /* Get the Capture 1 Register value */
  return TIMx->CCR1;
}

/*******************************************************************************
* Function Name  : TIM_GetCapture2
* Description    : Gets the TIMx Input Capture 2 value.
* Input          :  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                   peripheral.
* Output         : None
* Return         : Capture Compare 2 Register value.
*******************************************************************************/
u16 TIM_GetCapture2(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));

  /* Get the Capture 2 Register value */
  return TIMx->CCR2;
}

/*******************************************************************************
* Function Name  : TIM_GetCapture3
* Description    : Gets the TIMx Input Capture 3 value.
* Input          :  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                   peripheral.
* Output         : None
* Return         : Capture Compare 3 Register value.
*******************************************************************************/
u16 TIM_GetCapture3(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx)); 

  /* Get the Capture 3 Register value */
  return TIMx->CCR3;
}

/*******************************************************************************
* Function Name  : TIM_GetCapture4
* Description    : Gets the TIMx Input Capture 4 value.
* Input          :  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                   peripheral.
* Output         : None
* Return         : Capture Compare 4 Register value.
*******************************************************************************/
u16 TIM_GetCapture4(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_123458_PERIPH(TIMx));

  /* Get the Capture 4 Register value */
  return TIMx->CCR4;
}

/*******************************************************************************
* Function Name  : TIM_GetCounter
* Description    : Gets the TIMx Counter value.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
* Output         : None
* Return         : Counter Register value.
*******************************************************************************/
u16 TIM_GetCounter(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));

  /* Get the Counter Register value */
  return TIMx->CNT;
}

/*******************************************************************************
* Function Name  : TIM_GetPrescaler
* Description    : Gets the TIMx Prescaler value.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
* Output         : None
* Return         : Prescaler Register value.
*******************************************************************************/
u16 TIM_GetPrescaler(TIM_TypeDef* TIMx)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));

  /* Get the Prescaler Register value */
  return TIMx->PSC;
}

/*******************************************************************************
* Function Name  : TIM_GetFlagStatus
* Description    : Checks whether the specified TIM flag is set or not.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - TIM_FLAG: specifies the flag to check.
*                    This parameter can be one of the following values:
*                       - TIM_FLAG_Update: TIM update Flag
*                       - TIM_FLAG_CC1: TIM Capture Compare 1 Flag
*                       - TIM_FLAG_CC2: TIM Capture Compare 2 Flag
*                       - TIM_FLAG_CC3: TIM Capture Compare 3 Flag
*                       - TIM_FLAG_CC4: TIM Capture Compare 4 Flag
*                       - TIM_FLAG_COM: TIM Commutation Flag
*                       - TIM_FLAG_Trigger: TIM Trigger Flag
*                       - TIM_FLAG_Break: TIM Break Flag
*                       - TIM_FLAG_CC1OF: TIM Capture Compare 1 overcapture Flag
*                       - TIM_FLAG_CC2OF: TIM Capture Compare 2 overcapture Flag
*                       - TIM_FLAG_CC3OF: TIM Capture Compare 3 overcapture Flag
*                       - TIM_FLAG_CC4OF: TIM Capture Compare 4 overcapture Flag
* Output         : None
* Return         : The new state of TIM_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus TIM_GetFlagStatus(TIM_TypeDef* TIMx, u16 TIM_FLAG)
{ 
  ITStatus bitstatus = RESET;  

  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_GET_FLAG(TIM_FLAG));
  assert_param(IS_TIM_PERIPH_FLAG(TIMx, TIM_FLAG));
  
  if ((TIMx->SR & TIM_FLAG) != (u16)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : TIM_ClearFlag
* Description    : Clears the TIMx's pending flags.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - TIM_FLAG: specifies the flag bit to clear.
*                    This parameter can be any combination of the following values:
*                       - TIM_FLAG_Update: TIM update Flag
*                       - TIM_FLAG_CC1: TIM Capture Compare 1 Flag
*                       - TIM_FLAG_CC2: TIM Capture Compare 2 Flag
*                       - TIM_FLAG_CC3: TIM Capture Compare 3 Flag
*                       - TIM_FLAG_CC4: TIM Capture Compare 4 Flag
*                       - TIM_FLAG_COM: TIM Commutation Flag
*                       - TIM_FLAG_Trigger: TIM Trigger Flag
*                       - TIM_FLAG_Break: TIM Break Flag
*                       - TIM_FLAG_CC1OF: TIM Capture Compare 1 overcapture Flag
*                       - TIM_FLAG_CC2OF: TIM Capture Compare 2 overcapture Flag
*                       - TIM_FLAG_CC3OF: TIM Capture Compare 3 overcapture Flag
*                       - TIM_FLAG_CC4OF: TIM Capture Compare 4 overcapture Flag
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ClearFlag(TIM_TypeDef* TIMx, u16 TIM_FLAG)
{  
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_CLEAR_FLAG(TIMx, TIM_FLAG));
   
  /* Clear the flags */
  TIMx->SR = (u16)~TIM_FLAG;
}

/*******************************************************************************
* Function Name  : TIM_GetITStatus
* Description    : Checks whether the TIM interrupt has occurred or not.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - TIM_IT: specifies the TIM interrupt source to check.
*                    This parameter can be one of the following values:
*                       - TIM_IT_Update: TIM update Interrupt source
*                       - TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
*                       - TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
*                       - TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
*                       - TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
*                       - TIM_IT_COM: TIM Commutation Interrupt
*                         source
*                       - TIM_IT_Trigger: TIM Trigger Interrupt source
*                       - TIM_IT_Break: TIM Break Interrupt source
* Output         : None
* Return         : The new state of the TIM_IT(SET or RESET).
*******************************************************************************/
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, u16 TIM_IT)
{
  ITStatus bitstatus = RESET;  
  u16 itstatus = 0x0, itenable = 0x0;

  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_GET_IT(TIM_IT));
  assert_param(IS_TIM_PERIPH_IT(TIMx, TIM_IT));
   
  itstatus = TIMx->SR & TIM_IT;
  
  itenable = TIMx->DIER & TIM_IT;

  if ((itstatus != (u16)RESET) && (itenable != (u16)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : TIM_ClearITPendingBit
* Description    : Clears the TIMx's interrupt pending bits.
* Input          : - TIMx: where x can be 1 to 8 to select the TIM peripheral.
*                  - TIM_IT: specifies the pending bit to clear.
*                    This parameter can be any combination of the following values:
*                       - TIM_IT_Update: TIM1 update Interrupt source
*                       - TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
*                       - TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
*                       - TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
*                       - TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
*                       - TIM_IT_COM: TIM Commutation Interrupt
*                         source
*                       - TIM_IT_Trigger: TIM Trigger Interrupt source
*                       - TIM_IT_Break: TIM Break Interrupt source
* Output         : None
* Return         : None
*******************************************************************************/
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, u16 TIM_IT)
{
  /* Check the parameters */
  assert_param(IS_TIM_ALL_PERIPH(TIMx));
  assert_param(IS_TIM_PERIPH_IT(TIMx, TIM_IT));

  /* Clear the IT pending Bit */
  TIMx->SR = (u16)~TIM_IT;
}

/*******************************************************************************
* Function Name  : TI1_Config
* Description    : Configure the TI1 as Input.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICPolarity : The Input Polarity.
*                    This parameter can be one of the following values:
*                       - TIM_ICPolarity_Rising
*                       - TIM_ICPolarity_Falling
*                  - TIM_ICSelection: specifies the input to be used.
*                    This parameter can be one of the following values:
*                       - TIM_ICSelection_DirectTI: TIM Input 1 is selected to
*                         be connected to IC1.
*                       - TIM_ICSelection_IndirectTI: TIM Input 1 is selected to
*                         be connected to IC2.
*                       - TIM_ICSelection_TRC: TIM Input 1 is selected to be
*                         connected to TRC.
*                  - TIM_ICFilter: Specifies the Input Capture Filter.
*                    This parameter must be a value between 0x00 and 0x0F.
* Output         : None
* Return         : None
*******************************************************************************/
static void TI1_Config(TIM_TypeDef* TIMx, u16 TIM_ICPolarity, u16 TIM_ICSelection,
                       u16 TIM_ICFilter)
{
  u16 tmpccmr1 = 0, tmpccer = 0;

  /* Disable the Channel 1: Reset the CC1E Bit */
  TIMx->CCER &= CCER_CC1E_Reset;

  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;

  /* Select the Input and set the filter */
  tmpccmr1 &= CCMR_CC13S_Mask & CCMR_IC13F_Mask;
  tmpccmr1 |= TIM_ICSelection | (u16)(TIM_ICFilter << 4);

  /* Select the Polarity and set the CC1E Bit */
  tmpccer &= CCER_CC1P_Reset;
  tmpccer |= TIM_ICPolarity | CCER_CC1E_Set;

  /* Write to TIMx CCMR1 and CCER registers */
  TIMx->CCMR1 = tmpccmr1;
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TI2_Config
* Description    : Configure the TI2 as Input.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICPolarity : The Input Polarity.
*                    This parameter can be one of the following values:
*                       - TIM_ICPolarity_Rising
*                       - TIM_ICPolarity_Falling
*                  - TIM_ICSelection: specifies the input to be used.
*                    This parameter can be one of the following values:
*                       - TIM_ICSelection_DirectTI: TIM Input 2 is selected to
*                         be connected to IC2.
*                       - TIM_ICSelection_IndirectTI: TIM Input 2 is selected to
*                         be connected to IC1.
*                       - TIM_ICSelection_TRC: TIM Input 2 is selected to be
*                         connected to TRC.
*                  - TIM_ICFilter: Specifies the Input Capture Filter.
*                    This parameter must be a value between 0x00 and 0x0F.
* Output         : None
* Return         : None
*******************************************************************************/
static void TI2_Config(TIM_TypeDef* TIMx, u16 TIM_ICPolarity, u16 TIM_ICSelection,
                       u16 TIM_ICFilter)
{
  u16 tmpccmr1 = 0, tmpccer = 0, tmp = 0;

  /* Disable the Channel 2: Reset the CC2E Bit */
  TIMx->CCER &= CCER_CC2E_Reset;

  tmpccmr1 = TIMx->CCMR1;
  tmpccer = TIMx->CCER;
  tmp = (u16)(TIM_ICPolarity << 4);

  /* Select the Input and set the filter */
  tmpccmr1 &= CCMR_CC24S_Mask & CCMR_IC24F_Mask;
  tmpccmr1 |= (u16)(TIM_ICFilter << 12);
  tmpccmr1 |= (u16)(TIM_ICSelection << 8);

  /* Select the Polarity and set the CC2E Bit */
  tmpccer &= CCER_CC2P_Reset;
  tmpccer |=  tmp | CCER_CC2E_Set;

  /* Write to TIMx CCMR1 and CCER registers */
  TIMx->CCMR1 = tmpccmr1 ;
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TI3_Config
* Description    : Configure the TI3 as Input.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICPolarity : The Input Polarity.
*                    This parameter can be one of the following values:
*                       - TIM_ICPolarity_Rising
*                       - TIM_ICPolarity_Falling
*                  - TIM_ICSelection: specifies the input to be used.
*                    This parameter can be one of the following values:
*                       - TIM_ICSelection_DirectTI: TIM Input 3 is selected to
*                         be connected to IC3.
*                       - TIM_ICSelection_IndirectTI: TIM Input 3 is selected to
*                         be connected to IC4.
*                       - TIM_ICSelection_TRC: TIM Input 3 is selected to be
*                         connected to TRC.
*                  - TIM_ICFilter: Specifies the Input Capture Filter.
*                    This parameter must be a value between 0x00 and 0x0F.
* Output         : None
* Return         : None
*******************************************************************************/
static void TI3_Config(TIM_TypeDef* TIMx, u16 TIM_ICPolarity, u16 TIM_ICSelection,
                       u16 TIM_ICFilter)
{
  u16 tmpccmr2 = 0, tmpccer = 0, tmp = 0;

  /* Disable the Channel 3: Reset the CC3E Bit */
  TIMx->CCER &= CCER_CC3E_Reset;

  tmpccmr2 = TIMx->CCMR2;
  tmpccer = TIMx->CCER;
  tmp = (u16)(TIM_ICPolarity << 8);

  /* Select the Input and set the filter */
  tmpccmr2 &= CCMR_CC13S_Mask & CCMR_IC13F_Mask;
  tmpccmr2 |= TIM_ICSelection | (u16)(TIM_ICFilter << 4);

  /* Select the Polarity and set the CC3E Bit */
  tmpccer &= CCER_CC3P_Reset;
  tmpccer |= tmp | CCER_CC3E_Set;

  /* Write to TIMx CCMR2 and CCER registers */
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer;
}

/*******************************************************************************
* Function Name  : TI4_Config
* Description    : Configure the TI1 as Input.
* Input          : - TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM 
*                    peripheral.
*                  - TIM_ICPolarity : The Input Polarity.
*                    This parameter can be one of the following values:
*                       - TIM_ICPolarity_Rising
*                       - TIM_ICPolarity_Falling
*                  - TIM_ICSelection: specifies the input to be used.
*                    This parameter can be one of the following values:
*                       - TIM_ICSelection_DirectTI: TIM Input 4 is selected to
*                         be connected to IC4.
*                       - TIM_ICSelection_IndirectTI: TIM Input 4 is selected to
*                         be connected to IC3.
*                       - TIM_ICSelection_TRC: TIM Input 4 is selected to be
*                         connected to TRC.
*                  - TIM_ICFilter: Specifies the Input Capture Filter.
*                    This parameter must be a value between 0x00 and 0x0F.
* Output         : None
* Return         : None
*******************************************************************************/
static void TI4_Config(TIM_TypeDef* TIMx, u16 TIM_ICPolarity, u16 TIM_ICSelection,
                       u16 TIM_ICFilter)
{
  u16 tmpccmr2 = 0, tmpccer = 0, tmp = 0;

  /* Disable the Channel 4: Reset the CC4E Bit */
  TIMx->CCER &= CCER_CC4E_Reset;

  tmpccmr2 = TIMx->CCMR2;
  tmpccer = TIMx->CCER;
  tmp = (u16)(TIM_ICPolarity << 12);

  /* Select the Input and set the filter */
  tmpccmr2 &= CCMR_CC24S_Mask & CCMR_IC24F_Mask;
  tmpccmr2 |= (u16)(TIM_ICSelection << 8) | (u16)(TIM_ICFilter << 12);

  /* Select the Polarity and set the CC4E Bit */
  tmpccer &= CCER_CC4P_Reset;
  tmpccer |= tmp | CCER_CC4E_Set;

  /* Write to TIMx CCMR2 and CCER registers */
  TIMx->CCMR2 = tmpccmr2;
  TIMx->CCER = tmpccer ;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_usart.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the USART firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* USART UE Mask */
#define CR1_UE_Set                ((u16)0x2000)  /* USART Enable Mask */
#define CR1_UE_Reset              ((u16)0xDFFF)  /* USART Disable Mask */

/* USART WakeUp Method  */
#define CR1_WAKE_Mask             ((u16)0xF7FF)  /* USART WakeUp Method Mask */

/* USART RWU Mask */
#define CR1_RWU_Set               ((u16)0x0002)  /* USART mute mode Enable Mask */
#define CR1_RWU_Reset             ((u16)0xFFFD)  /* USART mute mode Enable Mask */

#define CR1_SBK_Set               ((u16)0x0001)  /* USART Break Character send Mask */

#define CR1_CLEAR_Mask            ((u16)0xE9F3)  /* USART CR1 Mask */

#define CR2_Address_Mask          ((u16)0xFFF0)  /* USART address Mask */

/* USART LIN Mask */
#define CR2_LINEN_Set              ((u16)0x4000)  /* USART LIN Enable Mask */
#define CR2_LINEN_Reset            ((u16)0xBFFF)  /* USART LIN Disable Mask */

/* USART LIN Break detection */
#define CR2_LBDL_Mask             ((u16)0xFFDF)  /* USART LIN Break detection Mask */

#define CR2_STOP_CLEAR_Mask       ((u16)0xCFFF)  /* USART CR2 STOP Bits Mask */
#define CR2_CLOCK_CLEAR_Mask      ((u16)0xF0FF)  /* USART CR2 Clock Mask */

/* USART SC Mask */
#define CR3_SCEN_Set              ((u16)0x0020)  /* USART SC Enable Mask */
#define CR3_SCEN_Reset            ((u16)0xFFDF)  /* USART SC Disable Mask */

/* USART SC NACK Mask */
#define CR3_NACK_Set              ((u16)0x0010)  /* USART SC NACK Enable Mask */
#define CR3_NACK_Reset            ((u16)0xFFEF)  /* USART SC NACK Disable Mask */

/* USART Half-Duplex Mask */
#define CR3_HDSEL_Set             ((u16)0x0008)  /* USART Half-Duplex Enable Mask */
#define CR3_HDSEL_Reset           ((u16)0xFFF7)  /* USART Half-Duplex Disable Mask */

/* USART IrDA Mask */
#define CR3_IRLP_Mask             ((u16)0xFFFB)  /* USART IrDA LowPower mode Mask */

#define CR3_CLEAR_Mask            ((u16)0xFCFF)  /* USART CR3 Mask */

/* USART IrDA Mask */
#define CR3_IREN_Set              ((u16)0x0002)  /* USART IrDA Enable Mask */
#define CR3_IREN_Reset            ((u16)0xFFFD)  /* USART IrDA Disable Mask */

#define GTPR_LSB_Mask             ((u16)0x00FF)  /* Guard Time Register LSB Mask */
#define GTPR_MSB_Mask             ((u16)0xFF00)  /* Guard Time Register MSB Mask */

#define IT_Mask                   ((u16)0x001F)  /* USART Interrupt Mask */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : USART_DeInit
* Description    : Deinitializes the USARTx peripheral registers to their
*                  default reset values.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_DeInit(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));

  switch (*(u32*)&USARTx)
  {
    case USART1_BASE:
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
      break;

    case USART2_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
      break;

    case USART3_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
      break;
    
    case UART4_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART4, DISABLE);
      break;
    
    case UART5_BASE:
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, ENABLE);
      RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART5, DISABLE);
      break;            

    default:
      break;
  }
}

/*******************************************************************************
* Function Name  : USART_Init
* Description    : Initializes the USARTx peripheral according to the specified
*                  parameters in the USART_InitStruct .
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_InitStruct: pointer to a USART_InitTypeDef structure
*                    that contains the configuration information for the
*                    specified USART peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_Init(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
  u32 tmpreg = 0x00, apbclock = 0x00;
  u32 integerdivider = 0x00;
  u32 fractionaldivider = 0x00;
  u32 usartxbase = 0;
  RCC_ClocksTypeDef RCC_ClocksStatus;

  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_BAUDRATE(USART_InitStruct->USART_BaudRate));  
  assert_param(IS_USART_WORD_LENGTH(USART_InitStruct->USART_WordLength));
  assert_param(IS_USART_STOPBITS(USART_InitStruct->USART_StopBits));
  assert_param(IS_USART_PARITY(USART_InitStruct->USART_Parity));
  assert_param(IS_USART_MODE(USART_InitStruct->USART_Mode));
  assert_param(IS_USART_HARDWARE_FLOW_CONTROL(USART_InitStruct->USART_HardwareFlowControl));
  /* The hardware flow control is available only for USART1, USART2 and USART3 */          
  assert_param(IS_USART_PERIPH_HFC(USARTx, USART_InitStruct->USART_HardwareFlowControl));
  
  usartxbase = (*(u32*)&USARTx);

/*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;
  /* Clear STOP[13:12] bits */
  tmpreg &= CR2_STOP_CLEAR_Mask;

  /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
  /* Set STOP[13:12] bits according to USART_StopBits value */
  tmpreg |= (u32)USART_InitStruct->USART_StopBits;
  
  /* Write to USART CR2 */
  USARTx->CR2 = (u16)tmpreg;

/*---------------------------- USART CR1 Configuration -----------------------*/
  tmpreg = USARTx->CR1;
  /* Clear M, PCE, PS, TE and RE bits */
  tmpreg &= CR1_CLEAR_Mask;

  /* Configure the USART Word Length, Parity and mode ----------------------- */
  /* Set the M bits according to USART_WordLength value */
  /* Set PCE and PS bits according to USART_Parity value */
  /* Set TE and RE bits according to USART_Mode value */
  tmpreg |= (u32)USART_InitStruct->USART_WordLength | USART_InitStruct->USART_Parity |
            USART_InitStruct->USART_Mode;

  /* Write to USART CR1 */
  USARTx->CR1 = (u16)tmpreg;

/*---------------------------- USART CR3 Configuration -----------------------*/  
  tmpreg = USARTx->CR3;
  /* Clear CTSE and RTSE bits */
  tmpreg &= CR3_CLEAR_Mask;

  /* Configure the USART HFC -------------------------------------------------*/
  /* Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
  tmpreg |= USART_InitStruct->USART_HardwareFlowControl;

  /* Write to USART CR3 */
  USARTx->CR3 = (u16)tmpreg;

/*---------------------------- USART BRR Configuration -----------------------*/
  /* Configure the USART Baud Rate -------------------------------------------*/
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  if (usartxbase == USART1_BASE)
  {
    apbclock = RCC_ClocksStatus.PCLK2_Frequency;
  }
  else
  {
    apbclock = RCC_ClocksStatus.PCLK1_Frequency;
  }

  /* Determine the integer part */
  integerdivider = ((0x19 * apbclock) / (0x04 * (USART_InitStruct->USART_BaudRate)));
  tmpreg = (integerdivider / 0x64) << 0x04;

  /* Determine the fractional part */
  fractionaldivider = integerdivider - (0x64 * (tmpreg >> 0x04));
  tmpreg |= ((((fractionaldivider * 0x10) + 0x32) / 0x64)) & ((u8)0x0F);

  /* Write to USART BRR */
  USARTx->BRR = (u16)tmpreg;
}

/*******************************************************************************
* Function Name  : USART_StructInit
* Description    : Fills each USART_InitStruct member with its default value.
* Input          : - USART_InitStruct: pointer to a USART_InitTypeDef structure
*                    which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_StructInit(USART_InitTypeDef* USART_InitStruct)
{
  /* USART_InitStruct members default value */
  USART_InitStruct->USART_BaudRate = 9600;
  USART_InitStruct->USART_WordLength = USART_WordLength_8b;
  USART_InitStruct->USART_StopBits = USART_StopBits_1;
  USART_InitStruct->USART_Parity = USART_Parity_No ;
  USART_InitStruct->USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStruct->USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
}

/*******************************************************************************
* Function Name  : USART_ClockInit
* Description    : Initializes the USARTx peripheral Clock according to the 
*                  specified parameters in the USART_ClockInitStruct .
* Input          : - USARTx: where x can be 1, 2, 3 to select the USART peripheral.
*                    Note: The Smart Card mode is not available for UART4 and UART5.
*                  - USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
*                    structure that contains the configuration information for 
*                    the specified USART peripheral.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_ClockInit(USART_TypeDef* USARTx, USART_ClockInitTypeDef* USART_ClockInitStruct)
{
  u32 tmpreg = 0x00;

  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  assert_param(IS_USART_CLOCK(USART_ClockInitStruct->USART_Clock));
  assert_param(IS_USART_CPOL(USART_ClockInitStruct->USART_CPOL));
  assert_param(IS_USART_CPHA(USART_ClockInitStruct->USART_CPHA));
  assert_param(IS_USART_LASTBIT(USART_ClockInitStruct->USART_LastBit));              
  
/*---------------------------- USART CR2 Configuration -----------------------*/
  tmpreg = USARTx->CR2;
  /* Clear CLKEN, CPOL, CPHA and LBCL bits */
  tmpreg &= CR2_CLOCK_CLEAR_Mask;

  /* Configure the USART Clock, CPOL, CPHA and LastBit ------------*/
  /* Set CLKEN bit according to USART_Clock value */
  /* Set CPOL bit according to USART_CPOL value */
  /* Set CPHA bit according to USART_CPHA value */
  /* Set LBCL bit according to USART_LastBit value */
  tmpreg |= (u32)USART_ClockInitStruct->USART_Clock | USART_ClockInitStruct->USART_CPOL | 
                 USART_ClockInitStruct->USART_CPHA | USART_ClockInitStruct->USART_LastBit;

  /* Write to USART CR2 */
  USARTx->CR2 = (u16)tmpreg;
}

/*******************************************************************************
* Function Name  : USART_ClockStructInit
* Description    : Fills each USART_ClockInitStruct member with its default value.
* Input          : - USART_ClockInitStruct: pointer to a USART_ClockInitTypeDef
*                    structure which will be initialized.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_ClockStructInit(USART_ClockInitTypeDef* USART_ClockInitStruct)
{
  /* USART_ClockInitStruct members default value */
  USART_ClockInitStruct->USART_Clock = USART_Clock_Disable;
  USART_ClockInitStruct->USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStruct->USART_CPHA = USART_CPHA_1Edge;
  USART_ClockInitStruct->USART_LastBit = USART_LastBit_Disable;
}

/*******************************************************************************
* Function Name  : USART_Cmd
* Description    : Enables or disables the specified USART peripheral.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                : - NewState: new state of the USARTx peripheral.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_Cmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the selected USART by setting the UE bit in the CR1 register */
    USARTx->CR1 |= CR1_UE_Set;
  }
  else
  {
    /* Disable the selected USART by clearing the UE bit in the CR1 register */
    USARTx->CR1 &= CR1_UE_Reset;
  }
}

/*******************************************************************************
* Function Name  : USART_ITConfig
* Description    : Enables or disables the specified USART interrupts.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_IT: specifies the USART interrupt sources to be
*                    enabled or disabled.
*                    This parameter can be one of the following values:
*                       - USART_IT_CTS:  CTS change interrupt (not available for
*                                        UART4 and UART5)
*                       - USART_IT_LBD:  LIN Break detection interrupt
*                       - USART_IT_TXE:  Tansmit Data Register empty interrupt
*                       - USART_IT_TC:   Transmission complete interrupt
*                       - USART_IT_RXNE: Receive Data register not empty 
*                                        interrupt
*                       - USART_IT_IDLE: Idle line detection interrupt
*                       - USART_IT_PE:   Parity Error interrupt
*                       - USART_IT_ERR:  Error interrupt(Frame error, noise
*                                        error, overrun error)
*                  - NewState: new state of the specified USARTx interrupts.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_ITConfig(USART_TypeDef* USARTx, u16 USART_IT, FunctionalState NewState)
{
  u32 usartreg = 0x00, itpos = 0x00, itmask = 0x00;
  u32 usartxbase = 0x00;

  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CONFIG_IT(USART_IT));
  assert_param(IS_USART_PERIPH_IT(USARTx, USART_IT)); /* The CTS interrupt is not available for UART4 and UART5 */     
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  usartxbase = (*(u32*)&(USARTx));

  /* Get the USART register index */
  usartreg = (((u8)USART_IT) >> 0x05);

  /* Get the interrupt position */
  itpos = USART_IT & IT_Mask;

  itmask = (((u32)0x01) << itpos);
    
  if (usartreg == 0x01) /* The IT is in CR1 register */
  {
    usartxbase += 0x0C;
  }
  else if (usartreg == 0x02) /* The IT is in CR2 register */
  {
    usartxbase += 0x10;
  }
  else /* The IT is in CR3 register */
  {
    usartxbase += 0x14; 
  }
  if (NewState != DISABLE)
  {
    *(vu32*)usartxbase  |= itmask;
  }
  else
  {
    *(vu32*)usartxbase &= ~itmask;
  }
}

/*******************************************************************************
* Function Name  : USART_DMACmd
* Description    : Enables or disables the USART’s DMA interface.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3 or UART4.
*                    Note: The DMA mode is not available for UART5.
*                  - USART_DMAReq: specifies the DMA request.
*                    This parameter can be any combination of the following values:
*                       - USART_DMAReq_Tx: USART DMA transmit request
*                       - USART_DMAReq_Rx: USART DMA receive request
*                  - NewState: new state of the DMA Request sources.
*                   This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_DMACmd(USART_TypeDef* USARTx, u16 USART_DMAReq, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_1234_PERIPH(USARTx));
  assert_param(IS_USART_DMAREQ(USART_DMAReq));  
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 

  if (NewState != DISABLE)
  {
    /* Enable the DMA transfer for selected requests by setting the DMAT and/or
       DMAR bits in the USART CR3 register */
    USARTx->CR3 |= USART_DMAReq;
  }
  else
  {
    /* Disable the DMA transfer for selected requests by clearing the DMAT and/or
       DMAR bits in the USART CR3 register */
    USARTx->CR3 &= (u16)~USART_DMAReq;
  }
}

/*******************************************************************************
* Function Name  : USART_SetAddress
* Description    : Sets the address of the USART node.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_Address: Indicates the address of the USART node.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_SetAddress(USART_TypeDef* USARTx, u8 USART_Address)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_ADDRESS(USART_Address)); 
    
  /* Clear the USART address */
  USARTx->CR2 &= CR2_Address_Mask;
  /* Set the USART address node */
  USARTx->CR2 |= USART_Address;
}

/*******************************************************************************
* Function Name  : USART_WakeUpConfig
* Description    : Selects the USART WakeUp method.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_WakeUp: specifies the USART wakeup method.
*                    This parameter can be one of the following values:
*                        - USART_WakeUp_IdleLine: WakeUp by an idle line detection
*                        - USART_WakeUp_AddressMark: WakeUp by an address mark
* Output         : None
* Return         : None
*******************************************************************************/
void USART_WakeUpConfig(USART_TypeDef* USARTx, u16 USART_WakeUp)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_WAKEUP(USART_WakeUp));
  
  USARTx->CR1 &= CR1_WAKE_Mask;
  USARTx->CR1 |= USART_WakeUp;
}

/*******************************************************************************
* Function Name  : USART_ReceiverWakeUpCmd
* Description    : Determines if the USART is in mute mode or not.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - NewState: new state of the USART mute mode.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_ReceiverWakeUpCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState)); 
  
  if (NewState != DISABLE)
  {
    /* Enable the USART mute mode  by setting the RWU bit in the CR1 register */
    USARTx->CR1 |= CR1_RWU_Set;
  }
  else
  {
    /* Disable the USART mute mode by clearing the RWU bit in the CR1 register */
    USARTx->CR1 &= CR1_RWU_Reset;
  }
}

/*******************************************************************************
* Function Name  : USART_LINBreakDetectLengthConfig
* Description    : Sets the USART LIN Break detection length.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_LINBreakDetectLength: specifies the LIN break
*                    detection length.
*                    This parameter can be one of the following values:
*                       - USART_LINBreakDetectLength_10b: 10-bit break detection
*                       - USART_LINBreakDetectLength_11b: 11-bit break detection
* Output         : None
* Return         : None
*******************************************************************************/
void USART_LINBreakDetectLengthConfig(USART_TypeDef* USARTx, u16 USART_LINBreakDetectLength)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_LIN_BREAK_DETECT_LENGTH(USART_LINBreakDetectLength));
  
  USARTx->CR2 &= CR2_LBDL_Mask;
  USARTx->CR2 |= USART_LINBreakDetectLength;  
}

/*******************************************************************************
* Function Name  : USART_LINCmd
* Description    : Enables or disables the USART’s LIN mode.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - NewState: new state of the USART LIN mode.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_LINCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the LIN mode by setting the LINEN bit in the CR2 register */
    USARTx->CR2 |= CR2_LINEN_Set;
  }
  else
  {
    /* Disable the LIN mode by clearing the LINEN bit in the CR2 register */
    USARTx->CR2 &= CR2_LINEN_Reset;
  }
}

/*******************************************************************************
* Function Name  : USART_SendData
* Description    : Transmits single data through the USARTx peripheral.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - Data: the data to transmit.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_SendData(USART_TypeDef* USARTx, u16 Data)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_DATA(Data)); 
    
  /* Transmit Data */
  USARTx->DR = (Data & (u16)0x01FF);
}

/*******************************************************************************
* Function Name  : USART_ReceiveData
* Description    : Returns the most recent received data by the USARTx peripheral.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
* Output         : None
* Return         : The received data.
*******************************************************************************/
u16 USART_ReceiveData(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  
  /* Receive Data */
  return (u16)(USARTx->DR & (u16)0x01FF);
}

/*******************************************************************************
* Function Name  : USART_SendBreak
* Description    : Transmits break characters.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_SendBreak(USART_TypeDef* USARTx)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  
  /* Send break characters */
  USARTx->CR1 |= CR1_SBK_Set;
}

/*******************************************************************************
* Function Name  : USART_SetGuardTime
* Description    : Sets the specified USART guard time.
* Input          : - USARTx: where x can be 1, 2 or 3 to select the USART
*                    peripheral.
*                  Note: The guard time bits are not available for UART4 and UART5.
*                  - USART_GuardTime: specifies the guard time.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_SetGuardTime(USART_TypeDef* USARTx, u8 USART_GuardTime)
{    
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  
  /* Clear the USART Guard time */
  USARTx->GTPR &= GTPR_LSB_Mask;
  /* Set the USART guard time */
  USARTx->GTPR |= (u16)((u16)USART_GuardTime << 0x08);
}

/*******************************************************************************
* Function Name  : USART_SetPrescaler
* Description    : Sets the system clock prescaler.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  Note: The function is used for IrDA mode with UART4 and UART5.
*                  - USART_Prescaler: specifies the prescaler clock.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_SetPrescaler(USART_TypeDef* USARTx, u8 USART_Prescaler)
{ 
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  
  /* Clear the USART prescaler */
  USARTx->GTPR &= GTPR_MSB_Mask;
  /* Set the USART prescaler */
  USARTx->GTPR |= USART_Prescaler;
}

/*******************************************************************************
* Function Name  : USART_SmartCardCmd
* Description    : Enables or disables the USART’s Smart Card mode.
* Input          : - USARTx: where x can be 1, 2 or 3 to select the USART
*                    peripheral. 
*                    Note: The Smart Card mode is not available for UART4 and UART5.
*                  - NewState: new state of the Smart Card mode.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_SmartCardCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the SC mode by setting the SCEN bit in the CR3 register */
    USARTx->CR3 |= CR3_SCEN_Set;
  }
  else
  {
    /* Disable the SC mode by clearing the SCEN bit in the CR3 register */
    USARTx->CR3 &= CR3_SCEN_Reset;
  }
}

/*******************************************************************************
* Function Name  : USART_SmartCardNACKCmd
* Description    : Enables or disables NACK transmission.
* Input          : - USARTx: where x can be 1, 2 or 3 to select the USART
*                    peripheral. 
*                    Note: The Smart Card mode is not available for UART4 and UART5.
*                  - NewState: new state of the NACK transmission.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_SmartCardNACKCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_123_PERIPH(USARTx));  
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the NACK transmission by setting the NACK bit in the CR3 register */
    USARTx->CR3 |= CR3_NACK_Set;
  }
  else
  {
    /* Disable the NACK transmission by clearing the NACK bit in the CR3 register */
    USARTx->CR3 &= CR3_NACK_Reset;
  }
}

/*******************************************************************************
* Function Name  : USART_HalfDuplexCmd
* Description    : Enables or disables the USART’s Half Duplex communication.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - NewState: new state of the USART Communication.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_HalfDuplexCmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the Half-Duplex mode by setting the HDSEL bit in the CR3 register */
    USARTx->CR3 |= CR3_HDSEL_Set;
  }
  else
  {
    /* Disable the Half-Duplex mode by clearing the HDSEL bit in the CR3 register */
    USARTx->CR3 &= CR3_HDSEL_Reset;
  }
}

/*******************************************************************************
* Function Name  : USART_IrDAConfig
* Description    : Configures the USART’s IrDA interface.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_IrDAMode: specifies the IrDA mode.
*                    This parameter can be one of the following values:
*                       - USART_IrDAMode_LowPower
*                       - USART_IrDAMode_Normal
* Output         : None
* Return         : None
*******************************************************************************/
void USART_IrDAConfig(USART_TypeDef* USARTx, u16 USART_IrDAMode)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_IRDA_MODE(USART_IrDAMode));
    
  USARTx->CR3 &= CR3_IRLP_Mask;
  USARTx->CR3 |= USART_IrDAMode;
}

/*******************************************************************************
* Function Name  : USART_IrDACmd
* Description    : Enables or disables the USART’s IrDA interface.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - NewState: new state of the IrDA mode.
*                    This parameter can be: ENABLE or DISABLE.
* Output         : None
* Return         : None
*******************************************************************************/
void USART_IrDACmd(USART_TypeDef* USARTx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  if (NewState != DISABLE)
  {
    /* Enable the IrDA mode by setting the IREN bit in the CR3 register */
    USARTx->CR3 |= CR3_IREN_Set;
  }
  else
  {
    /* Disable the IrDA mode by clearing the IREN bit in the CR3 register */
    USARTx->CR3 &= CR3_IREN_Reset;
  }
}

/*******************************************************************************
* Function Name  : USART_GetFlagStatus
* Description    : Checks whether the specified USART flag is set or not.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_FLAG: specifies the flag to check.
*                    This parameter can be one of the following values:
*                       - USART_FLAG_CTS:  CTS Change flag (not available for 
*                                          UART4 and UART5)
*                       - USART_FLAG_LBD:  LIN Break detection flag
*                       - USART_FLAG_TXE:  Transmit data register empty flag
*                       - USART_FLAG_TC:   Transmission Complete flag
*                       - USART_FLAG_RXNE: Receive data register not empty flag
*                       - USART_FLAG_IDLE: Idle Line detection flag
*                       - USART_FLAG_ORE:  OverRun Error flag
*                       - USART_FLAG_NE:   Noise Error flag
*                       - USART_FLAG_FE:   Framing Error flag
*                       - USART_FLAG_PE:   Parity Error flag
* Output         : None
* Return         : The new state of USART_FLAG (SET or RESET).
*******************************************************************************/
FlagStatus USART_GetFlagStatus(USART_TypeDef* USARTx, u16 USART_FLAG)
{
  FlagStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_FLAG(USART_FLAG));
  assert_param(IS_USART_PERIPH_FLAG(USARTx, USART_FLAG)); /* The CTS flag is not available for UART4 and UART5 */   

  if ((USARTx->SR & USART_FLAG) != (u16)RESET)
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  return bitstatus;
}

/*******************************************************************************
* Function Name  : USART_ClearFlag
* Description    : Clears the USARTx's pending flags.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_FLAG: specifies the flag to clear.
*                    This parameter can be any combination of the following values:
*                       - USART_FLAG_CTS:  CTS Change flag (not available for
*                                          UART4 and UART5).
*                       - USART_FLAG_LBD:  LIN Break detection flag.
*                       - USART_FLAG_TC:   Transmission Complete flag.
*                       - USART_FLAG_RXNE: Receive data register not empty flag.
*
*                  Notes:
*                        - PE (Parity error), FE (Framing error), NE (Noise error),
*                          ORE (OverRun error) and IDLE (Idle line detected) 
*                          flags are cleared by software sequence: a read 
*                          operation to USART_SR register (USART_GetFlagStatus()) 
*                          followed by a read operation to USART_DR register 
*                          (USART_ReceiveData()).
*                        - RXNE flag can be also cleared by a read to the 
*                          USART_DR register (USART_ReceiveData()).
*                        - TC flag can be also cleared by software sequence: a 
*                          read operation to USART_SR register 
*                          (USART_GetFlagStatus()) followed by a write operation
*                          to USART_DR register (USART_SendData()).                                                      
*                        - TXE flag is cleared only by a write to the USART_DR 
*                          register (USART_SendData()).                        
* Output         : None
* Return         : None
*******************************************************************************/
void USART_ClearFlag(USART_TypeDef* USARTx, u16 USART_FLAG)
{
  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_FLAG(USART_FLAG));
  assert_param(IS_USART_PERIPH_FLAG(USARTx, USART_FLAG)); /* The CTS flag is not available for UART4 and UART5 */   
   
  USARTx->SR = (u16)~USART_FLAG;
}

/*******************************************************************************
* Function Name  : USART_GetITStatus
* Description    : Checks whether the specified USART interrupt has occurred or not.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_IT: specifies the USART interrupt source to check.
*                    This parameter can be one of the following values:
*                       - USART_IT_CTS:  CTS change interrupt (not available for 
*                                        UART4 and UART5)
*                       - USART_IT_LBD:  LIN Break detection interrupt
*                       - USART_IT_TXE:  Tansmit Data Register empty interrupt
*                       - USART_IT_TC:   Transmission complete interrupt
*                       - USART_IT_RXNE: Receive Data register not empty 
*                                        interrupt
*                       - USART_IT_IDLE: Idle line detection interrupt
*                       - USART_IT_ORE:  OverRun Error interrupt
*                       - USART_IT_NE:   Noise Error interrupt
*                       - USART_IT_FE:   Framing Error interrupt
*                       - USART_IT_PE:   Parity Error interrupt
* Output         : None
* Return         : The new state of USART_IT (SET or RESET).
*******************************************************************************/
ITStatus USART_GetITStatus(USART_TypeDef* USARTx, u16 USART_IT)
{
  u32 bitpos = 0x00, itmask = 0x00, usartreg = 0x00;
  ITStatus bitstatus = RESET;

  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_GET_IT(USART_IT));
  assert_param(IS_USART_PERIPH_IT(USARTx, USART_IT)); /* The CTS interrupt is not available for UART4 and UART5 */  
  
  /* Get the USART register index */
  usartreg = (((u8)USART_IT) >> 0x05);

  /* Get the interrupt position */
  itmask = USART_IT & IT_Mask;

  itmask = (u32)0x01 << itmask;
  
  if (usartreg == 0x01) /* The IT  is in CR1 register */
  {
    itmask &= USARTx->CR1;
  }
  else if (usartreg == 0x02) /* The IT  is in CR2 register */
  {
    itmask &= USARTx->CR2;
  }
  else /* The IT  is in CR3 register */
  {
    itmask &= USARTx->CR3;
  }
  
  bitpos = USART_IT >> 0x08;

  bitpos = (u32)0x01 << bitpos;
  bitpos &= USARTx->SR;

  if ((itmask != (u16)RESET)&&(bitpos != (u16)RESET))
  {
    bitstatus = SET;
  }
  else
  {
    bitstatus = RESET;
  }
  
  return bitstatus;  
}

/*******************************************************************************
* Function Name  : USART_ClearITPendingBit
* Description    : Clears the USARTx’s interrupt pending bits.
* Input          : - USARTx: Select the USART or the UART peripheral. 
*                    This parameter can be one of the following values:
*                     - USART1, USART2, USART3, UART4 or UART5.
*                  - USART_IT: specifies the interrupt pending bit to clear.
*                    This parameter can be one of the following values:
*                       - USART_IT_CTS:  CTS change interrupt (not available for 
*                                        UART4 and UART5)
*                       - USART_IT_LBD:  LIN Break detection interrupt
*                       - USART_IT_TC:   Transmission complete interrupt. 
*                       - USART_IT_RXNE: Receive Data register not empty interrupt.
*                    
*                  Notes:
*                        - PE (Parity error), FE (Framing error), NE (Noise error),
*                          ORE (OverRun error) and IDLE (Idle line detected) 
*                          pending bits are cleared by software sequence: a read 
*                          operation to USART_SR register (USART_GetITStatus()) 
*                          followed by a read operation to USART_DR register 
*                          (USART_ReceiveData()).
*                        - RXNE pending bit can be also cleared by a read to the 
*                          USART_DR register (USART_ReceiveData()).
*                        - TC pending bit can be also cleared by software 
*                          sequence: a read operation to USART_SR register 
*                          (USART_GetITStatus()) followed by a write operation
*                          to USART_DR register (USART_SendData()).                                                      
*                        - TXE pending bit is cleared only by a write to the 
*                          USART_DR register (USART_SendData()).  
* Output         : None
* Return         : None
*******************************************************************************/
void USART_ClearITPendingBit(USART_TypeDef* USARTx, u16 USART_IT)
{
  u16 bitpos = 0x00, itmask = 0x00;

  /* Check the parameters */
  assert_param(IS_USART_ALL_PERIPH(USARTx));
  assert_param(IS_USART_CLEAR_IT(USART_IT));
  assert_param(IS_USART_PERIPH_IT(USARTx, USART_IT)); /* The CTS interrupt is not available for UART4 and UART5 */
  
  bitpos = USART_IT >> 0x08;

  itmask = (u16)((u16)0x01 << bitpos);
  USARTx->SR = (u16)~itmask;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
















/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_wwdg.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides all the WWDG firmware functions.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_wwdg.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* ----------- WWDG registers bit address in the alias region ----------- */
#define WWDG_OFFSET       (WWDG_BASE - PERIPH_BASE)

/* Alias word address of EWI bit */
#define CFR_OFFSET        (WWDG_OFFSET + 0x04)
#define EWI_BitNumber     0x09
#define CFR_EWI_BB        (PERIPH_BB_BASE + (CFR_OFFSET * 32) + (EWI_BitNumber * 4))

/* --------------------- WWDG registers bit mask ------------------------ */
/* CR register bit mask */
#define CR_WDGA_Set       ((u32)0x00000080)

/* CFR register bit mask */
#define CFR_WDGTB_Mask    ((u32)0xFFFFFE7F)
#define CFR_W_Mask        ((u32)0xFFFFFF80)

#define BIT_Mask          ((u8)0x7F)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : WWDG_DeInit
* Description    : Deinitializes the WWDG  peripheral registers to their default
*                  reset values.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_DeInit(void)
{
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_WWDG, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_WWDG, DISABLE);
}

/*******************************************************************************
* Function Name  : WWDG_SetPrescaler
* Description    : Sets the WWDG Prescaler.
* Input          : - WWDG_Prescaler: specifies the WWDG Prescaler.
*                    This parameter can be one of the following values:
*                       - WWDG_Prescaler_1: WWDG counter clock = (PCLK1/4096)/1
*                       - WWDG_Prescaler_2: WWDG counter clock = (PCLK1/4096)/2
*                       - WWDG_Prescaler_4: WWDG counter clock = (PCLK1/4096)/4
*                       - WWDG_Prescaler_8: WWDG counter clock = (PCLK1/4096)/8
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_SetPrescaler(u32 WWDG_Prescaler)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_WWDG_PRESCALER(WWDG_Prescaler));

  /* Clear WDGTB[1:0] bits */
  tmpreg = WWDG->CFR & CFR_WDGTB_Mask;

  /* Set WDGTB[1:0] bits according to WWDG_Prescaler value */
  tmpreg |= WWDG_Prescaler;

  /* Store the new value */
  WWDG->CFR = tmpreg;
}

/*******************************************************************************
* Function Name  : WWDG_SetWindowValue
* Description    : Sets the WWDG window value.
* Input          : - WindowValue: specifies the window value to be compared to
*                    the downcounter.
*                    This parameter value must be lower than 0x80.
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_SetWindowValue(u8 WindowValue)
{
  u32 tmpreg = 0;

  /* Check the parameters */
  assert_param(IS_WWDG_WINDOW_VALUE(WindowValue));

  /* Clear W[6:0] bits */
  tmpreg = WWDG->CFR & CFR_W_Mask;

  /* Set W[6:0] bits according to WindowValue value */
  tmpreg |= WindowValue & BIT_Mask;

  /* Store the new value */
  WWDG->CFR = tmpreg;
}

/*******************************************************************************
* Function Name  : WWDG_EnableIT
* Description    : Enables the WWDG Early Wakeup interrupt(EWI).
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_EnableIT(void)
{
  *(vu32 *) CFR_EWI_BB = (u32)ENABLE;
}

/*******************************************************************************
* Function Name  : WWDG_SetCounter
* Description    : Sets the WWDG counter value.
* Input          : - Counter: specifies the watchdog counter value.
*                    This parameter must be a number between 0x40 and 0x7F.
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_SetCounter(u8 Counter)
{
  /* Check the parameters */
  assert_param(IS_WWDG_COUNTER(Counter));

  /* Write to T[6:0] bits to configure the counter value, no need to do
     a read-modify-write; writing a 0 to WDGA bit does nothing */
  WWDG->CR = Counter & BIT_Mask;
}

/*******************************************************************************
* Function Name  : WWDG_Enable
* Description    : Enables WWDG and load the counter value.
*                  - Counter: specifies the watchdog counter value.
*                    This parameter must be a number between 0x40 and 0x7F.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_Enable(u8 Counter)
{
  /* Check the parameters */
  assert_param(IS_WWDG_COUNTER(Counter));

  WWDG->CR = CR_WDGA_Set | Counter;
}

/*******************************************************************************
* Function Name  : WWDG_GetFlagStatus
* Description    : Checks whether the Early Wakeup interrupt flag is set or not.
* Input          : None
* Output         : None
* Return         : The new state of the Early Wakeup interrupt flag (SET or RESET)
*******************************************************************************/
FlagStatus WWDG_GetFlagStatus(void)
{
  return (FlagStatus)(WWDG->SR);
}

/*******************************************************************************
* Function Name  : WWDG_ClearFlag
* Description    : Clears Early Wakeup interrupt flag.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WWDG_ClearFlag(void)
{
  WWDG->SR = (u32)RESET;
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/


















#include "SysClock.h"

/* Function pointer for timer */
static void (*SYSCLOCK_extFunc)(void);

/*******************************************************************************
* Function Name  : SYSCLOCK_Init
* Description    : Initialize the SYSTick Peripheral.
* Input          : timeSpan = Time span of the timer in micro seconds.
*                : ptr2Func = Pointer to function to call on timer tick.
* Output         : None
* Return         : None
*******************************************************************************/
void SYSCLOCK_Init(void (*ptr2Func)(void))
{	
	/* Set the priority and sub priority of the SysTick interrupt handler */
	NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 1, 2);

  /* Enable SysTick interrupt */
  SysTick_ITConfig(ENABLE);
	
	/* Set external function pointer */
	SYSCLOCK_extFunc = ptr2Func;
}

/*******************************************************************************
* Function Name  : SYSCLOCK_Enable
* Description    : Enable the SYSTick Peripheral.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SYSCLOCK_Enable()
{
	/* Enable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Enable);		
}

/*******************************************************************************
* Function Name  : SYSCLOCK_Disable
* Description    : Disable the SYSTick Peripheral.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SYSCLOCK_Disable()
{
	/* Disable the SysTick Counter */
  SysTick_CounterCmd(SysTick_Counter_Disable);		
}

/*******************************************************************************
* Function Name  : SYSCLOCK_SetTimeSpan
* Description    : Set the timespan for the SYSTick Peripheral timer.
* Input          : timeSpan = Time span of the timer in micro seconds.
* Output         : None
* Return         : None
*******************************************************************************/
void SYSCLOCK_SetTimeSpan(u32 timeSpan)
{
	/* Disable the SysTick Counter */
  /* SysTick_CounterCmd(SysTick_Counter_Disable); */
	
	/* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
  SysTick_SetReload(9*timeSpan);
	
	/* Enable the SysTick Counter */
  /* SysTick_CounterCmd(SysTick_Counter_Enable); */
}

/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTickHandler(void)
{
	(*SYSCLOCK_extFunc)();	
}



















/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_lib.c
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file provides device initialization and common
*                    : routines for all devices.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "VN_lib.h"







/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/




















/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN100.c
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file provides all of the firmware functions specific
*                    : to the VN100.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "VN_math.h"
#include <math.h>
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : VN_CrossP(float *A, float *B, float *C)
* Description    : Compute the cross product of vector A with vector B.
* Equation       : C = cross(A, B)                                        
* Input          : A -> 3x1 vector
*                  B -> 3x1 vector
*                  C -> 3x1 vector
* Output         : None
* Return         : None
*******************************************************************************/
void VN_CrossP(float *A, float *B, float *C){
  C[0] = A[1]*B[2]-A[2]*B[1];
  C[1] = A[2]*B[0]-A[0]*B[2];
  C[2] = A[0]*A[1]-A[1]*B[0];
}

/*******************************************************************************
* Function Name  : VN_VecAdd(float *A, float *B, unsigned long rows, float *C)
* Description    : Compute the addition of vector A with vector B.
* Equation       : C = A + B
* Input          : A -> vector with length given by rows
*                : B -> vector with length given by rows
*                : rows -> length of vector A, B, and C
* Output         : C -> result of vector addition
* Return         : None
*******************************************************************************/
void VN_VecAdd(float *A, float *B, unsigned long rows, float *C){
  unsigned long i;
  for(i=0;i<rows;i++) C[i] = A[i] + B[i];
}

/*******************************************************************************
* Function Name  : VN_VecSub(float *A, float *B, unsigned long rows, float *C)
* Description    : Compute the subtraction of vector A with vector B.
* Equation       : C = A - B                                        
* Input          : A -> vector with length given by rows
*                : B -> vector with length given by rows
*                : rows -> length of vector A, B, and C
* Output         : C -> result of vector subtraction
* Return         : None
*******************************************************************************/
void VN_VecSub(float *A, float *B, unsigned long rows, float *C){
  unsigned long i;
  for(i=0;i<rows;i++) C[i] = A[i] - B[i];
}

/*******************************************************************************
* Function Name  : VN_VecMultT(float *A, float *BT, unsigned long rows, float **C)
* Description    : Compute the multiplication of a vector with the transpose of
*                  another vector. The result will be a square matrix with the
*                  size of nxn where n=rows.
* Equation       : C = A * transpose(B)                                        
* Input          : A -> vector with length given by rows
*                : BT -> vector with length given by rows
*                : rows -> length of vector A and B
* Output         : C -> result of multiplication of A with the transpose of B
* Return         : None
*******************************************************************************/
void VN_VecMultT(float *A, float *BT, unsigned long rows, float **C){
  unsigned long i, j;
  for(i=0;i<rows; i++){
    for(j=0;j<rows; j++){
      C[i][j] = A[i]*BT[j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_Identity(float scalar, unsigned long Arows, unsigned long Acols, float *A)
* Description    : Create an zero matrix with the diagonal elements equal to
*                : the magnitude of scalar.
* Equation       : A = scalar * eye(Arows, Acols)                                        
* Input          : scalar -> desired magnitude of diagnonal terms
*                : Arows -> number of rows for the resulting matrix A
*                : Acols -> number of columns for the resulting matrix A 
* Output         : A -> resulting diagonal matrix
* Return         : None
*******************************************************************************/
void VN_Identity(float scalar, unsigned long Arows, unsigned long Acols, float **A){
  unsigned long i,j;
  
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      if(i==j){
        A[i][j] = scalar;
      }else{
        A[i][j] = 0;
      }
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatAdd(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C){
* Description    : Compute the addition of matrix A and B.
* Equation       : C = A + B                                        
* Input          : A -> Matrix A with size of Arows x Acols.
*                : B -> Matrix B with size of Arows x Acols.
*                : Arows -> Number of rows in matrix A and B.
*                : Acols -> Number of cols in matrix A and B. 
* Output         : C -> Result of matrix addition with size of Arows x Acols.
* Return         : None
*******************************************************************************/
void VN_MatAdd(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C){
  unsigned long i,j;
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      C[i][j] = A[i][j] + B[i][j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatSub(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C){
* Description    : Compute the subtraction of matrix A and B.
* Equation       : C = A - B                                        
* Input          : A -> Matrix A with size of Arows x Acols.
*                : B -> Matrix B with size of Arows x Acols.
*                : Arows -> Number of rows in matrix A and B.
*                : Acols -> Number of cols in matrix A and B. 
* Output         : C -> Result of matrix subtraction with size of Arows x Acols.
* Return         : None
*******************************************************************************/
void VN_MatSub(float **A, float **B, unsigned long Arows, unsigned long Acols, float **C){
  unsigned long i,j;
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++) C[i][j] = A[i][j] - B[i][j];
  }
}

/*******************************************************************************
* Function Name  : VN_MatMult(float **A, float **B, unsigned long Arows, unsigned long Acols, unsigned long Bcols, float **C)
* Description    : Compute the multplication of matrix A and B.
* Equation       : C = A * B
* Input          : A -> Matrix A with size of Arows x Acols
*                : B -> Matrix B with size of Acols x Bcols
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
*                : Bcols -> Number of columns in matrix B
* Output         : C -> Result of the matrix multplication with size Arows x Bcols
* Return         : None
*******************************************************************************/
void VN_MatMult(float **A, float **B, unsigned long Arows, unsigned long Acols, unsigned long Bcols, float **C){
  unsigned long i,j,k;
  float temp;
  for(i=0;i<Arows;i++){
    for(j=0;j<Bcols;j++){
      temp = 0;
      for(k=Acols; k--; ){
        temp += A[i][k]*B[k][j];
      }
      C[i][j] = temp;
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatMultMT(float **A, float **BT, unsigned long Arows, unsigned long Acols, unsigned long Bcols, float **C)
* Description    : Compute the multplication of matrix A with the transpose of matrix B.
* Equation       : C = A * transpose(B)
* Input          : A -> Matrix A with size of Arows x Acols
*                : B -> Matrix B with size of Acols x Bcols
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
*                : Bcols -> Number of columns in matrix B
* Output         : C -> Result of the matrix multplication with size Arows x Bcols
* Return         : None
*******************************************************************************/
void VN_MatMultMT(float **A, float **BT, unsigned long Arows, unsigned long Acols, unsigned long Brows, float **C){
  unsigned long i,j,k;
  float temp;
  for(i=0;i<Arows;i++){
    for(j=0;j<Brows;j++){
      temp = 0;
      for(k=Acols; k--; ){
        temp += A[i][k]*BT[j][k];
      }
      C[i][j] = temp;
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatScalarMult(double **A, double scalar, unsigned long Arows, unsigned long Acols, double **C)
* Description    : Compute the multplication of a scalar times a matrix.
* Equation       : C = scalar * A                                        
* Input          : scalar -> The scalar term
*                : A -> The matrix with the size Arows x Acols
*                : Arows -> The number of rows in the matrix A
*                : Acols -> The number of columns in the matrix B
* Output         : C -> The result of the operation scalar * A
* Return         : None
*******************************************************************************/
void VN_MatScalarMult(double **A, double scalar, unsigned long Arows, unsigned long Acols, double **C){
  unsigned long i,j;
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      C[i][j] = scalar*A[i][j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_MatVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C)
* Description    : Compute the multiplication of matrix A with vector B.
* Equation       : C = A * B                                        
* Input          : A -> Matrix with size Arows x Acols
*                : B -> Column vector with size Acols x 1
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
* Output         : C -> Matrix result with size Arows x Acols
* Return         : None
*******************************************************************************/
void VN_MatVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C){
  unsigned long i,k;
  for(i=0;i<Arows;i++){
    C[i] = 0;
    for(k=0;k<Acols;k++) C[i] += A[i][k]*B[k];
  }
}

/*******************************************************************************
* Function Name  : VN_MatTVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C)
* Description    : Multiply the transpose of the matrix A by the vector B.
* Equation       : C = transpose(A) * B                                        
* Input          : A -> Matrix with size Arows x Acols
*                : B -> Column vector with size Arows x 1
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
* Output         : C -> Matrix result with size Acols x Arows
* Return         : None
*******************************************************************************/
void VN_MatTVecMult(float **A, float *B, unsigned long Arows, unsigned long Acols, float *C){
  unsigned long i,k;
  for(i=0;i<Arows;i++){
    C[i] = 0;
    for(k=0;k<Acols;k++) C[i] += A[k][i]*B[k];
  }
}

/*******************************************************************************
* Function Name  : VN_MatCopy(float **A, unsigned long nrows, unsigned long ncols, float **B)
* Description    : Copy the values from one matrix to another.
* Equation       : B = A                                        
* Input          : A -> Matrix with size nrows x ncols
*                : nrows -> number of rows in matrix A
*                : ncols -> number of columns in matrix A
* Output         : B -> Resulting matrix with size nrows x ncols
* Return         : None
*******************************************************************************/
void VN_MatCopy(float **A, unsigned long nrows, unsigned long ncols, float **B){
  unsigned long i,j;
  for(i=0;i<nrows;i++){
    for(j=0;j<ncols;j++) B[i][j] = A[i][j];
  }
}

/*******************************************************************************
* Function Name  : VN_MatInv(float **A, s32 n, float **B)
* Description    : Compute the matrix inverse of A.
* Equation       : B = inv(A)                                        
* Input          : A -> Matrix with size n x n
*                : n -> Number of rows and columns in matrix A
* Output         : B -> Matrix result with size n x n
* Return         : None
*******************************************************************************/
void VN_MatInv(float **A, signed long n, float **B)
{
  int indxc[VN_INV_MAX_SIZE], indxr[VN_INV_MAX_SIZE], ipiv[VN_INV_MAX_SIZE];
  int i,icol,irow,j,k,l,ll;
  float big,dum,pivinv,temp;
  
  irow = 0;
  icol = 0;

  VN_MatCopy(A,n,n,B);

  for (j=0;j<n;j++) ipiv[j]=0;
  for (i=0;i<n;i++){
      big=0.0;
      for (j=0;j<n;j++)
        if (ipiv[j] != 1)
          for (k=0;k<n;k++) {
            if (ipiv[k] == 0) {
              if (fabs(B[j][k]) >= big) {
                big=(float)fabsf(B[j][k]);
                irow=j;
                icol=k;
              }
            }
          }
        ++(ipiv[icol]);
        if (irow != icol) {
          for (l=0;l<n;l++) VN_SWAP(B[irow][l],B[icol][l])
        }
        indxr[i]=irow;
        indxc[i]=icol;
        pivinv=1.0f/B[icol][icol];
        B[icol][icol]=1.0;
        for (l=0;l<n;l++) B[icol][l] *= pivinv;
          for (ll=0;ll<n;ll++)
            if (ll != icol) {
              dum=B[ll][icol];
              B[ll][icol]=0.0;
              for (l=0;l<n;l++) B[ll][l] -= B[icol][l]*dum;
            }
        }
        for (l=n-1;l>=0;l--) {
          if (indxr[l] != indxc[l])
            for (k=0;k<n;k++)
              VN_SWAP(B[k][indxr[l]],B[k][indxc[l]]);
        }
}

/*******************************************************************************
* Function Name  : VN_SkewMatrix(float *V, float **A)
* Description    : Compute the matrix cross product of vector V. This operation
*                : converts a vector into a matrix that when multplied by
*                : another vector would give the result of a cross product
*                : operation between the two vectors.
* Equation       : A = skew(V)  where A*b = cross(V,b)  if b is a vector same
*                : size as V.                                        
* Input          : V -> Vector of size 3x1 to perform operation on.
* Output         : A -> Resulting matrix with size 3x3.
* Return         : None
*******************************************************************************/
void VN_SkewMatrix(float *V, float **A){
  A[0][0] = 0;
  A[0][1] = -V[2];
  A[0][2] = V[1];
  A[1][0] = V[2];
  A[1][1] = 0;
  A[1][2] = -V[0];
  A[2][0] = -V[1];
  A[2][1] = V[0];
  A[2][2] = 0;
}

/*******************************************************************************
* Function Name  : VN_Transpose(float **A, unsigned long Arows, unsigned long Acols, float **B)
* Description    : Calculate the transpose of matrix A.
* Equation       : B = transpose(A)                                        
* Input          : A -> Matrix with size Arows x Acols
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
* Output         : B -> Resulting matrix with size Acols x Arows
* Return         : None
*******************************************************************************/
void VN_Transpose(float **A, unsigned long Arows, unsigned long Acols, float **B){
  unsigned long i,j;
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      B[j][i] = A[i][j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_Norm(float *A, unsigned long m)
* Description    : Compute the length of the given vector with size m x 1
* Equation       : norm(A)                                        
* Input          : A -> Vector to compute the length of with size m x 1
*                : m -> Number of terms in the vector A.
* Output         : None
* Return         : The length of the given vector with size m x 1
*******************************************************************************/
float VN_Norm(float *A, unsigned long m){
  float nrm = 0;
  unsigned long i;
  for(i=0; i < m; i++){
    nrm += A[i]*A[i];
  }
  return sqrtf(nrm);  
}

/*******************************************************************************
* Function Name  : VN_Normalize(float *V1, unsigned long m, float *V2)
* Description    : Compute the unit normal vector with the direction given by
*                : vector V1.
* Equation       : V2 = V1 ./ norm(V1)                                        
* Input          : V1 -> Vector with size m x 1
*                : m -> Number of terms in the vector V1.
* Output         : V2 -> Unit vector with the size of m x 1.
* Return         : None
*******************************************************************************/
void VN_Normalize(float *V1, unsigned long m, float *V2){
  float nrm = VN_Norm(V1, m);
  unsigned long i;
  for(i=0; i < m; i++){
    V2[i] = V1[i] / nrm;
  }
}

/*******************************************************************************
* Function Name  : VN_TriU2TriL(float **A, unsigned long rows)
* Description    : Copys the terms in the upper right triangular portion of
*                : matrix A into the lower left portion of A such that A becomes
*                : a symmetric matrix.
* Equation       : B = triu(A) + triu(A)' - diag(diag(A))                                         
* Input          : A -> Square matrix with size rows x rows
*                : rows -> Number of rows in square matrix B
* Output         : A -> Square symmetric matrix A
* Return         : None
*******************************************************************************/
void VN_TriU2TriL(float **A, unsigned long rows){
  unsigned long i,j;
  for(i=0;i<rows;i++){
    for(j=i+1;j<rows;j++){
      A[j][i] = A[i][j];
    }
  }
}

/*******************************************************************************
* Function Name  : VN_quat2DCM(float *q, float **A)
* Description    : Convert a quaternion into to a directional cosine matrix.
* Equation       : A = quat2dcm(q)                                        
* Input          : q -> Quaternion attitude
* Output         : A -> Directional cosine matrix (3x3)
* Return         : None
*******************************************************************************/
void VN_Quat2DCM(float *q, float **A){

  /* Temporary variables */
  float t[12];
  
  t[0] = q[0]*q[0];
  t[1] = q[1]*q[1];
  t[2] = q[2]*q[2];
  t[3] = q[3]*q[3];
  t[4] = q[0]*q[1]*2.0;
  t[5] = q[0]*q[2]*2.0;
  t[6] = q[0]*q[3]*2.0;
  t[7] = q[1]*q[2]*2.0;
  t[8] = q[1]*q[3]*2.0;
  t[9] = q[2]*q[3]*2.0;
  t[10]= t[0]-t[1];
  t[11]= t[3]-t[2];
  A[0][0] =  t[10]+t[11];
  A[1][1] = -t[10]+t[11];
  A[2][2] = -t[0]-t[1]+t[2]+t[3];
  A[0][1] = t[4]+t[9];
  A[1][0] = t[4]-t[9];
  A[1][2] =  t[6]+t[7];
  A[2][1] = -t[6]+t[7];
  A[0][2] = t[5]-t[8];
  A[2][0] = t[5]+t[8];
}

/*******************************************************************************
* Function Name  : VN_YPR2DCM(float *YPR, float **A)
* Description    : Convert the given yaw, pitch, and roll into a directional
*                : cosine matrix.
* Equation       : A = ANGLE2DCM(YPR[0], YPR[1], YPR[2], 'ZYX')                                         
* Input          : YPR -> Yaw, pitch, roll as a 3x1 vector
* Output         : A -> Directional cosine matrix
* Return         : None
*******************************************************************************/
void VN_YPR2DCM(float *YPR, float **A){

  /* Temporary variables */
  float t[12];
  
  t[0] = sinf(YPR[0]);
  t[1] = cosf(YPR[0]);
  t[2] = sinf(YPR[1]);
  t[3] = cosf(YPR[1]);
  t[4] = sinf(YPR[2]);
  t[5] = cosf(YPR[2]);
  t[6] = t[4]*t[2];
  t[7] = t[5]*t[2];
  A[0][0] = t[3]*t[1];
  A[0][1] = t[3]*t[0];
  A[0][2] = -t[2];
  A[1][0] = t[6]*t[1]-t[5]*t[0];
  A[1][1] = t[6]*t[0]+t[5]*t[1];
  A[1][2] = t[4]*t[3];
  A[2][0] = t[7]*t[1]+t[4]*t[0];
  A[2][1] = t[7]*t[0]-t[4]*t[1];
  A[2][2] = t[5]*t[3];
}

/*******************************************************************************
* Function Name  : VN_MatZeros(float **A, unsigned long Arows, unsigned long Acols)
* Description    : Sets all elements of matrix A equal to zero.
* Equation       : A = zeros(Arows, Acols)                                        
* Input          : A -> Matrix with size of Arows x Acols
*                : Arows -> Number of rows in matrix A
*                : Acols -> Number of columns in matrix A
* Output         : Matrix A with all elements set to zero
* Return         : None
*******************************************************************************/
void VN_MatZeros(float **A, unsigned long Arows, unsigned long Acols)
{
  unsigned long i,j;
  
  for(i=0;i<Arows;i++){
    for(j=0;j<Acols;j++){
      A[i][j] = 0.0;
    }
  }
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler121(float *q, float *Euler121)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 1,2,1 set.
* Equation       : Euler121 = quat2angle(q, 'XYX')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler121(float *q, float *Euler121){

float t1, t2;

t1 = atan2f(q[2],q[1]);
t2 = atan2f(q[0],q[3]);

Euler121[0] = t1+t2;
Euler121[1] = 2*acosf(sqrtf(q[3]*q[3]+q[0]*q[0]));
Euler121[2] = t2-t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler123(float *q, float *Euler123)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 1,2,3 set.                                         
* Equation       : Euler123 = quat2angle(q, 'XYZ')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler123(float *q, float *Euler123){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler123[0] = atan2f(-2*(q2*q3-q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
Euler123[1] = asinf(2*(q1*q3 + q0*q2));
Euler123[2]= atan2f(-2*(q1*q2-q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler131(float *q, float *Euler131)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 1,3,1 set.                                         
* Equation       : Euler131 = quat2angle(q, 'XZX')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler131(float *q, float *Euler131){

float t1, t2;

t1 = atan2f(q[1],q[2]);
t2 = atan2f(q[0],q[3]);

Euler131[0] = t2-t1;
Euler131[1] = 2*acosf(sqrtf(q[3]*q[3]+q[0]*q[0]));
Euler131[2] = t2+t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler132(float *q, float *Euler132)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 1,3,2 set.                                         
* Equation       : Euler132 = quat2angle(q, 'XZY')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler132(float *q, float *Euler132){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler132[0] = atan2f(2*(q2*q3+q0*q1),q0*q0-q1*q1+q2*q2-q3*q3);
Euler132[1] = asinf(-2*(q1*q2-q0*q3));
Euler132[2]= atan2f(2*(q1*q3 + q0*q2),q0*q0+q1*q1-q2*q2-q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler212(float *q, float *Euler212)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 2,1,2 set.                                         
* Equation       : Euler212 = quat2angle(q, 'YXY')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler212(float *q, float *Euler212){

float t1, t2;

t1 = atan2f(q[2],q[0]);
t2 = atan2f(q[1],q[3]);

Euler212[0] = t2-t1;
Euler212[1] = 2*acosf(sqrtf(q[3]*q[3]+q[1]*q[1]));
Euler212[2] = t2+t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler213(float *q, float *Euler213)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 2,1,3 set.                                         
* Equation       : Euler213 = quat2angle(q, 'YXZ')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler213(float *q, float *Euler213){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler213[0] = atan2f(2*(q1*q3 + q0*q2),q0*q0-q1*q1-q2*q2+q3*q3);
Euler213[1] = asinf(-2*(q2*q3-q0*q1));
Euler213[2]= atan2f(2*(q1*q2+q0*q3),q0*q0-q1*q1+q2*q2-q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler231(float *q, float *Euler231)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 2,3,1 set.                                         
* Equation       : Euler231 = quat2angle(q, 'YZX')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler231(float *q, float *Euler231){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler231[0] = atan2f(-2*(q1*q3-q0*q2), q0*q0+q1*q1-q2*q2-q3*q3);
Euler231[1] = asinf(2*(q1*q2+q0*q3));
Euler231[2]= atan2f(-2*(q2*q3-q0*q1),q0*q0-q1*q1+q2*q2-q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler232(float *q, float *Euler232)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 2,3,2 set.                                         
* Equation       : Euler232 = quat2angle(q, 'YZY')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler232(float *q, float *Euler232){

float t1, t2;

t1 = atan2f(q[0],q[2]);
t2 = atan2f(q[1],q[3]);

Euler232[0] = t1+t2;
Euler232[1] = 2*acosf(sqrtf(q[3]*q[3]+q[1]*q[1]));
Euler232[2] = t2-t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler312(float *q, float *Euler312)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 3,1,2 set.                                         
* Equation       : Euler312 = quat2angle(q, 'ZXY')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler312(float *q, float *Euler312){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler312[0] = atan2f(-2*(q1*q2-q0*q3),q0*q0-q1*q1+q2*q2-q3*q3);
Euler312[1] = asinf(2*(q2*q3+q0*q1));
Euler312[2]= atan2f(-2*(q1*q3-q0*q2),q0*q0-q1*q1-q2*q2+q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler313(float *q, float *Euler313)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 3,1,3 set.                                         
* Equation       : Euler313 = quat2angle(q, 'ZXZ')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler313(float *q, float *Euler313){

float t1, t2;

t1 = atan2f(q[1],q[0]);
t2 = atan2f(q[2],q[3]);

Euler313[0] = t1+t2;
Euler313[1] = 2*acosf(sqrtf(q[3]*q[3]+q[2]*q[2]));
Euler313[2] = t2-t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler321(float *q, float *Euler321)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 3,2,1 set.                                         
* Equation       : Euler321 = quat2angle(q, 'ZYX')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler321(float *q, float *Euler321){

float q0, q1, q2, q3;

q0 = q[3];
q1 = q[0];
q2 = q[1];
q3 = q[2];

Euler321[0] = atan2f(2*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
Euler321[1] = asinf(-2*(q1*q3-q0*q2));
Euler321[2]= atan2f(2*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
}

/*******************************************************************************
* Function Name  : VN_Quat2Euler323(float *q, float *Euler323)
* Description    : Convert a quaternion attitude representation to an Euler
*                : angle 3,2,3 set.                                         
* Equation       : Euler323 = quat2angle(q, 'ZYZ')                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Euler angles
* Return         : None
*******************************************************************************/
void VN_Quat2Euler323(float *q, float *Euler323){

float t1, t2;

t1 = atan2f(q[0],q[1]);
t2 = atan2f(q[2],q[3]);

Euler323[0] = t2-t1;
Euler323[1] = 2*acosf(sqrtf(q[3]*q[3]+q[2]*q[2]));
Euler323[2] = t2+t1;
}

/*******************************************************************************
* Function Name  : VN_Quat2Gibbs(float *q, float *Gibb)
* Description    : Convert a quaternion attitude representation to the Gibbs
*                : angle representation.                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : Gibb -> Gibbs vector
* Return         : None
*******************************************************************************/
void VN_Quat2Gibbs(float *q, float *Gibb){
Gibb[0] = q[0]/q[3];
Gibb[1] = q[1]/q[3];
Gibb[2] = q[2]/q[3];
}

/*******************************************************************************
* Function Name  : VN_Quat2MRP(float *q, float *MRP)
* Description    : Convert a quaternion attitude representation to an Modified
*                : Rodrigues Parameters representation.                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : MRP -> Modified Rodrigues Parameters
* Return         : None
*******************************************************************************/
void VN_Quat2MRP(float *q, float *MRP){
MRP[0] = q[0]/(1+q[3]);
MRP[1] = q[1]/(1+q[3]);
MRP[2] = q[2]/(1+q[3]);
}

/*******************************************************************************
* Function Name  : VN_Quat2PRV(float *q, float *PRV)
* Description    : Convert a quaternion attitude representation to the principal
*                : rotatin vector.                                         
* Input          : q -> Quaternion 4x1 vector
* Output         : PRV -> Principal rotation vector
* Return         : None
*******************************************************************************/
void VN_Quat2PRV(float *q, float *PRV){
float p, sp;
p = 2*acosf(q[3]);
sp = sinf(p/2);
PRV[0] = q[0]/sp*p;
PRV[1] = q[1]/sp*p;
PRV[2] = q[2]/sp*p;
}

/*******************************************************************************
* Function Name  : VN_AddQuat(float *q1, float *q2, float *q3)
* Description    : VN_AddQuat provides the quaternion which corresponds to
*                  performing two successive rotations from q1 and q2.
* Input          : q1 -> First quaternion
*                : q2 -> Second quaternion
* Output         : q3 -> Combined sucessive rotation of q1 and q2
* Return         : None
*******************************************************************************/
void VN_AddQuat(float *q1, float *q2, float *q3){
q3[3] = q2[3]*q1[3]-q2[0]*q1[0]-q2[1]*q1[1]-q2[2]*q1[2];
q3[0] = q2[0]*q1[3]+q2[3]*q1[0]+q2[2]*q1[1]-q2[1]*q1[2];
q3[1] = q2[1]*q1[3]-q2[2]*q1[0]+q2[3]*q1[1]+q2[0]*q1[2];
q3[2] = q2[2]*q1[3]+q2[1]*q1[0]-q2[0]*q1[1]+q2[3]*q1[2];
}

/*******************************************************************************
* Function Name  : VN_SubQuat(float *q1, float *q2, float *q3)
* Description    : VN_SubQuat provides the quaternion which cooresponds to
*                  the relative rotation from q2 to q1.                                    
* Input          : q1 -> First quaternion
*                : q2 -> Second quaternion
* Output         : q3 -> Relative rotation from q2 to q1
* Return         : None
*******************************************************************************/
void VN_SubQuat(float *q1, float *q2, float *q3){
q3[3] = q2[3]*q1[3]+q2[0]*q1[0]+q2[1]*q1[1]+q2[2]*q1[2];
q3[0] = -q2[0]*q1[3]+q2[3]*q1[0]+q2[2]*q1[1]-q2[1]*q1[2];
q3[1] = -q2[1]*q1[3]-q2[2]*q1[0]+q2[3]*q1[1]+q2[0]*q1[2];
q3[2] = -q2[2]*q1[3]+q2[1]*q1[0]-q2[0]*q1[1]+q2[3]*q1[2];
}

/*******************************************************************************
* Function Name  : VN_QuatKinematicDiffEq(float *q, float *rates, float *q_dot)
* Description    : Computes the time rate of change of the quaternion paramters
*                  as a function of the angular rates. You can use this function
*                  if you need to determine how the quaternion parameters are
*                  instantaniously changing as a function of time.                                        
* Input          : q -> Current attitude quaternion
*                  rates -> angular rates [rad/s]
* Output         : q_dot -> derivative of q
* Return         : None
*******************************************************************************/
void VN_QuatKinematicDiffEq(float *q, float *rates, float *q_dot){
q_dot[0] = 0.5f * ( q[3]*rates[0]-q[2]*rates[1]+q[1]*rates[2]);
q_dot[1] = 0.5f * ( q[2]*rates[0]+q[3]*rates[1]-q[0]*rates[2]);
q_dot[2] = 0.5f * (-q[1]*rates[0]+q[0]*rates[1]+q[3]*rates[2]);
q_dot[3] = 0.5f * (-q[0]*rates[0]-q[1]*rates[1]-q[2]*rates[2]);
}

/*******************************************************************************
* Function Name  : VN_YPRKinematicDiffEq(float *YPR, float *rates, float *YPR_dot)
* Description    : Computes the time rate of change of the 321 Euler angles
*                  (yaw, pitch, roll) as a function of the angular rates. You
*                  can use this function if you need to determine how the Euler
*                  angles are instantaniously changing as a function of time.                                        
* Input          : YPR -> Yaw, Pitch, Roll angles [rad]
*                  rates -> angular rates [rad/s]
* Output         : YPR_dot -> rate of change of yaw, pitch, roll [rad/s]
* Return         : None
*******************************************************************************/
void VN_YPRKinematicDiffEq(float *YPR, float *rates, float *YPR_dot){
YPR_dot[0] =          (sin(YPR[2])/cos(YPR[1]))*rates[1] + (cos(YPR[2])/cos(YPR[1]))*rates[2];
YPR_dot[1] =                        cos(YPR[2])*rates[1] -               sin(YPR[2])*rates[2];
YPR_dot[2] = rates[0] + sin(YPR[2])*tan(YPR[1])*rates[1] +   cos(YPR[2])*tan(YPR[1])*rates[2];
}




















/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_user.c
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file contains all the functions that are hardware 
*                    : specific. These functions need to be modified by the 
*                    : user to be compatible with their hardware architecture.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "VN_user.h"
#include "VN_lib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : VN_SPI_SetSS(unsigned char sensorID, bool LineState)
* Description    : This is a generic function that will set the SPI slave select
*                  line for the given sensor. This function needs to be added by
*                  the user with the logic specific to their hardware to perform
*                  the necessary actions to either raise or lower the slave
*                  select line for the given sensor.  If a multiplexer is used
*                  then the logic/communications neccessary to perform the
*                  actions should be placed here.                                        
* Input          : sensorID  -> The sensor to set the slave select line for.
*                : state -   -> The state to set the slave select to.
* Output         : None
* Return         : None
*******************************************************************************/
void VN_SPI_SetSS(unsigned char sensorID, VN_PinState state){

/* User code to set SPI SS lines goes here. */   
  switch(sensorID){
  
    case 0:
      if(state == VN_PIN_LOW){
        /* Start SPI Transaction - Pull SPI CS line low */
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
      }else{
        /* End SPI transaction - Pull SPI CS line high */
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
      }
      break;
  }
}

/*******************************************************************************
* Function Name  : VN_SPI_SendReceiveWord(unsigned long data)
* Description    : Transmits the given 32-bit word on the SPI bus. The user needs
*                  to place their hardware specific logic here to send 4 bytes
*                  out the SPI bus. The slave select line is controlled by the 
*                  function VN_SPI_SetSS given above, so the user only needs
*                  to deal with sending the data out the SPI bus with this
*                  function.
* Input          : data -> The 32-bit data to send over the SPI bus
* Output         : None
* Return         : The data received on the SPI bus
*******************************************************************************/
unsigned long VN_SPI_SendReceive(unsigned long data){

/* User code to send out 4 bytes over SPI goes here */
  unsigned long i;
  unsigned long ret = 0;
  
  for(i=0;i<4;i++){
    /* Wait for SPI1 Tx buffer empty */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
  
    /* Send SPI1 requests */
    SPI_I2S_SendData(SPI1, VN_BYTE(data, i));
  
    /* Wait for response from VN-100 */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

    /* Save received data in buffer */
    ret |= ((unsigned long)SPI_I2S_ReceiveData(SPI1) << (8*i));    
  }
  
  return ret;
}

/*******************************************************************************
* Function Name  : VN_Delay(unsigned long delay_uS)
* Description    : Delay the processor for deltaT time in microseconds.  The user
*                  needs to place the hardware specific code here necessary to 
*                  delay the processor for the time span given by delay_uS
*                  measured in micro seconds. This function doesn't need to be
*                  ultra precise. The only requirement on this function is that
*                  the processor is delayed a time NO LESS THAN 90% of the time 
*                  given by the variable delay_uS in microseconds. The minimum
*                  timespan that is used by the VectorNav library code is 50uS so
*                  the function call shouldn't affect the timing accuracy much.
*                  If you decide to modify this library or wish to have more
*                  precision on this delay function then you can comment out this
*                  function and replace it with an optimized macro instead. Many
*                  compilers have their own delay routines or macros so make sure
*                  you check your compiler documentation before attempting to
*                  write your own.
* Input          : delay_uS -> Time to delay the processor in microseconds
* Output         : None
* Return         : None
*******************************************************************************/
void VN_Delay(unsigned long delay_uS){

/* User code to delay the processor goes here. Below is example code that
   works for a 32-bit ARM7 Cortex processor clocked at 72 MHz.  For any 
   other processor you will need to replace this with code that works
   for your processor.  Many compilers will have their own delay routines
   so make sure you check your compiler documentation before attempting to
   write your own. */
  unsigned long i;
  for(i=delay_uS*10; i--; );
}

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/


















/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN100.c
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file provides all of the firmware functions specific
*                    : to the VN100.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "VN100.h"
#include "VN_lib.h"

#ifdef _VN100
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Buffer used for SPI read and write responses */
/* Both the read and write register SPI routines below use this packet 
   to store the returned SPI response. None of the write register commands 
   implemented in this library check the data that is returned by the sensor 
   to ensure that it is consistent with the data that was sent.  For normal
   cases this isn't necessary however if you wish to implement your own
   checking then this is the structure that you need to check after each 
   register set command.  The structure has the following form:
   VN_SPI_LastReceivedPacket.CmdID -> This is the ID for the command that
                                   the response is for
   VN_SPI_LastReceivedPacket.RegID -> This is the ID for the register that
                                   the response is for
   VN_SPI_LastReceivedPacket.Data[] -> This is the data that was returned by
                                    the sensor as an array of unsigned 32-bit
                                    integers  */
VN100_SPI_Packet VN_SPI_LastReceivedPacket = {0, 0, 0, 0, {0}};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : VN100_SPI_ReadRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth)
* Description    : Read the register with the ID regID on a VN-100 sensor
*                  using the SPI interface.                                     
* Input          : sensorID -> The sensor to get the requested data from.
*                : regID -> The requested register ID number
*                : regWidth -> The width of the requested register in 32-bit words
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_ReadRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth){

  unsigned long i;

  /* Pull SS line low to start transaction*/
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Send request */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, regID, VN100_CmdID_ReadRegister));
  VN_SPI_SendReceive(0);

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Delay for 50us */
  VN_Delay(100);

  /* Pull SS line low to start SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Get response over SPI */
  for(i=0;i<=regWidth;i++){
    *(((unsigned long*)&VN_SPI_LastReceivedPacket) + i) = VN_SPI_SendReceive(0);
  }

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);  


  /* Return Error code */
  return &VN_SPI_LastReceivedPacket;  
}

/*******************************************************************************
* Function Name  : VN100_SPI_WriteRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth, unsigned long* ptrWriteValues)
* Description    : Write to the register with the ID regID on VN-100 sensor
*                  using the SPI interface.                                        
* Input          : sensorID -> The sensor to write the requested data to.
*                : regID -> The register ID number
*                : regWidth -> The width of the register in 32-bit words
* Output         : ptrWriteValues -> The data to write to the requested register.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_WriteRegister(unsigned char sensorID, unsigned char regID, unsigned char regWidth, unsigned long* ptrWriteValues){

  unsigned long i;

  /* Pull SS line low to start transaction*/
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Send write command */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, regID, VN100_CmdID_WriteRegister));
  for(i=0;i<regWidth;i++){
    VN_SPI_SendReceive(ptrWriteValues[i]);
  }

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);

  /* Delay for 50us */
  VN_Delay(100);

  /* Pull SS line low to start SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);

  /* Get response over SPI */
  for(i=0;i<4;i++){
    *(((unsigned long*)&VN_SPI_LastReceivedPacket) + i) = VN_SPI_SendReceive(0);
  }

  /* Pull SS line high to end SPI transaction */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);  


  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetModel(unsigned char sensorID, char* model)
* Description    : Read the model number from the sensor.                                       
* Input          : sensorID -> The sensor to get the model number from.
* Output         : model -> Pointer to a character array where the requested
*                           model number is placed. This needs to be a character
*                           array that is 12 characters in size.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetModel(unsigned char sensorID, char* model){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_MODEL, 3);

  /* Get model number */
  for(i=0;i<3;i++){
    *((unsigned long*)model + i) = VN_SPI_LastReceivedPacket.Data[i].UInt;
  }

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetHWRev(unsigned char sensorID, unsigned long* revision)
* Description    : Get the hardware revision for the sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : revision -> The hardware revision requested.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetHWRev(unsigned char sensorID, unsigned long* revision){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_HWREV, 1);  
  
  /* Get hardware revision */
  *revision = VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetSerial(unsigned char sensorID, unsigned long* serialNumber)
* Description    : Get the serial number from the requested sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : serialNumber -> The serial number returned by the sensor.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetSerial(unsigned char sensorID, unsigned long* serialNumber){

  unsigned long i;

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_SN, 3);  
  
  /* Get model number */
  for(i=0;i<3;i++){
    *(serialNumber + i) = VN_SPI_LastReceivedPacket.Data[i].UInt;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetFWVer(unsigned char sensorID, unsigned long* firmwareVersion)
* Description    : Get the firmware version from the requested sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : firmwareVersion -> The firmware version returned.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetFWVer(unsigned char sensorID, unsigned long* firmwareVersion){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_FWVER, 1);  
  
  /* Get hardware revision */
  *firmwareVersion = VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetBaudRate(unsigned char sensorID, VN100_BaudType baudRate)
* Description    : Get the serial baud rate from the requested sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : baudRate -> The baud rate returned by the sensor.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetBaudRate(unsigned char sensorID, VN100_BaudType* baudRate){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_SBAUD, 1);  
  
  /* Get hardware revision */
  *baudRate = (VN100_BaudType)VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetBaudRate(unsigned char sensorID, VN100_BaudType baudRate)
* Description    : Set the serial baud rate for the requested sensor.                                        
* Input          : sensorID -> The sensor to set.
* Output         : baudRate -> The baud rate to set on the sensor.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetBaudRate(unsigned char sensorID, VN100_BaudType baudRate){

  unsigned long regValue = (unsigned long)baudRate;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_SBAUD, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetADOR(unsigned char sensorID, VN100_ADORType ADOR)
* Description    : Get the ADOR register value from the requested sensor.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : ADOR -> The value returned for the ADOR register.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetADOR(unsigned char sensorID, VN100_ADORType* ADOR){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ADOR, 1);  
  
  /* Get hardware revision */
  *ADOR = (VN100_ADORType)VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetADOR(unsigned char sensorID, VN100_ADORType ADOR)
* Description    : Set the ADOR register value from the requested sensor.                                
* Input          : sensorID -> The sensor to set.
* Output         : ADOR -> The value to set the ADOR register to.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetADOR(unsigned char sensorID, VN100_ADORType ADOR){

  unsigned long regValue = (unsigned long)ADOR;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ADOR, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetADOF(unsigned char sensorID, VN100_ADOFType ADOF)
* Description    : Get the async data output frequency.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : ADOR -> The frequency returned for the ADOF register.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetADOF(unsigned char sensorID, VN100_ADOFType* ADOF){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ADOF, 1);  
  
  /* Get hardware revision */
  *ADOF = (VN100_ADOFType)VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetADOF(unsigned char sensorID, VN100_ADOFType ADOF)
* Description    : Set the async data output frequency.
* Input          : sensorID -> The sensor to set.
* Output         : ADOR -> The desired frequency of the async data output.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetADOF(unsigned char sensorID, VN100_ADOFType ADOF){

  unsigned long regValue = (unsigned long)ADOF;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ADOR, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetYPR(unsigned char sensorID, float yaw, float pitch, float roll)
* Description    : Get the measured yaw, pitch, roll orientation angles.                                        
* Input          : sensorID -> The sensor to set.
* Output         : yaw -> The yaw angle measured in degrees.
*                  pitch -> The pitch angle measured in degrees.
*                  roll -> The roll angle measured in degrees.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetYPR(unsigned char sensorID, float* yaw, float* pitch, float* roll){

  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_YPR, 3);
  
  /* Get Yaw, Pitch, Roll */
  *yaw   = VN_SPI_LastReceivedPacket.Data[0].Float;
  *pitch = VN_SPI_LastReceivedPacket.Data[1].Float;
  *roll  = VN_SPI_LastReceivedPacket.Data[2].Float;
  
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuat(unsigned char sensorID, float* q)
* Description    : Get the measured attitude quaternion. The quaternion is a 4x1
*                  vector unit vector with the fourth term q[3] as the scalar
*                  term.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> The address of the location to write the returned
*                       measured quaternion (4x1). 
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuat(unsigned char sensorID, float* q){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTN, 4);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatMag(unsigned char sensorID, float* q, float* mag)
* Description    : Get the measured attitude quaternion and magnetic vector. The
*                  quaternion is a 4x1 unit vector with the fourth term q[3] as
*                  the scalar term. The magnetic is a 3x1 vector.  The measured
*                  magnetic vector does not have any usable units.  The magnetic
*                  vector is calibrated at the factory to have a magnitude of
*                  one on the XY plane.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> The address of the location to write the returned
*                       measured quaternion (4x1).
*                  mag -> The magnetic measured vector (3x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatMag(unsigned char sensorID, float* q, float* mag){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTM, 7);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatAcc(unsigned char sensorID, float* q, float* acc)
* Description    : Get the measured attitude quaternion and acceleration vector.
*                  The quaternion is a 4x1 unit vector with the fourth term q[3]
*                  as the scalar term.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatAcc(unsigned char sensorID, float* q, float* Acc){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTA, 7);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatRates(unsigned char sensorID, float* q, float* rates)
* Description    : Get the measured attitude quaternion and angular rates.                                       
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatRates(unsigned char sensorID, float* q, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QTR, 7);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatMagAcc(unsigned char sensorID, float* q, float* mag, float* acc)
* Description    : Get the measured attitude quaternion, magnetic and acceleration.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  mag -> The magnetic measured vector (3x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatMagAcc(unsigned char sensorID, float* q, float* mag, float* Acc){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QMA, 10);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i+7].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatAccRates(unsigned char sensorID, float* q, float* acc, float* rates)
* Description    : Get the measured attitude quaternion, acceleration, and angular rates.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatAccRates(unsigned char sensorID, float* q, float* acc, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QAR, 10);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    acc[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+7].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetQuatMagAccRates(unsigned char sensorID, float* q, float* mag, float* acc, float* rates)
* Description    : Get the measured attitude quaternion, magnetic, acceleration, and angular rates.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : q -> Measured quaternion (4x1).
*                  mag -> The magnetic measured vector (3x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetQuatMagAccRates(unsigned char sensorID, float* q, float* mag, float* acc, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_QMR, 13);
  
  /* Get Quaternion */
  for(i=0;i<4;i++){
    q[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+4].Float;
  }  
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    acc[i] = VN_SPI_LastReceivedPacket.Data[i+7].Float;
  }
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+10].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetYPRMagAccRates(unsigned char sensorID, float* YPR, float* mag, float* acc, float* rates)
* Description    : Get the yaw, pitch, roll, magnetic, acceleration, and angular rates.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : YPR -> Euler angles (Yaw, Pitch, Roll) in deg.
*                  mag -> The magnetic measured vector (3x1).
*                  acc -> Measured acceleration (3x1) in m/s^2.
*                  rates -> Measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetYPRMagAccRates(unsigned char sensorID, float* YPR, float* mag, float* acc, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_YMR, 12);
  
  /* Get Euler angles */
  for(i=0;i<3;i++){
    YPR[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i+3].Float;
  }  
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    acc[i] = VN_SPI_LastReceivedPacket.Data[i+6].Float;
  }
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+9].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetDCM(unsigned char sensorID, float* DCM)
* Description    : Get the measured attitude as a directional cosine matrix.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : DCM -> Directional Cosine Matrix (9x1). The order of the terms
*                         in the matrix is {first row, second row, third row}.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetDCM(unsigned char sensorID, float **DCM){

  unsigned long i,j;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_DCM, 9);
  
  /* Get Directional Cosine Matrix */
  for(i=0;i<3;i++){
    for(j=0;j<3;j++){
      DCM[i][j] = VN_SPI_LastReceivedPacket.Data[i*3+j].Float;
    }
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMag(unsigned char sensorID, float* mag)
* Description    : Get the measured magnetic field. The measured magnetic field
*                  does not have any usable units.  The magnetic vector is
*                  calibrated at the factory to have a magnitude of one on the
*                  XY plane.                                                
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : mag -> The magnetic measured vector (3x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetMag(unsigned char sensorID, float* mag){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_MAG, 3);
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAcc(unsigned char sensorID, float* Acc)
* Description    : Get the measured acceleration. The measured acceleration has
*                  the units of m/s^2 and its range is dependent upon the gain
*                  set by the VN100_SPI_SetAccGain() function.                                                
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : Acc -> The measured acceleration (3x1) in m/s^2.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetAcc(unsigned char sensorID, float* Acc){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ACC, 3);
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetRates(unsigned char sensorID, float* rates)
* Description    : Get the measured angular rates. The measured angular rates
*                  have units of rad/s. This is the filtered angular rate and is
*                  compensated by the onboard Kalman filter to account for gyro
*                  bias drift.                                                
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : rates -> The measured angular rates (3x1) in rad/s.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetRates(unsigned char sensorID, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_GYR, 3);
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMagAccRates(unsigned char sensorID, float* mag, float* Acc, float* rates)
* Description    : Get the measured magnetic, acceleration, and angular rates.
*                  The measurements are taken in the body reference frame.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : mag -> Measured magnetic field (3x1) [Non-dimensional].
*                  Acc -> Measured acceleration (3x1) [m/s^2].
*                  rates -> Measured angular rates (3x1) [rad/s].
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetMagAccRates(unsigned char sensorID, float* mag, float* Acc, float* rates){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_MAR, 9);
  
  /* Get Magnetic */
  for(i=0;i<3;i++){
    mag[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get Acceleration */
  for(i=0;i<3;i++){
    Acc[i] = VN_SPI_LastReceivedPacket.Data[i+3].Float;
  }    
  
  /* Get Angular Rates */
  for(i=0;i<3;i++){
    rates[i] = VN_SPI_LastReceivedPacket.Data[i+6].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc)
* Description    : Get the magnetic and acceleration reference vectors. The
*                  reference vectors are the vectors measured by the magnetomter
*                  and Accerometer respectively in the inertial reference
*                  frame.  The inertial reference frame is NED (North, East, Down).                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : refMag -> The reference vector for the magnetic field.
*                  refAcc -> The reference vector for the Accerometer (gravity).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_REF, 6);
  
  /* Get magnetic reference */
  for(i=0;i<3;i++){
    refMag[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
  
  /* Get acceleration reference */
  for(i=0;i<3;i++){
    refAcc[i] = VN_SPI_LastReceivedPacket.Data[i+3].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetMagAccReference(unsigned char sensorID, float* refMag, float* refAcc)
* Description    : Set the magnetic and acceleration reference vectors. The
*                  reference vectors are the vectors measured by the magnetometer
*                  and accelerometer respectively in the inertial reference
*                  frame.  The inertial reference frame is NED (North, East, Down).                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : refMag -> The reference vector for the magnetic field.
*                  refAcc -> The reference vector for the Accelerometer (gravity).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetMagAccRef(unsigned char sensorID, float* refMag, float* refAcc){

  float ref[6];
  
  ref[0] = refMag[0];
  ref[1] = refMag[1];
  ref[2] = refMag[2];
  ref[3] = refAcc[0];
  ref[4] = refAcc[1];
  ref[5] = refAcc[2];

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_REF, 6, (unsigned long*)ref);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetFiltMeasVar(unsigned char sensorID, float* measVar)
* Description    : Get the Kalman filter measurement variance parameters. This is
*                  discussed in the User Manual in Section 6.22. The measurement
*                  variance parameters controls how much weight the Kalman filter
*                  will place on each measurement.  See application note A001 for
*                  more details on how to set these values for your specific
*                  application.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : measVar -> The variance on the measured inputs to the
*                             filter. This is a (10x1) vector.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetFiltMeasVar(unsigned char sensorID, float* measVar){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_SIG, 10);
  
  /* Get filter measurement variance */
  for(i=0;i<10;i++){
    measVar[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetFiltMeasVar(unsigned char sensorID, float* measVar)
* Description    : Set the Kalman filter measurement variance parameters. This is
*                  discussed in the User Manual in Section 6.22. The measurement
*                  variance parameters controls how much weight the Kalman filter
*                  will place on each measurement.  See application note A001 for
*                  more details on how to set these values for your specific
*                  application.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : measVar -> The variance on the measured inputs to the
*                                  filter. This is a (10x1) vector.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetFiltMeasVar(unsigned char sensorID, float* measVar){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_SIG, 10, (unsigned long*)measVar);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetHardSoftIronComp(unsigned char sensorID, float* HSI)
* Description    : Get the magnetic hard/soft iron compensation parameters. These
*                  values allow the magnetometer to compensate for distortions in
*                  the local magnetic field due to ferromagnetic materials in the
*                  vacinity of the sensor. More information on the parameters can
*                  be found in the User Manual in Section 6.23.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : HSI -> magnetic hard/soft iron paramteters (12x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetHardSoftIronComp(unsigned char sensorID, float* HSI){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_HSI, 12);
  
  /* Get magnetic hard/soft iron compensation parameters */
  for(i=0;i<12;i++){
    HSI[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetHardSoftIronComp(unsigned char sensorID, float* HSI)
* Description    : Set the magnetic hard/soft iron compensation parameters. These
*                  values allow the magnetometer to compensate for distortions in
*                  the local magnetic field due to ferromagnetic materials in the
*                  vacinity of the sensor. More information on the parameters can
*                  be found in the User Manual in Section 6.23.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : HSI -> magnetic hard/soft iron parameters (12x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetHardSoftIronComp(unsigned char sensorID, float* HSI){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_HSI, 12, (unsigned long*)HSI);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA)
* Description    : Get the filter active tuning parameters. The active tuning
*                  parameters control how the filter handles dynamic disturbances
*                  in both magnetic and acceleration.  These values are not needed
*                  for normal operation.  More on these parameters can be found in
*                  the User Manual in Section 6.24.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : gainM -> Magnetic Disturbance Gain
*                  gainA -> Acceleration Disturbance Gain
*                  memM -> Magnetic Disturbance Memory
*                  memA -> Acceleration Disturbance Gain
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetFiltActTuning(unsigned char sensorID, float* gainM, float* gainA, float* memM, float* memA){
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ATP, 6);
  
  /* Get magnetic gain */
  *gainM = VN_SPI_LastReceivedPacket.Data[0].Float;
  
  /* Get acceleration gain */
  *gainA = VN_SPI_LastReceivedPacket.Data[3].Float;
  
  /* Get magnetic memory */
  *memM = VN_SPI_LastReceivedPacket.Data[6].Float;
  
  /* Get acceleration memory */
  *memA = VN_SPI_LastReceivedPacket.Data[9].Float;

  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA)
* Description    : Set the filter active tuning parameters. The active tuning
*                  parameters control how the filter handles dynamic disturbances
*                  in both magnetic and acceleration.  These values are not needed
*                  for normal operation.  More on these parameters can be found in
*                  the User Manual in Section 6.24.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : gainM -> Magnetic Disturbance Gain
*                  gainA -> Acceleration Disturbance Gain
*                  memM -> Magnetic Disturbance Memory
*                  memA -> Acceleration Disturbance Gain
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetFiltActTuning(unsigned char sensorID, float gainM, float gainA, float memM, float memA){

  float atp[4];
  
  atp[0] = gainM;
  atp[1] = gainA;
  atp[2] = memM;
  atp[3] = memA;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ATP, 6, (unsigned long*)atp);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAccComp(unsigned char sensorID, float* AccComp)
* Description    : Get the accelerometer compensation parameters. The purpose of
*                  these parameters are explained in Section 6.25 of the User
*                  Manual. These parameters are not required for normal operation.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : AccComp -> Acceleration compensation register values.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetAccComp(unsigned char sensorID, float* AccComp){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ACT, 12);
  
  /* Get accelerometer compensation parameters */
  for(i=0;i<12;i++){
    AccComp[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetAccComp(unsigned char sensorID, float* AccComp)
* Description    : Set the accelerometer compensation parameters. The purpose of
*                  these parameters is explained in Section 6.25 of the User
*                  Manual. These parameters are not required for normal operation.                                        
* Input          : sensorID -> The sensor to get the requested data from.
                   AccComp -> Acceleration compensation register values.
* Output:        : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetAccComp(unsigned char sensorID, float* AccComp){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ACT, 12, (unsigned long*)AccComp);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetRefFrameRot(unsigned char sensorID, float* refFrameRot)
* Description    : Get the reference frame rotation matrix. This matrix allows
*                  the user to transform all measured vectors from the body
*                  reference frame of the VN-100, to any other rigidly attached
*                  coordinate frame. The effect of this transformation is that
*                  the computed attitude solution and measured measurement
*                  vectors will now be measured in the chosen coordinate system
*                  of the user and not the VN-100 coordinate system.  This is
*                  further explained in Section 6.26 of the User Manual.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : refFrameRot -> Reference frame rotation matrix (9x1).
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetRefFrameRot(unsigned char sensorID, float* refFrameRot){

  unsigned long i;
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_RFR, 12);
  
  /* Get reference frame rotation parameters */
  for(i=0;i<12;i++){
    refFrameRot[i] = VN_SPI_LastReceivedPacket.Data[i].Float;
  }
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetRefFrameRot(unsigned char sensorID, float* refFrameRot)
* Description    : Set the reference frame rotation matrix. This matrix allows
*                  the user to transform all measured vectors from the body
*                  reference frame of the VN-100, to any other rigidly attached
*                  coordinate frame. The effect of this transformation is that
*                  the computed attitude solution and measured measurement
*                  vectors will now be measured in the chosen coordinate system
*                  of the user and not the VN-100 coordinate system.  This is
*                  further explained in Section 6.26 of the User Manual.                                        
* Input          : sensorID -> The sensor to get the requested data from.
*                  refFrameRot -> Reference frame rotation matrix (9x1).
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetRefFrameRot(unsigned char sensorID, float* refFrameRot){

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_RFR, 12, (unsigned long*)refFrameRot);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAccGain(unsigned char sensorID, VN100_AccGainType gain)
* Description    : Get the current accelerometer gain setting. The accelerometer
*                  on the VN-100 can be set to either a +/- 2g or +/- 6g gain
*                  setting.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : gain -> The current accelerometer gain setting.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_GetAccGain(unsigned char sensorID, VN100_AccGainType* gain){
  
  /* Read register */
  VN100_SPI_ReadRegister(sensorID, VN100_REG_ACG, 1);
  
  /* Get accelerometer gain */
  *gain = (VN100_AccGainType)VN_SPI_LastReceivedPacket.Data[0].UInt;
    
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_SetAccGain(unsigned char sensorID, VN100_AccGainType gain)
* Description    : Set the current accelerometer gain setting. The accelerometer
*                  on the VN-100 can be set to either a +/- 2g or +/- 6g gain
*                  setting.
* Input          : sensorID -> The sensor to get the requested data from.
*                : gain -> The current accelerometer gain setting.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_SetAccGain(unsigned char sensorID, VN100_AccGainType gain){

  unsigned long regValue = (unsigned long)gain;

  /* Write register and return SPI packet*/
  return VN100_SPI_WriteRegister(sensorID, VN100_REG_ACG, 1, &regValue);
}

/*******************************************************************************
* Function Name  : VN100_SPI_RestoreFactoryDefaultSettings(unsigned char sensorID)
* Description    : Restore the selected sensor to factory default state. The
*                  values for factory default state for each register can be
*                  found in Section 7 of the User Manual.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_RestoreFactorySettings(unsigned char sensorID){
  
  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_RestoreFactorySettings));
  VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  
  /* Delay for 50 uS */
  VN_Delay(50);
  
  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Get response bytes */
  *((unsigned long*)&VN_SPI_LastReceivedPacket    ) = VN_SPI_SendReceive(0);
  *((unsigned long*)&VN_SPI_LastReceivedPacket + 1) = VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_Tare(unsigned char sensorID)
* Description    : Send a tare command to the selected VN-100. The tare command
*                  will zero out the current sensor orientation.  The attitude
*                  of the sensor will be measured form this point onwards with
*                  respect to the attitude present when the tare command was
*                  issued.  It is important with v4 of the firmware to keep
*                  the device still for at least 3 seconds after performing a
*                  tare command.  The tare command will also set the reference
*                  vectors in the inertial frame to the vectors currently
*                  measured in the body frame.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
VN100_SPI_Packet* VN100_SPI_Tare(unsigned char sensorID){
  
  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_Tare));
  VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  
  /* Delay for 50 uS */
  VN_Delay(50);
  
  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Get response bytes */
  *((unsigned long*)&VN_SPI_LastReceivedPacket    ) = VN_SPI_SendReceive(0);
  *((unsigned long*)&VN_SPI_LastReceivedPacket + 1) = VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
  
  /* Return pointer to SPI packet */
  return &VN_SPI_LastReceivedPacket;
}

/*******************************************************************************
* Function Name  : VN100_SPI_Reset(unsigned char sensorID)
* Description    : Command the given sensor to perform a device hardware reset.
*                  This is equivalent to pulling the NRST pin low on the VN-100.
*                  Any changes to any of the registers on the VN-100 that were
*                  made since last issuing a Write Settings commands will be lost.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : None
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_Reset(unsigned char sensorID){

  /* Pull SS line low */
  VN_SPI_SetSS(sensorID, VN_PIN_LOW);
  
  /* Send command over SPI */
  VN_SPI_SendReceive(VN_BYTES2WORD(0, 0, 0, VN100_CmdID_Reset));
  VN_SPI_SendReceive(0);
  
  /* Pull SS line high */
  VN_SPI_SetSS(sensorID, VN_PIN_HIGH);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetAccInertial(unsigned char sensorID, float *AccI)
* Description    : Request the inertial acceleration from the VN-100. This
*                  function will internally request both the measured acceleration
*                  and attitude from the sensor, then compute the inertial
*                  acceleration.  If you are wanting to integrate your acceleration
*                  to find velocity or position, then this is the acceleration
*                  that you want to measure. It is measured in a fixed
*                  NED (North, East, Down) coordinate frame.                                        
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : AccI -> The inertial acceleration measured by the device.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_GetAccInertial(unsigned char sensorID, float *AccI){

  /* Create a matrix for the attitude */
#if __STDC_VERSION__ >= 199901L  
  VN_CreateMatrix(A, 3, 3, {0.0});
#else
  static float A_data[9] = {0.0};
  static float *A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
  static float **A = A_ptr;
#endif

  /* Attitude quaternion */
  float q[4];
  
  /* Body acceleration vector */
  float AccB[3];
  
  /* Get the attitude quaternion and acceleration from VN-100 */
  VN100_SPI_GetQuatAcc(sensorID, q, AccB);
  
  /* Convert the quaternion into a directional cosine matrix */
  VN_Quat2DCM(q, A);
  
  /* Multiply transpose of DCM by body acceleration to get inertial acceleration */
  VN_MatTVecMult(A, AccB, 3, 3, AccI);
}

/*******************************************************************************
* Function Name  : VN100_SPI_GetMagInertial(unsigned char sensorID, float *MagI)
* Description    : Request the inertial magnetic measurement from the VN-100. This
*                  function will internally request both the measured magnetic
*                  and attitude from the sensor, then compute the inertial
*                  magnetic measurement.  It is measured in a fixed
*                  NED (North, East, Down) coordinate frame.
* Input          : sensorID -> The sensor to get the requested data from.
* Output         : MagI -> The inertial magnetic measurement measured by the
*                : device.
* Return         : Pointer to SPI packet returned by the sensor
*******************************************************************************/
void VN100_SPI_GetMagInertial(unsigned char sensorID, float *MagI){

  /* Create a matrix for the attitude */
#if __STDC_VERSION__ >= 199901L  
  VN_CreateMatrix(A, 3, 3, {0.0});
#else
  static float A_data[9] = {0.0};
  static float *A_ptr[3] = {&A_data[0], &A_data[3], &A_data[6]};
  static float **A = A_ptr;
#endif

  /* Attitude quaternion */
  float q[4];
  
  /* Body magnetic vector */
  float MagB[3];
  
  /* Get the attitude quaternion and magnetic from VN-100 */
  VN100_SPI_GetQuatMag(sensorID, q, MagB);
  
  /* Convert the quaternion into a directional cosine matrix */
  VN_Quat2DCM(q, A);
  
  /* Multiply transpose of DCM by body magnetic to get inertial magnetic */
  VN_MatTVecMult(A, MagB, 3, 3, MagI);
}

#endif /* _VN100 */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/









































/***************** (C) COPYRIGHT 2009 VectorNav Technologies *******************
* File Name          : VN_lib.h
* Author             : John Brashear
* Version            : V1.0.0
* Date               : 09/26/2009
* Description        : This file includes the device header files for the user
*                    : application.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING
* CUSTOMERS WITH EXAMPLE CODE IN ORDER TO SAVE THEM TIME. AS A RESULT,
* VECTORNAV SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR 
* CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE 
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE 
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VN_LIB_H
#define __VN_LIB_H

/* Includes ------------------------------------------------------------------*/
#include "VN_type.h"
#include "VN_math.h"
#include "VN_user.h"

#ifdef _VN100
  #include "VN100.h"
#endif /*_VN100 */


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Convert 4 bytes to a 32-bit word in the order given with b1 as the most significant byte*/
#define VN_BYTES2WORD(b1, b2, b3, b4) (((unsigned long)(b1) << 24) | ((unsigned long)(b2) << 16) | ((unsigned long)(b3) << 8) | (unsigned long)(b4))

/* Bit mask to get the 1st byte (most significant) out of a 32-bit word */
#define VN_BYTE1(word)    ((unsigned char)(((word) & 0xFF000000) >> 24))

/* Bit mask to get the 2nd byte out of a 32-bit word */
#define VN_BYTE2(word)    ((unsigned char)(((word) & 0x00FF0000) >> 16))

/* Bit mask to get the 3rd byte (most significant) out of a 32-bit word */
#define VN_BYTE3(word)    ((unsigned char)(((word) & 0x0000FF00) >> 8))

/* Bit mask to get the 1st byte (least significant) out of a 32-bit word */
#define VN_BYTE4(word)    ((unsigned char)((word) & 0x000000FF))

/* Bit mask to get the nth byte out of a 32-bit word where n=0 is most significant and n=3 is least significant */
#define VN_BYTE(word, n)   ((unsigned char)((word & (0x000000FF << (n*8))) >> (n*8)))

/* Exported functions ------------------------------------------------------- */





#endif /* __VN_LIB_H */

/******************* (C) COPYRIGHT 2009 VectorNav Technologies *****************
***********************************END OF FILE*********************************/





















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


