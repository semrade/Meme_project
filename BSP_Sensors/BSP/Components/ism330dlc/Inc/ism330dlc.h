/*
 * imc330dlc.h
 *
 *  Created on: Mar 13, 2023
 *      Author: Tarik
 */

#ifndef IMC330DLC_H_
#define IMC330DLC_H_
#include "stdint.h"
#include "ism330dlc_int.h"


// function typedef

typedef void (Tism330dlc_InitFunciton)(uint16_t);
typedef void (Tism330dlc_DeInitFunction)(void);
typedef uint8_t (Tism330dlc_ReadId)(void);
typedef void (Tism330dlc_Reset)(void);
typedef void (Tism330dlc_LowPower)(uint16_t);
typedef void (Tism330dlc_ConfigureInt)(uint16_t);
typedef void (Tism330dlc_EnableInt)(uint8_t);
typedef void (Tism330dlc_DisableInt)(uint8_t);
typedef uint8_t (Tism330dlc_IntStatus)(uint16_t, uint16_t);
typedef void (Tism330dlc_ClearInt)(uint16_t, uint16_t);
typedef void (Tism330dlc_FilterConfig)(uint8_t);
typedef void (Tism330dlc_FilterCmd)(uint8_t);
typedef void (Tism330dlc_GetXYZ)(float*);
// function descriptor
typedef struct
{
	Tism330dlc_ReadId *ReadID;
} TstISM330DLC_DrvTypeDef;


void ISM330DLC_Init(uint16_t InitStruct);
void ISM330DLC_DeInit(uint16_t InitStruct);
uint8_t ISM330DLC_ReadID(void);

void Acc_Gyro_Imc330dlc_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void Acc_Gyro_Ism330dlc_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);


#endif /* IMC330DLC_H_ */
