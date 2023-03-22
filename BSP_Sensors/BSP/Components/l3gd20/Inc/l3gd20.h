/*
 * l3gd20.h
 *
 *  Created on: Mar 13, 2023
 *      Author: Tarik
 */

#ifndef L3GD20_H_
#define L3GD20_H_
#include "stdint.h"
#include "l3gd20_int.h"

typedef void (TfL3gd20_InitFunciton)(uint16_t);
typedef void (TfL3gd20_DeInitFunction)(void);
typedef uint8_t (TfL3gd20_ReadId)(void);
typedef void (TfL3gd20_Reset)(void);
typedef void (TfL3gd20_LowPower)(uint16_t);
typedef void (TfL3gd20_ConfigureInt)(uint16_t);
typedef void (TfL3gd20_EnableInt)(uint8_t);
typedef void (TfL3gd20_DisableInt)(uint8_t);
typedef uint8_t (TfL3gd20_IntStatus)(uint16_t, uint16_t);
typedef void (TfL3gd20_ClearInt)(uint16_t, uint16_t);
typedef void (TfL3gd20_FilterConfig)(uint8_t);
typedef void (TfL3gd20_FilterCmd)(uint8_t);
typedef void (TfL3gd20_GetXYZ)(float*);

typedef struct
{
	TfL3gd20_InitFunciton *Init;
	TfL3gd20_DeInitFunction *DeInit;
	TfL3gd20_ReadId *ReadID;
	TfL3gd20_Reset *Reset;
	TfL3gd20_LowPower *LowPower;
	TfL3gd20_ConfigureInt *ConfigIT;
	TfL3gd20_EnableInt *EnableIT;
	TfL3gd20_DisableInt *DisableIT;
	TfL3gd20_IntStatus *ITStatus;
	TfL3gd20_ClearInt *ClearIT;
	TfL3gd20_FilterConfig *FilterConfig;
	TfL3gd20_FilterCmd *FilterCmd;
	TfL3gd20_GetXYZ *GetXYZ;
} TstL3gd20_DrvTypeDef;

/* Sensor Configuration Functions */
void L3GD20_Init(uint16_t InitStruct);
void L3GD20_DeInit(void);
void L3GD20_LowPower(uint16_t InitStruct);
uint8_t L3GD20_ReadID(void);
void L3GD20_RebootCmd(void);

/* Interrupt Configuration Functions */
void L3GD20_INT1InterruptConfig(uint16_t Int1Config);
void L3GD20_EnableIT(uint8_t IntSel);
void L3GD20_DisableIT(uint8_t IntSel);

/* High Pass Filter Configuration Functions */
void L3GD20_FilterConfig(uint8_t FilterStruct);
void L3GD20_FilterCmd(uint8_t HighPassFilterState);
void L3GD20_ReadXYZAngRate(float *pfData);
uint8_t L3GD20_GetDataStatus(void);

void Gyro_l3gd20_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void Gyro_l3gd20_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);




#endif /* L3GD20_H_ */
