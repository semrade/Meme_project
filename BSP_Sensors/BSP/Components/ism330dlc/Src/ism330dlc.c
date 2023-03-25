/*
 * imc330dlc.c
 *
 *  Created on: Mar 13, 2023
 *      Author: Tarik
 */

/* Includes ------------------------------------------------------------------*/
#include <ism330dlc.h>
#include "stm32f429i_Discovery.h"



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief L3GD20 De-initialization
  * @param  None
  * @retval None
  */
void ISM330DLC_Init(uint16_t InitStruct)
{

}
/**
  * @brief L3GD20 De-initialization
  * @param  None
  * @retval None
  */
void ISM330DLC_DeInit(uint16_t InitStruct)
{

}
/**
 * @brief  Read ID address of L3GD20
 * @param  None
 * @retval ID name
 */
uint8_t ISM330DLC_ReadID(void)
{
	uint8_t tmp;

	Acc_Gyro_Ism330dlc_Read(&tmp,ISM330DLC_WHO_AM_I, 1);

	return tmp;
}

TstISM330DLC_DrvTypeDef Ism330dlcDrvDefinition =
{
	.ReadID = ISM330DLC_ReadID
};
