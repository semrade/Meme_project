/*
 * stm32f429i-Dsicovery.h
 *
 *  Created on: 14 mars 2023
 *      Author: macbookpro
 */

#ifndef STM32F429I_DISCOVERY_H_
#define STM32F429I_DISCOVERY_H_
#include "main.h"
#include "stdint.h"

/* Those are defined by cube IDE*/
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi5;



/*################################ GYROSCOPE and ACCELEROMERTERT #################################*/
/* Read/Write command */
#define READWRITE_CMD              ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

/* Chip Select macro definition for ism330dlc */
#define GYRO_ACC_CS_LOW()       HAL_GPIO_WritePin(NCS_MEMS_SPI3_GPIO_Port, NCS_MEMS_SPI3_Pin, GPIO_PIN_RESET)
#define GYRO_ACC_CS_HIGH()      HAL_GPIO_WritePin(NCS_MEMS_SPI3_GPIO_Port, NCS_MEMS_SPI3_Pin, GPIO_PIN_SET)

#define GYRO_CS_LOW()       	HAL_GPIO_WritePin(NCS_MEMS_SPI_GPIO_Port, NCS_MEMS_SPI_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()       	HAL_GPIO_WritePin(NCS_MEMS_SPI_GPIO_Port, NCS_MEMS_SPI_Pin, GPIO_PIN_SET)

void Acc_Gyro_Imc330dlc_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void Acc_Gyro_Imc330dlc_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);


void Gyro_l3gd20_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void Gyro_l3gd20_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

#endif /* STM32F429I_DISCOVERY_H_ */
