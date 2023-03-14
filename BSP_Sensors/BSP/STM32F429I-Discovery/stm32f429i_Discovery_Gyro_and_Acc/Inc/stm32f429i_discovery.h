/*
 * stm32f429i_discovery.h
 *
 *  Created on: 13 mars 2023
 *      Author: Tarik
 */

#ifndef STM32F429I_DISCOVERY_H_
#define STM32F429I_DISCOVERY_H_

/*################################ 3D accelerometer and 3D gyroscope #################################*/
#define READWRITE_CMD              ((uint8_t)0x80)
/* Multiple byte read/write command */
#define MULTIPLEBYTE_CMD           ((uint8_t)0x40)
/* Dummy Byte Send by the SPI Master device in order to generate the Clock to the Slave device */
#define DUMMY_BYTE                 ((uint8_t)0x00)

typedef struct Type_Address
{
	uint8_t adr;
	uint8_t type;
} Reg_Type_Address;

/* Chip Select macro definition */
#define ACC_CS_LOW()       HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET)
#define ACC_CS_HIGH()      HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin,  GPIO_PIN_SET)




void ACC_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
void ACC_IO_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);


//Tarik dev
void BSP_GYRO_10ms_Task(void);
void BSP_GYRO_Init_task(void);
void BSP_GYRO_Send_Regi(TL3GD20_iDriver_Description *map, uint8_t Num);
uint16_t BSP_ISM330DLC_ReadWrite_Reg(ISM330DLC_iDriver_Description *GyroAccDriverReg, uint16_t Num);
#endif /* STM32F429I_DISCOVERY_H_ */
