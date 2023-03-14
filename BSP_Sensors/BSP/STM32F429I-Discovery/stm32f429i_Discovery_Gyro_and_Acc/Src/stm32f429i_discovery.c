/*
 * stm32f429i_discovery.c
 *
 *  Created on: 13 mars 2023
 *      Author: Tarik
 */

#include "main.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f429i_discovery.h"



extern SPI_HandleTypeDef hspi3;
static uint8_t SPIx_WriteRead(uint8_t Byte);

static GYRO_DrvTypeDef *GyroscopeDrv;
static TL3GD20_iDriver_Description GyroscopeDriver;
static ISM330DLC_iDriver_Description GyroAccDriver;
uint16_t Update_Regiter_Flag = 0u;
uint8_t u8_TL3GD20_Reg [28] = {0};
uint8_t u8_ISM330DLC_Reg [104] = {0};

const TL3GD20_iDriver_Description Default_Config =
{
	.TL3GD20_Reg_Struct.Who_Am_I  	   		= 0b11010100,
	.TL3GD20_Reg_Struct.CTRL_REG1.All	   	= 0b00000111,
	.TL3GD20_Reg_Struct.CTRL_REG2.All	   	= 0b00000000,
	.TL3GD20_Reg_Struct.CTRL_REG3.All	   	= 0b00000000,
	.TL3GD20_Reg_Struct.CTRL_REG4.All	   	= 0b00000000,
	.TL3GD20_Reg_Struct.CTRL_REG5.All	   	= 0b00000000,
	.TL3GD20_Reg_Struct.REFERNCE.All  	   	= 0b00000000,
	.TL3GD20_Reg_Struct.OUT_TEMP.All  	  	= 0b00000000,
	.TL3GD20_Reg_Struct.STATUS_REG.All    	= 0b00000000,
	.TL3GD20_Reg_Struct.OUT_X_L   	   		= 0b00000000,
	.TL3GD20_Reg_Struct.OUT_X_H 	   		= 0b00000000,
	.TL3GD20_Reg_Struct.OUT_Y_L 	   		= 0b00000000,
	.TL3GD20_Reg_Struct.OUT_Y_H 	   		= 0b00000000,
	.TL3GD20_Reg_Struct.OUT_Z_L 	   		= 0b00000000,
	.TL3GD20_Reg_Struct.OUT_Z_H 	   		= 0b00000000,
	.TL3GD20_Reg_Struct.FIFO_CTRL_REG.All 	= 0b00000000,
	.TL3GD20_Reg_Struct.FIFO_SRC_REG.All  	= 0b00000000,
	.TL3GD20_Reg_Struct.INT1_CFG.All 	   	= 0b00000000,
	.TL3GD20_Reg_Struct.INT1_SRC.All 	   	= 0b00000000,
	.TL3GD20_Reg_Struct.INT1_TSH_XH  		= 0b00000000,
	.TL3GD20_Reg_Struct.INT1_TSH_XL  		= 0b00000000,
	.TL3GD20_Reg_Struct.INT1_TSH_YH  		= 0b00000000,
	.TL3GD20_Reg_Struct.INT1_TSH_YL  		= 0b00000000,
	.TL3GD20_Reg_Struct.INT1_TSH_ZH  		= 0b00000000,
	.TL3GD20_Reg_Struct.INT1_TSH_ZL  		= 0b00000000,
	.TL3GD20_Reg_Struct.INT1_DURATION 		= 0b00000000
};

uint8_t TL3GD20_Write_Read [] =
{
	0,
	1,	 //r    01
	0,
	3,	 //rw   11
	3,	 //rw
	3,	 //rw
	3,	 //rw
	3,	 //rw
	3,	 //rw
	1,	 //r
	1,	 //r
	1,	 //r
	1,	 //r
	1,	 //r
	1,	 //r
	1,	 //r
	1,	 //r
	3,	 //rw
	1,	 //r
	3,	 //rw
	1,	 //r
	3,	 //rw
	3,	 //rw
	3,	 //rw
	3,	 //rw
	3,	 //rw
	3,	 //rw
	3	 //rw
};
uint8_t TL3GD20_Reg_Addresse [] =
{
	0xFF,
	0x0F,
	0xFF,
	0x20,
	0x21,
	0x22,
	0x23,
	0x24,
	0x25,
	0x26,
	0x27,
	0x28,
	0x29,
	0x2A,
	0x2B,
	0x2C,
	0x2D,
	0x2E,
	0x2F,
	0x30,
	0x31,
	0x32,
	0x33,
	0x34,
	0x35,
	0x36,
	0x37,
	0x38

};

Reg_Type_Address IMS330DLC_Reg_Type_Adr [] =
{	//adr          //Type	           //R/W
	{.adr=0x00,    .type=0xFF},	       //-         //FF Reserved
	{.adr=0x01,    .type=0x03},	       //r/w       //0x02 Read
	{.adr=0x02,    .type=0xFF},	       //-         //0x03 Write
	{.adr=0x03,    .type=0xFF},        //-
	{.adr=0x04,    .type=0x03},        //r/w
	{.adr=0x05,    .type=0x03},        //r/w
	{.adr=0x06,    .type=0x03},        //r/w
	{.adr=0x07,    .type=0x03},        //r/w
	{.adr=0x08,    .type=0x03},        //r/w
	{.adr=0x09,    .type=0x03},        //r/w
	{.adr=0x0A,    .type=0x03},        //r/w
	{.adr=0x0B,    .type=0x03},        //r/w
	{.adr=0x0C,    .type=0xFF},        //-
	{.adr=0x0D,    .type=0x03},        //r/w
	{.adr=0x0E,    .type=0x03},        //r/w
	{.adr=0x0F,    .type=0x02},        //r
	{.adr=0x10,    .type=0x03},        //r/w
	{.adr=0x11,    .type=0x03},        //r/w
	{.adr=0x12,    .type=0x03},        //r/w
	{.adr=0x13,    .type=0x03},        //r/w
	{.adr=0x14,    .type=0x03},        //r/w
	{.adr=0x15,    .type=0x03},        //r/w
	{.adr=0x16,    .type=0x03},        //r/w
	{.adr=0x17,    .type=0x03},        //r/w
	{.adr=0x18,    .type=0x03},        //r/w
	{.adr=0x19,    .type=0x03},        //r/w
	{.adr=0x1A,    .type=0x03},        //r/w
	{.adr=0x1B,    .type=0x02},        //r
	{.adr=0x1C,    .type=0x02},        //r
	{.adr=0x1D,    .type=0x02},        //r
	{.adr=0x1E,    .type=0x02},        //r
	{.adr=0x1F,    .type=0xFF},        //-
	{.adr=0x20,    .type=0x02},        //r
	{.adr=0x21,    .type=0x02},        //r
	{.adr=0x22,    .type=0x02},        //r
	{.adr=0x23,    .type=0x02},        //r
	{.adr=0x24,    .type=0x02},        //r
	{.adr=0x25,    .type=0x02},        //r
	{.adr=0x26,    .type=0x02},        //r
	{.adr=0x27,    .type=0x02},        //r
	{.adr=0x28,    .type=0x02},        //r
	{.adr=0x29,    .type=0x02},        //r
	{.adr=0x2A,    .type=0x02},        //r
	{.adr=0x2B,    .type=0x02},        //r
	{.adr=0x2C,    .type=0x02},        //r
	{.adr=0x2D,    .type=0x02},        //r
	{.adr=0x2E,    .type=0x02},        //r
	{.adr=0x2F,    .type=0x02},        //r
	{.adr=0x30,    .type=0x02},        //r
	{.adr=0x31,    .type=0x02},        //r
	{.adr=0x32,    .type=0x02},        //r
	{.adr=0x33,    .type=0x02},        //r
	{.adr=0x34,    .type=0x02},        //r
	{.adr=0x35,    .type=0x02},        //r
	{.adr=0x36,    .type=0x02},        //r
	{.adr=0x37,    .type=0x02},        //r
	{.adr=0x38,    .type=0x02},        //r
	{.adr=0x39,    .type=0x02},        //r
	{.adr=0x3A,    .type=0x02},        //r
	{.adr=0x3B,    .type=0x02},        //r
	{.adr=0x3C,    .type=0x02},        //r
	{.adr=0x3D,    .type=0x02},        //r
	{.adr=0x3E,    .type=0x02},        //r
	{.adr=0x3F,    .type=0x02},        //r
	{.adr=0x40,    .type=0x02},        //r
	{.adr=0x41,    .type=0x02},        //r
	{.adr=0x42,    .type=0x03},        //r/w
	{.adr=0x43,    .type=0xFF},        //-
	{.adr=0x4D,    .type=0x02},        //r
	{.adr=0x4E,    .type=0x02},        //r
	{.adr=0x4F,    .type=0x02},        //r
	{.adr=0x50,    .type=0x02},        //r
	{.adr=0x51,    .type=0x02},        //r
	{.adr=0x52,    .type=0x02},        //r
	{.adr=0x53,    .type=0x02},        //r
	{.adr=0x54,    .type=0x02},        //r
	{.adr=0x55,    .type=0xFF},        //-
	{.adr=0x58,    .type=0x03},        //r/w
	{.adr=0x59,    .type=0x03},        //r/w
	{.adr=0x5A,    .type=0x03},        //r/w
	{.adr=0x5B,    .type=0x03},        //r/w
	{.adr=0x5C,    .type=0x03},        //r/w
	{.adr=0x5D,    .type=0x03},        //r/w
	{.adr=0x5E,    .type=0x03},        //r/w
	{.adr=0x5F,    .type=0x03},        //r/w
	{.adr=0x60,    .type=0x03},        //r/w
	{.adr=0x61,    .type=0x03},        //r/w
	{.adr=0x62,    .type=0xFF},        //-
	{.adr=0x66,    .type=0x02},        //r
	{.adr=0x67,    .type=0x02},        //r
	{.adr=0x68,    .type=0x02},        //r
	{.adr=0x69,    .type=0x02},        //r
	{.adr=0x6A,    .type=0x02},        //r
	{.adr=0x6B,    .type=0x02},        //r
	{.adr=0x6C,    .type=0xFF},        //-
	{.adr=0x6F,    .type=0x03},        //r/w
	{.adr=0x70,    .type=0x03},        //r/w
	{.adr=0x71,    .type=0x03},        //r/w
	{.adr=0x72,    .type=0x03},        //r/w
	{.adr=0x73,    .type=0x03},        //r/w
	{.adr=0x74,    .type=0x03},        //r/w
	{.adr=0x75,    .type=0x03},        //r/w
	{.adr=0x76,    .type=0xFF},        //-
};
uint8_t IMS330DLC_Reg_Adresse [] =
{


};

/**
  * @brief  Writes one byte to the Gyroscope.
  * @param  pBuffer: Pointer to the buffer containing the data to be written to the Gyroscope.
  * @param  WriteAddr: Gyroscope's internal address to write to.
  * @param  NumByteToWrite: Number of bytes to write.
  */
void ACC_IO_Write(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  /* Configure the MS bit:
       - When 0, the address will remain unchanged in multiple read/write commands.
       - When 1, the address will be auto incremented in multiple read/write commands.
  */
  if(NumByteToWrite > 0x01)
  {
    WriteAddr |= (uint8_t)MULTIPLEBYTE_CMD;
  }
  /* Set chip select Low at the start of the transmission */
  ACC_CS_LOW();

  /* Send the Address of the indexed register */
  SPIx_WriteRead(WriteAddr);

  /* Send the data that will be written into the device (MSB First) */
  while(NumByteToWrite >= 0x01)
  {
    SPIx_WriteRead(*pBuffer);
    NumByteToWrite--;
    pBuffer++;
  }

  /* Set chip select High at the end of the transmission */
  GYRO_CS_HIGH();
}

/**
  * @brief  Reads a block of data from the Gyroscope.
  * @param  pBuffer: Pointer to the buffer that receives the data read from the Gyroscope.
  * @param  ReadAddr: Gyroscope's internal address to read from.
  * @param  NumByteToRead: Number of bytes to read from the Gyroscope.
  */
void ACC_IO_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead) {
	if (NumByteToRead > 0x01) {
		ReadAddr |= (uint8_t) (READWRITE_CMD | MULTIPLEBYTE_CMD);
	} else {
		ReadAddr |= (uint8_t) READWRITE_CMD;
	}
	/* Set chip select Low at the start of the transmission */
	ACC_CS_LOW();

	/* Send the Address of the indexed register */
	SPIx_WriteRead(ReadAddr);

	/* Receive the data that will be read from the device (MSB First) */
	while (NumByteToRead > 0x00) {
		/* Send dummy byte (0x00) to generate the SPI clock to Gyroscope (Slave device) */
		*pBuffer = SPIx_WriteRead(DUMMY_BYTE);
		NumByteToRead--;
		pBuffer++;
	}

	/* Set chip select High at the end of the transmission */
	GYRO_CS_HIGH();
}

static uint8_t SPIx_WriteRead(uint8_t Byte) {
	uint8_t receivedbyte = 0;

	/* Send a Byte through the SPI peripheral */
	/* Read byte from the SPI bus */
	if (HAL_SPI_TransmitReceive(&hspi3, (uint8_t*) &Byte,
			(uint8_t*) &receivedbyte, 1, SpixTimeout) != HAL_OK) {
		//SPIx_Error();
	}

	return receivedbyte;
}


void BSP_GYRO_10ms_Task(void)
{


	//read write gyro
	BSP_GYRO_Send_Regi(&GyroscopeDriver,1);


}

void BSP_GYRO_Init_task(void)
{

	//Send the default configuration
	//BSP_GYRO_Send_Regi(&Default_Config);

	GyroscopeDriver.TL3GD20_Reg_Struct.Who_Am_I = I_AM_L3GD20;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG1.All = L3GD20_OUTPUT_DATARATE_1|L3GD20_BANDWIDTH_4|L3GD20_MODE_ACTIVE|L3GD20_AXES_ENABLE;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG2.All = L3GD20_HPM_NORMAL_MODE_RES|L3GD20_HPFCF_0;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG3.All = 0;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG4.All = L3GD20_BLE_LSB|L3GD20_FULLSCALE_2000|L3GD20_BlockDataUpdate_Continous;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG5.All = L3GD20_HIGHPASSFILTER_DISABLE;

	BSP_GYRO_Send_Regi(&GyroscopeDriver,1);

}

void BSP_GYRO_Send_Regi(TL3GD20_iDriver_Description *map, uint8_t Num)
{
	uint8_t index = 0u;

	if(L3gd20Drv.ReadID() == map->TL3GD20_Reg_Struct.Who_Am_I)
	{
		for (index = 3; index < (sizeof(map->TL3GD20_Reg_Tab)/sizeof(map->TL3GD20_Reg_Tab[0])); index++)
		{
			if (0xFF != TL3GD20_Reg_Addresse[index])
			{
				//check if any bit has been changed
				if (u8_TL3GD20_Reg[index] != map->TL3GD20_Reg_Tab[index])
				{
					//bit has been changed in the driver register updated it
					Update_Regiter_Flag = 1;
				}
				//loop through the buffer and read write if necessary
				if (1 == TL3GD20_Write_Read[index])
				{
					//read
					GYRO_IO_Read(&map->TL3GD20_Reg_Tab[index], TL3GD20_Reg_Addresse[index], Num);

					// Read data to a local buffer
					u8_TL3GD20_Reg[index] = map->TL3GD20_Reg_Tab[index];
				}
				else if (3 == TL3GD20_Write_Read[index])
				{
					if ((1u == Update_Regiter_Flag))
					{
						//write
						GYRO_IO_Write(&map->TL3GD20_Reg_Tab[index], TL3GD20_Reg_Addresse[index], Num);
						//reset the flag when write is done!
						Update_Regiter_Flag = 0;
					}
					//read
					GYRO_IO_Read(&map->TL3GD20_Reg_Tab[index], TL3GD20_Reg_Addresse[index], Num);
					//if write read is not good issue a problem.

					// Read data to a local buffer
					u8_TL3GD20_Reg[index] = map->TL3GD20_Reg_Tab[index];
				}
				else
				{

				}
			}
		}
	}
}

uint16_t BSP_ISM330DLC_ReadWrite_Reg(ISM330DLC_iDriver_Description *GyroAccDriverReg, uint16_t Num)
{
	uint16_t index;
	if(L3gd20Drv.ReadID() == GyroAccDriverReg->ISM330DLC_Reg_Struct.WHO_AM_I)
	{
		for (index = 0; index < (sizeof(GyroAccDriverReg->ISM330DLC_Reg_Tab)/sizeof(GyroAccDriverReg->ISM330DLC_Reg_Tab[0])); index++)
		{
			if (0xFF != IMS330DLC_Reg_Type_Adr[index].adr)
			{
				//check if any bit has been changed
				if (u8_ISM330DLC_Reg[index] != GyroAccDriverReg->ISM330DLC_Reg_Tab[index])
				{
					//bit has been changed in the driver register updated it
					ISM330DLC_Update_Regiter_Flag = 1;
				}
				//loop through the buffer and read write if necessary
				if (0x02 == IMS330DLC_Reg_Type_Adr[index].type)
				{
					//read
					GYRO_IO_Read(&GyroAccDriverReg->ISM330DLC_Reg_Tab[index], IMS330DLC_Reg_Type_Adr[index].adr, Num);

					// Read data to a local buffer
					u8_TL3GD20_Reg[index] = map->TL3GD20_Reg_Tab[index];
				}
				else if (0x03 == IMS330DLC_Reg_Type_Adr[index].type)
				{
					if ((1u == ISM330DLC_Update_Regiter_Flag))
					{
						//write
						GYRO_IO_Write(&GyroAccDriverReg->ISM330DLC_Reg_Tab[index], IMS330DLC_Reg_Type_Adr[index].adr, Num);
						//reset the flag when write is done!
						ISM330DLC_Update_Regiter_Flag = 0;
					}
					//read
					GYRO_IO_Read(&GyroAccDriverReg->ISM330DLC_Reg_Tab[index], IMS330DLC_Reg_Type_Adr[index].adr, Num);
					//if write read is not good issue a problem.

					// Read data to a local buffer
					u8_ISM330DLC_Reg[index] = GyroAccDriverReg->ISM330DLC_Reg_Tab[index];
				}
				else
				{

				}
			}
		}
		return 1;
	}
	else
	{
		return -1;
	}
}
