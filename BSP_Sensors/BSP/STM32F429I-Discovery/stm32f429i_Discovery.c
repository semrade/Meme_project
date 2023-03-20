/*
 * stm32f429i-Dsicovery.c
 *
 *  Created on: 14 mars 2023
 *      Author: macbookpro
 *      this file contains all the services for read, write, configure and init.
 */
#include <stm32f429i_Discovery.h>



static uint8_t SPI3_WriteRead(uint8_t Byte);
static uint8_t SPI5_WriteRead(uint8_t Byte);
/***************************************3D ACC and 3D Gyro Function Section******************************************/


/**
 * @brief  Writes one byte to the Gyroscope.
 * @param  pBuffer: Pointer to the buffer containing the data to be written to the Gyroscope.
 * @param  WriteAddr: Gyroscope's internal address to write to.
 * @param  NumByteToWrite: Number of bytes to write.
 */
void Acc_Gyro_Imc330dlc_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	/* Configure the MS bit:
	 - When 0, the address will remain unchanged in multiple read/write commands.
	 - When 1, the address will be auto incremented in multiple read/write commands.
	 */
	if (NumByteToWrite > 0x01) {
		WriteAddr |= (uint8_t) MULTIPLEBYTE_CMD;
	}
	/* Set chip select Low at the start of the transmission */
	GYRO_ACC_CS_LOW();

	/* Send the Address of the indexed register */
	SPI3_WriteRead(WriteAddr);

	/* Send the data that will be written into the device (MSB First) */
	while (NumByteToWrite >= 0x01) {
		SPI3_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}

	/* Set chip select High at the end of the transmission */
	GYRO_ACC_CS_HIGH();
}

/**
 * @brief  Reads a block of data from the Gyroscope.
 * @param  pBuffer: Pointer to the buffer that receives the data read from the Gyroscope.
 * @param  ReadAddr: Gyroscope's internal address to read from.
 * @param  NumByteToRead: Number of bytes to read from the Gyroscope.
 */
void Acc_Gyro_Imc330dlc_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	if (NumByteToRead > 0x01) {
		ReadAddr |= (uint8_t) (READWRITE_CMD | MULTIPLEBYTE_CMD);
	} else {
		ReadAddr |= (uint8_t) READWRITE_CMD;
	}
	/* Set chip select Low at the start of the transmission */
	GYRO_ACC_CS_LOW();

	/* Send the Address of the indexed register */
	SPI3_WriteRead(ReadAddr);

	/* Receive the data that will be read from the device (MSB First) */
	while (NumByteToRead > 0x00) {
		/* Send dummy byte (0x00) to generate the SPI clock to Gyroscope (Slave device) */
		*pBuffer = SPI3_WriteRead(DUMMY_BYTE);
		NumByteToRead--;
		pBuffer++;
	}

	/* Set chip select High at the end of the transmission */
	GYRO_ACC_CS_HIGH();
}

static uint8_t SPI3_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte = 0;

  /* Send a Byte through the SPI peripheral */
  /* Read byte from the SPI bus */
  if(HAL_SPI_TransmitReceive(&hspi3, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, 0X1000) != HAL_OK)
  {
    //SPIx_Error();
  }

  return receivedbyte;
}
/***************************************3D Gyro Dico Board Function Section******************************************/




/**
 * @brief  Writes one byte to the Gyroscope.
 * @param  pBuffer: Pointer to the buffer containing the data to be written to the Gyroscope.
 * @param  WriteAddr: Gyroscope's internal address to write to.
 * @param  NumByteToWrite: Number of bytes to write.
 */
void Gyro_l3gd20_Write(uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	/* Configure the MS bit:
	 - When 0, the address will remain unchanged in multiple read/write commands.
	 - When 1, the address will be auto incremented in multiple read/write commands.
	 */
	if (NumByteToWrite > 0x01) {
		WriteAddr |= (uint8_t) MULTIPLEBYTE_CMD;
	}
	/* Set chip select Low at the start of the transmission */
	GYRO_CS_LOW();

	/* Send the Address of the indexed register */
	SPI5_WriteRead(WriteAddr);

	/* Send the data that will be written into the device (MSB First) */
	while (NumByteToWrite >= 0x01) {
		SPI5_WriteRead(*pBuffer);
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
void Gyro_l3gd20_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	if (NumByteToRead > 0x01) {
		ReadAddr |= (uint8_t) (READWRITE_CMD | MULTIPLEBYTE_CMD);
	} else {
		ReadAddr |= (uint8_t) READWRITE_CMD;
	}
	/* Set chip select Low at the start of the transmission */
	GYRO_CS_LOW();

	/* Send the Address of the indexed register */
	SPI5_WriteRead(ReadAddr);

	/* Receive the data that will be read from the device (MSB First) */
	while (NumByteToRead > 0x00) {
		/* Send dummy byte (0x00) to generate the SPI clock to Gyroscope (Slave device) */
		*pBuffer = SPI5_WriteRead(DUMMY_BYTE);
		NumByteToRead--;
		pBuffer++;
	}

	/* Set chip select High at the end of the transmission */
	GYRO_CS_HIGH();
}







static uint8_t SPI5_WriteRead(uint8_t Byte)
{
  uint8_t receivedbyte = 0;

  /* Send a Byte through the SPI peripheral */
  /* Read byte from the SPI bus */
  if(HAL_SPI_TransmitReceive(&hspi5, (uint8_t*) &Byte, (uint8_t*) &receivedbyte, 1, 0X1000) != HAL_OK)
  {
    //SPIx_Error();
  }

  return receivedbyte;
}
