/*
 * stm32f429i_discovery.c
 *
 *  Created on: 13 mars 2023
 *      Author: Tarik
 */
#include "main.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f429i_Discovery.h"
#include "stm32f429i_discovery_Gyro_Acc.h"
#include "l3gd20.h"
#include "ism330dlc.h"
#include "ism330dlc_reg.h"
#include "string.h"

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
//static GYRO_DrvTypeDef *GyroscopeDrv;
static TL3GD20_iDriver_Description GyroscopeDriver;
uint8_t u8_TL3GD20_Reg [L3GD20_MAX_BUFFER_REG] = {0};
TstL3gd20_DrvTypeDef *Gyroscope;
uint16_t TL3GD20_Update_Regiter_Flag = 0;

static ISM330DLC_iDriver_Description GyroAccDriver;
uint8_t u8_ISM330DLC_Reg [ISM330DLC_MAX_BUFFER_REG] = {0};
TstISM330DLC_DrvTypeDef *GyroAcc;
uint16_t ISM330DLC_Update_Regiter_Flag = 0u;

typedef struct
{
	uint32_t interrupt_counter;
	uint32_t loop_counter;
	uint32_t id_failure;
}debug;
debug Debugvariable;
/* Extern variables ----------------------------------------------------------*/
extern TstL3gd20_DrvTypeDef L3gd20DrvDefinition;
extern TstISM330DLC_DrvTypeDef Ism330dlcDrvDefinition;

/* Private functions ---------------------------------------------------------*/

/* Main Example --------------------------------------------------------------*/



Reg_Type_Address TL3GD20_Reg_Type_Adr [] =
{
	{.adr=0xFF,    .type=0xFF},  //r    10              Reserved
	{.adr=0x0F,    .type=0X02},  //r	                WHO_AM_I
	{.adr=0xFF,    .type=0XFF},  //rw   11	            Reserved
	{.adr=0x20,    .type=0x03},  //rw		            CTRL_REG1
	{.adr=0x21,    .type=0x03},  //rw		            CTRL_REG2
	{.adr=0x22,    .type=0x03},  //rw                   CTRL_REG3
	{.adr=0x23,    .type=0x03},  //rw                   CTRL_REG4
	{.adr=0x24,    .type=0x03},  //rw                   CTRL_REG5
	{.adr=0x25,    .type=0x03},  //rw                   REFERENCE
	{.adr=0x26,    .type=0x02},  //r                    OUT_TEMP
	{.adr=0x27,    .type=0x02},  //r                    STATUS_REG
	{.adr=0x28,    .type=0x02},  //r                    OUT_X_L
	{.adr=0x29,    .type=0x02},  //r                    OUT_X_H
	{.adr=0x2A,    .type=0x02},  //r                    OUT_Y_L
	{.adr=0x2B,    .type=0x02},  //r                    OUT_Y_H
	{.adr=0x2C,    .type=0x02},  //r                    OUT_Z_L
	{.adr=0x2D,    .type=0x02},  //r                    OUT_Z_H
	{.adr=0x2E,    .type=0x03},  //rw                   FIFO_CTRL_REG
	{.adr=0x2F,    .type=0x02},  //r                    FIFO_SRC_REG
	{.adr=0x30,    .type=0x03},  //rw                   INT1_CFG
	{.adr=0x31,    .type=0x02},  //r                    INT1_SRC
	{.adr=0x32,    .type=0x03},  //rw                   INT1_TSH_XH
	{.adr=0x33,    .type=0x03},  //rw                   INT1_TSH_XL
	{.adr=0x34,    .type=0x03},  //rw                   INT1_TSH_YH
	{.adr=0x35,    .type=0x03},  //rw                   INT1_TSH_YL
	{.adr=0x36,    .type=0x03},  //rw                   INT1_TSH_ZH
	{.adr=0x37,    .type=0x03},  //rw                   INT1_TSH_ZL
	{.adr=0x38,    .type=0x03},  //rw                   INT1_DURATION
};

Reg_Type_Address IMS330DLC_Reg_Type_Adr [] =
{	//adr          //Type	           //R/W
	{.adr=0x00,    .type=0xFF},	       //-         //FF Reserved    RESERVED
	{.adr=0x01,    .type=0x03},	       //r/w       //0x02 Read      FUNC_CFG_ACCESS
	{.adr=0x02,    .type=0xFF},	       //-         //0x03 Write     RESERVED
	{.adr=0x03,    .type=0xFF},        //-                          RESERVED
	{.adr=0x04,    .type=0x03},        //r/w                        ENSOR_SYNC_TIME_FRAME
	{.adr=0x05,    .type=0x03},        //r/w                        SENSOR_SYNC_RES_RATIO
	{.adr=0x06,    .type=0x03},        //r/w                        FIFO_CTRL1
	{.adr=0x07,    .type=0x03},        //r/w                        FIFO_CTRL2
	{.adr=0x08,    .type=0x03},        //r/w                        FIFO_CTRL3
	{.adr=0x09,    .type=0x03},        //r/w                        FIFO_CTRL4
	{.adr=0x0A,    .type=0x03},        //r/w                        FIFO_CTRL5
	{.adr=0x0B,    .type=0x03},        //r/w                        DRDY_PULSE_CFG
	{.adr=0x0C,    .type=0xFF},        //-                          RESERVED
	{.adr=0x0D,    .type=0x03},        //r/w                        INT1_CTRL
	{.adr=0x0E,    .type=0x03},        //r/w                        INT2_CTRL
	{.adr=0x0F,    .type=0x02},        //r                          WHO_AM_I
	{.adr=0x10,    .type=0x03},        //r/w                        CTRL1_XL
	{.adr=0x11,    .type=0x03},        //r/w                        CTRL2_G
	{.adr=0x12,    .type=0x03},        //r/w                        CTRL3_C
	{.adr=0x13,    .type=0x03},        //r/w                        CTRL4_C
	{.adr=0x14,    .type=0x03},        //r/w                        CTRL5_C
	{.adr=0x15,    .type=0x03},        //r/w                        CTRL6_C
	{.adr=0x16,    .type=0x03},        //r/w                        CTRL7_G
	{.adr=0x17,    .type=0x03},        //r/w                        CTRL8_XL
	{.adr=0x18,    .type=0x03},        //r/w                        CTRL9_XL
	{.adr=0x19,    .type=0x03},        //r/w                        CTRL10_C
	{.adr=0x1A,    .type=0x03},        //r/w                        MASTER_CONFIG
	{.adr=0x1B,    .type=0x02},        //r                          WAKE_UP_SRC
	{.adr=0x1C,    .type=0x02},        //r                          TAP_SRC
	{.adr=0x1D,    .type=0x02},        //r                          D6D_SRC
	{.adr=0x1E,    .type=0x02},        //r                          STATUS_REG/(1)(2)
	{.adr=0x1F,    .type=0xFF},        //-                          RESERVED
	{.adr=0x20,    .type=0x02},        //r                          OUT_TEMP_L
	{.adr=0x21,    .type=0x02},        //r                          OUT_TEMP_H
	{.adr=0x22,    .type=0x02},        //r                          OUTX_L_G
	{.adr=0x23,    .type=0x02},        //r                          OUTX_H_G
	{.adr=0x24,    .type=0x02},        //r                          OUTY_L_G
	{.adr=0x25,    .type=0x02},        //r                          OUTY_H_G
	{.adr=0x26,    .type=0x02},        //r                          OUTZ_L_G
	{.adr=0x27,    .type=0x02},        //r                          OUTZ_H_G
	{.adr=0x28,    .type=0x02},        //r                          OUTX_L_XL
	{.adr=0x29,    .type=0x02},        //r                          OUTX_H_XL
	{.adr=0x2A,    .type=0x02},        //r                          OUTY_L_XL
	{.adr=0x2B,    .type=0x02},        //r                          OUTY_H_XL
	{.adr=0x2C,    .type=0x02},        //r                          OUTZ_L_XL
	{.adr=0x2D,    .type=0x02},        //r                          OUTZ_H_XL
	{.adr=0x2E,    .type=0x02},        //r                          SENSORHUB1_REG
	{.adr=0x2F,    .type=0x02},        //r                          SENSORHUB2_REG
	{.adr=0x30,    .type=0x02},        //r                          SENSORHUB3_REG
	{.adr=0x31,    .type=0x02},        //r                          SENSORHUB4_REG
	{.adr=0x32,    .type=0x02},        //r                          SENSORHUB5_REG
	{.adr=0x33,    .type=0x02},        //r                          SENSORHUB6_REG
	{.adr=0x34,    .type=0x02},        //r                          SENSORHUB7_REG
	{.adr=0x35,    .type=0x02},        //r                          SENSORHUB8_REG
	{.adr=0x36,    .type=0x02},        //r                          SENSORHUB9_REG
	{.adr=0x37,    .type=0x02},        //r                          SENSORHUB10_REG
	{.adr=0x38,    .type=0x02},        //r                          SENSORHUB11_REG
	{.adr=0x39,    .type=0x02},        //r                          SENSORHUB12_REG
	{.adr=0x3A,    .type=0x02},        //r                          FIFO_STATUS1
	{.adr=0x3B,    .type=0x02},        //r                          FIFO_STATUS2
	{.adr=0x3C,    .type=0x02},        //r                          FIFO_STATUS3
	{.adr=0x3D,    .type=0x02},        //r                          FIFO_STATUS4
	{.adr=0x3E,    .type=0x02},        //r                          FIFO_DATA_OUT_L
	{.adr=0x3F,    .type=0x02},        //r                          FIFO_DATA_OUT_H
	{.adr=0x40,    .type=0x02},        //r                          TIMESTAMP0_REG
	{.adr=0x41,    .type=0x02},        //r                          TIMESTAMP1_REG
	{.adr=0x42,    .type=0x03},        //r/w                        TIMESTAMP2_REG
	{.adr=0x43,    .type=0xFF},        //-                          RESERVED
	{.adr=0x4D,    .type=0x02},        //r                          SENSORHUB13_RE
	{.adr=0x4E,    .type=0x02},        //r                          SENSORHUB14_RE
	{.adr=0x4F,    .type=0x02},        //r                          SENSORHUB15_RE
	{.adr=0x50,    .type=0x02},        //r                          SENSORHUB16_RE
	{.adr=0x51,    .type=0x02},        //r                          SENSORHUB17_RE
	{.adr=0x52,    .type=0x02},        //r                          SENSORHUB18_RE
	{.adr=0x53,    .type=0x02},        //r                          FUNC_SRC1
	{.adr=0x54,    .type=0x02},        //r                          FUNC_SRC2
	{.adr=0x57,    .type=0xFF},        //-                          RESERVED
	{.adr=0x58,    .type=0x03},        //r/w                        TAP_CFG
	{.adr=0x59,    .type=0x03},        //r/w                        TAP_THS_6D
	{.adr=0x5A,    .type=0x03},        //r/w                        INT_DUR2
	{.adr=0x5B,    .type=0x03},        //r/w                        WAKE_UP_THS
	{.adr=0x5C,    .type=0x03},        //r/w                        WAKE_UP_DUR
	{.adr=0x5D,    .type=0x03},        //r/w                        FREE_FALL
	{.adr=0x5E,    .type=0x03},        //r/w                        MD1_CFG
	{.adr=0x5F,    .type=0x03},        //r/w                        MD2_CFG
	{.adr=0x60,    .type=0x03},        //r/w                        MASTER_CMD_COD
	{.adr=0x61,    .type=0x03},        //r/w                        SENS_SYNC_SPI_ERROR_CODE
	{.adr=0x65,    .type=0xFF},        //-                          RESERVED
	{.adr=0x66,    .type=0x02},        //r                          OUT_MAG_RAW_X_L
	{.adr=0x67,    .type=0x02},        //r                          OUT_MAG_RAW_X_H
	{.adr=0x68,    .type=0x02},        //r                          OUT_MAG_RAW_Y_L
	{.adr=0x69,    .type=0x02},        //r                          OUT_MAG_RAW_Y_H
	{.adr=0x6A,    .type=0x02},        //r                          OUT_MAG_RAW_Z_L
	{.adr=0x6B,    .type=0x02},        //r                          OUT_MAG_RAW_Z_H
	{.adr=0x6E,    .type=0xFF},        //-                          RESERVED
	{.adr=0x6F,    .type=0x03},        //r/w                        INT_OIS
	{.adr=0x70,    .type=0x03},        //r/w                        CTRL1_OIS
	{.adr=0x71,    .type=0x03},        //r/w                        CTRL2_OIS
	{.adr=0x72,    .type=0x03},        //r/w                        CTRL3_OIS
	{.adr=0x73,    .type=0x03},        //r/w                        X_OFS_USR
	{.adr=0x74,    .type=0x03},        //r/w                        Y_OFS_USR
	{.adr=0x75,    .type=0x03},        //r/w                        Z_OFS_USR
	{.adr=0x7F,    .type=0xFF},        //-                          RESERVED
};

void BSP_GYRO_10ms_Task(void)
{

	//read write gyro
	//BSP_TL3GD20_ReadWrite(&GyroscopeDriver,1);

	//Read write the external 3D acc 3D Gyro
	BSP_ISM330DLC_ReadWrite_Reg(&GyroAccDriver,1);

}

void BSP_GYRO_Init_task(void)
{
	//get the plug-in function from the driver layer
	Gyroscope = &L3gd20DrvDefinition;

	GyroscopeDriver.TL3GD20_Reg_Struct.Who_Am_I = I_AM_L3GD20;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG1.All = L3GD20_OUTPUT_DATARATE_1|L3GD20_BANDWIDTH_4|L3GD20_MODE_ACTIVE|L3GD20_AXES_ENABLE;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG2.All = L3GD20_HPM_NORMAL_MODE_RES|L3GD20_HPFCF_0;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG3.All = 0;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG4.All = L3GD20_BLE_LSB|L3GD20_FULLSCALE_2000|L3GD20_BlockDataUpdate_Continous;
	GyroscopeDriver.TL3GD20_Reg_Struct.CTRL_REG5.All = L3GD20_HIGHPASSFILTER_DISABLE;

	//Read the internal 3D gyro
	BSP_TL3GD20_ReadWrite(&GyroscopeDriver,1);



}
/**
 * @brief  Set High Pass Filter Modality
 * @param  FilterStruct: contains the configuration setting for the L3GD20.
 * @retval None
 */
void BSP_TL3GD20_ReadWrite(TL3GD20_iDriver_Description *map, uint8_t Num)
{
	uint8_t index = 0u;

	if(Gyroscope->ReadID() == map->TL3GD20_Reg_Struct.Who_Am_I)
	{
		for (index = 3; index < (sizeof(map->TL3GD20_Reg_Tab)/sizeof(map->TL3GD20_Reg_Tab[0])); index++)
		{
			if (0xFF != TL3GD20_Reg_Type_Adr[index].adr)
			{
				//check if any bit has been changed
				if (u8_TL3GD20_Reg[index] != map->TL3GD20_Reg_Tab[index])
				{
					//bit has been changed in the driver register updated it
					TL3GD20_Update_Regiter_Flag = 1;
				}
				//loop through the buffer and read write if necessary
				if (0x02 == TL3GD20_Reg_Type_Adr[index].type)
				{
					//read
					Gyro_l3gd20_Read(&map->TL3GD20_Reg_Tab[index], TL3GD20_Reg_Type_Adr[index].adr, Num);

					// Read data to a local buffer
					u8_TL3GD20_Reg[index] = map->TL3GD20_Reg_Tab[index];
				}
				else if (0X03 == TL3GD20_Reg_Type_Adr[index].type)
				{
					if ((1u == TL3GD20_Update_Regiter_Flag))
					{
						//write
						Gyro_l3gd20_Write(&map->TL3GD20_Reg_Tab[index], TL3GD20_Reg_Type_Adr[index].adr, Num);
						//reset the flag when write is done!
						TL3GD20_Update_Regiter_Flag = 0;
					}
					//read
					Gyro_l3gd20_Read(&map->TL3GD20_Reg_Tab[index], TL3GD20_Reg_Type_Adr[index].adr, Num);
					//if write read is not good issue a problem.

					// Read data to a local buffer
					u8_TL3GD20_Reg[index] = map->TL3GD20_Reg_Tab[index];
				}
				else
				{

				}
			}
			else
			{

			}
		}
	}
	else
	{
		//increase id failure

	}
}
uint8_t BSP_TL3GD20_ReadId(void)
{
	uint8_t id = 0;

	if(0!=Gyroscope->ReadID())
	{
		id = Gyroscope->ReadID();
	}

	return id;
}

void BSP_TL3GD20_Reset(void)
{
	if(NULL!=Gyroscope->Reset)
	{
		Gyroscope->Reset();
	}
}

void BSP_TL3GD20_EnableIt(uint8_t value)
{
	if(NULL!=Gyroscope->EnableIT) // important to distinguish between Gyroscope->EnableIT  and Gyroscope->EnableIT()
	{
		Gyroscope->EnableIT(value);
	}
}

void BSP_TL3GD20_DisableIt(uint8_t value)
{
	if(NULL!=Gyroscope->DisableIT)
	{
		Gyroscope->DisableIT(value);
	}
}

void BSP_TL3GD20_GetGyroValues(float * values)
{
	if(NULL != Gyroscope->GetXYZ)
	{
		Gyroscope->GetXYZ(values);
	}
}



void BSP_ISM330_GYRO_ACC_Init_task(void)
{
	//uint8_t lReadValue;
	Tun_Ctrl3_C lReadValue;
	Tun_Status_Reg lStatus;
	uint16_t size1;
	uint16_t size2;

	GyroAcc = &Ism330dlcDrvDefinition;

	//reset buffers
	memset(GyroAccDriver.ISM330DLC_Reg_Tab, 0x00, sizeof(GyroAccDriver.ISM330DLC_Reg_Tab));
	memset(u8_ISM330DLC_Reg, 0x00, sizeof(u8_ISM330DLC_Reg));

	size1 = sizeof(GyroAccDriver.ISM330DLC_Reg_Struct);
	size2 = sizeof(IMS330DLC_Reg_Type_Adr);
	Debugvariable.id_failure = 0;
	Debugvariable.interrupt_counter = 0;
	Debugvariable.loop_counter = 0;


	GyroAccDriver.ISM330DLC_Reg_Struct.WHO_AM_I  = 0x6A;
	//read Id
	if(GyroAcc->ReadID() == GyroAccDriver.ISM330DLC_Reg_Struct.WHO_AM_I)
	{
		//reset device
		GyroAccDriver.ISM330DLC_Reg_Struct.CTRL3_C.ISM330DLC_RegBits.SW_RESET = 1U;
		Acc_Gyro_Imc330dlc_Write((uint8_t*)&GyroAccDriver.ISM330DLC_Reg_Struct.CTRL3_C.All, ISM330DLC_CTRL3_C, 1);

		//check if reset is well done
		do
		{
			Acc_Gyro_Ism330dlc_Read((uint8_t*)&lReadValue.All, ISM330DLC_CTRL3_C, 1);
			//TODO add timeout if possible
		}while(lReadValue.ISM330DLC_RegBits.SW_RESET == 1);

		//load default values from device
		BSP_ISM330DLC_ReadWrite_Reg(&GyroAccDriver,1);

		//output data updated when lsb and msb are read using CTRL3_C.
		GyroAccDriver.ISM330DLC_Reg_Struct.CTRL1_XL.All = CTRL_XL_12_5_Hz|CTRL_XL_ACCE_FULL_SACAL_1;

		//output data rate set using CTRL1_XL
		GyroAccDriver.ISM330DLC_Reg_Struct.CTRL2_G.All = ISM330DL_XL_ODR_12Hz5|GYROSCOPE_FULL_SCALE_3;

	}

	//
	BSP_ISM330DLC_ReadWrite_Reg(&GyroAccDriver,1);
}

/**
 * @brief  This function is used to read the 3D Accelerometer and 3D geyrometer
 * @param  FilterStruct: contains the configuration setting for the L3GD20.
 * @retval None
 */

uint16_t BSP_ISM330DLC_ReadWrite_Reg(ISM330DLC_iDriver_Description *GyroAccDriverReg, uint16_t Num)
{
	uint16_t index;
	uint16_t sizeofBuffer;
	sizeofBuffer = 43;// sizeof(GyroAccDriverReg->ISM330DLC_Reg_Tab)/sizeof(GyroAccDriverReg->ISM330DLC_Reg_Tab[0]);

	if(GyroAcc->ReadID() == GyroAccDriverReg->ISM330DLC_Reg_Struct.WHO_AM_I)
	{
		for (index = 0; index < sizeofBuffer ; index++)
		{
			Debugvariable.loop_counter++;
			if (0xFF != IMS330DLC_Reg_Type_Adr[index].type)
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
					Acc_Gyro_Ism330dlc_Read(&GyroAccDriverReg->ISM330DLC_Reg_Tab[index], IMS330DLC_Reg_Type_Adr[index].adr, Num);

					// Read data to a local buffer
					u8_TL3GD20_Reg[index] = GyroAccDriverReg->ISM330DLC_Reg_Tab[index];

					//reset the flag when write is done!
					ISM330DLC_Update_Regiter_Flag = 0;
				}
				else if (0x03 == IMS330DLC_Reg_Type_Adr[index].type)
				{
					if ((1u == ISM330DLC_Update_Regiter_Flag))
					{
						//write
						Acc_Gyro_Imc330dlc_Write(&GyroAccDriverReg->ISM330DLC_Reg_Tab[index], IMS330DLC_Reg_Type_Adr[index].adr, Num);
						//reset the flag when write is done!
						ISM330DLC_Update_Regiter_Flag = 0;
					}
					//read
					Acc_Gyro_Ism330dlc_Read(&GyroAccDriverReg->ISM330DLC_Reg_Tab[index], IMS330DLC_Reg_Type_Adr[index].adr, Num);
					//if write read is not good issue a problem.

					// Read data to a local buffer
					u8_ISM330DLC_Reg[index] = GyroAccDriverReg->ISM330DLC_Reg_Tab[index];
				}
				else
				{
					//reset the flag when write is done!
					ISM330DLC_Update_Regiter_Flag = 0;
				}


			}
		}
		return 1;
	}
	else
	{
		Debugvariable.id_failure++;
		return -1;
	}
}


