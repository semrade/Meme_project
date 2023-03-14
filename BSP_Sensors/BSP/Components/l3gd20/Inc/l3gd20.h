/*
 * l3gd20.h
 *
 *  Created on: Mar 13, 2023
 *      Author: Tarik
 */

#ifndef L3GD20_H_
#define L3GD20_H_


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
typedef void (TfL3gd20_GetXYZ)(float *);


/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
#define L3GD20_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           0x2D  /* Output Register Z */
#define L3GD20_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define L3GD20_INT1_CFG_ADDR          0x30  /* Interrupt 1 configuration Register */
#define L3GD20_INT1_SRC_ADDR          0x31  /* Interrupt 1 source Register */
#define L3GD20_INT1_TSH_XH_ADDR       0x32  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_XL_ADDR       0x33  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_YH_ADDR       0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_YL_ADDR       0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_ZH_ADDR       0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_TSH_ZL_ADDR       0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_DURATION_ADDR     0x38  /* Interrupt 1 DURATION register */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/

#define I_AM_L3GD20                 ((uint8_t)0xD4)
#define I_AM_L3GD20_TR              ((uint8_t)0xD5)

/** @defgroup Power_Mode_selection
  * @{
  */
#define L3GD20_MODE_POWERDOWN       ((uint8_t)0x00)
#define L3GD20_MODE_ACTIVE          ((uint8_t)0x08)
/**
  * @}
  */

/** @defgroup OutPut_DataRate_Selection
  * @{
  */
#define L3GD20_OUTPUT_DATARATE_1    ((uint8_t)0x00)
#define L3GD20_OUTPUT_DATARATE_2    ((uint8_t)0x40)
#define L3GD20_OUTPUT_DATARATE_3    ((uint8_t)0x80)
#define L3GD20_OUTPUT_DATARATE_4    ((uint8_t)0xC0)
/**
  * @}
  */

/** @defgroup Axes_Selection
  * @{
  */
#define L3GD20_X_ENABLE            ((uint8_t)0x02)
#define L3GD20_Y_ENABLE            ((uint8_t)0x01)
#define L3GD20_Z_ENABLE            ((uint8_t)0x04)
#define L3GD20_AXES_ENABLE         ((uint8_t)0x07)
#define L3GD20_AXES_DISABLE        ((uint8_t)0x00)
/**
  * @}
  */

/** @defgroup Bandwidth_Selection
  * @{
  */
#define L3GD20_BANDWIDTH_1         ((uint8_t)0x00)
#define L3GD20_BANDWIDTH_2         ((uint8_t)0x10)
#define L3GD20_BANDWIDTH_3         ((uint8_t)0x20)
#define L3GD20_BANDWIDTH_4         ((uint8_t)0x30)
/**
  * @}
  */

/** @defgroup Full_Scale_Selection
  * @{
  */
#define L3GD20_FULLSCALE_250       ((uint8_t)0x00)
#define L3GD20_FULLSCALE_500       ((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000      ((uint8_t)0x20)
#define L3GD20_FULLSCALE_SELECTION ((uint8_t)0x30)
/**
  * @}
  */

/** @defgroup Full_Scale_Sensitivity
  * @{
  */
#define L3GD20_SENSITIVITY_250DPS  ((float)8.75f)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_500DPS  ((float)17.50f)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_2000DPS ((float)70.00f)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */
/**
  * @}
  */


/** @defgroup Block_Data_Update
  * @{
  */
#define L3GD20_BlockDataUpdate_Continous   ((uint8_t)0x00)
#define L3GD20_BlockDataUpdate_Single      ((uint8_t)0x80)
/**
  * @}
  */

/** @defgroup Endian_Data_selection
  * @{
  */
#define L3GD20_BLE_LSB                     ((uint8_t)0x00)
#define L3GD20_BLE_MSB	                   ((uint8_t)0x40)
/**
  * @}
  */

/** @defgroup High_Pass_Filter_status
  * @{
  */
#define L3GD20_HIGHPASSFILTER_DISABLE      ((uint8_t)0x00)
#define L3GD20_HIGHPASSFILTER_ENABLE	     ((uint8_t)0x10)
/**
  * @}
  */

/** @defgroup INT1_INT2_selection
  * @{
  */
#define L3GD20_INT1                        ((uint8_t)0x00)
#define L3GD20_INT2                        ((uint8_t)0x01)
/**
  * @}
  */

/** @defgroup INT1_Interrupt_status
  * @{
  */
#define L3GD20_INT1INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT1INTERRUPT_ENABLE        ((uint8_t)0x80)
/**
  * @}
  */

/** @defgroup INT2_Interrupt_status
  * @{
  */
#define L3GD20_INT2INTERRUPT_DISABLE       ((uint8_t)0x00)
#define L3GD20_INT2INTERRUPT_ENABLE        ((uint8_t)0x08)
/**
  * @}
  */

/** @defgroup INT1_Interrupt_ActiveEdge
  * @{
  */
#define L3GD20_INT1INTERRUPT_LOW_EDGE      ((uint8_t)0x20)
#define L3GD20_INT1INTERRUPT_HIGH_EDGE     ((uint8_t)0x00)
/**
  * @}
  */

/** @defgroup Boot_Mode_selection
  * @{
  */
#define L3GD20_BOOT_NORMALMODE             ((uint8_t)0x00)
#define L3GD20_BOOT_REBOOTMEMORY           ((uint8_t)0x80)
/**
  * @}
  */

/** @defgroup High_Pass_Filter_Mode
  * @{
  */
#define L3GD20_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define L3GD20_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define L3GD20_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define L3GD20_HPM_AUTORESET_INT           ((uint8_t)0x30)
/**
  * @}
  */

/** @defgroup High_Pass_CUT OFF_Frequency
  * @{
  */
#define L3GD20_HPFCF_0              0x00
#define L3GD20_HPFCF_1              0x01
#define L3GD20_HPFCF_2              0x02
#define L3GD20_HPFCF_3              0x03
#define L3GD20_HPFCF_4              0x04
#define L3GD20_HPFCF_5              0x05
#define L3GD20_HPFCF_6              0x06
#define L3GD20_HPFCF_7              0x07
#define L3GD20_HPFCF_8              0x08
#define L3GD20_HPFCF_9              0x09
/**
  * @}
  */

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Yen:1; /*X axis enable*/ //lsb
		uint8_t Xen:1; /*Y axis enable*/
		uint8_t Zen:1; /*Z axis enable*/
		uint8_t PD:1;  /*Power-down mode enable*/
		uint8_t BW0:1;
		uint8_t BW1:1; /*Bandwidth selection*/
		uint8_t DR0:1;
		uint8_t DR1:1; /*Output data rate selection*/ //msb
	} RegBits;
} Tst_CTRL_REG1;


typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t HPCF0:1;
		uint8_t HPCF1:1;
		uint8_t HPCF2:1;
		uint8_t HPCF3:1; /*High-pass filter cutoff frequency selection*/
		uint8_t HPM0:1;
		uint8_t HPM1:1;  /*High-pass filter mode selection*/
		uint8_t Reserved:2;
	} RegBits;
} Tst_CTRL_REG2;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t I2_Empty:1;   /*FIFO empty interrupt on DRDY/INT2. Default value: 0. (0: disable; 1: enable)  */
		uint8_t I2_ORun:1;    /*FIFO overrun interrupt on DRDY/INT2 Default value: 0. (0: disable; 1: enable) */
		uint8_t I2_WTM:1;     /*FIFO watermark interrupt on DRDY/INT2. Default value: 0. (0: disable; 1: enab */
		uint8_t I2_DRDY:1;    /*Date-ready on DRDY/INT2. Default value 0. (0: disable; 1: enable)             */
		uint8_t PP_OD:1;      /*Push-pull / Open drain. Default value: 0. (0: push- pull; 1: open drain)      */
		uint8_t H_Lactive:1;  /*Interrupt active configuration on INT1. Default value 0. (0: high; 1:low)     */
		uint8_t I1_Boot:1;    /*Boot status available on INT1. Default value 0. (0: disable; 1: enable)       */
		uint8_t I1_Int1:1;    /*Interrupt enable on INT1 pin. Default value 0. (0: disable; 1: enable)        */
	} RegBits;
} Tst_CTRL_REG3;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t SIM:1;             /*SPI serial interface mode selection*/
		uint8_t Reserved:3;
		uint8_t FS0:1;
		uint8_t FS1:1;             /*Full scale selection*/
		uint8_t BLE:1;             /*Big/little endian data selection*/
		uint8_t BDU:1;             /*Block data update.*/
	} RegBits;
} Tst_CTRL_REG4;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Out_Sel0:1;
		uint8_t Out_Sel1:1;       /*Out selection configuration*/
		uint8_t INT1_Sel0:1;
		uint8_t INT1_Sel1:1;      /*NT1 selection configuration*/
		uint8_t HPen:1;           /*High-pass filter enable*/
		uint8_t Reserved:1;
		uint8_t FIFO_EN:1;        /*FIFO enable*/
		uint8_t BOOT:1;           /*Reboot memory content*/
	} RegBits;
} Tst_CTRL_REG5;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Ref0:1; /*Reference value for interrupt generation*/
		uint8_t Ref1:1;
		uint8_t Ref2:1;
		uint8_t Ref3:1;
		uint8_t Ref4:1;
		uint8_t Ref5:1;
		uint8_t Ref6:1;
		uint8_t Ref7:1;
	} RegBits;
} Tst_Ref;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Temp0:1; /*Temperature data*/
		uint8_t Temp1:1;
		uint8_t Temp2:1;
		uint8_t Temp3:1;
		uint8_t Temp4:1;
		uint8_t Temp5:1;
		uint8_t Temp6:1;
		uint8_t Temp7:1;
	} RegBits;
} Tst_Temp;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t XDA:1; 			/*X axis new data available*/
		uint8_t YDA:1; 			/*Y axis new data available*/
		uint8_t ZDA:1; 			/*Z axis new data available*/
		uint8_t ZYXDA:1;		/* X, Y, Z -axis new data available*/
		uint8_t XOR:1; 			/*X axis data overrun*/
		uint8_t YOR:1; 			/*Y axis data overrun*/
		uint8_t ZOR:1; 			/*Z axis data overrun*/
		uint8_t ZYXOR:1;		/*X, Y, Z -axis data overrun*/
	} RegBits;
} Tst_Statu_Reg;


typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t WTM0:1;
		uint8_t WTM1:1;
		uint8_t WTM2:1;
		uint8_t WTM3:1;			/*FIFO threshold*/
		uint8_t WTM4:1;
		uint8_t FM0:1;
		uint8_t FM1:1;
		uint8_t FM2:1; 			/*FIFO mode selection*/
	} RegBits;
} Tst_Fifo_CTRL_Reg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FSS0:1;
		uint8_t FSS1:1;
		uint8_t FSS2:1;
		uint8_t FSS3:1;
		uint8_t FSS4:1;			/*FIFO stored data level*/
		uint8_t EMPTY:1;		/*FIFO empty bit.*/
		uint8_t OVRN:1;			/*Overrun bit status*/
		uint8_t WTM:1;			/*Watermark status*/
	}RegBits;
} Tst_Fifo_Src_Reg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t XLIE:1;			/*Enable interrupt generation on X low event*/
		uint8_t XHIE:1;			/*Enable interrupt generation on X high event*/
		uint8_t YLIE:1;			/*Enable interrupt generation on Y low event*/
		uint8_t YHIE:1;			/*Enable interrupt generation on Y high event*/
		uint8_t ZLIE:1;			/*Enable interrupt generation on Z low event*/
		uint8_t ZHIE:1;			/*Enable interrupt generation on Z high event*/
		uint8_t LIR:1;			/*Latch interrupt request*/
		uint8_t AND_OR:1; 		/*AND/OR combination of interrupt events*/
	}RegBits;
} Tst_Int1_Cfg_Reg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t XL:1;            /*X low*/
		uint8_t XH:1;            /*X high*/
		uint8_t YL:1;            /*Y low*/
		uint8_t YH:1;            /*Y high*/
		uint8_t ZL:1;            /*Z low*/
		uint8_t ZH:1;            /*Z high*/
		uint8_t IA:1;            /*Interrupt active*/
		uint8_t Reserved:1;
	}RegBits;
} Tst_Int1_Src_Reg;

typedef union
{
	struct
	{
		uint8_t 		  reserved;
		uint8_t 		  Who_Am_I;		//r
		uint8_t			  reserved1;
		Tst_CTRL_REG1     CTRL_REG1;	//rw
		Tst_CTRL_REG2     CTRL_REG2;	//rw
		Tst_CTRL_REG3     CTRL_REG3;	//rw
		Tst_CTRL_REG4     CTRL_REG4;	//rw
		Tst_CTRL_REG5     CTRL_REG5;	//rw
		Tst_Ref           REFERNCE;		//rw
		Tst_Temp	      OUT_TEMP;		//r
		Tst_Statu_Reg     STATUS_REG;	//r
		uint8_t 	      OUT_X_L;	    //r
		uint8_t 	      OUT_X_H;      //r
		uint8_t 	      OUT_Y_L;      //r
		uint8_t 	      OUT_Y_H;      //r
		uint8_t 	      OUT_Z_L;      //r
		uint8_t 	      OUT_Z_H;      //r
		Tst_Fifo_CTRL_Reg FIFO_CTRL_REG;//rw
		Tst_Fifo_Src_Reg  FIFO_SRC_REG; //r
		Tst_Int1_Cfg_Reg  INT1_CFG;		//rw
		Tst_Int1_Src_Reg  INT1_SRC;		//r
		uint8_t           INT1_TSH_XH;  //rw
		uint8_t           INT1_TSH_XL;  //rw
		uint8_t           INT1_TSH_YH;  //rw
		uint8_t           INT1_TSH_YL;  //rw
		uint8_t           INT1_TSH_ZH;  //rw
		uint8_t           INT1_TSH_ZL;  //rw
		uint8_t           INT1_DURATION;//rw
	} TL3GD20_Reg_Struct;
	uint8_t TL3GD20_Reg_Tab[28];
} TL3GD20_iDriver_Description;

typedef struct
{
	TfL3gd20_InitFunciton	*Init(uint16_t);
	TfL3gd20_DeInitFunction *DeInit(void);
	TfL3gd20_ReadId			*ReadID(void);
	TfL3gd20_Reset 			*Reset(void);
	TfL3gd20_LowPower       *LowPower(uint16_t);
	TfL3gd20_ConfigureInt   *ConfigIT(uint16_t);
	TfL3gd20_EnableInt      *EnableIT(uint8_t);
	TfL3gd20_DisableInt     *DisableIT(uint8_t);
	TfL3gd20_IntStatus      *ITStatus(uint16_t, uint16_t);
	TfL3gd20_ClearInt       *ClearIT(uint16_t, uint16_t);
	TfL3gd20_FilterConfig   *FilterConfig(uint8_t);
	TfL3gd20_FilterCmd      *FilterCmd(uint8_t);
	TfL3gd20_GetXYZ         *GetXYZ(float *);
} TstL3gd20_DrvTypeDef;

#endif /* L3GD20_H_ */
