/*
 * imc330dlc.h
 *
 *  Created on: Mar 13, 2023
 *      Author: Tarik
 */

#ifndef IMC330DLC_H_
#define IMC330DLC_H_

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved:7;
		uint8_t FUNC_CFG_EN:1; /*Enable access to the embedded functions configuration registers*/
	} ISM330DLC_RegBits;
} Tun_Runc_Cfg_Access;

typedef volatile union
{
	uint8_t All;
	struct
	{

		uint8_t TPH_3 :1; /*Sensor synchronization time frame with the step of 500 ms and full range of 5 s*/
		uint8_t TPH_2 :1;
		uint8_t TPH_1 :1;
		uint8_t TPH_0 :1;
		uint8_t Reserved:4;
	} ISM330DLC_RegBits;
} Tun_Sen_Tim_Fram;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t RR_0 :1; /*Resolution ratio of error code for sensor synchronization*/
		uint8_t RR_1 :1;
		uint8_t Reserved:6;
	} ISM330DLC_RegBits;
} Tun_Sen_Sync_Res_Ratio;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FTH_0 :1; /*FIFO threshold level setting.*/
		uint8_t FTH_1 :1;
		uint8_t FTH_2 :1;
		uint8_t FTH_3 :1;
		uint8_t FTH_4 :1;
		uint8_t FTH_5 :1;
		uint8_t FTH_6 :1;
		uint8_t FTH_7 :1;
	}ISM330DLC_RegBits;
} Tun_Fifo_Ctrl1;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FTH_8:1;
		uint8_t FTH_9:1;
		uint8_t FTH_10:1;
		uint8_t FIFO_TEMP_EN;  /*Enables the temperature data storage in FIFO*/
		uint8_t Reserved :3;
		uint8_t FIFO_TIMER_EN:1; /*Enables timestamp data to be stored as the 4th FIFO data se*/
	}ISM330DLC_RegBits;
} Tun_Fifo_Ctrl2;


typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DEC_FIFO_XL0:1; /*Accelerometer FIFO (second data set) decimation setting*/
		uint8_t DEC_FIFO_XL1:1;
		uint8_t DEC_FIFO_XL2:1;
		uint8_t DEC_FIFO_GYRO0:1; /*Gyro FIFO (first data set) decimation setting*/
		uint8_t DEC_FIFO_GYRO1:1;
		uint8_t DEC_FIFO_GYRO2:1;
		uint8_t Reserved:2;
	} ISM330DLC_RegBits;
} Tun_Fifo_Ctrl3;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DEC_DS3_FIFO0:1; /*Third FIFO data set decimation setting*/
		uint8_t DEC_DS3_FIFO1:1;
		uint8_t DEC_DS3_FIFO2:1;
		uint8_t DEC_DS4_FIFO0:1; /*Fourth FIFO data set decimation setting*/
		uint8_t DEC_DS4_FIFO1:1;
		uint8_t DEC_DS4_FIFO2:1;
		uint8_t ONLY_HIGH_DATA:1; /*8-bit data storage in FIFO*/
		uint8_t STOP_ON_FTH:1;	  /*Enable FIFO threshold level use*/
	} ISM330DLC_RegBits;
} Tun_Fifo_Ctrl4;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FIFO_MODE_0:1; /*FIFO mode selection bits, setting ODR_FIFO also*/
		uint8_t FIFO_MODE_1:1;
		uint8_t FIFO_MODE_2:1;
		uint8_t ODR_FIFO_0:1; /*FIFO ODR selection, setting FIFO_MODE also*/
		uint8_t ODR_FIFO_1:1;
		uint8_t ODR_FIFO_2:1;
		uint8_t ODR_FIFO_3:1;
		uint8_t Reserved:1;
	} ISM330DLC_RegBits;
} Tun_Fifo_Ctrl5;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reseved:7;
		uint8_t DRDY_PULSED:1; /*Enable pulsed data-ready mode*/
	} ISM330DLC_RegBits;
} Tun_Drdy_Pulse_Cfg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t NT1_FULL_FLAG:1;/*FIFO full flag interrupt enable on INT1 pad*/
		uint8_t INT1_FIFO_OVR:1;/*FIFO overrun interrupt on INT1 pad*/
		uint8_t INT1_FTH:1;		/*FIFO threshold interrupt on INT1 pad*/
		uint8_t INT1_BOOT:1;	/*Boot status available on INT1 pad*/
		uint8_t INT1_DRDY_G:1;	/*Gyroscope data-ready on INT1 pad*/
		uint8_t INT1_DRDY_XL:1;	/*Accelerometer data-ready on INT1 pad*/
		uint8_t Reversed:2;
	} ISM330DLC_RegBits;
} Tun_Int1_Ctrl;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t INT2_FULL_FLAG:1;		//FIFO full flag interrupt enable on INT2 pad
		uint8_t INT2_FIFO_OVR:1;		//FIFO overrun interrupt on INT2 pad
		uint8_t INT2_FTH:1;			//FIFO threshold interrupt on INT2 pad
		uint8_t INT2_DRDY_TEMP:1;		//Temperature data-ready on INT2 pad
		uint8_t INT2_DRDY_G:1;			//Gyroscope data-ready on INT2 pad
		uint8_t INT2_DRDY_XL:1;		//Accelerometer data-ready on INT2 pad
		uint8_t Reserved:2;
	}ISM330DLC_RegBits;
} Tun_Int2_Ctrl;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t BW0_XL:1; 	//Accelerometer analog chain bandwidth selection (only for accelerometer ODR ≥ 1.67 kHz)
		uint8_t LPF1_BW_SEL:1; //Accelerometer digital LPF (LPF1) bandwidth selection
		uint8_t FS_XL1:1; 	//Accelerometer full-scale selection
		uint8_t FS_XL2:1;
		uint8_t ODR_XL0:1;	//Output data rate and power mode selection
		uint8_t ODR_XL1:1;
		uint8_t ODR_XL2:1;
		uint8_t ODR_XL3:1;
	} ISM330DLC_RegBits;
} Tun_Ctrl1_Xl;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reversed:1;
		uint8_t FS_125:1; //Gyroscope full-scale at ±125 dps
		uint8_t FS_G0:1;  //Gyroscope full-scale selection
		uint8_t FS_G1:1;
		uint8_t ODR_G0:1; //Gyroscope output data rate selection
		uint8_t ODR_G1:1;
		uint8_t ODR_G2:1;
		uint8_t ODR_G3:1;
	} ISM330DLC_RegBits;
} Tun_Ctrl2_G;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t SW_RESET:1; //Software reset
		uint8_t BLE:1; 		//Big/Little Endian Data selection
		uint8_t IF_INC:1;	//Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI)
		uint8_t SIM:1;		//SPI Serial Interface Mode selection.
		uint8_t PP_OD:1;	//Push-pull/open-drain selection on INT1 and INT2 pads
		uint8_t H_LACTIVE:1;//Interrupt activation level.
		uint8_t BDU:1;      //Block Data Update.
		uint8_t BOOT:1; 	//Reboots memory content
	} ISM330DLC_RegBits;
} Tun_Ctrl3_C;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved:1;
		uint8_t LPF1_SEL_G:1;	//Enable gyroscope digital LPF1 if auxiliary SPI is disabled
		uint8_t I2C_DISABLE:1;	//Disable I²C interface
		uint8_t DRDY_MASK:1;	//Configuration 1 data available enable bit
		uint8_t DEN_DRDY_INT1:1;//DEN DRDY signal on INT1 pad
		uint8_t INT2_ON_INT1:1;	//All interrupt signals available on INT1 pad enable
		uint8_t SLEEP:1;	 	//Gyroscope sleep mode enable
		uint8_t DEN_XL_EN:1; 	//Extend DEN functionality to accelerometer sensor
	} ISM330DLC_RegBits;
} Tun_Ctrl4_C;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t ST0_XL:1;	 //Linear acceleration sensor self-test enable
		uint8_t ST1_XL:1;
		uint8_t ST0_G:1;	 //Angular rate sensor self-test enable
		uint8_t ST1_G:1;
		uint8_t DEN_LH:1;	 //DEN active level configuration
		uint8_t ROUNDING0:1; //Circular burst-mode (rounding) read from output registers through the primary interface
		uint8_t ROUNDING1:1;
		uint8_t ROUNDING2:1;
	} ISM330DLC_RegBits;
} Tun_Ctrl5_C;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FTYPE_0:1; 		//Gyroscope low-pass filter (LPF1) bandwidth selection
		uint8_t FTYPE_1:1;
		uint8_t Reversed:1;
		uint8_t USR_OFF_W:1; 	//Weight of XL user offset bits of registers
		uint8_t XL_HM_MDOE:1; 	//High-performance operating mode disable for accelerometer
		uint8_t LVL2_EN:1; 		//DEN level-sensitive latched enable
		uint8_t RTIG_EN:1; 		//DEN data edge-sensitive trigger enable
	} ISM330DLC_RegBits;
}Tun_Ctrl6_C;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved:2;
		uint8_t ROUNDING_STATUS:1; 	//Source register rounding function on WAKE_UP_SRC
		uint8_t Reserved1:1;
		uint8_t HPM0_G:1;			//Gyroscope digital HP filter cutoff selection.
		uint8_t HPM1_G:1;
		uint8_t HP_EN_G:1;			//Gyroscope digital high-pass filter enable
		uint8_t G_HM_MODE:1;		//High-performance operating mode disable for gyroscope
	} ISM330DLC_RegBits;
} Tun_Ctrl7_G;

typedef volatile union
{
	struct
	{
		uint8_t LOW_PASS_ON_6D:1;	//LPF2 on 6D function selection
		uint8_t Reserved:1;
		uint8_t HP_SLOPE_XL_EN:1;	//Accelerometer slope filter / high-pass filter selection
		uint8_t INPUT_COMPOSITE:1;	//Composite filter input selection
		uint8_t HP_REF_MODE:1;		//Enable HP filter reference mode
		uint8_t HPCF_XL0:1;			//Accelerometer LPF2 and high-pass filter configuration and cutoff setting
		uint8_t HPCF_XL1:1;
		uint8_t LPF2_XL_EN:1;		//Accelerometer low-pass filter LPF2 selection
	} ISM330DLC_RegBits;
} Tun_Ctrl8_Xl;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved:2;
		uint8_t SOFT_EN:1;	//Enable soft-iron correction algorithm for magnetometer
		uint8_t Reserved1:1;
		uint8_t DEN_XL_G:1;	//DEN stamping sensor selection.
		uint8_t DEN_Z:1;	//DEN value stored in LSB of Z-axis
		uint8_t DEN_Y:1;	//DEN value stored in LSB of Y-axis
		uint8_t DEN_X:1; 	//DEN value stored in LSB of X-axis.
	} ISM330DLC_RegBits;
} Tun_Ctrl9_Xl;


typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved:2;
		uint8_t FUNC_EN:1;	//Enable embedded functionalities
		uint8_t TILT_EN:1;	//Enable tilt calculation
		uint8_t Reserved1:1;
		uint8_t TIMER_EN:1;	//Enable timestamp count
		uint8_t Reserved2:2;
	} ISM330DLC_RegBits;
} Tun_Ctrl10_C;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DRDY_ON_INT1:1; 		//Manage the master DRDY signal on INT1 pad.
		uint8_t DATA_VALID_SEL_FIFO:1;	//Selection of FIFO data-valid signal
		uint8_t START_CONFIG:1;			//Sensor hub trigger signal selection
		uint8_t PULL_UP_EN:1;			//Auxiliary I²C pull-up
		uint8_t PASS_THROUGH_MODE:1;	//I²C interface pass-through
		uint8_t Reserved:1;
		uint8_t IRON_EN:1;				//Enable hard-iron correction algorithm for magnetometer
		uint8_t MASTER_ON:1;			//Sensor hub I²C master enable
	} ISM330DLC_RegBits;
} Tun_Master_Config;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FF_IA:1; 			//Free-fall event detection status
		uint8_t SLEEP_STATE_IA:1; 	//Sleep event status
		uint8_t WU_IA:1;			//Wakeup event detection status
		uint8_t X_WU:1;				//Wakeup event detection status on X-axis
		uint8_t Y_WU:1;				//Wakeup event detection status on Y-axis
		uint8_t Z_WU:1;				//Wakeup event detection status on Z-axis
		uint8_t Reserved:2;
	} ISM330DLC_RegBits;
} Tun_Wake_Up_Src;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t TAP_IA:1; 		//Tap event detection status
		uint8_t SINGLE_TAP:1;	//Single-tap event status
		uint8_t DOUBLE_TAP:1;	//Double-tap event detection status
		uint8_t TAP_SIGN:1;		//Sign of acceleration detected by tap event
		uint8_t X_TAP:1;		//Tap event detection status on X-axis
		uint8_t Y_TAP:1;		//Tap event detection status on Y-axis
		uint8_t Z_TAP:1;		//Tap event detection status on Z-axis
		uint8_t Reserved:1;
	} ISM330DLC_RegBits;
} Tun_Tap_Src;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DEN_DRDY:1; 	//DEN data-ready signal. It is set high when data output is related to the data coming from a DEN active	condition
		uint8_t D6D_IA:1;		//Interrupt active for change position portrait, landscape, face-up, face-down
		uint8_t ZH:1; 			//Z-axis high event (over threshold)
		uint8_t ZL:1;			//Z-axis low event (under threshold)
		uint8_t YH:1;			//Y-axis high event (over threshold)
		uint8_t YL:1;			//Y-axis low event (under threshold)
		uint8_t XH:1;			//X-axis high event (over threshold)
		uint8_t XL:1;			//X-axis low event (under threshold)
	} ISM330DLC_RegBits;
} Tun_D6d_Src;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t XLDA:1;	//Accelerometer new data available
		uint8_t GDA:1;	//Gyroscope new data available
		uint8_t TDA:1;	//Temperature new data available
		uint8_t Reserved:5;
	} ISM330DLC_RegBits;
} Tun_Status_Reg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t XLDA:1; 			//Accelerometer data available (reset when one of the high parts of the output data is read)
		uint8_t GDA:1;				//Gyroscope data available (reset when one of the high parts of the output data is read)
		uint8_t GYRO_SETTLING:1;	//Accelerometer data available (reset when one of the high parts of the output data is read)
		uint8_t Reserved:5;
	} ISM330DLC_RegBits;
} Tun_Status_Spi_Aux;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Temp0:1; //Temperature sensor output data
		uint8_t Temp1:1;
		uint8_t Temp2:1;
		uint8_t Temp3:1;
		uint8_t Temp4:1;
		uint8_t Temp5:1;
		uint8_t Temp6:1;
		uint8_t Temp7:1;
	} ISM330DLC_RegBits;
} Tun_Out_Temp_L;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Temp8:1; //Temperature sensor output data
		uint8_t Temp9:1;
		uint8_t Temp10:1;
		uint8_t Temp11:1;
		uint8_t Temp12:1;
		uint8_t Temp13:1;
		uint8_t Temp14:1;
		uint8_t Temp15:1;
	} ISM330DLC_RegBits;
} Tun_Out_Temp_H;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DIFF_FIFO_0:1;		//Number of unread words (16-bit axes) stored in FIFO
		uint8_t DIFF_FIFO_1:1;
		uint8_t DIFF_FIFO_2:1;
		uint8_t DIFF_FIFO_3:1;
		uint8_t DIFF_FIFO_4:1;
		uint8_t DIFF_FIFO_5:1;
		uint8_t DIFF_FIFO_6:1;
		uint8_t DIFF_FIFO_7:1;
	} ISM330DLC_RegBits;
} Tun_Fifo_Status1;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DIFF_FIFO_8:1;
		uint8_t DIFF_FIFO_9:1;
		uint8_t DIFF_FIFO_10:1;
		uint8_t Reserved:1;
		uint8_t FIFO_EMPTY:1;		//FIFO empty
		uint8_t FIFO_FULL_SMART:1;	//Smart FIFO full status
		uint8_t OVER_RUN:1;			//FIFO overrun status
		uint8_t WaterM:1;			//FIFO watermark status
	} ISM330DLC_RegBits;
} Tun_Fifo_Status2;


typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FIFO_PATTERN_0:1;	//Word of recursive pattern read at the next read
		uint8_t FIFO_PATTERN_1:1;
		uint8_t FIFO_PATTERN_2:1;
		uint8_t FIFO_PATTERN_3:1;
		uint8_t FIFO_PATTERN_4:1;
		uint8_t FIFO_PATTERN_5:1;
		uint8_t FIFO_PATTERN_6:1;
		uint8_t FIFO_PATTERN_7:1;
	} ISM330DLC_RegBits;
} Tun_Fifo_Status3;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FIFO_PATTERN_8:1;
		uint8_t FIFO_PATTERN_9:1;
		uint8_t Reserved:6;
	} ISM330DLC_RegBits;
} Tun_Fifo_Status4;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t SENSOR_HUB_END_OP:1;//Sensor hub communication status
		uint8_t SI_END_OP:1;		//Hard/soft-iron calculation status
		uint8_t HI_FAIL:1;			//Fail in hard/soft-ironing algorithm
		uint8_t Reserved:2;
		uint8_t TILT_IA:1;			//Tilt event detection status
		uint8_t Reserved1:2;
	} ISM330DLC_RegBits;
} Tun_Func_Src1;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved:3;
		uint8_t SLAVE0_NACK:1;
		uint8_t SLAVE1_NACK:1;
		uint8_t SLAVE2_NACK:1;
		uint8_t SLAVE3_NACK:1;
		uint8_t Reserved1:1;
	} ISM330DLC_RegBits;
} Tun_Func_Src2;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t LIR:1;			//Latched Interrupt
		uint8_t TAP_Z_EN:1;		//Enable Z direction in tap recognition
		uint8_t TAP_Y_EN:1;		//Enable Y direction in tap recognition
		uint8_t TAP_X_EN:1;		//Enable X direction in tap recognition
		uint8_t SLOPE_FDS:1;	//HPF or SLOPE filter selection on wake-up and activity/inactivity functions
		uint8_t INACT_EN0:1;		//Enable inactivity function
		uint8_t INACT_EN1:1;		//Enable inactivity function
		uint8_t INTERRUPTS_ENABLE:1;//Enable basic interrupts (6D/4D, free-fall, wake-up, tap, inactivity)
	} ISM330DLC_RegBits;
} Tun_Tap_Cfg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t TAP_THS0:1;		//Threshold for tap recognition
		uint8_t TAP_THS1:1;
		uint8_t TAP_THS2:1;
		uint8_t TAP_THS3:1;
		uint8_t TAP_THS4:1;
		uint8_t SIXD_THS0:1;	//Threshold for 4D/6D function
		uint8_t SIXD_THS1:1;
		uint8_t D4D_EN:1;		//4D orientation detection enable
	} ISM330DLC_RegBits;
} Tun_Tap_Ths_6D;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t SHOCK0:1;	//Maximum duration of overthreshold event
		uint8_t SHOCK1:1;
		uint8_t QUIET0:1;	//Expected quiet time after a tap detection
		uint8_t QUIET1:1;
		uint8_t DUR0:1;		//Duration of maximum time gap for double tap recognition
		uint8_t DUR1:1;
		uint8_t DUR2:1;
		uint8_t DUR3:1;
	} ISM330DLC_RegBits;
} Tun_Int_Dur2;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t WK_THS0:1;				//Threshold for wakeup
		uint8_t WK_THS1:1;
		uint8_t WK_THS2:1;
		uint8_t WK_THS3:1;
		uint8_t WK_THS4:1;
		uint8_t WK_THS5:1;
		uint8_t Reserved:1;				//
		uint8_t SINGLE_DOUBLE_TAP:1;	//Single/double-tap event enable
	} ISM330DLC_RegBits;
} Tun_Wake_Up_Ths;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t SLEEP_DUR0:1;	//Duration to go in sleep mode
		uint8_t SLEEP_DUR1:1;
		uint8_t SLEEP_DUR2:1;
		uint8_t SLEEP_DUR3:1;
		uint8_t TIMER_HR:1;		//Timestamp register resolution setting
		uint8_t WAKE_DUR0:1;	//Wake up duration event
		uint8_t WAKE_DUR1:1;
		uint8_t FF_DUR0:1;		//Free fall duration event.
	} ISM330DLC_RegBits;
} Tun_Wake_Up_Dur;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FF_THS0:1; 		//Free fall threshold setting
		uint8_t FF_THS1:1;
		uint8_t FF_THS2:1;
		uint8_t FF_DUR0:1;		//Free-fall duration event
		uint8_t FF_DUR1:1;
		uint8_t FF_DUR2:1;
		uint8_t FF_DUR3:1;
		uint8_t FF_DUR4:1;
	} ISM330DLC_RegBits;
} Tun_Free_Fall;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t INT1_TIMER:1;			//Routing of end counter event of timer on INT1
		uint8_t INT1_TILT:1;			//Routing of tilt event on INT1
		uint8_t INT1_6D:1;				//Routing of 6D event on INT1
		uint8_t INT1_DOUBLE_TAP:1;		//Routing of tap event on INT1
		uint8_t INT1_FF:1;				//Routing of free-fall event on INT1
		uint8_t INT1_WU:1;				//Routing of wakeup event on INT1
		uint8_t INT1_SINGLE_TAP:1;		//Single-tap recognition routing on INT1
		uint8_t INT1_INACT_STATE:1; 	//Routing on INT1 of inactivity mode
	} ISM330DLC_RegBits;
} Tun_Md1_Cfg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t INT2_IRON:1;		//Routing of soft-iron/hard-iron algorithm end event on INT2
		uint8_t INT2_TILT:1;		//Routing of tilt event on INT2
		uint8_t INT2_6D:1;			//Routing of 6D event on INT2
		uint8_t INT2_DOUBLE_TAP:1;	//Routing of tap event on INT2
		uint8_t INT2_FF:1;			//Routing of free-fall event on INT2
		uint8_t INT2_WU:1;			//Routing of wakeup event on INT2
		uint8_t INT2_SINGLE_TAP:1;	//Single-tap recognition routing on INT2
		uint8_t INT2_INACT_STATE:1;	//Routing on INT2 of inactivity mode
	} ISM330DLC_RegBits;
} Tun_Md2_Cfg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved:6;
		uint8_t LVL2_OIS:1;
		uint8_t INT2_DRDY_OIS:1;
	} ISM330DLC_RegBits;
} Tun_Int_Ois;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t OIS_EN_SPI2:1;	//Enables OIS chain data processing for gyro in Mode 3 and Mode 4 (mode4_en = 1) and accelerometer data	in and Mode 4 (mode4_en = 1).
		uint8_t FS_125_OIS:1;	//Selects gyroscope OIS chain full scale ±125 dps
		uint8_t FS0_G_OIS:1;	//Gyroscope OIS chain full-scale selection
		uint8_t FS1_G_OIS:1;	//
		uint8_t MODE4_EN:1;		//Enables accelerometer OIS chain if OIS_EN_SPI2 = 1
		uint8_t SIM_OIS:1;		//SPI2 3- or 4-wire mode
		uint8_t LVL1_OIS:1;		//Enables level-sensitive trigger mode on OIS chain
		uint8_t BLE_OIS:1; 		//Big/Little Endian data selection
	} ISM330DLC_RegBits;
} Tun_Ctrl1_Ois;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t HP_EN_OIS:1;	//Enables gyroscope's OIS chain HPF
		uint8_t FTYPE_0_OIS:1;	//Gyroscope's digital LPF1 filter bandwidth selection
		uint8_t FTYPE_1_OIS:1;
		uint8_t Reserved:1;
		uint8_t HPM0_OIS:1;		//Gyroscope's OIS chain digital high-pass filter cutoff selection
		uint8_t HPM1_OIS:1;		//
		uint8_t Reserved1:2;
	} ISM330DLC_RegBits;
} Tun_Ctrl2_Ois;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t ST_OIS_CLAMPDIS:1;		//Gyro OIS chain clamp disable
		uint8_t ST0_OIS:1;				//Gyroscope OIS chain self-test selection
		uint8_t ST1_OIS:1;				//
		uint8_t FILTER_XL_CONF_OIS_0:1;	//Accelerometer OIS channel bandwidth selection
		uint8_t FILTER_XL_CONF_OIS_1:1;
		uint8_t FS0_XL_OIS:1;			//
		uint8_t FS1_XL_OIS:1;			//Accelerometer OIS channel full-scale selection
		uint8_t DEN_LH_OIS:1; 			//Polarity of DEN signal on OIS chain
	} ISM330DLC_RegBits;
} Tun_Ctrl3_Ois;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t rw_0:1;			//Read/write operation on Sensor1
		uint8_t Slave0_add0:1; 	//I²C slave address of Sensor1 that can be read by sensor hub
		uint8_t Slave0_add1:1;
		uint8_t Slave0_add2:1;
		uint8_t Slave0_add3:1;
		uint8_t Slave0_add4:1;
		uint8_t Slave0_add5:1;
		uint8_t Slave0_add6:1;
	} ISM330DLC_RegBits;
} Tun_Slv0_Add;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Slave0_numop0:1;	//Number of read operations on Sensor1.
		uint8_t Slave0_numop1:1;
		uint8_t Slave0_numop2:1;
		uint8_t Src_mode:1;			//Source mode conditioned read
		uint8_t Aux_sens_on0:1;		//Number of external sensors to be read by sensor hub
		uint8_t Aux_sens_on1:1;
		uint8_t Slave0_rate0:1;	//Decimation of read operation on Sensor1 starting from the sensor hub trigger
		uint8_t Slave0_rate1:1;
	} ISM330DLC_RegBits;
} Tun_Slave0_Config;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t r_1:1;			//Read operation on Sensor2 enable
		uint8_t Slave1_add0:1;	//I²C slave address of Sensor2 that can be read by sensor hub
		uint8_t Slave1_add1:1;
		uint8_t Slave1_add2:1;
		uint8_t Slave1_add3:1;
		uint8_t Slave1_add4:1;
		uint8_t Slave1_add5:1;
		uint8_t Slave1_add6:1;
	} ISM330DLC_RegBits;
} Tun_Slv1_Add;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Slave1_numop0:1;
		uint8_t Slave1_numop1:1;
		uint8_t Slave1_numop2:1;
		uint8_t Reserved:2;
		uint8_t write_once:1;
		uint8_t Slave1_rate0:1;
		uint8_t Slave1_rate1:1;
	} ISM330DLC_RegBits;
} Tun_Slave1_Config;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t r_2:1;			//Read operation on Sensor2 enable
		uint8_t Slave2_add0:1;	//I²C slave address of Sensor2 that can be read by sensor hub
		uint8_t Slave2_add1:1;
		uint8_t Slave2_add2:1;
		uint8_t Slave2_add3:1;
		uint8_t Slave2_add4:1;
		uint8_t Slave2_add5:1;
		uint8_t Slave2_add6:1;
	} ISM330DLC_RegBits;
} Tun_Slv2_Add;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Slave2_numop0:1;
		uint8_t Slave2_numop1:1;
		uint8_t Slave2_numop2:1;
		uint8_t Reserved:3;
		uint8_t Slave2_rate0:1;
		uint8_t Slave2_rate1:1;
	} ISM330DLC_RegBits;
} Tun_Slave2_Config;


typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t r_3:1;			//Read operation on Sensor2 enable
		uint8_t Slave3_add0:1;	//I²C slave address of Sensor2 that can be read by sensor hub
		uint8_t Slave3_add1:1;
		uint8_t Slave3_add2:1;
		uint8_t Slave3_add3:1;
		uint8_t Slave3_add4:1;
		uint8_t Slave3_add5:1;
		uint8_t Slave3_add6:1;
	} ISM330DLC_RegBits;
} Tun_Slv3_Add;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Slave3_numop0:1;
		uint8_t Slave3_numop1:1;
		uint8_t Slave3_numop2:1;
		uint8_t Reserved:3;
		uint8_t Slave3_rate0:1;
		uint8_t Slave3_rate1:1;
	} ISM330DLC_RegBits;
} Tun_Slave3_Config;


typedef volatile union
{
	struct
	{
		uint8_t Reserved1;
		Tun_Runc_Cfg_Access FUNC_CFG_ACCESS;
		uint8_t Reserved2;
		uint8_t Reserved3;
		Tun_Sen_Tim_Fram SENSOR_SYNC_TIME_FRAME;
		Tun_Sen_Sync_Res_Ratio SENSOR_SYNC_RES_RATIO;
		Tun_Fifo_Ctrl1 FIFO_CTRL1;
		Tun_Fifo_Ctrl2 FIFO_CTRL2;
		Tun_Fifo_Ctrl3 FIFO_CTRL3;
		Tun_Fifo_Ctrl4 FIFO_CTRL4;
		Tun_Fifo_Ctrl5 FIFO_CTRL5;
		Tun_Drdy_Pulse_Cfg DRDY_PULSE_CFG;
		uint8_t Reserved4;
		Tun_Int1_Ctrl INT1_CTRL;
		Tun_Int2_Ctrl INT2_CTRL;
		uint8_t WHO_AM_I;
		Tun_Ctrl1_Xl CTRL1_XL;
		Tun_Ctrl2_G CTRL2_G;
		Tun_Ctrl3_C CTRL3_C;
		Tun_Ctrl4_C CTRL4_C;
		Tun_Ctrl5_C CTRL5_C;
		Tun_Ctrl6_C CTRL6_C;
		Tun_Ctrl7_G CTRL7_G;
		Tun_Ctrl8_Xl CTRL8_XL;
		Tun_Ctrl9_Xl CTRL9_XL;
		Tun_Ctrl10_C CTRL10_C;
		Tun_Master_Config MASTER_CONFIG;
		Tun_Wake_Up_Src WAKE_UP_SRC;
		Tun_Tap_Src TAP_SRC;
		Tun_D6d_Src D6D_SRC;
		Tun_Status_Reg STATUS_REG;
		Tun_Status_Spi_Aux STATUS_SPIAux;
		uint8_t Reserved44;
		Tun_Out_Temp_L OUT_TEMP_L;
		Tun_Out_Temp_H OUT_TEMP_H;
		uint8_t OUTX_L_G;
		uint8_t OUTX_H_G;
		uint8_t OUTY_L_G;
		uint8_t OUTY_H_G;
		uint8_t OUTZ_L_G;
		uint8_t OUTZ_H_G;
		uint8_t OUTX_L_XL;
		uint8_t OUTX_H_XL;
		uint8_t OUTY_L_XL;
		uint8_t OUTY_H_XL;
		uint8_t OUTZ_L_XL;
		uint8_t OUTZ_H_XL;
		uint8_t SENSORHUB1_REG;
		uint8_t SENSORHUB2_REG;
		uint8_t SENSORHUB3_REG;
		uint8_t SENSORHUB4_REG;
		uint8_t SENSORHUB5_REG;
		uint8_t SENSORHUB6_REG;
		uint8_t SENSORHUB7_REG;
		uint8_t SENSORHUB8_REG;
		uint8_t SENSORHUB9_REG;
		uint8_t SENSORHUB10_REG;
		uint8_t SENSORHUB11_REG;
		uint8_t SENSORHUB12_REG;
		Tun_Fifo_Status1 FIFO_STATUS1;
		Tun_Fifo_Status2 FIFO_STATUS2;
		Tun_Fifo_Status3 FIFO_STATUS3;
		Tun_Fifo_Status4 FIFO_STATUS4;
		uint8_t FIFO_DATA_OUT_L;
		uint8_t FIFO_DATA_OUT_H;
		uint8_t TIMESTAMP0_REG;
		uint8_t TIMESTAMP1_REG;
		uint8_t TIMESTAMP2_REG;
		uint8_t Reserved5;
		uint8_t SENSORHUB13_REG;
		uint8_t SENSORHUB14_REG;
		uint8_t SENSORHUB15_REG;
		uint8_t SENSORHUB16_REG;
		uint8_t SENSORHUB17_REG;
		uint8_t SENSORHUB18_REG;
		Tun_Func_Src1 FUNC_SRC1;
		Tun_Func_Src2 FUNC_SRC2;
		uint8_t Reserved6;
		Tun_Tap_Cfg TAP_CFG;
		Tun_Tap_Ths_6D TAP_THS_6D;
		Tun_Int_Dur2 INT_DUR2;
		Tun_Wake_Up_Ths WAKE_UP_THS;
		Tun_Wake_Up_Dur WAKE_UP_DUR;
		Tun_Free_Fall FREE_FALL;
		Tun_Md1_Cfg MD1_CFG;
		Tun_Md2_Cfg MD2_CFG;
		uint8_t MASTER_CMD_CODE;
		uint8_t SENS_SYNC_SPI_ERROR_CODE;
		uint8_t Reserved7;
		uint8_t OUT_MAG_RAW_X_L;
		uint8_t OUT_MAG_RAW_X_H;
		uint8_t OUT_MAG_RAW_Y_L;
		uint8_t OUT_MAG_RAW_Y_H;
		uint8_t OUT_MAG_RAW_Z_L;
		uint8_t OUT_MAG_RAW_Z_H;
		uint8_t Reserved8;
		Tun_Int_Ois INT_OIS;
		Tun_Ctrl1_Ois CTRL1_OIS;
		Tun_Ctrl2_Ois CTRL2_OIS;
		Tun_Ctrl3_Ois CTRL3_OIS;
		uint8_t X_OFS_USR;
		uint8_t Y_OFS_USR;
		uint8_t Z_OFS_USR;
		uint8_t Reserved9;
	} ISM330DLC_Reg_Struct;
	uint8_t ISM330DLC_Reg_Tab[104];
} ISM330DLC_iDriver_Description;



#endif /* IMC330DLC_H_ */
