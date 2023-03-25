/*
 * ism330dlc_int.h
 *
 *  Created on: Mar 14, 2023
 *      Author: macbookpro
 */

#ifndef ISM330DLC_INT_H_
#define ISM330DLC_INT_H_

#define ISM330DLC_FUNC_CFG_ACCESS              0x01U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved :7;
		uint8_t FUNC_CFG_EN :1; 	/*Enable access to the embedded functions configuration registers*/
	} ISM330DLC_RegBits;
} Tun_Runc_Cfg_Access;

#define ISM330DLC_SENSOR_SYNC_TIME_FRAME       0x04U
typedef volatile union
{
	uint8_t All;
	struct
	{

		uint8_t TPH_3 :1; 			/*Sensor synchronization time frame with the step of 500 ms and full range of 5 s*/
		uint8_t TPH_2 :1;
		uint8_t TPH_1 :1;
		uint8_t TPH_0 :1;
		uint8_t Reserved :4;
	} ISM330DLC_RegBits;
} Tun_Sen_Tim_Fram;

#define ISM330DLC_SENSOR_SYNC_RES_RATIO        0x05U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t RR_0 :1; 			/*Resolution ratio of error code for sensor synchronization*/
		uint8_t RR_1 :1;
		uint8_t Reserved :6;
	} ISM330DLC_RegBits;
} Tun_Sen_Sync_Res_Ratio;

#define ISM330DLC_FIFO_CTRL1                   0x06U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FTH_0 :1; 			/*FIFO threshold level setting.*/
		uint8_t FTH_1 :1;
		uint8_t FTH_2 :1;
		uint8_t FTH_3 :1;
		uint8_t FTH_4 :1;
		uint8_t FTH_5 :1;
		uint8_t FTH_6 :1;
		uint8_t FTH_7 :1;
	} ISM330DLC_RegBits;
} Tun_Fifo_Ctrl1;

#define ISM330DLC_FIFO_CTRL2                   0x07U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FTH_8 :1;
		uint8_t FTH_9 :1;
		uint8_t FTH_10 :1;
		uint8_t FIFO_TEMP_EN:1; 		/*Enables the temperature data storage in FIFO*/
		uint8_t Reserved :3;
		uint8_t FIFO_TIMER_EN :1; 	/*Enables timestamp data to be stored as the 4th FIFO data se*/
	} ISM330DLC_RegBits;
} Tun_Fifo_Ctrl2;

#define ISM330DLC_FIFO_CTRL3                   0x08U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DEC_FIFO_XL0 :1; 	/*Accelerometer FIFO (second data set) decimation setting*/
		uint8_t DEC_FIFO_XL1 :1;
		uint8_t DEC_FIFO_XL2 :1;
		uint8_t DEC_FIFO_GYRO0 :1; 	/*Gyro FIFO (first data set) decimation setting*/
		uint8_t DEC_FIFO_GYRO1 :1;
		uint8_t DEC_FIFO_GYRO2 :1;
		uint8_t Reserved :2;
	} ISM330DLC_RegBits;
} Tun_Fifo_Ctrl3;

#define ISM330DLC_FIFO_CTRL4                   0x09U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DEC_DS3_FIFO0 :1; 	/*Third FIFO data set decimation setting*/
		uint8_t DEC_DS3_FIFO1 :1;
		uint8_t DEC_DS3_FIFO2 :1;
		uint8_t DEC_DS4_FIFO0 :1; 	/*Fourth FIFO data set decimation setting*/
		uint8_t DEC_DS4_FIFO1 :1;
		uint8_t DEC_DS4_FIFO2 :1;
		uint8_t ONLY_HIGH_DATA :1; 	/*8-bit data storage in FIFO*/
		uint8_t STOP_ON_FTH :1; 	/*Enable FIFO threshold level use*/
	} ISM330DLC_RegBits;
} Tun_Fifo_Ctrl4;

#define ISM330DLC_FIFO_CTRL5                   0x0AU
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FIFO_MODE_0 :1; 	/*FIFO mode selection bits, setting ODR_FIFO also*/
		uint8_t FIFO_MODE_1 :1;
		uint8_t FIFO_MODE_2 :1;
		uint8_t ODR_FIFO_0 :1; 		/*FIFO ODR selection, setting FIFO_MODE also*/
		uint8_t ODR_FIFO_1 :1;
		uint8_t ODR_FIFO_2 :1;
		uint8_t ODR_FIFO_3 :1;
		uint8_t Reserved :1;
	} ISM330DLC_RegBits;
} Tun_Fifo_Ctrl5;

#define ISM330DLC_DRDY_PULSE_CFG               0x0BU
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reseved :7;
		uint8_t DRDY_PULSED :1; 	/*Enable pulsed data-ready mode*/
	} ISM330DLC_RegBits;
} Tun_Drdy_Pulse_Cfg;

#define ISM330DLC_INT1_CTRL                    0x0DU
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t NT1_FULL_FLAG :1;	/*FIFO full flag interrupt enable on INT1 pad*/
		uint8_t INT1_FIFO_OVR :1;	/*FIFO overrun interrupt on INT1 pad*/
		uint8_t INT1_FTH :1; 		/*FIFO threshold interrupt on INT1 pad*/
		uint8_t INT1_BOOT :1; 		/*Boot status available on INT1 pad*/
		uint8_t INT1_DRDY_G :1; 	/*Gyroscope data-ready on INT1 pad*/
		uint8_t INT1_DRDY_XL :1; 	/*Accelerometer data-ready on INT1 pad*/
		uint8_t Reversed :2;
	} ISM330DLC_RegBits;
} Tun_Int1_Ctrl;

#define ISM330DLC_INT2_CTRL                    0x0EU
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t INT2_FULL_FLAG :1;		//FIFO full flag interrupt enable on INT2 pad
		uint8_t INT2_FIFO_OVR :1;		//FIFO overrun interrupt on INT2 pad
		uint8_t INT2_FTH :1;			//FIFO threshold interrupt on INT2 pad
		uint8_t INT2_DRDY_TEMP :1;		//Temperature data-ready on INT2 pad
		uint8_t INT2_DRDY_G :1;			//Gyroscope data-ready on INT2 pad
		uint8_t INT2_DRDY_XL :1;		//Accelerometer data-ready on INT2 pad
		uint8_t Reserved :2;
	} ISM330DLC_RegBits;
} Tun_Int2_Ctrl;

#define ISM330DLC_WHO_AM_I                     0x0FU
#define ISM330DLC_CTRL1_XL                     0x10U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t BW0_XL :1; 		//Accelerometer analog chain bandwidth selection (only for accelerometer ODR ≥ 1.67 kHz)
		uint8_t LPF1_BW_SEL :1; //Accelerometer digital LPF (LPF1) bandwidth selection
		uint8_t FS_XL1 :1; 		//Accelerometer full-scale selection
		uint8_t FS_XL2 :1;
		uint8_t ODR_XL0 :1;		//Output data rate and power mode selection
		uint8_t ODR_XL1 :1;
		uint8_t ODR_XL2 :1;
		uint8_t ODR_XL3 :1;
	} ISM330DLC_RegBits;
} Tun_Ctrl1_Xl;

#define ISM330DLC_CTRL2_G                      0x11U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reversed :1;
		uint8_t FS_125 :1; //Gyroscope full-scale at ±125 dps
		uint8_t FS_G0 :1;  //Gyroscope full-scale selection
		uint8_t FS_G1 :1;
		uint8_t ODR_G0 :1; //Gyroscope output data rate selection
		uint8_t ODR_G1 :1;
		uint8_t ODR_G2 :1;
		uint8_t ODR_G3 :1;
	} ISM330DLC_RegBits;
} Tun_Ctrl2_G;

#define ISM330DLC_CTRL3_C                      0x12U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t SW_RESET :1; 	//Software reset
		uint8_t BLE :1; 		//Big/Little Endian Data selection
		uint8_t IF_INC :1;		//Register address automatically incremented during a multiple byte access with a serial interface (I²C or SPI)
		uint8_t SIM :1;			//SPI Serial Interface Mode selection.
		uint8_t PP_OD :1;		//Push-pull/open-drain selection on INT1 and INT2 pads
		uint8_t H_LACTIVE :1;	//Interrupt activation level.
		uint8_t BDU :1;      	//Block Data Update.
		uint8_t BOOT :1; 		//Reboots memory content
	} ISM330DLC_RegBits;
} Tun_Ctrl3_C;

#define ISM330DLC_CTRL4_C                      0x13U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved :1;
		uint8_t LPF1_SEL_G :1;		//Enable gyroscope digital LPF1 if auxiliary SPI is disabled
		uint8_t I2C_DISABLE :1;		//Disable I²C interface
		uint8_t DRDY_MASK :1;		//Configuration 1 data available enable bit
		uint8_t DEN_DRDY_INT1 :1;	//DEN DRDY signal on INT1 pad
		uint8_t INT2_ON_INT1 :1;	//All interrupt signals available on INT1 pad enable
		uint8_t SLEEP :1;	 		//Gyroscope sleep mode enable
		uint8_t DEN_XL_EN :1; 		//Extend DEN functionality to accelerometer sensor
	} ISM330DLC_RegBits;
} Tun_Ctrl4_C;

#define ISM330DLC_CTRL5_C                      0x14U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t ST0_XL :1;	 //Linear acceleration sensor self-test enable
		uint8_t ST1_XL :1;
		uint8_t ST0_G :1;	 //Angular rate sensor self-test enable
		uint8_t ST1_G :1;
		uint8_t DEN_LH :1;	 //DEN active level configuration
		uint8_t ROUNDING0 :1; //Circular burst-mode (rounding) read from output registers through the primary interface
		uint8_t ROUNDING1 :1;
		uint8_t ROUNDING2 :1;
	} ISM330DLC_RegBits;
} Tun_Ctrl5_C;

#define ISM330DLC_CTRL6_C                      0x15U
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FTYPE_0 :1; 		//Gyroscope low-pass filter (LPF1) bandwidth selection
		uint8_t FTYPE_1 :1;
		uint8_t Reversed :1;
		uint8_t USR_OFF_W :1; 	//Weight of XL user offset bits of registers
		uint8_t XL_HM_MDOE :1; 	//High-performance operating mode disable for accelerometer
		uint8_t LVL1_EN :1; 		//DEN level-sensitive latched enable
		uint8_t LVL2_EN :1; 		//DEN level-sensitive latched enable
		uint8_t RTIG_EN :1; 		//DEN data edge-sensitive trigger enable
	} ISM330DLC_RegBits;
} Tun_Ctrl6_C;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved :2;
		uint8_t ROUNDING_STATUS :1; 	//Source register rounding function on WAKE_UP_SRC
		uint8_t Reserved1 :1;
		uint8_t HPM0_G :1;			//Gyroscope digital HP filter cutoff selection.
		uint8_t HPM1_G :1;
		uint8_t HP_EN_G :1;			//Gyroscope digital high-pass filter enable
		uint8_t G_HM_MODE :1;		//High-performance operating mode disable for gyroscope
	} ISM330DLC_RegBits;
} Tun_Ctrl7_G;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t LOW_PASS_ON_6D :1;	//LPF2 on 6D function selection
		uint8_t Reserved :1;
		uint8_t HP_SLOPE_XL_EN :1;	//Accelerometer slope filter / high-pass filter selection
		uint8_t INPUT_COMPOSITE :1;	//Composite filter input selection
		uint8_t HP_REF_MODE :1;		//Enable HP filter reference mode
		uint8_t HPCF_XL0 :1;			//Accelerometer LPF2 and high-pass filter configuration and cutoff setting
		uint8_t HPCF_XL1 :1;
		uint8_t LPF2_XL_EN :1;		//Accelerometer low-pass filter LPF2 selection
	} ISM330DLC_RegBits;
} Tun_Ctrl8_Xl;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved :2;
		uint8_t SOFT_EN :1;		//Enable soft-iron correction algorithm for magnetometer
		uint8_t Reserved1 :1;
		uint8_t DEN_XL_G :1;	//DEN stamping sensor selection.
		uint8_t DEN_Z :1;		//DEN value stored in LSB of Z-axis
		uint8_t DEN_Y :1;		//DEN value stored in LSB of Y-axis
		uint8_t DEN_X :1; 		//DEN value stored in LSB of X-axis.
	} ISM330DLC_RegBits;
} Tun_Ctrl9_Xl;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved :2;
		uint8_t FUNC_EN :1;		//Enable embedded functionalities
		uint8_t TILT_EN :1;		//Enable tilt calculation
		uint8_t Reserved1 :1;
		uint8_t TIMER_EN :1;	//Enable timestamp count
		uint8_t Reserved2 :2;
	} ISM330DLC_RegBits;
} Tun_Ctrl10_C;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DRDY_ON_INT1 :1; 		//Manage the master DRDY signal on INT1 pad.
		uint8_t DATA_VALID_SEL_FIFO :1;	//Selection of FIFO data-valid signal
		uint8_t START_CONFIG :1;		//Sensor hub trigger signal selection
		uint8_t PULL_UP_EN :1;			//Auxiliary I²C pull-up
		uint8_t PASS_THROUGH_MODE :1;	//I²C interface pass-through
		uint8_t Reserved :1;
		uint8_t IRON_EN :1;				//Enable hard-iron correction algorithm for magnetometer
		uint8_t MASTER_ON :1;			//Sensor hub I²C master enable
	} ISM330DLC_RegBits;
} Tun_Master_Config;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FF_IA :1; 			//Free-fall event detection status
		uint8_t SLEEP_STATE_IA :1; 	//Sleep event status
		uint8_t WU_IA :1;			//Wakeup event detection status
		uint8_t X_WU :1;				//Wakeup event detection status on X-axis
		uint8_t Y_WU :1;				//Wakeup event detection status on Y-axis
		uint8_t Z_WU :1;				//Wakeup event detection status on Z-axis
		uint8_t Reserved :2;
	} ISM330DLC_RegBits;
} Tun_Wake_Up_Src;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t TAP_IA :1; 		//Tap event detection status
		uint8_t SINGLE_TAP :1;	//Single-tap event status
		uint8_t DOUBLE_TAP :1;	//Double-tap event detection status
		uint8_t TAP_SIGN :1;		//Sign of acceleration detected by tap event
		uint8_t X_TAP :1;		//Tap event detection status on X-axis
		uint8_t Y_TAP :1;		//Tap event detection status on Y-axis
		uint8_t Z_TAP :1;		//Tap event detection status on Z-axis
		uint8_t Reserved :1;
	} ISM330DLC_RegBits;
} Tun_Tap_Src;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DEN_DRDY :1; 	//DEN data-ready signal. It is set high when data output is related to the data coming from a DEN active	condition
		uint8_t D6D_IA :1;		//Interrupt active for change position portrait, landscape, face-up, face-down
		uint8_t ZH :1; 			//Z-axis high event (over threshold)
		uint8_t ZL :1;			//Z-axis low event (under threshold)
		uint8_t YH :1;			//Y-axis high event (over threshold)
		uint8_t YL :1;			//Y-axis low event (under threshold)
		uint8_t XH :1;			//X-axis high event (over threshold)
		uint8_t XL :1;			//X-axis low event (under threshold)
	} ISM330DLC_RegBits;
} Tun_D6d_Src;

#define ISM330DLC_STATUS_REG             0x1EU
typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t XLDA :1;	//Accelerometer new data available
		uint8_t GDA :1;	//Gyroscope new data available
		uint8_t TDA :1;	//Temperature new data available
		uint8_t Reserved :5;
	} ISM330DLC_RegBits;
} Tun_Status_Reg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t XLDA :1; 			//Accelerometer data available (reset when one of the high parts of the output data is read)
		uint8_t GDA :1;				//Gyroscope data available (reset when one of the high parts of the output data is read)
		uint8_t GYRO_SETTLING :1;	//Accelerometer data available (reset when one of the high parts of the output data is read)
		uint8_t Reserved :5;
	} ISM330DLC_RegBits;
} Tun_Status_Spi_Aux;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Temp0 :1; //Temperature sensor output data
		uint8_t Temp1 :1;
		uint8_t Temp2 :1;
		uint8_t Temp3 :1;
		uint8_t Temp4 :1;
		uint8_t Temp5 :1;
		uint8_t Temp6 :1;
		uint8_t Temp7 :1;
	} ISM330DLC_RegBits;
} Tun_Out_Temp_L;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Temp8 :1; //Temperature sensor output data
		uint8_t Temp9 :1;
		uint8_t Temp10 :1;
		uint8_t Temp11 :1;
		uint8_t Temp12 :1;
		uint8_t Temp13 :1;
		uint8_t Temp14 :1;
		uint8_t Temp15 :1;
	} ISM330DLC_RegBits;
} Tun_Out_Temp_H;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DIFF_FIFO_0 :1;		//Number of unread words (16-bit axes) stored in FIFO
		uint8_t DIFF_FIFO_1 :1;
		uint8_t DIFF_FIFO_2 :1;
		uint8_t DIFF_FIFO_3 :1;
		uint8_t DIFF_FIFO_4 :1;
		uint8_t DIFF_FIFO_5 :1;
		uint8_t DIFF_FIFO_6 :1;
		uint8_t DIFF_FIFO_7 :1;
	} ISM330DLC_RegBits;
} Tun_Fifo_Status1;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t DIFF_FIFO_8 :1;
		uint8_t DIFF_FIFO_9 :1;
		uint8_t DIFF_FIFO_10 :1;
		uint8_t Reserved :1;
		uint8_t FIFO_EMPTY :1;		//FIFO empty
		uint8_t FIFO_FULL_SMART :1;	//Smart FIFO full status
		uint8_t OVER_RUN :1;			//FIFO overrun status
		uint8_t WaterM :1;			//FIFO watermark status
	} ISM330DLC_RegBits;
} Tun_Fifo_Status2;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FIFO_PATTERN_0 :1;	//Word of recursive pattern read at the next read
		uint8_t FIFO_PATTERN_1 :1;
		uint8_t FIFO_PATTERN_2 :1;
		uint8_t FIFO_PATTERN_3 :1;
		uint8_t FIFO_PATTERN_4 :1;
		uint8_t FIFO_PATTERN_5 :1;
		uint8_t FIFO_PATTERN_6 :1;
		uint8_t FIFO_PATTERN_7 :1;
	} ISM330DLC_RegBits;
} Tun_Fifo_Status3;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FIFO_PATTERN_8 :1;
		uint8_t FIFO_PATTERN_9 :1;
		uint8_t Reserved :6;
	} ISM330DLC_RegBits;
} Tun_Fifo_Status4;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t SENSOR_HUB_END_OP :1;	//Sensor hub communication status
		uint8_t SI_END_OP :1;		//Hard/soft-iron calculation status
		uint8_t HI_FAIL :1;			//Fail in hard/soft-ironing algorithm
		uint8_t Reserved :2;
		uint8_t TILT_IA :1;			//Tilt event detection status
		uint8_t Reserved1 :2;
	} ISM330DLC_RegBits;
} Tun_Func_Src1;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved :3;
		uint8_t SLAVE0_NACK :1;
		uint8_t SLAVE1_NACK :1;
		uint8_t SLAVE2_NACK :1;
		uint8_t SLAVE3_NACK :1;
		uint8_t Reserved1 :1;
	} ISM330DLC_RegBits;
} Tun_Func_Src2;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t LIR :1;				//Latched Interrupt
		uint8_t TAP_Z_EN :1;		//Enable Z direction in tap recognition
		uint8_t TAP_Y_EN :1;		//Enable Y direction in tap recognition
		uint8_t TAP_X_EN :1;		//Enable X direction in tap recognition
		uint8_t SLOPE_FDS :1;		//HPF or SLOPE filter selection on wake-up and activity/inactivity functions
		uint8_t INACT_EN0 :1;		//Enable inactivity function
		uint8_t INACT_EN1 :1;		//Enable inactivity function
		uint8_t INTERRUPTS_ENABLE :1;		//Enable basic interrupts (6D/4D, free-fall, wake-up, tap, inactivity)
	} ISM330DLC_RegBits;
} Tun_Tap_Cfg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t TAP_THS0 :1;		//Threshold for tap recognition
		uint8_t TAP_THS1 :1;
		uint8_t TAP_THS2 :1;
		uint8_t TAP_THS3 :1;
		uint8_t TAP_THS4 :1;
		uint8_t SIXD_THS0 :1;		//Threshold for 4D/6D function
		uint8_t SIXD_THS1 :1;
		uint8_t D4D_EN :1;			//4D orientation detection enable
	} ISM330DLC_RegBits;
} Tun_Tap_Ths_6D;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t SHOCK0 :1;		//Maximum duration of overthreshold event
		uint8_t SHOCK1 :1;
		uint8_t QUIET0 :1;		//Expected quiet time after a tap detection
		uint8_t QUIET1 :1;
		uint8_t DUR0 :1;		//Duration of maximum time gap for double tap recognition
		uint8_t DUR1 :1;
		uint8_t DUR2 :1;
		uint8_t DUR3 :1;
	} ISM330DLC_RegBits;
} Tun_Int_Dur2;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t WK_THS0 :1;				//Threshold for wakeup
		uint8_t WK_THS1 :1;
		uint8_t WK_THS2 :1;
		uint8_t WK_THS3 :1;
		uint8_t WK_THS4 :1;
		uint8_t WK_THS5 :1;
		uint8_t Reserved :1;				//
		uint8_t SINGLE_DOUBLE_TAP :1;	//Single/double-tap event enable
	} ISM330DLC_RegBits;
} Tun_Wake_Up_Ths;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t SLEEP_DUR0 :1;	//Duration to go in sleep mode
		uint8_t SLEEP_DUR1 :1;
		uint8_t SLEEP_DUR2 :1;
		uint8_t SLEEP_DUR3 :1;
		uint8_t TIMER_HR :1;	//Timestamp register resolution setting
		uint8_t WAKE_DUR0 :1;	//Wake up duration event
		uint8_t WAKE_DUR1 :1;
		uint8_t FF_DUR0 :1;		//Free fall duration event.
	} ISM330DLC_RegBits;
} Tun_Wake_Up_Dur;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t FF_THS0 :1; 	//Free fall threshold setting
		uint8_t FF_THS1 :1;
		uint8_t FF_THS2 :1;
		uint8_t FF_DUR0 :1;		//Free-fall duration event
		uint8_t FF_DUR1 :1;
		uint8_t FF_DUR2 :1;
		uint8_t FF_DUR3 :1;
		uint8_t FF_DUR4 :1;
	} ISM330DLC_RegBits;
} Tun_Free_Fall;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t INT1_TIMER :1;			//Routing of end counter event of timer on INT1
		uint8_t INT1_TILT :1;			//Routing of tilt event on INT1
		uint8_t INT1_6D :1;				//Routing of 6D event on INT1
		uint8_t INT1_DOUBLE_TAP :1;		//Routing of tap event on INT1
		uint8_t INT1_FF :1;				//Routing of free-fall event on INT1
		uint8_t INT1_WU :1;				//Routing of wakeup event on INT1
		uint8_t INT1_SINGLE_TAP :1;		//Single-tap recognition routing on INT1
		uint8_t INT1_INACT_STATE :1; 	//Routing on INT1 of inactivity mode
	} ISM330DLC_RegBits;
} Tun_Md1_Cfg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t INT2_IRON :1;		//Routing of soft-iron/hard-iron algorithm end event on INT2
		uint8_t INT2_TILT :1;		//Routing of tilt event on INT2
		uint8_t INT2_6D :1;			//Routing of 6D event on INT2
		uint8_t INT2_DOUBLE_TAP :1;	//Routing of tap event on INT2
		uint8_t INT2_FF :1;			//Routing of free-fall event on INT2
		uint8_t INT2_WU :1;			//Routing of wakeup event on INT2
		uint8_t INT2_SINGLE_TAP :1;	//Single-tap recognition routing on INT2
		uint8_t INT2_INACT_STATE :1;	//Routing on INT2 of inactivity mode
	} ISM330DLC_RegBits;
} Tun_Md2_Cfg;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Reserved :6;
		uint8_t LVL2_OIS :1;
		uint8_t INT2_DRDY_OIS :1;
	} ISM330DLC_RegBits;
} Tun_Int_Ois;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t OIS_EN_SPI2 :1;	//Enables OIS chain data processing for gyro in Mode 3 and Mode 4 (mode4_en = 1) and accelerometer data	in and Mode 4 (mode4_en = 1).
		uint8_t FS_125_OIS :1;	//Selects gyroscope OIS chain full scale ±125 dps
		uint8_t FS0_G_OIS :1;	//Gyroscope OIS chain full-scale selection
		uint8_t FS1_G_OIS :1;	//
		uint8_t MODE4_EN :1;	//Enables accelerometer OIS chain if OIS_EN_SPI2 = 1
		uint8_t SIM_OIS :1;		//SPI2 3- or 4-wire mode
		uint8_t LVL1_OIS :1;	//Enables level-sensitive trigger mode on OIS chain
		uint8_t BLE_OIS :1; 	//Big/Little Endian data selection
	} ISM330DLC_RegBits;
} Tun_Ctrl1_Ois;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t HP_EN_OIS :1;	//Enables gyroscope's OIS chain HPF
		uint8_t FTYPE_0_OIS :1;	//Gyroscope's digital LPF1 filter bandwidth selection
		uint8_t FTYPE_1_OIS :1;
		uint8_t Reserved :1;
		uint8_t HPM0_OIS :1;	//Gyroscope's OIS chain digital high-pass filter cutoff selection
		uint8_t HPM1_OIS :1;	//
		uint8_t Reserved1 :2;
	} ISM330DLC_RegBits;
} Tun_Ctrl2_Ois;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t ST_OIS_CLAMPDIS :1;			//Gyro OIS chain clamp disable
		uint8_t ST0_OIS :1;					//Gyroscope OIS chain self-test selection
		uint8_t ST1_OIS :1;					//
		uint8_t FILTER_XL_CONF_OIS_0 :1;	//Accelerometer OIS channel bandwidth selection
		uint8_t FILTER_XL_CONF_OIS_1 :1;
		uint8_t FS0_XL_OIS :1;				//
		uint8_t FS1_XL_OIS :1;				//Accelerometer OIS channel full-scale selection
		uint8_t DEN_LH_OIS :1; 				//Polarity of DEN signal on OIS chain
	} ISM330DLC_RegBits;
} Tun_Ctrl3_Ois;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t rw_0 :1;			//Read/write operation on Sensor1
		uint8_t Slave0_add0 :1; 	//I²C slave address of Sensor1 that can be read by sensor hub
		uint8_t Slave0_add1 :1;
		uint8_t Slave0_add2 :1;
		uint8_t Slave0_add3 :1;
		uint8_t Slave0_add4 :1;
		uint8_t Slave0_add5 :1;
		uint8_t Slave0_add6 :1;
	} ISM330DLC_RegBits;
} Tun_Slv0_Add;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Slave0_numop0 :1;		//Number of read operations on Sensor1.
		uint8_t Slave0_numop1 :1;
		uint8_t Slave0_numop2 :1;
		uint8_t Src_mode :1;			//Source mode conditioned read
		uint8_t Aux_sens_on0 :1;		//Number of external sensors to be read by sensor hub
		uint8_t Aux_sens_on1 :1;
		uint8_t Slave0_rate0 :1;		//Decimation of read operation on Sensor1 starting from the sensor hub trigger
		uint8_t Slave0_rate1 :1;
	} ISM330DLC_RegBits;
} Tun_Slave0_Config;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t r_1 :1;					//Read operation on Sensor2 enable
		uint8_t Slave1_add0 :1;			//I²C slave address of Sensor2 that can be read by sensor hub
		uint8_t Slave1_add1 :1;
		uint8_t Slave1_add2 :1;
		uint8_t Slave1_add3 :1;
		uint8_t Slave1_add4 :1;
		uint8_t Slave1_add5 :1;
		uint8_t Slave1_add6 :1;
	} ISM330DLC_RegBits;
} Tun_Slv1_Add;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Slave1_numop0 :1;
		uint8_t Slave1_numop1 :1;
		uint8_t Slave1_numop2 :1;
		uint8_t Reserved :2;
		uint8_t write_once :1;
		uint8_t Slave1_rate0 :1;
		uint8_t Slave1_rate1 :1;
	} ISM330DLC_RegBits;
} Tun_Slave1_Config;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t r_2 :1;					//Read operation on Sensor2 enable
		uint8_t Slave2_add0 :1;			//I²C slave address of Sensor2 that can be read by sensor hub
		uint8_t Slave2_add1 :1;
		uint8_t Slave2_add2 :1;
		uint8_t Slave2_add3 :1;
		uint8_t Slave2_add4 :1;
		uint8_t Slave2_add5 :1;
		uint8_t Slave2_add6 :1;
	} ISM330DLC_RegBits;
} Tun_Slv2_Add;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Slave2_numop0 :1;
		uint8_t Slave2_numop1 :1;
		uint8_t Slave2_numop2 :1;
		uint8_t Reserved :3;
		uint8_t Slave2_rate0 :1;
		uint8_t Slave2_rate1 :1;
	} ISM330DLC_RegBits;
} Tun_Slave2_Config;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t r_3 :1;					//Read operation on Sensor2 enable
		uint8_t Slave3_add0 :1;			//I²C slave address of Sensor2 that can be read by sensor hub
		uint8_t Slave3_add1 :1;
		uint8_t Slave3_add2 :1;
		uint8_t Slave3_add3 :1;
		uint8_t Slave3_add4 :1;
		uint8_t Slave3_add5 :1;
		uint8_t Slave3_add6 :1;
	} ISM330DLC_RegBits;
} Tun_Slv3_Add;

typedef volatile union
{
	uint8_t All;
	struct
	{
		uint8_t Slave3_numop0 :1;
		uint8_t Slave3_numop1 :1;
		uint8_t Slave3_numop2 :1;
		uint8_t Reserved :3;
		uint8_t Slave3_rate0 :1;
		uint8_t Slave3_rate1 :1;
	} ISM330DLC_RegBits;
} Tun_Slave3_Config;

#define ISM330DLC_MAX_BUFFER_REG 103

typedef union
{
	struct
	{
		uint8_t Reserved1;                                                //RESERVED
		Tun_Runc_Cfg_Access FUNC_CFG_ACCESS;                              //FUNC_CFG_ACCESS
		uint8_t Reserved2;                                                //RESERVED
		uint8_t Reserved3;                                                //RESERVED
		Tun_Sen_Tim_Fram SENSOR_SYNC_TIME_FRAME;                          //ENSOR_SYNC_TIME_FRAME
		Tun_Sen_Sync_Res_Ratio SENSOR_SYNC_RES_RATIO;                     //SENSOR_SYNC_RES_RATIO
		Tun_Fifo_Ctrl1 FIFO_CTRL1;                                        //FIFO_CTRL1
		Tun_Fifo_Ctrl2 FIFO_CTRL2;                                        //FIFO_CTRL2
		Tun_Fifo_Ctrl3 FIFO_CTRL3;                                        //FIFO_CTRL3
		Tun_Fifo_Ctrl4 FIFO_CTRL4;                                        //FIFO_CTRL4
		Tun_Fifo_Ctrl5 FIFO_CTRL5;                                        //FIFO_CTRL5
		Tun_Drdy_Pulse_Cfg DRDY_PULSE_CFG;                                //DRDY_PULSE_CFG
		uint8_t Reserved4;                                                //RESERVED
		Tun_Int1_Ctrl INT1_CTRL;                                          //INT1_CTRL
		Tun_Int2_Ctrl INT2_CTRL;                                          //INT2_CTRL
		uint8_t WHO_AM_I;                                                 //WHO_AM_I
		Tun_Ctrl1_Xl CTRL1_XL;                                            //CTRL1_XL
		Tun_Ctrl2_G CTRL2_G;                                              //CTRL2_G
		Tun_Ctrl3_C CTRL3_C;                                              //CTRL3_C
		Tun_Ctrl4_C CTRL4_C;                                              //CTRL4_C
		Tun_Ctrl5_C CTRL5_C;                                              //CTRL5_C
		Tun_Ctrl6_C CTRL6_C;                                              //CTRL6_C
		Tun_Ctrl7_G CTRL7_G;                                              //CTRL7_G
		Tun_Ctrl8_Xl CTRL8_XL;                                            //CTRL8_XL
		Tun_Ctrl9_Xl CTRL9_XL;                                            //CTRL9_XL
		Tun_Ctrl10_C CTRL10_C;                                            //CTRL10_C
		Tun_Master_Config MASTER_CONFIG;                                  //MASTER_CONFIG
		Tun_Wake_Up_Src WAKE_UP_SRC;                                      //WAKE_UP_SRC
		Tun_Tap_Src TAP_SRC;                                              //TAP_SRC
		Tun_D6d_Src D6D_SRC;                                              //D6D_SRC
		Tun_Status_Reg STATUS_REG;                                        //STATUS_REG/(1)(2)
		uint8_t Reserved44;                                               //RESERVED
		Tun_Out_Temp_L OUT_TEMP_L;                                        //OUT_TEMP_L
		Tun_Out_Temp_H OUT_TEMP_H;                                        //OUT_TEMP_H
		uint8_t OUTX_L_G;                                                 //OUTX_L_G
		uint8_t OUTX_H_G;                                                 //OUTX_H_G
		uint8_t OUTY_L_G;                                                 //OUTY_L_G
		uint8_t OUTY_H_G;                                                 //OUTY_H_G
		uint8_t OUTZ_L_G;                                                 //OUTZ_L_G
		uint8_t OUTZ_H_G;                                                 //OUTZ_H_G
		uint8_t OUTX_L_XL;                                                //OUTX_L_XL
		uint8_t OUTX_H_XL;                                                //OUTX_H_XL
		uint8_t OUTY_L_XL;                                                //OUTY_L_XL
		uint8_t OUTY_H_XL;                                                //OUTY_H_XL
		uint8_t OUTZ_L_XL;                                                //OUTZ_L_XL
		uint8_t OUTZ_H_XL;                                                //OUTZ_H_XL
		uint8_t SENSORHUB1_REG;                                           //SENSORHUB1_REG
		uint8_t SENSORHUB2_REG;                                           //SENSORHUB2_REG
		uint8_t SENSORHUB3_REG;                                           //SENSORHUB3_REG
		uint8_t SENSORHUB4_REG;                                           //SENSORHUB4_REG
		uint8_t SENSORHUB5_REG;                                           //SENSORHUB5_REG
		uint8_t SENSORHUB6_REG;                                           //SENSORHUB6_REG
		uint8_t SENSORHUB7_REG;                                           //SENSORHUB7_REG
		uint8_t SENSORHUB8_REG;                                           //SENSORHUB8_REG
		uint8_t SENSORHUB9_REG;                                           //SENSORHUB9_REG
		uint8_t SENSORHUB10_REG;                                          //SENSORHUB10_REG
		uint8_t SENSORHUB11_REG;                                          //SENSORHUB11_REG
		uint8_t SENSORHUB12_REG;                                          //SENSORHUB12_REG
		Tun_Fifo_Status1 FIFO_STATUS1;                                    //FIFO_STATUS1
		Tun_Fifo_Status2 FIFO_STATUS2;                                    //FIFO_STATUS2
		Tun_Fifo_Status3 FIFO_STATUS3;                                    //FIFO_STATUS3
		Tun_Fifo_Status4 FIFO_STATUS4;                                    //FIFO_STATUS4
		uint8_t FIFO_DATA_OUT_L;                                          //FIFO_DATA_OUT_L
		uint8_t FIFO_DATA_OUT_H;                                          //FIFO_DATA_OUT_H
		uint8_t TIMESTAMP0_REG;                                           //TIMESTAMP0_REG
		uint8_t TIMESTAMP1_REG;                                           //TIMESTAMP1_REG
		uint8_t TIMESTAMP2_REG;                                           //TIMESTAMP2_REG
		uint8_t Reserved5;                                                //RESERVED
		uint8_t SENSORHUB13_REG;                                          //SENSORHUB13_RE
		uint8_t SENSORHUB14_REG;                                          //SENSORHUB14_RE
		uint8_t SENSORHUB15_REG;                                          //SENSORHUB15_RE
		uint8_t SENSORHUB16_REG;                                          //SENSORHUB16_RE
		uint8_t SENSORHUB17_REG;                                          //SENSORHUB17_RE
		uint8_t SENSORHUB18_REG;                                          //SENSORHUB18_RE
		Tun_Func_Src1 FUNC_SRC1;                                          //FUNC_SRC1
		Tun_Func_Src2 FUNC_SRC2;                                          //FUNC_SRC2
		uint8_t Reserved6;                                                //RESERVED
		Tun_Tap_Cfg TAP_CFG;                                              //TAP_CFG
		Tun_Tap_Ths_6D TAP_THS_6D;                                        //TAP_THS_6D
		Tun_Int_Dur2 INT_DUR2;                                            //INT_DUR2
		Tun_Wake_Up_Ths WAKE_UP_THS;                                      //WAKE_UP_THS
		Tun_Wake_Up_Dur WAKE_UP_DUR;                                      //WAKE_UP_DUR
		Tun_Free_Fall FREE_FALL;                                          //FREE_FALL
		Tun_Md1_Cfg MD1_CFG;                                              //MD1_CFG
		Tun_Md2_Cfg MD2_CFG;                                              //MD2_CFG
		uint8_t MASTER_CMD_CODE;                                          //MASTER_CMD_COD
		uint8_t SENS_SYNC_SPI_ERROR_CODE;                                 //SENS_SYNC_SPI_ERROR_CODE
		uint8_t Reserved7;                                                //RESERVED
		uint8_t OUT_MAG_RAW_X_L;                                          //OUT_MAG_RAW_X_L
		uint8_t OUT_MAG_RAW_X_H;                                          //OUT_MAG_RAW_X_H
		uint8_t OUT_MAG_RAW_Y_L;                                          //OUT_MAG_RAW_Y_L
		uint8_t OUT_MAG_RAW_Y_H;                                          //OUT_MAG_RAW_Y_H
		uint8_t OUT_MAG_RAW_Z_L;                                          //OUT_MAG_RAW_Z_L
		uint8_t OUT_MAG_RAW_Z_H;                                          //OUT_MAG_RAW_Z_H
		uint8_t Reserved8;                                                //RESERVED
		Tun_Int_Ois INT_OIS;                                              //INT_OIS
		Tun_Ctrl1_Ois CTRL1_OIS;                                          //CTRL1_OIS
		Tun_Ctrl2_Ois CTRL2_OIS;                                          //CTRL2_OIS
		Tun_Ctrl3_Ois CTRL3_OIS;                                          //CTRL3_OIS
		uint8_t X_OFS_USR;                                                //X_OFS_USR
		uint8_t Y_OFS_USR;                                                //Y_OFS_USR
		uint8_t Z_OFS_USR;                                                //Z_OFS_USR
		uint8_t Reserved9;                                                //RESERVED
	} ISM330DLC_Reg_Struct;
	uint8_t ISM330DLC_Reg_Tab[ISM330DLC_MAX_BUFFER_REG];
} ISM330DLC_iDriver_Description;

//SENSOR_SYNC_TIME_FRAME register
#define SENSOR_SYNC_TIME_FRAME_DISABLE	0U
#define SENSOR_SYNC_TIME_FRAME_500 	0x00U			  //500ms
#define SENSOR_SYNC_TIME_FRAME_1000 0x01U             //1000   ms
#define SENSOR_SYNC_TIME_FRAME_1500 0x02U             //1500   ms
#define SENSOR_SYNC_TIME_FRAME_2000 0x03U             //2000   ms
#define SENSOR_SYNC_TIME_FRAME_2500 0x04U             //2500   ms
#define SENSOR_SYNC_TIME_FRAME_3000 0x05U             //3000   ms
#define SENSOR_SYNC_TIME_FRAME_3500 0x06U             //3500   ms
#define SENSOR_SYNC_TIME_FRAME_4000 0x07U             //4000   ms
#define SENSOR_SYNC_TIME_FRAME_4500 0x08U             //4500   ms
#define SENSOR_SYNC_TIME_FRAME_5000 0x08U             //5000   ms

//SENSOR_SYNC_RES_RATIO register
#define RESOLUTION_RATIO_1 0x00U
#define RESOLUTION_RATIO_2 0x01U
#define RESOLUTION_RATIO_3 0x02U
#define RESOLUTION_RATIO_4 0x03U

//Ctrl1 and ctrl2
#define FIFO_TIMER_ENABLE 0x80U
#define FIFO_TEMP_ENABLE  0x04U
#define FIFO_THS_LEVEL0	  0x01U		//2bytes

//FIFO_CTRL3 register
#define DEC_FIFO_GYRO_1 0x00U
#define DEC_FIFO_GYRO_2 0x01U
#define DEC_FIFO_GYRO_3 0x02U
#define DEC_FIFO_GYRO_4 0x03U
#define DEC_FIFO_GYRO_5 0x04U
#define DEC_FIFO_GYRO_6 0x05U
#define DEC_FIFO_GYRO_7 0x06U
#define DEC_FIFO_GYRO_8 0x07U

#define DEC_FIFO_XL_1  0x00U
#define DEC_FIFO_XL_2  0x01U
#define DEC_FIFO_XL_3  0x02U
#define DEC_FIFO_XL_4  0x03U
#define DEC_FIFO_XL_5  0x04U
#define DEC_FIFO_XL_6  0x05U
#define DEC_FIFO_XL_7  0x06U
#define DEC_FIFO_XL_8  0x07U

//FIFO_CTRL4 registe
#define ENABLE_FIFO_THS_USE 0x80
#define ONLY_HIGH_DATA 0x40
#define DEC_DS4_FIFO_1 0x00U
#define DEC_DS4_FIFO_2 0x01U
#define DEC_DS4_FIFO_3 0x02U
#define DEC_DS4_FIFO_4 0x03U
#define DEC_DS4_FIFO_5 0x04U
#define DEC_DS4_FIFO_6 0x05U
#define DEC_DS4_FIFO_7 0x06U
#define DEC_DS4_FIFO_8 0x07U

#define DEC_DS3_FIFO_1 0x00U
#define DEC_DS3_FIFO_2 0x01U
#define DEC_DS3_FIFO_3 0x02U
#define DEC_DS3_FIFO_4 0x03U
#define DEC_DS3_FIFO_5 0x04U
#define DEC_DS3_FIFO_6 0x05U
#define DEC_DS3_FIFO_7 0x06U
#define DEC_DS3_FIFO_8 0x07U
//FIFO_CTRL5 register


#define ODR_FIFO_1 0x00U
#define ODR_FIFO_2 0x01U
#define ODR_FIFO_3 0x02U
#define ODR_FIFO_4 0x03U
#define ODR_FIFO_5 0x04U
#define ODR_FIFO_6 0x05U
#define ODR_FIFO_7 0x06U
#define ODR_FIFO_8 0x07U
#define ODR_FIFO_9 0x08U
#define ODR_FIFO_10 0x09U
#define ODR_FIFO_11 0x10U


//DRDY_PULSE_CFG register
#define ENABLE_DRDY_PULSED 0x80U


//INT1_CTRL register
//using number |= 1 << x;
//using number &= ~(1 << x);
#define ENABLE_INT1_FULL_FLAG 0x00U
#define ENBALE_INT1_FIFO_OVR  0x01U
#define ENBALE_INT1_FTH		  0x02U
#define ENBALE_INT1_BOOT	  0x03U
#define ENBALE_INT1_DRDY_G	  0x04U
#define ENBALE_INT1_DRDY_XL	  0x06U

//INT2_CTRL register
#define ENABLE_INT2_FULL_FLAG  0x00U
#define ENABLE_INT2_FIFO_OVR   0x01U
#define ENABLE_INT2_FTH        0x02U
#define ENABLE_INT2_DRDY_TEMP  0x03U
#define ENABLE_INT2_DRDY_G     0x04U
#define ENABLE_INT2_DRDY_XL    0x06U


//CTRL1_XL register

#define CTRL_XL_Power_down    0x00U
#define CTRL_XL_1_6Hz         0xB0U
#define CTRL_XL_12_5_Hz       0x10U
#define CTRL_XL_26_Hz         0x20U
#define CTRL_XL_52_Hz         0x30U
#define CTRL_XL_104_Hz        0x40U
#define CTRL_XL_208_Hz        0x50U
#define CTRL_XL_416_Hz        0x60U
#define CTRL_XL_833_Hz        0x70U
#define CTRL_XL_1_66kHz       0x80U
#define CTRL_XL_3_33_kHz      0x90U
#define CTRL_XL_6_66_kHz      0xAOU

#define CTRL_XL_ACCE_FULL_SACAL_1 0x00U	//+-2g
#define CTRL_XL_ACCE_FULL_SACAL_2 0x04U	//+-16g
#define CTRL_XL_ACCE_FULL_SACAL_3 0x08U	//+-4g
#define CTRL_XL_ACCE_FULL_SACAL_4 0x0CU	//+-8g

#define CTRL_XL_LPF1_BW_SEL 0x01U 		//BW selection


//CTRL2_G register
#define ISM330DL_XL_ODR_OFF 	0x00U
#define ISM330DL_XL_ODR_12Hz5 	0x10U
#define ISM330DL_XL_ODR_26Hz    0x20U
#define ISM330DL_XL_ODR_52Hz    0x30U
#define ISM330DL_XL_ODR_104Hz   0x40U
#define ISM330DL_XL_ODR_208Hz   0x50U
#define ISM330DL_XL_ODR_416Hz   0x60U
#define ISM330DL_XL_ODR_833Hz   0x70U
#define ISM330DL_XL_ODR_1k66Hz  0x80U
#define ISM330DL_XL_ODR_3k33Hz  0x90U
#define ISM330DL_XL_ODR_6k66Hz  0xA0U
#define ISM330DL_XL_ODR_1Hz6    0xB0U

#define GYROSCOPE_FULL_SCALE_0 0x00U		//250dps
#define GYROSCOPE_FULL_SCALE_1 0x04U		//500dps
#define GYROSCOPE_FULL_SCALE_2 0x08U		//1000dps
#define GYROSCOPE_FULL_SCALE_3 0x0CU		//2000dps

#define GYROSCOPE_FULL_SCALE_AT_125_BIT 1

//CTRL3_C register
#define CTRL3_C_BOOT   		0x07U
#define CTRL3_C_BDU			0x06U
#define CTRL3_C_H_LACTIVE	0x05U
#define CTRL3_C_PP_OD		0x04U
#define CTRL3_C_SIM			0x03U
#define CTRL3_C_IF_INC		0x02U
#define CTRL3_C_BLE			0x01U
#define CTRL3_C_SW_RESET	0x00U


//CTRL4_C register
#define CTRL4_C_DEN_XL_EN     0x07U
#define CTRL4_C_SLEEP         0x06U
#define CTRL4_C_INT2_on_INT1  0x05U
#define CTRL4_C_DEN_DRDY_INT1 0x04U
#define CTRL4_C_DRDY_MASK     0x03U
#define CTRL4_C_I2C_disable   0x02U
#define CTRL4_C_LPF1_SEL_G    0x01U

//CTRL5_C register description
#define CTRL5_ROUNDING_1      0x00U
#define CTRL5_ROUNDING_2      0x20U
#define CTRL5_ROUNDING_3      0x40U
#define CTRL5_ROUNDING_4      0x60U
#define CTRL5_ROUNDING_5      0x80U
#define CTRL5_ROUNDING_6      0xA0U
#define CTRL5_ROUNDING_7      0xB0U
#define CTRL5_ROUNDING_8      0xE0U


#define CTRL5_DEN_LH_ACTIV_LOW_HIGH_BIT 4
#define CTRL5_ANGULAR_SELF_TEST_MODE_1 0x00U
#define CTRL5_ANGULAR_SELF_TEST_MODE_2 0x04U
#define CTRL5_ANGULAR_SELF_TEST_MODE_3 0x08U
#define CTRL5_ANGULAR_SELF_TEST_MODE_4 0x0CU

#define CTRL5_LINEAR_SELF_TEST_MODE_1 0x00U
#define CTRL5_LINEAR_SELF_TEST_MODE_2 0x01U
#define CTRL5_LINEAR_SELF_TEST_MODE_3 0x02U
#define CTRL5_LINEAR_SELF_TEST_MODE_4 0x03U


//CTRL6_C register description
#define CTRL6_GYRO_LPF1_SELECTION_1 0x00U
#define CTRL6_GYRO_LPF1_SELECTION_2 0x01U
#define CTRL6_GYRO_LPF1_SELECTION_3 0x02U
#define CTRL6_GYRO_LPF1_SELECTION_4 0x03U

#define CTRL6_USR_OFF_W_BIT 3
#define CTRL6_XL_HM_MODE_BIT 4

#define CTRL6_TRIG_MODE_SELECTION_1 0x80U
#define CTRL6_TRIG_MODE_SELECTION_2 0x40U
#define CTRL6_TRIG_MODE_SELECTION_3 0x60U
#define CTRL6_TRIG_MODE_SELECTION_4 0xC0U

//CTRL7_G register description
#define CTRL7_G_ROUNDING_STATUS_BIT 2

#define CTRL6_G_HIGH_PASSE_FILTER_CUOFF_1	0x00U
#define CTRL6_G_HIGH_PASSE_FILTER_CUOFF_2	0x10U
#define CTRL6_G_HIGH_PASSE_FILTER_CUOFF_3	0x20U
#define CTRL6_G_HIGH_PASSE_FILTER_CUOFF_4	0x30U

#define CTRL6_G_GYRO_HIGH_PASS_ENABLE_BIT 6
#define CTRL6_G_High_PERFO_OPER_BIT 7

//CTRL8_XL register description
#define CTRL8_LOW_PASS_ON_6D_BIT 1
#define CTRL8_XL_ACC_LPF1_ENABLE_BIT 7
#define CTRL8_HP_SLOPE_XL_ENABLE_BIT 2
#define CTRL8_INPUT_COMPOSITE_BIT 3
#define CTRL8_HP_REF_MODE_BIT 4

#define CTRL8_HPCF_XL_1 0x20
#define CTRL8_HPCF_XL_2 0x40
#define CTRL8_HPCF_XL_3 0x60
#define CTRL8_HPCF_XL_4 0x80

//CTRL9_XL register description
#define CTRL9_DEN_X_BIT	7
#define CTRL9_DEN_Y_BIT 6
#define CTRL9_DEN_Z_BIT 5
#define CTRL9_DEN_XL_G_BIT 4
#define CTRL9_SOFT_EN_BIT 2

//CTRL10_C register description
#define CTRL10_C_FUNC_EN_BIT 2
#define CTRL10_C_TILT_EN_BIT 3
#define CTRL10_C_TIMER_EN_BIT 5

//MASTER_CONFIG register description
#define MASTER_CONFIG_DRDY_ON_INT1_BIT 			7
#define MASTER_CONFIG_DATA_VALID_SEL_FIFO_BIT 	6
#define MASTER_CONFIG_START_CONFIG_BIT 			4
#define MASTER_CONFIG_PULL_UP_EN_BIT 			3
#define MASTER_CONFIG_PASS_THROUGH_MODE_BIT 	2
#define MASTER_CONFIG_IRON_EN_BIT 				1
#define MASTER_CONFIG_MASTER_ON_BIT 			0

//TAP_CFG register description
#define TAP_CFG_INTERRUPTS_ENABLE_BIT 7

#define TAP_CFG_INACT_EN_1 0x00
#define TAP_CFG_INACT_EN_2 0x20
#define TAP_CFG_INACT_EN_3 0x40
#define TAP_CFG_INACT_EN_4 0x60

#define TAP_CFG_SLOPE_FDS_BIT	4
#define TAP_CFG_TAP_X_EN_BIT	3
#define TAP_CFG_TAP_Y_EN_BIT	2
#define TAP_CFG_TAP_Z_EN_BIT	1
#define TAP_CFG_LIR_BIT			0

//TAP_THS_6D register description
#define TAP_THS_6D_D4D_EN_BIT 7
#define TAP_THS_6D_SIXD_THS_1	0x00
#define TAP_THS_6D_SIXD_THS_2	0x20
#define TAP_THS_6D_SIXD_THS_3	0x40
#define TAP_THS_6D_SIXD_THS_4	0x60

//INT_DUR2 register description
#define INT_DUR2_SHOCK_1 	0x00
#define INT_DUR2_SHOCK_2 	0x01
#define INT_DUR2_SHOCK_3 	0x02
#define INT_DUR2_SHOCK_4 	0x03

#define INT_DUR2_QUIET_0 0x00
#define INT_DUR2_QUIET_1 0x04
#define INT_DUR2_QUIET_2 0x08
#define INT_DUR2_QUIET_3 0x0C

#define INT_DUR2_1       0x00
#define INT_DUR2_2       0x00
#define INT_DUR2_3       0x00
#define INT_DUR2_4       0x00
#define INT_DUR2_5       0x00
#define INT_DUR2_6       0x00

//WAKE_UP_THS register description
#define WAKE_UP_THS_SINGLE_DOUBLE_TAP 7

#define WAKE_UP_THS_WK_THS_0   0x00
#define WAKE_UP_THS_WK_THS_1   0x00
#define WAKE_UP_THS_WK_THS_2   0x00
#define WAKE_UP_THS_WK_THS_3   0x00
#define WAKE_UP_THS_WK_THS_4   0x00
//...

//WAKE_UP_DUR register description
#define WAKE_UP_DUR_FF_DUR5_BIT 7
#define WAKE_UP_DUR_WAKE_DUR_1 0x00
#define WAKE_UP_DUR_WAKE_DUR_2 0x20
#define WAKE_UP_DUR_WAKE_DUR_3 0x40
#define WAKE_UP_DUR_WAKE_DUR_4 0x60

#define WAKE_UP_DUR_TIMER_HR_0 0x00
#define WAKE_UP_DUR_TIMER_HR_1 0x00
//....


//FREE_FALL register description
#define FREE_FALL_FF_DUR_1 0x00
#define FREE_FALL_FF_DUR_2 0x00
#define FREE_FALL_FF_DUR_3 0x00
#define FREE_FALL_FF_DUR_4 0x00

#define FREE_FALL_FF_DUR_1_156_MG	0
#define FREE_FALL_FF_DUR_2_219_MG	1
#define FREE_FALL_FF_DUR_3_250_MG	2
#define FREE_FALL_FF_DUR_4_312_MG	3
#define FREE_FALL_FF_DUR_5_344_MG	4
#define FREE_FALL_FF_DUR_6_406_MG	5
#define FREE_FALL_FF_DUR_7_469_MG	6
#define FREE_FALL_FF_DUR_8_500_MG	7

//MD1_CFG register description
#define MD1_CFG_INT1_INACT_STATE_BIT  7
#define MD1_CFG_INT1_SINGLE_TAP_BIT   6
#define MD1_CFG_INT1_WU_BIT           5
#define MD1_CFG_INT1_FF_BIT           4
#define MD1_CFG_INT1_DOUBLE_TAP_BIT   3
#define MD1_CFG_INT1_6D_BIT           2
#define MD1_CFG_INT1_TILT_BIT         1
#define MD1_CFG_INT1_TIMER_BIT        0

//MD2_CFG register description
#define MD2_CFG_INT2_INACT_STATE_BIT   7
#define MD2_CFG_INT2_SINGLE_TAP_BIT    6
#define MD2_CFG_INT2_WU_BIT            5
#define MD2_CFG_INT2_FF_BIT            4
#define MD2_CFG_INT2_DOUBLE_TAP_BIT    3
#define MD2_CFG_INT2_6D_BIT            2
#define MD2_CFG_INT2_TILT_BIT          1
#define MD2_CFG_INT2_IRON_BIT          0

//INT_OIS register description
#define INT2_DRDY_OIS_BIT			  	7
#define LVL2_OIS_BIT					6

//CTRL1_OIS register description
#define CTRL1_OIS_LVL1_OIS_BIT	 7
#define CTRL1_OIS_BLE_OIS_BIT	 6
#define CTRL1_OIS_SIM_OIS_BIT	 5
#define CTRL1_OIS_MODE4_EN_BIT	 4
#define CTRL1_OIS_FS_1			 0x00
#define CTRL1_OIS_FS_2			 0x04
#define CTRL1_OIS_FS_3			 0x08
#define CTRL1_OIS_FS_4			 0x0C
#define CTRL1_OIS_FS_125_OIS_BIT  1
#define CTRL1_OIS_OIS_EN_SPI2_BIT 0

//CTRL2_OIS register description


#endif /* ISM330DLC_INT_H_ */
