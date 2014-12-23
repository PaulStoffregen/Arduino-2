/** \mainpage
*
****************************************************************************
* Copyright (C) 2013 - 2014 Bosch Sensortec GmbH
*
* File : bno055.h
*
* Date : 2014/09/10
*
* Revision : 2.0 $
*
* Usage: Sensor Driver file for BNO055 sensor
*
****************************************************************************
* \section License
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
* OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
* OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility
* for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
**************************************************************************/
/*! \file bno055.h
    \brief BNO055 Sensor Driver Support Header File */

#ifndef __BNO055_H__
#define __BNO055_H__
/****************************************************************/
/**\name	DATA TYPES INCLUDES		*/
/************************************************************/
/*!
*	These definition uses for define the data types
*/
/*!
*	While porting the API please consider the following
*	Please check the version of C standard
*	Are you using Linux platform
*/

/*!
*	This definition uses for the Linux platform support
*	Please use the types.h for your data types definitions
*/
#ifdef	__KERNEL__

#include <linux/types.h>

#else /* ! __KERNEL__ */
/*!
*	These definition uses for define the C
*	standard version data types
*/
# if !defined(__STDC_VERSION__)

/*!
*	compiler is C11 C standard
*/
#if (__STDC_VERSION__ == 201112L)

/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	_u8	uint8_t
#define	_u16	uint16_t
#define	_u32	uint32_t
#define	_u64	uint64_t

/*signed integer types*/
#define	_s8	int8_t
#define	_s16	int16_t
#define	_s32	int32_t
#define	_s64	int64_t
/*!
 *	compiler is C99 C standard
*/

#elif (__STDC_VERSION__ == 199901L)

/*!
*	stdint.h is a C99 supported c library.
*	which is used to fixed the integer size
*/
/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	_u8	uint8_t
#define	_u16	uint16_t
#define	_u32	uint32_t
#define	_u64	uint64_t

/*signed integer types*/
#define _s8	int8_t
#define	_s16	int16_t
#define	_s32	int32_t
#define	_s64	int64_t
/*!
*	compiler is C89 or other C standard
*/
#else /*  !defined(__STDC_VERSION__) */
/*!	By default it is defined as 32 bit machine configuration
*	define the definition based on your machine configuration
*	define the data types based on your
*	machine/compiler/controller configuration
*/
//#define  MACHINE_32_BIT
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
#define	_s8	signed char
#define	_s16	signed short int
#define	_s32	signed long int

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define _s64 long int
#define _u64 unsigned long int
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define _s64 long long int
#define _u64 unsigned long long int
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set _s64 manually.
#endif

/*unsigned integer types*/
#define	_u8	unsigned char
#define	_u16	unsigned short int
#define	_u32	unsigned long int

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
#define	_s8	signed char
#define	_s16	signed short int
#define	_s32	signed int
#define	_s64	signed long long int

/*unsigned integer types*/
#define	_u8	unsigned char
#define	_u16	unsigned short int
#define	_u32	unsigned int
#define	_u64	unsigned long long int

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined MACHINE_64_BIT
/*signed integer types*/
#define	_s8	signed char
#define	_s16	signed short int
#define	_s32	signed int
#define	_s64	signed long int

/*unsigned integer types*/
#define	_u8	unsigned char
#define	_u16	unsigned short int
#define	_u32	unsigned int
#define	_u64	unsigned long int

#else
/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	_u8	uint8_t
#define	_u16	uint16_t
#define	_u32	uint32_t
#define	_u64	uint64_t

/*signed integer types*/
#define _s8	int8_t
#define	_s16	int16_t
#define	_s32	int32_t
#define	_s64	int64_t
#endif
#endif

/*** This else will execute for the compilers
 *	which are not supported the C standards
 *	Like C89/C99/C11***/
#else
/*!
*	By default it is defined as 32 bit machine configuration*
*	define the definition based on your machine configuration*
*	define the data types based on your
*	machine/compiler/controller configuration
*/
//#define  MACHINE_32_BIT

/* If your machine support 16 bit
define the MACHINE_16_BIT*/
#ifdef MACHINE_16_BIT
#include <limits.h>
/*signed integer types*/
#define	_s8	signed char
#define	_s16	signed short int
#define	_s32	signed long int

#if defined(LONG_MAX) && LONG_MAX == 0x7fffffffffffffffL
#define _s64 long int
#define _u64 unsigned long int
#elif defined(LLONG_MAX) && (LLONG_MAX == 0x7fffffffffffffffLL)
#define _s64 long long int
#define _u64 unsigned long long int
#else
#warning Either the correct data type for signed 64 bit integer \
could not be found, or 64 bit integers are not supported in your environment.
#warning If 64 bit integers are supported on your platform, \
please set _s64 manually.
#endif

/*unsigned integer types*/
#define	_u8	unsigned char
#define	_u16	unsigned short int
#define	_u32	unsigned long int

/* If your machine support 32 bit
define the MACHINE_32_BIT*/
#elif defined MACHINE_32_BIT
/*signed integer types*/
#define	_s8	signed char
#define	_s16	signed short int
#define	_s32	signed int
#define	_s64	signed long long int

/*unsigned integer types*/
#define	_u8	unsigned char
#define	_u16	unsigned short int
#define	_u32	unsigned int
#define	_u64	unsigned long long int

/* If your machine support 64 bit
define the MACHINE_64_BIT*/
#elif defined  MACHINE_64_BIT
/*signed integer types*/
#define	_s8	signed char
#define	_s16	signed short int
#define	_s32	signed int
#define	_s64	signed long int

/*unsigned integer types*/
#define	_u8	unsigned char
#define	_u16	unsigned short int
#define	_u32	unsigned int
#define	_u64	unsigned long int

#else

/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
#define	_u8	uint8_t
#define	_u16	uint16_t
#define	_u32	uint32_t
#define	_u64	uint64_t

/*signed integer types*/
#define _s8	int8_t
#define	_s16	int16_t
#define	_s32	int32_t
#define	_s64	int64_t
#endif
#endif
#endif
/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTIONS           */
/***************************************************************/
#define BNO055_WR_FUNC_PTR _s8 (*bus_write)\
(_u8, _u8 , _u8 *, _u8)

#define BNO055_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
	bus_write(dev_addr, reg_addr, reg_data, wr_len)

#define BNO055_RD_FUNC_PTR _s8 \
(*bus_read)(_u8, _u8 , _u8 *, _u8)

#define BNO055_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, r_len)\
bus_read(dev_addr, reg_addr, reg_data, r_len)

#define BNO055_DELAY_RETURN_TYPE void

#define BNO055_DELAY_PARAM_TYPES _u32

#define BNO055_DELAY_FUNC(delay_in_msec)\
	delay_func(delay_in_msec)

/********************************************************/
/**\name	I2C ADDRESS DEFINITION FOR BNO055           */
/********************************************************/
/* bno055 I2C Address */
#define BNO055_I2C_ADDR1                0x28
#define BNO055_I2C_ADDR2                0x29

/***************************************************/
/**\name	REGISTER ADDRESS DEFINITION  */
/***************************************************/
/* Page id register definition*/
#define BNO055_PAGE_ID_ADDR				    0X07

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                 0x00
#define BNO055_ACCEL_REV_ID_ADDR			0x01
#define BNO055_MAG_REV_ID_ADDR              0x02
#define BNO055_GYRO_REV_ID_ADDR             0x03
#define BNO055_SW_REV_ID_LSB_ADDR			0x04
#define BNO055_SW_REV_ID_MSB_ADDR			0x05
#define BNO055_BL_REV_ID_ADDR				0X06

/* Accel data register*/
#define BNO055_ACCEL_DATA_X_LSB_ADDR			0X08
#define BNO055_ACCEL_DATA_X_MSB_ADDR			0X09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR			0X0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR			0X0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR			0X0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR			0X0D

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR			0X0E
#define BNO055_MAG_DATA_X_MSB_ADDR			0X0F
#define BNO055_MAG_DATA_Y_LSB_ADDR			0X10
#define BNO055_MAG_DATA_Y_MSB_ADDR			0X11
#define BNO055_MAG_DATA_Z_LSB_ADDR			0X12
#define BNO055_MAG_DATA_Z_MSB_ADDR			0X13

/*Gyro data registers*/
#define BNO055_GYRO_DATA_X_LSB_ADDR			0X14
#define BNO055_GYRO_DATA_X_MSB_ADDR			0X15
#define BNO055_GYRO_DATA_Y_LSB_ADDR			0X16
#define BNO055_GYRO_DATA_Y_MSB_ADDR			0X17
#define BNO055_GYRO_DATA_Z_LSB_ADDR			0X18
#define BNO055_GYRO_DATA_Z_MSB_ADDR			0X19

/*Euler data registers*/
#define BNO055_EULER_H_LSB_ADDR			0X1A
#define BNO055_EULER_H_MSB_ADDR			0X1B

#define BNO055_EULER_R_LSB_ADDR			0X1C
#define BNO055_EULER_R_MSB_ADDR			0X1D

#define BNO055_EULER_P_LSB_ADDR			0X1E
#define BNO055_EULER_P_MSB_ADDR			0X1F

/*Quaternion data registers*/
#define BNO055_QUATERNION_DATA_W_LSB_ADDR	0X20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR	0X21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR	0X22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR	0X23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR	0X24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR	0X25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR	0X26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR	0X27

/* Linear acceleration data registers*/
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR			0X28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR			0X29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR			0X2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR			0X2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR			0X2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR			0X2D

/*Gravity data registers*/
#define BNO055_GRAVITY_DATA_X_LSB_ADDR			0X2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR			0X2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR			0X30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR			0X31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR			0X32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR			0X33

/* Temperature data register*/
#define BNO055_TEMP_ADDR					0X34

/* Status registers*/
#define BNO055_CALIB_STAT_ADDR				0X35
#define BNO055_SELFTEST_RESULT_ADDR			0X36
#define BNO055_INT_STAT_ADDR				0X37
#define BNO055_SYS_CLK_STAT_ADDR			0X38
#define BNO055_SYS_STAT_ADDR				0X39
#define BNO055_SYS_ERR_ADDR					0X3A

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR				0X3B
#define BNO055_DATA_SELECT_ADDR				0X3C

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR				0X3D
#define BNO055_PWR_MODE_ADDR				0X3E

#define BNO055_SYS_TRIGGER_ADDR				0X3F
#define BNO055_TEMP_SOURCE_ADDR				0X40
/* Axis remap registers*/
#define BNO055_AXIS_MAP_CONFIG_ADDR			0X41
#define BNO055_AXIS_MAP_SIGN_ADDR			0X42

/* SIC registers*/
#define BNO055_SIC_MATRIX_0_LSB_ADDR		0X43
#define BNO055_SIC_MATRIX_0_MSB_ADDR		0X44
#define BNO055_SIC_MATRIX_1_LSB_ADDR		0X45
#define BNO055_SIC_MATRIX_1_MSB_ADDR		0X46
#define BNO055_SIC_MATRIX_2_LSB_ADDR		0X47
#define BNO055_SIC_MATRIX_2_MSB_ADDR		0X48
#define BNO055_SIC_MATRIX_3_LSB_ADDR		0X49
#define BNO055_SIC_MATRIX_3_MSB_ADDR		0X4A
#define BNO055_SIC_MATRIX_4_LSB_ADDR		0X4B
#define BNO055_SIC_MATRIX_4_MSB_ADDR		0X4C
#define BNO055_SIC_MATRIX_5_LSB_ADDR		0X4D
#define BNO055_SIC_MATRIX_5_MSB_ADDR		0X4E
#define BNO055_SIC_MATRIX_6_LSB_ADDR		0X4F
#define BNO055_SIC_MATRIX_6_MSB_ADDR		0X50
#define BNO055_SIC_MATRIX_7_LSB_ADDR		0X51
#define BNO055_SIC_MATRIX_7_MSB_ADDR		0X52
#define BNO055_SIC_MATRIX_8_LSB_ADDR		0X53
#define BNO055_SIC_MATRIX_8_MSB_ADDR		0X54

/* Accelerometer Offset registers*/
#define ACCEL_OFFSET_X_LSB_ADDR				0X55
#define ACCEL_OFFSET_X_MSB_ADDR				0X56
#define ACCEL_OFFSET_Y_LSB_ADDR				0X57
#define ACCEL_OFFSET_Y_MSB_ADDR				0X58
#define ACCEL_OFFSET_Z_LSB_ADDR				0X59
#define ACCEL_OFFSET_Z_MSB_ADDR				0X5A

/* Magnetometer Offset registers*/
#define MAG_OFFSET_X_LSB_ADDR				0X5B
#define MAG_OFFSET_X_MSB_ADDR				0X5C
#define MAG_OFFSET_Y_LSB_ADDR				0X5D
#define MAG_OFFSET_Y_MSB_ADDR				0X5E
#define MAG_OFFSET_Z_LSB_ADDR				0X5F
#define MAG_OFFSET_Z_MSB_ADDR				0X60

/* Gyroscope Offset registers*/
#define GYRO_OFFSET_X_LSB_ADDR				0X61
#define GYRO_OFFSET_X_MSB_ADDR				0X62
#define GYRO_OFFSET_Y_LSB_ADDR				0X63
#define GYRO_OFFSET_Y_MSB_ADDR				0X64
#define GYRO_OFFSET_Z_LSB_ADDR				0X65
#define GYRO_OFFSET_Z_MSB_ADDR				0X66

/* Radius registers*/
#define	ACCEL_RADIUS_LSB_ADDR				0X67
#define	ACCEL_RADIUS_MSB_ADDR				0X68
#define	MAG_RADIUS_LSB_ADDR					0X69
#define	MAG_RADIUS_MSB_ADDR					0X6A
/* PAGE0 REGISTERS DEFINITION END*/

/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define ACCEL_CONFIG_ADDR					0X08
#define MAG_CONFIG_ADDR						0X09
#define GYRO_CONFIG_ADDR					0X0A
#define GYRO_MODE_CONFIG_ADDR				0X0B
#define ACCEL_SLEEP_CONFIG_ADDR				0X0C
#define GYRO_SLEEP_CONFIG_ADDR				0X0D
#define MAG_SLEEP_CONFIG_ADDR				0x0E

/* Interrupt registers*/
#define INT_MASK_ADDR						0X0F
#define INT_ADDR							0X10
#define ACCEL_ANYMOTION_THRES_ADDR			0X11
#define ACCEL_INT_SETTINGS_ADDR				0X12
#define ACCEL_HIGH_G_DUR_ADDR				0X13
#define ACCEL_HIGH_G_THRES_ADDR				0X14
#define ACCEL_NOMOTION_THRES_ADDR			0X15
#define ACCEL_NOMOTION_SET_ADDR				0X16
#define GYRO_INT_SETING_ADDR				0X17
#define GYRO_HIGHRATE_X_SET_ADDR			0X18
#define GYRO_DUR_X_ADDR						0X19
#define GYRO_HIGHRATE_Y_SET_ADDR			0X1A
#define GYRO_DUR_Y_ADDR						0X1B
#define GYRO_HIGHRATE_Z_SET_ADDR			0X1C
#define GYRO_DUR_Z_ADDR						0X1D
#define GYRO_ANYMOTION_THRES_ADDR			0X1E
#define GYRO_ANYMOTION_SET_ADDR				0X1F
/* PAGE1 REGISTERS DEFINITION END*/


#define BNO055_MDELAY_DATA_TYPE           _u32

/*< This refers BNO055 return type as _s8 */
#define BNO055_RETURN_FUNCTION_TYPE        _s8

/* Compile switch definition for Float and double*/
#define BNO055_FLOAT_ENABLE
#define BNO055_DOUBLE_ENABLE
/**************************************************************/
/**\name	STRUCTURE DEFINITIONS                         */
/**************************************************************/
/*!
*	@brief bno055 struct
*/
struct bno055_t {
_u8 chip_id;/**< chip_id of bno055 */
_u16 sw_rev_id;/**< software revision id of bno055 */
_u8 page_id;/**< page_id of bno055 */
_u8 accel_rev_id;/**< accel revision id of bno055 */
_u8 mag_rev_id;/**< mag revision id of bno055 */
_u8 gyro_rev_id;/**< gyro revision id of bno055 */
_u8 bl_rev_id;/**< boot loader revision id of bno055 */
_u8 dev_addr;/**< i2c device address of bno055 */
BNO055_WR_FUNC_PTR;/**< bus write function pointer */
BNO055_RD_FUNC_PTR;/**<bus read function pointer */
void (*delay_msec)(BNO055_MDELAY_DATA_TYPE);/**< delay function pointer */
};
/*!
* @brief struct for accel data read from registers
*/
struct bno055_accel {
_s16 x;/**< accel x data */
_s16 y;/**< accel y data */
_s16 z;/**< accel z data */
};
/*!
* @brief struct for Mag data read from registers
*/
struct bno055_mag {
_s16 x;/**< mag x data */
_s16 y;/**< mag y data */
_s16 z;/**< mag z data */
};
/*!
* @brief struct for Gyro data read from registers
*/
struct bno055_gyro {
_s16 x;/**< gyro x data */
_s16 y;/**< gyro y data */
_s16 z;/**< gyro z data */
};
/*!
* @brief struct for Euler data read from registers
*/
struct bno055_euler {
_s16 h;/**< Euler h data */
_s16 r;/**< Euler r data */
_s16 p;/**< Euler p data */
};
/*!
* @brief struct for Quaternion data read from registers
*/
struct bno055_quaternion {
_s16 w;/**< Quaternion w data */
_s16 x;/**< Quaternion x data */
_s16 y;/**< Quaternion y data */
_s16 z;/**< Quaternion z data */
};
/*!
* @brief struct for Linear Accel data read from registers
*/
struct bno055_linear_accel {
_s16 x; /**< Linear Accel x data */
_s16 y; /**< Linear Accel y data */
_s16 z; /**< Linear Accel z data */
};
/*!
* @brief struct for Gravity data read from registers
*/
struct bno055_gravity {
_s16 x;/**< Gravity x data */
_s16 y;/**< Gravity y data */
_s16 z;/**< Gravity z data */
};
#ifdef	BNO055_DOUBLE_ENABLE
/*!
* @brief struct for Accel-output data of precision double
*/
struct bno055_accel_double {
double x;/**< Accel x double data */
double y;/**< Accel y double data */
double z;/**< Accel z double data */
};
/*!
* @brief struct for Mag-output data of precision double
*/
struct bno055_mag_double {
double x;/**< Mag x double data */
double y;/**< Mag y double data */
double z;/**< Mag z double data */
};
/*!
* @brief struct for Gyro-output data of precision double
*/
struct bno055_gyro_double {
double x;/**< Gyro x double data */
double y;/**< Gyro y double data */
double z;/**< Gyro z double data */
};
/*!
* @brief struct for Euler-output data of precision double
*/
struct bno055_euler_double {
double h;/**< Euler h double data */
double r;/**< Euler r double data */
double p;/**< Euler p double data */
};
/*!
* @brief struct for Linear Accel-output data of precision double
*/
struct bno055_linear_accel_double {
double x;/**< linear accel x double data */
double y;/**< linear accel y double data */
double z;/**< linear accel z double data */
};
/*!
* @brief struct for Gravity-output data of precision double
*/
struct bno055_gravity_double {
double x;/**< Gravity x double data */
double y;/**< Gravity y double data */
double z;/**< Gravity z double data */
};
#endif
#ifdef	BNO055_FLOAT_ENABLE
/*!
* @brief struct for Accel-output data of precision float
*/
struct bno055_accel_float {
float x;/**< accel x float data */
float y;/**< accel y float data */
float z;/**< accel z float data */
};
/*!
* @brief struct for Mag-output data of precision float
*/
struct bno055_mag_float {
float x;/**< Mag x float data */
float y;/**< Mag y float data */
float z;/**< Mag z float data */
};
/*!
* @brief struct for Gyro-output data of precision float
*/
struct bno055_gyro_float {
float x;/**< Gyro x float data */
float y;/**< Gyro y float data */
float z;/**< Gyro z float data */
};
/*!
* @brief struct for Euler-output data of precision float
*/
struct bno055_euler_float {
float h;/**< Euler h float data */
float r;/**< Euler r float data */
float p;/**< Euler p float data */
};
/*!
* @brief struct for Linear accel-output data of precision float
*/
struct bno055_linear_accel_float {
float x;/**< Linear accel x float data */
float y;/**< Linear accel y float data */
float z;/**< Linear accel z float data */
};
/*!
* @brief struct for Gravity-output data of precision float
*/
struct bno055_gravity_float {
float x;/**< Gravity x float data */
float y;/**< Gravity y float data */
float z;/**< Gravity z float data */
};
#endif
/*!
* @brief struct for Accel offset
*/
struct bno055_accel_offset {
_s16 x;/**< Accel offset x data */
_s16 y;/**< Accel offset y data */
_s16 z;/**< Accel offset z data */
_s16 r;/**< Accel radius r data */
};
/*!
* @brief struct for Gyro offset
*/
struct bno055_gyro_offset {
_s16 x;/**< Gyro offset x data */
_s16 y;/**< Gyro offset y data */
_s16 z;/**< Gyro offset z data */
};
/*!
* @brief struct for Mag offset
*/
struct bno055_mag_offset {
_s16 x;/**< Mag offset x data */
_s16 y;/**< Mag offset y data */
_s16 z;/**< Mag offset z data */
_s16 r;/**< Mag radius x data */
};
/*!
* @brief struct for soft iron calibration matrix
*/
struct bno055_sic_matrix {
_s16 sic_0;/**< soft iron calibration matrix 0 data */
_s16 sic_1;/**< soft iron calibration matrix 1 data */
_s16 sic_2;/**< soft iron calibration matrix 2 data */
_s16 sic_3;/**< soft iron calibration matrix 3 data */
_s16 sic_4;/**< soft iron calibration matrix 4 data */
_s16 sic_5;/**< soft iron calibration matrix 5 data */
_s16 sic_6;/**< soft iron calibration matrix 6 data */
_s16 sic_7;/**< soft iron calibration matrix 7 data */
_s16 sic_8;/**< soft iron calibration matrix 8 data */
};
/***************************************************/
/**\name	CONSTANT DEFINITIONS                   */
/***************************************************/
#define         BNO055_ZERO_U8X           ((_u8)0)
#define         BNO055_TWO_U8X			  ((_u8)2)
#define         BNO055_FOUR_U8X           ((_u8)4)
#define         BNO055_FIVE_U8X           ((_u8)5)
#define         BNO055_SIX_U8X            ((_u8)6)
#define         BNO055_SEVEN_U8X          ((_u8)7)
#define         BNO055_ELEVEN_U8X         ((_u8)11)
#define         BNO055_SIXTEEN_U8X        ((_u8)16)
#define			BNO055_EIGHT_U8X		  ((_u8)8)


#define         BNO055_SHIFT_8_POSITION	   ((_u8)8)


/*  BNO055 API error codes */
#define E_NULL_PTR                  ((_s8)-127)
#define E_BNO055_OUT_OF_RANGE       ((_s8)-2)
#define	SUCCESS						((_u8)0)
#define	ERROR						((_s8)-1)

/* Selection for bit enable and disable */
#define ENABLED		0x01
#define DISABLED	0x00

/* Page ID */
#define PAGE_ZERO		0X00
#define PAGE_ONE		0X01

/* Enable the temperature source */
#define	ACCEL_TEMP_EN	0x00
#define	GYRO_TEMP_EN	0x01
#define	MCU_TEMP_EN		0x03

/*Accel unit*/
#define ACCEL_UNIT_MSQ	0x00
#define ACCEL_UNIT_MG	0x01

/*Gyro unit*/
#define GYRO_UNIT_DPS	0x00
#define GYRO_UNIT_RPS	0x01

/* Euler unit*/
#define EULER_UNIT_DEG	0x00
#define EULER_UNIT_RAD	0x01

/*Temperature unit*/
#define TEMP_UNIT_CELSIUS		0x00
#define TEMP_UNIT_FAHRENHEIT	0x01

/*Accel division factor*/
#define	ACCEL_DIV_MSQ	100.0
#define	ACCEL_DIV_MG	1

/*Mag division factor*/
#define MAG_DIV_MICROTESLA	16.0

/*Gyro division factor*/
#define GYRO_DIV_DPS		16.0
#define GYRO_DIV_RPS		900.0

/*Euler division factor*/
#define EULER_DIV_DEG		16.0
#define EULER_DIV_RAD		900.0

/*Linear accel division factor*/
#define	LINEAR_ACCEL_DIV_MSQ	100.0

/*Gravity accel division factor*/
#define	GRAVITY_DIV_MSQ	100.0

/* Temperature division factor*/
#define TEMP_DIV_FAHRENHEIT	0.5
#define TEMP_DIV_CELSIUS	1


/* Operation mode settings*/
#define OPERATION_MODE_CONFIG			0X00
#define OPERATION_MODE_ACCONLY			0X01
#define OPERATION_MODE_MAGONLY			0X02
#define OPERATION_MODE_GYRONLY			0X03
#define OPERATION_MODE_ACCMAG			0X04
#define OPERATION_MODE_ACCGYRO			0X05
#define OPERATION_MODE_MAGGYRO			0X06
#define OPERATION_MODE_AMG				0X07
#define OPERATION_MODE_IMUPLUS			0X08
#define OPERATION_MODE_COMPASS			0X09
#define OPERATION_MODE_M4G				0X0A
#define OPERATION_MODE_NDOF_FMC_OFF		0X0B
#define OPERATION_MODE_NDOF				0X0C

/* Data output rates*/
#define FASTEST_MODE_1		0X00
#define FASTEST_MODE_2		0X01
#define GAME_MODE			0X02
#define UI_MODE				0X04
#define NORMAL_MODE			0X05

/* Power mode*/
#define POWER_MODE_NORMAL	0X00
#define POWER_MODE_LOWPOWER	0X01
#define POWER_MODE_SUSPEND	0X02

/* PAGE-1 definitions*/
/* Accel Range */

#define ACCEL_RANGE_2G		0X00
#define ACCEL_RANGE_4G		0X01
#define ACCEL_RANGE_8G		0X02
#define ACCEL_RANGE_16G		0X03

/* Accel Bandwidth*/
#define ACCEL_BW_7_81HZ		0x00
#define ACCEL_BW_15_63HZ	0x01
#define ACCEL_BW_31_25HZ	0x02
#define ACCEL_BW_62_5HZ		0X03
#define ACCEL_BW_125HZ		0X04
#define ACCEL_BW_250HZ		0X05
#define ACCEL_BW_500HZ		0X06
#define ACCEL_BW_1000HZ		0X07

/* Accel Power mode*/
#define ACCEL_NORMAL			0X00
#define ACCEL_SUSPEND			0X01
#define ACCEL_LOWPOWER_1		0X02
#define ACCEL_STANDBY			0X03
#define ACCEL_LOWPOWER_2		0X04
#define ACCEL_DEEPSUSPEND		0X05

/* Mag data output rate*/
#define MAG_DATA_OUTRATE_2HZ		0X00
#define MAG_DATA_OUTRATE_6HZ		0X01
#define MAG_DATA_OUTRATE_8HZ		0X02
#define MAG_DATA_OUTRATE_10HZ		0X03
#define MAG_DATA_OUTRATE_15HZ		0X04
#define MAG_DATA_OUTRATE_20HZ		0X05
#define MAG_DATA_OUTRATE_25HZ		0X06
#define MAG_DATA_OUTRATE_30HZ		0X07

/* Mag Operation mode*/
#define MAG_OPERATION_MODE_LOWPOWER				0X00
#define MAG_OPERATION_MODE_REGULAR				0X01
#define MAG_OPERATION_MODE_ENHANCED_REGULAR		0X02
#define MAG_OPERATION_MODE_HIGH_ACCURACY		0X03

/* Mag power mode*/
#define MAG_POWER_MODE_NORMAL					0X00
#define MAG_POWER_MODE_SLEEP					0X01
#define MAG_POWER_MODE_SUSPEND					0X02
#define MAG_POWER_MODE_FORCE_MODE				0X03

/* Gyro range*/
#define GYRO_RANGE_2000DPS		0x00
#define GYRO_RANGE_1000DPS		0x01
#define GYRO_RANGE_500DPS		0x02
#define GYRO_RANGE_250DPS		0x03
#define GYRO_RANGE_125DPS		0x04

/* Gyro Bandwidth*/
#define GYRO_BW_523HZ		0x00
#define GYRO_BW_230HZ		0x01
#define GYRO_BW_116HZ		0x02
#define GYRO_BW_47HZ		0x03
#define GYRO_BW_23HZ		0x04
#define GYRO_BW_12HZ		0x05
#define GYRO_BW_64HZ		0x06
#define GYRO_BW_32HZ		0x07

/* Gyro power mode*/
#define GYRO_POWER_MODE_NORMAL				0X00
#define GYRO_POWER_MODE_FASTPOWERUP			0X01
#define GYRO_POWER_MODE_DEEPSUSPEND			0X02
#define GYRO_POWER_MODE_SUSPEND				0X03
#define GYRO_POWER_MODE_ADVANCE_POWERSAVE	0X04

/* Accel Sleep Duration */
#define BNO055_ACCEL_SLEEP_DUR_0_5MS        0x05
/* sets sleep duration to 0.5 ms  */
#define BNO055_ACCEL_SLEEP_DUR_1MS          0x06
/* sets sleep duration to 1 ms */
#define BNO055_ACCEL_SLEEP_DUR_2MS          0x07
/* sets sleep duration to 2 ms */
#define BNO055_ACCEL_SLEEP_DUR_4MS          0x08
/* sets sleep duration to 4 ms */
#define BNO055_ACCEL_SLEEP_DUR_6MS          0x09
/* sets sleep duration to 6 ms*/
#define BNO055_ACCEL_SLEEP_DUR_10MS         0x0A
/* sets sleep duration to 10 ms */
#define BNO055_ACCEL_SLEEP_DUR_25MS         0x0B
 /* sets sleep duration to 25 ms */
#define BNO055_ACCEL_SLEEP_DUR_50MS         0x0C
 /* sets sleep duration to 50 ms */
#define BNO055_ACCEL_SLEEP_DUR_100MS        0x0D
 /* sets sleep duration to 100 ms */
#define BNO055_ACCEL_SLEEP_DUR_500MS        0x0E
 /* sets sleep duration to 500 ms */
#define BNO055_ACCEL_SLEEP_DUR_1S           0x0F
/* sets sleep duration to 1 s */

/* Gyro Auto sleep duration*/
#define BNO055_GYRO_No_AUTOSLPDUR		0x00
#define	BNO055_GYRO_4MS_AUTOSLPDUR		0x01
#define	BNO055_GYRO_5MS_AUTOSLPDUR		0x02
#define	BNO055_GYRO_8MS_AUTOSLPDUR		0x03
#define	BNO055_GYRO_10MS_AUTOSLPDUR		0x04
#define	BNO055_GYRO_15MS_AUTOSLPDUR		0x05
#define	BNO055_GYRO_20MS_AUTOSLPDUR		0x06
#define	BNO055_GYRO_40MS_AUTOSLPDUR		0x07

/* Accel Any/No motion axis selection*/
#define BNO055_ACCEL_ANYMOTION_NOMOTION_X_AXIS		0
#define BNO055_ACCEL_ANYMOTION_NOMOTION_Y_AXIS		1
#define BNO055_ACCEL_ANYMOTION_NOMOTION_Z_AXIS		2

/* Accel High g axis selection*/
#define BNO055_ACCEL_HIGH_G_X_AXIS		0
#define BNO055_ACCEL_HIGH_G_Y_AXIS		1
#define BNO055_ACCEL_HIGH_G_Z_AXIS		2

/* Gyro Any motion axis selection*/
#define BNO055_GYRO_ANYMOTION_X_AXIS		0
#define BNO055_GYRO_ANYMOTION_Y_AXIS		1
#define BNO055_GYRO_ANYMOTION_Z_AXIS		2


/* Gyro High rate axis selection*/
#define BNO055_GYRO_HIGHRATE_X_AXIS		0
#define BNO055_GYRO_HIGHRATE_Y_AXIS		1
#define BNO055_GYRO_HIGHRATE_Z_AXIS		2

/* Axis remap values*/
#define REMAP_X_Y			0X21
#define REMAP_Y_Z			0X18
#define REMAP_Z_X			0X06
#define REMAP_X_Y_Z_TYPE0	0X12
#define REMAP_X_Y_Z_TYPE1	0X09
#define DEFAULT_AXIS		0X24

/* Axis remap sign */
#define	REMAP_AXIS_POSITIVE	0X00
#define	REMAP_AXIS_NEGATIVE	0X01

/* Gyro anymotion and high rate filter configuration */
#define	FILTERED	0x00
#define	UNFILTERED	0x01
/*********************************************************/
/**\name PAGE0 DATA REGISTERS DEFINITION */
/*********************************************************/
/* Chip ID */
#define BNO055_CHIP_ID__POS             0
#define BNO055_CHIP_ID__MSK             0xFF
#define BNO055_CHIP_ID__LEN             8
#define BNO055_CHIP_ID__REG             BNO055_CHIP_ID_ADDR

/* Accel revision id*/
#define BNO055_ACCEL_REV_ID__POS             0
#define BNO055_ACCEL_REV_ID__MSK             0xFF
#define BNO055_ACCEL_REV_ID__LEN             8
#define BNO055_ACCEL_REV_ID__REG             BNO055_ACCEL_REV_ID_ADDR

/* Mag revision id*/
#define BNO055_MAG_REV_ID__POS             0
#define BNO055_MAG_REV_ID__MSK             0xFF
#define BNO055_MAG_REV_ID__LEN             8
#define BNO055_MAG_REV_ID__REG             BNO055_MAG_REV_ID_ADDR

/* Gyro revision id*/
#define BNO055_GYRO_REV_ID__POS             0
#define BNO055_GYRO_REV_ID__MSK             0xFF
#define BNO055_GYRO_REV_ID__LEN             8
#define BNO055_GYRO_REV_ID__REG             BNO055_GYRO_REV_ID_ADDR

/*Software revision id LSB*/
#define BNO055_SW_REV_ID_LSB__POS             0
#define BNO055_SW_REV_ID_LSB__MSK             0xFF
#define BNO055_SW_REV_ID_LSB__LEN             8
#define BNO055_SW_REV_ID_LSB__REG             BNO055_SW_REV_ID_LSB_ADDR

/*Software revision id MSB*/
#define BNO055_SW_REV_ID_MSB__POS             0
#define BNO055_SW_REV_ID_MSB__MSK             0xFF
#define BNO055_SW_REV_ID_MSB__LEN             8
#define BNO055_SW_REV_ID_MSB__REG             BNO055_SW_REV_ID_MSB_ADDR

/* BOOTLODER revision id*/
#define BNO055_BL_REV_ID__POS             0
#define BNO055_BL_REV_ID__MSK             0xFF
#define BNO055_BL_REV_ID__LEN             8
#define BNO055_BL_REV_ID__REG             BNO055_BL_REV_ID_ADDR

/*Page id*/
#define BNO055_PAGE_ID__POS             0
#define BNO055_PAGE_ID__MSK             0xFF
#define BNO055_PAGE_ID__LEN             8
#define BNO055_PAGE_ID__REG             BNO055_PAGE_ID_ADDR

/* Accel data X-LSB register*/
#define BNO055_ACCEL_DATA_X_LSB_VALUEX__POS             0
#define BNO055_ACCEL_DATA_X_LSB_VALUEX__MSK             0xFF
#define BNO055_ACCEL_DATA_X_LSB_VALUEX__LEN             8
#define BNO055_ACCEL_DATA_X_LSB_VALUEX__REG             \
BNO055_ACCEL_DATA_X_LSB_ADDR

/* Accel data X-MSB register*/
#define BNO055_ACCEL_DATA_X_MSB_VALUEX__POS             0
#define BNO055_ACCEL_DATA_X_MSB_VALUEX__MSK             0xFF
#define BNO055_ACCEL_DATA_X_MSB_VALUEX__LEN             8
#define BNO055_ACCEL_DATA_X_MSB_VALUEX__REG             \
BNO055_ACCEL_DATA_X_MSB_ADDR

/* Accel data Y-LSB register*/
#define BNO055_ACCEL_DATA_Y_LSB_VALUEY__POS             0
#define BNO055_ACCEL_DATA_Y_LSB_VALUEY__MSK             0xFF
#define BNO055_ACCEL_DATA_Y_LSB_VALUEY__LEN             8
#define BNO055_ACCEL_DATA_Y_LSB_VALUEY__REG             \
BNO055_ACCEL_DATA_Y_LSB_ADDR

/* Accel data Y-MSB register*/
#define BNO055_ACCEL_DATA_Y_MSB_VALUEY__POS             0
#define BNO055_ACCEL_DATA_Y_MSB_VALUEY__MSK             0xFF
#define BNO055_ACCEL_DATA_Y_MSB_VALUEY__LEN             8
#define BNO055_ACCEL_DATA_Y_MSB_VALUEY__REG             \
BNO055_ACCEL_DATA_Y_MSB_ADDR

/* Accel data Z-LSB register*/
#define BNO055_ACCEL_DATA_Z_LSB_VALUEZ__POS		0
#define BNO055_ACCEL_DATA_Z_LSB_VALUEZ__MSK		0xFF
#define BNO055_ACCEL_DATA_Z_LSB_VALUEZ__LEN		8
#define BNO055_ACCEL_DATA_Z_LSB_VALUEZ__REG     \
BNO055_ACCEL_DATA_Z_LSB_ADDR

/* Accel data Z-MSB register*/
#define BNO055_ACCEL_DATA_Z_MSB_VALUEZ__POS		0
#define BNO055_ACCEL_DATA_Z_MSB_VALUEZ__MSK		0xFF
#define BNO055_ACCEL_DATA_Z_MSB_VALUEZ__LEN		8
#define BNO055_ACCEL_DATA_Z_MSB_VALUEZ__REG     \
BNO055_ACCEL_DATA_Z_MSB_ADDR

/* Mag data X-LSB register*/
#define BNO055_MAG_DATA_X_LSB_VALUEX__POS             0
#define BNO055_MAG_DATA_X_LSB_VALUEX__MSK             0xFF
#define BNO055_MAG_DATA_X_LSB_VALUEX__LEN             8
#define BNO055_MAG_DATA_X_LSB_VALUEX__REG             \
BNO055_MAG_DATA_X_LSB_ADDR

/* Mag data X-MSB register*/
#define BNO055_MAG_DATA_X_MSB_VALUEX__POS             0
#define BNO055_MAG_DATA_X_MSB_VALUEX__MSK             0xFF
#define BNO055_MAG_DATA_X_MSB_VALUEX__LEN             8
#define BNO055_MAG_DATA_X_MSB_VALUEX__REG             BNO055_MAG_DATA_X_MSB_ADDR

/* Mag data Y-LSB register*/
#define BNO055_MAG_DATA_Y_LSB_VALUEY__POS             0
#define BNO055_MAG_DATA_Y_LSB_VALUEY__MSK             0xFF
#define BNO055_MAG_DATA_Y_LSB_VALUEY__LEN             8
#define BNO055_MAG_DATA_Y_LSB_VALUEY__REG             BNO055_MAG_DATA_Y_LSB_ADDR

/* Mag data Y-MSB register*/
#define BNO055_MAG_DATA_Y_MSB_VALUEY__POS             0
#define BNO055_MAG_DATA_Y_MSB_VALUEY__MSK             0xFF
#define BNO055_MAG_DATA_Y_MSB_VALUEY__LEN             8
#define BNO055_MAG_DATA_Y_MSB_VALUEY__REG             BNO055_MAG_DATA_Y_MSB_ADDR

/* Mag data Z-LSB register*/
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__POS             0
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__MSK             0xFF
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__LEN             8
#define BNO055_MAG_DATA_Z_LSB_VALUEZ__REG             BNO055_MAG_DATA_Z_LSB_ADDR

/* Mag data Z-MSB register*/
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__POS             0
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__MSK             0xFF
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__LEN             8
#define BNO055_MAG_DATA_Z_MSB_VALUEZ__REG             BNO055_MAG_DATA_Z_MSB_ADDR

/* Gyro data X-LSB register*/
#define BNO055_GYRO_DATA_X_LSB_VALUEX__POS	0
#define BNO055_GYRO_DATA_X_LSB_VALUEX__MSK	0xFF
#define BNO055_GYRO_DATA_X_LSB_VALUEX__LEN	8
#define BNO055_GYRO_DATA_X_LSB_VALUEX__REG	BNO055_GYRO_DATA_X_LSB_ADDR

/* Gyro data X-MSB register*/
#define BNO055_GYRO_DATA_X_MSB_VALUEX__POS	0
#define BNO055_GYRO_DATA_X_MSB_VALUEX__MSK	0xFF
#define BNO055_GYRO_DATA_X_MSB_VALUEX__LEN	8
#define BNO055_GYRO_DATA_X_MSB_VALUEX__REG	BNO055_GYRO_DATA_X_MSB_ADDR

/* Gyro data Y-LSB register*/
#define BNO055_GYRO_DATA_Y_LSB_VALUEY__POS	0
#define BNO055_GYRO_DATA_Y_LSB_VALUEY__MSK	0xFF
#define BNO055_GYRO_DATA_Y_LSB_VALUEY__LEN	8
#define BNO055_GYRO_DATA_Y_LSB_VALUEY__REG	BNO055_GYRO_DATA_Y_LSB_ADDR

/* Gyro data Y-MSB register*/
#define BNO055_GYRO_DATA_Y_MSB_VALUEY__POS	0
#define BNO055_GYRO_DATA_Y_MSB_VALUEY__MSK	0xFF
#define BNO055_GYRO_DATA_Y_MSB_VALUEY__LEN	8
#define BNO055_GYRO_DATA_Y_MSB_VALUEY__REG	BNO055_GYRO_DATA_Y_MSB_ADDR

/* Gyro data Z-LSB register*/
#define BNO055_GYRO_DATA_Z_LSB_VALUEZ__POS	0
#define BNO055_GYRO_DATA_Z_LSB_VALUEZ__MSK	0xFF
#define BNO055_GYRO_DATA_Z_LSB_VALUEZ__LEN	8
#define BNO055_GYRO_DATA_Z_LSB_VALUEZ__REG	BNO055_GYRO_DATA_Z_LSB_ADDR

/* Gyro data Z-MSB register*/
#define BNO055_GYRO_DATA_Z_MSB_VALUEZ__POS	0
#define BNO055_GYRO_DATA_Z_MSB_VALUEZ__MSK	0xFF
#define BNO055_GYRO_DATA_Z_MSB_VALUEZ__LEN	8
#define BNO055_GYRO_DATA_Z_MSB_VALUEZ__REG	BNO055_GYRO_DATA_Z_MSB_ADDR

/* Euler data HEADING-LSB register*/
#define BNO055_EULER_H_LSB_VALUEH__POS   0
#define BNO055_EULER_H_LSB_VALUEH__MSK   0xFF
#define BNO055_EULER_H_LSB_VALUEH__LEN   8
#define BNO055_EULER_H_LSB_VALUEH__REG  BNO055_EULER_H_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BNO055_EULER_H_MSB_VALUEH__POS  0
#define BNO055_EULER_H_MSB_VALUEH__MSK  0xFF
#define BNO055_EULER_H_MSB_VALUEH__LEN  8
#define BNO055_EULER_H_MSB_VALUEH__REG  BNO055_EULER_H_MSB_ADDR

/* Euler data ROLL-LSB register*/
#define BNO055_EULER_R_LSB_VALUER__POS  0
#define BNO055_EULER_R_LSB_VALUER__MSK  0xFF
#define BNO055_EULER_R_LSB_VALUER__LEN  8
#define BNO055_EULER_R_LSB_VALUER__REG  BNO055_EULER_R_LSB_ADDR

/* Euler data ROLL-MSB register*/
#define BNO055_EULER_R_MSB_VALUER__POS  0
#define BNO055_EULER_R_MSB_VALUER__MSK  0xFF
#define BNO055_EULER_R_MSB_VALUER__LEN  8
#define BNO055_EULER_R_MSB_VALUER__REG  BNO055_EULER_R_MSB_ADDR

/* Euler data PITCH-LSB register*/
#define BNO055_EULER_P_LSB_VALUEP__POS  0
#define BNO055_EULER_P_LSB_VALUEP__MSK  0xFF
#define BNO055_EULER_P_LSB_VALUEP__LEN  8
#define BNO055_EULER_P_LSB_VALUEP__REG  BNO055_EULER_P_LSB_ADDR

/* Euler data HEADING-MSB register*/
#define BNO055_EULER_P_MSB_VALUEP__POS  0
#define BNO055_EULER_P_MSB_VALUEP__MSK  0xFF
#define BNO055_EULER_P_MSB_VALUEP__LEN  8
#define BNO055_EULER_P_MSB_VALUEP__REG  BNO055_EULER_P_MSB_ADDR

/* Quaternion data W-LSB register*/
#define BNO055_QUATERNION_DATA_W_LSB_VALUEW__POS  0
#define BNO055_QUATERNION_DATA_W_LSB_VALUEW__MSK  0xFF
#define BNO055_QUATERNION_DATA_W_LSB_VALUEW__LEN  8
#define BNO055_QUATERNION_DATA_W_LSB_VALUEW__REG  \
BNO055_QUATERNION_DATA_W_LSB_ADDR

/* Quaternion data W-MSB register*/
#define BNO055_QUATERNION_DATA_W_MSB_VALUEW__POS  0
#define BNO055_QUATERNION_DATA_W_MSB_VALUEW__MSK  0xFF
#define BNO055_QUATERNION_DATA_W_MSB_VALUEW__LEN  8
#define BNO055_QUATERNION_DATA_W_MSB_VALUEW__REG  \
BNO055_QUATERNION_DATA_W_MSB_ADDR

/* Quaternion data X-LSB register*/
#define BNO055_QUATERNION_DATA_X_LSB_VALUEX__POS  0
#define BNO055_QUATERNION_DATA_X_LSB_VALUEX__MSK  0xFF
#define BNO055_QUATERNION_DATA_X_LSB_VALUEX__LEN  8
#define BNO055_QUATERNION_DATA_X_LSB_VALUEX__REG  \
BNO055_QUATERNION_DATA_X_LSB_ADDR

/* Quaternion data X-MSB register*/
#define BNO055_QUATERNION_DATA_X_MSB_VALUEX__POS  0
#define BNO055_QUATERNION_DATA_X_MSB_VALUEX__MSK  0xFF
#define BNO055_QUATERNION_DATA_X_MSB_VALUEX__LEN  8
#define BNO055_QUATERNION_DATA_X_MSB_VALUEX__REG \
BNO055_QUATERNION_DATA_X_MSB_ADDR

/* Quaternion data Y-LSB register*/
#define BNO055_QUATERNION_DATA_Y_LSB_VALUEY__POS  0
#define BNO055_QUATERNION_DATA_Y_LSB_VALUEY__MSK  0xFF
#define BNO055_QUATERNION_DATA_Y_LSB_VALUEY__LEN  8
#define BNO055_QUATERNION_DATA_Y_LSB_VALUEY__REG \
BNO055_QUATERNION_DATA_Y_LSB_ADDR

/* Quaternion data Y-MSB register*/
#define BNO055_QUATERNION_DATA_Y_MSB_VALUEY__POS  0
#define BNO055_QUATERNION_DATA_Y_MSB_VALUEY__MSK  0xFF
#define BNO055_QUATERNION_DATA_Y_MSB_VALUEY__LEN  8
#define BNO055_QUATERNION_DATA_Y_MSB_VALUEY__REG  \
NO055_QUATERNION_DATA_Y_MSB_ADDR

/* Quaternion data Z-LSB register*/
#define BNO055_QUATERNION_DATA_Z_LSB_VALUEZ__POS  0
#define BNO055_QUATERNION_DATA_Z_LSB_VALUEZ__MSK  0xFF
#define BNO055_QUATERNION_DATA_Z_LSB_VALUEZ__LEN  8
#define BNO055_QUATERNION_DATA_Z_LSB_VALUEZ__REG \
BNO055_QUATERNION_DATA_Z_LSB_ADDR

/* Quaternion data Z-MSB register*/
#define BNO055_QUATERNION_DATA_Z_MSB_VALUEZ__POS  0
#define BNO055_QUATERNION_DATA_Z_MSB_VALUEZ__MSK  0xFF
#define BNO055_QUATERNION_DATA_Z_MSB_VALUEZ__LEN  8
#define BNO055_QUATERNION_DATA_Z_MSB_VALUEZ__REG  \
BNO055_QUATERNION_DATA_Z_MSB_ADDR

/* Linear acceleration data X-LSB register*/
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX__POS  0
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX__MSK  0xFF
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX__LEN  8
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX__REG  \
BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR

/* Linear acceleration data X-MSB register*/
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX__POS  0
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX__MSK  0xFF
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX__LEN  8
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX__REG  \
BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR

/* Linear acceleration data Y-LSB register*/
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY__POS  0
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY__MSK  0xFF
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY__LEN  8
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY__REG  \
BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR

/* Linear acceleration data Y-MSB register*/
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY__POS  0
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY__MSK  0xFF
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY__LEN  8
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY__REG  \
BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR

/* Linear acceleration data Z-LSB register*/
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ__POS  0
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ__MSK  0xFF
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ__LEN  8
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ__REG \
BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR

/* Linear acceleration data Z-MSB register*/
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ__POS  0
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ__MSK  0xFF
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ__LEN  8
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ__REG  \
BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR

/* Gravity data X-LSB register*/
#define BNO055_GRAVITY_DATA_X_LSB_VALUEX__POS  0
#define BNO055_GRAVITY_DATA_X_LSB_VALUEX__MSK  0xFF
#define BNO055_GRAVITY_DATA_X_LSB_VALUEX__LEN  8
#define BNO055_GRAVITY_DATA_X_LSB_VALUEX__REG  \
BNO055_GRAVITY_DATA_X_LSB_ADDR

/* Gravity data X-MSB register*/
#define BNO055_GRAVITY_DATA_X_MSB_VALUEX__POS  0
#define BNO055_GRAVITY_DATA_X_MSB_VALUEX__MSK  0xFF
#define BNO055_GRAVITY_DATA_X_MSB_VALUEX__LEN  8
#define BNO055_GRAVITY_DATA_X_MSB_VALUEX__REG  \
BNO055_GRAVITY_DATA_X_MSB_ADDR

/* Gravity data Y-LSB register*/
#define BNO055_GRAVITY_DATA_Y_LSB_VALUEY__POS  0
#define BNO055_GRAVITY_DATA_Y_LSB_VALUEY__MSK  0xFF
#define BNO055_GRAVITY_DATA_Y_LSB_VALUEY__LEN  8
#define BNO055_GRAVITY_DATA_Y_LSB_VALUEY__REG  \
BNO055_GRAVITY_DATA_Y_LSB_ADDR

/* Gravity data Y-MSB register*/
#define BNO055_GRAVITY_DATA_Y_MSB_VALUEY__POS  0
#define BNO055_GRAVITY_DATA_Y_MSB_VALUEY__MSK  0xFF
#define BNO055_GRAVITY_DATA_Y_MSB_VALUEY__LEN  8
#define BNO055_GRAVITY_DATA_Y_MSB_VALUEY__REG  \
BNO055_GRAVITY_DATA_Y_MSB_ADDR

/* Gravity data Z-LSB register*/
#define BNO055_GRAVITY_DATA_Z_LSB_VALUEZ__POS  0
#define BNO055_GRAVITY_DATA_Z_LSB_VALUEZ__MSK  0xFF
#define BNO055_GRAVITY_DATA_Z_LSB_VALUEZ__LEN  8
#define BNO055_GRAVITY_DATA_Z_LSB_VALUEZ__REG  \
BNO055_GRAVITY_DATA_Z_LSB_ADDR

/* Gravity data Z-MSB register*/
#define BNO055_GRAVITY_DATA_Z_MSB_VALUEZ__POS  0
#define BNO055_GRAVITY_DATA_Z_MSB_VALUEZ__MSK  0xFF
#define BNO055_GRAVITY_DATA_Z_MSB_VALUEZ__LEN  8
#define BNO055_GRAVITY_DATA_Z_MSB_VALUEZ__REG  \
BNO055_GRAVITY_DATA_Z_MSB_ADDR

/* Temperature register*/
#define BNO055_TEMP__POS             0
#define BNO055_TEMP__MSK             0xFF
#define BNO055_TEMP__LEN             8
#define BNO055_TEMP__REG             BNO055_TEMP_ADDR

/*Mag_Calib status register*/
#define BNO055_MAG_CALIB_STAT__POS             0
#define BNO055_MAG_CALIB_STAT__MSK             0X03
#define BNO055_MAG_CALIB_STAT__LEN             2
#define BNO055_MAG_CALIB_STAT__REG             BNO055_CALIB_STAT_ADDR

/*Acc_Calib status register*/
#define BNO055_ACCEL_CALIB_STAT__POS             2
#define BNO055_ACCEL_CALIB_STAT__MSK             0X0C
#define BNO055_ACCEL_CALIB_STAT__LEN             2
#define BNO055_ACCEL_CALIB_STAT__REG             BNO055_CALIB_STAT_ADDR

/*Gyro_Calib status register*/
#define BNO055_GYRO_CALIB_STAT__POS             4
#define BNO055_GYRO_CALIB_STAT__MSK             0X30
#define BNO055_GYRO_CALIB_STAT__LEN             2
#define BNO055_GYRO_CALIB_STAT__REG             BNO055_CALIB_STAT_ADDR

/*Sys_Calib status register*/
#define BNO055_SYS_CALIB_STAT__POS             6
#define BNO055_SYS_CALIB_STAT__MSK             0XC0
#define BNO055_SYS_CALIB_STAT__LEN             2
#define BNO055_SYS_CALIB_STAT__REG             BNO055_CALIB_STAT_ADDR

/*ST_ACCEL register*/
#define BNO055_SELFTEST_ACCEL__POS             0
#define BNO055_SELFTEST_ACCEL__MSK             0X01
#define BNO055_SELFTEST_ACCEL__LEN             1
#define BNO055_SELFTEST_ACCEL__REG             BNO055_SELFTEST_RESULT_ADDR

/*ST_MAG register*/
#define BNO055_SELFTEST_MAG__POS             1
#define BNO055_SELFTEST_MAG__MSK             0X02
#define BNO055_SELFTEST_MAG__LEN             1
#define BNO055_SELFTEST_MAG__REG             BNO055_SELFTEST_RESULT_ADDR

/*ST_GYRO register*/
#define BNO055_SELFTEST_GYRO__POS             2
#define BNO055_SELFTEST_GYRO__MSK             0X04
#define BNO055_SELFTEST_GYRO__LEN             1
#define BNO055_SELFTEST_GYRO__REG             BNO055_SELFTEST_RESULT_ADDR

/*ST_MCU register*/
#define BNO055_SELFTEST_MCU__POS             3
#define BNO055_SELFTEST_MCU__MSK             0X08
#define BNO055_SELFTEST_MCU__LEN             1
#define BNO055_SELFTEST_MCU__REG             BNO055_SELFTEST_RESULT_ADDR

/*Interrupt status registers*/
#define BNO055_INT_STAT_GYRO_ANYMOTION__POS		2
#define BNO055_INT_STAT_GYRO_ANYMOTION__MSK		0X04
#define BNO055_INT_STAT_GYRO_ANYMOTION__LEN		1
#define BNO055_INT_STAT_GYRO_ANYMOTION__REG		BNO055_INT_STAT_ADDR

#define BNO055_INT_STAT_GYRO_HIGHRATE__POS		3
#define BNO055_INT_STAT_GYRO_HIGHRATE__MSK		0X08
#define BNO055_INT_STAT_GYRO_HIGHRATE__LEN		1
#define BNO055_INT_STAT_GYRO_HIGHRATE__REG		BNO055_INT_STAT_ADDR

#define BNO055_INT_STAT_ACCEL_HIGH_G__POS		5
#define BNO055_INT_STAT_ACCEL_HIGH_G__MSK		0X20
#define BNO055_INT_STAT_ACCEL_HIGH_G__LEN		1
#define BNO055_INT_STAT_ACCEL_HIGH_G__REG		BNO055_INT_STAT_ADDR

#define BNO055_INT_STAT_ACCEL_ANYMOTION__POS		6
#define BNO055_INT_STAT_ACCEL_ANYMOTION__MSK		0X40
#define BNO055_INT_STAT_ACCEL_ANYMOTION__LEN		1
#define BNO055_INT_STAT_ACCEL_ANYMOTION__REG		BNO055_INT_STAT_ADDR

#define BNO055_INT_STAT_ACCEL_NOMOTION__POS		7
#define BNO055_INT_STAT_ACCEL_NOMOTION__MSK		0X80
#define BNO055_INT_STAT_ACCEL_NOMOTION__LEN		1
#define BNO055_INT_STAT_ACCEL_NOMOTION__REG		BNO055_INT_STAT_ADDR

/* system clock status register*/
#define BNO055_SYS_MAIN_CLK__POS		0
#define BNO055_SYS_MAIN_CLK__MSK		0X10
#define BNO055_SYS_MAIN_CLK__LEN		1
#define BNO055_SYS_MAIN_CLK__REG		BNO055_SYS_CLK_STAT_ADDR

/* System registers*/
#define BNO055_SYS_STAT_CODE__POS		0
#define BNO055_SYS_STAT_CODE__MSK		0XFF
#define BNO055_SYS_STAT_CODE__LEN		8
#define BNO055_SYS_STAT_CODE__REG		BNO055_SYS_STAT_ADDR

#define BNO055_SYS_ERROR_CODE__POS			0
#define BNO055_SYS_ERROR_CODE__MSK			0XFF
#define BNO055_SYS_ERROR_CODE__LEN			8
#define BNO055_SYS_ERROR_CODE__REG			BNO055_SYS_ERR_ADDR

/* Accel_Unit register*/
#define BNO055_ACCEL_UNIT__POS             0
#define BNO055_ACCEL_UNIT__MSK             0X01
#define BNO055_ACCEL_UNIT__LEN             1
#define BNO055_ACCEL_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* Gyro_Unit register*/
#define BNO055_GYRO_UNIT__POS             1
#define BNO055_GYRO_UNIT__MSK             0X02
#define BNO055_GYRO_UNIT__LEN             1
#define BNO055_GYRO_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* Euler_Unit register*/
#define BNO055_EULER_UNIT__POS             2
#define BNO055_EULER_UNIT__MSK             0X04
#define BNO055_EULER_UNIT__LEN             1
#define BNO055_EULER_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* Tilt_Unit register*/
#define BNO055_TILT_UNIT__POS             3
#define BNO055_TILT_UNIT__MSK             0X08
#define BNO055_TILT_UNIT__LEN             1
#define BNO055_TILT_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* Temperature_Unit register*/
#define BNO055_TEMP_UNIT__POS             4
#define BNO055_TEMP_UNIT__MSK             0X10
#define BNO055_TEMP_UNIT__LEN             1
#define BNO055_TEMP_UNIT__REG             BNO055_UNIT_SEL_ADDR

/* ORI android-windows register*/
#define BNO055_DATA_OUTPUT_FORMAT__POS             7
#define BNO055_DATA_OUTPUT_FORMAT__MSK             0X80
#define BNO055_DATA_OUTPUT_FORMAT__LEN             1
#define BNO055_DATA_OUTPUT_FORMAT__REG             BNO055_UNIT_SEL_ADDR

/*Data Select register*/
/* Accel data select*/
#define BNO055_ACCEL_DATA_SELECT__POS			0
#define BNO055_ACCEL_DATA_SELECT__MSK			0X01
#define BNO055_ACCEL_DATA_SELECT__LEN			1
#define BNO055_ACCEL_DATA_SELECT__REG			BNO055_DATA_SELECT_ADDR

/* Mag data select*/
#define BNO055_MAG_DATA_SELECT__POS			1
#define BNO055_MAG_DATA_SELECT__MSK			0X02
#define BNO055_MAG_DATA_SELECT__LEN			1
#define BNO055_MAG_DATA_SELECT__REG			BNO055_DATA_SELECT_ADDR

/* Gyro data select*/
#define BNO055_GYRO_DATA_SELECT__POS			2
#define BNO055_GYRO_DATA_SELECT__MSK			0X04
#define BNO055_GYRO_DATA_SELECT__LEN			1
#define BNO055_GYRO_DATA_SELECT__REG			BNO055_DATA_SELECT_ADDR

/* Euler data select*/
#define BNO055_EULER_DATA_SELECT__POS			3
#define BNO055_EULER_DATA_SELECT__MSK			0X08
#define BNO055_EULER_DATA_SELECT__LEN			1
#define BNO055_EULER_DATA_SELECT__REG			BNO055_DATA_SELECT_ADDR

/* Quaternion data select*/
#define BNO055_QUATERNION_DATA_SELECT__POS			4
#define BNO055_QUATERNION_DATA_SELECT__MSK			0X10
#define BNO055_QUATERNION_DATA_SELECT__LEN			1
#define BNO055_QUATERNION_DATA_SELECT__REG			\
BNO055_DATA_SELECT_ADDR

/* Linear Accel data select*/
#define BNO055_LINEAR_ACCEL_DATA_SELECT__POS			5
#define BNO055_LINEAR_ACCEL_DATA_SELECT__MSK			0X20
#define BNO055_LINEAR_ACCEL_DATA_SELECT__LEN			1
#define BNO055_LINEAR_ACCEL_DATA_SELECT__REG			\
BNO055_DATA_SELECT_ADDR

/* Gravity data select*/
#define BNO055_GRAVITY_DATA_SELECT__POS			6
#define BNO055_GRAVITY_DATA_SELECT__MSK			0X40
#define BNO055_GRAVITY_DATA_SELECT__LEN			1
#define BNO055_GRAVITY_DATA_SELECT__REG			BNO055_DATA_SELECT_ADDR

/* Temperature data select*/
#define BNO055_TEMP_DATA_SELECT__POS			7
#define BNO055_TEMP_DATA_SELECT__MSK			0X80
#define BNO055_TEMP_DATA_SELECT__LEN			1
#define BNO055_TEMP_DATA_SELECT__REG			BNO055_DATA_SELECT_ADDR

/*Operation Mode data register*/
#define BNO055_OPERATION_MODE__POS			0
#define BNO055_OPERATION_MODE__MSK			0X0F
#define BNO055_OPERATION_MODE__LEN			4
#define BNO055_OPERATION_MODE__REG			BNO055_OPR_MODE_ADDR

/* output data rate register*/
#define BNO055_OUTPUT_DATA_RATE__POS		4
#define BNO055_OUTPUT_DATA_RATE__MSK		0X70
#define BNO055_OUTPUT_DATA_RATE__LEN		3
#define BNO055_OUTPUT_DATA_RATE__REG		BNO055_OPR_MODE_ADDR

/* Power Mode register*/
#define BNO055_POWER_MODE__POS             0
#define BNO055_POWER_MODE__MSK             0X03
#define BNO055_POWER_MODE__LEN             2
#define BNO055_POWER_MODE__REG             BNO055_PWR_MODE_ADDR

/*Self Test register*/
#define BNO055_SELFTEST__POS			0
#define BNO055_SELFTEST__MSK			0X01
#define BNO055_SELFTEST__LEN			1
#define BNO055_SELFTEST__REG			BNO055_SYS_TRIGGER_ADDR

/* RST_SYS register*/
#define BNO055_SYS_RST__POS             5
#define BNO055_SYS_RST__MSK             0X20
#define BNO055_SYS_RST__LEN             1
#define BNO055_SYS_RST__REG             BNO055_SYS_TRIGGER_ADDR

/* RST_INT register*/
#define BNO055_INT_RST__POS             6
#define BNO055_INT_RST__MSK             0X40
#define BNO055_INT_RST__LEN             1
#define BNO055_INT_RST__REG             BNO055_SYS_TRIGGER_ADDR

/* CLK_SRC register*/
#define BNO055_CLK_SRC__POS             7
#define BNO055_CLK_SRC__MSK             0X80
#define BNO055_CLK_SRC__LEN             1
#define BNO055_CLK_SRC__REG             BNO055_SYS_TRIGGER_ADDR

/* Temp source register*/
#define BNO055_TEMP_SOURCE__POS		0
#define BNO055_TEMP_SOURCE__MSK		0X03
#define BNO055_TEMP_SOURCE__LEN		2
#define BNO055_TEMP_SOURCE__REG		BNO055_TEMP_SOURCE_ADDR

/* Axis remap value register*/
#define BNO055_REMAP_AXIS_VALUE__POS		0
#define BNO055_REMAP_AXIS_VALUE__MSK		0X3F
#define BNO055_REMAP_AXIS_VALUE__LEN		6
#define BNO055_REMAP_AXIS_VALUE__REG		BNO055_AXIS_MAP_CONFIG_ADDR

/* Axis sign value register*/
#define BNO055_REMAP_Z_SIGN__POS		0
#define BNO055_REMAP_Z_SIGN__MSK		0X01
#define BNO055_REMAP_Z_SIGN__LEN		1
#define BNO055_REMAP_Z_SIGN__REG		BNO055_AXIS_MAP_SIGN_ADDR

#define BNO055_REMAP_Y_SIGN__POS		1
#define BNO055_REMAP_Y_SIGN__MSK		0X02
#define BNO055_REMAP_Y_SIGN__LEN		1
#define BNO055_REMAP_Y_SIGN__REG		BNO055_AXIS_MAP_SIGN_ADDR

#define BNO055_REMAP_X_SIGN__POS		2
#define BNO055_REMAP_X_SIGN__MSK		0X04
#define BNO055_REMAP_X_SIGN__LEN		1
#define BNO055_REMAP_X_SIGN__REG		BNO055_AXIS_MAP_SIGN_ADDR

/* Soft Iron Calibration matrix register*/
#define BNO055_SIC_MATRIX_0_LSB__POS		0
#define BNO055_SIC_MATRIX_0_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_0_LSB__LEN		8
#define BNO055_SIC_MATRIX_0_LSB__REG		BNO055_SIC_MATRIX_0_LSB_ADDR

#define BNO055_SIC_MATRIX_0_MSB__POS		0
#define BNO055_SIC_MATRIX_0_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_0_MSB__LEN		8
#define BNO055_SIC_MATRIX_0_MSB__REG		BNO055_SIC_MATRIX_0_MSB_ADDR

#define BNO055_SIC_MATRIX_1_LSB__POS		0
#define BNO055_SIC_MATRIX_1_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_1_LSB__LEN		8
#define BNO055_SIC_MATRIX_1_LSB__REG		BNO055_SIC_MATRIX_1_LSB_ADDR

#define BNO055_SIC_MATRIX_1_MSB__POS		0
#define BNO055_SIC_MATRIX_1_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_1_MSB__LEN		8
#define BNO055_SIC_MATRIX_1_MSB__REG		BNO055_SIC_MATRIX_1_MSB_ADDR

#define BNO055_SIC_MATRIX_2_LSB__POS		0
#define BNO055_SIC_MATRIX_2_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_2_LSB__LEN		8
#define BNO055_SIC_MATRIX_2_LSB__REG		BNO055_SIC_MATRIX_2_LSB_ADDR

#define BNO055_SIC_MATRIX_2_MSB__POS		0
#define BNO055_SIC_MATRIX_2_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_2_MSB__LEN		8
#define BNO055_SIC_MATRIX_2_MSB__REG		BNO055_SIC_MATRIX_2_MSB_ADDR

#define BNO055_SIC_MATRIX_3_LSB__POS		0
#define BNO055_SIC_MATRIX_3_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_3_LSB__LEN		8
#define BNO055_SIC_MATRIX_3_LSB__REG		BNO055_SIC_MATRIX_3_LSB_ADDR

#define BNO055_SIC_MATRIX_3_MSB__POS		0
#define BNO055_SIC_MATRIX_3_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_3_MSB__LEN		8
#define BNO055_SIC_MATRIX_3_MSB__REG		BNO055_SIC_MATRIX_3_MSB_ADDR

#define BNO055_SIC_MATRIX_4_LSB__POS		0
#define BNO055_SIC_MATRIX_4_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_4_LSB__LEN		8
#define BNO055_SIC_MATRIX_4_LSB__REG		BNO055_SIC_MATRIX_4_LSB_ADDR

#define BNO055_SIC_MATRIX_4_MSB__POS		0
#define BNO055_SIC_MATRIX_4_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_4_MSB__LEN		8
#define BNO055_SIC_MATRIX_4_MSB__REG		BNO055_SIC_MATRIX_4_MSB_ADDR

#define BNO055_SIC_MATRIX_5_LSB__POS		0
#define BNO055_SIC_MATRIX_5_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_5_LSB__LEN		8
#define BNO055_SIC_MATRIX_5_LSB__REG		BNO055_SIC_MATRIX_5_LSB_ADDR

#define BNO055_SIC_MATRIX_5_MSB__POS		0
#define BNO055_SIC_MATRIX_5_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_5_MSB__LEN		8
#define BNO055_SIC_MATRIX_5_MSB__REG		BNO055_SIC_MATRIX_5_MSB_ADDR

#define BNO055_SIC_MATRIX_6_LSB__POS		0
#define BNO055_SIC_MATRIX_6_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_6_LSB__LEN		8
#define BNO055_SIC_MATRIX_6_LSB__REG		BNO055_SIC_MATRIX_6_LSB_ADDR

#define BNO055_SIC_MATRIX_6_MSB__POS		0
#define BNO055_SIC_MATRIX_6_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_6_MSB__LEN		8
#define BNO055_SIC_MATRIX_6_MSB__REG		BNO055_SIC_MATRIX_6_MSB_ADDR

#define BNO055_SIC_MATRIX_7_LSB__POS		0
#define BNO055_SIC_MATRIX_7_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_7_LSB__LEN		8
#define BNO055_SIC_MATRIX_7_LSB__REG		BNO055_SIC_MATRIX_7_LSB_ADDR

#define BNO055_SIC_MATRIX_7_MSB__POS		0
#define BNO055_SIC_MATRIX_7_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_7_MSB__LEN		8
#define BNO055_SIC_MATRIX_7_MSB__REG		BNO055_SIC_MATRIX_7_MSB_ADDR

#define BNO055_SIC_MATRIX_8_LSB__POS		0
#define BNO055_SIC_MATRIX_8_LSB__MSK		0XFF
#define BNO055_SIC_MATRIX_8_LSB__LEN		8
#define BNO055_SIC_MATRIX_8_LSB__REG		BNO055_SIC_MATRIX_8_LSB_ADDR

#define BNO055_SIC_MATRIX_8_MSB__POS		0
#define BNO055_SIC_MATRIX_8_MSB__MSK		0XFF
#define BNO055_SIC_MATRIX_8_MSB__LEN		8
#define BNO055_SIC_MATRIX_8_MSB__REG		BNO055_SIC_MATRIX_8_MSB_ADDR

/*Accel Offset registers*/
#define BNO055_ACCEL_OFFSET_X_LSB__POS		0
#define BNO055_ACCEL_OFFSET_X_LSB__MSK		0XFF
#define BNO055_ACCEL_OFFSET_X_LSB__LEN		8
#define BNO055_ACCEL_OFFSET_X_LSB__REG		ACCEL_OFFSET_X_LSB_ADDR

#define BNO055_ACCEL_OFFSET_X_MSB__POS		0
#define BNO055_ACCEL_OFFSET_X_MSB__MSK		0XFF
#define BNO055_ACCEL_OFFSET_X_MSB__LEN		8
#define BNO055_ACCEL_OFFSET_X_MSB__REG		ACCEL_OFFSET_X_MSB_ADDR

#define BNO055_ACCEL_OFFSET_Y_LSB__POS		0
#define BNO055_ACCEL_OFFSET_Y_LSB__MSK		0XFF
#define BNO055_ACCEL_OFFSET_Y_LSB__LEN		8
#define BNO055_ACCEL_OFFSET_Y_LSB__REG		ACCEL_OFFSET_Y_LSB_ADDR

#define BNO055_ACCEL_OFFSET_Y_MSB__POS		0
#define BNO055_ACCEL_OFFSET_Y_MSB__MSK		0XFF
#define BNO055_ACCEL_OFFSET_Y_MSB__LEN		8
#define BNO055_ACCEL_OFFSET_Y_MSB__REG		ACCEL_OFFSET_Y_MSB_ADDR

#define BNO055_ACCEL_OFFSET_Z_LSB__POS		0
#define BNO055_ACCEL_OFFSET_Z_LSB__MSK		0XFF
#define BNO055_ACCEL_OFFSET_Z_LSB__LEN		8
#define BNO055_ACCEL_OFFSET_Z_LSB__REG		ACCEL_OFFSET_Z_LSB_ADDR

#define BNO055_ACCEL_OFFSET_Z_MSB__POS		0
#define BNO055_ACCEL_OFFSET_Z_MSB__MSK		0XFF
#define BNO055_ACCEL_OFFSET_Z_MSB__LEN		8
#define BNO055_ACCEL_OFFSET_Z_MSB__REG		ACCEL_OFFSET_Z_MSB_ADDR

/*Mag Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB__POS		0
#define BNO055_MAG_OFFSET_X_LSB__MSK		0XFF
#define BNO055_MAG_OFFSET_X_LSB__LEN		8
#define BNO055_MAG_OFFSET_X_LSB__REG		MAG_OFFSET_X_LSB_ADDR

#define BNO055_MAG_OFFSET_X_MSB__POS		0
#define BNO055_MAG_OFFSET_X_MSB__MSK		0XFF
#define BNO055_MAG_OFFSET_X_MSB__LEN		8
#define BNO055_MAG_OFFSET_X_MSB__REG		MAG_OFFSET_X_MSB_ADDR

#define BNO055_MAG_OFFSET_Y_LSB__POS		0
#define BNO055_MAG_OFFSET_Y_LSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Y_LSB__LEN		8
#define BNO055_MAG_OFFSET_Y_LSB__REG		MAG_OFFSET_Y_LSB_ADDR

#define BNO055_MAG_OFFSET_Y_MSB__POS		0
#define BNO055_MAG_OFFSET_Y_MSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Y_MSB__LEN		8
#define BNO055_MAG_OFFSET_Y_MSB__REG		MAG_OFFSET_Y_MSB_ADDR

#define BNO055_MAG_OFFSET_Z_LSB__POS		0
#define BNO055_MAG_OFFSET_Z_LSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Z_LSB__LEN		8
#define BNO055_MAG_OFFSET_Z_LSB__REG		MAG_OFFSET_Z_LSB_ADDR

#define BNO055_MAG_OFFSET_Z_MSB__POS		0
#define BNO055_MAG_OFFSET_Z_MSB__MSK		0XFF
#define BNO055_MAG_OFFSET_Z_MSB__LEN		8
#define BNO055_MAG_OFFSET_Z_MSB__REG		MAG_OFFSET_Z_MSB_ADDR

/* Gyro Offset registers*/
#define BNO055_GYRO_OFFSET_X_LSB__POS		0
#define BNO055_GYRO_OFFSET_X_LSB__MSK		0XFF
#define BNO055_GYRO_OFFSET_X_LSB__LEN		8
#define BNO055_GYRO_OFFSET_X_LSB__REG		GYRO_OFFSET_X_LSB_ADDR

#define BNO055_GYRO_OFFSET_X_MSB__POS		0
#define BNO055_GYRO_OFFSET_X_MSB__MSK		0XFF
#define BNO055_GYRO_OFFSET_X_MSB__LEN		8
#define BNO055_GYRO_OFFSET_X_MSB__REG		GYRO_OFFSET_X_MSB_ADDR

#define BNO055_GYRO_OFFSET_Y_LSB__POS		0
#define BNO055_GYRO_OFFSET_Y_LSB__MSK		0XFF
#define BNO055_GYRO_OFFSET_Y_LSB__LEN		8
#define BNO055_GYRO_OFFSET_Y_LSB__REG		GYRO_OFFSET_Y_LSB_ADDR

#define BNO055_GYRO_OFFSET_Y_MSB__POS		0
#define BNO055_GYRO_OFFSET_Y_MSB__MSK		0XFF
#define BNO055_GYRO_OFFSET_Y_MSB__LEN		8
#define BNO055_GYRO_OFFSET_Y_MSB__REG		GYRO_OFFSET_Y_MSB_ADDR

#define BNO055_GYRO_OFFSET_Z_LSB__POS		0
#define BNO055_GYRO_OFFSET_Z_LSB__MSK		0XFF
#define BNO055_GYRO_OFFSET_Z_LSB__LEN		8
#define BNO055_GYRO_OFFSET_Z_LSB__REG		GYRO_OFFSET_Z_LSB_ADDR

#define BNO055_GYRO_OFFSET_Z_MSB__POS		0
#define BNO055_GYRO_OFFSET_Z_MSB__MSK		0XFF
#define BNO055_GYRO_OFFSET_Z_MSB__LEN		8
#define BNO055_GYRO_OFFSET_Z_MSB__REG		GYRO_OFFSET_Z_MSB_ADDR

/* Radius register definition*/
#define BNO055_ACCEL_RADIUS_LSB__POS		0
#define BNO055_ACCEL_RADIUS_LSB__MSK		0XFF
#define BNO055_ACCEL_RADIUS_LSB__LEN		8
#define BNO055_ACCEL_RADIUS_LSB__REG		ACCEL_RADIUS_LSB_ADDR

#define BNO055_ACCEL_RADIUS_MSB__POS		0
#define BNO055_ACCEL_RADIUS_MSB__MSK		0XFF
#define BNO055_ACCEL_RADIUS_MSB__LEN		8
#define BNO055_ACCEL_RADIUS_MSB__REG		ACCEL_RADIUS_MSB_ADDR

#define BNO055_MAG_RADIUS_LSB__POS		0
#define BNO055_MAG_RADIUS_LSB__MSK		0XFF
#define BNO055_MAG_RADIUS_LSB__LEN		8
#define BNO055_MAG_RADIUS_LSB__REG		MAG_RADIUS_LSB_ADDR

#define BNO055_MAG_RADIUS_MSB__POS		0
#define BNO055_MAG_RADIUS_MSB__MSK		0XFF
#define BNO055_MAG_RADIUS_MSB__LEN		8
#define BNO055_MAG_RADIUS_MSB__REG		MAG_RADIUS_MSB_ADDR

/* PAGE0 DATA REGISTERS DEFINITION END*/
/*************************************************/
/**\name PAGE1 DATA REGISTERS DEFINITION   */
/*************************************************/
/* Configuration registers*/
/* Accel range configuration register*/
#define BNO055_ACCEL_RANGE__POS		0
#define BNO055_ACCEL_RANGE__MSK		0X03
#define BNO055_ACCEL_RANGE__LEN		2
#define BNO055_ACCEL_RANGE__REG		ACCEL_CONFIG_ADDR

/* Accel bandwidth configuration register*/
#define BNO055_ACCEL_BW__POS			2
#define BNO055_ACCEL_BW__MSK			0X1C
#define BNO055_ACCEL_BW__LEN			3
#define BNO055_ACCEL_BW__REG			ACCEL_CONFIG_ADDR

/* Accel power mode configuration register*/
#define BNO055_ACCEL_POWER_MODE__POS		5
#define BNO055_ACCEL_POWER_MODE__MSK		0XE0
#define BNO055_ACCEL_POWER_MODE__LEN		3
#define BNO055_ACCEL_POWER_MODE__REG		ACCEL_CONFIG_ADDR

/* Mag data output rate configuration register*/
#define BNO055_MAG_DATA_OUTPUT_RATE__POS		0
#define BNO055_MAG_DATA_OUTPUT_RATE__MSK		0X07
#define BNO055_MAG_DATA_OUTPUT_RATE__LEN		3
#define BNO055_MAG_DATA_OUTPUT_RATE__REG		MAG_CONFIG_ADDR

/* Mag operation mode configuration register*/
#define BNO055_MAG_OPERATION_MODE__POS		3
#define BNO055_MAG_OPERATION_MODE__MSK		0X18
#define BNO055_MAG_OPERATION_MODE__LEN		2
#define BNO055_MAG_OPERATION_MODE__REG		MAG_CONFIG_ADDR

/* Mag power mode configuration register*/
#define BNO055_MAG_POWER_MODE__POS		5
#define BNO055_MAG_POWER_MODE__MSK		0X60
#define BNO055_MAG_POWER_MODE__LEN		2
#define BNO055_MAG_POWER_MODE__REG		MAG_CONFIG_ADDR

/* Gyro range configuration register*/
#define BNO055_GYRO_RANGE__POS		0
#define BNO055_GYRO_RANGE__MSK		0X07
#define BNO055_GYRO_RANGE__LEN		3
#define BNO055_GYRO_RANGE__REG		GYRO_CONFIG_ADDR

/* Gyro bandwidth configuration register*/
#define BNO055_GYRO_BW__POS		3
#define BNO055_GYRO_BW__MSK		0X38
#define BNO055_GYRO_BW__LEN		3
#define BNO055_GYRO_BW__REG		GYRO_CONFIG_ADDR

/* Gyro power mode configuration register*/
#define BNO055_GYRO_POWER_MODE__POS		0
#define BNO055_GYRO_POWER_MODE__MSK		0X07
#define BNO055_GYRO_POWER_MODE__LEN		3
#define BNO055_GYRO_POWER_MODE__REG		GYRO_MODE_CONFIG_ADDR

/* Sleep configuration registers*/
/* Accel sleep mode configuration register*/
#define BNO055_ACCEL_SLEEP_MODE__POS		0
#define BNO055_ACCEL_SLEEP_MODE__MSK		0X01
#define BNO055_ACCEL_SLEEP_MODE__LEN		1
#define BNO055_ACCEL_SLEEP_MODE__REG		ACCEL_SLEEP_CONFIG_ADDR

/* Accel sleep duration configuration register*/
#define BNO055_ACCEL_SLEEP_DUR__POS		1
#define BNO055_ACCEL_SLEEP_DUR__MSK		0X1E
#define BNO055_ACCEL_SLEEP_DUR__LEN		4
#define BNO055_ACCEL_SLEEP_DUR__REG		ACCEL_SLEEP_CONFIG_ADDR

/* Gyro sleep duration configuration register*/
#define BNO055_GYRO_SLEEP_DUR__POS		0
#define BNO055_GYRO_SLEEP_DUR__MSK		0X07
#define BNO055_GYRO_SLEEP_DUR__LEN		3
#define BNO055_GYRO_SLEEP_DUR__REG		GYRO_SLEEP_CONFIG_ADDR

/* Gyro auto sleep duration configuration register*/
#define BNO055_GYRO_AUTO_SLEEP_DUR__POS		3
#define BNO055_GYRO_AUTO_SLEEP_DUR__MSK		0X38
#define BNO055_GYRO_AUTO_SLEEP_DUR__LEN		3
#define BNO055_GYRO_AUTO_SLEEP_DUR__REG		GYRO_SLEEP_CONFIG_ADDR

/* Mag sleep mode configuration register*/
#define BNO055_MAG_SLEEP_MODE__POS		0
#define BNO055_MAG_SLEEP_MODE__MSK		0X01
#define BNO055_MAG_SLEEP_MODE__LEN		1
#define BNO055_MAG_SLEEP_MODE__REG		MAG_SLEEP_CONFIG_ADDR

/* Mag sleep duration configuration register*/
#define BNO055_MAG_SLEEP_DUR__POS		1
#define BNO055_MAG_SLEEP_DUR__MSK		0X1E
#define BNO055_MAG_SLEEP_DUR__LEN		4
#define BNO055_MAG_SLEEP_DUR__REG		MAG_SLEEP_CONFIG_ADDR

/* Interrupt registers*/
/* Gyro any motion interrupt msk register*/
#define BNO055_GYRO_ANYMOTION_INT_MASK__POS		2
#define BNO055_GYRO_ANYMOTION_INT_MASK__MSK		0X04
#define BNO055_GYRO_ANYMOTION_INT_MASK__LEN		1
#define BNO055_GYRO_ANYMOTION_INT_MASK__REG		INT_MASK_ADDR

/* Gyro high rate interrupt msk register*/
#define BNO055_GYRO_HIGHRATE_INT_MASK__POS		3
#define BNO055_GYRO_HIGHRATE_INT_MASK__MSK		0X08
#define BNO055_GYRO_HIGHRATE_INT_MASK__LEN		1
#define BNO055_GYRO_HIGHRATE_INT_MASK__REG		INT_MASK_ADDR

/* Accel high g interrupt msk register*/
#define BNO055_ACCEL_HIGH_G_INT_MASK__POS		5
#define BNO055_ACCEL_HIGH_G_INT_MASK__MSK		0X20
#define BNO055_ACCEL_HIGH_G_INT_MASK__LEN		1
#define BNO055_ACCEL_HIGH_G_INT_MASK__REG		INT_MASK_ADDR

/* Accel any motion interrupt msk register*/
#define BNO055_ACCEL_ANYMOTION_INT_MASK__POS		6
#define BNO055_ACCEL_ANYMOTION_INT_MASK__MSK		0X40
#define BNO055_ACCEL_ANYMOTION_INT_MASK__LEN		1
#define BNO055_ACCEL_ANYMOTION_INT_MASK__REG		INT_MASK_ADDR

/* Accel any motion interrupt msk register*/
#define BNO055_ACCEL_NOMOTION_INT_MASK__POS		7
#define BNO055_ACCEL_NOMOTION_INT_MASK__MSK		0X80
#define BNO055_ACCEL_NOMOTION_INT_MASK__LEN		1
#define BNO055_ACCEL_NOMOTION_INT_MASK__REG		INT_MASK_ADDR

/* Gyro any motion interrupt register*/
#define BNO055_GYRO_ANYMOTION_INT__POS		2
#define BNO055_GYRO_ANYMOTION_INT__MSK		0X04
#define BNO055_GYRO_ANYMOTION_INT__LEN		1
#define BNO055_GYRO_ANYMOTION_INT__REG		INT_ADDR

/* Gyro high rate interrupt register*/
#define BNO055_GYRO_HIGHRATE_INT__POS		3
#define BNO055_GYRO_HIGHRATE_INT__MSK		0X08
#define BNO055_GYRO_HIGHRATE_INT__LEN		1
#define BNO055_GYRO_HIGHRATE_INT__REG		INT_ADDR

/* Accel high g interrupt register*/
#define BNO055_ACCEL_HIGH_G_INT__POS		5
#define BNO055_ACCEL_HIGH_G_INT__MSK		0X20
#define BNO055_ACCEL_HIGH_G_INT__LEN		1
#define BNO055_ACCEL_HIGH_G_INT__REG		INT_ADDR

/* Accel any motion interrupt register*/
#define BNO055_ACCEL_ANYMOTION_INT__POS		6
#define BNO055_ACCEL_ANYMOTION_INT__MSK		0X40
#define BNO055_ACCEL_ANYMOTION_INT__LEN		1
#define BNO055_ACCEL_ANYMOTION_INT__REG		INT_ADDR

/*Accel any motion interrupt register*/
#define BNO055_ACCEL_NOMOTION_INT__POS		7
#define BNO055_ACCEL_NOMOTION_INT__MSK		0X80
#define BNO055_ACCEL_NOMOTION_INT__LEN		1
#define BNO055_ACCEL_NOMOTION_INT__REG		INT_ADDR

/*Accel any motion threshold setting*/
#define BNO055_ACCEL_ANYMOTION_THRES__POS	0
#define BNO055_ACCEL_ANYMOTION_THRES__MSK	0XFF
#define BNO055_ACCEL_ANYMOTION_THRES__LEN	8
#define BNO055_ACCEL_ANYMOTION_THRES__REG	ACCEL_ANYMOTION_THRES_ADDR

/*Accel interrupt setting register*/
#define BNO055_ACCEL_ANYMOTION_DUR_SET__POS		0
#define BNO055_ACCEL_ANYMOTION_DUR_SET__MSK		0X03
#define BNO055_ACCEL_ANYMOTION_DUR_SET__LEN		2
#define BNO055_ACCEL_ANYMOTION_DUR_SET__REG		ACCEL_INT_SETTINGS_ADDR

/* Accel AM/NM axis selection register*/
#define BNO055_ACCEL_ANYMOTION_X_AXIS__POS		2
#define BNO055_ACCEL_ANYMOTION_X_AXIS__MSK		0X04
#define BNO055_ACCEL_ANYMOTION_X_AXIS__LEN		1
#define BNO055_ACCEL_ANYMOTION_X_AXIS__REG		ACCEL_INT_SETTINGS_ADDR

#define BNO055_ACCEL_ANYMOTION_Y_AXIS__POS		3
#define BNO055_ACCEL_ANYMOTION_Y_AXIS__MSK		0X08
#define BNO055_ACCEL_ANYMOTION_Y_AXIS__LEN		1
#define BNO055_ACCEL_ANYMOTION_Y_AXIS__REG		ACCEL_INT_SETTINGS_ADDR

#define BNO055_ACCEL_ANYMOTION_Z_AXIS__POS		4
#define BNO055_ACCEL_ANYMOTION_Z_AXIS__MSK		0X10
#define BNO055_ACCEL_ANYMOTION_Z_AXIS__LEN		1
#define BNO055_ACCEL_ANYMOTION_Z_AXIS__REG		ACCEL_INT_SETTINGS_ADDR

/* Accel high g axis selection register*/
#define BNO055_ACCEL_HIGH_G_X_AXIS__POS		5
#define BNO055_ACCEL_HIGH_G_X_AXIS__MSK		0X20
#define BNO055_ACCEL_HIGH_G_X_AXIS__LEN		1
#define BNO055_ACCEL_HIGH_G_X_AXIS__REG		ACCEL_INT_SETTINGS_ADDR

#define BNO055_ACCEL_HIGH_G_Y_AXIS__POS		6
#define BNO055_ACCEL_HIGH_G_Y_AXIS__MSK		0X40
#define BNO055_ACCEL_HIGH_G_Y_AXIS__LEN		1
#define BNO055_ACCEL_HIGH_G_Y_AXIS__REG		ACCEL_INT_SETTINGS_ADDR

#define BNO055_ACCEL_HIGH_G_Z_AXIS__POS		7
#define BNO055_ACCEL_HIGH_G_Z_AXIS__MSK		0X80
#define BNO055_ACCEL_HIGH_G_Z_AXIS__LEN		1
#define BNO055_ACCEL_HIGH_G_Z_AXIS__REG		ACCEL_INT_SETTINGS_ADDR

/* Accel High g duration setting register*/
#define BNO055_ACCEL_HIGH_G_DUR__POS		0
#define BNO055_ACCEL_HIGH_G_DUR__MSK		0XFF
#define BNO055_ACCEL_HIGH_G_DUR__LEN		8
#define BNO055_ACCEL_HIGH_G_DUR__REG		ACCEL_HIGH_G_DUR_ADDR

/* Accel High g threshold setting register*/
#define BNO055_ACCEL_HIGH_G_THRES__POS		0
#define BNO055_ACCEL_HIGH_G_THRES__MSK		0XFF
#define BNO055_ACCEL_HIGH_G_THRES__LEN		8
#define BNO055_ACCEL_HIGH_G_THRES__REG		ACCEL_HIGH_G_THRES_ADDR

/* Accel no/slow motion threshold setting*/
#define BNO055_ACCEL_SLOW_NO_MOTION_THRES__POS		0
#define BNO055_ACCEL_SLOW_NO_MOTION_THRES__MSK		0XFF
#define BNO055_ACCEL_SLOW_NO_MOTION_THRES__LEN		8
#define BNO055_ACCEL_SLOW_NO_MOTION_THRES__REG		\
ACCEL_NOMOTION_THRES_ADDR

/* Accel no/slow motion enable setting*/
#define BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__POS		0
#define BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__MSK		0X01
#define BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__LEN		1
#define BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__REG		ACCEL_NOMOTION_SET_ADDR

/* Accel no/slow motion duration setting*/
#define BNO055_ACCEL_SLOW_NO_MOTION_DUR__POS		1
#define BNO055_ACCEL_SLOW_NO_MOTION_DUR__MSK		0X7E
#define BNO055_ACCEL_SLOW_NO_MOTION_DUR__LEN		6
#define BNO055_ACCEL_SLOW_NO_MOTION_DUR__REG		ACCEL_NOMOTION_SET_ADDR

/*Gyro interrupt setting register*/
/*Gyro any motion axis setting*/
#define BNO055_GYRO_ANYMOTION_X_AXIS__POS		0
#define BNO055_GYRO_ANYMOTION_X_AXIS__MSK		0X01
#define BNO055_GYRO_ANYMOTION_X_AXIS__LEN		1
#define BNO055_GYRO_ANYMOTION_X_AXIS__REG		GYRO_INT_SETING_ADDR

#define BNO055_GYRO_ANYMOTION_Y_AXIS__POS		1
#define BNO055_GYRO_ANYMOTION_Y_AXIS__MSK		0X02
#define BNO055_GYRO_ANYMOTION_Y_AXIS__LEN		1
#define BNO055_GYRO_ANYMOTION_Y_AXIS__REG		GYRO_INT_SETING_ADDR

#define BNO055_GYRO_ANYMOTION_Z_AXIS__POS		2
#define BNO055_GYRO_ANYMOTION_Z_AXIS__MSK		0X04
#define BNO055_GYRO_ANYMOTION_Z_AXIS__LEN		1
#define BNO055_GYRO_ANYMOTION_Z_AXIS__REG		GYRO_INT_SETING_ADDR

/*Gyro high rate axis setting*/
#define BNO055_GYRO_HIGHRATE_X_AXIS__POS		3
#define BNO055_GYRO_HIGHRATE_X_AXIS__MSK		0X08
#define BNO055_GYRO_HIGHRATE_X_AXIS__LEN		1
#define BNO055_GYRO_HIGHRATE_X_AXIS__REG		GYRO_INT_SETING_ADDR

#define BNO055_GYRO_HIGHRATE_Y_AXIS__POS		4
#define BNO055_GYRO_HIGHRATE_Y_AXIS__MSK		0X10
#define BNO055_GYRO_HIGHRATE_Y_AXIS__LEN		1
#define BNO055_GYRO_HIGHRATE_Y_AXIS__REG		GYRO_INT_SETING_ADDR

#define BNO055_GYRO_HIGHRATE_Z_AXIS__POS		5
#define BNO055_GYRO_HIGHRATE_Z_AXIS__MSK		0X20
#define BNO055_GYRO_HIGHRATE_Z_AXIS__LEN		1
#define BNO055_GYRO_HIGHRATE_Z_AXIS__REG		GYRO_INT_SETING_ADDR

/* Gyro filter setting*/
#define BNO055_GYRO_ANYMOTION_FILTER__POS		6
#define BNO055_GYRO_ANYMOTION_FILTER__MSK		0X40
#define BNO055_GYRO_ANYMOTION_FILTER__LEN		1
#define BNO055_GYRO_ANYMOTION_FILTER__REG		GYRO_INT_SETING_ADDR

#define BNO055_GYRO_HIGHRATE_FILTER__POS		7
#define BNO055_GYRO_HIGHRATE_FILTER__MSK		0X80
#define BNO055_GYRO_HIGHRATE_FILTER__LEN		1
#define BNO055_GYRO_HIGHRATE_FILTER__REG		GYRO_INT_SETING_ADDR

/* Gyro high rate X axis settings*/
#define BNO055_GYRO_HIGHRATE_X_THRES__POS		0
#define BNO055_GYRO_HIGHRATE_X_THRES__MSK		0X1F
#define BNO055_GYRO_HIGHRATE_X_THRES__LEN		5
#define BNO055_GYRO_HIGHRATE_X_THRES__REG		GYRO_HIGHRATE_X_SET_ADDR

#define BNO055_GYRO_HIGHRATE_X_HYST__POS		5
#define BNO055_GYRO_HIGHRATE_X_HYST__MSK		0X60
#define BNO055_GYRO_HIGHRATE_X_HYST__LEN		2
#define BNO055_GYRO_HIGHRATE_X_HYST__REG		GYRO_HIGHRATE_X_SET_ADDR

#define BNO055_GYRO_HIGHRATE_X_DUR__POS		0
#define BNO055_GYRO_HIGHRATE_X_DUR__MSK		0XFF
#define BNO055_GYRO_HIGHRATE_X_DUR__LEN		8
#define BNO055_GYRO_HIGHRATE_X_DUR__REG		GYRO_DUR_X_ADDR

/* Gyro high rate Y axis settings*/
#define BNO055_GYRO_HIGHRATE_Y_THRES__POS		0
#define BNO055_GYRO_HIGHRATE_Y_THRES__MSK		0X1F
#define BNO055_GYRO_HIGHRATE_Y_THRES__LEN		5
#define BNO055_GYRO_HIGHRATE_Y_THRES__REG		GYRO_HIGHRATE_Y_SET_ADDR

#define BNO055_GYRO_HIGHRATE_Y_HYST__POS		5
#define BNO055_GYRO_HIGHRATE_Y_HYST__MSK		0X60
#define BNO055_GYRO_HIGHRATE_Y_HYST__LEN		2
#define BNO055_GYRO_HIGHRATE_Y_HYST__REG		GYRO_HIGHRATE_Y_SET_ADDR

#define BNO055_GYRO_HIGHRATE_Y_DUR__POS		0
#define BNO055_GYRO_HIGHRATE_Y_DUR__MSK		0XFF
#define BNO055_GYRO_HIGHRATE_Y_DUR__LEN		8
#define BNO055_GYRO_HIGHRATE_Y_DUR__REG		GYRO_DUR_Y_ADDR

/* Gyro high rate Z axis settings*/
#define BNO055_GYRO_HIGHRATE_Z_THRES__POS		0
#define BNO055_GYRO_HIGHRATE_Z_THRES__MSK		0X1F
#define BNO055_GYRO_HIGHRATE_Z_THRES__LEN		5
#define BNO055_GYRO_HIGHRATE_Z_THRES__REG		GYRO_HIGHRATE_Z_SET_ADDR

#define BNO055_GYRO_HIGHRATE_Z_HYST__POS		5
#define BNO055_GYRO_HIGHRATE_Z_HYST__MSK		0X60
#define BNO055_GYRO_HIGHRATE_Z_HYST__LEN		2
#define BNO055_GYRO_HIGHRATE_Z_HYST__REG		GYRO_HIGHRATE_Z_SET_ADDR

#define BNO055_GYRO_HIGHRATE_Z_DUR__POS		0
#define BNO055_GYRO_HIGHRATE_Z_DUR__MSK		0XFF
#define BNO055_GYRO_HIGHRATE_Z_DUR__LEN		8
#define BNO055_GYRO_HIGHRATE_Z_DUR__REG		GYRO_DUR_Z_ADDR

/*Gyro any motion threshold setting*/
#define BNO055_GYRO_ANYMOTION_THRES__POS		0
#define BNO055_GYRO_ANYMOTION_THRES__MSK		0X7F
#define BNO055_GYRO_ANYMOTION_THRES__LEN		7
#define BNO055_GYRO_ANYMOTION_THRES__REG		\
GYRO_ANYMOTION_THRES_ADDR

/* Gyro any motion slope sample setting*/
#define BNO055_GYRO_SLOPE_SAMPLES__POS		0
#define BNO055_GYRO_SLOPE_SAMPLES__MSK		0X03
#define BNO055_GYRO_SLOPE_SAMPLES__LEN		2
#define BNO055_GYRO_SLOPE_SAMPLES__REG		GYRO_ANYMOTION_SET_ADDR

/* Gyro awake duration setting*/
#define BNO055_GYRO_AWAKE_DUR__POS		2
#define BNO055_GYRO_AWAKE_DUR__MSK		0X0C
#define BNO055_GYRO_AWAKE_DUR__LEN		2
#define BNO055_GYRO_AWAKE_DUR__REG		GYRO_ANYMOTION_SET_ADDR

/* PAGE1 DATA REGISTERS DEFINITION END*/
/*************************************************/
/**\name GET AND SET BITSLICE FUNCTIONS    */
/*************************************************/
#define BNO055_GET_BITSLICE(regvar, bitname)\
((regvar & bitname##__MSK) >> bitname##__POS)


#define BNO055_SET_BITSLICE(regvar, bitname, val)\
((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))
/*************************************************/
/**\name FUNCTION DECLARATION    */
/*************************************************/
/**************************************************/
/**\name INITIALIZATION AND REVISION ID FUNCTIONS */
/**************************************************/
/*!
 *	@brief
 *	This function is used for initialize
 *	bus read, bus write function pointers,device
 *	address,accel revision id, gyro revision id
 *	mag revision id, software revision id, boot loader
 *	revision id and page id
 *
 *	@param  bno055 - structure pointer
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While changing the parameter of the bno055_t
 *	consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 */
BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055);
/*!
 *	@brief
 *	This API gives data to the given register and
 *	the data is written in the corresponding register address
 *
 *  @param addr : Address of the register
 *	@param data : Data to be written to the register
 *	@param len  : Length of the Data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
*/
BNO055_RETURN_FUNCTION_TYPE bno055_write_register(
_u8 addr, _u8 *data, _u8 len);
/*!
 *	@brief This API reads the data from
 *	the given register address
 *
 *  @param addr : Address of the register
 *  @param data : address of the variable, read value will be kept
 *  @param len  : Length of the data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_register(_u8 addr,
_u8 *data, _u8 len);
/*!
 *	@brief This API reads chip id
 *	from register 0x00 it is a byte of data
 *
 *
 *	@param chip_id : The chip id value 0xA0
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_chip_id(_u8 *chip_id);
/*!
 *	@brief This API reads software revision id
 *	from register 0x04 and 0x05 it is a two byte of data
 *
 *	@param sw_id : The SW revision id
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_sw_rev_id(_u16 *sw_id);
/*!
 *	@brief This API reads page id
 *	from register 0x07 it is a byte of data
 *
 *
 *	@param page_id : The value of page id
 *
 *	PAGE_ZERO -> 0x00
 *	PAGE_ONE  -> 0x01
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_page_id(_u8 *page_id);
/*!
 *	@brief This API used to write
 *	the page id register 0x07
 *
 *	@param page_id : The value of page id
 *
 *	PAGE_ZERO -> 0x00
 *	PAGE_ONE  -> 0x01
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(_u8 page_id);
/*!
 *	@brief This API reads accel revision id
 *	from register 0x01 it is a byte of value
 *
 *	@param accel_rev_id : The accel revision id 0xFB
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_rev_id(
_u8 *accel_rev_id);
/*!
 *	@brief This API reads mag revision id
 *	from register 0x02 it is a byte of value
 *
 *	@param mag_rev_id : The mag revision id 0x32
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_rev_id(
_u8 *mag_rev_id);
/*!
 *	@brief This API reads gyro revision id
 *	from register 0x03 it is a byte of value
 *
 *	@param gyro_rev_id : The gyro revision id 0xF0
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_rev_id(
_u8 *gyro_rev_id);
/*!
 *	@brief This API used to read boot loader revision id
 *	from register 0x06 it is a byte of value
 *
 *	@param bl_rev_id : The boot loader revision id
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_bl_rev_id(
_u8 *bl_rev_id);
/**************************************************/
/**\name ACCEL DATA READ FUNCTIONS */
/**************************************************/
/*!
 *	@brief This API reads acceleration data X values
 *	from register 0x08 and 0x09 it is a two byte data
 *
 *
 *
 *
 *	@param accel_x : The X raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_x(_s16 *accel_x);
/*!
 *	@brief This API reads acceleration data Y values
 *	from register 0x0A and 0x0B it is a two byte data
 *
 *
 *
 *
 *	@param accel_y : The Y raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_y(_s16 *accel_y);
/*!
 *	@brief This API reads acceleration data z values
 *	from register 0x0C and 0x0D it is a two byte data
 *
 *
 *
 *
 *	@param accel_z : The z raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_z(_s16 *accel_z);
/*!
 *	@brief This API reads acceleration data xyz values
 *	from register 0x08 to 0x0D it is a six byte data
 *
 *
 *	@param accel : The value of accel xyz data
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The accel x data
 *	 y        | The accel y data
 *	 z        | The accel z data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_xyz(
struct bno055_accel *accel);
/**************************************************/
/**\name MAG DATA READ FUNCTIONS */
/**************************************************/
/*!
 *	@brief This API reads mag data x values
 *	from register 0x0E and 0x0F it is a two byte data
 *
 *
 *
 *
 *	@param mag_x : The x raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_x(_s16 *mag_x);
/*!
 *	@brief This API reads mag data y values
 *	from register 0x10 and 0x11 it is a two byte data
 *
 *
 *
 *
 *	@param mag_y : The y raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_y(_s16 *mag_y);
/*!
 *	@brief This API reads mag data z values
 *	from register 0x12 and 0x13 it is a two byte data
 *
 *
 *
 *
 *	@param mag_z : The z raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_z(_s16 *mag_z);
/*!
 *	@brief This API reads mag data xyz values
 *	from register 0x0E to 0x13 it is a six byte data
 *
 *
 *	@param mag : The mag xyz values
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The mag x data
 *	 y        | The mag y data
 *	 z        | The mag z data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_xyz(struct bno055_mag *mag);
/**************************************************/
/**\name GYRO DATA READ FUNCTIONS */
/**************************************************/
/*!
 *	@brief This API reads gyro data x values
 *	from register 0x14 and 0x15 it is a two byte data
 *
 *
 *
 *
 *	@param gyro_x : The x raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_x(_s16 *gyro_x);
/*!
 *	@brief This API reads gyro data y values
 *	from register 0x16 and 0x17 it is a two byte data
 *
 *
 *
 *
 *	@param gyro_y : The y raw data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_y(_s16 *gyro_y);
/*!
 *	@brief This API reads gyro data z values
 *	from register 0x18 and 0x19 it is a two byte data
 *
 *	@param gyro_z : The z raw data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_z(_s16 *gyro_z);
/*!
 *	@brief This API reads gyro data xyz values
 *	from register 0x14 to 0x19 it is a six byte data
 *
 *
 *	@param gyro : The value of gyro xyz data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The gyro x data
 *	 y        | The gyro y data
 *	 z        | The gyro z data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_xyz(struct bno055_gyro *gyro);
/**************************************************/
/**\name EULER DATA READ FUNCTIONS */
/**************************************************/
/*!
 *	@brief This API reads gyro data z values
 *	from register 0x1A and 0x1B it is a two byte data
 *
 *	@param euler_h : The raw h data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_h(_s16 *euler_h);
/*!
 *	@brief This API reads Euler data r values
 *	from register 0x1C and 0x1D it is a two byte data
 *
 *	@param euler_r : The raw r data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_r(_s16 *euler_r);
/*!
 *	@brief This API reads Euler data p values
 *	from register 0x1E and 0x1F it is a two byte data
 *
 *	@param euler_p : The raw p data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_p(_s16 *euler_p);
/*!
 *	@brief This API reads Euler data hrp values
 *	from register 0x1A to 0x1F it is a six byte data
 *
 *
 *	@param euler : The Euler hrp data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 h        | The Euler h data
 *	 r        | The Euler r data
 *	 p        | The Euler p data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_hrp(
struct bno055_euler *euler);
/**************************************************/
/**\name QUATERNION DATA READ FUNCTIONS */
/**************************************************/
/*!
 *	@brief This API reads quaternion data w values
 *	from register 0x20 and 0x21 it is a two byte data
 *
 *	@param quaternion_w : The raw w data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_w(_s16 *quaternion_w);
/*!
 *	@brief This API reads quaternion data x values
 *	from register 0x22 and 0x23 it is a two byte data
 *
 *	@param quaternion_x : The raw x data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_x(_s16 *quaternion_x);
/*!
 *	@brief This API reads quaternion data y values
 *	from register 0x24 and 0x25 it is a two byte data
 *
 *	@param quaternion_y : The raw y data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_y(_s16 *quaternion_y);
/*!
 *	@brief This API reads quaternion data z values
 *	from register 0x26 and 0x27 it is a two byte data
 *
 *	@param quaternion_z : The raw z data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_z(_s16 *quaternion_z);
/*!
 *	@brief This API reads Quaternion data wxyz values
 *	from register 0x20 to 0x27 it is a six byte data
 *
 *
 *	@param quaternion : The value of quaternion wxyz data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 w        | The quaternion w data
 *	 x        | The quaternion x data
 *	 y        | The quaternion y data
 *	 z        | The quaternion z data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_wxyz(
struct bno055_quaternion *quaternion);
/**************************************************/
/**\name LINEAR ACCEL DATA READ FUNCTIONS */
/**************************************************/
/*!
 *	@brief This API reads Linear accel data x values
 *	from register 0x29 and 0x2A it is a two byte data
 *
 *	@param linear_accel_x : The raw x data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_x(
_s16 *linear_accel_x);
/*!
 *	@brief This API reads Linear accel data x values
 *	from register 0x2B and 0x2C it is a two byte data
 *
 *	@param linear_accel_y : The raw y data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_y(
_s16 *linear_accel_y);
/*!
 *	@brief This API reads Linear accel data x values
 *	from register 0x2C and 0x2D it is a two byte data
 *
 *	@param linear_accel_z : The raw z data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_z(
_s16 *linear_accel_z);
/*!
 *	@brief This API reads Linear accel data xyz values
 *	from register 0x28 to 0x2D it is a six byte data
 *
 *
 *	@param linear_accel : The value of linear accel xyz data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The linear accel x data
 *	 y        | The linear accel y data
 *	 z        | The linear accel z data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_linear_accel_xyz(
struct bno055_linear_accel *linear_accel);
/**************************************************/
/**\name GRAVITY DATA READ FUNCTIONS */
/**************************************************/
/*!
 *	@brief This API reads gravity data x values
 *	from register 0x2E and 0x2F it is a two byte data
 *
 *	@param gravity_x : The raw x data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_x(_s16 *gravity_x);
/*!
 *	@brief This API reads gravity data y values
 *	from register 0x30 and 0x31 it is a two byte data
 *
 *	@param gravity_y : The raw y data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_y(_s16 *gravity_y);
/*!
 *	@brief This API reads gravity data z values
 *	from register 0x32 and 0x33 it is a two byte data
 *
 *	@param gravity_z : The raw z data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_z(_s16 *gravity_z);
/*!
  *	@brief This API reads gravity data xyz values
 *	from register 0x2E to 0x33 it is a six byte data
 *
 *
 *	@param gravity : The value of gravity xyz data's
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | The gravity x data
 *	 y        | The gravity y data
 *	 z        | The gravity z data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_xyz(
struct bno055_gravity *gravity);
/**************************************************/
/**\name TEMPERATURE DATA READ FUNCTIONS */
/**************************************************/
/*!
 *	@brief This API reads temperature values
 *	from register 0x33 it is a byte data
 *
 *	@param temp : The raw temperature data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_temp_data(_s16 *temp);
#ifdef	BNO055_FLOAT_ENABLE
/********************************************************************/
/**\name FUNCTIONS FOR READING ACCEL DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to meterpersecseq output as float
 *
 *	@param accel_x : The accel x meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_x_msq(
float *accel_x);
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to meterpersecseq output as float
 *
 *	@param accel_y : The accel y meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_y_msq(
float *accel_y);
/*!
 *	@brief This API is used to convert the accel z raw data
 *	to meterpersecseq output as float
 *
 *	@param accel_z : The accel z meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_z_msq(
float *accel_z);
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to millig output as float
 *
 *	@param accel_x : The accel x millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_x_mg(
float *accel_x);
/*!
 *	@brief This API is used to convert the accel y raw data
 *	to millig output as float
 *
 *	@param accel_y : The accel y millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_y_mg(
float *accel_y);
/*!
 *	@brief This API is used to convert the accel z raw data
 *	to millig output as float
 *
 *	@param accel_z : The accel z millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_z_mg(
float *accel_z);
/*!
 *	@brief This API is used to convert the accel xyz raw data
 *	to meterpersecseq output as float
 *
 *	@param accel_xyz : The meterpersecseq data of accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | meterpersecseq data of accel
 *	 y        | meterpersecseq data of accel
 *	 z        | meterpersecseq data of accel
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_xyz_msq(
struct bno055_accel_float *accel_xyz);
/*!
 *	@brief This API is used to convert the accel xyz raw data
 *	to millig output as float
 *
 *	@param accel_xyz : The millig data of accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | millig data of accel
 *	 y        | millig data of accel
 *	 z        | millig data of accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_accel_xyz_mg(
struct bno055_accel_float *accel_xyz);
/********************************************************************/
/**\name FUNCTIONS FOR READING MAG DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/
/*!
 *	@brief This API is used to convert the mag x raw data
 *	to microTesla output as float
 *
 *	@param mag_x : The mag x microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_x_microtesla(
float *mag_x);
/*!
 *	@brief This API is used to convert the mag y raw data
 *	to microTesla output as float
 *
 *	@param mag_y : The mag y microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_y_microtesla(
float *mag_y);
/*!
 *	@brief This API is used to convert the mag z raw data
 *	to microTesla output as float
 *
 *	@param mag_z : The mag z microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_z_microtesla(
float *mag_z);
/*!
 *	@brief This API is used to convert the mag yz raw data
 *	to microTesla output as float
 *
 *	@param mag_xyz_data : The microTesla data of mag xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | microTesla data of mag
 *	  y       | microTesla data of mag
 *	  z       | microTesla data of mag
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_mag_xyz_microtesla(
struct bno055_mag_float *mag_xyz_data);
/********************************************************************/
/**\name FUNCTIONS FOR READING GYRO DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/
/*!
 *	@brief This API is used to convert the gyro x raw data
 *	to dps output as float
 *
 *	@param gyro_x : The gyro x dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_x_dps(
float *gyro_x);
/*!
 *	@brief This API is used to convert the gyro x raw data
 *	to rps output as float
 *
 *	@param gyro_x : The gyro x dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_x_rps(
float *gyro_x);
/*!
 *	@brief This API is used to convert the gyro y raw data
 *	to dps output as float
 *
 *	@param gyro_y : The gyro y dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_y_dps(
float *gyro_y);
/*!
 *	@brief This API is used to convert the gyro y raw data
 *	to rps output as float
 *
 *	@param gyro_y : The gyro y dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_y_rps(
float *gyro_y);
/*!
 *	@brief This API is used to convert the gyro z raw data
 *	to dps output as float
 *
 *	@param gyro_z : The gyro z dps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_z_dps(
float *gyro_z);
/*!
 *	@brief This API is used to convert the gyro z raw data
 *	to rps output as float
 *
 *	@param gyro_z : The gyro z rps float data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_z_rps(
float *gyro_z);
/*!
 *	@brief This API is used to convert the gyro xyz raw data
 *	to dps output as float
 *
 *	@param gyro_xyz_data : The dps data of gyro xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | dps data of gyro
 *	  y       | dps data of gyro
 *	  z       | dps data of gyro
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_xyz_dps(
struct bno055_gyro_float *gyro_xyz_data);
/*!
 *	@brief This API is used to convert the gyro xyz raw data
 *	to rps output as float
 *
 *	@param gyro_xyz_data : The rps data of gyro xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | rps data of gyro
 *	  y       | rps data of gyro
 *	  z       | rps data of gyro
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gyro_xyz_rps(
struct bno055_gyro_float *gyro_xyz_data);
/********************************************************************/
/**\name FUNCTIONS FOR READING EULER DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/
/*!
 *	@brief This API is used to convert the Euler h raw data
 *	to degree output as float
 *
 *	@param euler_h : The float value of Euler h degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_h_deg(
float *euler_h);
/*!
 *	@brief This API is used to convert the Euler h raw data
 *	to radians output as float
 *
 *	@param euler_h : The float value of Euler h radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_h_rad(
float *euler_h);
/*!
 *	@brief This API is used to convert the Euler r raw data
 *	to degree output as float
 *
 *	@param euler_r : The float value of Euler r degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_r_deg(
float *euler_r);
/*!
 *	@brief This API is used to convert the Euler r raw data
 *	to radians output as float
 *
 *	@param euler_r : The float value of Euler r radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_r_rad(
float *euler_r);
/*!
 *	@brief This API is used to convert the Euler p raw data
 *	to degree output as float
 *
 *	@param euler_p : The float value of Euler p degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_p_deg(
float *euler_p);
/*!
 *	@brief This API is used to convert the Euler p raw data
 *	to radians output as float
 *
 *	@param euler_p : The float value of Euler p radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_p_rad(
float *euler_p);
/*!
 *	@brief This API is used to convert the Euler hrp raw data
 *	to degree output as float
 *
 *	@param euler_hpr : The degree data of Euler hrp
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  h       | degree data of Euler
 *	  r       | degree data of Euler
 *	  p       | degree data of Euler
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_hpr_deg(
struct bno055_euler_float *euler_hpr);
/*!
 *	@brief This API is used to convert the Euler xyz raw data
 *	to radians output as float
 *
 *	@param euler_hpr : The radians data of Euler hrp
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  h       | radians data of Euler
 *	  r       | radians data of Euler
 *	  p       | radians data of Euler
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_euler_hpr_rad(
struct bno055_euler_float *euler_hpr);
/***************************************************************************/
/**\name FUNCTIONS FOR READING LINEAR ACCEL DATA OUTPUT AS FLOAT PRECISION */
/**************************************************************************/
/*!
 *	@brief This API is used to convert the linear
 *	accel x raw data to meterpersecseq output as float
 *
 *	@param linear_accel_x : The float value of linear accel x meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_x_msq(
float *linear_accel_x);
/*!
 *	@brief This API is used to convert the linear
 *	accel y raw data to meterpersecseq output as float
 *
 *	@param linear_accel_y : The float value of linear accel y meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_y_msq(
float *linear_accel_y);
/*!
 *	@brief This API is used to convert the linear
 *	accel z raw data to meterpersecseq output as float
 *
 *	@param linear_accel_z : The float value of linear accel z meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_z_msq(
float *linear_accel_z);
/*!
 *	@brief This API is used to convert the linear accel xyz raw data
 *	to meterpersecseq output as float
 *
 *	@param linear_accel_xyz : The meterpersecseq data of linear accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | meterpersecseq data of linear accel
 *	  y       | meterpersecseq data of linear accel
 *	  z       | meterpersecseq data of linear accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_xyz_msq(
struct bno055_linear_accel_float *linear_accel_xyz);
/********************************************************************/
/**\name FUNCTIONS FOR READING GRAVITY DATA OUTPUT AS FLOAT PRECISION */
/********************************************************************/
/*!
 *	@brief This API is used to convert the gravity
 *	x raw data to meterpersecseq output as float
 *
 *	@param gravity_x : The float value of gravity x meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_x_msq(
float *gravity_x);
/*!
 *	@brief This API is used to convert the gravity
 *	y raw data to meterpersecseq output as float
 *
 *	@param gravity_y : The float value of gravity y meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_y_msq(
float *gravity_y);
/*!
 *	@brief This API is used to convert the gravity
 *	z raw data to meterpersecseq output as float
 *
 *	@param gravity_z : The float value of gravity z meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_float_z_msq(
float *gravity_z);
/*!
 *	@brief This API is used to convert the gravity xyz raw data
 *	to meterpersecseq output as float
 *
 *	@param gravity_xyz : The meterpersecseq data of gravity xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	  x       | meterpersecseq data of gravity
 *	  y       | meterpersecseq data of gravity
 *	  z       | meterpersecseq data of gravity
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_gravity_xyz_msq(
struct bno055_gravity_float *gravity_xyz);
/**************************************************************************/
/**\name FUNCTIONS FOR READING TEMPERATURE DATA OUTPUT AS FLOAT PRECISION */
/*************************************************************************/
/*!
 *	@brief This API is used to convert the temperature
 *	data to Fahrenheit output as float
 *
 *	@param temp : The float value of temperature Fahrenheit
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_temp_fahrenheit(
float *temp);
/*!
 *	@brief This API is used to convert the temperature
 *	data to Celsius output as float
 *
 *	@param temp : The float value of temperature Celsius
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_temp_celsius(
float *temp);
#endif
#ifdef	BNO055_DOUBLE_ENABLE
/**************************************************************************/
/**\name FUNCTIONS FOR READING ACCEL DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to meterpersecseq output as double
 *
 *	@param accel_x : The accel x meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_x_msq(
double *accel_x);
/*!
 *	@brief This API is used to convert the accel y raw data
 *	to meterpersecseq output as double
 *
 *	@param accel_y : The accel y meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_y_msq(
double *accel_y);
/*!
 *	@brief This API is used to convert the accel z raw data
 *	to meterpersecseq output as double
 *
 *	@param accel_z : The accel z meterpersecseq data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_z_msq(
double *accel_z);
/*!
 *	@brief This API is used to convert the accel x raw data
 *	to millig output as double
 *
 *	@param accel_x : The accel x millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_x_mg(
double *accel_x);
/*!
 *	@brief This API is used to convert the accel y raw data
 *	to millig output as double
 *
 *	@param accel_y : The accel y millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_y_mg(
double *accel_y);
/*!
 *	@brief This API is used to convert the accel z raw data
 *	to millig output as double
 *
 *	@param accel_z : The accel z millig data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_z_mg(
double *accel_z);
/*!
 *	@brief This API is used to convert the accel xyz raw data
 *	to meterpersecseq output as double
 *
 *	@param accel_xyz : The meterpersecseq data of accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | meterpersecseq data of accel
 *	 y        | meterpersecseq data of accel
 *	 z        | meterpersecseq data of accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_xyz_msq(
struct bno055_accel_double *accel_xyz);
/*!
 *	@brief This API is used to convert the accel xyz raw data
 *	to millig output as double
 *
 *	@param accel_xyz : The millig data of accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | millig data of accel
 *	 y        | millig data of accel
 *	 z        | millig data of accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_accel_xyz_mg(
struct bno055_accel_double *accel_xyz);
/**************************************************************************/
/**\name FUNCTIONS FOR READING MAG DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/
/*!
 *	@brief This API is used to convert the mag x raw data
 *	to microTesla output as double
 *
 *	@param mag_x : The mag x microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_x_microtesla(
double *mag_x);
/*!
 *	@brief This API is used to convert the mag y raw data
 *	to microTesla output as double
 *
 *	@param mag_y : The mag y microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_y_microtesla(
double *mag_y);
/*!
 *	@brief This API is used to convert the mag z raw data
 *	to microTesla output as double
 *
 *	@param mag_z : The mag z microTesla data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_z_microtesla(
double *mag_z);
/*!
 *	@brief This API is used to convert the mag yz raw data
 *	to microTesla output as double
 *
 *	@param mag_xyz : The microTesla data of mag xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | microTesla data of mag
 *	 y        | microTesla data of mag
 *	 z        | microTesla data of mag
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_mag_xyz_microtesla(
struct bno055_mag_double *mag_xyz);
/**************************************************************************/
/**\name FUNCTIONS FOR READING GYRO DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/
/*!
 *	@brief This API is used to convert the gyro x raw data
 *	to dps output as double
 *
 *	@param gyro_x : The gyro x dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_x_dps(
double *gyro_x);
/*!
 *	@brief This API is used to convert the gyro y raw data
 *	to dps output as double
 *
 *	@param gyro_y : The gyro y dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_y_dps(
double *gyro_y);
/*!
 *	@brief This API is used to convert the gyro z raw data
 *	to dps output as double
 *
 *	@param gyro_z : The gyro z dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_z_dps(
double *gyro_z);
/*!
 *	@brief This API is used to convert the gyro x raw data
 *	to rps output as double
 *
 *	@param gyro_x : The gyro x dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_x_rps(
double *gyro_x);
/*!
 *	@brief This API is used to convert the gyro y raw data
 *	to rps output as double
 *
 *	@param gyro_y : The gyro y dps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_y_rps(
double *gyro_y);
/*!
 *	@brief This API is used to convert the gyro z raw data
 *	to rps output as double
 *
 *	@param gyro_z : The gyro z rps double data
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_z_rps(
double *gyro_z);
/*!
 *	@brief This API is used to convert the gyro xyz raw data
 *	to dps output as double
 *
 *	@param gyro_xyz : The dps data of gyro xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | dps data of gyro
 *	 y        | dps data of gyro
 *	 z        | dps data of gyro
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_xyz_dps(
struct bno055_gyro_double *gyro_xyz);
/*!
 *	@brief This API is used to convert the gyro xyz raw data
 *	to rps output as double
 *
 *	@param gyro_xyz : The rps data of gyro xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | rps data of gyro
 *	 y        | rps data of gyro
 *	 z        | rps data of gyro
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gyro_xyz_rps(
struct bno055_gyro_double *gyro_xyz);
/**************************************************************************/
/**\name FUNCTIONS FOR READING EULER DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/
/*!
 *	@brief This API is used to convert the Euler h raw data
 *	to degree output as double
 *
 *	@param euler_h : The double value of Euler h degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_h_deg(
double *euler_h);
/*!
 *	@brief This API is used to convert the Euler p raw data
 *	to degree output as double
 *
 *	@param euler_p : The double value of Euler p degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_p_deg(
double *euler_p);
/*!
 *	@brief This API is used to convert the Euler r raw data
 *	to degree output as double
 *
 *	@param euler_r : The double value of Euler r degree
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_r_deg(
double *euler_r);
/*!
 *	@brief This API is used to convert the Euler h raw data
 *	to radians output as double
 *
 *	@param euler_h : The double value of Euler h radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_h_rad(
double *euler_h);
/*!
 *	@brief This API is used to convert the Euler p raw data
 *	to radians output as double
 *
 *	@param euler_p : The double value of Euler p radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_p_rad(
double *euler_p);
/*!
 *	@brief This API is used to convert the Euler r raw data
 *	to radians output as double
 *
 *	@param euler_r : The double value of Euler r radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_r_rad(
double *euler_r);
/*!
 *	@brief This API is used to convert the Euler hpr raw data
 *	to degree output as double
 *
 *	@param euler_hpr : The degree data of Euler hpr
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 h        | degree data of Euler
 *	 r        | degree data of Euler
 *	 p        | degree data of Euler
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_hpr_deg(
struct bno055_euler_double *euler_hpr);
/*!
 *	@brief This API is used to convert the Euler hpr raw data
 *	to radians output as double
 *
 *	@param euler_hpr : The radians data of Euler hpr
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 h        | radians data of Euler
 *	 r        | radians data of Euler
 *	 p        | radians data of Euler
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_euler_hpr_rad(
struct bno055_euler_double *euler_hpr);
/****************************************************************************/
/**\name FUNCTIONS FOR READING LINEAR ACCEL DATA OUTPUT AS DOUBLE PRECISION */
/****************************************************************************/
/*!
 *	@brief This API is used to convert the linear
 *	accel x raw data to meterpersecseq output as double
 *
 *	@param linear_accel_x : The double value of linear accel x meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_x_msq(
double *linear_accel_x);
/*!
 *	@brief This API is used to convert the linear
 *	accel y raw data to meterpersecseq output as double
 *
 *	@param linear_accel_y : The double value of linear accel y meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_y_msq(
double *linear_accel_y);
/*!
 *	@brief This API is used to convert the linear
 *	accel z raw data to meterpersecseq output as double
 *
 *	@param linear_accel_z : The double value of linear accel z meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_z_msq(
double *linear_accel_z);
/*!
 *	@brief This API is used to convert the linear accel xyz raw data
 *	to meterpersecseq output as double
 *
 *	@param linear_accel_xyz : The meterpersecseq data of linear accel xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | meterpersecseq data of linear accel
 *	 y        | meterpersecseq data of linear accel
 *	 z        | meterpersecseq data of linear accel
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_linear_accel_xyz_msq(
struct bno055_linear_accel_double *linear_accel_xyz);
/**************************************************************************/
/**\name FUNCTIONS FOR READING GRAVITY DATA OUTPUT AS DOUBLE PRECISION */
/*************************************************************************/
/*!
 *	@brief This API is used to convert the gravity
 *	x raw data to meterpersecseq output as double
 *
 *	@param gravity_x : The double value of gravity x meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_x_msq(
double *gravity_x);
/*!
 *	@brief This API is used to convert the gravity
 *	y raw data to meterpersecseq output as double
 *
 *	@param gravity_y : The double value of gravity y meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_y_msq(
double *gravity_y);
/*!
 *	@brief This API is used to convert the gravity
 *	z raw data to meterpersecseq output as double
 *
 *	@param gravity_z : The double value of gravity z meterpersecseq
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */

BNO055_RETURN_FUNCTION_TYPE bno055_convert_gravity_double_z_msq(
double *gravity_z);
/*!
 *	@brief This API is used to convert the gravity xyz raw data
 *	to meterpersecseq output as double
 *
 *	@param gravity_xyz : The meterpersecseq data of gravity xyz
 *
 *	Parameter |    result
 *  --------- | -----------------
 *	 x        | meterpersecseq data of gravity
 *	 y        | meterpersecseq data of gravity
 *	 z        | meterpersecseq data of gravity
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_gravity_xyz_msq(
struct bno055_gravity_double *gravity_xyz);
/**************************************************************************/
/**\name FUNCTIONS FOR READING TEMPERATURE DATA OUTPUT AS DOUBLE PRECISION*/
/*************************************************************************/
/*!
 *	@brief This API is used to convert the temperature
 *	data to Fahrenheit output as double
 *
 *	@param temp : The double value of temperature Fahrenheit
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_temp_fahrenheit(
double *temp);
/*!
 *	@brief This API is used to convert the temperature
 *	data to Celsius output as double
 *
 *	@param temp : The double value of temperature Celsius
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_convert_double_temp_celsius(
double *temp);
#endif
/**************************************************************************/
/**\name FUNCTIONS FOR READING ACCEL,MAG,GYRO AND SYTEM CALIBRATION STATUS*/
/*************************************************************************/
/*!
 *	@brief This API used to read
 *	mag calibration status from register from 0x35 bit 0 and 1
 *
 *	@param mag_calib : The value of mag calib status
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_calib_stat(
_u8 *mag_calib);
/*!
 *	@brief This API used to read
 *	accel calibration status from register from 0x35 bit 2 and 3
 *
 *	@param accel_calib : The value of accel calib status
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_calib_stat(
_u8 *accel_calib);
/*!
 *	@brief This API used to read
 *	gyro calibration status from register from 0x35 bit 4 and 5
 *
 *	@param gyro_calib : The value of gyro calib status
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_calib_stat(
_u8 *gyro_calib);
/*!
 *	@brief This API used to read
 *	system calibration status from register from 0x35 bit 6 and 7
 *
 *	@param sys_calib : The value of system calib status
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_calib_stat(
_u8 *sys_calib);
/******************************************************************/
/**\name FUNCTIONS FOR READING ACCEL,MAG,GYRO AND SYTEM SELF TEST */
/******************************************************************/
/*!
 *	@brief This API used to read
 *	self test of accel from register from 0x36 bit 0
 *
 *	@param selftest_accel : The value of self test of accel
 *
 *     selftest_accel|  result
 *   --------------- | ---------------------
 *     0x00          | indicates test failed
 *     0x01          | indicated test passed
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_accel(
_u8 *selftest_accel);
/*!
 *	@brief This API used to read
 *	self test of mag from register from 0x36 bit 1
 *
 *	@param selftest_mag : The value of self test of mag
 *
 *     selftest_mag  |  result
 *   --------------  | ---------------------
 *     0x00          | indicates test failed
 *     0x01          | indicated test passed
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_mag(
_u8 *selftest_mag);
/*!
 *	@brief This API used to read
 *	self test of gyro from register from 0x36 bit 2
 *
 *	@param selftest_gyro : The value of self test of gyro
 *
 *     selftest_gyro  |  result
 *   ---------------  | ---------------------
 *     0x00           | indicates test failed
 *     0x01           | indicated test passed
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_gyro(
_u8 *selftest_gyro);
/*!
 *	@brief This API used to read
 *	self test of micro controller from register from 0x36 bit 3
 *
 *	@param selftest_mcu : The value of self test of micro controller
 *
 *     selftest_mcu  |  result
 *   --------------- | ---------------------
 *     0x00          | indicates test failed
 *     0x01          | indicated test passed
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest_mcu(
_u8 *selftest_mcu);
/*****************************************************/
/**\name FUNCTIONS FOR READING GYRO INTERRUPT STATUS */
/*****************************************************/
/*!
 *	@brief This API used to read the status of
 *	gyro anymotion interrupt from register from 0x37 bit 2
 *
 *	@param gyro_anymotion : The value of gyro anymotion interrupt
 *
 *     gyro_anymotion  |  result
 *    --------------   | ---------------------
 *     0x00            | indicates no interrupt triggered
 *     0x01            | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro anymotion interrupt can be enabled
 *	by the following APIs
 *
 *	bno055_set_intmsk_gyro_anymotion()
 *
 *	bno055_set_int_gyro_anymotion()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_stat_gyro_anymotion(
_u8 *gyro_anymotion);
/*!
 *	@brief This API used to read the status of
 *	gyro highrate interrupt from register from 0x37 bit 3
 *
 *	@param gyro_highrate : The value of gyro highrate interrupt
 *
 *     gyro_highrate  |  result
 *    --------------  | ---------------------
 *     0x00           | indicates no interrupt triggered
 *     0x01           | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate interrupt can be configured
 *			by the following APIs
 *
 *	bno055_set_int_mask_gyro_highrate()
 *
 *	bno055_set_int_gyro_highrate()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_stat_gyro_highrate(
_u8 *gyro_highrate);
/*****************************************************/
/**\name FUNCTIONS FOR READING ACCEL INTERRUPT STATUS */
/*****************************************************/
/*!
 *	@brief This API used to read the status of
 *	accel highg interrupt from register from 0x37 bit 5
 *
 *	@param accel_high_g : The value of accel highg interrupt
 *
 *     accel_high_g  |  result
 *    -------------  | -------------------------------
 *     0x00          | indicates no interrupt triggered
 *     0x01          | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel highg interrupt can be configured
 *			by the following APIs
 *
 *	bno055_set_int_mask_accel_high_g()
 *
 *	bno055_set_int_accel_high_g()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_stat_accel_highg(
_u8 *accel_high_g);
/*!
 *	@brief This API used to read the status of
 *	accel anymotion interrupt from register from 0x37 bit 6
 *
 *	@param accel_anymotion : The value of accel anymotion interrupt
 *
 *     accel_anymotion  |  result
 *    --------------  | -------------------------------
 *     0x00           | indicates no interrupt triggered
 *     0x01           | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel anymotion interrupt can be configured
 *			by the following APIs
 *
 *	bno055_set_int_mask_accel_anymotion()
 *
 *	bno055_set_int_accel_anymotion()
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_stat_accel_anymotion(
_u8 *accel_anymotion);
/*!
 *	@brief This API used to read the status of
 *	accel nomotion/slowmotion interrupt from register from 0x37 bit 6
 *
 *	@param accel_nomotion : The value of accel nomotion/slowmotion interrupt
 *
 *     accel_nomotion   |  result
 *    --------------  | -------------------------------
 *     0x00           | indicates no interrupt triggered
 *     0x01           | indicates interrupt triggered
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel nomotion/slowmotion interrupt can be configured
 *			by the following APIs
 *
 *	bno055_set_int_mask_accel_nomotion()
 *
 *	bno055_set_int_accel_nomotion()
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_interrupt_stat_accel_nomotion(
_u8 *accel_nomotion);
/**************************************************************************/
/**\name FUNCTIONS FOR READING SYSTEM CLOCK, STATUS AND ERROR CODE */
/*************************************************************************/
/*!
 *	@brief This API is used to read status of main clock
 *	from the register 0x38 bit 0
 *
 *	@param stat_main_clk : the status of main clock
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_stat_main_clk(
_u8 *stat_main_clk);
/*!
 *	@brief This API is used to read system status
 *	code from the register 0x39 it is a byte of data
 *
 *	@param sys_stat : the status of system
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_stat_code(
_u8 *sys_stat);
/*!
 *	@brief This API is used to read system error
 *	code from the register 0x3A it is a byte of data
 *
 *	@param sys_error : The value of system error code
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_error_code(
_u8 *sys_error);
/********************************************/
/**\name FUNCTIONS FOR ACCEL UNIT SELECTION */
/********************************************/
/*!
 *	@brief This API used to read the accel unit
 *	from register from 0x3B bit 0
 *
 *	@param accel_unit : The value of accel unit
 *
 *    accel_unit |   result
 *   ----------  | ---------------
 *        0x00   | ACCEL_UNIT_MSQ
 *        0x01   | ACCEL_UNIT_MG
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_unit(
_u8 *accel_unit);
/*!
 *	@brief This API used to write the accel unit
 *	from register from 0x3B bit 0
 *
 *	@param accel_unit : The value of accel unit
 *
 *    accel_unit   |   result
 *   ----------  | ---------------
 *        0x00   | ACCEL_UNIT_MSQ
 *        0x01   | ACCEL_UNIT_MG
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_unit(_u8 accel_unit);
/********************************************/
/**\name FUNCTIONS FOR GYRO UNIT SELECTION */
/********************************************/
/*!
 *	@brief This API used to read the gyro unit
 *	from register from 0x3B bit 1
 *
 *	@param gyro_unit : The value of accel unit
 *
 *	gyro_unit  |  result
 *	---------  | -----------
 *    0x00     | GYRO_UNIT_DPS
 *    0x01     | GYRO_UNIT_RPS
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_unit(_u8 *gyro_unit);
/*!
 *	@brief This API used to write the gyro unit
 *	from register from 0x3B bit 1
 *
 *	@param gyro_unit : The value of accel unit
 *
 *	gyro_unit   |  result
 *	---------  | -----------
 *    0x00     | GYRO_UNIT_DPS
 *    0x01     | GYRO_UNIT_RPS
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_unit(_u8 gyro_unit);
/********************************************/
/**\name FUNCTIONS FOR EULER UNIT SELECTION */
/********************************************/
/*!
 *	@brief This API used to read the Euler unit
 *	from register from 0x3B bit 2
 *
 *	@param euler_unit : The value of accel unit
 *
 *    euler_unit| result
 *   ---------- | -----------
 *      0x00    | EULER_UNIT_DEG
 *      0x01    | EULER_UNIT_RAD
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_unit(_u8 *euler_unit);
/*!
 *	@brief This API used to write the Euler unit
 *	from register from 0x3B bit 2
 *
 *	@param euler_unit : The value of Euler unit
 *
 *    euler_unit  | result
 *   ---------- | -----------
 *      0x00    | EULER_UNIT_DEG
 *      0x01    | EULER_UNIT_RAD
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_unit(_u8 euler_unit);
/********************************************/
/**\name FUNCTIONS FOR TILT UNIT SELECTION */
/********************************************/
/*!
 *	@brief This API used to write the tilt unit
 *	from register from 0x3B bit 3
 *
 *	@param tilt_unit : The value of tilt unit
 *
 *    tilt_unit  | result
 *   ----------- | ---------
 *     0x00      | degrees
 *     0x01      | radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_tilt_unit(
_u8 *tilt_unit);
/*!
 *	@brief This API used to write the tilt unit
 *	from register from 0x3B bit 3
 *
 *	@param tilt_unit : The value of tilt unit
 *
 *    tilt_unit  | result
 *   ----------- | ---------
 *     0x00      | degrees
 *     0x01      | radians
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 *
 *  \return Communication results
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_tilt_unit(_u8 tilt_unit);
/**************************************************/
/**\name FUNCTIONS FOR TEMPERATURE UNIT SELECTION */
/**************************************************/
/*!
 *	@brief This API used to read the temperature unit
 *	from register from 0x3B bit 4
 *
 *	@param temp_unit : The value of temperature unit
 *
 *    temp_unit  |  result
 *   ----------- | --------------
 *      0x00     | TEMP_UNIT_CEL
 *      0x01     | TEMP_UNIT_FARENHEIT
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_unit(
_u8 *temp_unit);
/*!
 *	@brief This API used to write the temperature unit
 *	from register from 0x3B bit 4
 *
 *	@param temp_unit : The value of temperature unit
 *
 *    temp_unit  |  result
 *   ----------- | --------------
 *      0x00     | TEMP_UNIT_CEL
 *      0x01     | TEMP_UNIT_FARENHEIT
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_unit(
_u8 temp_unit);
/**************************************************/
/**\name FUNCTIONS FOR DATA OUTPUT FORMAT SELECT */
/**************************************************/
/*!
 *	@brief This API used to read the current selected orientation mode
 *	from register from 0x3B bit 7
 *
 *	@param data_output_format : The value of data output format
 *
 *	  data_output_format  | result
 *   -------------------- | --------
 *    0x00                | Windows
 *    0x01                | Android
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_data_output_format(
_u8 *data_output_format);
/*!
 *	@brief This API used to write the current selected orientation mode
 *	from register from 0x3B bit 7
 *
 *	@param data_output_format : The value of data output format
 *
 *	  data_output_format  | result
 *   -------------------- | --------
 *    0x00                | Windows
 *    0x01                | Android
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_data_output_format(
_u8 data_output_format);
/***********************************************************/
/**\name FUNCTIONS FOR DATA SELECT OF ACCEL.MAG,GYRO,EULER,
QUATERNION, LINEAR ACCEL, GRAVITY AND TEMPERATURE **********/
/************************************************************/
/*!
 *	@brief This API used to read the accel data select
 *	from register from 0x3C bit 0
 *
 *	@param accel_data_select : The value of accel data select
 *
 *    accel_data_select  | result
 *   ------------------- | --------
 *      0x01             | ENABLED
 *      0x00             | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_data_select(
_u8 *accel_data_select);
/*!
 *	@brief This API used to write the accel data select
 *	from register from 0x3C bit 0
 *
 *	@param accel_data_select : The value of accel data select
 *
 *    accel_data_select  | result
 *   ------------------- | --------
 *      0x01             | ENABLED
 *      0x00             | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_data_select(
_u8 accel_data_select);
/*!
 *	@brief This API used to read the mag data select
 *	from register from 0x3C bit 1
 *
 *	@param mag_data_select : The value of mag data select
 *
 *	 mag_data_select   | result
 *   ----------------  | --------
 *      0x01           | ENABLED
 *      0x00           | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_data_select(
_u8 *mag_data_select);
/*!
 *	@brief This API used to write the mag data select
 *	from register from 0x3C bit 1
 *
 *	@param mag_data_select : The value of mag data select
 *
 *	 mag_data_select   | result
 *   ----------------  | --------
 *      0x01           | ENABLED
 *      0x00           | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_data_select(
_u8 mag_data_select);
/*!
 *	@brief This API used to read the gyro data select
 *	from register from 0x3C bit 2
 *
 *	@param gyro_data_select : The value of gyro data select
 *
 *	 gyro_data_select  | result
 *   ----------------  | --------
 *      0x01           | ENABLED
 *      0x00           | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_data_select(
_u8 *gyro_data_select);
/*!
 *	@brief This API used to write the gyro data select
 *	from register from 0x3C bit 2
 *
 *	@param gyro_data_select : The value of gyro data select
 *
 *	 gyro_data_select  | result
 *   ----------------  | --------
 *      0x01           | ENABLED
 *      0x00           | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_data_select(
_u8 gyro_data_select);
/*!
 *	@brief This API used to read the Euler data select
 *	from register from 0x3C bit 3
 *
 *	@param euler_data_select : The value of Euler data select
 *
 *	 euler_data_select   | result
 *   ------------------  | ----------
 *      0x01             | ENABLED
 *      0x00             | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_data_select(
_u8 *euler_data_select);
/*!
 *	@brief This API used to write the Euler data select
 *	from register from 0x3C bit 3
 *
 *	@param euler_data_select : The value of Euler data select
 *
 *	 euler_data_select   | result
 *   ------------------  | ----------
 *      0x01             | ENABLED
 *      0x00             | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_data_select(
_u8 euler_data_select);
/*!
 *	@brief This API used to read the quaternion data select
 *	from register from 0x3C bit 4
 *
 *	@param quaternion_data_select : The value of quaternion data select
 *
 *	 quaternion_data_select   |  result
 *   ------------------------ | ----------
 *         0x01               | ENABLED
 *         0x00               | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_quaternion_data_select(
_u8 *quaternion_data_select);
/*!
 *	@brief This API used to write the quaternion data select
 *	from register from 0x3C bit 4
 *
 *	@param quaternion_data_select : The value of quaternion data select
 *
 *	 quaternion_data_select   |  result
 *   ------------------------ | ----------
 *         0x01               | ENABLED
 *         0x00               | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_quaternion_data_select(
_u8 quaternion_data_select);
/*!
 *	@brief This API used to read the linear accel data select
 *	from register from 0x3C bit 5
 *
 *	@param linear_accel_data_select : The value of linear accel data select
 *
 *	 linear_accel_data_select   | result
 *   -------------------------- | ----------
 *             0x01             | ENABLED
 *             0x00             | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_linear_accel_data_select(
_u8 *linear_accel_data_select);
/*!
 *	@brief This API used to write the linear accel data select
 *	from register from 0x3C bit 5
 *
 *	@param linear_accel_data_select : The value of linear accel data select
 *
 *	 linear_accel_data_select   | result
 *   -------------------------- | ----------
 *             0x01             | ENABLED
 *             0x00             | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_linear_accel_data_select(
_u8 linear_accel_data_select);
/*!
 *	@brief This API used to read the gravity data select
 *	from register from 0x3C bit 6
 *
 *	@param gravity_data_select : The value of gravity data select
 *
 *	 gravity_data_select  | result
 *   -------------------  | ----------
 *          0x01          | ENABLED
 *          0x00          | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gravity_data_select(
_u8 *gravity_data_select);
/*!
 *	@brief This API used to write the gravity data select
 *	from register from 0x3C bit 6
 *
 *	@param gravity_data_select : The value of gravity data select
 *
 *	 gravity_data_select  | result
 *   -------------------  | ----------
 *          0x01          | ENABLED
 *          0x00          | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gravity_data_select(
_u8 gravity_data_select);
/*!
 *	@brief This API used to read the temperature data select
 *	from register from 0x3C bit 7
 *
 *	@param temp_data_select : The value of temperature data select
 *
 *	 temp_data_select  | result
 *   ----------------  | ----------
 *      0x01           | ENABLED
 *      0x00           | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_data_select(
_u8 *temp_data_select);
/*!
 *	@brief This API used to write the temperature data select
 *	from register from 0x3C bit 7
 *
 *	@param temp_data_select : The value of temperature data select
 *
 *	 temp_data_select  | result
 *   ----------------  | ----------
 *      0x01           | ENABLED
 *      0x00           | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_data_select(
_u8 temp_data_select);
/**************************************************/
/**\name FUNCTIONS FOR DATA OUTPUT RATE  */
/**************************************************/
/*!	@brief This API used to read the output data rate
 *	from register from 0x3D bit 4 to 6
 *
 *	@param output_data_rate : The value of output data rate
 *
 *   output_data_rate |  result
 *   ---------------- |---------------
 *      0x00          | FASTEST_MODE_1
 *      0x01          | FASTEST_MODE_2
 *      0x02          | GAME_MODE
 *      0x03          | UI_MODE
 *      0x04          | FASTEST_MODE_2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note actual output data rates depend on the
 *	fusion operation mode selected
 *	the mapping between operating modes and output data rates
 *	refer the data sheet Table 3-12 to Table 3-16
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_output_data_rate(
_u8 *output_data_rate);
/*!	@brief This API used to write the output data rate
 *	from register from 0x3D bit 4 to 6
 *
 *	@param output_data_rate : The value of output data rate
 *
 *   output_data_rate |  result
 *   ---------------- |---------------
 *      0x00          | FASTEST_MODE_1
 *      0x01          | FASTEST_MODE_2
 *      0x02          | GAME_MODE
 *      0x03          | UI_MODE
 *      0x04          | FASTEST_MODE_
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Actual output data rates depend on the
 *	fusion operation mode selected
 *	the mapping between operating modes and output data rates
 *	refer the data sheet Table 3-12 to Table 3-16
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_output_data_rate(
_u8 output_data_rate);
/**************************************************/
/**\name FUNCTIONS FOR DATA OPERATION MODE  */
/**************************************************/
/*!	@brief This API used to read the operation mode
 *	from register from 0x3D bit 0 to 3
 *
 *	@param operation_mode : The value of operation mode
 *
 * operation_mode |      result           | comments
 * ----------|----------------------------|----------------------------
 *  0x00     | OPERATION_MODE_CONFIG      | Configuration mode
 *  0x01     | OPERATION_MODE_ACCONLY     | Reads accel data alone
 *  0x02     | OPERATION_MODE_MAGONLY     | Reads mag data alone
 *  0x03     | OPERATION_MODE_GYRONLY     | Reads gyro data alone
 *  0x04     | OPERATION_MODE_ACCMAG      | Reads accel and mag data
 *  0x05     | OPERATION_MODE_ACCGYRO     | Reads accel and gyro data
 *  0x06     | OPERATION_MODE_MAGGYRO     | Reads accel and mag data
 *  0x07     | OPERATION_MODE_ANYMOTIONG         | Reads accel mag and gyro data
 *  0x08     | OPERATION_MODE_IMUPLUS     | Inertial measurement unit
 *   -       |       -                    | Reads accel,gyro and fusion data
 *  0x09     | OPERATION_MODE_COMPASS     | Reads accel, mag data
 *   -       |       -                    | and fusion data
 *  0x0A     | OPERATION_MODE_M4G         | Reads accel, mag data
 *    -      |       -                    | and fusion data
 *  0x0B     | OPERATION_MODE_NDOF_FMC_OFF| Nine degrees of freedom with
 *   -       |       -                    | fast magnetic calibration
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *  0x0C     | OPERATION_MODE_NDOF        | Nine degrees of freedom
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note In the config mode, all sensor and fusion data
 *	becomes zero and it is mainly derived
 *	to configure the various settings of the BNO
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_operation_mode(
_u8 *operation_mode);
/*!	@brief This API used to write the operation mode
 *	from register from 0x3D bit 0 to 3
 *
 *	@param operation_mode : The value of operation mode
 *
 *  operation_mode  |      result         | comments
 * ----------|----------------------------|----------------------------
 *  0x00     | OPERATION_MODE_CONFIG      | Configuration mode
 *  0x01     | OPERATION_MODE_ACCONLY     | Reads accel data alone
 *  0x02     | OPERATION_MODE_MAGONLY     | Reads mag data alone
 *  0x03     | OPERATION_MODE_GYRONLY     | Reads gyro data alone
 *  0x04     | OPERATION_MODE_ACCMAG      | Reads accel and mag data
 *  0x05     | OPERATION_MODE_ACCGYRO     | Reads accel and gyro data
 *  0x06     | OPERATION_MODE_MAGGYRO     | Reads accel and mag data
 *  0x07     | OPERATION_MODE_AMG         | Reads accel mag and gyro data
 *  0x08     | OPERATION_MODE_IMUPLUS     | Inertial measurement unit
 *   -       |       -                    | Reads accel,gyro and fusion data
 *  0x09     | OPERATION_MODE_COMPASS     | Reads accel, mag data
 *   -       |       -                    | and fusion data
 *  0x0A     | OPERATION_MODE_M4G         | Reads accel, mag data
 *    -      |       -                    | and fusion data
 *  0x0B     | OPERATION_MODE_NDOF_FMC_OFF| Nine degrees of freedom with
 *   -       |       -                    | fast magnetic calibration
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *  0x0C     | OPERATION_MODE_NDOF        | Nine degrees of freedom
 *   -       |       -                    | Reads accel,mag, gyro
 *   -       |       -                    | and fusion data
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note In the config mode, all sensor and fusion data
 *	becomes zero and it is mainly derived
 *	to configure the various settings of the BNO
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(_u8 operation_mode);
/**************************************************/
/**\name FUNCTIONS FOR POWER MODE  */
/**************************************************/
/*!	@brief This API used to read the power mode
 *	from register from 0x3E bit 0 to 1
 *
 *	@param power_mode : The value of power mode
 *
 * power_mode|      result         | comments
 * ----------|---------------------|-------------------------------------
 *  0x00     | POWER_MODE_NORMAL   | In the NORMAL mode the register
 *    -      |       -             | map and the internal peripherals
 *    -      |       -             | of the MCU are always
 *    -      |       -             | operative in this mode
 *  0x01     | POWER_MODE_LOWPOWER | This is first level of power saving mode
 *  0x02     | POWER_MODE_SUSPEND  | In suspend mode the system is
 *   -       |      -              | paused and all the sensors and
 *   -       |      -              | the micro controller are
 *   -       |      -              | put into sleep mode.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note For detailed about LOWPOWER mode
 *	refer data sheet 3.4.2
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_power_mode(
_u8 *power_mode);
/*!	@brief This API used to write the power mode
 *	from register from 0x3E bit 0 to 1
 *
 *	@param power_mode : The value of power mode
 *
 * power_mode |      result        | comments
 * ----------|---------------------|-------------------------------------
 *  0x00     | POWER_MODE_NORMAL   | In the NORMAL mode the register
 *    -      |       -             | map and the internal peripherals
 *    -      |       -             | of the MCU are always
 *    -      |       -             | operative in this mode
 *  0x01     | POWER_MODE_LOWPOWER | This is first level of power saving mode
 *  0x02     | POWER_MODE_SUSPEND  | In suspend mode the system is
 *   -       |      -              | paused and all the sensors and
 *   -       |      -              | the micro controller are
 *   -       |      -              | put into sleep mode.
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note For detailed about LOWPOWER mode
 *	refer data sheet 3.4.2
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_power_mode(
_u8 power_mode);
/**************************************************/
/**\name FUNCTIONS FOR RESET INTERRUPT  */
/**************************************************/
/*!
 *	@brief This API used to read the reset interrupt
 *	from register from 0x3F bit 6
 *	It resets all the interrupt bit and interrupt output
 *
 *	@param int_rst : The value of reset interrupt
 *
 *    int_rst  | result
 *   ----------|----------
 *     0x01    | ENABLED
 *     0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_rst(
_u8 *int_rst);
/*!
 *	@brief This API used to write the reset interrupt
 *	from register from 0x3F bit 6
 *	It resets all the interrupt bit and interrupt output
 *
 *	@param int_rst : The value of reset interrupt
 *
 *    int_rst  | result
 *   ----------|----------
 *     0x01    | ENABLED
 *     0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_rst(_u8 int_rst);
/**************************************************/
/**\name FUNCTIONS FOR CLOCK SOURCE  */
/**************************************************/
/*!
 *	@brief This API used to read the clk source
 *	from register from 0x3F bit 7
 *
 *	@param clk_src : The value of clk source
 *
 *	 clk_src   | result
 *   ----------|----------
 *     0x01    | ENABLED
 *     0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_clk_src(
_u8 *clk_src);
/*!
 *	@brief This API used to write the clock source
 *	from register from 0x3F bit 7
 *
 *	@param clk_src : The value of clock source
 *
 *	 clk_src   | result
 *   ----------|----------
 *     0x01    | ENABLED
 *     0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_clk_src(_u8 clk_src);
/**************************************************/
/**\name FUNCTIONS FOR RESET SYSTEM  */
/**************************************************/
/*!
 *	@brief This API used to read the reset system
 *	from register from 0x3F bit 5
 *
 *	@param sys_rst : The value of reset system
 *
 *	 sys_rst   | result
 *   ----------|----------
 *     0x01    | ENABLED
 *     0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note It resets the whole system
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_sys_rst(
_u8 *sys_rst);
/*!
 *	@brief This API used to write the reset system
 *	from register from 0x3F bit 5
 *
 *	@param sys_rst : The value of reset system
 *
 *	 sys_rst   | result
 *   ----------|----------
 *     0x01    | ENABLED
 *     0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note It resets the whole system
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_sys_rst(_u8 sys_rst);
/**************************************************/
/**\name FUNCTIONS FOR SELF TEST  */
/**************************************************/
/*!
 *	@brief This API used to read the self test
 *	from register from 0x3F bit 0
 *
 *	@param selftest : The value of self test
 *
 *	 selftest   | result
 *   -----------|----------
 *     0x01     | ENABLED
 *     0x00     | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note It triggers the self test
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_selftest(
_u8 *selftest);
/*!
 *	@brief This API used to write the self test
 *	from register from 0x3F bit 0
 *
 *	@param selftest : The value of self test
 *
 *	 selftest   | result
 *   -----------|----------
 *     0x01     | ENABLED
 *     0x00     | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note It triggers the self test
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_selftest(_u8 selftest);
/**************************************************/
/**\name FUNCTIONS FOR TEMPERATURE SOURCE  */
/**************************************************/
/*!
 *	@brief This API used to read the temperature source
 *	from register from 0x40 bit 0 and 1
 *
 *	@param temp_source : The value of selected temperature source
 *
 *     temp_source | result
 *    -----------  |---------------
 *      0x00       | ACCEL_TEMP_EN
 *      0X01       | GYRO_TEMP_EN
 *      0X03       | MCU_TEMP_EN
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_temp_source(
_u8 *temp_source);
/*!
 *	@brief This API used to write the temperature source
 *	from register from 0x40 bit 0 and 1
 *
 *	@param temp_source : The value of selected temperature source
 *
 *     temp_source | result
 *    -----------  |---------------
 *      0x00       | ACCEL_TEMP_EN
 *      0X01       | GYRO_TEMP_EN
 *      0X03       | MCU_TEMP_EN
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_source(_u8 temp_source);
/**************************************************/
/**\name FUNCTIONS FOR AXIS REMAP  */
/**************************************************/
/*!
 *	@brief This API used to read the axis remap value
 *	from register from 0x41 bit 0 and 5
 *
 *	@param remap_axis : The value of axis remapping
 *
 *    remap_axis |   result          | comments
 *   ------------|-------------------|------------
 *      0X21     | REMAP_X_Y         | Z=Z;X=Y;Y=X
 *      0X18     | REMAP_Y_Z         | X=X;Y=Z;Z=Y
 *      0X06     | REMAP_Z_X         | Y=Y;X=Z;Z=X
 *      0X12     | REMAP_X_Y_Z_TYPE0 | X=Z;Y=X;Z=Y
 *      0X09     | REMAP_X_Y_Z_TYPE1 | X=Y;Y=Z;Z=X
 *      0X24     | DEFAULT_AXIS      | X=X;Y=Y;Z=Z
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note : For axis sign remap refer the following functions
 *	x-axis :
 *
 *	bno055_set_x_remap_sign()
 *
 *	y-axis :
 *
 *	bno055_set_y_remap_sign()
 *
 *	z-axis :
 *
 *	bno055_set_z_remap_sign()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_axis_remap_value(
_u8 *remap_axis);
/*!
 *	@brief This API used to write the axis remap value
 *	from register from 0x41 bit 0 and 5
 *
 *	@param remap_axis : The value of axis remapping
 *
 *    remap_axis |   result          | comments
 *   ------------|-------------------|------------
 *      0X21     | REMAP_X_Y         | Z=Z;X=Y;Y=X
 *      0X18     | REMAP_Y_Z         | X=X;Y=Z;Z=Y
 *      0X06     | REMAP_Z_X         | Y=Y;X=Z;Z=X
 *      0X12     | REMAP_X_Y_Z_TYPE0 | X=Z;Y=X;Z=Y
 *      0X09     | REMAP_X_Y_Z_TYPE1 | X=Y;Y=Z;Z=X
 *      0X24     | DEFAULT_AXIS      | X=X;Y=Y;Z=Z
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note : For axis sign remap refer the following functions
 *	x-axis :
 *
 *	bno055_set_x_remap_sign()
 *
 *	y-axis :
 *
 *	bno055_set_y_remap_sign()
 *
 *	z-axis :
 *
 *	bno055_set_z_remap_sign()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_axis_remap_value(
_u8 remap_axis);
/**************************************************/
/**\name FUNCTIONS FOR AXIS REMAP SIGN  */
/**************************************************/
/*!
 *	@brief This API used to write the x-axis remap
 *	sign from register from 0x42 bit 2
 *
 *	@param remap_x_sign : The value of x-axis remap sign
 *
 *    remap_x_sign  |    result
 *   ---------------|--------------------
 *      0X00        | REMAP_AXIS_POSITIVE
 *      0X01        | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_x_sign(
_u8 *remap_x_sign);
/*!
 *	@brief This API used to write the x-axis remap
 *	sign from register from 0x42 bit 2
 *
 *	@param remap_x_sign : The value of x-axis remap sign
 *
 *    remap_x_sign  |    result
 *   ---------------|--------------------
 *      0X00        | REMAP_AXIS_POSITIVE
 *      0X01        | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_x_sign(
_u8 remap_x_sign);
/*!
 *	@brief This API used to read the y-axis remap
 *	sign from register from 0x42 bit 1
 *
 *	@param remap_y_sign : The value of y-axis remap sign
 *
 *    remap_y_sign  |   result
 *    --------------|-------------------
 *      0X00        | REMAP_AXIS_POSITIVE
 *      0X01        | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_y_sign(
_u8 *remap_y_sign);
/*!
 *	@brief This API used to write the y-axis remap
 *	sign from register from 0x42 bit 1
 *
 *	@param remap_y_sign : The value of y-axis remap sign
 *
 *    remap_y_sign  |   result
 *    --------------|-------------------
 *      0X00        | REMAP_AXIS_POSITIVE
 *      0X01        | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_y_sign(
_u8 remap_y_sign);
/*!
 *	@brief This API used to read the z-axis remap
 *	sign from register from 0x42 bit 0
 *
 *	@param remap_z_sign : The value of z-axis remap sign
 *
 *    remap_z_sign  |   result
 *   ---------------|--------------------
 *      0X00        | REMAP_AXIS_POSITIVE
 *      0X01        | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_remap_z_sign(
_u8 *remap_z_sign);
/*!
 *	@brief This API used to write the z-axis remap
 *	sign from register from 0x42 bit 0
 *
 *	@param remap_z_sign : The value of z-axis remap sign
 *
 *    remap_z_sign  |   result
 *   ---------------|--------------------
 *      0X00        | REMAP_AXIS_POSITIVE
 *      0X01        | REMAP_AXIS_NEGATIVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_remap_z_sign(
_u8 remap_z_sign);
/*****************************************************/
/**\name FUNCTIONS FOR SOFT IRON CALIBRATION MATRIX  */
/*****************************************************/
/*!
 *	@brief This API is used to read soft iron calibration matrix
 *	from the register 0x43 to 0x53 it is a 18 bytes of data
 *
 *	@param sic_matrix : The value of soft iron calibration matrix
 *
 *	sic_matrix         |           result
 * --------------------|----------------------------------
 *       sic_0         | soft iron calibration matrix zero
 *       sic_1         | soft iron calibration matrix one
 *       sic_2         | soft iron calibration matrix two
 *       sic_3         | soft iron calibration matrix three
 *       sic_4         | soft iron calibration matrix four
 *       sic_5         | soft iron calibration matrix five
 *       sic_6         | soft iron calibration matrix six
 *       sic_7         | soft iron calibration matrix seven
 *       sic_8         | soft iron calibration matrix eight
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note : Each soft iron calibration matrix range from -32768 to +32767
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_sic_matrix(
struct bno055_sic_matrix  *sic_matrix);
/*!
 *	@brief This API is used to write soft iron calibration matrix
 *	from the register 0x43 to 0x53 it is a 18 bytes of data
 *
 *	@param sic_matrix : The value of soft iron calibration matrix
 *
 *	sic_matrix         |           result
 * --------------------|----------------------------------
 *       sic_0         | soft iron calibration matrix zero
 *       sic_1         | soft iron calibration matrix one
 *       sic_2         | soft iron calibration matrix two
 *       sic_3         | soft iron calibration matrix three
 *       sic_4         | soft iron calibration matrix four
 *       sic_5         | soft iron calibration matrix five
 *       sic_6         | soft iron calibration matrix six
 *       sic_7         | soft iron calibration matrix seven
 *       sic_8         | soft iron calibration matrix eight
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note : Each soft iron calibration matrix range from -32768 to +32767
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_sic_matrix(
struct bno055_sic_matrix  *sic_matrix);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL OFFSET AND RADIUS */
/*****************************************************/
/*!
 *	@brief This API is used to read accel offset and accel radius
 *	offset form register 0x55 to 0x5A and radius form 0x67 and 0x68
 *
 *	@param accel_offset : The value of accel offset and radius
 *
 *	    bno055_accel_offset |     result
 *      ------------------- | ----------------
 *	             x          |  accel offset x
 *               y          |  accel offset y
 *	             z          |  accel offset z
 *               r          |  accel offset r
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the accel offset varies based on
 *	the G-range of accel sensor.
 *
 *  accel G range   |  offset range
 * ---------------  |  --------------
 *  ACCEL_RANGE_2G  |   +/-2000
 *  ACCEL_RANGE_4G  |   +/-4000
 *  ACCEL_RANGE_8G  |   +/-8000
 *  ACCEL_RANGE_16G |   +/-16000
 *
 *	accel G range can be configured by using the
 *	bno055_set_accel_range() function
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_offset(
struct bno055_accel_offset  *accel_offset);
/*!
 *	@brief This API is used to write accel offset and accel radius
 *	offset form register 0x55 to 0x5A and radius form 0x67 and 0x68
 *
 *	@param accel_offset : The value of accel offset and radius
 *
 *	    bno055_accel_offset |     result
 *      ------------------- | ----------------
 *	             x          |  accel offset x
 *               y          |  accel offset y
 *	             z          |  accel offset z
 *               r          |  accel offset r
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the accel offset varies based on
 *	the G-range of accel sensor.
 *
 *  accel G range   |  offset range
 * ---------------  |  --------------
 *  ACCEL_RANGE_2G  |   +/-2000
 *  ACCEL_RANGE_4G  |   +/-4000
 *  ACCEL_RANGE_8G  |   +/-8000
 *  ACCEL_RANGE_16G |   +/-16000
 *
 *	accel G range can be configured by using the
 *	bno055_set_accel_range() function
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_accel_offset(
struct bno055_accel_offset  *accel_offset);
/*****************************************************/
/**\name FUNCTIONS FOR MAG OFFSET AND RADIUS*/
/*****************************************************/
/*!
 *	@brief This API is used to read mag offset
 *	offset form register 0x69 to 0x6A
 *
 *	@param mag_offset :  The value of mag offset and radius
 *
 *	    bno055_mag_offset   |     result
 *      ------------------- | ----------------
 *	             x          |  mag offset x
 *               y          |  mag offset y
 *	             z          |  mag offset z
 *               r          |  mag radius r
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the magnetometer offset is +/-6400 in LSB
 */

BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_offset(
struct bno055_mag_offset  *mag_offset);
/*!
 *	@brief This API is used to read mag offset
 *	offset form register 0x69 to 0x6A
 *
 *	@param mag_offset :  The value of mag offset and radius
 *
 *	    bno055_mag_offset   |     result
 *      ------------------- | ----------------
 *	             x          |  mag offset x
 *               y          |  mag offset y
 *	             z          |  mag offset z
 *               r          |  mag radius r
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the magnetometer offset is +/-6400 in LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_mag_offset(
struct bno055_mag_offset  *mag_offset);
/*****************************************************/
/**\name FUNCTIONS FOR GYRO OFFSET */
/*****************************************************/
/*!
 *	@brief This API is used to read gyro offset
 *	offset form register 0x61 to 0x66
 *
 *	@param gyro_offset : The value of gyro offset
 *
 *	    bno055_gyro_offset  |     result
 *      ------------------- | ----------------
 *	             x          |  gyro offset x
 *               y          |  gyro offset y
 *	             z          |  gyro offset z
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the gyro offset varies based on
 *	the range of gyro sensor
 *
 *	gyro G range         | offset range
 * --------------------  | ------------
 *  GYRO_RANGE_2000DPS   | +/-32000
 *  GYRO_RANGE_1000DPS   | +/-16000
 *  GYRO_RANGE_500DPS    | +/-8000
 *  GYRO_RANGE_250DPS    | +/-4000
 *  GYRO_RANGE_125DPS    | +/-2000
 *
 *	Gyro range can be configured by using the
 *	bno055_set_gyro_range() function
 */
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_offset(
struct bno055_gyro_offset  *gyro_offset);
/*!
 *	@brief This API is used to read gyro offset
 *	offset form register 0x61 to 0x66
 *
 *	@param gyro_offset : The value of gyro offset
 *
 *	    bno055_gyro_offset  |     result
 *      ------------------- | ----------------
 *	             x          |  gyro offset x
 *               y          |  gyro offset y
 *	             z          |  gyro offset z
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note  The range of the gyro offset varies based on
 *	the range of gyro sensor
 *
 *	gyro G range         | offset range
 * --------------------  | ------------
 *  GYRO_RANGE_2000DPS   | +/-32000
 *  GYRO_RANGE_1000DPS   | +/-16000
 *  GYRO_RANGE_500DPS    | +/-8000
 *  GYRO_RANGE_250DPS    | +/-4000
 *  GYRO_RANGE_125DPS    | +/-2000
 *
 *	Gyro range can be configured by using the
 *	bno055_set_gyro_range() function
 */
BNO055_RETURN_FUNCTION_TYPE bno055_write_gyro_offset(
struct bno055_gyro_offset *gyro_offset);
/********************************************************/
/************** PAGE1 Functions *********************/
/********************************************************/
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL CONFIGURATION */
/*****************************************************/
/*!
 *	@brief This API used to read the accel range
 *	from page one register from 0x08 bit 0 and 1
 *
 *	@param accel_range : The value of accel range
 *		   accel_range     |   result
 *       ----------------- | --------------
 *              0x00       | ACCEL_RANGE_2G
 *              0x01       | ACCEL_RANGE_4G
 *              0x02       | ACCEL_RANGE_8G
 *              0x03       | ACCEL_RANGE_16G
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_range(
_u8 *accel_range);
/*!
 *	@brief This API used to write the accel range
 *	from page one register from 0x08 bit 0 and 1
 *
 *	@param accel_range : The value of accel range
 *
 *		   accel_range     |   result
 *       ----------------- | --------------
 *              0x00       | ACCEL_RANGE_2G
 *              0x01       | ACCEL_RANGE_4G
 *              0x02       | ACCEL_RANGE_8G
 *              0x03       | ACCEL_RANGE_16G
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_range(
_u8 accel_range);
/*!
 *	@brief This API used to read the accel bandwidth
 *	from page one register from 0x08 bit 2 to 4
 *
 *	@param accel_bw : The value of accel bandwidth
 *
 *		     accel_bw      |     result
 *       ----------------- | ---------------
 *              0x00       | ACCEL_BW_7_81HZ
 *              0x01       | ACCEL_BW_15_63HZ
 *              0x02       | ACCEL_BW_31_25HZ
 *              0x03       | ACCEL_BW_62_5HZ
 *              0x04       | ACCEL_BW_125HZ
 *              0x05       | ACCEL_BW_250HZ
 *              0x06       | ACCEL_BW_500HZ
 *              0x07       | ACCEL_BW_1000HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_bw(
_u8 *accel_bw);
/*!
 *	@brief This API used to write the accel bandwidth
 *	from page one register from 0x08 bit 2 to 4
 *
 *	@param accel_bw : The value of accel bandwidth
 *
 *		     accel_bw      |     result
 *       ----------------- | ---------------
 *              0x00       | ACCEL_BW_7_81HZ
 *              0x01       | ACCEL_BW_15_63HZ
 *              0x02       | ACCEL_BW_31_25HZ
 *              0x03       | ACCEL_BW_62_5HZ
 *              0x04       | ACCEL_BW_125HZ
 *              0x05       | ACCEL_BW_250HZ
 *              0x06       | ACCEL_BW_500HZ
 *              0x07       | ACCEL_BW_1000HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_bw(
_u8 accel_bw);
/*!
 *	@brief This API used to read the accel power mode
 *	from page one register from 0x08 bit 5 to 7
 *
 *	@param accel_power_mode : The value of accel power mode
 *   accel_power_mode    |   result
 *   -----------------   | -------------
 *              0x00     | ACCEL_NORMAL
 *              0x01     | ACCEL_SUSPEND
 *              0x02     | ACCEL_LOWPOWER_1
 *              0x03     | ACCEL_STANDBY
 *              0x04     | ACCEL_LOWPOWER_2
 *              0x05     | ACCEL_DEEPSUSPEND
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_power_mode(
_u8 *accel_power_mode);
/*!
 *	@brief This API used to write the accel power mode
 *	from page one register from 0x08 bit 5 to 7
 *
 *	@param accel_power_mode : The value of accel power mode
 *   accel_power_mode    |   result
 *   -----------------   | -------------
 *              0x00     | ACCEL_NORMAL
 *              0x01     | ACCEL_SUSPEND
 *              0x02     | ACCEL_LOWPOWER_1
 *              0x03     | ACCEL_STANDBY
 *              0x04     | ACCEL_LOWPOWER_2
 *              0x05     | ACCEL_DEEPSUSPEND
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_power_mode(
_u8 accel_power_mode);
/*****************************************************/
/**\name FUNCTIONS FOR MAG CONFIGURATION */
/*****************************************************/
/*!
 *	@brief This API used to read the mag output data rate
 *	from page one register from 0x09 bit 0 to 2
 *
 *	@param mag_data_output_rate : The value of mag output data rate
 *
 *  mag_data_output_rate |   result
 *  -----------------    |----------------------
 *     0x00              | MAG_DATA_OUTPUT_RATE_2HZ
 *     0x01              | MAG_DATA_OUTPUT_RATE_6HZ
 *     0x02              | MAG_DATA_OUTPUT_RATE_8HZ
 *     0x03              | MAG_DATA_OUTPUT_RATE_10HZ
 *     0x04              | MAG_DATA_OUTPUT_RATE_15HZ
 *     0x05              | MAG_DATA_OUTPUT_RATE_20HZ
 *     0x06              | MAG_DATA_OUTPUT_RATE_25HZ
 *     0x07              | MAG_DATA_OUTPUT_RATE_30HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_data_output_rate(
_u8 *mag_data_output_rate);
/*!
 *	@brief This API used to write the mag output data rate
 *	from page one register from 0x09 bit 0 to 2
 *
 *	@param mag_data_output_rate : The value of mag output data rate
 *
 *  mag_data_output_rate |   result
 *  -----------------    |----------------------
 *     0x00              | MAG_DATA_OUTPUT_RATE_2HZ
 *     0x01              | MAG_DATA_OUTPUT_RATE_6HZ
 *     0x02              | MAG_DATA_OUTPUT_RATE_8HZ
 *     0x03              | MAG_DATA_OUTPUT_RATE_10HZ
 *     0x04              | MAG_DATA_OUTPUT_RATE_15HZ
 *     0x05              | MAG_DATA_OUTPUT_RATE_20HZ
 *     0x06              | MAG_DATA_OUTPUT_RATE_25HZ
 *     0x07              | MAG_DATA_OUTPUT_RATE_30HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_data_output_rate(
_u8 mag_data_output_rate);
/*!
 *	@brief This API used to read the mag operation mode
 *	from page one register from 0x09 bit 3 to 4
 *
 *	@param mag_operation_mode : The value of mag operation mode
 *
 *  mag_operation_mode |           result
 * --------------------|----------------------------
 *     0x00            | MAG_OPR_MODE_LOWPOWER
 *     0x01            | MAG_OPR_MODE_REGULAR
 *     0x02            | MAG_OPR_MODE_ENHANCED_REGULAR
 *     0x03            | MAG_OPR_MODE_HIGH_ACCURACY
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_operation_mode(
_u8 *mag_operation_mode);
/*!
 *	@brief This API used to write the mag operation mode
 *	from page one register from 0x09 bit 3 to 4
 *
 *	@param mag_operation_mode : The value of mag operation mode
 *
 *  mag_operation_mode |           result
 * --------------------|----------------------------
 *     0x00            | MAG_OPR_MODE_LOWPOWER
 *     0x01            | MAG_OPR_MODE_REGULAR
 *     0x02            | MAG_OPR_MODE_ENHANCED_REGULAR
 *     0x03            | MAG_OPR_MODE_HIGH_ACCURACY
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_operation_mode(
_u8 mag_operation_mode);
/*!
 *	@brief This API used to read the mag power mode
 *	from page one register from 0x09 bit 4 to 6
 *
 *	@param mag_power_mode : The value of mag power mode
 *
 *     mag_power_mode  |   result
 * --------------------|-----------------
 *     0x00            | MAG_POWER_MODE_NORMAL
 *     0x01            | MAG_POWER_MODE_SLEEP
 *     0x02            | MAG_POWER_MODE_SUSPEND
 *     0x03            | MAG_POWER_MODE_FORCE_MODE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_power_mode(
_u8 *mag_power_mode);
/*!
 *	@brief This API used to write the mag power mode
 *	from page one register from 0x09 bit 4 to 6
 *
 *	@param mag_power_mode : The value of mag power mode
 *
 *     mag_power_mode  |   result
 * --------------------|-----------------
 *     0x00            | MAG_POWER_MODE_NORMAL
 *     0x01            | MAG_POWER_MODE_SLEEP
 *     0x02            | MAG_POWER_MODE_SUSPEND
 *     0x03            | MAG_POWER_MODE_FORCE_MODE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_power_mode(
_u8 mag_power_mode);
/*****************************************************/
/**\name FUNCTIONS FOR GYRO CONFIGURATION */
/*****************************************************/
/*!
 *	@brief This API used to read the gyro range
 *	from page one register from 0x0A bit 0 to 3
 *
 *	@param gyro_range : The value of gyro range
 *
 *     gyro_range      |   result
 * --------------------|-----------------
 *     0x00            | GYRO_RANGE_2000DPS
 *     0x01            | GYRO_RANGE_1000DPS
 *     0x02            | GYRO_RANGE_500DPS
 *     0x03            | GYRO_RANGE_250DPS
 *     0x04            | GYRO_RANGE_125DPS
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_range(
_u8 *gyro_range);
/*!
 *	@brief This API used to write the gyro range
 *	from page one register from 0x0A bit 0 to 3
 *
 *	@param gyro_range : The value of gyro range
 *
 *     gyro_range      |   result
 * --------------------|-----------------
 *     0x00            | GYRO_RANGE_2000DPS
 *     0x01            | GYRO_RANGE_1000DPS
 *     0x02            | GYRO_RANGE_500DPS
 *     0x03            | GYRO_RANGE_250DPS
 *     0x04            | GYRO_RANGE_125DPS
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_range(
_u8 gyro_range);
/*!
 *	@brief This API used to read the gyro bandwidth
 *	from page one register from 0x0A bit 3 to 5
 *
 *	@param gyro_bw : The value of gyro bandwidth
 *
 *     gyro_bw         |   result
 * --------------------|-----------------
 *     0x00            | GYRO_BW_523HZ
 *     0x01            | GYRO_BW_230HZ
 *     0x02            | GYRO_BW_116HZ
 *     0x03            | GYRO_BW_47HZ
 *     0x04            | GYRO_BW_23HZ
 *     0x05            | GYRO_BW_12HZ
 *     0x06            | GYRO_BW_64HZ
 *     0x07            | GYRO_BW_32HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_bw(
_u8 *gyro_bw);
/*!
 *	@brief This API used to write the gyro bandwidth
 *	from page one register from 0x0A bit 3 to 5
 *
 *	@param gyro_bw : The value of gyro bandwidth
 *
 *     gyro_bw         |   result
 * --------------------|-----------------
 *     0x00            | GYRO_BW_523HZ
 *     0x01            | GYRO_BW_230HZ
 *     0x02            | GYRO_BW_116HZ
 *     0x03            | GYRO_BW_47HZ
 *     0x04            | GYRO_BW_23HZ
 *     0x05            | GYRO_BW_12HZ
 *     0x06            | GYRO_BW_64HZ
 *     0x07            | GYRO_BW_32HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_bw(
_u8 gyro_bw);
/*!
 *	@brief This API used to read the gyro power mode
 *	from page one register from 0x0B bit 0 to 2
 *
 *	@param gyro_power_mode : The value of gyro power mode
 *
 *  gyro_power_mode      |          result
 * ----------------------|----------------------------
 *     0x00              | GYRO_OPR_MODE_NORMAL
 *     0x01              | GYRO_OPR_MODE_FASTPOWERUP
 *     0x02              | GYRO_OPR_MODE_DEEPSUSPEND
 *     0x03              | GYRO_OPR_MODE_SUSPEND
 *     0x04              | GYRO_OPR_MODE_ADVANCE_POWERSAVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_power_mode(
_u8 *gyro_power_mode);
/*!
 *	@brief This API used to write the gyro power mode
 *	from page one register from 0x0B bit 0 to 2
 *
 *	@param gyro_power_mode : The value of gyro power mode
 *
 *  gyro_power_mode      |          result
 * ----------------------|----------------------------
 *     0x00              | GYRO_OPR_MODE_NORMAL
 *     0x01              | GYRO_OPR_MODE_FASTPOWERUP
 *     0x02              | GYRO_OPR_MODE_DEEPSUSPEND
 *     0x03              | GYRO_OPR_MODE_SUSPEND
 *     0x04              | GYRO_OPR_MODE_ADVANCE_POWERSAVE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_power_mode(
_u8 gyro_power_mode);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL SLEEP SETTINGS  */
/*****************************************************/
/*!
 *	@brief This API used to read the accel sleep mode
 *	from page one register from 0x0C bit 0
 *
 *	@param sleep_tmr : The value of accel sleep mode
 *
 *  sleep_tmr   |   result
 * -------------|------------------------------------
 *     0x00     | enable EventDrivenSampling(EDT)
 *     0x01     | enable Equidistant sampling mode(EST)
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleeptmr_mode(
_u8 *sleep_tmr);
/*!
 *	@brief This API used to write the accel sleep mode
 *	from page one register from 0x0C bit 0
 *
 *	@param sleep_tmr : The value of accel sleep mode
 *
 *  sleep_tmr   |   result
 * -------------|------------------------------------
 *     0x00     | enable EventDrivenSampling(EDT)
 *     0x01     | enable Equidistant sampling mode(EST)
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleeptmr_mode(
_u8 sleep_tmr);
/*!
 *	@brief This API used to read the accel sleep duration
 *	from page one register from 0x0C bit 1 to 4
 *
 *	@param sleep_dur : The value of accel sleep duration
 *
 *   sleep_dur  |      result
 * -------------|-----------------------------
 *     0x05     | BNO055_ACCEL_SLEEP_DUR_0_5MS
 *     0x06     | BNO055_ACCEL_SLEEP_DUR_1MS
 *     0x07     | BNO055_ACCEL_SLEEP_DUR_2MS
 *     0x08     | BNO055_ACCEL_SLEEP_DUR_4MS
 *     0x09     | BNO055_ACCEL_SLEEP_DUR_6MS
 *     0x0A     | BNO055_ACCEL_SLEEP_DUR_10MS
 *     0x0B     | BNO055_ACCEL_SLEEP_DUR_25MS
 *     0x0C     | BNO055_ACCEL_SLEEP_DUR_50MS
 *     0x0D     | BNO055_ACCEL_SLEEP_DUR_100MS
 *     0x0E     | BNO055_ACCEL_SLEEP_DUR_500MS
 *     0x0F     | BNO055_ACCEL_SLEEP_DUR_1S
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_sleep_dur(
_u8 *sleep_dur);
/*!
 *	@brief This API used to write the accel sleep duration
 *	from page one register from 0x0C bit 1 to 4
 *
 *	@param sleep_dur : The value of accel sleep duration
 *
 *   sleep_dur  |      result
 * -------------|-----------------------------
 *     0x05     | BNO055_ACCEL_SLEEP_DUR_0_5MS
 *     0x06     | BNO055_ACCEL_SLEEP_DUR_1MS
 *     0x07     | BNO055_ACCEL_SLEEP_DUR_2MS
 *     0x08     | BNO055_ACCEL_SLEEP_DUR_4MS
 *     0x09     | BNO055_ACCEL_SLEEP_DUR_6MS
 *     0x0A     | BNO055_ACCEL_SLEEP_DUR_10MS
 *     0x0B     | BNO055_ACCEL_SLEEP_DUR_25MS
 *     0x0C     | BNO055_ACCEL_SLEEP_DUR_50MS
 *     0x0D     | BNO055_ACCEL_SLEEP_DUR_100MS
 *     0x0E     | BNO055_ACCEL_SLEEP_DUR_500MS
 *     0x0F     | BNO055_ACCEL_SLEEP_DUR_1S
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_sleep_dur(
_u8 sleep_dur);
/*****************************************************/
/**\name FUNCTIONS FOR GYRO SLEEP SETTINGS  */
/*****************************************************/
/*!
 *	@brief This API used to write the gyro sleep duration
 *	from page one register from 0x0D bit 0 to 2
 *
 *	@param sleep_dur : The value of gyro sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_sleep_dur(
_u8 *sleep_dur);
/*!
 *	@brief This API used to write the gyro sleep duration
 *	from page one register from 0x0D bit 0 to 2
 *
 *	@param sleep_dur : The value of gyro sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_sleep_dur(
_u8 sleep_dur);
/*!
 *	@brief This API used to read the gyro auto sleep duration
 *	from page one register from 0x0D bit 3 to 5
 *
 *	@param auto_sleep_dur : The value of gyro auto sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_auto_sleep_dur(
_u8 *auto_sleep_dur);
/*!
 *	@brief This API used to write the gyro auto sleep duration
 *	from page one register from 0x0D bit 3 to 5
 *
 *	@param auto_sleep_dur : The value of gyro auto sleep duration
 *	@param bw : The value of gyro bandwidth
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_gyro_set_auto_sleep_dur(
_u8 auto_sleep_dur, _u8 bw);
/*****************************************************/
/**\name FUNCTIONS FOR MAG SLEEP SETTINGS  */
/*****************************************************/
/*!
 *	@brief This API used to read the mag sleep mode
 *	from page one register from 0x0E bit 0
 *
 *	@param sleep_mode : The value of mag sleep mode
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_mode(
_u8 *sleep_mode);
/*!
 *	@brief This API used to write the mag sleep mode
 *	from page one register from 0x0E bit 0
 *
 *	@param sleep_mode : The value of mag sleep mode
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_mode(
_u8 sleep_mode);
/*!
 *	@brief This API used to read the mag sleep duration
 *	from page one register from 0x0E bit 1 to 4
 *
 *	@param sleep_dur : The value of mag sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_mag_sleep_dur(
_u8 *sleep_dur);
/*!
 *	@brief This API used to write the mag sleep duration
 *	from page one register from 0x0E bit 1 to 4
 *
 *	@param sleep_dur : The value of mag sleep duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_mag_sleep_dur(
_u8 sleep_dur);
/*****************************************************/
/**\name FUNCTIONS FOR GYRO INTERRUPT MASK  */
/*****************************************************/
/*!
 *	@brief This API used to read the gyro anymotion interrupt mask
 *	from page one register from 0x0F bit 2
 *
 *	@param gyro_anymotion : The value of gyro anymotion interrupt mask
 *		gyro_anymotion |   result
 *     --------------- |------------
 *              0x01   | ENABLED
 *              0x00   | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *	bno055_set_gyro_anymotion_axis_enable()
 *
 *	Filter setting:
 *	bno055_set_gyro_anymotion_filter()
 *
 *	Threshold :
 *
 *	bno055_set_gyro_anymotion_thres()
 *
 *	Slope samples :
 *
 *	bno055_set_gyro_anymotion_slope_samples()
 *
 *	Awake duration :
 *
 *	bno055_set_gyro_anymotion_awake_dur()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_mask_gyro_anymotion(
_u8 *gyro_anymotion);
/*!
 *	@brief This API used to write the gyro anymotion interrupt mask
 *	from page one register from 0x0F bit 2
 *
 *	@param gyro_anymotion : The value of gyro anymotion interrupt mask
 *		gyro_anymotion |   result
 *     --------------- |------------
 *              0x01   | ENABLED
 *              0x00   | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *	bno055_set_gyro_anymotion_axis_enable()
 *
 *	Filter setting:
 *	bno055_set_gyro_anymotion_filter()
 *
 *	Threshold :
 *
 *	bno055_set_gyro_anymotion_thres()
 *
 *	Slope samples :
 *
 *	bno055_set_gyro_anymotion_slope_samples()
 *
 *	Awake duration :
 *
 *	bno055_set_gyro_anymotion_awake_dur()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_mask_gyro_anymotion(
_u8 gyro_anymotion);
/*!
 *	@brief This API used to read the gyro highrate interrupt mask
 *	from page one register from 0x0F bit 3
 *
 *	@param gyro_highrate : The value of gyro highrate interrupt mask
 *		   gyro_highrate|  result
 *       -------------- | --------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro highrate interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_gyro_highrate_axis_enable()
 *
 *	Filter :
 *
 *	bno055_set_gyro_highrate_filter()
 *
 *	Threshold :
 *
 *	bno055_get_gyro_highrate_x_thres()
 *
 *	bno055_get_gyro_highrate_y_thres()
 *
 *	bno055_get_gyro_highrate_z_thres()
 *
 *	Hysteresis :
 *
 *	bno055_set_gyro_highrate_x_hyst()
 *
 *	bno055_set_gyro_highrate_y_hyst()
 *
 *	bno055_set_gyro_highrate_z_hyst()
 *
 *	Duration :
 *
 *	bno055_set_gyro_highrate_x_dur()
 *
 *	bno055_set_gyro_highrate_y_dur()
 *
 *	bno055_set_gyro_highrate_z_dur()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_mask_gyro_highrate(
_u8 *gyro_highrate);
/*!
 *	@brief This API used to write the gyro highrate interrupt mask
 *	from page one register from 0x0F bit 3
 *
 *	@param gyro_highrate : The value of gyro highrate interrupt mask
 *		   gyro_highrate|  result
 *       -------------- | --------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro highrate interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_gyro_highrate_axis_enable()
 *
 *	Filter :
 *
 *	bno055_set_gyro_highrate_filter()
 *
 *	Threshold :
 *
 *	bno055_get_gyro_highrate_x_thres()
 *
 *	bno055_get_gyro_highrate_y_thres()
 *
 *	bno055_get_gyro_highrate_z_thres()
 *
 *	Hysteresis :
 *
 *	bno055_set_gyro_highrate_x_hyst()
 *
 *	bno055_set_gyro_highrate_y_hyst()
 *
 *	bno055_set_gyro_highrate_z_hyst()
 *
 *	Duration :
 *
 *	bno055_set_gyro_highrate_x_dur()
 *
 *	bno055_set_gyro_highrate_y_dur()
 *
 *	bno055_set_gyro_highrate_z_dur()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_mask_gyro_highrate(
_u8 gyro_highrate);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL INTERRUPT MASK  */
/*****************************************************/
/*!
 *	@brief This API used to read the accel highg interrupt mask
 *	from page one register from 0x0F bit 5
 *
 *	@param accel_high_g : The value of accel highg interrupt mask
 *		   accel_high_g |   result
 *     ---------------- | ---------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_dur()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_mask_accel_high_g(
_u8 *accel_high_g);
/*!
 *	@brief This API used to write the accel highg interrupt mask
 *	from page one register from 0x0F bit 5
 *
 *	@param accel_high_g : The value of accel highg interrupt mask
 *		   accel_high_g |   result
 *     ---------------- | ---------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_dur()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_mask_accel_high_g(
_u8 accel_high_g);
/*!
 *	@brief This API used to read the accel anymotion interrupt mask
 *	from page one register from 0x0F bit 6
 *
 *	@param accel_anymotion : The value of accel anymotion interrupt mask
 *      accel_anymotion | result
 *        ------------- | -------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_dur()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_mask_accel_anymotion(
_u8 *accel_anymotion);
/*!
 *	@brief This API used to write the accel anymotion interrupt mask
 *	from page one register from 0x0F bit 6
 *
 *	@param accel_anymotion : The value of accel anymotion interrupt mask
 *		accel_anymotion | result
 *        ------------- | -------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_anymotion_nomotion_axis_enable()
 *
 *	Duration:
 *
 *	bno055_set_accel_anymotion_dur()
 *
 * Threshold:
 *
 *	bno055_set_accel_anymotion_thres()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_mask_accel_anymotion(
_u8 accel_anymotion);
/*!
 *	@brief This API used to read the accel nomotion interrupt mask
 *	from page one register from 0x0F bit 7
 *
 *	@param accel_nomotion : The value of accel nomotion interrupt mask
 *		 accel_nomotion | result
 *       -------------- | -----------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *
 *	@note While enabling the accel anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_anymotion_nomotion_axis_enable()
 *
 *	Duration:
 *
 *	bno055_set_accel_anymotion_dur()
 *
 * Threshold:
 *
 *	bno055_set_accel_anymotion_thres())
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_mask_accel_nomotion(
_u8 *accel_nomotion);
/*!
 *	@brief This API used to write the accel nomotion interrupt mask
 *	from page one register from 0x0F bit 7
 *
 *	@param accel_nomotion : The value of accel nomotion interrupt mask
 *		 accel_nomotion | result
 *       -------------- | -----------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel nomotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_anymotion_nomotion_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_slow_no_motion_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_slow_no_motion_dur()
 *
 *	Slow/no motion enable:
 *
 *	bno055_set_accel_slow_no_motion_enable()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_mask_accel_nomotion(
_u8 accel_nomotion);
/*****************************************************/
/**\name FUNCTIONS FOR GYRO INTERRUPT */
/*****************************************************/
/*!
 *	@brief This API used to read the gyro anymotion interrupt
 *	from page one register from 0x10 bit 2
 *
 *	@param gyro_anymotion : The value of gyro anymotion interrupt
 *		gyro_anymotion  | result
 *     ---------------- | ---------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *	bno055_set_gyro_anymotion_axis_enable()
 *
 *	Filter setting:
 *	bno055_set_gyro_anymotion_filter()
 *
 *	Threshold :
 *
 *	bno055_set_gyro_anymotion_thres()
 *
 *	Slope samples :
 *
 *	bno055_set_gyro_anymotion_slope_samples()
 *
 *	Awake duration :
 *
 *	bno055_set_gyro_anymotion_awake_dur()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_gyro_anymotion(
_u8 *gyro_anymotion);
/*!
 *	@brief This API used to write the gyro anymotion interrupt
 *	from page one register from 0x10 bit 2
 *
 *	@param gyro_anymotion : The value of gyro anymotion interrupt
 *	  gyro_anymotion    | result
 *     ---------------- | ---------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *	bno055_set_gyro_anymotion_axis_enable()
 *
 *	Filter setting:
 *	bno055_set_gyro_anymotion_filter()
 *
 *	Threshold :
 *
 *	bno055_set_gyro_anymotion_thres()
 *
 *	Slope samples :
 *
 *	bno055_set_gyro_anymotion_slope_samples()
 *
 *	Awake duration :
 *
 *	bno055_set_gyro_anymotion_awake_dur()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_gyro_anymotion(
_u8 gyro_anymotion);
/*!
 *	@brief This API used to read the gyro highrate interrupt
 *	from page one register from 0x10 bit 3
 *
 *	@param gyro_highrate : The value of gyro highrate interrupt
 *		gyro_highrate    | result
 *     ----------------- | ---------------
 *              0x01     | ENABLED
 *              0x00     | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro highrate interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_gyro_highrate_axis_enable()
 *
 *	Filter :
 *
 *	bno055_set_gyro_highrate_filter()
 *
 *	Threshold :
 *
 *	bno055_get_gyro_highrate_x_thres()
 *
 *	bno055_get_gyro_highrate_y_thres()
 *
 *	bno055_get_gyro_highrate_z_thres()
 *
 *	Hysteresis :
 *
 *	bno055_set_gyro_highrate_x_hyst()
 *
 *	bno055_set_gyro_highrate_y_hyst()
 *
 *	bno055_set_gyro_highrate_z_hyst()
 *
 *	Duration :
 *
 *	bno055_set_gyro_highrate_x_dur()
 *
 *	bno055_set_gyro_highrate_y_dur()
 *
 *	bno055_set_gyro_highrate_z_dur()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_gyro_highrate(
_u8 *gyro_highrate);
/*!
 *	@brief This API used to write the gyro highrate interrupt
 *	from page one register from 0x10 bit 3
 *
 *	@param gyro_highrate : The value of gyro highrate interrupt
 *		gyro_highrate    | result
 *     ----------------- | ---------------
 *              0x01     | ENABLED
 *              0x00     | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the gyro highrate interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_gyro_highrate_axis_enable()
 *
 *	Filter :
 *
 *	bno055_set_gyro_highrate_filter()
 *
 *	Threshold :
 *
 *	bno055_get_gyro_highrate_x_thres()
 *
 *	bno055_get_gyro_highrate_y_thres()
 *
 *	bno055_get_gyro_highrate_z_thres()
 *
 *	Hysteresis :
 *
 *	bno055_set_gyro_highrate_x_hyst()
 *
 *	bno055_set_gyro_highrate_y_hyst()
 *
 *	bno055_set_gyro_highrate_z_hyst()
 *
 *	Duration :
 *
 *	bno055_set_gyro_highrate_x_dur()
 *
 *	bno055_set_gyro_highrate_y_dur()
 *
 *	bno055_set_gyro_highrate_z_dur()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_gyro_highrate(
_u8 gyro_highrate);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL INTERRUPT  */
/*****************************************************/
/*!
 *	@brief This API used to read the accel highg interrupt
 *	from page one register from 0x10 bit 5
 *
 *	@param accel_high_g : The value of accel highg interrupt
 *		   accel_high_g     | result
 *     ---------------- | ---------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_dur()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_high_g(
_u8 *accel_high_g);
/*!
 *	@brief This API used to write the accel highg interrupt
 *	from page one register from 0x10 bit 5
 *
 *	@param accel_high_g : The value of accel highg interrupt
 *		   accel_high_g     | result
 *     ---------------- | ---------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel highg interrupt
 *	configure the below settings by using
 *	the following functions
 *
 *	Axis :
 *
 *	bno055_set_accel_high_g_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_high_g_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_high_g_dur()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_high_g(
_u8 accel_high_g);
/*!
 *	@brief This API used to read the accel anymotion interrupt
 *	from page one register from 0x10 bit 6
 *
 *	@param accel_anymotion : The value of accel anymotion interrupt
 *		accel_anymotion  | result
 *     ----------------- | ---------------
 *              0x01     | ENABLED
 *              0x00     | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_anymotion_nomotion_axis_enable()
 *
 *	Duration:
 *
 *	bno055_set_accel_anymotion_dur()
 *
 * Threshold:
 *
 *	bno055_set_accel_anymotion_thres()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_anymotion(
_u8 *accel_anymotion);
/*!
 *	@brief This API used to write the accel anymotion interrupt
 *	from page one register from 0x10 bit 6
 *
 *	@param accel_anymotion : The value of accel anymotion interrupt
 *	  accel_anymotion    | result
 *     ----------------- | ---------------
 *              0x01     | ENABLED
 *              0x00     | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel anymotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_anymotion_nomotion_axis_enable()
 *
 *	Duration:
 *
 *	bno055_set_accel_anymotion_dur()
 *
 * Threshold:
 *
 *	bno055_set_accel_anymotion_thres()
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_anymotion(
_u8 accel_anymotion);
/*!
 *	@brief This API used to read the accel nomotion interrupt
 *	from page one register from 0x10 bit 6
 *
 *	@param accel_nomotion : The value of accel nomotion interrupt
 *		  accel_nomotion| result
 *         ------------ | -----------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel nomotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_anymotion_nomotion_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_slow_no_motion_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_slow_no_motion_dur()
 *
 *	Slow/no motion enable:
 *
 *	bno055_set_accel_slow_no_motion_enable()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_nomotion(
_u8 *accel_nomotion);
/*!
 *	@brief This API used to write the accel nomotion interrupt
 *	from page one register from 0x10 bit 6
 *
 *	@param accel_nomotion : The value of accel nomotion interrupt
 *		accel_nomotion  | result
 *     ---------------- | ---------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note While enabling the accel nomotion interrupt
 *	configure the following settings
 *
 *	Axis:
 *
 *	bno055_set_accel_anymotion_nomotion_axis_enable()
 *
 *	Threshold :
 *
 *	bno055_set_accel_slow_no_motion_thres()
 *
 *	Duration :
 *
 *	bno055_set_accel_slow_no_motion_dur()
 *
 *	Slow/no motion enable:
 *
 *	bno055_set_accel_slow_no_motion_enable()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_nomotion(
_u8 accel_nomotion);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL ANYMOTION THRESHOLD  */
/*****************************************************/
/*!
 *	@brief This API used to read the accel anymotion threshold
 *	from page one register from 0x11 bit 0 to 7
 *
 *	@param accel_anymotion_thres : The value of any motion threshold
 * accel_anymotion_thres| result
 *  ------------------- | -------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel anymotion threshold dependent on the
 *	range values
 *
 *  accel_range	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_anymotion_thresh(
_u8 *accel_anymotion_thres);
/*!
 *	@brief This API used to write the accel any motion threshold
 *	from page one register from 0x11 bit 0 to 7
 *
 *	@param accel_anymotion_thres : The value of any motion threshold
 * accel_anymotion_thres| result
 *  ------------------- | -------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel anymotion threshold dependent on the
 *	range values
 *
 *  accel_range	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_anymotion_thres(
_u8 accel_anymotion_thres);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL ANYMOTION DURATION  */
/*****************************************************/
/*!
 *	@brief This API used to read the accel anymotion duration
 *	from page one register from 0x12 bit 0 to 1
 *
 *	@param accel_anymotion_dur : The value of accel anymotion duration
 * accel_anymotion_dur  | result
 *  ------------------- | -------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_anymotion_dur(
_u8 *accel_anymotion_dur);
/*!
 *	@brief This API used to write the accel anymotion duration
 *	from page one register from 0x12 bit 0 to 1
 *
 *	@param accel_anymotion_dur : The value of accel anymotion duration
 *  accel_anymotion_dur | result
 *  ------------------- | -------------
 *              0x01    | ENABLED
 *              0x00    | DISABLED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_anymotion_dur(
_u8 accel_anymotion_dur);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL ANYMOTION AXIS ENABLE  */
/*****************************************************/
/*!
 *	@brief This API used to read the accel anymotion enable
 *	from page one register from 0x12 bit 2 to 4
 *
 *	@param data : The value of accel anymotion enable
 *	    data     | result
 *  ------------ | -------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param channel : The value of accel anymotion axis selection
 *           channel                           | value
 *     --------------------------              | ----------
 *     BNO055_ACCEL_ANYMOTION_NOMOTION_X_AXIS  |   0
 *     BNO055_ACCEL_ANYMOTION_NOMOTION_Y_AXIS  |   1
 *     BNO055_ACCEL_ANYMOTION_NOMOTION_Y_AXIS  |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_anymotion_nomotion_axis_enable(
_u8 channel, _u8 *data);
/*!
 *	@brief This API used to write the accel anymotion enable
 *	from page one register from 0x12 bit 2 to 4
 *
 *	@param data : The value of accel anymotion enable
 *	    data     | result
 *  ------------ | -------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param channel : The value of accel anymotion axis selection
 *           channel                           | value
 *     ------------------------------------    | ----------
 *     BNO055_ACCEL_ANYMOTION_NOMOTION_X_AXIS  |   0
 *     BNO055_ACCEL_ANYMOTION_NOMOTION_Y_AXIS  |   1
 *     BNO055_ACCEL_ANYMOTION_NOMOTION_Y_AXIS  |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_anymotion_nomotion_axis_enable(
_u8 channel, _u8 data);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL HIGHG AXIS ENABLE  */
/*****************************************************/
/*!
 *	@brief This API used to read the accel highg enable
 *	from page one register from 0x12 bit 5 to 7
 *
 *	@param data : The value of accel highg enable
 *      data     | result
 *  ------------ | -------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param channel : The value of accel highg axis selection
 *               channel          | value
 *     -------------------------- | ----------
 *     BNO055_ACCEL_HIGH_G_X_AXIS |   0
 *     BNO055_ACCEL_HIGH_G_Y_AXIS |   1
 *     BNO055_ACCEL_HIGH_G_Z_AXIS |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_axis_enable(
_u8 channel, _u8 *data);
/*!
 *	@brief This API used to write the accel highg enable
 *	from page one register from 0x12 bit 5 to 7
 *
 *	@param data : The value of accel highg enable
 *      data     | result
 *  ------------ | -------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param channel : The value of accel highg axis selection
 *               channel          | value
 *     -------------------------- | ----------
 *     BNO055_ACCEL_HIGH_G_X_AXIS |   0
 *     BNO055_ACCEL_HIGH_G_Y_AXIS |   1
 *     BNO055_ACCEL_HIGH_G_Z_AXIS |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_axis_enable(
_u8 channel, _u8 data);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL HIGHG DURATION */
/*****************************************************/
/*!
 *	@brief This API used to read the accel highg duration
 *	from page one register from 0x13 bit 0 to 7
 *
 *	@param accel_high_g_dur : The value of accel highg duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note The high-g interrupt trigger delay according
 *	to [highg duration + 1] * 2 ms
 *
 *	in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_dur(
_u8 *accel_high_g_dur);
/*!
 *	@brief This API used to write the accel highg duration
 *	from page one register from 0x13 bit 0 to 7
 *
 *	@param accel_high_g_dur : The value of accel highg duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note The high-g interrupt trigger delay according
 *	to [highg duration + 1] * 2 ms
 *
 *	in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_dur(
_u8 accel_high_g_dur);
/*****************************************************/
/**\name FUNCTIONS FOR ACCEL HIGHG THRESHOLD */
/*****************************************************/
/*!
 *	@brief This API used to read the accel highg threshold
 *	from page one register from 0x14 bit 0 to 7
 *
 *	@param accel_high_g_thres : The value of accel highg threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel highg interrupt threshold dependent
 *	for accel g range
 *
 *  accel_range	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    7.81mg     |   1LSB
 *     4g        |    15.63mg    |   1LSB
 *     8g        |    31.25mg    |   1LSB
 *     16g       |    62.5mg     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_thres(
_u8 *accel_high_g_thres);
/*!
 *	@brief This API used to write the accel highg threshold
 *	from page one register from 0x14 bit 0 to 7
 *
 *	@param accel_high_g_thres : The value of accel highg threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel highg interrupt threshold dependent
 *	for accel g range
 *
 *  accel_range	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    7.81mg     |   1LSB
 *     4g        |    15.63mg    |   1LSB
 *     8g        |    31.25mg    |   1LSB
 *     16g       |    62.5mg     |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_thres(
_u8 accel_high_g_thres);
/**************************************************************/
/**\name FUNCTIONS FOR ACCEL SLOWNOMOTION THRESHOLD */
/**************************************************************/
/*!
 *	@brief This API used to read the accel slownomotion threshold
 *	from page one register from 0x15 bit 0 to 7
 *
 *	@param accel_slow_no_motion_thres :
 *	The value of accel slownomotion threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel slow no motion interrupt threshold dependent
 *	for accel g range
 *
 *  accel_range	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_thres(
_u8 *accel_slow_no_motion_thres);
/*!
 *	@brief This API used to write the accel slownomotion threshold
 *	from page one register from 0x15 bit 0 to 7
 *
 *	@param accel_slow_no_motion_thres :
 *	The value of accel slownomotion threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Accel slow no motion interrupt threshold dependent
 *	for accel g range
 *
 *  accel_range	 |	threshold	 |	LSB
 * ------------- | ------------- | ---------
 *     2g        |    3.19mg     |   1LSB
 *     4g        |    7.81mg     |   1LSB
 *     8g        |    15.63mg    |   1LSB
 *     16g       |    31.25mg    |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_thres(
_u8 accel_slow_no_motion_thres);
/**************************************************************/
/**\name FUNCTIONS FOR ACCEL SLOWNOMOTION ENABLE */
/**************************************************************/
/*!
 *	@brief This API used to read accel slownomotion enable
 *	from page one register from 0x16 bit 0
 *
 *	@param accel_slow_no_motion_en : The value of accel slownomotion enable
 *	  accel_slow_no_motion_en   | result
 *     ------------------------ | --------
 *              0x01            | Slow motion
 *              0x00            | No motion
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_enable(
_u8 *accel_slow_no_motion_en);
/*!
 *	@brief This API used to write accel slownomotion enable
 *	from page one register from 0x16 bit 0
 *
 *	@param accel_slow_no_motion_en : The value of accel slownomotion enable
 *	  accel_slow_no_motion_en   | result
 *     ------------------------ | --------
 *              0x01            | Slow motion
 *              0x00            | No motion
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_enable(
_u8 accel_slow_no_motion_en);
/**************************************************************/
/**\name FUNCTIONS FOR ACCEL SLOWNOMOTION DURATION */
/**************************************************************/
/*!
 *	@brief This API used to read accel slownomotion duration
 *	from page one register from 0x16 bit 1 to 6
 *
 *	@param accel_slow_no_motion_dur : The value of accel slownomotion duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_slow_no_motion_dur(
_u8 *accel_slow_no_motion_dur);
/*!
 *	@brief This API used to write accel slownomotion duration
 *	from page one register from 0x16 bit 1 to 6
 *
 *	@param accel_slow_no_motion_dur : The value of accel slownomotion duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_slow_no_motion_dur(
_u8 accel_slow_no_motion_dur);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANYMOTION AXIS ENABLE */
/**************************************************************/
/*!
 *	@brief This API used to read the gyro anymotion enable
 *	from page one register from 0x17 bit 0 to 2
 *
 *	@param data : The value of gyro anymotion enable
 *      data     | result
 *  ------------ |-------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param channel : The value of gyro anymotion axis selection
 *               channel              | value
 *     ------------------------       | ----------
 *     BNO055_GYRO_ANYMOTIONX_AXIS    |   0
 *     BNO055_GYRO_ANYMOTIONY_AXIS    |   1
 *     BNO055_GYRO_ANYMOTIONZ_AXIS    |   2
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_axis_enable(
_u8 channel, _u8 *data);
/*!
 *	@brief This API used to write the gyro anymotion enable
 *	from page one register from 0x17 bit 0 to 2
 *
 *	@param data : The value of gyro anymotion enable
 *      data     | result
 *  ------------ |-------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param channel : The value of gyro anymotion axis selection
 *               channel              | value
 *     ------------------------       | ----------
 *     BNO055_GYRO_ANYMOTIONX_AXIS    |   0
 *     BNO055_GYRO_ANYMOTIONY_AXIS    |   1
 *     BNO055_GYRO_ANYMOTIONZ_AXIS    |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_axis_enable(
_u8 channel, _u8 data);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE ENABLE */
/**************************************************************/
/*!
 *	@brief This API used to read the gyro highrate enable
 *	from page one register from 0x17 bit 3 to 5
 *
 *	@param data : The value of gyro highrate enable
 *      data     | result
 *  ------------ |-------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param channel : The value of gyro highrate axis selection
 *               channel              | value
 *     ------------------------       | ----------
 *     BNO055_GYRO_HIGHRATE_X_AXIS    |   0
 *     BNO055_GYRO_HIGHRATE_Y_AXIS    |   1
 *     BNO055_GYRO_HIGHRATE_Z_AXIS    |   2
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_axis_enable(
_u8 channel, _u8 *data);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE AXIS ENABLE */
/**************************************************************/
/*!
 *	@brief This API used to write the gyro highrate enable
 *	from page one register from 0x17 bit 3 to 5
 *
 *	@param data : The value of gyro highrate enable
 *      data     | result
 *  ------------ |-------------
 *      0x01     | ENABLED
 *      0x00     | DISABLED
 *	@param channel : The value of gyro highrate axis selection
 *               channel              | value
 *     ------------------------       | ----------
 *     BNO055_GYRO_HIGHRATE_X_AXIS    |   0
 *     BNO055_GYRO_HIGHRATE_Y_AXIS    |   1
 *     BNO055_GYRO_HIGHRATE_Z_AXIS    |   2
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_axis_enable(
_u8 channel, _u8 data);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANYMOTION FILTER */
/**************************************************************/
/*!
 *	@brief This API used to read gyro anymotion filter
 *	from page one register from 0x17 bit 6
 *
 *	@param gyro_anymotion_filter : The value of gyro anymotion filter
 *   gyro_anymotion_filter  | result
 *  ----------------------  |------------
 *      0x00                | FILTERED
 *      0x01                | UNFILTERED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_filter(
_u8 *gyro_anymotion_filter);
/*!
 *	@brief This API used to write gyro anymotion filter
 *	from page one register from 0x17 bit 6
 *
 *	@param gyro_anymotion_filter : The value of gyro anymotion filter
 *   gyro_anymotion_filter  | result
 *  ----------------------  |------------
 *      0x00                | FILTERED
 *      0x01                | UNFILTERED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_filter(
_u8 gyro_anymotion_filter);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE FILTER */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate filter
 *	from page one register from 0x17 bit 7
 *
 *	@param gyro_highrate_filter : The value of gyro highrate filter
 *   gyro_highrate_filter  | result
 *  ---------------------- |------------
 *         0x00            | FILTERED
 *         0x01            | UNFILTERED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_filter(
_u8 *gyro_highrate_filter);
/*!
 *	@brief This API used to write gyro highrate filter
 *	from page one register from 0x17 bit 7
 *
 *   gyro_highrate_filter  | result
 *  ---------------------- |------------
 *         0x00            | FILTERED
 *         0x01            | UNFILTERED
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_filter(
_u8 gyro_highrate_filter);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE X THRESHOLD */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate x threshold
 *	from page one register from 0x18 bit 0 to 4
 *
 *	@param gyro_highrate_x_thres : The value of gyro x highrate threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  gyro_range	  |	threshold		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.5dps      |   1LSB
 *     1000       |    31.25dps     |   1LSB
 *     500        |    15.625dps    |   1LSB
 *     125        |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_thres(
_u8 *gyro_highrate_x_thres);
/*!
 *	@brief This API used to write gyro highrate x threshold
 *	from page one register from 0x18 bit 0 to 4
 *
 *	@param gyro_highrate_x_thres : The value of gyro highrate x threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  gyro_range	  |	threshold		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.5dps      |   1LSB
 *     1000       |    31.25dps     |   1LSB
 *     500        |    15.625dps    |   1LSB
 *     125        |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_thres(
_u8 gyro_highrate_x_thres);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE X HYSTERESIS */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate x hysteresis
 *	from page one register from 0x18 bit 5 to 6
 *
 *	@param gyro_highrate_x_hyst : The value of gyro highrate x hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * gyro_highrate_x_hyst) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  gyro_range	  |	hysteresis		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.26dps     |   1LSB
 *     1000       |    31.13dps     |   1LSB
 *     500        |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_hyst(
_u8 *gyro_highrate_x_hyst);
/*!
 *	@brief This API used to write gyro highrate x hysteresis
 *	from page one register from 0x18 bit 5 to 6
 *
 *	@param gyro_highrate_x_hyst : The value of gyro highrate x hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * gyro_highrate_x_hyst) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  gyro_range	  |	hysteresis		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.26dps     |   1LSB
 *     1000       |    31.13dps     |   1LSB
 *     500        |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_hyst(
_u8 gyro_highrate_x_hyst);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE X DURATION */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate x duration
 *	from page one register from 0x19 bit 0 to 7
 *
 *	@param gyro_highrate_x_dur : The value of gyro highrate x duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + gyro_highrate_x_dur)*2.5ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_dur(
_u8 *gyro_highrate_x_dur);
/*!
 *	@brief This API used to write gyro highrate x duration
 *	from page one register from 0x19 bit 0 to 7
 *
 *	@param gyro_highrate_x_dur : The value of gyro highrate x duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + gyro_highrate_x_dur)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_x_dur(
_u8 gyro_highrate_x_dur);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Y THRESHOLD */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate y threshold
 *	from page one register from 0x1A bit 0 to 4
 *
 *	@param gyro_highrate_y_thres : The value of gyro highrate y threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  gyro_range	  |	threshold		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.5dps      |   1LSB
 *     1000       |    31.25dps     |   1LSB
 *     500        |    15.625dps    |   1LSB
 *     125        |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_thres(
_u8 *gyro_highrate_y_thres);
/*!
 *	@brief This API used to write gyro highrate y threshold
 *	from page one register from 0x1A bit 0 to 4
 *
 *	@param gyro_highrate_y_thres : The value of gyro highrate y threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  gyro_range	  |	threshold	    |	LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.5dps      |   1LSB
 *     1000       |    31.25dps     |   1LSB
 *     500        |    15.625dps    |   1LSB
 *     125        |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_thres(
_u8 gyro_highrate_y_thres);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Y HYSTERESIS */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate y hysteresis
 *	from page one register from 0x1A bit 5 to 6
 *
 *	@param gyro_highrate_y_hyst : The value of gyro highrate y hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * gyro_highrate_y_hyst) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  gyro_range	  |	hysteresis		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.26dps     |   1LSB
 *     1000       |    31.13dps     |   1LSB
 *     500        |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_hyst(
_u8 *gyro_highrate_y_hyst);
/*!
 *	@brief This API used to write gyro highrate y hysteresis
 *	from page one register from 0x1A bit 5 to 6
 *
 *	@param gyro_highrate_y_hyst : The value of gyro highrate y hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * gyro_highrate_y_hyst) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  gyro_range	  |	hysteresis		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.26dps     |   1LSB
 *     1000       |    31.13dps     |   1LSB
 *     500        |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_hyst(
_u8 gyro_highrate_y_hyst);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Y DURATION */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate y duration
 *	from page one register from 0x1B bit 0 to 7
 *
 *	@param gyro_highrate_y_dur : The value of gyro highrate y duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + gyro_highrate_y_dur)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_y_dur(
_u8 *gyro_highrate_y_dur);
/*!
 *	@brief This API used to write gyro highrate y duration
 *	from page one register from 0x1B bit 0 to 7
 *
 *	@param gyro_highrate_y_dur : The value of gyro highrate y duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + gyro_highrate_y_dur)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_dur(
_u8 gyro_highrate_y_dur);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Z THRESHOLD */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate z threshold
 *	from page one register from 0x1C bit 0 to 4
 *
 *	@param gyro_highrate_z_thres : The value of gyro highrate z threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  gyro_range	  |	threshold		|    LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.5dps      |   1LSB
 *     1000       |    31.25dps     |   1LSB
 *     500        |    15.625dps    |   1LSB
 *     125        |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_thres(
_u8 *gyro_highrate_z_thres);
/*!
 *	@brief This API used to write gyro highrate z threshold
 *	from page one register from 0x1C bit 0 to 4
 *
 *	@param gyro_highrate_z_thres : The value of gyro highrate z threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate threshold dependent on the
 *	selection of gyro range
 *
 *  gyro_range	  |	threshold		|    LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.5dps      |   1LSB
 *     1000       |    31.25dps     |   1LSB
 *     500        |    15.625dps    |   1LSB
 *     125        |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_thres(
_u8 gyro_highrate_z_thres);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Z HYSTERESIS */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate z hysteresis
 *	from page one register from 0x1C bit 5 to 6
 *
 *	@param gyro_highrate_z_hyst : The value of gyro highrate z hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * gyro_highrate_z_hyst) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  gyro_range	  |	hysteresis		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.26dps     |   1LSB
 *     1000       |    31.13dps     |   1LSB
 *     500        |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_hyst(
_u8 *gyro_highrate_z_hyst);
/*!
 *	@brief This API used to write gyro highrate z hysteresis
 *	from page one register from 0x1C bit 5 to 6
 *
 *	@param gyro_highrate_z_hyst : The value of gyro highrate z hysteresis
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro high rate hysteresis calculated by
 *
 *	using this (255 + 256 * gyro_highrate_z_hyst) *4 LSB
 *
 *	The high rate value scales with the range setting
 *
 *  gyro_range	  |	hysteresis		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.26dps     |   1LSB
 *     1000       |    31.13dps     |   1LSB
 *     500        |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_hyst(
_u8 gyro_highrate_z_hyst);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO HIGHRATE Z DURATION */
/**************************************************************/
/*!
 *	@brief This API used to read gyro highrate z duration
 *	from page one register from 0x1D bit 0 to 7
 *
 *	@param gyro_highrate_z_dur : The value of gyro highrate z duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + gyro_highrate_z_dur)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_dur(
_u8 *gyro_highrate_z_dur);
/*!
 *	@brief This API used to write gyro highrate z duration
 *	from page one register from 0x1D bit 0 to 7
 *
 *	@param gyro_highrate_z_dur : The value of gyro highrate z duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro highrate duration calculate by using the formula
 *
 *	(1 + gyro_highrate_z_dur)*2.5ms
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_dur(
_u8 gyro_highrate_z_dur);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANYMOTION THRESHOLD */
/**************************************************************/
/*!
 *	@brief This API used to read gyro anymotion threshold
 *	from page one register from 0x1E bit 0 to 6
 *
 *	@param gyro_anymotion_thres : The value of gyro anymotion threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro anymotion interrupt threshold dependent
 *	on the selection of gyro range
 *
 *  gyro_range	  |	threshold		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    1dps       |   1LSB
 *     1000       |    0.5dps     |   1LSB
 *     500        |    0.25dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_thres(
_u8 *gyro_anymotion_thres);
/*!
 *	@brief This API used to write gyro anymotion threshold
 *	from page one register from 0x1E bit 0 to 6
 *
 *	@param gyro_anymotion_thres : The value of gyro anymotion threshold
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 *	@note Gyro anymotion interrupt threshold dependent
 *	on the selection of gyro range
 *
 *  gyro_range	  |	threshold     |     LSB
 * -------------  | ------------- | ---------
 *     2000       |    1dps       |   1LSB
 *     1000       |    0.5dps     |   1LSB
 *     500        |    0.25dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_thres(
_u8 gyro_anymotion_thres);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANYMOTION SLOPE SAMPLES */
/**************************************************************/
/*!
 *	@brief This API used to read gyro anymotion slope samples
 *	from page one register from 0x1F bit 0 to 1
 *
 *	@param gyro_anymotion_slope_samples :
 *	The value of gyro anymotion slope samples
 *  gyro_anymotion_slope_samples   |   result
 *  ----------------------------   | -----------
 *            0                    |    8 samples
 *            1                    |    16 samples
 *            2                    |    32 samples
 *            3                    |    64 samples
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_slope_samples(
_u8 *gyro_anymotion_slope_samples);
/*!
 *	@brief This API used to write gyro anymotion slope samples
 *	from page one register from 0x1F bit 0 to 1
 *
 *	@param gyro_anymotion_slope_samples :
 *	The value of gyro anymotion slope samples
 *  gyro_anymotion_slope_samples   |   result
 *  ----------------------------   | -----------
 *            0                    |    8 samples
 *            1                    |    16 samples
 *            2                    |    32 samples
 *            3                    |    64 samples
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_slope_samples(
_u8 gyro_anymotion_slope_samples);
/**************************************************************/
/**\name FUNCTIONS FOR GYRO ANYMOTION AWAKE DURATION */
/**************************************************************/
/*!
 *	@brief This API used to read gyro anymotion awake duration
 *	from page one register from 0x1F bit 2 to 3
 *
 *	@param gyro_awake_dur : The value of gyro anymotion awake duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_awake_duration(
_u8 *gyro_awake_dur);
/*!
 *	@brief This API used to write gyro anymotion awake duration
 *	from page one register from 0x1F bit 2 to 3
 *
 *	@param gyro_awake_dur : The value of gyro anymotion awake duration
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval 1 -> Error
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_awake_duration(
_u8 gyro_awake_dur);
#endif
