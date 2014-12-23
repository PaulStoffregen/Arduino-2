/*
*
****************************************************************************
* Copyright (C) 2013 - 2014 Bosch Sensortec GmbH
*
* File : bno055.c
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

/*********************************************************/
/*				INCLUDES	*/
/*******************************************************/
#include "BNO055.h"
/*! file <BNO055 >
    brief <Sensor driver for BNO055> */
/*	STRUCTURE DEFINITIONS	*/
static struct bno055_t *p_bno055;
/*	 LOCAL FUNCTIONS	*/
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
BNO055_RETURN_FUNCTION_TYPE bno055_init(struct bno055_t *bno055)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 a_data_u8r = BNO055_ZERO_U8X;
	_u8 pagezero = PAGE_ZERO;
	_u8 a_SWID_u8r[2] = {0, 0};
	/* stuct parameters are assign to bno055*/
	p_bno055 = bno055;
	/* Write the default page as zero*/
	comres += p_bno055->BNO055_BUS_WRITE_FUNC
	(p_bno055->dev_addr,
	BNO055_PAGE_ID__REG, &pagezero, 1);
	/* Read the chip id of the sensor from page
	zero 0x00 register*/
	comres += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_CHIP_ID__REG, &a_data_u8r, 1);
	p_bno055->chip_id = a_data_u8r;
	/* Read the accel revision id from page
	zero 0x01 register*/
	comres += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_ACCEL_REV_ID__REG, &a_data_u8r, 1);
	p_bno055->accel_rev_id = a_data_u8r;
	/* Read the mag revision id from page
	zero 0x02 register*/
	comres += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_MAG_REV_ID__REG, &a_data_u8r, 1);
	p_bno055->mag_rev_id = a_data_u8r;
	/* Read the gyro revision id from page
	zero 0x02 register*/
	comres += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_GYRO_REV_ID__REG, &a_data_u8r, 1);
	p_bno055->gyro_rev_id = a_data_u8r;
	/* Read the boot loader revision from page
	zero 0x06 register*/
	comres += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_BL_REV_ID__REG, &a_data_u8r, 1);
	p_bno055->bl_rev_id = a_data_u8r;
	/* Read the software revision id from page
	zero 0x04 and 0x05 register( 2 bytes of data)*/
	comres += p_bno055->BNO055_BUS_READ_FUNC(p_bno055->dev_addr,
	BNO055_SW_REV_ID_LSB__REG, a_SWID_u8r, 2);
	a_SWID_u8r[0] = BNO055_GET_BITSLICE(a_SWID_u8r[0],
	BNO055_SW_REV_ID_LSB);
	p_bno055->sw_rev_id = (_u16)
	((((_u32)((_u8)a_SWID_u8r[1])) <<
	BNO055_SHIFT_8_POSITION) | (a_SWID_u8r[0]));
	/* Read the page id from the register 0x07*/
	comres += p_bno055->BNO055_BUS_READ_FUNC
	(p_bno055->dev_addr,
	BNO055_PAGE_ID__REG, &a_data_u8r, 1);
	p_bno055->page_id = a_data_u8r;

	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_write_register(_u8 addr,
_u8 *data, _u8 len)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
			/* Write the values of respective given register */
			comres = p_bno055->BNO055_BUS_WRITE_FUNC
			(p_bno055->dev_addr, addr, data, len);
		}
	return comres;
}
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
_u8 *data, _u8 len)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/* Read the value from given register*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr, addr, data, len);
		}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_chip_id(_u8 *chip_id)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 a_data_u8r = BNO055_ZERO_U8X;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (comres == SUCCESS) {
			/* Read the chip id*/
			comres += p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CHIP_ID__REG, &a_data_u8r, 1);
			*chip_id = a_data_u8r;
		} else {
		comres = ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_sw_rev_id(_u16 *sw_id)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	/* array having the software revision id
	a_data_u8r[0] - LSB
	a_data_u8r[1] - MSB*/
	_u8 a_data_u8r[2] = {0, 0};
	/* Check the struct  p_bno055 is empty*/
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (comres == SUCCESS) {
			/* Read the two byte value of software
			revision id*/
			comres += p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SW_REV_ID_LSB__REG, a_data_u8r, 2);
			a_data_u8r[0] =
			BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_SW_REV_ID_LSB);
			*sw_id = (_u16)
			((((_u32)((_u8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		comres = ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_page_id(_u8 *page_id)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 a_data_u8r = BNO055_ZERO_U8X;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/* Read the page id form 0x07*/
		comres = p_bno055->BNO055_BUS_READ_FUNC
		(p_bno055->dev_addr,
		BNO055_PAGE_ID__REG, &a_data_u8r, 1);
		if (comres == SUCCESS) {
			a_data_u8r = BNO055_GET_BITSLICE(a_data_u8r,
			BNO055_PAGE_ID);
			*page_id = a_data_u8r;
			p_bno055->page_id = a_data_u8r;
		} else {
		comres = ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_write_page_id(_u8 page_id)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
			/* Read the current page*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_PAGE_ID__REG, &v_data_u8r, 1);
			/* Check condition for communication success*/
			if (comres == SUCCESS) {
				v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_PAGE_ID, page_id);
				/* Write the page id*/
				comres += p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_PAGE_ID__REG, &v_data_u8r, 1);
				if (comres == SUCCESS)
					p_bno055->page_id = page_id;
			} else {
			comres = ERROR;
			}
		}
	return comres;
}
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
_u8 *accel_rev_id)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 a_data_u8r = BNO055_ZERO_U8X;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (comres == SUCCESS) {
			/* Read the accel revision id */
			comres += p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_REV_ID__REG, &a_data_u8r, 1);
			*accel_rev_id = a_data_u8r;
		} else {
		comres = ERROR;
		}
	}
	return comres;
}
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
_u8 *mag_rev_id)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 a_data_u8r = BNO055_ZERO_U8X;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
			if (comres == SUCCESS) {
				/* Read the mag revision id */
				comres += p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_MAG_REV_ID__REG, &a_data_u8r, 1);
				*mag_rev_id = a_data_u8r;
			} else {
			comres = ERROR;
			}
		}
	return comres;
}
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
_u8 *gyro_rev_id)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 a_data_u8r = BNO055_ZERO_U8X;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (comres == SUCCESS) {
			/* Read the gyro revision id */
			comres += p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_REV_ID__REG, &a_data_u8r, 1);
			*gyro_rev_id = a_data_u8r;
		} else {
		comres = ERROR;
		}
	}
	return comres;
}
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
_u8 *bl_rev_id)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 a_data_u8r = BNO055_ZERO_U8X;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (comres == SUCCESS) {
			/* Read the boot loader revision id */
			comres += p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_BL_REV_ID__REG, &a_data_u8r, 1);
			*bl_rev_id = a_data_u8r;
		} else {
		comres = ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_x(_s16 *accel_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the accel x value
	a_data_u8r[0] - LSB
	a_data_u8r[1] - MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the accel x axis two byte value*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_X_LSB_VALUEX__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACCEL_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACCEL_DATA_X_MSB_VALUEX);
			*accel_x = (_s16)((((_s32)
			(_s8)(a_data_u8r[1])) <<
			(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_y(_s16 *accel_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the accel y value
	a_data_u8r[0] - LSB
	a_data_u8r[1] - MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the accel y axis two byte value*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACCEL_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACCEL_DATA_Y_MSB_VALUEY);
			*accel_y = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_accel_z(_s16 *accel_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the accel z value
	a_data_u8r[0] - LSB
	a_data_u8r[1] - MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the accel z axis two byte value*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACCEL_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACCEL_DATA_Z_MSB_VALUEZ);
			*accel_z = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
struct bno055_accel *accel)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the accel xyz value
	a_data_u8r[0] - x->LSB
	a_data_u8r[1] - x->MSB
	a_data_u8r[2] - y->MSB
	a_data_u8r[3] - y->MSB
	a_data_u8r[4] - z->MSB
	a_data_u8r[5] - z->MSB
	*/
	_u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_X_LSB_VALUEX__REG, a_data_u8r, 6);
			/* Data X*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_ACCEL_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_ACCEL_DATA_X_MSB_VALUEX);
			accel->x = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data Y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_ACCEL_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_ACCEL_DATA_Y_MSB_VALUEY);
			accel->y = (_s16)((((_s32)
			((_s8)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data Z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_ACCEL_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_ACCEL_DATA_Z_MSB_VALUEZ);
			accel->z = (_s16)((((_s32)
			((_s8)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_x(_s16 *mag_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the mag x value
	a_data_u8r[0] - x->LSB
	a_data_u8r[1] - x->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/*Read the mag x two bytes of data */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_X_LSB_VALUEX__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_DATA_X_MSB_VALUEX);
			*mag_x = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_y(_s16 *mag_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the mag x value
	a_data_u8r[0] - y->LSB
	a_data_u8r[1] - y->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/*Read the mag y two bytes of data */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_DATA_Y_MSB_VALUEY);
			*mag_y = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_z(_s16 *mag_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the mag x value
	a_data_u8r[0] - z->LSB
	a_data_u8r[1] - z->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			/*Read the mag z two bytes of data */
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_DATA_Z_MSB_VALUEZ);
			*mag_z = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_mag_xyz(struct bno055_mag *mag)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the mag xyz value
	a_data_u8r[0] - x->LSB
	a_data_u8r[1] - x->MSB
	a_data_u8r[2] - y->MSB
	a_data_u8r[3] - y->MSB
	a_data_u8r[4] - z->MSB
	a_data_u8r[5] - z->MSB
	*/
	_u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/*Read the six byte value of mag xyz*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_X_LSB_VALUEX__REG, a_data_u8r, 6);
			/* Data X*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_MAG_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_MAG_DATA_X_MSB_VALUEX);
			mag->x = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data Y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_MAG_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_MAG_DATA_Y_MSB_VALUEY);
			mag->y = (_s16)((((_s32)
			((_s8)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data Z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_MAG_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_MAG_DATA_Z_MSB_VALUEZ);
			mag->z = (_s16)((((_s32)
			((_s8)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_x(_s16 *gyro_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the gyro 16 bit x value*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_X_LSB_VALUEX__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYRO_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYRO_DATA_X_MSB_VALUEX);
			*gyro_x = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_y(_s16 *gyro_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of gyro y */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYRO_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYRO_DATA_Y_MSB_VALUEY);
			*gyro_y = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_z(_s16 *gyro_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the gyro z 16 bit value*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYRO_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYRO_DATA_Z_MSB_VALUEZ);
			*gyro_z = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gyro_xyz(struct bno055_gyro *gyro)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the accel xyz value
	a_data_u8r[0] - x->LSB
	a_data_u8r[1] - x->MSB
	a_data_u8r[2] - y->MSB
	a_data_u8r[3] - y->MSB
	a_data_u8r[4] - z->MSB
	a_data_u8r[5] - z->MSB
	*/
	_u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the six bytes data of gyro xyz*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_X_LSB_VALUEX__REG, a_data_u8r, 6);
			/* Data x*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GYRO_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GYRO_DATA_X_MSB_VALUEX);
			gyro->x = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_GYRO_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_GYRO_DATA_Y_MSB_VALUEY);
			gyro->y = (_s16)((((_s32)
			((_s8)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_GYRO_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_GYRO_DATA_Z_MSB_VALUEZ);
			gyro->z = (_s16)((((_s32)
			((_s8)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_h(_s16 *euler_h)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the Euler h value
	a_data_u8r[0] - h->LSB
	a_data_u8r[1] - h->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the eulre heading data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_H_LSB_VALUEH__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE
			(a_data_u8r[0],
			BNO055_EULER_H_LSB_VALUEH);
			a_data_u8r[1] = BNO055_GET_BITSLICE
			(a_data_u8r[1],
			BNO055_EULER_H_MSB_VALUEH);
			*euler_h = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_r(_s16 *euler_r)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the Euler h value
	a_data_u8r[0] - r->LSB
	a_data_u8r[1] - r->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the Euler roll data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_R_LSB_VALUER__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_EULER_R_LSB_VALUER);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_EULER_R_MSB_VALUER);
			*euler_r = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_euler_p(_s16 *euler_p)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the Euler p value
	a_data_u8r[0] - p->LSB
	a_data_u8r[1] - p->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the Euler p data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_P_LSB_VALUEP__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_EULER_P_LSB_VALUEP);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_EULER_P_MSB_VALUEP);
			*euler_p = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
struct bno055_euler *euler)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the Euler hrp value
	a_data_u8r[0] - h->LSB
	a_data_u8r[1] - h->MSB
	a_data_u8r[2] - r->MSB
	a_data_u8r[3] - r->MSB
	a_data_u8r[4] - p->MSB
	a_data_u8r[5] - p->MSB
	*/
	_u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the six byte of Euler hrp data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_H_LSB_VALUEH__REG, a_data_u8r, 6);
			/* Data h*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_EULER_H_LSB_VALUEH);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_EULER_H_MSB_VALUEH);
			euler->h = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data r*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_EULER_R_LSB_VALUER);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_EULER_R_MSB_VALUER);
			euler->r = (_s16)((((_s32)
			((_s8)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data p*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_EULER_P_LSB_VALUEP);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_EULER_P_MSB_VALUEP);
			euler->p = (_s16)((((_s32)
			((_s8)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_w(
_s16 *quaternion_w)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the Quaternion w value
	a_data_u8r[0] - w->LSB
	a_data_u8r[1] - w->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of quaternion w data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_W_LSB_VALUEW__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_QUATERNION_DATA_W_LSB_VALUEW);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_QUATERNION_DATA_W_MSB_VALUEW);
			*quaternion_w = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_x(
_s16 *quaternion_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the quaternion x value
	a_data_u8r[0] - x->LSB
	a_data_u8r[1] - x->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of quaternion x data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_X_LSB_VALUEX__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_QUATERNION_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_QUATERNION_DATA_X_MSB_VALUEX);
			*quaternion_x = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_y(
_s16 *quaternion_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the quaternion y value
	a_data_u8r[0] - y->LSB
	a_data_u8r[1] - y->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of quaternion y data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_Y_LSB_VALUEY__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE
			(a_data_u8r[0],
			BNO055_QUATERNION_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE
			(a_data_u8r[1],
			BNO055_QUATERNION_DATA_Y_MSB_VALUEY);
			*quaternion_y = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_quaternion_z(
_s16 *quaternion_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the quaternion z value
	a_data_u8r[0] - z->LSB
	a_data_u8r[1] - z->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of quaternion z data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_Z_LSB_VALUEZ__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_QUATERNION_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_QUATERNION_DATA_Z_MSB_VALUEZ);
			*quaternion_z = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
struct bno055_quaternion *quaternion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the quaternion wxyz value
	a_data_u8r[0] - w->LSB
	a_data_u8r[1] - w->MSB
	a_data_u8r[2] - x->LSB
	a_data_u8r[3] - x->MSB
	a_data_u8r[4] - y->MSB
	a_data_u8r[5] - y->MSB
	a_data_u8r[6] - z->MSB
	a_data_u8r[7] - z->MSB
	*/
	_u8 a_data_u8r[8] = {0, 0, 0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the eight byte value
			of quaternion wxyz data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_W_LSB_VALUEW__REG,
			a_data_u8r, 8);
			/* Data W*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_QUATERNION_DATA_W_LSB_VALUEW);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_QUATERNION_DATA_W_MSB_VALUEW);
			quaternion->w = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data X*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_QUATERNION_DATA_X_LSB_VALUEX);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_QUATERNION_DATA_X_MSB_VALUEX);
			quaternion->x = (_s16)((((_s32)
			((_s8)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data Y*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_QUATERNION_DATA_Y_LSB_VALUEY);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_QUATERNION_DATA_Y_MSB_VALUEY);
			quaternion->y = (_s16)((((_s32)
			((_s8)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
			/* Data Z*/
			a_data_u8r[6] = BNO055_GET_BITSLICE(a_data_u8r[6],
			BNO055_QUATERNION_DATA_Z_LSB_VALUEZ);
			a_data_u8r[7] = BNO055_GET_BITSLICE(a_data_u8r[7],
			BNO055_QUATERNION_DATA_Z_MSB_VALUEZ);
			quaternion->z = (_s16)((((_s32)
			((_s8)a_data_u8r[7])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[6]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_s16 *linear_accel_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the linear accel x value
	a_data_u8r[0] - x->LSB
	a_data_u8r[1] - x->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of linear accel x data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX);
			*linear_accel_x = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_s16 *linear_accel_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the linear accel y value
	a_data_u8r[0] - y->LSB
	a_data_u8r[1] - y->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of linear accel y data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY);
			*linear_accel_y = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_s16 *linear_accel_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the linear accel z value
	a_data_u8r[0] - z->LSB
	a_data_u8r[1] - z->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of linear accel z data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ);
			*linear_accel_z = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
struct bno055_linear_accel *linear_accel)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the linear accel xyz value
	a_data_u8r[0] - x->LSB
	a_data_u8r[1] - x->MSB
	a_data_u8r[2] - y->MSB
	a_data_u8r[3] - y->MSB
	a_data_u8r[4] - z->MSB
	a_data_u8r[5] - z->MSB
	*/
	_u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the six byte value
			of linear accel xyz data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX__REG,
			a_data_u8r, 6);
			/* Data x*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_LINEAR_ACCEL_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_LINEAR_ACCEL_DATA_X_MSB_VALUEX);
			linear_accel->x = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_LINEAR_ACCEL_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_LINEAR_ACCEL_DATA_Y_MSB_VALUEY);
			linear_accel->y = (_s16)((((_s32)
			((_s8)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_LINEAR_ACCEL_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_LINEAR_ACCEL_DATA_Z_MSB_VALUEZ);
			linear_accel->z = (_s16)((((_s32)
			((_s8)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_x(
_s16 *gravity_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the gravity x value
	a_data_u8r[0] - x->LSB
	a_data_u8r[1] - x->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of gravity x data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_X_LSB_VALUEX__REG,
			a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GRAVITY_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GRAVITY_DATA_X_MSB_VALUEX);
			*gravity_x = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_y(
_s16 *gravity_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the gravity y value
	a_data_u8r[0] - y->LSB
	a_data_u8r[1] - y->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of gravity y data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_Y_LSB_VALUEY__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GRAVITY_DATA_Y_LSB_VALUEY);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GRAVITY_DATA_Y_MSB_VALUEY);
			*gravity_y = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_gravity_z(
_s16 *gravity_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the gravity z value
	a_data_u8r[0] - z->LSB
	a_data_u8r[1] - z->MSB
	*/
	_u8 a_data_u8r[2] = {0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the two byte value
			of gravity z data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_Z_LSB_VALUEZ__REG, a_data_u8r, 2);
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GRAVITY_DATA_Z_LSB_VALUEZ);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GRAVITY_DATA_Z_MSB_VALUEZ);
			*gravity_z = (_s16)((((_s32)
			((_s8)a_data_u8r[1])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
struct bno055_gravity *gravity)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the gravity xyz value
	a_data_u8r[0] - x->LSB
	a_data_u8r[1] - x->MSB
	a_data_u8r[2] - y->MSB
	a_data_u8r[3] - y->MSB
	a_data_u8r[4] - z->MSB
	a_data_u8r[5] - z->MSB
	*/
	_u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the six byte value
			of gravity xyz data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_X_LSB_VALUEX__REG, a_data_u8r, 6);
			/* Data x*/
			a_data_u8r[0] = BNO055_GET_BITSLICE(a_data_u8r[0],
			BNO055_GRAVITY_DATA_X_LSB_VALUEX);
			a_data_u8r[1] = BNO055_GET_BITSLICE(a_data_u8r[1],
			BNO055_GRAVITY_DATA_X_MSB_VALUEX);
			gravity->x = (_s16)(((_s32)
			((_s8)a_data_u8r[1]) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[0]));
			/* Data y*/
			a_data_u8r[2] = BNO055_GET_BITSLICE(a_data_u8r[2],
			BNO055_GRAVITY_DATA_Y_LSB_VALUEY);
			a_data_u8r[3] = BNO055_GET_BITSLICE(a_data_u8r[3],
			BNO055_GRAVITY_DATA_Y_MSB_VALUEY);
			gravity->y = (_s16)((((_s32)
			((_s8)a_data_u8r[3])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[2]));
			/* Data z*/
			a_data_u8r[4] = BNO055_GET_BITSLICE(a_data_u8r[4],
			BNO055_GRAVITY_DATA_Z_LSB_VALUEZ);
			a_data_u8r[5] = BNO055_GET_BITSLICE(a_data_u8r[5],
			BNO055_GRAVITY_DATA_Z_MSB_VALUEZ);
			gravity->z = (_s16)((((_s32)
			((_s8)a_data_u8r[5])) <<
			BNO055_SHIFT_8_POSITION) | (a_data_u8r[4]));
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_read_temp_data(_s16 *temp)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 a_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the raw temperature data */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP__REG, &a_data_u8r, 1);
			*temp = a_data_u8r;
		} else {
		return ERROR;
		}
	}
	return comres;
}
#ifdef	BNO055_FLOAT_ENABLE
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
float *accel_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_x = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MSQ)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (comres == SUCCESS) {
			/* Read the accel raw x data*/
			comres += bno055_read_accel_x(&reg_accel_x);
			p_bno055->delay_msec(50);
			if (comres == SUCCESS) {
				/* Convert the raw accel x to m/s2*/
				data = (float)reg_accel_x/ACCEL_DIV_MSQ;
				*accel_x = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *accel_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_x = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	 /* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MG)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (comres == SUCCESS) {
			/* Read the accel raw x data*/
			comres += bno055_read_accel_x(&reg_accel_x);
			if (comres == SUCCESS) {
				/* Convert the raw accel x to m/s2*/
				data = (float)reg_accel_x/ACCEL_DIV_MG;
				*accel_x = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *accel_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_y = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MSQ)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (comres == SUCCESS) {
			comres += bno055_read_accel_y(&reg_accel_y);
			p_bno055->delay_msec(50);
			if (comres == SUCCESS) {
				/* Convert the raw accel y to m/s2*/
				data = (float)reg_accel_y/ACCEL_DIV_MSQ;
				*accel_y = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *accel_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_y = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MG)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (comres == SUCCESS) {
			/* Read the accel raw z data*/
			comres += bno055_read_accel_y(&reg_accel_y);
			if (comres == SUCCESS) {
				/* Convert the raw accel z to mg*/
				data = (float)reg_accel_y/ACCEL_DIV_MG;
				*accel_y = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *accel_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_z = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MSQ)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (comres == SUCCESS) {
			/* Read the accel raw z data*/
			comres += bno055_read_accel_z(&reg_accel_z);
			p_bno055->delay_msec(50);
			if (comres == SUCCESS) {
				/* Convert the raw accel z to m/s2*/
				data = (float)reg_accel_z/ACCEL_DIV_MSQ;
				*accel_z = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *accel_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_z = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2 */
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MG)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (comres == SUCCESS) {
			/* Read the accel raw z data*/
			comres += bno055_read_accel_z(&reg_accel_z);
			if (comres == SUCCESS) {
				/* Convert the raw accel x to mg*/
				data = (float)reg_accel_z/ACCEL_DIV_MG;
				*accel_z = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_accel_float *accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_accel reg_accel_xyz = {0, 0, 0};
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MSQ)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (comres == SUCCESS) {
			/* Read the accel raw xyz data*/
			comres += bno055_read_accel_xyz(&reg_accel_xyz);
			if (comres == SUCCESS) {
				/* Convert the accel raw xyz to meterpersecseq*/
				accel_xyz->x =
				(float)reg_accel_xyz.x/ACCEL_DIV_MSQ;
				accel_xyz->y =
				(float)reg_accel_xyz.y/ACCEL_DIV_MSQ;
				accel_xyz->z =
				(float)reg_accel_xyz.z/ACCEL_DIV_MSQ;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_accel_float *accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_accel reg_accel_xyz = {0, 0, 0};
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MG)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (comres == SUCCESS) {
			/* Read the accel raw y data*/
			comres += bno055_read_accel_xyz(&reg_accel_xyz);
			if (comres == SUCCESS) {
				/*Convert the accel raw xyz to millig */
				accel_xyz->x =
				(float)reg_accel_xyz.x/ACCEL_DIV_MG;
				accel_xyz->y =
				(float)reg_accel_xyz.y/ACCEL_DIV_MG;
				accel_xyz->z =
				(float)reg_accel_xyz.z/ACCEL_DIV_MG;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *mag_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_mag_x = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	/* Read raw mag x data */
	comres = bno055_read_mag_x(&reg_mag_x);
	if (comres == SUCCESS) {
		/* Convert the raw mag x to uT*/
		data = (float)reg_mag_x/MAG_DIV_MICROTESLA;
		*mag_x = data;
	} else {
	comres = ERROR;
	}
	return comres;
}
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
float *mag_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_mag_y = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	/* Read raw mag y data */
	comres = bno055_read_mag_y(&reg_mag_y);
	if (comres == SUCCESS) {
		/* Convert the raw mag y to uT*/
		data = (float)reg_mag_y/MAG_DIV_MICROTESLA;
		*mag_y = data;
	} else {
	comres = ERROR;
	}
	return comres;
}
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
float *mag_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_mag_z = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	/* Read raw mag z data */
	comres = bno055_read_mag_z(&reg_mag_z);
	if (comres == SUCCESS) {
		/* Convert the raw mag z to uT*/
		data = (float)reg_mag_z/MAG_DIV_MICROTESLA;
		*mag_z = data;
	} else {
	comres = ERROR;
	}
	return comres;
}
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
struct bno055_mag_float *mag_xyz_data)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_mag mag_xyz = {0, 0, 0};
	/* Read raw mag x data */
	comres = bno055_read_mag_xyz(&mag_xyz);
	if (comres == SUCCESS) {
		/* Convert mag raw xyz to uT*/
		mag_xyz_data->x = (float)mag_xyz.x/MAG_DIV_MICROTESLA;
		mag_xyz_data->y = (float)mag_xyz.y/MAG_DIV_MICROTESLA;
		mag_xyz_data->z = (float)mag_xyz.z/MAG_DIV_MICROTESLA;
	} else {
	comres = ERROR;
	}

	return comres;
}
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
float *gyro_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_x = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_DPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (comres == SUCCESS) {
			/* Read gyro raw x data */
			comres += bno055_read_gyro_x(&reg_gyro_x);
			if (comres == SUCCESS) {
				/* Convert the raw gyro x to dps*/
				data = (float)reg_gyro_x/GYRO_DIV_DPS;
				*gyro_x = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *gyro_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_x = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_RPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (comres == SUCCESS) {
			/* Read gyro raw x data */
			comres += bno055_read_gyro_x(&reg_gyro_x);
			if (comres == SUCCESS) {
				/* Convert the raw gyro x to rps*/
				data = (float)reg_gyro_x/GYRO_DIV_RPS;
				*gyro_x = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *gyro_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_y = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_DPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (comres == SUCCESS) {
			/* Read gyro raw y data */
			comres += bno055_read_gyro_y(&reg_gyro_y);
			if (comres == SUCCESS) {
				/* Convert the raw gyro x to dps*/
				data = (float)reg_gyro_y/GYRO_DIV_DPS;
				*gyro_y = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *gyro_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_y = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_RPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (comres == SUCCESS) {
			/* Read gyro raw y data */
			comres += bno055_read_gyro_y(&reg_gyro_y);
			if (comres == SUCCESS) {
				/* Convert the raw gyro x to rps*/
				data = (float)reg_gyro_y/GYRO_DIV_RPS;
				*gyro_y = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *gyro_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_z = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_DPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (comres == SUCCESS) {
			/* Read gyro raw z data */
			comres += bno055_read_gyro_z(&reg_gyro_z);
			if (comres == SUCCESS) {
				/* Convert the raw gyro x to dps*/
				data = (float)reg_gyro_z/GYRO_DIV_DPS;
				*gyro_z = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *gyro_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_z = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_RPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (comres == SUCCESS) {
			/* Read gyro raw x data */
			comres += bno055_read_gyro_z(&reg_gyro_z);
			if (comres == SUCCESS) {
				/* Convert the raw gyro x to rps*/
				data = (float)reg_gyro_z/GYRO_DIV_RPS;
				*gyro_z = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_gyro_float *gyro_xyz_data)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_gyro gyro_xyz = {0, 0, 0};
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_DPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (comres == SUCCESS) {
			/* Read gyro raw xyz data */
			comres += bno055_read_gyro_xyz(&gyro_xyz);
			if (comres == SUCCESS) {
				/* Convert gyro raw xyz to dps*/
				gyro_xyz_data->x =
				(float)gyro_xyz.x/GYRO_DIV_DPS;
				gyro_xyz_data->y =
				(float)gyro_xyz.y/GYRO_DIV_DPS;
				gyro_xyz_data->z =
				(float)gyro_xyz.z/GYRO_DIV_DPS;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_gyro_float *gyro_xyz_data)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_gyro gyro_xyz = {0, 0, 0};
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_RPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (comres == SUCCESS) {
			/* Read gyro raw xyz data */
			comres += bno055_read_gyro_xyz(&gyro_xyz);
			if (comres == SUCCESS) {
				/* Convert gyro raw xyz to rps*/
				gyro_xyz_data->x =
				(float)gyro_xyz.x/GYRO_DIV_RPS;
				gyro_xyz_data->y =
				(float)gyro_xyz.y/GYRO_DIV_RPS;
				gyro_xyz_data->z =
				(float)gyro_xyz.z/GYRO_DIV_RPS;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *euler_h)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_h = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_DEG)
		comres += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (comres ==  SUCCESS) {
			/* Read Euler raw h data*/
			comres += bno055_read_euler_h(&reg_euler_h);
			if (comres == SUCCESS) {
				/* Convert raw Euler h data to degree*/
				data = (float)reg_euler_h/EULER_DIV_DEG;
				*euler_h = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *euler_h)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_h = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_RAD)
		/* Read the current Euler unit and set the
		unit as radians if the unit is in degree */
		comres += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (comres == SUCCESS) {
			/* Read Euler raw h data*/
			comres += bno055_read_euler_h(&reg_euler_h);
			if (comres == SUCCESS) {
				/* Convert raw Euler h data to degree*/
				data = (float)reg_euler_h/EULER_DIV_RAD;
				*euler_h = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *euler_r)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_r = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_DEG)
		comres += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (comres ==  SUCCESS) {
			/* Read Euler raw r data*/
			comres += bno055_read_euler_r(&reg_euler_r);
			if (comres == SUCCESS) {
				/* Convert raw Euler r data to degree*/
				data = (float)reg_euler_r/EULER_DIV_DEG;
				*euler_r = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *euler_r)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_r = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_RAD)
		comres += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (comres == SUCCESS) {
			/* Read Euler raw r data*/
			comres += bno055_read_euler_r(&reg_euler_r);
			if (comres == SUCCESS) {
				/* Convert raw Euler r data to radians*/
				data = (float)reg_euler_r/EULER_DIV_RAD;
				*euler_r = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *euler_p)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_p = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_DEG)
		comres += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (comres ==  SUCCESS) {
			/* Read Euler raw p data*/
			comres += bno055_read_euler_p(&reg_euler_p);
			if (comres == SUCCESS) {
				/* Convert raw Euler p data to degree*/
				data = (float)reg_euler_p/EULER_DIV_DEG;
				*euler_p = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *euler_p)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_p = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_RAD)
		comres += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (comres == SUCCESS) {
			/* Read Euler raw r data*/
			comres += bno055_read_euler_p(&reg_euler_p);
			if (comres == SUCCESS) {
				/* Convert raw Euler r data to radians*/
				data = (float)reg_euler_p/EULER_DIV_RAD;
				*euler_p = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_euler_float *euler_hpr)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_euler reg_euler = {0, 0, 0};
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_DEG)
		comres += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (comres == SUCCESS) {
			/* Read Euler raw hrp data*/
			comres += bno055_read_euler_hrp(&reg_euler);
			if (comres == SUCCESS) {
				/* Convert raw Euler hrp to degree*/
				euler_hpr->h = (float)reg_euler.h/EULER_DIV_DEG;
				euler_hpr->p = (float)reg_euler.p/EULER_DIV_DEG;
				euler_hpr->r = (float)reg_euler.r/EULER_DIV_DEG;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_euler_float *euler_hpr)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_euler reg_euler = {0, 0, 0};
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_RAD)
		comres += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (comres == SUCCESS) {
			/* Read Euler raw hrp data*/
			comres += bno055_read_euler_hrp(&reg_euler);
			if (comres == SUCCESS) {
				/* Convert raw hrp to radians */
				euler_hpr->h = (float)reg_euler.h/EULER_DIV_RAD;
				euler_hpr->p = (float)reg_euler.p/EULER_DIV_RAD;
				euler_hpr->r = (float)reg_euler.r/EULER_DIV_RAD;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_convert_float_linear_accel_x_maq(
float *linear_accel_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_linear_accel_x = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	comres = bno055_read_linear_accel_x(&reg_linear_accel_x);
	if (comres == SUCCESS) {
		/* Convert the raw linear accel x to m/s2*/
		data = (float)reg_linear_accel_x/LINEAR_ACCEL_DIV_MSQ;
		*linear_accel_x = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
float *linear_accel_y)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_linear_accel_y = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	/* Read the raw y of linear accel */
	comres = bno055_read_linear_accel_y(&reg_linear_accel_y);
	if (comres == SUCCESS) {
		/* Convert the raw linear accel x to m/s2*/
		data = (float)reg_linear_accel_y/LINEAR_ACCEL_DIV_MSQ;
		*linear_accel_y = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
float *linear_accel_z)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_linear_accel_z = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	comres = bno055_read_linear_accel_z(&reg_linear_accel_z);
	if (comres == SUCCESS) {
		/* Convert the raw linear accel z to m/s2*/
		data = (float)reg_linear_accel_z/LINEAR_ACCEL_DIV_MSQ;
		*linear_accel_z = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
struct bno055_linear_accel_float *linear_accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_linear_accel reg_linear_accel = {0, 0, 0};
	/* Read the raw x of linear accel */
	comres = bno055_read_linear_accel_xyz(&reg_linear_accel);
	if (comres == SUCCESS) {
		linear_accel_xyz->x =
		(float)reg_linear_accel.x/LINEAR_ACCEL_DIV_MSQ;
		linear_accel_xyz->y =
		(float)reg_linear_accel.y/LINEAR_ACCEL_DIV_MSQ;
		linear_accel_xyz->z =
		(float)reg_linear_accel.z/LINEAR_ACCEL_DIV_MSQ;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
float *gravity_x)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_gravity_x = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	/* Read raw gravity of x*/
	comres = bno055_read_gravity_x(&reg_gravity_x);
	if (comres == SUCCESS) {
		/* Convert the raw gravity x to m/s2*/
		data = (float)reg_gravity_x/GRAVITY_DIV_MSQ;
		*gravity_x = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
float *gravity_y)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_gravity_y = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	/* Read raw gravity of y*/
	comres = bno055_read_gravity_y(&reg_gravity_y);
	if (comres == SUCCESS) {
		/* Convert the raw gravity y to m/s2*/
		data = (float)reg_gravity_y/GRAVITY_DIV_MSQ;
		*gravity_y = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
float *gravity_z)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_gravity_z = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	/* Read raw gravity of z */
	comres = bno055_read_gravity_z(&reg_gravity_z);
	if (comres == SUCCESS) {
		/* Convert the raw gravity z to m/s2*/
		data = (float)reg_gravity_z/GRAVITY_DIV_MSQ;
		*gravity_z = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
struct bno055_gravity_float *gravity_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_gravity reg_gravity_xyz = {0, 0, 0};
	/* Read raw gravity of xyz */
	comres = bno055_read_gravity_xyz(&reg_gravity_xyz);
	if (comres == SUCCESS) {
		/* Convert raw gravity xyz to meterpersecseq */
		gravity_xyz->x = (float)reg_gravity_xyz.x/GRAVITY_DIV_MSQ;
		gravity_xyz->y = (float)reg_gravity_xyz.y/GRAVITY_DIV_MSQ;
		gravity_xyz->z = (float)reg_gravity_xyz.z/GRAVITY_DIV_MSQ;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
float *temp)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_temp = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 temp_unit = BNO055_ZERO_U8X;
	/* Read the current temperature unit and set the
	unit as Fahrenheit if the unit is in Celsius */
	comres += bno055_get_temp_unit(&temp_unit);
	if (temp_unit != TEMP_UNIT_FAHRENHEIT)
		comres += bno055_set_temp_unit(TEMP_UNIT_FAHRENHEIT);
		if (comres == SUCCESS) {
			/* Read the raw temperature data */
			comres += bno055_read_temp_data(&reg_temp);
			if (comres == SUCCESS) {
				/* Convert raw temperature data to Fahrenheit*/
				data = (float)reg_temp/TEMP_DIV_FAHRENHEIT;
				*temp = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
float *temp)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_temp = BNO055_ZERO_U8X;
	float data = BNO055_ZERO_U8X;
	_u8 temp_unit = BNO055_ZERO_U8X;
	/* Read the current temperature unit and set the
	unit as Fahrenheit if the unit is in Celsius */
	comres += bno055_get_temp_unit(&temp_unit);
	if (temp_unit != TEMP_UNIT_CELSIUS)
		comres += bno055_set_temp_unit(TEMP_UNIT_CELSIUS);
		if (comres ==  SUCCESS) {
			/* Read the raw temperature data */
			comres += bno055_read_temp_data(&reg_temp);
			if (comres == SUCCESS) {
				/* Convert raw temperature data to Fahrenheit*/
				data = (float)reg_temp/TEMP_DIV_CELSIUS;
				*temp = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
#endif
#ifdef	BNO055_DOUBLE_ENABLE
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
double *accel_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_x = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MSQ)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (comres == SUCCESS) {
			/* Read the accel raw y data*/
			comres += bno055_read_accel_x(&reg_accel_x);
			if (comres == SUCCESS) {
				/* Convert the raw x to m/s2 */
				data = (double)reg_accel_x/ACCEL_DIV_MSQ;
				*accel_x = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *accel_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_x = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MG)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (comres == SUCCESS) {
			/* Read the accel raw y data*/
			comres += bno055_read_accel_x(&reg_accel_x);
			if (comres == SUCCESS) {
				/* Convert the raw x to mg */
				data = (double)reg_accel_x/ACCEL_DIV_MG;
				*accel_x = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *accel_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_y = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MSQ)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (comres == SUCCESS) {
			/* Read the accel raw y data*/
			comres += bno055_read_accel_y(&reg_accel_y);
			if (comres == SUCCESS) {
				/* Convert the raw x to m/s2 */
				data = (double)reg_accel_y/ACCEL_DIV_MSQ;
				*accel_y = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *accel_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_y = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MG)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (comres == SUCCESS) {
			/* Read the accel raw y data*/
			comres += bno055_read_accel_y(&reg_accel_y);
			if (comres == SUCCESS) {
				/* Convert the raw y to mg */
				data = (double)reg_accel_y/ACCEL_DIV_MG;
				*accel_y = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *accel_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_z = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MSQ)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (comres == SUCCESS) {
			/* Read the accel raw z data*/
			comres += bno055_read_accel_z(&reg_accel_z);
			if (comres == SUCCESS) {
				/* Convert the raw z to m/s2 */
				data = (double)reg_accel_z/ACCEL_DIV_MSQ;
				*accel_z = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *accel_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_accel_z = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as mg if the unit is in m/s2*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MG)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (comres == SUCCESS) {
			/* Read the accel raw z data*/
			comres += bno055_read_accel_z(&reg_accel_z);
			if (comres == SUCCESS) {
				/* Convert the raw z to mg */
				data = (double)reg_accel_z/ACCEL_DIV_MG;
				*accel_z = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_accel_double *accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_accel reg_accel_xyz = {0, 0, 0};
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MSQ)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MSQ);
		if (comres == SUCCESS) {
			/* Read the accel raw xyz data*/
			comres += bno055_read_accel_xyz(&reg_accel_xyz);
			if (comres == SUCCESS) {
				/* Convert raw xyz to m/s2*/
				accel_xyz->x =
				(double)reg_accel_xyz.x/ACCEL_DIV_MSQ;
				accel_xyz->y =
				(double)reg_accel_xyz.y/ACCEL_DIV_MSQ;
				accel_xyz->z =
				(double)reg_accel_xyz.z/ACCEL_DIV_MSQ;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_accel_double *accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_accel reg_accel_xyz = {0, 0, 0};
	_u8 accel_unit = BNO055_ZERO_U8X;
	/* Read the current accel unit and set the
	unit as m/s2 if the unit is in mg*/
	comres += bno055_get_accel_unit(&accel_unit);
	if (accel_unit != ACCEL_UNIT_MG)
		comres += bno055_set_accel_unit(ACCEL_UNIT_MG);
		if (comres == SUCCESS) {
			/* Read the accel raw xyz data*/
			comres += bno055_read_accel_xyz(&reg_accel_xyz);
			if (comres == SUCCESS) {
				/* Convert raw xyz to mg*/
				accel_xyz->x =
				(double)reg_accel_xyz.x/ACCEL_DIV_MG;
				accel_xyz->y =
				(double)reg_accel_xyz.y/ACCEL_DIV_MG;
				accel_xyz->z =
				(double)reg_accel_xyz.z/ACCEL_DIV_MG;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *mag_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_mag_x = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	/* Read raw mag x data */
	comres = bno055_read_mag_x(&reg_mag_x);
	if (comres == SUCCESS) {
		/* Convert raw mag x to uT */
		data = (double)reg_mag_x/MAG_DIV_MICROTESLA;
		*mag_x = data;
	} else {
	comres = ERROR;
	}
	return comres;
}
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
double *mag_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_mag_y = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	/* Read raw mag y data */
	comres = bno055_read_mag_y(&reg_mag_y);
	if (comres == SUCCESS) {
		/* Convert raw mag y to uT */
		data = (double)reg_mag_y/MAG_DIV_MICROTESLA;
		*mag_y = data;
	} else {
	comres = ERROR;
	}
	return comres;
}
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
double *mag_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_mag_z = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	/* Read raw mag x */
	comres = bno055_read_mag_z(&reg_mag_z);
	if (comres == SUCCESS) {
		/* Convert raw mag x to uT */
		data = (double)reg_mag_z/MAG_DIV_MICROTESLA;
		*mag_z = data;
	} else {
	comres = ERROR;
	}
	return comres;
}
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
struct bno055_mag_double *mag_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_mag reg_mag_xyz = {0, 0, 0};
	/* Read raw mag xyz data */
	comres = bno055_read_mag_xyz(&reg_mag_xyz);
	if (comres == SUCCESS) {
		/* Convert raw mag xyz to uT*/
		mag_xyz->x = (double)reg_mag_xyz.x/MAG_DIV_MICROTESLA;
		mag_xyz->y = (double)reg_mag_xyz.y/MAG_DIV_MICROTESLA;
		mag_xyz->z = (double)reg_mag_xyz.z/MAG_DIV_MICROTESLA;
	} else {
	return ERROR;
	}

	return comres;
}
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
double *gyro_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_x = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_DPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (comres == SUCCESS) {
			/* Read gyro raw x data */
			comres += bno055_read_gyro_x(&reg_gyro_x);
			if (comres == SUCCESS) {
				/* Convert raw gyro x to dps */
				data = (double)reg_gyro_x/GYRO_DIV_DPS;
				*gyro_x = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *gyro_x)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_x = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_RPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (comres == SUCCESS) {
			/* Read gyro raw x data */
			comres += bno055_read_gyro_x(&reg_gyro_x);
			if (comres == SUCCESS) {
				/* Convert raw gyro x to rps */
				data = (double)reg_gyro_x/GYRO_DIV_RPS;
				*gyro_x = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *gyro_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_y = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_DPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (comres == SUCCESS) {
			/* Read gyro raw y data */
			comres += bno055_read_gyro_y(&reg_gyro_y);
			if (comres == SUCCESS) {
				/* Convert raw gyro y to dps */
				data = (double)reg_gyro_y/GYRO_DIV_DPS;
				*gyro_y = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *gyro_y)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_y = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_RPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (comres == SUCCESS) {
			/* Read gyro raw y data */
			comres += bno055_read_gyro_y(&reg_gyro_y);
			if (comres == SUCCESS) {
				/* Convert raw gyro y to rps */
				data = (double)reg_gyro_y/GYRO_DIV_RPS;
				*gyro_y = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *gyro_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_z = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_DPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (comres == SUCCESS) {
			/* Read gyro raw z data */
			comres += bno055_read_gyro_z(&reg_gyro_z);
			if (comres == SUCCESS) {
				/* Convert raw gyro z to dps */
				data = (double)reg_gyro_z/GYRO_DIV_DPS;
				*gyro_z = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *gyro_z)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_s16 reg_gyro_z = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_RPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (comres == SUCCESS) {
			/* Read gyro raw x data */
			comres += bno055_read_gyro_z(&reg_gyro_z);
			if (comres == SUCCESS) {
				/* Convert raw gyro x to rps */
				data = (double)reg_gyro_z/GYRO_DIV_RPS;
				*gyro_z = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_gyro_double *gyro_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_gyro reg_gyro_xyz = {0, 0, 0};
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as dps if the unit is in rps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_DPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_DPS);
		if (comres == SUCCESS) {
			/* Read gyro raw xyz data */
			comres += bno055_read_gyro_xyz(&reg_gyro_xyz);
			if (comres == SUCCESS) {
				/* Convert gyro raw xyz to dps*/
				gyro_xyz->x =
				(double)reg_gyro_xyz.x/GYRO_DIV_DPS;
				gyro_xyz->y =
				(double)reg_gyro_xyz.y/GYRO_DIV_DPS;
				gyro_xyz->z =
				(double)reg_gyro_xyz.z/GYRO_DIV_DPS;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_gyro_double *gyro_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_gyro reg_gyro_xyz = {0, 0, 0};
	_u8 gyro_unit = BNO055_ZERO_U8X;
	/* Read the current gyro unit and set the
	unit as rps if the unit is in dps */
	comres += bno055_get_gyro_unit(&gyro_unit);
	if (gyro_unit != GYRO_UNIT_RPS)
		comres += bno055_set_gyro_unit(GYRO_UNIT_RPS);
		if (comres == SUCCESS) {
			/* Read gyro raw x data */
			comres += bno055_read_gyro_xyz(&reg_gyro_xyz);
			if (comres == SUCCESS) {
				/* Convert the raw gyro xyz to rps*/
				gyro_xyz->x =
				(double)reg_gyro_xyz.x/GYRO_DIV_RPS;
				gyro_xyz->y =
				(double)reg_gyro_xyz.y/GYRO_DIV_RPS;
				gyro_xyz->z =
				(double)reg_gyro_xyz.z/GYRO_DIV_RPS;
			} else {
			comres = ERROR;
			}
		} else {
			comres = ERROR;
		}
	return comres;
}
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
double *euler_h)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_h = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_DEG)
		comres += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (comres ==  SUCCESS) {
			/* Read Euler raw h data*/
			comres += bno055_read_euler_h(&reg_euler_h);
			if (comres == SUCCESS) {
				/* Convert raw Euler h to degree */
				data = (double)reg_euler_h/EULER_DIV_DEG;
				*euler_h = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *euler_h)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_h = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_RAD)
		comres += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (comres == SUCCESS) {
			/* Read Euler raw h data*/
			comres += bno055_read_euler_h(&reg_euler_h);
			if (comres == SUCCESS) {
				/* Convert raw Euler h to radians */
				data = (double)reg_euler_h/EULER_DIV_RAD;
				*euler_h = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *euler_r)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_r = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_DEG)
		comres += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (comres ==  SUCCESS) {
			/* Read Euler raw r data*/
			comres += bno055_read_euler_r(&reg_euler_r);
			if (comres == SUCCESS) {
				/* Convert raw Euler r to degree */
				data = (double)reg_euler_r/EULER_DIV_DEG;
				*euler_r = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *euler_r)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_r = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_RAD)
		comres += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (comres == SUCCESS) {
			/* Read Euler raw r data*/
			comres += bno055_read_euler_r(&reg_euler_r);
			if (comres == SUCCESS) {
				/* Convert raw Euler r to radians */
				data = (double)reg_euler_r/EULER_DIV_RAD;
				*euler_r = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *euler_p)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_p = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_DEG)
		comres += bno055_set_euler_unit(EULER_UNIT_DEG);
		if (comres ==  SUCCESS) {
			/* Read Euler raw p data*/
			comres += bno055_read_euler_p(&reg_euler_p);
			if (comres == SUCCESS) {
				/* Convert raw Euler p to degree*/
				data = (double)reg_euler_p/EULER_DIV_DEG;
				*euler_p = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
double *euler_p)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_euler_p = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	comres += bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_RAD)
		comres += bno055_set_euler_unit(EULER_UNIT_RAD);
		if (comres == SUCCESS) {
			/* Read Euler raw p data*/
			comres += bno055_read_euler_p(&reg_euler_p);
			if (comres == SUCCESS) {
				/* Convert raw p to radians*/
				data = (double)reg_euler_p/EULER_DIV_RAD;
				*euler_p = data;
			} else {
			comres = ERROR;
			}
		} else {
		comres = ERROR;
		}
	return comres;
}
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
struct bno055_euler_double *euler_hpr)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_euler reg_euler = {0, 0, 0};
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as degree if the unit is in radians */
	comres = bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_DEG)
		comres += bno055_set_euler_unit(EULER_UNIT_DEG);
	if (comres == SUCCESS) {
		/* Read Euler raw h data*/
			comres += bno055_read_euler_hrp(&reg_euler);
		if (comres == SUCCESS) {
			/* Convert raw Euler hrp to degree*/
			euler_hpr->h = (double)reg_euler.h/EULER_DIV_DEG;
			euler_hpr->p = (double)reg_euler.p/EULER_DIV_DEG;
			euler_hpr->r = (double)reg_euler.r/EULER_DIV_DEG;
		} else {
		comres = ERROR;
		}
	} else {
	comres = ERROR;
	}
	return comres;
}
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
struct bno055_euler_double *euler_hpr)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_euler reg_euler = {0, 0, 0};
	_u8 euler_unit = BNO055_ZERO_U8X;
	/* Read the current Euler unit and set the
	unit as radians if the unit is in degree */
	comres = bno055_get_euler_unit(&euler_unit);
	if (euler_unit != EULER_UNIT_RAD)
		comres = bno055_set_euler_unit(EULER_UNIT_RAD);
		if (comres == SUCCESS) {
			/* Read the raw hrp */
			comres = bno055_read_euler_hrp(&reg_euler);
			if (comres == SUCCESS) {
				/* Convert raw Euler hrp to radians*/
				euler_hpr->h =
				(double)reg_euler.h/EULER_DIV_RAD;
				euler_hpr->p =
				(double)reg_euler.p/EULER_DIV_RAD;
				euler_hpr->r =
				(double)reg_euler.r/EULER_DIV_RAD;
			} else {
			comres = ERROR;
			}
		} else {
			comres = ERROR;
		}
	return comres;
}
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
double *linear_accel_x)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_linear_accel_x = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	comres = bno055_read_linear_accel_x(&reg_linear_accel_x);
	if (comres == SUCCESS) {
		/* Convert the raw x to m/s2 */
		data = (double)reg_linear_accel_x/LINEAR_ACCEL_DIV_MSQ;
		*linear_accel_x = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
double *linear_accel_y)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_linear_accel_y = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	comres = bno055_read_linear_accel_y(&reg_linear_accel_y);
	if (comres == SUCCESS) {
		/* Convert the raw y to m/s2 */
		data = (double)reg_linear_accel_y/LINEAR_ACCEL_DIV_MSQ;
		*linear_accel_y = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
double *linear_accel_z)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_linear_accel_z = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	/* Read the raw x of linear accel */
	comres = bno055_read_linear_accel_z(&reg_linear_accel_z);
	if (comres == SUCCESS) {
		/* Convert the raw z to m/s2 */
		data = (double)reg_linear_accel_z/LINEAR_ACCEL_DIV_MSQ;
		*linear_accel_z = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
struct bno055_linear_accel_double *linear_accel_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_linear_accel reg_linear_accel_xyz = {0, 0, 0};
	/* Read the raw xyz of linear accel */
	comres = bno055_read_linear_accel_xyz(&reg_linear_accel_xyz);
	if (comres == SUCCESS) {
		/* Convert the raw xyz of linear accel to m/s2 */
		linear_accel_xyz->x =
		(double)reg_linear_accel_xyz.x/LINEAR_ACCEL_DIV_MSQ;
		linear_accel_xyz->y =
		(double)reg_linear_accel_xyz.y/LINEAR_ACCEL_DIV_MSQ;
		linear_accel_xyz->z =
		(double)reg_linear_accel_xyz.z/LINEAR_ACCEL_DIV_MSQ;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
double *gravity_x)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_gravity_x = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	/* Read raw gravity of x*/
	comres = bno055_read_gravity_x(&reg_gravity_x);
	if (comres == SUCCESS) {
		/* Convert raw gravity of x to m/s2 */
		data = (double)reg_gravity_x/GRAVITY_DIV_MSQ;
		*gravity_x = data;
	} else {
		comres = ERROR;
	}
	return comres;
}
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
double *gravity_y)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_gravity_y = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	/* Read raw gravity of y */
	comres += bno055_read_gravity_y(&reg_gravity_y);
	if (comres == SUCCESS) {
		/* convert raw gravity of y to m/s2 */
		data = (double)reg_gravity_y/GRAVITY_DIV_MSQ;
		*gravity_y = data;
	} else {
		comres += ERROR;
	}
	return comres;
}
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
double *gravity_z)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_gravity_z = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	/* Read raw gravity of z */
	comres += bno055_read_gravity_z(&reg_gravity_z);
	if (comres == SUCCESS) {
		/* Convert raw gravity of z to m/s2 */
		data = (double)reg_gravity_z/GRAVITY_DIV_MSQ;
		*gravity_z = data;
	} else {
		comres += ERROR;
	}
	return comres;
}
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
struct bno055_gravity_double *gravity_xyz)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	struct bno055_gravity reg_gravity_xyz = {0, 0, 0};
	/* Read raw gravity of xyz */
	comres += bno055_read_gravity_xyz(&reg_gravity_xyz);
	if (comres == SUCCESS) {
		/* Convert raw gravity of xyz to m/s2 */
		gravity_xyz->x =
		(double)reg_gravity_xyz.x/GRAVITY_DIV_MSQ;
		gravity_xyz->y =
		(double)reg_gravity_xyz.y/GRAVITY_DIV_MSQ;
		gravity_xyz->z =
		(double)reg_gravity_xyz.z/GRAVITY_DIV_MSQ;
	} else {
		comres += ERROR;
	}
	return comres;
}
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
double *temp)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_temp = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 temp_unit = BNO055_ZERO_U8X;
	/* Read the current temperature unit and set the
	unit as Fahrenheit if the unit is in Celsius */
	comres += bno055_get_temp_unit(&temp_unit);
	if (temp_unit != TEMP_UNIT_FAHRENHEIT)
		comres += bno055_set_temp_unit(TEMP_UNIT_FAHRENHEIT);
		if (comres == SUCCESS) {
			/* Read the raw temperature data */
			comres += bno055_read_temp_data(&reg_temp);
			if (comres == SUCCESS) {
				/* Convert raw temperature data to Fahrenheit*/
				data = (double)reg_temp/TEMP_DIV_FAHRENHEIT;
				*temp = data;
			} else {
			comres += ERROR;
			}
		} else {
		comres += ERROR;
		}
	return comres;
}
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
double *temp)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_s16 reg_temp = BNO055_ZERO_U8X;
	double data = BNO055_ZERO_U8X;
	_u8 temp_unit = BNO055_ZERO_U8X;
	/* Read the current temperature unit and set the
	unit as Fahrenheit if the unit is in Celsius */
	comres += bno055_get_temp_unit(&temp_unit);
	if (temp_unit != TEMP_UNIT_CELSIUS)
		comres += bno055_set_temp_unit(TEMP_UNIT_CELSIUS);
		if (comres == SUCCESS) {
			/* Read the raw temperature data */
			comres += bno055_read_temp_data(&reg_temp);
			if (comres == SUCCESS) {
				/* Convert raw temperature data to Fahrenheit*/
				data = (double)reg_temp/TEMP_DIV_CELSIUS;
				*temp = data;
			} else {
			comres += ERROR;
			}
		} else {
		comres += ERROR;
		}
	return comres;
}
#endif
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
_u8 *mag_calib)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag calib
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the mag calib status */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_CALIB_STAT__REG, &v_data_u8r, 1);
			*mag_calib =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_MAG_CALIB_STAT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *accel_calib)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty*/
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel calib
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the accel calib status */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_CALIB_STAT__REG,
			&v_data_u8r, 1);
			*accel_calib =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_CALIB_STAT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *gyro_calib)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro calib
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the gyro calib status */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_CALIB_STAT__REG, &v_data_u8r, 1);
			*gyro_calib =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYRO_CALIB_STAT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *sys_calib)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty*/
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page,system calib
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the system calib */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_CALIB_STAT__REG, &v_data_u8r, 1);
			*sys_calib =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SYS_CALIB_STAT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *selftest_accel)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel self test is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the accel self test */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST_ACCEL__REG, &v_data_u8r, 1);
			*selftest_accel =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SELFTEST_ACCEL);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *selftest_mag)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, self test of mag is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the mag self test */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST_MAG__REG, &v_data_u8r, 1);
			*selftest_mag =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SELFTEST_MAG);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *selftest_gyro)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page self test of gyro is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the gyro self test */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST_GYRO__REG, &v_data_u8r, 1);
			*selftest_gyro =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SELFTEST_GYRO);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *selftest_mcu)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page self test of micro controller
		is available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the self test of micro controller*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST_MCU__REG, &v_data_u8r, 1);
			*selftest_mcu =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SELFTEST_MCU);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
 *	bno055_set_int_mask_gyro_anymotion()
 *
 *	bno055_set_int_gyro_anymotion()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_stat_gyro_anymotion(
_u8 *gyro_anymotion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion interrupt
		status is available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the gyro anymotion interrupt status*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_GYRO_ANYMOTION__REG, &v_data_u8r, 1);
			*gyro_anymotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INT_STAT_GYRO_ANYMOTION);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to read the status of
 *	gyro highrate interrupt from register from 0x37 bit 3
 *
 *	@param gyro_highrate : The value of gyro highrate interrupt
 *
 *     gyro_highrate   |  result
 *    --------------   | ---------------------
 *     0x00            | indicates no interrupt triggered
 *     0x01            | indicates interrupt triggered
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
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_stat_gyro_highrate(
_u8 *gyro_highrate)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the gyro highrate interrupt status*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_GYRO_HIGHRATE__REG, &v_data_u8r, 1);
			*gyro_highrate =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INT_STAT_GYRO_HIGHRATE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to read the status of
 *	accel highg interrupt from register from 0x37 bit 5
 *
 *	@param accel_highg : The value of accel highg interrupt
 *
 *     accel_high_g |  result
 *    ------------- | -------------------------------
 *     0x00         | indicates no interrupt triggered
 *     0x01         | indicates interrupt triggered
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_stat_accel_high_g(
_u8 *accel_high_g)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the accel highg interrupt status */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_ACCEL_HIGH_G__REG, &v_data_u8r, 1);
			*accel_high_g =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INT_STAT_ACCEL_HIGH_G);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to read the status of
 *	accel anymotion interrupt from register from 0x37 bit 6
 *
 *	@param accel_anymotion : The value of accel anymotion interrupt
 *
 *     accel_anymotion |  result
 *    --------------   | -------------------------------
 *     0x00            | indicates no interrupt triggered
 *     0x01            | indicates interrupt triggered
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
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_stat_accel_anymotion(
_u8 *accel_anymotion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the accel anymotion interrupt status */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_ACCEL_ANYMOTION__REG,
			&v_data_u8r, 1);
			*accel_anymotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INT_STAT_ACCEL_ANYMOTION);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to read the status of
 *	accel nomotion/slowmotion interrupt from register from 0x37 bit 6
 *
 *	@param accel_nomotion : The value of accel nomotion/slowmotion interrupt
 *
 *     accel_nomotion |  result
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
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_stat_accel_nomotion(
_u8 *accel_nomotion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel
		nomotion/slowmotion interrupt
		is available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the status of accel
			nomotion/slowmotion interrupt*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_STAT_ACCEL_NOMOTION__REG, &v_data_u8r, 1);
			*accel_nomotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_INT_STAT_ACCEL_NOMOTION);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *stat_main_clk)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, status of main clk is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the status of main clk */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_MAIN_CLK__REG, &v_data_u8r, 1);
			*stat_main_clk =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_SYS_MAIN_CLK);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *sys_stat)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, the status of system is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the the status of system*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_STAT_CODE__REG, &v_data_u8r, 1);
			*sys_stat =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SYS_STAT_CODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *sys_error)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, system error code is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the system error code*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_ERROR_CODE__REG, &v_data_u8r, 1);
			*sys_error =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SYS_ERROR_CODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *accel_unit)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel unit is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the accel unit */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_UNIT__REG, &v_data_u8r, 1);
			*accel_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_ACCEL_UNIT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to write the accel unit
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
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_unit(
_u8 accel_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the accel unit */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_UNIT__REG, &v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_UNIT, accel_unit);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_UNIT__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_unit(
_u8 *gyro_unit)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro unit is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the gyro unit */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_UNIT__REG, &v_data_u8r, 1);
			*gyro_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_GYRO_UNIT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to write the gyro unit
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_unit(_u8 gyro_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the gyro unit */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_UNIT__REG, &v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_UNIT, gyro_unit);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_UNIT__REG, &v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_euler_unit(
_u8 *euler_unit)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, Euler unit is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the Euler unit */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_UNIT__REG, &v_data_u8r, 1);
			*euler_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_EULER_UNIT);
		} else {
		return ERROR;
		}
	}
	return comres;
}

/*!
 *	@brief This API used to write the Euler unit
 *	from register from 0x3B bit 2
 *
 *	@param euler_unit : The value of Euler unit
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_euler_unit(_u8 euler_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the Euler unit*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_EULER_UNIT__REG, &v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_EULER_UNIT, euler_unit);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_EULER_UNIT__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *tilt_unit)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TILT_UNIT__REG, &v_data_u8r, 1);
			*tilt_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_TILT_UNIT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_tilt_unit(_u8 tilt_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_TILT_UNIT__REG, &v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_TILT_UNIT, tilt_unit);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_TILT_UNIT__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *temp_unit)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, temperature unit is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the temperature unit */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP_UNIT__REG, &v_data_u8r, 1);
			*temp_unit =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_TEMP_UNIT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 temp_unit)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the temperature unit */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_TEMP_UNIT__REG, &v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_TEMP_UNIT,
					temp_unit);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_TEMP_UNIT__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *data_output_format)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, data output format is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the data output format */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_DATA_OUTPUT_FORMAT__REG, &v_data_u8r, 1);
			*data_output_format =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_DATA_OUTPUT_FORMAT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 data_output_format)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the data output format */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_DATA_OUTPUT_FORMAT__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_DATA_OUTPUT_FORMAT,
					data_output_format);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_DATA_OUTPUT_FORMAT__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *accel_data_select)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the accel data select*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_SELECT__REG,
			&v_data_u8r, 1);
			*accel_data_select =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_DATA_SELECT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_data_select)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the accel data select */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_DATA_SELECT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_DATA_SELECT, accel_data_select);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_DATA_SELECT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *mag_data_select)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the mag mag data selects*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_SELECT__REG, &v_data_u8r, 1);
			*mag_data_select =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_MAG_DATA_SELECT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 mag_data_select)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the mag data select*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_SELECT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_MAG_DATA_SELECT,
				mag_data_select);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_MAG_DATA_SELECT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *gyro_data_select)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the gyro data select */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_SELECT__REG,
			&v_data_u8r, 1);
			*gyro_data_select =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_DATA_SELECT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_data_select)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the gyro data select */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_DATA_SELECT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GYRO_DATA_SELECT,
				gyro_data_select);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_DATA_SELECT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *euler_data_select)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, Euler data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the Euler data select*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_DATA_SELECT__REG,
			&v_data_u8r, 1);
			*euler_data_select =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_EULER_DATA_SELECT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 euler_data_select)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, Euler data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the Euler data select */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_EULER_DATA_SELECT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_EULER_DATA_SELECT,
				euler_data_select);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_EULER_DATA_SELECT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *quaternion_data_select)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, quaternion data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the quaternion data select */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_SELECT__REG, &v_data_u8r, 1);
			*quaternion_data_select =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_QUATERNION_DATA_SELECT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 quaternion_data_select)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, quaternion data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the quaternion data select*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_QUATERNION_DATA_SELECT__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_QUATERNION_DATA_SELECT,
				quaternion_data_select);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_QUATERNION_DATA_SELECT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *linear_accel_data_select)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, linear accel data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the linear accel data select*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_SELECT__REG,
			&v_data_u8r, 1);
			*linear_accel_data_select =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_LINEAR_ACCEL_DATA_SELECT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 linear_accel_data_select)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, linear accel data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the linear accel data select*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_LINEAR_ACCEL_DATA_SELECT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_LINEAR_ACCEL_DATA_SELECT,
				linear_accel_data_select);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_LINEAR_ACCEL_DATA_SELECT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *gravity_data_select)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gravity data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the gravity data select */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_SELECT__REG,
			&v_data_u8r, 1);
			*gravity_data_select =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GRAVITY_DATA_SELECT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gravity_data_select)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gravity data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the gravity data select*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GRAVITY_DATA_SELECT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GRAVITY_DATA_SELECT,
				gravity_data_select);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GRAVITY_DATA_SELECT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *temp_data_select)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, temperature data select is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the temperature data select */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP_DATA_SELECT__REG,
			&v_data_u8r, 1);
			*temp_data_select =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_TEMP_DATA_SELECT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 temp_data_select)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, chip id is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the temperature data select */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP_DATA_SELECT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_TEMP_DATA_SELECT,
				temp_data_select);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_TEMP_DATA_SELECT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *output_data_rate)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, output data rate is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of output data rate*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_OUTPUT_DATA_RATE__REG, &v_data_u8r, 1);
			*output_data_rate =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_OUTPUT_DATA_RATE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
 *      0x04          | FASTEST_MODE_2
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
_u8 output_data_rate)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the value of output data rate*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_OUTPUT_DATA_RATE__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_OUTPUT_DATA_RATE,
					output_data_rate);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_OUTPUT_DATA_RATE__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *operation_mode)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, operation mode is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of operation mode*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_OPERATION_MODE__REG, &v_data_u8r, 1);
			*operation_mode =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_OPERATION_MODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_operation_mode(_u8 operation_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			/* If the previous operation mode is config it is
				directly write the operation mode */
			if (prev_opmode == OPERATION_MODE_CONFIG) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_OPERATION_MODE__REG, &v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_OPERATION_MODE, operation_mode);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_OPERATION_MODE__REG,
					&v_data_u8r, 1);
					/* Config mode to other
					operation mode switching
					required delay of 600ms*/
					p_bno055->delay_msec(600);
				}
			} else {
				/* If the previous operation
				mode is not config it is
				 write the config mode */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_OPERATION_MODE__REG, &v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_OPERATION_MODE,
					OPERATION_MODE_CONFIG);
					comres += bno055_write_register(
					BNO055_OPERATION_MODE__REG,
					&v_data_u8r, 1);
					/* other mode to config mode switching
					required delay of 20ms*/
					p_bno055->delay_msec(20);
				}
				/* Write the operation mode */
				if (operation_mode !=
				OPERATION_MODE_CONFIG) {
					comres += p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_OPERATION_MODE__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_OPERATION_MODE,
						operation_mode);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_OPERATION_MODE__REG,
						&v_data_u8r, 1);
						/* Config mode to other
						operation mode switching
						required delay of 600ms*/
						p_bno055->delay_msec(600);
					}
				}
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *power_mode)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, power mode is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of power mode */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_POWER_MODE__REG, &v_data_u8r, 1);
			*power_mode =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_POWER_MODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_power_mode(_u8 power_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the value of power mode */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_POWER_MODE__REG, &v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_POWER_MODE, power_mode);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_POWER_MODE__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *int_rst)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page,  reset interrupt is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of reset interrupt*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_RST__REG, &v_data_u8r, 1);
			*int_rst =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_INT_RST);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_rst(_u8 int_rst)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, reset interrupt
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the value of reset interrupt */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_INT_RST__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_INT_RST, int_rst);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_INT_RST__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *clk_src)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, clk source is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of clk source */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CLK_SRC__REG, &v_data_u8r, 1);
			*clk_src =
			BNO055_GET_BITSLICE(v_data_u8r, BNO055_CLK_SRC);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to write the clk source
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_clk_src(_u8 clk_src)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, clk source is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the value of clk source */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_CLK_SRC__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_CLK_SRC, clk_src);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_CLK_SRC__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *sys_rst)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, reset system is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of reset system */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_RST__REG, &v_data_u8r, 1);
			*sys_rst =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SYS_RST);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_sys_rst(_u8 sys_rst)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, reset system is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Write the value of reset system */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SYS_RST__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_SYS_RST, sys_rst);
				comres =
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_SYS_RST__REG, &v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *selftest)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, self test is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of self test */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SELFTEST__REG, &v_data_u8r, 1);
			*selftest =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_SELFTEST);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_selftest(_u8 selftest)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the value of self test */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_SELFTEST__REG, &v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_SELFTEST,
					selftest);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_SELFTEST__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *temp_source)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, temperature source is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of temperature source */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_TEMP_SOURCE__REG, &v_data_u8r, 1);
			*temp_source =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_TEMP_SOURCE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_temp_source(_u8 temp_source)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the value of temperature source*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_TEMP_SOURCE__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_TEMP_SOURCE, temp_source);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_TEMP_SOURCE__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *remap_axis)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, axis remap is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of axis remap*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_AXIS_VALUE__REG, &v_data_u8r, 1);
			*remap_axis =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_REMAP_AXIS_VALUE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 remap_axis)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			/* Write the value of axis remap */
		if (status == SUCCESS) {
			switch (remap_axis) {
			case REMAP_X_Y:
			case REMAP_Y_Z:
			case REMAP_Z_X:
			case REMAP_X_Y_Z_TYPE0:
			case REMAP_X_Y_Z_TYPE1:
			case DEFAULT_AXIS:
				comres =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_AXIS_VALUE__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_REMAP_AXIS_VALUE,
					remap_axis);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_AXIS_VALUE__REG,
					&v_data_u8r, 1);
				}
			break;
			default:
				/* Write the default axis remap value */
				comres =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_AXIS_VALUE__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_REMAP_AXIS_VALUE,
					DEFAULT_AXIS);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_AXIS_VALUE__REG,
					&v_data_u8r, 1);
				}
			break;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode
	of previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
/*!
 *	@brief This API used to read the x-axis remap
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
_u8 *remap_x_sign)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, x-axis remap sign is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of x-axis remap sign */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_X_SIGN__REG, &v_data_u8r, 1);
			*remap_x_sign =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_REMAP_X_SIGN);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 remap_x_sign)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the value of x-axis remap */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_X_SIGN__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_REMAP_X_SIGN,
					remap_x_sign);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_X_SIGN__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *remap_y_sign)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, y-axis remap sign is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of y-axis remap sign*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_Y_SIGN__REG, &v_data_u8r, 1);
			*remap_y_sign =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_REMAP_Y_SIGN);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 remap_y_sign)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the value of y-axis remap sign*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_Y_SIGN__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_REMAP_Y_SIGN,
					remap_y_sign);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_Y_SIGN__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *remap_z_sign)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, z-axis remap sign is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read the value of z-axis remap sign*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_REMAP_Z_SIGN__REG, &v_data_u8r, 1);
			*remap_z_sign =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_REMAP_Z_SIGN);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 remap_z_sign)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
		/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status = bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status = bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write the value of z-axis remap sign*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_REMAP_Z_SIGN__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_REMAP_Z_SIGN,
					remap_z_sign);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_REMAP_Z_SIGN__REG,
					&v_data_u8r, 1);
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
struct bno055_sic_matrix  *sic_matrix)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the soft iron calibration matrix values
	a_data_u8r[0] - sic_0->LSB
	a_data_u8r[1] - sic_0->MSB
	a_data_u8r[2] - sic_1->LSB
	a_data_u8r[3] - sic_1->MSB
	a_data_u8r[4] - sic_2->LSB
	a_data_u8r[5] - sic_2->MSB
	a_data_u8r[6] - sic_3->LSB
	a_data_u8r[7] - sic_3->MSB
	a_data_u8r[8] - sic_4->LSB
	a_data_u8r[9] - sic_4->MSB
	a_data_u8r[10] - sic_5->LSB
	a_data_u8r[11] - sic_5->MSB
	a_data_u8r[12] - sic_6->LSB
	a_data_u8r[13] - sic_6->MSB
	a_data_u8r[14] - sic_7->LSB
	a_data_u8r[15] - sic_7->MSB
	a_data_u8r[16] - sic_8->LSB
	a_data_u8r[17] - sic_8->MSB
	*/
	_u8 a_data_u8r[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, soft iron calibration matrix is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read soft iron calibration matrix value
			it is eighteen bytes of data */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_SIC_MATRIX_0_LSB__REG, a_data_u8r, 18);
			if (comres == SUCCESS) {
				/*soft iron calibration matrix zero*/
				a_data_u8r[0] =
				BNO055_GET_BITSLICE(a_data_u8r[0],
				BNO055_SIC_MATRIX_0_LSB);
				a_data_u8r[1] =
				BNO055_GET_BITSLICE(a_data_u8r[1],
				BNO055_SIC_MATRIX_0_MSB);
				sic_matrix->sic_0 = (_s16)((((_s32)
				(_s8)(a_data_u8r[1])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));

				/*soft iron calibration matrix one*/
				a_data_u8r[2] =
				BNO055_GET_BITSLICE(a_data_u8r[2],
				BNO055_SIC_MATRIX_1_LSB);
				a_data_u8r[3] =
				BNO055_GET_BITSLICE(a_data_u8r[3],
				BNO055_SIC_MATRIX_1_MSB);
				sic_matrix->sic_1 = (_s16)((((_s32)
				(_s8)(a_data_u8r[3])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[2]));

				/*soft iron calibration matrix two*/
				a_data_u8r[4] =
				BNO055_GET_BITSLICE(a_data_u8r[4],
				BNO055_SIC_MATRIX_2_LSB);
				a_data_u8r[5] =
				BNO055_GET_BITSLICE(a_data_u8r[5],
				BNO055_SIC_MATRIX_2_MSB);
				sic_matrix->sic_2 = (_s16)((((_s32)
				(_s8)(a_data_u8r[5])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[4]));

				/*soft iron calibration matrix three*/
				a_data_u8r[6] =
				BNO055_GET_BITSLICE(a_data_u8r[6],
				BNO055_SIC_MATRIX_3_LSB);
				a_data_u8r[7] =
				BNO055_GET_BITSLICE(a_data_u8r[7],
				BNO055_SIC_MATRIX_3_LSB);
				sic_matrix->sic_3  = (_s16)((((_s32)
				(_s8)(a_data_u8r[7])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[6]));

				/*soft iron calibration matrix four*/
				a_data_u8r[8] =
				BNO055_GET_BITSLICE(a_data_u8r[8],
				BNO055_SIC_MATRIX_4_LSB);
				a_data_u8r[9] =
				BNO055_GET_BITSLICE(a_data_u8r[9],
				BNO055_SIC_MATRIX_4_LSB);
				sic_matrix->sic_4  = (_s16)((((_s32)
				(_s8)(a_data_u8r[9])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[8]));

				/*soft iron calibration matrix five*/
				a_data_u8r[10] =
				BNO055_GET_BITSLICE(a_data_u8r[10],
				BNO055_SIC_MATRIX_5_LSB);
				a_data_u8r[11] =
				BNO055_GET_BITSLICE(a_data_u8r[11],
				BNO055_SIC_MATRIX_5_LSB);
				sic_matrix->sic_5 = (_s16)((((_s32)
				(_s8)(a_data_u8r[11])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[10]));

				/*soft iron calibration matrix six*/
				a_data_u8r[12] =
				BNO055_GET_BITSLICE(a_data_u8r[12],
				BNO055_SIC_MATRIX_6_LSB);
				a_data_u8r[13] =
				BNO055_GET_BITSLICE(a_data_u8r[13],
				BNO055_SIC_MATRIX_6_LSB);
				sic_matrix->sic_6  = (_s16)((((_s32)
				(_s8)(a_data_u8r[13])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[12]));

				/*soft iron calibration matrix seven*/
				a_data_u8r[14] =
				BNO055_GET_BITSLICE(a_data_u8r[14],
				BNO055_SIC_MATRIX_7_LSB);
				a_data_u8r[15] =
				BNO055_GET_BITSLICE(a_data_u8r[15],
				BNO055_SIC_MATRIX_7_LSB);
				sic_matrix->sic_7  = (_s16)((((_s32)
				(_s8)(a_data_u8r[15])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[14]));

				/*soft iron calibration matrix eight*/
				a_data_u8r[16] =
				BNO055_GET_BITSLICE(a_data_u8r[16],
				BNO055_SIC_MATRIX_8_LSB);
				a_data_u8r[17] =
				BNO055_GET_BITSLICE(a_data_u8r[17],
				BNO055_SIC_MATRIX_8_LSB);
				sic_matrix->sic_8  = (_s16)((((_s32)
				(_s8)(a_data_u8r[17])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[16]));
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
struct bno055_sic_matrix  *sic_matrix)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data1_u8r = BNO055_ZERO_U8X;
_u8 v_data2_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status += bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					/* write soft iron calibration
					matrix zero value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_0_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_0 & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_0_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_0_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_0_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_0  >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_0_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_0_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write soft iron calibration
					matrix one value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_1_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_1 & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_1_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_1_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_1_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_1  >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_1_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_1_MSB__REG,
						&v_data2_u8r, 1);
					}

				/* write soft iron calibration
				matrix two value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_2_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_2 & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_2_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_2_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_2_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_2 >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_2_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_2_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write soft iron calibration
					matrix three value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_3_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_3 & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_3_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_3_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_3_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_3 >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_3_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_3_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write soft iron calibration
					matrix four value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_4_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_4 & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_4_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_4_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_4_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_4 >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_4_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_4_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write soft iron calibration
					matrix five value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_5_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_5 & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_5_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_5_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_5_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_5 >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_5_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_5_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write soft iron calibration
					matrix six value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_6_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_6 & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_6_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_6_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_6_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_6 >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_6_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_6_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write soft iron calibration
					matrix seven value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_7_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_7 & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_7_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_7_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_7_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_7 >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_7_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_7_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write soft iron calibration
					matrix eight value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_8_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_8 & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_8_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_8_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_SIC_MATRIX_8_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(sic_matrix->sic_8 >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_SIC_MATRIX_8_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_SIC_MATRIX_8_MSB__REG,
						&v_data2_u8r, 1);
					}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
struct bno055_accel_offset  *accel_offset)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the accel offset values
	a_data_u8r[0] - offset x->LSB
	a_data_u8r[1] - offset x->MSB
	a_data_u8r[2] - offset y->LSB
	a_data_u8r[3] - offset y->MSB
	a_data_u8r[4] - offset z->LSB
	a_data_u8r[5] - offset z->MSB
	*/
	_u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel offset is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read accel offset value it is six bytes of data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_OFFSET_X_LSB__REG, a_data_u8r, 6);
			if (comres == SUCCESS) {
				/* Read accel x offset value*/
				a_data_u8r[0] =
				BNO055_GET_BITSLICE(a_data_u8r[0],
				BNO055_ACCEL_OFFSET_X_LSB);
				a_data_u8r[1] =
				BNO055_GET_BITSLICE(a_data_u8r[1],
				BNO055_ACCEL_OFFSET_X_MSB);
				accel_offset->x = (_s16)((((_s32)
				(_s8)(a_data_u8r[1])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));

				/* Read accel y offset value*/
				a_data_u8r[2] =
				BNO055_GET_BITSLICE(a_data_u8r[2],
				BNO055_ACCEL_OFFSET_Y_LSB);
				a_data_u8r[3] =
				BNO055_GET_BITSLICE(a_data_u8r[3],
				BNO055_ACCEL_OFFSET_Y_MSB);
				accel_offset->y = (_s16)((((_s32)
				(_s8)(a_data_u8r[3])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[2]));

				/* Read accel z offset value*/
				a_data_u8r[4] =
				BNO055_GET_BITSLICE(a_data_u8r[4],
				BNO055_ACCEL_OFFSET_Z_LSB);
				a_data_u8r[5] =
				BNO055_GET_BITSLICE(a_data_u8r[5],
				BNO055_ACCEL_OFFSET_Z_MSB);
				accel_offset->z = (_s16)((((_s32)
				(_s8)(a_data_u8r[5])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[4]));

				/* Read accel radius value
				it is two bytes of data*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_RADIUS_LSB__REG, a_data_u8r, 2);
				/* Array holding the accel radius values
				a_data_u8r[0] - radius->LSB
				a_data_u8r[1] - radius->MSB
				*/
				if (comres == SUCCESS) {
					a_data_u8r[0] =
					BNO055_GET_BITSLICE(a_data_u8r[0],
					BNO055_ACCEL_RADIUS_LSB);
					a_data_u8r[1] =
					BNO055_GET_BITSLICE(a_data_u8r[1],
					BNO055_ACCEL_RADIUS_MSB);
					accel_offset->r = (_s16)((((_s32)
					(_s8)(a_data_u8r[1])) <<
					(BNO055_SHIFT_8_POSITION)) |
					(a_data_u8r[0]));
				} else {
				return ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
struct bno055_accel_offset  *accel_offset)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data1_u8r = BNO055_ZERO_U8X;
_u8 v_data2_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status += bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					/* write accel offset x value*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_X_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(accel_offset->x & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_X_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_X_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_X_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(accel_offset->x  >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_X_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_X_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write accel offset y value*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_Y_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(accel_offset->y & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_Y_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_Y_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_Y_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(accel_offset->y >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_Y_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_Y_MSB__REG,
						&v_data2_u8r, 1);
					}
				/* write accel offset z value*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_Z_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(accel_offset->z & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_Z_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_Z_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_OFFSET_Z_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(accel_offset->z >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_OFFSET_Z_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_OFFSET_Z_MSB__REG,
						&v_data2_u8r, 1);
					}

				/*write accel radius value*/
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_RADIUS_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(accel_offset->r & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_RADIUS_LSB,
						v_data1_u8r);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_RADIUS_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres += p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_RADIUS_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(accel_offset->r >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_ACCEL_RADIUS_MSB,
						v_data1_u8r);
						comres =
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_RADIUS_MSB__REG,
						&v_data2_u8r, 1);
					}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}

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
struct bno055_mag_offset  *mag_offset)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the mag offset values
	a_data_u8r[0] - offset x->LSB
	a_data_u8r[1] - offset x->MSB
	a_data_u8r[2] - offset y->LSB
	a_data_u8r[3] - offset y->MSB
	a_data_u8r[4] - offset z->LSB
	a_data_u8r[5] - offset z->MSB
	*/
	_u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag offset is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read mag offset value it the six bytes of data */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_OFFSET_X_LSB__REG, a_data_u8r, 6);
			if (comres == SUCCESS) {
				/* Read mag x offset value*/
				a_data_u8r[0] =
				BNO055_GET_BITSLICE(a_data_u8r[0],
				BNO055_MAG_OFFSET_X_LSB);
				a_data_u8r[1] =
				BNO055_GET_BITSLICE(a_data_u8r[1],
				BNO055_MAG_OFFSET_X_MSB);
				mag_offset->x = (_s16)((((_s32)
				(_s8)(a_data_u8r[1])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));

				/* Read mag y offset value*/
				a_data_u8r[2] =
				BNO055_GET_BITSLICE(a_data_u8r[2],
				BNO055_MAG_OFFSET_Y_LSB);
				a_data_u8r[3] =
				BNO055_GET_BITSLICE(a_data_u8r[3],
				BNO055_MAG_OFFSET_Y_MSB);
				mag_offset->y = (_s16)((((_s32)
				(_s8)(a_data_u8r[3])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[2]));

				/* Read mag z offset value*/
				a_data_u8r[4] =
				BNO055_GET_BITSLICE(a_data_u8r[4],
				BNO055_MAG_OFFSET_Z_LSB);
				a_data_u8r[5] =
				BNO055_GET_BITSLICE(a_data_u8r[5],
				BNO055_MAG_OFFSET_Z_MSB);
				mag_offset->z = (_s16)((((_s32)
				(_s8)(a_data_u8r[5])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[4]));

				/* Read mag radius value
				it the two bytes of data */
				comres += p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_MAG_RADIUS_LSB__REG, a_data_u8r, 2);
				if (comres == SUCCESS) {
					/* Array holding the mag radius values
					a_data_u8r[0] - radius->LSB
					a_data_u8r[1] - radius->MSB
					*/
					a_data_u8r[0] =
					BNO055_GET_BITSLICE(a_data_u8r[0],
					BNO055_MAG_RADIUS_LSB);
					a_data_u8r[1] =
					BNO055_GET_BITSLICE(a_data_u8r[1],
					BNO055_MAG_RADIUS_MSB);
					mag_offset->r = (_s16)((((_s32)
					(_s8)(a_data_u8r[1])) <<
					(BNO055_SHIFT_8_POSITION)) |
					(a_data_u8r[0]));
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
		} else {
		return ERROR;
		}
	}
	return comres;
}

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
struct bno055_mag_offset *mag_offset)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data1_u8r = BNO055_ZERO_U8X;
_u8 v_data2_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
		status += bno055_get_operation_mode(&prev_opmode);
		if (status == SUCCESS) {
			if (prev_opmode != OPERATION_MODE_CONFIG)
				status += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					/* write Mag offset x value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_X_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(mag_offset->x & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_X_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_X_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_X_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(mag_offset->x  >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_X_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_X_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write Mag offset y value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Y_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(mag_offset->y & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_Y_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_Y_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Y_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(mag_offset->y >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_Y_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_Y_MSB__REG,
						&v_data2_u8r, 1);
					}
				/* write Mag offset z value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Z_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(mag_offset->z & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_Z_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_Z_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OFFSET_Z_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(mag_offset->z >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_OFFSET_Z_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OFFSET_Z_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write Mag radius value*/
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_RADIUS_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(mag_offset->r & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_RADIUS_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_RADIUS_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_RADIUS_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(mag_offset->r >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_MAG_RADIUS_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_RADIUS_MSB__REG,
						&v_data2_u8r, 1);
					}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
struct bno055_gyro_offset  *gyro_offset)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	/* Array holding the gyro offset values
	a_data_u8r[0] - offset x->LSB
	a_data_u8r[1] - offset x->MSB
	a_data_u8r[2] - offset y->LSB
	a_data_u8r[3] - offset y->MSB
	a_data_u8r[4] - offset z->LSB
	a_data_u8r[5] - offset z->MSB
	*/
	_u8 a_data_u8r[6] = {0, 0, 0, 0, 0, 0};
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro offset is
		available in the page zero*/
		if (p_bno055->page_id != PAGE_ZERO)
			/* Write the page zero*/
			comres = bno055_write_page_id(PAGE_ZERO);
		if (status == SUCCESS) {
			/* Read gyro offset value it the six bytes of data*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_OFFSET_X_LSB__REG, a_data_u8r, 6);
			if (comres == SUCCESS) {
				/* Read gyro x offset value*/
				a_data_u8r[0] =
				BNO055_GET_BITSLICE(a_data_u8r[0],
				BNO055_GYRO_OFFSET_X_LSB);
				a_data_u8r[1] =
				BNO055_GET_BITSLICE(a_data_u8r[1],
				BNO055_GYRO_OFFSET_X_MSB);
				gyro_offset->x = (_s16)((((_s32)
				(_s8)(a_data_u8r[1])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[0]));

				/* Read gyro y offset value*/
				a_data_u8r[2] =
				BNO055_GET_BITSLICE(a_data_u8r[2],
				BNO055_GYRO_OFFSET_Y_LSB);
				a_data_u8r[3] =
				BNO055_GET_BITSLICE(a_data_u8r[3],
				BNO055_GYRO_OFFSET_Y_MSB);
				gyro_offset->y = (_s16)((((_s32)
				(_s8)(a_data_u8r[3])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[2]));

				/* Read gyro z offset value*/
				a_data_u8r[4] =
				BNO055_GET_BITSLICE(a_data_u8r[4],
				BNO055_GYRO_OFFSET_Z_LSB);
				a_data_u8r[5] =
				BNO055_GET_BITSLICE(a_data_u8r[5],
				BNO055_GYRO_OFFSET_Z_MSB);
				gyro_offset->z = (_s16)((((_s32)
				(_s8)(a_data_u8r[5])) <<
				(BNO055_SHIFT_8_POSITION)) | (a_data_u8r[4]));
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
struct bno055_gyro_offset  *gyro_offset)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data1_u8r = BNO055_ZERO_U8X;
_u8 v_data2_u8r = BNO055_ZERO_U8X;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
		mode is in config mode, this part of code is checking the
		current operation mode and set the config mode */
	status += bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
				status += bno055_set_operation_mode
				(OPERATION_MODE_CONFIG);
				if (status == SUCCESS) {
					/* write gryo offset x value*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_X_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(gyro_offset->x & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_X_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_X_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_X_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(gyro_offset->x  >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_X_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_X_MSB__REG,
						&v_data2_u8r, 1);
					}

					/* write gryo offset y value*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_Y_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(gyro_offset->y & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_Y_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_Y_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_Y_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(gyro_offset->y >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_Y_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_Y_MSB__REG,
						&v_data2_u8r, 1);
					}
				/* write gryo offset z value*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_Z_LSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(gyro_offset->z & 0x00FF));
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_Z_LSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_Z_LSB__REG,
						&v_data2_u8r, 1);
					}

					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_OFFSET_Z_MSB__REG,
					&v_data2_u8r, 1);
					if (comres == SUCCESS) {
						v_data1_u8r = ((_s8)
						(gyro_offset->z >>
						BNO055_SHIFT_8_POSITION)
						& 0x00FF);
						v_data2_u8r =
						BNO055_SET_BITSLICE(v_data2_u8r,
						BNO055_GYRO_OFFSET_Z_MSB,
						v_data1_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_OFFSET_Z_MSB__REG,
						&v_data2_u8r, 1);
					}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
/********************************************************/
 /************** PAGE1 Functions *********************/
/********************************************************/
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
_u8 *accel_range)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel g range */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_RANGE__REG,
			&v_data_u8r, 1);
			*accel_range =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_RANGE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_range)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				if (accel_range < BNO055_FIVE_U8X) {
					/* Write the value of accel range*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_RANGE__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACCEL_RANGE,
						accel_range);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_RANGE__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *accel_bw)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel bandwidth is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel bandwidth */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_BW__REG, &v_data_u8r, 1);
			*accel_bw =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_BW);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_bw)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				if (accel_bw < BNO055_EIGHT_U8X) {
					/* Write the accel */
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_BW__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r, BNO055_ACCEL_BW,
						accel_bw);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_BW__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *accel_power_mode)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel power mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel bandwidth */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_POWER_MODE__REG, &v_data_u8r, 1);
			*accel_power_mode =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_POWER_MODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_power_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				if (accel_power_mode < BNO055_SIX_U8X) {
					/* Write the value of accel bandwidth*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_POWER_MODE__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACCEL_POWER_MODE,
						accel_power_mode);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_POWER_MODE__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *mag_data_output_rate)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, output data rate
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the mag output data rate*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_DATA_OUTPUT_RATE__REG,
			&v_data_u8r, 1);
			*mag_data_output_rate =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_DATA_OUTPUT_RATE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 mag_data_output_rate)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
	if (status == SUCCESS) {
		/* Write page as one */
		pg_stat = bno055_write_page_id(PAGE_ONE);
		if (pg_stat == SUCCESS) {
			if (mag_data_output_rate
				< BNO055_EIGHT_U8X) {
				/* Write the value of
				mag output data rate*/
				comres =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_MAG_DATA_OUTPUT_RATE__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_MAG_DATA_OUTPUT_RATE,
					mag_data_output_rate);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_DATA_OUTPUT_RATE__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = E_BNO055_OUT_OF_RANGE;
			}
		} else {
		comres = ERROR;
		}
	} else {
	return ERROR;
	}
} else {
return ERROR;
}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *mag_operation_mode)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag operation mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of mag operation mode*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_OPERATION_MODE__REG, &v_data_u8r, 1);
			*mag_operation_mode =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_OPERATION_MODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 mag_operation_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				if (mag_operation_mode
					< BNO055_FIVE_U8X) {
					/* Write the value
					of mag operation mode*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_OPERATION_MODE__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_MAG_OPERATION_MODE,
						mag_operation_mode);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_OPERATION_MODE__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *mag_power_mode)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, mag power mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of mag power mode */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_POWER_MODE__REG,
			&v_data_u8r, 1);
			*mag_power_mode =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_POWER_MODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 mag_power_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode(
		OPERATION_MODE_CONFIG);
	if (status == SUCCESS) {
		/* Write page as one */
		pg_stat = bno055_write_page_id(PAGE_ONE);
		if (pg_stat == SUCCESS) {
			if (mag_power_mode < BNO055_FOUR_U8X) {
				/* Write the value of mag power mode*/
				comres =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_MAG_POWER_MODE__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_MAG_POWER_MODE, mag_power_mode);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_POWER_MODE__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = E_BNO055_OUT_OF_RANGE;
			}
		} else {
		comres = ERROR;
		}
	} else {
	return ERROR;
	}
} else {
	return ERROR;
}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_range)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro range */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_RANGE__REG, &v_data_u8r, 1);
			*gyro_range =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_RANGE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_range)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				if (gyro_range < BNO055_FIVE_U8X) {
					/* Write the value of gyro range*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_RANGE__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_GYRO_RANGE,
						gyro_range);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_RANGE__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_bw)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro bandwidth is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_BW__REG,
			&v_data_u8r, 1);
			*gyro_bw =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_BW);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_bw)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 gyro_opmode = BNO055_ZERO_U8X;
_u8 gyro_autoslpdur = BNO055_ZERO_U8X;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
	if (status == SUCCESS) {
		/* Write page as one */
		pg_stat = bno055_write_page_id(PAGE_ONE);
	if (pg_stat == SUCCESS) {
		/* Write the value of gyro bandwidth */
		if ((gyro_bw == BNO055_ZERO_U8X ||
			gyro_bw > BNO055_ZERO_U8X) &&
			gyro_bw < BNO055_EIGHT_U8X) {
			switch (gyro_bw) {
			case GYRO_BW_523HZ:
			gyro_bw = GYRO_BW_523HZ;
			break;
			case GYRO_BW_230HZ:
			gyro_bw = GYRO_BW_230HZ;
			break;
			case GYRO_BW_116HZ:
			gyro_bw = GYRO_BW_116HZ;
			break;
			case GYRO_BW_47HZ:
			gyro_bw = GYRO_BW_47HZ;
			break;
			case GYRO_BW_23HZ:
			gyro_bw = GYRO_BW_23HZ;
			break;
			case GYRO_BW_12HZ:
			gyro_bw = GYRO_BW_12HZ;
			break;
			case GYRO_BW_64HZ:
			gyro_bw = GYRO_BW_64HZ;
			break;
			case GYRO_BW_32HZ:
			gyro_bw = GYRO_BW_32HZ;
			break;
			default:
			break;
			}
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_BW__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r = BNO055_SET_BITSLICE
				(v_data_u8r,
				BNO055_GYRO_BW,
				gyro_bw);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_BW__REG,
				&v_data_u8r, 1);
			}
			comres = bno055_get_gyro_power_mode
			(&gyro_opmode);
			if (comres == SUCCESS) {
				if (gyro_opmode ==
				GYRO_POWER_MODE_ADVANCE_POWERSAVE) {
					comres +=
					bno055_get_gyro_auto_sleep_dur
					(&gyro_autoslpdur);
					if (comres == SUCCESS) {
						comres +=
						bno055_gyro_set_auto_sleep_dur
						(gyro_autoslpdur,
						gyro_bw);
					}
				}
			}
		} else {
		comres = E_BNO055_OUT_OF_RANGE;
		}
	} else {
	comres = ERROR;
	}
} else {
return ERROR;
}
} else {
return ERROR;
}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_power_mode)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro power mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Write the value of gyro power mode*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_POWER_MODE__REG,
			&v_data_u8r, 1);
			*gyro_power_mode =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_POWER_MODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_power_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 gyro_autosleepduration = BNO055_ZERO_U8X;
_u8 gyro_bw = BNO055_ZERO_U8X;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
	if (status == SUCCESS) {
		/* Write page as one */
		pg_stat = bno055_write_page_id(PAGE_ONE);
		if (pg_stat == SUCCESS) {
			/* Write the value of power mode*/
			if ((gyro_power_mode == BNO055_ZERO_U8X ||
			gyro_power_mode > BNO055_ZERO_U8X) &&
			gyro_power_mode < BNO055_FIVE_U8X) {
				switch (gyro_power_mode) {
				case GYRO_POWER_MODE_NORMAL:
				gyro_power_mode =
				GYRO_POWER_MODE_NORMAL;
				break;
				case GYRO_POWER_MODE_FASTPOWERUP:
				gyro_power_mode =
				GYRO_POWER_MODE_FASTPOWERUP;
				break;
				case GYRO_POWER_MODE_DEEPSUSPEND:
				gyro_power_mode =
				GYRO_POWER_MODE_DEEPSUSPEND;
				break;
				case GYRO_POWER_MODE_SUSPEND:
				gyro_power_mode =
				GYRO_POWER_MODE_SUSPEND;
				break;
				case GYRO_POWER_MODE_ADVANCE_POWERSAVE:
				comres = bno055_get_gyro_bw
				(&gyro_bw);
				comres += bno055_get_gyro_auto_sleep_dur
				(&gyro_autosleepduration);
				if (comres == SUCCESS)
					bno055_gyro_set_auto_sleep_dur
					(gyro_autosleepduration,
					gyro_bw);
					comres +=
					bno055_write_page_id(PAGE_ONE);
					gyro_power_mode =
					GYRO_POWER_MODE_ADVANCE_POWERSAVE;
				break;
				default:
				break;
				}
				if (comres == SUCCESS)
					comres +=
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_POWER_MODE__REG,
					&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_POWER_MODE,
					gyro_power_mode);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_POWER_MODE__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = E_BNO055_OUT_OF_RANGE;
			}
		} else {
		comres = ERROR;
		}
	} else {
	return ERROR;
	}
} else {
	return ERROR;
}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *sleep_tmr)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel sleep mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* read the value of accel sleep mode */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLEEP_MODE__REG, &v_data_u8r, 1);
			*sleep_tmr =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLEEP_MODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 sleep_tmr)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					if (sleep_tmr < BNO055_TWO_U8X) {
						/*Write the value
						of accel sleep mode*/
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_SLEEP_MODE__REG,
						&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACCEL_SLEEP_MODE,
						sleep_tmr);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_SLEEP_MODE__REG,
						&v_data_u8r, 1);
					}
					} else {
					comres = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *sleep_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel sleep duration
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel sleep duration */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLEEP_DUR__REG, &v_data_u8r, 1);
			*sleep_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLEEP_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 sleep_dur)
	{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					if (sleep_dur < BNO055_SIXTEEN_U8X) {
						/* Write the accel
						sleep duration*/
						comres =
						p_bno055->BNO055_BUS_READ_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_SLEEP_DUR__REG,
						&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_ACCEL_SLEEP_DUR,
						sleep_dur);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_SLEEP_DUR__REG,
						&v_data_u8r, 1);
					}
					} else {
					comres = E_BNO055_OUT_OF_RANGE;
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_sleep_dur(_u8 *sleep_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the gyro sleep duration */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_SLEEP_DUR__REG, &v_data_u8r, 1);
			*sleep_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_SLEEP_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_sleep_dur(_u8 sleep_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				if (sleep_dur < BNO055_EIGHT_U8X) {
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_SLEEP_DUR__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						/* Write the gyro
						sleep duration */
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_GYRO_SLEEP_DUR,
						sleep_dur);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_SLEEP_DUR__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *auto_sleep_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro auto sleep duration */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_AUTO_SLEEP_DUR__REG, &v_data_u8r, 1);
			*auto_sleep_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_AUTO_SLEEP_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 auto_sleep_dur, _u8 bw)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 v_auto_sleep_dur_u8r;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value of gyro sleep duration */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_AUTO_SLEEP_DUR__REG,
				&v_data_u8r, 1);
				if (auto_sleep_dur < BNO055_EIGHT_U8X) {
					switch (bw) {
					case GYRO_BW_523HZ:
					if (auto_sleep_dur >
						BNO055_GYRO_4MS_AUTOSLPDUR)
						v_auto_sleep_dur_u8r =
						auto_sleep_dur;
					else
						v_auto_sleep_dur_u8r =
						BNO055_GYRO_4MS_AUTOSLPDUR;
					break;
					case GYRO_BW_230HZ:
					if (auto_sleep_dur >
						BNO055_GYRO_4MS_AUTOSLPDUR)
						v_auto_sleep_dur_u8r =
						auto_sleep_dur;
					else
						v_auto_sleep_dur_u8r =
						BNO055_GYRO_4MS_AUTOSLPDUR;
					break;
					case GYRO_BW_116HZ:
					if (auto_sleep_dur >
						BNO055_GYRO_4MS_AUTOSLPDUR)
						v_auto_sleep_dur_u8r =
						auto_sleep_dur;
					else
						v_auto_sleep_dur_u8r =
						BNO055_GYRO_4MS_AUTOSLPDUR;
					break;
					case GYRO_BW_47HZ:
					if (auto_sleep_dur >
						BNO055_GYRO_5MS_AUTOSLPDUR)
						v_auto_sleep_dur_u8r =
						auto_sleep_dur;
					else
						v_auto_sleep_dur_u8r =
						BNO055_GYRO_5MS_AUTOSLPDUR;
					break;
					case GYRO_BW_23HZ:
					if (auto_sleep_dur >
						BNO055_GYRO_10MS_AUTOSLPDUR)
						v_auto_sleep_dur_u8r =
						auto_sleep_dur;
					else
						v_auto_sleep_dur_u8r =
						BNO055_GYRO_10MS_AUTOSLPDUR;
					break;
					case GYRO_BW_12HZ:
					if (auto_sleep_dur >
						BNO055_GYRO_20MS_AUTOSLPDUR)
						v_auto_sleep_dur_u8r =
						auto_sleep_dur;
					else
						v_auto_sleep_dur_u8r =
						BNO055_GYRO_20MS_AUTOSLPDUR;
					break;
					case GYRO_BW_64HZ:
					if (auto_sleep_dur >
						BNO055_GYRO_10MS_AUTOSLPDUR)
						v_auto_sleep_dur_u8r =
						auto_sleep_dur;
					else
						v_auto_sleep_dur_u8r =
						BNO055_GYRO_10MS_AUTOSLPDUR;
					break;
					case GYRO_BW_32HZ:
					if (auto_sleep_dur >
						BNO055_GYRO_20MS_AUTOSLPDUR)
						v_auto_sleep_dur_u8r =
						auto_sleep_dur;
					else
						v_auto_sleep_dur_u8r =
						BNO055_GYRO_20MS_AUTOSLPDUR;
					break;
					default:
					if (auto_sleep_dur >
						BNO055_GYRO_4MS_AUTOSLPDUR)
						v_auto_sleep_dur_u8r =
						auto_sleep_dur;
					else
						v_auto_sleep_dur_u8r =
						BNO055_GYRO_4MS_AUTOSLPDUR;
					break;
					}
					if (comres == SUCCESS) {
						v_data_u8r = BNO055_SET_BITSLICE
						(v_data_u8r,
						BNO055_GYRO_AUTO_SLEEP_DUR,
						v_auto_sleep_dur_u8r);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_AUTO_SLEEP_DUR__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = E_BNO055_OUT_OF_RANGE;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *sleep_mode)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page,mag sleep mode is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of mag sleep mode*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_SLEEP_MODE__REG, &v_data_u8r, 1);
			*sleep_mode =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_SLEEP_MODE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 sleep_mode)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_SLEEP_MODE__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						/* Write the value
						of mag sleep mode*/
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_MAG_SLEEP_MODE,
						sleep_mode);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_SLEEP_MODE__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *sleep_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page,mag sleep duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of mag sleep duration*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_MAG_SLEEP_DUR__REG, &v_data_u8r, 1);
			*sleep_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_MAG_SLEEP_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 sleep_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_MAG_SLEEP_DUR__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						/* Write the value of
						mag sleep duration */
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_MAG_SLEEP_DUR,
						sleep_dur);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_MAG_SLEEP_DUR__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_mask_gyro_anymotion(
_u8 *gyro_anymotion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro anymotion interrupt mask*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANYMOTION_INT_MASK__REG,
			&v_data_u8r, 1);
			*gyro_anymotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANYMOTION_INT_MASK);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to write the gyro anymotion interrupt mask
 *	from page one register from 0x0F bit 2
 *
 *	@param gyro_anymotion : The value of gyro anymotion interrupt mask
 *		gyro_anymotion |   result
 *     --------------- | ------------
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
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_mask_gyro_anymotion(
_u8 gyro_anymotion)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Write the value of gyro anymotion interrupt mask*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANYMOTION_INT_MASK__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GYRO_ANYMOTION_INT_MASK, gyro_anymotion);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANYMOTION_INT_MASK__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *gyro_highrate)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate interrupt mask*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_INT_MASK__REG,
			&v_data_u8r, 1);
			*gyro_highrate =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_INT_MASK);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_highrate)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_INT_MASK__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				/* Write the value of gyro
				highrate interrupt mask*/
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_INT_MASK, gyro_highrate);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_INT_MASK__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *accel_high_g)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel highg interrupt mask*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_INT_MASK__REG,
			&v_data_u8r, 1);
			*accel_high_g =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_HIGH_G_INT_MASK);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_high_g)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_INT_MASK__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				/* Write the value of accel
				highg interrupt mask*/
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_INT_MASK, accel_high_g);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_INT_MASK__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *accel_anymotion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* The value of accel anymotion interrupt mask*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANYMOTION_INT_MASK__REG, &v_data_u8r, 1);
			*accel_anymotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_ANYMOTION_INT_MASK);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_anymotion)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Write the value of accel anymotion interrupt mask*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANYMOTION_INT_MASK__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANYMOTION_INT_MASK,
				accel_anymotion);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_INT_MASK__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *accel_nomotion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel nomotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel nomotion interrupt mask*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_NOMOTION_INT_MASK__REG, &v_data_u8r, 1);
			*accel_nomotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_NOMOTION_INT_MASK);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_nomotion)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel
		nomotion interrupt mask is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_NOMOTION_INT_MASK__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				/* Write the value of accel
				nomotion interrupt mask*/
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_NOMOTION_INT_MASK, accel_nomotion);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_NOMOTION_INT_MASK__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *gyro_anymotion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion interrupt  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro anymotion interrupt */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANYMOTION_INT__REG, &v_data_u8r, 1);
			*gyro_anymotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANYMOTION_INT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_anymotion)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion interrupt  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Write the value of gyro anymotion interrupt */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANYMOTION_INT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GYRO_ANYMOTION_INT, gyro_anymotion);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANYMOTION_INT__REG, &v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *gyro_highrate)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate interrupt is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate interrupt */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_INT__REG, &v_data_u8r, 1);
			*gyro_highrate =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_INT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to write the gyro highrate interrupt
 *	from page one register from 0x10 bit 3
 *
 *	@param gyro_highrate : The value of gyro highrate interrupt
 *	   gyro_highrate     | result
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
_u8 gyro_highrate)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate interrupt is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_INT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				/* Write the value of gyro highrate interrupt */
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_INT, gyro_highrate);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_INT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to read the accel highg interrupt
 *	from page one register from 0x10 bit 5
 *
 *	@param accel_high_g : The value of accel highg interrupt
 *		   accel_high_g | result
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_high_g(
_u8 *accel_high_g)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg interrupt  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel highg interrupt*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_INT__REG, &v_data_u8r, 1);
			*accel_high_g =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_HIGH_G_INT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to write the accel highg interrupt
 *	from page one register from 0x10 bit 5
 *
 *	@param accel_high_g : The value of accel highg interrupt
 *		   accel_high_g | result
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
_u8 accel_high_g)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg interrupt is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_INT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				/* Write the value of accel highg interrupt*/
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_INT, accel_high_g);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_INT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to read the accel anymotion interrupt
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
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_int_accel_anymotion(
_u8 *accel_anymotion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion interrupt  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel anymotion interrupt */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANYMOTION_INT__REG, &v_data_u8r, 1);
			*accel_anymotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_ANYMOTION_INT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
 *	Threshold:
 *
 *	bno055_set_accel_anymotion_thres()
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_int_accel_anymotion(
_u8 accel_anymotion)
{
	BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel range is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Write the value of accel anymotion interrupt */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANYMOTION_INT__REG,
			&v_data_u8r, 1);
			if (comres == SUCCESS) {
				v_data_u8r =
				BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANYMOTION_INT,
				accel_anymotion);
				comres +=
				p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_INT__REG,
				&v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 *accel_nomotion)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel nomotion interrupt is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel nomotion interrupt*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_NOMOTION_INT__REG, &v_data_u8r, 1);
			*accel_nomotion =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_NOMOTION_INT);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_nomotion)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
	} else {
	/*condition check for page,
	accel nomotion interrupt is
	available in the page one*/
	if (p_bno055->page_id != PAGE_ONE)
		/* Write page as one */
		status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_NOMOTION_INT__REG, &v_data_u8r, 1);
			if (comres == SUCCESS) {
				/* Write the value of
				accel nomotion interrupt */
				v_data_u8r = BNO055_SET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_NOMOTION_INT, accel_nomotion);
				comres += p_bno055->BNO055_BUS_WRITE_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_NOMOTION_INT__REG, &v_data_u8r, 1);
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to read the accel any motion threshold
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
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_anymotion_thres(
_u8 *accel_anymotion_thres)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel any motion threshold  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel any motion threshold */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANYMOTION_THRES__REG, &v_data_u8r, 1);
			*accel_anymotion_thres =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_ANYMOTION_THRES);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_anymotion_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_THRES__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					/* Write the value of
					accel any motion threshold*/
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_ANYMOTION_THRES,
					accel_anymotion_thres);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANYMOTION_THRES__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *accel_anymotion_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion duration  is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel anymotion duration */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_ANYMOTION_DUR_SET__REG, &v_data_u8r, 1);
			*accel_anymotion_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_ANYMOTION_DUR_SET);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to write the accel anymotion duration
 *	from page one register from 0x12 bit 0 to 1
 *
 *	@param accel_anymotion_dur : The value of accel anymotion duration
 *
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_anymotion_dur(
_u8 accel_anymotion_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_DUR_SET__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					/* Write the value of
					accel anymotion duration*/
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_ANYMOTION_DUR_SET,
					accel_anymotion_dur);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANYMOTION_DUR_SET__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 channel, _u8 *data)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel anymotion enable is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			switch (channel) {
			case BNO055_ACCEL_ANYMOTION_NOMOTION_X_AXIS:
				/* Read the value of accel anymotion x enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_X_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANYMOTION_X_AXIS);
				break;
			case BNO055_ACCEL_ANYMOTION_NOMOTION_Y_AXIS:
				/* Read the value of accel anymotion y enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_Y_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANYMOTION_Y_AXIS);
				break;
			case BNO055_ACCEL_ANYMOTION_NOMOTION_Z_AXIS:
				/* Read the value of accel anymotion z enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_Z_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_ANYMOTION_Z_AXIS);
				break;
			default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 channel, _u8 data)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				switch (channel) {
				case BNO055_ACCEL_ANYMOTION_NOMOTION_X_AXIS:
				/* Write the value of
				accel anymotion x enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_X_AXIS__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_ACCEL_ANYMOTION_X_AXIS,
					data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANYMOTION_X_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				case BNO055_ACCEL_ANYMOTION_NOMOTION_Y_AXIS:
				/* Write the value of
				accel anymotion y enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_Y_AXIS__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_ACCEL_ANYMOTION_Y_AXIS,
					data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANYMOTION_Y_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				case BNO055_ACCEL_ANYMOTION_NOMOTION_Z_AXIS:
				/* Write the value of
				accel anymotion z enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_ANYMOTION_Z_AXIS__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_ACCEL_ANYMOTION_Z_AXIS,
					data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_ANYMOTION_Z_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 channel, _u8 *data)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg enable is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			switch (channel) {
			case BNO055_ACCEL_HIGH_G_X_AXIS:
				/* Read the value of accel x highg enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_X_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_X_AXIS);
				break;
			case BNO055_ACCEL_HIGH_G_Y_AXIS:
				/* Read the value of accel y highg enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_Y_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_Y_AXIS);
				break;
			case BNO055_ACCEL_HIGH_G_Z_AXIS:
				/* Read the value of accel z highg enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_Z_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_ACCEL_HIGH_G_Z_AXIS);
				break;
			default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 channel, _u8 data)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				switch (channel) {
				case BNO055_ACCEL_HIGH_G_X_AXIS:
				/* Write the value of
				accel x highg enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_X_AXIS__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_HIGH_G_X_AXIS, data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_X_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				case BNO055_ACCEL_HIGH_G_Y_AXIS:
				/* Write the value of
				accel y highg enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_Y_AXIS__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_HIGH_G_Y_AXIS,
					data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_Y_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				case BNO055_ACCEL_HIGH_G_Z_AXIS:
				/* Write the value of
				accel z highg enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_HIGH_G_Z_AXIS__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_HIGH_G_Z_AXIS, data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_Z_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
 *	to [highg duration  + 1] * 2 ms
 *
 *	in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_accel_high_g_dur(
_u8 *accel_high_g_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X) {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel highg duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel highg duration*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_DUR__REG,
			&v_data_u8r, 1);
			*accel_high_g_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_HIGH_G_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
 *	to [highg duration  + 1] * 2 ms
 *
 *	in a range from 2 ms to 512 ms
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_accel_high_g_dur(
_u8 accel_high_g_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X) {
	return E_NULL_PTR;
} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_DUR__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						/* Write the value of
						accel highg duration*/
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_ACCEL_HIGH_G_DUR,
						accel_high_g_dur);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_HIGH_G_DUR__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *accel_high_g_thres)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, highg threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of highg threshold */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_HIGH_G_THRES__REG,
			&v_data_u8r, 1);
			*accel_high_g_thres =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_HIGH_G_THRES);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_high_g_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_HIGH_G_THRES__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						/* Write the value of
						accel highg threshold */
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_ACCEL_HIGH_G_THRES,
						accel_high_g_thres);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_ACCEL_HIGH_G_THRES__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *accel_slow_no_motion_thres)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel slownomotion threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of slownomotion threshold */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLOW_NO_MOTION_THRES__REG,
			&v_data_u8r, 1);
			*accel_slow_no_motion_thres =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLOW_NO_MOTION_THRES);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_slow_no_motion_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value of
				slownomotion threshold */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_SLOW_NO_MOTION_THRES__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_SLOW_NO_MOTION_THRES,
					accel_slow_no_motion_thres);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_SLOW_NO_MOTION_THRES__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *accel_slow_no_motion_en)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel slownomotion enable is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of accel slownomotion enable */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__REG,
			&v_data_u8r, 1);
			*accel_slow_no_motion_en =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLOW_NO_MOTION_ENABLE);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to write accel slownomotion enable
 *	from page one register from 0x16 bit 0
 *
 *	@param accel_slow_no_motion_en : The value of accel slownomotion enable
 *	 accel_slow_no_motion_en    | result
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
_u8 accel_slow_no_motion_en)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					/* Read the value of
					accel slownomotion enable */
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_SLOW_NO_MOTION_ENABLE,
					accel_slow_no_motion_en);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_SLOW_NO_MOTION_ENABLE__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *accel_slow_no_motion_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, accel slownomotion duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/*read value of accel slownomotion duration*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_ACCEL_SLOW_NO_MOTION_DUR__REG, &v_data_u8r, 1);
			*accel_slow_no_motion_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_ACCEL_SLOW_NO_MOTION_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 accel_slow_no_motion_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_ACCEL_SLOW_NO_MOTION_DUR__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					/*Write the value of accel
					slownomotion duration*/
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_ACCEL_SLOW_NO_MOTION_DUR,
					accel_slow_no_motion_dur);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_ACCEL_SLOW_NO_MOTION_DUR__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 channel, _u8 *data)
{
/* Variable used to return value of
communication routine*/
BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/*condition check for page, gyro anymotion axis is
	available in the page one*/
	if (p_bno055->page_id != PAGE_ONE)
		/* Write page as one */
		status = bno055_write_page_id(PAGE_ONE);
	if (status == SUCCESS) {
		switch (channel) {
		case BNO055_GYRO_ANYMOTION_X_AXIS:
			/* Read the gyro anymotion x enable*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANYMOTION_X_AXIS__REG,
			&v_data_u8r, 1);
			*data =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANYMOTION_X_AXIS);
			break;
		case BNO055_GYRO_ANYMOTION_Y_AXIS:
			/* Read the gyro anymotion y enable*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANYMOTION_Y_AXIS__REG,
			&v_data_u8r, 1);
			*data =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANYMOTION_Y_AXIS);
			break;
		case BNO055_GYRO_ANYMOTION_Z_AXIS:
			/* Read the gyro anymotion z enable*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANYMOTION_Z_AXIS__REG,
			&v_data_u8r, 1);
			*data =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANYMOTION_Z_AXIS);
			break;
		default:
			comres = E_BNO055_OUT_OF_RANGE;
			break;
		}
	} else {
	return ERROR;
	}
}
return comres;
}
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
_u8 channel, _u8  data)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				switch (channel) {
				case BNO055_GYRO_ANYMOTION_X_AXIS:
					/* Write the gyro
					anymotion x enable*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANYMOTION_X_AXIS__REG,
					&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_ANYMOTION_X_AXIS,
					data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANYMOTION_X_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				case BNO055_GYRO_ANYMOTION_Y_AXIS:
					/* Write the gyro
					anymotion y enable*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANYMOTION_Y_AXIS__REG,
					&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_ANYMOTION_Y_AXIS, data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANYMOTION_Y_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				case BNO055_GYRO_ANYMOTION_Z_AXIS:
					/* Write the gyro
					anymotion z enable*/
					comres =
					p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANYMOTION_Z_AXIS__REG,
					&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_ANYMOTION_Z_AXIS,
					data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANYMOTION_Z_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				default:
					comres = E_BNO055_OUT_OF_RANGE;
					break;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 channel, _u8 *data)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate enable is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			switch (channel) {
			case BNO055_GYRO_HIGHRATE_X_AXIS:
				/* Read the gyro highrate x enable */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_X_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_X_AXIS);
				break;
			case BNO055_GYRO_HIGHRATE_Y_AXIS:
				/* Read the gyro highrate y enable */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Y_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_Y_AXIS);
				break;
			case BNO055_GYRO_HIGHRATE_Z_AXIS:
				/* Read the gyro highrate z enable */
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Z_AXIS__REG,
				&v_data_u8r, 1);
				*data =
				BNO055_GET_BITSLICE(v_data_u8r,
				BNO055_GYRO_HIGHRATE_Z_AXIS);
				break;
			default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
			}
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 channel, _u8 data)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				switch (channel) {
				case BNO055_GYRO_HIGHRATE_X_AXIS:
				/* Write the value of
				gyro highrate x enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_X_AXIS__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_HIGHRATE_X_AXIS, data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_X_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				case BNO055_GYRO_HIGHRATE_Y_AXIS:
				/* Write the value of
				gyro highrate y enable*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Y_AXIS__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(
					v_data_u8r,
					BNO055_GYRO_HIGHRATE_Y_AXIS, data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Y_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				case BNO055_GYRO_HIGHRATE_Z_AXIS:
				/* Write the value of
				gyro highrate z enable*/
				comres =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Z_AXIS__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r = BNO055_SET_BITSLICE
					(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Z_AXIS, data);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Z_AXIS__REG,
					&v_data_u8r, 1);
				}
				break;
				default:
				comres = E_BNO055_OUT_OF_RANGE;
				break;
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_anymotion_filter)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion filter is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro anymotion filter*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANYMOTION_FILTER__REG,
			&v_data_u8r, 1);
			*gyro_anymotion_filter =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANYMOTION_FILTER);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_anymotion_filter)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value of
				gyro anymotion filter*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANYMOTION_FILTER__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_ANYMOTION_FILTER,
					gyro_anymotion_filter);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANYMOTION_FILTER__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_highrate_filter)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate filter is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate filter */
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_FILTER__REG, &v_data_u8r, 1);
			*gyro_highrate_filter =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_FILTER);
		} else {
		return ERROR;
		}
	}
	return comres;
}
/*!
 *	@brief This API used to write gyro highrate filter
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
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_filter(
_u8 gyro_highrate_filter)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value of
				gyro highrate filter*/
				comres =
				p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_FILTER__REG,
				&v_data_u8r,
				1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_FILTER,
					gyro_highrate_filter);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_FILTER__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_highrate_x_thres)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate x threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate threshold*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_X_THRES__REG, &v_data_u8r, 1);
			*gyro_highrate_x_thres =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_X_THRES);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_highrate_x_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value of
				gyro highrate x threshold*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_X_THRES__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_X_THRES,
					gyro_highrate_x_thres);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_X_THRES__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_x_hyst(
_u8 *gyro_highrate_x_hyst)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page,gyro highrate x hysteresis is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate x hysteresis*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_X_HYST__REG, &v_data_u8r, 1);
			*gyro_highrate_x_hyst =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_X_HYST);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
 *	using this (255 + 256 * gyro_highrate_x_hys) *4 LSB
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
_u8 gyro_highrate_x_hyst)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/*Write the value of
				gyro highrate x hysteresis*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_X_HYST__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_X_HYST,
					gyro_highrate_x_hyst);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_X_HYST__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_highrate_x_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate x duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate x duration*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_X_DUR__REG, &v_data_u8r, 1);
			*gyro_highrate_x_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_X_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_highrate_x_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					/* Write the value
					of gyro highrate x duration*/
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_X_DUR__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_GYRO_HIGHRATE_X_DUR,
						gyro_highrate_x_dur);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_HIGHRATE_X_DUR__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *gyro_highrate_y_thres)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate y threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate y threshold*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Y_THRES__REG, &v_data_u8r, 1);
			*gyro_highrate_y_thres =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Y_THRES);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
 *  gyro_range	  |	threshold		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.5dps      |   1LSB
 *     1000       |    31.25dps     |   1LSB
 *     500        |    15.625dps    |   1LSB
 *     125        |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_y_thres(
_u8 gyro_highrate_y_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value
				of gyro highrate y threshold*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Y_THRES__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Y_THRES,
					gyro_highrate_y_thres);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Y_THRES__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_highrate_y_hyst)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate y hysteresis is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate y hysteresis*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Y_HYST__REG, &v_data_u8r, 1);
			*gyro_highrate_y_hyst =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Y_HYST);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_highrate_y_hyst)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value of
				gyro highrate y hysteresis*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Y_HYST__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Y_HYST,
					gyro_highrate_y_hyst);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Y_HYST__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_highrate_y_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate y duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate y duration*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Y_DUR__REG, &v_data_u8r, 1);
			*gyro_highrate_y_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Y_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_highrate_y_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);

			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					/* Write the value
					of gyro highrate y duration*/
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Y_DUR__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_GYRO_HIGHRATE_Y_DUR,
						gyro_highrate_y_dur);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_HIGHRATE_Y_DUR__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
 *  gyro_range	  |	threshold		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.5dps      |   1LSB
 *     1000       |    31.25dps     |   1LSB
 *     500        |    15.625dps    |   1LSB
 *     125        |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_thres(
_u8 *gyro_highrate_z_thres)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate z threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate z threshold*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Z_THRES__REG,
			&v_data_u8r, 1);
			*gyro_highrate_z_thres =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Z_THRES);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
 *  gyro_range	  |	threshold		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.5dps      |   1LSB
 *     1000       |    31.25dps     |   1LSB
 *     500        |    15.625dps    |   1LSB
 *     125        |    7.8125dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_highrate_z_thres(
_u8 gyro_highrate_z_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value
				of gyro highrate z threshold*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Z_THRES__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Z_THRES,
					gyro_highrate_z_thres);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Z_THRES__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
 *  gyro_range	  |	 hysteresis		|     LSB
 * -------------  | -------------   | ---------
 *     2000       |    62.26dps     |   1LSB
 *     1000       |    31.13dps     |   1LSB
 *     500        |    15.56dps     |   1LSB
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_highrate_z_hyst(
_u8 *gyro_highrate_z_hyst)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate z hysteresis is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate z hysteresis*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Z_HYST__REG, &v_data_u8r, 1);
			*gyro_highrate_z_hyst =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Z_HYST);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_highrate_z_hyst)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value
				of gyro highrate z hysteresis*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_HIGHRATE_Z_HYST__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_HIGHRATE_Z_HYST,
					gyro_highrate_z_hyst);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Z_HYST__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_highrate_z_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro highrate z duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro highrate z duration*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_HIGHRATE_Z_DUR__REG, &v_data_u8r, 1);
			*gyro_highrate_z_dur =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_HIGHRATE_Z_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_highrate_z_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					/* Write the value of
					gyro highrate z duration*/
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_HIGHRATE_Z_DUR__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_GYRO_HIGHRATE_Z_DUR,
						gyro_highrate_z_dur);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_HIGHRATE_Z_DUR__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
 *  gyro_range	  |	threshold	  |	   LSB
 * -------------  | ------------- | ---------
 *     2000       |    1dps       |   1LSB
 *     1000       |    0.5dps     |   1LSB
 *     500        |    0.25dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_get_gyro_anymotion_thres(
_u8 *gyro_anymotion_thres)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page,gyro anymotion threshold is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro anymotion threshold*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_ANYMOTION_THRES__REG, &v_data_u8r, 1);
			*gyro_anymotion_thres =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_ANYMOTION_THRES);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
 *  gyro_range	  |	threshold	  |	   LSB
 * -------------  | ------------- | ---------
 *     2000       |    1dps       |   1LSB
 *     1000       |    0.5dps     |   1LSB
 *     500        |    0.25dps    |   1LSB
 *
 */
BNO055_RETURN_FUNCTION_TYPE bno055_set_gyro_anymotion_thres(
_u8 gyro_anymotion_thres)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
_u8 pg_stat = SUCCESS;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
} else {
/* The write operation effective only if the operation
mode is in config mode, this part of code is checking the
current operation mode and set the config mode */
status = bno055_get_operation_mode(&prev_opmode);
if (status == SUCCESS) {
	if (prev_opmode != OPERATION_MODE_CONFIG)
		status = bno055_set_operation_mode
		(OPERATION_MODE_CONFIG);
		if (status == SUCCESS) {
			/* Write page as one */
			pg_stat = bno055_write_page_id(PAGE_ONE);
			if (pg_stat == SUCCESS) {
				/* Write the value
				of gyro anymotion threshold*/
				comres = p_bno055->BNO055_BUS_READ_FUNC
				(p_bno055->dev_addr,
				BNO055_GYRO_ANYMOTION_THRES__REG,
				&v_data_u8r, 1);
				if (comres == SUCCESS) {
					v_data_u8r =
					BNO055_SET_BITSLICE(v_data_u8r,
					BNO055_GYRO_ANYMOTION_THRES,
					gyro_anymotion_thres);
					comres +=
					p_bno055->BNO055_BUS_WRITE_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_ANYMOTION_THRES__REG,
					&v_data_u8r, 1);
				}
			} else {
			comres = ERROR;
			}
		} else {
		return ERROR;
		}
	} else {
	return ERROR;
	}
}
if (prev_opmode != OPERATION_MODE_CONFIG)
	/* set the operation mode of
	previous operation mode*/
	comres += bno055_set_operation_mode
	(prev_opmode);
return comres;
}
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
_u8 *gyro_anymotion_slope_samples)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion slope samples is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/*Read the value of gyro anymotion slope samples*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_SLOPE_SAMPLES__REG, &v_data_u8r, 1);
			*gyro_anymotion_slope_samples =
			BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_SLOPE_SAMPLES);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_anymotion_slope_samples)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					/* Write the value of
					gyro anymotion slope samples*/
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_SLOPE_SAMPLES__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_GYRO_SLOPE_SAMPLES,
						gyro_anymotion_slope_samples);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_SLOPE_SAMPLES__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode of
		previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
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
_u8 *gyro_awake_dur)
{
	/* Variable used to return value of
	communication routine*/
	BNO055_RETURN_FUNCTION_TYPE comres = BNO055_ZERO_U8X;
	_u8 v_data_u8r = BNO055_ZERO_U8X;
	_u8 status = SUCCESS;
	/* Check the struct p_bno055 is empty */
	if (p_bno055 == BNO055_ZERO_U8X)  {
		return E_NULL_PTR;
		} else {
		/*condition check for page, gyro anymotion awake duration is
		available in the page one*/
		if (p_bno055->page_id != PAGE_ONE)
			/* Write page as one */
			status = bno055_write_page_id(PAGE_ONE);
		if (status == SUCCESS) {
			/* Read the value of gyro anymotion awake duration*/
			comres = p_bno055->BNO055_BUS_READ_FUNC
			(p_bno055->dev_addr,
			BNO055_GYRO_AWAKE_DUR__REG, &v_data_u8r, 1);
			*gyro_awake_dur = BNO055_GET_BITSLICE(v_data_u8r,
			BNO055_GYRO_AWAKE_DUR);
		} else {
		return ERROR;
		}
	}
	return comres;
}
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
_u8 gyro_awake_dur)
{
BNO055_RETURN_FUNCTION_TYPE comres = SUCCESS;
_u8 v_data_u8r = BNO055_ZERO_U8X;
_u8 status = SUCCESS;
_u8 pg_stat = SUCCESS;
_u8 prev_opmode = OPERATION_MODE_CONFIG;
/* Check the struct p_bno055 is empty */
if (p_bno055 == BNO055_ZERO_U8X)  {
	return E_NULL_PTR;
	} else {
	/* The write operation effective only if the operation
	mode is in config mode, this part of code is checking the
	current operation mode and set the config mode */
	status = bno055_get_operation_mode(&prev_opmode);
	if (status == SUCCESS) {
		if (prev_opmode != OPERATION_MODE_CONFIG)
			status = bno055_set_operation_mode
			(OPERATION_MODE_CONFIG);
			if (status == SUCCESS) {
				/* Write page as one */
				pg_stat = bno055_write_page_id(PAGE_ONE);
				if (pg_stat == SUCCESS) {
					/* Write the value of gyro
					anymotion awake duration*/
					comres = p_bno055->BNO055_BUS_READ_FUNC
					(p_bno055->dev_addr,
					BNO055_GYRO_AWAKE_DUR__REG,
					&v_data_u8r, 1);
					if (comres == SUCCESS) {
						v_data_u8r =
						BNO055_SET_BITSLICE(v_data_u8r,
						BNO055_GYRO_AWAKE_DUR,
						gyro_awake_dur);
						comres +=
						p_bno055->BNO055_BUS_WRITE_FUNC
						(p_bno055->dev_addr,
						BNO055_GYRO_AWAKE_DUR__REG,
						&v_data_u8r, 1);
					}
				} else {
				comres = ERROR;
				}
			} else {
			return ERROR;
			}
		} else {
		return ERROR;
		}
	}
	if (prev_opmode != OPERATION_MODE_CONFIG)
		/* set the operation mode
		of previous operation mode*/
		comres += bno055_set_operation_mode
		(prev_opmode);
	return comres;
}
