/******************************************************************************
 *  Copyright (c) 2021, Xilinx, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1.  Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2.  Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *  3.  Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS py_intERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#pragma once


#define BME680_POLL_PERIOD_MS		10
#define BME680_CHIP_ID  			0x61
#define BME680_COEFF_SIZE			41
#define BME680_COEFF_ADDR1_LEN		25
#define BME680_COEFF_ADDR2_LEN		16
#define BME680_FIELD_LENGTH			15
#define BME680_FIELD_ADDR_OFFSET	17
#define BME680_SOFT_RESET_CMD   	0xb6
#define BME680_OK					0
#define BME680_E_NULL_PTR		    -1
#define BME680_E_COM_FAIL		    -2
#define BME680_E_DEV_NOT_FOUND		-3
#define BME680_E_INVALID_LENGTH		-4
#define BME680_W_DEFINE_PWR_MODE	1
#define BME680_W_NO_NEW_DATA        2
#define BME680_I_MIN_CORRECTION		1
#define BME680_I_MAX_CORRECTION		2
#define BME680_ADDR_RES_HEAT_VAL_ADDR	0x00
#define BME680_ADDR_RES_HEAT_RANGE_ADDR	0x02
#define BME680_ADDR_RANGE_SW_ERR_ADDR	0x04
#define BME680_ADDR_SENS_CONF_START	0x5A
#define BME680_ADDR_GAS_CONF_START	0x64
#define BME680_FIELD0_ADDR			0x1d
#define BME680_RES_HEAT0_ADDR		0x5a
#define BME680_GAS_WAIT0_ADDR		0x64
#define BME680_CONF_HEAT_CTRL_ADDR			0x70
#define BME680_CONF_ODR_RUN_GAS_NBC_ADDR	0x71
#define BME680_CONF_OS_H_ADDR			0x72
#define BME680_MEM_PAGE_ADDR			0xf3
#define BME680_CONF_T_P_MODE_ADDR		0x74
#define BME680_CONF_ODR_FILT_ADDR		0x75
#define BME680_COEFF_ADDR1				0x89
#define BME680_COEFF_ADDR2				0xe1
#define BME680_CHIP_ID_ADDR				0xd0
#define BME680_SOFT_RESET_ADDR			0xe0
#define BME680_ENABLE_HEATER			0x00
#define BME680_DISABLE_HEATER			0x08
#define BME680_DISABLE_GAS_MEAS			0x00
#define BME680_ENABLE_GAS_MEAS			0x01
#define BME680_OS_NONE					0
#define BME680_OS_1X					1
#define BME680_OS_2X					2
#define BME680_OS_4X					3
#define BME680_OS_8X					4
#define BME680_OS_16X					5
#define BME680_FILTER_SIZE_0			0
#define BME680_FILTER_SIZE_1	1
#define BME680_FILTER_SIZE_3	2
#define BME680_FILTER_SIZE_7	3
#define BME680_FILTER_SIZE_15	4
#define BME680_FILTER_SIZE_31	5
#define BME680_FILTER_SIZE_63	6
#define BME680_FILTER_SIZE_127	7
#define BME680_SLEEP_MODE	0
#define BME680_FORCED_MODE	1
#define BME680_RESET_PERIOD	10
#define BME680_MEM_PAGE0	0x10
#define BME680_MEM_PAGE1	0x00
#define BME680_HUM_REG_SHIFT_VAL	4
#define BME680_RUN_GAS_DISABLE	0
#define BME680_RUN_GAS_ENABLE	1
#define BME680_TMP_BUFFER_LENGTH	40
#define BME680_REG_BUFFER_LENGTH	6
#define BME680_FIELD_DATA_LENGTH	3
#define BME680_GAS_REG_BUF_LENGTH	20
#define BME680_GAS_HEATER_PROF_LEN_MAX  10
#define BME680_OST_SEL			1
#define BME680_OSP_SEL			2
#define BME680_OSH_SEL			4
#define BME680_GAS_MEAS_SEL		8
#define BME680_FILTER_SEL		16
#define BME680_HCNTRL_SEL		32
#define BME680_RUN_GAS_SEL		64
#define BME680_NBCONV_SEL		128
#define BME680_GAS_SENSOR_SEL		(BME680_GAS_MEAS_SEL | BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)
#define BME680_NBCONV_MIN		0
#define BME680_NBCONV_MAX		10
#define BME680_GAS_MEAS_MSK	0x30
#define BME680_NBCONV_MSK	0X0F
#define BME680_FILTER_MSK	0X1C
#define BME680_OST_MSK		0XE0
#define BME680_OSP_MSK		0X1C
#define BME680_OSH_MSK		0X07
#define BME680_HCTRL_MSK	0x08
#define BME680_RUN_GAS_MSK	0x10
#define BME680_MODE_MSK		0x03
#define BME680_RHRANGE_MSK	0x30
#define BME680_RSERROR_MSK	0xf0
#define BME680_NEW_DATA_MSK	0x80
#define BME680_GAS_INDEX_MSK	0x0f
#define BME680_GAS_RANGE_MSK	0x0f
#define BME680_GASM_VALID_MSK	0x20
#define BME680_HEAT_STAB_MSK	0x10
#define BME680_MEM_PAGE_MSK	0x10
#define BME680_SPI_RD_MSK	0x80
#define BME680_SPI_WR_MSK	0x7f
#define	BME680_BIT_H1_DATA_MSK	0x0F
#define BME680_GAS_MEAS_POS	4
#define BME680_FILTER_POS	2
#define BME680_OST_POS		5
#define BME680_OSP_POS		2
#define BME680_RUN_GAS_POS	4
#define BME680_T2_LSB_REG	(1)
#define BME680_T2_MSB_REG	(2)
#define BME680_T3_REG		(3)
#define BME680_P1_LSB_REG	(5)
#define BME680_P1_MSB_REG	(6)
#define BME680_P2_LSB_REG	(7)
#define BME680_P2_MSB_REG	(8)
#define BME680_P3_REG		(9)
#define BME680_P4_LSB_REG	(11)
#define BME680_P4_MSB_REG	(12)
#define BME680_P5_LSB_REG	(13)
#define BME680_P5_MSB_REG	(14)
#define BME680_P7_REG		(15)
#define BME680_P6_REG		(16)
#define BME680_P8_LSB_REG	(19)
#define BME680_P8_MSB_REG	(20)
#define BME680_P9_LSB_REG	(21)
#define BME680_P9_MSB_REG	(22)
#define BME680_P10_REG		(23)
#define BME680_H2_MSB_REG	(25)
#define BME680_H2_LSB_REG	(26)
#define BME680_H1_LSB_REG	(26)
#define BME680_H1_MSB_REG	(27)
#define BME680_H3_REG		(28)
#define BME680_H4_REG		(29)
#define BME680_H5_REG		(30)
#define BME680_H6_REG		(31)
#define BME680_H7_REG		(32)
#define BME680_T1_LSB_REG	(33)
#define BME680_T1_MSB_REG	(34)
#define BME680_GH2_LSB_REG	(35)
#define BME680_GH2_MSB_REG	(36)
#define BME680_GH1_REG		(37)
#define BME680_GH3_REG		(38)
#define BME680_REG_FILTER_INDEX		(5)
#define BME680_REG_TEMP_INDEX		(4)
#define BME680_REG_PRES_INDEX		(4)
#define BME680_REG_HUM_INDEX		(2)
#define BME680_REG_NBCONV_INDEX		(1)
#define BME680_REG_RUN_GAS_INDEX	(1)
#define BME680_REG_HCTRL_INDEX		(0)
#define BME680_CONCAT_BYTES(msb, lsb)	(((uint16_t)msb << 8) | (uint16_t)lsb)
#define BME680_SET_BITS(reg_data, bitname, data) ((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) & bitname##_MSK))
#define BME680_GET_BITS(reg_data, bitname)	((reg_data & (bitname##_MSK)) >> (bitname##_POS))
#define BME680_SET_BITS_POS_0(reg_data, bitname, data) ((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))
#define BME680_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname##_MSK))
