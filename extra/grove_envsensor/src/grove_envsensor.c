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
 
#include <stdint.h>
#include <i2c.h>
#include <stdbool.h>
#include "timer.h"
#include <pytypes.h>
#include <sys/errno.h>
#include <pytypes.h>
#include <grove_envsensor_hw.h>
#include <grove_envsensor.h>
#include <grove_interfaces.h>



#define I2C_ADDRESS 0x77
#define DEVICE_MAX 4

struct grove_envsensor_info {
    i2c i2c_dev;
    unsigned char address;
    int data;
    int count;
};



uint32_t gaslookupTable1[16] = { UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
                              UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777), UINT32_C(2147483647),
                              UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228), UINT32_C(2147483647), UINT32_C(2126008810),
                              UINT32_C(2147483647), UINT32_C(2147483647)
                            };
uint32_t gaslookupTable2[16] = { UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
                              UINT32_C(255744255), UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016), UINT32_C(
                                  8000000), UINT32_C(4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000), UINT32_C(250000),
                              UINT32_C(125000)
                            };

typedef py_int grove_envsensor;

typedef struct	bme680_field_data {
   uint8_t status;
   uint8_t gas_index;
   uint8_t meas_index;
   int16_t temperature;
   uint32_t pressure;
   uint32_t humidity;
   uint32_t gas_resistance;
}bme680_field_data_t;

typedef struct	bme680_calib_data {
    uint16_t par_h1;
    uint16_t par_h2;
    int8_t par_h3;
    int8_t par_h4;
    int8_t par_h5;
    uint8_t par_h6;
    int8_t par_h7;
    int8_t par_gh1;
    int16_t par_gh2;
    int8_t par_gh3;
    uint16_t par_t1;
    int16_t par_t2;
    int8_t par_t3;
    uint16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int16_t par_p4;
    int16_t par_p5;
    int8_t par_p6;
    int8_t par_p7;
    int16_t par_p8;
    int16_t par_p9;
    uint8_t par_p10;
    int32_t t_fine;
    uint8_t res_heat_range;
    int8_t res_heat_val;
    int8_t range_sw_err;
} bme680_calib_data_t;

typedef struct	bme680_tph_sett {
    uint8_t os_hum;
    uint8_t os_temp;
    uint8_t os_pres;
    uint8_t filter;
}bme680_tph_sett_t;


typedef struct	bme680_gas_sett {
     uint8_t nb_conv;
    uint8_t heatr_ctrl;
    uint8_t run_gas;
    uint16_t heatr_temp;
    uint16_t heatr_dur;
}bme680_gas_sett_t;

typedef struct	bme680_dev { 
    int8_t amb_temp;                    
    bme680_calib_data_t calib;   
	bme680_tph_sett_t tph_sett; 
    bme680_gas_sett_t gas_sett;      
    uint8_t power_mode;	
	uint8_t new_fields; 
	uint8_t info_msg; 
}bme680_dev_t;

typedef struct Result {
    float temperature;
    float pressure;
    float humidity;
    float gas;
} sensor_result_t;


static struct grove_envsensor_info info[DEVICE_MAX];

static int grove_envsensor_next_index() {
    for (int i = 0; i < DEVICE_MAX; ++i) {
        if (info[i].count == 0) return i;
    }
    return -1;
}

grove_envsensor grove_envsensor_open_at_address(int grove_id, int address) {
    grove_envsensor dev_id = grove_envsensor_next_index();
    info[dev_id].count++;
    info[dev_id].i2c_dev = i2c_open_grove(grove_id);
    info[dev_id].address = address;
    info[dev_id].data = 0; // Static data initialisation
    return dev_id;
}

grove_envsensor grove_envsensor_open(int grove_id) {
    return grove_envsensor_open_at_address(grove_id, I2C_ADDRESS);
}


void grove_envsensor_close(grove_envsensor p) {
    if (--info[p].count != 0) return;
    i2c i2c_dev = info[p].i2c_dev;
    i2c_close(i2c_dev);
}

static int envsensor_write(grove_envsensor p, unsigned char addr, unsigned char value) {

    i2c i2c_dev = info[p].i2c_dev;
    unsigned char temp[2];
    temp[0] = addr;
    temp[1] = value;
    if(i2c_write(i2c_dev, info[p].address, temp, 2) != 2) return -EIO;
    else return 0;
}

static int envsensor_read(grove_envsensor p, unsigned char addr, unsigned char length, unsigned char data[]) {
 
    i2c i2c_dev = info[p].i2c_dev;
    if (i2c_write(i2c_dev, info[p].address, &addr, 1) != 1) return -EIO;
    if (i2c_read(i2c_dev, info[p].address, data, length) != length) return -EIO;
    return 0;
}

static int envsensor_write_len(grove_envsensor p, unsigned char* temp, unsigned char len) {

    i2c i2c_dev = info[p].i2c_dev;
    if(i2c_write(i2c_dev, info[p].address, temp, len) != len) return -EIO;
    else return 0;
}


py_int grove_envsensor_reg_read(grove_envsensor p, unsigned char addr)
{
  unsigned char val;
  if(envsensor_read(p, addr, 1, &val)) return -EIO;
  return val;
}

py_int grove_envsensor_reg_write(grove_envsensor p, unsigned char addr, unsigned char val)
{
  if(envsensor_write(p, addr, val)) return -EIO;
  return 0;
}


static int bme680_get_regs(grove_envsensor p, unsigned char reg_addr, unsigned char* reg_data, unsigned char len) {
    int8_t rslt = 0;
	rslt = envsensor_read(p, reg_addr, len, reg_data);
	if (rslt != 0) {
		rslt = BME680_E_COM_FAIL;
	}
    return rslt;
}


static int bme680_set_regs(grove_envsensor p,const unsigned char* reg_addr, const unsigned char* reg_data, unsigned char len) {
    int8_t rslt;
    unsigned char tmplen = len;
	while(tmplen--) {
        rslt = envsensor_write(p,*reg_addr++,*reg_data++);
        if(rslt)
            return -EIO;
    }
    return rslt;
}


static int bme680_readChipID(grove_envsensor p, unsigned char* result) {
	return bme680_get_regs(p,0xD0,result,1);
	
}

static int bme680_readfield(grove_envsensor p, unsigned char* result) {
	return bme680_get_regs(p,BME680_FIELD0_ADDR,result,1);
}

static int get_calib_data(grove_envsensor p,bme680_dev_t* dev) {
    int8_t rslt;
    unsigned char coeff_array[BME680_COEFF_SIZE] = { 0 };
    unsigned char temp_var = 0; 
	rslt = bme680_get_regs(p,BME680_COEFF_ADDR1, coeff_array, BME680_COEFF_ADDR1_LEN);
	if (rslt == BME680_OK)
		rslt = bme680_get_regs(p,BME680_COEFF_ADDR2, &coeff_array[BME680_COEFF_ADDR1_LEN] , BME680_COEFF_ADDR2_LEN);
	dev->calib.par_t1 = (uint16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_T1_MSB_REG],coeff_array[BME680_T1_LSB_REG]));
	dev->calib.par_t2 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_T2_MSB_REG], coeff_array[BME680_T2_LSB_REG]));
	dev->calib.par_t3 = (int8_t)(coeff_array[BME680_T3_REG]);
	dev->calib.par_p1 = (uint16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_P1_MSB_REG], coeff_array[BME680_P1_LSB_REG]));
	dev->calib.par_p2 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_P2_MSB_REG],coeff_array[BME680_P2_LSB_REG]));
	dev->calib.par_p3 = (int8_t) coeff_array[BME680_P3_REG];
	dev->calib.par_p4 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_P4_MSB_REG], coeff_array[BME680_P4_LSB_REG]));
	dev->calib.par_p5 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_P5_MSB_REG],coeff_array[BME680_P5_LSB_REG]));
	dev->calib.par_p6 = (int8_t)(coeff_array[BME680_P6_REG]);
	dev->calib.par_p7 = (int8_t)(coeff_array[BME680_P7_REG]);
	dev->calib.par_p8 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_P8_MSB_REG],coeff_array[BME680_P8_LSB_REG]));
	dev->calib.par_p9 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_P9_MSB_REG],coeff_array[BME680_P9_LSB_REG]));
	dev->calib.par_p10 = (uint8_t)(coeff_array[BME680_P10_REG]);
	dev->calib.par_h1 = (uint16_t)(((uint16_t) coeff_array[BME680_H1_MSB_REG] << BME680_HUM_REG_SHIFT_VAL) | (coeff_array[BME680_H1_LSB_REG] & BME680_BIT_H1_DATA_MSK));
	dev->calib.par_h2 = (uint16_t)(((uint16_t) coeff_array[BME680_H2_MSB_REG] << BME680_HUM_REG_SHIFT_VAL) | ((coeff_array[BME680_H2_LSB_REG]) >> BME680_HUM_REG_SHIFT_VAL));
	dev->calib.par_h3 = (int8_t) coeff_array[BME680_H3_REG];
	dev->calib.par_h4 = (int8_t) coeff_array[BME680_H4_REG];
	dev->calib.par_h5 = (int8_t) coeff_array[BME680_H5_REG];
	dev->calib.par_h6 = (uint8_t) coeff_array[BME680_H6_REG];
	dev->calib.par_h7 = (int8_t) coeff_array[BME680_H7_REG];
	dev->calib.par_gh1 = (int8_t) coeff_array[BME680_GH1_REG];
	dev->calib.par_gh2 = (int16_t)(BME680_CONCAT_BYTES(coeff_array[BME680_GH2_MSB_REG], coeff_array[BME680_GH2_LSB_REG]));
	dev->calib.par_gh3 = (int8_t) coeff_array[BME680_GH3_REG];
	if (rslt == BME680_OK) {
		rslt = bme680_get_regs(p,BME680_ADDR_RES_HEAT_RANGE_ADDR, &temp_var, 1);
		dev->calib.res_heat_range = ((temp_var & BME680_RHRANGE_MSK) / 16);
		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(p,BME680_ADDR_RES_HEAT_VAL_ADDR, &temp_var, 1);
			dev->calib.res_heat_val = (int8_t) temp_var;
			if (rslt == BME680_OK) {
				rslt = bme680_get_regs(p,BME680_ADDR_RANGE_SW_ERR_ADDR, &temp_var, 1);
			}
		}
	}
	dev->calib.range_sw_err = ((int8_t) temp_var & (int8_t) BME680_RSERROR_MSK) / 16;
    return rslt;
}

static int bme680_soft_reset(grove_envsensor p) {
    int8_t rslt;
    unsigned char reg_addr = BME680_SOFT_RESET_ADDR;
    unsigned char soft_rst_cmd = BME680_SOFT_RESET_CMD;

	rslt = bme680_set_regs(p, &reg_addr, &soft_rst_cmd, 1);
	delay_ms(BME680_RESET_PERIOD);

    return rslt;
}

static int bme680_init(grove_envsensor p, bme680_dev_t* dev) {
    int8_t rslt = BME680_OK;
	unsigned char chip_id;
	rslt = bme680_soft_reset(p);
	if (rslt == BME680_OK) {
		rslt = bme680_readChipID(p, &chip_id);
		if (rslt == BME680_OK) {
			if (chip_id == BME680_CHIP_ID) {
				rslt = get_calib_data(p,dev);
			} else {
				rslt = BME680_E_DEV_NOT_FOUND;
			}
		}
	}
    return rslt;
}

static uint8_t calc_heater_dur(uint16_t dur) {
    uint8_t factor = 0;
    uint8_t durval;
    if (dur >= 0xfc0) {
        durval = 0xff; 
    } else {
        while (dur > 0x3F) {
            dur = dur / 4;
            factor += 1;
        }
        durval = (uint8_t)(dur + (factor * 64));
    }
    return durval;
}

static uint8_t calc_heater_res(uint16_t temp, const  bme680_dev_t* dev) {
    uint8_t heatr_res;
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;
    if (temp < 200) { 
        temp = 200;
    } else if (temp > 400) {
        temp = 400;
    }

    var1 = (((int32_t) dev->amb_temp * dev->calib.par_gh3) / 1000) * 256;
    var2 = (dev->calib.par_gh1 + 784) * (((((dev->calib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (dev->calib.res_heat_range + 4));
    var5 = (131 * dev->calib.res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);

    return heatr_res;
}

static int8_t set_gas_config(grove_envsensor p, bme680_dev_t* dev) {
    int8_t rslt = BME680_OK;
	unsigned char reg_addr[2] = {0};
	unsigned char reg_data[2] = {0};
	if (dev->power_mode == BME680_FORCED_MODE) {
		reg_addr[0] = BME680_RES_HEAT0_ADDR;
		reg_data[0] = calc_heater_res(dev->gas_sett.heatr_temp, dev);
		reg_addr[1] = BME680_GAS_WAIT0_ADDR;
		reg_data[1] = calc_heater_dur(dev->gas_sett.heatr_dur);
		dev->gas_sett.nb_conv = 0;
	} else {
		rslt = BME680_W_DEFINE_PWR_MODE;
	}
	if (rslt == BME680_OK) {
		rslt = bme680_set_regs(p, reg_addr, reg_data, 2);
	}
    return rslt;
}

static int8_t bme680_set_sensor_mode(grove_envsensor p, bme680_dev_t* dev) {
    int8_t rslt;
    uint8_t tmp_pow_mode;
    uint8_t pow_mode = 0;
    uint8_t reg_addr = BME680_CONF_T_P_MODE_ADDR;
	do {
		rslt = bme680_get_regs(p, BME680_CONF_T_P_MODE_ADDR, &tmp_pow_mode, 1);
		if (rslt == BME680_OK) {
			pow_mode = (tmp_pow_mode & BME680_MODE_MSK);

			if (pow_mode != BME680_SLEEP_MODE) {
				tmp_pow_mode = tmp_pow_mode & (~BME680_MODE_MSK);
				rslt = bme680_set_regs(p, &reg_addr, &tmp_pow_mode, 1);
				delay_ms(BME680_POLL_PERIOD_MS);
			}
		}
	} while (pow_mode != BME680_SLEEP_MODE);
	if (dev->power_mode != BME680_SLEEP_MODE) {
		tmp_pow_mode = (tmp_pow_mode & ~BME680_MODE_MSK) | (dev->power_mode & BME680_MODE_MSK);
		if (rslt == BME680_OK) {
			rslt = bme680_set_regs(p, &reg_addr, &tmp_pow_mode, 1);
		}
	}
    

    return rslt;
}

static int8_t boundary_check(uint8_t* value, uint8_t min, uint8_t max,  bme680_dev_t* dev) {
    int8_t rslt = BME680_OK;
    if (value != NULL) {
        if (*value < min) {
            *value = min;
            dev->info_msg |= BME680_I_MIN_CORRECTION;
        }
        if (*value > max) {
            *value = max;
            dev->info_msg |= BME680_I_MAX_CORRECTION;
        }
    } else {
        rslt = BME680_E_NULL_PTR;
    }

    return rslt;
}

static int8_t bme680_set_sensor_settings(grove_envsensor p, uint16_t desired_settings,  bme680_dev_t* dev) {
    int8_t rslt;
    unsigned char reg_addr;
    unsigned char data = 0;
    unsigned char count = 0;
    unsigned char reg_array[BME680_REG_BUFFER_LENGTH] = { 0 };
    unsigned char data_array[BME680_REG_BUFFER_LENGTH] = { 0 };
    uint8_t intended_power_mode = dev->power_mode; 
	if (desired_settings & BME680_GAS_MEAS_SEL) {
		rslt = set_gas_config(p,dev);
        if(rslt)
            return rslt;
	}

	dev->power_mode = BME680_SLEEP_MODE;
	if (rslt == BME680_OK) {
		rslt = bme680_set_sensor_mode(p,dev);
        if(rslt)
            return rslt;
	}

	if (desired_settings & BME680_FILTER_SEL) {
		rslt = boundary_check(&dev->tph_sett.filter, BME680_FILTER_SIZE_0, BME680_FILTER_SIZE_127, dev);
		reg_addr = BME680_CONF_ODR_FILT_ADDR;

		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(p, reg_addr, &data, 1);
            if(rslt)
                return rslt;
		}

		if (desired_settings & BME680_FILTER_SEL) {
			data = BME680_SET_BITS(data, BME680_FILTER, dev->tph_sett.filter);
		}

		reg_array[count] = reg_addr; 
		data_array[count] = data;
		count++;
	}

	if (desired_settings & BME680_HCNTRL_SEL) {
		rslt = boundary_check(&dev->gas_sett.heatr_ctrl, BME680_ENABLE_HEATER, BME680_DISABLE_HEATER, dev);
		reg_addr = BME680_CONF_HEAT_CTRL_ADDR;
		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(p, reg_addr, &data, 1);
            if(rslt)
                return rslt;
		}
		data = BME680_SET_BITS_POS_0(data, BME680_HCTRL, dev->gas_sett.heatr_ctrl);
		reg_array[count] = reg_addr; 
		data_array[count] = data;
		count++;
	}

	if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL)) {
		rslt = boundary_check(&dev->tph_sett.os_temp, BME680_OS_NONE, BME680_OS_16X, dev);
		reg_addr = BME680_CONF_T_P_MODE_ADDR;

		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(p, reg_addr, &data, 1);
            if(rslt)
                return rslt;
		}

		if (desired_settings & BME680_OST_SEL) {
			data = BME680_SET_BITS(data, BME680_OST, dev->tph_sett.os_temp);
		}

		if (desired_settings & BME680_OSP_SEL) {
			data = BME680_SET_BITS(data, BME680_OSP, dev->tph_sett.os_pres);
		}

		reg_array[count] = reg_addr;
		data_array[count] = data;
		count++;
	}

	if (desired_settings & BME680_OSH_SEL) {
		rslt = boundary_check(&dev->tph_sett.os_hum, BME680_OS_NONE, BME680_OS_16X, dev);
		reg_addr = BME680_CONF_OS_H_ADDR;

		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(p, reg_addr, &data, 1);
            if(rslt)
                return rslt;
		}
		data = BME680_SET_BITS_POS_0(data, BME680_OSH, dev->tph_sett.os_hum);

		reg_array[count] = reg_addr;
		data_array[count] = data;
		count++;
	}

	if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)) {
		rslt = boundary_check(&dev->gas_sett.run_gas, BME680_RUN_GAS_DISABLE,
							  BME680_RUN_GAS_ENABLE, dev);
		if (rslt == BME680_OK) {
			rslt = boundary_check(&dev->gas_sett.nb_conv, BME680_NBCONV_MIN,
								  BME680_NBCONV_MAX, dev);
		}

		reg_addr = BME680_CONF_ODR_RUN_GAS_NBC_ADDR;

		if (rslt == BME680_OK) {
			rslt = bme680_get_regs(p, reg_addr, &data, 1);
            if(rslt)
                return rslt;
		}

		if (desired_settings & BME680_RUN_GAS_SEL) {
			data = BME680_SET_BITS(data, BME680_RUN_GAS, dev->gas_sett.run_gas);
		}

		if (desired_settings & BME680_NBCONV_SEL) {
			data = BME680_SET_BITS_POS_0(data, BME680_NBCONV, dev->gas_sett.nb_conv);
		}

		reg_array[count] = reg_addr;
		data_array[count] = data;
		count++;
	}

	if (rslt == BME680_OK) {
		rslt = bme680_set_regs(p, reg_array, data_array, count);
	}
	dev->power_mode = intended_power_mode;

    return rslt;
}

static int8_t get_gas_config(grove_envsensor p, bme680_dev_t* dev) {
    int8_t rslt;
    unsigned char reg_addr1 = BME680_ADDR_SENS_CONF_START;
    unsigned char reg_addr2 = BME680_ADDR_GAS_CONF_START;
    unsigned char data_array[BME680_GAS_HEATER_PROF_LEN_MAX] = { 0 };
    uint8_t index;
	rslt = bme680_get_regs(p, reg_addr1, data_array, BME680_GAS_HEATER_PROF_LEN_MAX);
	if (rslt == BME680_OK) {
		for (index = 0; index < BME680_GAS_HEATER_PROF_LEN_MAX; index++) {
			dev->gas_sett.heatr_temp = data_array[index];
		}
	}

	rslt = bme680_get_regs(p, reg_addr2, data_array, BME680_GAS_HEATER_PROF_LEN_MAX);
	if (rslt == BME680_OK) {
		for (index = 0; index < BME680_GAS_HEATER_PROF_LEN_MAX; index++) {
			dev->gas_sett.heatr_dur = data_array[index];
		}
	}
    
    return rslt;
}

static int8_t bme680_get_sensor_settings(grove_envsensor p, uint16_t desired_settings,  bme680_dev_t* dev) {
    int8_t rslt;
    unsigned char reg_addr = BME680_CONF_HEAT_CTRL_ADDR;
    unsigned char data_array[BME680_REG_BUFFER_LENGTH] = { 0 };
	rslt = bme680_get_regs(p, reg_addr, data_array, BME680_REG_BUFFER_LENGTH);

	if (rslt == BME680_OK) {
		if (desired_settings & BME680_GAS_MEAS_SEL) {
			rslt = get_gas_config(p, dev);
		}
		if (desired_settings & BME680_FILTER_SEL)
			dev->tph_sett.filter = BME680_GET_BITS(data_array[BME680_REG_FILTER_INDEX],BME680_FILTER);

		if (desired_settings & (BME680_OST_SEL | BME680_OSP_SEL)) {
			dev->tph_sett.os_temp = BME680_GET_BITS(data_array[BME680_REG_TEMP_INDEX], BME680_OST);
			dev->tph_sett.os_pres = BME680_GET_BITS(data_array[BME680_REG_PRES_INDEX], BME680_OSP);
		}

		if (desired_settings & BME680_OSH_SEL)
			dev->tph_sett.os_hum = BME680_GET_BITS_POS_0(data_array[BME680_REG_HUM_INDEX], BME680_OSH);

		if (desired_settings & BME680_HCNTRL_SEL)
			dev->gas_sett.heatr_ctrl = BME680_GET_BITS_POS_0(data_array[BME680_REG_HCTRL_INDEX], BME680_HCTRL);

		if (desired_settings & (BME680_RUN_GAS_SEL | BME680_NBCONV_SEL)) {
			dev->gas_sett.nb_conv = BME680_GET_BITS_POS_0(data_array[BME680_REG_NBCONV_INDEX], BME680_NBCONV);
			dev->gas_sett.run_gas = BME680_GET_BITS(data_array[BME680_REG_RUN_GAS_INDEX], BME680_RUN_GAS);
		}
	}

    return rslt;
}

static int8_t bme680_get_sensor_mode(grove_envsensor p, bme680_dev_t* dev) {
    int8_t rslt;
    unsigned char mode;
	rslt = bme680_get_regs(p, BME680_CONF_T_P_MODE_ADDR, &mode, 1);
	dev->power_mode = mode & BME680_MODE_MSK;
    return rslt;
}

void bme680_set_profile_dur(uint16_t duration,  bme680_dev_t* dev) {
    uint32_t tph_dur; 
    uint32_t meas_cycles;
    uint8_t os_to_meas_cycles[6] = {0, 1, 2, 4, 8, 16};
    meas_cycles = os_to_meas_cycles[dev->tph_sett.os_temp];
    meas_cycles += os_to_meas_cycles[dev->tph_sett.os_pres];
    meas_cycles += os_to_meas_cycles[dev->tph_sett.os_hum];
    tph_dur = meas_cycles * (1963);
    tph_dur += (477 * 4); 
    tph_dur += (477 * 5); 
    tph_dur += (500);
    tph_dur /= (1000); 
    tph_dur += (1); 
    dev->gas_sett.heatr_dur = duration - (uint16_t) tph_dur;
}

void bme680_get_profile_dur(uint16_t* duration, const bme680_dev_t* dev) {
    uint32_t tph_dur; 
    uint32_t meas_cycles;
    uint8_t os_to_meas_cycles[6] = {0, 1, 2, 4, 8, 16};

    meas_cycles = os_to_meas_cycles[dev->tph_sett.os_temp];
    meas_cycles += os_to_meas_cycles[dev->tph_sett.os_pres];
    meas_cycles += os_to_meas_cycles[dev->tph_sett.os_hum];
    tph_dur = meas_cycles * (1963);
    tph_dur += (477 * 4); 
    tph_dur += (477 * 5); 
    tph_dur += (500); 
    tph_dur /= (1000); 
    tph_dur += (1); 

    *duration = (uint16_t) tph_dur;

    if (dev->gas_sett.run_gas) {
        *duration += dev->gas_sett.heatr_dur;
    }
}

static int16_t calc_temperature(uint32_t temp_adc,  bme680_dev_t* dev) {
    int64_t var1;
    int64_t var2;
    int64_t var3;
    int16_t calc_temp;

    var1 = ((int32_t) temp_adc >> 3) - ((int32_t) dev->calib.par_t1 << 1);
    var2 = (var1 * (int32_t) dev->calib.par_t2) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t) dev->calib.par_t3 << 4)) >> 14;
    dev->calib.t_fine = (int32_t)(var2 + var3);
    calc_temp = (int16_t)(((dev->calib.t_fine * 5) + 128) >> 8);

    return calc_temp;
}

static uint32_t calc_pressure(uint32_t pres_adc, const  bme680_dev_t* dev) {
    int32_t var1 = 0;
    int32_t var2 = 0;
    int32_t var3 = 0;
    int32_t var4 = 0;
    int32_t pressure_comp = 0;

    var1 = (((int32_t)dev->calib.t_fine) >> 1) - 64000;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
            (int32_t)dev->calib.par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)dev->calib.par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)dev->calib.par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
             ((int32_t)dev->calib.par_p3 << 5)) >> 3) +
           (((int32_t)dev->calib.par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)dev->calib.par_p1) >> 15;
    pressure_comp = 1048576 - pres_adc;
    pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
    var4 = (1 << 31);
    if (pressure_comp >= var4) {
        pressure_comp = ((pressure_comp / (uint32_t)var1) << 1);
    } else {
        pressure_comp = ((pressure_comp << 1) / (uint32_t)var1);
    }
    var1 = ((int32_t)dev->calib.par_p9 * (int32_t)(((pressure_comp >> 3) *
            (pressure_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(pressure_comp >> 2) *
            (int32_t)dev->calib.par_p8) >> 13;
    var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
            (int32_t)(pressure_comp >> 8) *
            (int32_t)dev->calib.par_p10) >> 17;

    pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
                    ((int32_t)dev->calib.par_p7 << 7)) >> 4);

    return (uint32_t)pressure_comp;
}

static uint32_t calc_humidity(uint16_t hum_adc, const  bme680_dev_t* dev) {
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t var6;
    int32_t temp_scaled;
    int32_t calc_hum;

    temp_scaled = (((int32_t) dev->calib.t_fine * 5) + 128) >> 8;
    var1 = (int32_t)(hum_adc - ((int32_t)((int32_t) dev->calib.par_h1 * 16)))
           - (((temp_scaled * (int32_t) dev->calib.par_h3) / ((int32_t) 100)) >> 1);
    var2 = ((int32_t) dev->calib.par_h2
            * (((temp_scaled * (int32_t) dev->calib.par_h4) / ((int32_t) 100))
               + (((temp_scaled * ((temp_scaled * (int32_t) dev->calib.par_h5) / ((int32_t) 100))) >> 6)
                  / ((int32_t) 100)) + (int32_t)(1 << 14))) >> 10;
    var3 = var1 * var2;
    var4 = (int32_t) dev->calib.par_h6 << 7;
    var4 = ((var4) + ((temp_scaled * (int32_t) dev->calib.par_h7) / ((int32_t) 100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

    if (calc_hum > 100000) { /* Cap at 100%rH */
        calc_hum = 100000;
    } else if (calc_hum < 0) {
        calc_hum = 0;
    }

    return (uint32_t) calc_hum;
}

static uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const  bme680_dev_t* dev) {
    int64_t var1;
    uint64_t var2;
    int64_t var3;
    uint32_t calc_gas_res;

    var1 = (int64_t)((1340 + (5 * (int64_t) dev->calib.range_sw_err)) *
                     ((int64_t) gaslookupTable1[gas_range])) >> 16;
    var2 = (((int64_t)((int64_t) gas_res_adc << 15) - (int64_t)(16777216)) + var1);

    var3 = (((int64_t) gaslookupTable2[gas_range] * (int64_t) var1) >> 9);
    calc_gas_res = (uint32_t)((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);

    return calc_gas_res;
}

static int8_t read_field_data(grove_envsensor p, bme680_field_data_t* data,  bme680_dev_t* dev) {
    int8_t rslt = BME680_OK;
    unsigned char buff[BME680_FIELD_LENGTH] = { 0 };
    uint8_t gas_range;
    uint32_t adc_temp;
    uint32_t adc_pres;
    uint16_t adc_hum;
    uint16_t adc_gas_res;
    uint8_t tries = 100;

    do {
        if (rslt == BME680_OK) {
            rslt = bme680_get_regs(p,((uint8_t)(BME680_FIELD0_ADDR)), buff, (uint16_t) BME680_FIELD_LENGTH);

            data->status = buff[0] & BME680_NEW_DATA_MSK;
            data->gas_index = buff[0] & BME680_GAS_INDEX_MSK;
            data->meas_index = buff[1];

            adc_pres = (uint32_t)(((uint32_t) buff[2] * 4096) | ((uint32_t) buff[3] * 16)
                                  | ((uint32_t) buff[4] / 16));
            adc_temp = (uint32_t)(((uint32_t) buff[5] * 4096) | ((uint32_t) buff[6] * 16)
                                  | ((uint32_t) buff[7] / 16));
            adc_hum = (uint16_t)(((uint32_t) buff[8] * 256) | (uint32_t) buff[9]);
            adc_gas_res = (uint16_t)((uint32_t) buff[13] * 4 | (((uint32_t) buff[14]) / 64));
            gas_range = buff[14] & BME680_GAS_RANGE_MSK;

            data->status |= buff[14] & BME680_GASM_VALID_MSK;
            data->status |= buff[14] & BME680_HEAT_STAB_MSK;

            if (data->status & BME680_NEW_DATA_MSK) {
                data->temperature = calc_temperature(adc_temp, dev);
                data->pressure = calc_pressure(adc_pres, dev);
                data->humidity = calc_humidity(adc_hum, dev);
                data->gas_resistance = calc_gas_resistance(adc_gas_res, gas_range, dev);

                break;
            }
            delay_ms(BME680_POLL_PERIOD_MS);
        }
        tries--;
    } while (tries);

    if (!tries) {
        rslt = BME680_W_NO_NEW_DATA;
    }

    return rslt;
}

static int8_t bme680_get_sensor_data(grove_envsensor p, bme680_field_data_t* data,  bme680_dev_t* dev) {
    int8_t rslt;
	rslt = read_field_data(p, data, dev);
	if (rslt == BME680_OK) {
		if (data->status & BME680_NEW_DATA_MSK) {
			dev->new_fields = 1;
		} else {
			dev->new_fields = 0;
		}
	}

    return rslt;
}

bme680_dev_t sensor_param;
sensor_result_t sensor_result_value;

py_int grove_envsensor_read_data(grove_envsensor p) {

    struct bme680_field_data data;
    int ret;
    sensor_param.power_mode = BME680_FORCED_MODE;
    uint16_t settings_sel;
    sensor_param.tph_sett.os_hum = BME680_OS_1X;
    sensor_param.tph_sett.os_pres = BME680_OS_16X;
    sensor_param.tph_sett.os_temp = BME680_OS_2X;
    sensor_param.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    // sensor_param.gas_sett.heatr_dur = 150;
    // sensor_param.gas_sett.heatr_temp = 320;
    sensor_param.gas_sett.heatr_dur = 100;
    sensor_param.gas_sett.heatr_temp = 300;
    settings_sel = BME680_OST_SEL | BME680_OSH_SEL | BME680_OSP_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL;

    if ((ret = bme680_set_sensor_settings(p, settings_sel, &sensor_param))) {
        return 1;
    }
    
    if ((ret = bme680_set_sensor_mode(p, &sensor_param))) {
        return 2;
    }

    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &sensor_param);

    delay_us(meas_period); 

    if ((ret = bme680_get_sensor_data(p, &data, &sensor_param))) {
        return 3;
    }

    sensor_result_value.temperature = data.temperature / 100.0;
    sensor_result_value.humidity = data.humidity / 1000.0;
    sensor_result_value.pressure = data.pressure;
    if (data.status & BME680_HEAT_STAB_MSK) {
        sensor_result_value.gas = data.gas_resistance;
    } else {
        sensor_result_value.gas = 0;
    }
    return BME680_OK;
}

py_float grove_envsensor_read_temperature(grove_envsensor p) {
	int ret = 0; 
    if (ret = grove_envsensor_read_data(p)) {
        return ret;
    }
    return sensor_result_value.temperature;
}

py_float grove_envsensor_read_pressure(grove_envsensor p) {
	int ret = 0; 
    if (ret = grove_envsensor_read_data(p)) {
        return ret;
    }
    return sensor_result_value.pressure;
}

py_float grove_envsensor_read_humidity(grove_envsensor p) {
	int ret = 0; 
    if (ret = grove_envsensor_read_data(p)) {
        return ret;
    }
    return sensor_result_value.humidity;
}

py_float grove_envsensor_read_gas(grove_envsensor p) {
	int ret = 0; 
    if (ret = grove_envsensor_read_data(p)) {
        return ret;
    }
    return sensor_result_value.gas;
}

py_int grove_envsensor_init(grove_envsensor p) {
	unsigned char result = 0;
	sensor_param.amb_temp = 25;
    int ret;
    if ((ret = bme680_init(p, &sensor_param))) {
        return false;
    }
    return true;
}
