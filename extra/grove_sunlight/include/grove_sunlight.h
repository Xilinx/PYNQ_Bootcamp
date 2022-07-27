#pragma once

#include <grove_constants.h>
#include <pytypes.h>
#include <stdint.h>

// Device typedef
typedef py_int grove_sunlight;

//defines
#define DEVICE_ADDRESS      0x53
#define RESET_CMD_CTR       0x00
#define RESET_SW            0x01
#define FORCE               0x11
#define PAUSE               0x12
#define START               0x13

#define PART_ID 			0x00
#define REV_ID 				0x01
#define MFR_ID 				0x02
#define INFO_0 				0x03
#define INFO_1 				0x04
#define HOSTIN_3 			0x07
#define HOSTIN_2 			0x08

#define HOSTIN_0 			0x0A
#define COMMAND 			0x0B
#define IRQ_ENABLE 			0x0F
#define RESPONSE_0 			0x11
#define RESPONSE_1 			0x10

#define IRQ_STATUS 			0x12
#define HOSTOUT_0 			0x13
#define HOSTOUT_1 			0x14
#define HOSTOUT_2 			0x15
#define HOSTOUT_3 			0x16
#define HOSTOUT_4 			0x17
#define HOSTOUT_5 			0x18
#define HOSTOUT_6 			0x19
#define HOSTOUT_7 			0x1A
#define HOSTOUT_8 			0x1B
#define HOSTOUT_9 			0x1C
#define HOSTOUT_10 			0x1D
#define HOSTOUT_11 			0x1E
#define HOSTOUT_12 			0x1F
#define HOSTOUT_13 			0x20
#define HOSTOUT_14 			0x21
#define HOSTOUT_15 			0x22
#define HOSTOUT_16 			0x23
#define HOSTOUT_17 			0x24
#define HOSTOUT_18 			0x25
#define HOSTOUT_19 			0x26
#define HOSTOUT_20 			0x27
#define HOSTOUT_21 			0x28
#define HOSTOUT_22 			0x29
#define HOSTOUT_23 			0x2A
#define HOSTOUT_24 			0x2B
#define HOSTOUT_25 			0x2C

#define I2C_ADDR 			0x00
#define CHAN_LIST   		0x01

#define ADCCONFIG_0  		0x02
#define ADCSENS_0  			0x03
#define ADCPOST_0  			0x04
#define MEASCONFIG_0  		0x05

#define ADCCONFIG_1  		0x06
#define ADCPOST_1  			0x08
#define ADCSENS_1  			0x07
#define MEASCONFIG_1  		0x09

#define ADCCONFIG_2  		0x0A
#define ADCSENS_2  			0x0B
#define ADCPOST_2  			0x0C
#define MEASCONFIG_2  		0x0D

#define ADCCONFIG_3  		0x0E
#define ADCSENS_3  			0x0F
#define ADCPOST_3  			0x10
#define MEASCONFIG_3  		0x11

#define ADCCONFIG_4  		0x12
#define ADCSENS_4  			0x13
#define ADCPOST_4  			0x14
#define MEASCONFIG_4  		0x15

#define ADCCONFIG_5  		0x16
#define ADCSENS_5  			0x17
#define ADCPOST_5  			0x18
#define MEASCONFIG_5  		0x19

#define MEASRATE_H  		0x1A
#define MEASRATE_L  		0x1B
#define MEASCOUNT_0  		0x1C
#define MEASCOUNT_1  		0x1D
#define MEASCOUNT_2  		0x1E

#define LED1_A  			0x1F
#define LED1_B  			0x20
#define LED2_A  			0x21
#define LED2_B  			0x22
#define LED3_A  			0x23
#define LED3_B  			0x24

#define THRESHOLD0_H 		0x25
#define THRESHOLD0_L  		0x26
#define THRESHOLD1_H  		0x27
#define THRESHOLD1_L  		0x28
#define THRESHOLD2_H  		0x29
#define THRESHOLD2_L  		0x2A

#define BURST  				0x2B


// Device lifetime functions
grove_sunlight grove_sunlight_open(int grove_id);
void grove_sunlight_close(grove_sunlight p);

// Device functions
//void grove_sunlight_example(grove_sunlight p, const char* buffer, int length);

py_int grove_sunlight_is_device_ready(grove_sunlight p);

py_int grove_sunlight_configure_sunlight_sensor(grove_sunlight p);

py_int grove_sunlight_read_visible_light(grove_sunlight p);

py_int grove_sunlight_read_infrared_light(grove_sunlight p);

py_float grove_sunlight_read_ultravoilet_light(grove_sunlight p);











