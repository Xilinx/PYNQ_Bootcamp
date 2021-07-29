#include <pytypes.h>
#include <sys/errno.h>
#include <i2c.h>
#include <stdbool.h>
#include "timer.h"
#include <grove_interfaces.h>
#include <grove_barometer.h>
#include <grove_barometer_hw.h>

#define I2C_ADDRESS 0x77
#define DEVICE_MAX 4

struct grove_barometer_info {
    i2c i2c_dev;
    unsigned char address;
    py_int data;
    py_int count;
};

static py_int coeffs[BPS_COEFFICIENT_COUNT-2];
static py_int coeff_c00, coeff_c10;


typedef enum {
  BPS_OSR_1,
  BPS_OSR_2,
  BPS_OSR_4,
  BPS_OSR_8,
  BPS_OSR_16,
  BPS_OSR_32,
  BPS_OSR_64,
  BPS_OSR_128,
  BPS_OSR_invalid
} bps_osr_t;


typedef enum {
  BPS_MR_1,
  BPS_MR_2,
  BPS_MR_4,
  BPS_MR_8,
  BPS_MR_16,
  BPS_MR_32,
  BPS_MR_64,
  BPS_MR_128,
  BPS_MR_invalid
} bps_mr_t;

typedef enum {
  BPS_MODE_STANDBY,
  BPS_MODE_PSR,
  BPS_MODE_TMP,
  BPS_MODE_INVALID1,
  BPS_MODE_INVALID2,
  BPS_MODE_CONTINUOUS_PSR,
  BPS_MODE_CONTINUOUS_TMP,
  BPS_MODE_CONTINUOUS_PSR_TMP,
  BPS_MODE_INVALID3
} bps_mode_t;



const py_int bps_conversion_time[8] = { 4,
                                            7,
                                            10,
                                            17,
                                            30,
                                            59,
                                            112,
                                            220
};

static py_int oversample_factor[] = {524288, 1572864, 3670016, 7864320,
                                      253952, 516096,  1040384, 2088960};


typedef py_int grove_barometer;


typedef struct {
  py_int psr_oversample_rate;
  py_int tmp_oversample_rate;
  py_int psr_measurement_rate;
  py_int tmp_measurement_rate;
  py_int mode; 
}barometer_config_t;

typedef enum {
  BAROMETER_IDLE,
  BAROMETER_PRESSURE,
  BAROMETER_TEMPERATURE,
  BAROMETER_INVALID1,
  BAROMETER_INVALID2,
  BAROMETER_CONTINUOUS_PSR,
  BAROMETER_CONTINUOUS_TMP,
  BAROMETER_CONTINUOUS_PSR_TMP,
  BAROMETER_INVALID3
}barometer_measurement_t;

static barometer_config_t bps = {BPS_OSR_128, BPS_OSR_128, BPS_MR_128, BPS_MR_128, BPS_MODE_STANDBY};

static struct grove_barometer_info info[DEVICE_MAX];

static py_int bps_write(grove_barometer p, unsigned char addr, unsigned char value) {

    i2c i2c_dev = info[p].i2c_dev;
    unsigned char temp[2];
    temp[0] = addr;
    temp[1] = value;
    if(i2c_write(i2c_dev, info[p].address, temp, 2) != 2) return -EIO;
    else return 0;
}

static py_int bps_read(grove_barometer p, unsigned char addr, unsigned char length, unsigned char data[]) {

    i2c i2c_dev = info[p].i2c_dev;
    if (i2c_write(i2c_dev, info[p].address, &addr, 1) != 1) return -EIO;
    if (i2c_read(i2c_dev, info[p].address, data, length) != length) return -EIO;
    return 0;
}


static py_int bps_decimal_conversion(int raw_coeff, int length)
{
  if (raw_coeff > (py_int)(((py_int)1 << (length - 1)) - 1))
  {
    raw_coeff -= (py_int)1 << length;
  }

  return raw_coeff;
}

static py_int bps_read_coeffs(grove_barometer p)
{
  unsigned char status=0, i;
  unsigned char coeff_buffer[BPS_COEFFICIENT_SIZE];
  int coeff_temp_16;
  int coeff_temp_32;

  while (!(status & (1<<7))){
    if(bps_read(p, BPS_REG_MEASCFG, 1, &status)) return -EIO;
  }

  for(i=0; i<BPS_COEFFICIENT_SIZE; i++)
  {
    if(bps_read(p, BPS_REG_COEFF_BASE+i, 1, coeff_buffer+i)) return -EIO;
  }


  coeff_temp_16 = ((py_int)coeff_buffer[0] << 4) | (((py_int)coeff_buffer[1] >> 4) & 0x0F);
  coeffs[0] = (py_int)bps_decimal_conversion(coeff_temp_16, 12);

  coeff_temp_16 = (((py_int)coeff_buffer[1] & 0x0F) << 8) | coeff_buffer[2];
  coeffs[1] = (py_int)bps_decimal_conversion(coeff_temp_16, 12);

  coeff_temp_32 = ((py_int)coeff_buffer[3] << 12) | ((py_int)coeff_buffer[4] << 4) |
          (((py_int)coeff_buffer[5] >> 4) & 0x0F);
  coeff_c00 = bps_decimal_conversion(coeff_temp_32, 20);

  coeff_temp_32 = (((py_int)coeff_buffer[5] & 0x0F) << 16) | ((py_int)coeff_buffer[6] << 8) |
          (py_int)coeff_buffer[7];
  coeff_c10 = bps_decimal_conversion(coeff_temp_32, 20);

  for(i=0; i<BPS_COEFFICIENT_COUNT-4; i++)
  {
    coeff_temp_16 = ((py_int)coeff_buffer[8+i*2] << 8) | (py_int)coeff_buffer[9+i*2];
    coeffs[i+2] = (py_int)bps_decimal_conversion(coeff_temp_16, 16);
  }

  return 0;
}

static py_int bps_present(grove_barometer p)
{
  unsigned char id=0;
  while(!(id & 0xC0))
  {
      if(bps_read(p, BPS_REG_MEASCFG, 1, &id)) return id;
      delay_us(500);
  }
  return 0;
}

static py_void barometer_calculate(int adc_temp, int adc_press, float *temperature, float *pressure)
{
  float temp_scaled, press_scaled;
  temp_scaled = (py_float) bps_decimal_conversion(adc_temp, 24);
  temp_scaled = temp_scaled / oversample_factor[bps.tmp_oversample_rate];

  press_scaled = (py_float) bps_decimal_conversion(adc_press, 24);
  press_scaled = press_scaled / oversample_factor[bps.psr_oversample_rate];

  *temperature = coeffs[0] / 2.0 + coeffs[1] * temp_scaled;

  *pressure = coeff_c00 + press_scaled * (coeff_c10 + press_scaled * ((py_int)coeffs[4] + press_scaled * (py_int)coeffs[6])) +
      temp_scaled * (py_int)coeffs[2] +  temp_scaled * press_scaled * ((py_int)coeffs[3] + press_scaled * (py_int)coeffs[5]);
  *pressure /= 100;
  return PY_SUCCESS;
}

py_float grove_barometer_calculate_pressure(grove_barometer p, int raw_temp, int raw_press) {
  float press, temp;
  barometer_calculate(raw_temp, raw_press, &temp, &press);
  return press;
}

py_float grove_barometer_calculate_temperature(grove_barometer p, int raw_temp, int raw_press) {
  float press, temp;
  barometer_calculate(raw_temp, raw_press, &temp, &press);
  return temp;
}

static py_int grove_barometer_next_index() {
    for (py_int i = 0; i < DEVICE_MAX; ++i) {
        if (info[i].count == 0) return i;
    }
    return -1;
}

grove_barometer grove_barometer_open_at_address(py_int grove_id, py_int address) {
    grove_barometer dev_id = grove_barometer_next_index();
    info[dev_id].count++;
    info[dev_id].i2c_dev = i2c_open_grove(grove_id);
    info[dev_id].address = address;
    info[dev_id].data = 0; // Static data initialisation
    return dev_id;
}

grove_barometer grove_barometer_open(py_int grove_id) {
    return grove_barometer_open_at_address(grove_id, I2C_ADDRESS);
}


void grove_barometer_close(grove_barometer p) {
    if (--info[p].count != 0) return;
    i2c i2c_dev = info[p].i2c_dev;
    i2c_close(i2c_dev);
}

py_void grove_barometer_pressure_oversample_rate(grove_barometer p, int value) {
    bps.psr_oversample_rate = value;
    return PY_SUCCESS;
}

py_void grove_barometer_temperature_oversample_rate(grove_barometer p, int value) {
    bps.tmp_oversample_rate= value;
    return PY_SUCCESS;
}

py_void grove_barometer_pressure_measurement_rate(grove_barometer p, int value) {
    bps.psr_measurement_rate = value;
    return PY_SUCCESS;
}

py_void grove_barometer_temperature_measurement_rate(grove_barometer p, int value) {
    bps.tmp_measurement_rate = value;
    return PY_SUCCESS;
}

py_void grove_barometer_mode(grove_barometer p, py_int value){
    bps.mode = value;
    return PY_SUCCESS;
}

py_int grove_barometer_fifo_empty(grove_barometer p){
  unsigned char reg_value = 0;
  if(bps_read(p, BPS_REG_FIFO_STS, 1, &reg_value)) return -EIO;
  return reg_value & 0x1;
}

py_int grove_barometer_fifo_full(grove_barometer p){
  unsigned char reg_value = 0;
  if(bps_read(p, BPS_REG_FIFO_STS, 1, &reg_value)) return -EIO;
  return reg_value & 0x2;
}

py_int grove_barometer_enable_fifo(grove_barometer p, int value){
    unsigned char reg_value = 0;
    if(bps_read(p, BPS_REG_CFGREG, 1, &reg_value)) return -EIO;
    if(bps_write(p, BPS_REG_CFGREG, reg_value | ((value << 1) & 0x02))) return -EIO;
    return 0;
}

py_int grove_barometer_reset(grove_barometer p)
{
  return bps_write(p, BPS_REG_RESET, BPS_CMD_RESET);
}

py_int grove_barometer_status(grove_barometer p)
{
    return grove_barometer_reg_read(p, 0x08);
}

py_int grove_barometer_reg_read(grove_barometer p, unsigned char addr)
{
  unsigned char val=0;
  if(bps_read(p, addr, 1, &val)) return -EIO;
  return val;
}

py_int grove_barometer_reg_write(grove_barometer p, unsigned char addr, unsigned char val)
{
  if(bps_write(p, addr, val)) return -EIO;
  return 0;
}


py_int grove_barometer_start_conversion(grove_barometer p, int measurement_type)
{
 unsigned char reg_value;
 if(bps_read(p, BPS_REG_MEASCFG, 1, &reg_value)) return -EIO;

 return bps_write(p, BPS_REG_MEASCFG, reg_value | measurement_type);

  return 0;
}

py_int grove_barometer_pressure_raw(grove_barometer p)
{
  unsigned char i, result_buff[3];
  unsigned int result=0;

  if(bps_read(p, BPS_REG_MEASCFG, 1, &i)) return -EIO;

  if(i & 0x10){
    for(i=0; i<3; i++)
      {
        if(bps_read(p, BPS_REG_PRS_BASE+i, 1, result_buff+i)) return -EIO;
      }
      result = ((unsigned py_int)result_buff[0]<<16) | ((unsigned py_int)result_buff[1]<<8) | result_buff[2];
      return result;
  }

  return -ENODATA;
}

py_int grove_barometer_temperature_raw(grove_barometer p)
{
  unsigned char i, result_buff[3];
  int result=0;

  if(bps_read(p, BPS_REG_MEASCFG, 1, &i)) return 1;

  if(i & 0x20){
    for(i=0; i<3; i++)
      {
        if(bps_read(p, BPS_REG_TMP_BASE+i, 1, result_buff+i)) return 2;
      }
      result = ((py_int)result_buff[0]<<16) | ((py_int)result_buff[1]<<8) | result_buff[2];
      return result;
  }

  return -ENODATA;
}

py_float grove_barometer_temperature(grove_barometer p)
{
  int raw_temp, raw_press;
  float press, temp;

  grove_barometer_start_conversion(p, BAROMETER_TEMPERATURE);
  delay_us(bps_conversion_time[bps.tmp_oversample_rate]*1000);
  raw_temp = grove_barometer_temperature_raw(p);

  grove_barometer_start_conversion(p, BAROMETER_PRESSURE);
  delay_us(bps_conversion_time[bps.psr_oversample_rate]*1000);
  raw_press = grove_barometer_pressure_raw(p);

  barometer_calculate(raw_temp, raw_press, &temp, &press);
  return temp;
}

py_float grove_barometer_pressure(grove_barometer p)
{
  int raw_temp, raw_press;
  float press, temp;

  grove_barometer_start_conversion(p, BAROMETER_TEMPERATURE);
  delay_us(bps_conversion_time[bps.tmp_oversample_rate]*1000);
  raw_temp = grove_barometer_temperature_raw(p);

  grove_barometer_start_conversion(p, BAROMETER_PRESSURE);
  delay_us(bps_conversion_time[bps.psr_oversample_rate]*1000);
  raw_press = grove_barometer_pressure_raw(p);

  barometer_calculate(raw_temp, raw_press, &temp, &press);
  return press;
}

py_int grove_barometer_read_fifo(grove_barometer p)
{
  unsigned char result_buff[3];
  unsigned int i, result=0;

  if(grove_barometer_fifo_empty(p)) return -EPERM;

  for(i=0; i<3; i++)
    {
      if(bps_read(p, BPS_REG_PRS_BASE+i, 1, result_buff+i)) return -EIO;
    }
    result = ((unsigned py_int)result_buff[0]<<16) | ((unsigned py_int)result_buff[1]<<8) | result_buff[2];
    return result;

  return -ENODATA;
}

py_int grove_barometer_configure(grove_barometer p)
{
    unsigned char reg_value, shift_value=0;
    int x = 0;
    i2c i2c_dev = info[p].i2c_dev;

    if(grove_barometer_reset(p)) return 1;
    delay_us(5000);
    x = bps_present(p);
    if(x) return x;


    if(bps_read(p, BPS_REG_PRSCFG, 1, &reg_value)) return 2;
    if(bps_write(p, BPS_REG_PRSCFG, reg_value | bps.psr_oversample_rate)) return 3;

    if(bps_read(p, BPS_REG_TMPSRC, 1, &reg_value)) return 4;
    if(reg_value & 0x80) {
      reg_value = 0x80;
    }
    if(bps_write(p, BPS_REG_TMPCFG, reg_value | bps.tmp_oversample_rate)) return 5;


    if(bps_read(p, BPS_REG_CFGREG, 1, &reg_value)) return 6;
    if(bps.tmp_oversample_rate > BPS_OSR_8) {
      shift_value = BPS_CMD_T_SHIFT;
    }
    if(bps_write(p, BPS_REG_CFGREG, reg_value | shift_value)) return 7;

    if(bps_read(p, BPS_REG_CFGREG, 1, &reg_value)) return 8;
    if(bps.psr_oversample_rate > BPS_OSR_8) {
      shift_value = BPS_CMD_P_SHIFT;
    }
    if(bps_write(p, BPS_REG_CFGREG, reg_value | shift_value)) return 9;


    if(bps_read(p, BPS_REG_CFGREG, 1, &reg_value)) return 10;
    if(bps_write(p, BPS_REG_CFGREG, reg_value | bps.mode)) return 11;


    if(bps.mode == BPS_MODE_CONTINUOUS_PSR || bps.mode == BPS_MODE_CONTINUOUS_PSR_TMP) {
        if(bps_read(p, BPS_REG_PRSCFG, 1, &reg_value)) return 15;
        if(bps_write(p, BPS_REG_PRSCFG, reg_value | (bps.psr_measurement_rate << 4))) return 12;
    }


    if(bps.mode == BPS_MODE_CONTINUOUS_TMP || bps.mode == BPS_MODE_CONTINUOUS_PSR_TMP) {
        if(bps_read(p, BPS_REG_TMPCFG, 1, &reg_value)) return 16;
        if(bps_write(p, BPS_REG_TMPCFG, reg_value | (bps.tmp_measurement_rate << 4))) return 13;
    }

    if(bps_read_coeffs(p)) return 14;

    return 0;
}
