#include <grove_interfaces.h>
#include <grove_sunlight.h>
#include <stdint.h>
#include <stdio.h>

#define I2C_ADDRESS 0x53
#define DEVICE_MAX 4


struct info {
    i2c i2c_dev;
    int count;
};

static struct info info[DEVICE_MAX];

static int next_index() {
    for (int i = 0; i < DEVICE_MAX; ++i) {
        if (info[i].count == 0) return i;
    }
    return -ENOMEM;
}

grove_sunlight grove_sunlight_open(int grove_id) {
    //i2c i2c_dev = i2c_open_grove(grove_id);
    
    grove_sunlight dev_id = next_index();
    if (dev_id >= 0) {
        info[dev_id].count++;
        info[dev_id].i2c_dev = i2c_open_grove(grove_id);

    }
    return dev_id;
}

void grove_sunlight_close(grove_sunlight p) {
    i2c i2c_dev = p;
    i2c_close(i2c_dev);
}



static int get_device_id(grove_sunlight p) {
    i2c i2c_dev = info[p].i2c_dev;
    unsigned char buffer;
    buffer = 0x00;
    if (i2c_write(i2c_dev, I2C_ADDRESS, &buffer, 1) != 1) return -EIO;
    if (i2c_read(i2c_dev, I2C_ADDRESS, &buffer, 1) != 1) return -EIO;
    return buffer;
}


/**
 * Writes data over i2c
 */
static void write_data(grove_sunlight p, uint8_t addr, uint8_t *data, size_t len){
    i2c i2c_dev = info[p].i2c_dev;
    unsigned char buffer[len];
    //buffer[0] = addr;
    for(int i = 0; i<=len; i++)
    {
        buffer[i] = *(data+i);
    }
    
    i2c_write(i2c_dev, addr , buffer, len);
    //return buffer[0];
    //Wire.beginTransmission(addr);
    //Wire.write(data, len);
    //Wire.endTransmission();
    
}

/**
 * Reads data from a register over i2c
 */
static int read_register(grove_sunlight p, uint8_t addr, uint8_t reg, int bytesOfData){
    i2c i2c_dev = info[p].i2c_dev;
    int val;
    unsigned char buffer;

    write_data(p, addr, &reg, sizeof(reg));

    buffer = reg;
    val = i2c_read(i2c_dev, addr, &buffer, bytesOfData);
    if(val == bytesOfData)
        return buffer;
    return -1;
    //return val;
    //return buffer;
}


//Sends command to the command register

static void send_command(grove_sunlight p, uint8_t code){
    uint8_t packet[2];
    int r;
    int cmmnd_ctr;
    do {
        cmmnd_ctr = read_register(p, DEVICE_ADDRESS, RESPONSE_0, 1);   
        packet[0] = COMMAND;
        packet[1] = code;  
        write_data(p, DEVICE_ADDRESS, packet, sizeof(packet));
        r = read_register(p, DEVICE_ADDRESS, RESPONSE_0, 1); 
    } while(r > cmmnd_ctr);
    //return cmmnd_ctr;
}


static void param_set(grove_sunlight p, uint8_t loc, uint8_t val){
    uint8_t packet[2];
    int r;
    int cmmnd_ctr;

    do {
        cmmnd_ctr = read_register(p, DEVICE_ADDRESS, RESPONSE_0, 1);
      
        packet[0] = HOSTIN_0;
        packet[1] = val;
        write_data(p, DEVICE_ADDRESS, packet, sizeof(packet));
      
        packet[0] = COMMAND;
        packet[1] = loc | (0B10 << 6);
        write_data(p, DEVICE_ADDRESS, packet, sizeof(packet));
      
        r = read_register(p, DEVICE_ADDRESS, RESPONSE_0, 1); 
    } while(r > cmmnd_ctr); 
}


static void config_channel(grove_sunlight p, uint8_t index, uint8_t *conf){
    int len = sizeof(conf);
  
    if(len != 4 || index < 0 || index > 5)
      return;

    int inc = index * len;
    
    param_set(p, ADCCONFIG_0 + inc, conf[0]);
    param_set(p, ADCSENS_0 + inc, conf[1]);
    param_set(p, ADCPOST_0 + inc, conf[2]);
    param_set(p, MEASCONFIG_0 + inc, conf[3]);   
}


static int get_int_from_bytes(uint8_t *data, size_t len){
    int result = 0;
    int shift = 8 * len;
    
    for(int i = 0; i < len; i++){
        shift -= 8;
        result |= ((data[i] << shift) & (0xFF << shift));
    }
    
    return result;
}

py_int grove_sunlight_is_device_ready(grove_sunlight p) {
    if (get_device_id(p) != 0x51) {
        return 0;
    }
    return 1;
}

py_int grove_sunlight_configure_sunlight_sensor(grove_sunlight p) {
    
    i2c i2c_dev = info[p].i2c_dev;
    unsigned char buffer;
    

    //send_command(p, START);
    param_set(p, CHAN_LIST, 0B000010);

    param_set(p, MEASRATE_H, 0);
    param_set(p, MEASRATE_L, 1);  
    param_set(p, MEASCOUNT_0, 5); 
    param_set(p, MEASCOUNT_1, 10);
    param_set(p, MEASCOUNT_2, 10);
    param_set(p, THRESHOLD0_L, 200);
    param_set(p, THRESHOLD0_H, 0);

    
    buffer = 0B000010;
    if (i2c_write(i2c_dev, I2C_ADDRESS, &buffer, 1) != 1) return -1;
    
    uint8_t conf[4];

    conf[0] = 0B01100001;
    conf[1] = 0B10100010,
    conf[2] = 0B01111000;
    conf[3] = 0B11000000;
    config_channel(p, 1, conf);

    conf[0] = 0B01100001;
    conf[1] = 0B10100011,
    conf[2] = 0B01000000;
    conf[3] = 0B11000000;
    config_channel(p, 3, conf);

    send_command(p, START);
    

//     conf[0] = 0B00000000;
//     conf[1] = 0B00000010,
//     conf[2] = 0B01000000;
//     conf[3] = 0B01000001;
//     config_channel(p, 1, conf);

//     conf[0] = 0B00000000;
//     conf[1] = 0B00000010,
//     conf[2] = 0B01000000;
//     conf[3] = 0B10000001;
//     config_channel(p, 3, conf);
    
//     send_command(p, START);
    
    return 1;
}

py_int grove_sunlight_read_visible_light(grove_sunlight p) {
    
    send_command(p, FORCE);
    uint8_t data[3];
    data[0] = read_register(p, DEVICE_ADDRESS, HOSTOUT_0, 1);
    data[1] = read_register(p, DEVICE_ADDRESS, HOSTOUT_1, 1);
    data[2] = read_register(p, DEVICE_ADDRESS, HOSTOUT_2, 1);
    //return (get_int_from_bytes(data, sizeof(data)));
    //return ((data[2] * 256 + data[1])/(3*85))+65; 
    return (data[2] * 256 + data[1])/3; 
   
}

py_int grove_sunlight_read_infrared_light(grove_sunlight p) {
    
    send_command(p, FORCE);
    uint8_t data[3];
    //data[0] = read_register(p, DEVICE_ADDRESS, HOSTOUT_0, 1);
    data[1] = read_register(p, DEVICE_ADDRESS, HOSTOUT_1, 1);
    data[2] = read_register(p, DEVICE_ADDRESS, HOSTOUT_2, 1);
    //return ((data[2] * 256 + data[1])/(85))+65; 
    return data[2] * 256 + data[1];
   
}

py_float grove_sunlight_read_ultravoilet_light(grove_sunlight p) {
    
    send_command(p, FORCE);
    uint8_t data[3];
    //data[0] = read_register(p, DEVICE_ADDRESS, HOSTOUT_0, 1);
    data[1] = read_register(p, DEVICE_ADDRESS, HOSTOUT_1, 1);
    data[2] = read_register(p, DEVICE_ADDRESS, HOSTOUT_2, 1);
    //return ((data[2] * 256 + data[1])/3)*0.0012;
    //return (((data[2] * 256 + data[1])/(3*85))+65)*0.0012;
    return ((data[2] * 256 + data[1])/3)*0.0012;
   
}








// py_int grove_sunlight_read_visible_light(grove_sunlight p) {
    
//     send_command(p, FORCE);
//     uint8_t data[3];
//     data[0] = read_register(p, DEVICE_ADDRESS, HOSTOUT_0, 1);
//     data[1] = read_register(p, DEVICE_ADDRESS, HOSTOUT_1, 1);
//     send_command(p, PAUSE);
//     return (data[0] * 256 + data[1])/3; 
   
// }

// py_int grove_sunlight_read_infrared_light(grove_sunlight p) {
    
//     send_command(p, FORCE);
//     uint8_t data[3];
//     data[0] = read_register(p, DEVICE_ADDRESS, HOSTOUT_0, 1);
//     data[1] = read_register(p, DEVICE_ADDRESS, HOSTOUT_1, 1);
//     return data[0] * 256 + data[1];
   
// }

// py_float grove_sunlight_read_ultravoilet_light(grove_sunlight p) {
    
//     send_command(p, FORCE);
//     uint8_t data[3];
//     data[0] = read_register(p, DEVICE_ADDRESS, HOSTOUT_0, 1);
//     data[1] = read_register(p, DEVICE_ADDRESS, HOSTOUT_1, 1);
//     return ((data[0] * 256 + data[1])/3)*0.0012;
   
// }
































