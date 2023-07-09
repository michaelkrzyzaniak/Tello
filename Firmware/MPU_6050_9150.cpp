/*!
 * @author
 *  Written by Michael Krzyzaniak at Arizona State 
 *  University's School of Arts, Media + Engineering
 *  in Fall of 2013.
 *
 *  mkrzyzan@asu.edu
 */

#include <Wire.h>
#include "MPU_6050_9150.h"

#define MPU_MSB(z)         (  (uint8_t)(z>>8)  )
#define MPU_LSB(z)         (  (uint8_t)(z&0x00FF)  )
#define MPU_MAKEWORD(H, L) (  ((uint8_t)(H)<<8) | (uint8_t)(L)  )
#define MPU_BIG_ENDIAN    0
#define MPU_LITTLE_ENDIAN 1
#define PI 3.141592653589793


/* if ADC0 pin is connected to ground */
#define MPU_1_SLAVE_ADDRESS   0x68
/* if ADC0 pin is connected to VCC */
#define MPU_2_SLAVE_ADDRESS   0x69
/* put MPU into pass through mode before using this */
#define MPU_MAG_SLAVE_ADDRESS 0x0C 

#define MPU_CONFIG_ADDRESS         0x1A

//#define MPU_USER_CTRL_ADDRESS 0x6A //clear bit 5 to read mag

/* degrees per second */
#define MPU_GYRO_CONFIG_ADDRESS    0x1B
#define MPU_ACCEL_CONFIG_ADDRESS   0x1C

#define MPU_TEMP_ADDRESS           0x41

#define MPU_ACCEL_X_ADDRESS        0x3B
#define MPU_ACCEL_Y_ADDRESS        0x3D
#define MPU_ACCEL_Z_ADDRESS        0x3F

#define MPU_GYRO_X_ADDRESS         0x43
#define MPU_GYRO_Y_ADDRESS         0x45
#define MPU_GYRO_Z_ADDRESS         0x47

/* the first 6 EXT_SENS_DATA registers of MPU9150 */
#define MPU_MAG_X_ADDRESS          0x49
#define MPU_MAG_Y_ADDRESS          0x51
#define MPU_MAG_Z_ADDRESS          0x53

#define MPU_MAG_X_NATIVE_ADDRESS   0x03
#define MPU_MAG_Y_NATIVE_ADDRESS   0x05
#define MPU_MAG_Z_NATIVE_ADDRESS   0x07

/* for sleep and wake */
#define MPU_POWER_ADDRESS          0x6B

/* for direcly accessing mag */
#define MPU_USER_CTRL_ADDRESS      0x6A /* write bit 5 low for direct access*/
#define MPU_BYPASS_ADDRESS         0x37 /* write bit 1 high for direct access*/

/* for configuring 'external' sensor (i.e. mag) */
#define MPU_I2C_MST_CTRL_ADDRESS   0x24
#define MPU_SLV0_ADDR_ADDRESS      0x25
#define MPU_SLV0_REG_ADDRESS       0x26
#define MPU_SLV0_CTRL_ADDRESS      0x27
#define MPU_SLV1_ADDR_ADDRESS      0x28
#define MPU_SLV1_REG_ADDRESS       0x29
#define MPU_SLV1_CTRL_ADDRESS      0x2A
#define MPU_SLV1_DATA_OUT_ADDRESS  0x64
#define MPU_SLV4_ADDR_ADDRESS      0x31
#define MPU_SLV4_REG_ADDRESS       0x32
#define MPU_SLV4_DATA_OUT_ADDRESS  0x33
#define MPU_SLV4_CTRL_ADDRESS      0x34
#define MPU_SLV4_DATA_IN_ADDRESS   0x35

/*----------------------------------------------------*/
struct opaque_mpu_device_struct
{
  uint8_t slave_address;
  float   accel_scaling;
  float   gyro_scaling;
};

MPU MPU_1_STATIC;
MPU MPU_2_STATIC;

MPU* MPU_1 = &MPU_1_STATIC;
MPU* MPU_2 = &MPU_2_STATIC;

/*----------------------------------------------------*/
void     i2c_write_8    (uint8_t slave_address, uint8_t  mem_address, uint8_t data);
uint8_t  i2c_read_8     (uint8_t slave_address, uint8_t  mem_address);
void     i2c_write_16   (uint8_t slave_address, uint16_t mem_address, uint16_t data, int endian);
uint16_t i2c_read_16    (uint8_t slave_address, uint16_t mem_address, int endian);
void     i2c_read_buffer(uint8_t slave_address, uint16_t mem_address, char* buffer, int num_bytes);

void     mpu_set_resolution (uint8_t slave_address, uint8_t  registerAddress, uint8_t sensitivity);
uint8_t  mpu_get_resolution (uint8_t slave_address, uint8_t registerAddress);
void     mpu_setup_mag(MPU* self);

/*----------------------------------------------------*/
void i2c_write_8(uint8_t slave_address, uint8_t mem_address, uint8_t data)
{
  Wire.beginTransmission(slave_address);
  Wire.write            (mem_address  );
  Wire.write            (data         );
  Wire.endTransmission  (             );
}

/*----------------------------------------------------*/
uint8_t i2c_read_8(uint8_t slave_address, uint8_t mem_address)
{
    unsigned int timeout = 500;
    Wire.beginTransmission(slave_address);
    Wire.write            (mem_address  );
    Wire.endTransmission  (             );
    Wire.requestFrom((int)slave_address, (int)1);
    while((Wire.available() < 1) && (timeout-- > 0))
      delayMicroseconds(1);
    uint8_t data = Wire.read();
    return data;
}

/*----------------------------------------------------*/
void i2c_write_16(uint8_t slave_address, uint16_t mem_address, uint16_t data, int endian)
{
  Wire.beginTransmission(slave_address);
  Wire.write(mem_address);
  if(endian == MPU_LITTLE_ENDIAN)
    {
      Wire.write(MPU_MSB(data));
      Wire.write(MPU_LSB(data));
    }
  else
    {
      Wire.write(MPU_LSB(data));
      Wire.write(MPU_MSB(data));
    }
  Wire.endTransmission();
}

/*----------------------------------------------------*/
uint16_t i2c_read_16(uint8_t slave_address, uint16_t mem_address, int endian)
{
    unsigned int timeout = 500;
    Wire.beginTransmission(slave_address);
    Wire.write            (mem_address  );
    Wire.endTransmission  (             );
    Wire.requestFrom((int)slave_address, (int)2);
    while((Wire.available() < 2) && (timeout-- > 0))
      delayMicroseconds(1);
    uint16_t MPU_MSB = Wire.read();
    uint16_t MPU_LSB = Wire.read();
    if(endian == MPU_BIG_ENDIAN)
      return MPU_MAKEWORD(MPU_MSB, MPU_LSB);
    else
      return MPU_MAKEWORD(MPU_LSB, MPU_MSB);
}

/*----------------------------------------------------*/
void i2c_read_buffer(uint8_t slave_address, uint16_t mem_address, uint8_t* buffer, int num_bytes)
{
    unsigned int timeout = 500;
    Wire.beginTransmission(slave_address);
    Wire.write(mem_address);
    Wire.endTransmission();
    Wire.requestFrom((int)slave_address, num_bytes);
    while((Wire.available() < num_bytes) && (timeout-- > 0))
      delayMicroseconds(1);
    while(num_bytes-- > 0)
      *buffer++ = Wire.read();
}

/*----------------------------------------------------*/
uint8_t mpu_i2c_slave_read_8(MPU* self, uint8_t address, uint8_t reg)
{
  i2c_write_8(self->slave_address, MPU_SLV4_ADDR_ADDRESS, 0x80 | address);
  i2c_write_8(self->slave_address, MPU_SLV4_REG_ADDRESS , reg);
  i2c_write_8(self->slave_address, MPU_SLV4_CTRL_ADDRESS, 0x80); //trigger the i2c transaction 
  delay(1);
  return i2c_read_8(self->slave_address, MPU_SLV4_DATA_IN_ADDRESS);  
}

/*----------------------------------------------------*/
void mpu_i2c_slave_write_8(MPU* self, uint8_t address, uint8_t reg, uint8_t val)
{
  i2c_write_8(self->slave_address, MPU_SLV4_ADDR_ADDRESS, address);
  i2c_write_8(self->slave_address, MPU_SLV4_REG_ADDRESS , reg);
  i2c_write_8(self->slave_address, MPU_SLV4_DATA_OUT_ADDRESS, val);
  i2c_write_8(self->slave_address, MPU_SLV4_CTRL_ADDRESS, 0x80); //trigger the i2c transaction 
  delay(1);  
}

/*----------------------------------------------------*/
uint8_t mpu_magnetometer_read_8(MPU* self, uint8_t reg)
{
  return mpu_i2c_slave_read_8(self, MPU_MAG_SLAVE_ADDRESS,  reg);
}

/*----------------------------------------------------*/
void mpu_magnetometer_write_8(MPU* self, uint8_t reg, uint8_t val)
{
  mpu_i2c_slave_write_8(self, MPU_MAG_SLAVE_ADDRESS,  reg, val);
}


/*----------------------------------------------------*/
void mpu_setup     (MPU* self, uint8_t accel_resolution, uint8_t gyro_resolution)
{
  uint8_t resolution, config;
  Wire.begin();
  Wire.setClock(1000000);
  self->slave_address = (self == MPU_1) ? MPU_1_SLAVE_ADDRESS : MPU_2_SLAVE_ADDRESS;

  mpu_wake(self);
  mpu_set_gyro_resolution(self, gyro_resolution);
  mpu_set_accel_resolution(self, accel_resolution);
  //#ifdef MPU_9150
  mpu_setup_mag(self);
 //#endif //mpu9150
}

/*----------------------------------------------------*/
void mpu_sleep(MPU* self)
{
  int value = i2c_read_8(self->slave_address, MPU_POWER_ADDRESS);
  value |= 0x40;
  i2c_write_8(self->slave_address, MPU_POWER_ADDRESS, value);
}

/*----------------------------------------------------*/
void mpu_wake(MPU* self)
{
  delay(200);
  int value = i2c_read_8(self->slave_address, MPU_POWER_ADDRESS);
  value &= 0xBF; //wake
  value |= 0x01; //use gyro as clock
  i2c_write_8(self->slave_address, MPU_POWER_ADDRESS, value);
  delay(200);
}

/*----------------------------------------------------*/
void mpu_get_gyro (MPU* self, float *x, float *y, float *z)
{
  uint8_t buffer[6];
  int16_t ix, iy, iz;
  i2c_read_buffer(self->slave_address, MPU_GYRO_X_ADDRESS, buffer, 6);
  ix = MPU_MAKEWORD(buffer[0], buffer[1]);
  iy = MPU_MAKEWORD(buffer[2], buffer[3]);
  iz = MPU_MAKEWORD(buffer[4], buffer[5]); 
  *x = ix / self->gyro_scaling;
  *y = iy / self->gyro_scaling;
  *z = iz / self->gyro_scaling;
}

/*----------------------------------------------------*/
void mpu_get_accel(MPU* self, float *x, float *y, float *z)
{
  uint8_t buffer[6];
  int16_t ix, iy, iz;
  i2c_read_buffer(self->slave_address, MPU_ACCEL_X_ADDRESS, buffer, 6);
  ix = MPU_MAKEWORD(buffer[0], buffer[1]);
  iy = MPU_MAKEWORD(buffer[2], buffer[3]);
  iz = MPU_MAKEWORD(buffer[4], buffer[5]); 
  *x = ix / self->accel_scaling;
  *y = iy / self->accel_scaling;
  *z = iz / self->accel_scaling;
}

/*----------------------------------------------------*/
void mpu_get_mag  (MPU* self, float *x, float *y, float *z)
{
  uint8_t buffer[6];
  int16_t ix, iy, iz;

  i2c_read_buffer(self->slave_address, MPU_MAG_X_ADDRESS, buffer, 6);
  ix = MPU_MAKEWORD(buffer[0], buffer[1]);
  iy = MPU_MAKEWORD(buffer[2], buffer[3]);
  iz = MPU_MAKEWORD(buffer[4], buffer[5]);
 
  //*x = ix / 4096.0;
  //*y = iy / 4096.0;
  //*z = iz / 4096.0;

  *x = ix;
  *y = iy;
  *z = iz;
}

/*----------------------------------------------------*/
void mpu_get_temp (MPU* self, float *temp)
{
  int t = i2c_read_16(self->slave_address, MPU_TEMP_ADDRESS, MPU_BIG_ENDIAN);
  //*temp = (t / 340.0) + 35; //Celsius
  *temp = 1.8 * t / 340 + 95; //Farenheit
}

/*----------------------------------------------------*/
void mpu_set_gyro_resolution (MPU* self, uint8_t resolution)
{
  mpu_set_resolution(self->slave_address, MPU_GYRO_CONFIG_ADDRESS, resolution);
  switch(resolution)
    {
      case MPU_GYRO_RESOLUTION_250 : self->gyro_scaling = (32768.0 * 180.0) / (250.0 * PI) ; break;
      case MPU_GYRO_RESOLUTION_500 : self->gyro_scaling = (32768.0 * 180.0) / (500.0 * PI); break;
      case MPU_GYRO_RESOLUTION_1000: self->gyro_scaling = (32768.0 * 180.0) / (1000.0 * PI); break;
      case MPU_GYRO_RESOLUTION_2000: self->gyro_scaling = (32768.0 * 180.0) / (2000.0 * PI); break;
      default: self->gyro_scaling = 1;
    }
}

/*----------------------------------------------------*/
void mpu_set_accel_resolution (MPU* self, uint8_t resolution)
{
  mpu_set_resolution(self->slave_address, MPU_ACCEL_CONFIG_ADDRESS, resolution);
  switch(resolution)
    {
      case MPU_ACCEL_RESOLUTION_2_G : self->accel_scaling = 16384; break;
      case MPU_ACCEL_RESOLUTION_4_G : self->accel_scaling = 8192 ; break;
      case MPU_ACCEL_RESOLUTION_8_G : self->accel_scaling = 4096 ; break;
      case MPU_ACCEL_RESOLUTION_16_G: self->accel_scaling = 2048 ; break;
      default: self->accel_scaling = 1;
    }
}

/*----------------------------------------------------*/
uint8_t  mpu_get_gyro_resolution (MPU* self)
{
  return mpu_get_resolution(self->slave_address, MPU_GYRO_CONFIG_ADDRESS);
}

/*----------------------------------------------------*/
uint8_t  mpu_get_accel_resolution (MPU* self)
{
  return mpu_get_resolution(self->slave_address, MPU_ACCEL_CONFIG_ADDRESS);
}

/*----------------------------------------------------*/
void mpu_set_resolution(uint8_t slave_address, uint8_t register_address, uint8_t resolution)
{
  //uint8_t val = mpu_get_resolution(slave_address, register_address);
  uint8_t val = i2c_read_8(slave_address, register_address);
  val &= 0xE7;
  val |= resolution; //set sensitivity bits
  i2c_write_8(slave_address, register_address, val); 
}

/*----------------------------------------------------*/
uint8_t mpu_get_resolution(uint8_t slave_address, uint8_t register_address)
{
  uint8_t val;
  val = i2c_read_8(slave_address, register_address);

  val &= 0x18;



  return val;
}

/*----------------------------------------------------*/
void mpu_setup_mag(MPU* self)
{
   //genral config of the i2c bus
  i2c_write_8(self->slave_address, MPU_I2C_MST_CTRL_ADDRESS          , 0x5D); //set i2c clock 400 kHz
  i2c_write_8(self->slave_address, MPU_USER_CTRL_ADDRESS             , 0x20); //disconnect aux i2c from primary i2c
  i2c_write_8(self->slave_address, MPU_CONFIG_ADDRESS                , 0x01); //set DLPF freq to 1 kHz

  i2c_write_8(self->slave_address, 0x19                              , 1); //set sensor sample rate = 1ms * (this_number + 1), e.g. 4 for 5ms sample rate, 0 for 1kHz

  i2c_write_8(self->slave_address, 0x38                              , 0x01); //enable interrupt on data ready
  
  
  i2c_write_8(MPU_MAG_SLAVE_ADDRESS, 0x0A, 0x00); //PowerDownMode
  delayMicroseconds(100);
  i2c_write_8(MPU_MAG_SLAVE_ADDRESS, 0x0A, 0x0F); //SelfTest
  delayMicroseconds(100);
  i2c_write_8(MPU_MAG_SLAVE_ADDRESS, 0x0A, 0x00); //PowerDownMode


  //i2c_write_8(self->slave_address, MPU_I2C_MST_CTRL_ADDRESS, 0x40);
  i2c_write_8(self->slave_address, MPU_SLV0_ADDR_ADDRESS   , 0x80 | MPU_MAG_SLAVE_ADDRESS);
  i2c_write_8(self->slave_address, MPU_SLV0_REG_ADDRESS    , MPU_MAG_X_NATIVE_ADDRESS);
  i2c_write_8(self->slave_address, MPU_SLV0_CTRL_ADDRESS   , 0xD6); //0x87

  // put "single measuerment mode" in mag's CNTL register
  i2c_write_8(self->slave_address, MPU_SLV1_ADDR_ADDRESS, MPU_MAG_SLAVE_ADDRESS);
  i2c_write_8(self->slave_address, MPU_SLV1_REG_ADDRESS , 0x0A);
  i2c_write_8(self->slave_address, MPU_SLV1_CTRL_ADDRESS, 0x81);
  i2c_write_8(self->slave_address, MPU_SLV1_DATA_OUT_ADDRESS, 0x01);
  
  i2c_write_8(self->slave_address, 0x67, 0x03); //set delay rate

  //i2c_write_8(self->slave_address, 0x34, 0x04); //set i2c slv4 delay
  //i2c_write_8(self->slave_address, MPU_SLV1_DATA_OUT_ADDRESS, 0x00); //override register
  //i2c_write_8(self->slave_address, MPU_USER_CTRL_ADDRESS    , 0x00); //clear usr setting
  //i2c_write_8(self->slave_address, MPU_SLV1_DATA_OUT_ADDRESS, 0x01); //override register
  
  i2c_write_8(self->slave_address, MPU_USER_CTRL_ADDRESS   , 0x20); 
  i2c_write_8(self->slave_address, 0x34, 0x13); //disable slv4
}

/*----------------------------------------------------*/
//accel, temperature, gyro (BIG ENDIAN) mag (LITTLE ENDIAN) all 2 bytes
void mpu_read_sensors(MPU* self, mpu_sensor_data_t* data)
{
  //accel, temperature, gyro, mag  
  unsigned char b[20];
  i2c_read_buffer(self->slave_address, MPU_ACCEL_X_ADDRESS, b, 20);

  int16_t ax = MPU_MAKEWORD(b[0], b[1]);
  int16_t ay = MPU_MAKEWORD(b[2], b[3]);
  int16_t az = MPU_MAKEWORD(b[4], b[5]);
  int16_t gx = MPU_MAKEWORD(b[8], b[9]);
  int16_t gy = MPU_MAKEWORD(b[10], b[11]);
  int16_t gz = MPU_MAKEWORD(b[12], b[13]);
  int16_t mx = MPU_MAKEWORD(b[14], b[15]);
  int16_t my = MPU_MAKEWORD(b[16], b[17]);
  int16_t mz = MPU_MAKEWORD(b[18], b[19]);

  data->accel[0] = 0;
  data->accel[1] = ax / self->accel_scaling;
  data->accel[2] = ay / self->accel_scaling;
  data->accel[3] = az / self->accel_scaling;
  data->gyro[0] = 0;
  data->gyro [1] = gx / self->gyro_scaling;
  data->gyro [2] = gy / self->gyro_scaling;
  data->gyro [3] = gz / self->gyro_scaling;
  //mag has x and y sqapped and negative z
  data->mag  [0] =  0;
  data->mag  [1] =  my;
  data->mag  [2] =  mx;
  data->mag  [3] = -mz;
}
